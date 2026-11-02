//=============================================================================
// tb_osd_overlay.v — OSD 自检 testbench（T10 + T14）
//------------------------------------------------------------------------------
//   P1 窗口外全透传：rgb_out === rgb_in（逐像素）
//   P2 窗口内 = INK / EDGE / 与 rgb_in 一致（三选一；实验10 起字形自带 1px 描边）
//   P3 窗口内 ink 像素总数 === 各字符字形 ink 数之和（强校验，能抓错格/错位/错字）
//   P4 换帧可更新：改字符缓冲后 ink 计数随之改变
//   P5 CDC：源域发 N 拍 nd_a -> 目的域收 N 拍 nd_b，val_b == 最后一次的值
//   P6 osd_num_writer(MODE=0) 端到端：fps=30 -> 第 1 行出现 "FPS=030"
//   P7 imu_display 端到端（T14 新增）：50MHz 域发姿态 -> 跨域 -> 第 3/4 行
//      · 正常值  roll= 2994 -> "ROLL= 29.9" ; pitch=-125 -> "PITCH= -1.3"
//      · 零值    roll=    0 -> "ROLL=  0.0" ; pitch=   0 -> "PITCH=  0.0"
//      · 极值    roll= 9000 -> "ROLL= 90.0" ; pitch=-9000 -> "PITCH=-90.0"
//      · 两行互不覆盖，且 fps 行(第 1 行)不受影响
//   P9 描边（实验10 新增）：逐行描边数 === gen_font 精算值（不是估算）；
//      且量级护栏 0 < edge < 3*ink（防"把整块画黑"= 把底框又画回来）
//   P10 div_by_const 单测（M2 新增）：9 组被除数，与 Python 精算的 floor(x/1e6) 比对
//   P11 PCLK 行端到端（M2 新增）：row0 = "PCLK <MHz>"；
//       24 -> "PCLK 24" / 7 -> "PCLK  7" / 27.5 -> "PCLK 27"（floor，不是四舍五入）
//   P12 写口仲裁（M2 新增）：同拍触发 fps 与 PCLK，两行都要更新，
//       且全程"同拍争用计数器" wr_coll 必须为 0
//
// 行布局（NROW=4）：
//   row0 = TB 直写（字形自检）/ 帧 9 起改为 PCLK 行；row1 = fps；row2 = ROLL；row3 = PITCH
//   ⇒ 行号与 imu_display 的默认值(1/2)不同，是**故意错开**避免与 fps 抢行；
//     实际集成时按补丁清单把三行排成 0/1/2。
//
// 写源共 4 个：TB 直写 / fps / imu / PCLK，共享 cbuf 单个写口。
//   TB 内做优先 mux（直写 > fps > imu > pclk）；帧 12 专门同拍触发 fps+PCLK 验证仲裁，
//   另外全程有 wr_coll 计数器兜底（末尾断言必须为 0）。
//=============================================================================
`timescale 1ns/1ps

module tb_osd_overlay;
    localparam integer H     = 800;
    localparam integer VSCAN = 80;
    localparam integer CW    = 8;
    localparam integer CH    = 16;
    localparam integer NROW  = 4;
    localparam integer NCOL  = 40;
    localparam integer BX    = 8;
    localparam integer BY    = 8;
    localparam integer BOXW  = NCOL*CW;
    localparam integer BOXH  = NROW*CH;
    localparam [15:0]  INKP  = 16'h07E0;   // 笔画色
    localparam [15:0]  EDGEP = 16'h0000;   // 描边色（纯黑）—— 实验10
    localparam [15:0]  BGP   = 16'h0000;   // 旧底框色；保留仅为兼容旧判据，已不参与合成

    // 期望 ink（由 gen_font.py 的 ink 表算出）
    localparam integer INK_1234  = 82;
    localparam integer INK_FPS030= 153;
    localparam integer INK_RL1   = 179;   // "ROLL= 29.9"
    localparam integer INK_PT1   = 166;   // "PITCH= -1.3"
    localparam integer INK_RL0   = 158;   // "ROLL=  0.0"
    localparam integer INK_PT0   = 175;   // "PITCH=  0.0"
    localparam integer INK_RL9   = 184;   // "ROLL= 90.0"
    localparam integer INK_PT9   = 205;   // "PITCH=-90.0"
    localparam integer INK_RL30  = 179;   // "ROLL= 30.0"
    localparam integer INK_PT30  = 200;   // "PITCH=-30.0"

    reg pclk = 0, rst_n = 0;
    always #15 pclk = ~pclk;
    reg clk_src = 0;
    always #10 clk_src = ~clk_src;

    reg  [11:0] x = 0, y = 0;
    reg         frame_on = 0;
    reg  [15:0] rgb_in;
    wire        de_out;
    wire [15:0] rgb_out;

    // ---- TB 直写口 ----
    reg        we = 0;
    reg [4:0]  wrow = 0;
    reg [5:0]  wcol = 0;
    reg [7:0]  wcode = 8'h20;
    reg        disp_en = 1;

    // ---- fps writer 写口 ----
    wire        nw_we, nw_busy;
    wire [4:0]  nw_row;
    wire [5:0]  nw_col;
    wire [7:0]  nw_code;
    wire [15:0] nw_val_dbg;
    wire        nw_nd_dbg;

    // ---- imu_display 写口 ----
    wire        im_we;
    wire [4:0]  im_row;
    wire [5:0]  im_col;
    wire [7:0]  im_code;
    wire        im_rbusy, im_pbusy;

    // ---- PCLK writer 写口（M2 新增，写 row0）----
    wire        pk_we;
    wire [4:0]  pk_row;
    wire [5:0]  pk_col;
    wire [7:0]  pk_code;
    wire [15:0] pk_val_dbg, pk_val_dbg2;
    wire        pk_nd_dbg;

    // ---- 四方写口 mux（直写 > fps > imu > pclk）----
    //   pclk 由 pclk_row_src 保证在 fps 突发结束后才发 nd，正常不会与别人抢；
    //   优先级顺序只是防御性写法（真同拍也不会把 cbuf 写坏成不可解释的值）。
    wire        dut_we    = we | nw_we | im_we | pk_we;
    wire [4:0]  dut_wrow  = we ? wrow : (nw_we ? nw_row : (im_we ? im_row : pk_row));
    wire [5:0]  dut_wcol  = we ? wcol : (nw_we ? nw_col : (im_we ? im_col : pk_col));
    wire [7:0]  dut_wcode = we ? wcode: (nw_we ? nw_code: (im_we ? im_code: pk_code));

    // ---- 写口冲突计数（仲裁是否真的成立；末尾断言必须为 0）----
    wire [2:0] wr_src_cnt = {2'b00, we} + {2'b00, nw_we} + {2'b00, im_we} + {2'b00, pk_we};
    integer    wr_coll = 0;
    always @(posedge pclk) if (rst_n && (wr_src_cnt > 3'd1)) wr_coll = wr_coll + 1;

    always @(*) rgb_in = {y[7:0], x[7:0]};

    osd_overlay #(
        .H_ACTIVE(H), .V_ACTIVE(480),
        .CHAR_W(CW), .CHAR_H(CH), .NROW(NROW), .NCOL(NCOL),
        .BOX_X(BX), .BOX_Y(BY), .INK(INKP), .EDGE(EDGEP), .BGC(BGP)
    ) dut (
        .pclk(pclk), .rst_n(rst_n),
        .de_in(frame_on), .x_in(x), .y_in(y), .rgb_in(rgb_in),
        .de_out(de_out), .rgb_out(rgb_out),
        .we(dut_we), .wrow(dut_wrow), .wcol(dut_wcol), .wcode(dut_wcode),
        .disp_en(disp_en)
    );

    //====================================================================
    // 扫描发生器
    //====================================================================
    always @(posedge pclk) begin
        if (!rst_n) begin x <= 12'd0; y <= 12'd0; end
        else if (frame_on) begin
            if (x == H-1) begin
                x <= 12'd0;
                y <= (y == VSCAN-1) ? 12'd0 : (y + 12'd1);
            end else x <= x + 12'd1;
        end
    end

    //====================================================================
    // 检查器（同一拍读 x/y 与 rgb_out；按行分别统计 ink）
    //====================================================================
    integer err     = 0;
    integer out_cnt = 0;
    integer pas_cnt = 0;
    integer amb_cnt = 0;   // 前提自检：窗口内 rgb_in 不得等于 INK/EDGE 色（否则计数有歧义）
    integer ink_tot = 0;
    integer ink_row [0:3];
    integer edge_tot = 0;
    integer edge_row [0:3];
    integer trow_i;

    reg cap  [0:BOXH-1][0:BOXW-1];   // 笔画像素（打印成 '#'）
    reg ecap [0:BOXH-1][0:BOXW-1];   // 描边像素（打印成 'o'）

    //------------------------------------------------------------------
    // 描边期望值 = 由 gen_font.py 对每帧文本**逐字精算**（不是估算），
    //   精算用的 rom 与 osd_overlay 里 include 的是同一份 font_rom_init.vh。
    //   e_row[fid][row] / e_tot[fid]，fid 与 run_frame 的第一个参数一致。
    //   ⚠️ 这套值同时是「描边没有侵入笔画」的判据：ink 期望（run_frame 里的
    //      104/82/59/235/580/568/624/614）必须**一字不变**，否则就是描边改坏了本体。
    //------------------------------------------------------------------
    integer e_row [1:12][0:3];
    integer e_tot [1:12];
    initial begin
        e_row[1][0]=256; e_row[1][1]=  0; e_row[1][2]=  0; e_row[1][3]=  0; e_tot[1]= 256;
        e_row[2][0]=212; e_row[2][1]=  0; e_row[2][2]=  0; e_row[2][3]=  0; e_tot[2]= 212;
        e_row[3][0]=152; e_row[3][1]=  0; e_row[3][2]=  0; e_row[3][3]=  0; e_tot[3]= 152;
        e_row[4][0]=212; e_row[4][1]=376; e_row[4][2]=  0; e_row[4][3]=  0; e_tot[4]= 588;
        e_row[5][0]=212; e_row[5][1]=376; e_row[5][2]=443; e_row[5][3]=407; e_tot[5]=1438;
        e_row[6][0]=212; e_row[6][1]=376; e_row[6][2]=379; e_row[6][3]=422; e_tot[6]=1389;
        e_row[7][0]=212; e_row[7][1]=376; e_row[7][2]=443; e_row[7][3]=500; e_tot[7]=1531;
        e_row[8][0]=212; e_row[8][1]=376; e_row[8][2]=435; e_row[8][3]=492; e_tot[8]=1515;
        // ---- M2 新增：帧 9..12 的 row0 = "PCLK <MHz>"，row1 = fps ----
        e_row[9][0] =329; e_row[9][1] =376; e_row[9][2] =435; e_row[9][3] =492; e_tot[9] =1632;
        e_row[10][0]=262; e_row[10][1]=376; e_row[10][2]=435; e_row[10][3]=492; e_tot[10]=1565;
        e_row[11][0]=326; e_row[11][1]=376; e_row[11][2]=435; e_row[11][3]=492; e_tot[11]=1629;
        e_row[12][0]=342; e_row[12][1]=355; e_row[12][2]=435; e_row[12][3]=492; e_tot[12]=1624;
    end

    always @(posedge pclk) begin
        if (rst_n && frame_on) begin
            if ((x >= BX) && (x < BX+BOXW) && (y >= BY) && (y < BY+BOXH)) begin
                trow_i = (y - BY) / CH;
                // P2（实验10 改）：纯叠加 + 字形自带 1px 描边 => 窗口内合法值三种：
                //   ① rgb_out === INK    : 笔画像素
                //   ② rgb_out === EDGE   : 描边像素
                //   ③ rgb_out === rgb_in : 未覆盖，直接透传（含窗口内的空白像素）
                // 旧判据「必须是 INK/BGC」与「INK 或 rgb_in」均已失效 —— 不得恢复底框。
                if (rgb_out === INKP) begin
                    ink_tot = ink_tot + 1;
                    if (trow_i < 4) ink_row[trow_i] = ink_row[trow_i] + 1;
                    cap [y-BY][x-BX] = 1'b1;
                    ecap[y-BY][x-BX] = 1'b0;
                end else if (rgb_out === EDGEP) begin
                    edge_tot = edge_tot + 1;
                    if (trow_i < 4) edge_row[trow_i] = edge_row[trow_i] + 1;
                    cap [y-BY][x-BX] = 1'b0;
                    ecap[y-BY][x-BX] = 1'b1;
                end else if (rgb_out === rgb_in) begin
                    cap [y-BY][x-BX] = 1'b0;
                    ecap[y-BY][x-BX] = 1'b0;
                end else begin
                    out_cnt = out_cnt + 1;
                    if (out_cnt <= 5)
                        $display("!! P2 FAIL: 窗口内既非 INK/EDGE 也不等于 rgb_in: out=%h in=%h @(%0d,%0d)",
                                 rgb_out, rgb_in, x, y);
                end
                // 前提自检：窗口内 ink/edge 靠「颜色」判定；
                // 若原图像在窗口内出现这两种颜色，计数就会失真。
                if (rgb_in === INKP || rgb_in === EDGEP) amb_cnt = amb_cnt + 1;
            end else begin
                pas_cnt = pas_cnt + 1;
                if (rgb_out !== rgb_in) begin
                    err = err + 1;
                    if (err <= 5)
                        $display("!! P1 FAIL: 窗口外透传错误 out=%h in=%h @(%0d,%0d)",
                                 rgb_out, rgb_in, x, y);
                end
            end
        end
    end

    //====================================================================
    // TB 直写
    //====================================================================
    task put_char(input [4:0] r, input [5:0] c, input [7:0] code);
        begin
            @(posedge pclk);
            we = 1'b1; wrow = r; wcol = c; wcode = code;
            @(posedge pclk);
            we = 1'b0;
            @(posedge pclk);
        end
    endtask

    task put_str0(input [7:0] a, input [7:0] b, input [7:0] c, input [7:0] d);
        begin
            put_char(5'd0, 6'd0, a);
            put_char(5'd0, 6'd1, b);
            put_char(5'd0, 6'd2, c);
            put_char(5'd0, 6'd3, d);
        end
    endtask

    //====================================================================
    // 跑一帧
    //====================================================================
    task run_frame(input integer fid, input integer exp_tot,
                   input integer e0, input integer e1,
                   input integer e2, input integer e3);
        begin
            ink_tot = 0; out_cnt = 0; pas_cnt = 0; amb_cnt = 0; edge_tot = 0;
            for (trow_i = 0; trow_i < 4; trow_i = trow_i + 1) begin
                ink_row [trow_i] = 0;
                edge_row[trow_i] = 0;
            end
            frame_on = 1'b1;
            @(posedge pclk);
            wait (y == VSCAN-1 && x == H-1);
            @(posedge pclk);
            frame_on = 1'b0;
            @(posedge pclk);

            $display("[FRAME %0d] ink=%0d (expect %0d) | row0=%0d/%0d row1=%0d/%0d row2=%0d/%0d row3=%0d/%0d | 透传=%0d 非法色=%0d",
                     fid, ink_tot, exp_tot,
                     ink_row[0], e0, ink_row[1], e1, ink_row[2], e2, ink_row[3], e3,
                     pas_cnt, out_cnt);
            $display("[FRAME %0d] edge=%0d (expect %0d) | row0=%0d/%0d row1=%0d/%0d row2=%0d/%0d row3=%0d/%0d | edge/ink=%0d.%02d",
                     fid, edge_tot, e_tot[fid],
                     edge_row[0], e_row[fid][0], edge_row[1], e_row[fid][1],
                     edge_row[2], e_row[fid][2], edge_row[3], e_row[fid][3],
                     edge_tot*100/ink_tot/100, edge_tot*100/ink_tot%100);
            if (ink_tot == exp_tot) $display("    P3 OK (总 ink 吻合)");
            else begin $display("    !! P3 FAIL: 总 ink 不符"); err = err + 1; end
            if (ink_row[0]==e0 && ink_row[1]==e1 && ink_row[2]==e2 && ink_row[3]==e3)
                $display("    OK (逐行 ink 全部吻合)");
            else begin $display("    !! FAIL: 逐行 ink 不符"); err = err + 1; end
            // ---- 实验10 新增：描边逐行精算值比对 ----
            if (edge_tot == e_tot[fid] &&
                edge_row[0]==e_row[fid][0] && edge_row[1]==e_row[fid][1] &&
                edge_row[2]==e_row[fid][2] && edge_row[3]==e_row[fid][3])
                $display("    P9 OK (描边与 gen_font 精算值逐行吻合)");
            else begin $display("    !! P9 FAIL: 描边与精算值不符"); err = err + 1; end
            // ---- 量级护栏：描边必须存在，且不得把整块画黑（>3x ink 就等于画回底框）----
            if (edge_tot > 0 && edge_tot < ink_tot*3)
                $display("    P9 OK (量级护栏: 0 < edge %0d < 3*ink %0d)", edge_tot, ink_tot*3);
            else begin
                $display("    !! FAIL: 描边量级越界 edge=%0d ink=%0d", edge_tot, ink_tot);
                err = err + 1;
            end
            if (out_cnt == 0) $display("    P2 OK (窗口内 = INK / EDGE / 与 rgb_in 一致)");
            else              err = err + 1;
            if (amb_cnt == 0) $display("    OK (前提满足: 窗口内 rgb_in 不含 INK/BGC 色, ink 计数无歧义)");
            else begin
                $display("    !! FAIL: 窗口内 rgb_in 出现 INK/BGC 色 %0d 像素 => P3 计数有歧义", amb_cnt);
                err = err + 1;
            end
            if (pas_cnt > 0)  $display("    P1 OK (窗口外 %0d 像素全部纯透传)", pas_cnt);
            else begin $display("    !! P1 FAIL: 未检到窗口外像素"); err = err + 1; end
        end
    endtask

    //====================================================================
    // 打印文本行前 nct 格（肉眼核对字形与描边）：'#'=笔画  'o'=描边
    //====================================================================
    task print_textrow(input integer trow, input integer nct);
        integer r, c, base;
        begin
            base = trow * CH;
            $display("    ---- row %0d（前 %0d 格）----", trow, nct);
            for (r = 0; r < CH; r = r + 1) begin
                $write("    |");
                for (c = 0; c < nct*CW; c = c + 1)
                    $write("%s", cap[base+r][c] ? "#" : (ecap[base+r][c] ? "o" : "."));
                $write("|\n");
            end
        end
    endtask

    //====================================================================
    // 直接读 cbuf 的字符码（比 ink 计数强）
    //   为什么必须有它：本项目字形里 '2'/'3' 的 ink 都是 21、'9'/'0' 都是 26
    //   ⇒ **ink 计数分辨不出 "29.9" 与 "30.0"**。要验证"四舍五入 vs 截断"，
    //   必须直接看字符码。这条是被 MiMo 点出来的（§13 #14）。
    //====================================================================
    task chk_cells(input [8*16:1] tag, input integer row, input integer col0,
                   input [7:0] e0, input [7:0] e1, input [7:0] e2,
                   input [7:0] e3, input [7:0] e4);
        begin
            if (dut.cbuf[row*NCOL+col0+0] === e0 && dut.cbuf[row*NCOL+col0+1] === e1 &&
                dut.cbuf[row*NCOL+col0+2] === e2 && dut.cbuf[row*NCOL+col0+3] === e3 &&
                dut.cbuf[row*NCOL+col0+4] === e4)
                $display("    OK   %0s = \"%c%c%c%c%c\"", tag, e0,e1,e2,e3,e4);
            else begin
                $display("    !! FAIL %0s: 实得 \"%c%c%c%c%c\" 期望 \"%c%c%c%c%c\"", tag,
                         dut.cbuf[row*NCOL+col0+0], dut.cbuf[row*NCOL+col0+1],
                         dut.cbuf[row*NCOL+col0+2], dut.cbuf[row*NCOL+col0+3],
                         dut.cbuf[row*NCOL+col0+4], e0,e1,e2,e3,e4);
                err = err + 1;
            end
        end
    endtask

    //====================================================================
    // P5：CDC
    //====================================================================
    reg  [15:0] cdc_val_a = 0;
    reg         cdc_nd_a  = 0;
    wire [15:0] cdc_val_b;
    wire        cdc_nd_b;
    integer got_b = 0;

    osd_cdc_nd u_cdc (
        .clk_a(clk_src), .rst_n_a(rst_n), .val_a(cdc_val_a), .nd_a(cdc_nd_a),
        .clk_b(pclk),    .rst_n_b(rst_n), .val_b(cdc_val_b), .nd_b(cdc_nd_b)
    );
    always @(posedge pclk) if (rst_n && cdc_nd_b) got_b = got_b + 1;

    integer got_nw = 0;
    always @(posedge pclk) if (rst_n && nw_nd_dbg) got_nw = got_nw + 1;

    //====================================================================
    // fps writer（MODE=0，写 row1）
    //====================================================================
    reg  [15:0] nw_val = 0;
    reg         nw_nd  = 0;

    osd_num_writer #(
        .MODE(0), .NDIG(3), .LEAD_ZERO(1), .VAL_MAX(999),
        .LBL(48'h4650533D0000), .LBL_LEN(4),      // "FPS=" 左对齐
        .ROW(5'd1), .COL0(6'd0), .NCOL(NCOL)
    ) u_nw (
        .clk_src(clk_src), .val_src(nw_val), .nd_src(nw_nd),
        .pclk(pclk), .rst_n(rst_n), .stall(1'b0),
        .we(nw_we), .wrow(nw_row), .wcol(nw_col), .wcode(nw_code), .busy(nw_busy),
        .val_dbg(nw_val_dbg), .nd_dbg(nw_nd_dbg)
    );

    task send_fps(input [15:0] v);
        begin
            @(posedge clk_src);
            nw_val = v; nw_nd = 1'b1;
            @(posedge clk_src);
            nw_nd = 1'b0;
        end
    endtask

    //====================================================================
    // imu_display（T14：两个 MODE=1 writer，写 row2 / row3）
    //====================================================================
    reg signed [15:0] imu_roll = 0;
    reg signed [15:0] imu_pitch = 0;
    reg               imu_nd = 0;

    imu_display #(
        .NCOL(NCOL),
        .ROLL_ROW (5'd2), .ROLL_COL0 (6'd0),
        .ROLL_LBL (48'h524F4C4C3D00), .ROLL_LBL_LEN (5),     // "ROLL="
        .PITCH_ROW(5'd3), .PITCH_COL0(6'd0),
        .PITCH_LBL(48'h50495443483D),  .PITCH_LBL_LEN(6),    // "PITCH="
        .ANG_LIMIT(9000)
    ) u_imu (
        .clk_src(clk_src), .rst_n(rst_n),
        .roll_x100(imu_roll), .pitch_x100(imu_pitch), .nd_src(imu_nd),
        .pclk(pclk),
        .we(im_we), .wrow(im_row), .wcol(im_col), .wcode(im_code),
        .roll_busy(im_rbusy), .pitch_busy(im_pbusy)
    );

    integer imu_wr = 0;
    always @(posedge pclk) if (rst_n && im_we) imu_wr = imu_wr + 1;

    task send_imu(input signed [15:0] r, input signed [15:0] p);
        begin
            @(posedge clk_src);
            imu_roll = r; imu_pitch = p; imu_nd = 1'b1;
            @(posedge clk_src);
            imu_nd = 1'b0;
        end
    endtask

    //====================================================================
    // PCLK 行链路（M2 新增）：pclk_row_src -> osd_num_writer(ROW=0, "PCLK")
    //   真实工程里两个 writer 都由 cam_rate_valid 触发；
    //   TB 里显式触发，便于精确构造"同拍触发"的仲裁用例。
    //====================================================================
    reg  [31:0] tb_pclk_cnt  = 32'd0;
    reg         tb_pclk_trig = 1'b0;
    wire        pk_nd_src;
    wire        pk_busy_dbg;
    wire        pk_w_busy;

    pclk_row_src #(.DIVISOR(1_000_000), .DEFER_CYC(24), .VAL_MAX(999)) u_pksrc (
        .clk_src (clk_src),
        .rst_n   (rst_n),
        .pclk_cnt(tb_pclk_cnt),
        .trig    (tb_pclk_trig),
        .fps_nd_b(nw_nd_dbg),          // 观测 fps writer 的 nd（pclk 域）
        .val_src (pk_val_dbg),
        .nd_src  (pk_nd_src),
        .val_dbg (),
        .busy_dbg(pk_busy_dbg)
    );

    osd_num_writer #(
        .MODE(0),
        .NDIG(3), .LEAD_ZERO(0),                 // 0 = 前导空格 ⇒ "PCLK 24"
        // ⚠️ 本目录的 osd_num_writer 是 T14 扩展版：LBL 是 **48 位左对齐**，
        //    且多了 stall/busy 两个口。少接 stall 会让突发永不推进（实测踩过：
        //    stall 悬空=Z -> "busy && !stall" 为假 -> 一个字都不写）。
        //    工程里用的是**旧版 32 位 LBL 且无 stall**，顶层按旧版接（见顶层）。
        .LBL(48'h50434C4B0000), .LBL_LEN(4),     // "PCLK"
        .ROW(5'd0), .COL0(6'd0),                 // 第 0 行；FPS 仍在第 1 行不动
        .NCOL(NCOL), .VAL_MAX(999)
    ) u_pk_w (
        .clk_src(clk_src),
        .val_src(pk_val_dbg),
        .nd_src (pk_nd_src),
        .pclk   (pclk),
        .rst_n  (rst_n),
        .stall  (1'b0),                          // 仲裁在 pclk_row_src 的**触发侧**做
        .we     (pk_we),
        .wrow   (pk_row),
        .wcol   (pk_col),
        .wcode  (pk_code),
        .busy   (pk_w_busy),
        .val_dbg(pk_val_dbg2),
        .nd_dbg (pk_nd_dbg)
    );

    // ---- 触发一次 PCLK 更新（源域单拍）----
    task trig_pclk(input [31:0] pcnt);
        begin
            @(posedge clk_src);
            tb_pclk_cnt  = pcnt;
            tb_pclk_trig = 1'b1;
            @(posedge clk_src);
            tb_pclk_trig = 1'b0;
        end
    endtask

    // ---- 同拍触发 fps 与 PCLK（模拟真实设计：两者都由 cam_rate_valid 触发）----
    task trig_both(input [15:0] fpsv, input [31:0] pcnt);
        begin
            @(posedge clk_src);
            nw_val = fpsv;      nw_nd = 1'b1;
            tb_pclk_cnt = pcnt; tb_pclk_trig = 1'b1;
            @(posedge clk_src);
            nw_nd = 1'b0;       tb_pclk_trig = 1'b0;
        end
    endtask

    // ---- div_by_const 单测实例（DIVISOR=1e6）----
    reg  [31:0] td_div_dd    = 32'd0;
    reg         td_div_start = 1'b0;
    wire [31:0] td_div_quot;
    wire        td_div_busy, td_div_done;

    div_by_const #(.DIVISOR(1_000_000), .DW(32)) u_tdiv (
        .clk(clk_src), .rst_n(rst_n), .start(td_div_start),
        .dividend(td_div_dd), .quot(td_div_quot),
        .busy(td_div_busy), .done(td_div_done)
    );

    integer div_chk = 0;
    task chk_div(input [31:0] x, input [31:0] exp);
        begin
            @(posedge clk_src);
            td_div_dd = x; td_div_start = 1'b1;
            @(posedge clk_src);
            td_div_start = 1'b0;
            @(posedge clk_src);
            if (td_div_busy !== 1'b1) begin
                $display("    !! FAIL div(%0d): start 之后 busy 未拉高", x);
                err = err + 1;
            end
            wait (td_div_done);
            @(posedge clk_src);
            div_chk = div_chk + 1;
            if (td_div_quot === exp) $display("    OK   div(%0d, 1e6) = %0d", x, td_div_quot);
            else begin
                $display("    !! FAIL div(%0d, 1e6): 实得 %0d 期望 %0d", x, td_div_quot, exp);
                err = err + 1;
            end
            repeat (4) @(posedge clk_src);
        end
    endtask


    //====================================================================
    // 主流程
    //====================================================================
    integer i;
    initial begin
        $display("==== tb_osd_overlay 开始（T10 + T14 + M2/PCLK）====");
        rst_n = 0;
        repeat (20) @(posedge pclk);
        rst_n = 1;
        repeat (10) @(posedge pclk);

        //---------------- P5：CDC ----------------
        got_b = 0;
        for (i = 0; i < 5; i = i + 1) begin
            @(posedge clk_src);
            cdc_val_a = 16'h1000 + i[15:0];
            cdc_nd_a  = 1'b1;
            @(posedge clk_src);
            cdc_nd_a  = 1'b0;
            repeat (80) @(posedge clk_src);
        end
        repeat (200) @(posedge pclk);
        $display("[P5 CDC ] 源域发 5 拍, 目的域收 %0d 拍, val_b=%h (expect 1004)",
                 got_b, cdc_val_b);
        if (got_b == 5 && cdc_val_b === 16'h1004) $display("    P5 OK");
        else begin $display("    !! P5 FAIL"); err = err + 1; end

        //---------------- 帧 1-3：row0 直写，字形与更新 ----------------
        put_str0(8'h30, 8'h30, 8'h30, 8'h30);      // "0000" -> 104
        run_frame(1, 104, 104, 0, 0, 0);
        print_textrow(0, 4);

        put_str0(8'h31, 8'h32, 8'h33, 8'h34);      // "1234" -> 82
        run_frame(2, 82, 82, 0, 0, 0);
        print_textrow(0, 4);

        put_str0(8'h39, 8'h2E, 8'h35, 8'h2D);      // "9.5-" -> 59
        run_frame(3, 59, 59, 0, 0, 0);
        print_textrow(0, 4);

        //---------------- 帧 4：row0 "1234" + row1 "FPS=030"（P6 回归）----------------
        put_str0(8'h31, 8'h32, 8'h33, 8'h34);
        send_fps(16'd30);
        repeat (400) @(posedge pclk);
        $display("[P6 NUMWR] fps writer 目的域收到 nd 次数=%0d, val=%0d", got_nw, nw_val_dbg);
        if (got_nw >= 1 && nw_val_dbg === 16'd30) $display("    OK (CDC 搬运正确)");
        else begin $display("    !! FAIL CDC"); err = err + 1; end
        run_frame(4, INK_1234 + INK_FPS030, INK_1234, INK_FPS030, 0, 0);
        print_textrow(1, 8);

        //---------------- 帧 5：姿态正常值（P7）----------------
        $display("[P7 IMU ] roll=2994 pitch=-125 -> 期望 \"ROLL= 29.9\" / \"PITCH= -1.3\"");
        send_imu(16'sd2994, -16'sd125);
        repeat (500) @(posedge pclk);
        $display("    imu 写口动作次数=%0d", imu_wr);
        if (imu_wr >= 21) $display("    OK (写口动作 %0d 次, 期望 >=21 = 10+11 字符)", imu_wr);
        else begin $display("    !! FAIL: 写口动作过少"); err = err + 1; end
        run_frame(5, INK_1234+INK_FPS030+INK_RL1+INK_PT1, INK_1234, INK_FPS030, INK_RL1, INK_PT1);
        chk_cells("row2 num", 2, 5, 8'h20, 8'h32, 8'h39, 8'h2E, 8'h39);   // " 29.9"
        chk_cells("row3 num", 3, 6, 8'h20, 8'h2D, 8'h31, 8'h2E, 8'h33);   // " -1.3"
        print_textrow(2, 10);
        print_textrow(3, 11);

        //---------------- 帧 6：零值 ----------------
        $display("[P7b] roll=0 pitch=0 -> 期望 \"ROLL=  0.0\" / \"PITCH=  0.0\"");
        send_imu(16'sd0, 16'sd0);
        repeat (500) @(posedge pclk);
        run_frame(6, INK_1234+INK_FPS030+INK_RL0+INK_PT0, INK_1234, INK_FPS030, INK_RL0, INK_PT0);
        print_textrow(2, 10);
        print_textrow(3, 11);

        //---------------- 帧 7：极值 ----------------
        $display("[P7c] roll=+9000 pitch=-9000 -> 期望 \"ROLL= 90.0\" / \"PITCH=-90.0\"");
        send_imu(16'sd9000, -16'sd9000);
        repeat (500) @(posedge pclk);
        run_frame(7, INK_1234+INK_FPS030+INK_RL9+INK_PT9, INK_1234, INK_FPS030, INK_RL9, INK_PT9);
        chk_cells("row2 num", 2, 5, 8'h20, 8'h39, 8'h30, 8'h2E, 8'h30);   // " 90.0"
        chk_cells("row3 num", 3, 6, 8'h2D, 8'h39, 8'h30, 8'h2E, 8'h30);   // "-90.0"
        print_textrow(2, 10);
        print_textrow(3, 11);

        //---------------- 帧 8：进位边界（分辨"四舍五入"与"截断"）----------------
        //   299.6 度*100 -> q=(2996+5)/10=300 -> "30.0"
        //   若实现是截断: q=2996/10=299 -> "29.9"  <-- 必须能区分出来
        //   ⚠ ink 分辨不了（'2'/'3' 同 ink=21，'9'/'0' 同 ink=26），所以看字符码。
        $display("[P8 CARRY] roll=+2996 pitch=-2996 -> 期望 \"ROLL= 30.0\" / \"PITCH=-30.0\"");
        send_imu(16'sd2996, -16'sd2996);
        repeat (500) @(posedge pclk);
        run_frame(8, INK_1234+INK_FPS030+INK_RL30+INK_PT30, INK_1234, INK_FPS030, INK_RL30, INK_PT30);
        chk_cells("row2 num", 2, 5, 8'h20, 8'h33, 8'h30, 8'h2E, 8'h30);   // " 30.0"
        chk_cells("row3 num", 3, 6, 8'h2D, 8'h33, 8'h30, 8'h2E, 8'h30);   // "-30.0"
        print_textrow(2, 10);
        print_textrow(3, 11);

        //================================================================
        // P10：div_by_const 单测（变量 / 常量，32 拍恢复余数法）
        //   期望值全部由 Python 精算（floor(x/1e6)），不是估算。
        //================================================================
        $display("[P10 DIV  ] div_by_const 单测（DIVISOR = 1e6）");
        chk_div(32'd24_000_000, 32'd24);
        chk_div(32'd 7_000_000, 32'd 7);
        chk_div(32'd27_500_000, 32'd27);   // 27.5 -> 27（floor，不是四舍五入）
        chk_div(32'd25_000_000, 32'd25);
        chk_div(32'd 1_000_000, 32'd 1);
        chk_div(32'd   999_999, 32'd 0);   // 小于除数 -> 0
        chk_div(32'd23_999_999, 32'd23);   // 边界：差 1 就到 24
        chk_div(32'd50_000_000, 32'd50);
        chk_div(32'd         0, 32'd 0);
        $display("    OK (%0d 个除法用例全过)", div_chk);

        //================================================================
        // 帧 9-11：PCLK 行端到端（只动 PCLK 通路，fps 行保持 "FPS=030"）
        //   row0 = "PCLK <MHz>"  （LBL="PCLK", LBL_LEN=4, LEAD_ZERO=0）
        //================================================================
        $display("[P11 PCLK ] 24 MHz -> 期望 \"PCLK 24\"");
        trig_pclk(32'd24_000_000);
        repeat (400) @(posedge pclk);
        run_frame(9, 660, 128, 153, 179, 200);
        chk_cells("row0 lbl", 0, 0, 8'h50, 8'h43, 8'h4C, 8'h4B, 8'h20);   // "PCLK "
        chk_cells("row0 num", 0, 5, 8'h32, 8'h34, 8'h20, 8'h20, 8'h20);   // "24"
        print_textrow(0, 7);

        $display("[P11b PCLK] 7 MHz -> 期望 \"PCLK  7\"（前导空格）");
        trig_pclk(32'd7_000_000);
        repeat (400) @(posedge pclk);
        run_frame(10, 634, 102, 153, 179, 200);
        chk_cells("row0 num", 0, 5, 8'h20, 8'h37, 8'h20, 8'h20, 8'h20);   // " 7"
        print_textrow(0, 7);

        $display("[P11c PCLK] 27.5 MHz -> 期望 \"PCLK 27\"（floor，可区分四舍五入）");
        trig_pclk(32'd27_500_000);
        repeat (400) @(posedge pclk);
        run_frame(11, 655, 123, 153, 179, 200);
        chk_cells("row0 num", 0, 5, 8'h32, 8'h37, 8'h20, 8'h20, 8'h20);   // "27"
        print_textrow(0, 7);

        //================================================================
        // 帧 12：写口仲裁 —— 同拍触发 fps 与 PCLK（= 真实设计的时序）
        //   fps 保持优先（照常发 nd），PCLK 让路（等 fps 突发结束）
        //   ⇒ 两行都要更新，且不能出现同拍争用
        //================================================================
        $display("[P12 ARB  ] 同拍触发 fps=31 与 PCLK=25MHz -> 两行都更新且不冲突");
        trig_both(16'd31, 32'd25_000_000);
        repeat (600) @(posedge pclk);
        run_frame(12, 655, 131, 145, 179, 200);
        chk_cells("row0 num", 0, 5, 8'h32, 8'h35, 8'h20, 8'h20, 8'h20);   // "25"
        chk_cells("row1 lbl", 1, 0, 8'h46, 8'h50, 8'h53, 8'h3D, 8'h30);   // "FPS=0"
        chk_cells("row1 num", 1, 5, 8'h33, 8'h31, 8'h20, 8'h20, 8'h20);   // "31"
        print_textrow(0, 7);
        print_textrow(1, 7);

        // ---- 写口冲突总检（仲裁的核心断言）----
        $display("[P12 ARB  ] 全程写口同拍冲突计数 = %0d (expect 0)", wr_coll);
        if (wr_coll == 0) $display("    P12 OK (四个写源从未同拍争用 cbuf)");
        else begin $display("    !! P12 FAIL: 出现同拍冲突 %0d 次", wr_coll); err = err + 1; end

        //---------------- 汇总 ----------------
        if (err == 0) $display("=== tb_osd_overlay: ALL PASS ===");
        else          $display("=== tb_osd_overlay: %0d ERROR(S) ===", err);
        $finish;
    end

endmodule
