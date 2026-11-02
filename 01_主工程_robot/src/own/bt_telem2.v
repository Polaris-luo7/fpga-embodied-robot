//=============================================================================
// bt_telem2.v -- 遥测/应答发送器（任务四 P1/P3）: **单一 TX 拥有者**
//-----------------------------------------------------------------------------
// 【为什么需要它】bt_tx 只有一根线。$PONG / $TEL / 应答帧三个字节源若各用各的
//   发送器会互相打断。本模块把三者合进一个发送器，独占 TX ⇒ 零冲突。
//
// 【三种帧】IDLE 处按优先级挑选（帧边界切换, 不做帧内抢占 ⇒ 每帧完整,
//          最坏等待一帧 2.4ms, 对人机交互毫无影响）:
//     1) $PONG\n                     6 B   链路应答（$PING 触发, 最高优先）
//     2) 应答帧 $TAG,aa\n            8 B   $ACK/$DONE/$ERR/$EVT  (P1/P3)
//     3) $TEL 遥测帧                 27 B  10Hz
//
// 【应答帧统一 8 字节】  $ + TAG(3) + , + AA(2 位数字) + \n
//     R_ADD   -> $ADD,02\n   已录入第 2 个途径点
//     R_CLR   -> $CLR,00\n   队列已清空
//     R_READY -> $RDY,03\n   解读完成, 3 个点待命
//     R_GO    -> $RUN,00\n   开始导航
//     R_STOP  -> $STP,00\n   已停止
//     R_DONE  -> $DON,02\n   已到达第 2 个点
//     R_ERR   -> $ERR,01\n   错误码 1
//     R_WP    -> $WPT,02\n   从第 2 点续跑
//     R_MOV   -> $MOV,00\n   手动动作已接受
//   ⇒ 原始串口监视器里也能直接读懂；App 再翻译成中文（见方案 §13.4）。
//
// 【遥测帧格式】v3（2026-10-14 加电量/速度，与 webapp/index.html 的解析器对齐）
//   $TEL,±xxxx,±yyyy,tttt,ww,s,±vvvv,±wwww,bbb\n   43 字节（定长零填充）
//     x,y   = mm    (4 位 + 符号)
//     th    = 0.1 度(4 位)
//     wp    = 目标点号(2 位)
//     s     = 状态码(1 位)
//     vx,vy = mm/s  (4 位 + 符号)   ← 新增
//     bbb   = 电池 0.1V(3 位, 126=12.6V) ← 新增
//   ⚠️ 位置表（byte_at 的 case 必须与之一一对应）：
//      0$ 1T 2E 3L 4, | 5sgn 6..9 x | 10, | 11sgn 12..15 y | 16, |
//      17..20 th | 21, | 22..23 wp | 24, | 25 s | 26, |
//      27sgn 28..31 vx | 32, | 33sgn 34..37 vy | 38, | 39..41 bbb | 42 \n
//   ⇒ 43 B @115200 ≈ 3.73 ms/帧；BLE 小包 20B ⇒ 拆 3 包（App 按 \n 行缓冲重组）
//   ★ 旧 v2 帧 27 字节只有前 6 字段；加了 vx/vy/batt 后**必须同步检查**
//     App 解析器与 tb 的字节位置断言。
//
// 【握手】一次只发 1 字节: S_X 置 send_en -> S_XD 等 tx_done -> 下一个。
//   ★ 踩过的坑: 曾"置 send_en 的同拍就递增位置" ⇒ 字节整体错位 1 格。
//=============================================================================
`timescale 1ns/1ps

module bt_telem2 #(
    parameter integer PERIOD = 5_000_000     // 10Hz @50MHz（tb 改小加速）
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        pong_req,             // 1 拍: 回 $PONG
    input  wire        stat_req,             // 1 拍: **立即**回一帧 $TEL（$STAT 用，不等 10Hz 周期）
    input  wire        resp_vld,             // 1 拍: 发一条应答帧
    input  wire [3:0]  resp_type,            // 1..9 (见上)
    input  wire [4:0]  resp_arg,             // 附带数值(点号/错误码/计数)
    input  wire        tel_en,               // 1: 使能 10Hz 遥测
    input  wire signed [31:0] x_q16,         // mm  Q16.16
    input  wire signed [31:0] y_q16,
    input  wire signed [31:0] th_q16,        // 度  Q16.16 (0~360)
    input  wire        [4:0]  wp,            // 当前目标点号
    input  wire        [2:0]  st_in,         // 状态码
    input  wire signed [31:0] vx_q16,        // mm/s Q16.16   ← 新增（odom 输出）
    input  wire signed [31:0] vy_q16,        // mm/s Q16.16   ← 新增
    input  wire        [7:0]  batt_01v,
    /*  v74：$DBG 观测帧输入（来自 nav_ctrl.dbg_*，纯透传） */
    input  wire       [2:0]  dbg_phase,
    input  wire       [1:0]  dbg_sub,
    input  wire       [2:0]  dbg_step,
    input  wire       [1:0]  dbg_near,
    input  wire       [1:0]  dbg_far,
    input  wire              dbg_unk,
    input  wire signed [15:0] dbg_samp,
    input  wire signed [31:0] dbg_odo,
    input  wire signed [31:0] dbg_w,
    input  wire       [1:0]  dbg_gear,      // 电池电压 0.1V ← 新增（vbat_mon 输出）
    output reg  [7:0]  tx_byte,
    output reg         tx_send_en,
    input  wire        tx_done,
    output reg  [15:0] tel_cnt,              // 已发 $TEL 帧数（存活判据）
    output reg  [7:0]  pong_cnt,             // 已回 $PONG 次数
    output reg  [15:0] resp_cnt              // 已发应答帧数
);
    reg [27:0] per_cnt;

    reg signed [31:0] L_x, L_y;
    reg        [31:0] L_th;
    reg        [4:0]  L_wp;
    reg        [2:0]  L_st;
    reg signed [31:0] L_vx, L_vy;            // mm/s（整数，已从 Q16.16 移位）
    reg        [7:0]  L_vb;                  // 电池 0.1V
    reg        [3:0]  L_rt;                  // 锁存的应答类型
    reg        [4:0]  L_ra;                  // 锁存的应答参数

    wire [31:0] ux = L_x[31] ? (32'sd0 - L_x) : L_x;
    wire [31:0] uy = L_y[31] ? (32'sd0 - L_y) : L_y;
    wire [31:0] ut = L_th;
    wire [31:0] uvx = L_vx[31] ? (32'sd0 - L_vx) : L_vx;
    wire [31:0] uvy = L_vy[31] ? (32'sd0 - L_vy) : L_vy;
    wire [3:0] x3=(ux/1000)%10, x2=(ux/100)%10, x1=(ux/10)%10, x0=ux%10;
    wire [3:0] y3=(uy/1000)%10, y2=(uy/100)%10, y1=(uy/10)%10, y0=uy%10;
    wire [3:0] t3=(ut/1000)%10, t2=(ut/100)%10, t1=(ut/10)%10, t0=ut%10;
    wire [3:0] w1=(L_wp/10)%10, w0=L_wp%10;
    wire [3:0] s0 = L_st%10;
    wire [3:0] A3=(uvx/1000)%10, A2=(uvx/100)%10, A1=(uvx/10)%10, A0=uvx%10;
    wire [3:0] B3=(uvy/1000)%10, B2=(uvy/100)%10, B1=(uvy/10)%10, B0=uvy%10;
    wire [3:0] b2=(L_vb/100)%10, b1=(L_vb/10)%10, b0=L_vb%10;

    function [7:0] byte_at(input [5:0] p);
        begin
            case (p)
                6'd0 : byte_at = 8'h24;  6'd1 : byte_at = 8'h54;
                6'd2 : byte_at = 8'h45;  6'd3 : byte_at = 8'h4C;
                6'd4 : byte_at = 8'h2C;
                6'd5 : byte_at = L_x[31] ? 8'h2D : 8'h2B;
                6'd6 : byte_at = 8'h30 + x3;  6'd7 : byte_at = 8'h30 + x2;
                6'd8 : byte_at = 8'h30 + x1;  6'd9 : byte_at = 8'h30 + x0;
                6'd10: byte_at = 8'h2C;
                6'd11: byte_at = L_y[31] ? 8'h2D : 8'h2B;
                6'd12: byte_at = 8'h30 + y3;  6'd13: byte_at = 8'h30 + y2;
                6'd14: byte_at = 8'h30 + y1;  6'd15: byte_at = 8'h30 + y0;
                6'd16: byte_at = 8'h2C;
                6'd17: byte_at = 8'h30 + t3;  6'd18: byte_at = 8'h30 + t2;
                6'd19: byte_at = 8'h30 + t1;  6'd20: byte_at = 8'h30 + t0;
                6'd21: byte_at = 8'h2C;
                6'd22: byte_at = 8'h30 + w1;  6'd23: byte_at = 8'h30 + w0;
                6'd24: byte_at = 8'h2C;
                6'd25: byte_at = 8'h30 + s0;
                // ---- v3 新增：vx / vy / batt ----
                6'd26: byte_at = 8'h2C;
                6'd27: byte_at = L_vx[31] ? 8'h2D : 8'h2B;
                6'd28: byte_at = 8'h30 + A3;  6'd29: byte_at = 8'h30 + A2;
                6'd30: byte_at = 8'h30 + A1;  6'd31: byte_at = 8'h30 + A0;
                6'd32: byte_at = 8'h2C;
                6'd33: byte_at = L_vy[31] ? 8'h2D : 8'h2B;
                6'd34: byte_at = 8'h30 + B3;  6'd35: byte_at = 8'h30 + B2;
                6'd36: byte_at = 8'h30 + B1;  6'd37: byte_at = 8'h30 + B0;
                6'd38: byte_at = 8'h2C;
                6'd39: byte_at = 8'h30 + b2;  6'd40: byte_at = 8'h30 + b1;
                6'd41: byte_at = 8'h30 + b0;
                default: byte_at = 8'h0A;                 // p = 42 落在 \n
            endcase
        end
    endfunction

    function [7:0] pong_at(input [2:0] p);
        begin
            case (p)
                3'd0: pong_at = 8'h24;  3'd1: pong_at = 8'h50;
                3'd2: pong_at = 8'h4F;  3'd3: pong_at = 8'h4E;
                3'd4: pong_at = 8'h47;  default: pong_at = 8'h0A;
            endcase
        end
    endfunction

    // 应答 TAG（3 字符, 打包成 24 位; "ADD" = 0x414444）
    function [23:0] tag_of(input [3:0] t);
        begin
            case (t)
                4'd1: tag_of = "ADD";   4'd2: tag_of = "CLR";
                4'd3: tag_of = "RDY";   4'd4: tag_of = "RUN";
                4'd5: tag_of = "STP";   4'd6: tag_of = "DON";
                4'd7: tag_of = "ERR";   4'd8: tag_of = "WPT";
                4'd9: tag_of = "MOV";   4'd10: tag_of = "EVT";
                default: tag_of = "UNK";
            endcase
        end
    endfunction

    function [7:0] resp_at(input [2:0] p);
        reg [23:0] tg;
        begin
            tg = tag_of(L_rt);
            case (p)
                3'd0: resp_at = 8'h24;              // '$'
                3'd1: resp_at = tg[23:16];
                3'd2: resp_at = tg[15:8];
                3'd3: resp_at = tg[7:0];
                3'd4: resp_at = 8'h2C;              // ','
                3'd5: resp_at = 8'h30 + ((L_ra/10)%10);
                3'd6: resp_at = 8'h30 + (L_ra%10);
                default: resp_at = 8'h0A;
            endcase
        end
    endfunction

    localparam [3:0] S_IDLE=4'd0, S_TX=4'd1, S_TXD=4'd2,
                     S_PG=4'd3,  S_PGD=4'd4, S_RS=4'd5, S_RSD=4'd6,
                     S_DG=4'd7,  S_DGD=4'd8;   /* v74：$DBG 帧 */
    reg [3:0] state;
    reg [5:0] tpos;                 // v3: 0..42（帧长 43）⇒ 必须 6 位
    reg [2:0] ppos;
    reg [2:0] rpos;
    reg       pong_pending;
    reg       resp_pending;
    reg       stat_pending;      // $STAT：锁存后立即发一帧 $TEL

    // ---------- v74：$DBG 帧（$TEL 结束后紧跟一帧，纯观测）----------
    reg [5:0]  dpos;
    reg [2:0]  D_ph;
    reg [1:0]  D_sub;
    reg [2:0]  D_step;
    reg [1:0]  D_near, D_far;
    reg        D_unk;
    reg [15:0] D_samp;
    reg signed [31:0] D_odo, D_w;   // Q16.16
    reg [1:0]  D_gear;

    wire [31:0] d_odo_u = D_odo[31] ? (32'sd0 - D_odo) : D_odo;
    wire [31:0] d_odo_q = d_odo_u >>> 16;
    wire [31:0] d_odo_i = (d_odo_q > 32'd9999) ? 32'd9999 : d_odo_q;
    wire [31:0] d_w_u   = D_w[31] ? (32'sd0 - D_w) : D_w;
    wire [31:0] d_w_q   = d_w_u >>> 16;
    wire [31:0] d_w_i   = (d_w_q > 32'd999) ? 32'd999 : d_w_q;
    wire [31:0] d_s_i   = D_samp[15] ? 32'd999 : {16'd0, D_samp};
    wire [31:0] d_s_c   = (d_s_i > 32'd999) ? 32'd999 : d_s_i;
    wire [3:0]  sm2=(d_s_c/100)%10, sm1=(d_s_c/10)%10, sm0=d_s_c%10;
    wire [3:0]  od3=(d_odo_i/1000)%10, od2=(d_odo_i/100)%10,
                od1=(d_odo_i/10)%10,   od0=d_odo_i%10;
    wire [3:0]  wd2=(d_w_i/100)%10, wd1=(d_w_i/10)%10, wd0=d_w_i%10;

    /*  $DBG,ph,sub,step,near,far,unk,samp,odo_deg,w_dps,gear\n   34 字节
        0 $ 1 D 2 B 3 G 4 , | 5 ph | 6 , | 7 sub | 8 , | 9 step | 10 , |
        11 near | 12 , | 13 far | 14 , | 15 unk | 16 , | 17..19 samp | 20 , |
        21 sgn 22..25 odo | 26 , | 27 sgn 28..30 w | 31 , | 32 gear | 33 \n  */
    function [7:0] dbg_at(input [5:0] p);
        begin
            case (p)
                6'd0: dbg_at = 8'h24;  6'd1: dbg_at = 8'h44;
                6'd2: dbg_at = 8'h42;  6'd3: dbg_at = 8'h47;
                6'd4: dbg_at = 8'h2C;
                6'd5: dbg_at = 8'h30 + {5'd0, D_ph};
                6'd6: dbg_at = 8'h2C;
                6'd7: dbg_at = 8'h30 + {6'd0, D_sub};
                6'd8: dbg_at = 8'h2C;
                6'd9: dbg_at = 8'h30 + {5'd0, D_step};
                6'd10: dbg_at = 8'h2C;
                6'd11: dbg_at = 8'h30 + {6'd0, D_near};
                6'd12: dbg_at = 8'h2C;
                6'd13: dbg_at = 8'h30 + {6'd0, D_far};
                6'd14: dbg_at = 8'h2C;
                6'd15: dbg_at = 8'h30 + {7'd0, D_unk};
                6'd16: dbg_at = 8'h2C;
                6'd17: dbg_at = 8'h30 + sm2;  6'd18: dbg_at = 8'h30 + sm1;
                6'd19: dbg_at = 8'h30 + sm0;
                6'd20: dbg_at = 8'h2C;
                6'd21: dbg_at = D_odo[31] ? 8'h2D : 8'h2B;
                6'd22: dbg_at = 8'h30 + od3;  6'd23: dbg_at = 8'h30 + od2;
                6'd24: dbg_at = 8'h30 + od1;  6'd25: dbg_at = 8'h30 + od0;
                6'd26: dbg_at = 8'h2C;
                6'd27: dbg_at = D_w[31] ? 8'h2D : 8'h2B;
                6'd28: dbg_at = 8'h30 + wd2;  6'd29: dbg_at = 8'h30 + wd1;
                6'd30: dbg_at = 8'h30 + wd0;
                6'd31: dbg_at = 8'h2C;
                6'd32: dbg_at = 8'h30 + {6'd0, D_gear};
                default: dbg_at = 8'h0A;                 // p = 33 落在 \n
            endcase
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE; per_cnt <= 28'd0; tpos <= 6'd0; ppos <= 3'd0; rpos <= 3'd0;
            tx_send_en <= 1'b0; tx_byte <= 8'h00;
            tel_cnt <= 16'd0; pong_cnt <= 8'd0; resp_cnt <= 16'd0;
            L_x <= 32'sd0; L_y <= 32'sd0; L_th <= 32'd0; L_wp <= 5'd0; L_st <= 3'd0;
            L_vx <= 32'sd0; L_vy <= 32'sd0; L_vb <= 8'd0;
            L_rt <= 4'd0; L_ra <= 5'd0;
            pong_pending <= 1'b0; resp_pending <= 1'b0; stat_pending <= 1'b0;
        end
        else begin
            tx_send_en <= 1'b0;
            // ---- 请求锁存（帧边界才响应, 保证当前帧完整）----
            if (pong_req) pong_pending <= 1'b1;
            if (stat_req) stat_pending <= 1'b1;     // 任何状态都能收，帧边界才发
            if (resp_vld) begin
                L_rt <= resp_type; L_ra <= resp_arg; resp_pending <= 1'b1;
            end

            case (state)
            S_IDLE: begin
                per_cnt <= per_cnt + 28'd1;
                if (pong_pending) begin
                    pong_pending <= 1'b0;
                    ppos <= 3'd0;
                    state <= S_PG;
                end
                else if (resp_pending) begin
                    resp_pending <= 1'b0;
                    rpos <= 3'd0;
                    state <= S_RS;
                end
                else if (stat_pending) begin
                    // ★ $STAT = "当前位置"按钮：立即回一帧，不等 10Hz 周期
                    stat_pending <= 1'b0;
                    L_x  <= x_q16 >>> 16;
                    L_y  <= y_q16 >>> 16;
                    L_th <= (th_q16 * 7'sd10) >>> 16;
                    L_wp <= wp;
                    L_st <= st_in;
                    L_vx <= vx_q16 >>> 16;
                    L_vy <= vy_q16 >>> 16;
                    L_vb <= batt_01v;
                    per_cnt <= 28'd0;        // 重置周期，避免紧接着又补一帧
                    tpos    <= 6'd0;
                    state   <= S_TX;
                end
                else if (tel_en && (per_cnt >= PERIOD[27:0])) begin
                    L_x  <= x_q16 >>> 16;
                    L_y  <= y_q16 >>> 16;
                    L_th <= (th_q16 * 7'sd10) >>> 16;
                    L_wp <= wp;
                    L_st <= st_in;
                    L_vx <= vx_q16 >>> 16;
                    L_vy <= vy_q16 >>> 16;
                    L_vb <= batt_01v;
                    per_cnt <= 28'd0;
                    tpos    <= 6'd0;
                    state   <= S_TX;
                end
            end
            S_TX: begin
                tx_byte    <= byte_at(tpos);
                tx_send_en <= 1'b1;
                state      <= S_TXD;
            end
            S_TXD: if (tx_done) begin
                if (tpos >= 6'd42) begin
                    tel_cnt <= tel_cnt + 16'd1;
                    per_cnt <= 28'd0;
                    /* v74：$TEL 帧尾锁存 DBG 快照，紧跟一帧 $DBG（无抢占、逐帧完整） */
                    D_ph   <= dbg_phase;  D_sub  <= dbg_sub;
                    D_step <= dbg_step;   D_near <= dbg_near;  D_far <= dbg_far;
                    D_unk  <= dbg_unk;    D_samp <= {1'b0, dbg_samp[14:0]};
                    D_odo  <= dbg_odo;    D_w    <= dbg_w;     D_gear <= dbg_gear;
                    dpos   <= 6'd0;
                    state  <= S_DG;
                end else begin
                    tpos  <= tpos + 6'd1;
                    state <= S_TX;
                end
            end
            S_PG: begin
                tx_byte    <= pong_at(ppos);
                tx_send_en <= 1'b1;
                state      <= S_PGD;
            end
            S_PGD: if (tx_done) begin
                if (ppos >= 3'd5) begin
                    pong_cnt <= pong_cnt + 8'd1;
                    per_cnt  <= 28'd0;
                    state    <= S_IDLE;
                end else begin
                    ppos  <= ppos + 3'd1;
                    state <= S_PG;
                end
            end
            S_RS: begin
                tx_byte    <= resp_at(rpos);
                tx_send_en <= 1'b1;
                state      <= S_RSD;
            end
            S_RSD: if (tx_done) begin
                if (rpos >= 3'd7) begin
                    resp_cnt <= resp_cnt + 16'd1;
                    state    <= S_IDLE;
                end else begin
                    rpos  <= rpos + 3'd1;
                    state <= S_RS;
                end
            end
            /* ---------- v74：$DBG 帧发送 ---------- */
            S_DG: begin
                tx_byte    <= dbg_at(dpos);
                tx_send_en <= 1'b1;
                state      <= S_DGD;
            end
            S_DGD: if (tx_done) begin
                if (dpos >= 6'd33) begin
                    state <= S_IDLE;
                end else begin
                    dpos  <= dpos + 6'd1;
                    state <= S_DG;
                end
            end
            default: state <= S_IDLE;
            endcase
        end
    end

endmodule
