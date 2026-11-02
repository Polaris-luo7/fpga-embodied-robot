//=============================================================================
//  tb_bt_link.v —— bt_link 行为仿真（任务四 PB 阶段）
//-----------------------------------------------------------------------------
//  【两条铁律】
//    ① 自带独立的发送器与解码器 —— 绝不引用 DUT 内部信号、绝不复用 DUT 的
//       bt_rx_byte / bt_tx_byte。否则"同错同错"，仿真过了也说明不了问题。
//       （解码器沿用 rtl/uart_diag/tb_uart_diag.v 已验证有效的做法：
//          negedge 触发 + 绝对延时对准位中心 + busy 防重入 + 停止位校验）
//    ② 用**参数加速**，不用 force：DUT 例化时传 TIMEOUT_GAP=2000（40 µs），
//       逻辑完全相同，只是不必让仿真真等 304 µs。
//
//  【时序基准】115200 8N1 @50 MHz ⇒ 1 bit = 8680.56 ns；1 字节 = 10 bit = 86.81 µs
//    send_byte 每字节 = 10 个位周期 = 86.81 µs（**背靠背**，不留额外空隙，
//    与手机 App 一次发一整条时蓝牙模块从 UART 吐字节的真实情况一致）
//    回一帧 "$PONG\n" = 6 字节 × 10 bit = 521 µs
//    TIMEOUT_GAP 用**出厂真值 15190 拍 = 303.8 µs**（不加速；加速下限必须 > 4340 拍）
//    ⇒ 每次"发完 + 等应答"的等待 ≥ 700 µs；TC2 还要多等一个 TIMEOUT_GAP ⇒ 1000 µs
//
//  【四个用例】
//    TC1 正常帧     发 "$PING\n"    ⇒ 应回 "$PONG\n"，pong_cnt=1
//    TC2 超时断帧   只发 "$PING"     ⇒ 静默超时后仍应回 "$PONG\n"，pong_cnt=2
//    TC3 干扰不应答 "PING\n" / "$PIN\n" / "$PINGX\n" / 垃圾 ⇒ 一律不得回 PONG
//    TC4 CRLF 容错  发 "$PING\r\n"  ⇒ 应回 "$PONG\n"，pong_cnt=3
//  最终断言：pong_cnt 恰好 = 3，且停止位错误数 = 0
//
//  【注意】$display 全部用 ASCII —— ModelSim 控制台编码不可控，中文会变乱码，
//          注释里保留中文即可（注释不打印）。
//
//  【版本】2026-10-12 WorkBuddy（任务四 PB 阶段）
//=============================================================================
`timescale 1ns/1ps

module tb_bt_link;

    //------------------------------------------------------------------
    // 时钟 / 复位
    //------------------------------------------------------------------
    reg clk = 1'b0;
    always #10 clk = ~clk;              // 20 ns = 50 MHz

    reg  rst_n = 1'b0;
    reg  bt_rx = 1'b1;                  // UART 空闲为高

    wire       bt_tx;
    wire [7:0] o_rx_byte;
    wire       o_rx_byte_vld;
    wire [5:0] o_frame_len;
    wire       o_frame_vld;
    wire [255:0] o_frame_bus;            // bt_link 引出的整帧（给 bt_cmd2 解码）
    // ---- bt_cmd2 的输出：回灌 bt_link 的 i_cmd（★单一解码器）----
    wire [7:0]         t_cmd;
    wire signed [31:0] t_arg0, t_arg1;
    wire               t_cmd_vld;
    wire [7:0] o_pong_cnt;

    //------------------------------------------------------------------
    // DUT —— 用参数加速超时，不用 force
    //------------------------------------------------------------------
    bt_link #(
        // ★ 这里**不加速**，直接用出厂真值 15190 —— 测的就是要烧进去的那个数。
        //   （曾试过加速到 2000 拍 = 40 µs，结果比一个字节的传输时间
        //     4340 拍 = 86.8 µs 还短 ⇒ 每个字节都被判成独立帧，全链失效。
        //     结论：TIMEOUT_GAP 的加速下限必须 > 一个字节周期 4340 拍。）
        .TIMEOUT_GAP (15190),           // 115200 档真值（教材 ch9 Baud_Set=4）
        .FRAME_MAX   (32  )
    ) dut (
        .clk           (clk           ),
        .rst_n         (rst_n         ),
        // 遥测输入: 本 tb 专测 $PING/$PONG 链路, 遥测全关(避免干扰字节流)
        .i_x_q16       (32'sd0        ),
        .i_y_q16       (32'sd0        ),
        .i_th_q16      (32'sd0        ),
        .i_wp          (5'd0          ),
        .i_st          (3'd0          ),
        .i_vx_q16      (32'sd0        ),   // v3：速度/电量也全零
        .i_vy_q16      (32'sd0        ),
        .i_batt_01v    (8'd0          ),
        .i_cmd         (t_cmd    ),
        .i_arg0        (t_arg0   ),
        .i_cmd_vld     (t_cmd_vld),
        .i_resp_vld    (1'b0          ),   // 本 tb 不测应答帧(bt_telem2 的 tb 覆盖)
        .i_resp_type   (4'd0          ),
        .i_resp_arg    (5'd0          ),
        .o_tel_cnt     (              ),
        .o_frame       (o_frame_bus),
        .o_resp_cnt    (              ),
        .bt_rx         (bt_rx         ),
        .bt_tx         (bt_tx         ),
        .o_rx_byte     (o_rx_byte     ),
        .o_rx_byte_vld (o_rx_byte_vld ),
        .o_frame_len   (o_frame_len   ),
        .o_frame_vld   (o_frame_vld   ),
        .o_pong_cnt    (o_pong_cnt    )
    );

    // ---- 命令解码级：bt_cmd2 吃 bt_link 引出的整帧，回灌 i_cmd ----
    bt_cmd2 #( .FRAME_MAX(32) ) u_cmd2 (
        .clk       (clk        ),
        .rst_n     (rst_n      ),
        .frame     (o_frame_bus),
        .frame_len (o_frame_len),
        .frame_vld (o_frame_vld),
        .cmd       (t_cmd      ),
        .arg0      (t_arg0     ),
        .arg1      (t_arg1     ),
        .cmd_vld   (t_cmd_vld  )
    );

    localparam real BIT_NS = 8680.56;   // 115200 baud @ 50 MHz

    //==================================================================
    //  独立发送器（手机侧模型）—— 不引用 DUT 任何内部逻辑
    //==================================================================
    task send_byte(input [7:0] b);
        integer k;
        begin
            bt_rx = 1'b0;               // 起始位
            #(BIT_NS);
            for (k = 0; k < 8; k = k + 1) begin
                bt_rx = b[k];           // LSB 先发
                #(BIT_NS);
            end
            bt_rx = 1'b1;               // 停止位
            #(BIT_NS);
            // ★ 不留额外空隙：字节**背靠背**发送，与手机 App「一次发一整条」时
            //   蓝牙模块从 UART 吐字节的真实情况一致。
        end
    endtask

    // 发一段字符串：s 里是右对齐的 n 个字节（Verilog 字符串字面量的存放方式）
    task send_str(input [127:0] s, input integer n);
        integer k;
        reg [7:0] bb;
        begin
            for (k = 0; k < n; k = k + 1) begin
                bb = s[(n - 1 - k)*8 +: 8];
                send_byte(bb);
            end
        end
    endtask

    //==================================================================
    //  独立解码器（PC/手机侧模型）—— negedge 触发 + 绝对延时对准位中心
    //==================================================================
    localparam integer MAXCH = 64;
    reg [7:0]  rx_sh;
    reg [7:0]  rxbuf [0:MAXCH-1];
    integer    nch      = 0;
    integer    stop_err = 0;
    reg        dec_busy = 1'b0;

    always @(negedge bt_tx) begin
        if (!dec_busy) begin
            dec_busy = 1'b1;
            #(BIT_NS * 1.5);            // 起始位中线 -> bit0 中线
            rx_sh[0] = bt_tx; #(BIT_NS);
            rx_sh[1] = bt_tx; #(BIT_NS);
            rx_sh[2] = bt_tx; #(BIT_NS);
            rx_sh[3] = bt_tx; #(BIT_NS);
            rx_sh[4] = bt_tx; #(BIT_NS);
            rx_sh[5] = bt_tx; #(BIT_NS);
            rx_sh[6] = bt_tx; #(BIT_NS);
            rx_sh[7] = bt_tx; #(BIT_NS);
            // 此刻位于停止位中线 —— 必须为高
            if (bt_tx !== 1'b1) stop_err = stop_err + 1;
            if (nch < MAXCH) begin
                rxbuf[nch] = rx_sh;
                nch = nch + 1;
            end
            dec_busy = 1'b0;
        end
    end

    //==================================================================
    //  期望值 / 判据
    //==================================================================
    reg [7:0] EXP_PONG [0:5];           // "$PONG\n"
    initial begin
        EXP_PONG[0] = 8'h24;   // '$'
        EXP_PONG[1] = 8'h50;   // 'P'
        EXP_PONG[2] = 8'h4F;   // 'O'
        EXP_PONG[3] = 8'h4E;   // 'N'
        EXP_PONG[4] = 8'h47;   // 'G'
        EXP_PONG[5] = 8'h0A;   // '\n'
    end

    integer tc_pass = 0;
    integer tc_fail = 0;

    // 打印解码器的十六进制内容（不靠肉眼认字符）
    task dump_rx;
        integer k;
        begin
            $write("        decoded %0d bytes: ", nch);
            for (k = 0; k < nch; k = k + 1)
                $write("%02h ", rxbuf[k]);
            $write(" | ");
            for (k = 0; k < nch; k = k + 1)
                if (rxbuf[k] >= 8'h20 && rxbuf[k] < 8'h7F) $write("%c", rxbuf[k]);
                else if (rxbuf[k] == 8'h0A) $write("<LF>");
                else if (rxbuf[k] == 8'h0D) $write("<CR>");
                else $write(".");
            $display("");
        end
    endtask

    // 期望：收到完整 "$PONG\n"，且 pong_cnt == exp_cnt
    task expect_pong(input integer tc_id, input integer exp_cnt);
        integer k;
        reg ok;
        begin
            ok = 1'b1;
            if (nch !== 6) ok = 1'b0;
            else for (k = 0; k < 6; k = k + 1)
                     if (rxbuf[k] !== EXP_PONG[k]) ok = 1'b0;
            if (o_pong_cnt !== exp_cnt) ok = 1'b0;
            if (ok) begin
                $display("  TC%0d PASS   got full $PONG\\n, pong_cnt=%0d", tc_id, o_pong_cnt);
                tc_pass = tc_pass + 1;
            end
            else begin
                $display("  TC%0d FAIL   nch=%0d  pong_cnt=%0d (expected %0d)",
                         tc_id, nch, o_pong_cnt, exp_cnt);
                dump_rx;
                tc_fail = tc_fail + 1;
            end
        end
    endtask

    // 期望：什么都没收到，且 pong_cnt 不变
    task expect_none(input integer tc_id, input integer exp_cnt);
        reg ok;
        begin
            ok = (nch === 0) && (o_pong_cnt === exp_cnt);
            if (ok) begin
                $display("  TC%0d PASS   no reply (correct), pong_cnt still %0d",
                         tc_id, o_pong_cnt);
                tc_pass = tc_pass + 1;
            end
            else begin
                $display("  TC%0d FAIL   should NOT reply! nch=%0d pong_cnt=%0d (expected %0d)",
                         tc_id, nch, o_pong_cnt, exp_cnt);
                dump_rx;
                tc_fail = tc_fail + 1;
            end
        end
    endtask

    task clear_rx;
        begin
            nch = 0;
            rxbuf[0] = 8'h00;
        end
    endtask

    //==================================================================
    //  主流程
    //==================================================================
    initial begin
        $display("=========================================================");
        $display(" tb_bt_link -- task4 PB bluetooth link sim (115200 8N1)");
        $display("=========================================================");
        #200 rst_n = 1'b1;
        #2000;

        //--------------------------------------------------------------
        //  前导：用真实命令 $TEL,0 关掉 10Hz 遥测
        //  ★ v2.0 起遥测上电**默认开**（tel_en_r 复位值 = 1）。本 tb 只测 $PING/$PONG，
        //    必须先用真实命令关掉遥测，否则 27 字节的 $TEL 帧会串进接收缓冲。
        //    顺带验证了 $TEL 这条路。（首帧周期遥测要 100ms 后才发，此处仅 ~700us。）
        //--------------------------------------------------------------
        send_str("$TEL,0", 6);
        #700_000;

        //--------------------------------------------------------------
        // TC1 正常帧：$PING + \n
        //--------------------------------------------------------------
        $display("[TC1] send \"$PING\\n\"  ==> expect \"$PONG\\n\"");
        clear_rx;
        send_str("$PING\n", 6);
        #700_000;                        // 等应答（回 6 字节 = 521 us）
        expect_pong(1, 1);

        //--------------------------------------------------------------
        // TC2 超时断帧：只发 $PING，不发 \n
        //--------------------------------------------------------------
        $display("[TC2] send \"$PING\" (no \\n) ==> timeout-terminated, still expect \"$PONG\\n\"");
        clear_rx;
        send_str("$PING", 5);
        #1_000_000;                      // 静默 304us 触发断帧 + 应答 521us
        expect_pong(2, 2);

        //--------------------------------------------------------------
        // TC3 干扰不应答（4 个子例，共算 1 个 PASS）
        //--------------------------------------------------------------
        $display("[TC3] four kinds of garbage ==> must NOT reply PONG");
        clear_rx;
        send_str("PING\n", 5);           // 缺 '$'
        #600_000;
        send_str("$PIN\n", 5);           // 命令字不全
        #600_000;
        send_str("$PINGX\n", 7);         // 长度不对
        #600_000;
        send_byte(8'hFF); send_byte(8'h00); send_byte(8'h55);   // 纯垃圾
        #600_000;
        expect_none(3, 2);

        //--------------------------------------------------------------
        // TC4 CRLF 容错：$PING + \r + \n
        // ★ 注意：Verilog 的字符串转义表里**没有 \r**（只有 \n \t \\ \" \ddd），
        //   写成 "$PING\r\n" 会被 ModelSim 解释成 "$PING" + 'r' + \n（0x72 而不是 0x0D），
        //   于是报文变成 6 字节的 "$PINGr" —— 本 tb 第一版就栽在这里。
        //   ⇒ CR 必须显式发 8'h0D。
        //--------------------------------------------------------------
        $display("[TC4] send \"$PING\" + CR(0x0D) + LF(0x0A)  ==> expect \"$PONG\\n\"");
        clear_rx;
        send_str("$PING", 5);
        send_byte(8'h0D);                // CR
        send_byte(8'h0A);                // LF
        #700_000;
        expect_pong(4, 3);

        //--------------------------------------------------------------
        // 汇总
        //--------------------------------------------------------------
        $display("---------------------------------------------------------");
        $display(" stop_bit_errors = %0d", stop_err);
        $display(" final pong_cnt  = %0d  (expected 3)", o_pong_cnt);
        if ((o_pong_cnt === 8'd3) && (stop_err === 0) && (tc_fail === 0))
            $display(" >>> ALL PASS : PASS=%0d FAIL=%0d  <<<", tc_pass, tc_fail);
        else
            $display(" >>> FAILURES  : PASS=%0d FAIL=%0d stop_err=%0d pong_cnt=%0d <<<",
                     tc_pass, tc_fail, stop_err, o_pong_cnt);
        $display("=========================================================");
        $finish;
    end

    // 超时保护，防止跑飞
    initial begin
        #50_000_000;
        $display("!!! SIM TIMEOUT GUARD (50 ms) !!!");
        $finish;
    end

endmodule
