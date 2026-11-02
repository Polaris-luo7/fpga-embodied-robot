//=============================================================================
// tb_bt_telem2.v -- 遥测发送器用例
//   tb 自带"假 UART 发送器": 收到 tx_send_en 就收字节, 若干拍后回 tx_done。
//   解码出的字节流与期望字符串逐字符比对（不依赖被测模块内部逻辑）。
//=============================================================================
`timescale 1ns/1ps

module tb_bt_telem2;
    localparam integer PERIOD = 200;      // tb 加速(真机 5_000_000)

    reg clk=0, rst_n=0, pong_req=0, tel_en=0, stat_req=0;
    reg resp_vld=0; reg [3:0] resp_type=0; reg [4:0] resp_arg=0;
    reg signed [31:0] x_q16=0, y_q16=0, th_q16=0;
    reg [4:0] wp=0; reg [2:0] st_in=0;
    reg signed [31:0] vx_q16=0, vy_q16=0;   // ★ v3：速度
    reg [7:0] batt_01v=0;                   // ★ v3：电池 0.1V
    wire [7:0] tx_byte; wire tx_send_en; reg tx_done=0;
    wire [15:0] tel_cnt; wire [7:0] pong_cnt; wire [15:0] resp_cnt;

    bt_telem2 #(.PERIOD(PERIOD)) dut (
        .clk(clk), .rst_n(rst_n), .pong_req(pong_req), .stat_req(stat_req),
        .resp_vld(resp_vld), .resp_type(resp_type), .resp_arg(resp_arg),
        .tel_en(tel_en),
        .x_q16(x_q16), .y_q16(y_q16), .th_q16(th_q16), .wp(wp), .st_in(st_in),
        .vx_q16(vx_q16), .vy_q16(vy_q16), .batt_01v(batt_01v),
        .tx_byte(tx_byte), .tx_send_en(tx_send_en), .tx_done(tx_done),
        .tel_cnt(tel_cnt), .pong_cnt(pong_cnt), .resp_cnt(resp_cnt)
    );

    always #10 clk = ~clk;

    // ---- 假 UART 发送器: 收集字节 + 产生 tx_done ----
    reg [7:0] rx [0:63];
    integer   rxn = 0;
    reg [3:0] done_cnt = 0;
    always @(posedge clk) begin
        tx_done <= 1'b0;
        if (tx_send_en) begin
            rx[rxn] <= tx_byte;          // 采集发送字节
            rxn     <= rxn + 1;
            done_cnt<= 4'd5;             // 模拟 5 拍发送耗时
        end
        else if (done_cnt > 0) begin
            done_cnt <= done_cnt - 1;
            if (done_cnt == 1) tx_done <= 1'b1;
        end
    end

    // ---- 期望字符串比对 ----
    integer i, fails = 0;
    reg [8*64-1:0] expect_str;
    integer explen;
    task clear_rx; begin rxn = 0; end endtask

    // 关遥测 -> 等当前帧排空 -> 清采集缓冲 -> 重新开遥测
    task drain(input reg keep_on);
        begin
            tel_en = 0;
            // ★等发送器真正回到 IDLE，避免"最后一个在途字节"落进下一用例
            while (dut.state != 3'd0) @(posedge clk);
            repeat (30) @(posedge clk);
            rxn = 0;
            if (keep_on) tel_en = 1;
        end
    endtask

    integer nb;
    task cmp_frame(input [8*40-1:0] name);
        begin
            nb = (rxn < explen) ? rxn : explen;
            $display("--- %0s: 收到 %0d 字节, 期望 %0d ---", name, rxn, explen);
            for (i = 0; i < nb; i = i + 1) $write("%c", rx[i]);
            $write("\n");
            if (rxn < explen) begin
                fails = fails + 1;
                $display("[FAIL] %0s 字节数不足: 收到 %0d < 期望 %0d", name, rxn, explen);
            end
            for (i = 0; i < nb; i = i + 1) begin
                if (rx[i] !== expect_str[8*(explen-1-i) +: 8]) begin
                    fails = fails + 1;
                    $display("[FAIL] %0s byte[%0d] got=%02x %c want=%02x %c", name, i,
                             rx[i], rx[i], expect_str[8*(explen-1-i) +: 8],
                             expect_str[8*(explen-1-i) +: 8]);
                end
            end
            if (fails == 0) $display("[PASS] %0s (%0d 字节逐字节一致)", name, explen);
        end
    endtask

    initial begin
        $display("=== tb_bt_telem2 start ===");
        repeat (5) @(posedge clk); rst_n = 1; repeat (3) @(posedge clk);

        //--------------------------------------------------- T1 遥测帧
        x_q16 = 32'sd1500 << 16;      // +1500 mm
        y_q16 = -32'sd250 << 16;      // -250 mm
        th_q16= 32'sd35   << 16;      // 35.0 度 => 0.1度 = 350
        wp    = 5'd2; st_in = 3'd1;
        vx_q16= 32'sd200  << 16;      // +200 mm/s
        vy_q16= -32'sd150 << 16;      // -150 mm/s
        batt_01v = 8'd126;            // 12.6 V
        tel_en= 1'b1;
        clear_rx;
        repeat (PERIOD + 900) @(posedge clk);   // 等一帧(43B)发完
        expect_str = "$TEL,+1500,-0250,0350,02,1,+0200,-0150,126\n"; explen = 43;
        cmp_frame("T1_TEL");
        tel_en = 0;                              // 关闭遥测并等流排空
        repeat (400) @(posedge clk);

        //--------------------------------------------------- T2 $PONG
        drain(1'b0); fails = 0;
        @(posedge clk); pong_req = 1;
        @(posedge clk); pong_req = 0;
        repeat (200) @(posedge clk);
        expect_str = "$PONG\n"; explen = 6;
        cmp_frame("T2_PONG");
        tel_en = 0; repeat (200) @(posedge clk);

        //--------------------------------------------------- T3 关闭遥测
        drain(1'b0);
        tel_en = 0;
        repeat (PERIOD * 3) @(posedge clk);
        if (rxn == 0) $display("[PASS] T3_TEL_OFF (关闭后未发帧)");
        else begin fails = fails + 1;
            $display("[FAIL] T3_TEL_OFF 关闭后仍发了 %0d 字节", rxn); end

        //--------------------------------------------------- T4 零值/满值边界
        x_q16 = 0; y_q16 = 0; th_q16 = 0; wp = 0; st_in = 0;
        vx_q16 = 0; vy_q16 = 0; batt_01v = 8'd0;
        drain(1'b0); fails = 0; tel_en = 1'b1;
        repeat (PERIOD + 900) @(posedge clk);
        expect_str = "$TEL,+0000,+0000,0000,00,0,+0000,+0000,000\n"; explen = 43;
        cmp_frame("T4_zero");
        tel_en = 0; repeat (400) @(posedge clk);

        //--------------------------------------------------- T5 大角度
        x_q16 = -32'sd9999 << 16 >> 0; y_q16 = 32'sd9999 << 16;
        th_q16 = 32'sd359 << 16; wp = 5'd16; st_in = 3'd5;
        vx_q16 = -32'sd300 << 16; vy_q16 = 32'sd300 << 16; batt_01v = 8'd98;
        drain(1'b0); fails = 0; tel_en = 1'b1;
        repeat (PERIOD + 900) @(posedge clk);
        expect_str = "$TEL,-9999,+9999,3590,16,5,-0300,+0300,098\n"; explen = 43;
        cmp_frame("T5_max");

        //--------------------------------------------------- T6 应答帧 $DON
        drain(1'b0); fails = 0; tel_en = 0;
        resp_type = 4'd6; resp_arg = 5'd2;      // R_DONE, 点 2
        @(posedge clk); resp_vld = 1;
        @(posedge clk); resp_vld = 0;
        repeat (150) @(posedge clk);
        expect_str = "$DON,02\n"; explen = 8;
        cmp_frame("T6_DONE");

        //--------------------------------------------------- T7 应答帧 $RDY,03
        drain(1'b0); fails = 0; tel_en = 0;
        resp_type = 4'd3; resp_arg = 5'd3;      // R_READY, 3 个点
        @(posedge clk); resp_vld = 1;
        @(posedge clk); resp_vld = 0;
        repeat (150) @(posedge clk);
        expect_str = "$RDY,03\n"; explen = 8;
        cmp_frame("T7_READY");

        //------------------------------ T8 遥测帧中途来应答 -> 当前帧发完再发应答
        drain(1'b0); fails = 0;
        x_q16 = 0; y_q16 = 0; th_q16 = 0; wp = 0; st_in = 0;
        vx_q16 = 0; vy_q16 = 0; batt_01v = 8'd0;
        tel_en = 1'b1;
        repeat (PERIOD + 100) @(posedge clk);   // 让 TEL 帧发到一半
        resp_type = 4'd7; resp_arg = 5'd1;      // R_ERR, 错误码 1
        @(posedge clk); resp_vld = 1;
        @(posedge clk); resp_vld = 0;
        tel_en = 1'b0;                           // 之后不再发 TEL
        repeat (500) @(posedge clk);
        expect_str = "$TEL,+0000,+0000,0000,00,0,+0000,+0000,000\n$ERR,01\n"; explen = 51;
        cmp_frame("T8_resp_in_tel");

        //----------------------------------------------- T9 $STAT -> 立即回一帧 $TEL
        $display("[T9] stat_req -> immediate $TEL (no 10Hz wait)");
        drain(1'b0); fails = 0; tel_en = 1'b0;      // 周期遥测关掉，验证"立即"
        x_q16 = 32'sd1234 <<< 16; y_q16 = 32'sd0; th_q16 = 32'sd0;
        wp = 5'd2; st_in = 3'd2;
        vx_q16 = 32'sd100 <<< 16; vy_q16 = 32'sd0; batt_01v = 8'd120;
        @(posedge clk); stat_req = 1;
        @(posedge clk); stat_req = 0;
        repeat (500) @(posedge clk);
        expect_str = "$TEL,+1234,+0000,0000,02,2,+0100,+0000,120\n"; explen = 43;
        cmp_frame("T9_STAT");

        //----------------------------------------------- T10 EVT 标签（P4 事件上报用）
        $display("[T10] resp_type=10 -> $EVT tag");
        drain(1'b0); fails = 0; tel_en = 1'b0;
        resp_type = 4'd10; resp_arg = 5'd3;         // $EVT,03
        @(posedge clk); resp_vld = 1;
        @(posedge clk); resp_vld = 0;
        repeat (150) @(posedge clk);
        expect_str = "$EVT,03\n"; explen = 8;
        cmp_frame("T10_EVT");

        $display("=== tb_bt_telem2 done: fails=%0d ===", fails);
        $finish;
    end
endmodule
