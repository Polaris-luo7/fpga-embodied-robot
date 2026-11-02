//=============================================================================
// tb_quad_decoder.v — self-checking testbench for quad_decoder
// Every step produces exactly one valid A/B edge; pipeline is drained before
// each check. Tests: forward count / clear / reverse count / jitter net-zero /
// single cycle / T-method period (fast + slow) / T-method stop timeout.
//=============================================================================
`timescale 1ns/1ps

module tb_quad_decoder;

    localparam T_TO = 5000;   // 本仿真用的停转超时（clk 周期），比默认 5M 小以便快速验证

    reg  clk = 0, rst_n = 0, clr = 0, a = 0, b = 0;
    wire signed [31:0] pos;
    wire dir;
    wire [23:0] period;
    wire period_valid, period_stb;

    quad_decoder #(.CNT_WIDTH(32), .PERIOD_WIDTH(24), .T_TIMEOUT(T_TO)) dut (
        .clk(clk), .rst_n(rst_n), .enc_a(a), .enc_b(b),
        .cnt_clr(clr), .position(pos), .dir(dir),
        .period(period), .period_valid(period_valid), .period_stb(period_stb)
    );

    always #10 clk = ~clk;   // 50 MHz

    // T法 stb 计数（由 stb_clr 清零）
    integer stb_cnt = 0;
    reg     stb_clr = 0;
    always @(posedge clk) begin
        if (stb_clr)      stb_cnt <= 0;
        else if (period_stb) stb_cnt <= stb_cnt + 1;
    end

    // forward sequence: 01 -> 11 -> 10 -> 00 -> 01 (each step = +1)
    function [1:0] fwd_next(input [1:0] s);
        case (s)
            2'b01: fwd_next = 2'b11;
            2'b11: fwd_next = 2'b10;
            2'b10: fwd_next = 2'b00;
            default: fwd_next = 2'b01;
        endcase
    endfunction

    // reverse sequence: 00 -> 10 -> 11 -> 01 -> 00 (each step = -1)
    function [1:0] rev_next(input [1:0] s);
        case (s)
            2'b00: rev_next = 2'b10;
            2'b10: rev_next = 2'b11;
            2'b11: rev_next = 2'b01;
            default: rev_next = 2'b00;
        endcase
    endfunction

    reg [1:0] st;

    // 以 gap 个 clk 为间隔推进一个正交状态（= 一个四倍频边沿）
    task drive_gap(input [1:0] s, input integer gap);
        begin {a, b} = s; repeat(gap) @(posedge clk); end
    endtask

    task drive(input [1:0] s);
        begin drive_gap(s, 4); end
    endtask

    integer i, err;

    initial begin
        err = 0;
        st = 2'b00; {a, b} = 2'b00;
        repeat(5) @(posedge clk);
        rst_n = 1;
        repeat(6) @(posedge clk);

        // ---- T1: forward 40 valid edges -> +40 ----
        for (i = 0; i < 40; i = i + 1) begin st = fwd_next(st); drive(st); end
        repeat(8) @(posedge clk);
        $display("[T1 FWD ] 40 edges -> pos=%0d (expect 40) dir=%b", pos, dir);
        if (pos !== 40)   begin $display("   !! FAIL T1");     err = err + 1; end
        if (dir !== 1'b1) begin $display("   !! FAIL T1 dir"); err = err + 1; end

        // ---- T2: synchronous clear ----
        clr = 1; @(posedge clk); clr = 0; repeat(6) @(posedge clk);
        $display("[T2 CLR ] pos=%0d (expect 0)", pos);
        if (pos !== 0) begin $display("   !! FAIL T2"); err = err + 1; end

        // ---- T3: reverse 28 valid edges -> -28 ----
        for (i = 0; i < 28; i = i + 1) begin st = rev_next(st); drive(st); end
        repeat(8) @(posedge clk);
        $display("[T3 REV ] 28 edges -> pos=%0d (expect -28) dir=%b", pos, dir);
        if (pos !== -28)  begin $display("   !! FAIL T3");     err = err + 1; end
        if (dir !== 1'b0) begin $display("   !! FAIL T3 dir"); err = err + 1; end

        // ---- T4: jitter 00<->01 20 times -> net 0 ----
        drive(2'b00); st = 2'b00; repeat(8) @(posedge clk);
        clr = 1; @(posedge clk); clr = 0; repeat(6) @(posedge clk);
        for (i = 0; i < 20; i = i + 1) begin drive(2'b01); drive(2'b00); end
        repeat(8) @(posedge clk);
        $display("[T4 JIT ] 20 toggles -> pos=%0d (expect 0)", pos);
        if (pos !== 0) begin $display("   !! FAIL T4 (jitter accumulates)"); err = err + 1; end

        // ---- T5: single cycle = 4 edges ----
        clr = 1; @(posedge clk); clr = 0; repeat(6) @(posedge clk);
        drive(2'b01); drive(2'b11); drive(2'b10); drive(2'b00);
        repeat(8) @(posedge clk);
        $display("[T5 CYC ] one full cycle -> pos=%0d (expect 4)", pos);
        if (pos !== 4) begin $display("   !! FAIL T5"); err = err + 1; end

        // ---- T6: T-method, fast (gap = 7 clk between edges) ----
        stb_clr = 1; @(posedge clk); stb_clr = 0;
        for (i = 0; i < 20; i = i + 1) begin st = fwd_next(st); drive_gap(st, 7); end
        repeat(8) @(posedge clk);
        $display("[T6 TFAST] gap=7 -> period=%0d (expect 7) valid=%b stb=%0d/20",
                 period, period_valid, stb_cnt);
        if (period !== 24'd7)      begin $display("   !! FAIL T6 period"); err = err + 1; end
        if (period_valid !== 1'b1) begin $display("   !! FAIL T6 valid");  err = err + 1; end
        if (stb_cnt !== 20)        begin $display("   !! FAIL T6 stb");    err = err + 1; end

        // ---- T7: T-method, slow (gap = 500 clk between edges) ----
        stb_clr = 1; @(posedge clk); stb_clr = 0;
        for (i = 0; i < 8; i = i + 1) begin st = fwd_next(st); drive_gap(st, 500); end
        repeat(8) @(posedge clk);
        $display("[T7 TSLOW] gap=500 -> period=%0d (expect 500) valid=%b stb=%0d/8",
                 period, period_valid, stb_cnt);
        if (period !== 24'd500)    begin $display("   !! FAIL T7 period"); err = err + 1; end
        if (period_valid !== 1'b1) begin $display("   !! FAIL T7 valid");  err = err + 1; end
        if (stb_cnt !== 8)         begin $display("   !! FAIL T7 stb");    err = err + 1; end

        // ---- T8: stop for > T_TIMEOUT -> period_valid drops, period retained ----
        repeat(T_TO + 50) @(posedge clk);
        $display("[T8 STOP] stop %0d clks -> valid=%b (expect 0) period=%0d (expect 500)",
                 T_TO + 50, period_valid, period);
        if (period_valid !== 1'b0) begin $display("   !! FAIL T8 valid not cleared"); err = err + 1; end
        if (period !== 24'd500)    begin $display("   !! FAIL T8 period lost");       err = err + 1; end

        // ---- T9: resume after stop -> valid again with correct period ----
        stb_clr = 1; @(posedge clk); stb_clr = 0;
        for (i = 0; i < 4; i = i + 1) begin st = fwd_next(st); drive_gap(st, 40); end
        repeat(8) @(posedge clk);
        $display("[T9 RESUME] gap=40 -> period=%0d (expect 40) valid=%b",
                 period, period_valid);
        if (period !== 24'd40)     begin $display("   !! FAIL T9 period"); err = err + 1; end
        if (period_valid !== 1'b1) begin $display("   !! FAIL T9 valid");  err = err + 1; end

        if (err == 0) $display("\n==== ALL TESTS PASS ====");
        else          $display("\n==== %0d TEST(S) FAILED ====", err);
        $finish;
    end

    initial begin
        #4000000;
        $display("!! TIMEOUT");
        $finish;
    end

endmodule