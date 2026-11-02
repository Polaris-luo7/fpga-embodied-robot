//=============================================================================
// tb_pi_controller.v — closed-loop self-checking testbench
// Includes a first-order motor model (actual = 2*out at steady state).
// Sustained at accelerated rate (en every 100 clk) for fast simulation.
// Checks: convergence, reverse, disturbance rejection, output saturation,
//         dead-zone, integral anti-windup.
//=============================================================================
`timescale 1ns/1ps

module tb_pi_controller;

    reg clk = 0, rst_n = 0, en = 0, clr_i = 0;
    reg signed [15:0] target = 0, actual = 0, kp = 1, ki = 1;
    wire signed [15:0] out;
    wire signed [31:0] integ_out;

    pi_controller dut (
        .clk(clk), .rst_n(rst_n), .en(en), .clr_i(clr_i),
        .target(target), .actual(actual), .kp(kp), .ki(ki),
        .out(out), .integ_out(integ_out)
    );

    always #10 clk = ~clk;   // 50 MHz

    // enable pulse generator (accelerated: every 100 clk)
    integer cnt_en = 0;
    reg freeze = 0;   // freeze motor model for steady-state checks
    always @(posedge clk) begin
        if (cnt_en == 99) begin cnt_en <= 0; en <= 1'b1; end
        else begin cnt_en <= cnt_en + 1; en <= 1'b0; end
    end

    // first-order motor model: actual -> 2*out at steady state
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) actual <= 16'sd0;
        else if (en && !freeze) actual <= actual + (out >>> 3) - (actual >>> 4);
    end

    integer k, err;
    task run_cyc(input integer n);
        begin for (k = 0; k < n; k = k + 1) @(posedge en); end
    endtask

    initial begin
        err = 0;
        repeat(5) @(posedge clk);
        rst_n = 1;
        @(posedge en);

        // ---- P1: forward step, convergence ----
        target = 16'sd1000;
        run_cyc(300);
        $display("[P1 CONV ] target=1000 -> actual=%0d out=%0d integ=%0d", actual, out, integ_out);
        if (actual >= 950 && actual <= 1050) $display("   P1 OK (converged)");
        else begin $display("   !! P1 not converged"); err = err + 1; end

        // ---- P2: reverse ----
        target = -16'sd800;
        run_cyc(400);
        $display("[P2 REV  ] target=-800 -> actual=%0d out=%0d", actual, out);
        if (actual <= -750 && actual >= -850) $display("   P2 OK");
        else begin $display("   !! P2 failed"); err = err + 1; end

        // ---- P3: disturbance rejection (halve speed) ----
        target = 16'sd1000;
        run_cyc(300);
        actual = actual / 2;          // 施加扰动
        run_cyc(300);
        $display("[P3 DIST ] after disturbance -> actual=%0d (expect ~1000)", actual);
        if (actual >= 950 && actual <= 1050) $display("   P3 OK (recovered)");
        else begin $display("   !! P3 failed"); err = err + 1; end

        // ---- P4: output saturation ----
        target = 16'sd32000;
        run_cyc(100);
        $display("[P4 SAT  ] huge target -> out=%0d (expect |out|<=4000)", out);
        if (out <= 4000 && out >= -4000) $display("   P4 OK (limited)");
        else begin $display("   !! P4 failed"); err = err + 1; end

        // ---- P5: dead-zone (tiny error, frozen speed -> no output) ----
        freeze = 1'b1;
        target = actual + 2;
        clr_i = 1; @(posedge clk); clr_i = 0;
        run_cyc(5);
        $display("[P5 DEAD ] within deadzone -> out=%0d (expect 0)", out);
        if (out === 16'sd0) $display("   P5 OK");
        else begin $display("   !! P5 failed"); err = err + 1; end
        freeze = 1'b0;

        // ---- P6: integral anti-windup (after saturation, clear I recovers fast) ----
        target = 16'sd1000;
        clr_i = 1; @(posedge clk); clr_i = 0;
        run_cyc(400);
        $display("[P6 WIND ] clear-I then target=1000 -> actual=%0d integ=%0d", actual, integ_out);
        if (integ_out <= 4000 && integ_out >= -4000) $display("   P6 OK (I limited)");
        else begin $display("   !! P6 failed (I not limited)"); err = err + 1; end
        if (actual >= 950 && actual <= 1050) $display("   P6 OK (converged)");
        else begin $display("   !! P6 not converged actual=%0d", actual); err = err + 1; end

        if (err == 0) $display("\n==== ALL TESTS PASS ====");
        else          $display("\n==== %0d TEST(S) FAILED ====", err);
        $finish;
    end

    initial begin
        #20000000;
        $display("!! TIMEOUT");
        $finish;
    end

endmodule