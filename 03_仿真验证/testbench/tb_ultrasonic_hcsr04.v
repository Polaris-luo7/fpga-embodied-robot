//=============================================================================
// tb_ultrasonic_hcsr04.v — HC-SR04 controller self-checking testbench
// Models the sensor: on TRIG rising, waits 200us, drives ECHO high for a width
// corresponding to the target distance (cm * 58 us).
//=============================================================================
`timescale 1ns/1ps

module tb_ultrasonic_hcsr04;

    reg  clk = 0, rst_n = 0, start = 0;
    wire trig;
    reg  echo = 0;
    wire [15:0] echo_us, dist_cm;
    wire valid, busy;

    ultrasonic_hcsr04 dut (
        .clk(clk), .rst_n(rst_n), .start(start), .trig(trig), .echo(echo),
        .echo_us(echo_us), .dist_cm(dist_cm), .valid(valid), .busy(busy)
    );

    always #10 clk = ~clk;   // 50 MHz

    integer echo_ns;   // ECHO pulse width in ns
    reg     respond;   // whether sensor answers

    // ---- HC-SR04 model: answer TRIG with an ECHO pulse ----
    always @(posedge trig) begin
        if (respond) begin
            #200000;               // 200 us fixed turnaround
            echo = 1'b1;
            #(echo_ns);
            echo = 1'b0;
        end
    end

    integer err;
    integer t_cm;
    reg [15:0] cap_us, cap_cm;

    // capture result on valid
    always @(posedge clk) if (valid) begin cap_us <= echo_us; cap_cm <= dist_cm; end

    task measure(input integer cm);
        begin
            echo_ns = cm * 58 * 1000;   // cm*58 us -> ns
            respond = 1'b1;
            start = 1'b1; @(posedge clk); start = 1'b0;
            wait (valid);
            repeat(3) @(posedge clk);
        end
    endtask

    initial begin
        err = 0; echo_ns = 0; respond = 0;
        repeat(5) @(posedge clk);
        rst_n = 1;
        repeat(5) @(posedge clk);

        // ---- T1: 30 cm ----
        measure(30);
        $display("[T1] target=30cm -> dist=%0d cm, echo=%0d us", cap_cm, cap_us);
        if (cap_cm !== 16'd30)   begin $display("   !! FAIL T1"); err = err + 1; end
        if (cap_us !== 16'd1740) begin $display("   !! FAIL T1 echo"); err = err + 1; end

        // ---- T2: 100 cm ----
        measure(100);
        $display("[T2] target=100cm -> dist=%0d cm, echo=%0d us", cap_cm, cap_us);
        if (cap_cm !== 16'd100)  begin $display("   !! FAIL T2"); err = err + 1; end

        // ---- T3: 5 cm ----
        measure(5);
        $display("[T3] target=5cm -> dist=%0d cm, echo=%0d us", cap_cm, cap_us);
        if (cap_cm !== 16'd5)    begin $display("   !! FAIL T3"); err = err + 1; end

        // ---- T4: 2 cm (min range) ----
        measure(2);
        $display("[T4] target=2cm -> dist=%0d cm, echo=%0d us", cap_cm, cap_us);
        if (cap_cm !== 16'd2)    begin $display("   !! FAIL T4"); err = err + 1; end

        // ---- T5: no echo -> timeout, must not produce valid, return to idle ----
        respond = 1'b0;
        start = 1'b1; @(posedge clk); start = 1'b0;
        #25000000;   // wait 25 ms > WAIT_US (20 ms)
        $display("[T5] no-echo timeout -> busy=%b (expect 0)", busy);
        if (busy !== 1'b0) begin $display("   !! FAIL T5 (stuck busy)"); err = err + 1; end

        if (err == 0) $display("\n==== ALL TESTS PASS ====");
        else          $display("\n==== %0d TEST(S) FAILED ====", err);
        $finish;
    end

    initial begin
        #200000000;
        $display("!! TIMEOUT");
        $finish;
    end

endmodule