//=============================================================================
// tb_nav_motor.v -- nav_motor 用例
//   重点验证【符号约定】——这是唯一"上板才发现就晚了"的东西：
//     前进  => 四轮目标同号(正)
//     左移  => FL/RR 负, FR/RL 正
//     左转  => FL/RL 负, FR/RR 正
//   以及方向脚映射（与任务二一致：前进 in1=1, in2=0）与 run=0 的安全默认。
//   tb 用 NEG_TARGET=0/INVERT_DIR=0（直接对应关系），真机默认 NEG_TARGET=1。
//=============================================================================
`timescale 1ns/1ps

module tb_nav_motor;
    reg clk=0, rst_n=0, run=0;
    reg signed [31:0] vx=0, vy=0, w=0;
    reg [3:0] enc_a=4'b0101, enc_b=4'b0011;   // 静态（不产生计数）
    wire [3:0] n_in1, n_in2, n_pwm;
    wire       n_stdby;
    wire signed [15:0] v_fl,v_fr,v_rl,v_rr, spd0,spd1,spd2,spd3;

    nav_motor #(
        .WHEEL_DIA_UM (100_000),
        .LL_WW_MM     (413    ),
        .NEG_TARGET   (0      ),
        .INVERT_DIR   (0      )
    ) dut (
        .clk(clk), .rst_n(rst_n),
        .vx_q16(vx), .vy_q16(vy), .w_q16(w),
        .run(run), .enc_a(enc_a), .enc_b(enc_b),
        .n_in1(n_in1), .n_in2(n_in2), .n_pwm(n_pwm), .n_stdby(n_stdby),
        .v_fl(v_fl), .v_fr(v_fr), .v_rl(v_rl), .v_rr(v_rr),
        .spd0(spd0), .spd1(spd1), .spd2(spd2), .spd3(spd3)
    );

    always #10 clk = ~clk;

    integer fails=0, ncase=0, i;
    task chk16(input [8*20-1:0] name, input signed [15:0] got, input real want, input real tol);
        real g, w2, err;
        begin
            ncase = ncase + 1;
            g = got; w2 = want;
            err = (w2 != 0.0) ? (g - w2) / w2 : (g - w2);
            if (err < 0) err = -err;
            if (err <= tol) $display("[PASS] %0s got=%0d want=%.1f", name, got, w2);
            else begin fails = fails + 1;
                $display("[FAIL] %0s got=%0d want=%.1f err=%.2f%%", name, got, w2, err*100); end
        end
    endtask

    task chkint(input [8*20-1:0] name, input integer got, input integer want);
        begin
            ncase = ncase + 1;
            if (got == want) $display("[PASS] %0s = %04b", name, got);
            else begin fails = fails + 1;
                $display("[FAIL] %0s got=%04b want=%04b", name, got, want); end
        end
    endtask

    task settle;   // 等速度环 PI(1kHz) 稳定（约 5 个 tick）
        begin
            repeat (6) @(posedge dut.u_sl0.u_tick.tick);
            repeat (300) @(posedge clk);
        end
    endtask

    initial begin
        $display("=== tb_nav_motor start ===");
        repeat (5) @(posedge clk); rst_n = 1; repeat (5) @(posedge clk);

        //------------------------------------------------ T1 run=0 安全默认
        $display("[T1] run=0 => all outputs zero");
        run = 0; vx = 32'sd300<<16; vy = 0; w = 0;
        settle;
        chkint("T1.in1",   n_in1,   4'b0000);
        chkint("T1.in2",   n_in2,   4'b0000);
        chkint("T1.stdby", n_stdby, 0);

        //------------------------------------------------ T2 前进
        $display("[T2] forward 300mm/s");
        run = 1; vx = 32'sd300<<16; vy = 0; w = 0;
        settle;
        // 300 mm/s => 300*1440/(pi*100) = 1375 计数/秒
        chk16("T2.fl", v_fl, 1375.2, 0.05);
        chk16("T2.fr", v_fr, 1375.2, 0.05);
        chk16("T2.rl", v_rl, 1375.2, 0.05);
        chk16("T2.rr", v_rr, 1375.2, 0.05);
        chkint("T2.in1", n_in1, 4'b1111);        // 前进：四路 IN1 高
        chkint("T2.in2", n_in2, 4'b0000);
        chkint("T2.stdby", n_stdby, 1);

        //------------------------------------------------ T3 左移
        $display("[T3] left shift 300mm/s");
        vx = 0; vy = 32'sd300<<16; w = 0;
        settle;
        chk16("T3.fl", v_fl, -1375.2, 0.05);     // FL 反转
        chk16("T3.fr", v_fr,  1375.2, 0.05);
        chk16("T3.rl", v_rl,  1375.2, 0.05);
        chk16("T3.rr", v_rr, -1375.2, 0.05);
        chkint("T3.in1", n_in1, 4'b0110);        // {RR,RL,FR,FL} = 0,1,1,0
        chkint("T3.in2", n_in2, 4'b1001);

        //------------------------------------------------ T4 原地左转
        $display("[T4] spin CCW 90deg/s");
        vx = 0; vy = 0; w = 32'sd90<<16;
        settle;
        chkint("T4.in1", n_in1, 4'b1010);        // {RR,RL,FR,FL} = 1,0,1,0
        chkint("T4.in2", n_in2, 4'b0101);

        //------------------------------------------------ T5 停车（run 掉）
        $display("[T5] run=0 again => zeros");
        run = 0; vx = 0; vy = 0; w = 0;
        settle;
        chkint("T5.in1", n_in1, 4'b0000);
        chkint("T5.stdby", n_stdby, 0);

        $display("=== tb_nav_motor done: cases=%0d fails=%0d ===", ncase, fails);
        $finish;
    end
endmodule
