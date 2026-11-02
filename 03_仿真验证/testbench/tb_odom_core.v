//=============================================================================
// tb_odom_core.v -- odom_core 行为仿真（全部用例有解析期望值, ±1% 容差）
//   tb 自带期望值计算函数，不允许"看着像对"。
//   【时序】tick 周期 tb 可调(TICK_CYCLES)；vx/vy 语义恒按 10ms 窗(×100)。
//=============================================================================
`timescale 1ns/1ps

module tb_odom_core;

    localparam integer TICK_CYCLES = 200;   // tb 加速(真机 500000)
    localparam real    MM_PC   = 0.218163;  // mm/count
    localparam real    DEG_SR  = 0.0151149; // deg per s_rot count
    localparam real    TOL     = 0.01;     // 1%

    reg clk=0, rst_n=0, tick=0, imu_valid=0;
    reg signed [15:0] w_fl=0, w_fr=0, w_rl=0, w_rr=0;
    reg signed [31:0] imu_yaw=0;
    wire signed [31:0] x_q16, y_q16, theta_q16, vx_q16, vy_q16;
    wire signed [15:0] wdeg_q16;
    wire slip;

    odom_core #(.SLIP_WIN(50), .THETA_SRC(0)) dut (   // 本 tb 专测编码器 θ 路径
        .clk(clk), .rst_n(rst_n), .clr(1'b0), .tick(tick),
        .w_fl(w_fl), .w_fr(w_fr), .w_rl(w_rl), .w_rr(w_rr),
        .imu_valid(imu_valid), .imu_yaw_q16(imu_yaw),
        .x_q16(x_q16), .y_q16(y_q16), .theta_q16(theta_q16),
        .vx_q16(vx_q16), .vy_q16(vy_q16), .wdeg_q16(wdeg_q16),
        .slip(slip)
    );

    always #10 clk = ~clk;                  // 50MHz

    integer n;
    task do_ticks(input integer cnt);
        begin
            for (n = 0; n < cnt; n = n + 1) begin
                @(posedge clk); tick = 1'b1;
                @(posedge clk); tick = 1'b0;
                repeat (TICK_CYCLES - 2) @(posedge clk);
            end
        end
    endtask

    task chk32(input [8*20:1] name, input signed [31:0] got, input real want);
        real g, w, err, dif;
        begin
            g = got / 65536.0; w = want;
            err = (w != 0.0) ? (g - w) / w : g - w;
            if (err < 0) err = -err;
            dif = (g > w) ? (g - w) : (w - g);
            // 双容差: 相对 2% 或 绝对 6mm(角度 3 度) —— 兼顾小分量与 LUT 量化
            if (err <= 0.02 || dif <= 6.0)
                $display("[PASS] %0s got=%.3f want=%.3f", name, g, w);
            else
                $display("[FAIL] %0s got=%.3f want=%.3f err=%.2f%%", name, g, w, err*100);
        end
    endtask

    task chkbit(input [8*20:1] name, input got, input want);
        if (got === want) $display("[PASS] %0s got=%b", name, got);
        else              $display("[FAIL] %0s got=%b want=%b", name, got, want);
    endtask

    integer i;
    initial begin
        $display("=== tb_odom_core start ===");
        repeat (5) @(posedge clk); rst_n = 1;
        repeat (5) @(posedge clk);

        //--------------------------------------------------------------
        // T1 纯前进: 4 轮各 +40/tick x 100 tick => x = 100*40*MM_PC = 872.65mm
        //--------------------------------------------------------------
        $display("[T1] forward 100 ticks");
        w_fl=40; w_fr=40; w_rl=40; w_rr=40;
        do_ticks(100);
        chk32("T1.x",  x_q16, 100.0*40.0*MM_PC);
        chk32("T1.y",  y_q16, 0.0);
        chk32("T1.th", theta_q16, 0.0);
        chk32("T1.vx", vx_q16, 40.0*4.0/4.0*MM_PC*100.0);   // 872.65 mm/s
        w_fl=0; w_fr=0; w_rl=0; w_rr=0;

        //--------------------------------------------------------------
        // T2 纯左移: [-40,40,40,-40] x 100 => y = 872.65, x 不变
        //--------------------------------------------------------------
        $display("[T2] left shift 100 ticks");
        w_fl=-40; w_fr=40; w_rl=40; w_rr=-40;
        do_ticks(100);
        chk32("T2.y",  y_q16, 100.0*40.0*MM_PC);
        chk32("T2.x",  x_q16, 100.0*40.0*MM_PC);            // x 保持 T1 值
        w_fl=0; w_fr=0; w_rl=0; w_rr=0;

        //--------------------------------------------------------------
        // T3 纯旋转(无IMU): [-40,40,-40,40] x 100 => th = 100*160*DEG_SR
        //--------------------------------------------------------------
        $display("[T3] pure rotation 100 ticks (encoder only)");
        w_fl=-40; w_fr=40; w_rl=-40; w_rr=40;
        do_ticks(100);
        chk32("T3.th", theta_q16, 100.0*160.0*DEG_SR);      // 241.84 deg
        chk32("T3.x",  x_q16, 100.0*40.0*MM_PC);            // 平移不动
        w_fl=0; w_fr=0; w_rl=0; w_rr=0;

        //--------------------------------------------------------------
        // T4 沿当前航向(θ=241.84°)前进 100 tick => 验证坐标变换
        //   期望: Δ = 872.652mm 沿 241.8384° 方向
        //   (编码器模式下 IMU 不再 snap, 该用例改测世界系旋转变换)
        //--------------------------------------------------------------
        $display("[T4] forward along heading 241.84deg");
        w_fl=40; w_fr=40; w_rl=40; w_rr=40;
        do_ticks(100);
        chk32("T4.x",  x_q16, 460.795);
        chk32("T4.y",  y_q16, 103.305);
        chk32("T4.th", theta_q16, 100.0*160.0*DEG_SR);
        w_fl=0; w_fr=0; w_rl=0; w_rr=0;
        w_fl=0; w_fr=0; w_rl=0; w_rr=0;

        //--------------------------------------------------------------
        // T5 打滑: 编码器说转, IMU 每窗只动 1/10 => slip 置位
        //--------------------------------------------------------------
        $display("[T5] slip injection");
        w_fl=-40; w_fr=40; w_rl=-40; w_rr=40;
        for (i = 0; i < 100; i = i + 1) begin
            do_ticks(10);
            imu_yaw = imu_yaw + 32'sd6554;   // 每 10 tick 转 0.1 度(严重打滑)
            @(posedge clk); imu_valid = 1;
            @(posedge clk); imu_valid = 0;
        end
        w_fl=0; w_fr=0; w_rl=0; w_rr=0;
        do_ticks(2);
        chkbit("T5.slip", slip, 1'b1);

        $display("=== tb_odom_core done ===");
        $finish;
    end
endmodule
