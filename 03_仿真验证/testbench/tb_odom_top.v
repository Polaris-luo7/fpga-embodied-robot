//=============================================================================
// tb_odom_top.v -- odom_top 集成仿真：真实 quad_dec + speed_meter 链路
//   自由跑正交波形发生器：每 25 clk 一次状态迁移(4 迁移=1 计数)
//   => 每窗(WIN=5000, defparam 缩短)恰好 50 计数/轮。
//   用例：
//     T1 四轮同向 100 窗 => x = 100*50*MM_PC = 1090.8mm
//     T2 左移波序 100 窗 => y += 1090.8, x 不变
//     T3 旋转波序      => θ 累加
//     T4 clr 中途清零   => x/y/θ 归零后重新累积
//     T5 打滑           => slip 置位
//=============================================================================
`timescale 1ns/1ps

module tb_odom_top;

    reg clk=0, rst_n=0, clr=0, imu_valid=0;
    reg [3:0] dir = 4'b0000;               // 1=该轮反转
    reg signed [31:0] imu_yaw = 0;
    wire [3:0] enc_a, enc_b;
    wire signed [31:0] x_q16, y_q16, theta_q16, vx_q16, vy_q16;
    wire signed [15:0] wdeg_q16;
    wire slip;
    wire [7:0] enc_err;

    odom_top #( .WIN_CYCLES(5000) ) dut (      // 窗口缩短加速仿真(真机 500000)
        .clk(clk), .rst_n(rst_n),
        .enc_a(enc_a), .enc_b(enc_b),
        .clr(clr), .imu_valid(imu_valid), .imu_yaw_q16(imu_yaw),
        .x_q16(x_q16), .y_q16(y_q16), .theta_q16(theta_q16),
        .vx_q16(vx_q16), .vy_q16(vy_q16), .wdeg_q16(wdeg_q16),
        .slip(slip), .enc_err(enc_err)
    );

    always #10 clk = ~clk;

    // ---- 正交波形发生器：25 clk/迁移，4 迁移/计数 => 50 计数/窗 ----
    reg [15:0] div_cnt = 0;
    reg [1:0]  phase [0:3];
    integer gi;
    initial begin
        for (gi = 0; gi < 4; gi = gi + 1) phase[gi] = 2'b00;  // ★必须初始化(否则恒X)
    end
    always @(posedge clk) begin
        if (div_cnt == 16'd24) begin
            div_cnt <= 0;
            for (gi = 0; gi < 4; gi = gi + 1)
                phase[gi] <= dir[gi] ? phase[gi] - 1 : phase[gi] + 1;
        end else
            div_cnt <= div_cnt + 1;
    end
    assign enc_a[0] = phase[0][1]; assign enc_b[0] = phase[0][1] ^ phase[0][0];
    assign enc_a[1] = phase[1][1]; assign enc_b[1] = phase[1][1] ^ phase[1][0];
    assign enc_a[2] = phase[2][1]; assign enc_b[2] = phase[2][1] ^ phase[2][0];
    assign enc_a[3] = phase[3][1]; assign enc_b[3] = phase[3][1] ^ phase[3][0];

    // ---- 等待 n 个 tick（tick_d 经 1 拍延迟进 core，末尾多等 2 拍确保积分生效）----
    //    ★循环变量必须独立(曾与调用方共用 n -> 死循环)
    integer k;
    task wait_ticks(input integer cnt);
        begin
            for (k = 0; k < cnt; k = k + 1)
                @(posedge dut.u_s0.tick);
            // ★流水后延迟: tick(1) + tick_d(1) + 3 级流水(3) = 5 拍, 多等 2 拍保险
            repeat (7) @(posedge clk);
        end
    endtask

    task chk32(input [8*20:1] name, input signed [31:0] got, input real want);
        real g, w, err;
        begin
            g = got / 65536.0; w = want;
            err = (w != 0.0) ? (g - w) / w : g - w;
            if (err < 0) err = -err;
            if (err <= 0.02) $display("[PASS] %0s got=%.3f want=%.3f", name, g, w);
            else             $display("[FAIL] %0s got=%.3f want=%.3f err=%.2f%%", name, g, w, err*100);
        end
    endtask

    localparam real MM_PC  = 0.218163;
    localparam real DEG_SR = 0.0151149;
    integer t5i;

    // IMU 增量注入: 推进 imu_yaw 并给 1 拍 valid
    task imu_step(input integer q16);
        begin
            imu_yaw = imu_yaw + q16;
            @(posedge clk); imu_valid = 1;
            @(posedge clk); imu_valid = 0;
        end
    endtask

    initial begin
        $display("=== tb_odom_top start ===");
        repeat (5) @(posedge clk); rst_n = 1;
        repeat (10) @(posedge clk);

        //--------------------------------------------------------------
        // T1 四轮前进 100 窗（每轮 200 计数/窗, quad_dec 4 倍频）
        //--------------------------------------------------------------
        $display("[T1] all-forward 100 windows");
        dir = 4'b0000;
        wait_ticks(100);
        chk32("T1.x",  x_q16, 100.0*200.0*MM_PC);
        chk32("T1.y",  y_q16, 0.0);
        chk32("T1.th", theta_q16, 0.0);

        //--------------------------------------------------------------
        // T2 左移 20 窗（dir=1001: FL/RR 反转 => [-,+,+,-]）
        //--------------------------------------------------------------
        $display("[T2] left-shift 20 windows");
        dir = 4'b1001;
        wait_ticks(20);
        chk32("T2.y",  y_q16, 20.0*200.0*MM_PC);
        chk32("T2.x",  x_q16, 100.0*200.0*MM_PC);

        //--------------------------------------------------------------
        // T3 左转 20 窗（θ 由 IMU 增量主导: 每窗注入 +12.0919 度）
        //   编码器同时给左转波序(供 slip 检测用)
        //--------------------------------------------------------------
        $display("[T3] rotation 20 windows (theta from IMU increments)");
        dir = 4'b0101;
        for (t5i = 0; t5i < 20; t5i = t5i + 1) begin
            wait_ticks(1);
            imu_step(32'sd792456);           // 12.0919 度 Q16.16
        end
        repeat (3) @(posedge clk);           // 等最后一次 NBA 提交
        chk32("T3.th", theta_q16, 20.0*800.0*DEG_SR);
        chk32("T3.x",  x_q16, 100.0*200.0*MM_PC);
        chk32("T3.y",  y_q16, 20.0*200.0*MM_PC);

        //--------------------------------------------------------------
        // T4 clr 清零（丢弃 clr 后第一窗）再旋转 20 窗（IMU 增量驱动 θ）
        //--------------------------------------------------------------
        $display("[T4] clr mid-run then rotate 20 windows");
        @(posedge clk); clr = 1;
        @(posedge clk); clr = 0;
        wait_ticks(1);                       // ★丢弃假 delta 窗
        chk32("T4.x0", x_q16, 0.0);
        chk32("T4.y0", y_q16, 0.0);
        chk32("T4.t0", theta_q16, 0.0);
        for (t5i = 0; t5i < 20; t5i = t5i + 1) begin
            wait_ticks(1);
            imu_step(32'sd792456);
        end
        repeat (3) @(posedge clk);
        chk32("T4.th1", theta_q16, 20.0*800.0*DEG_SR);
        chk32("T4.x1", x_q16, 0.0);
        chk32("T4.y1", y_q16, 0.0);

        //--------------------------------------------------------------
        // T5 打滑：继续旋转，但 IMU 每拍只报 0.1 度 => 编码器 vs IMU 严重失配
        //--------------------------------------------------------------
        $display("[T5] slip injection (imu frozen)");
        imu_yaw = 32'sd0;
        for (t5i = 0; t5i < 60; t5i = t5i + 1) begin
            wait_ticks(1);
            imu_yaw = imu_yaw + 32'sd6554;      // 0.1 度/窗
            @(posedge clk); imu_valid = 1;
            @(posedge clk); imu_valid = 0;
        end
        wait_ticks(1);
        chkbit("T5.slip", slip, 1'b1);

        $display("=== tb_odom_top done ===");
        $finish;
    end

    task chkbit(input [8*20:1] name, input got, input want);
        if (got === want) $display("[PASS] %0s got=%b", name, got);
        else              $display("[FAIL] %0s got=%b want=%b", name, got, want);
    endtask
endmodule
