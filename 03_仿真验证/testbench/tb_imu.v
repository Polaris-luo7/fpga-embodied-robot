//=============================================================================
// tb_imu.v -- T13 IMU 全链路 testbench（I2C 主机 + 从机模型 + 姿态解算）
//-----------------------------------------------------------------------------
// 链路：mpu6050_ctrl(初始化/连读) --I2C--> mpu6050_model(从机行为模型)
//                                   |
//                                   +--> imu_attitude(定点互补滤波) --> roll/pitch
//
// 【自检项】
//   P1 初始化序列：从机寄存器堆里 5 个配置寄存器被写成了期望值
//   P2 连读 14 字节：ctrl 输出的 accel/gyro 与从机预置值逐字节一致
//   P3 静止（重力在 z 轴）：roll/pitch 收敛到 0
//   P4 倾斜 30 度（绕 x 轴）：roll 收敛到 30 度，pitch 保持 0
//   P5 注入已知角速度：pitch 变化方向与量纲正确
//
// 【仿真参数与真机不同（为了跑得快）】
//   仿真 : SYS_CLK=1MHz, SCL=125kHz, 采样 500Hz(dt=2ms), tau=64ms
//   真机 : SYS_CLK=50MHz, SCL=400kHz, 采样 100Hz(dt=10ms), tau=0.32s
//   ⇒ RTL 全参数化，两套参数都能跑；tb 只覆盖功能，不替代上板标定。
//   ⚠ 仿真数据**不得**当作实测数据写进报告（纪律第 2 条）。
//
// 【调试开关】+VI2C 打印 START/STOP 事件：
//   vsim -c +VI2C -do run.do work.tb_imu
//=============================================================================
`timescale 1ns / 1ps

module tb_imu;

    // ------------------------------------------------------------------
    // 参数
    // ------------------------------------------------------------------
    localparam integer SYS_CLK_HZ = 1_000_000;   // 仿真时钟 1 MHz
    localparam integer SCL_HZ     = 100_000;     // -> 实际 SCL = 1e6/(4*2) = 125 kHz
    localparam integer PWRUP_MS   = 1;
    localparam integer PERIOD_MS  = 2;           // 500 Hz 采样
    localparam integer DT_DIV     = 500;         // 与 PERIOD_MS 匹配
    localparam integer ALPHA_SHF  = 5;           // tau = 2ms * 32 = 64 ms

    // 静止水平时 ±4g 档的重力读数（8192 LSB/g）
    localparam signed [15:0] G_1G = 16'sd8192;

    // ------------------------------------------------------------------
    // 时钟与复位
    // ------------------------------------------------------------------
    reg clk = 1'b0;
    reg rst_n = 1'b0;
    always #500 clk = ~clk;          // 周期 1 us

    // ------------------------------------------------------------------
    // I2C 总线（开漏 + 上拉）
    // ------------------------------------------------------------------
    wire i2c_scl;
    wire i2c_sda;
    wire i2c_sda_o, i2c_sda_oe;

    assign i2c_sda = (i2c_sda_oe && !i2c_sda_o) ? 1'b0 : 1'bz;
    pullup (i2c_sda);
    pullup (i2c_scl);

    // ------------------------------------------------------------------
    // DUT: MPU6050 控制器
    // ------------------------------------------------------------------
    wire signed [15:0] acc_x, acc_y, acc_z, temp_raw, gyro_x, gyro_y, gyro_z;
    wire               data_valid;

    mpu6050_ctrl #(
        .SYS_CLK_HZ (SYS_CLK_HZ),
        .SCL_HZ     (SCL_HZ),
        .PWRUP_MS   (PWRUP_MS),
        .PERIOD_MS  (PERIOD_MS),
        .DEV_ADDR   (7'h68)
    ) dut_ctrl (
        .clk         (clk),
        .rst_n       (rst_n),
        .i2c_scl     (i2c_scl),
        .i2c_sda_in  (i2c_sda),
        .i2c_sda_o   (i2c_sda_o),
        .i2c_sda_oe  (i2c_sda_oe),
        .accel_x     (acc_x),
        .accel_y     (acc_y),
        .accel_z     (acc_z),
        .temp_raw    (temp_raw),
        .gyro_x      (gyro_x),
        .gyro_y      (gyro_y),
        .gyro_z      (gyro_z),
        .data_valid  (data_valid),
        .ack_err     (),
        .dbg_state   (),
        .dbg_sub     ()
    );

    // ------------------------------------------------------------------
    // 从机行为模型
    // ------------------------------------------------------------------
    mpu6050_model #(
        .SYS_CLK_HZ (SYS_CLK_HZ),
        .DEV_ADDR   (7'h68)
    ) u_model (
        .clk   (clk),
        .rst_n (rst_n),
        .scl   (i2c_scl),
        .sda   (i2c_sda)
    );

    // ------------------------------------------------------------------
    // DUT: 姿态解算
    // ------------------------------------------------------------------
    wire signed [31:0] roll_q16, pitch_q16, yaw_q16;
    wire signed [15:0] roll_x100, pitch_x100, yaw_x100;

    imu_attitude #(
        .ALPHA_SHIFT (ALPHA_SHF),
        .GYRO_Q      (3996),
        .DT_DIV      (DT_DIV)
    ) dut_att (
        .clk          (clk),
        .rst_n        (rst_n),
        .sample_valid (data_valid),
        .acc_x        (acc_x),
        .acc_y        (acc_y),
        .acc_z        (acc_z),
        .gyro_x       (gyro_x),
        .gyro_y       (gyro_y),
        .gyro_z       (gyro_z),
        .roll_q16     (roll_q16),
        .pitch_q16    (pitch_q16),
        .yaw_q16      (yaw_q16),
        .roll_x100    (roll_x100),
        .pitch_x100   (pitch_x100),
        .yaw_x100     (yaw_x100),
        .busy         ()
    );

    // ------------------------------------------------------------------
    // 工具
    // ------------------------------------------------------------------
    integer err_cnt = 0;

    task check_eq8(input [8*24:1] name, input [7:0] got, input [7:0] exp);
        begin
            if (got !== exp) begin
                $display("  !! FAIL %0s: got=0x%02X exp=0x%02X", name, got, exp);
                err_cnt = err_cnt + 1;
            end else begin
                $display("  OK   %0s = 0x%02X", name, got);
            end
        end
    endtask

    task check_i16(input [8*24:1] name, input integer got, input integer exp, input integer tol);
        begin
            if (got > exp + tol || got < exp - tol) begin
                $display("  !! FAIL %0s: got=%0d exp=%0d(+/-%0d)", name, got, exp, tol);
                err_cnt = err_cnt + 1;
            end else begin
                $display("  OK   %0s = %0d (exp %0d +/-%0d)", name, got, exp, tol);
            end
        end
    endtask

    task check_range(input [8*24:1] name, input integer got, input integer lo, input integer hi);
        begin
            if (got < lo || got > hi) begin
                $display("  !! FAIL %0s: got=%0d 不在 [%0d,%0d]", name, got, lo, hi);
                err_cnt = err_cnt + 1;
            end else begin
                $display("  OK   %0s = %0d 在 [%0d,%0d]", name, got, lo, hi);
            end
        end
    endtask

    // 等待 n 个数据帧
    task wait_frames(input integer n);
        integer c;
        begin
            c = 0;
            while (c < n) begin
                @(posedge clk);
                if (data_valid === 1'b1) c = c + 1;
            end
        end
    endtask

    // 往从机寄存器堆注入一帧传感器数据
    task set_imu(input signed [15:0] ax, ay, az, gx, gy, gz);
        begin
            u_model.regs[8'h3B] = ax[15:8]; u_model.regs[8'h3C] = ax[7:0];
            u_model.regs[8'h3D] = ay[15:8]; u_model.regs[8'h3E] = ay[7:0];
            u_model.regs[8'h3F] = az[15:8]; u_model.regs[8'h40] = az[7:0];
            u_model.regs[8'h41] = 8'h00;    u_model.regs[8'h42] = 8'h00;   // TEMP
            u_model.regs[8'h43] = gx[15:8]; u_model.regs[8'h44] = gx[7:0];
            u_model.regs[8'h45] = gy[15:8]; u_model.regs[8'h46] = gy[7:0];
            u_model.regs[8'h47] = gz[15:8]; u_model.regs[8'h48] = gz[7:0];
        end
    endtask

    // ------------------------------------------------------------------
    // 可选 I2C 事件打印（+VI2C）
    // ------------------------------------------------------------------
    reg  vI2C;
    reg  scl_p, sda_p;
    initial vI2C = $test$plusargs("VI2C");

    always @(posedge clk) begin
        scl_p <= i2c_scl;
        sda_p <= i2c_sda;
        if (vI2C) begin
            if (i2c_scl && sda_p && !i2c_sda) $display("    <START> t=%0t", $time);
            if (i2c_scl && !sda_p && i2c_sda) $display("    <STOP > t=%0t", $time);
        end
    end

    // 控制器 GO 脉冲跟踪（+VMODEL）
    reg go_p;
    always @(posedge clk) begin
        go_p <= dut_ctrl.go;
        if (vI2C && dut_ctrl.go && !go_p)
            $display("    [C] GO   st=%0d idx=%0d ph=%0d rd_step=%0d cmd=%b tx=0x%02X",
                     dut_ctrl.st, dut_ctrl.ini_idx, dut_ctrl.ini_ph,
                     dut_ctrl.rd_step, dut_ctrl.cmd_lat, dut_ctrl.tx_lat);
    end

    // ------------------------------------------------------------------
    // 超时保护
    //   全部自检约需 843 个采样帧 x 3.224 ms ≈ 2.72 s 仿真时间，
    //   留一倍余量设为 6 s（仿真时钟 1 MHz 下 Modelsim 实际耗时几秒）。
    // ------------------------------------------------------------------
    initial begin
        repeat (6_000_000) @(posedge clk);   // 6e6 clk @ 1MHz = 6 s
        $display("!! TIMEOUT: 仿真超过 6 s 未结束");
        $display("==== tb_imu: 1 ERROR(S) (timeout) ====");
        $finish;
    end

    // ------------------------------------------------------------------
    // 主激励
    // ------------------------------------------------------------------
    initial begin
        $display("==== tb_imu 开始（SYS_CLK=%0d Hz, SCL=%0d Hz, 采样 %0d Hz）====",
                 SYS_CLK_HZ, SCL_HZ, DT_DIV);

        repeat (10) @(posedge clk);
        rst_n = 1'b1;

        // ---- 从机预置：静止水平（重力全在 z 轴）----
        set_imu(16'sd0, 16'sd0, G_1G, 16'sd0, 16'sd0, 16'sd0);

        // ================= P1: 初始化序列 =================
        $display("[P1] 初始化寄存器写入（等第一帧出来即证明初始化走完）");
        wait_frames(1);
        check_eq8("PWR_MGMT_1  (0x6B)", u_model.regs[8'h6B], 8'h01);
        check_eq8("SMPLRT_DIV  (0x19)", u_model.regs[8'h19], 8'h09);
        check_eq8("CONFIG      (0x1A)", u_model.regs[8'h1A], 8'h03);
        check_eq8("GYRO_CONFIG (0x1B)", u_model.regs[8'h1B], 8'h18);
        check_eq8("ACCEL_CONFIG(0x1C)", u_model.regs[8'h1C], 8'h08);

        // ================= P2: 连读 14 字节 =================
        $display("[P2] 连读 14 字节（accel/temp/gyro）");
        check_i16("accel_x", acc_x, 0,    2);
        check_i16("accel_y", acc_y, 0,    2);
        check_i16("accel_z", acc_z, G_1G, 2);
        check_i16("gyro_x",  gyro_x, 0,   2);
        check_i16("gyro_y",  gyro_y, 0,   2);
        check_i16("gyro_z",  gyro_z, 0,   2);

        // ---- 换一组非零数据，验证不是"碰巧" ----
        set_imu(16'sd100, -16'sd200, 16'sd300, 16'sd11, -16'sd22, 16'sd33);
        wait_frames(2);
        check_i16("accel_x(2)", acc_x, 100,  2);
        check_i16("accel_y(2)", acc_y, -200, 2);
        check_i16("accel_z(2)", acc_z, 300,  2);
        check_i16("gyro_x (2)", gyro_x, 11,  2);
        check_i16("gyro_y (2)", gyro_y, -22, 2);
        check_i16("gyro_z (2)", gyro_z, 33,  2);

        // ================= P3: 静止姿态 =================
        $display("[P3] 静止水平：roll/pitch 应收敛到 0（等 200 帧 = 400 ms ≈ 6 tau）");
        set_imu(16'sd0, 16'sd0, G_1G, 16'sd0, 16'sd0, 16'sd0);
        wait_frames(200);
        check_i16("P3 roll ", roll_x100,  0, 50);
        check_i16("P3 pitch", pitch_x100, 0, 50);

        // ================= P4: 倾斜 30 度 =================
        // 绕 x 轴转 30 度：ay = g*sin30 = 4096, az = g*cos30 = 7094
        $display("[P4] 绕 x 轴倾斜 30 度：roll 应收敛到 +30 度");
        set_imu(16'sd0, 16'sd4096, 16'sd7094, 16'sd0, 16'sd0, 16'sd0);
        wait_frames(200);
        check_i16("P4 roll ", roll_x100,  3000, 150);
        check_i16("P4 pitch", pitch_x100,    0, 150);

        // ================= P4b: 抬头 20 度 =================
        // 绕 y 轴转 20 度（抬头为正）：ax = -g*sin20 = -2801, az = g*cos20 = 7698
        $display("[P4b] 绕 y 轴抬头 20 度：pitch 应收敛到 +20 度");
        set_imu(-16'sd2801, 16'sd0, 16'sd7698, 16'sd0, 16'sd0, 16'sd0);
        wait_frames(200);
        check_i16("P4b roll ", roll_x100,    0, 150);
        check_i16("P4b pitch", pitch_x100, 2000, 150);

        // ================= P5: 陀螺积分方向与量纲 =================
        $display("[P5] 回水平，注入 gyro_y = +3280 (约 +200 deg/s)，观察 pitch 变化");
        set_imu(16'sd0, 16'sd0, G_1G, 16'sd0, 16'sd0, 16'sd0);
        wait_frames(200);                      // 先回到 0
        check_i16("P5 base pitch", pitch_x100, 0, 50);

        // 注入：+200 deg/s，20 帧 = 40 ms。
        // 理论：纯积分 200*0.04 = 8.0 度；加速度修正（tau=64ms）会拉回一部分
        // -> 递推 p_{n+1}=(p_n+0.4)*(31/32)，20 步后约 +5.8 度 = 580
        set_imu(16'sd0, 16'sd0, G_1G, 16'sd0, 16'sd3280, 16'sd0);
        wait_frames(20);
        $display("     实测 pitch = %0d (0.01度)  [方向应为正]", pitch_x100);
        check_range("P5 pitch POS-dir", pitch_x100, 350, 800);

        // 反向：应得到反向的 pitch
        $display("[P5b] 注入 gyro_y = -3280，pitch 应向负方向变化");
        set_imu(16'sd0, 16'sd0, G_1G, 16'sd0, -16'sd3280, 16'sd0);
        wait_frames(20);
        $display("     实测 pitch = %0d (0.01度)  [方向应为负]", pitch_x100);
        check_range("P5b pitch NEG-dir", pitch_x100, -1000, -100);

        // ================= 结果 =================
        if (err_cnt == 0)
            $display("==== tb_imu: ALL PASS ====");
        else
            $display("==== tb_imu: %0d ERROR(S) ====", err_cnt);

        $finish;
    end

endmodule
