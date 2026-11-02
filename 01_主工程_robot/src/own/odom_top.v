//=============================================================================
// odom_top.v -- P0 里程计顶层封装：4×quad_dec + 4×speed_meter + odom_core
//-----------------------------------------------------------------------------
// 【设计要点】
//   ① 编码器引脚 (enc_a/enc_b) 与 selftest_ir 内部的 quad_dec **并行读取**
//      —— 输入脚多实例读取合法，selftest_ir 一个字节不动（纪律）。
//   ② tick = speed_meter 的 10ms 窗口节拍（四路共用一个窗口即可，取第一路的
//      tick；speed_meter 内部各自计数同一周期，天然同步）。
//   ③ 轮序映射（与 selftest_ir 一致）：
//        P1=M1 左前 FL (D19/B16)   P2=M2 右前 FR (C14/D17)
//        P3=M3 左后 RL (C15/C17)   P4=M4 右后 RR (B15/E16)
//      ⚠ 各轮"正方向 = 向前滚"。若某轮装反/接反导致符号相反，
//        用 FLIP_* 参数逐轮翻号（上板标定时定，仿真里默认全不翻）。
//   ④ IMU yaw 由 imu 链路（att 输出 Q16.16 度）接入；接法在顶层例化时完成。
//   ⑤ 复位/清零：clr 脉冲把 x/y/θ 清零（摆车到 A 后按一次）。
//=============================================================================
`timescale 1ns/1ps

module odom_top #(
    parameter FLIP_FL = 0,        // 1 = 该轮计数取反（装反/接反标定用）
    parameter FLIP_FR = 0,
    parameter FLIP_RL = 0,
    parameter FLIP_RR = 0,
    parameter integer WIN_CYCLES = 500_000,  // 测速窗口: 50MHz 下 500000 = 10ms
    //   THETA_SRC: 1 = IMU 增量主导航向（需 IMU 已标零偏）；0 = 纯编码器航向
    parameter integer THETA_SRC  = 1,
    //  ★EXT_VEL_EN=1：四轮计数改用外部速度（counts/10ms，如任务三 board 的 obs_velocities，
    //    其轮序/符号经他们实车闭环验证）⇒ 跳过本模块内部 quad_dec/speed_meter 的符号/轮序风险
    parameter integer EXT_VEL_EN = 0,
    parameter integer ROT_SIGN   = 1   // ★-1 = 旋转方向整体取反（推测试后若 θ 反向改这里）
)(
    input  wire       clk,          // 50MHz
    input  wire       rst_n,
    input  wire [3:0] enc_a,        // A 相: D19/C14/C15/B15
    input  wire [3:0] enc_b,        // B 相: B16/D17/C17/E16
    input  wire       clr,          // 1 拍: 位姿清零(摆车到 A 后按)
    input  wire       imu_valid,    // IMU yaw 样本有效
    input  wire signed [31:0] imu_yaw_q16,
    input  wire       motion_cmd,   // ★2026-10-22 电机命令中（theta 门控条件之一）
    output wire signed [31:0] x_q16, y_q16, theta_q16,
    output wire signed [31:0] vx_q16, vy_q16,
    output wire signed [15:0] wdeg_q16,
    output wire       slip,
    output wire [31:0] enc_err,     // 四路非法跳变计数(调试, 每路 8 位)
    input  wire signed [15:0] ext_w_fl, ext_w_fr, ext_w_rl, ext_w_rr
);

    // ------------------------------------------------------------------
    // 4× 正交解码（与 selftest_ir 同款 quad_dec 逻辑，独立实例并行读取）
    // ------------------------------------------------------------------
    wire signed [31:0] pos0, pos1, pos2, pos3;
    wire [7:0] err0, err1, err2, err3;

    quad_dec u_q0(.clk(clk), .rst_n(rst_n), .a(enc_a[0]), .b(enc_b[0]), .clr(clr), .pos(pos0), .err_cnt(err0));
    quad_dec u_q1(.clk(clk), .rst_n(rst_n), .a(enc_a[1]), .b(enc_b[1]), .clr(clr), .pos(pos1), .err_cnt(err1));
    quad_dec u_q2(.clk(clk), .rst_n(rst_n), .a(enc_a[2]), .b(enc_b[2]), .clr(clr), .pos(pos2), .err_cnt(err2));
    quad_dec u_q3(.clk(clk), .rst_n(rst_n), .a(enc_a[3]), .b(enc_b[3]), .clr(clr), .pos(pos3), .err_cnt(err3));

    // ------------------------------------------------------------------
    // 4× 测速：10ms 窗口计数增量 + 节拍
    // ------------------------------------------------------------------
    wire signed [15:0] w0, w1, w2, w3;
    wire tick;
    speed_meter #( .WIN(WIN_CYCLES) ) u_s0(.clk(clk), .rst_n(rst_n), .pos(pos0), .spd(w0), .tick(tick));
    speed_meter #( .WIN(WIN_CYCLES) ) u_s1(.clk(clk), .rst_n(rst_n), .pos(pos1), .spd(w1), .tick());
    speed_meter #( .WIN(WIN_CYCLES) ) u_s2(.clk(clk), .rst_n(rst_n), .pos(pos2), .spd(w2), .tick());
    speed_meter #( .WIN(WIN_CYCLES) ) u_s3(.clk(clk), .rst_n(rst_n), .pos(pos3), .spd(w3), .tick());

    // 轮序映射 + 可选翻号：FL=w0, FR=w1, RL=w2, RR=w3
    wire signed [15:0] w_fl = EXT_VEL_EN ? ext_w_fl : (FLIP_FL ? -w0 : w0);
    wire signed [15:0] w_fr = EXT_VEL_EN ? ext_w_fr : (FLIP_FR ? -w1 : w1);
    wire signed [15:0] w_rl = EXT_VEL_EN ? ext_w_rl : (FLIP_RL ? -w2 : w2);
    wire signed [15:0] w_rr = EXT_VEL_EN ? ext_w_rr : (FLIP_RR ? -w3 : w3);

    // ------------------------------------------------------------------
    // 位姿积分核心
    //   ★ tick 打一拍再给 core：speed_meter 的 spd 与 tick 同拍更新，
    //     直接采样会取到上一窗的旧 spd(滞后一窗)。延迟 1 拍后采样正确。
    // ------------------------------------------------------------------
    reg tick_d;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) tick_d <= 1'b0;
        else        tick_d <= tick;

    odom_core #(.THETA_SRC(THETA_SRC), .SGN_ENC_THETA(ROT_SIGN)) u_core (
        .clk(clk), .rst_n(rst_n), .clr(clr), .tick(tick_d),
        .w_fl(w_fl), .w_fr(w_fr), .w_rl(w_rl), .w_rr(w_rr),
        .imu_valid(imu_valid), .imu_yaw_q16(imu_yaw_q16), .motion_cmd(motion_cmd),
        .x_q16(x_q16), .y_q16(y_q16), .theta_q16(theta_q16),
        .vx_q16(vx_q16), .vy_q16(vy_q16), .wdeg_q16(wdeg_q16),
        .slip(slip)
    );

    assign enc_err = {err3, err2, err1, err0};

endmodule
