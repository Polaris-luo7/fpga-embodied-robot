//=============================================================================
// pi_controller.v — 定点 PI 速度控制器（★ 2026-10-14 改为 3 级流水）
//------------------------------------------------------------------------------
// 【为什么改】原组合版把 "误差→死区→kp×e→+积分→限幅→抗饱和→再限幅" 全塞在
//   一拍里，再加 mech_ik 的长链，实测从 u_nav_motor/w_tmp 到本模块 out 的
//   组合延迟 **124 ns（周期仅 20 ns）** —— exp41 里 CLKOUT1 域 483 个违例端点。
//   本模块的使能 en 是 **1 kHz 脉冲**（每 50000 拍一次），一次控制运算分 3 拍
//   完成毫无代价。
//
// 【流水级】级1: 误差→死区→e_eff_r
//           级2: 比例项 kp×e_eff_r、积分增量 ki×e_eff_r
//           级3: 积分累加+抗饱和+输出限幅 → out / integ / integ_out
//   总延迟 3 拍 @en（= 3 µs）。端口与语义对外**完全不变**。
//
// 对应工程约束：定点数(#3)、积分限幅抗饱和(#4)、误差死区(#5)、
//               输出最小有效占空比(#6)、增益在线可调(#12)
//=============================================================================
`timescale 1ns/1ps

module pi_controller #(
    parameter integer DEADZONE = 3,      // 误差死区
    parameter integer OUT_MAX  = 4000,   // 输出限幅（占空比上限）
    parameter integer I_LIMIT  = 4000,   // 积分限幅（抗饱和）
    parameter integer MIN_DUTY = 100,    // 最小有效占空比
    parameter integer KP_SHIFT = 3,      // 比例增益右移（增益 = kp >> KP_SHIFT）
    parameter integer KI_SHIFT = 4       // 积分增益右移
)(
    input  wire               clk,
    input  wire               rst_n,
    input  wire               en,        // 控制使能脉冲（1 kHz）
    input  wire               clr_i,     // 积分清零
    input  wire signed [15:0] target,    // 目标速度
    input  wire signed [15:0] actual,    // 实测速度
    input  wire signed [15:0] kp,        // 比例增益（在线可调）
    input  wire signed [15:0] ki,        // 积分增益（在线可调）
    output reg  signed [15:0] out,       // 控制输出
    output reg  signed [31:0] integ_out  // 积分器现值（调试/观测）
);

    // ---- 级 1: 误差 + 死区 ----
    wire signed [16:0] error = target - actual;
    reg  signed [16:0] e_eff_r;
    reg                en1;

    // ---- 级 2: 两个乘法 ----
    reg signed [31:0] p_r, id_r;
    reg               en2;

    // ---- 级 3: 积分与限幅 ----
    reg signed [31:0] integ;

    wire signed [31:0] i_next  = integ + id_r;
    wire signed [31:0] i_clamp = (i_next > I_LIMIT) ? I_LIMIT :
                                 ((i_next < -I_LIMIT) ? -I_LIMIT : i_next);

    // 抗饱和（条件积分）：仅当输出已到限幅、且误差仍把它往同方向推时冻结积分
    wire signed [31:0] out_probe = p_r + integ;
    // ★时序修复 2026-10-17：原用组合误差 error，形成 speed_meas → out 的长路径（实测 −5.8ns）。
    //   改用已寄存的 e_eff_r 判方向（抗饱和只需方向，晚一拍无影响）⇒ 切断该组合路径。
    wire sat_freeze = ((out_probe >= OUT_MAX) && (e_eff_r > 0)) ||
                      ((out_probe <= -OUT_MAX) && (e_eff_r < 0));
    wire signed [31:0] i_appl  = sat_freeze ? integ : i_clamp;
    wire signed [31:0] out_raw = p_r + i_appl;
    wire signed [31:0] out_clamp = (out_raw > OUT_MAX) ? OUT_MAX :
                                   ((out_raw < -OUT_MAX) ? -OUT_MAX : out_raw);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            e_eff_r <= 17'sd0; p_r <= 32'sd0; id_r <= 32'sd0;
            integ <= 32'sd0; out <= 16'sd0; integ_out <= 32'sd0;
            en1 <= 1'b0; en2 <= 1'b0;
        end else if (clr_i) begin
            e_eff_r <= 17'sd0; p_r <= 32'sd0; id_r <= 32'sd0;
            integ <= 32'sd0; out <= 16'sd0; integ_out <= 32'sd0;
            en1 <= 1'b0; en2 <= 1'b0;
        end else begin
            en1 <= en;
            en2 <= en1;

            // 级 1
            if (en) begin
                if (error > DEADZONE || error < -DEADZONE) e_eff_r <= error;
                else                                       e_eff_r <= 17'sd0;
            end
            // 级 2
            if (en1) begin
                p_r  <= (kp * e_eff_r) >>> KP_SHIFT;
                id_r <= (ki * e_eff_r) >>> KI_SHIFT;
            end
            // 级 3
            if (en2) begin
                integ     <= i_appl;
                integ_out <= i_appl;
                if (out_clamp < MIN_DUTY && out_clamp > -MIN_DUTY)
                    out <= 16'sd0;                 // 低于启动阈 -> 不输出
                else
                    out <= out_clamp[15:0];
            end
        end
    end

endmodule
