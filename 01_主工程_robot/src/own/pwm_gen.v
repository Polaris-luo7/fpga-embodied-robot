//=============================================================================
// pwm_gen.v — 带换向死区的 PWM 发生器（驱动板接口：PWM + DIR）
//------------------------------------------------------------------------------
// - 计数比较式 PWM，频率 = CLK_FREQ_HZ / PWM_PERIOD
//   默认 PWM_PERIOD=4000 → 50MHz/4000 = 12.5 kHz（工程要求 10~20 kHz，
//   低于 8 kHz 有啸叫）
// - duty 单位 = 计数（0..PWM_PERIOD，与 pi_controller 的 OUT_MAX=4000 对齐，
//   4000 = 100%），无需乘除换算
// - 换向死区（工程要求）：dir_req 与 dir_out 不一致期间强制 PWM 输出低，
//   持续 DEAD_CYCLES 个 clk 后才允许 DIR 翻转 —— 防止驱动桥在高占空比下
//   直接反接
// - run=0 时输出强制低（PI 未使能/系统未运行时不给驱动板任何驱动）
//=============================================================================
`timescale 1ns/1ps

module pwm_gen #(
    parameter integer CLK_FREQ_HZ  = 50_000_000,
    parameter integer PWM_PERIOD   = 4000,  // 计数上限；f = CLK/PERIOD
    parameter integer DEAD_CYCLES  = 100    // 换向死区（clk 数，100@50MHz = 2 µs）
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        run,        // 1 = 允许输出
    input  wire        dir_req,    // 请求方向（来自 PI 输出符号位，语义由顶层定义）
    input  wire [15:0] duty,       // 占空比幅值 0..PWM_PERIOD
    output wire        pwm_out,
    output reg         dir_out     // 实际输出方向（带死区延迟）
);
    wire dir_mismatch = (dir_req != dir_out);

    // ---- 换向死区 ----
    localparam integer CW = (DEAD_CYCLES <= 1) ? 1 : $clog2(DEAD_CYCLES);
    reg [CW-1:0] dead_cnt;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dir_out  <= 1'b0;
            dead_cnt <= 0;
        end else if (dir_mismatch) begin
            if (dead_cnt >= DEAD_CYCLES - 1) begin
                dir_out  <= dir_req;
                dead_cnt <= 0;
            end else begin
                dead_cnt <= dead_cnt + 1'b1;
            end
        end else begin
            dead_cnt <= 0;
        end
    end

    // ---- PWM 计数比较 ----
    localparam integer PW = $clog2(PWM_PERIOD);
    reg [PW-1:0] cnt;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) cnt <= 0;
        else if (cnt == PWM_PERIOD - 1) cnt <= 0;
        else cnt <= cnt + 1'b1;
    end

    wire [15:0] duty_clamped = (duty > PWM_PERIOD) ? PWM_PERIOD[15:0] : duty;
    assign pwm_out = run && !dir_mismatch && (cnt < duty_clamped);
endmodule
