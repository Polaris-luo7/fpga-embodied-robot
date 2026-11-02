//=============================================================================
// dist_filter.v —— 超声波距离中值滤波 + 周期触发（P4 避障的前端）
//-----------------------------------------------------------------------------
// 【职责】两个，都与引脚无关（引脚定案前即可完成并仿真）：
//   1) 周期触发：每 TRIG_DIV 拍给 ultrasonic_hcsr04 一个 start 单拍
//   2) 中值滤波：连续 3 次采样取中位数 ⇒ 单发野值（多径回波/电机噪声）
//      被直接剔除，不会让避障误触发
//
// 【为什么 3 采中值】HC-SR04 对纸箱这类软表面偶发野值（提前回波），
//   均值滤波会被野值拉偏，中值天然免疫单点野值 —— 3 个数排个序取中间即可，
//   零乘零除，纯比较器（时序友好）。
//
// 【用法】与 ultrasonic_hcsr04 串联：
//     dist_filter ──trig_start──▶ ultrasonic.start
//     ultrasonic.dist_cm/valid ──▶ dist_filter.dist_cm/valid
//     dist_filter.filt_cm ——▶ nav_ctrl.i_dist_cm
//
// 【参数】TRIG_DIV：触发周期。默认 5_000_000 拍 @50MHz = 100ms（10Hz）。
//   避障判定需要连续 OBS_N=3 次 < 阈值（0.3s 确认），10Hz 采样足够。
//
// 【版本】2026-10-14 WorkBuddy（P4 预制件）
//=============================================================================
`timescale 1ns/1ps

module dist_filter #(
    parameter integer TRIG_DIV = 5_000_000,   // 触发周期（拍）@50MHz = 10Hz
    parameter integer MIN_CM   = 2,           // 量程下限（cm）：小于此判野值（含 0）
    parameter integer MAX_CM   = 450,         // 量程上限（cm）：HC-SR04 标称 400，留余量
    parameter integer TMO_TRIG = 10           // 连续多少次无有效样本判"传感器掉线" ⇒ 清窗
)(
    input  wire        clk,
    input  wire        rst_n,

    output reg         trig_start,            // → ultrasonic.start（单拍）
    input  wire [15:0] dist_cm,               // ← ultrasonic 原始距离
    input  wire        valid,                 // ← ultrasonic valid（单拍）

    output reg  [15:0] filt_cm,               // 滤波后距离（cm）
    output reg         filt_vld,              // 滤波值更新（单拍）
    output reg  [15:0] conv_cnt               // 已完成滤波次数（存活判据）
);
    localparam integer W = 3;                  // 窗口深度

    reg [31:0]  trig_cnt;
    reg [3:0]   tmo_cnt;                       // 连续无有效样本的触发次数
    wire        in_range = (dist_cm >= MIN_CM) && (dist_cm <= MAX_CM);
    reg [15:0]  win [0:W-1];                   // 滑动窗口（最新在 [0]）
    reg [1:0]   win_n;                         // 已入窗样本数（0..3）

    // ---- 周期触发 ----
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            trig_cnt   <= 32'd0;
            trig_start <= 1'b0;
        end
        else begin
            trig_start <= 1'b0;
            if (trig_cnt >= (TRIG_DIV - 1)) begin
                trig_cnt   <= 32'd0;
                trig_start <= 1'b1;
            end
            else begin
                trig_cnt <= trig_cnt + 32'd1;
            end
        end
    end

    // ---- 滑动窗口：valid 时新样本进 [0]，旧的右移 ----
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            win[0] <= 16'd0; win[1] <= 16'd0; win[2] <= 16'd0;
            win_n  <= 2'd0;
        end
        else if (valid && in_range) begin      // ★ 越界样本直接丢弃（不污染中值）
            win[2] <= win[1];
            win[1] <= win[0];
            win[0] <= dist_cm;
            if (win_n < 2'd3) win_n <= win_n + 2'd1;
        end
        else if (tmo_cnt >= TMO_TRIG) begin    // ★ 传感器掉线 ⇒ 清窗，避免用陈旧值做避障判断
            win_n <= 2'd0;
        end
    end

    // ---- 掉线计时：有有效样本清零，否则每触发 +1（饱和）----
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) tmo_cnt <= 4'd0;
        else if (valid && in_range) tmo_cnt <= 4'd0;
        else if (trig_start) begin
            if (tmo_cnt < 4'd15) tmo_cnt <= tmo_cnt + 4'd1;
        end
    end

    // ---- 三数中值（纯比较器，无乘除）----
    wire [15:0] min01  = (win[0] < win[1]) ? win[0] : win[1];
    wire [15:0] max01  = (win[0] > win[1]) ? win[0] : win[1];
    wire [15:0] median = (max01 < win[2])  ? max01 : ((min01 > win[2]) ? min01 : win[2]);

    // ---- valid 延迟一拍：保证采样 median 时窗口已完成移位 ----
    //  （若在 valid 当拍取 median，用的是移位前的旧窗口 ⇒ 结果错位一个样本）
    reg  valid_d;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) valid_d <= 1'b0;
        else        valid_d <= valid;

    // ---- 窗口满 ⇒ 输出中值 ----
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            filt_cm  <= 16'd0;
            filt_vld <= 1'b0;
            conv_cnt <= 16'd0;
        end
        else begin
            filt_vld <= 1'b0;
            if (valid_d && win_n == 2'd3) begin
                filt_cm  <= median;
                filt_vld <= 1'b1;
                conv_cnt <= conv_cnt + 16'd1;
            end
        end
    end

endmodule
