//=============================================================================
// tick_gen.v — 硬件时基（1 kHz 控制节拍）
//------------------------------------------------------------------------------
// 从 clk 分频产生单周期使能脉冲。速度环 1 kHz 节拍由硬逻辑产生，
// 抖动 = 0（分频计数固定），不依赖软核调度 —— 对应工程约束 #1。
//=============================================================================
`timescale 1ns/1ps

module tick_gen #(
    parameter integer CLK_FREQ_HZ = 50_000_000,
    parameter integer TICK_FREQ_HZ = 1000
)(
    input  wire clk,
    input  wire rst_n,
    output reg  tick        // 单周期高脉冲，频率 = TICK_FREQ_HZ
);
    localparam integer DIV = CLK_FREQ_HZ / TICK_FREQ_HZ;  // 50_000_000/1000 = 50000

    reg [$clog2(DIV)-1:0] cnt;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt  <= 0;
            tick <= 1'b0;
        end else if (cnt == DIV - 1) begin
            cnt  <= 0;
            tick <= 1'b1;
        end else begin
            cnt  <= cnt + 1'b1;
            tick <= 1'b0;
        end
    end
endmodule
