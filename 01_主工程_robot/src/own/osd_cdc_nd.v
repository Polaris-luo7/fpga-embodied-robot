//=============================================================================
// osd_cdc_nd.v — 跨时钟域「数据 + 单拍使能」搬运（两拍同步 + 翻转握手）
//------------------------------------------------------------------------------
// 用途：数据源在 50 MHz 逻辑域算出 fps / 姿态角 / 电压（16 位值 + 单拍 nd），
//       需要安全送进 33 MHz 显示域写 OSD 字符缓冲。
//
// 为什么不能直接打两拍搬数据：多比特信号各比特到达时间不同，直接两级同步会在
//       目的域采到"半新半旧"的错值。正解是 **数据保持不动 + 只同步一个翻转位**：
//         a) 源域：nd_a 时把数据锁进 val_hold，同时翻转 req_a；
//         b) 目的域：req_a 打三拍，检测跳变得到单拍 nd_b —— 此刻 val_hold
//            已稳定 ≥2 个目的时钟，可安全采。
//       这正是本项目规约里"打两拍 + 单拍 nd 握手"的落地形式。
//
// 2026-10-01 WorkBuddy 新建（T10，W1 离线线）
//=============================================================================
`timescale 1ns/1ps

module osd_cdc_nd (
    // ---- 源域 ----
    input  wire        clk_a,
    input  wire        rst_n_a,
    input  wire [15:0] val_a,
    input  wire        nd_a,      // 源域单拍有效
    // ---- 目的域 ----
    input  wire        clk_b,
    input  wire        rst_n_b,
    output reg  [15:0] val_b,
    output reg         nd_b       // 目的域单拍有效（与 val_b 同拍更新）
);
    // ---- 源域：锁数据 + 翻转请求位 ----
    reg        req_a;
    reg [15:0] val_hold;

    always @(posedge clk_a or negedge rst_n_a) begin
        if (!rst_n_a) begin
            req_a    <= 1'b0;
            val_hold <= 16'd0;
        end else if (nd_a) begin
            val_hold <= val_a;
            req_a    <= ~req_a;
        end
    end

    // ---- 目的域：请求位打三拍，检测跳变 ----
    reg s1, s2, s3;
    always @(posedge clk_b or negedge rst_n_b) begin
        if (!rst_n_b) {s1, s2, s3} <= 3'b000;
        else          {s1, s2, s3} <= {req_a, s1, s2};
    end
    wire nd_b_pulse = s2 ^ s3;

    always @(posedge clk_b or negedge rst_n_b) begin
        if (!rst_n_b) begin
            val_b <= 16'd0;
            nd_b  <= 1'b0;
        end else begin
            nd_b <= nd_b_pulse;
            if (nd_b_pulse) val_b <= val_hold;
        end
    end

endmodule
