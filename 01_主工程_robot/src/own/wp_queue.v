//=============================================================================
// wp_queue.v -- 途径点队列（任务四 P1/P2）: 16 点深度, 支持断点续跑
//-----------------------------------------------------------------------------
// 【用途】手机 $ADD,x,y 逐条追加；$NAV/$GO 时按顺序取出；$WP,n 直接跳到
//         第 n 点续跑（排队指针复位到 n）。
//
// 【寄存器实现】16 点 × 2 × 32bit = 1024 bit。用**分布式寄存器**（少量 LUT）
//   而非 BSRAM：深度小、读写都是单拍、无 BRAM 端口时序复杂度。
//
// 【语义】
//   clr        清空队列（wr/rd 指针归零，count=0）
//   push       count<16 时写入 (x,y)，count++；满则 full=1（调用方回 $ERR）
//   pop        取队头 -> head_x/head_y，rd++，count--（1 拍脉冲）
//   seek_n[3:0] + seek_en  把 rd 指针直接置到索引 n（$WP,n 断点续跑）
//   head_x/head_y 组合给出当前队头元素（pop 前可直接读）
//
// 【验收】tb_wp_queue.v：push 满/清空/顺序 pop/seek/边界（空队列 pop）
//=============================================================================
`timescale 1ns/1ps

module wp_queue #(
    parameter integer DEPTH = 16
)(
    input  wire               clk,
    input  wire               rst_n,
    input  wire               clr,
    input  wire               push,
    input  wire signed [31:0] push_x,
    input  wire signed [31:0] push_y,
    input  wire               pop,
    input  wire               seek_en,
    input  wire [4:0]         seek_n,

    output wire signed [31:0] head_x,
    output wire signed [31:0] head_y,
    output reg  [4:0]         count,
    output wire               empty,
    output wire               full
);
    reg signed [31:0] qx [0:DEPTH-1];
    reg signed [31:0] qy [0:DEPTH-1];
    reg [4:0] wr, rd;

    assign empty = (count == 5'd0);
    assign full  = (count >= DEPTH[4:0]);
    // ★时序修复 2026-10-17：原为组合 RAM 读出，形成 wp_queue → nav_ctrl → 方位计算 的长路径
    //   （实测 −6.16ns）。加一级寄存；nav_ctrl 在 10ms 节拍上读取，晚 1 拍（20ns）无影响。
    reg signed [31:0] head_x_r, head_y_r;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin head_x_r <= 32'sd0; head_y_r <= 32'sd0; end
        else begin head_x_r <= qx[rd[3:0]]; head_y_r <= qy[rd[3:0]]; end
    assign head_x = head_x_r;
    assign head_y = head_y_r;

    integer i;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr <= 5'd0; rd <= 5'd0; count <= 5'd0;
            for (i = 0; i < DEPTH; i = i + 1) begin
                qx[i] <= 32'sd0; qy[i] <= 32'sd0;
            end
        end
        else if (clr) begin
            wr <= 5'd0; rd <= 5'd0; count <= 5'd0;
        end
        else begin
            if (push && !full) begin
                qx[wr[3:0]] <= push_x;
                qy[wr[3:0]] <= push_y;
                wr <= wr + 5'd1;
                count <= count + 5'd1;
            end
            if (seek_en) begin                    // $WP,n: 指针直达(n 从 0 起)
                rd    <= seek_n;
                count <= (wr > seek_n) ? (wr - seek_n) : 5'd0;
            end
            else if (pop && !empty) begin
                rd    <= rd + 5'd1;
                count <= count - 5'd1;
            end
        end
    end

endmodule
