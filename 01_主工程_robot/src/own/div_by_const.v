//=============================================================================
// div_by_const.v — 变量被除数 / 常量除数 顺序除法器（恢复余数法，DW 拍）
//------------------------------------------------------------------------------
// quot = dividend / DIVISOR（DIVISOR 是编译期常量，必须 > 0）
// start 单拍启动（busy 期间忽略）；busy 下降沿 quot 有效，done 同拍单脉冲。
//
// ⚠️ 为什么不直接复用 rtl/motor_loop/div_const.v（MiMo 建议的那颗）：
//    验证过它的接口 —— `parameter DIVIDEND` 是**编译期常量被除数**，
//    `divisor` 是**运行时输入**，即它算的是「常量 ÷ 变量」。
//    本项目这里要的是「**变量 ÷ 常量**」（pclk_cnt / 1e6），方向正好相反，
//    不是"换个参数就能用"。照抄它的算法（恢复余数法）写一个反方向版本，
//    并把新除法器留在 OSD 侧，**不动 motor_loop 那颗已被速度环验证过的模块**。
//    （改 div_const 的接口会波及 speed_meas.v 的 T3/T5 证据，性价比不划算。）
//=============================================================================
`timescale 1ns/1ps

module div_by_const #(
    parameter integer DIVISOR = 1_000_000,
    parameter integer DW      = 32
)(
    input  wire            clk,
    input  wire            rst_n,
    input  wire            start,
    input  wire [DW-1:0]   dividend,
    output reg  [DW-1:0]   quot,
    output reg             busy,
    output wire            done        // busy 下降沿，单拍
);
    localparam [DW-1:0] DV       = DIVISOR;      // 常量除数（避免到处做参数位选）
    localparam [5:0]    CNT_INI  = DW - 1;       // 迭代次数 = DW

    reg [DW-1:0] dd;      // 被除数移位寄存器（每次左移一位，MSB 喂进余数）
    reg [DW-1:0] rem;     // 余数（恒 < DIVISOR）
    reg [DW-1:0] q;       // 商（MSB 先出）
    reg [5:0]    cnt;

    wire [DW-1:0] rem_sh = {rem[DW-2:0], dd[DW-1]};
    wire          ge     = (rem_sh >= DV);

    reg busy_d;
    assign done = busy_d & ~busy;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            busy   <= 1'b0;
            busy_d <= 1'b0;
            quot   <= {DW{1'b0}};
            dd     <= {DW{1'b0}};
            rem    <= {DW{1'b0}};
            q      <= {DW{1'b0}};
            cnt    <= 6'd0;
        end else begin
            busy_d <= busy;
            if (!busy) begin
                if (start) begin
                    busy <= 1'b1;
                    dd   <= dividend;
                    rem  <= {DW{1'b0}};
                    q    <= {DW{1'b0}};
                    cnt  <= CNT_INI;
                end
            end else begin
                if (cnt != 6'd0) begin
                    rem <= ge ? (rem_sh - DV) : rem_sh;
                    q   <= {q[DW-2:0], ge};
                    dd  <= {dd[DW-2:0], 1'b0};
                    cnt <= cnt - 6'd1;
                end else begin
                    quot <= {q[DW-2:0], ge};
                    busy <= 1'b0;
                end
            end
        end
    end

endmodule
