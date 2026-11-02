//=============================================================================
// div_const.v — 常量被除数顺序除法器（恢复余数法，32 拍）
//------------------------------------------------------------------------------
// quot = DIVIDEND / divisor（divisor > 0，24 bit）
// start 脉冲启动（busy 时忽略）；busy 下降沿即结果就绪（quot 有效）。
// 用于 speed_meas 的两处：period → 速度、elapsed → 静默上界。
//=============================================================================
`timescale 1ns/1ps

module div_const #(
    parameter integer DIVIDEND = 50_000_000,  // 26 bit
    parameter integer DIVW     = 24
)(
    input  wire             clk,
    input  wire             rst_n,
    input  wire             start,
    input  wire [DIVW-1:0]  divisor,   // 必须 > 0
    output reg  [31:0]      quot,
    output reg              busy
);
    reg [31:0] dividend;
    reg [31:0] dvsr;
    reg [31:0] rem_r;
    reg [5:0]  bit_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            quot     <= 32'd0;
            busy     <= 1'b0;
            dividend <= 32'd0;
            dvsr     <= 32'd0;
            rem_r    <= 32'd0;
            bit_cnt  <= 6'd0;
        end else if (!busy) begin
            if (start && divisor != 0) begin
                busy     <= 1'b1;
                dividend <= DIVIDEND;
                dvsr     <= {{(32-DIVW){1'b0}}, divisor};
                rem_r    <= 32'd0;
                quot     <= 32'd0;
                bit_cnt  <= 6'd31;
            end
        end else begin
            if (bit_cnt != 6'd0) begin
                if ({rem_r[30:0], dividend[31]} >= dvsr) begin
                    rem_r   <= {rem_r[30:0], dividend[31]} - dvsr;
                    quot    <= {quot[30:0], 1'b1};
                end else begin
                    rem_r   <= {rem_r[30:0], dividend[31]};
                    quot    <= {quot[30:0], 1'b0};
                end
                dividend <= dividend << 1;
                bit_cnt  <= bit_cnt - 1'b1;
            end else begin
                if ({rem_r[30:0], dividend[31]} >= dvsr)
                    quot <= {quot[30:0], 1'b1};
                else
                    quot <= {quot[30:0], 1'b0};
                busy <= 1'b0;
            end
        end
    end
endmodule
