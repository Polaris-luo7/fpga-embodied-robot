//=============================================================================
// cordic_atan2.v -- 定点 CORDIC（向量模式）：一次同时得到 atan2 与模长
//-----------------------------------------------------------------------------
// 【为什么用它】
//   姿态解算要 atan2，而且要开根号：
//       roll  = atan2(ay, az)
//       pitch = atan2(-ax, sqrt(ay^2+az^2))
//   CORDIC 向量模式正好一次给出"角度 + 模长"，sqrt 问题顺带解决。
//   全部由加/减/移位构成，无乘法、无 DSP、无浮点——符合工程规约第 3 条。
//
// 【定点格式（写死，改这里必须同步改调用方）】
//   输入  : 32 位有符号整数。小数位数由**调用方左移**决定，本模块不自带 shift。
//           - 第一级：输入 = (加速度原始 LSB) << 12
//           - 第二级：输入 = 第一级的 mag 输出（已是 <<12 尺度）
//   角度输出: Q16.16，单位 = **度**。例：45 度 -> 2949120。180 度 -> 11796480。
//   模长输出: Q(同输入)，含 CORDIC 增益 A_n=1.646760。**只应当作下一级的 x 输入**
//             （角度与尺度无关），不要直接当真实模长用。
//
// 【收敛范围与象限折叠（重要）】
//   向量模式的固有收敛域只有 ±sum(atan(2^-i)), i=0..15 ≈ ±99.88 度，
//   超出会饱满到 99.88 —— 这是第一版 Python 自校验抓出来的
//   （(x,y)=(-8192,8192) 期望 135 度、实得 99.88 度）。
//   本模块内置折叠：x_in<0 时把 (x,y) 整体取反（角度落回 ±90 内），
//   迭代结束后按 **原始 y 的符号** ±180 修正，从而覆盖 (-180,180]。
//
// 【迭代结构】每拍一级，ITER 拍出 done，纯时序无长组合路径
//   （32 级组合 CORDIC 在 50MHz 下时序不可达，故必须打拍）。
//
// 【常量来源】角度表由 rtl/imu/gen_cordic_const.py 生成并自校验，
//   不手算——atan(2^-i) 手算到 7 位有效数字极易出错，且错了表现为
//   "角度整体偏一点"，极难定位。
//=============================================================================
`timescale 1ns / 1ps

module cordic_atan2 #(
    parameter integer ITER = 16          // 迭代次数；角度精度 ≈ atan(2^-ITER)
)(
    input  wire               clk,
    input  wire               rst_n,
    input  wire               start,     // 单拍启动
    input  wire signed [31:0] x_in,
    input  wire signed [31:0] y_in,
    output reg                busy,
    output reg                done,      // 单拍完成
    output reg  signed [31:0] angle_q16, // Q16.16，单位：度
    output reg  signed [31:0] mag_q16    // 含增益 A_n，仅供下一级当 x 输入
);

    localparam signed [31:0] ANG180 = 32'sd11796480;   // 180 度 = 180*65536

    // ---------------------------------------------------------------------
    // atan(2^-i) 角度表（Q16.16，单位：度）—— 由 gen_cordic_const.py 生成
    // ---------------------------------------------------------------------
    function signed [31:0] ATAN_Q16(input [4:0] k);
        case (k)
            5'd0 : ATAN_Q16 = 32'sd2949120;   // 45.000000
            5'd1 : ATAN_Q16 = 32'sd1740967;   // 26.565051
            5'd2 : ATAN_Q16 = 32'sd919879;    // 14.036243
            5'd3 : ATAN_Q16 = 32'sd466945;    //  7.125016
            5'd4 : ATAN_Q16 = 32'sd234379;    //  3.576334
            5'd5 : ATAN_Q16 = 32'sd117304;    //  1.789911
            5'd6 : ATAN_Q16 = 32'sd58666;     //  0.895174
            5'd7 : ATAN_Q16 = 32'sd29335;     //  0.447614
            5'd8 : ATAN_Q16 = 32'sd14668;     //  0.223811
            5'd9 : ATAN_Q16 = 32'sd7334;      //  0.111906
            5'd10: ATAN_Q16 = 32'sd3667;      //  0.055953
            5'd11: ATAN_Q16 = 32'sd1833;      //  0.027976
            5'd12: ATAN_Q16 = 32'sd917;       //  0.013988
            5'd13: ATAN_Q16 = 32'sd458;       //  0.006994
            5'd14: ATAN_Q16 = 32'sd229;       //  0.003497
            default: ATAN_Q16 = 32'sd115;     //  0.001749
        endcase
    endfunction

    reg signed [31:0] x, y, z;
    reg [4:0]  i;
    reg        x_neg;      // 原始 x 的符号（决定是否做 +-180 修正）
    reg        y_neg;      // 原始 y 的符号（决定 +180 还是 -180）

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            busy      <= 1'b0;
            done      <= 1'b0;
            i         <= 5'd0;
            x         <= 32'sd0;
            y         <= 32'sd0;
            z         <= 32'sd0;
            x_neg     <= 1'b0;
            y_neg     <= 1'b0;
            // ⚠️ 2026-10-08：angle_q16/mag_q16 是**纯组合输出**，只能由下面那个
            //   always @(*) 驱动。原先在这里也赋了值 ⇒ 两个 always 块驱动同一 reg
            //   ⇒ Gowin 报 EX2000/EX1999（Modelsim 容忍，故仿真未暴露）。已删。
        end else begin
            done <= 1'b0;

            if (!busy) begin
                if (start) begin
                    // 象限折叠：x<0 时整体取反
                    x_neg <= x_in[31];
                    y_neg <= y_in[31];
                    x     <= x_in[31] ? -x_in : x_in;
                    y     <= x_in[31] ? -y_in : y_in;
                    z     <= 32'sd0;
                    i     <= 5'd0;
                    busy  <= 1'b1;
                end
            end else begin
                // 向量模式：把 (x,y) 旋到 x 轴
                x <= (y < 0) ? (x - (y >>> i)) : (x + (y >>> i));
                y <= (y < 0) ? (y + (x >>> i)) : (y - (x >>> i));
                z <= (y < 0) ? (z - ATAN_Q16(i)) : (z + ATAN_Q16(i));

                if (i == ITER[4:0] - 5'd1) begin
                    busy <= 1'b0;
                    done <= 1'b1;
                end else begin
                    i <= i + 5'd1;
                end
            end
        end
    end

    // ---------------------------------------------------------------------
    // 输出：象限修正
    //   x>=0            -> z
    //   x<0 且 y_orig>=0 -> z + 180
    //   x<0 且 y_orig<0  -> z - 180
    // ---------------------------------------------------------------------
    always @(*) begin
        if (!x_neg) begin
            angle_q16 = z;
        end else if (y_neg) begin
            angle_q16 = z - ANG180;
        end else begin
            angle_q16 = z + ANG180;
        end
        mag_q16 = x;
    end

endmodule
