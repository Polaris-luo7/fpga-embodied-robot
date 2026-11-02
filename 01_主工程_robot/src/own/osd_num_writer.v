//=============================================================================
// osd_num_writer.v — 把数值格式化成定宽 ASCII 并写入 OSD 字符缓冲
//------------------------------------------------------------------------------
// 结构：
//   50 MHz 逻辑域 --(val_src, nd_src)--> osd_cdc_nd --> 33 MHz 显示域
//        --> 格式化 --> 逐字符写 osd_overlay 的 cbuf
//
// 为什么格式化放在显示域做：cbuf 写口在 pclk 域；跨域只搬一个 16 位数值
//   （用 osd_cdc_nd 的"数据保持 + 翻转握手"），比搬一整串 ASCII 简单且更安全。
//
// -----------------------------------------------------------------------------
// 【两种模式 MODE】
//   MODE = 0  无符号定宽整数（原功能，fps 用）
//               例：LEAD_ZERO=1, NDIG=3, LBL="FPS="  ->  "FPS=030"
//   MODE = 1  有符号一位小数，定宽 5 字符 [-]NN.N（右对齐）—— 姿态角用
//               ⚠ 输入单位必须与 imu_attitude 的 roll_x100 / pitch_x100
//                 **完全对齐**：**0.01 度单位的 int16**。本模块不另定单位。
//               例： 2994 -> " 29.9"    -125 -> " -1.3"    0 -> "  0.0"
//               舍入：四舍五入到 0.1 度（2994 -> 29.9；-125 -> -1.3）
//               超界：钳位到 +/-ANG_LIMIT（默认 9000 = 90.00 度）
//               右对齐规则：符号与数字紧贴，空格全部填在最左侧
//                 （所以 "-1.3" 显示成 " -1.3"，而不是 "- 1.3"）
//
// 【定宽策略（MODE=0）】
//   LEAD_ZERO = 1 → 前导零，"FPS=007"
//   LEAD_ZERO = 0 → 前导空格，"FPS=  7"（个位永远显示）
//
// 【前缀 LABEL】
//   用十六进制常量传，不用字符串参数（字符串参数的位选在部分综合器上不保险）。
//   **左对齐**语义：LBL 的最高字节是前缀的第一个字符，LBL_LEN 给长度。
//     空位必须补 0（会被当空格写出去，不影响）。
//   例（左对齐 48 位）：
//     "FPS="   -> 48'h4650533D0000, LBL_LEN=4
//     "ROLL="  -> 48'h524F4C4C3D0000, LBL_LEN=5
//     "PITCH=" -> 48'h50495443483D,   LBL_LEN=6
//   LBL_LEN 上限 6；索引算式 (5-i) 只依赖 i，永远落在 0..40 位内，无越界风险。
//
// 【stall / busy】
//   多个 writer 共用 osd_overlay 的**单个写口**时，仲裁器用 stall 让低优先级的
//   一方暂停、用 busy 判断谁在写。stall=0 时本模块行为与单 writer 版**逐位一致**。
//
// 【位置】ROW / COL0 全部参数化 —— 上板调观感只改参数，不动设计。
//
// 【修订记录】
//   2026-10-01 WorkBuddy 新建（T10 集成件）
//   2026-10-02 WorkBuddy T14：新增 MODE=1 有符号小数；LBL 由 32 位右对齐
//              改为 48 位左对齐；新增 stall 输入 / busy 输出（供 imu_display
//              双 writer 仲裁）。**MODE=0 的格式化逻辑一行未动**。
//=============================================================================
`timescale 1ns/1ps

module osd_num_writer #(
    parameter integer MODE       = 0,                  // 0=无符号整数; 1=有符号小数 [-]NN.N
    // ---- MODE=0 用 ----
    parameter integer NDIG       = 3,                  // 十进制位数
    parameter integer LEAD_ZERO  = 1,                  // 1=前导零, 0=前导空格
    parameter integer VAL_MAX    = 999,                // 数值上限（防溢出到 4 位）
    // ---- MODE=1 用 ----
    parameter integer ANG_LIMIT  = 9000,               // 钳位（0.01 度单位；9000 = 90.00 度）
    // ---- 公共 ----
    parameter [47:0]  LBL        = 48'h4650533D0000,   // 左对齐前缀；默认 "FPS="
    parameter integer LBL_LEN    = 4,                  // 前缀有效字符数, 0..6
    parameter [4:0]   ROW        = 5'd0,               // 写入哪一行
    parameter [5:0]   COL0       = 6'd0,               // 起始列
    parameter integer NCOL       = 40                  // 与 osd_overlay 一致
)(
    // ---- 50 MHz 源域 ----
    input  wire        clk_src,
    input  wire [15:0] val_src,
    input  wire        nd_src,
    // ---- 33 MHz 显示域 ----
    input  wire        pclk,
    input  wire        rst_n,        // 两个域共用（本工程同一复位）
    input  wire        stall,        // 1 = 暂停写（单 writer 使用时接 0）
    // ---- 写口 -> osd_overlay ----
    output reg         we,
    output reg  [4:0]  wrow,
    output reg  [5:0]  wcol,
    output reg  [7:0]  wcode,
    output reg         busy,         // 正在逐字符写（供仲裁）
    // ---- 观测（可挂 GAO）----
    output wire [15:0] val_dbg,
    output wire        nd_dbg
);
    localparam integer NCHR = (MODE == 0) ? NDIG : 5;   // 内容字符数
    localparam integer NTOT = LBL_LEN + NCHR;           // 总字符数

    //====================================================================
    // 1. 跨时钟域搬运数值
    //====================================================================
    wire [15:0] val_p;
    wire        nd_p;

    osd_cdc_nd u_cdc (
        .clk_a(clk_src), .rst_n_a(rst_n), .val_a(val_src), .nd_a(nd_src),
        .clk_b(pclk),    .rst_n_b(rst_n), .val_b(val_p),   .nd_b(nd_p)
    );
    assign val_dbg = val_p;
    assign nd_dbg  = nd_p;

    //====================================================================
    // 2. MODE=0：十进制展开（组合；除数 100/10 为常量，综合会折成乘法/移位）
    //    位宽显式收窄，避免 GowinSyn 报 EX3791（17 位截到 8 位）
    //====================================================================
    wire [15:0] v  = (val_p > VAL_MAX[15:0]) ? VAL_MAX[15:0] : val_p;
    wire [15:0] q2 = v / 16'd100;          // 百位
    wire [15:0] q1 = (v / 16'd10) % 16'd10;// 十位
    wire [15:0] q0 = v % 16'd10;           // 个位
    wire [7:0]  d2 = 8'h30 + q2[7:0];
    wire [7:0]  d1 = 8'h30 + q1[7:0];
    wire [7:0]  d0 = 8'h30 + q0[7:0];

    //====================================================================
    // 3. MODE=1：有符号小数 [-]NN.N（5 字符，右对齐）
    //    输入 = 0.01 度单位 int16（与 imu_attitude 输出同单位）
    //--------------------------------------------------------------------
    //    ① 用 17 位有符号做钳位（避免 16 位参数带 signed 的可移植性问题）
    //    ② q = round(|v| / 10)   —— 四舍五入到 0.1 度，0..900
    //    ③ 拆 tens / ones / tenth，配符号，右对齐到 5 个字符格
    //====================================================================
    wire signed [16:0] vv17  = {val_p[15], val_p};
    wire signed [16:0] lim17 = 17'sd9000;                       // ANG_LIMIT 的 17 位形式
    wire signed [16:0] vc17  = (vv17 >  lim17) ?  lim17 :
                               ((vv17 < -lim17) ? -lim17 : vv17);
    wire               neg   = vc17[16];
    wire        [16:0] av    = neg ? (~vc17 + 17'd1) : vc17;    // |vc|, 0..9000

    wire [15:0] q       = (av[15:0] + 16'd5) / 16'd10;          // 0.1 度单位, 0..900
    wire [15:0] f_tens  = q / 16'd100;                          // 十位
    wire [15:0] f_ones  = (q / 16'd10) % 16'd10;                // 个位
    wire [15:0] f_tenth = q % 16'd10;                           // 十分位
    wire        has_tens = (q >= 16'd100);                      // 角度 >= 10.0 度

    wire [7:0] c_tens  = 8'h30 + f_tens[7:0];
    wire [7:0] c_ones  = 8'h30 + f_ones[7:0];
    wire [7:0] c_tenth = 8'h30 + f_tenth[7:0];
    wire [7:0] c_sign  = neg ? 8'h2D : 8'h20;                   // '-' 或 ' '

    // 右对齐 5 格（f0 最左, f4 最右）。符号与数字紧贴，空格只出现在最左。
    //   has_tens neg
    //      0       0  -> "   X.Y"
    //      0       1  -> "  -X.Y"
    //      1       0  -> "  XX.Y"
    //      1       1  -> "-XX.Y"
    // ⚠ f1 的判据是 **has_tens 优先**，不能写成 (has_tens && !neg)：
    //   否则 "-90.0" 会被拆成 "-" + 空格 + "0.0"（本 bug 由 Python 逐位复算抓出）。
    wire [7:0] f0 = ( has_tens &&  neg) ? c_sign : 8'h20;
    wire [7:0] f1 = has_tens ? c_tens : (neg ? c_sign : 8'h20);
    wire [7:0] f2 = c_ones;
    wire [7:0] f3 = 8'h2E;                                      // '.'
    wire [7:0] f4 = c_tenth;

    //====================================================================
    // 4. 组装字符串 + 逐字符写出
    //====================================================================
    reg [7:0] str [0:11];     // 最多 6 前缀 + 5 内容
    reg [3:0] idx;

    integer i;
    always @(posedge pclk or negedge rst_n) begin
        if (!rst_n) begin
            we    <= 1'b0;
            busy  <= 1'b0;
            idx   <= 4'd0;
            wrow  <= ROW;
            wcol  <= COL0;
            wcode <= 8'h20;
            for (i = 0; i < 12; i = i + 1) str[i] <= 8'h20;
        end else begin
            we <= 1'b0;

            if (nd_p) begin
                // ---- 前缀（左对齐常量展开）----
                for (i = 0; i < 6; i = i + 1) begin
                    str[i] <= (i < LBL_LEN) ? LBL[(5-i)*8 +: 8] : 8'h20;
                end

                // ---- 内容 ----
                if (MODE == 0) begin
                    // 原整数逻辑，逐字保留
                    if (LEAD_ZERO) begin
                        str[LBL_LEN]   <= d2;
                        str[LBL_LEN+1] <= d1;
                        str[LBL_LEN+2] <= d0;
                    end else begin
                        str[LBL_LEN]   <= (v >= 16'd100) ? d2 : 8'h20;
                        str[LBL_LEN+1] <= (v >= 16'd10 ) ? d1 : 8'h20;
                        str[LBL_LEN+2] <= d0;
                    end
                end else begin
                    str[LBL_LEN+0] <= f0;
                    str[LBL_LEN+1] <= f1;
                    str[LBL_LEN+2] <= f2;
                    str[LBL_LEN+3] <= f3;
                    str[LBL_LEN+4] <= f4;
                end

                busy <= 1'b1;
                idx  <= 4'd0;
            end else if (busy && !stall) begin
                // ---- 逐字符写（idx 走满 NTOT 拍）----
                // 注意 idx 是 4 位，要补成 6 位再与 COL0 相加
                we   <= 1'b1;
                wrow <= ROW;
                wcol <= (COL0 + {2'b00, idx} < NCOL) ? (COL0 + {2'b00, idx}) : 6'd0;
                wcode<= str[idx];
                if (idx >= NTOT[3:0] - 4'd1) busy <= 1'b0;
                else                          idx  <= idx + 4'd1;
            end
        end
    end

endmodule
