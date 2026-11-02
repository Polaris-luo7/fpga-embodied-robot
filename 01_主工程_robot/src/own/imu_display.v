//=============================================================================
// imu_display.v — IMU 姿态角显示桥（50 MHz 逻辑域 -> 33 MHz 显示域 -> OSD 第 2/3 行）
//------------------------------------------------------------------------------
// 【在系统里的位置】
//   imu_attitude（50 MHz 域）出 roll_x100 / pitch_x100（**0.01 度单位的 int16**）
//        --> 本模块：跨域搬运 + 格式化成 "ROLL= 29.9" / "PITCH= -1.3"
//        --> osd_overlay 的字符缓冲写口（33 MHz 域）
//
// 【为什么必须有这一层】
//   1. 跨时钟域：osd_num_writer 的 cbuf 写口在 pclk(33 MHz) 域，姿态值在 50 MHz 域。
//      **不能直连** —— 本项目规约（交接文档 §8.2）明写"任何跨时钟域的信号一律打两拍
//      + 单拍 nd 握手或走 FIFO"。这里由 osd_num_writer 内部的 osd_cdc_nd 完成。
//   2. 单位对齐：直接复用 imu_attitude 的 roll_x100 / pitch_x100，
//      **不在这一层另定一套角度单位**（避免"两处各有一套单位"的经典事故）。
//   3. 写口仲裁：osd_overlay 只有 **一个** 写口，两个 writer 不能同时写。
//      本模块用 stall/busy 做优先级仲裁（见下）。
//
// 【写口仲裁（roll 优先）】
//   osd_overlay 的写口一拍只能接收一个 (row,col,code)。
//   策略：**roll 优先** —— pitch 在 roll 正在逐字符写期间被 stall 暂停。
//   为什么安全：writer 的写序列是"连续若干拍每次写 1 个字符"，roll 一旦 busy
//     就是连续 NTOT 拍都有 we；把 pitch 压住即可，不会出现同拍两个 we。
//   为什么不会丢数据：每个 writer 在 nd 到来时就把整串字符**锁进自己的 str[]**，
//     stall 只推迟写出、不影响内容；若同一位姿在两串写完前又更新，则刷新为新值
//     （屏幕上永远显示最新姿态，符合预期）。
//
// 【行号】roll 默认第 2 行(ROW=1)、pitch 默认第 3 行(ROW=2)。
//   想改成同一行：把 PITCH_ROW 设成与 ROLL_ROW 相同、PITCH_COL0 设到 roll 串之后
//   （例如 ROLL_COL0=0 占 10 字符 -> PITCH_COL0=10）。
//   （osd_overlay 的 NROW=4，行 0..3 可用；行 0 留给 fps。）
//
// 【修订记录】
//   2026-10-02 WorkBuddy 新建（T14，任务一离线部分收尾）
//=============================================================================
`timescale 1ns/1ps

module imu_display #(
    parameter integer NCOL = 40,

    // ---- roll（默认第 2 行）----
    parameter [4:0]   ROLL_ROW      = 5'd1,
    parameter [5:0]   ROLL_COL0     = 6'd0,
    parameter [47:0]  ROLL_LBL      = 48'h524F4C4C3D00,    // "ROLL=" 左对齐（6 字节）
    parameter integer ROLL_LBL_LEN  = 5,

    // ---- pitch（默认第 3 行）----
    parameter [4:0]   PITCH_ROW     = 5'd2,
    parameter [5:0]   PITCH_COL0    = 6'd0,
    parameter [47:0]  PITCH_LBL     = 48'h50495443483D,    // "PITCH=" 左对齐
    parameter integer PITCH_LBL_LEN = 6,

    // ---- 与 imu_attitude 一致的钳位（0.01 度）----
    parameter integer ANG_LIMIT     = 9000
)(
    // ---- 50 MHz 源域 ----
    input  wire               clk_src,
    input  wire               rst_n,
    input  wire signed [15:0] roll_x100,      // 0.01 度单位（直接用 imu_attitude 的输出）
    input  wire signed [15:0] pitch_x100,
    input  wire               nd_src,         // 姿态更新单拍脉冲

    // ---- 33 MHz 显示域 ----
    input  wire               pclk,

    // ---- 写口（-> osd_overlay）----
    output wire               we,
    output wire [4:0]         wrow,
    output wire [5:0]         wcol,
    output wire [7:0]         wcode,

    // ---- 观测 ----
    output wire               roll_busy,
    output wire               pitch_busy
);

    //====================================================================
    // roll writer
    //====================================================================
    wire        r_we;
    wire [4:0]  r_row;
    wire [5:0]  r_col;
    wire [7:0]  r_code;
    wire        r_busy;

    osd_num_writer #(
        .MODE     (1),
        .ANG_LIMIT(ANG_LIMIT),
        .LBL      (ROLL_LBL),
        .LBL_LEN  (ROLL_LBL_LEN),
        .ROW      (ROLL_ROW),
        .COL0     (ROLL_COL0),
        .NCOL     (NCOL)
    ) u_roll (
        .clk_src (clk_src), .val_src (roll_x100), .nd_src (nd_src),
        .pclk    (pclk),    .rst_n   (rst_n),
        .stall   (1'b0),                    // roll 优先，永不被暂停
        .we      (r_we),   .wrow (r_row), .wcol (r_col), .wcode (r_code),
        .busy    (r_busy),
        .val_dbg (), .nd_dbg ()
    );

    //====================================================================
    // pitch writer —— 在 roll 逐字符写期间被 stall
    //====================================================================
    wire        p_we;
    wire [4:0]  p_row;
    wire [5:0]  p_col;
    wire [7:0]  p_code;
    wire        p_busy;

    osd_num_writer #(
        .MODE     (1),
        .ANG_LIMIT(ANG_LIMIT),
        .LBL      (PITCH_LBL),
        .LBL_LEN  (PITCH_LBL_LEN),
        .ROW      (PITCH_ROW),
        .COL0     (PITCH_COL0),
        .NCOL     (NCOL)
    ) u_pitch (
        .clk_src (clk_src), .val_src (pitch_x100), .nd_src (nd_src),
        .pclk    (pclk),    .rst_n   (rst_n),
        .stall   (r_busy),                  // ← 仲裁：roll 忙则 pitch 暂停
        .we      (p_we),   .wrow (p_row), .wcol (p_col), .wcode (p_code),
        .busy    (p_busy),
        .val_dbg (), .nd_dbg ()
    );

    //====================================================================
    // 写口合并（roll 优先；因 stall 机制，二者不会同拍同时置 we）
    //====================================================================
    assign we    = r_we | p_we;
    assign wrow  = r_we ? r_row  : p_row;
    assign wcol  = r_we ? r_col  : p_col;
    assign wcode = r_we ? r_code : p_code;

    assign roll_busy  = r_busy;
    assign pitch_busy = p_busy;

endmodule
