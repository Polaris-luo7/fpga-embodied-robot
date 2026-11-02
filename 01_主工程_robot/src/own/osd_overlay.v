//=============================================================================
// osd_overlay.v — 字符叠加（OSD）：在视频流上叠 ASCII 字符
//------------------------------------------------------------------------------
// 依据：教材第 31 章《HDMI 和 TFT 显示屏的静态字符显示》的 char_rom + char_extract
//       架构，做一处关键改造 ——
//   第31章把"整串静态点阵"烧进 ROM（176bit x 32 行），内容焊死在比特流里；
//   本模块拆成 font_rom（不可变字库）+ cbuf（可变 ASCII 串，带硬逻辑写入口），
//   于是能显示每秒在变的 fps / 姿态角 / 电压。
//
// 叠加位置：disp_driver 输出之后、TFT/HDMI 并口之前（本模块串在显示时序链上）。
//
// 时序要点（关键）：font_rom 读出延迟 1 拍。为让"当拍像素"就取到字形字节，
//   ROM 地址取 **x_in+1（前视 1 像素）** ⇒ 本拍 ROM 输出 = 上拍地址 =
//   针对当前 x_in 的地址。因此 rgb_in -> rgb_out 是**纯组合**路径，无需延时
//   对齐、也不会横向错位一格；格内列 cc 由当前 x_in 组合算出，与 ROM 输出严格同拍。
//
// 跨时钟域：cbuf 写口在 pclk 域。数据源若在别的时钟域（如 50 MHz），必须先用
//   osd_cdc_nd（两拍同步 + 单拍 nd 握手）搬过来 —— 本模块不承担 CDC。
//
// 2026-10-01 WorkBuddy 新建（T10，W1 离线线）
// 2026-10-03 WorkBuddy 修订（实验9）：去底框改纯叠加。
// 2026-10-03 WorkBuddy 修订（实验10）：字库加 1 像素描边平面（亮背景下可读）。
//   接口仅多一个 EDGE 参数；ROM 由 4096 项 x 8 bit 改为 2048 项 x 16 bit（**等长**，
//   索引由 code[7:0]*16+row 折成 code[6:0]*16+row，只用 ASCII 0..127）。
//=============================================================================
`timescale 1ns/1ps

module osd_overlay #(
    parameter integer H_ACTIVE = 800,
    parameter integer V_ACTIVE = 480,
    parameter integer CHAR_W   = 8,          // 字形宽（像素）
    parameter integer CHAR_H   = 16,         // 字形高（行）
    parameter integer NROW     = 4,          // 文本行数
    parameter integer NCOL     = 40,         // 每行字符数
    parameter integer BOX_X    = 8,          // 文本窗口左上角 X
    parameter integer BOX_Y    = 8,          // 文本窗口左上角 Y
    parameter [15:0]  INK      = 16'h07E0,   // 字符色（RGB565 绿）
    parameter [15:0]  EDGE     = 16'h0000,   // 描边色（纯黑）—— 实验10 新增
    parameter [15:0]  BGC      = 16'h0000,   // 旧底框色；实验9 起不再参与合成（保留以兼容调用方）
    parameter integer FONT_DEPTH = 2048,     // 字库项数 = 128 code x 16 row
    parameter         FONT_INIT  = "font_rom.hex"
)(
    input  wire        pclk,        // 显示像素时钟域（TFT 33 MHz 侧）
    input  wire        rst_n,

    // ---- 显示时序输入（来自 disp_driver）----
    input  wire        de_in,
    input  wire [11:0] x_in,
    input  wire [11:0] y_in,
    input  wire [15:0] rgb_in,

    // ---- 叠加后输出（去 TFT/HDMI 并口）----
    output wire        de_out,
    output wire [15:0] rgb_out,

    // ---- 字符缓冲写口（pclk 域；来自 num2ascii 或软核）----
    input  wire        we,
    input  wire [4:0]  wrow,
    input  wire [5:0]  wcol,
    input  wire [7:0]  wcode,

    // ---- 控制 ----
    input  wire        disp_en     // 0 = 全透传（演示兜底：OSD 出问题可一键关）
);

    localparam integer BOX_W  = NCOL * CHAR_W;
    localparam integer BOX_H  = NROW * CHAR_H;
    localparam integer CBUF_N = NROW * NCOL;

    //====================================================================
    // 1. 可变字符缓冲（小容量 LUTRAM；复位清成空格，避免未初始化区出 x）
    //====================================================================
    reg [7:0] cbuf [0:CBUF_N-1];
    integer bi;
    integer fi;      // 供 font_rom_init.vh 的零填充循环使用
    always @(posedge pclk) begin
        if (!rst_n) begin
            for (bi = 0; bi < CBUF_N; bi = bi + 1) cbuf[bi] <= 8'h20;  // ' '
        end else if (we) begin
            cbuf[wrow*NCOL + wcol] <= wcode;
        end
    end

    //====================================================================
    // 2. 字库 ROM（Block ROM；同步读，1 拍延迟）
    //    索引 = code[6:0]*16 + row，每项 2 字节 {edge[7:0], ink[7:0]}，各 bit7 = 最左像素
    //    ⚠️ code 折成 7 位后只用 ASCII 0..127：2048 项 x 16 bit = 32768 bit，
    //       与旧版 4096 项 x 8 bit **等长** ⇒ BSRAM 占用不应变化，
    //       而且同一个读口一次就把「笔画 + 描边」两个平面一起取回。
    //    （若将来要用 code >= 128，需把 faddr_n 与两个平面一起加宽，见修订记录）
    //
    //    内容来源（两种，默认选安全的那种）：
    //      默认  `include "font_rom_init.vh"  —— 同目录、随源码走，零 cwd 依赖。
    //            `include 相对的是**本文件所在目录**，比下一条可靠得多。
    //      可选  `define FONT_USE_READMEMH 后走 $readmemh(FONT_INIT) —— 需要
    //            FONT_INIT 是可解析路径（Gowin 综合 cwd 通常是工程根）。
    //
    //    ⚠️ 之所以默认不用 $readmemh：路径不对时它**不报错**，而是静默读到空
    //       ⇒ 字库全 0 ⇒ 屏上一片空白，很容易误判成叠加逻辑写错了。
    //====================================================================
    (* rom_style = "block" *) reg [15:0] fnt [0:FONT_DEPTH-1];
`ifdef FONT_USE_READMEMH
    initial $readmemh(FONT_INIT, fnt);
`else
    `include "font_rom_init.vh"
`endif

    //====================================================================
    // 3. 当前像素几何
    //====================================================================
    wire in_y   = (y_in >= BOX_Y) && (y_in < BOX_Y + BOX_H);
    wire in_box = de_in && (x_in >= BOX_X) && (x_in < BOX_X + BOX_W) && in_y;

    wire [11:0] xr = (x_in >= BOX_X) ? (x_in - BOX_X) : 12'd0;
    wire [11:0] yr = in_y            ? (y_in - BOX_Y) : 12'd0;
    wire [2:0]  cc = xr % CHAR_W;                     // 格内列（CHAR_W=8 → xr[2:0]）

    //====================================================================
    // 4. 前视 1 像素算 ROM 地址（抵消 ROM 的 1 拍读出延迟）
    //====================================================================
    wire [11:0] xp   = x_in + 12'd1;
    wire        in_xp = (xp >= BOX_X) && (xp < BOX_X + BOX_W);
    wire [11:0] xrp  = in_xp ? (xp - BOX_X) : 12'd0;
    wire [4:0]  row_n = yr / CHAR_H;                  // 显示行（CHAR_H=16 → yr[8:4]）
    wire [3:0]  lr_n  = yr % CHAR_H;                  // 字形内行
    wire [5:0]  ci_n  = xrp / CHAR_W;                 // 字符列
    wire [7:0]  code_n = cbuf[row_n*NCOL + ci_n];
    wire [10:0] faddr_n = {code_n[6:0], lr_n};        // code[6:0]*16 + row（折 7 位，见第 2 节）

    reg [15:0] fdata_r;
    always @(posedge pclk) fdata_r <= fnt[faddr_n];

    //====================================================================
    // 5. 合成：【纯叠加 + 文字描边】（不画底框）
    //    修订记录 2026-10-03（实验9）：
    //      旧写法 `(in_box && disp_en) ? (bit_on ? INK : BGC) : rgb_in`
    //      会把整个 NROW x NCOL 字符网格刷成底色 BGC ⇒ 屏上出现一块大底框
    //      （本项目 40x4 网格 = 320x64 px，而 fps 只占 1 行 8 字符 ⇒ 框里约 90% 是空的）。
    //      新写法只在 bit_on=1 时覆盖，窗口内的空白像素也透传原图像 ⇒ 纯叠加。
    //    修订记录 2026-10-03（实验10）：纯叠加暴露新问题 —— 白墙/白桌面上绿字发糊。
    //      本版给字形加 1 像素描边（EDGE，纯黑）：
    //        ink_on =1  -> INK
    //        edge_on=1  -> EDGE
    //        否则        -> rgb_in 透传
    //      描边由字库离线生成（8 邻域膨胀减本体），与 ink 互斥；此处顺序只是防御写法。
    //      ⇒ 亮背景可读性恢复，且画面上**不出现任何大块底色矩形**。
    //    接口不变：BGC 参数保留（不再参与合成），disp_en 仍为总开关。
    //====================================================================
    wire [7:0] ink_b  = fdata_r[7:0];      // 笔画平面
    wire [7:0] edge_b = fdata_r[15:8];     // 描边平面

    wire ink_on  = ink_b [CHAR_W-1 - cc];
    wire edge_on = edge_b[CHAR_W-1 - cc];

    assign de_out  = de_in;
    assign rgb_out = (in_box && disp_en)
                   ? (ink_on ? INK : (edge_on ? EDGE : rgb_in))
                   : rgb_in;

endmodule
