//=============================================================================
// ov5640_ddr3_tft.v — 任务一 TFT 版：OV5640 采集 → DDR3 帧缓存 → 800x480 RGB 屏
//-----------------------------------------------------------------------------
// 基线：官方第 47 章 ov5640_ddr3_hdmi（摄像头 + DDR3 帧缓存链路保持不变）
// 改动：
//   1) 显示端由 HDMI(TMDS) 换成 J1 上的 800x480 RGB565 并口屏（第 28 章已验证）
//   2) 显示像素时钟 33MHz（tft_pll，参数取自第 28 章例程）
//   3) 摄像头输出分辨率 1280x720 → 800x480（初始化表 0x3808-0x380b 本就按
//      IMAGE_WIDTH/IMAGE_HEIGHT 参数生成，改参数即可）
//   4) disp_parameter_cfg.v 切换为 HW_TFT50 + MODE_RGB565 + Resolution_800x480
// 引脚：TFT_* 与官方《ACG720-138&60K通用管脚分配表》TFT 段逐针一致
//=============================================================================
`timescale 1ns/1ps

module gao_ov5640_ddr_tft(
    //System clock reset
    input       clk50M        , //系统时钟输入，50MHz
    input       reset_n       , //复位信号输入（板上 S4 / B21；原 F15 已让给任务二 key[0]）

    //camera interface
    output      camera_sclk   ,
    inout       camera_sdat   ,
    input       camera_vsync  ,
    input       camera_href   ,
    input       camera_pclk   ,
    output      camera_xclk   ,
    input  [7:0]camera_data   ,
    output      camera_rst_n  ,
    output      camera_pwdn   ,
    //led
    output [3:0]led           ,

    //==== 任务二「运动控制」接口（2026-10-12 整合）====
    //  引脚全部沿用组长已验证的 ACG720_task2_ir.cst（车上的线就是按它接的，零改线）
    input  [3:0] key          , // 板载按键 S0~S3：F15/A20/B20/A21（按下为低）
    input        ir_in        , // 红外接收头输出 F21
    input  [3:0] enc_a        , // 四路编码器 A 相：D19/C14/C15/B15
    input  [3:0] enc_b        , // 四路编码器 B 相：B16/D17/C17/E16
    output [3:0] m_in1        , // D24A 方向脚 xIN1：D16/A14/A15/A16
    output [3:0] m_in2        , // D24A 方向脚 xIN2：B17/A19/B18/F14
    output [3:0] m_pwm        , // D24A 调速 PWM：E13/C18/C13/B13
    output       stdby        , // D24A 使能（低=待机；急停通路）C19
    output       uart_tx      , // 串口 TX（预留，当前恒为高）M15
    //==== 任务四 PB：蓝牙人机通道（2026-10-12）====
    //  P8 针1 USER_0(球号 H14) = bt_rx【FPGA 输入】← 蓝牙模块 TXD
    //  P8 针2 USER_2(球号 H20) = bt_tx【FPGA 输出】→ 蓝牙模块 RXD
    //  ★ 独立通道：与 M15→CH340E→COM13 的 115200 诊断**互不相干、两条并存**
    //  ★ 模块供电走 P8 的 VCC3V3（3.3V），绝不能接 5V
    input        bt_rx        ,
    output       bt_tx        ,
    //==== 任务四 P4：超声波（2026-10-17 引脚定案）====
    //  P7 第 34 针 = 球号 C20 = ultra_trig【FPGA 输出】→ 模块 TRIG（3.3V 脉冲，模块兼容）
    //  P7 第 35 针 = 球号 E19 = ultra_echo【FPGA 输入】← 模块 ECHO
    //  ⚠️ ECHO 是 5V 电平 ⇒ 必须经电平转换模块降到 3.3V 再进 FPGA
    output       ultra_trig    ,
    input        ultra_echo    ,
    //==== 任务四 P3.5：电池电压监测（板载 ADC128S102，2026-10-14 新增）====
    //  原理图第 15 页 15_ADDA.SchDoc：U43 = ADC128S102CIMT（8 通道 12bit SPI，Vref=3.3V）
    //  8 个模拟输入经 22R+1nF 滤波接到 P5（Header 8X2）：
    //     P5 第 1 脚 = Analog_In0 … 第 15 脚 = Analog_In7；左列(2/4/6/…/16) 全 AGND
    //  ⇒ 电池分压中点 → P5 第 1 脚；分压地 → P5 第 2 脚（与 FPGA 共地）
    //  ⚠️ 板上 12V(VCC12/J4) 没有现成分压到 ADC ⇒ 必须外部接两个电阻
    output       adc_cs_n     , // → U43.1  (球号 M6)
    output       adc_din      , // → U43.14 (球号 M5)  通道控制字
    input        adc_dout     , // ← U43.15 (球号 P6)  转换结果
    output       adc_sclk     , // → U43.16 (球号 N5)
    //I2C -> MPU6050（走 P7 空闲 GPIO0 脚；VCCIO=3.3V 档已实测）
    output       IIC_SCL      , // -> P7 第31脚 (GPIO0_26, 球号 F16)
    inout        IIC_SDA      , // -> P7 第32脚 (GPIO0_27, 球号 E17)，顶层做开漏三态

    //TFT (J1) RGB565 parallel output
    output [15:0] TFT_rgb     ,
    output        TFT_clk     ,
    output        TFT_de      ,
    output        TFT_hs      ,
    output        TFT_vs      ,
    output        TFT_pwm     , //背光，高电平点亮

    //ddr
    output [13:0] O_ddr_addr  ,
    output [2:0]  O_ddr_ba    ,
    output        O_ddr_cs_n  ,
    output        O_ddr_ras_n ,
    output        O_ddr_cas_n ,
    output        O_ddr_we_n  ,
    output        O_ddr_clk   ,
    output        O_ddr_clk_n ,
    output        O_ddr_cke   ,
    output        O_ddr_odt   ,
    output        O_ddr_reset_n,
    output [1:0]  O_ddr_dqm   ,
    inout [15:0]  IO_ddr_dq   ,
    inout [1:0]   IO_ddr_dqs  ,
    inout [1:0]   IO_ddr_dqs_n
);

    //采集与显示同为 800x480（TFT 原生分辨率）
    parameter IMAGE_WIDTH  = 800;
    parameter IMAGE_HEIGHT = 480;

    // ========================================================================
    // 【任务五/六 视觉任务 · 2026-10-29 从 total_project/task123456_v1 整合】
    //   来源：组长整机工程，原生文件 src/ov5640_ddr3_tft.v（1402 行，MD5 bb074a05…）
    //   说明：声明区从原文件的第 96 行位置整体前移到此处（紧跟参数区），
    //         原因见记忆规范「信号声明必须在首次使用前」，且 Gowin 对隐式网
    //         特别敏感。声明提前不会改变任何逻辑，只是为了规避隐式网络警告。
    //   回滚：_备份_整合前_20261029/ov5640_ddr3_tft.v（整合前 1258 行原版）
    // ========================================================================
    wire vision_start,vision_active,vision_drive,vision_selected_unused;
    wire [3:0] vision_state,vision_reason;
    wire signed [31:0] vision_vx,vision_w;
    reg vision_owner;
    wire vision_nav_vld;
    wire [7:0] vision_nav_cmd;
    wire [2:0] combined_state;

    wire          loc_clk24m;
    wire          g_rst_p;
    wire          clk_200M;
    wire          pll_lock;
    wire          loc_clk50m;
    wire          clk_tft;        //33MHz TFT 像素时钟

    //camera
    wire          camera_init_done;
    wire          pclk_bufg_o;

    //ddr
    wire          fifo_aclr;
    wire          fifo_wrreq;
    wire [15:0]   fifo_wrdata;
    wire          init_calib_complete;
    wire          wr_load;

    //display
    wire [11:0]   hcount;
    wire [11:0]   vcount;
    wire [15:0]   disp_data;      //RGB565
    wire          video_hs;
    wire          video_vs;
    wire          video_de;
    wire [4:0]    video_r;
    wire [5:0]    video_g;
    wire [4:0]    video_b;

    // ⚠️ 2026-10-12 整合：led[3:0] 改由**任务二（运动控制）**驱动
    //   （LED0 心跳 / LED1 运行中 / LED2 闭环锁定 / LED3 故障报警）——
    //   现场演示时比摄像头状态更有用，且组长那份代码就是这么定义的。
    //   ↓ 下面两行是原来的"摄像头状态灯"，如需改回：恢复这两行，
    //     并把文件末尾 u_task2 的 .led(led) 改成 .led()（不连接）。
    // assign g_rst_p = ~pll_lock;
    // assign led = {~g_rst_p, camera_init_done, init_calib_complete, pll_lock};

    //---- 33MHz TFT 像素时钟（参数来自第28章例程：MDIV=33, ODIV0=50 @50MHz） ----
    tft_pll u_tft_pll(
        .lock   (          ),
        .clkout0(clk_tft   ),
        .clkin  (clk50M    ),
        .reset  (~reset_n  )
    );

    //---- 摄像头时钟：clkout1 = 24MHz XCLK（clkout0 原供 375MHz TMDS，已不用） ----
    camera_pll camera_pll(
        .clkout0(          ),
        .clkout1(loc_clk24m),
        .clkin  (loc_clk50m)
    );

    Gowin_PLL Gowin_PLL(
        .lock   (pll_lock  ),
        .clkout0(clk_200M  ),
        .clkout1(loc_clk50m),
        .clkin  (clk50M    ),
        .reset  (~reset_n  )
    );

    assign camera_xclk = loc_clk24m;

    //---- 摄像头寄存器配置（分辨率随参数变化） ----
    camera_init
    #(
        .SYS_CLOCK      ( 50_000_000   ),
        .SCL_CLOCK      ( 400_000      ),
        .CAMERA_TYPE    ( "ov5640"     ),
        .IMAGE_TYPE     ( 0            ),
        .IMAGE_WIDTH    ( IMAGE_WIDTH  ),
        .IMAGE_HEIGHT   ( IMAGE_HEIGHT ),
        .IMAGE_FLIP_EN  ( 0            ),
        .IMAGE_MIRROR_EN( 0            )
    )camera_init
    (
        .Clk         (loc_clk50m       ),
        .Rst_n       (reset_n          ),
        .Init_Done   (camera_init_done ),
        .camera_rst_n(camera_rst_n     ),
        .camera_pwdn (camera_pwdn      ),
        .i2c_sclk    (camera_sclk      ),
        .i2c_sdat    (camera_sdat      )
    );

    // ---- camera_pclk 直连（2026-10-10 说明）----
    //   曾尝试实例化 Gowin 原语 BUFG 以消除 PR1014，但综合器报
    //   ERROR (EX3937) Instantiating unknown module 'BUFG'
    //   —— Gowin 的时钟缓冲是【自动推断】的，不允许用户例化。
    //   维持直连（与小梅哥厂商例程 seed 工程一致）；PD1014 属警告，
    //   且 camera_pclk 域时序实测 0 违例，故接受。
    assign pclk_bufg_o = camera_pclk;

    //---- DVP 采集（前 10 帧丢弃，RGB565 16bit） ----
    /* ★2026-10-29 整合修正：vision_raw_pixel / vision_cam_x / vision_cam_y
       原组长文件把它们声明在 DVP_Capture 例化【之后】，综合时触发
       "Undeclared symbol ... assumed default net type 'wire'" + EX3638 已隐式声明。
       本项目规范要求"声明必须在首次使用前"，故把三根线提到例化之前。 */
    wire [15:0] vision_raw_pixel;
    wire [10:0] vision_cam_x,vision_cam_y;

    DVP_Capture DVP_Capture(
        .Rst_n      (reset_n         ),
        .PCLK       (pclk_bufg_o     ),
        .Vsync      (camera_vsync    ),
        .Href       (camera_href     ),
        .Data       (camera_data     ),
        .ImageState (fifo_aclr       ),
        .DataValid  (fifo_wrreq      ),
        .DataPixel  (vision_raw_pixel),
        .DataHs     (                ),
        .DataVs     (wr_load         ),
        .Xaddr      (vision_cam_x    ),
        .Yaddr      (vision_cam_y    )
    );

    // ========================================================================
    // 【任务五 视觉前端 · 2026-10-29 整合】来源：组长工程 src/ov5640_ddr3_tft.v
    //   TASK56 V1: teammate front/blob plus new detector wrapper and mission RTL.
    //   No registers in the DDR write path. Raw pixel and DataValid stay aligned.
    //   ★ 关键：DataPixel 走 vision_raw_pixel 直连 DDR 写通路，不插寄存器；
    //     vision_front 只做「显示通路」的旁路叠加，不挡写通路。
    //   （三根输入线的声明已上移到 DVP_Capture 例化之前，见上）
    // ========================================================================
    // DVP_Capture exports one-based coordinates on its valid pixel cycles.
    wire [10:0] vision_x=vision_cam_x==0 ? 11'd0:vision_cam_x-11'd1;
    wire [10:0] vision_y=vision_cam_y==0 ? 11'd0:vision_cam_y-11'd1;
    wire vision_red,vision_box,vision_frame;
    wire [10:0] vision_xmin,vision_xmax,vision_ymin,vision_ymax;
    wire [19:0] vision_count;
    // Display uses native one-based live coordinates. Convert the slowly
    // changing bbox ONCE into registers, rather than subtracting from every
    // live pixel coordinate on its critical path to the write FIFO.
    reg [11:0] draw_xmin,draw_xmax,draw_ymin,draw_ymax;
    reg draw_valid;
    always @(posedge pclk_bufg_o or negedge reset_n)
      if(!reset_n)begin
        draw_xmin<=0;draw_xmax<=0;draw_ymin<=0;draw_ymax<=0;draw_valid<=0;
      end else begin
        draw_xmin<={1'b0,vision_xmin}+12'd1;draw_xmax<={1'b0,vision_xmax}+12'd1;
        draw_ymin<={1'b0,vision_ymin}+12'd1;draw_ymax<={1'b0,vision_ymax}+12'd1;
        draw_valid<=vision_box;
      end
    //====================================================================
    // 【调试用 · 闪示诊断】2026-10-25 加（组长工程）
    //   约每 1 秒在两种画面之间切一次，用来一眼看清"判据到底命中了什么"：
    //     ① 彩色原图 + 白框（SHOW_MODE=4 的正常输出）
    //     ② 目标色二值图（白 = 判为目标色，黑 = 不是）
    //   判读：
    //     · ②里目标物是白的、手/背景是黑的，但 ①没有框 ⇒ 是探测器闸门问题
    //     · ②里手/背景也变白 ⇒ 判据太松，要收紧（vision_front 的 r_thr 系数）
    //   ★ 演示前把 DIAG_BLINK 改成 0 并重编即可关掉。
    //====================================================================
    localparam integer DIAG_BLINK = 0;   // 0 = 关闭闪示诊断（正式演示版：屏幕常显彩色画面）

    wire [15:0] vision_disp_raw;
    reg  [27:0] diag_cnt;
    reg         diag_sel;

    always @(posedge pclk_bufg_o or negedge reset_n)
      if(!reset_n)                          begin diag_cnt<=28'd0; diag_sel<=1'b0; end
      else if(DIAG_BLINK==0)                begin diag_cnt<=28'd0; diag_sel<=1'b0; end
      else if(diag_cnt==28'd173_999_999)    begin diag_cnt<=28'd0; diag_sel<=~diag_sel; end  // 87 MHz ≈ 2 s/相
      else                                  begin diag_cnt<=diag_cnt+28'd1; end

    assign fifo_wrdata = ((DIAG_BLINK!=0) && diag_sel)
                       ? (vision_red ? 16'hFFFF : 16'h0000)   // 二值图
                       : vision_disp_raw;                     // 彩色 + 框

    // 2026-10-26：按用户要求**去掉白框**。SHOW_MODE=0 = 原图纯透传（不画框）。
    //   现在屏幕只剩两种相位（由下面的 DIAG_BLINK 交替）：
    //     ① 原始彩色画面
    //     ② 目标色二值图（白 = 判为红色目标，黑 = 其余）
    /* 显示模式：0 = 纯彩色原图，无任何叠加（演示定稿的样子）。
       检测是否成立的判断不靠画面叠加，改由「合成速度」那个诊断数 + 网页状态标签给出。 */
    vision_front #(.SHOW_MODE(0),.BOX_T(4)) u_vision_front(
      .clk(pclk_bufg_o),.rst_n(reset_n),.px_valid(fifo_wrreq),
      .px(vision_raw_pixel),.px_x(vision_cam_x),.px_y(vision_cam_y),
      .tgt_xmin(draw_xmin),.tgt_xmax(draw_xmax),
      .tgt_ymin(draw_ymin),.tgt_ymax(draw_ymax),
      .tgt_valid(draw_valid),.red_now(vision_red),.disp_px(vision_disp_raw),.gray_now());
    vision_detector #(.WIDTH(IMAGE_WIDTH),.HEIGHT(IMAGE_HEIGHT)) u_vision_detector(
      .clk(pclk_bufg_o),.rst_n(reset_n),.frame_active(wr_load),
      .pixel_valid(fifo_wrreq),.red_now(vision_red),.x(vision_x),.y(vision_y),
      .frame_strobe(vision_frame),.box_valid(vision_box),
      .xmin(vision_xmin),.xmax(vision_xmax),.ymin(vision_ymin),.ymax(vision_ymax),
      .red_count(vision_count));
    wire [64:0] vision_snapshot;
    wire vision_snapshot_strobe;
    task123_snapshot_cdc #(.WIDTH(65)) u_vision_cdc(
      .src_clk(pclk_bufg_o),.dst_clk(loc_clk50m),.rst_n(reset_n),
      .src_strobe(vision_frame),
      .src_data({vision_box,vision_xmin,vision_xmax,vision_ymin,vision_ymax,vision_count}),
      .dst_data(vision_snapshot),.dst_strobe(vision_snapshot_strobe),.fresh());


    wire          rdfifo_rden;
    wire          wrfifo_clr;
    wire          wrfifo_wren;
    wire [15:0]   wrfifo_din;
    wire          wrfifo_clk;
    wire          rdfifo_clr;
    wire          rdfifo_clk;
    wire [15:0]   rdfifo_dout;

    assign wrfifo_clr  = fifo_aclr;
    assign wrfifo_clk  = pclk_bufg_o;
    assign wrfifo_wren = fifo_wrreq;
    assign wrfifo_din  = fifo_wrdata;
    assign rdfifo_clr  = 1'b0;
    assign rdfifo_clk  = clk_tft;
    assign disp_data   = rdfifo_dout;      //RGB565 直接送显示（disp_driver 按 Red_Bits/Green_Bits/Blue_Bits 切分）

    wire [27:0] app_addr_max = IMAGE_WIDTH * IMAGE_HEIGHT;
    wire [7:0]  burst_len    = IMAGE_WIDTH[10:3];

    ddr3_ctrl_2port ddr3_ctrl_2port(
        .clk(loc_clk50m)                 ,
        .pll_lock(pll_lock)              ,
        .clk_200m(clk_200M)              ,
        .sys_rst_n(reset_n)              ,
        .init_calib_complete(init_calib_complete) ,

        .rd_load(video_vs)               ,
        .wr_load(wr_load)                ,
        .app_addr_rd_min(28'd0)          ,
        .app_addr_rd_max(app_addr_max)   ,
        .rd_bust_len(burst_len)          ,
        .app_addr_wr_min(28'd0)          ,
        .app_addr_wr_max(app_addr_max)   ,
        .wr_bust_len(burst_len)          ,

        .wr_clk(wrfifo_clk)              ,
        .wfifo_wren(wrfifo_wren)         ,
        .wfifo_din(wrfifo_din)           ,
        .rd_clk(rdfifo_clk)              ,
        .rfifo_rden(rdfifo_rden)         ,
        .rfifo_dout(rdfifo_dout)         ,

        .ddr3_dq(IO_ddr_dq)              ,
        .ddr3_dqs_n(IO_ddr_dqs_n)        ,
        .ddr3_dqs_p(IO_ddr_dqs)          ,
        .ddr3_addr(O_ddr_addr)           ,
        .ddr3_ba(O_ddr_ba)               ,
        .ddr3_ras_n(O_ddr_ras_n)         ,
        .ddr3_cas_n(O_ddr_cas_n)         ,
        .ddr3_we_n(O_ddr_we_n)           ,
        .ddr3_reset_n(O_ddr_reset_n)     ,
        .ddr3_ck_p(O_ddr_clk)            ,
        .ddr3_ck_n(O_ddr_clk_n)          ,
        .ddr3_cke(O_ddr_cke)             ,
        .ddr3_cs_n(O_ddr_cs_n)           ,
        .ddr3_dm(O_ddr_dqm)              ,
        .ddr3_odt(O_ddr_odt)
    );

    //---- 800x480 RGB565 时序发生器（disp_driver，配置见 disp_parameter_cfg.v） ----
    disp_driver disp_driver(
        .ClkDisp(clk_tft),
        .Rst_n(reset_n),
        .Data(disp_data),
        .DataReq(rdfifo_rden),
        .H_Addr(hcount),
        .V_Addr(vcount),
        .Disp_HS(video_hs),
        .Disp_VS(video_vs),
        .Disp_Red(video_r),
        .Disp_Green(video_g),
        .Disp_Blue(video_b),
        .Frame_Begin(),
        .Disp_DE(video_de),
        .Disp_PCLK(TFT_clk)
    );

    //====================================================================
    // OSD 字符叠加（T10）
    //   位置：disp_driver 输出之后、TFT 并口之前（教材第31章要求的位置）
    //   链路：cam_rate_meter(50MHz 域实测 fps) -> osd_num_writer(跨域+十进制展开)
    //         -> osd_overlay(字库查表 + 像素叠加)
    //   参数化：位置(OSD_BOX_X/Y、FPS_ROW/COL0)与配色(OSD_INK/BG)全部在下面改，
    //           上板调观感只动参数，不动设计。
    //====================================================================
    localparam integer OSD_BOX_X = 8;
    localparam integer OSD_BOX_Y = 8;
    localparam integer OSD_NROW  = 4;
    localparam integer OSD_NCOL  = 40;
    localparam [15:0]  OSD_INK   = 16'h07E0;   // 字符色：RGB565 绿
    localparam [15:0]  OSD_BG    = 16'h0000;   // 窗口底色：黑（保证可读）
    localparam [4:0]   FPS_ROW   = 5'd1;       // fps 写在第 1 行（**位置不动**，保 M3 基线）
    localparam [5:0]   FPS_COL0  = 6'd0;
    localparam integer FPS_NDIG  = 3;          // 定宽 3 位
    localparam integer FPS_LEAD0 = 1;          // 1=前导零 "FPS=030"; 0=前导空格

    // ---- T14：姿态角两行（行 0=PCLK、行 1=FPS 均**保持不动**，姿态占行 2、3）----
    localparam [4:0]   ROLL_ROW   = 5'd2;      // 第 3 行
    localparam [5:0]   ROLL_COL0  = 6'd0;
    localparam [4:0]   PITCH_ROW  = 5'd3;      // 第 4 行
    localparam [5:0]   PITCH_COL0 = 6'd0;
    localparam [47:0]  ROLL_LBL   = 48'h524F4C4C3D00;   // "ROLL="  左对齐 48 位
    localparam [47:0]  PITCH_LBL  = 48'h50495443483D;   // "PITCH=" 左对齐 48 位

    // PCLK 行（M2）：pclk_cnt / 1e6 = MHz，写第 0 行（fps 仍在第 1 行）
    //   LBL 只有 4 字符（工程里是旧版 32 位 LBL 的 writer），放不下 "PCLK="+单位，
    //   所以 **屏上"PCLK 后面那个数就是 MHz"**，与 pclk_row_src.v 的注释一致。
    localparam [4:0]   PCLK_ROW   = 5'd0;
    localparam [5:0]   PCLK_COL0  = 6'd0;
    localparam integer PCLK_LEAD0 = 0;         // 0 = 前导空格 ⇒ "PCLK 24"（24 MHz）

    // 叠加总开关（演示兜底：OSD 出问题可一键关掉，画面恢复纯透传）
    wire osd_en = 1'b1;

    //---- 帧率实测（50MHz 域）----
    wire [31:0] cam_frame_cnt;
    wire [31:0] cam_pclk_cnt;
    wire        cam_rate_valid;
    cam_rate_meter u_cam_rate (
        .clk      (loc_clk50m   ),
        .rst_n    (reset_n      ),
        .pclk     (pclk_bufg_o  ),
        .vsync    (camera_vsync ),
        .frame_cnt(cam_frame_cnt),
        .pclk_cnt (cam_pclk_cnt ),
        .valid    (cam_rate_valid)
    );

    //---- fps -> 字符缓冲（跨 50M -> 33M）----
    wire        fps_we, fps_busy;
    wire [4:0]  fps_wrow;
    wire [5:0]  fps_wcol;
    wire [7:0]  fps_wcode;

    //====================================================================
    // T14：IMU 姿态链路（MPU6050 -> 定点互补滤波 -> OSD 第 3/4 行）
    //   时钟域：I2C 与解算都在 loc_clk50m（50MHz）；OSD 写口在 clk_tft（33MHz）
    //====================================================================
    wire        imu_sda_o, imu_sda_oe;
    wire        imu_scl_od;      // SCL 开漏中间信号：1=释放(高)，0=拉低
    wire        IIC_SDA_in;
    wire signed [15:0] acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, temp_raw;
    // ==== IMU 轴向映射（2026-10-10 上板实测标定）====
    //   安装：模块平放(芯片面朝上)、排针在车右侧、车头=INT端、车尾=VCC端。
    //   实测：抬车头 -> 只有 acc_y 变；抬右侧 -> 只有 acc_x 变
    //         => 模块 X 轴对着车的左右方向、Y 轴对着车的前后方向（绕竖直轴差 90 度）。
    //   处理：先把模块轴系绕 Z 轴转 90 度对齐到车轴系，再送姿态解算：
    //         att_ax = +acc_y ;  att_ay = -acc_x ;  att_az = +acc_z
    //   ★ 陀螺必须同步映射（角速度与加速度在同一坐标变换下分量一致），
    //     否则互补滤波的陀螺分支与加速度分支错配，角度会漂。
    //   ★ 标定依据：exp21 诊断版上板读数（平放 -0.0/0.1；抬头 (0,-0.2)；右抬 (-0.2,0)）。
    wire signed [15:0] att_ax =  acc_y;
    wire signed [15:0] att_ay = -acc_x;
    wire signed [15:0] att_az =  acc_z;
    wire signed [15:0] att_gx =  gyro_y;
    wire signed [15:0] att_gy = -gyro_x;
    wire signed [15:0] att_gz =  gyro_z;
    wire        imu_data_valid;
    wire        imu_ack_err;

    assign IIC_SDA    = (imu_sda_oe && !imu_sda_o) ? 1'b0 : 1'bz;   // 开漏：只拉低
    assign IIC_SDA_in = IIC_SDA;
    assign IIC_SCL    = imu_scl_od ? 1'bz : 1'b0;   // SCL 改真开漏（合规 I2C，兼容从机时钟延展）

    mpu6050_ctrl #(
        .SYS_CLK_HZ(50_000_000), .SCL_HZ(100_000),   // 降速：400k -> 100k（提高信号容错）
        .PWRUP_MS(100), .PERIOD_MS(10),      // 100 Hz 采样
        .DEV_ADDR(7'h68)
    ) u_mpu6050 (
        .clk        (loc_clk50m),
        .rst_n      (reset_n),
        .i2c_scl    (imu_scl_od),   // 不直连引脚，走下面开漏三态
        .i2c_sda_in (IIC_SDA_in),
        .i2c_sda_o  (imu_sda_o),
        .i2c_sda_oe (imu_sda_oe),
        .accel_x(acc_x), .accel_y(acc_y), .accel_z(acc_z),
        .temp_raw(temp_raw),
        .gyro_x(gyro_x), .gyro_y(gyro_y), .gyro_z(gyro_z),
        .data_valid(imu_data_valid),
        .ack_err   (imu_ack_err),
        .dbg_state(), .dbg_sub()
    );

    wire signed [31:0] roll_q16, pitch_q16, yaw_q16;
    wire signed [15:0] roll_x100, pitch_x100, yaw_x100;

    imu_attitude #(
        .ALPHA_SHIFT(5), .GYRO_Q(4711), .DT_DIV(100),   // dt=10ms -> tau≈0.32s
                                                        // ★2026-10-22 二次微调 4668→4711（+0.93%）：
                                                        //   用"1080=三整圈应回到原方向"测得实际多转 ~10°
                                                        // ★2026-10-22 实测标定 3996→4668（×1.168）：
                                                        //   6 组数据显示/实际 ≈1.168 恒定（此前改模块默认值
                                                        //   被此处显式参数覆盖 ⇒ 标定从未生效——已修正）
        .GYRO_Z_BIAS(0)                                 // yaw 零偏待静置标定
    ) u_attitude (
        .clk          (loc_clk50m),
        .rst_n        (reset_n),
        .sample_valid (imu_data_valid),
        .acc_x(att_ax), .acc_y(att_ay), .acc_z(att_az),   // 经轴向映射
        .gyro_x(att_gx), .gyro_y(att_gy), .gyro_z(att_gz), // 经轴向映射
        .roll_q16(roll_q16), .pitch_q16(pitch_q16), .yaw_q16(yaw_q16),
        .roll_x100(roll_x100), .pitch_x100(pitch_x100), .yaw_x100(yaw_x100),
        .busy()
    );


    // ==== 开机零偏标定（2026-10-10 加）====
    //   上电后约 2 秒（此时车应静止平放）采样一次 roll/pitch 作为"零点"，
    //   之后显示的角度都减去它 —— 用于消掉模块安装倾斜带来的**固定零偏**。
    //   实测零偏：ROLL 约 +2.0 度、PITCH 约 -6.6 度（模块背面焊点+垫片导致装不平）。
    //   ★ 只消"安装零偏"；"换个地方数字就变"是场地真实倾斜（桌面本身斜），属正常测量，不消。
    //   ★ 上电时请保持车静止平放；否则零点会标偏（重新上电即可重标）。
    reg  [26:0]        calib_cnt;
    reg                calib_done;
    reg  signed [15:0] zero_roll, zero_pitch;

    always @(posedge loc_clk50m or negedge reset_n) begin
        if (!reset_n) begin
            calib_cnt   <= 27'd0;
            calib_done  <= 1'b0;
            zero_roll   <= 16'sd0;
            zero_pitch  <= 16'sd0;
        end else if (!calib_done) begin
            if (calib_cnt == 27'd99_999_999) begin   // 100M / 50MHz = 2 秒
                calib_done  <= 1'b1;
                zero_roll   <= roll_x100;
                zero_pitch  <= pitch_x100;
            end else begin
                calib_cnt <= calib_cnt + 27'd1;
            end
        end
    end

    wire        imu_we;
    wire [4:0]  imu_wrow;
    wire [5:0]  imu_wcol;
    wire [7:0]  imu_wcode;
    wire        imu_rbusy, imu_pbusy;

    imu_display #(
        .NCOL(OSD_NCOL),
        .ROLL_ROW(ROLL_ROW),   .ROLL_COL0(ROLL_COL0),
        .ROLL_LBL(ROLL_LBL),   .ROLL_LBL_LEN(5),
        .PITCH_ROW(PITCH_ROW), .PITCH_COL0(PITCH_COL0),
        .PITCH_LBL(PITCH_LBL), .PITCH_LBL_LEN(6),
        .ANG_LIMIT(9000)                        // ±90.00 度钳位
    ) u_imu_disp (
        .clk_src (loc_clk50m), .rst_n(reset_n),
        // 实测标定(2026-10-10)：抬右侧 -> ROLL 增大(方向正确)；抬车头 -> PITCH 减小(符号相反)
        //   => 仅对 PITCH 取反，ROLL 不动。只翻显示方向，不改融合模块内部状态(保留其仿真验证)。
        .roll_x100(roll_x100  - zero_roll),          // 减安装零偏
        .pitch_x100(-(pitch_x100 - zero_pitch)),    // 先减零偏、再翻方向
        .nd_src  (imu_data_valid),
        .pclk    (clk_tft),
        .we(imu_we), .wrow(imu_wrow), .wcol(imu_wcol), .wcode(imu_wcode),
        .roll_busy(imu_rbusy), .pitch_busy(imu_pbusy)
    );

    //====================================================================
    // 写口仲裁：osd_overlay 只有一个写口，三源分时（stall/busy 链）
    //   优先级 imu(100Hz,21字符) > fps(1Hz,7字符) > pclk(1Hz,7字符)
    //   性质：writer 拿到 nd 先把整串锁进 str[]，stall 只推迟写出、不丢内容；
    //        被压住的一方在其 stall=1 期间绝不置 we ⇒ 不会同拍冲突。
    //====================================================================
    wire osd_imu_busy     = imu_rbusy | imu_pbusy;
    wire osd_imu_fps_busy = osd_imu_busy | fps_busy;

    wire        osd_we_m    = imu_we  | fps_we    | osd_pwe;
    wire [4:0]  osd_wrow_m  = imu_we ? imu_wrow  : (fps_we ? fps_wrow  : osd_pwrow);
    wire [5:0]  osd_wcol_m  = imu_we ? imu_wcol  : (fps_we ? fps_wcol  : osd_pwcol);
    wire [7:0]  osd_wcode_m = imu_we ? imu_wcode : (fps_we ? fps_wcode : osd_pwcode);

    wire [15:0] osd_fps_dbg;
    wire        osd_nd_dbg;

    osd_num_writer #(
        .MODE(0), .NDIG(FPS_NDIG), .LEAD_ZERO(FPS_LEAD0),
        .LBL(48'h4650533D0000), .LBL_LEN(4),   // "FPS=" 左对齐 48 位
        .ROW(FPS_ROW), .COL0(FPS_COL0),
        .NCOL(OSD_NCOL), .VAL_MAX(999)
    ) u_osd_fps (
        .clk_src(loc_clk50m),
        .val_src(cam_frame_cnt[15:0]),
        .nd_src (cam_rate_valid),
        .pclk   (clk_tft),
        .rst_n  (reset_n),
        .stall  (osd_imu_busy),
        .we     (fps_we),
        .wrow   (fps_wrow),
        .wcol   (fps_wcol),
        .wcode  (fps_wcode),
        .busy   (fps_busy),
        .val_dbg(osd_fps_dbg),
        .nd_dbg (osd_nd_dbg)
    );

    //---- PCLK 频率 -> 数值源（50MHz 域；内含 /1e6 除法 + 写口让路）----
    //   屏上格式：LBL="PCLK" + 3 位十进制 MHz，前导空格 ⇒ "PCLK 24"
    wire [15:0] osd_pclk_val;
    wire        osd_pclk_nd;
    pclk_row_src #(.DIVISOR(1_000_000), .DEFER_CYC(24), .VAL_MAX(999)) u_pclk_src (
        .clk_src (loc_clk50m),
        .rst_n   (reset_n),
        .pclk_cnt(cam_pclk_cnt),
        .trig    (cam_rate_valid),     // 与 fps 同一个测量完成脉冲
        .fps_nd_b(osd_nd_dbg),         // 观测 fps writer 的 nd（pclk 域）⇒ 已写完就让路结束
        .val_src (osd_pclk_val),
        .nd_src  (osd_pclk_nd),
        .val_dbg (),
        .busy_dbg()
    );

    wire        osd_pwe, pclk_busy;
    wire [4:0]  osd_pwrow;
    wire [5:0]  osd_pwcol;
    wire [7:0]  osd_pwcode;

    osd_num_writer #(
        .MODE(0), .NDIG(FPS_NDIG), .LEAD_ZERO(PCLK_LEAD0),
        .LBL(48'h50434C4B0000), .LBL_LEN(4),   // "PCLK" 左对齐 48 位
        .ROW(PCLK_ROW), .COL0(PCLK_COL0),
        .NCOL(OSD_NCOL), .VAL_MAX(999)
    ) u_osd_pclk (
        .clk_src(loc_clk50m),
        .val_src(osd_pclk_val),
        .nd_src (osd_pclk_nd),
        .pclk   (clk_tft),
        .rst_n  (reset_n),
        .stall  (osd_imu_fps_busy),
        .we     (osd_pwe),
        .busy   (pclk_busy),
        .wrow   (osd_pwrow),
        .wcol   (osd_pwcol),
        .wcode  (osd_pwcode),
        .val_dbg(        ),
        .nd_dbg (        )
    );

    // （原二源仲裁块已删除：三源 mux 已提到 fps writer 之前，见上）

    //---- 叠加：RGB565 打包 -> osd_overlay -> TFT 并口 ----
    wire [15:0] v_rgb565 = {video_b, video_g, video_r};
    wire [15:0] osd_rgb;

    //====================================================================
    // 【实验 12】纯色轮播选择器 —— 任务一 DoD 第 7 项「颜色位序」
    //   位置：disp_driver 输出之后、osd_overlay 之前（**只动显示端**）
    //   目的：投 5 个已知纯色到屏上，判定 R/B 位序。
    //     本工程打包 = {video_b, video_g, video_r}（官方管脚表）
    //       ⇒ 纯红 = 16'h001F、纯蓝 = 16'hF800（与常见 RGB565 相反）
    //     屏上第 1 色应显【红】、第 3 色应显【蓝】；
    //     若第 1 色显蓝 ⇒ R/B 倒了（改打包顺序即可）。
    //   ⚠️ COLOR_MODE = 0 时本模块退化为**纯组合透传**（计数器被折叠、零开销）
    //      —— 测试做完把下面这行改回 0 即可关掉，不需要删代码。
    //   ⚠️ 绝不改摄像头表：本实验"只动显示端"是能定性的前提。
    //--------------------------------------------------------------------
    // 【实验12 结论 · 2026-10-03】颜色位序判定：**R/B 未反**，实验关闭。
    //   证据等级（如实记账，**不可当"直接实测"引用**）：
    //     ① 实拍照片（10-03 12:16 / 12:44）中屏上深蓝色册子显示仍为蓝色 ——
    //        若 R/B 反，高 B 低 R 的蓝色物体应显示成红/橙色且整屏偏色，实际没有；
    //     ② 代码 {video_b, video_g, video_r} 与官方管脚表标注自洽
    //        （例程 .cst 那套相反的位序本工程未采用）。
    //     ⚠️ **"纯色序列逐帧核对"这一步没做**（用户回报的是实拍观感 + 代码自洽，
    //        非判据级验证）⇒ 报告措辞只能用"经实拍对照与代码自洽确认"，
    //        **不要写"经纯色序列逐帧验证"**。
    //   【另一半】图像方向/镜像 = ⏳ 待装车后定（台面测试测不出装车后的前后朝向），
    //     届时需改摄像头表 => 必须重新授权（动了守卫会 FAIL）。
    //--------------------------------------------------------------------
    localparam integer COLOR_MODE  = 0;             // 0=关掉(透传) ; 1=自动轮播(测试用)
    localparam integer CBAR_CLK_HZ = 33_000_000;    // clk_tft = 33 MHz
    localparam integer CBAR_SEC    = 1;             // 每色 1 秒
    wire [15:0] bar_rgb;

    color_bar #(
        .COLOR_MODE (COLOR_MODE ),
        .CLK_HZ     (CBAR_CLK_HZ),
        .ROLL_SEC   (CBAR_SEC   )
    ) u_color_bar (
        .pclk       (clk_tft    ),
        .rst_n      (reset_n    ),
        .de_in      (video_de   ),
        .rgb_in     (v_rgb565   ),
        .de_out     (           ),
        .rgb_out    (bar_rgb    ),
        .idx_dbg    (           ),
        .roll_active(           )
    );

    osd_overlay #(
        .H_ACTIVE(800), .V_ACTIVE(480),
        .CHAR_W(8), .CHAR_H(16),
        .NROW(OSD_NROW), .NCOL(OSD_NCOL),
        .BOX_X(OSD_BOX_X), .BOX_Y(OSD_BOX_Y),
        .INK(OSD_INK), .BGC(OSD_BG)
    ) u_osd (
        .pclk   (clk_tft),
        .rst_n  (reset_n),
        .de_in  (video_de),
        .x_in   (hcount[11:0]),
        .y_in   (vcount[11:0]),
        .rgb_in (bar_rgb),
        .de_out (        ),
        .rgb_out(osd_rgb),
        .we     (osd_we_m   ),
        .wrow   (osd_wrow_m ),
        .wcol   (osd_wcol_m ),
        .wcode  (osd_wcode_m),
        .disp_en(osd_en)
    );

    //---- TFT 并口输出 ----
    // 官方管脚表：TFT_rgb[15:11]=B 通道, [10:5]=G 通道, [4:0]=R 通道
    assign TFT_rgb = osd_rgb;
    assign TFT_de  = video_de;
    assign TFT_hs  = video_hs;
    assign TFT_vs  = video_vs;
    assign TFT_pwm = 1'b1;

    //==================================================================
    //  ★ 2026-10-29 v77：以下全部 wire 声明集中前置（原分散在 750~1290 行各实例之间）
    //    —— 原因：Gowin 对「先使用、后声明」会报 EX3638 并自动建隐式 1-bit 网。
    //    对 1-bit 信号功能无害，但对 [15:0]/[31:0]/数组 这类多位信号是**静默截断**。
    //    本次统一提到首次使用之前，消除 ALL 26 条 EX3638（组长原版有 23 条）。
    //    改这一块时请保持「声明全部在实例之前」的约定。
    //==================================================================
    // ---- 任务二/三（他们的 2:1 MUX 与声呐门控）----
    wire               nav_hold;
    wire [3:0]         t2_in1, t2_in2, t2_pwm;   // 任务二电机输出（原直连引脚）★必须 [3:0]
    wire               t2_stdby;
    wire [3:0]         t3_trig;
    wire               sonar_gated;
    wire               task3_sensor_enabled;
    wire signed [15:0] obs_front_mm;
    wire               obs_front_vld;
    //  ★注意：这两个必须是 **packed 向量**（4×32 位压成 128/64 位），
    //    不能写成 unpacked 数组 [0:3] —— Gowin 综合器报 EX3629 不一致维度 + EX3998 直接访问内存。
    wire [127:0] obs_positions;
    wire [63:0]  obs_velocities;
    wire               imu_snap_strobe;
    // ---- v74：避障透视线（u_nav -> u_bt_link -> bt_telem2 -> $DBG）----
    wire [2:0]         dbg_phase;
    wire [1:0]         dbg_sub;
    wire [2:0]         dbg_step;
    wire [1:0]         dbg_near;
    wire [1:0]         dbg_far;
    wire               dbg_unk;
    wire signed [15:0] dbg_samp;
    wire signed [31:0] dbg_odo;
    wire signed [31:0] dbg_w;
    wire [1:0]         dbg_gear;

    //==================================================================
    //  任务二 · 运动控制（组长交付，2026-10-12 整合）
    //  ------------------------------------------------------------------
    //  源文件：src/task2/selftest_ir.v（主程序：红外/状态机/PI 环）
    //          src/task2/selftest_top.v（公共子模块：key_deb/quad_dec/
    //          speed_meter/pid_pi/fake_motor/motor_out）
    //  ★ 组长那份代码**内部逻辑一字未改**，此处只做端口映射。
    //  ★ 时钟用 clk50M（Y18 的 50MHz 晶振）—— 与组长调参时用的时钟一致。
    //  ★ 复位：共用本顶层的 reset_n（B21 = S4）。按下 S4 会同时复位
    //     摄像头通路与运动控制，这是预期行为。
    //  ★ led[3:0] 交给任务二驱动（见上文说明）。
    //
    //  【任务三（避障）接入点 —— 给第三位队友】
    //    按组长交接报告 §4.2「方案 A」：在 selftest_ir.v 第十节运动学处，
    //    把算好的 vx_w 乘一个速度限速系数（Q8，256=100%/0=停）：
    //        wire signed [15:0] vx_cmd = (act_now == A_FWD) ? RUN_SPD : ...;
    //        assign vx_w = (vx_cmd * speed_scale) >>> 8;
    //    紧急停车请拉低 stdby（已验证最可靠：驱动模块整体高阻）。
    //    超声波引脚已预留：Trig=P7-34/35/37、Echo=P7-38/39/40
    //    （详见 mimo_material/任务二_合并_差异对照表.md §3.8）
    //==================================================================
    //  ★★★ 2026-10-19 反向合并：任务二实例（selftest_ir）换成他们的任务二三运动链
    //    （task123_task2_fix_board = route/backend/auto_core/PI/保护/声呐/遥控/遥测）
    //    他们的链不依赖他们的显示链/DDR3 ⇒ 与本工程的花屏隐患完全无关。
    //    board 版本带 nav_hold/obs_* 扩展（src/task3/task123_task2_fix_board.v）。
    //    IMU 用我们自己的 imu_attitude（上电自动标零 ⇒ 治好他们 yaw 漂移的根因）。
    task123_sonar_manual_gate u_sensor_gate(
        .clk       (clk50M    ),
        .rst_n     (reset_n   ),
        .enable_n  (key[1]    ),
        .disable_n (1'b1      ),
        .raw_trig  (t3_trig[0]),
        .gated_trig(sonar_gated),
        .enabled   (task3_sensor_enabled)
    );
    task123_task2_fix_board #(
        .MOTOR_ARMED       (1               ),
        .ROUTE_CALIBRATED  (1               ),
        .MAX_SESSION_CYCLES(32'd3000000000),
        .PHASE_TICKS       (1200            ),
        .TURN_COUNTS       (1200            ),
        .OFFSET_COUNTS     (2000            ),
        .PASS_COUNTS       (3200            )
    ) u_task123 (
        .sys_clk (clk50M),
        .rst_n   (reset_n),
        .ir_in   (ir_in  ),
        .key     (key    ),
        .enc_a   (enc_a  ),
        .enc_b   (enc_b  ),
        .imu_valid (imu_snap_strobe),   // ★经邮箱跨域后的样本脉冲
        .roll_q16  (imu_snap[31:0] ),
        .pitch_q16 (imu_snap[63:32]),
        .yaw_q16   (imu_snap[95:64]),
        .echo    ({2'b00, ultra_echo}),
        .trig    (t3_trig ),
        //  电机输出 → 内部 wire → 既有 2:1 MUX（与原任务二同路径）
        .m_in1   (t2_in1  ),
        .m_in2   (t2_in2  ),
        .m_pwm   (t2_pwm  ),
        .stdby   (t2_stdby),
        //  任务四接口
        .nav_hold        (nav_hold       ),

        .obs_front_mm    (obs_front_mm   ),
        .obs_front_vld   (obs_front_vld  ),
        .obs_positions   (obs_positions  ),
        .obs_velocities  (obs_velocities ),
        .led     (led     ),
        .uart_tx (uart_tx )
    );

    //  ★CDC（2026-10-17 补）：obs_velocities 在 clk50M 域，odom 在 loc_clk50m 域 ——
    //    多位总线无同步跨域会读到撕裂值（上一版 u_odom 23 条违例的根因）。
    //    复用他们的应答式邮箱 task123_snapshot_cdc：源 10ms 一拍快照，目的域拿到稳定值。
    //    源/目的同频（同一 50MHz 源）⇒ 相位差恒定 ⇒ 1:1 不丢窗。
    reg  [18:0] vel_str_cnt;
    reg         vel_strobe;
    always @(posedge clk50M or negedge reset_n)
        if (!reset_n)                          begin vel_str_cnt <= 19'd0; vel_strobe <= 1'b0; end
        else if (vel_str_cnt == 19'd499_999)   begin vel_str_cnt <= 19'd0; vel_strobe <= 1'b1; end
        else                                   begin vel_str_cnt <= vel_str_cnt + 19'd1; vel_strobe <= 1'b0; end
    wire [63:0] vel_snap;
    wire        vel_snap_strobe;
    //  ★CDC：我们的 imu_attitude（loc_clk50m 域，自动标零）→ 他们 board（clk50M 域）。
    //    板级 imu_valid 用目的域的 dst_strobe（每样本一拍）；数据 96 位 = yaw/pitch/roll。
    wire [95:0] imu_snap;
    // ★ imu_snap_strobe 已上移至第 751 行块（防重复声明）
    task123_snapshot_cdc #(.WIDTH(96)) u_imu_cdc(
        .src_clk    (loc_clk50m                     ),
        .dst_clk    (clk50M                         ),
        .rst_n      (reset_n                        ),
        .src_strobe (imu_data_valid                 ),
        .src_data   ({yaw_q16, pitch_q16, roll_q16}),
        .dst_data   (imu_snap                       ),
        .dst_strobe (imu_snap_strobe                ),
        .fresh      (                               )
    );
    task123_snapshot_cdc #(.WIDTH(64)) u_vel_cdc(
        .src_clk    (clk50M        ),
        .dst_clk    (loc_clk50m    ),
        .rst_n      (reset_n       ),
        .src_strobe (vel_strobe    ),
        .src_data   (obs_velocities),
        .dst_data   (vel_snap      ),
        .dst_strobe (vel_snap_strobe),
        .fresh      (              )
    );

    //==================================================================
    //  任务四 P0/P1 位姿信号（由下方 u_odom 驱动；此处先声明，
    //  因为 u_bt_link 在 u_odom 之前例化、要引用它们做遥测）
    //==================================================================
    wire signed [31:0] odom_x;
    wire signed [31:0] odom_y;
    wire signed [31:0] odom_th;
    wire signed [31:0] odom_vx;
    wire signed [31:0] odom_vy;
    wire signed [15:0] odom_wd;
    wire               odom_slip;
    wire       [31:0]  odom_encerr;
    wire       [15:0]  odom_telcnt;

    // ---- 任务四 P3.5 电池电压（由下方 u_vbat_mon 驱动；先声明，
    //      因为 u_bt_link 在它之前例化、要引用 vbat_01v 做遥测）----
    wire       [7:0]   vbat_01v;      // 电池 0.1V（126 = 12.6V）
    wire               vbat_low;      // 1 = 低于阈值
    wire       [11:0]  vbat_raw;      // ADC 原始码（诊断）
    wire       [15:0]  vbat_cnt;      // ADC 转换次数（存活判据）

    // ---- 任务四 P1 连线（BT 帧 -> 指令解析 -> 导航 -> 途径点队列 -> 应答）----
    wire [32*8-1:0]    bt_frame_bus;    // bt_link 的整帧输出（FRAME_MAX=32）
    wire [5:0]         bt_frame_len;
    wire               bt_frame_vld;
    wire [7:0]         p1_cmd;          // bt_cmd2 命令码
    wire signed [31:0] p1_arg0, p1_arg1;
    wire               p1_cmd_vld;

    wire               q_push, q_pop, q_clr, q_seek_en;
    wire [4:0]         q_seek_n;
    wire signed [31:0] q_px, q_py;
    wire signed [31:0] q_head_x, q_head_y;
    wire [4:0]         q_count;
    wire               q_empty, q_full;

    wire               nav_hold_off;      // ★nav_ctrl 的强制关断（$STOP/$ESTOP 200ms）
    wire               nav_resp_vld;
    wire [3:0]         nav_resp_type;
    wire [4:0]         nav_resp_arg;
    wire signed [31:0] nav_vx, nav_vy, nav_w;   // P2 接 mech_ik（本阶段悬空）
    wire [2:0]         nav_st;
    wire [4:0]         nav_wp;

    // ---- 任务四 P2 电机通路连线 ----
    // ★ 2026-10-30 v77r2：t2_in1/t2_in2/t2_pwm/t2_stdby 已上移至第 740 行块（防重复声明）
    wire [3:0]         nm_in1, nm_in2, nm_pwm;   // nav 电机通路输出
    wire               nm_stdby;
    wire signed [15:0] nm_vfl, nm_vfr, nm_vrl, nm_vrr;
    wire signed [15:0] nm_sp0, nm_sp1, nm_sp2, nm_sp3;
    reg                motor_owner;              // 0=任务二(默认) 1=nav 接管
    //  ★反向合并 2026-10-17 追加声明
    //  ★ 2026-10-30 v77r2：t3_trig/sonar_gated/task3_sensor_enabled/nav_hold/
    //    obs_front_mm/obs_front_vld/obs_positions/obs_velocities 已上移至第 739 行块
    //    （消除 EX3638 隐式声明；重复声明会导致 EX3629）
    wire [3:0]  ab_a, ab_b;              // ab_filter 后的编码器
    reg         dist_vld_d;
    wire        dist_vld_1shot;
    wire [15:0] nav_dist_cm;
    reg signed [31:0] od_x_q, od_y_q, od_th_q;      // odom→nav 边界寄存
    reg signed [31:0] nav_vx_q, nav_vy_q, nav_w_q;  // nav→motor 边界寄存
    wire pose_clr = p1_cmd_vld && (p1_cmd == 8'd13);  // $ORG 重置原点
    wire cmd_go    = p1_cmd_vld && (p1_cmd == 8'd6);
    wire cmd_estop = p1_cmd_vld && (p1_cmd == 8'd8);
    //  ★2026-10-19 修：$MV 手动动作也必须拿电机所有权 —— 否则 MUX 仍选任务二链，
    //    nav_motor 的输出根本到不了电机（实测现象：$MV 有应答、车一点不动）
    wire cmd_mv    = p1_cmd_vld && (p1_cmd == 8'd10);
    // ★2026-10-19 运行时可切换极性（$CFG,1,encMask / $CFG,2,dirMask）
    //   目的：逐轮试出正确极性，不用每次重编 20 分钟
    //   初值 4'b1010 = 右两轮(FR,RR)编码器反向 —— 依据：AB_SWAP 全反后"右轮正常、左轮停"
    wire cmd_cfg   = p1_cmd_vld && (p1_cmd == 8'd14);
    reg  [3:0] cfg_enc, cfg_dir, cfg_rst;
    reg  [1:0] cfg_gear;                    // ★速度档位 0慢/1中/2快（默认中）
    always @(posedge loc_clk50m or negedge reset_n) begin
        if (!reset_n) begin cfg_enc <= 4'b1010; cfg_dir <= 4'b0000; cfg_rst <= 4'd0; cfg_gear <= 2'd1; end
        else if (cmd_cfg) begin
            if      (p1_arg0 == 32'sd1) cfg_enc <= p1_arg1[3:0];
            else if (p1_arg0 == 32'sd2) cfg_dir <= p1_arg1[3:0];
            else if (p1_arg0 == 32'sd3) cfg_gear <= p1_arg1[1:0];   // ★速度档位
            cfg_rst <= 4'd10;                    // 极性变更后让速度环复位 10 tick，清积分
        end
        else if (cfg_rst != 4'd0) cfg_rst <= cfg_rst - 4'd1;
    end
    localparam [23:0] GATE_200MS = 24'd10_000_000;
    //  声呐：他们的 board 管 TRIG，本顶层做"导航接管自动放行"门控
    //  ★2026-10-19 修（F1）：原写成 `sonar_gated || motor_owner` —— 接管后 TRIG 被拉成常高，
    //    超声波收不到 10us 触发脉冲，导航期间的避障形同虚设。改为"接管时直接用他们的原始脉冲"：
    //    motor_owner=1 ⇒ t3_trig[0]（他们 sensing 自由跑，不受 S1 手动门控影响）
    //  ★2026-10-21：motor_owner(loc域) 与 sonar_gated(clk50M域) 混合驱动输出引脚有毛刺风险
    //    ⇒ motor_owner 先 2 级同步进 clk50M，再寄存输出（TRIG 毛刺会引发假测距/假避障）
    reg motor_owner_c1, motor_owner_c2;
    always @(posedge clk50M or negedge reset_n)
        if (!reset_n) {motor_owner_c2, motor_owner_c1} <= 2'b00;
        else          {motor_owner_c2, motor_owner_c1} <= {motor_owner_c1, motor_owner};
    reg ultra_trig_r;
    always @(posedge clk50M or negedge reset_n)
        if (!reset_n) ultra_trig_r <= 1'b0;
        /* ★2026-10-22 彻底旁路 S1 门控：实测未按 S1 时 gated_trig 恒 0（enabled 需要
           "先按 S1 置 requested"）⇒ 声呐收不到 TRIG ⇒ 串口 SF=1/F=00000 ⇒ 避障永不触发。
           TRIG 由他们的 sensing 模块单一驱动，直通最安全。*/
        else          ultra_trig_r <= t3_trig[0];
    assign ultra_trig = ultra_trig_r;
    wire nav_motor_run = motor_owner && (gate_cnt >= GATE_200MS) && (nav_st != 3'd4) && (cfg_rst == 4'd0);

    //==================================================================
    //  任务四 PB · 蓝牙链路（2026-10-12 WorkBuddy）
    //  ------------------------------------------------------------------
    //  源文件：src/own/bt_link.v（v2.0：bt_rx_byte / bt_tx_byte / bt_frame / bt_link，
    //          旧 bt_cmd/bt_telem 已删；命令统一由 u_bt_cmd2 解码后回灌 i_cmd）
    //  原始模块：bt_rx_byte / bt_tx_byte / bt_frame /
    //          bt_cmd / bt_telem 五个子模块）
    //  ★ 独立通道：与 u_task2 内部 M15→CH340E→COM13 的 115200 诊断**两条并存**，
    //    互不相干。**src/task2/selftest_ir.v 一个字节未动**（诊断通道就在那个文件里）。
    //  ★ 时钟 = loc_clk50m（PLL 输出）—— 与 u_odom / u_nav / u_wp_queue 同域，
    //    整条任务四链路单时钟，零跨时钟域处理。
    //  ★ 复位共用 reset_n（B21 = S4），与任务二一致。
    //  ★ 验收（PB 阶段）：手机（安卓 BLE App，连上后点 Notify）发 $PING ⇒ 回 $PONG
    //  ★ 观测口 o_* 本阶段**故意悬空**；P0 做遥测 / P1 加 $GO,$WP 时再接线，结构不动。
    //==================================================================
    bt_link #(
        .TIMEOUT_GAP (15190),   // 115200 断帧超时（教材 ch9 Baud_Set=4 原值，304 µs）
        .FRAME_MAX   (32    )   // 帧缓冲 32 字节（$VEL,500,500,300 = 16，2× 余量）
    ) u_bt_link (
        .clk           (loc_clk50m),
        .rst_n         (reset_n ),
        .bt_rx         (bt_rx   ),
        .bt_tx         (bt_tx   ),
        // ---- P1/P3 遥测输入: 直接吃 odom 的位姿 ⇒ odom 有了真实下游 ----
        //   （exp36 曾因 odom 输出全悬空被整块优化掉: WARN NL0002 swept）
        .i_x_q16       (odom_x  ),
        .i_y_q16       (odom_y  ),
        .i_th_q16      (odom_th ),
        /* ★2026-10-28 诊断通道（组长工程整合）：视觉任务期间把「当前目标点」字段挪用作 vision_reason
           （$TEL 的 wp 字段是两位十进制，0~15 放得下）。网页「当前目标点」在故障态
           显示 R<reason>，不必再加 OSD/串口字段。非视觉任务时行为完全不变。 */
        .i_wp          (vision_owner ? {1'b0, vision_reason} : nav_wp ),     // nav_ctrl 目标点号 / 视觉故障码
        .i_st          (combined_state),     // nav_ctrl 状态码（视觉任务时映射视觉状态）
        // ---- v3 新增：速度与电池电量进 $TEL 帧 ----
        .i_vx_q16      (odom_vx   ),
        .i_vy_q16      (odom_vy   ),
        .i_batt_01v    (vbat_01v  ),
        /* ---- v74 避障观测（来自 u_nav 的 dbg_*，打包进 $DBG 帧）---- */
        .i_dbg_phase   (dbg_phase ),
        .i_dbg_sub     (dbg_sub   ),
        .i_dbg_step    (dbg_step  ),
        .i_dbg_near    (dbg_near  ),
        .i_dbg_far     (dbg_far   ),
        .i_dbg_unk     (dbg_unk   ),
        .i_dbg_samp    (dbg_samp  ),
        .i_dbg_odo     (dbg_odo   ),
        .i_dbg_w       (dbg_w     ),
        .i_dbg_gear    (dbg_gear  ),
        // ---- ④ 单一解码器：bt_cmd2 的解码结果回灌 ----
        //   bt_link 只管链路级语义（$PING/$STAT/$TEL,0|1），运动类交 nav_ctrl
        .i_cmd         (p1_cmd    ),
        .i_arg0        (p1_arg0   ),
        .i_cmd_vld     (p1_cmd_vld),
        // ---- P1/P3 应答输入: nav_ctrl 的 resp_* ⇒ 回 $ACK/$DON/$ERR ----
        .i_resp_vld    (nav_resp_vld ),
        .i_resp_type   (nav_resp_type),
        .i_resp_arg    (nav_resp_arg ),
        .o_tel_cnt     (odom_telcnt),
        .o_resp_cnt    (        ),
        .o_rx_byte     (        ),
        .o_rx_byte_vld (        ),
        .o_frame       (bt_frame_bus),
        .o_frame_len   (bt_frame_len),
        .o_frame_vld   (bt_frame_vld),
        .o_pong_cnt    (        )
    );

    //==================================================================
    //  任务四 P0 · 里程计（2026-10-13 WorkBuddy）
    //  ------------------------------------------------------------------
    //  源文件：src/own/odom_top.v（内含 quad_dec/speed_meter 的独立实例 +
    //          odom_core 位姿积分 + sin_cos_rom 查找表）
    //  ★ 与 u_task2 内部编码器解码**并行读取同一组引脚**（输入脚多实例合法），
    //    因此 src/task2/*.v **一个字节未动**，M15→COM13 诊断通道完好。
    //  ★ 时钟用 loc_clk50m —— 与 IMU 链路(yaw_q16/imu_data_valid)同域，
    //    零跨时钟域处理。
    //  ★ θ 源：odom_core 的 THETA_SRC=1（IMU 增量主导，免疫打滑）。
    //  ★ clr 本阶段接 1'b0（只靠 reset_n 复位）；P1 由 bt_cmd 的指令驱动。
    //  ★ 输出 x/y/θ 本阶段**故意悬空**（加 syn_keep 防综合优化掉），
    //    P1 做导航状态机 / P3 做遥测时直接接线，结构不动。
    //==================================================================

    //  ★里程坏样本闸门（2026-10-20）：物理上限 ≈2.7m/s = 124 counts/10ms。
    //    横移/旋转实测出现 40 倍里程发散 + 日志"位置跳变 1.0m" ⇒ 疑有异常速度样本。
    //    任一轮超出 ⇒ 该帧整帧喂 0（位置冻结一拍），绝不把垃圾积分进里程。
    wire signed [15:0] vw0 = $signed(vel_snap[15:0]), vw1 = $signed(vel_snap[31:16]),
                       vw2 = $signed(vel_snap[47:32]), vw3 = $signed(vel_snap[63:48]);
    wire vel_plausible =
        (vw0 > -16'sd40) && (vw0 < 16'sd40) &&      /* ★2026-10-21 150→80：物理上限 11.5 counts
                                                       （250mm/s），40 = 3.5 倍余量（80~40 之间的成串尖峰曾冻住门控 ⇒ 一跳一跳），拦更小的尖峰 */
        (vw1 > -16'sd40) && (vw1 < 16'sd40) &&
        (vw2 > -16'sd40) && (vw2 < 16'sd40) &&
        (vw3 > -16'sd40) && (vw3 < 16'sd40);
    reg  [1:0] vel_freeze;                       /* ★坏样本后冻结 2 帧（连续 2 帧正常才恢复）*/
    always @(posedge loc_clk50m or negedge reset_n)
        if (!reset_n)                            vel_freeze <= 2'd0;
        else if (!vel_plausible)                 vel_freeze <= 2'd2;
        else if (vel_freeze != 2'd0)             vel_freeze <= vel_freeze - 2'd1;
    wire [63:0] vel_safe = (vel_plausible && vel_freeze == 2'd0) ? vel_snap : 64'd0;

    odom_top #(
        .WIN_CYCLES (500_000),   // 10ms 测速窗口 @50MHz
        .THETA_SRC  (1),         // ★2026-10-21 改 IMU 航向：原地旋转打滑会让编码器航向虚高 1.5×
                                 //   （实测 1080° 只转 720°），污染后续导航 ⇒ 弧线根源。IMU 直接量车体。
                                 //   静止漂移由上电 2s 自动标零压制（演示时长内可忽略）。
        .EXT_VEL_EN (1)          // ★四轮计数用他们的 obs_velocities（counts/10ms，轮序/符号经实车验证）
    ) u_odom (
        .clk          (loc_clk50m),
        .rst_n        (reset_n  ),
        .enc_a        (ab_a     ),
        .enc_b        (ab_b     ),
        .clr          (pose_clr ),
        .imu_valid    (imu_data_valid),
        .imu_yaw_q16  (yaw_q16  ),
        /* ★2026-10-22 修正：不能用 nav_motor_run（它是"电机使能"，拿到控制权后恒 1 ⇒
           静止时门控常开 ⇒ 继续漂 0.5→1.2）。改用"导航正在执行动作"= S_NAV 或 S_AVOID。*/
        .motion_cmd   (vision_owner ? vision_drive : ((nav_st == 3'd1) || (nav_st == 3'd3))),
        .x_q16        (odom_x   ),
        .y_q16        (odom_y   ),
        .theta_q16    (odom_th  ),
        .vx_q16       (odom_vx  ),
        .vy_q16       (odom_vy  ),
        .wdeg_q16     (odom_wd  ),
        .slip         (odom_slip),
        .enc_err      (odom_encerr),
        .ext_w_fl     (vel_safe[15:0] ),
        .ext_w_fr     (vel_safe[31:16]),
        .ext_w_rl     (vel_safe[47:32]),
        .ext_w_rr     (vel_safe[63:48])
    );

    //==================================================================
    //  任务四 P3.5 · 电池电压监测（2026-10-14 WorkBuddy）
    //  ------------------------------------------------------------------
    //  源文件：src/own/vbat_mon.v（内含 adc128s102.v —— 教材第 24 章原码，
    //          串行时序逻辑一字未改，只统一端口命名）
    //  ★ 时钟 = loc_clk50m ⇒ 与 u_bt_link / bt_telem2 同域，零跨时钟域。
    //  ★ 激发 = P5 第 1 脚（Analog_In0）上的电池分压：
    //        电池+ ─[R1=10k]─┬─→ P5 第 1 脚
    //                        [R2=2.2k]
    //        电池− ──────────┴─→ P5 第 2 脚 (AGND)
    //  ★ 输出 0.1V 单位电压喂进 $TEL 的 batt 字段；低电压阈值 11.0V。
    //  ★ 未接分压时 ADC 读到 ≈0 ⇒ App 显示 0.0V（不会误报"有电"）。
    //  ⚠️ R1_OHM / R2_OHM 必须与**实际焊上去的电阻**一致，否则电压换算会错。
    //==================================================================
    vbat_mon #(
        .DIV_PARAM (13),          // SCLK = 50MHz/(2*13) = 1.923MHz（教材原值）
        .TRIG_DIV  (2_500_000),   // 20Hz 触发（一拍 20ns ⇒ 50ms）
        .R1_OHM    (10_000),      // ★ 上分压 10kΩ（接电源"端子2"：开关后、保险丝前）
        .R2_OHM    (4_700),       // ★ 下分压 4.7kΩ（跨焊在 P5 的 1↔2 脚之间）
        .LOW_01V   (70)           // 7.0V 低电压告警（7.4V 2S 锂电，满电 8.4V）
    ) u_vbat_mon (
        .clk         (loc_clk50m),
        .rst_n       (reset_n   ),
        .adc_cs_n    (adc_cs_n  ),
        .adc_din     (adc_din   ),
        .adc_dout    (adc_dout  ),
        .adc_sclk    (adc_sclk  ),
        .o_vbat_01v  (vbat_01v  ),
        .o_vbat_low  (vbat_low  ),
        .o_raw       (vbat_raw  ),
        .o_conv_cnt  (vbat_cnt  )
    );

    //==================================================================
    //  任务四 P1 · 指令解析（2026-10-14 WorkBuddy）
    //  ------------------------------------------------------------------
    //  源文件：src/own/bt_cmd2.v
    //  ★ 旁路扩展：**不动已验证的 bt_link.v**，只吃它引出的 o_frame。
    //  ★ 识别的命令码与方案 v3 §11/§12 一致:
    //     PING/STAT/CLR/ADD/ARM/GO/STOP/ESTOP/WP/MV/GOP/TEL
    //==================================================================
    bt_cmd2 #(
        .FRAME_MAX (32)
    ) u_bt_cmd2 (
        .clk       (loc_clk50m),
        .rst_n     (reset_n  ),
        .frame     (bt_frame_bus),
        .frame_len (bt_frame_len),
        .frame_vld (bt_frame_vld),
        .cmd       (p1_cmd   ),
        .arg0      (p1_arg0  ),
        .arg1      (p1_arg1  ),
        .cmd_vld   (p1_cmd_vld)
    );

    //==================================================================
    //  任务四 P1 · 途径点队列（16 点 FIFO + seek 断点续跑）
    //  源文件：src/own/wp_queue.v
    //==================================================================
    wp_queue #(
        .DEPTH (16)
    ) u_wp_queue (
        .clk      (loc_clk50m),
        .rst_n    (reset_n  ),
        .clr      (q_clr    ),
        .push     (q_push   ),
        .push_x   (q_px     ),
        .push_y   (q_py     ),
        .pop      (q_pop    ),
        .seek_en  (q_seek_en),
        .seek_n   (q_seek_n ),
        .head_x   (q_head_x ),
        .head_y   (q_head_y ),
        .count    (q_count  ),
        .empty    (q_empty  ),
        .full     (q_full   )
    );

    //==================================================================
    //  任务四 P1 · 导航状态机（全向场向量控制 + 两段式 ARM/GO）
    //  源文件：src/own/nav_ctrl.v
    //  ★ vx/vy/w 本阶段**故意悬空**（P2 再接 mech_ik 并做与任务二的仲裁）；
    //    nav 本身因 resp_* 接进了 bt_link 而不会被优化掉。
    //==================================================================
    //==================================================================
    //  任务四 P4 · 超声波测距链（HC-SR04 类，硬件计时 + 3 采中值）
    //  dist_filter（10Hz 触发 + 量程门限 + 掉线清窗 + 中值）
    //        └─▶ ultrasonic_hcsr04（10us TRIG + ECHO 硬件计宽 + Q16 点乘换算）
    //  输出 ultra_filt_cm/vld ──▶ nav_ctrl 的避障判定（OBS_TH_CM）
    //  ⚠️ 传感器未接/无回波时 vld 不置位 ⇒ 避障天然不激活（不会误掰方向）
    //==================================================================
    //  ★反向合并 2026-10-17：声呐由他们的 sensing 链接管（在上方 u_task123 内部）。
    //    原 dist_filter + ultrasonic_hcsr04 移除（一个 TRIG 只能有一个驱动者）。
    //    nav 避障输入改吃他们的 control 样本：mm→cm 无除法 + 上升沿检测
    //    （他们的 control_valid 在近距/无回波时是电平，直接喂会让避障计数瞬间打满）。
    //  ★CDC：obs_front_mm/vld 也是 clk50M 域 ⇒ 同样过邮箱（17 位 = vld + mm）。
    //    源持续写（邮箱自节流），目的域始终持有最新样本；vld 的边沿检测在目的域做。
    //  ★2026-10-19 修（F2+F3）：他们的 obs_front_vld 是 control_valid（**电平**），
    //    下游做上升沿检测 ⇒ 一次接近只出一个边沿 ⇒ nav 的"连续 3 次确认"永远凑不满 ⇒ 避障不动作。
    //    改法：board 侧把 obs_front_vld 改成**每次采样的脉冲**（见 F2），邮箱用它做 src_strobe，
    //    目的域直接用 dst_strobe 当"新样本"边沿（每 65ms 一拍，3 次确认 ≈ 195ms）。
    wire [15:0] dist_snap;
    wire        dist_snap_strobe;
    task123_snapshot_cdc #(.WIDTH(16)) u_dist_cdc(
        .src_clk    (clk50M                        ),
        .dst_clk    (loc_clk50m                    ),
        .rst_n      (reset_n                       ),
        .src_strobe (obs_front_vld                 ),   // 他们的采样脉冲（F2 已改为脉冲）
        .src_data   (obs_front_mm                  ),
        .dst_data   (dist_snap                     ),
        .dst_strobe (dist_snap_strobe              ),
        .fresh      (                              )
    );
    assign dist_vld_1shot = dist_snap_strobe;
    assign nav_dist_cm    = (dist_snap * 32'd6554) >> 16;

    //  ★反向合并：AB 滤波（2 级同步 + 16 拍稳定）⇒ 治"静止漂移"
    task3_ab_filter u_ab_filt (
        .clk   (loc_clk50m),
        .rst_n (reset_n),
        .a     (enc_a),
        .b     (enc_b),
        .a_out (ab_a),
        .b_out (ab_b)
    );
    //  ★边界寄存：odom → nav_ctrl（切断 odom.y → w_cmd 长路径）
    always @(posedge loc_clk50m or negedge reset_n)
        if (!reset_n) begin od_x_q <= 32'sd0; od_y_q <= 32'sd0; od_th_q <= 32'sd0; end
        else begin od_x_q <= odom_x; od_y_q <= odom_y; od_th_q <= odom_th; end
    //  ★边界寄存：nav_ctrl → nav_motor（切断 wp_queue → 逆解乘法器 长路径）
    //  ★2026-10-29 整合：视觉任务时由 vision_mission 接管速度输出
    always @(posedge loc_clk50m or negedge reset_n)
        if (!reset_n) begin nav_vx_q <= 32'sd0; nav_vy_q <= 32'sd0; nav_w_q <= 32'sd0; end
        else begin
          nav_vx_q <= vision_owner ? (vision_drive ? vision_vx:32'sd0):nav_vx;
          nav_vy_q <= vision_owner ? 32'sd0:nav_vy;
          nav_w_q <= vision_owner ? (vision_drive ? vision_w:32'sd0):nav_w;
        end

    nav_ctrl #(
        .SPEED_MAX (200    ),   // mm/s 导航最大平移（★2026-10-19 300→200：现场演示要稳，不追速度）
        .MAN_SPEED (200    ),   // mm/s 手动平移
        .ARR_MM    (30     ),   // mm 到达阈值（★与 v41 位流一致：30mm；v41 实测 4 点导航到点偏差 2~6cm）
                                // ★2026-10-22 60→30：实测停在 0.94（差 7cm，符合 60mm 设计），
                                //   收紧到 30mm ⇒ 到点误差约 3~4cm
                                // ★2026-10-22 150→60：模块默认早在 10-21 就改成 60，但顶层这里
                                //   显式传 150 覆盖了它（与 GYRO_Q/PERIOD 同类：改默认值必须查顶层）
        .SOFT_MS   (1000   ),   // ms GO 后软延时
        .TICK_DIV  (500_000),   // 10ms 控制周期
        .SPIN_RATE (55     )    // 度/s 旋转（★2026-10-20 45→55：让轮缘线速度≈前进 200mm/s，视觉协调）
    ) u_nav (
        .clk       (loc_clk50m),
        .rst_n     (reset_n  ),
        .cmd_vld   (vision_nav_vld),   // ★2026-10-29 整合：$FIND 时向 nav 注入 STOP
        .cmd       (vision_nav_cmd),
        .arg0      (p1_arg0  ),
        .arg1      (p1_arg1  ),
        .q_push    (q_push   ),
        .q_pop     (q_pop    ),
        .q_clr     (q_clr    ),
        .q_seek_en (q_seek_en),
        .q_seek_n  (q_seek_n ),
        .q_px      (q_px     ),
        .q_py      (q_py     ),
        .q_head_x  (q_head_x ),
        .q_head_y  (q_head_y ),
        .q_count   (q_count  ),
        .q_empty   (q_empty  ),
        .x_q16     (od_x_q   ),
        .y_q16     (od_y_q   ),
        .th_q16    (od_th_q  ),
        // ---- P4 避障输入：接 dist_filter 的中值滤波输出（≥3 个新鲜样本才置 vld）----
        .i_dist_cm (nav_dist_cm   ),
        .i_dist_vld(dist_vld_1shot),
        .vx_cmd    (nav_vx   ),
        .vy_cmd    (nav_vy   ),
        .w_cmd     (nav_w    ),
        .i_gear    (cfg_gear  ),
        .i_imu_yaw (yaw_q16   ),   // ★IMU 航向（loc_clk50m 同域）—— 旋转闭环主反馈
        .o_hold    (nav_hold_off),
        .st        (nav_st   ),
        .wp        (nav_wp   ),
        .resp_vld  (nav_resp_vld ),
        .resp_type (nav_resp_type),
        .resp_arg  (nav_resp_arg ),
        /* ---- v74 避障观测（透传到 u_bt_link 的 $DBG 帧，看任务三/四内部相位）---- */
        .dbg_phase (dbg_phase ),
        .dbg_sub   (dbg_sub   ),
        .dbg_step  (dbg_step  ),
        .dbg_near  (dbg_near  ),
        .dbg_far   (dbg_far   ),
        .dbg_unk   (dbg_unk   ),
        .dbg_samp  (dbg_samp  ),
        .dbg_odo   (dbg_odo   ),
        .dbg_w     (dbg_w     ),
        .dbg_gear  (dbg_gear  )
    );

    //==================================================================
    //  任务四 P2 · 电机通路（2026-10-14 WorkBuddy）
    //  ------------------------------------------------------------------
    //  源文件：src/own/nav_motor.v（内含 mech_ik + 4×speed_loop）
    //  ★ 与任务二**完全并行的第二条电机通路**，靠下面 2:1 多路选择切换：
    //      motor_owner = 0（默认/复位/$ESTOP） ⇒ 任务二（已验证行为，不变）
    //      motor_owner = 1（首次 $ARM 起锁定到复位）⇒ nav
    //  ★ src/task2/selftest_ir.v **一个字节未动**。
    //  ★ 安全设计：nav 未接管时 nav_motor.run=0 ⇒ 它输出全 0；
    //     nav 进入 FAULT($ESTOP) 时 nav_motor_run=0 ⇒ stdby 拉低(驱动高阻)。
    //  ★ 上板标定：若整车方向相反，翻 nav_motor 的 NEG_TARGET / INVERT_DIR。
    //==================================================================
    //  ★所有权 FSM（与定稿一致）：S3 硬线最高优先 / $GO 拿权 / $ESTOP 交还 / 200ms 空档
    reg  [1:0] s3_sync;
    reg  [3:0] s3_cnt;
    reg [23:0] gate_cnt;
    always @(posedge loc_clk50m or negedge reset_n)
        if (!reset_n) begin s3_sync <= 2'b11; s3_cnt <= 4'd0; end
        else begin
            s3_sync <= {s3_sync[0], key[3]};
            if (!s3_sync[1]) begin if (s3_cnt != 4'hF) s3_cnt <= s3_cnt + 1'b1; end
            else             s3_cnt <= 4'd0;
        end
    wire s3_pressed = s3_cnt[3];
    always @(posedge loc_clk50m or negedge reset_n)
        if (!reset_n)        motor_owner <= 1'b0;
        else if (s3_pressed) motor_owner <= 1'b0;   // ★S3 硬线最高优先
        //  ★2026-10-19 审计修（F4）：软急停**不再交还所有权**。
        //    原行为：$ESTOP → owner=0 ⇒ 他们的链（若曾被 S1/S2 使能）会立刻接管并可能自动起跑 ⇒ 危险。
        //    现行为：owner 保持 1（他们的链继续被 nav_hold 压住），nav 进入 FAULT ⇒ nav_motor_run=0
        //            ⇒ stdby=0（电机断电）—— 同样是"停"，但不会把车交给他们的自动流程。
        //    要把控制权交回他们：按 S3（硬线，最高优先），或在页面按"断开"后重新上电。
        else if (cmd_estop)  motor_owner <= motor_owner;
        else if (cmd_go)     motor_owner <= 1'b1;   // $GO 拿权
        else if (cmd_mv)     motor_owner <= 1'b1;   // ★$MV 手动动作也拿权（锁定到复位/S3/$ESTOP）
        else if (vision_start) motor_owner <= 1'b1; // ★2026-10-29 整合：$FIND 也拿权（视觉任务需要电机）
    /* ★★★ 2026-10-30 11:20 撤销 v80 的三处"修复" ★★★
       用户实测：v73 及以前「前进1m/退1m」正常，导航不动是 nav_ctrl 的 ex/ey 缺陷。
       既然如此，这三处我自认为在"修"的改动应当全部回退，恢复已验证的原式：
         1. .run (nav_motor_run)        → 本就是 nav_motor_run（v80 反而加过门控又删回）
         2. nav_hold = motor_owner_c2 || vision_owner_c2  → 恢复 motor_owner
         3. stdby 三层嵌套              → 恢复 motor_owner ? (nav_hold_off ? 0 : nm_stdby) : t2_stdby
       唯一保留：nav_hold 恢复 v73 原式 = motor_owner（不用同步版，避免引入新的行为差异）。
       随之删掉 vision_owner_c1/c2（原只服务 nav_hold，无其它消费者，留着会被综合器 sweep）。 */
    assign nav_hold = motor_owner;

    always @(posedge loc_clk50m or negedge reset_n)
        if (!reset_n)         gate_cnt <= 24'd0;
        else if (!motor_owner) gate_cnt <= 24'd0;
        else if (gate_cnt < GATE_200MS) gate_cnt <= gate_cnt + 24'd1;

    nav_motor #(
        .WHEEL_DIA_UM (100_000),   // 实测 D = 100 mm
        .LL_WW_MM     (413    ),   // 实测 L+W = 413.5 mm
        .KP_Q         (16'sd1500), // 任务二标定值
        .KI_Q         (16'sd130 ),
        .NEG_TARGET   (1      ),   // 对齐任务二 `mc = -u` 极性
        .INVERT_DIR   (0      )
    ) u_nav_motor (
        .clk      (loc_clk50m  ),
        .rst_n    (reset_n    ),
        .vx_q16   (nav_vx_q   ),
        .vy_q16   (nav_vy_q   ),
        .w_q16    (nav_w_q    ),
        /* ★2026-10-30 v80 修复（车不动 · 实测 10:41~10:45）：
           原写法 `run = nav_motor_run && (!vision_owner || vision_drive)`，
           而 vision_drive 只由任务五/六 mission 产生 —— 一旦点过 $FIND，
           vision_owner 被置 1 且 \$MV/\$GO 都产生不了 vision_drive
           ⇒ run 恒 0 ⇒ n_stdby=0 ⇒ stdby=0 ⇒ 电机断电，S3 也救不回来
           （vision_owner 清零条件是 `... && !s3_pressed`，按 S3 时反而不清）。
           正确语义：vision_owner 只决定「速度指令取自 mission 还是导航」，
           不参与 run 门控。run 只看 nav_motor_run（已含 owner/空档/FAULT/复位）。 */
        .run      (nav_motor_run),
        .inv_enc_i(cfg_enc),
        .inv_dir_i(cfg_dir),
        .enc_a    (ab_a       ),
        .enc_b    (ab_b       ),
        .n_in1    (nm_in1     ),
        .n_in2    (nm_in2     ),
        .n_pwm    (nm_pwm     ),
        .n_stdby  (nm_stdby   ),
        .v_fl     (nm_vfl     ),
        .v_fr     (nm_vfr     ),
        .v_rl     (nm_vrl     ),
        .v_rr     (nm_vrr     ),
        .spd0     (nm_sp0     ),
        .spd1     (nm_sp1     ),
        .spd2     (nm_sp2     ),
        .spd3     (nm_sp3     )
    );

    // ---- 2:1 多路选择（唯一改动电机引脚的地方）----
    assign m_in1 = motor_owner ? nm_in1 : t2_in1;
    assign m_in2 = motor_owner ? nm_in2 : t2_in2;
    assign m_pwm = motor_owner ? nm_pwm : t2_pwm;
    /* ★★★ 2026-10-30 11:20 撤销 v80 的 stdby 改写，恢复 v73/v77 原式 ★★★
       原式：stdby = motor_owner ? (nav_hold_off ? 0 : nm_stdby) : t2_stdby
       v73 及以前上板验证正常（前进/后退/旋转都对），导航不动是 nav_ctrl 的
       ex/ey 漏赋值导致 arrived 恒 1，与 stdby 无关 ⇒ 不该动这里。 */
    assign stdby = motor_owner ? (nav_hold_off ? 1'b0 : nm_stdby) : t2_stdby;


    // ========================================================================
    // 【任务五/六 · mission 仲裁 · 2026-10-29 从组长整机工程整合】
    //   TASK56 V1 mission arbitration. Selected remains latched on STOP/fault/done.
    //   Only a fresh GO/MV after mission stop returns control to baseline nav.
    //   vision_dist_mm：超声无回波时报 0，vision_mission 会误判"前方 0mm 有东西"
    //     ⇒ 只给视觉任务做 0→9999 映射；nav_dist_cm（手动驾驶/基线避障）不动。
    // ========================================================================
    wire [15:0] vision_dist_mm = (dist_snap == 16'd0) ? 16'd9999 : dist_snap;

    wire vision_find,vision_stop;
    assign vision_start=vision_find && !s3_pressed && cfg_rst==0 &&
                      (nav_st==3'd0 || nav_st==3'd2) && !vision_active;
    wire vision_cancel=vision_stop || s3_pressed ||
       (p1_cmd_vld && (p1_cmd==8'd7 || p1_cmd==8'd8 || p1_cmd==8'd14));
    always @(posedge loc_clk50m or negedge reset_n)
      if(!reset_n)vision_owner<=0;
      else if(vision_start)vision_owner<=1;
      /* ★2026-10-30 v80 修复（保留）：原条件 `!vision_active && (cmd_go||cmd_mv) && !s3_pressed`
         —— 按 S3 时 `!s3_pressed=0` ⇒ vision_owner 永远清不掉，S3 急停后
         视觉标志一直挂着，$MV/$GO 全被卡死。S3 是最高优先硬急停，
         必须能强制交还控制权 ⇒ 把 s3_pressed 改为**置清条件**而非阻断条件。
         （这是整合引入的逻辑缺陷，与导航不动无关，但方向正确，予以保留。） */
      else if(s3_pressed)vision_owner<=0;
      else if(!vision_active && (cmd_go || cmd_mv))vision_owner<=0;
    vision_command u_vision_command(.clk(loc_clk50m),.rst_n(reset_n),
      .frame_vld(bt_frame_vld),.frame(bt_frame_bus),.frame_len(bt_frame_len),
      .find_pulse(vision_find),.stop_pulse(vision_stop));
    vision_mission u_vision_mission(.clk(loc_clk50m),.rst_n(reset_n),
      .start(vision_start),.stop(vision_cancel),.estop(cmd_estop),
      .permit(!s3_pressed && cfg_rst==0 && nav_st!=3'd4 && pll_lock),
      .frame_strobe(vision_snapshot_strobe),.box_valid(vision_snapshot[64]),
      .xmin(vision_snapshot[63:53]),.xmax(vision_snapshot[52:42]),
      /* 快照打包顺序 {box,xmin,xmax,ymin,ymax,count}：ymin/ymax 用于尺寸闸门 */
      .ymin(vision_snapshot[41:31]),.ymax(vision_snapshot[30:20]),
      .sonar_strobe(dist_snap_strobe),.distance_mm(vision_dist_mm),
      .selected(vision_selected_unused),.active(vision_active),.drive_enable(vision_drive),
      .state(vision_state),.reason(vision_reason),.vx(vision_vx),.w(vision_w));
    // Stop nav on FIND and block competing motion/queue/config commands while busy.
    assign vision_nav_vld=vision_start || (p1_cmd_vld &&
      (!vision_active || p1_cmd==8'd7 || p1_cmd==8'd8));
    assign vision_nav_cmd=vision_start ? 8'd7:p1_cmd;
    assign combined_state=vision_owner ?
      (vision_state==4'd5 ? 3'd4:vision_state==4'd4 ? 3'd2:
       vision_active ? 3'd1:3'd0):nav_st;

endmodule
