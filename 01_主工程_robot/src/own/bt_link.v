//=============================================================================
//  bt_link.v —— 任务四 PB 阶段 · 蓝牙人机通道（正点原子 ATK-BLE02，115200 8N1）
//-----------------------------------------------------------------------------
//  【本文件包含 5 个模块】
//    ① bt_rx_byte   8N1 接收（移植教材第 7 章 uart_byte_rx，16 倍过采样）
//    ② bt_tx_byte   8N1 发送（移植教材第 6 章 uart_byte_tx）
//    ③ bt_frame     字节流 → 完整帧（新写；$ 起帧 / \n 或超时断帧）
//    ④ 命令/应答语义（外部 bt_cmd2 统一解码 + 内部 bt_telem2 打包）
//    ⑥ bt_link      顶层（把上面 5 个连起来）
//
//  【例化位置】
//    顶层 robot/src/ov5640_ddr3_tft.v 中与 u_task2 并列例化。
//    ★ 刻意**不**放进 selftest_ir.v —— 因为 M15 → 板载 CH340E → USB(J2) → PC COM13
//      那条 115200 诊断通道就在 selftest_ir.v 里，放顶层才能保证**它一个字节都不动**。
//    ⇒ 板上两条独立通道并存：
//        · 诊断通道  M15  @115200  报文 "P1=… P2=… P3=… P4=…"（任务二，未改）
//        · 蓝牙通道  H14/H20 @115200  $PING / $PONG（本文件，新增）
//
//  【时钟域】全部 clk50M（Y18），与任务二同域 ⇒ 零跨时钟域处理。
//
//  【引脚（顶层 .cst 约束）】
//    bt_rx = P8 针1 USER_0 (球号 H14) ← 蓝牙模块 TXD   【FPGA 输入】
//    bt_tx = P8 针2 USER_2 (球号 H20) → 蓝牙模块 RXD   【FPGA 输出】
//    ⛔ 模块供电走 P8 的 VCC3V3（3.3V），绝不能接 5V（ATK-BLE02 核心 2.0~3.6V）
//
//  【波特率常数 —— 全部直接抄教材原码的 Baud_Set=4（115200）那一列】
//    教材原码路径：ACG720开发板资料/02_例程源码/GW5AT138K/第6|7|9章…/*.rar
//      教材 ch6 uart_byte_tx ：case(baud_set) 4: bps_DR <= 16'd433;   （非过采样）
//      教材 ch7 uart_byte_rx ：case(baud_set) 4: bps_DR <= 16'd26;    （16 倍过采样）
//      教材 ch9 uart_data_rx ：TIMEOUT = 15190                         （超时断帧）
//    校验：
//      ① TX  434 拍/位 = 8680 ns  ⇒ 115200.0 baud（误差 -0.006%）
//      ② RX  27 拍/采样 ×16 = 432 拍/位 = 8640 ns ⇒ 115740.7 baud（误差 +0.47%）
//      ③ 超时 15190 拍 = 303.8 µs（≈3.5 个字节时间）
//    分档对照（教材原码 case 表）：0=9600 / 1=19200 / 2=38400 / 3=57600 / 4=115200
//
//  【与教材原码的差异（逐条列明，其余照搬）】
//    bt_rx_byte：① 删除 baud_set 端口 → localparam BPS_DR=16'd26（固定 115200）
//                ② 端口名 Clk/Rst_n → clk/rst_n（统一项目命名）
//                ③ 模块名 uart_byte_rx → bt_rx_byte（避免与工程内同名冲突）
//    bt_tx_byte：① 删除 baud_set → localparam BPS_DR=16'd433
//                ② 端口名统一小写  ③ 模块名 → bt_tx_byte
//    ★ 教材 TX 的"每拍按 bps_cnt 的 case 给 uart_tx 赋值"写法**原样保留** ——
//      它让每一位天然保持满一个位周期。任务二里手写的那版诊断发送器是"移位 +
//      计数到点直接拉高"，才会让 bit7 只有 1 拍宽、把 \n(0x0A) 变成 0x8A。
//      ⇒ 本次移植**天然免疫**，不需要 U_HOLD 补丁。（见交接文档 §9）
//
//  【版本】2026-10-12 WorkBuddy（任务四 PB 阶段）
//    本项目无 git ⇒ 变更记录写在此文件头。
//    v2.0  2026-10-14 统一解码器：删旧 bt_cmd/bt_telem；改吃 bt_cmd2 的 i_cmd/i_arg0/i_cmd_vld；
//                     新增 $STAT→立即$TEL、$TEL,0/1→遥测开关；bt_telem2 加 EVT 标签
//    v1.0  2026-10-12 首版：bt_rx_byte / bt_tx_byte / bt_frame / bt_cmd / bt_telem / bt_link
//=============================================================================
`timescale 1ns/1ps

//=============================================================================
//  ① bt_rx_byte —— 8N1 串口接收（移植教材第 7 章 uart_byte_rx）
//-----------------------------------------------------------------------------
//  保留原码的全部抗干扰机制（别自作聪明简化）：
//    · 两级同步寄存器 + 两级数据寄存器 ⇒ 异步输入消亚稳态 + 取下降沿
//    · 16 倍过采样：一位采 16 拍，在位的第 22~27 拍（位中心）累计
//    · 6 取 3 中值投票（data_byte_pre[k] 累加 6 拍后取 [2]）⇒ 抗抖动
//    · 起始位校验：bps_cnt==12 时 START_BIT>2 判为假起始，中止
//    · 停止位校验：bps_cnt==155 时 STOP_BIT<3 判为帧错，中止
//=============================================================================
module bt_rx_byte(
    input  wire       clk,        // 50 MHz
    input  wire       rst_n,      // 低有效
    input  wire       uart_rx,    // 串口输入（来自蓝牙模块 TXD）

    output reg  [7:0] data_byte,  // 收到的一字节
    output reg        rx_done     // 1 拍脉冲（原码里 bps_cnt==159 是单拍状态）
);
    // ---- 固定 115200：教材 ch7 case(baud_set) 4: bps_DR <= 16'd26 ----
    localparam [15:0] BPS_DR = 16'd26;

    wire reset = ~rst_n;

    reg  uart_rx_sync1, uart_rx_sync2;   // 同步寄存器
    reg  uart_rx_reg1,  uart_rx_reg2;    // 数据寄存器

    reg  [15:0] div_cnt;                 // 分频计数器
    reg         bps_clk;                 // 波特率采样脉冲
    reg  [7:0]  bps_cnt;                 // 过采样计数器 0..159
    reg         uart_state;              // 接收进行中

    wire uart_rx_nedge;

    reg  [2:0] START_BIT;
    reg  [2:0] STOP_BIT;
    reg  [2:0] data_byte_pre [7:0];

    // ---- 同步串行输入，消除亚稳态 ----
    always @(posedge clk or posedge reset)
        if (reset) begin uart_rx_sync1 <= 1'b0; uart_rx_sync2 <= 1'b0; end
        else       begin uart_rx_sync1 <= uart_rx; uart_rx_sync2 <= uart_rx_sync1; end

    // ---- 数据寄存器 ----
    always @(posedge clk or posedge reset)
        if (reset) begin uart_rx_reg1 <= 1'b0; uart_rx_reg2 <= 1'b0; end
        else       begin uart_rx_reg1 <= uart_rx_sync2; uart_rx_reg2 <= uart_rx_reg1; end

    // ---- 下降沿检测 ----
    assign uart_rx_nedge = !uart_rx_reg1 & uart_rx_reg2;

    // ---- 分频计数（仅在接收期间计数）----
    always @(posedge clk or posedge reset)
        if (reset)
            div_cnt <= 16'd0;
        else if (uart_state) begin
            if (div_cnt == BPS_DR) div_cnt <= 16'd0;
            else                   div_cnt <= div_cnt + 1'b1;
        end
        else
            div_cnt <= 16'd0;

    // ---- bps_clk 生成 ----
    always @(posedge clk or posedge reset)
        if (reset)
            bps_clk <= 1'b0;
        else if (div_cnt == 16'd1) bps_clk <= 1'b1;
        else                       bps_clk <= 1'b0;

    // ---- 过采样计数器（0..159；159 是单拍状态，因为复位条件每拍都判）----
    always @(posedge clk or posedge reset)
        if (reset)
            bps_cnt <= 8'd0;
        else if (bps_cnt == 8'd159 | (bps_cnt == 8'd12 && (START_BIT > 2)))
            bps_cnt <= 8'd0;
        else if (bps_clk)
            bps_cnt <= bps_cnt + 1'b1;
        else
            bps_cnt <= bps_cnt;

    // ---- 单字节接收完成（1 拍脉冲）----
    always @(posedge clk or posedge reset)
        if (reset)
            rx_done <= 1'b0;
        else if (bps_cnt == 8'd159) rx_done <= 1'b1;
        else                        rx_done <= 1'b0;

    // ---- 位采样累加（6 取 3 中值投票）----
    always @(posedge clk or posedge reset)
        if (reset) begin
            START_BIT        <= 3'd0;
            data_byte_pre[0] <= 3'd0;
            data_byte_pre[1] <= 3'd0;
            data_byte_pre[2] <= 3'd0;
            data_byte_pre[3] <= 3'd0;
            data_byte_pre[4] <= 3'd0;
            data_byte_pre[5] <= 3'd0;
            data_byte_pre[6] <= 3'd0;
            data_byte_pre[7] <= 3'd0;
            STOP_BIT         <= 3'd0;
        end
        else if (bps_clk) begin
            case (bps_cnt)
                0:  begin
                        START_BIT        <= 3'd0;
                        data_byte_pre[0] <= 3'd0;
                        data_byte_pre[1] <= 3'd0;
                        data_byte_pre[2] <= 3'd0;
                        data_byte_pre[3] <= 3'd0;
                        data_byte_pre[4] <= 3'd0;
                        data_byte_pre[5] <= 3'd0;
                        data_byte_pre[6] <= 3'd0;
                        data_byte_pre[7] <= 3'd0;
                        STOP_BIT         <= 3'd0;
                    end
                6,7,8,9,10,11:            START_BIT        <= START_BIT        + uart_rx_sync2;
                22,23,24,25,26,27:        data_byte_pre[0] <= data_byte_pre[0] + uart_rx_sync2;
                38,39,40,41,42,43:        data_byte_pre[1] <= data_byte_pre[1] + uart_rx_sync2;
                54,55,56,57,58,59:        data_byte_pre[2] <= data_byte_pre[2] + uart_rx_sync2;
                70,71,72,73,74,75:        data_byte_pre[3] <= data_byte_pre[3] + uart_rx_sync2;
                86,87,88,89,90,91:        data_byte_pre[4] <= data_byte_pre[4] + uart_rx_sync2;
                102,103,104,105,106,107:  data_byte_pre[5] <= data_byte_pre[5] + uart_rx_sync2;
                118,119,120,121,122,123:  data_byte_pre[6] <= data_byte_pre[6] + uart_rx_sync2;
                134,135,136,137,138,139:  data_byte_pre[7] <= data_byte_pre[7] + uart_rx_sync2;
                150,151,152,153,154,155:  STOP_BIT         <= STOP_BIT         + uart_rx_sync2;
                default: begin
                        START_BIT        <= START_BIT;
                        data_byte_pre[0] <= data_byte_pre[0];
                        data_byte_pre[1] <= data_byte_pre[1];
                        data_byte_pre[2] <= data_byte_pre[2];
                        data_byte_pre[3] <= data_byte_pre[3];
                        data_byte_pre[4] <= data_byte_pre[4];
                        data_byte_pre[5] <= data_byte_pre[5];
                        data_byte_pre[6] <= data_byte_pre[6];
                        data_byte_pre[7] <= data_byte_pre[7];
                        STOP_BIT         <= STOP_BIT;
                    end
            endcase
        end

    // ---- 锁存输出（6 取 3 中值投票的判定位是 [2]）----
    always @(posedge clk or posedge reset)
        if (reset)
            data_byte <= 8'd0;
        else if (bps_cnt == 8'd159) begin
            data_byte[0] <= data_byte_pre[0][2];
            data_byte[1] <= data_byte_pre[1][2];
            data_byte[2] <= data_byte_pre[2][2];
            data_byte[3] <= data_byte_pre[3][2];
            data_byte[4] <= data_byte_pre[4][2];
            data_byte[5] <= data_byte_pre[5][2];
            data_byte[6] <= data_byte_pre[6][2];
            data_byte[7] <= data_byte_pre[7][2];
        end

    // ---- 接收状态机：下降沿启动；完成 / 假起始 / 帧错时中止 ----
    always @(posedge clk or posedge reset)
        if (reset)
            uart_state <= 1'b0;
        else if (uart_rx_nedge)
            uart_state <= 1'b1;
        else if (rx_done || (bps_cnt == 8'd12 && (START_BIT > 2))
                          || (bps_cnt == 8'd155 && (STOP_BIT < 3)))
            uart_state <= 1'b0;
        else
            uart_state <= uart_state;

endmodule


//=============================================================================
//  ② bt_tx_byte —— 8N1 串口发送（移植教材第 6 章 uart_byte_tx）
//-----------------------------------------------------------------------------
//  bps_cnt 的 12 拍循环（原样保留）：
//    0 = 空闲高 / 1 = 起始位 / 2..9 = D0..D7 / 10 = 停止位 / 11 = 收尾
//  ★ uart_tx 是"每拍按 bps_cnt 赋值"，而 bps_cnt 只在 bps_clk 时递增
//    ⇒ 每一位天然保持满一个位周期（434 拍 = 8680 ns）。
//  ★ send_en 必须是 **1 拍脉冲**：原码里 `if(send_en) uart_state<=1` 的优先级
//    高于 `else if(bps_cnt==11) uart_state<=0`，持续拉高会让它反复重启。
//=============================================================================
module bt_tx_byte(
    input  wire       clk,        // 50 MHz
    input  wire       rst_n,      // 低有效
    input  wire [7:0] data_byte,  // 待发送字节
    input  wire       send_en,    // 发送使能（1 拍脉冲）

    output reg        uart_tx,    // 串口输出（去蓝牙模块 RXD）
    output reg        tx_done     // 1 拍脉冲
);
    // ---- 固定 115200：教材 ch6 case(baud_set) 4: bps_DR <= 16'd433 ----
    localparam [15:0] BPS_DR = 16'd433;

    localparam START_BIT = 1'b0;
    localparam STOP_BIT  = 1'b1;

    wire reset = ~rst_n;

    reg         bps_clk;
    reg  [15:0] div_cnt;
    reg  [3:0]  bps_cnt;
    reg  [7:0]  data_byte_reg;
    reg         uart_state;

    always @(posedge clk or posedge reset)
        if (reset)
            uart_state <= 1'b0;
        else if (send_en)
            uart_state <= 1'b1;
        else if (bps_cnt == 4'd11)
            uart_state <= 1'b0;
        else
            uart_state <= uart_state;

    always @(posedge clk or posedge reset)
        if (reset)                  data_byte_reg <= 8'd0;
        else if (send_en)           data_byte_reg <= data_byte;
        else                        data_byte_reg <= data_byte_reg;

    always @(posedge clk or posedge reset)
        if (reset)
            div_cnt <= 16'd0;
        else if (uart_state) begin
            if (div_cnt == BPS_DR) div_cnt <= 16'd0;
            else                   div_cnt <= div_cnt + 1'b1;
        end
        else
            div_cnt <= 16'd0;

    always @(posedge clk or posedge reset)
        if (reset)
            bps_clk <= 1'b0;
        else if (div_cnt == 16'd1) bps_clk <= 1'b1;
        else                       bps_clk <= 1'b0;

    always @(posedge clk or posedge reset)
        if (reset)
            bps_cnt <= 4'd0;
        else if (bps_cnt == 4'd11)
            bps_cnt <= 4'd0;
        else if (bps_clk)
            bps_cnt <= bps_cnt + 1'b1;
        else
            bps_cnt <= bps_cnt;

    always @(posedge clk or posedge reset)
        if (reset)
            tx_done <= 1'b0;
        else if (bps_cnt == 4'd11) tx_done <= 1'b1;
        else                       tx_done <= 1'b0;

    always @(posedge clk or posedge reset)
        if (reset)
            uart_tx <= 1'b1;
        else begin
            case (bps_cnt)
                0:       uart_tx <= 1'b1;             // 空闲
                1:       uart_tx <= START_BIT;        // 起始位
                2:       uart_tx <= data_byte_reg[0];
                3:       uart_tx <= data_byte_reg[1];
                4:       uart_tx <= data_byte_reg[2];
                5:       uart_tx <= data_byte_reg[3];
                6:       uart_tx <= data_byte_reg[4];
                7:       uart_tx <= data_byte_reg[5];
                8:       uart_tx <= data_byte_reg[6];
                9:       uart_tx <= data_byte_reg[7];
                10:      uart_tx <= STOP_BIT;         // 停止位
                default: uart_tx <= 1'b1;
            endcase
        end

endmodule


//=============================================================================
//  ③ bt_frame —— 字节流 → 完整帧（新写）
//-----------------------------------------------------------------------------
//  S_IDLE    : 等 '$'(0x24)；命中 ⇒ fbuf[0]='$', len=1, → S_COLLECT
//  S_COLLECT : 每来一个字节：
//                · 0x24 '$'  ⇒ 重新起帧（丢掉前面半截垃圾，防止残帧串味）
//                · 0x0D '\r' ⇒ 丢弃（CRLF 容错），不计长度、不计时中断
//                · 0x0A '\n' ⇒ 断帧，frame_vld 1 拍（长度**不含** \n），→ S_IDLE
//                · len>=FRAME_MAX ⇒ 溢出：整帧丢弃，→ S_IDLE（防死锁）
//                · 其它      ⇒ fbuf[len] <= byte, len <= len+1
//              同时 gap_cnt 计数；gap_cnt==TIMEOUT_GAP ⇒ 超时断帧
//
//  ★ 三条容易做错的点（本实现已处理）：
//    1. 超时计数器**只在收到 '$' 之后**才启动（只在 S_COLLECT 计数）——
//       否则空闲时 gap_cnt 一直涨，会凭空产生空帧。
//    2. '\n' 不入缓冲 ⇒ "$PING\n" 解出长度恰好 = 5，比较逻辑干净。
//    3. 溢出必须**丢弃整帧**并回 IDLE，不能只截断——否则一帧长垃圾会把状态机卡死。
//
//  ★ 超时常数 TIMEOUT_GAP = 15190（教材 ch9 Baud_Set==4 原值）：
//      · 线速链路合理（≈3.5 个字节时间，15190 拍 = 303.8 µs）
//      · ⚠️ BLE 场景：手机 App 必须**一次发完整条 $PING\n**（模块会把这几个字节背靠背
//        从 UART 吐出，字节间隔 ≈87 µs ⇒ 304 µs 够用）。若 App 只能逐字符发
//        （每字符一个 BLE 包，间隔 ≈10 ms+），则每个字符都会被判成独立帧 ⇒
//        把本参数改为 20'd500000（10 ms）重编即可。'\n' 断帧是主判据，超时只是兜底。
//=============================================================================
module bt_frame #(
    parameter integer TIMEOUT_GAP = 15190,   // 9600→182291 / 115200→15190（教材 ch9）
    parameter integer FRAME_MAX   = 32       // 帧缓冲字节数（$VEL,500,500,300 = 16 字节，2× 余量）
)(
    input  wire                   clk,
    input  wire                   rst_n,
    input  wire [7:0]             rx_byte,
    input  wire                   rx_byte_vld,  // 1 拍脉冲

    output wire [FRAME_MAX*8-1:0] frame,        // 帧内容（组合给出；采样时机见下）
    output reg  [5:0]             frame_len,    // 帧长度（字节数，不含 \r\n）
    output reg                    frame_vld     // 1 拍脉冲
);
    // FRAME_MAX = 32 ⇒ 下标占 5 位。★ 若改 FRAME_MAX，这里要同步改。
    localparam IDX_W = 5;

    localparam [1:0] S_IDLE = 2'd0, S_COLLECT = 2'd1;

    reg  [1:0]  st;
    reg  [31:0] gap_cnt;
    reg  [5:0]  len_r;
    reg  [7:0]  fbuf [0:FRAME_MAX-1];

    // 组合拼装：纯连线，无逻辑
    reg  [FRAME_MAX*8-1:0] frame_asm;
    integer bi;
    always @(*) begin
        frame_asm = {(FRAME_MAX*8){1'b0}};
        for (bi = 0; bi < FRAME_MAX; bi = bi + 1)
            frame_asm[bi*8 +: 8] = fbuf[bi];
    end
    assign frame = frame_asm;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st        <= S_IDLE;
            gap_cnt   <= 32'd0;
            len_r     <= 6'd0;
            frame_len <= 6'd0;
            frame_vld <= 1'b0;
        end
        else begin
            frame_vld <= 1'b0;                    // 默认：脉冲只持续 1 拍

            case (st)
            //--------------------------------------------------------------
            S_IDLE: begin
                gap_cnt <= 32'd0;
                if (rx_byte_vld && (rx_byte == 8'h24)) begin     // '$'
                    fbuf[0]  <= 8'h24;
                    len_r   <= 6'd1;
                    st      <= S_COLLECT;
                end
            end
            //--------------------------------------------------------------
            S_COLLECT: begin
                if (rx_byte_vld) begin
                    gap_cnt <= 32'd0;
                    if (rx_byte == 8'h24) begin                  // '$' 重新起帧
                        fbuf[0] <= 8'h24;
                        len_r  <= 6'd1;
                    end
                    else if (rx_byte == 8'h0D) begin             // '\r' 丢弃
                        len_r <= len_r;
                    end
                    else if (rx_byte == 8'h0A) begin             // '\n' 断帧
                        frame_len <= len_r;
                        frame_vld <= 1'b1;
                        st        <= S_IDLE;
                    end
                    else if (len_r >= FRAME_MAX) begin           // 溢出：整帧丢弃
                        st <= S_IDLE;
                    end
                    else begin
                        fbuf[len_r[IDX_W-1:0]] <= rx_byte;
                        len_r                 <= len_r + 6'd1;
                    end
                end
                else if (gap_cnt >= TIMEOUT_GAP) begin           // 超时断帧
                    frame_len <= len_r;
                    frame_vld <= 1'b1;
                    st        <= S_IDLE;
                end
                else begin
                    gap_cnt <= gap_cnt + 32'd1;
                end
            end
            //--------------------------------------------------------------
            default: st <= S_IDLE;
            endcase
        end
    end

endmodule


//=============================================================================
//  ④ bt_link —— 顶层（上面 3 个子模块 + 链路级命令语义）
//     命令解码由**外部 bt_cmd2** 统一完成并回灌 i_cmd；应答由 bt_telem2 打包。
//=============================================================================
module bt_link #(
    parameter integer TIMEOUT_GAP = 15190,   // 115200 断帧超时（教材 ch9 Baud_Set=4 原值）
    parameter integer FRAME_MAX   = 32       // 帧缓冲字节数
)(
    input  wire        clk,           // 50 MHz（clk50M / Y18）
    input  wire        rst_n,         // 低有效（reset_n / B21 = S4）
    input  wire        bt_rx,         // ← 蓝牙模块 TXD（P8 针1 = H14）
    output wire        bt_tx,         // → 蓝牙模块 RXD（P8 针2 = H20）

    // ---- 遥测输入（P1/P3: 由 odom_top 的位姿驱动, 发 $TEL 给手机）----
    input  wire signed [31:0] i_x_q16,      // 位置 mm  Q16.16
    input  wire signed [31:0] i_y_q16,
    input  wire signed [31:0] i_th_q16,     // 航向 度 Q16.16 (0~360)
    input  wire        [4:0]  i_wp,         // 当前目标点号
    input  wire        [2:0]  i_st,         // 状态码
    input  wire signed [31:0] i_vx_q16,     // 速度 mm/s Q16.16（odom 输出）★ v3 新增
    input  wire signed [31:0] i_vy_q16,     // 速度 mm/s Q16.16                    ★ v3 新增
    input  wire        [7:0]  i_batt_01v,
    /*  v74：$DBG 观测帧透传（nav_ctrl.dbg_* -> 内部 bt_telem2） */
    input  wire       [2:0]  i_dbg_phase,
    input  wire       [1:0]  i_dbg_sub,
    input  wire       [2:0]  i_dbg_step,
    input  wire       [1:0]  i_dbg_near,
    input  wire       [1:0]  i_dbg_far,
    input  wire              i_dbg_unk,
    input  wire signed [15:0] i_dbg_samp,
    input  wire signed [31:0] i_dbg_odo,
    input  wire signed [31:0] i_dbg_w,
    input  wire       [1:0]  i_dbg_gear,   // 电池电压 0.1V（vbat_mon 输出）      ★ v3 新增
    // ---- 命令输入（P1/P3）★ 单一解码器：bt_cmd2 统一解码后回灌，
    //   bt_link 只负责**链路级语义**（$PING→$PONG / $STAT→立即$TEL / $TEL,0|1→遥测开关）；
    //   运动类语义交给 nav_ctrl。⇒ 消除"两套解码器并存"（旧 bt_cmd 只认 $PING，已删）
    input  wire [7:0]         i_cmd,        // 命令码（与 bt_cmd2 的 CMD_* 一致）
    input  wire signed [31:0] i_arg0,       // 参数 0（$TEL,0/1 用）
    input  wire               i_cmd_vld,    // 1 拍有效

    // ---- 应答输入（P1/P3: 由 nav_ctrl 的 resp_* 驱动，回 $ACK/$DON/$ERR）----
    input  wire               i_resp_vld,    // 1 拍: 发一条应答帧
    input  wire [3:0]         i_resp_type,   // 1..9
    input  wire [4:0]         i_resp_arg,    // 点号/错误码

    // ---- 观测口（PB 阶段顶层悬空；P0 起接遥测/显示）----
    output wire [7:0]  o_rx_byte,
    output wire        o_rx_byte_vld,
    output wire [5:0]  o_frame_len,
    output wire        o_frame_vld,   // 收到完整帧就 1 拍（可引出做 H15 诊断脚）
    output wire [FRAME_MAX*8-1:0] o_frame,  // ★整帧内容(P1: 供顶层 bt_cmd2 解析)
    output wire [7:0]  o_pong_cnt,    // 已回 PONG 次数
    output wire [15:0] o_tel_cnt,     // 已发 $TEL 帧数（观测/存活判据）
    output wire [15:0] o_resp_cnt     // 已发应答帧数
);
    wire [7:0]             rx_byte;
    wire                   rx_byte_vld;
    wire [FRAME_MAX*8-1:0] frame;
    wire [5:0]             frame_len;
    wire                   frame_vld;
    wire [7:0]             tx_byte;
    wire                   tx_send_en;
    wire                   tx_done;

    // ---- ① 接收 ----
    bt_rx_byte u_bt_rx_byte (
        .clk       (clk        ),
        .rst_n     (rst_n      ),
        .uart_rx   (bt_rx      ),
        .data_byte (rx_byte    ),
        .rx_done   (rx_byte_vld)
    );

    // ---- ③ 组帧 ----
    bt_frame #(
        .TIMEOUT_GAP (TIMEOUT_GAP),
        .FRAME_MAX   (FRAME_MAX  )
    ) u_bt_frame (
        .clk          (clk        ),
        .rst_n        (rst_n      ),
        .rx_byte      (rx_byte    ),
        .rx_byte_vld  (rx_byte_vld),
        .frame        (frame      ),
        .frame_len    (frame_len  ),
        .frame_vld    (frame_vld  )
    );

    // ---- ⑤ 应答打包 ----
    // P1/P3: 换成"单一 TX 拥有者" bt_telem2 —— 同时管 $PONG 与 $TEL,
    //        并顺带把 odom 的位姿变成**真实下游**(避免 odom 被综合优化掉)。
    // ---- ④ 链路级命令语义（命令码见 bt_cmd2：PING=1 / STAT=2 / TEL=12）----
    wire pong_req = i_cmd_vld && (i_cmd == 8'd1);   // $PING -> 回 $PONG
    wire stat_req = i_cmd_vld && (i_cmd == 8'd2);   // $STAT -> 立即回一帧 $TEL
    reg  tel_en_r;                                  // $TEL,0/1 控制的遥测开关
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)                             tel_en_r <= 1'b1;   // 上电默认开
        else if (i_cmd_vld && (i_cmd == 8'd12)) tel_en_r <= i_arg0[0];
    end
    /* ★2026-10-22 遥测 10Hz→5Hz：43B/帧 × 10Hz 在 20B MTU 下是 23 包/秒入向，
       实测会掉包 ⇒ 分片错位 ⇒ 垃圾帧（100.2V / −106.6V）与速率塌到 1-2Hz。
       降到 5Hz 入向负载减半，网页用 20Hz 图标平滑 + 外推补视觉连续性。*/
    bt_telem2 #( .PERIOD(10_000_000) ) u_bt_telem2 (
        .clk        (clk      ),
        .rst_n      (rst_n    ),
        .pong_req   (pong_req ),
        .stat_req   (stat_req ),
        .resp_vld   (i_resp_vld),
        .resp_type  (i_resp_type),
        .resp_arg   (i_resp_arg),
        .tel_en     (tel_en_r ),
        .x_q16      (i_x_q16  ),
        .y_q16      (i_y_q16  ),
        .th_q16     (i_th_q16 ),
        .wp         (i_wp     ),
        .st_in      (i_st     ),
        .vx_q16     (i_vx_q16 ),      // ★ v3：速度与电量进遥测帧
        .vy_q16     (i_vy_q16 ),
        .batt_01v   (i_batt_01v),
        .dbg_phase  (i_dbg_phase),
        .dbg_sub    (i_dbg_sub  ),
        .dbg_step   (i_dbg_step ),
        .dbg_near   (i_dbg_near ),
        .dbg_far    (i_dbg_far  ),
        .dbg_unk    (i_dbg_unk  ),
        .dbg_samp   (i_dbg_samp ),
        .dbg_odo    (i_dbg_odo  ),
        .dbg_w      (i_dbg_w    ),
        .dbg_gear   (i_dbg_gear ),
        .tx_byte    (tx_byte  ),
        .tx_send_en (tx_send_en),
        .tx_done    (tx_done  ),
        .tel_cnt    (o_tel_cnt),
        .pong_cnt   (o_pong_cnt),
        .resp_cnt   (o_resp_cnt)
    );

    // ---- ② 发送 ----
    bt_tx_byte u_bt_tx_byte (
        .clk       (clk      ),
        .rst_n     (rst_n    ),
        .data_byte (tx_byte  ),
        .send_en   (tx_send_en),
        .uart_tx   (bt_tx    ),
        .tx_done   (tx_done  )
    );

    // ---- 观测口引出 ----
    assign o_rx_byte     = rx_byte;
    assign o_rx_byte_vld = rx_byte_vld;
    assign o_frame       = frame;
    assign o_frame_len   = frame_len;
    assign o_frame_vld   = frame_vld;

endmodule
