//=============================================================================
// adc128s102.v —— 板载 ADC128S102 驱动（8 通道 12bit SAR ADC，SPI）
//-----------------------------------------------------------------------------
// 【来源】教材《第 24 章 线性序列机与串行接口 ADC 驱动设计与验证》
//         原码 ch24_ADCadc128S102.rar : src/adc128s102.v（作者：小梅哥）
//   ★ 串行时序逻辑**一字未改**（位级时序靠仿真验证，不自作聪明简化）。
//     本文件只做三件事：① 端口名统一小写 ② 补中文注释
//                       ③ 把硬件出处与 .cst 对应关系写清楚
//
// 【硬件出处 —— 2026-10-14 用 PyMuPDF 坐标法实查原理图第 15 页 15_ADDA.SchDoc】
//   U43 = ADC128S102CIMT       8 通道 12bit，Vref = VA = VCC3P3_VA = 3.3V
//   U43.1  CS_N   ← FPGA ADC_CS_N   (球号 M6，FPGA 输出)
//   U43.14 DIN    ← FPGA ADC_SADDR  (球号 M5，FPGA 输出)  通道控制字
//   U43.15 DOUT   → FPGA ADC_SDAT   (球号 P6，FPGA 输入)  转换结果
//   U43.16 SCLK   ← FPGA ADC_SCLK   (球号 N5，FPGA 输出)
//   8 个模拟输入各经 22R 串阻 + 1nF 到地，接到 P5（Header 8X2）排针：
//         P5 第 1 脚  = Analog_In0 → ADC_IN0      ← 电池分压接这里
//         P5 第 3 脚  = Analog_In1 → ADC_IN1
//         P5 第 5 脚  = Analog_In2 → ADC_IN2
//         P5 第 7 脚  = Analog_In3 → ADC_IN3
//         P5 第 9/11/13/15 脚 = ADC_IN4..7
//         P5 左列(第 2/4/6/8/10/12/14/16 脚) **全部是 AGND**
//   ⚠️ 板上 12V(VCC12) **没有**任何现成分压到 ADC ⇒ 必须外部分压后才能测电池。
//
// 【时序】CS_N 拉低 → 16 个 SCLK 周期输出 16 位 DOUT：
//          前 4 位为前导（0），后 12 位为转换结果，MSB first；
//          DIN 的 bit15..13 = 通道地址 ADD2..ADD0。
//        本驱动在 SCLK_GEN_CNT = 10,12,...,32 共 12 个上升沿采样 DOUT。
//        ★ 从机在 SCLK **下降沿**更新 DOUT ⇒ 上升沿采到的是刚更新的位，
//          相位恰好对齐（教材原码 + 原 tb 已验证，本次照搬）。
//
// 【SCLK 频率】= fclk / (2 × DIV_PARAM)。教材取 13 ⇒ 50MHz/26 = 1.923 MHz。
//          远低于 ADC 上限 16MHz；接 P5 排针用杜邦线时低速更稳 ⇒ 沿用 13。
//          一次转换 = 34 个 SCLK 周期 ≈ 17.7 µs。
//
// 【接口约定】En_Conv 为**单拍脉冲**，触发一次转换；转换期间 en 锁存，
//          期间再来脉冲无效。Conv_Done 在转换完成当拍给出 1 拍高脉冲，
//          同时 Data 更新。ADC_State = CS_N（转换中低、空闲高）。
//
// 【版本】2026-10-14 WorkBuddy（任务四·电量监测）
//   本项目无 git ⇒ 变更记录写在此文件头。
//=============================================================================
`timescale 1ns/1ps

module adc128s102(
    input  wire        clk,          // 50 MHz（与本工程其他逻辑同域）
    input  wire        rst_n,        // 低有效

    input  wire [2:0]  channel,      // 通道选择 0..7
    output reg  [11:0] data,         // 转换结果

    input  wire        en_conv,      // 单拍脉冲：使能一次转换
    output reg         conv_done,    // 转换完成（1 拍脉冲）
    output wire        adc_state,    // 1=空闲 / 0=转换中
    input  wire [7:0]  div_param,    // 分频：SCLK = fclk/(2*div_param)

    output reg         adc_sclk,     // ← ADC SCLK
    output reg         adc_cs_n,     // ← ADC CS_N（低有效）
    input  wire        adc_dout,     // → 来自 ADC DOUT
    output reg         adc_din       // ← ADC DIN（通道控制字）
);
    reg [2:0]  r_channel;   // 转换开始时锁存的通道号
    reg [11:0] r_data;      // 移位接收缓冲

    reg [7:0]  div_cnt;     // 分频计数器
    reg        sclk2x;      // 2 倍 SCLK 的节拍使能

    reg [5:0]  sclk_gen_cnt;// 序列机计数器 0..33
    reg        en;          // 转换进行中标志

    // ---- 锁存通道号（转换过程中不变）----
    always @(posedge clk or negedge rst_n)
        if (!rst_n)        r_channel <= 3'd0;
        else if (en_conv)  r_channel <= channel;
        else               r_channel <= r_channel;

    // ---- 转换进行中标志 ----
    always @(posedge clk or negedge rst_n)
        if (!rst_n)          en <= 1'b0;
        else if (en_conv)    en <= 1'b1;
        else if (conv_done)  en <= 1'b0;
        else                 en <= en;

    // ---- 2 倍 SCLK 节拍分频 ----
    always @(posedge clk or negedge rst_n)
        if (!rst_n)
            div_cnt <= 8'd0;
        else if (en) begin
            if (div_cnt == (div_param - 8'd1)) div_cnt <= 8'd0;
            else                               div_cnt <= div_cnt + 8'd1;
        end
        else
            div_cnt <= 8'd0;

    always @(posedge clk or negedge rst_n)
        if (!rst_n)                                     sclk2x <= 1'b0;
        else if (en && (div_cnt == (div_param - 8'd1)))  sclk2x <= 1'b1;
        else                                             sclk2x <= 1'b0;

    // ---- 序列机计数器 ----
    always @(posedge clk or negedge rst_n)
        if (!rst_n)
            sclk_gen_cnt <= 6'd0;
        else if (sclk2x && en) begin
            if (sclk_gen_cnt == 6'd33) sclk_gen_cnt <= 6'd0;
            else                       sclk_gen_cnt <= sclk_gen_cnt + 6'd1;
        end
        else
            sclk_gen_cnt <= sclk_gen_cnt;

    // ---- 线性序列机：SCLK / CS_N / DIN 波形生成 + DOUT 采样 ----
    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin
            adc_sclk <= 1'b1;
            adc_cs_n <= 1'b1;
            adc_din  <= 1'b1;
        end
        else if (en) begin
            if (sclk2x) begin
                case (sclk_gen_cnt)
                    6'd0 : adc_cs_n <= 1'b0;                              // 拉低片选
                    6'd1 : begin adc_sclk <= 1'b0; adc_din <= 1'b0; end   // 第 1 个空位
                    6'd2 : adc_sclk <= 1'b1;
                    6'd3 : adc_sclk <= 1'b0;
                    6'd4 : adc_sclk <= 1'b1;
                    6'd5 : begin adc_sclk <= 1'b0; adc_din <= r_channel[2]; end // ADD2
                    6'd6 : adc_sclk <= 1'b1;
                    6'd7 : begin adc_sclk <= 1'b0; adc_din <= r_channel[1]; end // ADD1
                    6'd8 : adc_sclk <= 1'b1;
                    6'd9 : begin adc_sclk <= 1'b0; adc_din <= r_channel[0]; end // ADD0

                    // 上升沿采样 DOUT，循环左移进 12 位缓冲（MSB first）
                    6'd10,6'd12,6'd14,6'd16,6'd18,6'd20,6'd22,6'd24,6'd26,6'd28,6'd30,6'd32:
                        begin adc_sclk <= 1'b1; r_data <= {r_data[10:0], adc_dout}; end
                    6'd11,6'd13,6'd15,6'd17,6'd19,6'd21,6'd23,6'd25,6'd27,6'd29,6'd31:
                        begin adc_sclk <= 1'b0; end

                    6'd33: adc_cs_n <= 1'b1;                                  // 收工抬 CS
                    default: adc_cs_n <= 1'b1;
                endcase
            end
        end
        else begin
            adc_cs_n <= 1'b1;
        end

    // ---- 转换完成：输出结果 + 1 拍脉冲 ----
    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin
            data      <= 12'd0;
            conv_done <= 1'b0;
        end
        else if (en && sclk2x && (sclk_gen_cnt == 6'd33)) begin
            data      <= r_data;
            conv_done <= 1'b1;
        end
        else begin
            data      <= data;
            conv_done <= 1'b0;
        end

    assign adc_state = adc_cs_n;

endmodule
