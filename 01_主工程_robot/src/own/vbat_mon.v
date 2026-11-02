//=============================================================================
// vbat_mon.v —— 电池电压监测（任务四·创新加分项）
//-----------------------------------------------------------------------------
// 【作用】周期性读板载 ADC128S102 的通道 0，把 12bit 码换算成电池电压，
//         以 **0.1V 为单位** 输出给蓝牙遥测帧（$TEL 的 batt 字段）。
//
// 【为什么要分压】ACG720 的 DC 座 J4 输入是 12V（VCC12），
//   但板上 **没有任何** 把 12V 引到 ADC 的通路 ⇒ 必须在外面做电阻分压，
//   再把分压中点接到 P5 排针第 1 脚（Analog_In0）。
//
//   电池+ ──[R1]──┬──→ P5 第 1 脚 (Analog_In0)
//                 │
//                [R2]
//                 │
//   电池- ────────┴──→ P5 第 2 脚 (AGND)          ← 与 FPGA 共地
//
//   ⚠️ 分压后的电压必须 **< 3.3V**（ADC 的 Vref = VA = 3.3V，超了会超量程甚至
//      打坏输入级）。推荐取值见下表（本车电池 = **7.4V 2S 锂电，满电 8.4V**）：
//
//     电池        R1(上)   R2(下)   满电 8.4V 分压到  满量程可测   分辨率
//     ----------------------------------------------------------------
//     7.4V 2S     10kΩ    4.7kΩ     2.69V            10.3V       ~2.5mV  ← 本车实配
//     7.4V 2S     10kΩ    3.3kΩ     2.08V            13.3V       ~3.3mV
//     7.4V 2S     10kΩ    2.2kΩ     1.52V            18.3V       ~4.5mV
//
//   ★ 本车定案 **R1=10kΩ / R2=4.7kΩ**（余量足、精度最高）。
//   ★ 分压点与 GND 之间可并联 **0.1µF 电容**（板上 1nF 之外再加一个），
//     把电机 PWM 带来的纹波滤掉，读数更稳。
//   ★ 低电压告警：2S 锂电取 **7.0V**（保护板通常 6.0~6.4V 才断）。
//
// 【换算】Vbat = Code × 3300mV / 4096 / k,   k = R2/(R1+R2)
//         本模块把 (3300/4096/100/k) 预先折成 **定点乘法**（Q14），
//         避免常量除法（本项目的时序杀手，见交接文档 §5.1 教训 4）：
//             o_vbat_01v = ((adc_code * K_NUM) + 8192) >> 14      ← 加半 LSB 四舍五入
//         K_NUM = 13200 * (R1+R2) / (100*R2)
//         代入 **10k/4.7k（本车定案值）** ⇒ K_NUM = 13200×14700/470000 = 412
//         校验：Code=3333 (=8.40V) ⇒ (3333×412+8192)>>14 = 84  ⇒ 8.4V ✓
//               Code=2778 (=7.00V) ⇒ 70  ⇒ 7.0V（告警阈值）✓
//               Code=4095          ⇒ 103 ⇒ 10.3V（满量程）✓
//
// 【更新率】TRIG_DIV 拍触发一次转换。默认 2_500_000 @50MHz = 20Hz，
//         远高于遥测帧率（10Hz）⇒ 每帧电压都是新的。
//         一次转换本身只要 17.7µs，20Hz 触发绰绰有余。
//
// 【端口】全部同步于 clk（50MHz，与 bt_link / bt_telem2 同域 ⇒ 零跨域）。
//
// 【版本】2026-10-14 WorkBuddy（任务四·电量监测）
//=============================================================================
`timescale 1ns/1ps

module vbat_mon #(
    parameter integer DIV_PARAM = 13,           // SCLK = 50MHz/(2*13) = 1.923MHz（教材原值）
    parameter integer TRIG_DIV  = 2_500_000,    // 触发间隔（拍）@50MHz = 20Hz
    parameter integer R1_OHM    = 10_000,       // 上分压电阻（接正极）
    parameter integer R2_OHM    = 4_700,        // 下分压电阻（接 AGND）
    parameter integer LOW_01V   = 70            // 低电压告警阈值（0.1V 单位）＝7.0V（2S 锂电）
)(
    input  wire        clk,
    input  wire        rst_n,

    // ---- ADC 物理引脚（顶层直连，.cst 见 robot/src/ov5640_ddr3_tft.cst）----
    output wire        adc_cs_n,      // M6
    output wire        adc_din,       // M5
    input  wire        adc_dout,      // P6
    output wire        adc_sclk,      // N5

    // ---- 结果 ----
    output reg  [7:0]  o_vbat_01v,    // 电池电压，0.1V 单位（如 126 = 12.6V）
    output reg         o_vbat_low,    // 1 = 低于 LOW_01V（可接 LED 慢闪 / $EVT 上报）
    output reg  [11:0] o_raw,         // 原始 ADC 码（诊断用）
    output reg  [15:0] o_conv_cnt     // 累计转换次数（存活判据：>0 说明 ADC 在跑）
);
    // ---- 定点换算系数（Q14）----
    //   K = 3300mV × 16384 / (4096 × 100 × k) = 13200 × (R1+R2) / (100 × R2)
    //   代入 10k/2.2k：13200 × 12200 / 220000 = 732  ⇒ 0.044678 V/code ✓
    localparam integer K_NUM   = 13200 * (R1_OHM + R2_OHM) / (100 * R2_OHM);
    localparam integer K_SHIFT = 14;
    localparam integer CH_SEL  = 0;      // ★ 固定采通道 0 = P5 第 1 脚

    wire [11:0] adc_data;
    wire        conv_done;
    wire        adc_state;               // 1 = 空闲

    reg  [31:0] trig_cnt;
    reg         en_conv;

    // ---- 周期触发（只在 ADC 空闲时给脉冲）----
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            trig_cnt <= 32'd0;
            en_conv  <= 1'b0;
        end
        else begin
            en_conv <= 1'b0;                       // 默认单拍
            if (trig_cnt >= (TRIG_DIV - 1)) begin
                trig_cnt <= 32'd0;
                en_conv  <= adc_state;             // 到点触发（20Hz）；只在 ADC 空闲时给
            end
            else begin
                trig_cnt <= trig_cnt + 32'd1;
            end
        end
    end

    // ---- 结果换算 + 告警 ----
    //   ★ 加半个 LSB 再右移 = 四舍五入（不移会系统性偏小 1 个码，见交接文档 §5.1 教训 4）
    wire [31:0] v01v_calc = ((adc_data * K_NUM) + (1 << (K_SHIFT-1))) >> K_SHIFT;   // 0.1V 单位

    //  ★2026-10-22 样本有效性过滤：分压接线松动 / SPI 偶发失败会让读数抽风到 0.0V
    //    （实测 8.1V ↔ 0.0V 反复跳）。3.0V~9.0V 视为有效样本，其余丢弃并保持上次值。
    wire sample_ok = (v01v_calc > 32'd30) && (v01v_calc < 32'd90);
    reg  [7:0] low_cnt, ok_cnt;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            o_vbat_01v <= 8'd0;
            o_vbat_low <= 1'b0;          /* ★上电默认"正常"，由连续低样本才触发（原默认 1 是隐患）*/
            o_raw      <= 12'd0;
            o_conv_cnt <= 16'd0;
            low_cnt    <= 8'd0;
            ok_cnt     <= 8'd0;
        end
        else if (conv_done) begin
            o_conv_cnt <= o_conv_cnt + 16'd1;
            if (sample_ok) begin
                o_raw      <= adc_data;
                o_vbat_01v <= (v01v_calc > 32'd255) ? 8'd255 : v01v_calc[7:0];
                /*  ★连续计数去抖：连续 100 次真低才跳闸（约 0.1~1s）；连续 20 次正常才解除。
                    单次 0.0V 抽风不再切电机（实测误跳闸根因）。*/
                if (v01v_calc < LOW_01V) begin
                    low_cnt <= low_cnt + 8'd1;
                    ok_cnt  <= 8'd0;
                    if (low_cnt >= 8'd100) o_vbat_low <= 1'b1;
                end else begin
                    ok_cnt  <= ok_cnt + 8'd1;
                    low_cnt <= 8'd0;
                    if (ok_cnt >= 8'd20) o_vbat_low <= 1'b0;
                end
            end
            /* sample_ok = 0（抽风）⇒ 保持原值与原告警态，不参与判断 */
        end
    end

    // ---- ADC 驱动实例（教材 ch24 原码，逻辑未改）----
    adc128s102 u_adc128s102 (
        .clk         (clk        ),
        .rst_n       (rst_n      ),
        .channel     (CH_SEL[2:0]),
        .data        (adc_data   ),
        .en_conv     (en_conv    ),
        .conv_done   (conv_done  ),
        .adc_state   (adc_state  ),
        .div_param   (DIV_PARAM[7:0]),
        .adc_sclk    (adc_sclk   ),
        .adc_cs_n    (adc_cs_n   ),
        .adc_dout    (adc_dout   ),
        .adc_din     (adc_din    )
    );

endmodule
