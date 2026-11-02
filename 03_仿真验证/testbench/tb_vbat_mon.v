//=============================================================================
// tb_vbat_mon.v —— vbat_mon 行为仿真（ModelSim）
//-----------------------------------------------------------------------------
// 【验证什么】
//   1) 序列机确实按 ADC128S102 的 SPI 时序发出 CS_N / SCLK / DIN
//   2) 采到的 12 位码能被正确换算成 0.1V 单位的电压
//   3) 低电压告警阈值（LOW_01V）判对
//
// 【从机模型】按真实 ADC128S102 的时序写：
//     CS_N 低期间，每个 SCLK **下降沿** 输出一位，MSB first；
//     16 位输出帧 = 前 4 位前导 0 + 12 位转换结果。
//   ★ 与教材第 24 章原 tb（gene_DOUT task）行为一致。
//
// 【换算对照】R1=10k / R2=4.7k（本车实配）⇒ K_NUM=412, SHIFT=14（带四舍五入）
//     vbat_01v = (code * 412 + 8192) >> 14
//     code=3333 ->  84 (8.40V 满电)   code=4095 -> 103 (10.3V 满量程)
//     code=2778 ->  70 (7.00V 阈值)   code=2700 ->  68 (6.80V 告警)
//
// ⚠️ tb 的 $display 一律 ASCII（控制台编码不可控，中文会乱码）。
//=============================================================================
`timescale 1ns/1ps

module tb_vbat_mon;

    reg clk;
    reg rst_n;

    wire adc_cs_n;
    wire adc_din;
    wire adc_sclk;
    reg  adc_dout;

    wire [7:0]  vbat;
    wire        low;
    wire [11:0] raw;
    wire [15:0] conv_cnt;

    integer err;
    integer pass;

    // ---- DUT ----
    vbat_mon #(
        .DIV_PARAM (13),
        .TRIG_DIV  (2000),        // 40us @50MHz（真实 2_500_000，仿真加速）
        .R1_OHM    (10000),
        .R2_OHM    (4700),        // ★ 本车实配：10k / 4.7k
        .LOW_01V   (70)           // 7.0V 告警（7.4V 2S 锂电）
    ) dut (
        .clk        (clk       ),
        .rst_n      (rst_n     ),
        .adc_cs_n   (adc_cs_n  ),
        .adc_din    (adc_din   ),
        .adc_dout   (adc_dout  ),
        .adc_sclk   (adc_sclk  ),
        .o_vbat_01v (vbat      ),
        .o_vbat_low (low       ),
        .o_raw      (raw       ),
        .o_conv_cnt (conv_cnt  )
    );

    // ---- 50 MHz ----
    initial clk = 1'b0;
    always #10 clk = ~clk;

    // ---------------------------------------------------------------
    //  ADC128S102 从机模型
    // ---------------------------------------------------------------
    reg [11:0] inj;          // 注入的转换结果
    reg [15:0] shift;        // 正在串出的 16 位帧
    reg        din_seen;     // 抓 DIN 上的 ADD2 位（诊断）

    initial begin
        inj       = 12'd0;
        shift     = 16'd0;
        adc_dout  = 1'b0;
        din_seen  = 1'b0;
    end

    // CS 下降沿：装载 16 位帧（4 位前导 0 + 12 位数据）
    always @(negedge adc_cs_n)
        shift <= {4'b0, inj};

    // SCLK 下降沿：串出下一位（仅在 CS 低期间）
    always @(negedge adc_sclk) begin
        if (!adc_cs_n) begin
            adc_dout <= shift[15];
            shift    <= {shift[14:0], 1'b0};
            din_seen <= adc_din;      // 顺带记录 DIN（不做判据，只看活动）
        end
    end

    // ---------------------------------------------------------------
    //  测例
    // ---------------------------------------------------------------
    task check;
        input [11:0] code;
        input [7:0]  exp_v;
        input        exp_low;
        begin
            inj = code;
            @(posedge dut.conv_done);   // 本次转换完成（用了新的 inj）
            @(posedge clk);             // 等输出寄存器落定
            #1;
            if (vbat !== exp_v) begin
                $display("FAIL code=%0d  vbat=%0d (exp %0d)", code, vbat, exp_v);
                err = err + 1;
            end
            else if (low !== exp_low) begin
                $display("FAIL code=%0d  low=%0d (exp %0d)", code, low, exp_low);
                err = err + 1;
            end
            else begin
                pass = pass + 1;
                $display("PASS code=%0d -> %0d.%0d V  low=%0d  raw=%0d",
                          code, vbat/10, vbat%10, low, raw);
            end
        end
    endtask

    initial begin
        err    = 0;
        pass   = 0;
        rst_n  = 1'b0;
        #201;
        rst_n  = 1'b1;
        #500;

        $display("---- tb_vbat_mon start ----");

        // 10k/4.7k ⇒ K_NUM=412, SHIFT=14；v01v = (code*412 + 8192) >> 14
        // code=3333 -> 84  (8.40V 满电)
        check(12'd3333, 8'd84,  1'b0);
        // code=4095 -> 103 (10.3V 满量程)
        check(12'd4095, 8'd103, 1'b0);
        // code=3175 -> 80  (8.00V)
        check(12'd3175, 8'd80,  1'b0);
        // code=2778 -> 70  (7.00V，等于阈值 ⇒ 不告警，< 才告警)
        check(12'd2778, 8'd70,  1'b0);
        // code=2700 -> 68  (6.80V ⇒ 告警)
        check(12'd2700, 8'd68,  1'b1);
        // code=2381 -> 60  (6.00V ⇒ 告警)
        check(12'd2381, 8'd60,  1'b1);
        // code=0    -> 0   (未接分压 ⇒ 0.0V，告警)
        check(12'd0,    8'd0,   1'b1);

        $display("---- conv_cnt = %0d ----", conv_cnt);
        if (err == 0) $display("==== tb_vbat_mon: ALL PASS (%0d/%0d) ====", pass, pass);
        else          $display("==== tb_vbat_mon: %0d FAILED ====", err);
        $finish;
    end

endmodule
