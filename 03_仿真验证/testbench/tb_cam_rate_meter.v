//=============================================================================
// tb_cam_rate_meter.v — self-checking testbench for cam_rate_meter
//------------------------------------------------------------------------------
// 说明：真实默认窗口是 1 秒，仿真跑 1 秒太慢，所以这里把窗口缩短到 1 ms
//       （GATE_CYCLES = 50_000 @50MHz），期望值按"窗口内周期数"换算：
//         T1: PCLK 100 MHz, 1ms 窗口 → 期望 100_000 个 PCLK 周期
//         T2: PCLK  25 MHz, 1ms 窗口 → 期望  25_000 个
//         T3: 停帧（VSYNC 恒低）        → 期望 frame_cnt = 0
//         T4: valid 是单拍脉冲，次数与测量次数一致
//       VSYNC 周期 50 us → 1ms 窗口内期望 20 帧（±1，窗口边界量化）
//=============================================================================
`timescale 1ns/1ps

module tb_cam_rate_meter;

    // ---- DUT 配置：窗口 1 ms / 空隙 10 us（真实使用用默认参数即可）----
    localparam integer CLK_HZ = 50_000_000;
    localparam integer GATE   = 50_000;   // 1 ms
    localparam integer GAP    = 500;      // 10 us

    reg  clk   = 0;
    reg  rst_n = 0;
    reg  pclk  = 0;
    reg  vsync = 0;

    wire [31:0] frame_cnt;
    wire [31:0] pclk_cnt;
    wire        valid;

    cam_rate_meter #(
        .CLK_FREQ_HZ (CLK_HZ),
        .GATE_CYCLES (GATE  ),
        .GAP_CYCLES  (GAP   )
    ) dut (
        .clk(clk), .rst_n(rst_n), .pclk(pclk), .vsync(vsync),
        .frame_cnt(frame_cnt), .pclk_cnt(pclk_cnt), .valid(valid)
    );

    // ---- 时钟：clk 50 MHz；pclk 半周期可变（先 5ns=100MHz，后 20ns=25MHz）----
    always #10 clk = ~clk;
    integer pclk_half = 5;
    always #(pclk_half) pclk = ~pclk;

    // ---- VSYNC：50 us 周期；vsync_en=0 时恒低 ----
    reg vsync_en = 1'b0;
    initial begin
        vsync = 1'b0;
        forever begin
            #25_000;
            if (vsync_en) begin vsync = 1'b1; #25_000; vsync = 1'b0; end
            else          begin vsync = 1'b0; end
        end
    end

    // ---- valid 统计：事件数 与 高电平拍数 应相等（证明确是单拍脉冲）----
    integer ev_cnt = 0;   // valid 事件数
    integer vh_cnt = 0;   // valid 为高的 clk 拍数
    always @(posedge clk) begin
        if (valid) begin ev_cnt <= ev_cnt + 1; vh_cnt <= vh_cnt + 1; end
    end

    // ---- 等一次测量完成（valid 单拍脉冲）----
    task wait_valid;
        begin
            @(posedge clk);
            while (!valid) @(posedge clk);
            repeat (4) @(posedge clk);   // 等寄存器更新稳定
        end
    endtask

    integer err = 0;
    integer ev_before;

    initial begin
        rst_n = 0;
        repeat (10) @(posedge clk);
        rst_n = 1;
        vsync_en = 1;                       // 开帧（50 us 周期）

        // ---------------- T1: PCLK 100 MHz ----------------
        wait_valid;                         // 第 1 次可能受复位边界影响，丢弃
        wait_valid;                         // 取第 2 次
        $display("[T1 PCLK100M] frame_cnt=%0d (expect 20±1)  pclk_cnt=%0d (expect 100000±8)",
                 frame_cnt, pclk_cnt);
        if (frame_cnt > 21 || frame_cnt < 19) begin
            $display("   !! FAIL T1 frame_cnt"); err = err + 1; end
        if (pclk_cnt > 100_008 || pclk_cnt < 99_992) begin
            $display("   !! FAIL T1 pclk_cnt");  err = err + 1; end

        // ---------------- T2: PCLK 25 MHz ----------------
        pclk_half = 20;                     // 40ns 周期 = 25 MHz
        wait_valid;
        $display("[T2 PCLK25M ] frame_cnt=%0d (expect 20±1)  pclk_cnt=%0d (expect 25000±8)",
                 frame_cnt, pclk_cnt);
        if (frame_cnt > 21 || frame_cnt < 19) begin
            $display("   !! FAIL T2 frame_cnt"); err = err + 1; end
        if (pclk_cnt > 25_008 || pclk_cnt < 24_992) begin
            $display("   !! FAIL T2 pclk_cnt");  err = err + 1; end

        // ---------------- T3: 停帧，PCLK 仍在跑 ----------------
        vsync_en = 0;
        repeat (1000) @(posedge clk);       // 等 VSYNC 稳定为低
        wait_valid;
        $display("[T3 NOSYNC  ] frame_cnt=%0d (expect 0)     pclk_cnt=%0d (expect 25000±8)",
                 frame_cnt, pclk_cnt);
        if (frame_cnt !== 32'd0) begin
            $display("   !! FAIL T3 frame_cnt"); err = err + 1; end
        if (pclk_cnt > 25_008 || pclk_cnt < 24_992) begin
            $display("   !! FAIL T3 pclk_cnt (停帧不应影响 PCLK 计数)"); err = err + 1; end

        // ---------------- T4: valid 为单拍脉冲，且每次窗口一次 ----------------
        ev_before = ev_cnt;
        repeat (3) wait_valid;
        $display("[T4 VALID   ] 3 个窗口内 valid 事件=%0d (expect 3)", ev_cnt - ev_before);
        if ((ev_cnt - ev_before) !== 3) begin
            $display("   !! FAIL T4 valid 次数"); err = err + 1; end
        if (vh_cnt !== ev_cnt) begin
            $display("   !! FAIL T4 valid 不是单拍脉冲 (vh=%0d ev=%0d)", vh_cnt, ev_cnt);
            err = err + 1; end
        else
            $display("            valid 高电平拍数=%0d = 事件数 → 确认为单拍脉冲", vh_cnt);

        if (err == 0) $display("\n==== ALL TESTS PASS ====");
        else          $display("\n==== %0d TEST(S) FAILED ====", err);
        $finish;
    end

    initial begin
        #20_000_000;                        // 20 ms 看门狗
        $display("!! TIMEOUT");
        $finish;
    end

endmodule
