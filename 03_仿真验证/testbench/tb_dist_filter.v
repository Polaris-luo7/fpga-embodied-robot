//=============================================================================
// tb_dist_filter.v —— dist_filter 行为仿真
//-----------------------------------------------------------------------------
// 【验证】
//   1) 触发周期正确（TRIG_DIV 拍一次 start）
//   2) 3 采中值：单发野值（5、200）被剔除
//   3) 窗口未满时不输出（filt_vld 不置位）
//
// 【注入方式】同步任务式：send_sample 等 trig_start 后恰好注入 1 个样本。
//   ★ 不能用"#固定等待 + 后台进程"—— 等待时长若与触发周期同拍会产生
//     注入次数漂移（首版踩过：中值结果整体错位一个样本）。
// ⚠️ $display 一律 ASCII。
//=============================================================================
`timescale 1ns/1ps

module tb_dist_filter;

    localparam integer TRIG_DIV = 200;        // 加速：4us 触发一次

    reg clk = 0, rst_n = 0;
    wire trig_start;
    reg  [15:0] dist_cm = 0;
    reg         valid = 0;
    wire [15:0] filt_cm;
    wire        filt_vld;
    wire [15:0] conv_cnt;

    integer err = 0, pass = 0;

    dist_filter #(.TRIG_DIV(TRIG_DIV)) dut (
        .clk(clk), .rst_n(rst_n),
        .trig_start(trig_start),
        .dist_cm(dist_cm), .valid(valid),
        .filt_cm(filt_cm), .filt_vld(filt_vld), .conv_cnt(conv_cnt)
    );

    always #10 clk = ~clk;

    // 每次调用注入一个样本：等触发 → 模拟 1us 测量耗时 → valid 一拍
    task send_sample;
        input [15:0] d;
        begin
            @(posedge trig_start);
            #1000;
            dist_cm = d; valid = 1; #20; valid = 0;
        end
    endtask

    task check;
        input [15:0] exp;
        begin
            @(posedge filt_vld);
            #1;
            if (filt_cm !== exp) begin
                $display("FAIL filt=%0d exp=%0d", filt_cm, exp);
                err = err + 1;
            end else begin
                pass = pass + 1;
                $display("PASS filt=%0d cm", filt_cm);
            end
        end
    endtask

    initial begin
        $display("---- tb_dist_filter start ----");
        #201; rst_n = 1;
        #500;

        // ---- 窗口未满（<3 个样本）：不应输出 ----
        send_sample(16'd30);  #300;
        if (conv_cnt !== 0) begin $display("FAIL 窗口未满就输出"); err = err + 1; end
        else begin $display("PASS 窗口未满不输出"); pass = pass + 1; end

        send_sample(16'd5);   #300;
        if (conv_cnt !== 0) begin $display("FAIL 窗口未满就输出2"); err = err + 1; end
        else begin $display("PASS 窗口未满不输出2"); pass = pass + 1; end

        // ---- 窗口满：中值序列（野值 5 / 200 均应被剔除）----
        // {31,5,30} -> 30      (5 是野值)
        send_sample(16'd31);  check(16'd30);
        // {32,31,5} -> 31
        send_sample(16'd32);  check(16'd31);
        // {200,32,31} -> 32    (200 是野值)
        send_sample(16'd200); check(16'd32);
        // {30,200,32} -> 32
        send_sample(16'd30);  check(16'd32);
        // {30,30,200} -> 30
        send_sample(16'd30);  check(16'd30);
        // {31,30,30} -> 30
        send_sample(16'd31);  check(16'd30);

        $display("---- conv_cnt=%0d ----", conv_cnt);
        if (err == 0) $display("==== tb_dist_filter: ALL PASS (%0d) ====", pass);
        else          $display("==== tb_dist_filter: %0d FAILED ====", err);
        $finish;
    end

endmodule
