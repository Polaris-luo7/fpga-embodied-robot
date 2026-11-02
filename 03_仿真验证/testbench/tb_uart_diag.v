//=============================================================================
// tb_uart_diag.v — 验证 selftest_ir 新增的「编码器诊断 UART 输出」
//-----------------------------------------------------------------------------
// 【验证目标】
//   ① uart_tx 到底有没有在发（不是恒高）
//   ② 报文格式是不是 "P1=00000 P2=00000 P3=00000 P4=00000\r\n"
//
// 【加速技巧】
//   源码里首帧要等 200 ms（ugap 计到 9,999,999 才发）。
//   为了不让仿真跑 200 ms（50 MHz 下 1000 万拍，很慢），
//   这里用 `force dut.ugap = 24'd9_999_999` **直接跳过等待**，再 release。
//   ⇒ 只跑 5 ms 就能收到一整帧（37 字节 × 10 bit × 8680 ns ≈ 3.2 ms）。
//
// 【REF】2026-10-12 WorkBuddy（任务二编码器标定排查）
//=============================================================================
`timescale 1ns/1ps

module tb_uart_diag;

    // ---- 50 MHz 时钟 ----
    reg clk = 1'b0;
    always #10 clk = ~clk;                 // 20 ns 周期 = 50 MHz

    // ---- DUT 输入 ----
    reg        rst_n  = 1'b0;
    reg  [3:0] key    = 4'hF;              // 全高 = 未按下
    reg        ir_in  = 1'b1;              // 红外空闲高
    reg  [3:0] enc_a  = 4'h0;
    reg  [3:0] enc_b  = 4'h0;

    // ---- DUT 输出 ----
    wire [3:0] m_in1, m_in2, m_pwm, led;
    wire       stdby, uart_tx;

    selftest_ir dut (
        .sys_clk (clk    ),
        .rst_n   (rst_n  ),
        .key     (key    ),
        .ir_in   (ir_in  ),
        .enc_a   (enc_a  ),
        .enc_b   (enc_b  ),
        .m_in1   (m_in1  ),
        .m_in2   (m_in2  ),
        .m_pwm   (m_pwm  ),
        .stdby   (stdby  ),
        .led     (led    ),
        .uart_tx (uart_tx)
    );

    //=====================================================================
    // UART 软解码（115200 8N1 @50MHz ⇒ 434 拍/位）
    //   LSB 先发；起始位后等 217 拍（半位）采样 bit0，再每 434 拍采样一位
    //=====================================================================
    //=====================================================================
    // UART 软解码（115200 ⇒ 8680.6 ns/bit）
    //   用 negedge 触发 + busy 防重入；采样点用绝对延时对准"位中心"
    //   （第一版按拍计数对齐有 bug，解出乱码；改成本方式）
    //=====================================================================
    localparam real BIT_NS = 8680.6;
    reg [7:0]  rx_sh;
    reg        busy = 1'b0;
    reg [7:0]  linebuf [0:63];
    integer    nchar = 0;
    integer    i;

    always @(negedge uart_tx) begin
        if (!busy) begin
            busy = 1'b1;
            #(BIT_NS * 1.5);                 // 起始位中线 -> bit0 中线
            rx_sh[0] = uart_tx;  #(BIT_NS);
            rx_sh[1] = uart_tx;  #(BIT_NS);
            rx_sh[2] = uart_tx;  #(BIT_NS);
            rx_sh[3] = uart_tx;  #(BIT_NS);
            rx_sh[4] = uart_tx;  #(BIT_NS);
            rx_sh[5] = uart_tx;  #(BIT_NS);
            rx_sh[6] = uart_tx;  #(BIT_NS);
            rx_sh[7] = uart_tx;  #(BIT_NS * 0.5);
            if (nchar < 63) begin linebuf[nchar] = rx_sh; nchar = nchar + 1; end
            busy = 1'b0;
        end
    end

    // 统计 uart_tx 跳变（证明"有没有在动"）
    integer nedge = 0;
    always @(negedge uart_tx) nedge = nedge + 1;

    //=====================================================================
    initial begin
        #200 rst_n = 1'b1;
        #1000;
        $display("=== 跳过 200ms 等待：force ugap ===");
        force dut.ugap = 24'd9_999_999;
        #1000;
        release dut.ugap;
        #5_000_000;                                   // 5 ms，足够发完一整帧

        $display("=== uart_tx 下降沿次数 = %0d ===", nedge);
        $display("=== 解出字符数 = %0d ===", nchar);
        if (nchar > 0) begin
            $write("内容: ");
            for (i = 0; i < nchar; i = i + 1) begin
                if      (linebuf[i] == 8'h0D) $write("<CR>");
                else if (linebuf[i] == 8'h0A) $write("<LF>\n");
                else if (linebuf[i] >= 8'h20 && linebuf[i] < 8'h7F) $write("%c", linebuf[i]);
                else $write("[%02h]", linebuf[i]);
            end
            $display("");
        end else begin
            $display("!!! 没收到任何字符 !!!");
        end
        $finish;
    end

endmodule
