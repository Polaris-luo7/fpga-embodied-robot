//=============================================================================
// cam_rate_meter.v — 摄像头帧率 / PCLK 频率测量（任务一验收实测数据来源）
//------------------------------------------------------------------------------
// 用途：把摄像头的 VSYNC 与 PCLK 接进来，输出"每个测量窗口内的帧数"和
//       "每个测量窗口内的 PCLK 周期数"。默认参数下窗口 = 1 秒，所以这两个
//       数就是 **帧率(fps)** 与 **PCLK 频率(Hz)**。
//
// 读法（上板）：
//   - GAO 在线逻辑分析仪：把 frame_cnt / pclk_cnt / valid 挂上去，valid 单拍脉冲
//     触发采样（两秒一次即可）；
//   - 或把 frame_cnt 低 8 位接到板载 LED / 数码管观察变化；
//   - 接软核后也可作为只读寄存器读。
//
// 设计要点（与芯路恒论坛设计实例 `camera_param_get`，tid=29112 的差别）：
//   论坛原实现对 PCLK 先做 256 分频再在 Clk 域计数，量化误差 256 Hz；
//   本模块改为**在 PCLK 域直接计数**，再用"锁存值 + 翻转握手位"跨时钟回送，
//   量化误差降到 ±1 个 PCLK 周期（84 MHz 下约 0.0000012%），
//   且不要求 PCLK 与 Clk 谁快谁慢。
//
// 时序关系：
//   1) Clk 域产生 gate：高 GATE_CYCLES 拍（测量窗口），低 GAP_CYCLES 拍（回送空隙）；
//   2) PCLK 域把 gate 两级同步，窗口内累加 PCLK 周期数；窗口关闭时把最终值
//      锁存到 pcnt_hold，并翻转 cap_tgl；
//   3) Clk 域同步 cap_tgl 并检测跳变 → 此时 pcnt_hold 已稳定（空隙内不变），
//      更新 pclk_cnt 并打一拍 valid；
//   4) frame_cnt 在 Clk 域窗口关闭当拍锁存。
//   → 空隙必须足够长，让握手走完（GAP_CYCLES 默认 1 ms，远大于所需）。
//
// ⚠️ 边界：窗口边界处可能少计 1 个 VSYNC 沿 / 1 个 PCLK 周期（±1 量化），
//    验收取多窗口平均即可；这是整数周期计数的固有性质。
// ⚠️ 本模块 rst_n 在 PCLK 域作异步复位，属"测量/调试"级用法；若要严格 CDC
//    复位，需为 PCLK 域单独做复位同步器。
//=============================================================================
`timescale 1ns/1ps

module cam_rate_meter #(
    parameter integer CLK_FREQ_HZ = 50_000_000,          // Clk 的已知频率（整数 Hz）
    parameter integer GATE_CYCLES = CLK_FREQ_HZ,         // 测量窗口：默认 1 秒
    parameter integer GAP_CYCLES  = CLK_FREQ_HZ / 1000,  // 窗口间空隙：默认 1 ms
    parameter integer CNT_WIDTH   = 32                   // PCLK 计数位宽（32 位可到 4.29 GHz）
)(
    input  wire                    clk,       // 已知频率的时钟（如板上 50 MHz）
    input  wire                    rst_n,     // 异步复位，低有效
    input  wire                    pclk,      // 摄像头 PCLK（与 clk 异步）
    input  wire                    vsync,     // 摄像头 VSYNC（与 clk 异步）
    output reg  [31:0]             frame_cnt, // 每个窗口内的帧数（默认 1s → 即 fps）
    output reg  [CNT_WIDTH-1:0]    pclk_cnt,  // 每个窗口内的 PCLK 周期数（默认 1s → 即 Hz）
    output reg                     valid      // 一次测量完成，单拍脉冲
);

    localparam integer TOTAL_CYCLES = GATE_CYCLES + GAP_CYCLES;

    //--------------------------------------------------
    // 1. Clk 域：测量窗口 gate
    //--------------------------------------------------
    reg [31:0] sec_cnt;
    reg        gate;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sec_cnt <= 32'd0;
            gate    <= 1'b0;
        end else begin
            sec_cnt <= (sec_cnt >= TOTAL_CYCLES - 1) ? 32'd0 : (sec_cnt + 32'd1);
            gate    <= (sec_cnt < GATE_CYCLES);
        end
    end

    //--------------------------------------------------
    // 2. Clk 域：VSYNC 两级同步 + 上升沿检测 + 窗口内数帧
    //--------------------------------------------------
    reg v1, v2, v3;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) {v1, v2, v3} <= 3'b000;
        else        {v1, v2, v3} <= {vsync, v1, v2};
    end
    wire vsync_rise = v2 & ~v3;

    reg [31:0] frm_acc;
    reg        gate_d;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            frm_acc   <= 32'd0;
            frame_cnt <= 32'd0;
            gate_d    <= 1'b0;
        end else begin
            gate_d <= gate;
            if (!gate && gate_d) begin              // 窗口关闭：锁存本窗口帧数
                frame_cnt <= frm_acc;
                frm_acc   <= 32'd0;
            end else if (gate && vsync_rise) begin  // 窗口内：数 VSYNC 上升沿
                frm_acc <= frm_acc + 32'd1;
            end
        end
    end

    //--------------------------------------------------
    // 3. PCLK 域：窗口内数 PCLK 周期，窗口关闭时锁存 + 翻转握手位
    //--------------------------------------------------
    reg g1, g2, gd;
    reg [CNT_WIDTH-1:0] pcnt;
    reg [CNT_WIDTH-1:0] pcnt_hold;
    reg                 cap_tgl;

    always @(posedge pclk or negedge rst_n) begin
        if (!rst_n) begin
            g1 <= 1'b0; g2 <= 1'b0; gd <= 1'b0;
            pcnt      <= {CNT_WIDTH{1'b0}};
            pcnt_hold <= {CNT_WIDTH{1'b0}};
            cap_tgl   <= 1'b0;
        end else begin
            g1 <= gate;                 // gate 是 Clk 域长电平，两级同步安全
            g2 <= g1;
            gd <= g2;

            if (g2) pcnt <= pcnt + 1'b1;            // 窗口内累加
            else    pcnt <= {CNT_WIDTH{1'b0}};      // 窗口外清零（为下一次准备）

            if (gd && !g2) begin                    // 窗口关闭当拍：锁存最终值
                pcnt_hold <= pcnt;                  // 注意：此时 pcnt 尚未清零
                cap_tgl   <= ~cap_tgl;              // 通知 Clk 域"有新的计数"
            end
        end
    end

    //--------------------------------------------------
    // 4. Clk 域：同步握手位，检测跳变 → 取回计数并打 valid
    //--------------------------------------------------
    reg t1, t2;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin t1 <= 1'b0; t2 <= 1'b0; end
        else        begin t1 <= cap_tgl; t2 <= t1; end
    end
    wire cap_new = t1 ^ t2;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pclk_cnt <= {CNT_WIDTH{1'b0}};
            valid    <= 1'b0;
        end else begin
            valid <= cap_new;                        // 单拍脉冲
            if (cap_new) pclk_cnt <= pcnt_hold;      // 此时 pcnt_hold 已稳定
        end
    end

endmodule
