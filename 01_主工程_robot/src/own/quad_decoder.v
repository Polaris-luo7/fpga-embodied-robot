//=============================================================================
// quad_decoder.v — 增量式正交编码器 硬件四倍频解码 + T 法测周
//------------------------------------------------------------------------------
// 功能：对编码器 A/B 两相正交信号做四倍频（4x）解码，输出带符号位置计数。
//       每个 A/B 状态跳变（一个电周期共 4 个边沿）贡献 ±1 计数。
// 优势（本项目论证点）：纯硬件并行解码，不占 CPU，不丢脉冲；软核只读结果寄存器。
// 设计要点：
//   - 两级同步器消除跨时钟域亚稳态（编码器信号相对 clk 异步）
//   - 用 4bit 状态转移（a_prev,b_prev,a_now,b_now）判方向，避免漏边沿
//   - signed 计数，正转递增、反转递减
//
// T 法测周（与 M 法并存，低速高精度）：
//   - period = 相邻两次有效解码事件之间的 clk 周期数（四倍频边沿为事件）
//   - 低速时 period 大，量化误差 1 clk 相对比例小 → 低速精度优于 M 法
//   - 高速时用 M 法（对 position 按固定窗口差分），低速用 T 法，二者互补
//   - 换算：每秒计数值 = CLK_FREQ_HZ / period；
//           轮速(rpm) = 60 * CLK_FREQ_HZ / (period * 每轮四倍频计数)
//           本项目电机 12PPR × 4 × 减速1:30 = 1440 计数/轮 rev
//   - period_valid：有有效测量且未停转超时（超时阈值 T_TIMEOUT），
//     停转时该位拉低 → 速度按 0 处理，避免把陈旧 period 送给闭环
//   - period_valid==0 时 period 值仍保留最后一次测量，仅供查看
//   - cnt_clr 只清位置计数，不影响 T 法测量（两者并存、互不干扰）
//=============================================================================
`timescale 1ns/1ps

module quad_decoder #(
    parameter CNT_WIDTH    = 32,
    parameter PERIOD_WIDTH = 24,      // 周期计数位宽，上限 2^24-1 个 clk
    parameter T_TIMEOUT    = 5000000  // 停转判定：50MHz 下 ≈100ms；须 < 2^PERIOD_WIDTH
)(
    input  wire                     clk,        // 系统时钟
    input  wire                     rst_n,      // 异步复位，低有效
    input  wire                     enc_a,      // 编码器 A 相
    input  wire                     enc_b,      // 编码器 B 相
    input  wire                     cnt_clr,    // 同步清零（软核下发）
    output wire signed [CNT_WIDTH-1:0] position, // 四倍频位置计数（带符号）
    output reg                      dir,        // 最近方向：1=正转 0=反转
    output reg  [PERIOD_WIDTH-1:0]  period,     // T法：上两次事件间隔（clk 周期数）
    output reg                      period_valid, // 1=测量有效且未停转超时
    output reg                      period_stb    // 单拍脉冲：period 更新当拍
);

    //--------------------------------------------------
    // 1. 两级同步（防亚稳态）
    //--------------------------------------------------
    reg a_meta, a_sync;
    reg b_meta, b_sync;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            a_meta <= 1'b0; a_sync <= 1'b0;
            b_meta <= 1'b0; b_sync <= 1'b0;
        end else begin
            a_meta <= enc_a; a_sync <= a_meta;
            b_meta <= enc_b; b_sync <= b_meta;
        end
    end

    // 上一拍状态（用于状态转移判定）
    reg a_d, b_d;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            a_d <= 1'b0; b_d <= 1'b0;
        end else begin
            a_d <= a_sync; b_d <= b_sync;
        end
    end

    //--------------------------------------------------
    // 2. 四倍频增量判定（状态转移表）
    //    正转序列: 00->01->11->10->00
    //    反转序列: 00->10->11->01->00
    //--------------------------------------------------
    reg signed [1:0] delta;
    always @(*) begin
        case ({a_d, b_d, a_sync, b_sync})
            4'b0001, 4'b0111, 4'b1110, 4'b1000: delta =  2'sd1; // 正转
            4'b0010, 4'b1011, 4'b1101, 4'b0100: delta = -2'sd1; // 反转
            default:                              delta =  2'sd0; // 无跳变/非法跳变
        endcase
    end

    //--------------------------------------------------
    // 3. 位置计数
    //--------------------------------------------------
    reg signed [CNT_WIDTH-1:0] pos_r;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            pos_r <= {CNT_WIDTH{1'b0}};
        else if (cnt_clr)
            pos_r <= {CNT_WIDTH{1'b0}};
        else
            pos_r <= pos_r + delta;
    end

    // 最近方向
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            dir <= 1'b1;
        else if (delta != 2'sd0)
            dir <= (delta > 2'sd0);
    end

    //--------------------------------------------------
    // 4. T 法测周：相邻两次有效事件的 clk 周期数
    //    事件 = 四倍频解码 delta != 0
    //--------------------------------------------------
    wire tick = (delta != 2'sd0);
    reg [PERIOD_WIDTH-1:0] t_cnt;   // 距上次事件的周期计数
    reg                    t_armed; // 已收到至少一个事件，可开始计间隔

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            t_cnt        <= {PERIOD_WIDTH{1'b0}};
            t_armed      <= 1'b0;
            period       <= {PERIOD_WIDTH{1'b0}};
            period_valid <= 1'b0;
            period_stb   <= 1'b0;
        end else begin
            period_stb <= 1'b0;
            if (tick) begin
                if (t_armed) begin
                    period       <= t_cnt;
                    period_stb   <= 1'b1;
                    period_valid <= 1'b1;
                end
                t_cnt   <= {{(PERIOD_WIDTH-1){1'b0}}, 1'b1}; // 本拍算 1 个周期
                t_armed <= 1'b1;
            end else if (t_cnt != {PERIOD_WIDTH{1'b1}}) begin
                t_cnt <= t_cnt + 1'b1;
                if ((t_cnt + 1'b1) >= T_TIMEOUT)  // 超时无事件 → 判停转
                    period_valid <= 1'b0;
            end
        end
    end

    assign position = pos_r;

endmodule