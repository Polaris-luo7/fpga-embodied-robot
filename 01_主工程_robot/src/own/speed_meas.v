//=============================================================================
// speed_meas.v — 编码器速度测量（T 法主通道 + 静默平滑衰减上界）
//------------------------------------------------------------------------------
// 输入 period/period_valid/period_stb 来自 quad_decoder（四倍频事件间隔，
// 单位 = clk 周期数）。
//
// 主通道（T 法，低速高精度）：
//   speed_T = CLK_FREQ_HZ / period（计数/秒），顺序除法器（div_const）实现，
//   商 > 32767 钳位；带符号 = dir × speed_T。
//
// 静默衰减（关键设计，防陈旧反馈极限环）：
//   仅用 T 法原值会在"低速事件稀疏/换向过零"时把旧速度挂住十~百 ms
//   （台阶式反馈 → 停车振荡）。因此另算一条**静默上界**：
//     bound = CLK_FREQ_HZ / elapsed   （elapsed = 距上次事件的 clk 数）
//   物理依据：若持续无新事件，则真实速度必然已低于 CLK/elapsed。
//   输出 speed = speed_T 与 bound 取小（带符号同向）。
//   性质：匀速时 bound 恒 ≥ speed_T（=CLK/interval），**永不误钳**；
//         减速/过零静默时 bound 随 elapsed 连续下降 → 平滑到 0，
//         无台阶、无陈旧平台。
//   bound 每个 tick（1 kHz）重算一次（32 拍除法 << 50000 拍窗口）。
//
// 调试通道（M 法）：
//   speed_m = 每 tick 的位置增量（计数/ms），仅供观测对比。
//
// 修订记录（本项目无 git 历史，以文件内注释为准）：
//   2026-10-01  修正 1：bound 结果改为"完成沿锁存"（新增 b_hold），
//                      不再组合直接取用 b_quot —— 后者在 32 拍除法窗口内是 0/部分位。
//               修正 2：period_valid=0（停转超时）时 T 法幅值按 0 处理；
//                      原设计把该信号注释为"仅调试输出使用"，未参与归零。
//               依据：rtl\motor_loop\probe_TICK_2008.log（tick 拍实测，30 点 bbusy 全 0、bnd 全非 0）
//                     与 probe_FIX_2010.log（修正后）。
//               结果：tb_speed_loop ALL PASS（修正前 1 ERROR）；P1~P4 数值不变。
//=============================================================================
`timescale 1ns/1ps

module speed_meas #(
    parameter integer CLK_FREQ_HZ  = 50_000_000,
    parameter integer PERIOD_WIDTH = 24,
    parameter integer CNT_MAX      = 32767
)(
    input  wire                      clk,
    input  wire                      rst_n,
    input  wire                      tick,          // 1 kHz（bound 重算 + M 法）
    input  wire [PERIOD_WIDTH-1:0]   period,
    input  wire                      period_valid,  // 1=有效且未停转超时；=0 时 T 法幅值作废
    input  wire                      period_stb,    // period 更新单拍
    input  wire                      dir,           // quad_decoder 方向：1=正
    input  wire signed [31:0]        position,      // 四倍频位置计数（M 法）
    output wire signed [15:0]        speed,         // 计数/秒（带符号，主输出）
    output reg  signed [15:0]        speed_m        // 计数/ms（M 法调试）
);

    // ===================== T 法：period → speed_r =====================
    reg                     t_pend;
    reg  [PERIOD_WIDTH-1:0] t_pend_val;
    reg                     t_dir;
    wire                    t_busy;
    wire [31:0]             t_quot;
    reg [15:0]              speed_r;
    reg                     speed_r_sign;   // 1=正

    div_const #(.DIVIDEND(CLK_FREQ_HZ), .DIVW(PERIOD_WIDTH)) u_div_t (
        .clk(clk), .rst_n(rst_n),
        .start(t_pend && !t_busy),
        .divisor(t_pend_val),
        .quot(t_quot), .busy(t_busy)
    );

    reg t_busy_d;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) t_busy_d <= 1'b0;
        else        t_busy_d <= t_busy;
    end
    wire t_done = t_busy_d && !t_busy;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            t_pend      <= 1'b0;
            t_pend_val  <= {PERIOD_WIDTH{1'b0}};
            t_dir       <= 1'b1;
            speed_r     <= 16'd0;
            speed_r_sign<= 1'b1;
        end else begin
            if (period_stb) begin
                t_pend     <= 1'b1;
                t_pend_val <= period;
                t_dir      <= dir;
            end else if (t_pend && !t_busy) begin
                t_pend <= 1'b0;      // 已被 div_const 采样（同拍 start=1）
            end
            if (t_done) begin
                if (t_quot > CNT_MAX) speed_r <= CNT_MAX[15:0];
                else                  speed_r <= t_quot[15:0];
                speed_r_sign <= t_dir;
            end
        end
    end

    // ===================== 静默上界：elapsed → bound =====================
    reg  [PERIOD_WIDTH-1:0] elapsed;      // 距上次事件（饱和计数）
    wire                    b_busy;
    wire [31:0]             b_quot;
    reg  [31:0]             b_hold;       // bound 结果锁存：仅在除法完成沿更新
    reg                     b_valid;      // bound 结果有效
    reg                     b_busy_d;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) elapsed <= {PERIOD_WIDTH{1'b0}};
        else if (period_stb) elapsed <= {PERIOD_WIDTH{1'b0}};
        else if (!elapsed[PERIOD_WIDTH-1])   // 饱和
            elapsed <= elapsed + 1'b1;
    end

    div_const #(.DIVIDEND(CLK_FREQ_HZ), .DIVW(PERIOD_WIDTH)) u_div_b (
        .clk(clk), .rst_n(rst_n),
        .start(tick && !b_busy && (elapsed != 0)),
        .divisor(elapsed),
        .quot(b_quot), .busy(b_busy)
    );

    // 【修正 1】div_const 在 start 拍会把 quot 清 0，且 busy 期间只有部分位有效。
    // 若组合直接取用 b_quot，任何落在那 32 拍除法窗口内的读取都会拿到 0/垃圾值，
    // speed 会在"tick 之后的 32 拍"内假性归零（实测：TB 打印与 tick 同相，每次读到 0）。
    // 故必须锁存"完成沿"的值，对外只暴露 b_hold。
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            b_busy_d <= 1'b0;
            b_valid  <= 1'b0;
            b_hold   <= 32'd0;
        end else begin
            b_busy_d <= b_busy;
            if (b_busy_d && !b_busy) begin   // 除法完成沿：quot 此时才是最终值
                b_hold  <= b_quot;
                b_valid <= 1'b1;
            end
        end
    end

    // ===================== 输出合成：speed_T 与 bound 取小 =====================
    // 【修正 2】period_valid=0 表示停转超时（>T_TIMEOUT 无事件）⇒ T 法结果已作废，
    // 幅值必须按 0 处理。原设计把它注释成"仅调试输出使用"、未参与归零，
    // 导致停转/极低速时陈旧 period 仍被换算成速度喂给闭环。
    wire [31:0] abs_r_u = period_valid ? {16'd0, speed_r} : 32'd0;
    wire        clamp   = b_valid && (b_hold < abs_r_u);
    wire [31:0] mag_o   = clamp ? b_hold : abs_r_u;
    wire signed [16:0] mag_s = {1'b0, mag_o[15:0]};
    assign speed = speed_r_sign ? mag_s[15:0] : -mag_s[15:0];

    // ===================== M 法：每 tick 差分（调试） =====================
    reg signed [31:0] pos_prev;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pos_prev <= 32'sd0;
            speed_m  <= 16'sd0;
        end else if (tick) begin
            speed_m  <= position - pos_prev;   // 计数/ms
            pos_prev <= position;
        end
    end

endmodule
