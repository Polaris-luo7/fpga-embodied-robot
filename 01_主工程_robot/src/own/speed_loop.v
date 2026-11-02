//=============================================================================
// speed_loop.v — 单电机速度闭环顶层：编码器 + 测速 + PI + PWM 串接
//------------------------------------------------------------------------------
// 数据流（全部硬逻辑，1 kHz 节拍由 tick_gen 产生）：
//
//   enc_a/enc_b ─► quad_decoder ─► position / period / dir
//                    │                 │
//                    │                 ▼
//                    │            speed_meas (T法: CLK/period → 计数/秒)
//                    │                 │ speed
//   target ──────────┴──► pi_controller ┴──► out(带符号)
//                          ▲  en=tick&run      │
//                     tick_gen(1kHz)           ▼
//                                    pwm_gen (|out|→duty, sign→dir_req)
//                                          │
//                                     pwm_out / dir_out ─► 驱动板
//
// 工程约束对应：#1 节拍硬逻辑；#2 解码硬逻辑并行；#3~#6 见 pi_controller；
//               #12 kp/ki 为输入端口，可由软核在线下发。
// 参数单位：target/speed = 计数/秒（每轮 1440 计数）；
//           duty/out = 0..PWM_PERIOD(4000)（4000=100%）。
//
// 修订记录（本项目无 git 历史，以文件内注释为准）：
//   2026-10-01  修正 3：新增停机带 STOP_BAND。|target|<=DEADZONE 且 |speed|<=STOP_BAND 时
//                      判为停机：清零积分 + 强制 duty_cmd=0（防止零速命令下
//                      积分残留 → 残余驱动 → 低速极限环/爬行）。
//                      影响面：目标非零时 stopped 恒 0 ⇒ P1~P4 行为不变（已实测核对）。
//               依据：probe_TICK_2008.log 显示 P5 过零后稳定在 ±1000 计数/s 极限环，
//                     duty≈-2×speed（P 项主导）+ 测量滞后，非积分饱和、非假速度。
//               结果：tb_speed_loop ALL PASS（修正前 1 ERROR）。
//=============================================================================
`timescale 1ns/1ps

module speed_loop #(
    parameter integer CLK_FREQ_HZ  = 50_000_000,
    parameter integer PWM_PERIOD   = 4000,   // 12.5 kHz @50MHz
    parameter integer DEAD_CYCLES  = 100,    // 换向死区 2 µs
    parameter integer T_TIMEOUT    = 100_000, // 停转判定 2 ms（50MHz）
                                    // ← 关键：低速时事件间隔长，若超时阈值取
                                    //   大值（100ms），换向/停车过程中 speed 会
                                    //   挂着陈旧值十~百 ms，形成极限环振荡。
                                    //   2ms 静默即判 0 → 可测最低速度 = 50e6/2ms
                                    //   = 500 计数/s（≈0.1 m/s 轮速，够用）；
    // PI 参数（复位值；kp/ki 端口可在运行中覆盖）
    parameter integer DEADZONE     = 3,
    parameter integer OUT_MAX      = 4000,
    parameter integer I_LIMIT      = 4000,
    parameter integer MIN_DUTY     = 100,
    parameter integer KP_SHIFT     = 3,
    parameter integer KI_SHIFT     = 4,
    // 【修正 3】停机带：|target| <= DEADZONE 且 |speed| <= STOP_BAND ⇒ 判停机
    parameter integer STOP_BAND    = 600
)(
    input  wire               clk,
    input  wire               rst_n,
    // 编码器
    input  wire               enc_a,
    input  wire               enc_b,
    input  wire               cnt_clr,     // 位置计数清零（软核下发）
    // 控制
    input  wire               run,         // 闭环使能
    input  wire               clr_i,       // 积分清零
    input  wire signed [15:0] target,      // 目标速度（计数/秒）
    input  wire signed [15:0] kp,
    input  wire signed [15:0] ki,
    // 执行器
    output wire               pwm_out,
    output wire               dir_out,
    // 观测/调试（可接 LED、GAO、寄存器回读）
    output wire signed [31:0] position,
    output wire signed [15:0] speed,       // T 法主测速（计数/秒）
    output wire signed [15:0] speed_m,     // M 法差分（计数/ms，调试）
    output wire signed [15:0] duty,        // PI 输出（带符号）
    output wire               period_valid_dbg,
    output wire               tick_dbg
);
    // ---- 1 kHz 硬件时基 ----
    wire tick;
    tick_gen #(
        .CLK_FREQ_HZ (CLK_FREQ_HZ),
        .TICK_FREQ_HZ(1000)
    ) u_tick (
        .clk(clk), .rst_n(rst_n), .tick(tick)
    );
    assign tick_dbg = tick;

    // ---- 四倍频解码 + T 法测周 ----
    wire                      dir;
    wire [23:0]               period;
    wire                      period_valid, period_stb;
    quad_decoder #(
        .CNT_WIDTH    (32),
        .PERIOD_WIDTH (24),
        .T_TIMEOUT    (T_TIMEOUT)
    ) u_dec (
        .clk(clk), .rst_n(rst_n),
        .enc_a(enc_a), .enc_b(enc_b), .cnt_clr(cnt_clr),
        .position(position), .dir(dir),
        .period(period), .period_valid(period_valid), .period_stb(period_stb)
    );
    assign period_valid_dbg = period_valid;

    // ---- 测速（T 法主通道） ----
    speed_meas #(
        .CLK_FREQ_HZ (CLK_FREQ_HZ),
        .PERIOD_WIDTH(24)
    ) u_speed (
        .clk(clk), .rst_n(rst_n),
        .tick(tick),
        .period(period), .period_valid(period_valid), .period_stb(period_stb),
        .dir(dir), .position(position),
        .speed(speed), .speed_m(speed_m)
    );

    // ---- 停机带判定（【修正 3】）----
    // 目标为零（|target| <= DEADZONE）且实测已落入 STOP_BAND ⇒ 判停机：
    //   清零积分 + 强制输出 0。避免"零速命令下积分残留 → 残余驱动 → 低速极限环/爬行"。
    // 影响面：目标非零时 stopped 恒为 0，故 P1~P4 行为不变。
    wire stopped = (target <=  DEADZONE) && (target >= -DEADZONE) &&
                   (speed  <=  STOP_BAND) && (speed  >= -STOP_BAND);

    // ---- 定点 PI（1 kHz 使能） ----
    wire signed [15:0] out;
    wire signed [31:0] integ_dbg;
    pi_controller #(
        .DEADZONE(DEADZONE), .OUT_MAX(OUT_MAX), .I_LIMIT(I_LIMIT),
        .MIN_DUTY(MIN_DUTY), .KP_SHIFT(KP_SHIFT), .KI_SHIFT(KI_SHIFT)
    ) u_pi (
        .clk(clk), .rst_n(rst_n),
        .en(tick && run), .clr_i(clr_i || !run || stopped),
        .target(target), .actual(speed),
        .kp(kp), .ki(ki),
        .out(out), .integ_out(integ_dbg)
    );
    wire signed [15:0] duty_cmd = stopped ? 16'sd0 : out;
    assign duty = duty_cmd;

    // ---- PWM + 换向死区 ----
    wire [15:0] mag = duty_cmd[15] ? (-duty_cmd) : duty_cmd;   // 幅值
    wire [15:0] mag_clamped = (mag > PWM_PERIOD) ? PWM_PERIOD[15:0] : mag;
    pwm_gen #(
        .CLK_FREQ_HZ (CLK_FREQ_HZ),
        .PWM_PERIOD  (PWM_PERIOD),
        .DEAD_CYCLES (DEAD_CYCLES)
    ) u_pwm (
        .clk(clk), .rst_n(rst_n),
        .run(run),
        .dir_req(duty_cmd[15]),   // 符号位：1=负/反转，0=正/反转 —— 见下注
        .duty(mag_clamped),
        .pwm_out(pwm_out),
        .dir_out(dir_out)
    );
    // 注：dir_req 取 duty_cmd 的符号位 —— <0 时 dir_req=1 表示反转。
    //     驱动板方向脚极性以接线实测为准，若相反只需在板级对 dir_out 取反，
    //     不影响闭环（反馈经 quad_decoder 自动闭合）。
endmodule
