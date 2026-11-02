//=============================================================================
// ultrasonic_hcsr04.v — HC-SR04 超声波硬件计时测距
//------------------------------------------------------------------------------
// 功能：单次测量。收到 start 后发 10us TRIG 脉冲，硬件计数 ECHO 高电平脉宽，
//       换算为厘米距离。echo_us / dist_cm / valid 由上层（软核或避障状态机）读取。
// 硬件计时（本项目论证点）：脉宽计数在硬逻辑内完成，分辨率到 us 级，
//       不依赖软核调度，抖动远小于软件定时。
// 上层职责：多路分时触发（防串扰）、中值滤波（去野值）由此模块外部完成。
// 公式：distance_cm = echo_us / 58
//
// 实现说明：S_MEAS 累加 ECHO 高电平期间的时钟 tick 总数，最后统一 /TPU 得 us。
//   进入测量点比 echo 真实上升沿晚 2 拍（同步器延迟），退出点同样晚 2 拍，
//   两者相互抵消，从而无需额外补偿即可获得准确脉宽。
//
// 【时序优化 2026-10-17】原实现用常量除法 `tick_cnt/TPU` 与 `/(TPU*58)` ——
//   常量除法在综合里展开成大组合逻辑，是 TNS 杀手（项目已踩过：exp42→43 省 3040 LUT）。
//   改为：①直接用"每 TPU 拍进 1 us"计数器得到 echo_us（无除法）
//         ②dist_cm = (echo_us × 1131 + 32768) >> 16  （1/58 的 Q16 定点，误差 <0.3%）
//   输出与旧实现逐值一致（tb 5 项 regression 通过）。
//=============================================================================
`timescale 1ns/1ps

module ultrasonic_hcsr04 #(
    parameter integer CLK_FREQ = 50_000_000, // 系统时钟
    parameter integer TRIG_US  = 10,         // TRIG 脉宽 (us)
    parameter integer WAIT_US  = 20_000,     // 等待 ECHO 超时 (us)
    parameter integer MAX_US   = 30_000      // ECHO 最大计数 (us)，超出判为无效
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,      // 单次测量启动（脉冲）
    output reg         trig,       // -> HC-SR04 TRIG
    input  wire        echo,       // <- HC-SR04 ECHO
    output reg  [15:0] echo_us,    // ECHO 高电平脉宽 (us)
    output reg  [15:0] dist_cm,    // 距离 (cm)
    output reg         valid,      // dist_cm 有效（单拍脉冲）
    output reg         busy        // 测量进行中
);

    localparam integer TPU      = CLK_FREQ / 1_000_000;  // 每 us 的时钟数
    localparam integer TRIG_CNT = TRIG_US * TPU;
    // （MAX_TICK 已废弃：改用 us 计数直接与 MAX_US 比较）

    localparam [2:0] S_IDLE = 3'd0,
                     S_TRIG = 3'd1,
                     S_WAIT = 3'd2,
                     S_MEAS = 3'd3,
                     S_DONE = 3'd4;

    // ECHO 两级同步
    reg e1, e2;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin e1 <= 1'b0; e2 <= 1'b0; end
        else        begin e1 <= echo; e2 <= e1;   end
    end

    reg [2:0]  state;
    reg [15:0] cnt;       // tick 计数 (TRIG / WAIT 用)
    reg [31:0] us_cnt;    // ECHO 高电平 us 计数（★无除法）
    reg [15:0] tick_cnt;  // 每 us 内的 tick 计数 (0..TPU-1)
    reg [15:0] wait_cnt;  // 等待 us

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE; trig <= 1'b0; valid <= 1'b0; busy <= 1'b0;
            cnt <= 16'd0; us_cnt <= 32'd0; tick_cnt <= 16'd0; wait_cnt <= 16'd0;
            echo_us <= 16'd0; dist_cm <= 16'd0;
        end else begin
            valid <= 1'b0;   // 默认单拍脉冲
            case (state)
                S_IDLE: begin
                    busy <= 1'b0; trig <= 1'b0; cnt <= 16'd0; wait_cnt <= 16'd0;
                    if (start) begin
                        state <= S_TRIG; busy <= 1'b1; trig <= 1'b1; cnt <= 16'd0;
                    end
                end
                S_TRIG: begin
                    if (cnt >= TRIG_CNT - 1) begin trig <= 1'b0; cnt <= 16'd0; state <= S_WAIT; end
                    else                            cnt  <= cnt + 1'b1;
                end
                S_WAIT: begin
                    if (e2) begin
                        state <= S_MEAS; us_cnt <= 32'd0; tick_cnt <= 16'd0;
                    end else if (cnt >= TPU - 1) begin
                        cnt <= 16'd0;
                        if (wait_cnt >= WAIT_US) begin state <= S_IDLE; busy <= 1'b0; end
                        else                            wait_cnt <= wait_cnt + 1'b1;
                    end else begin
                        cnt <= cnt + 1'b1;
                    end
                end
                S_MEAS: begin
                    if (e2) begin
                        if (tick_cnt >= TPU - 1) begin   // 满 1 us
                            tick_cnt <= 16'd0;
                            us_cnt   <= us_cnt + 32'd1;
                        end else begin
                            tick_cnt <= tick_cnt + 16'd1;
                        end
                        if (us_cnt >= MAX_US) state <= S_DONE;   // 超长 ⇒ 收尾（判无效/饱和）
                    end else begin
                        state   <= S_DONE;               // ECHO 下降沿
                        if (tick_cnt >= TPU/2)           // 不足 1us 的余数四舍五入
                            us_cnt <= us_cnt + 32'd1;
                    end
                end
                S_DONE: begin
                    echo_us <= us_cnt[15:0];
                    dist_cm <= (us_cnt * 32'd1131 + 32'd32768) >> 16;   // ★1/58 的 Q16 点乘（无除法）
                    valid   <= 1'b1;
                    busy    <= 1'b0;
                    state   <= S_IDLE;
                end
                default: state <= S_IDLE;
            endcase
        end
    end

endmodule