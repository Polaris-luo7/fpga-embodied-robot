//=============================================================================
// imu_attitude.v -- 定点互补滤波姿态解算（roll / pitch / yaw）
//-----------------------------------------------------------------------------
// 【算法】一阶互补滤波，逐样本两步：
//   ① 陀螺积分：angle_g = angle + gyro_raw * GYRO_Q / DT_DIV
//   ② 加速度修正：angle = angle_g + ((angle_acc - angle_g) >> ALPHA_SHIFT)
//   加速度角度用 cordic_atan2 两级算出：
//       第一级 CORDIC(az, ay)          -> roll_acc  = atan2(ay, az)
//                                       -> mag1      = A_n*sqrt(ay^2+az^2)
//       第二级 CORDIC(mag1, -ax*A_n)   -> pitch_acc = atan2(-ax, sqrt(ay^2+az^2))
//   ⚠ 第二级的两个输入必须"同尺度"。第一级的 mag 含 CORDIC 增益 A_n，
//     若第二个输入不乘 A_n，atan2 的比值就被破坏 —— 这在自校验里表现为
//     pitch 恒 ≈ 0（详见 gen_cordic_const.py 的注释与交接记录）。

// 【定点格式（写死）】
//   角度状态 : Q16.16，单位 = 度。45 度 = 2949120。
//   陀螺标度 : GYRO_Q = 65536 / 16.4 = 3996.09 -> 取 3996
//              （16.4 LSB/(deg/s) 对应 GYRO_CONFIG = 0x18，即 ±2000 deg/s）
//              校验：gyro_raw=164(=10deg/s) -> 164*3996/100 = 6553 -> 0.09999 度/样本
//                    100 样本 = 9.999 度 ✓
//   采样周期 : dt = 1/DT_DIV 秒。DT_DIV=100 对应真机 100 Hz；
//              仿真里可用 1000（1 kHz）加快收敛，**必须与 ctrl 的 PERIOD_MS 匹配**。
//   互补系数 : 有效时间常数 tau = dt * 2^ALPHA_SHIFT。
//              dt=10ms, ALPHA_SHIFT=5 -> tau = 0.32 s（默认）
//
// 【限幅】积分必须限幅（工程规约第 4 条）：roll/pitch ±90 度，yaw ±180 度。
//
// 【yaw 说明（占位，不是完成品）】
//   yaw 目前是**纯陀螺积分 + 可配置零偏**，没有磁力计/编码器航向修正，
//   所以会缓慢漂移，必须整机静止标定 GYRO_Z_BIAS 后才能用。
//   任务一只要求 roll/pitch（DoD 项 5），yaw 按规约先给占位。
//   任务二"原地旋转用陀螺航向闭环"（规约第 8 条）时再补标定流程。
//=============================================================================
`timescale 1ns / 1ps

module imu_attitude #(
    parameter integer ALPHA_SHIFT = 5,       // 加速度修正右移位数
    parameter integer GYRO_Q      = 4775,    // gyro_raw -> Q16.16 度/秒
                                             // ★2026-10-21 实测标定 +19.5%（3996→4775）：
                                             //   四点实测（手转90/电机90/135/180/360）真实/IMU ≈ 1.19 恒定，
                                             //   高度可疑为采样率 ≈120Hz 而代码按 100Hz 积分。
    parameter integer DT_DIV      = 100,     // 采样率（Hz）：dt = 1/DT_DIV
    parameter signed [31:0] ANG_LIMIT = 32'sd5898240,    // ±90 度（roll/pitch）
    parameter signed [31:0] YAW_LIMIT = 32'sd11796480,   // ±180 度（yaw）
    parameter signed [31:0] GYRO_Z_BIAS = 32'sd0         // yaw 零偏（待标定）
)(
    input  wire               clk,
    input  wire               rst_n,
    input  wire               sample_valid,   // 单拍：新样本就绪
    input  wire signed [15:0] acc_x,
    input  wire signed [15:0] acc_y,
    input  wire signed [15:0] acc_z,
    input  wire signed [15:0] gyro_x,
    input  wire signed [15:0] gyro_y,
    input  wire signed [15:0] gyro_z,

    output reg  signed [31:0] roll_q16,       // Q16.16 度
    output reg  signed [31:0] pitch_q16,
    output reg  signed [31:0] yaw_q16,
    output wire signed [15:0] roll_x100,      // 单位 0.01 度（给 OSD/显示用）
    output wire signed [15:0] pitch_x100,
    output wire signed [15:0] yaw_x100,
    output reg                busy            // 解算中（一个样本 32+ 拍）
);

    // ---------------------------------------------------------------------
    // 输入符号扩展
    // ---------------------------------------------------------------------
    wire signed [31:0] ax32 = {{16{acc_x[15]}}, acc_x};
    wire signed [31:0] ay32 = {{16{acc_y[15]}}, acc_y};
    wire signed [31:0] az32 = {{16{acc_z[15]}}, acc_z};
    wire signed [31:0] gx32 = {{16{gyro_x[15]}}, gyro_x};
    wire signed [31:0] gy32 = {{16{gyro_y[15]}}, gyro_y};
    wire signed [31:0] gz32 = {{16{gyro_z[15]}}, gyro_z};

    wire signed [31:0] neg_ax32 = -ax32;

    // ---------------------------------------------------------------------
    // ×A_n 的移位加法近似（零 DSP）：
    //   1 + 1/2 + 1/8 + 1/64 + 1/256 = 1.644531，相对 A_n=1.646760 误差 -0.135%
    // ---------------------------------------------------------------------
    function signed [31:0] MUL_AN(input signed [31:0] v);
        MUL_AN = v + (v >>> 1) + (v >>> 3) + (v >>> 6) + (v >>> 8);
    endfunction

    // ---------------------------------------------------------------------
    // 限幅
    // ---------------------------------------------------------------------
    function signed [31:0] CLAMP_ANG(input signed [31:0] v);
        if (v > ANG_LIMIT)        CLAMP_ANG = ANG_LIMIT;
        else if (v < -ANG_LIMIT)  CLAMP_ANG = -ANG_LIMIT;
        else                      CLAMP_ANG = v;
    endfunction

    function signed [31:0] CLAMP_YAW(input signed [31:0] v);
        if (v > YAW_LIMIT)        CLAMP_YAW = YAW_LIMIT;
        else if (v < -YAW_LIMIT)  CLAMP_YAW = -YAW_LIMIT;
        else                      CLAMP_YAW = v;
    endfunction

    // ---------------------------------------------------------------------
    // 陀螺增量（每样本，Q16.16 度）
    // ---------------------------------------------------------------------
    //  ★2026-10-21 上电陀螺零偏自标定（补规约第 8 条的标定流程）：
    //    前 256 样本（2.56s @100Hz）平均 gz 作动态零偏 —— 与既有"上电 2s 静止"流程一致，
    //    要求上电时整机静止。原 GYRO_Z_BIAS 参数=0 未标定 ⇒ 静止时 yaw 持续漂移。
    reg  signed [41:0] gz_bias_acc;
    reg  [8:0]         gz_bias_cnt;
    reg                gz_bias_done;
    reg  signed [31:0] gz_bias_dyn;
    wire signed [31:0] gz_bias32 = gz32 - gz_bias_dyn;

    /* ★2026-10-30 v89：静止自动重标零（治温漂）。
       现象：上电 2s 标定只做一次，运行 10 分钟后陀螺仪温度上升、零偏漂移，
             航向积分持续注入误差 ⇒ 转向过度/不够、对准变差（实测：跑得越久越明显）。
       方案：标定完成后，若 |gx|、|gy|、|gz−bias| 全部 < ~1°/s（16 LSB 原始单位）且
             连续保持 512 样本（~5.1s @100Hz），用这 512 样本均值更新 gz_bias_dyn。
       fail-safe 三条：
         ① 任何一样本超阈 ⇒ 计数与累加清零、窗口重来（车一动就中止）；
         ② 窗口期间继续用旧零偏积分，yaw 无跳变；更新只改 bias，不动 yaw；
         ③ 若静默判据永不满足（如 gx 自身零偏超阈）⇒ 永不触发，行为与 v87 完全一致。
       旋转/平移中 gz 远超阈值 ⇒ 导航与旋转期间不可能误触发。 */
    localparam signed [31:0] CALM_TH = 32'sd16;      // 16 LSB ≈ 1.0 deg/s（±2000dps 量程 16.4 LSB/°/s，原始单位）
    wire calm = (gx32      > -CALM_TH) && (gx32      < CALM_TH)
             && (gy32      > -CALM_TH) && (gy32      < CALM_TH)
             && (gz_bias32 > -CALM_TH) && (gz_bias32 < CALM_TH);

    // ★ 2026-10-14: 常量除法 `/DT_DIV` 折成 **Q8 常量乘法**（去掉除法器网络）。
    //   原式 (raw*GYRO_Q)/DT_DIV 综合后是"乘法 + 常量除法"两级长链，实测
    //   u_attitude 内这条链 **49 ns（周期 20 ns）**，是 CLKOUT1 域最差违例之一。
    //   K_GYRO_Q8 = round(GYRO_Q/DT_DIV * 256)；量级 32768*10230 = 3.35e8 < 2^31 ✓
    //   误差 +0.002%（10230/256 = 39.9609 vs 39.96），同拍完成、语义不变。
    localparam integer K_GYRO_Q8 = (GYRO_Q * 256 + DT_DIV/2) / DT_DIV;

    wire signed [31:0] gyro_dx = (gx32      * K_GYRO_Q8) >>> 8;   // roll 轴
    wire signed [31:0] gyro_dy = (gy32      * K_GYRO_Q8) >>> 8;   // pitch 轴
    wire signed [31:0] gyro_dz = (gz_bias32 * K_GYRO_Q8) >>> 8;   // yaw 轴

    // ---------------------------------------------------------------------
    // CORDIC 实例（两级复用同一个）
    // ---------------------------------------------------------------------
    reg                c_start;
    reg  signed [31:0] c_x, c_y;
    wire               c_done;
    wire signed [31:0] c_ang, c_mag;

    cordic_atan2 #(.ITER(16)) u_cordic (
        .clk       (clk),
        .rst_n     (rst_n),
        .start     (c_start),
        .x_in      (c_x),
        .y_in      (c_y),
        .busy      (),
        .done      (c_done),
        .angle_q16 (c_ang),
        .mag_q16   (c_mag)
    );

    // ---------------------------------------------------------------------
    // 解算状态机：IDLE -> C1(16拍) -> C2(16拍) -> FUSE
    // ---------------------------------------------------------------------
    localparam [2:0] ST_IDLE = 3'd0,
                     ST_C1W  = 3'd1,
                     ST_C2S  = 3'd2,
                     ST_C2W  = 3'd3,
                     ST_FUSE = 3'd4;

    reg [2:0]          st;
    reg signed [31:0]  roll_g, pitch_g;
    reg signed [31:0]  roll_acc, pitch_acc, mag1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st        <= ST_IDLE;
            c_start   <= 1'b0;
            c_x       <= 32'sd0;
            c_y       <= 32'sd0;
            roll_q16  <= 32'sd0;
            pitch_q16 <= 32'sd0;
            yaw_q16   <= 32'sd0;
            gz_bias_acc <= 42'sd0; gz_bias_cnt <= 9'd0; gz_bias_done <= 1'b0; gz_bias_dyn <= 32'sd0;
            roll_g    <= 32'sd0;
            pitch_g   <= 32'sd0;
            roll_acc  <= 32'sd0;
            pitch_acc <= 32'sd0;
            mag1      <= 32'sd0;
            busy      <= 1'b0;
        end else begin
            c_start <= 1'b0;     // 默认单拍

            case (st)

            ST_IDLE: begin
                busy <= 1'b0;
                if (sample_valid) begin
                    // ---- ⓪ 陀螺零偏自标定（前 256 样本）----
                    if (!gz_bias_done) begin
                        gz_bias_acc <= gz_bias_acc + gz32;
                        gz_bias_cnt <= gz_bias_cnt + 9'd1;
                        if (gz_bias_cnt == 9'd255) begin
                            gz_bias_dyn  <= gz_bias_acc[39:8];   // /256（累加和右移 8 位）
                            gz_bias_done <= 1'b1;
                            gz_bias_acc  <= 42'sd0;              // ★v89：清累加器，供连续重标零复用
                            gz_bias_cnt  <= 9'd0;
                        end
                    end
                    // ---- ⓪′ v89：静止连续重标零（512 样本静默窗口，见上方注释）----
                    else begin
                        if (calm) begin
                            gz_bias_acc <= gz_bias_acc + gz32;
                            gz_bias_cnt <= gz_bias_cnt + 9'd1;
                            if (gz_bias_cnt == 9'd511) begin
                                gz_bias_dyn  <= gz_bias_acc[40:9];   // /512
                                gz_bias_acc  <= 42'sd0;
                                gz_bias_cnt  <= 9'd0;
                            end
                        end
                        else begin
                            gz_bias_acc <= 42'sd0;
                            gz_bias_cnt <= 9'd0;
                        end
                    end
                    // ---- ① 陀螺积分 ----
                    roll_g    <= CLAMP_ANG(roll_q16  + gyro_dx);
                    pitch_g   <= CLAMP_ANG(pitch_q16 + gyro_dy);
                    if (gz_bias_done) begin
                        // ★yaw 改 [0,360) 回绕：±180 钳位会让 >180° 旋转的累计判据永不满足
                        if (yaw_q16 + gyro_dz >= 32'sd23592960)
                            yaw_q16 <= yaw_q16 + gyro_dz - 32'sd23592960;
                        else if (yaw_q16 + gyro_dz < 32'sd0)
                            yaw_q16 <= yaw_q16 + gyro_dz + 32'sd23592960;
                        else
                            yaw_q16 <= yaw_q16 + gyro_dz;
                    end else begin
                        yaw_q16 <= 32'sd0;               // 标定期间保持 0（要求整机静止）
                    end

                    // ---- 启动第一级：x = az<<12, y = ay<<12 ----
                    c_x     <= az32 <<< 12;
                    c_y     <= ay32 <<< 12;
                    c_start <= 1'b1;
                    busy    <= 1'b1;
                    st      <= ST_C1W;
                end
            end

            ST_C1W: begin
                if (c_done) begin
                    roll_acc <= c_ang;      // atan2(ay, az)
                    mag1     <= c_mag;      // A_n*sqrt(ay^2+az^2)，Q12 尺度
                    st       <= ST_C2S;
                end
            end

            ST_C2S: begin
                // ---- 启动第二级：x = mag1, y = (-ax)*A_n <<12 ----
                c_x     <= mag1;
                c_y     <= MUL_AN(neg_ax32) <<< 12;
                c_start <= 1'b1;
                st      <= ST_C2W;
            end

            ST_C2W: begin
                if (c_done) begin
                    pitch_acc <= c_ang;     // atan2(-ax, sqrt(ay^2+az^2))
                    st        <= ST_FUSE;
                end
            end

            ST_FUSE: begin
                // ---- ② 加速度修正（一阶互补）----
                roll_q16  <= CLAMP_ANG(roll_g  + ((roll_acc  - roll_g ) >>> ALPHA_SHIFT));
                pitch_q16 <= CLAMP_ANG(pitch_g + ((pitch_acc - pitch_g) >>> ALPHA_SHIFT));
                st        <= ST_IDLE;
            end

            default: st <= ST_IDLE;
            endcase
        end
    end

    // ---------------------------------------------------------------------
    // Q16.16 度 -> 0.01 度（int16，供 OSD 显示）
    // ---------------------------------------------------------------------
    assign roll_x100  = (roll_q16  * 32'sd100) >>> 16;
    assign pitch_x100 = (pitch_q16 * 32'sd100) >>> 16;
    assign yaw_x100   = (yaw_q16   * 32'sd100) >>> 16;

endmodule
