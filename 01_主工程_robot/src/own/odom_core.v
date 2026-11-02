//=============================================================================
// odom_core.v -- 麦克纳姆轮正运动学位姿积分核心（任务四 P0）
//-----------------------------------------------------------------------------
// 【在系统里的位置】
//   4 路 quad_dec(不在此模块) -> 每 10ms tick 给出各轮计数增量 -> 本模块
//   积分出世界系位姿 (x, y, theta) 与速度 (vx, vy)。
//   顶层另有 4 个 quad_dec 直读编码器引脚（与 selftest_ir 内部的并行，输入脚
//   可多驱动读），tick 由 speed_meter 的窗口节拍给出。
//
// 【约定】(与 mech_ik.v 一致，改必须同步)
//   x 轴 = 摆车时车头方向(+x)；y = 左；theta 正 = 逆时针(左转)，单位度，0~360。
//   轮序: FL/FR/RL/RR；计数正值 = 该轮向前滚。
//   正运动学(由 mech_ik 逆运动学反解，tb 已核验):
//       vx_body = ( fl + fr + rl + rr)/4 * mm_per_cnt
//       vy_body = (-fl + fr + rl - rr)/4 * mm_per_cnt
//       k*w     = (-fl + fr - rl + rr)/4 * mm_per_cnt      k = (L+W)/2
//   世界系积分:  x += dx*cos(th) - dy*sin(th) ;  y += dx*sin(th) + dy*cos(th)
//
// 【定点】
//   mm/count = 0.218163  -> Q16.16 = 14298
//   deg per s_rot count = 0.0151149 -> Q16.16 = 991
//   sin/cos LUT: 720 点 x 0.5 度, Q2.14 (16384 = 1.0), $readmemh 初始化
//   theta: 度 Q16.16, 保持 [0, 360)
//
// 【航向融合(P0 v1)】
//   theta 以 IMU yaw 为主: imu_valid 脉冲到来时直接 snap imu_yaw；
//   两次 snap 之间用编码器 dθ 死推。imu 长期不来 = 纯编码器(会漂)。
//   打滑检测 v1: SLIP_WIN 窗口内累计 |dθ_enc| vs |ΣΔθ_imu|，两者都有且
//   差值超阈值 => slip 拉高一个窗口。
//
// 【验证】tb_odom_core.v: T1 前进 / T2 左移 / T3 纯旋转 / T4 IMU snap 90 度
//   后前进 / T5 打滑注入 —— 全部有解析期望值，±1% 容差，不允许"看着像对"。
//=============================================================================
`timescale 1ns/1ps

module odom_core #(
    parameter signed [15:0] MM_PER_CNT_Q16 = 16'sd13840, // 0.211182 mm/count（2026-10-31 实测标定：−3.2%）
    parameter signed [15:0] DEG_PER_SROT   = 16'sd991,   // 0.0151149 deg / s_rot
    parameter integer       SLIP_WIN       = 50,         // 打滑检测窗口(tick 数)
    parameter signed [31:0] SLIP_THR_Q16   = 32'sd327680,// 5 度 Q16.16
    parameter signed [31:0] TWO_PI_Q16     = 32'sd23592960, // 360 度 Q16.16
    // θ 源: 0 = 编码器死推  1 = IMU 增量主导(免疫打滑, 默认)
    //   两种模式都保留: 编码器模式供纯 RTL 自测, IMU 模式为实机方案
    parameter integer       THETA_SRC      = 1,
    parameter integer       SGN_ENC_THETA  = 1   // ★编码器航向符号：-1 = 整体取反（轮序镜像时用）
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        clr,                   // 1 拍: 位姿清零(摆车到 A 后按)
    input  wire        tick,                  // 10ms 窗口节拍(1 拍)
    input  wire signed [15:0] w_fl,           // 各轮本窗口计数增量(带符号)
    input  wire signed [15:0] w_fr,
    input  wire signed [15:0] w_rl,
    input  wire signed [15:0] w_rr,
    input  wire        imu_valid,             // 1 拍: yaw 样本有效
    input  wire signed [31:0] imu_yaw_q16,    // 度 Q16.16(调用方保证 0~360)
    input  wire       motion_cmd,   // ★2026-10-22 电机命令中（我方导航驱动）——theta 门控条件之一
    output reg  signed [31:0] x_q16,          // 世界系 mm Q16.16
    output reg  signed [31:0] y_q16,
    output reg  signed [31:0] theta_q16,      // 0~360 度 Q16.16
    output reg  signed [31:0] vx_q16,         // mm/s Q16.16 (dx_per_tick*100)
    output reg  signed [31:0] vy_q16,
    output reg  signed [15:0] wdeg_q16,       // 编码器角速度 度/s Q16.16
    output reg         slip                    // 打滑检出(窗口级,保持到下一窗)
);

    // ------------------------------------------------------------------
    // sin/cos ROM: 由 sin_cos_rom.v 提供(纯 RTL case 表, 零外部文件依赖)
    //   ROM 地址: theta_q16 / 32768 => 0.5 度步进(0~719)
    // ------------------------------------------------------------------
    wire [9:0] rom_addr = theta_q16[24:15];
    wire signed [15:0] c_sin, c_cos;
    sin_cos_rom u_lut (.addr(rom_addr), .sin_d(c_sin), .cos_d(c_cos));

    // ------------------------------------------------------------------
    // 每 tick 的运动学计算(组合)
    // 【注意】16 位负数必须符号扩展(曾犯 {4'b0,w} 零扩展错误: -40 => +65528)
    // ------------------------------------------------------------------
    wire signed [19:0] fl_e = {{4{w_fl[15]}}, w_fl};
    wire signed [19:0] fr_e = {{4{w_fr[15]}}, w_fr};
    wire signed [19:0] rl_e = {{4{w_rl[15]}}, w_rl};
    wire signed [19:0] rr_e = {{4{w_rr[15]}}, w_rr};

    //  ★静止航向冻结（2026-10-21）：MPU6050 无磁力计，yaw=陀螺仪积分 ⇒ 静止时偏置/噪声
    //    使航向持续漂移（实测：车不动、网页朝向一直在变）。编码器测"动没动"是可靠的。
    //    车轮动 ⇒ IMU 增量主导（免疫旋转打滑）；车轮静止 ⇒ 冻结 θ（不漂）。
    wire signed [21:0] w_abs_sum = (fl_e[19] ? -fl_e : fl_e) + (fr_e[19] ? -fr_e : fr_e)
                                 + (rl_e[19] ? -rl_e : rl_e) + (rr_e[19] ? -rr_e : rr_e);
    //  ★2026-10-21 运动判定加强：**四轮全部 |w|>4** 才算动（原：四轮之和 >12 ⇒ 单/双轮
    //    速度尖峰即开门 ⇒ 静止漂移 79°/0.57m/12min）＋ 2 窗去抖（孤立尖峰必被拒）。
    //    真实运动四轮幅度：250mm/s 平移 11.5、55°/s 旋转 ~9（均 >4 ✓）
    wire        wm0 = (fl_e[19] ? -fl_e : fl_e) > 20'sd4;
    wire        wm1 = (fr_e[19] ? -fr_e : fr_e) > 20'sd4;
    wire        wm2 = (rl_e[19] ? -rl_e : rl_e) > 20'sd4;
    wire        wm3 = (rr_e[19] ? -rr_e : rr_e) > 20'sd4;
    wire        wheels_now = wm0 && wm1 && wm2 && wm3;
    reg  [1:0]  motion_cnt;
    always @(posedge clk or negedge rst_n)
        if (!rst_n)                 motion_cnt <= 2'd0;
        else if (wheels_now)        motion_cnt <= 2'd3;
        else if (motion_cnt != 0)   motion_cnt <= motion_cnt - 2'd1;
    wire        wheel_moving = (motion_cnt != 2'd0);   // 去抖后的运动标志（沿用旧名）
                                                 // （高于静止噪声，低于任何真实运动；
                                                 //   55°/s 旋转时四轮合计 ~36 ✓）
    wire signed [19:0] s_fwd_raw = fl_e + fr_e + rl_e + rr_e;
    wire signed [19:0] s_lat_raw = -fl_e + fr_e + rl_e - rr_e;
    wire signed [19:0] s_rot_raw = -fl_e + fr_e - rl_e + rr_e;
    //  ★2026-10-21 窗口增量物理限幅：10ms 窗每轴 ≤60 counts（≈13mm = 1.3m/s）。
    //    开环起步瞬间他们的速度样本会同向尖峰（四轮门控拦不住同向尖峰）⇒ 里程被
    //    "打飞 2m"（实测）。硬钳后垃圾尖峰从数学上进不了里程。
    /*  ★2026-10-22 ★★单位错误修正：s_fwd/s_lat/s_rot 是【四轮之和】，
        正常行驶 200mm/s 时 s_fwd ≈ 4×9.2 = 37 counts、250mm/s 时 46 counts。
        原值 25（按"单轮"算）会把真实位移砍到 54~68% ⇒ 里程少记 1/3
        （实测：点 1m 车实际跑 1.53m；方形四段只有 0.91~0.97）。
        正确上限：物理最快 0.5m/s ⇒ 单轮 11.5 ⇒ 四轮和 46；取 3 倍余量 ⇒ 140。
        作用不变：只拦"尖峰"（单窗 >140 counts = 30mm/10ms = 3m/s 以上），正常运动完全不触发。 */
    /*  ★★★2026-10-22 最终定值 150（关键：限幅作用在【四轮之和】上）
        实测换算：快档 250mm/s ⇒ 每轮 11.5 counts/10ms ⇒ s_fwd ≈ 46；
        旧值 25 ⇒ 里程只记 25/46 = 54% ⇒ "点 1m 实际跑 2m"（1/0.54 = 1.85m）。
        150 = 46 的 3.2 倍余量：正常运动（含快档、加速瞬态）完全不触发；
        仍能拦真尖峰（单窗 >150 counts = 32mm/10ms = 3.2m/s）。 */
    localparam signed [21:0] WIN_MAX = 22'sd150;
    localparam signed [19:0] WIN_LIM = 20'sd150;
    wire signed [21:0] s_fwd_e = s_fwd_raw;
    wire signed [21:0] s_lat_e = s_lat_raw;
    wire signed [21:0] s_rot_e = s_rot_raw;
    wire signed [19:0] s_fwd = (s_fwd_e >  WIN_MAX) ? WIN_LIM : (s_fwd_e < -WIN_MAX) ? -WIN_LIM : s_fwd_raw[19:0];
    wire signed [19:0] s_lat = (s_lat_e >  WIN_MAX) ? WIN_LIM : (s_lat_e < -WIN_MAX) ? -WIN_LIM : s_lat_raw[19:0];
    wire signed [19:0] s_rot = (s_rot_e >  WIN_MAX) ? WIN_LIM : (s_rot_e < -WIN_MAX) ? -WIN_LIM : s_rot_raw[19:0];

    // 车体系位移 Q16.16: (sum * mm_per_cnt) >> 2  (sum 已含 /4)
    wire signed [33:0] dx_b = (s_fwd * MM_PER_CNT_Q16) >>> 2;
    wire signed [33:0] dy_b = (s_lat * MM_PER_CNT_Q16) >>> 2;
    // 编码器转角 Q16.16: s_rot * DEG_PER_SROT
    wire signed [35:0] dth_e = s_rot * DEG_PER_SROT;


    // 世界系位移: dx*cos - dy*sin ; dx*sin + dy*cos   (Q16.16 x Q2.14 >> 14)
    wire signed [49:0] wx = (dx_b * c_cos - dy_b * c_sin) >>> 14;
    wire signed [49:0] wy = (dx_b * c_sin + dy_b * c_cos) >>> 14;

    // 速度输出: 每 10ms 位移 x100 = mm/s (Q16.16 保持)
    wire signed [31:0] vx_n = dx_b[31:0] * 7'd100;
    wire signed [31:0] vy_n = dy_b[31:0] * 7'd100;

    // ------------------------------------------------------------------
    // 积分时序 —— **3 级流水**（2026-10-14）
    //   级 1: 四轮求和 -> 寄存器
    //   级 2: 乘定点常数 -> 寄存器
    //   级 3: 旋转到世界系 + 累加 + θ 回绕（组合消费 ROM 与级 2 寄存器）
    //   10ms 节拍下多 2 拍延迟完全无影响。
    //   ★ 为什么是 3 级而不是 5 级：5 级（把 ROM 输出与乘积再各打一拍）会让
    //     Gowin 报 `sin_cos_rom ... is swept in optimizing`（exp39 起出现、
    //     exp38 及以前没有），且 Logic 反而少 1074 —— 无法确证是"内联"还是
    //     "删除"。3 级版不触发该警告、TNS 仅差 17（-2036 vs -2019），故取 3 级。
    // ------------------------------------------------------------------
    reg  clr_skip;                            // clr 后丢弃首个 tick(假 delta)
    reg signed [31:0] last_yaw;               // IMU 上一拍样本(增量法用)
    reg               yaw_seen;                // ★首个 IMU 样本只作基准，不加增量
    reg [1:0] pst;                            // 流水级: 0 空闲 / 1 求和 / 2 乘常数 / 3 旋转累加
    reg signed [19:0] s_fwd_r, s_lat_r, s_rot_r;
    reg signed [33:0] dx_r, dy_r;
    reg signed [35:0] dth_r;

    // 级 3 的组合部分（直接消费 sin_cos_rom 的 c_sin/c_cos）
    wire signed [49:0] wx_r = (dx_r * c_cos - dy_r * c_sin) >>> 14;
    wire signed [49:0] wy_r = (dx_r * c_sin + dy_r * c_cos) >>> 14;
    wire signed [31:0] vx_r = dx_r[31:0] * 7'sd100;
    wire signed [31:0] vy_r = dy_r[31:0] * 7'sd100;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            x_q16 <= 32'sd0;  y_q16 <= 32'sd0;  theta_q16 <= 32'sd0;
            vx_q16 <= 32'sd0; vy_q16 <= 32'sd0; wdeg_q16 <= 16'sd0;
            clr_skip <= 1'b0; pst <= 2'd0;
            s_fwd_r <= 20'sd0; s_lat_r <= 20'sd0; s_rot_r <= 20'sd0;
            dx_r <= 34'sd0; dy_r <= 34'sd0; dth_r <= 36'sd0;
        end else if (clr) begin
            // 位姿清零；quad_dec pos 同拍被清，紧随的第一个 tick delta 无效 => 丢弃
            x_q16 <= 32'sd0;  y_q16 <= 32'sd0;  theta_q16 <= 32'sd0;
            vx_q16 <= 32'sd0; vy_q16 <= 32'sd0; wdeg_q16 <= 16'sd0;
            clr_skip <= 1'b1; pst <= 2'd0;
        end
        // ---- IMU 增量主导 θ（模式 1），优先级高于编码器流水 ----
        else if (imu_valid && (THETA_SRC == 1) && !yaw_seen) begin
            theta_q16 <= theta_q16;            // ★首个样本只当基准（否则偏 90°）
        end
        // ★跳变保护 2026-10-17：他们的 attitude 快照上电"先 0 后跳真实值(180°)"，
        //   首帧保护挡不住第二帧的跳变（实测初值偏 180）。相邻样本差 >30° 视为重新对基准，
        //   不累加（100Hz 采样下人不可能 10ms 转 30°）。
        else if (imu_valid && (THETA_SRC == 1)) begin
            //  ★2026-10-21 先解卷绕：yaw ∈ [0,360)，跨 0/360 边界的单样本差 ±360
            //    （100Hz 下真实每样本 <10°）；解卷绕后仍 >30° 才是真正的"重新对基准"
            if ((imu_yaw_q16 - last_yaw) > 32'sd11796480)
                theta_q16 <= theta_q16 + (imu_yaw_q16 - last_yaw - TWO_PI_Q16);
            else if ((imu_yaw_q16 - last_yaw) < -32'sd11796480)
                theta_q16 <= theta_q16 + (imu_yaw_q16 - last_yaw + TWO_PI_Q16);
            /* ★2026-10-22 门控改双条件（轮动 OR 电机命令中）：
               - 真静止（车没动且无命令）⇒ 冻结 ⇒ 治零偏残余漂移（实测 355.7 持续下滑）
               - 电机命令中（含刹车慢速段）⇒ 始终跟踪 ⇒ 治"显示卡 81 不跟车"（v19 实测）
               两版实测教训：只拆门控 ⇒ 静止漂；只留门控 ⇒ 显示卡。双条件两者皆治。*/
            else if (!(wheel_moving || motion_cmd))
                theta_q16 <= theta_q16;                  // ★真静止 ⇒ 冻结防漂
            else if ((imu_yaw_q16 - last_yaw) > 32'sd1966080 ||
                     (imu_yaw_q16 - last_yaw) < -32'sd1966080)
                theta_q16 <= theta_q16;                  // 仍 >30° ⇒ 重新对基准，不累加
            else begin
                theta_q16 <= theta_q16 + (imu_yaw_q16 - last_yaw);
                if (theta_q16 + (imu_yaw_q16 - last_yaw) >= TWO_PI_Q16)
                    theta_q16 <= theta_q16 + (imu_yaw_q16 - last_yaw) - TWO_PI_Q16;
                else if (theta_q16 + (imu_yaw_q16 - last_yaw) < 32'sd0)
                    theta_q16 <= theta_q16 + (imu_yaw_q16 - last_yaw) + TWO_PI_Q16;
            end
        end
        else if (!(wheel_moving || motion_cmd)) begin   /* ★2026-10-22 位置也加 motion_cmd：
                                                        导航减速逼近段轮速<阈值 ⇒ 位置冻结 ⇒ 里程不增长
                                                        ⇒ 到点判据永不满足 ⇒ 龟速爬不停（实测视频）。*/
            pst <= 2'd0;                        // ★静止：整个里程积分冻结（位置不漂）
        end
        else begin
            case (pst)
            //------------------------------------------ 级 1: 求和
            2'd0: if (tick) begin
                if (clr_skip) clr_skip <= 1'b0;      // 丢弃这一窗
                else begin
                    s_fwd_r <= s_fwd;
                    s_lat_r <= s_lat;
                    s_rot_r <= s_rot;
                    pst     <= 2'd1;
                end
            end
            //------------------------------------------ 级 2: 乘常数
            2'd1: begin
                dx_r  <= (s_fwd_r * MM_PER_CNT_Q16) >>> 2;
                dy_r  <= (s_lat_r * MM_PER_CNT_Q16) >>> 2;
                dth_r <= s_rot_r * DEG_PER_SROT;
                pst   <= 2'd2;
            end
            //------------------------------------------ 级 3: 旋转 + 累加
            2'd2: begin
                x_q16 <= x_q16 + wx_r[31:0];
                y_q16 <= y_q16 + wy_r[31:0];
                vx_q16 <= vx_r;
                vy_q16 <= vy_r;
                wdeg_q16 <= dth_r[31:16] * 7'sd100;   // 度/tick -> 度/s
                if (THETA_SRC == 0) begin
                    theta_q16 <= theta_q16 + (SGN_ENC_THETA > 0 ? dth_r[31:0] : -$signed(dth_r[31:0]));
                    if (theta_q16 >= TWO_PI_Q16)
                        theta_q16 <= theta_q16 + dth_r[31:0] - TWO_PI_Q16;
                    else if (theta_q16 < 32'sd0)
                        theta_q16 <= theta_q16 + dth_r[31:0] + TWO_PI_Q16;
                end
                pst <= 2'd0;
            end
            default: pst <= 2'd0;
            endcase
        end
    end

    // ------------------------------------------------------------------
    // 打滑检测 v1: 窗口内 |acc_dth_enc| vs |acc_dth_imu|
    //   imu 样本不足(<5) 则不判(无 IMU 时纯编码器,无从交叉)
    // ------------------------------------------------------------------
    reg signed [35:0] acc_enc, acc_imu;
    reg [7:0] win_cnt, imu_cnt;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            acc_enc <= 36'sd0; acc_imu <= 36'sd0;
            win_cnt <= 8'd0;  imu_cnt <= 8'd0; slip <= 1'b0;
        end else begin
            if (tick) begin
                acc_enc <= acc_enc + $signed(dth_e);
                win_cnt <= win_cnt + 8'd1;
                if (win_cnt == SLIP_WIN[7:0] - 8'd1) begin
                    // 窗口结算
                    if (imu_cnt >= 8'd5 &&
                        ((acc_enc - acc_imu) > SLIP_THR_Q16 ||
                         (acc_imu - acc_enc) > SLIP_THR_Q16))
                        slip <= 1'b1;
                    else
                        slip <= 1'b0;
                    acc_enc <= 36'sd0; acc_imu <= 36'sd0;
                    win_cnt <= 8'd0;   imu_cnt <= 8'd0;
                end
            end
            if (imu_valid) begin
                acc_imu <= acc_imu + (imu_yaw_q16 - last_yaw);
                imu_cnt <= imu_cnt + 8'd1;
            end
        end
    end
    always @(posedge clk or negedge rst_n)
        if (!rst_n)                          last_yaw <= 32'sd0;
        else if (clr)                        last_yaw <= imu_yaw_q16;  // 清零时与 IMU 对齐
        else if (imu_valid)                  last_yaw <= imu_yaw_q16;

    // ★首个 IMU 样本标志：只用于"取基准"，之后才允许累加增量
    always @(posedge clk or negedge rst_n)
        if (!rst_n)              yaw_seen <= 1'b0;
        else if (clr)            yaw_seen <= 1'b0;
        else if (imu_valid)      yaw_seen <= 1'b1;

endmodule
