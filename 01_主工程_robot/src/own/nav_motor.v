//=============================================================================
// nav_motor.v -- 任务四 P2: 导航速度指令 -> 四轮闭环 -> 电机引脚
//-----------------------------------------------------------------------------
//  链路：
//    nav_ctrl 的 vx/vy/w（车体系, Q16.16: mm/s, mm/s, deg/s）
//      -> ① 单位换算 + 限幅（int16: mm/s, mm/s, mrad/s）
//      -> ② mech_ik 逆运动学（实测几何: 轮径 D=100mm, L+W=413.5mm）
//      -> ③ 四路独立 speed_loop（自带 quad_decoder + 1kHz PI + PWM + 换向死区）
//      -> ④ 方向脚映射（与任务二相同约定: 前进 in1=1, in2=0）
//
//  ★ 与任务二的关系：**完全并行的第二条电机通路**，靠顶层 2:1 多路选择切换
//    （nav 接管时用本模块输出，否则用 u_task2 输出）。selftest_ir.v 不改。
//  ★ 默认 run=0 ⇒ 输出全 0，系统行为与已验证的任务二完全一致（安全默认）。
//
//  ★ 标定要点（上板必做，都在本文件的参数里，改一处即可）：
//     · WHEEL_DIA_UM / LL_WW_MM = 实测几何（10-13 已量：D=100mm, L+W=413.5mm）
//     · KP_Q/KI_Q               = 沿用任务二标定值 1500/130（Q8）
//     · NEG_TARGET              = 1 表示把 mech_ik 输出取反再进速度环 ——
//                                 任务二在 selftest_ir.v 里有 `mc = -u` 的
//                                 "方向极性修正（正指令=车前进）"，此处对齐。
//     · INVERT_DIR              = 方向脚整体取反（接线实测为准）
//   任一接反的表现都很直观：车反向走 ⇒ 翻 NEG_TARGET 或 INVERT_DIR。
//=============================================================================
`timescale 1ns/1ps

module nav_motor #(
    parameter integer        WHEEL_DIA_UM = 100_000,  // 轮径 0.001mm（100000 = 100.000mm）
    parameter integer        LL_WW_MM     = 413,      // 轴距 L + 轮距 W（mm），实测 413.5
    parameter integer        V_MAX_CPS    = 30_000,   // speed_loop 目标上限(计数/秒)
    parameter signed [15:0]  KP_Q         = 16'sd1500,// 速度环 Kp（Q8, 任务二标定值）
    parameter signed [15:0]  KI_Q         = 16'sd130, // 速度环 Ki（Q8, 任务二标定值）
    parameter integer        NEG_TARGET   = 1,        // 1=目标取反（对齐任务二极性）
    parameter integer        INVERT_DIR   = 0,        // 1=方向脚整体取反
    //  ★2026-10-19 加：AB 对调（只翻速度环反馈极性，不影响方向脚/驱动方向）
    //    实测现象：$MV 后越转越快、$STOP 无效、只有急停(断电)能停 ⇒ 速度环正反馈
    parameter integer        AB_SWAP      = 1,        // （保留，运行时以 inv_enc_i 为准）
    //  ★2026-10-19：闭环不可用的应急方案 —— 开环占空比（duty ∝ 目标速度，不用 PID）
    //    原因：KP/KI 沿用任务二标定(1500/130, Q8)，在我们"计数/秒"误差尺度下输出立即饱和
    //    （1500/256×917≈5400 > PWM 满量程 4000）⇒ 任何目标都变满速。开环先保固化演示。
    parameter integer        OPEN_LOOP    = 1,        // 1=开环（默认），0=闭环 PI
    /*  ★2026-10-24 回滚 300→1100（实测铁证）：
        v60a（=1100）4 点方形 4 次 ALIGN 全部精准，闭合 0.2cm；
        v61/v62（=300）到点后 ALIGN 卡死 6 秒报 ERR11 —— 唯一变量就是本参数。
        机制：抬升 os_lift=(avg<FLOOR)?FLOOR-avg:0 是"把四轮平均拉到 FLOOR"，
              纯旋转时四轮 |duty| 相同（ALIGN_MIN 20°/s ⇒ 各约 264），avg<FLOOR
              ⇒ 抬完四轮全等于 FLOOR ⇒ 差动只剩方向位。
              FLOOR=300 对应轮速 82mm/s，麦轮原地旋转的力矩不足以克服旋转静摩擦
              ⇒ 轮子空转、车身不转 ⇒ IMU 看到的角度不变 ⇒ 停滞超时 ERR11。
              FLOOR=1100 对应轮速 ~300mm/s，力矩足够 ⇒ 旋转正常。
        ⇒ 结论：下限必须按"纯旋转所需最小力矩"取，不能按"直行末段软停"取。
          软停改由速度层（CRUISE_SLOW 400mm 线性减速 + CRUISE_MIN 120）保证。 */
    parameter integer        OL_MIN_DUTY  = 1100,     // 27.5%：四轮"平均"占空比下限（越牵引死区/保旋转力矩）
                                                     //   v61: 1100->300 (300/4000 => 375cps ~= 82mm/s,
                                                     //   above stiction floor & below CRUISE_MIN 120 => soft-stop restored.
                                                     //   Common-mode lift (:240-244) raises the AVERAGE only, same lift on all 4 wheels.
                                                     //   ★2026-10-23 语义修正：只抬四轮平均，不再逐轮 clamp。
                                                     //   逐轮 clamp（原实现）会把四轮差动一起抹平 ⇒ 航向修正
                                                     //   与比例旋转失效（直线走弓形、转弯定速过冲摆动）。
                                                     //   详见下方 ol_duty / ol_lift 注释。
    parameter integer        OL_CPS_FULL  = 5000,     // 满占空对应计数/秒（≈1.1 m/s 物理上限附近）
    //  ★2026-10-20 实测效率补偿（Q8）：前进 0.772 / 横移 0.52 / 旋转 0.50（麦轮打滑）
    //    1/效率 才是补偿系数：1.30 / 1.92 / 2.00
    //  ★2026-10-20 第二轮实测反推（旧的 0.772/0.52/0.50 那组是坏测量）：
    //    补偿 1.30 时前进 106%、补偿 1.92 时横移 155%、补偿 2.00 时旋转 176%
    //    ⇒ 真实效率：前进 0.815、横移 0.807、旋转 0.878（三轴几乎一致）
    //    ⇒ 补偿系数：1/0.815=1.227→314、1/0.807=1.239→317、1/0.878=1.139→291
    //    取整后用 310 / 310 / 291（再留一点余量，避免过冲）
    parameter integer        EFF_VX_Q8   = 310,       // 1.211 ≈ 1/0.826
    parameter integer        EFF_VY_Q8   = 310,       // 同上（横移与前进效率相同）
    parameter integer        EFF_W_Q8    = 291,       // 1.137 ≈ 1/0.879
    parameter integer        PWM_PERIOD   = 4000,
    parameter integer        MIN_DUTY     = 100
)(
    input  wire               clk,
    input  wire               rst_n,

    // ---- 导航速度指令（车体系, Q16.16）----
    input  wire signed [31:0] vx_q16,   // mm/s   前正
    input  wire signed [31:0] vy_q16,   // mm/s   左正
    input  wire signed [31:0] w_q16,    // deg/s  逆时针正
    input  wire [3:0]         inv_enc_i,  // ★运行时：逐轮编码器反向（bit0=FL 1=FR 2=RL 3=RR）
    input  wire [3:0]         inv_dir_i,  // ★运行时：逐轮驱动方向反向

    input  wire               run,      // 1 = nav 接管电机（关断时输出全 0）

    // ---- 编码器（与 u_task2 / u_odom 并行读同一组引脚）----
    input  wire [3:0]         enc_a,
    input  wire [3:0]         enc_b,

    // ---- 电机引脚（与任务二同样约定）----
    output wire [3:0]         n_in1,
    output wire [3:0]         n_in2,
    output wire [3:0]         n_pwm,
    output wire               n_stdby,

    // ---- 观测（调试/GAO）----
    output wire signed [15:0] v_fl, v_fr, v_rl, v_rr,   // mech_ik 目标(计数/秒)
    output wire signed [15:0] spd0, spd1, spd2, spd3    // 实测轮速(计数/秒)
);

    // ------------------------------------------------------------------
    // ① 单位换算 + int16 限幅
    // ------------------------------------------------------------------
    function signed [15:0] sat16(input signed [31:0] v);
        begin
            if      (v >  32'sd32767) sat16 =  16'sd32767;
            else if (v < -32'sd32768) sat16 = -16'sd32768;
            else                      sat16 = v[15:0];
        end
    endfunction

    // ★ 2026-10-14: 输入换算**打一拍**再进 mech_ik ——
    //   原因：w_q16 →(×17 乘法)→ omi → mech_ik 原本串在一条组合链上，
    //   实测 124 ns；nav 的设定值每 10 ms 才更新一次，打拍零代价。
    //   deg/s(Q16.16) -> mrad/s(int)：×π/180×1000 = ×17.4533
    //     用 17.5 近似（+0.27%）：(w×17 + w/2) >> 16
    wire signed [31:0] w_tmp = (w_q16 * 7'sd17) + (w_q16 >>> 1);
    reg  signed [15:0] vxi_r, vyi_r, omi_r;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin vxi_r <= 16'sd0; vyi_r <= 16'sd0; omi_r <= 16'sd0; end
        else begin
            // ★三轴效率补偿（开环实测：横移/旋转只有一半效率）
            vxi_r <= sat16(((vx_q16  >>> 16) * EFF_VX_Q8) >>> 8);
            vyi_r <= sat16(((vy_q16  >>> 16) * EFF_VY_Q8) >>> 8);
            omi_r <= sat16(((w_tmp   >>> 16) * EFF_W_Q8 ) >>> 8);
        end

    // ------------------------------------------------------------------
    // ② 逆运动学（实测几何）
    // ------------------------------------------------------------------
    wire signed [15:0] ik_fl, ik_fr, ik_rl, ik_rr;
    wire               ik_sat;
    mech_ik #(
        .COUNTS_PER_REV (1440        ),
        .WHEEL_DIA_UM   (WHEEL_DIA_UM),
        .LL_WW_MM       (LL_WW_MM    ),
        .V_MAX_CPS      (V_MAX_CPS   )
    ) u_ik (
        .clk   (clk   ),
        .rst_n (rst_n ),
        .vx    (vxi_r ),
        .vy    (vyi_r ),
        .omega (omi_r ),
        .v_fl  (ik_fl ),
        .v_fr  (ik_fr ),
        .v_rl  (ik_rl ),
        .v_rr  (ik_rr ),
        .sat   (ik_sat)
    );

    assign v_fl = ik_fl;
    assign v_fr = ik_fr;
    assign v_rl = ik_rl;
    assign v_rr = ik_rr;

    // 目标极性对齐（任务二 `mc = -u`）
    function signed [15:0] neg_opt(input signed [15:0] v);
        begin
            neg_opt = NEG_TARGET ? (-v) : v;
        end
    endfunction

    wire signed [15:0] t_fl = neg_opt(ik_fl);
    wire signed [15:0] t_fr = neg_opt(ik_fr);
    wire signed [15:0] t_rl = neg_opt(ik_rl);
    wire signed [15:0] t_rr = neg_opt(ik_rr);

    // ------------------------------------------------------------------
    // ③ 四路独立速度环（自带 tick/解码/测速/PI/PWM）
    // ------------------------------------------------------------------
    wire [3:0] pwm_w, dir_w;

    speed_loop #(
        .PWM_PERIOD (PWM_PERIOD), .MIN_DUTY (MIN_DUTY)
    ) u_sl0 (
        .clk(clk), .rst_n(rst_n), .enc_a(inv_enc_i[0] ? enc_b[0] : enc_a[0]), .enc_b(inv_enc_i[0] ? enc_a[0] : enc_b[0]),
        .cnt_clr(1'b0), .run(run), .clr_i(~run),
        .target(t_fl), .kp(KP_Q), .ki(KI_Q),
        .pwm_out(pwm_w[0]), .dir_out(dir_w[0]),
        .position(), .speed(spd0), .speed_m(), .duty(),
        .period_valid_dbg(), .tick_dbg()
    );
    speed_loop #(
        .PWM_PERIOD (PWM_PERIOD), .MIN_DUTY (MIN_DUTY)
    ) u_sl1 (
        .clk(clk), .rst_n(rst_n), .enc_a(inv_enc_i[1] ? enc_b[1] : enc_a[1]), .enc_b(inv_enc_i[1] ? enc_a[1] : enc_b[1]),
        .cnt_clr(1'b0), .run(run), .clr_i(~run),
        .target(t_fr), .kp(KP_Q), .ki(KI_Q),
        .pwm_out(pwm_w[1]), .dir_out(dir_w[1]),
        .position(), .speed(spd1), .speed_m(), .duty(),
        .period_valid_dbg(), .tick_dbg()
    );
    speed_loop #(
        .PWM_PERIOD (PWM_PERIOD), .MIN_DUTY (MIN_DUTY)
    ) u_sl2 (
        .clk(clk), .rst_n(rst_n), .enc_a(inv_enc_i[2] ? enc_b[2] : enc_a[2]), .enc_b(inv_enc_i[2] ? enc_a[2] : enc_b[2]),
        .cnt_clr(1'b0), .run(run), .clr_i(~run),
        .target(t_rl), .kp(KP_Q), .ki(KI_Q),
        .pwm_out(pwm_w[2]), .dir_out(dir_w[2]),
        .position(), .speed(spd2), .speed_m(), .duty(),
        .period_valid_dbg(), .tick_dbg()
    );
    speed_loop #(
        .PWM_PERIOD (PWM_PERIOD), .MIN_DUTY (MIN_DUTY)
    ) u_sl3 (
        .clk(clk), .rst_n(rst_n), .enc_a(inv_enc_i[3] ? enc_b[3] : enc_a[3]), .enc_b(inv_enc_i[3] ? enc_a[3] : enc_b[3]),
        .cnt_clr(1'b0), .run(run), .clr_i(~run),
        .target(t_rr), .kp(KP_Q), .ki(KI_Q),
        .pwm_out(pwm_w[3]), .dir_out(dir_w[3]),
        .position(), .speed(spd3), .speed_m(), .duty(),
        .period_valid_dbg(), .tick_dbg()
    );

    // ------------------------------------------------------------------
    // ④ 开环 PWM（OPEN_LOOP=1 时用）：duty ∝ |目标速度|，方向取符号
    //    常量除法折算成乘法：K_OL_Q10 = (PWM_PERIOD*1024)/OL_CPS_FULL
    // ------------------------------------------------------------------
    //  ★2026-10-23 改为三级流水：原 5C04 的 clk50M Fmax 仅 ~50.015MHz，裕量极小，
    //    新增逻辑绝不能串在"逆解乘法器 → 占空比 → pwm_gen"这条长链上。打拍后
    //    每级都比原路径更短（延迟 60ns，对 10ms 的设定值更新周期无影响）。
    // ------------------------------------------------------------------
    localparam integer K_OL_Q10 = (PWM_PERIOD * 1024) / OL_CPS_FULL;
    /* ★v63 两档共模下限（保底版唯一的结构改动）：
       · 平移/巡航/末段刹车：300（7.5%）⇒ duty<300 才抬，末段能减速到 ~82mm/s 再停；
       · 纯旋转/ALIGN/避障换向：1100（27.5%）⇒ 麦轮原地旋转力矩足够（v60a 实测闭合 0.2cm）。
       判据必须同时看平移分量与旋转分量：只看旋转分量会把"直行+航向修正"的差动
       误判成旋转 ⇒ 地板回 1100 ⇒ 减速失效。
       单位是计数/秒：a_fwd<60 ≈ 13mm/s（几乎不前进）、a_rot>100 ⇒ ω>6.3deg/s。 */
    localparam [15:0]  OL_FLOOR_MOVE = 16'd300;      // 平移地板
    localparam [15:0]  OL_FLOOR_ROT  = 16'd400;       // ★v69 解耦：400 < 三档旋转 raw(527/826/1128) ⇒ 不抬升、三档可分；> 300 ⇒ ALIGN 仍有力矩
    // 单轮"纯比例"占空比（不含下限）。名字沿用 ol_duty，既有测试可直接调用。
    function [15:0] ol_duty(input signed [15:0] tv);
        reg signed [31:0] av;
        begin
            av = tv;
            if (av < 0) av = -av;
            if (av > OL_CPS_FULL) av = OL_CPS_FULL;
            ol_duty = (av * K_OL_Q10) >> 10;
        end
    endfunction
    /*  ★共模抬升 2026-10-23 —— 取代原来的"逐轮 clamp 到下限"
        原实现：每个非零轮各自被抬到 OL_MIN_DUTY ⇒ 四轮占空比之差被一并抹平。
        而"四轮之差"是航向修正与比例旋转的唯一手段，于是实车出现：
          ① 直行段航向修正在电机上完全失效，车按机械不对称性漂 ⇒ 直线段走成弓形；
          ② 对准段转速不再与角度误差成比例（非零即满下限）⇒ 冲过目标角再反向补 ⇒ 来回摆。
        现在只抬"四轮平均"，四轮之差原样保留：
          · 平均 ≥ 下限：四轮值 = 原始值，逐值等同旧行为，不动速度曲线；
          · 平均 < 下限（低速爬行）：均值抬到下限、差值不变 ⇒ 既越过牵引死区
            （仿真模型假设 25% 以下无牵引），又保留差动 ⇒ 航向与对准重新可控。
        抬升量对四轮完全相同，故差动严格保留；÷4 用右移，无除法器。*/
    // ---- 流水级 1：四轮原始占空比寄存 ----
    reg [15:0] osw_fl, osw_fr, osw_rl, osw_rr;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin osw_fl <= 16'd0; osw_fr <= 16'd0; osw_rl <= 16'd0; osw_rr <= 16'd0; end
        else begin
            osw_fl <= ol_duty(t_fl); osw_fr <= ol_duty(t_fr);
            osw_rl <= ol_duty(t_rl); osw_rr <= ol_duty(t_rr);
        end
    // ---- 流水级 1b：两档地板判据（与 osw_* 同拍寄存，不进组合长链）----
    //   20 位中间量：四轮各已被 sat16 限到 ±32767，四轮和最大 131068，20 位裕量充足。
    //   不用 {N{bit}} 式符号扩展 —— Gowin 解析器对此报 EX3863（v63 首编实测）。
    wire signed [19:0] t_fwd_q = t_fl + t_fr + t_rl + t_rr;
    wire signed [19:0] t_rot_q = t_fl + t_rl - t_fr - t_rr;   /* ★v68: 原 t_fl+t_rr-t_fr-t_rl
         在纯旋转(−,+,−,+)下恒为 0、在纯横移(−,+,+,−)下=−4vy 被误判成旋转 ⇒ 两档地板用反。
         按 IK 更正：ω ∝ (fr+rr−fl−rl)/4，此处取反号后再取绝对值，等价。 */
    wire signed [19:0] t_fwd_a = (t_fwd_q >>> 2);
    wire signed [19:0] a_fwd   = t_fwd_a[19] ? -t_fwd_a : t_fwd_a;
    wire signed [19:0] a_rot   = t_rot_q[19] ? -t_rot_q : t_rot_q;
    wire               rot_now = (a_rot > 20'sd100) && (a_fwd < 20'sd60);
    reg  [1:0] rot_cnt;
    reg        is_rot_r;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin rot_cnt <= 2'd0; is_rot_r <= 1'b0; end
        else if (rot_now) begin
            if (rot_cnt != 2'd3) rot_cnt <= rot_cnt + 2'd1;
            if (rot_cnt >= 2'd2) is_rot_r <= 1'b1;   // 连续 3 拍确认才升地板
        end else begin
            rot_cnt <= 2'd0;
            is_rot_r <= 1'b0;                        // 立即退回平移地板
        end

    // ---- 流水级 2：共模抬升量（四轮共用同一标量）----
    wire [16:0] os_sum = {1'b0, osw_fl} + {1'b0, osw_fr} + {1'b0, osw_rl} + {1'b0, osw_rr};
    wire [15:0] os_avg = os_sum >> 2;                                      // ÷4
    reg  [16:0] os_lift;
    wire [15:0] floor_dyn = is_rot_r ? OL_FLOOR_ROT : OL_FLOOR_MOVE;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) os_lift <= 17'd0;
        else        os_lift <= (os_avg < floor_dyn) ? ({1'b0, floor_dyn} - {1'b0, os_avg}) : 17'd0;
    // ---- 流水级 3：逐轮抬升 + 限幅 ----
    function [15:0] ol_apply(input [15:0] raw, input [16:0] lift, input [15:0] tv);
        reg [16:0] v;
        begin
            if (tv == 16'd0) ol_apply = 16'd0;                             // 零目标恒零
            else begin
                v = {1'b0, raw} + lift;
                ol_apply = (v > PWM_PERIOD) ? PWM_PERIOD[15:0] : v[15:0];
            end
        end
    endfunction
    reg [15:0] ol_d_fl, ol_d_fr, ol_d_rl, ol_d_rr;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin ol_d_fl <= 16'd0; ol_d_fr <= 16'd0; ol_d_rl <= 16'd0; ol_d_rr <= 16'd0; end
        else begin
            ol_d_fl <= ol_apply(osw_fl, os_lift, t_fl);
            ol_d_fr <= ol_apply(osw_fr, os_lift, t_fr);
            ol_d_rl <= ol_apply(osw_rl, os_lift, t_rl);
            ol_d_rr <= ol_apply(osw_rr, os_lift, t_rr);
        end
    wire [3:0]  ol_dir  = { t_rr[15], t_rl[15], t_fr[15], t_fl[15] };   // 1 = 反转
    /* ★开环方波：4 路 pwm_gen（n_pwm 是"每路 1 位方波"，不是占空比数值）*/
    wire [3:0]  ol_pwm;
    genvar gi;
    generate
        for (gi = 0; gi < 4; gi = gi + 1) begin : g_ol
            pwm_gen #(.PWM_PERIOD(PWM_PERIOD)) u_ol (
                .clk(clk), .rst_n(rst_n), .run(run),
                .dir_req(ol_dir[gi]),
                .duty(gi == 0 ? ol_d_fl : gi == 1 ? ol_d_fr : gi == 2 ? ol_d_rl : ol_d_rr),
                .pwm_out(ol_pwm[gi]), .dir_out()
            );
        end
    endgenerate

    // ------------------------------------------------------------------
    // ⑤ 方向脚映射 —— 与任务二 motor_out 完全一致：in1 = ~dir, in2 = dir
    //    （speed_loop 约定 dir_out=1 表示"负=反转"）
    // ------------------------------------------------------------------
    wire [3:0] use_dir = OPEN_LOOP ? ol_dir : dir_w;
    wire [3:0] ndir    = use_dir ^ inv_dir_i;

    assign n_in1   = run ? ~ndir : 4'b0000;
    assign n_in2   = run ?  ndir : 4'b0000;
    assign n_pwm   = run ? (OPEN_LOOP ? ol_pwm : pwm_w) : 4'b0000;
    assign n_stdby = run;

endmodule
