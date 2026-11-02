//=============================================================================
// nav_ctrl.v -- 任务四导航状态机（P1）
//-----------------------------------------------------------------------------
// 【职责】手机指令 + 途径点队列 + 里程计位姿  ->  速度指令：
//     vx_cmd / vy_cmd  (mm/s, 车体系, Q16.16)
//     w_cmd            (度/s, Q16.16)
//   下游接 mech_ik（逆运动学 → 四轮目标转速）。
//
// 【运动策略 —— 两阶段"对准-直行"（2026-10-15 用户定案）】
//   每段导航拆成两个子阶段（ph）：
//     PH_ALIGN  原地旋转对准目标方位角（比例转速 w=4×err，限 ±SPIN_RATE；
//               误差 <2° 视为对准）—— 此时不平移。
//     PH_CRUISE 沿车头正前方恒速直行（四轮同速 = 打滑风险最低的姿态），
//               航向锁定 w=4×err（打滑偏航自动修正），接近目标线性减速。
//   ⇒ 轨迹 = 一串直线段，转弯全部原地转 —— 观众一眼看懂，且比场矢量
//     （边走边滑的弧线）稳定：纯前向是麦轮最不易打滑的姿态。
//   方位角 φ 由板载 cordic_atan2（向量模式）算出：输入 (ex,ey) 右移 8 位
//   （Q8.8 mm，防增益溢出），mag 顺带给出"距离×1.6468"，直接供减速曲线用。
//   横移能力保留在手动 $MV,LEFT/RIGHT（M_DIST），不进导航主线。
//
// 【状态机】0 IDLE / 1 NAV / 2 ARRIVED / 3 AVOID(预留) / 4 FAULT / 5 ARMED
// 【两段式】ARM 只装载校验(回 READY)，GO 才动；GO 后先软延时 SOFT_MS 再起步。
//
// 【验收】tb_nav_ctrl.v: 命令行 → 状态迁移 → 速度方向/大小 → 到点 DONE。
//=============================================================================
`timescale 1ns/1ps

module nav_ctrl #(
    parameter integer SPEED_MAX = 300,       // mm/s  导航最大平移速度
    parameter integer MAN_SPEED = 200,       // mm/s  手动平移速度
    parameter integer ARR_MM    = 60,        // mm: retain existing tolerance; 1m forward confirmed normal by user
    parameter integer SOFT_MS   = 1000,      // ms    GO 后软延时
    parameter integer TICK_DIV  = 500_000,   // 控制周期: 10ms @50MHz
    parameter integer SPIN_RATE = 90         // 度/s  手动旋转速度
)(
    input  wire               clk,
    input  wire               rst_n,

    input  wire               cmd_vld,
    input  wire [7:0]         cmd,
    input  wire signed [31:0] arg0,
    input  wire signed [31:0] arg1,
    input  wire [1:0]         i_gear,      // ★速度档位（$CFG,3）：0=慢 150 / 1=中 200 / 2=快 280
    input  wire signed [31:0] i_imu_yaw,   // ★IMU 航向（Q16.16 度，同域）—— 旋转闭环主反馈（不受打滑影响）

    output reg                q_push,
    output reg                q_pop,
    output reg                q_clr,
    output reg                q_seek_en,
    output reg  [4:0]         q_seek_n,
    output reg signed [31:0]  q_px,
    output reg signed [31:0]  q_py,
    input  wire signed [31:0] q_head_x,
    input  wire signed [31:0] q_head_y,
    input  wire [4:0]         q_count,
    input  wire               q_empty,

    input  wire signed [31:0] x_q16,
    input  wire signed [31:0] y_q16,
    input  wire signed [31:0] th_q16,        // 0~360 度 Q16.16

    // ---- P4 避障输入（超声波中值滤波后）----
    //   ★ 顶层暂接 i_dist_vld=1'b0（引脚未定案）⇒ 避障天然不激活；
    //     传感器接入后自动生效，无需再改本模块。
    input  wire [15:0]        i_dist_cm,     // 距离 cm（中值滤波后）
    input  wire               i_dist_vld,    // 距离有效（10Hz 单拍）

    output reg signed [31:0]  vx_cmd,        // mm/s Q16.16 (车体系)
    output reg signed [31:0]  vy_cmd,
    output reg signed [31:0]  w_cmd,         // 度/s Q16.16
    output reg  [2:0]         st,
    output reg  [4:0]         wp,
    output reg                o_hold,        // ★1=强制电机关断 ~200ms（$STOP/$ESTOP 物理停车）
    output reg                resp_vld,
    output reg  [3:0]         resp_type,
    output reg  [4:0]         resp_arg,

    /*  v74：避障状态机透视（纯观测，零逻辑改动；端口 33 -> 43 属预期） */
    output wire [2:0]         dbg_phase,   // av_phase
    output wire [1:0]         dbg_sub,     // av_sub
    output wire [2:0]         dbg_step,    // av_step
    output wire [1:0]         dbg_near,    // av_near
    output wire [1:0]         dbg_far,     // av_far
    output wire               dbg_unk,     // av_unk
    output wire signed [15:0] dbg_samp,    // av_samp ? av_samp_cm : 999
    output wire signed [31:0] dbg_odo,     // imu_odo_n（航向账 Q16.16）
    output wire signed [31:0] dbg_w,       // w_cmd（度/s Q16.16）
    output wire [1:0]         dbg_gear     // i_gear
);
    // ---- 命令码（与 bt_cmd2 一致）----
    localparam [7:0] C_CLR=8'd3, C_ADD=8'd4, C_ARM=8'd5, C_GO=8'd6, C_ORG=8'd13,   /* ★合并：重置原点时清队列 */
                     C_STOP=8'd7, C_ESTOP=8'd8, C_WP=8'd9, C_MV=8'd10;
    // ---- 状态 ----
    localparam [2:0] S_IDLE=3'd0, S_NAV=3'd1, S_ARR=3'd2, S_AVOID=3'd3,
                     S_FAULT=3'd4, S_ARMED=3'd5;
    // ---- 应答 ----
    localparam [3:0] R_ADD=4'd1, R_CLR=4'd2, R_READY=4'd3, R_GO=4'd4,
                     R_STOP=4'd5, R_DONE=4'd6, R_ERR=4'd7, R_WP=4'd8, R_MV=4'd9,
                     R_EVT=4'd10;                                   // P4: $EVT 上报（网页已支持 EVT 标签）
    // ---- 运动模式 ----
    localparam [1:0] M_WP=2'd0, M_DIST=2'd1, M_SPIN=2'd2;
    // ---- 导航子阶段（对准-直行）----
    localparam [1:0] PH_ALIGN=2'd0, PH_CRUISE=2'd1;

    // ---- 常数（声明必须在引用之前）----
    localparam integer SOFT_TICKS = (SOFT_MS * 50000) / TICK_DIV;       // ms -> tick 数(50MHz)
    localparam signed [31:0] MAN_STEP = (SPIN_RATE <<< 16) / 100;       // 每 tick 转过的角
    localparam signed [31:0] ARR_Q16  = ARR_MM <<< 16;

    // ---- 两阶段导航常数 ----
    localparam signed [31:0] ANG180    = 32'sd11796480;   // 180° Q16.16
    localparam signed [31:0] ANG360    = 32'sd23592960;   // 360° Q16.16
    localparam signed [31:0] ALIGN_TOL = 32'sd52429;      /* ★0.8°：2° 残差让航点拐角变圆 */
    localparam integer CRUISE_SLOW = 400;                 // mm  距此开始线性减速
    /*  ★2026-10-23 60 → 120：合并时用了 60，实测末段 71mm/s → 30mm/s 时麦轮转不动
        ⇒ 卡在离点 14cm ⇒ 2 秒无进展 ⇒ ERR11 停滞故障（14:27 实测）。
        120mm/s 对应 duty≈440（> 最小占空比 300）⇒ 能持续爬行；v41 用这个值实测停在 0.953。 */
    localparam integer CRUISE_MIN  = 120;                 // mm/s 爬行速度下限
    // CORDIC 输入 = (ex,ey)>>>8（Q8.8 mm）；mag 含增益 1.64676 ⇒ 每 mm 的 mag = 421.58
    localparam integer SLOW_MAG = CRUISE_SLOW * 42158 / 100;          // 减速起点（mag 尺度）
    localparam integer K_V_Q16  = (SPEED_MAX * 65536) / SLOW_MAG;     // mag → mm/s（减速曲线斜率）
    localparam signed [31:0] W_KP = 32'sd4;               // 转向 P 增益（°/s 每度误差）
    localparam signed [31:0] W_LIM  = SPIN_RATE <<< 16;   // w 限幅
    localparam signed [31:0] ALIGN_MIN = (SPIN_RATE < 35 ? SPIN_RATE : 35) <<< 16;   /* v63 定稿：35°/s（v60b 的 20°/s 已随 ERR11 修复一并回退） */
    localparam signed [31:0] W_LIM30 = 30 <<< 16;         // 航向锁定 w 限幅（直行中小幅修正）
    // ---- P4 避障常数（策略：停 → 原地转 90° → 直行绕开 → 重新对准目标）----
    /*  ★2026-10-22 阈值定为 25cm + 增加"接近判定"：
        声呐对空地面的回波约 30~50cm（固定值），任何 ≥30cm 的阈值都会被"地面"恒定命中
        ⇒ 导航被误触发、走一半停死（实测 v29：45cm 阈值下导航停住）。
        25cm 低于地面回波；再加入"比上次更近 ≥3cm"的判据 ⇒ 静止地面永不触发，真趋近才触发。 */
    localparam [15:0] OBS_TH_CM  = 16'd30;    /* ★30cm（v41 实测触发点） */
    localparam integer OBS_N     = 1;                     /* ★1 次确认（声呐稀疏，2 次凑不满） */
                                                          // ★待办：声呐 55% 无回波时"连续 2 次"常被无效样本打断
                                                          //   ⇒ 若要更灵敏可改 1（已有有效性窗口+趋近判据双重过滤）。
    /*  ★2026-10-23 v52：450 → 600（定量依据，不是试错）。
        箱 230×130 近面朝车 ⇒ 半宽 115mm；车 300×260 ⇒ 半宽 130mm；声呐阈值 300mm(车鼻→箱面)
          ⇒ 纯平行通过需横向 X-115 > 300 ⇒ X > 415mm；
        旧动作"绕完立刻回插目标"在箱身处横向只剩 0.625·X ⇒ 需 X > 580mm
          ⇒ 450 必然二次触发（实测三次侧移 0.42/0.70/0.81m 正是该不等式逐次迭代）
          ⇒ 600：平行段距箱心 601mm、距箱面 450mm，余量 270mm。 */
    localparam integer BYPASS_MM = 600;                   /* 侧移直行距离（v52：450→600） */
    localparam integer BYPASS_TICKS = (BYPASS_MM * 100) / SPEED_MAX;  // 满速所需 tick（600 ⇒ 300）
    localparam integer PARA_MM   = 550;                   /* ★v52 新增：绕过箱侧后沿原航向平行前进距离 */
    localparam integer PARA_TICKS = (PARA_MM * 100) / SPEED_MAX;      // 550 ⇒ 275
    localparam integer AVOID_MAX = 5;                     // 同一点最多连续避障 5 次
    localparam integer AV_HOLD_TICKS = 30;                // 避障前停 0.3s
    localparam integer AVOID_REARM_CM = 8;                /* ★v60b ① 绕完必须先离开触发点 8cm 才允许再触发（16 位 mm 精度）*/

    // ---- 10ms 节拍 ----
    reg [27:0] tick_cnt;
    wire       tick = (tick_cnt >= (TICK_DIV[27:0] - 28'd1));   // ★-1: 否则 0..N 共 N+1 拍/tick(off-by-one)

    // （原 sin_cos_rom 查找表已随场矢量策略移除 —— 两阶段策略不需要旋转矩阵，
    //   直行沿车体 x 轴，mech_ik 在车体系工作；odom_core 内另有自己的实例不受影响）

    // ---- 运行态 ----
    reg [1:0]  mode;
    reg [1:0]  ph;                    // M_WP 子阶段：PH_ALIGN / PH_CRUISE
    reg [31:0] soft_cnt;
    reg [31:0] need_ticks, done_ticks;
    reg        spin_neg;
    reg signed [31:0] spin_target, spin_acc;
    reg [2:0]  cmd_dir_r;            // 手动平移方向码(0 FWD 1 BACK 2 LEFT 3 RIGHT)

    // ---- P4 避障运行态 ----
    reg [1:0]  obs_cnt;              // 连续障碍采样计数
    reg [15:0] obs_dist_prev;        // ★趋近基准距离（cm）：每 4 个采样刷新一次
    reg        obs_dist_v;           // 基准是否有效
    reg [1:0]  obs_ref_cnt;          // 基准刷新计数（0~3）
    /*  ★★2026-10-22 关键：趋近判据必须"跨多个采样"比较。
        采样间隔 ~65ms，200mm/s 下每次采样只近 1.3cm —— 若与"上一次采样"比 3cm，永远不满足
        ⇒ 避障会被彻底废掉（我在本节自查时发现）。改为与"4 个采样前（≈0.26s）"比 5cm：
        正常趋近 200mm/s ⇒ 0.26s 走 5.2cm ✓；地面回波是固定值 ⇒ 差值恒 ~0 ✓ 永不误触发。 */
    reg [15:0] dist_pending_cm;
    reg dist_pending;
    reg arrival_braking;
    reg arrived;
    wire sample_window = (st == S_NAV) && (mode == M_WP) &&
                         (ph == PH_CRUISE) && (soft_cnt == 0) && !arrival_braking && !arrived;
    // Rejected commands must not cancel an in-progress braking transaction.
    wire route_command = cmd_vld && ((cmd == C_CLR) || (cmd == C_STOP) ||
                         (cmd == C_ESTOP) || (cmd == C_MV) ||
                         ((cmd == C_ADD) && (st != S_NAV)) ||
                         ((cmd == C_ARM) && !q_empty) ||
                         ((cmd == C_GO) && ((st == S_ARMED) || ((st == S_IDLE) && !q_empty))) ||
                         ((cmd == C_WP) && (arg0 < q_count)));
    wire dist_event = dist_pending || i_dist_vld;
    wire [15:0] dist_cm = i_dist_vld ? i_dist_cm : dist_pending_cm;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin dist_pending <= 0; dist_pending_cm <= 0; end
        else if (!sample_window || route_command) begin dist_pending <= 0; end
        else if (i_dist_vld) begin
            dist_pending_cm <= i_dist_cm;
            dist_pending <= !tick || cmd_vld;
        end else if (tick && !cmd_vld) dist_pending <= 0;
    /*  ★v72：声呐样本进 tick 域的锁存。
        i_dist_vld 是 task123_snapshot_cdc 输出的一拍脉冲（dst_strobe<=0 每拍清零，
        有请求时置 1）== 20ns，而 3'd5/3'd6/obs_near20 全在 if(tick)（10ms=500000 拍）
        里读它 ⇒ 命中率 1/500000。实测三次上板全部“0 帧”就是这个原因。
        此锁存每拍捕获、tick 消费后清，样本最多滞后 10ms。 */
    reg        av_samp;
    reg [15:0] av_samp_cm;
    wire       av_samp_ok = av_samp && (av_samp_cm > 16'd8) && (av_samp_cm < 16'd400);
    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin av_samp <= 1'b0; av_samp_cm <= 16'd0; end
        else if (i_dist_vld) begin av_samp <= 1'b1; av_samp_cm <= i_dist_cm; end
        else if (tick) av_samp <= 1'b0;
    wire obs_dist_ok = (dist_cm > 16'd8) && (dist_cm < 16'd400);
    /* v61: abort probe - fresh valid samples only (i_dist_vld), never the dist_pending_cm hold value;
       8cm lower bound = same semantics as obs_dist_ok, excludes 0 (=no echo). */
    wire obs_near20 = av_samp && (av_samp_cm > 16'd8) && (av_samp_cm < 16'd20);
    reg [15:0] obs_hist1, obs_hist2, obs_hist3;
    reg [2:0] obs_hist_n;   // ★有效读数窗口(8~400cm)
    reg [7:0] avoid_cool;   // ★绕行后冷却（150 tick = 1.5s 内不再触发避障）
    reg [2:0]  avoid_n;              /* ★v72：2→3 位。原来 AVOID_MAX[1:0]=5&3=1 ⇒ 每个航点只能避障 1 次，
                                        第 2 次触发直接 EVT03 停车（实测 22:15 那跑“不动”） */              // 当前点已避障次数（防围死）
    reg        av_side;              // 避让转向 0=逆时针(左) 1=顺时针(右)
    reg [1:0]  av_near_cnt;          // v61: consecutive <20cm frames during strafe/parallel (2 => abort)
    reg [2:0]  av_phase;             // ★v52 扩位：0 停 / 1 转 +90° / 2 侧移 / 3 转 −90° / 4 平行前进
    reg [31:0] av_cnt;               // 阶段计数
    reg signed [31:0] av_odo_acc, av_th_prev;   // ★避障转向的不回绕累加
    /*  ★v60b ① 反重复触发：绕行结束记下位置，离开 8cm 后才允许再触发。
        16 位 mm（取 x_q16[31:16]）——不进 32 位减法/绝对值/比较链（50MHz 域已有存量违例） */
    reg signed [15:0] av_rearm_x_mm, av_rearm_y_mm;
    reg               av_rearm_v;            // 1 = 刚绕完，必须先离开才能再触发
    wire signed [15:0] av_dx_mm = x_q16[31:16] - av_rearm_x_mm;
    wire signed [15:0] av_dy_mm = y_q16[31:16] - av_rearm_y_mm;
    wire [15:0] av_adx_mm = av_dx_mm[15] ? -av_dx_mm : av_dx_mm;
    wire [15:0] av_ady_mm = av_dy_mm[15] ? -av_dy_mm : av_dy_mm;
    wire        av_rearm_ok = (({1'b0, av_adx_mm} + {1'b0, av_ady_mm}) >= (AVOID_REARM_CM * 10));
    /*  ★v64 组1：扇形扫描选向的内部状态（与 av_phase 正交，不改 phase 编码序列）*/
    reg [2:0]  av_step;              /* 角点序号 0..5 */
    reg [1:0]  av_sub;               /* 子状态 0=转向 1=停稳 2=采样 3=判定 */
    reg [1:0]  av_frm;               /* 已采帧数 */
    reg [1:0]  av_near, av_far;      /* 近帧(<35cm)/远帧计数 */
    reg [15:0] av_wait;              /* 停稳/超时计数 */
    reg [1:0]  av_retry;             /* ★v64 组2：补轮次数 0/1/2 */
    reg        av_unk;               /* 本角点/本轮未知态已重采过一次 */
    /* ★v71：角点由寄存器改为**组合**查表。原实现只在 3'd5 内赋值 ⇒ 每进入 3'd5 的第一拍、
       以及每次 av_step 递增后的第一拍，:963/:971 的到位判据读到的仍是**上一角点**的值；
       而上一角点恰好等于车头当前朝向 ⇒ |目标-当前|≈0 ⇒ 立即判"已到位"，整轮扫描一动不动。
       改组合后判据永远用当前角点。 */
    wire signed [31:0] av_deg =
        (av_step == 3'd0) ? (av_side ? -32'sd90 :  32'sd90)
      : (av_step == 3'd1) ? (av_side ?  32'sd90 : -32'sd90)
      : (av_step == 3'd2) ? (av_side ? -32'sd75 :  32'sd75)
      : (av_step == 3'd3) ? (av_side ? -32'sd45 :  32'sd45)
      : (av_step == 3'd4) ? (av_side ?  32'sd75 : -32'sd75)
      :                     (av_side ?  32'sd45 : -32'sd45);
    reg [15:0] av_adv_ticks;         /* 3'd2 实际 tick 数 */
    reg [15:0] av3_1, av3_2, av3_3;  /* ★v64 组2：3'd6 复测三帧缓冲 */
    reg [1:0]  av3_n;                /* ★v64 组2：复测有效帧数 */
    wire [15:0] av3_lo  = (av3_1 < av3_2) ? av3_1 : av3_2;
    wire [15:0] av3_hi  = (av3_1 > av3_2) ? av3_1 : av3_2;
    wire [15:0] av3_med = (av3_hi < av3_3) ? av3_hi : ((av3_lo > av3_3) ? av3_lo : av3_3);
    reg [4:0]  hold_cnt;
    reg [7:0]  arr_idle_cnt;         // ★S_ARR 停留计数（1s 后自动回 S_IDLE）
    reg signed [31:0] mv_ref_th;     // ★手动平移的航向锁定基准（治"前进有点斜"）
    //  ★2026-10-20：手动动作改**里程闭环**（原来纯按时间走 → 横移/旋转因打滑严重发散：
    //    实测同样"左移 50cm"出现 0.02m 和 0.84m；"转 90°"出现 66° 和 138°）
    reg signed [31:0] mv_x0, mv_y0, mv_th0;   // 动作起点位姿
    reg [31:0]        mv_tgt_mm;              // 目标距离（mm，Q16.16 前）

    /* ★★★ v85 ①：位姿镜像寄存器（根治 mv_x0 长链）★★★
       现象（v83 实测，CLKOUT1 域最差族，14 条，slack −5.955 ~ −7.062）：
           `u_nav/mv_x0_1_s0/Q → u_nav/vx_cmd_17..31_s1/D`，dataDelay 25.9~27.0 ns。
       根因（本次逐级核过）：mv_x0 只在 $MV/$ARM/$GO/$ORG 那一拍装载（10ms 级事件），
       之后就一直"挂着"某次动作的起点，但它是**普通寄存器 = 1 个 LUT 输入**。
       于是只要有一处组合逻辑直接吃它，这一整条
           「mv_x0 → 32位减法 → 取绝对值 → 32位加法 → (×)比较 → 斜坡 → vx_cmd」
       就被并进一个 20 ns 周期里，而链条本身已经 25~27 ns。
       本文件里吃 mv_x0 的组合口只有 mv_dx/mv_dy 这一处（→ mv_dabs）；
       vx_cmd 又是**其他条件分支**的赋值目标 ⇒ 工具为省资源把两条路焊在一起，
       把 mv_* 这条慢链的延迟带进了 vx_cmd 的 D 端 ⇒ 表现为"导航速度指令"违例。
       修法：给位姿打一拍镜像（x_q16_m 等），让 mv_* 全部吃"寄存器出来的镜像"。
       时序影响：姿态晚 1 拍 = 20 ns。
       行为影响：**无** —— odom 路径本来就有两级同步 + 二级寄存（od_x_q 等），
                 这里再加一拍共 60 ns，仍远小于一个 10 ms 控制 tick；
                 mv_x0/mv_y0/mv_th0 的装载值同步改用镜像值，起点基准一致。
       刻意**不平移 x_q16/y_q16/th_q16 本身** —— 它们被几十处比较器广泛使用，
       打包成专用镜像反而会让 LUT 数量暴涨；只镜像"涉及减法的那一路"。 */
    reg signed [31:0] x_q16_m, y_q16_m, th_q16_m;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin x_q16_m <= 32'sd0; y_q16_m <= 32'sd0; th_q16_m <= 32'sd0; end
        else        begin x_q16_m <= x_q16; y_q16_m <= y_q16; th_q16_m <= th_q16; end

    // ---- 误差与到达判定（分量判据, 免开方）----
    /* ★2026-10-30 v78 时序优化：ex/ey 打一拍再进 CORDIC。
       原形态：ex = q_head_x - x_q16（32 位减法）→ >>>8 → CORDIC 16 级迭代，
               全在一拍内 ⇒ 实测 slack −6.604 ns（CLKOUT1 域，v77 时序报告
               `od_y_q_1_s0/Q -> u_nav/vx_cmd_28_s1/D`）。
       代价极小：控制周期 TICK_DIV=500_000（10 ms），多 1 拍 = 20 ns = 2e-6 个控制周期，
               对导航行为**无任何可观测影响**（到达判定/对准/CORDIC 全都是 10ms 级决策）。
       ⇒ 本改动只碰时序结构，不碰任何算法参数与判据阈值。 */
    wire signed [31:0] ex_raw = q_head_x - x_q16_m;      /* v85①：吃镜像 */
    wire signed [31:0] ey_raw = q_head_y - y_q16_m;      /* v85①：吃镜像 */
    reg  signed [31:0] ex, ey;                 // 打拍后的版本（供 CORDIC / 到达判定）
    /* ★★★ 2026-10-30 v81 致命缺陷修复 ★★★
       v78「打一拍」时**只声明了 ex/ey 两个 reg，却漏写了给它们赋值的 always 块**。
       后果：Gowin 综合器把未赋值的 reg 当成常量 0 常量传播 ⇒
             arrived_comb = (0 < 30mm) && (0 > -30mm) && ... = **恒 1**
             ⇒ arrived 恒 1 ⇒ 一按「② 开始导航」就在第一个 10ms tick 直接判"已到达"，
             电机速度全程为 0 ⇒ **车根本不动，立刻回 $DON**（2026-10-30 11:15 实测症状）。
       同时 CORDIC 输入 ex_c/ey_c 也恒 0 ⇒ 方位角失效。
       本修复即补回被漏掉的那一拍寄存。行为等价于 v77 及以前的组合版本
       （只多 20ns 延迟 = 2e-6 个 10ms 控制周期，不可观测）。 */
    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin ex <= 32'sd0; ey <= 32'sd0; end
        else        begin ex <= ex_raw; ey <= ey_raw; end
    /* ★2026-10-30 v79 二次优化：再补一级 **CORDIC 输入专用**流水（ex_c/ey_c）。
       v78 只打一拍时，实测仍剩 13 条同源路径
         `od_x_q_0_s0/Q -> u_nav/vx_cmd_*_s1/D`，slack −5.8 ~ −6.4 ns。
       原因：一拍之后仍要串「32 位减法 → >>>8 算术右移 → 16 级 CORDIC 首级加/减」，
             50 MHz / 20 ns 预算下装不下。
       ⇒ 把 >>>8 的结果再寄存一拍，CORDIC 只面对「已对齐到 Q8.8 的寄存器输出」。
       总计 2 拍 = 40 ns = 4e-6 个控制周期（10 ms），**对行为无可观测影响**。
       只碰时序结构，不动任何参数与判据。 */
    reg  signed [31:0] ex_c, ey_c;             // CORDIC 输入专用（已 >>>8）
    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin ex_c <= 32'sd0; ey_c <= 32'sd0; end
        else        begin ex_c <= (ex >>> 8); ey_c <= (ey >>> 8); end

    wire arrived_comb = (ex < ARR_Q16) && (ex > -ARR_Q16) &&
                   (ey < ARR_Q16) && (ey > -ARR_Q16);
    // 20ns boundary: control decisions are made only every 10ms.
    always @(posedge clk or negedge rst_n)
        if (!rst_n) arrived <= 0;
        else arrived <= arrived_comb;

    // ---- 方位角 CORDIC（自由运行：非 busy 即启动，16 拍/次 << 10ms tick）----
    //   输入 (ex,ey)>>>8 缩到 Q8.8 mm，防 CORDIC 增益(×1.6468)溢出 32 位；
    //   mag = 距离×1.64676（比例正确），直接供减速曲线（阈值同尺度换算）。
    wire       cord_busy, cord_done;
    wire signed [31:0] cord_ang_work, cord_mag_work;
    reg signed [31:0] cord_ang, cord_mag;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin cord_ang <= 0; cord_mag <= 0; end
        else if (cord_done) begin cord_ang <= cord_ang_work; cord_mag <= cord_mag_work; end
    cordic_atan2 #(.ITER(16)) u_cordic (
        .clk(clk), .rst_n(rst_n),
        .start(~cord_busy),
        .x_in(ex_c), .y_in(ey_c),
        .busy(cord_busy), .done(cord_done),
        .angle_q16(cord_ang_work), .mag_q16(cord_mag_work)
    );

    // ---- 航向误差（度 Q16.16，wrap 到 ±180）----
    //   cordic 输出 ∈ (-180,180]，th ∈ [0,360) ⇒ 原始差 ∈ (-540,360]，wrap 一次即入 ±180
    //   ★ 特判 ey==0：cordic 对 y 恰好为 0 的输入有 ~1° 残差（首拍强制 y 减分支的
    //     实现细节，任务一 IMU 场景不会踩到）—— 而"正前方目标"是导航常见场景，
    //     必须特判：φ = 0（ex≥0）或 ±180（ex<0）。仿真 T3 已实测踩中。
    wire        ey_zero = (ey == 32'sd0);
    wire signed [31:0] phi = ey_zero ? ((ex >= 0) ? 32'sd0 : -ANG180) : cord_ang;
    wire signed [32:0] err_raw = {phi[31], phi} - {th_q16_m[31], th_q16_m};   /* v85①：吃镜像 */
    // ★手动平移的航向保持误差（参考航向 - 当前航向，wrap ±180）
    wire signed [32:0] mv_err_raw = {mv_ref_th[31], mv_ref_th} - {th_q16_m[31], th_q16_m};   /* v85①：吃镜像 */
    wire signed [31:0] mv_err_deg = (mv_err_raw > $signed({1'b0, ANG180}))  ? (mv_err_raw[31:0] - ANG360)
                                  : (mv_err_raw < -$signed({1'b0, ANG180})) ? (mv_err_raw[31:0] + ANG360)
                                  : mv_err_raw[31:0];

    wire signed [31:0] err_deg_comb = (err_raw > $signed({1'b0, ANG180}))  ? (err_raw[31:0] - ANG360)
                               : (err_raw < -$signed({1'b0, ANG180})) ? (err_raw[31:0] + ANG360)
                               : err_raw[31:0];
    reg signed [31:0] err_deg;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) err_deg <= 0;
        else err_deg <= err_deg_comb;

    localparam signed [31:0] AV_SIDE_DB = 32'sd196608;                 /* 3° << 16 */
    wire av_err_big = (err_deg > AV_SIDE_DB) || (err_deg < -AV_SIDE_DB);

    wire [2:0]  dir_new = arg0[2:0];   // ★本条 $MV 的方向码（判"同方向续命令"）
    // ---- ★手动动作的里程闭环判据 ----
    /* v85①：全部改吃镜像，把「mv_x0 → 减法 → 取绝对值 → 加法 → 比较 → 斜坡 → vx_cmd」
       这条 25~27 ns 的组合链从 vx_cmd 的 D 端彻底摘出去。 */
    /* ★2026-10-30 v87：mv 里程闭环判据拆三级流水（根治 vx_cmd 残余违例族）。
       v86 实测 top-25 = 100% u_nav vx/vy_cmd（slack −4.9~−7.1ns），链路逐级核过：
         x_q16_m →(32位减法)→ mv_dx →(绝对值+加法+>>>16)→ mv_dabs
               →(+60 加法+比较 mv_done_near)→ case 选择 → vx_cmd D 端，全组合 34.25ns。
       拆法：减法打一拍(mv_dx_q/mv_dy_q)，绝对值+加法+移位再打一拍(mv_dabs_q)，
       下游 mv_done_d / mv_stag / mv_done_near 全部改吃 mv_dabs_q。
       行为影响：判据晚 2 拍 = 40ns；决策周期 tick = 10ms ⇒ 无任何可观测影响。
       算术一字未改（连原有溢出特性都保持逐位等价，只是插入两级寄存器）。 */
    reg  signed [31:0] mv_dx_q, mv_dy_q;
    reg  [31:0]        mv_dabs_q;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin mv_dx_q <= 32'sd0; mv_dy_q <= 32'sd0; mv_dabs_q <= 32'd0; end
        else begin
            mv_dx_q   <= x_q16_m - mv_x0;
            mv_dy_q   <= y_q16_m - mv_y0;
            mv_dabs_q <= ((mv_dx_q[31] ? -mv_dx_q : mv_dx_q)
                        + (mv_dy_q[31] ? -mv_dy_q : mv_dy_q)) >>> 16;
        end
    wire        mv_done_d = (mv_dabs_q >= mv_tgt_mm);        /* 已走够目标距离（mm）*/
    // ---- ★旋转闭环：不回绕累加（2026-10-20）----
    //   旧判据 |wrap(mv_dth)| >= target 有 ±180 回绕缺陷：180/360/720 永远差一点
    //   （实测 720 过转 1.875×）。改为每 tick 累加小增量，天然支持多圈。
    reg  signed [31:0] spin_odo_acc, spin_th_prev;
    //  ★IMU 旋转累加器：编码器航向在原地旋转时因辊子打滑虚高 ~1.5×（实测 1080° 只转 720°），
    //    IMU 陀螺仪直接量车体转动，不受打滑影响 ⇒ 旋转闭环以 IMU 累计为主判据。
    //    IMU 若无数据（|imu 累计|不增长）⇒ 永不满足 ⇒ 2× 超时兜底，安全。
    reg  signed [31:0] imu_odo_acc, imu_th_prev;
    wire signed [32:0] imu_d_raw = {i_imu_yaw[31], i_imu_yaw} - {imu_th_prev[31], imu_th_prev};
    wire signed [31:0] imu_d_comb = (imu_d_raw >  $signed({1'b0, ANG180})) ? (imu_d_raw[31:0] - ANG360)
                             : (imu_d_raw < -$signed({1'b0, ANG180})) ? (imu_d_raw[31:0] + ANG360)
                             : imu_d_raw[31:0];
    reg signed [31:0] imu_d;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) imu_d <= 0;
        else imu_d <= imu_d_comb;
    wire signed [31:0] imu_odo_n = imu_odo_acc + imu_d;
    wire [31:0] imu_odo_abs = imu_odo_n[31] ? -imu_odo_n : imu_odo_n;
    //  双保险完成判据：IMU 累计到位（主）‖ 编码器累计达 1.5×target（防 IMU 失效失控）
    //  ★2026-10-21 目标补偿 −2.4°：比例刹车的停止延迟造成恒定欠转（方形实测
    //    87.3~87.9°，方差 <0.6°）⇒ 判据提前 2.4° 触发，落点 ±0.6°
    /*  提前量 2.4°（与 v35 位流一致）。★待办：实测 90° 停 87.7（每角少 2.3°），
        原因是刹车下限 35°/s 后车体几乎不滑行，2.4° 提前量显得过大 ⇒ 建议改为 0.3°（19661），
        预期 90°→90±0.3、方形闭合 15cm→2~3cm。改完务必复测 90/180/360/720/1080。 */
    /*  ★v62 定稿 1.9°（124518）：v61 实测 lead 1.38°（90400）时 4×90° 累计差 −5~10°；
        v60a 实测 lead 0.3°（19661）+35°/s 时累计 +13° ⇒ 零点落在 1.8~2.0°，取 1.9°。 */
    localparam signed [31:0] SPIN_LEAD = 32'sd108134;   // 1.65°（v63 定稿值；v62 的 1.9° 已证实过量）
    //  ★2026-10-21 补偿上限 = 目标/4：固定 2.4° 对 5° 小角度命令占一半 ⇒ 车只转 2.6°（实测"5° 不动"）
    wire [31:0] spin_lead = (spin_target >>> 2 < SPIN_LEAD[31:0]) ? (spin_target >>> 2) : SPIN_LEAD[31:0];
    /* ★2026-10-30 v88：旋转闭环判据拆流水（补齐与 imu_d v64 打拍风格的对称）。
       v87 实测 top-25 = u_nav st/w_cmd 25 条（−5.4~−7.7ns），链路逐级核过：
         od_th_q →(33位减法+wrap)→ spin_d →(+acc)→ spin_odo_n →(绝对值)
               →(lead/lim/enc_lim 比较簇)→ st / w_cmd D 端，全组合 49 级。
       拆法：spin_d 打一拍（spin_d_q）；两个绝对值再打一拍（imu_odo_abs_q / spin_odo_abs_q）。
       行为影响：判据晚 1~2 拍 = 20~40ns；决策周期 tick = 10ms ⇒ 无可观测影响；
       算术逐位不变，只插入寄存器。imu 侧累计 imu_odo_acc <= imu_odo_n 保持原样。 */
    reg signed [31:0] spin_d_q;
    reg  [31:0] imu_odo_abs_q, spin_odo_abs_q;
    wire        spin_done_imu = (imu_odo_abs_q + spin_lead >= spin_target);
    /* ★v67 修2：转向兜底从 2x 降到 1.1875x（1/8 + 1/16）。IMU 丢帧时最多多转 18.75% 就强制停，
       原来 2x 意味着"想转 90° 可以转到 180°"，转过头之后 3'd6/3'd4 全部错位。 */
    wire signed [31:0] spin_lim = spin_target + (spin_target >>> 3) + (spin_target >>> 4);
    //  ★2026-10-22 编码器保险 1.5×→2.0×：实测 720/1080 都短 20° 左右 ⇒ 长圈旋转时编码器/车体比例
    //    超过 1.5（打滑累积）⇒ 保险判据先于 IMU 主判据触发 ⇒ 提前停。保险只为防失控，放 2.0× 足够。
    wire [31:0] spin_enc_lim  = spin_target + spin_target;           // 2.0×（纯防失控）
    wire signed [32:0] spin_d_raw = {th_q16[31], th_q16} - {spin_th_prev[31], spin_th_prev};
    wire signed [31:0] spin_odo_n = spin_odo_acc + spin_d_q;
    wire        spin_done_enc = (spin_odo_abs_q >= spin_enc_lim);
    wire        spin_done = (spin_odo_abs_q >= spin_target);
    wire [31:0] spin_remain = (imu_odo_abs_q < spin_target) ? (spin_target - imu_odo_abs_q) : 32'd0;
    /* v88 流水寄存器：spin_d wrap / 两个绝对值（本块必须在 spin_odo_n 之后） */
    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin
            spin_d_q <= 32'sd0; imu_odo_abs_q <= 32'd0; spin_odo_abs_q <= 32'd0;
        end
        else begin
            spin_d_q       <= (spin_d_raw >  $signed({1'b0, ANG180})) ? (spin_d_raw[31:0] - ANG360)
                            : (spin_d_raw < -$signed({1'b0, ANG180})) ? (spin_d_raw[31:0] + ANG360)
                            : spin_d_raw[31:0];
            imu_odo_abs_q  <= imu_odo_n[31] ? -imu_odo_n : imu_odo_n;
            spin_odo_abs_q <= spin_odo_n[31] ? -spin_odo_n : spin_odo_n;
        end
    //  ★2026-10-22 超时下限 1s：小角度（<8°）时 2×目标时间比实际转完还短 ⇒ 会被超时截断（5° 只转 3.5°）
    wire [31:0] spin_tmo = ((spin_target <<< 1) > (55 <<< 16)) ? (spin_target <<< 1) : (55 <<< 16);
    //  ★2026-10-22 刹车剩余量改用 IMU 累计（与完成判据同源；编码器在旋转时刻度不可靠）
    reg  [1:0]  mv_done_cnt;                              /* ★平移到位去抖：连续 2 tick 才算完成 */
    reg  [1:0]  sp_done_cnt;                              /* ★旋转到位去抖 */
    //  ★2026-10-22 兜底改"里程停滞检测"：原无条件 (mv_dabs+60 ≥ 目标) 会在正常行驶中也先命中
    //    ⇒ 每条命令都提前 40~60mm 结束（实测四段 1m 全部 0.91~0.97）。改为仅当里程冻结才启用。
    reg [31:0] mv_abs_prev;
    reg [5:0]  mv_stag;
    always @(posedge clk or negedge rst_n)
        if (!rst_n)                                  begin mv_abs_prev <= 32'd0; mv_stag <= 6'd0; end
        else if (route_command || (st != S_NAV) || (mode != M_DIST))  begin mv_abs_prev <= 32'd0; mv_stag <= 6'd0; end
        else if (!tick) begin end
        else if (mv_dabs_q == mv_abs_prev)           mv_stag <= (mv_stag == 6'd63) ? mv_stag : (mv_stag + 6'd1);
        else                                          begin mv_abs_prev <= mv_dabs_q; mv_stag <= 6'd0; end
    wire        mv_frozen    = (mv_stag >= 6'd50);                                  /* 0.5s 无进展 */
    wire        mv_done_near = mv_frozen && (mv_dabs_q + 32'sd60 >= mv_tgt_mm);     /* 仅冻结时才兜底 */
    wire        mv_done_d2 = (mv_done_d || mv_done_near) && (mv_done_cnt >= 2'd1);
    wire        sp_done_s2 = spin_done_imu && (sp_done_cnt >= 2'd1);

    // 旋转速度跟档位：慢 35 / 中 55 / 快 75 °/s（轮缘线速度与对应平移档协调）
    localparam signed [31:0] ROT_SLOW = 32'sd35 <<< 16,   // 35 °/s（Q16.16）
                              ROT_MID  = 32'sd55 <<< 16,   // 55 °/s
                              ROT_FAST = 32'sd75 <<< 16;   // 75 °/s
    wire signed [31:0] rot_cur = (i_gear == 2'd0) ? ROT_SLOW : (i_gear == 2'd2) ? ROT_FAST : ROT_MID;

    // Waypoint liveness: 200 control ticks without >=2mm translation
    // (CRUISE) or >=2deg heading progress (ALIGN) faults; never claims DONE.
    // Integer coordinates and registered differences keep this off long Q16 paths.
    reg signed [15:0] progress_x, progress_y, progress_th;
    reg signed [16:0] progress_dx, progress_dy, progress_dt;
    reg [7:0] progress_ticks;                   /* v65 回退1：恢复单档 8 位预算（200 tick）*/
    reg progress_phase;
    wire progress_moved = (ph == PH_ALIGN) ?
        ((progress_dt >= 17'sd2) || (progress_dt <= -17'sd2)) :
        ((progress_dx >= 17'sd2) || (progress_dx <= -17'sd2) ||
         (progress_dy >= 17'sd2) || (progress_dy <= -17'sd2));
    /*  v65 回退1：恢复单档停滞预算 200 tick（2s）。ERR11 的真正根因是末段占空比过低、
        麦轮转不动，已由 nav_motor 两档占空比地板（OL_FLOOR_MOVE/OL_FLOOR_ROT）解决，
        不再需要 "ALIGN 段 6s" 这种加长预算去掩盖。 */
    wire progress_timeout = (progress_ticks >= 8'd200);
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            progress_x <= 0; progress_y <= 0; progress_th <= 0;
            progress_dx <= 0; progress_dy <= 0; progress_dt <= 0;
            progress_ticks <= 0; progress_phase <= PH_ALIGN;
        end else begin
            progress_dx <= $signed(x_q16[31:16]) - $signed(progress_x);
            progress_dy <= $signed(y_q16[31:16]) - $signed(progress_y);
            progress_dt <= $signed(th_q16[31:16]) - $signed(progress_th);
            if (route_command || st != S_NAV || mode != M_WP ||
                soft_cnt != 0 || arrival_braking || ph != progress_phase) begin
                progress_ticks <= 0;
                progress_x <= x_q16[31:16]; progress_y <= y_q16[31:16];
                progress_th <= th_q16[31:16]; progress_phase <= ph;
            end else if (tick) begin
                if (progress_moved) begin
                    progress_ticks <= 0;
                    progress_x <= x_q16[31:16]; progress_y <= y_q16[31:16];
                    progress_th <= th_q16[31:16];
                end else if (progress_ticks != 8'hff)
                    progress_ticks <= progress_ticks + 1'b1;
            end
        end
    end

    // ---- 转向速度指令（ALIGN 主用 / CRUISE 航向锁定共用）----
    //   w = clamp(err × Kp, ±lim)；P 控制天然减速，无过冲
    function signed [31:0] w_of_err(input signed [31:0] err, input signed [31:0] lim);
        reg signed [33:0] w;
        begin
            w = err * W_KP;
            if      (w >  lim) w = lim;
            else if (w < -lim) w = -lim;
            w_of_err = w[31:0];
        end
    endfunction

    function signed [31:0] clampl(input signed [49:0] v, input integer lim);
        begin
            if      (v >  (lim <<< 16)) clampl =  (lim <<< 16);
            else if (v < -(lim <<< 16)) clampl = -(lim <<< 16);
            else                        clampl = v[31:0];
        end
    endfunction

    // ------------------------------------------------------------------
    // ★ 坐标/角度饱和（2026-10-14 用户定案：**限幅尽量大、等于没有限制**）
    //   设计取舍：不做"越界拒绝"（用户要"越大越好、甚至没有限制"），
    //   只做**溢出保护** —— Q16.16 存在 32 位里，物理上限是 ±32767.999；
    //   若直接 `arg <<< 16` 而 |arg| > 32767，移位的最高位会**回绕成负数**，
    //   静默产生一个反方向的目标点（最难查的一类 bug）。
    //   ⇒ 饱和到 ±32000 mm（±32 米）。任何比赛场地都在 3 m 内，
    //     等于"无限制"，但永不静默出错。
    // ------------------------------------------------------------------
    localparam signed [31:0] POS_MAX_MM = 32'sd32000;   // ±32 m（Q16.16@32bit 上限 32767.999）
    function signed [31:0] pos_sat(input signed [31:0] v);
        begin
            if      (v >  POS_MAX_MM) pos_sat =  POS_MAX_MM;
            else if (v < -POS_MAX_MM) pos_sat = -POS_MAX_MM;
            else                      pos_sat = v;
        end
    endfunction

    // ---- 手动平移所需 tick 数: |val| mm / (MAN_SPEED mm/s × 10ms) ----
    // ★ 2026-10-14: 原式 `(man_abs*100)/MAN_SPEED` 是 32 位常量除法，实测形成
    //   u_bt_cmd2/arg1 -> u_nav/need_ticks 的 **49 ns** 长链（CLKOUT1 域最差路径）。
    //   改为 Q10 常量乘法；man_abs 先饱和到 32767 防溢出。
    localparam integer K_NEED_Q10 = (100 * 1024) / MAN_SPEED;   // (100/MAN_SPEED)*1024
    wire [31:0] man_raw = arg1[31] ? (32'sd0 - arg1) : arg1;
    wire [31:0] man_abs = (man_raw > 32'd32767) ? 32'd32767 : man_raw;
    wire [31:0] need_n  = (man_abs * K_NEED_Q10) >> 10;

    //  ★速度三档（2026-10-20，$CFG,3 运行时切换）
    //    55°/s 旋转 ≈ 前进 200mm/s 的轮缘线速度（ω×k = 0.96×206.75 ≈ 199mm/s）⇒ 视觉协调
    localparam signed [31:0] SPD_SLOW = 32'sd150, SPD_MID = 32'sd200, SPD_FAST = 32'sd250;   // ★快档 280→250（制动距离/避障安全）
    wire signed [31:0] spd_cur = (i_gear == 2'd0) ? SPD_SLOW : (i_gear == 2'd2) ? SPD_FAST : SPD_MID;
    //  ★加减速斜坡：20 mm/s 每 tick（2000 mm/s²）—— 到点滑行停，不再"咔"一下
    localparam signed [31:0] RAMP_STEP = 32'sd1310720;    // 20 << 16
    localparam integer       RAMP_TICKS = (SPEED_MAX * 65536) / RAMP_STEP;   /* ★v52：0→全速的加速拍数（200 ⇒ 10） */
    localparam integer       ADV_TICKS  = BYPASS_TICKS + RAMP_TICKS + 4;     /* ★v52：③ 侧移超时兜底（300+10+4=314） */
    localparam integer       PARA_TICKS_TOTAL = PARA_TICKS + RAMP_TICKS + 4; /* ★v52：⑤ 平行超时兜底（275+10+4=289） */
    /*  ★v64 组1：扇形扫描选向（六角点）+ 组4 直行距离（首轮 650 / 补轮 200）*/
    localparam integer SCAN_SETTLE_TICKS = 80;                /* ★v66 0.4s→0.8s：给声呐足够时间稳定再采样 */
    localparam signed [31:0] SCAN_TOL_Q16 = 32'sd3 <<< 16;    /* 到位容差 3° */
    /* ★v66：35→45。35cm 与室内地面/杂物回波（30~50cm）重叠，任意方向都可能判堵；
       45cm 越过该区间下沿，仍远高于真实障碍（触发点 30cm）。 */
    localparam integer SCAN_NEAR_CM   = 45;                   /* 三态阈值 45cm */
    localparam integer SCAN_FRAMES    = 4;                    /* 每角采 4 帧 */
    localparam integer SCAN_TMO_TICKS = 900;                  /* ★v67 修4：单角超时 9s（最坏 180°@35°/s=5.14s，原 3s 会误判堵）*/
    localparam integer SCAN_SAMPLE_TMO = 120;                 /* ★v66 0.6s→1.2s：45% 无回波下 0.6s 常凑不满 4 帧 */
    localparam integer RESCAN_TMO_TICKS = 100;                /* ★v68：复测只需采 3 帧≈0.3s，留 1.0s；不复用 9s */
    /* ★v67 修1：侧移距离必须按"当前档位"现算 —— 3'd2 的 vx 目标就是 spd_cur，
       每 tick 走 spd_cur/100 mm。用 SPEED_MAX=200 只在"中档"等价：
       慢档(150) 少走 22%、快档(250) 多走 30%。 */
    /* ★v76：侧移降速锁 120mm/s（不再随档位）—— 麦轮横移打滑随速度上升，
       满速横移使编码器少记 ~30cm、odom 带误差直达终点（v75 三跑 X 偏右 30cm 的根因）。
       tick 数必须与 120 配套，否则横移距离不足。 */
    localparam integer AV_STRAFE_SPD = 200;                               /* ★v76 侧移固定速度 mm/s */
    localparam integer ADV_F_STRAFE = (650 * 100) / AV_STRAFE_SPD + RAMP_TICKS + 4;   /* 561 tick => 650mm@120mm/s */
    localparam integer ADV_R_STRAFE = (200 * 100) / AV_STRAFE_SPD + RAMP_TICKS + 4;   /* 186 tick => 200mm@120mm/s */
    localparam integer ADV_F_SLOW = ADV_F_STRAFE;   /* ★v76 三档统一：侧移不再随档位变速 */
    localparam integer ADV_F_MID  = ADV_F_STRAFE;
    localparam integer ADV_F_FAST = ADV_F_STRAFE;
    localparam integer ADV_R_SLOW = ADV_R_STRAFE;
    localparam integer ADV_R_MID  = ADV_R_STRAFE;
    localparam integer ADV_R_FAST = ADV_R_STRAFE;
    wire [15:0] adv_tk_first = (i_gear == 2'd0) ? ADV_F_SLOW : (i_gear == 2'd2) ? ADV_F_FAST : ADV_F_MID;
    wire [15:0] adv_tk_retry = (i_gear == 2'd0) ? ADV_R_SLOW : (i_gear == 2'd2) ? ADV_R_FAST : ADV_R_MID;
    reg  signed [31:0] vx_slew;
    /* ★2026-10-30 v78 时序优化：减速曲线乘法器打一拍。
      原形态：vx_slow = clampl(cord_mag * K_V_Q16, spd_cur)
              —— cord_mag（CORDIC 输出寄存）直接进 32×32 乘法器 → 50 位比较链 → vx_cmd，
              全组合 ⇒ 实测 slack −6.604 ns（`od_y_q_1_s0/Q -> u_nav/vx_cmd_28_s1/D`）。
      现在：乘法与限幅结果打一拍得 cord_scaled，后续只用打拍值。
      时序影响：巡航速度晚 1 拍生效 = 20 ns。
      行为影响：**无** —— 减速曲线本身按 tick(10 ms) 计算，20 ns 不改变任何斜坡步进；
               v65 当年的"末段被拖成 30~70mm/s"是因为**在限速回路上串了逻辑**，
               与本次"纯打拍"不同：这里不改变 vx_cruise_t 的取值，只延迟一拍呈现。 */
    wire signed [31:0] cord_scaled_w = clampl(cord_mag * K_V_Q16, spd_cur);
    reg  signed [31:0] cord_scaled;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) cord_scaled <= 32'sd0;
        else        cord_scaled <= cord_scaled_w;

    wire signed [31:0] vx_cruise_min = ((spd_cur < CRUISE_MIN) ? spd_cur : CRUISE_MIN) <<< 16;
    wire signed [31:0] vx_cruise_t = (cord_mag >= SLOW_MAG) ? (spd_cur <<< 16)
                           : ((cord_scaled < vx_cruise_min) ? vx_cruise_min : cord_scaled);
    wire               cruise_on   = (st == S_NAV) && (mode == M_WP) && (ph == PH_CRUISE) && !arrived && !arrival_braking;
    /*  v65 回退2/3：删除 v61 的"巡航遇障限速"整块与 v64 的"限速结果打一拍"，恢复为一行组合逻辑。
        理由：① "遇障减速"已由避障状态机（转 90°/侧移/复测）实现，巡航段限速与它语义重复；
              ② 打拍让限速晚 1 拍，与限速寄存器叠加后把末段速度拖成 30~70mm/s；
              ③ 那个限速寄存器还引入 18 条 AG0100 组合环告警。
        恢复后：巡航段速度只由 vx_cruise_t（距离/MAG 分档）决定，不再受声呐影响。 */
    wire signed [31:0] vx_target   = cruise_on ? vx_cruise_t : 32'sd0;
    wire signed [31:0] vx_slew_n   = (vx_slew <  vx_target) ? ((vx_target - vx_slew >  RAMP_STEP) ? vx_slew + RAMP_STEP : vx_target)
                            : ((vx_slew - vx_target >  RAMP_STEP) ? vx_slew - RAMP_STEP : vx_target);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st <= S_IDLE; wp <= 5'd0; mode <= M_WP; ph <= PH_ALIGN;
            q_push <= 1'b0; q_pop <= 1'b0; q_clr <= 1'b0;
            q_seek_en <= 1'b0; q_seek_n <= 5'd0; q_px <= 32'sd0; q_py <= 32'sd0;
            vx_cmd <= 32'sd0; vy_cmd <= 32'sd0; w_cmd <= 32'sd0;
            resp_vld <= 1'b0; resp_type <= 4'd0; resp_arg <= 5'd0;
            soft_cnt <= 32'd0; need_ticks <= 32'd0; done_ticks <= 32'd0; arr_idle_cnt <= 8'd0;
            spin_neg <= 1'b0; spin_target <= 32'sd0; spin_acc <= 32'sd0;
            hold_cnt <= 5'd0; mv_ref_th <= 32'sd0;
            mv_x0 <= 32'sd0; mv_y0 <= 32'sd0; mv_th0 <= 32'sd0; mv_tgt_mm <= 32'sd0;
            vx_slew <= 32'sd0;
            spin_odo_acc <= 32'sd0; spin_th_prev <= 32'sd0;
            imu_odo_acc <= 32'sd0; imu_th_prev <= 32'sd0;
            mv_done_cnt <= 2'd0; sp_done_cnt <= 2'd0;
            cmd_dir_r <= 3'd0; tick_cnt <= 28'd0;
            obs_cnt <= 2'd0; avoid_n <= 2'd0; av_side <= 1'b0; avoid_cool <= 8'd0; av_near_cnt <= 2'd0;
            av_phase <= 3'd0; av_cnt <= 32'd0;
            av_step <= 3'd0; av_sub <= 2'd0; av_frm <= 2'd0;
            av_near <= 2'd0; av_far <= 2'd0; av_wait <= 16'd0;
            av_retry <= 2'd0; av_unk <= 1'b0; av3_n <= 2'd0;
            av3_1 <= 16'd0; av3_2 <= 16'd0; av3_3 <= 16'd0;
            av_adv_ticks <= adv_tk_first;
            arrival_braking <= 0;
            obs_dist_prev <= 0; obs_dist_v <= 0; obs_ref_cnt <= 0;
            obs_hist1 <= 0; obs_hist2 <= 0; obs_hist3 <= 0; obs_hist_n <= 0;
        end
        else begin
            q_push <= 1'b0; q_pop <= 1'b0; q_clr <= 1'b0;
            q_seek_en <= 1'b0; resp_vld <= 1'b0;
            tick_cnt <= tick ? 28'd0 : (tick_cnt + 28'd1);

            //======================= 指令处理 =======================
            if (cmd_vld) begin
                // Route/motion commands invalidate queued samples and approach history.
                if (route_command) begin
                arrival_braking <= 0;
                obs_cnt <= 0; obs_dist_v <= 0; obs_ref_cnt <= 0; obs_hist_n <= 0;
                obs_dist_prev <= 0; obs_hist1 <= 0; obs_hist2 <= 0; obs_hist3 <= 0;
                end
                case (cmd)
                C_ADD: begin
                    if (st != S_NAV) begin
                        /*  ★2026-10-22 去重（合并自我们那条线）：网页对 $ADD 有"确认重发"，
                            固件若照单全收会重复入队 ⇒ 产生"零距离段"⇒ 对准方位角是垃圾值 ⇒ 弧线。
                            与"最后一个已入队点"完全相同 ⇒ 只回应答不入队。 */
                        if (q_count != 5'd0 &&
                            ((pos_sat(arg0) <<< 16) == q_px) &&
                            ((pos_sat(arg1) <<< 16) == q_py)) begin
                            resp_vld <= 1'b1; resp_type <= R_ADD; resp_arg <= q_count;
                        end
                        else begin
                            // ★ 饱和到 ±32000mm（= 无限制，但防移位回绕）
                            q_px <= pos_sat(arg0) <<< 16;
                            q_py <= pos_sat(arg1) <<< 16;
                            q_push <= 1'b1;
                            resp_vld <= 1'b1; resp_type <= R_ADD; resp_arg <= q_count + 5'd1;
                        end
                    end
                    else begin
                        resp_vld <= 1'b1; resp_type <= R_ERR; resp_arg <= 5'd1;
                    end
                end
                C_ORG: begin
                    /*  ★合并自我们那条线：重置原点 = 彻底重来（清空航点队列并回 IDLE）。
                        原实现里 $ORG 只重定位姿，队列残留 ⇒ "刷新/重置后重新加点变成累加"。 */
                    q_clr <= 1'b1; st <= S_IDLE; wp <= 5'd0;
                    vx_cmd <= 32'sd0; vy_cmd <= 32'sd0; w_cmd <= 32'sd0; vx_slew <= 32'sd0;
                    resp_vld <= 1'b1; resp_type <= R_CLR; resp_arg <= 5'd0;
                end
                C_CLR: begin
                    q_clr <= 1'b1; st <= S_IDLE; wp <= 5'd0;
                    vx_cmd <= 32'sd0; vy_cmd <= 32'sd0; w_cmd <= 32'sd0;
                    resp_vld <= 1'b1; resp_type <= R_CLR; resp_arg <= 5'd0;
                end
                C_ARM: begin
                    if (!q_empty) begin
                        st <= S_ARMED; wp <= 5'd0;
                        resp_vld <= 1'b1; resp_type <= R_READY; resp_arg <= q_count;
                    end
                    else begin
                        resp_vld <= 1'b1; resp_type <= R_ERR; resp_arg <= 5'd2;
                    end
                end
                C_GO: begin
                    if ((st == S_ARMED) || (st == S_ARR) || ((st == S_IDLE) && !q_empty)) begin
                        soft_cnt <= SOFT_TICKS;
                        mode     <= M_WP;
                        ph       <= PH_ALIGN; arr_idle_cnt <= 8'd0;      // ★ 每段从"对准"开始
                        st       <= S_NAV;
                        /*  ★合并：① $GO 也接受 S_ARR（否则到点后进 S_ARR 就再也点不动，只能断电）
                            ② 应答里带固件队列点数，网页据此与本地列表核对（暴露 $ADD 丢包）。 */
                        resp_vld <= 1'b1; resp_type <= R_GO; resp_arg <= q_count[4:0];
                    end
                    else begin
                        resp_vld <= 1'b1; resp_type <= R_ERR; resp_arg <= 5'd3;
                    end
                end
                C_STOP: begin
                    st <= S_IDLE; vx_cmd <= 32'sd0; vy_cmd <= 32'sd0; w_cmd <= 32'sd0;
                    hold_cnt <= 5'd20;                       // ★强制关断 20 tick = 200ms
                    soft_cnt <= 32'd0; vx_slew <= 32'sd0;    /* ★清斜坡 */
                    mv_x0 <= x_q16_m; mv_y0 <= y_q16_m;          /* ★闭环基准同步到当前位置 */
                    mv_th0 <= th_q16_m;
                    obs_cnt <= 2'd0; avoid_n <= 2'd0; av_phase <= 3'd0; av_cnt <= 32'd0;
                    obs_dist_v <= 1'b0; obs_ref_cnt <= 2'd0;   /* ★新航点：距离基准重新建立 */
                    resp_vld <= 1'b1; resp_type <= R_STOP; resp_arg <= 5'd0;
                end
                C_ESTOP: begin
                    st <= S_FAULT; vx_cmd <= 32'sd0; vy_cmd <= 32'sd0; w_cmd <= 32'sd0;
                    hold_cnt <= 5'd20;                       // ★强制关断 20 tick = 200ms
                    soft_cnt <= 32'd0; vx_slew <= 32'sd0;    /* ★清斜坡 */
                    mv_x0 <= x_q16_m; mv_y0 <= y_q16_m; mv_th0 <= th_q16_m;
                    obs_cnt <= 2'd0; avoid_n <= 2'd0; av_phase <= 3'd0; av_cnt <= 32'd0;
                    resp_vld <= 1'b1; resp_type <= R_STOP; resp_arg <= 5'd1;
                end
                C_WP: begin
                    if (arg0 < q_count) begin
                        q_seek_en <= 1'b1; q_seek_n <= arg0[4:0]; wp <= arg0[4:0];
                        mode <= M_WP; ph <= PH_ALIGN; st <= S_NAV; soft_cnt <= 32'd0;
                        resp_vld <= 1'b1; resp_type <= R_WP; resp_arg <= arg0[4:0];
                    end
                    else begin
                        resp_vld <= 1'b1; resp_type <= R_ERR; resp_arg <= 5'd4;
                    end
                end
                C_MV: begin
                    mv_done_cnt <= 0;
                    cmd_dir_r <= arg0[2:0];
                    if (arg0 == 32'sd4) begin                // SPIN
                        mode        <= M_SPIN;
                        spin_target <= pos_sat(arg1[31] ? (32'sd0 - arg1) : arg1) <<< 16;  // 角度同样饱和
                        spin_acc    <= 32'sd0;
                        spin_neg    <= arg1[31];
                        mv_th0 <= th_q16_m;                    // ★闭环起点（角度）
                        spin_odo_acc <= 32'sd0; spin_th_prev <= th_q16_m;   // ★不回绕累加从 0 起
                        imu_odo_acc <= 32'sd0; imu_th_prev <= i_imu_yaw;  // ★IMU 累加从 0 起
                        mv_done_cnt <= 2'd0; sp_done_cnt <= 2'd0;         /* ★去抖计数一并清零 */
                    end
                    else begin                               // FWD/BACK/LEFT/RIGHT
                        mode       <= M_DIST;
                        need_ticks <= need_n;
                        done_ticks <= 32'd0;
                        spin_neg   <= arg1[31];
                        //  ★同方向续命令不重锁航向基准（2026-10-21）：键盘 220ms 连发会
                        //    反复重锁 ⇒ 锁定失效 ⇒ 轮速差漂移不受控（实测 S 弯）。
                        //    仅"非续命令"（换方向/从静止起）才重锁。
                        if (!(st == S_NAV && mode == M_DIST && cmd_dir_r == dir_new))
                            mv_ref_th  <= th_q16_m;
                        mv_x0 <= x_q16_m; mv_y0 <= y_q16_m; mv_th0 <= th_q16_m;   // ★闭环起点
                                                               //  （SOFT 期间静止 ⇒ 起点即停止位）
                        mv_tgt_mm <= man_abs;                // ★目标距离（mm）
                    end
                    st <= S_NAV; soft_cnt <= 32'd0;
                    resp_vld <= 1'b1; resp_type <= R_MV; resp_arg <= arg0[4:0];
                end
                default: ;
                endcase
            end
    //======================= 每 tick 出速度 =======================
            else if (tick) begin
                if (hold_cnt != 5'd0) hold_cnt <= hold_cnt - 5'd1;
                if ((st != S_AVOID) && (avoid_cool != 8'd0)) avoid_cool <= avoid_cool - 8'd1;  /* ★v64：绕行中冻结，退出避障才开始计时 */
                if (soft_cnt != 32'd0) begin
                    soft_cnt <= soft_cnt - 32'd1;
                    vx_cmd <= 32'sd0; vy_cmd <= 32'sd0; w_cmd <= 32'sd0;
                end
                else if (st == S_ARR) begin
                    /*  ★合并自我们那条线：到点后 S_ARR 无行为分支 ⇒ 速度全 0、$GO 又拒 ⇒ 死锁。
                        这里停车并 1 秒后自动回 S_IDLE（网页徽标变"待机"，可再次 $GO）。 */
                    vx_cmd <= 32'sd0; vy_cmd <= 32'sd0; w_cmd <= 32'sd0;
                    arr_idle_cnt <= arr_idle_cnt + 8'd1;
                    if (arr_idle_cnt >= 8'd100) begin st <= S_IDLE; arr_idle_cnt <= 8'd0; end
                end
                else if (st == S_NAV) begin
                    case (mode)
                    //============= 两阶段导航：对准 → 直行 =============
                    M_WP: begin
                        if (arrived || arrival_braking) begin
                            arrival_braking <= 1;
                            if (vx_slew != 32'sd0) begin
                                // ★软停：先让速度滑行到 0（≤100ms），再判"到达"
                                vx_cmd <= vx_slew_n; vy_cmd <= 32'sd0;
                                w_cmd <= 0;
                                vx_slew <= vx_slew_n;
                            end
                            else begin
                            // 点就在旁边（含 ALIGN 中）：直接完成，不用转
                            vx_cmd <= 32'sd0; vy_cmd <= 32'sd0; w_cmd <= 32'sd0;
                            resp_vld <= 1'b1; resp_type <= R_DONE; resp_arg <= wp;
                            q_pop <= 1'b1;
                            ph <= PH_ALIGN;                 // 下一段重新对准
                            avoid_n <= 2'd0;                // 到点重置避障计数
                            if (q_count <= 5'd1) st <= S_ARR;
                            else                 wp <= wp + 5'd1;
                            arrival_braking <= 0;
                            obs_cnt <= 0; obs_dist_v <= 0; obs_hist_n <= 0;
                            end
                        end
                        else if (progress_timeout) begin
                            st <= S_FAULT;
                            vx_cmd <= 0; vy_cmd <= 0; w_cmd <= 0; vx_slew <= 0;
                            resp_vld <= 1; resp_type <= R_ERR; resp_arg <= 5'd11;
                        end
                        else if (ph == PH_ALIGN) begin
                            // ---- 阶段 1：原地旋转对准（不平移）----
                            vx_cmd <= 32'sd0; vy_cmd <= 32'sd0;
                            vx_slew <= 32'sd0;                    /* ★ALIGN 期间清斜坡，防残留速度窜入 CRUISE */
                            if (err_deg < ALIGN_TOL && err_deg > -ALIGN_TOL) begin
                                w_cmd <= 32'sd0;
                                ph   <= PH_CRUISE;          // 对准完成 → 直行
                            end
                            else begin
                                if (err_deg > 0 && w_of_err(err_deg, W_LIM) < ALIGN_MIN)
                                    w_cmd <= ALIGN_MIN;
                                else if (err_deg < 0 && w_of_err(err_deg, W_LIM) > -ALIGN_MIN)
                                    w_cmd <= -ALIGN_MIN;
                                else w_cmd <= w_of_err(err_deg, W_LIM);
                            end
                        end
                        else begin
                            // ---- 阶段 2：沿车头直行 + 航向锁定 + 近距减速 ----
                            //   航向锁定：w = clamp(err×Kp, ±30°/s)，打滑偏航自动拉回
                            /*  ★2026-10-23 末段不修航向：距目标 <150mm（mag<63237）时 w=0。
                                原来航向锁始终瞄准目标点，最后几厘米 2cm 横偏就对应 20°+ 方位角
                                ⇒ 车头明显扭一下（实测 8.6°）。代价：到点横偏 +1~2cm。
                                v65：恢复 v63 的 150mm（v61 曾改 60mm，收尾朝向抖动变大）。 */
                            if (cord_mag < 32'sd63237) w_cmd <= 32'sd0;
                            else                       w_cmd  <= w_of_err(err_deg, W_LIM30);
                            vy_cmd <= 32'sd0;
                            vx_cmd <= vx_slew;            // ★斜坡输出（含到点前的滑行减速）
                            vx_slew <= vx_slew_n;
                            /* ★v60b ① 已离开触发点 8cm ⇒ 解除再触发封锁（快照当拍距离=0，不会自清）*/
                            if (av_rearm_v && av_rearm_ok) av_rearm_v <= 1'b0;
                            // ---- P4 障碍检测（10Hz，连续 3 次确认）----
                            if (dist_event) begin
                                /*  ★2026-10-23 去掉"趋近判据"（合并时从旧版带进来的）：
                                    实测 v41 能触发（10:07 日志）而带该判据的版本完全不触发 ——
                                    声呐 55% 无回波，有效样本稀疏，"比 4 采样前近 5cm"凑不满。
                                    误触发已由 obs_dist_ok(8~400cm) 挡住（无回波=0、空地面=1.58m 都进不来）。 */
                                if (obs_dist_ok && dist_cm < OBS_TH_CM && (avoid_cool == 8'd0)
                                    && (!av_rearm_v || av_rearm_ok)) begin      // ★v60b ① 刚绕完、没离开 8cm 不再触发
                                    obs_cnt <= obs_cnt + 2'd1;
                                    // 连续确认 ⇒ 触发避障（同点超 3 次判围死，放弃）
                                    if (obs_cnt >= (OBS_N[1:0] - 2'd1)) begin
                                        if (avoid_n < AVOID_MAX[2:0]) begin
                                            st       <= S_AVOID;
                                            av_phase <= 3'd5; av_cnt <= 32'd0; av_near_cnt <= 2'd0;  /* ★v64：先扫描选向 */
                                            av_step  <= 3'd0; av_sub <= 2'd0; av_frm <= 2'd0;
                                            av_near  <= 2'd0; av_far <= 2'd0; av_wait <= 16'd0;
                                            av_retry <= 2'd0; av_unk <= 1'b0; av3_n <= 2'd0;
                                            av_adv_ticks <= adv_tk_first;
                                            vx_slew  <= 32'sd0;            /* ★v52：进避障清斜坡，③⑤ 从 0 起坡 */
                                            av_side  <= 1'b0;   /* ★v73：选边固定先左（step0=+90），堵则 step1=−90 右绕 */
                                            avoid_n  <= avoid_n + 3'd1;
                                            spin_acc <= 32'sd0;
                                            imu_odo_acc <= 32'sd0; imu_th_prev <= i_imu_yaw;  /* ★v64：扫描转向以触发时刻为基准 */
                                            spin_target <= 90 <<< 16;      // 让 90°
                                            vx_cmd <= 32'sd0; vy_cmd <= 32'sd0; w_cmd <= 32'sd0;
                                            resp_vld <= 1'b1; resp_type <= R_EVT;
                                            resp_arg <= 5'd1;                  /* ★v73：锁左后 EVT 恒 01 */
                                        end
                                        else begin
                                            // 被围死：放弃该点，停车待人工
                                            st <= S_IDLE;
                                            vx_cmd <= 32'sd0; vy_cmd <= 32'sd0; w_cmd <= 32'sd0;
                                            resp_vld <= 1'b1; resp_type <= R_EVT; resp_arg <= 5'd3;
                                        end
                                    end
                                end
                                else begin
                                    obs_cnt <= 2'd0;
                                end
                                if (obs_dist_ok) begin
                                    // Rolling four-valid-sample history, not four-sample blocks.
                                    obs_hist1 <= dist_cm;
                                    obs_hist2 <= obs_hist1;
                                    obs_hist3 <= obs_hist2;
                                    obs_dist_prev <= obs_hist3;
                                    if (obs_hist_n < 4) obs_hist_n <= obs_hist_n + 1'b1;
                                    obs_dist_v <= (obs_hist_n >= 3);
                                end
                                /* 无效样本：基准与计数都不动 */
                            end
                        end
                    end
                    M_DIST: begin
                        // ★航向保持：平移时用 mv_err_deg 拉回基准航向（限 ±20°/s），治"前进跑偏"
                        w_cmd <= w_of_err(mv_err_deg, (20 <<< 16));
                        done_ticks <= done_ticks + 32'd1;
                        mv_done_cnt <= (mv_done_d || mv_done_near) ?
                                       ((mv_done_cnt == 3) ? 2'd3 : mv_done_cnt + 2'd1) : 2'd0;
                        if (mv_done_d2 || (done_ticks >= (need_ticks <<< 1))) begin   // ★里程到位(去抖) / 2× 超时
                            vx_cmd <= 32'sd0; vy_cmd <= 32'sd0; st <= S_IDLE;
                        end
                        else begin
                            case (cmd_dir_r)
                            3'd0: begin vx_cmd <=  (spd_cur <<< 16); vy_cmd <= 32'sd0; end
                            3'd1: begin vx_cmd <= -(spd_cur <<< 16); vy_cmd <= 32'sd0; end
                            3'd2: begin vx_cmd <= 32'sd0; vy_cmd <=  (spd_cur <<< 16); end
                            default: begin vx_cmd <= 32'sd0; vy_cmd <= -(spd_cur <<< 16); end
                            endcase
                        end
                    end
                    M_SPIN: begin
                        vx_cmd <= 32'sd0; vy_cmd <= 32'sd0;
                        spin_acc <= spin_acc + MAN_STEP;
                        spin_odo_acc <= spin_odo_n;              // ★编码器累加（符号/保险用）
                        spin_th_prev <= th_q16_m;
                        imu_odo_acc <= imu_odo_n;                // ★IMU 累加（主判据）
                        imu_th_prev <= i_imu_yaw;
                        sp_done_cnt <= spin_done_imu ? (sp_done_cnt + 2'd1) : 2'd0;
                        if (sp_done_s2 || spin_done_enc ||
                            (spin_acc >= spin_tmo)) begin   // ★IMU 转够(去抖) / 编码器保险 / 超时(≥1s)
                            w_cmd <= 32'sd0; st <= S_IDLE;
                        end
                        else begin
                            //  ★比例刹车：剩余 <15° 时 w=4×剩余（min 8°/s）⇒ 平滑收敛，
                            //    消除"恒速到线+停止延迟"的 ~5.5° 恒定过冲（方形实测）
                            if (spin_remain > (15 <<< 16))
                                w_cmd <= spin_neg ? -rot_cur : rot_cur;
                            else begin
                                if ((spin_remain <<< 2) > rot_cur)
                                    w_cmd <= spin_neg ? -rot_cur : rot_cur;          /* ★上限=档位速度 */
                                else if ((spin_remain <<< 2) > (8 <<< 16))
                                    w_cmd <= spin_neg ? -(spin_remain <<< 2) : (spin_remain <<< 2);
                                else
                                    w_cmd <= spin_neg ? -(35 <<< 16) : (35 <<< 16);   /* ★v60a 定稿 35°/s（同 89.83° 实测配置）：
                                                 小角度（5°）时 8~20°/s 克服不了静摩擦 ⇒ 车不动
                                                 （实测）。35°/s 约 5° 用 150ms。本轮只改 lead 2.4°→0.3°，
                                                 地板保持实测值，一次只动一个变量。*/
                            end
                        end
                    end
                    default: begin
                        vx_cmd <= 32'sd0; vy_cmd <= 32'sd0; w_cmd <= 32'sd0;
                    end
                    endcase
                end
                else if (st == S_AVOID) begin
                    //===== P4 避障 v66：停 → 扫描选向(车头已在侧移朝向) → 侧移650 → 转回90° → 复测 → 平行550 → 回 S_NAV/PH_ALIGN =====
                    if (i_dist_vld) obs_cnt <= 2'd0;        // 避障期间不计障
                    spin_th_prev <= th_q16_m;                 // ★持续更新 prev（供累加）
                    imu_th_prev  <= i_imu_yaw;              // ★IMU prev 同步更新
                    case (av_phase)
                    3'd0: begin                              // (1) 停 0.3s（让机身稳定）
                        vx_cmd <= 32'sd0; vy_cmd <= 32'sd0; w_cmd <= 32'sd0;
                        av_cnt <= av_cnt + 32'd1;
                        if (av_cnt >= (AV_HOLD_TICKS - 32'd1)) begin
                            av_phase <= 3'd1; av_cnt <= 32'd0; spin_acc <= 32'sd0;
                            imu_odo_acc <= 32'd0; imu_th_prev <= i_imu_yaw;   /* 避障转向也用 IMU 累计（编码器旋转读数不可靠）*/
                        end
                    end
                    3'd1: begin                              // (2) 原地转 +90°（不回绕闭环）
                        vx_cmd <= 32'sd0; vy_cmd <= 32'sd0;
                        spin_acc <= spin_acc + MAN_STEP;
                        imu_odo_acc <= imu_odo_n;            // IMU 累加（imu_th_prev 已在上方更新）
                        if (spin_done_imu || (spin_acc >= spin_lim)) begin
                            w_cmd <= 32'sd0; av_phase <= 3'd2; av_cnt <= 32'd0;
                        end
                        else begin
                            w_cmd <= av_side ? -rot_cur : rot_cur;
                        end
                    end
                    3'd2: begin                              // (3) 侧移 BYPASS_MM（机头已转 90°，前进即侧移）
                        w_cmd <= 32'sd0;
                        av_cnt <= av_cnt + 32'd1;
                        /* ★v72：按“样本”计数，不按 tick。样本 65ms 一个、tick 10ms，
                           原来的 else 每拍清零会让计数器永远到不了 2。 */
                        if (av_samp) begin
                            if ((av_samp_cm > 16'd8) && (av_samp_cm < 16'd20))
                                av_near_cnt <= av_near_cnt + 2'd1;
                            else
                                av_near_cnt <= 2'd0;
                        end
                        if (av_cnt >= (av_adv_ticks - 32'd1)) begin   /* ★v64：首轮 650mm / 补轮 200mm */
                            vx_slew <= 32'sd0; vx_cmd <= 32'sd0; vy_cmd <= 32'sd0;
                            spin_target <= 32'sd90 <<< 16;   /* ★v66fix：3'd3 一律先固定转回 90°，与 3'd3 反号逻辑配对 */
                            av_phase <= 3'd3; av_cnt <= 32'd0;
                            spin_acc <= 32'sd0;
                            imu_odo_acc <= 32'd0; imu_th_prev <= i_imu_yaw;   /* 两样都必须清：否则 (4) 首拍即满足 done 判据 => 根本不转 */
                        end
                        else if (obs_near20 && (av_near_cnt >= 2'd1)) begin
                            /* v61: 2 consecutive <20cm while strafing => abort. ★v64 Δ10: 改为重扫，不中止。 */
                            vx_slew <= 32'sd0; vx_cmd <= 32'sd0; vy_cmd <= 32'sd0; w_cmd <= 32'sd0;
                            av_near_cnt <= 2'd0;
                            /*  ★v64 Δ10：不中止。还有补轮余量 ⇒ 回 3'd5 重扫选角（不是原方向再试）；
                                余量用尽 ⇒ EVT03 停车。 */
                            if (av_retry < 2'd2) begin
                                av_retry <= av_retry + 2'd1;
                                av_phase <= 3'd5;
                                resp_vld <= 1'b1; resp_type <= R_EVT; resp_arg <= 5'd5;  /* v74：侧移受阻·重扫（原静默） */
                                /* v75：重扫跳过刚撞壁的角点（EVT05 证明它堵），从下一角点继续；账本不清（触发时刻基准） */
                                av_step  <= (av_step == 3'd5) ? 3'd5 : av_step + 3'd1; av_sub <= 2'd0; av_frm <= 2'd0;
                                av_near  <= 2'd0; av_far <= 2'd0; av_wait <= 16'd0;
                                av_unk   <= 1'b0; av3_n <= 2'd0;
                                spin_acc <= 32'sd0;
                                /* ★v73 方案B：此处不再清 IMU 累计 —— 保持与 av_deg 同样的触发时刻基准 */
                            end
                            else begin
                                st      <= S_IDLE;
                                resp_vld <= 1'b1; resp_type <= R_EVT; resp_arg <= 5'd3;
                            end
                        end
                        else begin
                            if (vx_slew < (AV_STRAFE_SPD <<< 16)) vx_slew <= vx_slew + RAMP_STEP;   /* ★v76 侧移锁 120mm/s 减打滑 */
                            else                            vx_slew <= (AV_STRAFE_SPD <<< 16);
                            vx_cmd <= vx_slew; vy_cmd <= 32'sd0;
                        end
                    end
                    3'd3: begin                              // (4) 原地转 -90°（与 (2) 严格反号）
                        vx_cmd <= 32'sd0; vy_cmd <= 32'sd0;
                        spin_acc <= spin_acc + MAN_STEP;
                        imu_odo_acc <= imu_odo_n;
                        if (spin_done_imu || (spin_acc >= spin_lim)) begin
                            w_cmd <= 32'sd0; av_phase <= 3'd6; av_cnt <= 32'd0;
                            imu_odo_acc <= 32'd0; imu_th_prev <= i_imu_yaw;
                            vx_slew <= 32'sd0;
                            /* ★v64 组2：转回原航向后先复测横向让开量，够才进平行段 */
                            av_sub <= 2'd0; av_wait <= 16'd0;
                            av3_1 <= 16'd0; av3_2 <= 16'd0; av3_3 <= 16'd0; av3_n <= 2'd0;
                            av_unk <= 1'b0;
                        end
                        else begin
                            w_cmd <= av_side ? rot_cur : -rot_cur;
                        end
                    end
                    3'd4: begin                              // (5) 平行前进 PARA_MM（沿原航向越过箱侧）
                        w_cmd <= 32'sd0;
                        av_cnt <= av_cnt + 32'd1;
                        av_near_cnt <= obs_near20 ? (av_near_cnt + 2'd1) : 2'd0;
                        if (av_cnt >= (PARA_TICKS_TOTAL - 32'd1)) begin
                            vx_slew <= 32'sd0; vx_cmd <= 32'sd0; vy_cmd <= 32'sd0;
                            av_retry <= 2'd0; av_unk <= 1'b0; av_adv_ticks <= adv_tk_first;  /* ★v64：下次避障重新用首轮距离 */
                            st <= S_NAV; ph <= PH_ALIGN;     /* 注：此处到目标方位约 49°，PH_ALIGN 还需转 ~0.9s -- 不是避障变慢 */
                            avoid_cool <= 8'd150;   /* 绕行后冷却 1.5s，防刚绕开就二次触发 */
                            av_rearm_x_mm <= x_q16[31:16];   /* ★v60b ① 记下绕行结束点 */
                            av_rearm_y_mm <= y_q16[31:16];
                            av_rearm_v    <= 1'b1;
                        end
                        else if (obs_near20 && (av_near_cnt >= 2'd1)) begin
                            /* v61: 2 consecutive <20cm while running parallel => stop (same rule as strafe). */
                            vx_slew <= 32'sd0; vx_cmd <= 32'sd0; vy_cmd <= 32'sd0; w_cmd <= 32'sd0;
                            av_near_cnt <= 2'd0;
                            /*  ★v64 Δ10：不中止。还有补轮余量 ⇒ 回 3'd5 重扫选角（不是原方向再试）；
                                余量用尽 ⇒ EVT03 停车。 */
                            if (av_retry < 2'd2) begin
                                av_retry <= av_retry + 2'd1;
                                av_phase <= 3'd5;
                                resp_vld <= 1'b1; resp_type <= R_EVT; resp_arg <= 5'd5;  /* v74：侧移受阻·重扫（原静默） */
                                /* v75：重扫跳过刚撞壁的角点（EVT05 证明它堵），从下一角点继续；账本不清（触发时刻基准） */
                                av_step  <= (av_step == 3'd5) ? 3'd5 : av_step + 3'd1; av_sub <= 2'd0; av_frm <= 2'd0;
                                av_near  <= 2'd0; av_far <= 2'd0; av_wait <= 16'd0;
                                av_unk   <= 1'b0; av3_n <= 2'd0;
                                spin_acc <= 32'sd0;
                                /* ★v73 方案B：此处不再清 IMU 累计 —— 保持与 av_deg 同样的触发时刻基准 */
                            end
                            else begin
                                st      <= S_IDLE;
                                resp_vld <= 1'b1; resp_type <= R_EVT; resp_arg <= 5'd3;
                            end
                        end
                        else begin
                            if (vx_slew < (spd_cur <<< 16)) vx_slew <= vx_slew + RAMP_STEP;
                            else                            vx_slew <= (spd_cur <<< 16);
                            vx_cmd <= vx_slew; vy_cmd <= 32'sd0;
                        end
                    end
                    3'd5: begin                              /* ★v64 组1：扇形扫描选向（六角点按需取用）*/
                        vx_cmd <= 32'sd0; vy_cmd <= 32'sd0; w_cmd <= 32'sd0;
                        av_wait <= (av_wait == 16'hffff) ? av_wait : (av_wait + 16'd1);
                        /*  角点查表：目标侧优先(90/75/45)，其次对侧(-90/-75/-45)；
                            0..5 全部走完仍堵 ⇒ EVT03。第 0 个角点与 v62 固定 90° 等价。 */
                        /* ★v71：角点查表已改为模块级组合 wire av_deg（见声明处），此处不再赋值 */
                        case (av_sub)
                        2'd0: begin                       /* 转向该角点（IMU 闭环 3°，以触发时刻为基准）*/
                            imu_odo_acc <= imu_odo_n;
                            if (((av_deg <<< 16) - imu_odo_n <= SCAN_TOL_Q16) &&
                                ((av_deg <<< 16) - imu_odo_n >= -SCAN_TOL_Q16)) begin
                                av_sub <= 2'd1; av_wait <= 16'd0;
                            end
                            else if (av_wait >= SCAN_TMO_TICKS) begin
                                av_sub <= 2'd3; av_near <= 2'd2;      /* 超时按堵 */
                            end
                            else
                                w_cmd <= ((av_deg <<< 16) > imu_odo_n) ? rot_cur : -rot_cur;
                        end
                        2'd1: begin                       /* 停稳 0.4s */
                            if (av_wait >= SCAN_SETTLE_TICKS) begin
                                av_wait <= 16'd0; av_frm <= 2'd0;
                                av_near <= 2'd0; av_far <= 2'd0; av_sub <= 2'd2;
                            end
                        end
                        2'd2: begin                       /* 采 4 帧（只看新鲜有效样本）*/
                            if (av_samp) begin
                                if (av_samp_ok && (av_samp_cm < SCAN_NEAR_CM))
                                    av_near <= av_near + 2'd1;
                                else if (av_samp_ok)
                                    av_far  <= av_far  + 2'd1;
                                av_frm <= (av_frm == 2'd3) ? av_frm : (av_frm + 2'd1);
                                if (av_frm >= (SCAN_FRAMES - 1)) av_sub <= 2'd3;
                            end
                            else if (av_wait >= SCAN_SAMPLE_TMO) begin
                                av_frm <= 2'd0; av_sub <= 2'd3;   /* ★v64：无回声超时 ⇒ 按已采样本判定（不得死等）*/
                            end
                        end
                        default: begin                    /* 2'd3 判定（单拍）*/
                            if (av_near >= 2'd2) begin                       /* 堵 */
                                /* ★v73：扫描判堵 ⇒ 补发 $EVT,04（观测用）。
                                   本分支单拍即转出（切角点或降级），每角点天然只发一次。*/
                                resp_vld <= 1'b1; resp_type <= R_EVT; resp_arg <= 5'd4;
                                if (av_step >= 3'd1) begin
                                    /* ★v66 降级：两个角点都堵 ⇒ **不报 EVT03**，改走目标侧 90°（= v63 行为）
                                       设计原则：扫描只做增强，不做新的失败点；最坏情况与保底版等价 */
                                    av_adv_ticks <= adv_tk_first;
                                    /* ★v66fix：此刻车头在"对侧角点"(av_deg=∓90，相对触发时刻)，
                                       要回到目标侧 ±90 ⇒ 相对当前朝向需再转 180°，不是 90° */
                                    spin_target  <= 32'sd180 <<< 16;
                                    spin_acc     <= 32'sd0;
                                    imu_odo_acc  <= 32'sd0;
                                    imu_th_prev  <= i_imu_yaw;
                                    av_unk       <= 1'b0;
                                    av_phase     <= 3'd1; av_cnt <= 32'd0;
                                end
                                else begin
                                    av_step <= av_step + 3'd1; av_sub <= 2'd0;
                                    av_frm <= 2'd0; av_near <= 2'd0; av_far <= 2'd0;
                                    av_unk <= 1'b0; av_wait <= 16'd0;
                                end
                            end
                            else if (av_far >= 2'd2) begin                   /* 空 ⇒ 选定 */
                                av_adv_ticks <= adv_tk_first;
                                av_side      <= av_deg[31];   /* 非冗余：step0 恒等、step1 恰好翻转 */
                                /* ★v66fix：探测角已把车头转到 av_deg（=侧移方向），不能再叠加 90°；
                                   直接进 3'd2；spin_target 固定 90° 留给 3'd3 转回原航向 */
                                spin_target  <= 32'sd90 <<< 16;
                                spin_acc     <= 32'sd0;
                                av_unk       <= 1'b0;
                                vx_slew <= 32'sd0; vx_cmd <= 32'sd0; vy_cmd <= 32'sd0;
                                av_near_cnt  <= 2'd0;   /* ★v67 修3：清残留，防补轮后首个近距样本立即中止 */
                                av_phase     <= 3'd2; av_cnt <= 32'd0;
                            end
                            else if (!av_unk) begin                          /* 未知 ⇒ 再采一轮 */
                                av_unk <= 1'b1; av_sub <= 2'd2;
                                av_frm <= 2'd0; av_near <= 2'd0; av_far <= 2'd0;
                            end
                            else begin
                                /* ★v66 关键：两轮采样仍未知 ⇒ **按"空"处理**，用当前角点试探
                                   理由：v65 "未知⇒按堵"导致六角全堵、12s 后直接 EVT03 停车，
                                        比保底版还差；而试探有 <20cm 中止兜底，撞不上。
                                   当前 av_step=0 的角点 = 目标侧 90°，与 v63 行为一致。 */
                                av_adv_ticks <= adv_tk_first;
                                av_side      <= av_deg[31];   /* 非冗余：step0 恒等、step1 恰好翻转 */
                                /* ★v66fix：探测角已把车头转到 av_deg（=侧移方向），不能再叠加 90°；
                                   直接进 3'd2；spin_target 固定 90° 留给 3'd3 转回原航向 */
                                spin_target  <= 32'sd90 <<< 16;
                                spin_acc     <= 32'sd0;
                                av_unk       <= 1'b0;
                                vx_slew <= 32'sd0; vx_cmd <= 32'sd0; vy_cmd <= 32'sd0;
                                av_near_cnt  <= 2'd0;   /* ★v67 修3：清残留，防补轮后首个近距样本立即中止 */
                                av_phase     <= 3'd2; av_cnt <= 32'd0;
                            end
                        end
                        endcase
                    end
                    3'd6: begin                              /* ★v64 组2：复测横向让开量 */
                        /*  侧移途中箱子出波束 ⇒ 前向声呐判据恒真；复测是判断
                            "横向够不够"的唯一可靠手段。 */
                        vx_cmd <= 32'sd0; vy_cmd <= 32'sd0; w_cmd <= 32'sd0;
                        av_wait <= (av_wait == 16'hffff) ? av_wait : (av_wait + 16'd1);
                        case (av_sub)
                        2'd0: begin                               /* 采 3 帧 */
                            if (av_samp_ok) begin
                                av3_3 <= av3_2; av3_2 <= av3_1; av3_1 <= av_samp_cm;
                                av3_n <= (av3_n == 2'd3) ? av3_n : (av3_n + 2'd1);
                            end
                            if ((av3_n >= 2'd2) || (av_wait >= RESCAN_TMO_TICKS)) begin
                                av_sub <= 2'd1; av_wait <= 16'd0;
                            end
                        end
                        default: begin                            /* 判定（单拍）*/
                            if ((av3_n >= 2'd2) && (av3_med > 16'd60)) begin
                                /* 采到帧且 >60cm ⇒ 已让开 ⇒ 平行 */
                                av_sub <= 2'd0; av_cnt <= 32'd0;
                                av_phase <= 3'd4;
                            end
                            else if (av3_n >= 2'd2) begin         /* 采到帧但 ≤60cm ⇒ 确实没让开 */
                                if (av_retry < 2'd2) begin
                                    av_retry     <= av_retry + 2'd1;
                                    av_adv_ticks <= adv_tk_retry;     /* 补 200mm */
                                    av_sub       <= 2'd0; av_unk <= 1'b0;
                                    av_cnt       <= 32'd0; av_wait <= 16'd0;
                                    spin_target  <= 32'sd90 <<< 16;   /* ★v72：显式钉死，不靠 3'd3 残值 */
                                    spin_acc     <= 32'sd0;
                                    imu_odo_acc  <= 32'sd0;
                                    imu_th_prev  <= i_imu_yaw;
                                    av_near_cnt  <= 2'd0;
                                    av_phase     <= 3'd1;
                                end
                                else begin
                                    st <= S_IDLE;
                                    resp_vld <= 1'b1; resp_type <= R_EVT; resp_arg <= 5'd3;
                                end
                            end
                            else if (av_unk) begin
                                /* ★v68：两轮均无有效回波 ⇒ 声呐视野内无物 ⇒ 按已让开，
                                   与 3'd5（1026-1043）"两轮未知 ⇒ 按空"原则统一。
                                   安全兜底仍在：3'd4 平行段有 obs_near20 连续 2 帧 <20cm 即中止。 */
                                av_sub <= 2'd0; av_cnt <= 32'd0; av_wait <= 16'd0;
                                av_phase <= 3'd4;
                            end
                            else begin                            /* 第一轮有效帧 <2 ⇒ 再采一轮 */
                                av_unk <= 1'b1; av_sub <= 2'd0;
                                av3_n <= 2'd0; av_wait <= 16'd0;
                            end
                        end
                        endcase
                    end
                    default: begin                           /* 非法态安全停车（决不承担"前进"） */
                        av_near_cnt <= 2'd0;
                        vx_cmd <= 32'sd0; vy_cmd <= 32'sd0; w_cmd <= 32'sd0;
                        vx_slew <= 32'sd0;
                        av_phase <= 3'd0; av_cnt <= 32'd0;
                        av_rearm_x_mm <= x_q16[31:16];   /* ★v60b ① */
                        av_rearm_y_mm <= y_q16[31:16];
                        av_rearm_v    <= 1'b1;
                        st <= S_NAV; ph <= PH_ALIGN;
                    end
                    endcase
                end
                else begin
                    vx_cmd <= 32'sd0; vy_cmd <= 32'sd0; w_cmd <= 32'sd0;
                end
            end
        end
    end

    // ★强制关断标志（顶层用它直接把 stdby 拉 0，物理断电机）—— 组合输出，避免双驱动
    always @(*) o_hold = (hold_cnt != 5'd0);

    // ---- v74：纯 assign 观测口（不驱动任何控制逻辑）----
    assign dbg_phase = av_phase;
    assign dbg_sub   = av_sub;
    assign dbg_step  = av_step;
    assign dbg_near  = av_near;
    assign dbg_far   = av_far;
    assign dbg_unk   = av_unk;
    assign dbg_samp  = av_samp ? av_samp_cm : 16'sd999;
    assign dbg_odo   = imu_odo_n;
    assign dbg_w     = w_cmd;
    assign dbg_gear  = i_gear;

endmodule
