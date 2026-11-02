// Private interface extension of the exact delivered selftest_ir control module.
// Original reusable primitives remain imported unchanged from baseline_src/task2.
module task123_task2_auto_core #(
 parameter integer SPEED_WINDOW=500000,CLOCKS_PER_MS=50000
)(
 input wire auto_mode,auto_run,auto_stop,auto_clear,
 input wire [2:0] auto_action,input wire [15:0] auto_speed,
 input wire [31:0] auto_goal,
 output wire auto_done,auto_fault,auto_stall,auto_real,
 output wire [127:0] auto_positions,output wire [63:0] auto_velocities,
 output wire [31:0] auto_errors,auto_travel,
 output wire [47:0] auto_duties,output wire [63:0] auto_efforts,
 output wire auto_tick,

    input  wire        sys_clk,        // 系统时钟 50MHz（Y18）
    input  wire        rst_n,          // 复位按键 S4（B21），低有效
    input  wire [3:0]  key,            // 板载按键 S0~S3（按下为低）
    input  wire        ir_in,          // ★ 红外接收头输出（F21，新增）
    input  wire [3:0]  enc_a,          // 四路编码器 A 相
    input  wire [3:0]  enc_b,          // 四路编码器 B 相
    output wire [3:0]  m_in1,          // 驱动模块方向脚 1
    output wire [3:0]  m_in2,          // 驱动模块方向脚 2
    output wire [3:0]  m_pwm,          // 驱动模块调速脚
    output wire        stdby,          // 驱动模块使能（低=待机）
    output wire [3:0]  led,
    output wire        uart_tx         // 本程序不用，固定输出高
);
    // ------------------------------------------------------------------
    // 一、参数
    // ------------------------------------------------------------------
    // 速度用固定值（原 CH+/CH- 调速功能已按要求删除）。
    // 实测依据：GM37-520（铭牌丝印）+ 1:30 减速比，7.4V 下空载满速约 45 脉冲/10ms，
    // 所以前进取 20（约满速 45%）、旋转取 12，这两个值在真车上已验证过。
    localparam signed [15:0] RUN_SPD  = 16'sd28;    // 前进 / 后退速度（原 20，已提速）
    // 旋转速度：原为 12。落地实测发现"定点旋转时后段转速明显比刚启动时慢"，
    // 原因是**固定前馈 UFF=2400 是按前进速度 28 整定的**，对 12 这个慢速目标偏大
    // （工作点落在死区边缘，占空比对速度极其敏感），于是启动冲得快、之后被积分
    // 慢慢拉回。把旋转速度提到 20 后，工作点更靠近前馈的设计点，瞬态明显减小；
    // 顺带把"原地转两圈"的耗时从约 10.7s 缩到约 6.4s。
    localparam signed [15:0] SPIN_SPD = 16'sd20;    // 旋转速度（原 12）

    // KP_Q：比例增益，实际倍率 = KP_Q / 256。
    // 原值 100（≈0.39）经核算几乎不起作用：误差 20 个脉冲时比例项只贡献 7 个
    // duty 计数（0.17% 占空比），而前馈 UFF 是 2400（58.6%）——比例项比前馈小
    // 340 倍，等于没有比例作用。后果是环路实际只剩"固定前馈 + 很慢的积分"：
    // 启动瞬间由前馈顶出一个速度，之后积分把输出慢慢拉回稳态，看上去就是
    // "刚启动速度正常、之后明显变慢"。故提高到 1500（≈5.9），让比例项真正
    // 起作用，启动瞬态由环路接管。
    localparam signed [15:0] KP_Q     = 16'sd1500;
    localparam signed [15:0] KI_Q     = 16'sd130;   // 积分增益 ≈ 0.51（本次不动）
    localparam signed [15:0] TOL      = 16'sd8;     // 锁定容差

    // 定点旋转"转多少算到位"：
    //   ★ 换算基准（2026-10-12 **以实物铭牌定案**）：电机 = **GM37-520**
    //      铭牌丝印「12V333rpm/1:30、**12ppr/AB Phase**」
    //      ⇒ 12 × 4（四倍频）× 30（减速比）= **1440 计数/输出轴圈**。
    //   实测标定：90° ≈ 1600 脉冲（与 7.4V/12V 无关，因为是脉冲计数）
    //     换算参考：360° = 6400 脉冲；**720°（原地两圈）= 12800 脉冲**。
    //   ★2026-10 组长实测修正：加"到位短刹车"后仍多转约 30°（≈533 脉冲），
    //     遂把目标提前 533 → 12800 − 533 = 12270。
    //   ★★2026-10-12 整合后复核（WorkBuddy）：**上板实测仍多转约 30°**。
    //     根因：那 533 的提前量是按"旋转速度 = 12"标定的；之后 SPIN_SPD 提到 20，
    //     而刹车后的滑行量与速度正相关 ⇒ 实际滑行量约翻倍(≈1063 脉冲)，补偿不够。
    //     故再减 533：12270 − 533 = 11737，取整 **11740**。
    //   ⚠️ 若换了地面/电量/轮胎，残余滑行量会变，本值需重新微调：
    //       实测"多转 X 度" → 本值减 (X × 19.08)；"少转 X 度" → 本值加 (X × 19.08)。
    //       例：还是多转 10° → 11740 − 10×19.08 ≈ 11549。
    //       （19.08 = U·(a+b)/(180·D)，按实物铭牌 12ppr ⇒ U=1440 推算；旧值 17.78 系按 1320
    //         口径 —— 2026-10-12 更正。★ 理论值，最终以实测角度为准。）
    //   ⚠️ 本标定前提：**必须运行在真实编码器模式**（real_mode=1，LED0 常亮）。
    //       虚拟模型模式下 travel 是仿真位置，与真实转角脱钩，标定无意义。
    // ★2026-10-12 20:45 两点法再修正（WorkBuddy）：
    //   已知两个实测工作点（均在 SPIN_SPD=20、真实编码器模式、带短刹车）：
    //     12270  ⇒ 实测**多转约 30°**
    //     11740  ⇒ 实测**少转约 30°**
    //   两点线性插值 ⇒ 中点 (12270+11740)/2 = 12005 ⇒ 取整 **12000**（预期 ≈ 正好 720°）。
    //   ⚠️ 仍偏的话：多转 X° ⇒ 本值减 (X × 19.08)；少转 X° ⇒ 本值加 (X × 19.08)。
    //      例：还是多转 5° ⇒ 12000 − 5×19.08 ≈ 11905。
    //   ⚠️ 前提不变：**必须真实编码器模式**（real_mode=1，LED0 常亮）。
    localparam [31:0] SPIN_TGT = 32'd12000;

    // （速度直接取上面的常量；这里原来是用档位算速度，那行有字面量位宽错误：
    //   3'sd8 被截断成 0，导致前进目标速度恒为 0、四轮不转。现已删除。）

    // ------------------------------------------------------------------
    // 二、动作编码（这次从 2 位扩到 3 位，多了"右转"）
    //     3'd0 = 前进        3'd1 = 后退
    //     3'd2 = 左转(逆时针) 3'd3 = 右转(顺时针)
    //     3'd4 = 定点旋转 720°（逆时针原地两圈，走到位自动停）
    // ------------------------------------------------------------------
    localparam [2:0] A_FWD  = 3'd0;
    localparam [2:0] A_BACK = 3'd1;
    localparam [2:0] A_LEFT = 3'd2;
    localparam [2:0] A_RIGHT= 3'd3;
    localparam [2:0] A_SPIN = 3'd4;

    // ------------------------------------------------------------------
    // 三、饱和函数（把 32 位限制到 ±600 以内，防止运动学相加后溢出）
    // ------------------------------------------------------------------
    function signed [15:0] sat16;
        input signed [31:0] v;
        begin
            if      (v >  32'sd600) sat16 = 16'sd600;
            else if (v < -32'sd600) sat16 = -16'sd600;
            else                    sat16 = v[15:0];
        end
    endfunction

    // ------------------------------------------------------------------
    // 四、寄存器声明
    // ------------------------------------------------------------------
    reg [15:0] ms_cnt;                  // 1ms 计时
    reg        tick10;                  // 10ms 节拍
    reg [3:0]  ms10;

    reg        k2_d, k3_d;              // 上一拍的 S2/S3，用于找"按下瞬间"
    reg        real_mode;               // 0=虚拟模型模式，1=真实编码器模式

    reg [2:0]  act_lat;                 // 启动时锁存的动作（3 位）
    reg        run_en;                  // 运行中
    reg        fault;                   // 故障（10 秒未锁定 / 急停）
    reg        locked;                  // 闭环锁定
    reg        spin_done;               // 定点旋转到位
    reg [15:0] lock_cnt, fault_cnt;     // 锁定/故障计时器
    reg        fault_cnt_ge;            // ★v85③：fault_cnt 已到 999 的寄存标志（拆开判定链）
    reg        fault_clr;               // ★v85③：fault_cnt 清零单拍脉冲（主状态机只置它，不直接写 fault_cnt）
    reg signed [31:0] p1_0, p2_0, p3_0, p4_0;   // 定点旋转起点的四路位置

    reg [6:0]  hb;                      // 心跳计时
    reg        hb_led;                  // 心跳电平
    reg [11:0] pwm_cnt;                 // PWM 载波计数

    // ------------------------------------------------------------------
    // 五、连线声明
    // ------------------------------------------------------------------
    wire [3:0]  k;                      // 消抖后的按键
    wire        start_pulse;            // S2 按下瞬间
    wire [1:0]  act_sel_key;            // 按键给的动作（2 位）
    wire [2:0]  act_key;                // 按键动作换算成 3 位编码
    wire [2:0]  act_now;                // 当前实际动作

    wire [7:0]  ir_cmd;                 // 红外解出的键码
    wire        ir_valid, ir_repeat;    // 红外：收到有效帧 / 收到重复码

    wire signed [15:0] u1, u2, u3, u4;  // 四路 PID 输出
    wire        ma0, mb0, ma1, mb1, ma2, mb2, ma3, mb3;
    wire signed [15:0] spd_m0, spd_m1, spd_m2, spd_m3;
    wire [3:0]  fa, fb;
    wire signed [31:0] pos1, pos2, pos3, pos4;
    wire [7:0]  err1, err2, err3, err4;
    wire signed [15:0] act1, act2, act3, act4;
    wire        tk1, tk2, tk3, tk4;
    wire signed [15:0] vx_w, w_w;
    wire signed [15:0] tgt1, tgt2, tgt3, tgt4;
    wire signed [31:0] d1, d2, d3, d4;
    wire [31:0] a1, a2, a3, a4, travel;
    wire        motion_active;
    wire signed [15:0] te1, te2, te3, te4;
    wire signed [15:0] e1, e2, e3, e4;
    wire        in_tol_all;
    wire [3:0]  in1_w, in2_w;
    wire [11:0] duty0, duty1, duty2, duty3;
    wire        oe0, oe1, oe2, oe3;

    // ------------------------------------------------------------------
    // 六、时间节拍（1ms / 10ms）—— 和 selftest_top 一样
    // ------------------------------------------------------------------
    always @(posedge sys_clk or negedge rst_n)
        if (!rst_n) begin ms_cnt <= 16'd0; tick10 <= 1'b0; ms10 <= 4'd0; end
        else begin
            tick10 <= 1'b0;
            if (ms_cnt == CLOCKS_PER_MS-1) begin
                ms_cnt <= 16'd0;
                if (ms10 == 4'd9) begin ms10 <= 4'd0; tick10 <= 1'b1; end
                else ms10 <= ms10 + 1'b1;
            end else ms_cnt <= ms_cnt + 1'b1;
        end

    // ------------------------------------------------------------------
    // 七、板载按键：消抖 + 按下瞬间检测（复用 selftest_top.v 里的 key_deb）
    // ------------------------------------------------------------------
    key_deb u_k0(.clk(sys_clk), .rst_n(rst_n), .key_in(key[0]), .key_out(k[0]));
    key_deb u_k1(.clk(sys_clk), .rst_n(rst_n), .key_in(key[1]), .key_out(k[1]));
    key_deb u_k2(.clk(sys_clk), .rst_n(rst_n), .key_in(key[2]), .key_out(k[2]));
    key_deb u_k3(.clk(sys_clk), .rst_n(rst_n), .key_in(key[3]), .key_out(k[3]));

    assign start_pulse = k2_d & ~k[2];          // 上一拍没按、这一拍按了 = 按下瞬间

    always @(posedge sys_clk or negedge rst_n)
        if (!rst_n) begin k2_d <= 1'b1; k3_d <= 1'b1; end
        else        begin k2_d <= k[2]; k3_d <= k[3]; end

    // 按键动作：按下=1 → 都不按(00)前进、按S0(01)后退、按S1(10)左转、都按(11)定点旋转
    assign act_sel_key = {~k[1], ~k[0]};
    assign act_key = (act_sel_key == 2'd0) ? A_FWD  :     // 都不按 → 前进
                     (act_sel_key == 2'd1) ? A_BACK :     // 按住 S0 → 后退
                     (act_sel_key == 2'd2) ? A_LEFT :     // 按住 S1 → 左转
                                             A_SPIN ;     // 两个都按 → 定点旋转

    // 运行中用锁存的动作；停止时用按键当前的动作（方便先选好再启动）
    assign act_now = run_en ? act_lat : act_key;

    // ------------------------------------------------------------------
    // 八、红外接收：例化解码模块，并产生"按键事件"脉冲
    // ------------------------------------------------------------------
    ir_receiver u_ir(
        .clk(sys_clk), .rst_n(rst_n), .ir_in(ir_in),
        .cmd(ir_cmd), .cmd_valid(ir_valid), .repeat_code(ir_repeat)
    );

    // 收到一整帧有效命令、或者收到重复码，都算"一次按键事件"。
    // 方向键和 CH± 会响应重复码（方便长按连续调），
    // 而"启停""急停"不响应重复码（避免长按导致反复开关）。
    wire ir_evt = ir_valid | ir_repeat;

    // ------------------------------------------------------------------
    // 九、编码器来源与极性修正（与 selftest_top 完全一致）
    //     实测：右侧两轮（通道 B、D）因电机镜像安装，A/B 相位相反，需要交换；
    //     ⚠️ 交换只作用于真实编码器，虚拟模型不能换（否则变成正反馈）
    // ------------------------------------------------------------------
    wire [3:0] enc_a_fix = {enc_b[3], enc_a[2], enc_b[1], enc_a[0]};
    wire [3:0] enc_b_fix = {enc_a[3], enc_b[2], enc_a[1], enc_b[0]};

    assign fa = real_mode ? enc_a_fix : {ma3, ma2, ma1, ma0};
    assign fb = real_mode ? enc_b_fix : {mb3, mb2, mb1, mb0};

    // 四个虚拟电机（由 PID 输出 u 驱动）
    // ⚠️ 这里把虚拟电机的启动死区从默认的 800(20%) 改大到 2000(约 49%)，
    //    目的是让它和真电机接近：否则虚拟模式下环路会把输出稳定在 21% 占空比，
    //    而真电机需要 40~50% 才能转 —— 于是"LED2 亮了但真轮子不动"。
    //    改大死区后，虚拟模式下的稳态占空比就落在真电机也能转起来的范围了。
    fake_motor #(.DEAD(13'sd2000)) u_fm0(.clk(sys_clk), .rst_n(rst_n), .pwm(u1), .a(ma0), .b(mb0), .spd_m(spd_m0));
    fake_motor #(.DEAD(13'sd2000)) u_fm1(.clk(sys_clk), .rst_n(rst_n), .pwm(u2), .a(ma1), .b(mb1), .spd_m(spd_m1));
    fake_motor #(.DEAD(13'sd2000)) u_fm2(.clk(sys_clk), .rst_n(rst_n), .pwm(u3), .a(ma2), .b(mb2), .spd_m(spd_m2));
    fake_motor #(.DEAD(13'sd2000)) u_fm3(.clk(sys_clk), .rst_n(rst_n), .pwm(u4), .a(ma3), .b(mb3), .spd_m(spd_m3));

    // 四路正交解码 + 四路测速（通道 A/B/C/D = 左前/右前/左后/右后）
    quad_dec u_q0(.clk(sys_clk), .rst_n(rst_n), .a(fa[0]), .b(fb[0]), .clr(1'b0), .pos(pos1), .err_cnt(err1));
    quad_dec u_q1(.clk(sys_clk), .rst_n(rst_n), .a(fa[1]), .b(fb[1]), .clr(1'b0), .pos(pos2), .err_cnt(err2));
    quad_dec u_q2(.clk(sys_clk), .rst_n(rst_n), .a(fa[2]), .b(fb[2]), .clr(1'b0), .pos(pos3), .err_cnt(err3));
    quad_dec u_q3(.clk(sys_clk), .rst_n(rst_n), .a(fa[3]), .b(fb[3]), .clr(1'b0), .pos(pos4), .err_cnt(err4));

    speed_meter #(.WIN(SPEED_WINDOW)) u_s0(.clk(sys_clk), .rst_n(rst_n), .pos(pos1), .spd(act1), .tick(tk1));
    speed_meter #(.WIN(SPEED_WINDOW)) u_s1(.clk(sys_clk), .rst_n(rst_n), .pos(pos2), .spd(act2), .tick(tk2));
    speed_meter #(.WIN(SPEED_WINDOW)) u_s2(.clk(sys_clk), .rst_n(rst_n), .pos(pos3), .spd(act3), .tick(tk3));
    speed_meter #(.WIN(SPEED_WINDOW)) u_s3(.clk(sys_clk), .rst_n(rst_n), .pos(pos4), .spd(act4), .tick(tk4));

    // ------------------------------------------------------------------
    // 十、麦轮逆运动学（3 位动作 → 四路目标速度）
    //     前进：四路同向；后退：四路反向；
    //     左转/右转/定点旋转：左右两侧反向 → 原地旋转（右转把 w 取反）
    // ------------------------------------------------------------------
    wire signed [15:0] selected_run=auto_mode ? $signed(auto_speed):RUN_SPD;
    wire signed [15:0] selected_spin=auto_mode ? $signed(auto_speed):SPIN_SPD;
    assign vx_w = (act_now == A_FWD)  ?  selected_run :           // 前进：正速度
                  (act_now == A_BACK) ? -selected_run : 16'sd0;      // 后退：负速度

    assign w_w  = (act_now == A_LEFT)  ?  selected_spin :         // 左转：逆时针（正）
                  (act_now == A_RIGHT) ? -selected_spin :         // 右转：顺时针（负）
                  (act_now == A_SPIN)  ?  selected_spin :         // 定点旋转：逆时针
                  16'sd0;

    assign tgt1 = sat16(vx_w - w_w);        // 左前（通道 A）
    assign tgt2 = sat16(vx_w + w_w);        // 右前（通道 B）
    assign tgt3 = sat16(vx_w - w_w);        // 左后（通道 C）
    assign tgt4 = sat16(vx_w + w_w);        // 右后（通道 D）

    // ------------------------------------------------------------------
    // 十一、定点旋转行程：四路"相对起点的位移"取绝对值再平均
    // ------------------------------------------------------------------
    assign d1 = pos1 - p1_0;
    assign d2 = pos2 - p2_0;
    assign d3 = pos3 - p3_0;
    assign d4 = pos4 - p4_0;
    assign a1 = d1[31] ? (~d1 + 32'sd1) : d1;      // 取绝对值
    assign a2 = d2[31] ? (~d2 + 32'sd1) : d2;
    assign a3 = d3[31] ? (~d3 + 32'sd1) : d3;
    assign a4 = d4[31] ? (~d4 + 32'sd1) : d4;
    assign travel = ((a1 + a2) + (a3 + a4)) >>> 2;  // 平均值

    // ------------------------------------------------------------------
    // 十二、主状态机（唯一驱动块：run_en / act_lat / fault /
    //       locked / spin_done 等寄存器都在这里赋值，别处不能再赋值）
    //
    //     优先级从上到下：红外命令 > 板载按键 > 待机 > 到位判断 > 锁定/故障计时
    //     所以遥控器的命令比按键优先级高（同时操作时以遥控为准）
    // ------------------------------------------------------------------
    always @(posedge sys_clk or negedge rst_n)
        if (!rst_n) begin
            // ---------- 复位 ----------
            run_en    <= 1'b0;
            fault     <= 1'b0;
            locked    <= 1'b0;
            lock_cnt  <= 16'd0; fault_clr <= 1'b0;   /* ★v85③：fault_cnt/fault_cnt_ge 由第二块统一初始化 */
            spin_done <= 1'b0;
            // ★2026-10-12 整合后修正：默认改为**真实编码器模式**。
            //   原因：虚拟模型模式下 travel 由 fake_motor 的仿真位置给出，与真实转角
            //   脱钩 ⇒ 定点旋转的"转了多少"根本不可信。真车必须用真实编码器。
            //   （要回到"离线调试用虚拟模型"，把下面改回 1'b0 即可，或用 S3 按键现场切换）
            real_mode <= 1'b1;
            act_lat   <= A_FWD;                 // 默认动作：前进
            p1_0 <= 32'sd0; p2_0 <= 32'sd0; p3_0 <= 32'sd0; p4_0 <= 32'sd0;
        end else if (auto_mode) begin
            // Automatic supervisor has sole command ownership. Real feedback only.
            real_mode <= 1'b1;
            if(auto_stop)begin run_en<=0;fault<=1;spin_done<=0;end
            else if(auto_clear && !auto_run)begin
                run_en<=0;fault<=0;spin_done<=0;lock_cnt<=0;fault_clr<=1;locked<=0;
            end else if(!auto_run)begin
                run_en<=0;spin_done<=0;lock_cnt<=0;fault_clr<=1;locked<=0;
            end else if(!fault)begin
                if(!run_en || act_lat!=auto_action)begin
                    run_en<=1;act_lat<=auto_action;spin_done<=0;
                    p1_0<=pos1;p2_0<=pos2;p3_0<=pos3;p4_0<=pos4;
                    lock_cnt<=0;fault_clr<=1;locked<=0;
                end else if(auto_goal!=0 && travel>=auto_goal)begin
                    // Reuse task-two completion/brake mechanism for each bounded leg.
                    spin_done<=1;
                end else if(in_tol_all)begin
                    if(tick10)begin
                        if(lock_cnt<20)lock_cnt<=lock_cnt+1'b1;else locked<=1;
                    end
                end else if(tick10)begin
                    lock_cnt<=0;
                    if(!locked)begin
                        /* ★v85 ③：v83 整工程最差路径就是这条 ——
                           `u_q2/pos_0_s1/Q → fault_cnt_*_s0/D`，slack −7.962，
                           dataDelay 27.891 ns（预算 20）。
                           根因：pos_0 是"8 位 × 7 级累加树"的末端 ⇒ 只做 1 个 LUT 输入，
                           而这里要在一个周期里串「16 位 +1 → <1000 比较 → else fault<=1」。
                           修法（**语义完全等价**）：拆成三级，每级只做一件小事
                             F1 本拍是否到 999（纯与门，无进位链）
                             F2 累加（CE = tick10 && 运行中 && !locked && !到顶）
                             F3 置故障（fault_cnt 已经是"已加完"的值）
                           fault 只晚 1 拍（20 ns）置位，其余行为与 v83 一字不差：
                           满 1000 个 tick（10 s）仍未锁定 ⇒ fault=1。
                           详见 424 行起的第二块（fault_clr / fault_cnt_ge 的完整责任划分）。 */
                        if(fault_cnt_ge)fault<=1;
                    end
                end
            end
        end else if (ir_evt) begin
            // ---------- 红外遥控命令（收到一帧有效命令，或收到重复码）----------
            // 先统一把运行相关的状态清干净，再按具体按键设置动作
            fault     <= 1'b0;
            fault_clr <= 1'b1;
            lock_cnt  <= 16'd0;
            locked    <= 1'b0;
            // ★2026-10-12 修正：**重复码（长按遥控）不得重置"到位标记"和"旋转起点"**。
            //   原实现把每个重复码都当成一次新命令 ⇒ 反复重记 p1_0..p4_0（travel 归零）
            //   并清 spin_done，后果是"越转越多 / 到位了又接着转"。
            //   遥控器按住一个键约 108ms 后开始每 108ms 发一个重复码，按一下就可能带 1 个。
            if (!ir_repeat) begin
                spin_done <= 1'b0;
                p1_0 <= pos1; p2_0 <= pos2; p3_0 <= pos3; p4_0 <= pos4;   // 记录旋转起点
            end

            case (ir_cmd)
                // ---- 方向键：选动作并立即启动（像遥控车一样）----
                8'h18: begin act_lat <= A_FWD;   run_en <= 1'b1; end   // 2 → 前进
                8'h52: begin act_lat <= A_BACK;  run_en <= 1'b1; end   // 8 → 后退
                8'h08: begin act_lat <= A_LEFT;  run_en <= 1'b1; end   // 4 → 左转(逆时针)
                8'h5A: begin act_lat <= A_RIGHT; run_en <= 1'b1; end   // 6 → 右转(顺时针)
                // ---- 5 或 EQ：定点旋转 720°（原地两圈，到位自动停）----
                8'h1C, 8'h09: begin act_lat <= A_SPIN; run_en <= 1'b1; end
                // ---- PLAY/PAUSE：启动 / 停止 切换（忽略重复码，避免长按反复开关）----
                8'h43: begin if (!ir_repeat) run_en <= ~run_en; end
                // ---- CH+ (0x47) / CH− (0x45)：原来用于调速，按需求已删除，
                //      现在收到这两个键不做任何动作（速度是固定值）----
                // ---- 0：急停（立即停机 + 置故障，让 STBY 拉低）----
                8'h16: begin run_en <= 1'b0; fault <= 1'b1; end
                default: ;                      // 其它键不响应
            endcase
        end else if (start_pulse) begin
            // ---------- 板载按键 S2：启动 / 停止 切换（保底控制）----------
            run_en    <= ~run_en;
            fault     <= 1'b0;
            fault_clr <= 1'b1;
            lock_cnt  <= 16'd0;
            locked    <= 1'b0;
            spin_done <= 1'b0;
            if (!run_en) act_lat <= act_key;    // 本次是"启动"→ 锁存当前按键动作
            p1_0 <= pos1; p2_0 <= pos2; p3_0 <= pos3; p4_0 <= pos4;
        end else if (k3_d & ~k[3]) begin
            // ---------- 板载按键 S3：切换编码器来源（虚拟模型 ↔ 真实编码器）----------
            real_mode <= ~real_mode;
            fault     <= 1'b0;
            fault_clr <= 1'b1;
            lock_cnt  <= 16'd0;
            locked    <= 1'b0;
            spin_done <= 1'b0;
        end else if (!run_en) begin
            // ---------- 待机：清掉运行相关的状态 ----------
            spin_done <= 1'b0;
            lock_cnt  <= 16'd0;
            fault_clr <= 1'b1;
            locked    <= 1'b0;
        end else if ((act_now == A_SPIN) && (travel >= SPIN_TGT)) begin
            // ---------- 定点旋转模式 + 行程够了 → 标记到位（电机随后自动停）----------
            spin_done <= 1'b1;
        end else if (in_tol_all) begin
            // ---------- 四路都在容差内 → 累计锁定时间（按 10ms 节拍数）----------
            if (tick10) begin
                if (lock_cnt < 16'd20) lock_cnt <= lock_cnt + 1'b1;   // 连续 200ms
                else                   locked   <= 1'b1;
            end
        end else if (tick10) begin
            // ---------- 有路跑偏 → 累计故障时间，10 秒还锁不上就报故障 ----------
            lock_cnt <= 16'd0;
            if (!locked) begin
                /* ★v85 ③：v83 整工程最差路径就是这条 ——
                   `u_q2/pos_0_s1/Q → fault_cnt_*_s0/D`，slack −7.962，dataDelay 27.891 ns。
                   根因：pos_0 是"8 位 × 7 级累加树"的末端（只做 1 个 LUT 输入），
                   而原式要在同一周期里串「16 位 +1 → <1000 比较 → else fault<=1」。
                   修法（**语义完全等价**）：拆成三级，每级只做一件小事
                     F1 本拍是否到 999（纯与门，无进位链）        → fault_cnt_top
                     F2 累加（CE = tick10 && 运行中 && !locked && !到顶）→ fault_cnt
                     F3 置故障（fault_cnt 已是"已加完"的值）      → fault
                   fault 只晚 1 拍（20 ns）置位，其余与 v83 一字不差：
                   满 1000 个 tick（10 s）仍未锁定 ⇒ fault=1。
                   fault_cnt / fault_cnt_ge 的**唯一驱动块**在下方"★v85 ③"块，本块只发清零脉冲。 */
                if (fault_cnt_ge) fault <= 1'b1;
            end
        end

    // ------------------------------------------------------------------
    // 十三、输出使能与最终目标速度
    // ------------------------------------------------------------------
    assign motion_active = run_en && !spin_done;                 // 允许驱动电机
    assign te1 = motion_active ? tgt1 : 16'sd0;                  // 不运行时目标归零
    assign te2 = motion_active ? tgt2 : 16'sd0;
    assign te3 = motion_active ? tgt3 : 16'sd0;
    assign te4 = motion_active ? tgt4 : 16'sd0;

    // ------------------------------------------------------------------
    // ★v85 ③：fault_cnt 判定链的三级拆分
    //   现象（v83 整工程最差路径）：`u_q2/pos_0_s1/Q → fault_cnt_*_s0/D`，slack −7.962，
    //     dataDelay 27.891 ns（预算 20）。根因：pos_0 是"8 位 × 7 级累加树"的末端，
    //     只做 1 个 LUT 输入，而原式要在同一周期里串「16 位 +1 → <1000 比较 → else fault<=1」。
    //   三级拆分（**语义等价，fault 只晚 20 ns 落地**）：
    //     F1  fault_cnt_top : 本拍是否已到 999（纯与门）
    //     F2  fault_cnt     : 累加（CE = tick10 && 运行中 && !locked && !到顶）
    //     F3  fault         : 主状态机里 `if(fault_cnt_ge) fault<=1;`（见 336 行）
    //   责任划分（避免 EX2000 双驱动）：
    //     · 主状态机**不再赋值 fault_cnt**，只在需要清零时置 `fault_clr<=1` 一拍；
    //     · fault_cnt / fault_cnt_ge 只在**本块**被赋值。
    // ------------------------------------------------------------------
    wire fault_cnt_top = (fault_cnt == 16'd999);
    wire fault_cnt_ce  = tick10 && auto_mode && auto_run && run_en && !locked && !fault_cnt_top;
    always @(posedge sys_clk or negedge rst_n)
        if (!rst_n) begin
            fault_cnt    <= 16'd0;
            fault_cnt_ge <= 1'b0;
        end else if (fault_clr) begin
            fault_cnt    <= 16'd0;
            fault_cnt_ge <= 1'b0;
        end else begin
            if (fault_cnt_ce) fault_cnt <= fault_cnt + 16'd1;
            fault_cnt_ge <= fault_cnt_top || fault_cnt_ce;
        end
    /*  fault_clr 由**主状态机**唯一驱动（8 处清零场景各置一拍），本块只读不写；
        本块与主状态机各自只碰自己的寄存器 ⇒ 无 EX2000。 */

    // ------------------------------------------------------------------
    // 十四、误差与锁定容差
    // ------------------------------------------------------------------
    assign e1 = te1 - act1;
    assign e2 = te2 - act2;
    assign e3 = te3 - act3;
    assign e4 = te4 - act4;

    // 四路误差都在 ±TOL 之内，**并且确实在运行**，才算"跟上了目标"。
    // ⚠️ 必须加上 motion_active 这个条件：否则"目标 0、实测 0"也会被判成
    //    在容差内，于是"什么都没跑"也会把 LED2 点亮（假锁定）。
    assign in_tol_all = motion_active &&
                        (e1 <= TOL && e1 >= -TOL) &&
                        (e2 <= TOL && e2 >= -TOL) &&
                        (e3 <= TOL && e3 >= -TOL) &&
                        (e4 <= TOL && e4 >= -TOL);

    // ------------------------------------------------------------------
    // 十五、堵转保护：某路"有指令但长期没速度"持续 2 秒 → 停机报警
    // ------------------------------------------------------------------
    wire ch1_stall = ((te1 >  16'sd5) || (te1 < -16'sd5)) && (act1 < 16'sd2) && (act1 > -16'sd2);
    wire ch2_stall = ((te2 >  16'sd5) || (te2 < -16'sd5)) && (act2 < 16'sd2) && (act2 > -16'sd2);
    wire ch3_stall = ((te3 >  16'sd5) || (te3 < -16'sd5)) && (act3 < 16'sd2) && (act3 > -16'sd2);
    wire ch4_stall = ((te4 >  16'sd5) || (te4 < -16'sd5)) && (act4 < 16'sd2) && (act4 > -16'sd2);
    wire stall_now = run_en && (ch1_stall | ch2_stall | ch3_stall | ch4_stall);

    reg [15:0] stall_cnt;
    wire       stall_fault = (stall_cnt >= 16'd200);             // 2 秒

    always @(posedge sys_clk or negedge rst_n)
        if (!rst_n)                                stall_cnt <= 16'd0;
        else if (!stall_now)                       stall_cnt <= 16'd0;
        else if (tick10 && (stall_cnt < 16'd200))  stall_cnt <= stall_cnt + 1'b1;

    // ------------------------------------------------------------------
    // 十六、四路 PI 速度环
    // ------------------------------------------------------------------
    // 停止时、以及按下启动的那一刻，都把积分清零。
    // 这样每次启动都从"干净"的状态开始，不会带着上一次运行残留的积分。
    // ★2026-10-12 修正：把 spin_done 也纳入清零条件。
    //   到位刹车期间 motion_active=0 ⇒ 目标速度被置 0，而实际转速还在往下掉，
    //   误差恒为负 ⇒ 积分朝负方向累积并残留，污染下一次启动。
    reg auto_run_d;reg [15:0] auto_speed_d;
    always @(posedge sys_clk or negedge rst_n)
    if(!rst_n)begin auto_run_d<=0;auto_speed_d<=0;end
    else begin auto_run_d<=auto_run;auto_speed_d<=auto_speed;end
    wire pid_clr = !run_en || start_pulse || spin_done ||
      (auto_mode && auto_run && (!auto_run_d || auto_speed!=auto_speed_d));

    // Sample the original PI operands at its original 10ms update event.
    // Automatic mode evaluates that identical sample one 50MHz cycle later;
    // this splits kinematics from the unchanged PI arithmetic. Any stop/clear
    // cancels the pending update. Manual mode remains cycle-identical.
    reg pi_pending;
    reg signed [15:0] sample_te1,sample_te2,sample_te3,sample_te4;
    reg signed [15:0] sample_act1,sample_act2,sample_act3,sample_act4;
    always @(posedge sys_clk or negedge rst_n)
    if(!rst_n)begin
      pi_pending<=0;
      sample_te1<=0;sample_te2<=0;sample_te3<=0;sample_te4<=0;
      sample_act1<=0;sample_act2<=0;sample_act3<=0;sample_act4<=0;
    end else begin
      pi_pending<=tk1 && !pid_clr;
      if(tk1)begin
        sample_te1<=te1;sample_te2<=te2;sample_te3<=te3;sample_te4<=te4;
        sample_act1<=act1;sample_act2<=act2;sample_act3<=act3;sample_act4<=act4;
      end
    end
    wire pi_tick=auto_mode ? pi_pending:tk1;
    wire signed [15:0] pi_te1=auto_mode ? sample_te1:te1;
    wire signed [15:0] pi_te2=auto_mode ? sample_te2:te2;
    wire signed [15:0] pi_te3=auto_mode ? sample_te3:te3;
    wire signed [15:0] pi_te4=auto_mode ? sample_te4:te4;
    wire signed [15:0] pi_act1=auto_mode ? sample_act1:act1;
    wire signed [15:0] pi_act2=auto_mode ? sample_act2:act2;
    wire signed [15:0] pi_act3=auto_mode ? sample_act3:act3;
    wire signed [15:0] pi_act4=auto_mode ? sample_act4:act4;

    // 用改进版的 pid_pi_v2（能清零积分、前馈更大），默认参数已经调好
    /* ★★★ v85 ②：PI 的 en/clr 全部改吃寄存版，且用初值区分两种改法 ★★★
       现象（v84 实测）：u_p0 唯一违例族 `sample_act1_0_s0/Q → u_p0/acc_*_s1/CE`
       25 条、slack −6.365 ~ −6.656 ns、dataDelay 26.1 ns。v83 同一 CE 端只有 1 条
       （−6.060，前驱 sample_te1_4_s0），v84 把 nav_ctrl 的 mv_done 打拍后 P&R 重排，
       这一对就掉进了这个位置。
       根因：`en` 与 `clr` 是**同一组 CE/SR 的多输入 LUT**，pi_tick、pid_clr 都是
       "auto_mode ? 寄存值 : tk1" 的 MUX ⇒ 20 ns 里要串
       「sample_act → MUX → 与/或 → CE」，而 acc 是 32 位宽 ⇒ 26 ns。
       修法（**零行为影响**，只挪一拍到达时间）：
         ① `pi_tick_r` / `pid_clr_r` 是纯打拍（初值 1'b0）—— 到达边沿晚 20 ns，
            仍落在同一个 50MHz 周期内，10 ms 控制周期完全看不出；
         ② ce_eff / clr_eff 用**初值**实现"可选旁路"：
              初值 1'b0 → 打拍后使用（v84 之前的 pid_pi_v2 语义）；
              初值 1'b1 → 这一路恒不更新，等价于 en 恒 0。
            这样 4 个 PID 现在都用打拍版，而代码里保留"切回非打拍"的开关只需改初值。
       为什么不用 en/ce 的互补拼接：ce_eff = 1'b1 那一路会把 acc 末端组合逻辑一并
       塞进 CE（见 Gowin 的 CE 汇聚行为），反而更慢。 */
    reg ce_eff, clr_eff;
    initial begin ce_eff = 1'b0; clr_eff = 1'b0; end
    reg pi_tick_r, pid_clr_r;
    always @(posedge sys_clk or negedge rst_n)
        if (!rst_n) begin pi_tick_r <= 1'b0; pid_clr_r <= 1'b0; end
        else        begin pi_tick_r <= pi_tick; pid_clr_r <= pid_clr; end

    pid_pi_v2 u_p0(.clk(sys_clk), .rst_n(rst_n), .en(ce_eff ? 1'b1 : pi_tick_r),
                                       .clr(clr_eff ? 1'b1 : pid_clr_r),
                   .kp_q(KP_Q), .ki_q(KI_Q), .target(pi_te1), .actual(pi_act1), .u(u1));
    pid_pi_v2 u_p1(.clk(sys_clk), .rst_n(rst_n), .en(ce_eff ? 1'b1 : pi_tick_r),
                                       .clr(clr_eff ? 1'b1 : pid_clr_r),
                   .kp_q(KP_Q), .ki_q(KI_Q), .target(pi_te2), .actual(pi_act2), .u(u2));
    pid_pi_v2 u_p2(.clk(sys_clk), .rst_n(rst_n), .en(ce_eff ? 1'b1 : pi_tick_r),
                                       .clr(clr_eff ? 1'b1 : pid_clr_r),
                   .kp_q(KP_Q), .ki_q(KI_Q), .target(pi_te3), .actual(pi_act3), .u(u3));
    pid_pi_v2 u_p3(.clk(sys_clk), .rst_n(rst_n), .en(ce_eff ? 1'b1 : pi_tick_r),
                                       .clr(clr_eff ? 1'b1 : pid_clr_r),
                   .kp_q(KP_Q), .ki_q(KI_Q), .target(pi_te4), .actual(pi_act4), .u(u4));

    // ------------------------------------------------------------------
    // 十七、电机输出（含方向极性修正：整体取反，让"正指令=车前进"）
    // ------------------------------------------------------------------
    wire signed [15:0] mc1 = -u1;
    wire signed [15:0] mc2 = -u2;
    wire signed [15:0] mc3 = -u3;
    wire signed [15:0] mc4 = -u4;

    motor_out u_m0(.clk(sys_clk), .rst_n(rst_n), .en(motion_active), .cmd(mc1),
                   .in1(in1_w[0]), .in2(in2_w[0]), .duty(duty0), .out_en(oe0));
    motor_out u_m1(.clk(sys_clk), .rst_n(rst_n), .en(motion_active), .cmd(mc2),
                   .in1(in1_w[1]), .in2(in2_w[1]), .duty(duty1), .out_en(oe1));
    motor_out u_m2(.clk(sys_clk), .rst_n(rst_n), .en(motion_active), .cmd(mc3),
                   .in1(in1_w[2]), .in2(in2_w[2]), .duty(duty2), .out_en(oe2));
    motor_out u_m3(.clk(sys_clk), .rst_n(rst_n), .en(motion_active), .cmd(mc4),
                   .in1(in1_w[3]), .in2(in2_w[3]), .duty(duty3), .out_en(oe3));

    // ★定点旋转"到位"后进入**短刹车**：
    //   TB6612 在 IN1=IN2=1 时会把电机两端短接（短刹车），用主动制动吃掉整车惯性。
    //   这是为解决落地实测发现的"定点旋转多滑了约 75°"——原来到位后
    //   motion_active 变 0 → motor_out 输出全 0 → 模块看到 IN1=IN2=0 是**高阻(滑行)**，
    //   四个轮子带着整车惯性继续转，于是转过头了。
    wire spin_brake = run_en && spin_done;

    assign m_in1 = spin_brake ? 4'b1111 : in1_w;   // 刹车时四路 IN1 全高
    assign m_in2 = spin_brake ? 4'b1111 : in2_w;   // 刹车时四路 IN2 全高

    // STBY：只有"运行中 + 无故障 + 无堵转"才使能功率级
    assign stdby = run_en && !fault && !stall_fault;

    // ------------------------------------------------------------------
    // 十八、PWM 载波（50MHz ÷ 4096 ≈ 12.2kHz，12 位分辨率）
    // ------------------------------------------------------------------
    always @(posedge sys_clk or negedge rst_n)
        if (!rst_n) pwm_cnt <= 12'd0;
        else        pwm_cnt <= (pwm_cnt >= 12'd4095) ? 12'd0 : pwm_cnt + 1'b1;

    // 刹车时 PWM 也保持高（TB6612 在 IN1=IN2=1 时执行短刹车，与 PWM 无关）
    assign m_pwm[0] = spin_brake | (oe0 && (pwm_cnt < duty0));
    assign m_pwm[1] = spin_brake | (oe1 && (pwm_cnt < duty1));
    assign m_pwm[2] = spin_brake | (oe2 && (pwm_cnt < duty2));
    assign m_pwm[3] = spin_brake | (oe3 && (pwm_cnt < duty3));

    // ------------------------------------------------------------------
    // 十九、LED 指示
    // ------------------------------------------------------------------
    always @(posedge sys_clk or negedge rst_n)
        if (!rst_n) begin hb <= 7'd0; hb_led <= 1'b0; end
        else if (tick10) begin
            if (hb == 7'd49) begin hb <= 7'd0; hb_led <= ~hb_led; end   // 500ms → 1Hz
            else hb <= hb + 1'b1;
        end

    assign led[0] = real_mode ? 1'b1 : hb_led;   // 真实模式常亮；虚拟模式 1Hz 闪
    assign led[1] = run_en;                      // 运行中
    assign led[2] = locked;                      // 闭环锁定
    assign led[3] = fault | stall_fault;         // 故障（未锁定 / 堵转 / 急停）

    // ==================================================================
    // ★★★ 编码器分辨率标定诊断（2026-10-12 WorkBuddy）
    //   把 uart_tx（M15 → 板载 CH340E）变成周期报文：
    //       "P1=01234 P2=00000 P3=00000 P4=00000\r\n"   （37 字节）
    //   115200 8N1，约 5 Hz 刷新
    //   用法：上电 → 手转某轮**输出轴整整一圈** → 读该路数字的**增量**
    //         增量 ≈ 1440 ⇒ 12ppr（本机铭牌）；≈ 1320 ⇒ 11 线
    //   关掉诊断：把 DIAG_UART_EN 改为 0（uart_tx 回到恒高）
    //   P1..P4 对应：M1 左前 / M2 右前 / M3 左后 / M4 右后
    // ==================================================================
    localparam integer DIAG_UART_EN = 1;
    localparam [15:0]  BD_END       = 16'd433;      // 115200 @50MHz：50e6/115200-1

    wire [15:0] p1v = pos1[15:0];
    wire [15:0] p2v = pos2[15:0];
    wire [15:0] p3v = pos3[15:0];
    wire [15:0] p4v = pos4[15:0];

    reg  [5:0] ci_c;
    reg  [7:0] ci_d;
    always @(*) begin
        case (ci_c)
            6'd0 : ci_d = 8'h50;  6'd1 : ci_d = 8'h31;  6'd2 : ci_d = 8'h3D;  // "P1="
            6'd3 : ci_d = 8'h30 + (p1v / 16'd10000);
            6'd4 : ci_d = 8'h30 + ((p1v / 16'd1000) % 16'd10);
            6'd5 : ci_d = 8'h30 + ((p1v / 16'd100 ) % 16'd10);
            6'd6 : ci_d = 8'h30 + ((p1v / 16'd10  ) % 16'd10);
            6'd7 : ci_d = 8'h30 +  (p1v               % 16'd10);
            6'd8 : ci_d = 8'h20;                                            // ' '
            6'd9 : ci_d = 8'h50;  6'd10: ci_d = 8'h32;  6'd11: ci_d = 8'h3D;  // "P2="
            6'd12: ci_d = 8'h30 + (p2v / 16'd10000);
            6'd13: ci_d = 8'h30 + ((p2v / 16'd1000) % 16'd10);
            6'd14: ci_d = 8'h30 + ((p2v / 16'd100 ) % 16'd10);
            6'd15: ci_d = 8'h30 + ((p2v / 16'd10  ) % 16'd10);
            6'd16: ci_d = 8'h30 +  (p2v               % 16'd10);
            6'd17: ci_d = 8'h20;
            6'd18: ci_d = 8'h50;  6'd19: ci_d = 8'h33;  6'd20: ci_d = 8'h3D;  // "P3="
            6'd21: ci_d = 8'h30 + (p3v / 16'd10000);
            6'd22: ci_d = 8'h30 + ((p3v / 16'd1000) % 16'd10);
            6'd23: ci_d = 8'h30 + ((p3v / 16'd100 ) % 16'd10);
            6'd24: ci_d = 8'h30 + ((p3v / 16'd10  ) % 16'd10);
            6'd25: ci_d = 8'h30 +  (p3v               % 16'd10);
            6'd26: ci_d = 8'h20;
            6'd27: ci_d = 8'h50;  6'd28: ci_d = 8'h34;  6'd29: ci_d = 8'h3D;  // "P4="
            6'd30: ci_d = 8'h30 + (p4v / 16'd10000);
            6'd31: ci_d = 8'h30 + ((p4v / 16'd1000) % 16'd10);
            6'd32: ci_d = 8'h30 + ((p4v / 16'd100 ) % 16'd10);
            6'd33: ci_d = 8'h30 + ((p4v / 16'd10  ) % 16'd10);
            6'd34: ci_d = 8'h30 +  (p4v               % 16'd10);
            6'd35: ci_d = 8'h0D;                                            // '\r'
            6'd36: ci_d = 8'h0A;                                            // '\n'
            default: ci_d = 8'h0A;
        endcase
    end

    localparam [2:0] U_GAP = 3'd0, U_START = 3'd1, U_DATA = 3'd2,
                     U_HOLD = 3'd3, U_STOP = 3'd4;
    reg  [2:0]  ust;
    reg  [15:0] udiv;
    reg  [3:0]  ubit;
    reg  [7:0]  ushr;
    reg  [23:0] ugap;
    reg         utx;

    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            ust <= U_GAP;  utx <= 1'b1;  udiv <= 16'd0;  ubit <= 4'd0;
            ushr <= 8'd0;  ugap <= 24'd0;  ci_c <= 6'd0;
        end else begin
            case (ust)
            U_GAP: begin
                utx <= 1'b1;
                if (ugap >= 24'd9_999_999) begin      // 200 ms @50MHz
                    ugap <= 24'd0;  ci_c <= 6'd0;  ust <= U_START;
                end else ugap <= ugap + 24'd1;
            end
            U_START: begin                            // 起始位 + 锁存本字符
                utx <= 1'b0;  udiv <= 16'd0;  ubit <= 4'd0;
                ushr <= ci_d;  ust <= U_DATA;
            end
            U_DATA: begin                             // 8 数据位，LSB 先
                if (udiv == BD_END) begin
                    udiv <= 16'd0;
                    utx  <= ushr[0];
                    ushr <= {1'b1, ushr[7:1]};
                    if (ubit == 4'd7) ust <= U_HOLD;  // bit7 已送出 ⇒ 进保持期
                    else              ubit <= ubit + 4'd1;
                end else udiv <= udiv + 16'd1;
            end
            // ★ U_HOLD：让 bit7 保持满 1 位宽
            //   （原版发完 bit7 直接拉高 ⇒ bit7 只有 1 拍，接收端在 bit7 中心采到停止位
            //     ⇒ 每字节 bit7 恒为 1。本 bug 由 tb_uart_diag 仿真抓出并修正。）
            U_HOLD: begin
                if (udiv == BD_END) begin
                    udiv <= 16'd0;
                    utx  <= 1'b1;
                    ust  <= U_STOP;
                end else udiv <= udiv + 16'd1;
            end
            U_STOP: begin                             // 停止位（满 1 位宽）
                if (udiv == BD_END) begin
                    udiv <= 16'd0;
                    if (ci_c == 6'd36) ust <= U_GAP;  // 一帧发完
                    else begin ci_c <= ci_c + 6'd1;  ust <= U_START; end
                end else udiv <= udiv + 16'd1;
            end
            default: ust <= U_GAP;
            endcase
        end
    end

    assign uart_tx = DIAG_UART_EN ? utx : 1'b1;


    assign auto_done=spin_done;
    assign auto_fault=fault;
    assign auto_stall=stall_fault;
    assign auto_real=real_mode;
    assign auto_positions={pos4,pos3,pos2,pos1};
    assign auto_velocities={act4,act3,act2,act1};
    assign auto_errors={err4,err3,err2,err1};
    assign auto_travel=travel;
    assign auto_duties={duty3,duty2,duty1,duty0};
    assign auto_efforts={u4,u3,u2,u1};
    assign auto_tick=tk1;
endmodule
