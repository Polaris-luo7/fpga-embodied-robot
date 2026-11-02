// ============================================================================
//  【任务二 · 运动控制】—— 组长交付（2026-10-11/12），2026-10-12 由 WorkBuddy 整合进本工程
//
//  ★ 本文件**内部逻辑一字未改**，仅在本注释块下方原样保留。
//  ★ 引脚全部按组长**已在实物上验证过**的 `ACG720_task2_ir.cst`（33 脚）。
//  ★ 顶层例化点：`robot/src/ov5640_ddr3_tft.v` 末尾的 `u_task2`。
//  ★ 引脚逐脚对照与冲突说明：`mimo_material/任务二_合并_差异对照表.md`
//  ★ 整合日期：2026-10-12
// ============================================================================

// ============================================================================
//  selftest_top.v  ——  ACG720 板级自测顶层（TB6612 驱动模块尚未到货期间使用）
//  ---------------------------------------------------------------------------
//  端口与 ACG720_task2.cst 完全一致，直接复用该约束文件，无需改管脚。
//  功率级：WHEELTEC D24A 四路 TB6612 模块（PWMx / xIN1 / xIN2 / ExA / ExB）
//  PWM：12 位分辨率，载波 50MHz/4096 ≈ 12.2kHz（手册推荐 10kHz）
//
//  这个顶层做两件事：
//   (A) 【虚拟电机闭环自测】FPGA 内部生成"虚拟电机模型"：
//         PID → PWM → 虚拟电机 → 假编码器 A/B → 正交解码 → 测速 → PID
//       整条链路真实闭合。收敛锁定点亮 LED2，超时失败点亮 LED3。
//   (B) 【真实编码器验证】按 KEY3 切换到真实编码器输入，手转轮子即可
//       验证 quad_dec / 测速 / 标定每圈脉冲数 U（编码器只需 3.3V + GND + A/B）。
//
//  按键功能
//   KEY0 (S0) : 动作选择 bit0  ┐ 按键按下为低电平，程序里按"按下=1"处理：
//   KEY1 (S1) : 动作选择 bit1  ┘ 都松开=前进  按S0=后退  按S1=转向  两个都按=定点旋转
//   KEY2 (S2) : 启动 / 停止（按一下切换）
//   KEY3 (S3) : 编码器来源切换（按一下切换）
//               默认 = 虚拟模型自测；切到真实编码器模式时 LED0/LED3 交替闪
//   KEY4 (S4/B21) : 复位
//
//  LED 含义（高电平点亮）
//   LED0 : 心跳（500ms 翻转）        LED2 : 闭环锁定 / 检测到编码器脉冲
//   LED1 : 运行中                    LED3 : 故障（启动后 3s 未锁定）
//   注：切到"真实编码器模式"后，LED3 改为与 LED0 交替闪，用来提示当前处在哪种模式
//
//  GAO 在线逻辑分析仪建议观察的信号（教材第4章）
//   te1 / act1        第 1 路目标速度与实测速度（脉冲/10ms）
//   u1                第 1 路 PID 输出（PWM 指令，满量程 ±1000）
//   pos1 / pos2       编码器累计脉冲      fa[0] / fb[0]   编码器 A/B 相
//   pwm_cnt / duty0 / m_pwm[0]           载波计数与 PWM 输出
//   travel            定点旋转已走脉冲数
//   locked / fault / spin_done           自检状态
// ============================================================================

// ---------------------------------------------------------------------------
// 按键消抖（20ms 稳定判据，输出高 = 未按下）
// ---------------------------------------------------------------------------
module key_deb #(parameter CNT = 1_000_000)(   // 20ms @50MHz
    input  wire clk, rst_n, key_in,
    output reg  key_out
);
    reg [1:0]  sync;
    reg [19:0] cnt;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin
            sync <= 2'b11; cnt <= 20'd0; key_out <= 1'b1;
        end else begin
            sync <= {sync[0], key_in};
            if (sync[1] == key_out)  cnt <= 20'd0;
            else if (cnt == CNT - 1) begin cnt <= 20'd0; key_out <= sync[1]; end
            else                     cnt <= cnt + 1'b1;
        end
endmodule

// ---------------------------------------------------------------------------
// 正交解码（4 倍频，两级同步 + 非法跳变计数）
// ---------------------------------------------------------------------------
module quad_dec(
    input  wire clk, rst_n, a, b, clr,
    output reg  signed [31:0] pos,
    output reg  [7:0] err_cnt
);
    reg [1:0] a_s, b_s, ab_d;
    reg up, dn;
    wire [1:0] ab = {a_s[1], b_s[1]};

    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin a_s <= 2'b00; b_s <= 2'b00; end
        else        begin a_s <= {a_s[0], a}; b_s <= {b_s[0], b}; end

    always @(posedge clk or negedge rst_n)
        if (!rst_n) ab_d <= 2'b00;
        else        ab_d <= ab;

    always @(*) begin                       // {A_prev,B_prev,A_now,B_now}
        up = 1'b0; dn = 1'b0;
        case ({ab_d, ab})
            4'b0001, 4'b0111, 4'b1110, 4'b1000: up = 1'b1;
            4'b0010, 4'b1011, 4'b1101, 4'b0100: dn = 1'b1;
            default: ;
        endcase
    end

    always @(posedge clk or negedge rst_n)
        if (!rst_n)   begin pos <= 32'sd0; err_cnt <= 8'd0; end
        else if (clr) begin pos <= 32'sd0; err_cnt <= 8'd0; end
        else if (up)  pos <= pos + 32'sd1;
        else if (dn)  pos <= pos - 32'sd1;
        else if (ab_d != ab) err_cnt <= err_cnt + 8'd1;
endmodule

// ---------------------------------------------------------------------------
// 测速：每 10ms 取一次脉冲增量 = 速度（脉冲/10ms，带符号）
// ---------------------------------------------------------------------------
module speed_meter #(parameter WIN = 500_000)(     // 10ms @50MHz
    input  wire clk, rst_n,
    input  wire signed [31:0] pos,
    output reg  signed [15:0] spd,
    output reg  tick
);
    reg [31:0] cnt;
    reg signed [31:0] pos_d;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin cnt <= 32'd0; pos_d <= 32'sd0; spd <= 16'sd0; tick <= 1'b0; end
        else begin
            tick <= 1'b0;
            if (cnt >= WIN - 1) begin
                cnt   <= 32'd0;
                spd   <= (pos - pos_d);
                pos_d <= pos;
                tick  <= 1'b1;
            end else cnt <= cnt + 1'b1;
        end
endmodule

// ---------------------------------------------------------------------------
// PI 速度环（位置式，积分限幅 + 输出限幅 + 抗积分饱和）
// ---------------------------------------------------------------------------
module pid_pi #(
    parameter signed [15:0] UMAX = 16'sd4000,
    parameter signed [15:0] UFF  = 16'sd1500        // 死区前馈：目标非 0 时叠加的基础输出
)(
    input  wire clk, rst_n, en,
    input  wire signed [15:0] kp_q, ki_q,       // Q8 增益：Kp = kp_q/256
    input  wire signed [15:0] target, actual,
    output reg  signed [15:0] u
);
    wire signed [15:0] e  = target - actual;
    wire signed [31:0] pw = ($signed(e) * $signed(kp_q)) >>> 8;
    wire signed [31:0] iw = ($signed(e) * $signed(ki_q)) >>> 8;

    // 死区前馈：目标不为 0 时先给一个基础输出，让电机立刻越过"启动死区"，
    // 环路只需要在这个工作点附近微调 —— 这是真电机比虚拟模型难调的关键点，
    // 加了它以后 PID 不用再"慢慢积分爬过死区"，收敛快很多、也不容易饱和。
    wire signed [15:0] ff = (target == 16'sd0) ? 16'sd0 :
                            ((target > 16'sd0) ? UFF : -UFF);

    reg  signed [31:0] acc;
    wire signed [31:0] acc_nxt = acc + iw;
    wire signed [31:0] raw     = pw + acc_nxt + ff;
    wire signed [31:0] acc_cl  = (acc_nxt >  32'sd40000) ?  32'sd40000 :
                                 (acc_nxt < -32'sd40000) ? -32'sd40000 : acc_nxt;
    wire signed [31:0] u_cl    = (raw >  UMAX) ?  UMAX :
                                 (raw < -UMAX) ? -UMAX : raw;
    wire sat = (raw != u_cl) && ((e > 0) == (raw > 0));

    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin acc <= 32'sd0; u <= 16'sd0; end
        else if (en) begin
            acc <= sat ? acc : acc_cl;
            u   <= u_cl;
        end
endmodule

// ---------------------------------------------------------------------------
// 虚拟电机模型：一阶惯性 + 死区，转速正比于 PWM
//   模型速度 |spd| = (|pwm| - DEAD) / 4   （单位：脉冲/10ms）
//   输出与真实霍尔编码器一致的 A/B 正交信号
// ---------------------------------------------------------------------------
module fake_motor #(
    parameter signed [12:0] DEAD = 13'sd800,
    parameter MODEL_SHIFT = 2
)(
    input  wire clk, rst_n,
    input  wire signed [15:0] pwm,
    output reg  a, b,
    output reg  signed [15:0] spd_m         // 模型内部速度，仅调试观察
);
    wire signed [12:0] p13 = pwm[12:0];
    wire signed [12:0] pe  = (p13 >  DEAD) ? (p13 - DEAD) :
                             (p13 < -DEAD) ? (p13 + DEAD) : 13'sd0;
    wire signed [15:0] pe16    = pe;                      // 符号扩展
    wire signed [15:0] spd_tgt = pe16 >>> MODEL_SHIFT;

    reg [15:0] ms_cnt;
    reg [31:0] acc;
    reg [1:0]  phase;
    wire [15:0] spd_abs = spd_m[15] ? (~spd_m + 16'd1) : spd_m;
    wire        dir     = spd_m[15];                      // 1 = 反转
    wire [31:0] acc_nxt = acc + spd_abs;

    // 一阶惯性：每 1ms 向目标逼近 1/8（时间常数约 8ms，接近真实电机）
    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin ms_cnt <= 16'd0; spd_m <= 16'sd0; end
        else if (ms_cnt == 16'd49_999) begin
            ms_cnt <= 16'd0;
            spd_m  <= spd_m + ((spd_tgt - spd_m) >>> 3);
        end else ms_cnt <= ms_cnt + 1'b1;

    // 相位累加：每 10ms（500000 clk）推进 |spd_m| 个相位步
    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin
            acc <= 32'd0; phase <= 2'd0;
        end else if (acc_nxt >= 32'd500_000) begin
            acc   <= acc_nxt - 32'd500_000;
            phase <= dir ? phase - 2'd1 : phase + 2'd1;
        end else begin
            acc <= acc_nxt;
        end

    // 格雷码：00→01→11→10 循环，正是 quad_dec 识别的正转序列
    always @(*) begin
        a = phase[1];
        b = phase[1] ^ phase[0];
    end
endmodule

// ---------------------------------------------------------------------------
// 电机方向 + PWM 输出（换向死区保护、上电安全态）
// ---------------------------------------------------------------------------
module motor_out(
    input  wire clk, rst_n, en,
    input  wire signed [15:0] cmd,
    output reg  in1, in2,
    output reg  [11:0] duty,
    output reg  out_en
);
    reg [3:0] dv;
    reg       last;
    wire        dir = cmd[15];
    wire [15:0] mag = dir ? (~cmd + 16'd1) : cmd;

    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin
            in1 <= 1'b0; in2 <= 1'b0; duty <= 12'd0; out_en <= 1'b0;
            dv <= 4'd0;  last <= 1'b0;
        end else if (!en || mag < 16'd80) begin
            in1 <= 1'b0; in2 <= 1'b0; duty <= 12'd0; out_en <= 1'b0; dv <= 4'd0;
        end else if (dir != last) begin               // 换向：先停一段再反向
            in1 <= 1'b0; in2 <= 1'b0; out_en <= 1'b0;
            dv  <= 4'd15; last <= dir;
        end else if (dv != 4'd0) begin
            dv <= dv - 1'b1; out_en <= 1'b0;
        end else begin
            in1 <= ~dir; in2 <= dir; out_en <= 1'b1;
            duty <= (mag > 16'd4095) ? 12'd4095 : mag[11:0];
        end
endmodule

// ===========================================================================
//                              自 测 顶 层
// ===========================================================================
module selftest_top(
    input  wire        sys_clk,        // Y18  50MHz
    input  wire        rst_n,          // B21  按键 S4
    input  wire [3:0]  key,            // S0~S3
    input  wire [3:0]  enc_a,          // 真实编码器 A 相
    input  wire [3:0]  enc_b,          // 真实编码器 B 相
    output wire [3:0]  m_in1,          // TB6612 IN1
    output wire [3:0]  m_in2,          // TB6612 IN2
    output wire [3:0]  m_pwm,          // TB6612 PWM
    output wire        stdby,          // TB6612 STBY
    output wire [3:0]  led,
    output wire        uart_tx
);
    // ---------------------- 参数 ----------------------
    // 前进/后退目标速度（脉冲/10ms）
    // 实测：GM37-520（铭牌丝印）+ 1:30 减速比，在 7.4V 下空载满速约 45 左右脉冲/10ms，
    // 所以目标取 20（约满速的 45%）。想跑快一点就把它调大（但别超过 35）。
    localparam signed [15:0] RUN_SPD  = 16'sd20;
    localparam signed [15:0] SPIN_SPD = 16'sd12;    // 旋转目标速度（脉冲/10ms）
    // 定点旋转到位所需的脉冲数
    // 估算依据：原地转 θ 角 → 每个轮子行程 = (a+b)·θ，脉冲数 = U·(a+b)·θ/(πD)
    // 取 U=1440（实物铭牌 GM37-520 丝印「12ppr/AB Phase」× 4 倍频 × 1:30 减速）、(a+b)≈0.155m、轮径 D≈0.065m。
    //   ★ 按 U=1440 估算 90° ≈ 1700 脉冲；下文 1600 系早期按 U=1320 的旧估（仅占位，本模块为冻结保底件）。
    //     真值以 selftest_ir.v 的**实测标定**（11740 脉冲 ≈ 720°）为准。
    // ⚠️ 这是估算值，必须实测标定：命令转 90°，量实际角度，再按比例修正这个数。
    localparam        [31:0] SPIN_TGT = 32'd1600;
    localparam signed [15:0] KP_Q     = 16'sd100;   // Kp = 100/256 ≈ 0.39
    localparam signed [15:0] KI_Q     = 16'sd130;   // Ki = 130/256 ≈ 0.51（大会补偿快，但太大易抖）
    localparam signed [15:0] TOL      = 16'sd8;     // 锁定容差

    // ---------------------- 饱和函数 ----------------------
    function signed [15:0] sat16;
        input signed [31:0] v;
        begin
            if      (v >  32'sd600) sat16 = 16'sd600;
            else if (v < -32'sd600) sat16 = -16'sd600;
            else                    sat16 = v[15:0];
        end
    endfunction

    // ---------------------- 所有寄存器声明（先声明后使用） ----------------------
    reg [15:0] ms_cnt;
    reg        tick10;
    reg [3:0]  ms10;
    reg        k2_d;
    reg        k3_d;
    reg        real_mode;              // 编码器来源：0=虚拟模型(默认) 1=真实编码器
    reg [1:0]  act_lat;                // 启动时锁存的动作
    reg        run_en, fault, locked, spin_done;
    reg [15:0] lock_cnt, fault_cnt;
    reg signed [31:0] p1_0, p2_0, p3_0, p4_0;
    reg [6:0]  hb;
    reg        hb_led;
    reg [11:0] pwm_cnt;

    // ---------------------- 关键连线声明 ----------------------
    wire [3:0]  k;
    wire        start_pulse;
    wire [1:0]  act_sel;               // 动作选择（按下为 1）
    wire [1:0]  act_now;               // 实际生效的动作（运行中=锁存值）
    wire signed [15:0] u1, u2, u3, u4;              // PID 输出
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
    wire [31:0] a1, a2, a3, a4;
    wire [31:0] travel;
    wire        motion_active;
    wire signed [15:0] te1, te2, te3, te4;
    wire signed [15:0] e1, e2, e3, e4;
    wire        in_tol_all;
    wire [3:0]  in1_w, in2_w;
    wire [11:0] duty0, duty1, duty2, duty3;
    wire        oe0, oe1, oe2, oe3;
    wire        enc_alive;

    // ---------------------- 节拍：1ms / 10ms ----------------------
    always @(posedge sys_clk or negedge rst_n)
        if (!rst_n) begin ms_cnt <= 16'd0; tick10 <= 1'b0; ms10 <= 4'd0; end
        else begin
            tick10 <= 1'b0;
            if (ms_cnt == 16'd49_999) begin
                ms_cnt <= 16'd0;
                if (ms10 == 4'd9) begin ms10 <= 4'd0; tick10 <= 1'b1; end
                else ms10 <= ms10 + 1'b1;
            end else ms_cnt <= ms_cnt + 1'b1;
        end

    // ---------------------- 按键消抖与边沿 ----------------------
    key_deb u_k0(.clk(sys_clk), .rst_n(rst_n), .key_in(key[0]), .key_out(k[0]));
    key_deb u_k1(.clk(sys_clk), .rst_n(rst_n), .key_in(key[1]), .key_out(k[1]));
    key_deb u_k2(.clk(sys_clk), .rst_n(rst_n), .key_in(key[2]), .key_out(k[2]));
    key_deb u_k3(.clk(sys_clk), .rst_n(rst_n), .key_in(key[3]), .key_out(k[3]));

    assign start_pulse = k2_d & ~k[2];              // KEY2 下降沿：启动/停止切换
    // 按键按下为低电平，这里只做边沿检测；real_mode 的切换放在下方的
    // "唯一驱动块"里（同一个 reg 只能由一个 always 块赋值）
    always @(posedge sys_clk or negedge rst_n)
        if (!rst_n) begin k2_d <= 1'b1; k3_d <= 1'b1; end
        else        begin k2_d <= k[2]; k3_d <= k[3]; end

    // ---------------------- 虚拟电机模型 ×4（输入 PID 输出，输出 A/B） ----------------------
    fake_motor u_fm0(.clk(sys_clk), .rst_n(rst_n), .pwm(u1), .a(ma0), .b(mb0), .spd_m(spd_m0));
    fake_motor u_fm1(.clk(sys_clk), .rst_n(rst_n), .pwm(u2), .a(ma1), .b(mb1), .spd_m(spd_m1));
    fake_motor u_fm2(.clk(sys_clk), .rst_n(rst_n), .pwm(u3), .a(ma2), .b(mb2), .spd_m(spd_m2));
    fake_motor u_fm3(.clk(sys_clk), .rst_n(rst_n), .pwm(u4), .a(ma3), .b(mb3), .spd_m(spd_m3));

    // 真实编码器输入：通道 B（下标1）与通道 D（下标3）是左右镜像安装，
// 它们的 A/B 相位与左侧相反，所以这两路交换 a/b。
// ⚠️ 交换只能作用在【真实编码器】这一侧：虚拟模型本身四路极性一致，
//    如果连虚拟模型一起交换，虚拟模式下那两路会变成正反馈、永远锁不上。
    wire [3:0] enc_a_fix = {enc_b[3], enc_a[2], enc_b[1], enc_a[0]};
    wire [3:0] enc_b_fix = {enc_a[3], enc_b[2], enc_a[1], enc_b[0]};

    assign fa = real_mode ? enc_a_fix : {ma3, ma2, ma1, ma0};
    assign fb = real_mode ? enc_b_fix : {mb3, mb2, mb1, mb0};

    // ---------------------- 正交解码 + 测速 ×4 ----------------------
    // 编码器极性修正（2026-10-05 实测标定）：
    //   左侧两轮（通道 A、C）前进时计数递增 ✓；右侧两轮（通道 B、D）前进时计数
    //   递减 ✗ —— 因为左右电机是镜像安装的，A/B 相位关系相反。
    //   所以把通道 B（下标 1）和通道 D（下标 3）的 A/B 相交换，使"车轮前进 = 计数递增"。
    quad_dec u_q0(.clk(sys_clk), .rst_n(rst_n), .a(fa[0]), .b(fb[0]), .clr(1'b0), .pos(pos1), .err_cnt(err1));
    quad_dec u_q1(.clk(sys_clk), .rst_n(rst_n), .a(fa[1]), .b(fb[1]), .clr(1'b0), .pos(pos2), .err_cnt(err2));
    quad_dec u_q2(.clk(sys_clk), .rst_n(rst_n), .a(fa[2]), .b(fb[2]), .clr(1'b0), .pos(pos3), .err_cnt(err3));
    quad_dec u_q3(.clk(sys_clk), .rst_n(rst_n), .a(fa[3]), .b(fb[3]), .clr(1'b0), .pos(pos4), .err_cnt(err4));

    speed_meter u_s0(.clk(sys_clk), .rst_n(rst_n), .pos(pos1), .spd(act1), .tick(tk1));
    speed_meter u_s1(.clk(sys_clk), .rst_n(rst_n), .pos(pos2), .spd(act2), .tick(tk2));
    speed_meter u_s2(.clk(sys_clk), .rst_n(rst_n), .pos(pos3), .spd(act3), .tick(tk3));
    speed_meter u_s3(.clk(sys_clk), .rst_n(rst_n), .pos(pos4), .spd(act4), .tick(tk4));

    // ---------------------- 动作选择 + 麦轮逆运动学 ----------------------
    // 完整形式 t1=vx-vy-w, t2=vx+vy+w, t3=vx+vy-w, t4=vx-vy+w（此处 vy = 0）
    assign act_sel = {~k[1], ~k[0]};        // 按下为 1：00=前进 01=后退 10=转向 11=定点旋转
    // 启动那一刻把动作锁存下来，之后松手也不变（否则运行中要一直按着键才能保持动作）
    assign act_now = run_en ? act_lat : act_sel;
    assign vx_w = (act_now == 2'd0) ? RUN_SPD :
                  (act_now == 2'd1) ? -RUN_SPD : 16'sd0;
    assign w_w  = ((act_now == 2'd2) || (act_now == 2'd3)) ? SPIN_SPD : 16'sd0;

    assign tgt1 = sat16(vx_w - w_w);                // 左前
    assign tgt2 = sat16(vx_w + w_w);                // 右前
    assign tgt3 = sat16(vx_w - w_w);                // 左后
    assign tgt4 = sat16(vx_w + w_w);                // 右后

    // ---------------------- 定点旋转行程（4 路 |Δpos| 的平均） ----------------------
    assign d1 = pos1 - p1_0;
    assign d2 = pos2 - p2_0;
    assign d3 = pos3 - p3_0;
    assign d4 = pos4 - p4_0;
    assign a1 = d1[31] ? (~d1 + 32'sd1) : d1;
    assign a2 = d2[31] ? (~d2 + 32'sd1) : d2;
    assign a3 = d3[31] ? (~d3 + 32'sd1) : d3;
    assign a4 = d4[31] ? (~d4 + 32'sd1) : d4;
    assign travel = ((a1 + a2) + (a3 + a4)) >>> 2;

    // ---------------------- 启停 / 到位 / 自检状态（唯一驱动块） ----------------------
    always @(posedge sys_clk or negedge rst_n)
        if (!rst_n) begin
            run_en    <= 1'b0;  fault <= 1'b0; locked <= 1'b0;
            lock_cnt  <= 16'd0; fault_cnt <= 16'd0;
            spin_done <= 1'b0;
            real_mode <= 1'b0;                    // 上电默认：虚拟模型自测
            act_lat   <= 2'd0;                    // 默认动作：前进
            p1_0 <= 32'sd0; p2_0 <= 32'sd0; p3_0 <= 32'sd0; p4_0 <= 32'sd0;
        end else if (start_pulse) begin
            run_en    <= ~run_en;                 // 按一下切换启停
            fault     <= 1'b0;
            fault_cnt <= 16'd0;
            lock_cnt  <= 16'd0;
            locked    <= 1'b0;
            spin_done <= 1'b0;
            if (!run_en) act_lat <= act_sel;      // 本次是"启动"→ 锁存当前动作
            p1_0 <= pos1; p2_0 <= pos2; p3_0 <= pos3; p4_0 <= pos4;
        end else if (k3_d & ~k[3]) begin          // 按一下 S3：切换编码器来源
            real_mode <= ~real_mode;
            fault     <= 1'b0;
            fault_cnt <= 16'd0;
            lock_cnt  <= 16'd0;
            locked    <= 1'b0;
            spin_done <= 1'b0;
        end else if (!run_en) begin
            spin_done <= 1'b0;
            lock_cnt  <= 16'd0;
            fault_cnt <= 16'd0;
            locked    <= 1'b0;
        end else if ((act_now == 2'd3) && (travel >= SPIN_TGT)) begin
            spin_done <= 1'b1;                    // 定点旋转到位（两种模式都要判断！）
        end else if (in_tol_all) begin
            if (tick10) begin                              // 按 10ms 节拍计数
                if (lock_cnt < 16'd20) lock_cnt <= lock_cnt + 1'b1;   // 连续 200ms
                else                   locked   <= 1'b1;
            end
        end else if (tick10) begin
            lock_cnt <= 16'd0;
            if (!locked) begin
                if (fault_cnt < 16'd1000) fault_cnt <= fault_cnt + 1'b1;   // 10s 未锁定才报故障
                else                     fault     <= 1'b1;
            end
        end

    // 真实编码器模式下也要驱动电机（真正闭环测试要用），所以去掉 !real_mode
    assign motion_active = run_en && !spin_done;
    assign te1 = motion_active ? tgt1 : 16'sd0;
    assign te2 = motion_active ? tgt2 : 16'sd0;
    assign te3 = motion_active ? tgt3 : 16'sd0;
    assign te4 = motion_active ? tgt4 : 16'sd0;

    // ---------------------- 容差判断（4 路全部在容差内） ----------------------
    assign e1 = te1 - act1;
    assign e2 = te2 - act2;
    assign e3 = te3 - act3;
    assign e4 = te4 - act4;
    assign in_tol_all = (e1 <= TOL && e1 >= -TOL) &&
                        (e2 <= TOL && e2 >= -TOL) &&
                        (e3 <= TOL && e3 >= -TOL) &&
                        (e4 <= TOL && e4 >= -TOL);

    // ---------------------- 堵转保护 ----------------------
    // 判定条件：某一路"指令有速度（|目标|>5）"但"实测长期没速度（|实测|<2）"
    // 持续 2 秒 → 判堵转，立刻拉低 STBY 停机并点亮 LED3。
    // 目的：轮子卡死时电流会是正常的好几倍，必须尽早切断，避免烧驱动芯片。
    wire ch1_stall = ((te1 >  16'sd5) || (te1 < -16'sd5)) && (act1 < 16'sd2) && (act1 > -16'sd2);
    wire ch2_stall = ((te2 >  16'sd5) || (te2 < -16'sd5)) && (act2 < 16'sd2) && (act2 > -16'sd2);
    wire ch3_stall = ((te3 >  16'sd5) || (te3 < -16'sd5)) && (act3 < 16'sd2) && (act3 > -16'sd2);
    wire ch4_stall = ((te4 >  16'sd5) || (te4 < -16'sd5)) && (act4 < 16'sd2) && (act4 > -16'sd2);
    wire stall_now = run_en && (ch1_stall | ch2_stall | ch3_stall | ch4_stall);

    reg [15:0] stall_cnt;
    wire       stall_fault = (stall_cnt >= 16'd200);      // 2 秒（10ms × 200）

    always @(posedge sys_clk or negedge rst_n)
        if (!rst_n)              stall_cnt <= 16'd0;
        else if (!stall_now)     stall_cnt <= 16'd0;
        else if (tick10 && (stall_cnt < 16'd200)) stall_cnt <= stall_cnt + 1'b1;

    // ---------------------- 4 路 PI 速度环 ----------------------
    pid_pi u_p0(.clk(sys_clk), .rst_n(rst_n), .en(tk1), .kp_q(KP_Q), .ki_q(KI_Q),
                .target(te1), .actual(act1), .u(u1));
    pid_pi u_p1(.clk(sys_clk), .rst_n(rst_n), .en(tk1), .kp_q(KP_Q), .ki_q(KI_Q),
                .target(te2), .actual(act2), .u(u2));
    pid_pi u_p2(.clk(sys_clk), .rst_n(rst_n), .en(tk1), .kp_q(KP_Q), .ki_q(KI_Q),
                .target(te3), .actual(act3), .u(u3));
    pid_pi u_p3(.clk(sys_clk), .rst_n(rst_n), .en(tk1), .kp_q(KP_Q), .ki_q(KI_Q),
                .target(te4), .actual(act4), .u(u4));

    // ---------------------- 方向 + PWM 输出 ----------------------
    // 电机极性修正（2026-10-05 实测标定）：
    //   motor_check 单轮巡检发现 —— 命令 IN1=1/IN2=0 时，四个轮子都往【车尾】转。
    //   所以要整体取反，使"正的指令 = 车轮朝车头方向前进"，符合运动学约定。
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

    assign m_in1 = in1_w;
    assign m_in2 = in2_w;
    assign stdby = run_en && !fault && !stall_fault;   // 未启动/故障/堵转时不使能功率级

    // PWM 载波：50MHz / 4096 ≈ 12.2kHz，12 位分辨率
    // （D24A 手册给的范围是 0~100kHz，推荐 10kHz；选用 12.2kHz 兼顾平滑度与分辨率）
    always @(posedge sys_clk or negedge rst_n)
        if (!rst_n) pwm_cnt <= 12'd0;
        else        pwm_cnt <= (pwm_cnt >= 12'd4095) ? 12'd0 : pwm_cnt + 1'b1;

    assign m_pwm[0] = oe0 && (pwm_cnt < duty0);
    assign m_pwm[1] = oe1 && (pwm_cnt < duty1);
    assign m_pwm[2] = oe2 && (pwm_cnt < duty2);
    assign m_pwm[3] = oe3 && (pwm_cnt < duty3);

    // ---------------------- LED 指示 ----------------------
    // LED0：心跳。虚拟模型模式 1Hz 闪；真实编码器模式【常亮】
    always @(posedge sys_clk or negedge rst_n)
        if (!rst_n) begin hb <= 7'd0; hb_led <= 1'b0; end
        else if (tick10) begin
            if (hb == 7'd49) begin hb <= 7'd0; hb_led <= ~hb_led; end    // 500ms → 1Hz
            else hb <= hb + 1'b1;
        end

    // 真实编码器模式下 LED2 表示"检测到脉冲"（手转轮子就亮）
    assign enc_alive = (pos1 != 32'sd0) || (pos2 != 32'sd0) ||
                       (pos3 != 32'sd0) || (pos4 != 32'sd0);

    assign led[0] = real_mode ? 1'b1 : hb_led;       // 真实模式：常亮；虚拟模式：1Hz 闪
    assign led[1] = run_en;
    assign led[2] = locked;                          // 两种模式下都表示"闭环锁定"
    assign led[3] = fault | stall_fault;             // 故障（10s 未锁定 或 2s 堵转）

    assign uart_tx = 1'b1;                      // 本自测版不用串口

endmodule