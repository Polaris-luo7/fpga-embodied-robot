//=============================================================================
// mech_ik.v -- 麦克纳姆轮逆运动学（vx, vy, omega -> 四轮目标转速）
//-----------------------------------------------------------------------------
// 【这一层在系统里的位置】
//   上层（导航状态机）给"想怎么走"，本模块把它翻译成"四个轮子各转多快"，
//   输出**直接就是 speed_loop 的 target（计数/秒）**。
//
// 【★ 2026-10-14 改为流水实现】原因：
//   原组合版有两个 `/1000` 常量除法 + 两次乘法串成一条链，实测综合后
//   **组合延迟 124 ns（周期仅 20 ns）**，在 exp41 里单独贡献了
//   CLKOUT1 域 483 个违例端点、TNS −34387。
//   本模块是 10 ms 级的"设定值"路径，多几拍延迟毫无代价 ⇒ 改为 4 级流水，
//   并把两处常量除法折成**预计算的 Q12 常量乘法**（无除法器）。
//
// 【流水级】级1 输入寄存 + omega 项 / 级2 三路合成 / 级3 mm/s→计数/s / 级4 限幅·翻号
//   总延迟 4 个 clk（80 ns），相对于 10 ms 的设定值更新周期可忽略。
//
// 【坐标与符号约定】
//   车身坐标： x = 车头正前方为正； y = 车身左侧为正； omega = 俯视逆时针为正
//   四轮：     FL 左前 / FR 右前 / RL 左后 / RR 右后
//      v_fl = vx - vy - k*omega
//      v_fr = vx + vy + k*omega
//      v_rl = vx + vy - k*omega
//      v_rr = vx - vy + k*omega
//   k = (轴距 L + 轮距 W) / 2  —— 原地旋转时每个轮子的等效力臂
//=============================================================================
`timescale 1ns / 1ps

module mech_ik #(
    // ---- 机械参数（2026-10-13 已实测）----
    parameter integer COUNTS_PER_REV = 1440,     // 编码器每轮计数
    parameter integer WHEEL_DIA_UM   = 100_000,  // 轮径 0.001mm（100000 = 100.000 mm）
    parameter integer LL_WW_MM       = 413,      // 轴距 L + 轮距 W（mm），实测 413.5
    // ---- 输出限幅（计数/秒）----
    parameter integer V_MAX_CPS      = 30_000,
    // ---- 实物朝向修正（1 = 该轮翻号）----
    parameter integer FLIP_FL = 0,
    parameter integer FLIP_FR = 0,
    parameter integer FLIP_RL = 0,
    parameter integer FLIP_RR = 0
)(
    input  wire               clk,
    input  wire               rst_n,
    input  wire signed [15:0] vx,      // mm/s，前正
    input  wire signed [15:0] vy,      // mm/s，左正
    input  wire signed [15:0] omega,   // mrad/s，逆时针正

    output reg  signed [15:0] v_fl,    // 计数/秒（直接喂 speed_loop.target）
    output reg  signed [15:0] v_fr,
    output reg  signed [15:0] v_rl,
    output reg  signed [15:0] v_rr,
    output reg                sat      // 任一输出被限幅（延迟 4 拍）
);

    // ---------------------------------------------------------------------
    // 编译期常量（全部 Q12 定点乘法，无除法器）
    //   K_CPS_Q12    = COUNTS_PER_REV / 轮周长(mm)         * 4096    （mm/s -> 计数/s）
    //   K_OMCPS_Q12  = ((L+W)/2)/1000 * COUNTS_PER_REV/轮周长(mm) * 4096
    //                = omega[mrad/s] 直接转成"计数/s"的系数
    //   ★ omega 项**在计数域一次算出**（不再先转 mm/s 再转计数），
    //     避免二级量化：实测 P5 用例曾因 v_om 取整(99.999->99)导致 −1% 偏差。
    //   ★ 表达式按 (x*4096)/PERIM_UM 排序以避免中间量溢出 32 位。
    // ---------------------------------------------------------------------
    localparam integer PI_X1000    = 3142;                                       // pi*1000
    localparam integer PERIM_UM    = (PI_X1000 * WHEEL_DIA_UM) / 1000;           // 轮周长(um)
    localparam integer K_CPS_Q12   = (COUNTS_PER_REV * 4096 * 10) / (PERIM_UM / 100);
    localparam integer K_OMCPS_Q12 = ((LL_WW_MM / 2) * COUNTS_PER_REV * 4096) / PERIM_UM;

    // ---------------------------------------------------------------------
    // 级 1: 输入符号扩展并寄存
    // ---------------------------------------------------------------------
    wire signed [31:0] vx32  = {{16{vx[15]}},    vx};
    wire signed [31:0] vy32  = {{16{vy[15]}},    vy};
    wire signed [31:0] om32  = {{16{omega[15]}}, omega};

    reg signed [31:0] vx_r, vy_r, om_r;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin vx_r <= 32'sd0; vy_r <= 32'sd0; om_r <= 32'sd0; end
        else        begin vx_r <= vx32;   vy_r <= vy32;   om_r <= om32;   end

    // ---- Q12 定点乘 + **对称舍入** ----
    //   ★ 必须对称：`>>>` 是向 −∞ 取整，会让 +x 与 −x 的幅值差 1
    //     （原版用 `/1000` 是"向 0 截断"所以天然对称）。
    //     正解 = **统一先加半个 LSB 再下取整**：floor(p/4096 + 0.5) 即四舍五入，
    //     且 +x 与 −x 结果严格互为相反数（仅 exact .5 的平局点偏向 +∞）。
    //     ⚠ 不能用"按符号加减半个 LSB"：那等价于截断而非四舍五入（已踩）。
    function signed [31:0] q12r(input signed [31:0] v, input integer k);
        begin
            q12r = (v * k + 32'sd2048) >>> 12;
        end
    endfunction

    // ---------------------------------------------------------------------
    // 级 2: 前进/侧移的"和/差"两路 + omega 项（计数/s）
    //   ⚠ 这四个符号组合就是全部约定所在，改动前先看文件头的约定表
    // ---------------------------------------------------------------------
    reg signed [31:0] sp_r, sm_r, com_r;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin sp_r <= 32'sd0; sm_r <= 32'sd0; com_r <= 32'sd0; end
        else begin
            sp_r  <= vx_r + vy_r;                        // FR/RL 方向
            sm_r  <= vx_r - vy_r;                        // FL/RR 方向
            com_r <= q12r(om_r, K_OMCPS_Q12);             // 自转项（计数/s）
        end

    // ---------------------------------------------------------------------
    // 级 3: mm/s -> 计数/s
    //   量级检查：|s| <= 65534；65534 * 18772 = 1.23e9 < 2^31 ✓
    // ---------------------------------------------------------------------
    reg signed [31:0] cp_r, cm_r;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin cp_r <= 32'sd0; cm_r <= 32'sd0; end
        else begin
            cp_r <= q12r(sp_r, K_CPS_Q12);
            cm_r <= q12r(sm_r, K_CPS_Q12);
        end

    // ---------------------------------------------------------------------
    // 级 4: 四轮合成 + 限幅 + 翻号 + 输出
    //   v_fl = vx - vy - k*om ; v_fr = vx + vy + k*om
    //   v_rl = vx + vy - k*om ; v_rr = vx - vy + k*om
    // ---------------------------------------------------------------------
    wire signed [31:0] c_fl = cm_r - com_r;
    wire signed [31:0] c_fr = cp_r + com_r;
    wire signed [31:0] c_rl = cp_r - com_r;
    wire signed [31:0] c_rr = cm_r + com_r;

    function signed [15:0] LIM(input signed [31:0] v);
        begin
            if      (v >  V_MAX_CPS) LIM =  V_MAX_CPS;
            else if (v < -V_MAX_CPS) LIM = -V_MAX_CPS;
            else                     LIM = v[15:0];
        end
    endfunction

    wire signed [15:0] l_fl = LIM(c_fl);
    wire signed [15:0] l_fr = LIM(c_fr);
    wire signed [15:0] l_rl = LIM(c_rl);
    wire signed [15:0] l_rr = LIM(c_rr);

    wire s_any = (c_fl > V_MAX_CPS) || (c_fl < -V_MAX_CPS) ||
                 (c_fr > V_MAX_CPS) || (c_fr < -V_MAX_CPS) ||
                 (c_rl > V_MAX_CPS) || (c_rl < -V_MAX_CPS) ||
                 (c_rr > V_MAX_CPS) || (c_rr < -V_MAX_CPS);

    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin
            v_fl <= 16'sd0; v_fr <= 16'sd0; v_rl <= 16'sd0; v_rr <= 16'sd0; sat <= 1'b0;
        end else begin
            v_fl <= FLIP_FL ? -l_fl : l_fl;
            v_fr <= FLIP_FR ? -l_fr : l_fr;
            v_rl <= FLIP_RL ? -l_rl : l_rl;
            v_rr <= FLIP_RR ? -l_rr : l_rr;
            sat  <= s_any;
        end

endmodule
