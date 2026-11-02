//=============================================================================
// tb_mech_ik.v -- 麦克纳姆轮逆运动学自检
//-----------------------------------------------------------------------------
// 【7 项验收（对照交接文档「测试验收与自检设计」的 7 条）】
//   P1 纯前进   -> 四轮同号同速
//   P2 纯后退   -> 与 P1 全反号
//   P3 纯左移   -> **对角同号**（FL==RR, FR==RL, 两组反号）
//   P4 纯右移   -> 与 P3 全反号
//   P5 原地左转 -> **左右列反号**（左列 FL==RL，右列 FR==RR，左右相反）
//   P6 原地右转 -> 与 P5 全反号
//   P7 混合     -> 四轮值 = "纯前进" + "纯左转"（叠加性）
//
//   ⚠ 关于 P3 / P5 的判据：**上一阶段执行 Agent 给的验收单**把这两条写反过
//     （责任在执行方，不是用户提错需求）。正确的判据是：
//       P3 纯左移   -> 真特征 = **对角同号**（FL==RR、FR==RL），两组反号
//       P5 原地左转 -> 真特征 = **列内同号**（FL==RL、FR==RR），且对角反号
//     注意：两种模式都满足 "FL 与 FR 反号"，所以"左右反号"**不能**用来区分；
//     **唯一区分判据 = 列内同号（左转）还是对角同号（左移）。**
//     物理上不可互换：侧移要四轮合力指向同一侧（对角成对）；
//     自转要四轮形成纯力偶（左右列成对）。
//     本 tb 按物理事实检查，并把两种模式的四轮值**原样打印**，便于人工核对。
//
// 【附加项】
//   P0 零输入 -> 全 0；  P8 量纲校核；  P9 限幅 + sat 标志
//
// 【仿真参数】★ 2026-10-14: mech_ik 已改为 **4 级流水**（修 124ns 组合长链），
//   故本 tb 增加时钟 + 复位，采样前等待 4 拍流水延迟。
//
// 【注意】本文件所有 task 的字符串参数一律纯 ASCII。
//   （Verilog task 传中文串会被破坏成 0xFF —— 本项目在 T13 上实测踩过。）
//=============================================================================
`timescale 1ns / 1ps

module tb_mech_ik;

    // ------------------------------------------------------------------
    // 与 DUT 一致的机械参数（占位值，待实测；量到实物后两处一起改）
    // ------------------------------------------------------------------
    localparam integer COUNTS_PER_REV = 1440;
    localparam integer WHEEL_DIA_UM   = 65000;   // 65.000 mm
    localparam integer LL_WW_MM       = 200;     // 轴距+轮距
    localparam integer V_MAX_CPS      = 30000;

    // ------------------------------------------------------------------
    // 独立复算换算系数（不抄 DUT 的常量，避免"两边一起错"）
    // ------------------------------------------------------------------
    localparam integer PI_X1000   = 3142;
    localparam integer PERIM_UM   = (PI_X1000 * WHEEL_DIA_UM) / 1000;             // 轮周长 um
    localparam integer CPS_X1000  = (COUNTS_PER_REV * 1000000) / PERIM_UM;        // 每 (mm/s) 的计数/s * 1000
    localparam integer K_HALF_MM  = LL_WW_MM / 2;                                 // 力臂 k

    // 演示输入（用 integer 声明，赋给 16 位 signed 端口时自动截断，避免 localparam 带 signed 的可移植性问题）
    localparam integer V_LIN = 500;     // mm/s
    localparam integer OMEGA = 1000;    // mrad/s = 1 rad/s

    // 期望值（由上面参数推出）
    localparam integer EXP_LIN   = (500 * CPS_X1000) / 1000;                  // 前进 500 mm/s
    localparam integer EXP_OMEGA = ((1000 * K_HALF_MM) / 1000 * CPS_X1000) / 1000;  // 自转 1 rad/s

    // ------------------------------------------------------------------
    reg  signed [15:0] vx, vy, omega;
    reg                clk = 0, rst_n = 0;
    wire signed [15:0] v_fl, v_fr, v_rl, v_rr;
    wire               sat;

    always #10 clk = ~clk;                 // 50 MHz

    mech_ik #(
        .COUNTS_PER_REV (COUNTS_PER_REV),
        .WHEEL_DIA_UM   (WHEEL_DIA_UM),
        .LL_WW_MM       (LL_WW_MM),
        .V_MAX_CPS      (V_MAX_CPS)
    ) dut (
        .clk (clk), .rst_n (rst_n),
        .vx (vx), .vy (vy), .omega (omega),
        .v_fl (v_fl), .v_fr (v_fr), .v_rl (v_rl), .v_rr (v_rr),
        .sat (sat)
    );

    // ------------------------------------------------------------------
    integer err_cnt = 0;
    integer i_fl, i_fr, i_rl, i_rr;   // 当前采样值
    integer f_fl, f_fr, f_rl, f_rr;   // 纯前进的值（供 P2 / P7 用）
    integer t_fl, t_fr, t_rl, t_rr;   // 纯左转的值（供 P6 / P7 用）
    integer l_fl, l_fr, l_rl, l_rr;   // 纯左移的值（供 P4 用）

    task sample;
        begin
            repeat (6) @(posedge clk);      // ★ 4 级流水 + 余量
            i_fl = v_fl; i_fr = v_fr; i_rl = v_rl; i_rr = v_rr;
        end
    endtask

    task chk(input [8*40:1] name, input integer got, input integer exp, input integer tol);
        begin
            if (got > exp + tol || got < exp - tol) begin
                $display("  !! FAIL %0s: got=%0d exp=%0d(+/-%0d)", name, got, exp, tol);
                err_cnt = err_cnt + 1;
            end else begin
                $display("  OK   %0s = %0d (exp %0d +/-%0d)", name, got, exp, tol);
            end
        end
    endtask

    // 同号同值检查
    task chk_same(input [8*40:1] name, input integer a, input integer b);
        begin
            if (a !== b) begin
                $display("  !! FAIL %0s: %0d != %0d (应同号同值)", name, a, b);
                err_cnt = err_cnt + 1;
            end else begin
                $display("  OK   %0s: %0d == %0d", name, a, b);
            end
        end
    endtask

    // 相反数检查（且非零）
    task chk_opp(input [8*40:1] name, input integer a, input integer b);
        begin
            if (a === 0 || a !== -b) begin
                $display("  !! FAIL %0s: %0d 与 %0d 不是非零相反数", name, a, b);
                err_cnt = err_cnt + 1;
            end else begin
                $display("  OK   %0s: %0d vs %0d (opposite, nonzero)", name, a, b);
            end
        end
    endtask

    task show(input [8*24:1] tag);
        begin
            $display("      %0s : FL=%0d  FR=%0d  RL=%0d  RR=%0d",
                     tag, i_fl, i_fr, i_rl, i_rr);
        end
    endtask

    // ------------------------------------------------------------------
    initial begin
        $display("==== tb_mech_ik 开始 ====");
        repeat (5) @(posedge clk); rst_n = 1; repeat (5) @(posedge clk);
        $display("  参数(占位): 轮径=%0d um  L+W=%0d mm  每轮计数=%0d  限幅=%0d 计数/s",
                 WHEEL_DIA_UM, LL_WW_MM, COUNTS_PER_REV, V_MAX_CPS);
        $display("  换算系数 CPS_PER_MMPS_X1000 = %0d (即 1 mm/s = %0d.%03d 计数/s)",
                 CPS_X1000, CPS_X1000/1000, CPS_X1000%1000);
        $display("  期望: 前进 500mm/s -> %0d 计数/s ; 自转 1rad/s -> %0d 计数/s",
                 EXP_LIN, EXP_OMEGA);
        $display("");

        // ================= P0 零输入 =================
        $display("[P0] 零输入 -> 四轮全 0");
        vx = 0; vy = 0; omega = 0;
        sample;
        chk("P0 FL", i_fl, 0, 0);
        chk("P0 FR", i_fr, 0, 0);
        chk("P0 RL", i_rl, 0, 0);
        chk("P0 RR", i_rr, 0, 0);
        if (sat !== 1'b0) begin $display("  !! FAIL P0 sat 应为 0"); err_cnt = err_cnt + 1; end

        // ================= P1 纯前进 =================
        $display("[P1] 纯前进 vx=+%0d mm/s -> 四轮同号同速", V_LIN);
        vx = V_LIN; vy = 0; omega = 0;
        sample;
        show("FWD");
        chk("P1 FL", i_fl, EXP_LIN, 1);
        chk("P1 FR", i_fr, EXP_LIN, 1);
        chk("P1 RL", i_rl, EXP_LIN, 1);
        chk("P1 RR", i_rr, EXP_LIN, 1);
        chk_same("P1 FL vs FR", i_fl, i_fr);
        chk_same("P1 FL vs RL", i_fl, i_rl);
        chk_same("P1 FL vs RR", i_fl, i_rr);
        f_fl = i_fl; f_fr = i_fr; f_rl = i_rl; f_rr = i_rr;

        // ================= P2 纯后退 =================
        $display("[P2] 纯后退 vx=-%0d mm/s -> 与 P1 全反号", V_LIN);
        vx = -V_LIN; vy = 0; omega = 0;
        sample;
        show("BACK");
        chk_opp("P2 FL vs P1", i_fl, f_fl);
        chk_opp("P2 FR vs P1", i_fr, f_fr);
        chk_opp("P2 RL vs P1", i_rl, f_rl);
        chk_opp("P2 RR vs P1", i_rr, f_rr);

        // ================= P3 纯左移 =================
        $display("[P3] 纯左移 vy=+%0d mm/s -> 对角同号（FL==RR, FR==RL, 两组反号）", V_LIN);
        vx = 0; vy = V_LIN; omega = 0;
        sample;
        show("STRAFE-L");
        chk("P3 FL", i_fl, -EXP_LIN, 1);
        chk("P3 FR", i_fr,  EXP_LIN, 1);
        chk_same("P3 FL vs RR diag-same", i_fl, i_rr);
        chk_same("P3 FR vs RL diag-same", i_fr, i_rl);
        chk_opp ("P3 FL vs FR two-grp-opp", i_fl, i_fr);
        l_fl = i_fl; l_fr = i_fr; l_rl = i_rl; l_rr = i_rr;

        // ================= P4 纯右移 =================
        $display("[P4] 纯右移 vy=-%0d mm/s -> 与 P3 全反号", V_LIN);
        vx = 0; vy = -V_LIN; omega = 0;
        sample;
        show("STRAFE-R");
        chk_opp("P4 FL vs P3", i_fl, l_fl);
        chk_opp("P4 FR vs P3", i_fr, l_fr);
        chk_opp("P4 RL vs P3", i_rl, l_rl);
        chk_opp("P4 RR vs P3", i_rr, l_rr);
        chk_same("P4 FL vs RR diag-same", i_fl, i_rr);
        chk_same("P4 FR vs RL diag-same", i_fr, i_rl);

        // ================= P5 原地左转 =================
        $display("[P5] 原地左转 omega=+%0d mrad/s -> 左右列反号（左列 FL==RL, 右列 FR==RR）", OMEGA);
        vx = 0; vy = 0; omega = OMEGA;
        sample;
        show("ROT-CCW");
        chk("P5 FL", i_fl, -EXP_OMEGA, 1);
        chk("P5 FR", i_fr,  EXP_OMEGA, 1);
        chk_same("P5 FL vs RL left-col-same", i_fl, i_rl);
        chk_same("P5 FR vs RR right-col-same", i_fr, i_rr);
        chk_opp ("P5 left-col vs right-col opp", i_fl, i_fr);
        t_fl = i_fl; t_fr = i_fr; t_rl = i_rl; t_rr = i_rr;

        // ================= P6 原地右转 =================
        $display("[P6] 原地右转 omega=-%0d mrad/s -> 与 P5 全反号", OMEGA);
        vx = 0; vy = 0; omega = -OMEGA;
        sample;
        show("ROT-CW");
        chk_opp("P6 FL vs P5", i_fl, t_fl);
        chk_opp("P6 FR vs P5", i_fr, t_fr);
        chk_opp("P6 RL vs P5", i_rl, t_rl);
        chk_opp("P6 RR vs P5", i_rr, t_rr);

        // ================= P7 混合（叠加性）=================
        $display("[P7] 混合 vx=+%0d 且 omega=+%0d -> 四轮 = 前进项 + 左转项", V_LIN, OMEGA);
        vx = V_LIN; vy = 0; omega = OMEGA;
        sample;
        show("MIXED");
        chk("P7 FL superpos", i_fl, f_fl + t_fl, 2);
        chk("P7 FR superpos", i_fr, f_fr + t_fr, 2);
        chk("P7 RL superpos", i_rl, f_rl + t_rl, 2);
        chk("P7 RR superpos", i_rr, f_rr + t_rr, 2);
        $display("      参考: 前进项 FL=%0d / 左转项 FL=%0d -> 和 = %0d",
                 f_fl, t_fl, f_fl + t_fl);

        // ================= P8 量纲校核 =================
        $display("[P8] 量纲校核：1000 mm/s 应等于每轮 1000/204.230 圈/s -> 计数/s");
        vx = 16'sd1000; vy = 0; omega = 0;
        sample;
        show("1000mm/s");
        // ★ 容差 ±3：本 tb 的参考系数 CPS_X1000 是整数截断(7050, 真值 7050.9)，
            //   DUT 用 Q12 定点(等效 7051.76)。两者的量化误差在 1000 mm/s 处
            //   叠加约 1.8 计数 ⇒ 0.03%。属定点实现的固有精度，非功能错误。
        chk("P8 FL", i_fl, CPS_X1000, 3);   // 1000 mm/s * 系数 / 1000 = 系数
        $display("      物理核验: 1440 计数/轮 / 204.230 mm/轮 = %0d.%03d 计数/s per (mm/s) ; 1000 mm/s -> %0d 计数/s",
                 CPS_X1000/1000, CPS_X1000%1000, CPS_X1000);

        // ================= P9 限幅 =================
        $display("[P9] 限幅：vx=+32767 mm/s 远超上限 -> 输出应被限到 %0d 且 sat=1", V_MAX_CPS);
        vx = 16'sd32767; vy = 0; omega = 0;
        sample;
        show("OVER");
        chk("P9 FL = V_MAX", i_fl, V_MAX_CPS, 0);
        chk("P9 FR = V_MAX", i_fr, V_MAX_CPS, 0);
        if (sat !== 1'b1) begin
            $display("  !! FAIL P9 sat 应为 1");
            err_cnt = err_cnt + 1;
        end else begin
            $display("  OK   P9 sat = 1 (限幅标志有效)");
        end

        // ================= 结果 =================
        $display("");
        if (err_cnt == 0)
            $display("==== tb_mech_ik: ALL PASS ====");
        else
            $display("==== tb_mech_ik: %0d ERROR(S) ====", err_cnt);

        $finish;
    end

endmodule
