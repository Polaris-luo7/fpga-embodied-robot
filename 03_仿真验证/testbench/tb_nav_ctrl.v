//=============================================================================
// tb_nav_ctrl.v -- 导航状态机用例（P1）
//   tb 自带: ① 途径点队列行为模型 ② 可控位姿 ③ 应答捕获
//   期望值全部显式给出（速度方向/大小、状态迁移、DONE 序号）。
//=============================================================================
`timescale 1ns/1ps

module tb_nav_ctrl;
    localparam integer TICK_DIV = 50;      // 仿真加速(真机 500000)
    localparam integer SOFT_MS  = 1;       // 软启动 1ms -> SOFT_TICKS=1000

    reg clk=0, rst_n=0, cmd_vld=0;
    reg [7:0] cmd=0;
    reg signed [31:0] arg0=0, arg1=0;

    wire q_push, q_pop, q_clr, q_seek_en;
    wire [4:0] q_seek_n;
    wire signed [31:0] q_px, q_py;
    reg signed [31:0] q_head_x=0, q_head_y=0;
    reg [4:0] q_count=0;
    reg q_empty=1;

    reg signed [31:0] x_q16=0, y_q16=0, th_q16=0;
    reg  [15:0] dist_cm = 0;              // P4：超声波距离（cm）
    reg         dist_vld = 0;             // P4：距离有效
    integer n;

    wire signed [31:0] vx_cmd, vy_cmd, w_cmd;
    wire [2:0] st;
    wire [4:0] wp;
    wire resp_vld;
    wire [3:0] resp_type;
    wire [4:0] resp_arg;

    nav_ctrl #(.TICK_DIV(TICK_DIV), .SOFT_MS(SOFT_MS)) dut (
        .clk(clk), .rst_n(rst_n), .cmd_vld(cmd_vld), .cmd(cmd), .arg0(arg0), .arg1(arg1),
        .q_push(q_push), .q_pop(q_pop), .q_clr(q_clr), .q_seek_en(q_seek_en),
        .q_seek_n(q_seek_n), .q_px(q_px), .q_py(q_py),
        .q_head_x(q_head_x), .q_head_y(q_head_y), .q_count(q_count), .q_empty(q_empty),
        .x_q16(x_q16), .y_q16(y_q16), .th_q16(th_q16),
        .i_dist_cm(dist_cm), .i_dist_vld(dist_vld),
        .vx_cmd(vx_cmd), .vy_cmd(vy_cmd), .w_cmd(w_cmd),
        .st(st), .wp(wp), .resp_vld(resp_vld), .resp_type(resp_type), .resp_arg(resp_arg)
    );
    always #10 clk = ~clk;

    //------------------------------------------------------------------------
    // 途径点队列行为模型（16 点）
    //------------------------------------------------------------------------
    reg signed [31:0] qx [0:15], qy [0:15];
    reg [4:0] q_wr=0, q_rd=0;
    integer qi;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            q_wr <= 0; q_rd <= 0; q_count <= 0; q_empty <= 1;
            for (qi=0; qi<16; qi=qi+1) begin qx[qi] <= 0; qy[qi] <= 0; end
            q_head_x <= 0; q_head_y <= 0;
        end
        else begin
            if (q_clr) begin q_wr <= 0; q_rd <= 0; end
            if (q_push) begin qx[q_wr[3:0]] <= q_px; qy[q_wr[3:0]] <= q_py; q_wr <= q_wr + 1; end
            if (q_pop)  q_rd <= q_rd + 1;
            if (q_seek_en) q_rd <= q_seek_n;
            // 组合式头/计数（下一拍生效, 与真实 wp_queue 语义一致）
            q_count <= (q_wr + (q_push?1:0)) - (q_rd + (q_pop?1:0));
            q_head_x <= qx[q_rd[3:0]];
            q_head_y <= qy[q_rd[3:0]];
            q_empty <= ((q_wr + (q_push?1:0)) - (q_rd + (q_pop?1:0))) == 0;
        end
    end

    //------------------------------------------------------------------------
    // 驱动器/检查
    //------------------------------------------------------------------------
    integer fails=0, ncase=0;
    task do_cmd(input [7:0] c, input signed [31:0] a0, input signed [31:0] a1);
        begin
            cmd = c; arg0 = a0; arg1 = a1;
            @(posedge clk); cmd_vld = 1;
            @(posedge clk); cmd_vld = 0;
            repeat (3) @(posedge clk);
        end
    endtask

    task chk32(input [8*24-1:0] name, input signed [31:0] got, input real want);
        real g;
        begin
            ncase = ncase + 1;
            g = got / 65536.0;
            if (g > want - 1.0 && g < want + 1.0) $display("[PASS] %0s = %.1f", name, g);
            else begin fails = fails + 1;
                $display("[FAIL] %0s got=%.1f want=%.1f", name, g, want); end
        end
    endtask

    task chkint(input [8*24-1:0] name, input integer got, input integer want);
        begin
            ncase = ncase + 1;
            if (got == want) $display("[PASS] %0s = %0d", name, got);
            else begin fails = fails + 1;
                $display("[FAIL] %0s got=%0d want=%0d", name, got, want); end
        end
    endtask

    integer i;
    initial begin
        $display("=== tb_nav_ctrl start ===");
        repeat (5) @(posedge clk); rst_n = 1; repeat (3) @(posedge clk);

        //------------------------------------------------ T1 加两个途径点
        $display("[T1] ADD x2");
        do_cmd(8'd4, 32'sd1500, 32'sd0);      // $ADD,1500,0
        chkint("T1.count", q_count, 1);
        chk32("T1.px", q_px, 1500);
        chkint("T1.resp", resp_type, 1);      // R_ADD
        do_cmd(8'd4, 32'sd1500, 32'sd1000);   // $ADD,1500,1000
        chkint("T1.count2", q_count, 2);

        //------------------------------------------------ T2 ARM
        $display("[T2] ARM");
        do_cmd(8'd5, 32'sd0, 32'sd0);
        chkint("T2.st", st, 5);               // ARMED
        chkint("T2.resp", resp_type, 3);      // READY
        chkint("T2.arg", resp_arg, 2);

        //------------------------------------------------ T3 GO + 软启动
        $display("[T3] GO (soft start)");
        do_cmd(8'd6, 32'sd0, 32'sd0);
        chkint("T3.st", st, 1);               // NAV
        chk32("T3.vx_soft", vx_cmd, 0.0);     // 软启动期间不动
        // 目标(1500,0) vs th=0：方位角 0°，误差 0 ⇒ 首个 tick 即对准完成转直行
        repeat (1012*TICK_DIV) @(posedge clk);
        chk32("T3.vx_after", vx_cmd, 300.0);  // CRUISE 满速
        chk32("T3.vy_after", vy_cmd, 0.0);
        chk32("T3.w_after", w_cmd, 0.0);      // 航向锁定：err=0 ⇒ w=0

        //------------------------------------------------ T4 逼近目标 -> DONE
        $display("[T4] arrive wp0");
        x_q16 = 32'sd1400 <<< 16;             // 距目标 100mm < 150 阈值
        repeat (3*TICK_DIV) @(posedge clk);
        chkint("T4.done_type", resp_type, 6); // R_DONE
        chkint("T4.done_arg", resp_arg, 0);   // 第 0 点
        chkint("T4.wp_next", wp, 1);

        //------------------------------------------------ T5 第二段先对准（不平移）
        $display("[T5] segment 2 ALIGN (rotate only)");
        // 目标 (1500,1000)，车 (1400,0)，th=0 ⇒ 方位角 atan2(1000,100)=84.29°
        //   err=+84.29° > 2° ⇒ ALIGN：w=clamp(84.29*4,±90)=+90，且 vx=vy=0（不平移）
        repeat (2*TICK_DIV) @(posedge clk);
        chk32("T5.w", w_cmd, 90.0);
        chk32("T5.vx", vx_cmd, 0.0);
        chk32("T5.vy", vy_cmd, 0.0);

        //------------------------------------------------ T6 对准完成 -> 直行
        $display("[T6] aligned -> CRUISE");
        th_q16 = 32'sd5524660;                // 84.30° ≈ 方位角 ⇒ err≈0
        repeat (4*TICK_DIV) @(posedge clk);   // ALIGN 判定 1 tick + CRUISE 出速 1 tick + 余量
        chk32("T6.vx", vx_cmd, 300.0);        // 沿车头直行
        chk32("T6.vy", vy_cmd, 0.0);
        chk32("T6.w", w_cmd, 0.0);

        //------------------------------------------------ T6b 到最后一个点 -> ARRIVED
        $display("[T6b] arrive last -> ARRIVED");
        x_q16 = 32'sd1500 <<< 16;
        y_q16 = 32'sd950  <<< 16;             // 距目标 (1500,1000) 50mm < 150
        repeat (3*TICK_DIV) @(posedge clk);
        chkint("T6b.st", st, 2);              // S_ARR
        chk32("T6b.vx", vx_cmd, 0.0);

        //------------------------------------------------ T7 手动前进 100mm
        $display("[T7] MV FWD 100mm");
        x_q16 = 0; y_q16 = 0;
        do_cmd(8'd10, 32'sd0, 32'sd100);      // dir=0(FWD), 100mm
        repeat (TICK_DIV + 5) @(posedge clk); // ★等第一个 tick 出速度
        chk32("T7.vx", vx_cmd, 200.0);        // MAN_SPEED
        // need_ticks = 100*100/200 = 50 个 tick
        repeat (60*TICK_DIV) @(posedge clk);
        chkint("T7.st", st, 0);               // 走完回 IDLE

        //------------------------------------------------ T8 手动旋转 90 度
        $display("[T8] MV SPIN 90deg");
        do_cmd(8'd10, 32'sd4, 32'sd90);       // dir=4(SPIN), 90 度
        repeat (TICK_DIV + 5) @(posedge clk); // ★等第一个 tick 出速度
        chk32("T8.w", w_cmd, 90.0);           // SPIN_RATE 正(逆时针)
        repeat (130*TICK_DIV) @(posedge clk);   // SPIN 90 度需 100 个 tick
        chkint("T8.st", st, 0);

        //------------------------------------------------ T9 急停
        $display("[T9] ESTOP");
        do_cmd(8'd8, 32'sd0, 32'sd0);
        chkint("T9.st", st, 4);               // FAULT
        chk32("T9.vx", vx_cmd, 0.0);

        //------------------------------------------------ T10 清空队列
        $display("[T10] CLR");
        do_cmd(8'd3, 32'sd0, 32'sd0);
        chkint("T10.count", q_count, 0);
        chkint("T10.st", st, 0);

        //------------------------------------------------ T11 大坐标 20 m（应正常通过，无限制）
        $display("[T11] ADD 20000,20000 (20 m)");
        do_cmd(8'd4, 32'sd20000, 32'sd20000);
        repeat (4) @(posedge clk);
        chk32("T11.px", q_px, 20000.0);
        chk32("T11.py", q_py, 20000.0);
        chkint("T11.count", q_count, 1);

        //-------------------------------- T12 超大坐标 40000 mm（应饱和到 32000，不得回绕成负数）
        $display("[T12] ADD 40000,-40000 (saturate, must NOT wrap)");
        do_cmd(8'd3, 32'sd0, 32'sd0);           // 先清空
        do_cmd(8'd4, 32'sd40000, -32'sd40000);
        repeat (4) @(posedge clk);
        chk32("T12.px", q_px, 32000.0);         // 饱和值
        chk32("T12.py", q_py, -32000.0);
        if (q_px[31] === 1'b1) begin
            fails = fails + 1;
            $display("[FAIL] T12.px 回绕成负数! (>=0 才是对的)");
        end else begin
            ncase = ncase + 1;
            $display("[PASS] T12.px 未回绕（正数）");
        end

        //------------------------------------------------ T13 P4 避障全流程
        $display("[T13] obstacle -> AVOID x3 -> give up");
        do_cmd(8'd3, 32'sd0, 32'sd0);         // CLR（清 T12 的点）
        x_q16 = 32'sd0 <<< 16; y_q16 = 32'sd0 <<< 16; th_q16 = 32'sd0 <<< 16;
        do_cmd(8'd4, 32'sd3000, 32'sd0);      // 目标 (3000,0)：正前方
        do_cmd(8'd5, 32'sd0, 32'sd0);         // ARM
        do_cmd(8'd6, 32'sd0, 32'sd0);         // GO
        repeat (1020*TICK_DIV) @(posedge clk);// soft 过 → 对准(0°即过) → CRUISE
        // 注入障碍 20cm（vld 恒 1 = 10Hz 采样语义的加速版）
        dist_cm = 16'd20; dist_vld = 1;
        n = 0; while (st !== 3 && n < 100000) begin @(posedge clk); n = n + 1; end
        chkint("T13.trigger_st", st, 3);      // S_AVOID
        // 避障三阶段完成 → 回 NAV（ALIGN→CRUISE，st=1）
        n = 0; while (st !== 1 && n < 500000) begin @(posedge clk); n = n + 1; end
        // 障碍仍在（vld 恒 1）⇒ 连续 3 次避障后被判围死：放弃停车 $EVT,03
        n = 0; while (st !== 0 && n < 3000000) begin @(posedge clk); n = n + 1; end
        chkint("T13.giveup_st", st, 0);
        chkint("T13.giveup_evt", resp_type, 10);   // R_EVT
        chkint("T13.giveup_arg", resp_arg, 3);
        dist_vld = 0;

        $display("=== tb_nav_ctrl done: cases=%0d fails=%0d ===", ncase, fails);
        $finish;
    end
endmodule
