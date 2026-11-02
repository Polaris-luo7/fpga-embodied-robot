//=============================================================================
// tb_speed_loop.v — 速度闭环自检 testbench
//------------------------------------------------------------------------------
// 结构：
//   - DUT = speed_loop（quad_decoder + speed_meas + pi_controller + pwm_gen）
//   - 被控对象：一阶电机模型（真实时间 1 kHz 更新），
//       v_ss = 6000 计数/s × duty/4000 × dist_scale，v ← v+0.2(v_ss−v)
//   - 编码器激励：按 |v| 产生四相序列（事件间隔 = 50e6/|v| 个 clk）
//   - tick_gen 按真实 50000 clk 分频（不加速，兼验 1 kHz 时基）
// 检查项：
//   P1 正转阶跃收敛   P2 反转收敛+换向方向   P3 持续扰动恢复
//   P4 输出饱和       P5 停车归零            P6 tick 周期≈50000 clk
//   P7 换向死区期间 PWM 必须为低且时长≥100 clk（持续监测）
//=============================================================================
`timescale 1ns/1ps

module tb_speed_loop;

    reg clk = 0, rst_n = 0;
    reg run = 0, clr_i = 0, cnt_clr = 0;
    reg signed [15:0] target = 0, kp = 0, ki = 0;

    // 编码器激励（先声明，后被 DUT 引用）
    reg enc_a = 0, enc_b = 0;
    reg [1:0] qs = 2'b00;
    integer ph_cnt = 0;
    integer reload = 0;      // 事件间隔（clk），0 = 停转
    real    v;               // 轮速（计数/秒，带符号）
    real    dist_scale;      // 扰动：负载使增益下降

    wire        pwm_out, dir_out;
    wire signed [31:0] position;
    wire signed [15:0] speed, speed_m, duty;
    wire        period_valid_dbg, tick_dbg;

    speed_loop dut (
        .clk(clk), .rst_n(rst_n),
        .enc_a(enc_a), .enc_b(enc_b), .cnt_clr(cnt_clr),
        .run(run), .clr_i(clr_i),
        .target(target), .kp(kp), .ki(ki),
        .pwm_out(pwm_out), .dir_out(dir_out),
        .position(position), .speed(speed), .speed_m(speed_m), .duty(duty),
        .period_valid_dbg(period_valid_dbg), .tick_dbg(tick_dbg)
    );

    always #10 clk = ~clk;   // 50 MHz

    // ===================== 编码器激励 =====================
    always @(posedge clk) begin
        if (reload <= 0) begin
            ph_cnt <= 0;
        end else if (ph_cnt >= reload - 1) begin
            ph_cnt <= 0;
            if (v >= 0.0) begin      // 正转序列 00->01->11->10->00
                case (qs)
                    2'b00: qs <= 2'b01;
                    2'b01: qs <= 2'b11;
                    2'b11: qs <= 2'b10;
                    default: qs <= 2'b00;
                endcase
            end else begin            // 反转序列 00->10->11->01->00
                case (qs)
                    2'b00: qs <= 2'b10;
                    2'b10: qs <= 2'b11;
                    2'b11: qs <= 2'b01;
                    default: qs <= 2'b00;
                endcase
            end
        end else begin
            ph_cnt <= ph_cnt + 1;
        end
        enc_a <= qs[1];
        enc_b <= qs[0];
    end

    // ===================== 一阶电机模型（1 ms 更新） =====================
    localparam real VS_MAX  = 6000.0;   // 满占空稳态速度（计数/s）
    localparam real ALPHA   = 0.2;
    integer ms_cnt = 0;
    integer duty_i;
    real v_ss;
    always @(posedge clk) begin
        duty_i = duty;                  // 带符号扩展到 integer
        if (ms_cnt == 49999) begin
            ms_cnt <= 0;
            v_ss   = VS_MAX * ($itor(duty_i) / 4000.0) * dist_scale;
            v      = v + ALPHA * (v_ss - v);
            if (v > -1.0 && v < 1.0) reload = 0;
            else reload = $rtoi(50000000.0 / ((v < 0.0) ? -v : v));
        end else begin
            ms_cnt <= ms_cnt + 1;
        end
    end

    // ===================== 监测器 =====================
    integer err = 0;

    // P7: 换向死区监测（dir_req≠dir_out 期间 PWM 必须为低；时长≥98 clk）
    wire dir_req = duty[15];
    integer mismatch_cnt = 0;
    integer dead_events = 0;
    always @(posedge clk) begin
        if (!rst_n) mismatch_cnt <= 0;
        else if (dir_req !== dut.dir_out) begin
            mismatch_cnt <= mismatch_cnt + 1;
            if (pwm_out !== 1'b0) begin
                $display("!! P7 FAIL: pwm high during dir dead-time @%0t", $time);
                err = err + 1;
            end
        end else begin
            if (mismatch_cnt > 0) begin
                if (mismatch_cnt < 98) begin   // DEAD_CYCLES=100，容差 2
                    $display("!! P7 FAIL: dead-time too short (%0d clk) @%0t",
                             mismatch_cnt, $time);
                    err = err + 1;
                end
                dead_events = dead_events + 1;
            end
            mismatch_cnt <= 0;
        end
    end

    // P6: tick 周期监测（容差 ±1 clk，覆盖采样相位）
    integer tick_cnt = 0, tick_seen = 0;
    always @(posedge clk) begin
        tick_cnt <= tick_cnt + 1;
        if (tick_dbg) begin
            if (tick_seen && tick_cnt != 49999 && tick_cnt != 50000) begin
                $display("!! P6 FAIL: tick gap = %0d (expect 50000) @%0t",
                         tick_cnt, $time);
                err = err + 1;
            end
            tick_seen <= 1;
            tick_cnt  <= 0;
        end
    end

    // ---- P5 前 30ms 的事件/除法日志 ----
    reg p5dbg = 0;
    initial begin
        wait (p5dbg === 1'b1);
        repeat (30 * 50000) @(posedge clk);
        p5dbg = 0;
    end
    always @(posedge clk) begin
        if (p5dbg && dut.period_stb)
            $display("  [STB ] t=%0t period=%0d dir=%b", $time, dut.period, dut.u_dec.dir);
        if (p5dbg && dut.u_speed.t_done)
            $display("  [DIVT] t=%0t q=%0d", $time, dut.u_speed.t_quot);
    end

    // ---- 【诊断】tick 拍上 PI 实际消费的量（纯只读层次引用，不改设计）----
    // 默认关闭，保持日志简短。需要 tick 级证据时用 plusarg 打开：
    //     vsim -c +VTICK -do run.do work.tb_speed_loop
    // 保留理由：2026-10-01 正是靠它分辨出"打印假象 vs 真断"，
    //           没有它会把组合量的相位假象当成真实故障。
    reg vTICK;
    initial vTICK = $test$plusargs("VTICK");

    always @(posedge clk) begin
        if (vTICK && p5dbg && tick_dbg)
            $display("  [TICK] t=%0t speed=%0d piact=%0d sr=%0d sg=%b | bnd=%0d bv=%b bbusy=%b elp=%0d duty=%0d st=%b",
                     $time, speed, dut.u_pi.actual, dut.u_speed.speed_r, dut.u_speed.speed_r_sign,
                     dut.u_speed.b_hold, dut.u_speed.b_valid, dut.u_speed.b_busy,
                     dut.u_speed.elapsed, duty, dut.stopped);
    end

    // ===================== 测试序列 =====================
    integer k, j;

    // ---- 诊断：TB 侧实测编码器事件间隔 ----
    integer ev_gap = 0, ev_last_gap = 0;
    reg [1:0] qs_prev;
    always @(posedge clk) begin
        qs_prev <= qs;
        ev_gap  <= ev_gap + 1;
        if (qs !== qs_prev) begin
            ev_last_gap <= ev_gap;
            ev_gap      <= 0;
        end
    end
    task run_ms(input integer n);   // 等待 n 个真实 ms（50000 clk）
        begin for (k = 0; k < n * 50000; k = k + 1) @(posedge clk); end
    endtask

    initial begin
        v = 0.0; dist_scale = 1.0;
        repeat (10) @(posedge clk);
        rst_n = 1;
        kp = 16; ki = 4;
        repeat (10) @(posedge clk);
        run = 1;

        // ---- P1: 正转阶跃 4000 计数/s ----
        target = 16'sd4000;
        run_ms(300);
        $display("[P1 CONV ] v=%0.1f speed=%0d duty=%0d (target=4000)",
                 v, speed, duty);
        if (v >= 3600.0 && v <= 4400.0) $display("   P1 OK");
        else begin $display("   !! P1 FAIL"); err = err + 1; end

        // ---- P2: 反转 -3000 ----
        target = -16'sd3000;
        run_ms(400);
        $display("[P2 REV  ] v=%0.1f speed=%0d duty=%0d dir_out=%b (target=-3000)",
                 v, speed, duty, dir_out);
        if (v >= -3300.0 && v <= -2700.0) $display("   P2 OK");
        else begin $display("   !! P2 FAIL"); err = err + 1; end
        if (dir_out === 1'b1) $display("   P2 dir OK (reverse)");
        else begin $display("   !! P2 dir FAIL"); err = err + 1; end

        // ---- P3: 持续扰动（负载使增益×0.75）恢复 ----
        target = 16'sd4000;
        run_ms(300);
        dist_scale = 0.75;
        run_ms(400);
        $display("[P3 DIST ] dist=0.75 -> v=%0.1f speed=%0d duty=%0d (expect ~4000)",
                 v, speed, duty);
        if (v >= 3600.0 && v <= 4400.0) $display("   P3 OK (rejected)");
        else begin $display("   !! P3 FAIL"); err = err + 1; end
        dist_scale = 1.0;

        // ---- P4: 输出饱和（不可达目标） ----
        target = 16'sd30000;
        run_ms(80);
        $display("[P4 SAT  ] duty=%0d (expect 4000)", duty);
        if (duty == 16'sd4000) $display("   P4 OK (saturated)");
        else begin $display("   !! P4 FAIL"); err = err + 1; end

        // ---- P5: 停车归零 ----
        target = 16'sd0;
        p5dbg = 1;
        for (j = 0; j < 12; j = j + 1) begin
            run_ms(20);
        $display("[P5 dbg  ] t=%0t v=%0.1f speed=%0d piact=%0d duty=%0d i=%0d | speed_r=%0d sg=%b ev_gap=%0d | bnd=%0d bv=%b bbusy=%b elp=%0d tb=%b tp=%b",
                 $time, v, speed, dut.u_pi.actual, duty,
                 dut.u_pi.integ_out,
                 dut.u_speed.speed_r, dut.u_speed.speed_r_sign,
                 ev_last_gap,
                 dut.u_speed.b_quot, dut.u_speed.b_valid, dut.u_speed.b_busy,
                 dut.u_speed.elapsed, dut.u_speed.t_busy, dut.u_speed.t_pend);
        end
        $display("[P5 STOP ] v=%0.1f speed=%0d duty=%0d (expect all ~0)",
                 v, speed, duty);
        if (speed == 0 && duty == 0 && v > -60.0 && v < 60.0)
            $display("   P5 OK");
        else begin $display("   !! P5 FAIL"); err = err + 1; end

        // ---- 总结 ----
        $display("[P7 SUM  ] 换向死区事件 %0d 次，全部 ≥98 clk 且期间 PWM=0",
                 dead_events);
        if (err == 0) $display("=== tb_speed_loop: ALL PASS ===");
        else          $display("=== tb_speed_loop: %0d ERROR(S) ===", err);
        $finish;
    end
endmodule
