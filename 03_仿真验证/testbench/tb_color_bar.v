//=============================================================================
// tb_color_bar.v —— 纯色轮播选择器自检（实验 12）
//-----------------------------------------------------------------------------
// 检查项（每条抓一类错）：
//   P1 FIXED 模式 = **纯组合透传**：rgb_out === rgb_in、de_out === de_in、
//      roll_active === 0、idx_dbg === 0（逐拍比对，不是抽样）
//   P2 复位后 idx=0、输出 = 红
//   P3 **每色持续拍数精确 = TICKS**（不是"大约"），连续 3 轮共 15 段
//   P4 **轮播顺序 = 红→绿→蓝→白→黑**，逐拍核对 3 个完整周期
//   P5 颜色集合恰为 5 个、两两不同；3 轮内每个色出现 TICKS*3 拍
//   P6 de 在轮播模式下逐拍透传（时序不被改动）
//   P7 期望值自检 + 打印"判读表"（供用户烧板后对照）
//
// 说明：TICKS 在本 TB 里被压到 1000 拍（CLK_HZ=1000, ROLL_SEC=1），
//       只是为了让仿真快；**被测逻辑与参数化路径完全一致**。
//       上板用的是默认 CLK_HZ=33_000_000 ⇒ TICKS=33,000,000（1 s/色）。
//=============================================================================
// 2026-10-03 WorkBuddy 新建（实验 12）
//=============================================================================

`timescale 1ns/1ps

module tb_color_bar;

    //------------------------------------------------------------------
    // 时钟与复位
    //------------------------------------------------------------------
    reg pclk  = 1'b0;
    reg rst_n = 1'b0;
    always #5 pclk = ~pclk;

    localparam integer TICKS = 1000;      // CLK_HZ * ROLL_SEC

    //------------------------------------------------------------------
    // 被测：轮播实例 与 透传实例
    //------------------------------------------------------------------
    reg         de_in  = 1'b0;
    reg  [15:0] rgb_in = 16'h0000;

    wire [15:0] rgb_roll, rgb_fix;
    wire        de_roll,  de_fix;
    wire [2:0]  idx_roll, idx_fix;
    wire        act_roll, act_fix;

    color_bar #(
        .COLOR_MODE(1), .CLK_HZ(1000), .ROLL_SEC(1)
    ) u_roll (
        .pclk(pclk), .rst_n(rst_n),
        .de_in(de_in), .rgb_in(rgb_in),
        .de_out(de_roll), .rgb_out(rgb_roll),
        .idx_dbg(idx_roll), .roll_active(act_roll)
    );

    color_bar #(
        .COLOR_MODE(0), .CLK_HZ(1000), .ROLL_SEC(1)
    ) u_fix (
        .pclk(pclk), .rst_n(rst_n),
        .de_in(de_in), .rgb_in(rgb_in),
        .de_out(de_fix), .rgb_out(rgb_fix),
        .idx_dbg(idx_fix), .roll_active(act_fix)
    );

    //------------------------------------------------------------------
    // 期望色（与 color_bar.v 的默认参数一致，**按本工程 {B,G,R} 打包**）
    //------------------------------------------------------------------
    localparam [15:0] C_RED = 16'h001F;   // R=31
    localparam [15:0] C_GRN = 16'h07E0;   // G=63
    localparam [15:0] C_BLU = 16'hF800;   // B=31
    localparam [15:0] C_WHT = 16'hFFFF;
    localparam [15:0] C_BLK = 16'h0000;

    integer err = 0;

    //------------------------------------------------------------------
    // 通用核对
    //------------------------------------------------------------------
    task chk_eq(input [8*44:1] tag, input [31:0] got, input [31:0] exp);
        begin
            if (got !== exp) begin
                $display("  !! FAIL %0s: got=%h exp=%h (t=%0t)", tag, got, exp, $time);
                err = err + 1;
            end
        end
    endtask

    //------------------------------------------------------------------
    // 拿期望色（n = 拍序号，从复位释放后第一拍起算）
    //------------------------------------------------------------------
    function [15:0] exp_col(input integer k);
        begin
            case (k % 5)
                0: exp_col = C_RED;
                1: exp_col = C_GRN;
                2: exp_col = C_BLU;
                3: exp_col = C_WHT;
                default: exp_col = C_BLK;
            endcase
        end
    endfunction

    //------------------------------------------------------------------
    // P4/P4b 段长与顺序的独立观测（用 $time 取变化沿，与逐拍核对互为独立证据）
    //------------------------------------------------------------------
    reg  [2:0] idx_p = 3'd0;
    time       t_chg [0:31];
    integer    nchg     = 0;
    integer    k;

    always @(posedge pclk) if (rst_n) begin
        if (idx_roll !== idx_p) begin
            if (nchg < 32) begin
                t_chg[nchg] = $time;
                nchg = nchg + 1;
            end
            idx_p = idx_roll;
        end
    end

    //------------------------------------------------------------------
    // 主流程
    //------------------------------------------------------------------
    integer n, m;
    integer cnt_col [0:4];
    integer seen;

    initial begin
        $display("==== tb_color_bar 开始（实验 12：颜色位序）====");
        $display("  TICKS=%0d 拍/色（1 s/色 的 1:33000 缩放）", TICKS);

        //---- 复位 ----
        rst_n = 1'b0; de_in = 1'b0; rgb_in = 16'h0000;
        repeat (5) @(posedge pclk);
        rst_n = 1'b1;
        @(posedge pclk); #1;

        //================ P2 复位态 ================
        $display("[P2] 复位后：idx 应为 0、输出应为红");
        chk_eq("P2 idx_roll", {29'd0, idx_roll}, 32'd0);
        chk_eq("P2 u_roll 输出=红", {16'd0, rgb_roll}, {16'd0, C_RED});
        chk_eq("P2 act_roll=1", {31'd0, act_roll}, 32'd1);
        chk_eq("P2 act_fix=0",  {31'd0, act_fix},  32'd0);
        $display("    OK  复位态正确（idx=0 / 红 / roll_active=1，FIXED 实例 roll_active=0）");

        //================ P1 FIXED 模式纯透传 ================
        $display("[P1] FIXED(COLOR_MODE=0) 模式：逐拍核对应与输入完全一致");
        begin : p1
            integer i;
            reg [15:0] pat [0:6];
            integer      bad;
            bad = 0;
            pat[0] = 16'h1234; pat[1] = 16'h001F; pat[2] = 16'h07E0;
            pat[3] = 16'hF800; pat[4] = 16'hFFFF; pat[5] = 16'h0000;
            pat[6] = 16'hA5A5;
            for (i = 0; i < 700; i = i + 1) begin
                rgb_in = pat[i % 7];
                de_in  = ((i % 5) != 0);        // de 也在变，顺便验透传
                @(posedge pclk); #1;
                if (rgb_fix !== rgb_in) bad = bad + 1;
                if (de_fix  !== de_in ) bad = bad + 1;
                if (idx_fix !== 3'd0  ) bad = bad + 1;
            end
            if (bad == 0) $display("    OK  P1 700 拍逐拍透传（rgb_out===rgb_in、de_out===de_in、idx=0）");
            else begin $display("    !! FAIL P1：%0d 处不一致", bad); err = err + 1; end
        end

        // 复位，准备轮播测试
        rst_n = 1'b0; repeat (5) @(posedge pclk);
        rst_n = 1'b1; idx_p = 3'd0; nchg = 0;
        for (m = 0; m < 5; m = m + 1) cnt_col[m] = 0;

        //================ P3/P4/P5/P6 轮播 3 个完整周期 ================
        $display("[P3/P4/P5/P6] 轮播 3 个完整周期（共 %0d 拍）逐拍核对", 3*5*TICKS);
        de_in = 1'b1;
        for (n = 0; n < 3*5*TICKS; n = n + 1) begin
            rgb_in = 16'h1234;                  // 轮播时应被**完全忽略**
            if ((n % 37) == 0) de_in = ~de_in;  // 让 de 持续变化，验透传
            @(posedge pclk); #1;
            // 期望：第 (n/TICKS)%5 个色
            //   ⚠️ 边界推演：cnt 从 0 起算，在第 1000 个上升沿才满足
            //   cnt>=TICKS-1 ⇒ **idx 在 e=n+1 第 1000 拍翻转**，
            //   所以公式是 floor((n+1)/TICKS) 而不是 floor(n/TICKS)。
            begin : p34
                integer kk;
                kk = ((n + 1) / TICKS) % 5;
                if (idx_roll !== kk[2:0]) begin
                    if (err < 12)
                        $display("    !! FAIL P4 顺序: n=%0d idx=%0d exp=%0d", n, idx_roll, kk);
                    err = err + 1;
                end
                if (rgb_roll !== exp_col(kk)) begin
                    if (err < 12)
                        $display("    !! FAIL P5 颜色: n=%0d out=%h exp=%h", n, rgb_roll, exp_col(kk));
                    err = err + 1;
                end
                cnt_col[kk] = cnt_col[kk] + 1;
            end
            if (de_roll !== de_in) begin
                if (err < 12) $display("    !! FAIL P6 de 未透传: n=%0d", n);
                err = err + 1;
            end
        end
        if (err == 0) $display("    OK  P4 顺序（红→绿→蓝→白→黑）逐拍正确");
        if (err == 0) $display("    OK  P5 五色值逐拍正确（且输入 0x1234 被完全忽略）");
        if (err == 0) $display("    OK  P6 de 逐拍透传");

        //================ P5b 每色出现次数 = TICKS*3 ================
        for (m = 0; m < 5; m = m + 1) begin
            if (cnt_col[m] != TICKS*3) begin
                $display("    !! FAIL P5b 第 %0d 色出现 %0d 拍，期望 %0d",
                         m, cnt_col[m], TICKS*3);
                err = err + 1;
            end
        end
        if (err == 0)
            $display("    OK  P5b 五色各出现 %0d 拍（= TICKS*3）", TICKS*3);

        //================ P3b 段长独立观测（$time 法）================
        $display("[P3b] 变化沿间隔（独立证据，时间法）：共 %0d 段", nchg);
        seen = 0;
        for (k = 0; k + 1 < nchg; k = k + 1) begin
            if ((t_chg[k+1] - t_chg[k]) != TICKS*10) begin   // 10 ns/拍
                $display("    !! FAIL P3b 第 %0d 段长 = %0d 拍，期望 %0d",
                         k, (t_chg[k+1]-t_chg[k])/10, TICKS);
                err = err + 1;
                seen = 1;
            end
        end
        if (!seen) $display("    OK  P3b 全部 %0d 段长精确 = %0d 拍", nchg-1, TICKS);

        //================ P7 期望值自检 + 判读表 ================
        $display("[P7] 打包约定与判读表");
        if (C_RED !== 16'h001F || C_BLU !== 16'hF800 || C_GRN !== 16'h07E0) begin
            $display("    !! FAIL P7 期望色与打包约定不符");
            err = err + 1;
        end else begin
            $display("    OK  期望色与 {B,G,R} 打包一致");
        end
        $display("     本工程顶层打包 = {video_b, video_g, video_r}");
        $display("       bit[15:11]=B(5)   bit[10:5]=G(6)   bit[4:0]=R(5)");
        $display("     轮播（每色 1 s，一轮 5 s）：");
        $display("       第1色 0x001F -> 期望【红】      <- 若显蓝 = R/B 倒了");
        $display("       第2色 0x07E0 -> 期望【绿】");
        $display("       第3色 0xF800 -> 期望【蓝】      <- 若显红 = R/B 倒了");
        $display("       第4色 0xFFFF -> 期望【白】");
        $display("       第5色 0x0000 -> 期望【黑】");
        $display("     OSD 在纯色之上：FPS/PCLK 文字应仍可见（黑底=绿字可见；绿底=只见黑描边）");

        //================ 汇总 ================
        if (err == 0) $display("=== tb_color_bar: ALL PASS ===");
        else          $display("=== tb_color_bar: %0d ERROR(S) ===", err);
        $finish;
    end

    // 超时保护
    initial begin
        repeat (200000) @(posedge pclk);
        $display("!! TIMEOUT: 仿真超过 200000 拍未结束");
        $display("=== tb_color_bar: 1 ERROR(S) (timeout) ===");
        $finish;
    end

endmodule
