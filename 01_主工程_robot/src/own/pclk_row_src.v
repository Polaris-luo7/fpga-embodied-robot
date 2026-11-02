//=============================================================================
// pclk_row_src.v — PCLK 频率 -> OSD 数值源（50 MHz 逻辑域）
//------------------------------------------------------------------------------
// 输入：cam_rate_meter 的 pclk_cnt（默认 1 秒窗口内累计的 PCLK 周期数 = PCLK Hz）
//       + trig（单拍，接 cam_rate_valid）
// 输出：MHz 整数（16 位）+ 单拍 nd，可直接喂 osd_num_writer。
//
// 显示格式（**约定，务必与顶层参数一致**）：
//   MHz = floor(pclk_cnt / 1e6)，限幅 0..999（3 位十进制）。
//   顶层配 LBL="PCLK" + LBL_LEN=4 + LEAD_ZERO=0 ⇒ 屏上看到
//        PCLK 24      （= 24 MHz）
//        PCLK  7      （= 7 MHz，前导空格）
//   ⚠️ **单位 MHz 不在屏上** —— 工程里用的是旧版 osd_num_writer，LBL 只有 32 位
//      （最多 4 个字符），装不下 "PCLK=" + 单位。屏上"PCLK 后面那个数就是 MHz"，
//      这条约定写在这里、实验记录里、以及交付说明里。若要屏上带单位，得等 T14
//      把 writer 换成 48 位 LBL 版本后另排版面。
//
// 写口仲裁（照 imu_display「优先 + 让路」的模式，但落地方式不同）：
//   工程里的 osd_num_writer 是**旧版，没有 stall/busy 输出口**，无法在突发中途把
//   writer 暂停，所以把"让路"做到**触发侧**：
//     · fps 行保持最高优先：它照旧在 cam_rate_valid 当拍就发 nd，本模块不影响它；
//     · 本模块的 nd 要等两件事都满足才发：
//         a) 自己的除法已完成（32 拍 = 640 ns，本身已把时间推后）；
//         b) 观测到 fps writer 的 nd（fps_nd_b，来自它的 nd_dbg，pclk 域）之后，
//            保护窗 DEFER_CYC 个 50 MHz 拍走完 ⇒ fps 突发已结束。
//   ⇒ 两个 writer 的突发在时间上不重叠，共享 cbuf 写口不会同拍冲突。
//
//   余量算术（写清楚，便于将来 NDIG/LBL_LEN 变大时复核）：
//     fps 突发全程 ≈ CDC 3 拍 + 突发 8 拍 ≈ 11 个 pclk(30.3 ns) ≈ 333 ns ≈ 17 个 50M 拍；
//     本模块最早发 nd 的时刻 = 除法 32 拍 = 640 ns ≈ 32 个 50M 拍。
//     ⇒ 余量 ≈ 15 个 50M 拍。DEFER_CYC=24 是叠加在观测量上的第二道保护，不是唯一依据。
//=============================================================================
`timescale 1ns/1ps

module pclk_row_src #(
    parameter integer DIVISOR   = 1_000_000,
    parameter integer DEFER_CYC = 24,     // fps 突发保护窗（50 MHz 拍）
    parameter integer VAL_MAX   = 999
)(
    input  wire        clk_src,     // 50 MHz 逻辑域
    input  wire        rst_n,
    input  wire [31:0] pclk_cnt,    // 1 秒窗口内的 PCLK 周期数（Hz）
    input  wire        trig,        // 单拍：新测量完成（接 cam_rate_valid）
    input  wire        fps_nd_b,    // fps writer 的 nd_dbg（pclk 域）—— 观测 fps 是否在写
    output reg  [15:0] val_src,     // MHz（整数，已限幅）
    output reg         nd_src,      // 单拍
    output wire [15:0] val_dbg,
    output reg         busy_dbg
);
    localparam [7:0] DEFER_INI = DEFER_CYC;
    localparam [1:0] S_IDLE = 2'd0, S_DIV = 2'd1, S_DEFER = 2'd2;

    //------------------------------------------------------------------
    // 1. fps 突发观测：pclk 域的 nd 打两拍进 50M 域，检测上升沿
    //    （单拍脉冲在 33 MHz 域宽 30.3 ns > 50 MHz 周期 20 ns，必被采到）
    //------------------------------------------------------------------
    reg f1, f2, f3;
    always @(posedge clk_src or negedge rst_n) begin
        if (!rst_n) {f1, f2, f3} <= 3'b000;
        else        {f1, f2, f3} <= {fps_nd_b, f1, f2};
    end
    wire fps_nd_rise = f2 & ~f3;

    reg [7:0] guard;
    always @(posedge clk_src or negedge rst_n) begin
        if (!rst_n)             guard <= 8'd0;
        else if (fps_nd_rise)   guard <= DEFER_INI;
        else if (guard != 8'd0) guard <= guard - 8'd1;
    end
    wire fps_busy = (guard != 8'd0);

    //------------------------------------------------------------------
    // 2. 除法器（变量 / 常量）
    //------------------------------------------------------------------
    reg  [31:0] dd_lat;
    reg         div_start;
    wire [31:0] div_quot;
    wire        div_busy, div_done;

    div_by_const #(.DIVISOR(DIVISOR), .DW(32)) u_div (
        .clk     (clk_src),
        .rst_n   (rst_n),
        .start   (div_start),
        .dividend(dd_lat),
        .quot    (div_quot),
        .busy    (div_busy),
        .done    (div_done)
    );

    //------------------------------------------------------------------
    // 3. 主状态机：trig -> 除法 -> 等 fps 空闲 -> 发 nd
    //------------------------------------------------------------------
    reg [1:0]  st;
    reg [15:0] mhz;

    always @(posedge clk_src or negedge rst_n) begin
        if (!rst_n) begin
            st        <= S_IDLE;
            div_start <= 1'b0;
            dd_lat    <= 32'd0;
            mhz       <= 16'd0;
            val_src   <= 16'd0;
            nd_src    <= 1'b0;
        end else begin
            nd_src    <= 1'b0;          // nd 单拍
            div_start <= 1'b0;          // start 单拍
            case (st)
                S_IDLE: if (trig) begin
                            dd_lat    <= pclk_cnt;
                            div_start <= 1'b1;
                            st        <= S_DIV;
                        end
                S_DIV:  if (div_done) begin
                            // 限幅到 3 位十进制（writer 那边也有 VAL_MAX 兜底）
                            mhz <= (div_quot > VAL_MAX) ? VAL_MAX[15:0] : div_quot[15:0];
                            st  <= S_DEFER;
                        end
                S_DEFER: if (!fps_busy) begin
                            val_src <= mhz;       // 先稳住值（CDC 在 nd 同拍采样）
                            nd_src  <= 1'b1;
                            st      <= S_IDLE;
                        end
                default: st <= S_IDLE;
            endcase
        end
    end

    always @(posedge clk_src or negedge rst_n) begin
        if (!rst_n) busy_dbg <= 1'b0;
        else        busy_dbg <= (st != S_IDLE);
    end

    assign val_dbg = val_src;

endmodule
