////////////////////////////////////////////////////////////////////////////////
// Module Name   : vision_blob
// Company       : 上海大学 · 全国大学生嵌入式芯片与系统设计竞赛'2026（高云）选题三
// Create Date   : 2026-10-14
// Description   : 任务五 目标定位 V1（精简版）—— 逐帧统计目标像素的包围框
//
//   输入：目标色判据（red_now）+ 该像素坐标（px_x / px_y），逐像素更新
//   输出：包围框 (x_min,x_max,y_min,y_max) + tgt_valid
//
// ★ 为什么输出只有包围框（2026-10-14 精简）：
//     初版还输出了 cx/cy/w/h/pix_cnt/sum_x/sum_y，其中 sum_x/sum_y 是 32 位累加器。
//     实测这会把 camera_pclk 域（本项目唯一时序净区，基线 TNS = 0.000 / 0 端点）
//     打进 32 个违例端点 —— 因为该域时钟走通用布线、偏斜本就达 -1.8 ns，
//     增量面积/布局扰动会把贴着 0 的 CDC 路径推负。
//     ⇒ 在相机域**只保留当前真正要用的最小逻辑**：
//         · 框中心 cx/cy 不需要除法：消费端用 (x_min+x_max)>>1 即可
//         · 宽高 w/h 同理：x_max-x_min+1
//         · pix_cnt / sum_x / sum_y 留到"阶段一 第 2 步"（要在 OSD 显示目标坐标时）再加，
//           届时先过一遍时序再决定是否放本域
//
// ★ 帧同步：用 frame_start（= 顶层 wr_load = DVP_Capture 的 DataVs）的**上升沿**
//     作为帧边界。本模块只"读"strobe 与坐标，**不驱动**它们 ⇒ 不影响写 DDR3 时序。
// ★ 延迟：输出为**上一帧**的结果。视觉伺服对 1 帧延迟不敏感。
// ★ 已知局限：画面里若有**多个互不相连**的红色物体，本模块给出的是能同时框住
//     它们的**大框**（不是各自一个框）。本项目单目标演示场景下无影响。
//     升级路径：将来需要"多目标 + 按面积取最大"时换成连通域标记（CCL），接口不变。
////////////////////////////////////////////////////////////////////////////////

module vision_blob (
    input               pclk,
    input               rst_n,

    // ---- 像素流（camera_pclk 域，来自 DVP_Capture）----
    input               px_valid,       // 像素有效（= DataValid）
    input               red_now,        // 目标色判据（来自 vision_front，组合）
    input      [10:0]   px_x,           // 列号 0..799
    input      [10:0]   px_y,           // 行号 0..479
    input               frame_start,    // 帧边界（= DataVs，消隐期间为高）

    // ---- 逐帧输出（包围框）----
    output reg  [11:0]  x_min,
    output reg  [11:0]  x_max,
    output reg  [11:0]  y_min,
    output reg  [11:0]  y_max,
    output reg          tgt_valid       // 本帧是否找到目标
);

    // ---- 帧内工作寄存器 ----
    reg [11:0] wx_min, wx_max, wy_min, wy_max;
    reg        w_found;

    // ---- frame_start 上升沿检测 ----
    reg  fs_d;
    wire fs_rise = frame_start & ~fs_d;

    always @(posedge pclk) fs_d <= frame_start;

    wire [11:0] x12 = {1'b0, px_x};
    wire [11:0] y12 = {1'b0, px_y};

    always @(posedge pclk or negedge rst_n) begin
        if (!rst_n) begin
            wx_min <= 12'hFFF;  wx_max <= 12'd0;
            wy_min <= 12'hFFF;  wy_max <= 12'd0;
            w_found <= 1'b0;

            x_min <= 12'd0;  x_max <= 12'd0;
            y_min <= 12'd0;  y_max <= 12'd0;
            tgt_valid <= 1'b0;
        end
        else if (fs_rise) begin
            // ===== 帧边界：锁存上一帧包围框，随后复位工作寄存器 =====
            tgt_valid <= w_found;
            x_min <= w_found ? wx_min : 12'd0;
            x_max <= w_found ? wx_max : 12'd0;
            y_min <= w_found ? wy_min : 12'd0;
            y_max <= w_found ? wy_max : 12'd0;

            wx_min <= 12'hFFF;  wx_max <= 12'd0;
            wy_min <= 12'hFFF;  wy_max <= 12'd0;
            w_found <= 1'b0;
        end
        else if (px_valid && red_now) begin
            // ===== 命中一个目标像素 =====
            w_found <= 1'b1;
            if (!w_found) begin
                wx_min <= x12;  wx_max <= x12;
                wy_min <= y12;  wy_max <= y12;
            end
            else begin
                if (x12 < wx_min) wx_min <= x12;
                if (x12 > wx_max) wx_max <= x12;
                if (y12 < wy_min) wy_min <= y12;
                if (y12 > wy_max) wy_max <= y12;
            end
        end
    end

endmodule