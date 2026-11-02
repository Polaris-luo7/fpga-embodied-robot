//=============================================================================
// bt_cmd2.v -- 任务四指令解析器（P1）: $PING/$STAT/$CLR/$ADD/$ARM/$GO/$STOP/
//             $ESTOP/$WP/$MV/$GOP/$TEL
//-----------------------------------------------------------------------------
// 【为什么单独一个文件】bt_link.v（含 bt_frame/bt_cmd/bt_telem）已在 exp35
//   /exp36 位流中验证过，**不动它**。本模块直接吃 bt_frame 的输出
//   （frame + frame_len + frame_vld），旁路扩展指令集。
//
// 【帧格式】`$` <字段0> [`,` <字段1> [`,` <字段2>]]  —— ASCII，字段间逗号分隔
//   字段0 = 命令字（大写）；字段1/2 = 参数（可有符号十进制 或 MV 的方向名）
//
// 【解析方式】单遍扫描状态机（比"按长度穷举比对"综合代价小得多且好扩展）：
//   F_MNEM -> F_F1 -> F_F2，每个字段同时累加其"数值"与"原始文本"。
//   数值: 十进制，支持前导 '-'；非数字字符不参与数值(仅文本字段)。
//
// 【输出】
//   cmd[7:0]     见 CMD_* 常量
//   arg0/arg1    有符号 32bit（mm / 0.1度 / 点号 / 方向码，随 cmd 而定）
//   cmd_vld      1 拍脉冲（与本模块同域）
//
// 【验收】tb_bt_cmd2.v：12 条指令逐条注入并核对 cmd 与参数（含负参数、
//   非法帧、超长帧）。仿真 0 error。
//=============================================================================
`timescale 1ns/1ps

module bt_cmd2 #(
    parameter integer FRAME_MAX = 32
)(
    input  wire                   clk,
    input  wire                   rst_n,
    input  wire [FRAME_MAX*8-1:0] frame,
    input  wire [5:0]             frame_len,
    input  wire                   frame_vld,

    output reg  [7:0]             cmd,
    output reg  signed [31:0]     arg0,
    output reg  signed [31:0]     arg1,
    output reg                    cmd_vld
);
    // ---- 命令码 ----
    localparam [7:0] CMD_NONE  = 8'd0,
                     CMD_PING  = 8'd1,
                     CMD_STAT  = 8'd2,
                     CMD_CLR   = 8'd3,
                     CMD_ADD   = 8'd4,
                     CMD_ARM   = 8'd5,
                     CMD_GO    = 8'd6,
                     CMD_STOP  = 8'd7,
                     CMD_ESTOP = 8'd8,
                     CMD_WP    = 8'd9,
                     CMD_MV    = 8'd10,
                     CMD_GOP   = 8'd11,
                     CMD_TEL   = 8'd12,
                     CMD_ORG   = 8'd13,   // ★重置原点：把当前位姿/朝向设为世界系原点
                     CMD_CFG   = 8'd14;   // ★运行时可切换极性：$CFG,1,encMask / $CFG,2,dirMask

    // MV 方向码（放入 arg0）
    localparam [7:0] MV_FWD = 8'd0, MV_BACK = 8'd1, MV_LEFT = 8'd2,
                     MV_RIGHT = 8'd3, MV_SPIN = 8'd4, MV_ARC = 8'd5;

    localparam [1:0] F_IDLE = 2'd0, F_MNEM = 2'd1, F_F1 = 2'd2, F_F2 = 3'd3;
    reg  [2:0] fld_st;
    reg  [5:0] idx;
    reg  [23:0] mnem;      // 字段0 原始文本(取低 8 位足够比对)
    reg  [23:0] txt1;      // 字段1 原始文本(MV 方向名 / 数字的 ASCII)
    reg  signed [31:0] n1, n2;      // 字段1/2 数值
    reg        neg1, neg2;
    reg        d1, d2;     // 该字段是否见过数字
    reg  [1:0] c1n;        // 字段1 已收字符数(只留前 3 个, 供 MV 方向判定)

    wire [7:0] bcur = frame[idx*8 +: 8];
    wire is_end  = (idx >= frame_len);
    wire is_comma= (bcur == 8'h2C);            // ','
    wire is_digit= (bcur >= 8'h30) && (bcur <= 8'h39);
    wire is_minus= (bcur == 8'h2D);            // '-'

    // ------------------------------------------------------------------
    // 命令字译码（组合）: 以字段0 的低 24bit 文本比对
    // ------------------------------------------------------------------

    // 直接取帧头字节做精确比对（避免移位窗口长度歧义）
    wire [7:0] b1 = frame[15:8],  b2 = frame[23:16], b3 = frame[31:24],
               b4 = frame[39:32], b5 = frame[47:40];

    wire m_ping  = (b1==8'h50)&&(b2==8'h49)&&(b3==8'h4E)&&(b4==8'h47)&&(frame_len==6'd5);
    wire m_stat  = (b1==8'h53)&&(b2==8'h54)&&(b3==8'h41)&&(b4==8'h54)&&(frame_len==6'd5);
    wire m_clr   = (b1==8'h43)&&(b2==8'h4C)&&(b3==8'h52)&&(frame_len==6'd4);
    wire m_arm   = (b1==8'h41)&&(b2==8'h52)&&(b3==8'h4D)&&(frame_len==6'd4);
    wire m_go    = (b1==8'h47)&&(b2==8'h4F)&&(frame_len==6'd3);
    wire m_stop  = (b1==8'h53)&&(b2==8'h54)&&(b3==8'h4F)&&(b4==8'h50)&&(frame_len==6'd5);
    wire m_estop = (b1==8'h45)&&(b2==8'h53)&&(b3==8'h54)&&(b4==8'h4F)&&(b5==8'h50)&&(frame_len==6'd6);
    wire m_add   = (b1==8'h41)&&(b2==8'h44)&&(b3==8'h44)&&(b4==8'h2C);
    wire m_wp    = (b1==8'h57)&&(b2==8'h50)&&(b3==8'h2C);
    wire m_mv    = (b1==8'h4D)&&(b2==8'h56)&&(b3==8'h2C);
    wire m_gop   = (b1==8'h47)&&(b2==8'h4F)&&(b3==8'h50)&&(b4==8'h2C);
    wire m_tel   = (b1==8'h54)&&(b2==8'h45)&&(b3==8'h4C)&&(b4==8'h2C);
    wire m_org   = (b1==8'h4F)&&(b2==8'h52)&&(b3==8'h47)&&(frame_len==6'd4);   // "$ORG\n"
    wire m_cfg   = (b1==8'h43)&&(b2==8'h46)&&(b3==8'h47)&&(b4==8'h2C);          // "$CFG,"

    reg [7:0] mnem_code;
    always @(*) begin
        if      (m_estop) mnem_code = CMD_ESTOP;
        else if (m_ping ) mnem_code = CMD_PING;
        else if (m_stat ) mnem_code = CMD_STAT;
        else if (m_stop ) mnem_code = CMD_STOP;
        else if (m_clr  ) mnem_code = CMD_CLR;
        else if (m_arm  ) mnem_code = CMD_ARM;
        else if (m_go   ) mnem_code = CMD_GO;
        else if (m_add  ) mnem_code = CMD_ADD;
        else if (m_wp   ) mnem_code = CMD_WP;
        else if (m_mv   ) mnem_code = CMD_MV;
        else if (m_gop  ) mnem_code = CMD_GOP;
        else if (m_tel  ) mnem_code = CMD_TEL;
        else if (m_org  ) mnem_code = CMD_ORG;
        else if (m_cfg  ) mnem_code = CMD_CFG;
        else              mnem_code = CMD_NONE;
    end

    // MV 方向码（由字段1 文本判定: txt1 低 3 字节 = 方向名前 3 字符）
    wire [7:0] t2 = txt1[23:16], t1 = txt1[15:8], t0 = txt1[7:0];
    wire [7:0] dir_code =
        ((t2==8'h46)&&(t1==8'h57)&&(t0==8'h44)) ? MV_FWD   :  // FWD
        ((t2==8'h42)&&(t1==8'h41)&&(t0==8'h43)) ? MV_BACK  :  // BAC
        ((t2==8'h4C)&&(t1==8'h45)&&(t0==8'h46)) ? MV_LEFT  :  // LEF
        ((t2==8'h52)&&(t1==8'h49)&&(t0==8'h47)) ? MV_RIGHT :  // RIG
        ((t2==8'h53)&&(t1==8'h50)&&(t0==8'h49)) ? MV_SPIN  :  // SPI
        ((t2==8'h41)&&(t1==8'h52)&&(t0==8'h43)) ? MV_ARC   :  // ARC
                                                 8'hFF;

    // 统一装配（避免 arg0 被两个 always 块驱动）：
    //   MV 的 arg0 = 方向码；其余命令 arg0 = 字段1 数值
    wire signed [31:0] n1s = neg1 ? -n1 : n1;
    wire signed [31:0] n2s = neg2 ? -n2 : n2;
    wire signed [31:0] arg0_asm = (mnem_code == CMD_MV) ? $signed({24'd0, dir_code}) : n1s;

    integer i;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fld_st <= F_IDLE; idx <= 6'd0;
            mnem <= 24'd0; txt1 <= 24'd0;
            n1 <= 32'sd0; n2 <= 32'sd0; neg1 <= 1'b0; neg2 <= 1'b0;
            d1 <= 1'b0; d2 <= 1'b0; c1n <= 2'd0;
            cmd <= CMD_NONE; arg0 <= 32'sd0; arg1 <= 32'sd0; cmd_vld <= 1'b0;
        end
        else begin
            cmd_vld <= 1'b0;
            case (fld_st)
            F_IDLE: if (frame_vld) begin
                idx <= 6'd1;                  // 跳过 '$'
                mnem <= 24'd0; txt1 <= 24'd0;
                n1 <= 32'sd0; n2 <= 32'sd0;
                neg1 <= 1'b0; neg2 <= 1'b0; d1 <= 1'b0; d2 <= 1'b0;
                fld_st <= F_MNEM;
            end
            F_MNEM: begin
                if (is_end) begin                // 无参数命令结束
                    fld_st <= F_IDLE;
                    cmd    <= mnem_code;
                    arg0   <= 32'sd0; arg1 <= 32'sd0;   // 无参命令
                    cmd_vld<= 1'b1;
                end
                else if (is_comma) begin
                    idx <= idx + 6'd1; fld_st <= F_F1; c1n <= 2'd0;
                end
                else begin
                    mnem <= {mnem[15:0], bcur};
                    idx  <= idx + 6'd1;
                end
            end
            F_F1: begin
                if (is_end || is_comma) begin
                    fld_st <= is_end ? F_IDLE : F_F2;
                    idx    <= idx + 6'd1;
                    if (is_end) begin
                        cmd    <= mnem_code;
                        arg0   <= arg0_asm;
                        arg1   <= 32'sd0;
                        cmd_vld<= 1'b1;
                    end
                end
                else begin
                    // ★只保留前 3 个字符(MV 方向名比对用; 曾误用"最后 3 个"导致
                    //   SPIN/RIGHT 等 4~5 字符的名字窗口错位 -> dir=0xFF)
                    if (c1n < 2'd3) begin
                        txt1 <= {txt1[15:0], bcur};
                        c1n  <= c1n + 2'd1;
                    end
                    if (is_digit) begin
                        n1 <= n1*10 + (bcur - 8'h30); d1 <= 1'b1;
                    end
                    else if (is_minus) neg1 <= 1'b1;
                    idx <= idx + 6'd1;
                end
            end
            F_F2: begin
                if (is_end) begin
                    fld_st <= F_IDLE;
                    cmd    <= mnem_code;
                    arg0   <= arg0_asm;
                    arg1   <= n2s;
                    cmd_vld<= 1'b1;
                end
                else begin
                    if (is_digit) begin
                        n2 <= n2*10 + (bcur - 8'h30); d2 <= 1'b1;
                    end
                    else if (is_minus) neg2 <= 1'b1;
                    idx <= idx + 6'd1;
                end
            end
            default: fld_st <= F_IDLE;
            endcase
        end
    end

endmodule
