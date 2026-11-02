//------------------------------------------------------------------------------
// ⚠️ 2026-10-08 改名说明（**别改回去**）
//   原模块名 `i2c_bit_shift` 与工程里 `src/camera_init/i2c_control/i2c_bit_shift.v`
//   **重名** ⇒ 集成进 robot 时报 `ERROR (EX3794): Duplicate module name`。
//   摄像头那颗是**正在工作的既有件，不能动**，所以**本模块改名 `i2c_bit_shift_imu`**。
//   本文件与实例化它的 mpu6050_ctrl.v 已同步改名；rtl/ 与 robot/src/own/ 两边一致。
//------------------------------------------------------------------------------
// i2c_bit_shift.v -- I2C 通用位传输单元（主机侧）
//-----------------------------------------------------------------------------
// 【来源与取舍】
//   教材第 32 章《I2C 接口控制器设计与验证》提出「通用传输逻辑建模」：
//   把 START / STOP / 写字节 / 读字节 / ACK / NACK 拆成 6 条可组合命令，
//   由 cmd[5:0] 独热编码一次性下发。本模块按该思路重写（不照抄源码），
//   有三处差异，都是推演后认为必须改的：
//     ① 补 else 出口：教材 GEN_STA 只有 WR/RD 两个出口，若只发 START 会卡死；
//        这里补 else -> IDLE，并在 IDLE 补 STO 出口。
//     ② 空闲时释放 SDA：教材 IDLE 恒 sda_oe=1，会产生一个冲突窗口——
//        从机在 ACK 周期的下降沿就开始驱动下一位，而主机仍拉着 SDA 低，
//        若从机该位是 1（高阻）就会被吞掉。本模块 IDLE 且未启动时释放 SDA，
//        由外部上拉拉高，冲突窗口消失。
//     ③ 全参数化：SYS_CLK_HZ / SCL_HZ 走 parameter，换时钟只改参数。
//
// 【时序结构】每个 SCL 周期分 4 个节拍（sclk_plus 是节拍脉冲）：
//     节拍 0 = 建立数据（SCL 仍为低）
//     节拍 1 = SCL 上升
//     节拍 2 = SCL 保持高（**采样点**）
//     节拍 3 = SCL 下降
//   ⇒ 一个 bit 占 4*(SCL_CNT_M+1) 个系统时钟。
//   实际 SCL 频率 = SYS_CLK_HZ / (4*(SCL_CNT_M+1))，见下方 localparam 注释。
//
// 【电气】SDA 一律开漏：只在需要"拉低"时驱动 0（sda_oe=1 && sda_o=0），
//   其余交给外部上拉。顶层三态写法：
//     assign i2c_sda = (sda_oe && !sda_o) ? 1'b0 : 1'bz;
//   SCL 由主机推挽驱动（主机独占，教材同此处理）。
//=============================================================================
`timescale 1ns / 1ps

module i2c_bit_shift_imu #(
    parameter integer SYS_CLK_HZ = 50_000_000,   // 系统时钟
    parameter integer SCL_HZ     = 400_000       // 目标 SCL 频率（MPU6050 支持 400k）
)(
    input  wire       clk,
    input  wire       rst_n,

    // ---- 命令接口 ----
    input  wire [5:0] cmd,          // 独热命令组合（见下方 localparam）
    input  wire       go,           // 单拍启动脉冲（锁存 cmd / tx_data）
    input  wire [7:0] tx_data,      // 待发送字节

    // ---- 数据与状态 ----
    input  wire       sda_in,       // 总线 SDA 回读（用于 ACK / 读数据）
    output reg  [7:0] rx_data,      // 读回字节
    output reg        trans_done,   // 本次传输完成（单拍）
    output reg        ack_o,        // 从机应答：0=ACK，1=NACK

    // ---- 物理层 ----
    output reg        scl_out,      // SCL（1=高，0=低）
    output reg        sda_o,        // SDA 输出值（1=不拉低，0=拉低）
    output reg        sda_oe        // SDA 输出使能
);

    // ---------------------------------------------------------------------
    // 命令（独热，可按位或组合，例如 STA|WR|STO）
    // ---------------------------------------------------------------------
    localparam [5:0] CMD_WR   = 6'b000001,
                     CMD_STA  = 6'b000010,
                     CMD_RD   = 6'b000100,
                     CMD_STO  = 6'b001000,
                     CMD_ACK  = 6'b010000,
                     CMD_NACK = 6'b100000;

    // ---------------------------------------------------------------------
    // SCL 分频：SCL_CNT_M 个系统时钟 = 1 个节拍 = 1/4 个 SCL 周期
    //   50e6 / (400e3 * 4) = 31.25 -> 取整 31 -> -1 = 30
    //   实际 SCL = 50e6 / (4*31) = 403.2 kHz（误差 +0.8%，I2C 无影响）
    // ---------------------------------------------------------------------
    localparam integer SCL_CNT_M = (SYS_CLK_HZ / (SCL_HZ * 4)) - 1;

    // ---------------------------------------------------------------------
    // 状态机（与教材 32.7.2 的 7 状态一致）
    // ---------------------------------------------------------------------
    localparam [2:0] ST_IDLE      = 3'd0,
                     ST_GEN_STA   = 3'd1,
                     ST_WR_DATA   = 3'd2,
                     ST_RD_DATA   = 3'd3,
                     ST_CHECK_ACK = 3'd4,
                     ST_GEN_ACK   = 3'd5,
                     ST_GEN_STO   = 3'd6,
                     ST_STO_WAIT  = 3'd7;    // STOP 后保持总线空闲（I2C t_BUF 要求）

    reg [2:0]  state;
    reg [5:0]  cmd_r;       // 锁存的命令
    reg [7:0]  tx_r;        // 锁存的待发数据
    reg [4:0]  cnt;         // 节拍计数（0..31 = 8bit x 4拍；0..3 = 单拍动作）
    reg        en_div;
    reg [15:0] div_cnt;

    wire sclk_plus = (div_cnt >= SCL_CNT_M[15:0]);   // 节拍脉冲

    // ---------------- 分频计数器 ----------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            div_cnt <= 16'd0;
        end else if (en_div) begin
            if (div_cnt >= SCL_CNT_M[15:0]) div_cnt <= 16'd0;
            else                            div_cnt <= div_cnt + 16'd1;
        end else begin
            div_cnt <= 16'd0;
        end
    end

    // ---------------- 主状态机 ----------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= ST_IDLE;
            cnt        <= 5'd0;
            scl_out    <= 1'b1;      // I2C 空闲：SCL 高
            sda_o      <= 1'b1;      // 不拉低
            sda_oe     <= 1'b0;      // 释放总线
            trans_done <= 1'b0;
            ack_o      <= 1'b1;
            rx_data    <= 8'h00;
            en_div     <= 1'b0;
            cmd_r      <= 6'd0;
            tx_r       <= 8'h00;
        end else begin
            trans_done <= 1'b0;      // 默认单拍

            case (state)

            // ================= 空闲 =================
            ST_IDLE: begin
                cnt <= 5'd0;
                // 未启动时释放 SDA（见头部说明②）
                sda_oe <= 1'b0;
                sda_o  <= 1'b1;
                if (go) begin
                    cmd_r  <= cmd;
                    tx_r   <= tx_data;
                    en_div <= 1'b1;
                    if      (cmd & CMD_STA) state <= ST_GEN_STA;
                    else if (cmd & CMD_WR ) state <= ST_WR_DATA;
                    else if (cmd & CMD_RD ) state <= ST_RD_DATA;
                    else if (cmd & CMD_STO) state <= ST_GEN_STO;
                    else begin
                        state  <= ST_IDLE;
                        en_div <= 1'b0;
                        trans_done <= 1'b1;   // 空命令：立即报完成，避免上层悬挂
                    end
                end
            end

            // ================= 起始位 =================
            // 4 步：SDA 置 1 -> SCL 拉高 -> SDA 拉低（SCL 高时下降沿 = START）-> SCL 拉低
            ST_GEN_STA: begin
                if (sclk_plus) begin
                    if (cnt == 5'd3) cnt <= 5'd0;
                    else             cnt <= cnt + 5'd1;
                    case (cnt)
                        5'd0: begin sda_o  <= 1'b1; sda_oe <= 1'b1; end
                        5'd1: begin scl_out<= 1'b1;                 end
                        5'd2: begin sda_o  <= 1'b0; scl_out <= 1'b1;end
                        5'd3: begin scl_out<= 1'b0;                 end
                        default: ;
                    endcase
                    if (cnt == 5'd3) begin
                        if      (cmd_r & CMD_WR) state <= ST_WR_DATA;
                        else if (cmd_r & CMD_RD) state <= ST_RD_DATA;
                        else if (cmd_r & CMD_STO) state <= ST_GEN_STO;
                        else begin
                            state      <= ST_IDLE;
                            en_div     <= 1'b0;
                            trans_done <= 1'b1;
                        end
                    end
                end
            end

            // ================= 写 1 字节 =================
            // 32 拍 = 8 bit x 4 拍；cnt[4:2] 即 bit 序号（0..7）
            ST_WR_DATA: begin
                if (sclk_plus) begin
                    if (cnt == 5'd31) cnt <= 5'd0;
                    else              cnt <= cnt + 5'd1;
                    case (cnt)
                        5'd0, 5'd4, 5'd8, 5'd12, 5'd16, 5'd20, 5'd24, 5'd28:
                            begin sda_o <= tx_r[7 - cnt[4:2]]; sda_oe <= 1'b1; end
                        5'd1, 5'd5, 5'd9, 5'd13, 5'd17, 5'd21, 5'd25, 5'd29:
                            scl_out <= 1'b1;
                        5'd2, 5'd6, 5'd10, 5'd14, 5'd18, 5'd22, 5'd26, 5'd30:
                            scl_out <= 1'b1;
                        5'd3, 5'd7, 5'd11, 5'd15, 5'd19, 5'd23, 5'd27, 5'd31:
                            scl_out <= 1'b0;
                        default: ;
                    endcase
                    if (cnt == 5'd31) state <= ST_CHECK_ACK;
                end
            end

            // ================= 读 1 字节 =================
            // 采样点在 cnt%4==2（SCL 保持高的第二拍）
            ST_RD_DATA: begin
                if (sclk_plus) begin
                    if (cnt == 5'd31) cnt <= 5'd0;
                    else              cnt <= cnt + 5'd1;
                    case (cnt)
                        5'd0, 5'd4, 5'd8, 5'd12, 5'd16, 5'd20, 5'd24, 5'd28:
                            begin sda_oe <= 1'b0; scl_out <= 1'b0; end
                        5'd1, 5'd5, 5'd9, 5'd13, 5'd17, 5'd21, 5'd25, 5'd29:
                            scl_out <= 1'b1;
                        5'd2, 5'd6, 5'd10, 5'd14, 5'd18, 5'd22, 5'd26, 5'd30:
                            begin scl_out <= 1'b1; rx_data <= {rx_data[6:0], sda_in}; end
                        5'd3, 5'd7, 5'd11, 5'd15, 5'd19, 5'd23, 5'd27, 5'd31:
                            scl_out <= 1'b0;
                        default: ;
                    endcase
                    if (cnt == 5'd31) state <= ST_GEN_ACK;
                end
            end

            // ================= 检查从机应答 =================
            ST_CHECK_ACK: begin
                if (sclk_plus) begin
                    if (cnt == 5'd3) cnt <= 5'd0;
                    else             cnt <= cnt + 5'd1;
                    case (cnt)
                        5'd0: begin sda_oe <= 1'b0; scl_out <= 1'b0; end
                        5'd1: scl_out <= 1'b1;
                        5'd2: begin ack_o <= sda_in; scl_out <= 1'b1; end
                        5'd3: scl_out <= 1'b0;
                        default: ;
                    endcase
                    if (cnt == 5'd3) begin
                        if (cmd_r & CMD_STO) begin
                            state <= ST_GEN_STO;
                        end else begin
                            state      <= ST_IDLE;
                            en_div     <= 1'b0;
                            trans_done <= 1'b1;
                        end
                    end
                end
            end

            // ================= 主机给应答 =================
            ST_GEN_ACK: begin
                if (sclk_plus) begin
                    if (cnt == 5'd3) cnt <= 5'd0;
                    else             cnt <= cnt + 5'd1;
                    case (cnt)
                        5'd0: begin
                            sda_oe <= 1'b1;
                            scl_out<= 1'b0;
                            if      (cmd_r & CMD_ACK ) sda_o <= 1'b0;
                            else if (cmd_r & CMD_NACK) sda_o <= 1'b1;
                        end
                        5'd1: scl_out <= 1'b1;
                        5'd2: scl_out <= 1'b1;
                        5'd3: scl_out <= 1'b0;
                        default: ;
                    endcase
                    if (cnt == 5'd3) begin
                        if (cmd_r & CMD_STO) begin
                            state <= ST_GEN_STO;
                        end else begin
                            state      <= ST_IDLE;
                            en_div     <= 1'b0;
                            trans_done <= 1'b1;
                        end
                    end
                end
            end

            // ================= 停止位 =================
            // SDA 拉低 -> SCL 拉高 -> SDA 释放（SCL 高时上升沿 = STOP）-> SCL 保持高
            ST_GEN_STO: begin
                if (sclk_plus) begin
                    if (cnt == 5'd3) cnt <= 5'd0;
                    else             cnt <= cnt + 5'd1;
                    case (cnt)
                        5'd0: begin sda_o  <= 1'b0; sda_oe <= 1'b1; end
                        5'd1: begin scl_out<= 1'b1;                 end
                        5'd2: begin sda_o  <= 1'b1; scl_out <= 1'b1;end
                        5'd3: begin scl_out<= 1'b1;                 end
                        default: ;
                    endcase
                    if (cnt == 5'd3) begin
                        state  <= ST_STO_WAIT;   // 不立即报完成：先让总线空闲
                    end
                end
            end

            // ============= STOP 后的总线空闲保持（I2C t_BUF）=============
            //  2026-10-09 补：写操作以 STOP 结束，规范要求 STOP 到下一个 START
            //  之间留 t_BUF = 4.7us(100k) / 1.3us(400k)。原实现 STOP 后立刻
            //  trans_done，间隔仅几十 ns ⇒ 违反 t_BUF，真机可能导致器件不识别
            //  新的 START（仿真模型不查这一条，故仿真通过）。
            //  8 个节拍：100k 约 20us、400k 约 5us，两种速率都满足。
            ST_STO_WAIT: begin
                if (sclk_plus) begin
                    if (cnt == 5'd7) begin
                        cnt        <= 5'd0;
                        state      <= ST_IDLE;
                        en_div     <= 1'b0;
                        trans_done <= 1'b1;
                    end else begin
                        cnt <= cnt + 5'd1;
                    end
                end
            end

            default: state <= ST_IDLE;
            endcase
        end
    end

endmodule
