//=============================================================================
// mpu6050_ctrl.v -- MPU6050 初始化 + 周期读取（14 字节突发读）
//-----------------------------------------------------------------------------
// 【做什么】
//   上电后：等待 PWRUP_MS  ->  按表写 5 个配置寄存器  ->  进入无限循环：
//     写指针 0x3B  ->  repeated START 读地址  ->  连读 14 字节  ->  等待 PERIOD_MS
//   每次读完整帧后拉一拍 data_valid，并更新 accel/gyro/temp 输出。
//
// 【MPU6050 关键事实（写死在这里，改型号必须同步改）】
//   从机地址   : 0x68（AD0 接地；AD0 接高则 0x69）—— 7 位地址，读写位另算
//   寄存器窗口 : 0x3B~0x48 共 14 字节，顺序固定：
//                ACCEL_XOUT[15:8], [7:0], ACCEL_YOUT, ACCEL_ZOUT,
//                TEMP_OUT[15:8], [7:0], GYRO_XOUT, GYRO_YOUT, GYRO_ZOUT
//   自动递增   : 连读时寄存器指针自动 +1，所以一次读 14 字节即可拿到全部
//
// 【初始化表（5 条）与理由】
//   0x6B PWR_MGMT_1  = 0x01  -> 解除睡眠 + 时钟源选 PLL(X 陀螺参考)，
//                               比内部 8M RC 稳，姿态积分漂移更小
//   0x19 SMPLRT_DIV  = 0x09  -> 采样率 = 1kHz/(1+9) = 100 Hz（与软件 dt=10ms 对应）
//   0x1A CONFIG      = 0x03  -> DLPF_CFG=3：陀螺 42Hz / 加速度 44Hz 低通，
//                               抑制车体振动
//   0x1B GYRO_CONFIG = 0x18  -> FS_SEL=3：±2000 deg/s，灵敏度 16.4 LSB/(deg/s)
//   0x1C ACCEL_CONFIG= 0x08  -> AFS_SEL=2：±4 g，灵敏度 8192 LSB/g
//                               （机器人有振动与落地冲击，不用 ±2g）
//
// 【为什么每个字节单独发一次 go，而不是一条长序列】
//   复用 i2c_bit_shift 的通用命令接口，序列推进放在本模块，
//   好处是"命令组合"只在一处（bit_shift），本模块只关心"下一个字节是谁"。
//=============================================================================
`timescale 1ns / 1ps

module task123_mpu6050_checked #(
    parameter integer SYS_CLK_HZ = 50_000_000,
    parameter integer SCL_HZ     = 400_000,
    parameter integer PWRUP_MS   = 100,      // 上电等待（MPU6050 内部起振需要时间）
    parameter integer PERIOD_MS  = 10,       // 采样周期（10ms -> 100 Hz）
    parameter [6:0]   DEV_ADDR   = 7'h68     // 7 位从机地址
)(
    input  wire        clk,
    input  wire        rst_n,

    // ---- I2C 物理层（顶层做三态：sda = (oe && !o) ? 0 : z）----
    output wire        i2c_scl,
    input  wire        i2c_sda_in,
    output wire        i2c_sda_o,
    output wire        i2c_sda_oe,

    // ---- 传感器数据（原始 LSB）----
    output reg  signed [15:0] accel_x,
    output reg  signed [15:0] accel_y,
    output reg  signed [15:0] accel_z,
    output reg  signed [15:0] temp_raw,
    output reg  signed [15:0] gyro_x,
    output reg  signed [15:0] gyro_y,
    output reg  signed [15:0] gyro_z,
    output reg                data_valid,     // 每次 14 字节读完拉一拍

    // ---- 观测 ----
    output reg                frame_done,
    output reg                sample_good,
    output reg                ack_err,        // 曾出现过 NACK（粘滞，供诊断）
    output reg  [3:0]         dbg_state,
    output reg  [3:0]         dbg_sub
);

    // ---------------------------------------------------------------------
    // 命令位（与 i2c_bit_shift 内部定义一致）
    // ---------------------------------------------------------------------
    localparam [5:0] CMD_WR   = 6'b000001,
                     CMD_STA  = 6'b000010,
                     CMD_RD   = 6'b000100,
                     CMD_STO  = 6'b001000,
                     CMD_ACK  = 6'b010000,
                     CMD_NACK = 6'b100000;

    localparam integer N_INIT = 5;

    localparam integer PWRUP_CNT  = (SYS_CLK_HZ / 1000) * PWRUP_MS;
    localparam integer PERIOD_CNT = (SYS_CLK_HZ / 1000) * PERIOD_MS;

    // ⚠️ 诊断临时（2026-10-10）：读窗口起点从 0x3B(ACCEL_XOUT_H) 改为 0x75(WHO_AM_I)
    //   WHO_AM_I 是出厂固定值 0x68(=104)，只读、不需配置 ⇒ 用它判定"通信到底通不通"。
    //   还原时改回 8'h3B。
    localparam [7:0] RD_BASE = 8'h3B;   // 读窗口起始寄存器（加速度）

    // ---------------------------------------------------------------------
    // 初始化寄存器表
    // ---------------------------------------------------------------------
    function [7:0] INI_REG(input [2:0] i);
        case (i)
            3'd0: INI_REG = 8'h6B;   // PWR_MGMT_1
            3'd1: INI_REG = 8'h19;   // SMPLRT_DIV
            3'd2: INI_REG = 8'h1A;   // CONFIG
            3'd3: INI_REG = 8'h1B;   // GYRO_CONFIG
            3'd4: INI_REG = 8'h1C;   // ACCEL_CONFIG
            default: INI_REG = 8'h6B;
        endcase
    endfunction

    function [7:0] INI_VAL(input [2:0] i);
        case (i)
            3'd0: INI_VAL = 8'h01;   // 唤醒 + PLL(X gyro)
            3'd1: INI_VAL = 8'h09;   // 100 Hz
            3'd2: INI_VAL = 8'h03;   // DLPF 42Hz/44Hz
            3'd3: INI_VAL = 8'h18;   // ±2000 deg/s
            3'd4: INI_VAL = 8'h08;   // ±4 g
            default: INI_VAL = 8'h00;
        endcase
    endfunction

    // ---------------------------------------------------------------------
    // 子模块：通用位传输
    // ---------------------------------------------------------------------
    reg  [5:0] cmd_lat;
    reg  [7:0] tx_lat;
    reg        go;
    wire [7:0] rx_data;
    wire       trans_done;
    wire       ack_o;

    i2c_bit_shift_imu #(
        .SYS_CLK_HZ (SYS_CLK_HZ),
        .SCL_HZ     (SCL_HZ)
    ) u_bit (
        .clk        (clk),
        .rst_n      (rst_n),
        .cmd        (cmd_lat),
        .go         (go),
        .tx_data    (tx_lat),
        .sda_in     (i2c_sda_in),
        .rx_data    (rx_data),
        .trans_done (trans_done),
        .ack_o      (ack_o),
        .scl_out    (i2c_scl),
        .sda_o      (i2c_sda_o),
        .sda_oe     (i2c_sda_oe)
    );

    // ---------------------------------------------------------------------
    // 主序列状态机
    // ---------------------------------------------------------------------
    localparam [3:0] S_WAIT   = 4'd0,   // 上电等待
                     S_INIT_W = 4'd1,   // 初始化序列推进
                     S_RD_W   = 4'd2,   // 读序列推进
                     S_GAP    = 4'd3;   // 采样间隔

    reg init_ok, frame_ok;
    reg [3:0] st;
    reg [2:0] ini_idx;
    reg [1:0] ini_ph;       // 0=地址(STA|WR) 1=寄存器 2=数据(STO)
    reg [1:0] rd_step;      // 0=地址(写) 1=寄存器 2=地址(读) 3=读数据中
    reg [3:0] rd_idx;       // 0..13

    reg [31:0] pwr_cnt;
    reg [31:0] gap_cnt;

    // 读回的 14 字节
    reg [7:0] d0,  d1,  d2,  d3,  d4,  d5,  d6,  d7,
              d8,  d9,  d10, d11, d12, d13;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st         <= S_WAIT;
            ini_idx    <= 3'd0;
            ini_ph     <= 2'd0;
            rd_step    <= 2'd0;
            rd_idx     <= 4'd0;
            go         <= 1'b0;
            cmd_lat    <= 6'd0;
            tx_lat     <= 8'h00;
            pwr_cnt    <= 32'd0;
            gap_cnt    <= 32'd0;
            data_valid <= 1'b0;
            ack_err    <= 1'b0;
            init_ok <= 1'b0; frame_ok <= 1'b0;
            frame_done <= 1'b0; sample_good <= 1'b0;
            accel_x <= 16'sd0; accel_y <= 16'sd0; accel_z <= 16'sd0;
            temp_raw<= 16'sd0;
            gyro_x  <= 16'sd0; gyro_y  <= 16'sd0; gyro_z  <= 16'sd0;
            dbg_state  <= 4'd0;
            dbg_sub    <= 4'd0;
        end else begin
            go         <= 1'b0;      // 默认单拍脉冲
            data_valid <= 1'b0;
            frame_done <= 1'b0;
            dbg_state  <= st;

            case (st)

            // ---------------- 上电等待 ----------------
            S_WAIT: begin
                dbg_sub <= ini_ph;
                if (pwr_cnt >= PWRUP_CNT) begin
                    pwr_cnt <= 32'd0;
                    ini_idx <= 3'd0;
                    init_ok <= 1'b1;
                    // ini_ph 语义 = "**刚发完**的命令序号"：0=地址 1=寄存器 2=数据。
                    // 这一拍发的是地址，所以置 0（曾误置 1，导致"写寄存器号"整步被跳过）。
                    ini_ph  <= 2'd0;
                    cmd_lat <= CMD_WR | CMD_STA;
                    tx_lat  <= {DEV_ADDR, 1'b0};
                    go      <= 1'b1;
                    st      <= S_INIT_W;
                end else begin
                    pwr_cnt <= pwr_cnt + 32'd1;
                end
            end

            // ---------------- 初始化序列推进 ----------------
            S_INIT_W: begin
                dbg_sub <= {ini_idx, ini_ph};
                if (trans_done) begin
                    if (ack_o) begin ack_err <= 1'b1; init_ok <= 1'b0; end // 记录 NACK（不中断）

                    if (ini_ph == 2'd0) begin
                        // 刚发完 地址(STA|WR) -> 发寄存器号
                        cmd_lat <= CMD_WR;
                        tx_lat  <= INI_REG(ini_idx);
                        go      <= 1'b1;
                        ini_ph  <= 2'd1;
                    end else if (ini_ph == 2'd1) begin
                        // 刚发完寄存器号 -> 发数据 + STOP
                        cmd_lat <= CMD_WR | CMD_STO;
                        tx_lat  <= INI_VAL(ini_idx);
                        go      <= 1'b1;
                        ini_ph  <= 2'd2;
                    end else begin
                        // 一组写完 -> 下一组，或转读序列
                        ini_ph <= 2'd0;
                        if (ini_idx == N_INIT - 1) begin
                            // Finish the STOP before retrying a failed configuration.
                            if (init_ok && !ack_o) begin
                                frame_ok <= 1'b1;
                                cmd_lat <= CMD_WR | CMD_STA;
                                tx_lat <= {DEV_ADDR, 1'b0}; go <= 1'b1;
                                rd_step <= 2'd0; rd_idx <= 4'd0; st <= S_RD_W;
                            end else begin
                                pwr_cnt <= 0; st <= S_WAIT;
                            end
                        end else begin
                            ini_idx <= ini_idx + 3'd1;
                            cmd_lat <= CMD_WR | CMD_STA;
                            tx_lat  <= {DEV_ADDR, 1'b0};
                            go      <= 1'b1;
                        end
                    end
                end
            end

            // ---------------- 读序列推进 ----------------
            S_RD_W: begin
                dbg_sub <= {rd_step, rd_idx[1:0]};
                if (trans_done) begin
                    // ack_o is meaningful for the three WRITE phases only.
                    if (rd_step != 2'd3 && ack_o) begin ack_err <= 1'b1; frame_ok <= 1'b0; end
                    case (rd_step)
                    2'd0: begin    // 刚发完 地址(写)
                        cmd_lat <= CMD_WR;
                        tx_lat  <= RD_BASE;
                        go      <= 1'b1;
                        rd_step <= 2'd1;
                    end
                    2'd1: begin    // 刚发完 寄存器指针 -> repeated START + 读地址
                        cmd_lat <= CMD_WR | CMD_STA;
                        tx_lat  <= {DEV_ADDR, 1'b1};
                        go      <= 1'b1;
                        rd_step <= 2'd2;
                    end
                    2'd2: begin    // 刚发完 读地址 -> 开始读第 0 字节
                        cmd_lat <= CMD_RD | CMD_ACK;
                        go      <= 1'b1;
                        rd_idx  <= 4'd0;
                        rd_step <= 2'd3;
                    end
                    default: begin // 刚读回一个字节
                        case (rd_idx)
                            4'd0 : d0  <= rx_data;
                            4'd1 : d1  <= rx_data;
                            4'd2 : d2  <= rx_data;
                            4'd3 : d3  <= rx_data;
                            4'd4 : d4  <= rx_data;
                            4'd5 : d5  <= rx_data;
                            4'd6 : d6  <= rx_data;
                            4'd7 : d7  <= rx_data;
                            4'd8 : d8  <= rx_data;
                            4'd9 : d9  <= rx_data;
                            4'd10: d10 <= rx_data;
                            4'd11: d11 <= rx_data;
                            4'd12: d12 <= rx_data;
                            default: d13 <= rx_data;
                        endcase
                        if (rd_idx == 4'd13) begin
                            st      <= S_GAP;     // 最后一字节已带 NACK|STO
                            gap_cnt <= 32'd0;
                        end else begin
                            rd_idx  <= rd_idx + 4'd1;
                            // 倒数第二个之后要读的那个字节是最后一个 -> NACK + STOP
                            cmd_lat <= (rd_idx == 4'd12) ? (CMD_RD | CMD_NACK | CMD_STO)
                                                         : (CMD_RD | CMD_ACK);
                            go      <= 1'b1;
                        end
                    end
                    endcase
                end
            end

            // ---------------- 采样间隔 ----------------
            S_GAP: begin
                if (gap_cnt == 32'd0) begin
                    // Publish only an acknowledged, fully initialized frame.
                    frame_done <= 1'b1;
                    sample_good <= frame_ok && init_ok;
                    if (frame_ok && init_ok) begin
                    accel_x  <= {d0,  d1 };
                    accel_y  <= {d2,  d3 };
                    accel_z  <= {d4,  d5 };
                    temp_raw <= {d6,  d7 };
                    gyro_x   <= {d8,  d9 };
                    gyro_y   <= {d10, d11};
                    gyro_z   <= {d12, d13};
                    data_valid <= 1'b1;
                    end
                    gap_cnt  <= 32'd1;
                end else if (gap_cnt >= PERIOD_CNT) begin
                    frame_ok <= 1'b1;
                    gap_cnt <= 32'd0;
                    cmd_lat <= CMD_WR | CMD_STA;
                    tx_lat  <= {DEV_ADDR, 1'b0};
                    go      <= 1'b1;
                    rd_step <= 2'd0;
                    rd_idx  <= 4'd0;
                    st      <= S_RD_W;
                end else begin
                    gap_cnt <= gap_cnt + 32'd1;
                end
            end

            default: st <= S_WAIT;
            endcase
        end
    end

endmodule
