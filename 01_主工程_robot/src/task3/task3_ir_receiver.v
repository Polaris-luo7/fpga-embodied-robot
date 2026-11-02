// NEC receiver copied into task-three namespace; original retained.
module task3_ir_receiver(
    input  wire clk,            // 50MHz 系统时钟
    input  wire rst_n,          // 复位，低有效
    input  wire ir_in,          // 红外接收头输出（空闲高、收到载波时低）
    output reg  [7:0] cmd,      // 解出来的命令字节（就是键码）
    output reg  cmd_valid,      // 解出一整帧有效命令时，输出一个时钟周期的高脉冲
    output reg  repeat_code     // 收到"重复码"时输出一个脉冲（表示按键被长按）
                                // 注：不能叫 repeat —— repeat 是 Verilog 保留关键字
);
    // ------------------------------------------------------------------
    // 1) 输入同步：外部信号相对时钟是"异步"的，先打两拍再用，
    //    这是所有异步输入的通用做法（同按键消抖里的处理）
    // ------------------------------------------------------------------
    reg [2:0] s;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) s <= 3'b111;                 // 复位时按"空闲高"处理
        else        s <= {s[1:0], ir_in};        // 移位：ir_in 依次进入 s[0]、s[1]、s[2]
    wire ir = s[2];                              // 用最稳的那一级

    // ------------------------------------------------------------------
    // 2) 边沿检测 + 脉宽计数
    //    ir_d 是上一拍的 ir；比较 ir_d 和 ir 就能知道"刚才跳变了吗"
    // ------------------------------------------------------------------
    reg        ir_d;
    wire       fall = ir_d & ~ir;                // 下降沿：载波开始（一段低电平的起点）
    wire       rise = ~ir_d & ir;                // 上升沿：载波结束（一段低电平的终点）

    reg [19:0] cnt;                              // 当前这段电平已经持续了多少个时钟
    reg [19:0] low_cnt, high_cnt;                // 上一段低电平 / 高电平的时长
    reg        leader_low, bit_low;              // 这段低电平是"引导码的低"还是"数据位的低"

    // 15ms（750000 时钟）没有跳变 → 认为信号结束，放弃当前帧
    wire timeout = (cnt >= 20'd750_000);

    reg [31:0] data;                             // 正在拼装的 32 位数据
    reg [5:0]  bitcnt;                           // 已经收到第几位（0~31）
    reg        in_frame;                         // 是否正在接收一帧

    // 在"下降沿"这一拍，cnt 里装的就是刚刚测完的高电平时长，
    // 用它判断这一位是 0 还是 1（阈值取 1.1ms = 55000 时钟，正好在 0.56 和 1.69 中间）
    wire high_is_one = (cnt > 20'd55_000);
    // 把这一位拼进数据里：低位先发，所以往右移、新位放最左边，
    // 收完 32 位后 data[0] 就是最先收到的那个位，正好是地址的最低位
    wire [31:0] data_nxt = {high_is_one, data[31:1]};

    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin
            ir_d <= 1'b1; cnt <= 20'd0; low_cnt <= 20'd0; high_cnt <= 20'd0;
            leader_low <= 1'b0; bit_low <= 1'b0;
            data <= 32'd0; bitcnt <= 6'd0; in_frame <= 1'b0;
            cmd <= 8'd0; cmd_valid <= 1'b0; repeat_code <= 1'b0;
        end else begin
            ir_d <= ir;
            cmd_valid   <= 1'b0;                 // 这两个都是"单周期脉冲"，默认拉低
            repeat_code <= 1'b0;

            // ---- 脉宽计数：有跳变就清零，否则一直加（超时后停住不再加）----
            if (fall || rise)  cnt <= 20'd0;
            else if (!timeout) cnt <= cnt + 1'b1;
            else               cnt <= cnt;

            // ---- 上升沿：一段低电平结束，记下它的长度并判断性质 ----
            if (rise) begin
                low_cnt <= cnt;
                // 约 9ms（>7ms）的低电平 = 引导码的低电平
                leader_low <= (cnt > 20'd350_000);
                // 约 0.56ms（0.3~0.9ms）的低电平 = 数据位的低电平
                bit_low    <= (cnt > 20'd15_000) && (cnt < 20'd45_000);
            end

            // ---- 下降沿：一段高电平结束，可以判定"前一段低 + 这一段高"是什么了 ----
            if (fall) begin
                high_cnt <= cnt;
                if (leader_low) begin
                    if ((cnt > 20'd150_000) && (cnt < 20'd300_000)) begin
                        // 高 ≈4.5ms → 这是引导码：准备接收后面 32 位数据
                        in_frame <= 1'b1; bitcnt <= 6'd0; data <= 32'd0;
                    end else if ((cnt > 20'd70_000) && (cnt < 20'd150_000)) begin
                        // 高 ≈2.25ms → 这是重复码（按键被长按）
                        repeat_code <= 1'b1;
                    end
                end else if (bit_low && in_frame) begin
                    // 这是数据位：把这一位拼进去
                    data <= data_nxt;
                    if (bitcnt == 6'd31) begin
                        // 收满 32 位：先做校验，再输出命令
                        bitcnt   <= 6'd0;
                        in_frame <= 1'b0;
                        // 地址反码、命令反码都要对得上，才算一帧有效数据
                        if ((data_nxt[15:8] == ~data_nxt[7:0]) &&
                            (data_nxt[31:24] == ~data_nxt[23:16])) begin
                            cmd       <= data_nxt[23:16];   // 命令字节就是键码
                            cmd_valid <= 1'b1;
                        end
                    end else begin
                        bitcnt <= bitcnt + 1'b1;
                    end
                end
            end

            // ---- 超时：长时间没有跳变，说明这一帧结束了 ----
            if (timeout) in_frame <= 1'b0;
        end
endmodule
