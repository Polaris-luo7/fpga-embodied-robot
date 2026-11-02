// Extracted task2 primitives; original reference files are unchanged.
// task3_ namespace; speed delta saturation and explicit PI output narrowing added.

module task3_quad_dec(
    input  wire clk, rst_n, a, b, clr,
    output reg  signed [31:0] pos,
    output reg  [7:0] err_cnt
);
    reg [1:0] a_s, b_s, ab_d;
    reg up, dn;
    wire [1:0] ab = {a_s[1], b_s[1]};

    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin a_s <= 2'b00; b_s <= 2'b00; end
        else        begin a_s <= {a_s[0], a}; b_s <= {b_s[0], b}; end

    always @(posedge clk or negedge rst_n)
        if (!rst_n) ab_d <= 2'b00;
        else        ab_d <= ab;

    always @(*) begin                       // {A_prev,B_prev,A_now,B_now}
        up = 1'b0; dn = 1'b0;
        case ({ab_d, ab})
            4'b0001, 4'b0111, 4'b1110, 4'b1000: up = 1'b1;
            4'b0010, 4'b1011, 4'b1101, 4'b0100: dn = 1'b1;
            default: ;
        endcase
    end

    always @(posedge clk or negedge rst_n)
        if (!rst_n)   begin pos <= 32'sd0; err_cnt <= 8'd0; end
        else if (clr) begin pos <= 32'sd0; err_cnt <= 8'd0; end
        else if (up)  pos <= pos + 32'sd1;
        else if (dn)  pos <= pos - 32'sd1;
        else if (ab_d != ab) err_cnt <= err_cnt + 8'd1;
endmodule

module task3_speed_meter #(parameter WIN = 500_000)(     // 10ms @50MHz
    input  wire clk, rst_n,
    input  wire signed [31:0] pos,
    output reg  signed [15:0] spd,
    output reg  tick
);
    reg [31:0] cnt;
    reg signed [31:0] pos_d;
    wire signed [31:0] delta = pos-pos_d;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin cnt <= 32'd0; pos_d <= 32'sd0; spd <= 16'sd0; tick <= 1'b0; end
        else begin
            tick <= 1'b0;
            if (cnt >= WIN - 1) begin
                cnt   <= 32'd0;
                spd   <= (delta > 32'sd32767) ? 16'sd32767 :
                         (delta < -32'sd32768) ? -16'sd32768 : delta[15:0];
                pos_d <= pos;
                tick  <= 1'b1;
            end else cnt <= cnt + 1'b1;
        end
endmodule

module task3_pid_pi_v2 #(
    parameter signed [15:0] UMAX = 16'sd4000,
    parameter signed [15:0] UFF  = 16'sd2400
)(
    input  wire clk, rst_n, en, clr,
    input  wire signed [15:0] kp_q, ki_q,
    input  wire signed [15:0] target, actual,
    output reg  signed [15:0] u
);
    wire signed [15:0] e  = target - actual;
    wire signed [31:0] pw = ($signed(e) * $signed(kp_q)) >>> 8;   // 比例项
    wire signed [31:0] iw = ($signed(e) * $signed(ki_q)) >>> 8;   // 积分增量
    wire signed [15:0] ff = (target == 16'sd0) ? 16'sd0 :
                            ((target > 16'sd0) ? UFF : -UFF);     // 死区前馈

    reg  signed [31:0] acc;
    wire signed [31:0] acc_nxt = acc + iw;
    wire signed [31:0] raw     = pw + acc_nxt + ff;
    wire signed [31:0] acc_cl  = (acc_nxt >  32'sd12000) ?  32'sd12000 :
                                 (acc_nxt < -32'sd12000) ? -32'sd12000 : acc_nxt;
    wire signed [31:0] u_cl    = (raw >  UMAX) ?  UMAX :
                                 (raw < -UMAX) ? -UMAX : raw;
    wire sat = (raw != u_cl) && ((e > 0) == (raw > 0));           // 抗积分饱和

    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin acc <= 32'sd0; u <= 16'sd0; end
        else if (clr) begin acc <= 32'sd0; u <= 16'sd0; end       // ← 积分清零
        else if (en) begin
            acc <= sat ? acc : acc_cl;
            u   <= u_cl[15:0];
        end
endmodule
