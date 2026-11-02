// Raw measurements remain observable; 4000 is an internal control sentinel,
// never reported as a measured distance. User authorizes one obstacle in open space.
// All unavailable returns are tolerated only in the explicit bypass window.
// Short pulses remain near stops; stale acquisition still faults.
module task123_small_target_sensing #(
 parameter integer CLK_HZ=50000000, STALE_US=600000
)(input wire clk,rst_n,echo,allow_no_return,
 output wire trig,output wire sample_begin,
 output wire [1:0] raw_level,output wire [2:0] raw_fault,
 output wire [15:0] raw_front,
 output wire sample_strobe,raw_valid,output wire [15:0] raw_mm,
 output wire [2:0] sample_error,
 output wire [1:0] control_level,output wire control_bad,
 output wire [15:0] control_front,output wire control_valid,
 output wire [15:0] control_mm);
 wire tick_us;wire [1:0] channel;wire [2:0] control_fault;
 // Once bypass has started, every unavailable return follows the user's
 // one-small-obstacle assumption. A short Echo is explicitly near, not clear.
 wire too_close=!raw_valid && sample_error==3'd5;
 wire no_return=allow_no_return && !raw_valid && !too_close;
 assign control_valid=raw_valid || too_close || no_return;
 assign control_mm=too_close ? 16'd0 : no_return ? 16'd4000 : raw_mm;
 assign control_bad=control_fault[0];
 hcsr04_small_target_scan #(.CLK_HZ(CLK_HZ),.SENSOR_COUNT(1)) u_scan(
 .clk(clk),.rst_n(rst_n),.echo(echo),.trig(trig),.tick_us(tick_us),.sample_begin(sample_begin),
 .sample_strobe(sample_strobe),.sample_channel(channel),.sample_mm(raw_mm),
 .sample_valid(raw_valid),.sample_error(sample_error));
 obstacle_decision #(.SENSOR_COUNT(1),.STALE_US(STALE_US)) u_raw(
 .clk(clk),.rst_n(rst_n),.tick_us(tick_us),.sample_strobe(sample_strobe),
 .sample_channel(channel),.sample_mm(raw_mm),.sample_valid(raw_valid),
 .speed_level(raw_level),.stop_req(),.slow_req(),.avoid_dir(),.sensor_fault(raw_fault),
 .front_mm(raw_front),.left_mm(),.right_mm());
 obstacle_decision #(.SENSOR_COUNT(1),.STALE_US(STALE_US)) u_control(
 .clk(clk),.rst_n(rst_n),.tick_us(tick_us),.sample_strobe(sample_strobe),
 .sample_channel(channel),.sample_mm(control_mm),.sample_valid(control_valid),
 .speed_level(control_level),.stop_req(),.slow_req(),.avoid_dir(),.sensor_fault(control_fault),
 .front_mm(control_front),.left_mm(),.right_mm());
endmodule
