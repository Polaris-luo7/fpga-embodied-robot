// Independent nominal-90-degree rectangular bypass; measured angle calibration remains pending.
// Counts/rotation envelope require calibration; no side/rear visibility is claimed.
module task123_task2_route #(
 parameter integer CALIBRATED=0,PHASE_TICKS=200,SETTLE_TICKS=3,BRAKE_TICKS=20,
 parameter integer STOP_MM=350,DEPART_MM=450,TURN_CLEAR_MM=150,
 parameter [31:0] TURN_COUNTS=1600,OFFSET_COUNTS=2000,PASS_COUNTS=3200,
 parameter [15:0] NORMAL_SPEED=8,SLOW_SPEED=4,TURN_SPEED=4
)(input wire clk,rst_n,run_enable,emergency_stop,clear_fault,
 input wire sensor_bad,sample_strobe,sample_valid,front_trigger,
 input wire forward_observation_ok,motion_done,
 input wire [31:0] engine_progress,output wire [31:0] motion_goal,
 input wire [15:0] front_mm,sample_mm,input wire [1:0] speed_level,
 input wire [127:0] positions,input wire feedback_tick,stationary,motion_fault,feedback_fresh,
 output reg [2:0] command,output reg [15:0] speed,
 output reg [3:0] state,output reg [2:0] segment,
 output reg fault,output reg [2:0] fault_code,output reg completed,
 output wire busy,output wire [31:0] progress);
 localparam IDLE=0,CRUISE=1,SCAN=2,MOVE=3,DONE=4,BRAKE=5,FAULT=15;
 reg [31:0] phase_age,settle_age,brake_age;
 reg trigger_d,measurement_armed,route_finished;
 wire forward_segment=segment==1 || segment==3 || segment==5;
 wire bad_sample=sample_strobe && !sample_valid;
 wire blocked=sample_strobe && sample_valid && sample_mm<=STOP_MM;
 wire hard_close=sample_strobe && sample_valid && sample_mm<=TURN_CLEAR_MM;
 wire [31:0] goal=forward_segment ? (segment==3 ? PASS_COUNTS:OFFSET_COUNTS):TURN_COUNTS;
 wire valid_goals=TURN_COUNTS!=0 && OFFSET_COUNTS!=0 && PASS_COUNTS!=0 &&
 TURN_COUNTS<32'h40000000 && OFFSET_COUNTS<32'h40000000 && PASS_COUNTS<32'h40000000;
 assign busy=state==SCAN || state==MOVE || state==BRAKE;
 assign motion_goal=(state==MOVE || state==BRAKE)?goal:32'd0;
 always @(*)begin
 command=0;speed=0;
 if(rst_n && run_enable && !emergency_stop && !fault && !motion_fault &&
 !sensor_bad && !bad_sample && feedback_fresh)begin
 if(state==CRUISE && speed_level!=0 && !blocked)begin
 command=1;speed=(speed_level==2)?NORMAL_SPEED:SLOW_SPEED;end
 if(state==BRAKE)begin
 command=forward_segment ? 1:((segment==0 || segment==6)?3:4);
 speed=forward_segment ? SLOW_SPEED:TURN_SPEED;end
 if(state==MOVE)begin
 if(forward_segment)begin
 if(front_mm>STOP_MM && !blocked)begin command=1;speed=SLOW_SPEED;end
 end else if(front_mm>TURN_CLEAR_MM && !hard_close)begin
 command=((segment==0 || segment==6))?3:4;speed=TURN_SPEED;end
 end end end
 // Task-two controller owns mean absolute wheel travel and completion/brake.
 assign progress=(state==MOVE || state==BRAKE)?engine_progress:32'd0;
 task fail;input [2:0] reason;begin fault<=1;fault_code<=reason;state<=FAULT;measurement_armed<=0;end endtask
 always @(posedge clk or negedge rst_n)
 if(!rst_n)begin
 state<=IDLE;segment<=0;route_finished<=0;fault<=0;fault_code<=0;completed<=0;
 phase_age<=0;settle_age<=0;brake_age<=0;trigger_d<=0;measurement_armed<=0;
 end else begin
 completed<=0;trigger_d<=front_trigger;
 if(state!=SCAN || !stationary)measurement_armed<=0;
 else if(front_trigger && !trigger_d)measurement_armed<=settle_age>=SETTLE_TICKS;
 if(emergency_stop)fail(1);
 else if(clear_fault && !run_enable && !motion_fault)begin
 state<=IDLE;segment<=0;route_finished<=0;fault<=0;fault_code<=0;phase_age<=0;settle_age<=0;measurement_armed<=0;
 end else if(fault)state<=FAULT;
 else if(motion_fault)fail(2);
 else if(!run_enable)begin state<=IDLE;phase_age<=0;settle_age<=0;measurement_armed<=0;end
 else if(sensor_bad || bad_sample || !feedback_fresh)fail(3);
 else if(state==MOVE && ((forward_segment && (front_mm<=STOP_MM || blocked)) ||
 (!forward_segment && (front_mm<=TURN_CLEAR_MM || hard_close))))fail(4);
 else if(busy && phase_age>=PHASE_TICKS)fail(5);
 else begin
 if(feedback_tick && busy)phase_age<=phase_age+1;
 case(state)
 IDLE:begin state<=CRUISE;phase_age<=0;end
 CRUISE:if(front_mm<=STOP_MM || blocked)begin
 if(CALIBRATED && valid_goals && !route_finished)begin state<=SCAN;segment<=0;phase_age<=0;settle_age<=0;measurement_armed<=0;end
 else fail(6);
 end
 SCAN:begin
 if(!stationary)begin settle_age<=0;measurement_armed<=0;end
 else begin
 if(feedback_tick && settle_age<SETTLE_TICKS)settle_age<=settle_age+1;
 if(sample_strobe)begin
 measurement_armed<=0;
 if(measurement_armed && sample_valid)begin
 // A new forward-facing observation must be real echo or a normal no-return timeout.
 if((forward_segment || segment==7) && !forward_observation_ok)fail(3);
 else if(((forward_segment || segment==7) && sample_mm>=DEPART_MM) || (!forward_segment && segment!=7 && sample_mm>TURN_CLEAR_MM))begin
 if(segment==7)begin state<=CRUISE;route_finished<=1;completed<=1;end
 else begin state<=MOVE;end
 phase_age<=0;settle_age<=0;
 end else if(forward_segment || segment==7)begin
 // Remain stopped and request another fresh observation; phase timeout still runs.
 // A blocked path never starts translation and never silently advances the route.
 end else fail(4);
 end end end end
 MOVE:if(motion_done)begin
 phase_age<=0;settle_age<=0;measurement_armed<=0;brake_age<=0;state<=BRAKE;
 end
 BRAKE:begin
 // Keep task-two run/done asserted so its proven short-brake output is held.
 if(feedback_tick && brake_age<BRAKE_TICKS)brake_age<=brake_age+1'b1;
 if(stationary && brake_age>=BRAKE_TICKS)begin
  phase_age<=0;settle_age<=0;measurement_armed<=0;segment<=segment+1;state<=SCAN;
 end end
 DONE:begin
 // Wait for the final coast to finish before reporting completion once.
 if(stationary && feedback_tick && settle_age<SETTLE_TICKS)begin
 settle_age<=settle_age+1;
 if(settle_age==SETTLE_TICKS-1)completed<=1;
 end else if(!stationary)settle_age<=0;
 end
 default:state<=IDLE;
 endcase end end
endmodule
