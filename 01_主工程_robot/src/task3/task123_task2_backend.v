// Supervisory safety around the delivered task-two controller; PI/encoder/output
// primitives and mean-travel short-brake completion are the original modules.
module task123_task2_backend #(
 parameter integer CLK_HZ=50000000,STALL_TICKS=20,WRONG_WAY_TICKS=10,
 parameter integer SETTLE_TICKS=3
)(input wire clk,rst_n,enable,emergency_stop,clear_fault,
 input wire [2:0] backend_action,input wire [63:0] wheel_targets,
 input wire [31:0] goal_counts,input wire [3:0] enc_a,enc_b,
 output wire [47:0] trace_duties,output wire [63:0] trace_efforts,
 output wire [3:0] m_in1,m_in2,m_pwm,output wire stdby,
 output wire [127:0] positions,output wire [63:0] velocities,
 output wire [31:0] encoder_errors,output reg [3:0] fault_wheels,
 output wire feedback_tick,output reg stationary,output reg fault,
 output reg [2:0] fault_code,output wire motion_done,braking,
 output wire [31:0] motion_progress);
 wire signed [15:0] first=$signed(wheel_targets[15:0]);
 wire [15:0] speed=first[15] ? -first:first;
 wire [3:0] target_valid;
 wire valid_command=backend_action<=3 && speed>0 && speed<=28 && &target_valid && goal_counts<32'h10000000;
 // Register the wide four-wheel command check before the original PI core.
 // Invalid commands still cut physical outputs immediately and latch a fault.
 reg valid_command_q;
 always @(posedge clk or negedge rst_n)
 if(!rst_n)valid_command_q<=0;
 else valid_command_q<=valid_command;
 wire core_run=enable && valid_command_q && !fault && !emergency_stop;
 wire [3:0] raw_in1,raw_in2,raw_pwm;
 wire raw_stdby,core_fault,core_stall,real_mode;
 wire [47:0] raw_duties;wire [63:0] efforts;
 task123_task2_auto_core #(.SPEED_WINDOW(CLK_HZ/100),.CLOCKS_PER_MS(CLK_HZ/1000)) u_task2(
 .sys_clk(clk),.rst_n(rst_n),.key(4'hf),.ir_in(1'b1),.enc_a(enc_a),.enc_b(enc_b),
 .auto_mode(1'b1),.auto_run(core_run),.auto_stop(emergency_stop),
 .auto_clear(clear_fault && !enable),.auto_action(backend_action),.auto_speed(speed),.auto_goal(goal_counts),
 .auto_done(motion_done),.auto_fault(core_fault),.auto_stall(core_stall),.auto_real(real_mode),
 .auto_positions(positions),.auto_velocities(velocities),.auto_errors(encoder_errors),
 .auto_travel(motion_progress),.auto_duties(raw_duties),.auto_efforts(efforts),.auto_tick(feedback_tick),
 .m_in1(raw_in1),.m_in2(raw_in2),.m_pwm(raw_pwm),.stdby(raw_stdby),.led(),.uart_tx());
 assign stdby=rst_n && core_run && valid_command && raw_stdby;
 assign m_in1=stdby ? raw_in1:4'b0;
 assign m_in2=stdby ? raw_in2:4'b0;
 assign m_pwm=stdby ? raw_pwm:4'b0;
 assign braking=stdby && motion_done;
 assign trace_duties=!stdby ? 48'd0 : braking ? {4{12'hfff}}:raw_duties;
 assign trace_efforts=efforts;
 reg [31:0] errors_d;reg was_enable;
 reg [31:0] stalled[0:3],wrong_way[0:3],settled;
 wire [3:0] quiet,directed,wrong,expired,bad_way,changed;
 genvar i;
 generate for(i=0;i<4;i=i+1)begin: wheels
 wire negative=backend_action==1 || (backend_action==2 && (i==0 || i==2)) ||
   (backend_action==3 && (i==1 || i==3));
 wire signed [15:0] expected=negative ? -$signed(speed):$signed(speed);
 wire signed [15:0] velocity=$signed(velocities[16*i+:16]);
 assign target_valid[i]=$signed(wheel_targets[16*i+:16])==expected;
 assign quiet[i]=velocity>=-1 && velocity<=1;
 assign directed[i]=negative ? velocity<0:velocity>0;
 assign wrong[i]=negative ? velocity>1:velocity< -1;
 assign expired[i]=stalled[i]>=STALL_TICKS;
 assign bad_way[i]=wrong_way[i]>=WRONG_WAY_TICKS;
 assign changed[i]=encoder_errors[8*i+:8]!=errors_d[8*i+:8];
 end endgenerate
 integer j;
 always @(posedge clk or negedge rst_n)
 if(!rst_n)begin
  fault<=0;fault_code<=0;fault_wheels<=0;errors_d<=0;was_enable<=0;settled<=0;stationary<=0;
  for(j=0;j<4;j=j+1)begin stalled[j]<=0;wrong_way[j]<=0;end
 end else begin
  errors_d<=encoder_errors;was_enable<=enable;
  if(emergency_stop)begin fault<=1;fault_code<=1;fault_wheels<=0;end
  else if(clear_fault && !enable)begin fault<=0;fault_code<=0;fault_wheels<=0;end
  else if(!fault)begin
   if(enable && !valid_command)begin fault<=1;fault_code<=4;fault_wheels<=0;end
   else if(core_stall || (core_run && !motion_done && |expired))begin fault<=1;fault_code<=2;fault_wheels<=core_stall ? 4'hf:expired;end
   else if(enable && was_enable && |changed)begin fault<=1;fault_code<=3;fault_wheels<=changed;end
   else if(core_run && !motion_done && |bad_way)begin fault<=1;fault_code<=5;fault_wheels<=bad_way;end
   else if(core_fault)begin fault<=1;fault_code<=4;fault_wheels<=0;end
  end
  if(feedback_tick)begin
   if(&quiet)begin if(settled<SETTLE_TICKS)settled<=settled+1'b1;if(settled>=SETTLE_TICKS-1)stationary<=1;end
   else begin settled<=0;stationary<=0;end
   for(j=0;j<4;j=j+1)begin
    if(!core_run || motion_done || directed[j])stalled[j]<=0;
    else if(stalled[j]<STALL_TICKS)stalled[j]<=stalled[j]+1'b1;
    if(!core_run || motion_done || !wrong[j])wrong_way[j]<=0;
    else if(wrong_way[j]<WRONG_WAY_TICKS)wrong_way[j]<=wrong_way[j]+1'b1;
   end
  end
 end
endmodule
