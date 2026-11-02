// P35 single-ranger candidate. Defaults forbid physical motor drive.
// Route enable means user accepted provisional counts, not measured physical calibration.
// 12s phase and a bounded 60s session for four turns; other motor protections retained. No-return allowed after measured obstacle entry.
module task123_task2_fix_board #(
 parameter integer CLK_HZ=50000000,UART_BAUD=115200,TELEMETRY_CYCLES=CLK_HZ/20,
 parameter integer MOTOR_ARMED=0,ROUTE_CALIBRATED=0,
 parameter [31:0] MAX_SESSION_CYCLES=60*CLK_HZ,
 parameter integer PHASE_TICKS=1200,
 parameter [31:0] TURN_COUNTS=1200,OFFSET_COUNTS=2000,PASS_COUNTS=3200
)(input wire sys_clk,rst_n,ir_in,input wire [3:0] key,enc_a,enc_b,
 input wire imu_valid,input wire [31:0] roll_q16,pitch_q16,yaw_q16,
 input wire [2:0] echo,output wire [0:0] trig,
 output wire [3:0] m_in1,m_in2,m_pwm,led,output wire stdby,uart_tx,
 //==== 任务四接入（2026-10-17）====
 input  wire        nav_hold,          // 1 = 任务四接管电机：本模块整链停
 output wire [15:0] obs_front_mm,      // 滤波后距离 mm（来自 u_sensing）
 output wire        obs_front_vld,     // 距离有效
 output wire [127:0] obs_positions,    // 四轮累计计数（交叉校验）
 output wire [63:0]  obs_velocities);  // 四轮速度 counts/10ms
 wire [3:0] effective_key,filtered_a,filtered_b;wire run_request,estop,clear;
 wire [7:0] ir_cmd;wire ir_valid,ir_repeat;
 wire run_permit,timeout_stop;wire safe_estop=estop || timeout_stop;
 wire [1:0] level,avoid_dir,sample_channel;wire [2:0] sensors,sample_error;
 wire [15:0] front,left_unused,right_unused,sample_mm,speed;
 wire strobe,valid,stop_unused,slow_unused,acquisition_begin;
 wire [15:0] control_front,control_sample_mm;
 wire [1:0] control_level;
 wire control_valid,control_bad;
 wire [127:0] positions;wire [63:0] velocities;wire [31:0] errors;
 wire [3:0] fault_wheels,state;wire [2:0] command,motor_code,route_code,segment;
 wire route_fault,adapter_fault,motor_fault,stationary,tick,busy,completed;
 wire all_fault=route_fault || adapter_fault || motor_fault || timeout_stop;
 wire requested=(MOTOR_ARMED!=0) && run_permit && !nav_hold;  // 任务四接管时整链停
 reg no_return_window;
 always @(posedge sys_clk or negedge rst_n)
 if(!rst_n)no_return_window<=0;
 else if(!requested || (clear && !run_request))no_return_window<=0;
 else if(state==2 && segment==0)no_return_window<=1;
 // Keep the user-authorized single-obstacle window through final resume until
 // the run ends. No-return never arms itself in idle or initial approach.

 // A valid return or ordinary no-return timeout can decide the next forward leg.
 // A completed out-of-range pulse (error3) follows the authorized single-obstacle
 // no-return policy. Pre-trigger-high Echo (error4) still rejects forward motion.
 // This does not turn an unavailable raw sample into a reported real distance.
 wire forward_segment=segment==1 || segment==3 || segment==5;
 wire forward_observation_ok=valid || (no_return_window && (sample_error==1 || sample_error==2 || sample_error==3));
 wire route_sensor_bad=control_bad || (no_return_window && strobe && !valid &&
   (sample_error==4) && (state==1 || (state==3 && forward_segment)));
 wire [31:0] progress,motion_goal,engine_progress;wire motion_done,braking;
 wire [47:0] trace_duties;wire [63:0] trace_efforts;
 reg feedback_seen;reg [31:0] feedback_age;
 wire feedback_fresh=feedback_seen && feedback_age<CLK_HZ/10;
 always @(posedge sys_clk or negedge rst_n)
 if(!rst_n)begin feedback_seen<=0;feedback_age<=0;end
 else if(tick)begin feedback_seen<=1;feedback_age<=0;end
 else if(feedback_age<CLK_HZ/10)feedback_age<=feedback_age+1;
 task3_ab_filter u_filter(sys_clk,rst_n,enc_a,enc_b,filtered_a,filtered_b);
 task3_ir_receiver u_ir(sys_clk,rst_n,ir_in,ir_cmd,ir_valid,ir_repeat);
 task3_remote_keys #(.PRESS_CYCLES(3*(CLK_HZ/50))) u_remote(
 .clk(sys_clk),.rst_n(rst_n),.key(key),.ir_cmd(ir_cmd),.ir_valid(ir_valid),.ir_repeat(ir_repeat),.effective_key(effective_key));
 task3_board_control #(.DEBOUNCE_CYCLES(CLK_HZ/50)) u_keys(
 .clk(sys_clk),.rst_n(rst_n),.key(effective_key),.fault(all_fault),.run_request(run_request),.emergency_stop(estop),.clear_fault(clear));
 task3_run_limit #(.MAX_RUN_CYCLES(MAX_SESSION_CYCLES)) u_limit(sys_clk,rst_n,run_request && !nav_hold,clear,run_permit,timeout_stop);
 task123_small_target_sensing #(.CLK_HZ(CLK_HZ)) u_sensing(
 .clk(sys_clk),.rst_n(rst_n),.echo(echo[0]),.trig(trig[0]),.allow_no_return(no_return_window),
 .sample_begin(acquisition_begin),
 .raw_level(level),.raw_fault(sensors),.raw_front(front),
 .sample_strobe(strobe),.raw_valid(valid),.raw_mm(sample_mm),.sample_error(sample_error),
 .control_level(control_level),.control_bad(control_bad),.control_front(control_front),
 .control_valid(control_valid),.control_mm(control_sample_mm));
 task123_task2_route #(.CALIBRATED(ROUTE_CALIBRATED),.PHASE_TICKS(PHASE_TICKS),.STOP_MM(350),.DEPART_MM(450),.TURN_COUNTS(TURN_COUNTS),.NORMAL_SPEED(20),.SLOW_SPEED(12),.TURN_SPEED(20),
 .OFFSET_COUNTS(OFFSET_COUNTS),.PASS_COUNTS(PASS_COUNTS)) u_route(
 .clk(sys_clk),.rst_n(rst_n),.run_enable(requested && feedback_seen),.emergency_stop(safe_estop),
 .clear_fault(clear),.sensor_bad(route_sensor_bad),.sample_strobe(strobe),.sample_valid(control_valid),
 .forward_observation_ok(forward_observation_ok),.motion_done(motion_done),.engine_progress(engine_progress),.motion_goal(motion_goal),
 .front_trigger(acquisition_begin),.front_mm(control_front),.sample_mm(control_sample_mm),.speed_level(control_level),
 .positions(positions),.feedback_tick(tick),.stationary(stationary),.motion_fault(motor_fault || adapter_fault),
 .feedback_fresh(feedback_fresh),.command(command),.speed(speed),.state(state),.segment(segment),
 .fault(route_fault),.fault_code(route_code),.completed(completed),.busy(busy),.progress(progress));
 wire backend_run;wire [2:0] backend_action;wire [63:0] targets;
 task3_task2_adapter u_adapter(.clk(sys_clk),.rst_n(rst_n),.run_enable(requested && feedback_seen),
 .emergency_stop(safe_estop),.clear_fault(clear),.backend_ready(feedback_fresh),.backend_fault(motor_fault),
 .command(route_fault ? 3'd0:command),.speed(speed),.backend_run(backend_run),
 .backend_action(backend_action),.wheel_targets(targets),.fault(adapter_fault));
 reg motor_enable_q;reg [2:0] action_q;reg [63:0] targets_q;
 always @(posedge sys_clk or negedge rst_n)
 if(!rst_n)begin motor_enable_q<=0;action_q<=0;targets_q<=0;end
 else begin motor_enable_q<=backend_run;action_q<=backend_action;targets_q<=targets;end
 task123_task2_backend #(.CLK_HZ(CLK_HZ),.STALL_TICKS(20)) u_motor(
 .clk(sys_clk),.rst_n(rst_n),.enable(motor_enable_q && requested && !all_fault && !control_bad && command!=0),
 .emergency_stop(safe_estop),.clear_fault(clear && !run_request),.backend_action(action_q),.wheel_targets(targets_q),
 .trace_duties(trace_duties),.trace_efforts(trace_efforts),
 .goal_counts(motion_goal),.motion_done(motion_done),.braking(braking),.motion_progress(engine_progress),
 .enc_a(filtered_a),.enc_b(filtered_b),.m_in1(m_in1),.m_in2(m_in2),.m_pwm(m_pwm),.stdby(stdby),
 .positions(positions),.velocities(velocities),.encoder_errors(errors),.fault_wheels(fault_wheels),
 .feedback_tick(tick),.stationary(stationary),.fault(motor_fault),.fault_code(motor_code));
 // ST encodes segment while moving (8..E), scan=2, cruise=1, fault=F.
 wire [3:0] display_state=(state==3)?(4'd8+{1'b0,segment}):state;
 // Freeze the exact first-fault sample subtype, independent of later recovery.
 reg [2:0] fault_sample_error;reg fault_sample_saved;
 always @(posedge sys_clk or negedge rst_n)
 if(!rst_n)begin fault_sample_error<=0;fault_sample_saved<=0;end
 else if(clear && !run_request)begin fault_sample_error<=0;fault_sample_saved<=0;end
 else if(all_fault && !fault_sample_saved)begin
   fault_sample_error<=sample_error;fault_sample_saved<=1;
 end
 wire [66:0] route_diagnostic;
 // First route fault freezes the pre-fault state, inputs and directed progress.
 // Bits: 0 sensor bad, 1 invalid sample, 2 stale feedback, 3 close,
 //       4 motor fault, 5 adapter fault, 6 emergency, 7 session timeout.
 wire [7:0] diagnostic_causes={timeout_stop,safe_estop,adapter_fault,motor_fault,
   (state==3 && (((segment==1 || segment==3 || segment==5) && control_front<=350) ||
     (!(segment==1 || segment==3 || segment==5) && control_front<=150))),
   !feedback_fresh,(strobe && !control_valid),route_sensor_bad};
 task123_route_fault_record u_record(.clk(sys_clk),.rst_n(rst_n),
   .clear(clear && !run_request),.fault(route_fault),.route_code(route_code),
   .route_state(state),.segment(segment),.causes(diagnostic_causes),
   .front_mm(front),.progress(progress),.record(route_diagnostic));
 wire [199:0] history_word;wire [6:0] history_count;wire history_frozen;
 wire telemetry_done;reg [5:0] history_index;reg [2:0] history_divider;
 wire history_sample=tick && history_divider==4;
 always @(posedge sys_clk or negedge rst_n)
 if(!rst_n)begin history_divider<=0;history_index<=0;end
 else begin
   if(clear && !run_request)begin history_divider<=0;history_index<=0;end
   else begin
     if(tick)history_divider<=history_divider==4 ? 3'd0 : history_divider+1'b1;
     if(telemetry_done)history_index<=history_index+1'b1;
   end
 end
 // payload: front16, V64, effective duty48, PI(C)16, target(C)16,
 // flags8, route state4, segment3, command3, speed level2.
 wire [7:0] history_flags={timeout_stop,safe_estop,adapter_fault,motor_fault,
    sensors[0],feedback_fresh,stdby,run_permit};
 task123_motion_history u_history(.clk(sys_clk),.rst_n(rst_n),
   .clear(clear && !run_request),.run_enable(run_permit),.fault(all_fault),
   .sample_tick(history_sample),
   .payload({front,velocities,trace_duties,trace_efforts[47:32],targets_q[47:32],
      history_flags,state,segment,command,level}),
   .read_index(history_index),.read_word(history_word),.count(history_count),.frozen(history_frozen));
 task123_task2_fix_telemetry #(.CLK_HZ(CLK_HZ),.BAUD(UART_BAUD),.FRAME_CYCLES(TELEMETRY_CYCLES),
 .SENSOR_COUNT(1),.ENCODER_REPORT(1),.ERROR_REPORT(1)) u_telemetry(
 .clk(sys_clk),.rst_n(rst_n),.front_mm(front),.left_mm(16'd0),.right_mm(16'd0),.sensor_fault(sensors),
 .motor_fault_code(motor_code),.speed_level(level),.route_state(display_state),.armed(MOTOR_ARMED!=0),
 .run_request(run_permit),.drive_enabled(stdby),.emergency_stop(safe_estop),.fault(all_fault),
 .positions(positions),.velocities(velocities),.encoder_errors(errors),.fault_wheels(fault_wheels),.imu_valid(imu_valid),.roll_q16(roll_q16),.pitch_q16(pitch_q16),.yaw_q16(yaw_q16),.route_diagnostic(route_diagnostic),
 .sample_error(sample_error),.fault_sample_error(fault_sample_error),
 .history_index(history_index),.history_count(history_count),.history_frozen(history_frozen),
 .history_word(history_word),.frame_done(telemetry_done),.tx(uart_tx));
 assign led={all_fault || sensors[0],stdby,run_request,rst_n};
 //==== 任务四观测口（直连内部 wire，无额外逻辑）====
 assign obs_front_mm   = control_sample_mm;   // 他们的控制用样本距离（mm）
 // ★2026-10-19 修（F2）：原来直连 control_valid（电平），下游做上升沿检测只能用一次；
 //   改为**每采样一拍的脉冲**（u_sensing 的 strobe），配合顶层的 dst_strobe 边沿使用。
 assign obs_front_vld  = strobe;
 assign obs_positions  = positions;
 assign obs_velocities = velocities;
endmodule
