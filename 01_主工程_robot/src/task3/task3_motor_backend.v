// Separate task-three backend derived from preserved task3_motion.
// Gains follow the completed task-two handoff; physical calibration pending.
// Real encoder feedback only. Units: quadrature counts per SPEED_WIN clocks.
// Polarity defaults reproduce selftest_ir; hardware calibration is still needed.
module task3_motor_backend #(
    parameter integer CLK_HZ = 50_000_000,
    parameter integer SPEED_WIN = CLK_HZ/100,
    parameter integer STALL_TICKS = 200,
    parameter integer WRONG_WAY_TICKS = 10,
    parameter integer SETTLE_TICKS = 3,
    parameter integer REVERSE_TICKS = 5,
    parameter integer MAX_SPEED = 100,
    parameter integer MAX_PWM_DUTY = 4095,
    parameter integer STARTUP_CYCLES=0,STARTUP_DUTY=2400,
    parameter integer SPEED_FEEDFORWARD=0,FF_PER_COUNT=200,
    parameter signed [15:0] KP_Q=1500, KI_Q=130,
    parameter [3:0] ENCODER_SWAP = 4'b1010,
    parameter [3:0] MOTOR_INVERT = 4'b1111
)(
    input wire clk, rst_n, enable, emergency_stop, clear_fault,
    input wire [2:0] backend_action, // 0=forward,1=reverse,2=left,3=right
    input wire [63:0] wheel_targets,
    input wire [3:0] enc_a, enc_b,
    output wire [3:0] m_in1, m_in2, m_pwm,
    output wire stdby,
    output wire [127:0] positions, // wheel i = bits [32*i +: 32]
    output wire [63:0] velocities, // wheel i = bits [16*i +: 16]
    output wire [31:0] encoder_errors,
    output reg [3:0] fault_wheels,
    output wire feedback_tick,
    output reg stationary,
    output reg fault,
    output reg [2:0] fault_code // 1=estop,2=stall,3=encoder,4=command,5=wrong-way
);
    wire [2:0] command=enable ? backend_action+3'd1 : 3'd0;
    wire signed [15:0] first_target=$signed(wheel_targets[15:0]);
    wire [15:0] speed=first_target[15] ? -first_target : first_target;
    wire [3:0] target_valid;
    wire [3:0] fixed_a = (enc_a & ~ENCODER_SWAP) | (enc_b & ENCODER_SWAP);
    wire [3:0] fixed_b = (enc_b & ~ENCODER_SWAP) | (enc_a & ENCODER_SWAP);
    wire [3:0] ticks;
    wire [31:0] errors;
    assign encoder_errors=errors;
    reg [31:0] errors_d;
    reg [31:0] stalled [0:3];
    reg [31:0] wrong_way [0:3];
    reg [31:0] settled;
    reg [2:0] last_command;
    reg [31:0] reverse_wait;
    reg [11:0] carrier;
    reg [31:0] startup_remaining;
    wire valid_command = backend_action<=3 && speed>0 && speed<=MAX_SPEED && &target_valid;
    wire requested = enable;
    wire active = rst_n && requested && valid_command && !emergency_stop && !fault && reverse_wait == 0;
    wire command_changed = command != last_command;
    wire drive_ok = active && !command_changed;
    wire startup_active = STARTUP_CYCLES!=0 && startup_remaining!=0 && drive_ok;
    localparam signed [15:0] STARTUP_EFFORT=STARTUP_DUTY[15:0];
    wire [3:0] quiet;
    wire [3:0] stall_expired;
    wire [3:0] wrong_direction,wrong_expired;
    wire [3:0] error_changed;
    assign feedback_tick = ticks[0];
    assign stdby = drive_ok;
    integer j;

    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin
            carrier<=0; fault<=0; fault_code<=0; errors_d<=0;
            fault_wheels<=0;
            stationary<=0; settled<=0; last_command<=0; reverse_wait<=0;
            startup_remaining<=STARTUP_CYCLES;
            for(j=0;j<4;j=j+1) begin stalled[j]<=0;wrong_way[j]<=0;end
        end else begin
            carrier <= carrier+1'b1;
            // One burst budget per reset/stopped clear, never refilled by a
            // speed-level change or a temporary loss of the drive gate.
            if(clear_fault && !enable) startup_remaining<=STARTUP_CYCLES;
            else if(startup_active) startup_remaining<=startup_remaining-1'b1;
            last_command <= command;
            // Blank any motion mode change for 50ms by default, clear PI below.
            // Add one tick so arbitrary command phase still gets at least
            // REVERSE_TICKS complete feedback periods (50ms at default settings).
            if (command_changed && command != 0) reverse_wait <= REVERSE_TICKS+1;
            else if (feedback_tick && reverse_wait != 0) reverse_wait <= reverse_wait-1'b1;
            if (emergency_stop) begin fault<=1;fault_code<=1;fault_wheels<=0;end
            else if (clear_fault && !enable && command==0) begin
                fault<=0;fault_code<=0;errors_d<=errors;fault_wheels<=0;
            end else if (!fault) begin
                if (requested && !valid_command) begin fault<=1;fault_code<=4;fault_wheels<=0;end
                else if (drive_ok && |stall_expired) begin fault<=1;fault_code<=2;fault_wheels<=stall_expired;end
                else if (requested && errors != errors_d) begin fault<=1;fault_code<=3;fault_wheels<=error_changed;end
                else if (drive_ok && |wrong_expired) begin fault<=1;fault_code<=5;fault_wheels<=wrong_expired;end
            end
            if (feedback_tick) begin
                errors_d <= errors;
                if (&quiet) begin
                    if (settled < SETTLE_TICKS) settled<=settled+1'b1;
                    if (settled >= SETTLE_TICKS-1) stationary<=1;
                end else begin settled<=0;stationary<=0;end
                for(j=0;j<4;j=j+1) begin
                    if (!drive_ok || !quiet[j]) stalled[j]<=0;
                    else if (stalled[j]<STALL_TICKS) stalled[j]<=stalled[j]+1'b1;
                    if (!drive_ok || !wrong_direction[j]) wrong_way[j]<=0;
                    else if (wrong_way[j]<WRONG_WAY_TICKS) wrong_way[j]<=wrong_way[j]+1'b1;
                end
            end
        end

    genvar i;
    generate for(i=0;i<4;i=i+1) begin: wheels
        wire signed [31:0] position;
        wire signed [15:0] velocity;
        wire [7:0] error_count;
        wire negative = command==2 || (command==3 && (i==0 || i==2)) ||
                                      (command==4 && (i==1 || i==3));
        wire signed [15:0] input_target=$signed(wheel_targets[16*i +: 16]);
        wire signed [15:0] expected_target=negative ? -$signed(speed) : $signed(speed);
        assign target_valid[i]=input_target==expected_target;
        wire signed [15:0] target = drive_ok ? input_target : 16'sd0;
        wire signed [15:0] pi_effort;
        wire signed [15:0] effort=startup_active ?
            (target[15] ? -STARTUP_EFFORT : STARTUP_EFFORT) : pi_effort;
        // Do not reverse a wheel to correct positive-speed overshoot.
        wire effort_matches = (effort[15] == target[15]) && target != 0;
        wire signed [15:0] motor_command = MOTOR_INVERT[i] ? -effort : effort;
        wire [15:0] magnitude = motor_command[15] ? -motor_command : motor_command;
        wire [11:0] applied_duty = magnitude > MAX_PWM_DUTY ? MAX_PWM_DUTY[11:0] : magnitude[11:0];
        wire wheel_on = drive_ok && effort_matches && magnitude >= 80 && applied_duty != 0;
        assign positions[32*i +: 32] = position;
        assign velocities[16*i +: 16] = velocity;
        assign errors[8*i +: 8] = error_count;
        assign error_changed[i]=errors[8*i +: 8]!=errors_d[8*i +: 8];
        assign quiet[i] = velocity >= -16'sd1 && velocity <= 16'sd1;
        assign stall_expired[i] = stalled[i]>=STALL_TICKS;
        assign wrong_direction[i] = negative ? velocity>16'sd1 : velocity < -16'sd1;
        assign wrong_expired[i] = wrong_way[i]>=WRONG_WAY_TICKS;
        task3_quad_dec u_encoder(.clk(clk),.rst_n(rst_n),.a(fixed_a[i]),.b(fixed_b[i]),
            .clr(1'b0),.pos(position),.err_cnt(error_count));
        task3_speed_meter #(.WIN(SPEED_WIN)) u_speed(.clk(clk),.rst_n(rst_n),
            .pos(position),.spd(velocity),.tick(ticks[i]));
        task3_vehicle_pi #(.UMAX(MAX_PWM_DUTY < 4000 ? MAX_PWM_DUTY : 4000),
            .SPEED_FEEDFORWARD(SPEED_FEEDFORWARD),.FF_PER_COUNT(FF_PER_COUNT)) u_pi(.clk(clk),.rst_n(rst_n),.en(feedback_tick && drive_ok),
            .clr(!drive_ok),.kp_q(KP_Q),.ki_q(KI_Q),
            .target(target),.actual(velocity),.u(pi_effort));
        assign m_in1[i] = wheel_on && !motor_command[15];
        assign m_in2[i] = wheel_on && motor_command[15];
        assign m_pwm[i] = wheel_on && carrier < applied_duty;
    end endgenerate
endmodule
