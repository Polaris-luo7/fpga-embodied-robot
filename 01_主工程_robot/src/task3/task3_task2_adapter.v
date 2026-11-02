// Task-three command contract -> task-two logical wheel targets.
// This does not instantiate or edit selftest_ir. All ports use the same clock.
// STOP requires backend_run=0: task-two action 0 alone means FORWARD.
module task3_task2_adapter #(
    parameter integer MAX_SPEED=100
)(
    input wire clk,rst_n,run_enable,emergency_stop,clear_fault,
    input wire backend_ready,backend_fault,
    input wire [2:0] command,
    input wire [15:0] speed,
    output reg backend_run,
    output reg [2:0] backend_action,
    output reg [63:0] wheel_targets,
    output reg fault
);
    wire illegal = command>4 ||
                   (command!=0 && (speed==0 || speed>MAX_SPEED || speed>32767));
    wire reject = run_enable && (illegal || !backend_ready || backend_fault);
    wire drive = rst_n && run_enable && !emergency_stop && !fault && !reject;
    wire signed [15:0] positive_speed=$signed(speed);
    wire signed [15:0] negative_speed=-positive_speed;
    always @(posedge clk or negedge rst_n)
        if(!rst_n) fault<=0;
        else if(emergency_stop || reject) fault<=1;
        else if(clear_fault && !run_enable && !backend_fault) fault<=0;
    always @(*) begin
        backend_run=0;backend_action=0;wheel_targets=0;
        if(drive && command!=0) begin
            backend_run=1;
            case(command)
                1: begin backend_action=0;wheel_targets={4{positive_speed}};end
                2: begin backend_action=1;wheel_targets={4{negative_speed}};end
                3: begin backend_action=2;
                    wheel_targets={positive_speed,negative_speed,positive_speed,negative_speed};end
                4: begin backend_action=3;
                    wheel_targets={negative_speed,positive_speed,negative_speed,positive_speed};end
                default: begin backend_run=0;backend_action=0;wheel_targets=0;end
            endcase
        end
    end
endmodule
