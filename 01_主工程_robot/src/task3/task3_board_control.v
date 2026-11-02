// Active-low board buttons: S2 toggle run, S3 emergency, S0 stopped clear.
// Emergency is latched independently of debounce; no automatic restart on release.
module task3_board_control #(parameter integer DEBOUNCE_CYCLES=1_000_000)(
    input wire clk,rst_n,
    input wire [3:0] key,
    input wire fault,
    output reg run_request,
    output wire emergency_stop,
    output reg clear_fault
);
    reg [3:0] meta,sync,stable,previous;
    reg [31:0] count;
    reg estop_latched,start_armed;
    reg [2:0] clear_count;
    wire start_press=previous[2] && !stable[2];
    wire clear_press=previous[0] && !stable[0];
    assign emergency_stop=estop_latched || !key[3];
    always @(posedge clk or negedge rst_n)
        if(!rst_n) begin
            meta<=15;sync<=15;stable<=15;previous<=15;count<=0;
            run_request<=0;estop_latched<=0;start_armed<=0;clear_fault<=0;clear_count<=0;
        end else begin
            meta<=key;sync<=meta;previous<=stable;
            if(sync==stable) count<=0;
            else if(count>=DEBOUNCE_CYCLES-1) begin stable<=sync;count<=0;end
            else count<=count+1'b1;
            if(key[2] && stable[2]) start_armed<=1;
            if(clear_count!=0) begin clear_count<=clear_count-1'b1;clear_fault<=1;end
            else clear_fault<=0;
            if(!key[3]) begin estop_latched<=1;run_request<=0;start_armed<=0;end
            else if(clear_press && !run_request && stable[3]) begin
                estop_latched<=0;clear_count<=4;clear_fault<=1;start_armed<=0;
            end else if(fault || estop_latched || clear_count!=0 || clear_fault) run_request<=0;
            else if(start_press && start_armed) begin run_request<=!run_request;start_armed<=0;end
        end
endmodule
