// Diagnostic only: S1 enables sonar after external power is on; S0 disables it.
// Enable is applied while raw Trig is low so the first pulse is not truncated.
module task123_sonar_manual_gate #(
    parameter integer DEBOUNCE_CYCLES=1000000
)(
    input wire clk,rst_n,enable_n,disable_n,raw_trig,
    output wire gated_trig,
    output reg enabled
);
    reg meta,sync,stable,previous,start_armed,requested;
    reg [31:0] age;
    wire press=previous && !stable;
    assign gated_trig=rst_n && disable_n && enabled && raw_trig;
    always @(posedge clk or negedge rst_n)
    if(!rst_n)begin
        meta<=1;sync<=1;stable<=1;previous<=1;age<=0;
        start_armed<=0;requested<=0;enabled<=0;
    end else begin
        meta<=enable_n;sync<=meta;previous<=stable;
        if(sync==stable)age<=0;
        else if(age>=DEBOUNCE_CYCLES-1)begin stable<=sync;age<=0;end
        else age<=age+1'b1;
        if(enable_n && stable)start_armed<=1;
        if(!disable_n)begin requested<=0;enabled<=0;start_armed<=0;end
        else begin
            if(press && start_armed)begin requested<=1;start_armed<=0;end
            if(!requested)enabled<=0;
            else if(!raw_trig)enabled<=1;
        end
    end
endmodule
