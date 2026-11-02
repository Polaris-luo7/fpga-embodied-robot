// A bounded manual session; expiration latches until a stopped explicit clear.
// MAX_RUN_CYCLES=0 preserves the previous unlimited-session behavior.
module task3_run_limit #(parameter integer MAX_RUN_CYCLES=0)(
    input wire clk,rst_n,run_request,clear_fault,
    output wire run_permit,timeout_stop
);
generate if(MAX_RUN_CYCLES==0) begin: unlimited
    assign run_permit=rst_n && run_request;
    assign timeout_stop=1'b0;
end else begin: bounded
    reg [31:0] elapsed;
    reg expired;
    assign timeout_stop=expired;
    assign run_permit=rst_n && run_request && !expired;
    always @(posedge clk or negedge rst_n)
        if(!rst_n) begin elapsed<=0;expired<=0;end
        else if(clear_fault && !run_request) begin elapsed<=0;expired<=0;end
        else begin
            if(!run_request) elapsed<=0;
            else if(!expired) begin
                if(elapsed>=MAX_RUN_CYCLES-1) expired<=1;
                else elapsed<=elapsed+1'b1;
            end
        end
end endgenerate
endmodule
