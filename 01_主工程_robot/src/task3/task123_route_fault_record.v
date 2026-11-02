// Read-only observation: preserve the first route fault until stopped S0/reset.
// Route fault becomes visible one clock after its decision. Sample the inputs
// continuously so the saved state/progress belongs to the decision, not FAULT.
module task123_route_fault_record(
 input wire clk,rst_n,clear,fault,
 input wire [2:0] route_code,segment,
 input wire [3:0] route_state,
 input wire [7:0] causes,
 input wire [15:0] front_mm,
 input wire [31:0] progress,
 output reg [66:0] record
);
 reg [62:0] previous_context;
 always @(posedge clk or negedge rst_n)
 if(!rst_n)begin record<=0;previous_context<=0;end
 else begin
  previous_context<={route_state,segment,causes,front_mm,progress};
  if(clear)record<=0;
  else if(fault && !record[66])record<={1'b1,route_code,previous_context};
 end
endmodule
