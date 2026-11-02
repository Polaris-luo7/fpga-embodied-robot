// Coherent AB qualification after two-stage synchronization.
// A persistent illegal two-bit transition is deliberately passed to the decoder.
module task3_ab_filter #(parameter integer STABLE_CYCLES=16)(
 input wire clk,rst_n,input wire [3:0] a,b,output wire [3:0] a_out,b_out);
 genvar i;
 generate for(i=0;i<4;i=i+1) begin: channels
 reg [1:0] a_sync,b_sync,candidate,accepted;
 reg [31:0] stable_count;
 wire [1:0] sample={a_sync[1],b_sync[1]};
 assign a_out[i]=accepted[1]; assign b_out[i]=accepted[0];
 always @(posedge clk or negedge rst_n)
 if(!rst_n) begin a_sync<=0;b_sync<=0;candidate<=0;accepted<=0;stable_count<=0;end
 else begin
 a_sync<={a_sync[0],a[i]};b_sync<={b_sync[0],b[i]};
 if(STABLE_CYCLES<=1) begin accepted<=sample;candidate<=sample;stable_count<=0;end
 else if(sample!=candidate) begin candidate<=sample;stable_count<=1;end
 else if(stable_count<STABLE_CYCLES) begin
 stable_count<=stable_count+1;
 if(stable_count==STABLE_CYCLES-1) accepted<=candidate;
 end
 end
 end endgenerate
endmodule
