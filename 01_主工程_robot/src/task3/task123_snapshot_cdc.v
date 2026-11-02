// Acknowledged bundled-data mailbox. Source holds the whole snapshot until ack.
module task123_snapshot_cdc #(parameter integer WIDTH=97,STALE_CYCLES=15000000)(
 input wire src_clk,dst_clk,rst_n,input wire src_strobe,input wire [WIDTH-1:0] src_data,
 output reg [WIDTH-1:0] dst_data,output reg dst_strobe,output wire fresh);
 reg [WIDTH-1:0] hold_data;reg request,ack;reg ack1,ack2,req1,req2,req3;
 reg [31:0] age;reg seen;
 assign fresh=seen && age<STALE_CYCLES;
 always @(posedge src_clk or negedge rst_n)
 if(!rst_n)begin request<=0;hold_data<=0;ack1<=0;ack2<=0;end
 else begin
 ack1<=ack;ack2<=ack1;
 if(src_strobe && request==ack2)begin hold_data<=src_data;request<=~request;end
 end
 always @(posedge dst_clk or negedge rst_n)
 if(!rst_n)begin req1<=0;req2<=0;req3<=0;ack<=0;dst_data<=0;dst_strobe<=0;age<=0;seen<=0;end
 else begin
 req1<=request;req2<=req1;req3<=req2;dst_strobe<=0;
 if(age<STALE_CYCLES)age<=age+1;
 if(req3!=ack)begin dst_data<=hold_data;dst_strobe<=1;ack<=req3;age<=0;seen<=1;end
 end
endmodule
