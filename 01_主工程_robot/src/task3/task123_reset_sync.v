module task123_reset_sync(input wire clk,arst_n,output wire rst_n);
 reg [1:0] sync;
 always @(posedge clk or negedge arst_n)if(!arst_n)sync<=0;else sync<={sync[0],1'b1};
 assign rst_n=sync[1];
endmodule
