// Exact, no-argument preset: $FIND (find red bottle), $VSTOP.
// Existing bt_cmd2 and all baseline commands remain byte-for-byte unchanged.
module vision_command(input wire clk,rst_n,frame_vld,
 input wire [255:0] frame,input wire [5:0] frame_len,
 output reg find_pulse,stop_pulse);
 always @(posedge clk or negedge rst_n)
 if(!rst_n)begin find_pulse<=0;stop_pulse<=0;end
 else begin
  find_pulse<=frame_vld && frame_len==5 && frame[39:0]==40'h444e494624;
  stop_pulse<=frame_vld && frame_len==6 && frame[47:0]==48'h504f54535624;
 end
endmodule
