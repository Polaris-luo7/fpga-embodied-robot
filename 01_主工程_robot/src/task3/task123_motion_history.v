// 64 records, one per 50ms in the board wrapper, plus start/fault records.
// The first stop/fault freezes the ring until stopped clear or FPGA reset.
// The observer never drives the motor control path; RAM need not be reset.
module task123_motion_history(
 input wire clk,rst_n,clear,run_enable,fault,sample_tick,
 input wire [179:0] payload,
 input wire [5:0] read_index,
 output wire [199:0] read_word,
 output reg [6:0] count,
 output reg frozen
);
 reg [199:0] memory[0:63];
 reg [5:0] write_index;
 reg [15:0] sequence_number;
 reg recording;
 wire stopping=recording && (fault || !run_enable);
 wire take_sample=rst_n && !clear && !frozen &&
    ((run_enable && !recording) || (recording && sample_tick) || stopping);
 always @(posedge clk)
  if(take_sample)memory[write_index]<={4'd0,sequence_number,payload};
 assign read_word=(count==64 || {1'b0,read_index}<count)?memory[read_index]:200'd0;
 always @(posedge clk or negedge rst_n)
 if(!rst_n)begin write_index<=0;sequence_number<=0;count<=0;frozen<=0;recording<=0;end
 else if(clear)begin write_index<=0;sequence_number<=0;count<=0;frozen<=0;recording<=0;end
 else if(!frozen)begin
  if(run_enable && !recording)recording<=1;
  if(take_sample)begin
   write_index<=write_index+1'b1;sequence_number<=sequence_number+1'b1;
   if(count<64)count<=count+1'b1;
  end
  if(stopping)begin frozen<=1;recording<=0;end
 end
endmodule
