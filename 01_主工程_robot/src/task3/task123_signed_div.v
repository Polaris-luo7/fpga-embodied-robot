// Exact signed constant division, truncates toward zero; 32 clocks.
module task123_signed_div #(parameter integer DIVISOR=100)(
 input wire clk,rst_n,start,input wire signed [31:0] dividend,
 output reg signed [31:0] quotient,output reg busy,done);
 localparam integer RW=$clog2(DIVISOR)+1;
 reg [31:0] work;reg [RW-1:0] remainder;reg negative;reg [5:0] bit_count;
 wire [RW:0] trial={remainder,work[31]};
 wire take_bit=trial>=DIVISOR;
 wire [31:0] next_work={work[30:0],take_bit};
 always @(posedge clk or negedge rst_n)
 if(!rst_n)begin work<=0;remainder<=0;negative<=0;bit_count<=0;quotient<=0;busy<=0;done<=0;end
 else begin
 done<=0;
 if(start && !busy)begin
 work<=dividend[31] ? -dividend:dividend;negative<=dividend[31];remainder<=0;bit_count<=0;busy<=1;
 end else if(busy)begin
 remainder<=take_bit ? trial-DIVISOR:trial;work<=next_work;
 if(bit_count==31)begin quotient<=negative ? -$signed(next_work):$signed(next_work);busy<=0;done<=1;end
 else bit_count<=bit_count+1;
 end end
endmodule
