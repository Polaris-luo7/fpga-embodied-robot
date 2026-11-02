// Private vehicle PI: same Q8 arithmetic and anti-windup as preserved task2.
// One update completes six clocks after en; the 10ms speed window is unchanged.
// clr cancels every pending stage and clears output immediately at the clock.
module task123_speedstep_pi #(parameter signed [15:0] UMAX=4000,UFF=2400,
 parameter integer SPEED_FEEDFORWARD=0,FF_PER_COUNT=200)(
 input wire clk,rst_n,en,clr,
 input wire signed [15:0] kp_q,ki_q,target,actual,
 output reg signed [15:0] u);
 reg [2:0] phase;
 reg signed [15:0] e_q,ff_q,kp_qr,ki_qr;
 reg signed [15:0] previous_target;
 reg signed [31:0] pw,iw,acc,acc_next,partial,raw;
 // Optional calibration mode. Legacy constant feedforward remains the default.
 wire signed [31:0] scaled_ff=$signed(target)*FF_PER_COUNT;
 wire signed [15:0] speed_ff=scaled_ff>UFF ? UFF :
                             scaled_ff< -UFF ? -UFF : scaled_ff[15:0];
 wire signed [31:0] bounded_acc=acc_next>12000?32'sd12000:
                               acc_next< -12000?-32'sd12000:acc_next;
 wire signed [31:0] bounded_u=raw>UMAX?{{16{UMAX[15]}},UMAX}:
                              raw< -UMAX?-{{16{UMAX[15]}},UMAX}:raw;
 wire saturated=(raw!=bounded_u) && ((e_q>0)==(raw>0));
 always @(posedge clk or negedge rst_n)
 if(!rst_n)begin phase<=0;e_q<=0;ff_q<=0;kp_qr<=0;ki_qr<=0;
 previous_target<=0;pw<=0;iw<=0;acc<=0;acc_next<=0;partial<=0;raw<=0;u<=0;end
 else if(clr)begin phase<=0;acc<=0;u<=0;previous_target<=0;end
 else case(phase)
 0:if(en)begin e_q<=target-actual;
 // A new setpoint starts with its own feedforward and zero integral.
 // Avoid carrying a negative high-speed correction into the slow target.
 // No startup burst refill, PWM limit increase or stall delay change.
 if(target!=previous_target)acc<=0;
 previous_target<=target;
 ff_q<=SPEED_FEEDFORWARD!=0 ? speed_ff : target==0?16'sd0:target>0?UFF:-UFF;
 kp_qr<=kp_q;ki_qr<=ki_q;phase<=1;end
 1:begin pw<=($signed(e_q)*$signed(kp_qr))>>>8;
 iw<=($signed(e_q)*$signed(ki_qr))>>>8;phase<=2;end
 2:begin acc_next<=acc+iw;phase<=3;end
 3:begin partial<=pw+acc_next;phase<=4;end
 4:begin raw<=partial+ff_q;phase<=5;end
 5:begin acc<=saturated?acc:bounded_acc;u<=bounded_u[15:0];phase<=0;end
 default:phase<=0;
 endcase
endmodule
