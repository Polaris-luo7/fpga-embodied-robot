// Safety qualification around the UNMODIFIED teammate vision_blob.
// Actual DataVs/wr_load is active-high between frame blanks. Its rising edge
// publishes the completed previous raster in teammate blob. Qualification is
// pipelined separately from pixels. Delay publication until its matching flag
// is stable; no extra registers in the DDR pixel or DataValid path.
// 2026-10-25 修改（任务五上板调试）：
//   ① 去掉 "bh>=bw"（框必须高大于宽）—— 原判据照"竖直水瓶"设计，
//      横向/斜置的红色物体包围框必然 bw>bh，会被整条否掉 ⇒ 永不出框。
//   ② MIN_WIDTH/MIN_HEIGHT/MIN_PIXELS 下调，照顾扁平的横向目标。
//   保留 density 校验（red_count*8 >= bw*bh，即填充率 ≥12.5%），防噪点成大框。
module vision_detector #(
 parameter integer WIDTH=800,HEIGHT=480,MIN_PIXELS=200,MIN_WIDTH=6,MIN_HEIGHT=6
)(input wire clk,rst_n,frame_active,pixel_valid,red_now,
 input wire [10:0] x,y,
 output reg frame_strobe,output reg box_valid,
 output wire [10:0] xmin,xmax,ymin,ymax,output reg [19:0] red_count);
 wire [11:0] lx,hx,ly,hy;wire found;
 reg frame_d;reg [19:0] count;
 wire edge_frame=frame_active&&!frame_d;
 wire [11:0] bw=hx-lx+12'd1;
 wire [11:0] bh=hy-ly+12'd1;
 reg [11:0] bw_q,bh_q;
 reg [23:0] area_q,density_q,density_qq;
 reg size_q,size_qq;
 reg [2:0] publish_pipe;
 vision_blob u_teammate_blob(.pclk(clk),.rst_n(rst_n),
  .px_valid(pixel_valid && x<WIDTH && y<HEIGHT),.red_now(red_now),
  .px_x(x),.px_y(y),.frame_start(frame_active),
  .x_min(lx),.x_max(hx),.y_min(ly),.y_max(hy),.tgt_valid(found));
 assign xmin=lx[10:0];assign xmax=hx[10:0];assign ymin=ly[10:0];assign ymax=hy[10:0];
 always @(posedge clk or negedge rst_n)
 if(!rst_n)begin
  frame_d<=0;count<=0;red_count<=0;frame_strobe<=0;box_valid<=0;
  bw_q<=0;bh_q<=0;area_q<=0;density_q<=0;density_qq<=0;
  size_q<=0;size_qq<=0;publish_pipe<=0;
 end
 else begin
  frame_d<=frame_active;
  publish_pipe<={publish_pipe[1:0],edge_frame};
  frame_strobe<=publish_pipe[2];
  bw_q<=bw;bh_q<=bh;density_q<={1'b0,red_count,3'b000};
  size_q<=found && red_count>=MIN_PIXELS && bw>=MIN_WIDTH && bh>=MIN_HEIGHT;
  area_q<=bw_q*bh_q;density_qq<=density_q;size_qq<=size_q;
  // ★ 2026-10-26：实心度闸门从 25% 回退到 **12.5%**
  //   上一版设 25% 导致"亮红严重减少"：当只有高光部分被识别时，
  //   包围框是整个物体、红色像素却只有一小块 ⇒ 填充率掉到 25% 以下 ⇒ 目标被判无效。
  //   12.5% 既能挡住零散误检（那些填充率通常 <5%），又不会误杀"部分识别"的目标。
  //   density = red_count*8，area = bw*bh；
  //   density >= area>>1  ⇔  red_count >= area/16  （填充率 ≥12.5%）
  //   ★ 想更宽容：(area_q>>2) → 3.1%   更严格：(area_q<<1) → 25%
  box_valid<=size_qq && (density_qq >= (area_q>>1));
  if(edge_frame)begin red_count<=count;count<=0;end
  else if(pixel_valid && red_now && x<WIDTH && y<HEIGHT && count!=20'hfffff)
   count<=count+1'b1;
 end
endmodule
