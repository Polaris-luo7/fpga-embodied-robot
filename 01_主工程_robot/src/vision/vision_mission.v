// Deterministic FPGA preset. Motion units: mm/s and deg/s Q16.16.
// No automatic restart after arrival/fault. New FIND is always required.
module vision_mission #(
 parameter integer TICK_DIV=500000, FRAME_TIMEOUT=25, SONAR_TIMEOUT=35,
 parameter integer SEARCH_TICKS=2000, MISSION_TICKS=6000,
 parameter integer CENTER_X=400, CENTER_BAND=90, LOCK_FRAMES=3,
 parameter integer EDGE_MARGIN=40,
 parameter integer MIN_BOX_W=80, MIN_BOX_H=50,
 parameter integer STOP_MM=200, HARD_STOP_MM=140,
 parameter integer FAR_SPEED=200, NEAR_SPEED=160, SEARCH_RATE=64,
 parameter integer ALIGN_RATE=40,
 parameter integer HIT_FRAMES=3, LOST_FRAMES=20,
 parameter integer SEARCH_ON=35, SEARCH_OFF=20, LOSE_ARRIVE_MM=500,
 parameter integer TURN_SIGN=1
)(input wire clk,rst_n,start,stop,estop,permit,
 input wire frame_strobe,box_valid,input wire [10:0] xmin,xmax,ymin,ymax,
 input wire sonar_strobe,input wire [15:0] distance_mm,
 output reg selected,output wire active,output wire drive_enable,
 output reg [3:0] state,reason,
 output reg signed [31:0] vx,w);
 localparam IDLE=0,SEARCH=1,ALIGN=2,APPROACH=3,ARRIVED=4,FAULT=5;
 reg [31:0] div;
 wire tick=div==TICK_DIV-1;
 reg [15:0] frame_age,sonar_age,elapsed,search_elapsed;
 reg seen_frame,seen_sonar,target;
 reg [11:0] cx;
 reg [15:0] dist;
 reg [7:0] locks,near_hits;
 reg [2:0] hit_cnt;
 reg [4:0] miss_cnt;
 /* 搜索的"转—停"相位计数：SEARCH 不再连续转，而是一个周期内转 SEARCH_ON 拍、停 SEARCH_OFF 拍。
    停的那段画面静止，没有运动模糊，检测器每帧都拿到干净图像 ⇒ 命中更稳；
    同时平均转速降到 ON/(ON+OFF)，比直接降 SEARCH_RATE 有效
    （开环驱动有个 27.5% 占空比地板，命令值降下去实际转速并不会变）。 */
 reg [5:0] srch_ph;
 wire centered=(cx>=CENTER_X-CENTER_BAND && cx<=CENTER_X+CENTER_BAND);
 wire incoming_centered=({1'b0,xmin}+{1'b0,xmax})>=2*(CENTER_X-CENTER_BAND) &&
                        ({1'b0,xmin}+{1'b0,xmax})<=2*(CENTER_X+CENTER_BAND);
 /* ★2026-10-29 入框判据：目标要"完整出现在画面里"才允许从对准切到接近。
    背景：`incoming_centered` 只看包围框**中点**，板子被画面边缘裁掉一半时
    （例如 xmin=0 / xmax=400）中点仍算得出 400、照样判"居中"，于是车会朝一个
    只看得到一半的目标开过去 —— 这是路径向左偏的成因之一。
      · fully_in  : 包围框左右都留出 EDGE_MARGIN 余量 ⇒ 完整入框
      · span_wide : 左右都贴边 ⇒ 板子已经占满整幅宽度（贴很近时的正常现象，
                    不这样放行的话近距离会永远进不了 APPROACH，一直转到 R7）
    单侧贴边（只占左半边或右半边）两条都不成立 ⇒ 拒绝起步，先去转正。 */
 wire fully_in  = (xmin >  EDGE_MARGIN)                && (xmax <  2*CENTER_X - EDGE_MARGIN);
 wire span_wide = (xmin <= EDGE_MARGIN)                && (xmax >= 2*CENTER_X - EDGE_MARGIN);
 /* ★2026-10-29 尺寸闸门：板子必须在画面里占够大的一块，才允许起步靠近。
    上一版只有左右入框判据、没有任何尺寸要求，导致板子"刚露一条边"（几十像素宽）
    只要包围框没碰到左右边框就会被判成可以接近。这里宽度和高度都要达标：
    演示距离 1.5m 时板子约 185×150 像素，门槛取 80×50（约 40%），正常起步有 2 倍余量。 */
 wire [11:0] box_w = ({1'b0,xmax} - {1'b0,xmin});
 wire [11:0] box_h = ({1'b0,ymax} - {1'b0,ymin});
 wire big_enough = (box_w >= MIN_BOX_W) && (box_h >= MIN_BOX_H);
 wire aim_ok    = (fully_in || span_wide) && big_enough;
 assign active=(state==SEARCH || state==ALIGN || state==APPROACH);
 /* 2026-10-28：贴脸判定分流。原来只要超声 ≤HARD_STOP_MM 一律 FAULT（R6），
    可 APPROACH 阶段本来就是要贴上去，目标有效时该判 ARRIVED 而不是故障。
    只有"目标无效却贴脸"才当障碍物急停。 */
 wire hard_hit       = sonar_strobe && distance_mm<=HARD_STOP_MM;
 wire arrive_by_prox = hard_hit && target && (state==APPROACH);
 // Independent combinational kill, before the 10ms control tick.
 assign drive_enable=selected && active && permit && !stop && !estop &&
  seen_frame && seen_sonar && frame_age<FRAME_TIMEOUT && sonar_age<SONAR_TIMEOUT &&
  dist>HARD_STOP_MM && (state==SEARCH || target);
 always @(posedge clk or negedge rst_n)
 if(!rst_n)begin
  div<=0;selected<=0;state<=IDLE;reason<=0;vx<=0;w<=0;
  frame_age<=FRAME_TIMEOUT;sonar_age<=SONAR_TIMEOUT;elapsed<=0;search_elapsed<=0;
  seen_frame<=0;seen_sonar<=0;target<=0;cx<=0;dist<=0;locks<=0;near_hits<=0;
  hit_cnt<=0;miss_cnt<=0;srch_ph<=6'd0;
 end else begin
  if(tick)div<=0;else div<=div+1'b1;
  if(!active) srch_ph<=6'd0;
  else if(tick) srch_ph <= (srch_ph >= SEARCH_ON+SEARCH_OFF-1) ? 6'd0 : srch_ph+6'd1;
  if(tick)begin
   if(frame_age<FRAME_TIMEOUT)frame_age<=frame_age+1'b1;
   if(sonar_age<SONAR_TIMEOUT)sonar_age<=sonar_age+1'b1;
   if(active && elapsed<MISSION_TICKS)elapsed<=elapsed+1'b1;
   if(state==SEARCH && search_elapsed<SEARCH_TICKS)search_elapsed<=search_elapsed+1'b1;
  end
  if(frame_strobe)begin
   seen_frame<=1;frame_age<=0;target<=box_valid;
   cx<=({1'b0,xmin}+{1'b0,xmax})>>1;
  end
  if(sonar_strobe)begin
   seen_sonar<=1;sonar_age<=0;dist<=distance_mm;
  end
  // Priority: safety > explicit start > samples > control tick.
  if(stop || estop || (selected && !permit))begin
   vx<=0;w<=0;locks<=0;near_hits<=0;
   if(selected || start)begin state<=FAULT;reason<=estop ? 4'd1:4'd2;end
  end else if(start && permit && !active)begin
   selected<=1;state<=SEARCH;reason<=0;vx<=0;w<=0;
   elapsed<=0;search_elapsed<=0;locks<=0;near_hits<=0;hit_cnt<=0;miss_cnt<=0;
   // Require NEW post-command camera and sonar samples before any motion.
   seen_frame<=0;seen_sonar<=0;frame_age<=0;sonar_age<=0;target<=0;
  end else if(active)begin
   /* 2026-10-28：reason 细分，便于上板定位（原实现所有超时都报 3，看不出是哪一路）。
      3=画面不新鲜 5=超声不新鲜 6=距离≤HARD_STOP 7=搜索超时 8=任务总超时
      另：原式 (seen && age>=T) || (!seen && age>=T) 恒等于 age>=T，这里直接化简。 */
   if(frame_age>=FRAME_TIMEOUT || sonar_age>=SONAR_TIMEOUT ||
      elapsed>=MISSION_TICKS || search_elapsed>=SEARCH_TICKS ||
      (hard_hit && !arrive_by_prox))begin
    state<=FAULT;vx<=0;w<=0;
    reason <= (frame_age>=FRAME_TIMEOUT)      ? 4'd3 :
              (sonar_age>=SONAR_TIMEOUT)      ? 4'd5 :
              (hard_hit && !arrive_by_prox)   ? 4'd6 :
              (search_elapsed>=SEARCH_TICKS)  ? 4'd7 : 4'd8;
   end else begin
    if(frame_strobe)begin
    /* 2026-10-28：原来"跟踪中丢一帧就 FAULT"，实测一次误检（背景红物单帧命中）就把整个
       任务打掉（上板读回 R4）。改为两级容忍：
         ① 进 ALIGN 需要连续 HIT_FRAMES 帧命中 ⇒ 单帧噪点不再触发状态切换；
         ② ALIGN/APPROACH 中丢目标先计数，连续丢满 LOST_FRAMES 帧才退回 SEARCH 重新找，
            不再进 FAULT —— 丢目标不是安全问题，安全由超声 HARD_STOP_MM 兜底。 */
    if(box_valid)begin if(hit_cnt!=3'd3)hit_cnt<=hit_cnt+1'b1; end
    else hit_cnt<=3'd0;
    if(state!=SEARCH && !box_valid)begin
     if(miss_cnt>=LOST_FRAMES-1)begin
      /* ★2026-10-28：丢目标时先看距离。已经贴到 LOSE_ARRIVE_MM 以内就判到达，
         不再退回 SEARCH 重新扫。近距离丢目标有明确的物理原因（板子占满画面、
         相机偏心导致板子漂出视野、近距离像素整体变亮让判据在边界抖动），
         而车都已经走到跟前了，这时候回去扫一圈是错的行为。 */
      if(dist<=LOSE_ARRIVE_MM)begin
       state<=ARRIVED;reason<=0;
      end else begin
       state<=SEARCH;
      end
      miss_cnt<=0;locks<=0;near_hits<=0;vx<=0;w<=0;
     end else miss_cnt<=miss_cnt+1'b1;
    end else begin
     miss_cnt<=0;
     if(box_valid)begin
      if(state==SEARCH)begin
       if(hit_cnt>=HIT_FRAMES-1)begin state<=ALIGN;vx<=0;w<=0;locks<=0;end
      end else if(incoming_centered && aim_ok)begin
       /* ★2026-10-29：进入接近除了"中点居中"还要"完整入框"（aim_ok）。
          这只作用于 ALIGN→APPROACH 的转换点；接近过程中不判，
          因为贴上去之后板子必然占满画面、必然贴边。 */
       if(locks<LOCK_FRAMES)locks<=locks+1'b1;
       if(locks>=LOCK_FRAMES-1)state<=APPROACH;
      end else begin locks<=0;near_hits<=0;state<=ALIGN;vx<=0;end
     end
    end
    end
    // 到点判定只看距离（见下方注释），不再用画面居中做前置条件。
    if(sonar_strobe && state==APPROACH)begin
    /* ★2026-10-28：到点判据去掉 `centered`。
       相机装在车体中心偏右，逼近后板子在画面里会持续往一侧漂，`centered` 长期不成立
       ⇒ 车已经在 20cm 却永远判不到到达 ⇒ 继续往前磨、最后丢目标退回 SEARCH 又转两圈，
       这就是"快到 20cm 突然重新扫描两次"的直接原因。
       任务目标是"移动到其附近"，距离是唯一必要条件；居中的作用只是让前进方向不跑偏。 */
    if(distance_mm<=STOP_MM)begin
     vx<=0;w<=0;
     if(near_hits>=2)begin state<=ARRIVED;reason<=0;end
     else near_hits<=near_hits+1'b1;
    end else near_hits<=0;
    end
    if(tick && !frame_strobe && !sonar_strobe)begin
    vx<=0;w<=0;
    if(drive_enable)begin
     if(state==SEARCH)
      // 转—停交替：停的那段画面静止，检测最稳；平均转速 = SEARCH_RATE × ON/(ON+OFF)
      w <= (srch_ph < SEARCH_ON) ? TURN_SIGN*SEARCH_RATE*65536 : 32'sd0;
     else if(!centered)begin
      /* 对准单独用 ALIGN_RATE（2026-10-28 拆分）：搜索调快不影响对准，
         对准转速过高会在中心带两侧来回过头、迟迟凑不满居中帧数。 */
      // Image-left => positive CCW yaw (must be verified on the real camera).
      w <= cx<CENTER_X ? TURN_SIGN*ALIGN_RATE*65536 : -TURN_SIGN*ALIGN_RATE*65536;
     end else if(state==APPROACH && dist>STOP_MM)
      vx<=dist>600 ? FAR_SPEED*65536:NEAR_SPEED*65536;
    end
    end
    // ★贴脸到点：APPROACH 阶段目标有效且已进 HARD_STOP_MM 内 ⇒ 直接判到达。
    //   放在本分支最后，用非阻塞赋值覆盖前面各块对 state/vx/w 的写，优先级最高。
    if(arrive_by_prox)begin state<=ARRIVED;reason<=0;vx<=0;w<=0;near_hits<=0;end
   end
  end else begin vx<=0;w<=0;end
 end
endmodule
