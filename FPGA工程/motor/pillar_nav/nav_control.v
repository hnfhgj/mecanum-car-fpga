// Position loop in mm -> encoder counts/10ms. Body +forward/+left, yaw CCW.
// Sensed point is the LIDAR origin; no unknown chassis lever arm is assumed.
module nav_control #(parameter integer CLK_HZ=50000000,POS_TOL_MM=75,MAX_COMPONENT=120)(
 input wire clk,rst_n,template_valid,learn_req,clear_cal,align_pulse,go_pulse,stop_pulse,
 input wire [2:0] page,input wire pose_update,pose_ok,
 input wire signed [15:0] pose_x,pose_y,pose_yaw,input wire [15:0] target_x,target_y,width_mm,height_mm,
 input wire heading_ready,wheel_fault,input wire signed [15:0] heading_turn,
 output reg active,aligned,output wire fresh,can_go,output reg [3:0] status,
 output reg signed [15:0] lf_target,rf_target,lb_target,rb_target,
 output wire signed [15:0] body_yaw);
 reg [31:0] age;reg [2:0] stable_count,arrived;
 reg signed [15:0] mounting_yaw,forward,left;
 reg [3:0] state;reg awaiting;
 reg signed [31:0] ex,ey;
 reg [9:0] phase;
 wire signed [15:0] sn,cs;
 reg signed [15:0] sn_q,cs_q;
 reg math_start;reg signed [63:0] a,temp;reg signed [31:0] b;
 wire math_done;wire signed [63:0] result;
 function signed [15:0] wrap(input signed [31:0] v);
 begin if(v>=18000)wrap=v-36000;else if(v< -18000)wrap=v+36000;else wrap=v;end endfunction
 function [31:0] abs32(input signed [31:0] v);begin abs32=v<0?-v:v;end endfunction
 function signed [15:0] component(input signed [63:0] v);
 reg signed [63:0] t;
 begin t=v>>>16; // Q14 rotation, then proportional gain 1/4 count/mm.
  if(t>MAX_COMPONENT)component=MAX_COMPONENT;else if(t< -MAX_COMPONENT)component=-MAX_COMPONENT;else component=t;
 end endfunction
 wire signed [31:0] heading_ext=pose_yaw,mount_ext=mounting_yaw;
 assign body_yaw=wrap(heading_ext-mount_ext);
 wire signed [31:0] body_ext=body_yaw;
 wire [16:0] positive_yaw=body_yaw<0?body_ext+32'sd36000:body_ext;
 assign fresh=pose_ok && age<CLK_HZ/3;
 wire in_bounds=target_x>=200 && target_y>=200 && target_x<=width_mm-200 && target_y<=height_mm-200;
 assign can_go=template_valid && aligned && fresh && stable_count>=3 && heading_ready && !wheel_fault && in_bounds;
 wire drive=active && fresh && !awaiting && heading_ready && !wheel_fault && page==5;
 // Standard X-roller mecanum. All terms bounded: |120|+|120|+|150|<=390.
 // Break online-age -> validity -> mixing -> wheel PID into separate cycles.
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin lf_target<=0;rf_target<=0;lb_target<=0;rb_target<=0;end
  else begin
   lf_target<=drive?forward-left-heading_turn:16'sd0;
   rf_target<=drive?forward+left+heading_turn:16'sd0;
   lb_target<=drive?forward+left-heading_turn:16'sd0;
   rb_target<=drive?forward-left+heading_turn:16'sd0;
  end
 end
 n10_sincos trig(.phase(phase),.sine(sn),.cosine(cs));
 nav_math math(.clk(clk),.rst_n(rst_n),.cancel(clear_cal || learn_req),.start(math_start),.op(2'd0),.a(a),.b(b),
  .busy(),.done(math_done),.zero_divide(),.result(result));
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin age<=32'hffffffff;stable_count<=0;arrived<=0;mounting_yaw<=0;forward<=0;left<=0;
   state<=0;awaiting<=1;ex<=0;ey<=0;phase<=0;sn_q<=0;cs_q<=0;math_start<=0;a<=0;b<=0;temp<=0;
   active<=0;aligned<=0;status<=0;
  end else begin
   math_start<=0;
   if(age!=32'hffffffff)age<=age+1'b1;
   if(pose_update)begin
    if(pose_ok)begin age<=0;if(stable_count<3)stable_count<=stable_count+1'b1;end
    else stable_count<=0;
   end
   if(!fresh)stable_count<=0;
   if(clear_cal || learn_req || !template_valid)begin
    active<=0;aligned<=0;stable_count<=0;arrived<=0;status<=0;state<=0;forward<=0;left<=0;awaiting<=1;
   end else begin
    if(!active && status<4)status<=!aligned?1:(!can_go?2:3);
    if(align_pulse && fresh && stable_count>=3)begin
     mounting_yaw<=pose_yaw;aligned<=1;active<=0;forward<=0;left<=0;arrived<=0;status<=2;awaiting<=1;
    end
    if(go_pulse && can_go)begin active<=1;arrived<=0;forward<=0;left<=0;awaiting<=1;status<=4;end
    if(active && (!fresh || (pose_update && !pose_ok) || wheel_fault || !heading_ready))begin
     active<=0;status<=6;forward<=0;left<=0;awaiting<=1;
    end
    if(stop_pulse || (active && page!=5))begin active<=0;status<=7;forward<=0;left<=0;awaiting<=1;end
    if(pose_update && pose_ok && active && fresh)begin
     if(abs32($signed({1'b0,target_x})-$signed(pose_x))<=POS_TOL_MM &&
        abs32($signed({1'b0,target_y})-$signed(pose_y))<=POS_TOL_MM)begin
      if(arrived>=2)begin active<=0;status<=5;forward<=0;left<=0;awaiting<=1;end
      else begin arrived<=arrived+1'b1;forward<=0;left<=0;awaiting<=1;end
     end else begin arrived<=0;awaiting<=0;end
    end
    case(state)
     0:if(pose_update && pose_ok)begin
      ex<=$signed({1'b0,target_x})-$signed(pose_x);ey<=$signed({1'b0,target_y})-$signed(pose_y);
      phase<=positive_yaw>=35975?10'd0:(positive_yaw+17'd25)/17'd50;state<=1;
     end
     1:begin sn_q<=sn;cs_q<=cs;state<=2;end
     2:begin a<=ex;b<=cs_q;math_start<=1;state<=3;end
     3:if(math_done)begin temp<=result;a<=ey;b<=sn_q;math_start<=1;state<=4;end
     4:if(math_done)begin if(active && arrived==0)forward<=component(temp+result);state<=5;end
     5:begin a<=-ex;b<=sn_q;math_start<=1;state<=6;end
     6:if(math_done)begin temp<=result;a<=ey;b<=cs_q;math_start<=1;state<=7;end
     7:if(math_done)begin if(active && arrived==0)left<=component(temp+result);state<=0;end
     default:state<=0;
    endcase
   end
  end
 end
endmodule
