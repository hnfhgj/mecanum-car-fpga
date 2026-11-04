// Replan-driven translation with IMU heading hold; no autonomous start.
// Direction is a safe grid edge from the most recent committed map.
module basic_nav_drive #(parameter integer CLK_HZ=50000000)(
 input wire clk,rst_n,calibrate,go,stop,tick,input wire [2:0] page,
 input wire calibrated,pose_update,pose_ok,input wire signed [15:0] pose_x,pose_y,pose_yaw,
 input wire [15:0] target_x,target_y,width_mm,height_mm,
 input wire route_done,route_found,route_goal,input wire [1:0] route_direction,
 input wire clearance_fresh,
 input wire [6:0] front_limit,back_limit,left_limit,right_limit,
 input wire near_obstacle,heading_ready,wheel_fault,input wire signed [15:0] heading_turn,heading_error,
 output reg active,output wire fresh,can_go,output reg [3:0] status,
 output reg signed [15:0] lf_target,rf_target,lb_target,rb_target,input wire [2:0] speed_level);
 reg [31:0] pose_age,route_age,run_age;reg [2:0] stable,arrival;
 reg route_valid;reg [1:0] direction;reg request;
 reg signed [15:0] vx,vy,angle,f,l;
 wire projected;wire signed [15:0] right_component,forward_component;
 function [31:0] ab(input signed [31:0] v);begin ab=v<0?-v:v;end endfunction
 wire at_goal=ab($signed({1'b0,target_x})-$signed(pose_x))<=75 && ab($signed({1'b0,target_y})-$signed(pose_y))<=75;
 wire in_bounds=target_x>=270 && target_y>=270 && target_x+270<=width_mm && target_y+270<=height_mm;
 wire [6:0] selected_limit=direction==0?right_limit:(direction==1?front_limit:(direction==2?left_limit:back_limit));
 wire [15:0] scaled_limit;
 task_translation_scale nav_speed_scale(selected_limit,speed_level,scaled_limit);
 assign fresh=pose_ok && pose_age<CLK_HZ/3;
 assign can_go=calibrated && stable>=3 && fresh && heading_ready && !wheel_fault && !near_obstacle && clearance_fresh && in_bounds;
 wire drive=active && fresh && route_valid && route_age<CLK_HZ/3 && !near_obstacle && heading_ready && !wheel_fault && page==5 && clearance_fresh && selected_limit!=0 && !at_goal;
 square_project transform(.clk(clk),.rst_n(rst_n),.cancel(calibrate || stop),.start(request),
  .forward_mm(vy),.left_mm(-vx),.angle_cdeg(angle),.busy(),.done(projected),.x(right_component),.y(forward_component));
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin active<=0;status<=0;pose_age<=32'hffffffff;route_age<=32'hffffffff;run_age<=0;
   stable<=0;arrival<=0;route_valid<=0;direction<=0;request<=0;vx<=0;vy<=0;angle<=0;f<=0;l<=0;
   lf_target<=0;rf_target<=0;lb_target<=0;rb_target<=0;
  end else begin
   request<=0;
   if(pose_age!=32'hffffffff)pose_age<=pose_age+1'b1;
   if(route_age!=32'hffffffff)route_age<=route_age+1'b1;
   if(active)run_age<=run_age+1'b1;
   if(pose_update)begin
    if(pose_ok)begin pose_age<=0;if(stable<3)stable<=stable+1'b1;end else stable<=0;
   end
   if(!fresh)stable<=0;
   if(route_done)begin route_valid<=route_found;direction<=route_direction;route_age<=0;end
   // Compute after route registers settle, refresh also for changed pose yaw.
   if(route_age==1 || (pose_update && pose_ok) || tick)begin
    vx<=direction==0?$signed(scaled_limit):(direction==2?-$signed(scaled_limit):16'sd0);
    vy<=direction==1?$signed(scaled_limit):(direction==3?-$signed(scaled_limit):16'sd0);
    angle<=16'sd9000-pose_yaw;
   end
   if(route_age==2 || (pose_update && pose_ok) || tick)request<=1;
   if(projected)begin f<=forward_component;l<=-right_component;end
   // First face +Y within two degrees, then permit translation. Heading
   // correction remains enabled throughout lateral/longitudinal travel.
   lf_target<=drive?((ab(heading_error)<=200?f-l:16'sd0)-heading_turn):0;
   rf_target<=drive?((ab(heading_error)<=200?f+l:16'sd0)+heading_turn):0;
   lb_target<=drive?((ab(heading_error)<=200?f+l:16'sd0)-heading_turn):0;
   rb_target<=drive?((ab(heading_error)<=200?f-l:16'sd0)+heading_turn):0;
   if(!active && status<4)status<=!calibrated?0:(!can_go?2:3);
   if(go && can_go)begin active<=1;status<=4;arrival<=0;run_age<=0;route_valid<=0;end
   if(active && pose_update && pose_ok && at_goal)begin
    route_valid<=0;
    if(arrival>=2)begin active<=0;status<=5;end else arrival<=arrival+1'b1;
   end else if(pose_update)arrival<=0;
   if(active && (!fresh || !clearance_fresh || wheel_fault || !heading_ready || near_obstacle ||
      (route_done && !route_found) || (route_valid && selected_limit==0) || run_age>=CLK_HZ*30 || (pose_update && !pose_ok)))begin active<=0;route_valid<=0;status<=6;end
   if(stop || (active && page!=5))begin active<=0;route_valid<=0;status<=7;end
   if(calibrate || !calibrated)begin active<=0;stable<=0;route_valid<=0;status<=0;end
  end
 end
endmodule
