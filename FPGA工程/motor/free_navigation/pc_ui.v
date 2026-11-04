// One task at a time: S1 back/stop, S2 next, S3 enter/+, S4 -/stop.
module pc_ui #(parameter integer CLK_HZ=50000000)(
 input wire clk,rst_n,active,input wire [3:0] key_n,
 input wire remote_valid,input wire [7:0] remote_seq,remote_op,
 input wire [15:0] remote_a,remote_b,remote_c,
 input wire motion_ready,range_ready,nav_ready,
 output reg ack_valid,output reg [7:0] ack_seq,ack_status,
 input wire calibrated,input wire [15:0] width_mm,height_mm,
 output reg [2:0] page,group_id,output reg menu_view,
 output reg [3:0] cursor,output reg [2:0] motion_selection,output reg slow_test,
 output reg demo_start,single_start,route_start,stop_pulse,calibrate_pulse,
 output reg [15:0] target_x,target_y,output wire [63:0] points_x,points_y,
 output reg [1:0] edit_index,
 output reg [3:0] gain_selection,output reg [31:0] heading_kp,heading_kd,
 output reg [2:0] speed_level,
 input wire manual_active,manual_range_ready,output reg drive_valid,output reg [5:0] drive_mask,output reg [15:0] stop_fb_mm,stop_lr_mm,output reg [1:0] cone_mode);
 wire [3:0] press;
 reg [15:0] max_x,max_y;reg [2:0] bounds_ready;
 reg cached;reg [7:0] cached_seq,cached_status;
 reg [63:0] cached_payload;
 reg remote_owned,seen_active;reg [31:0] lease_age;
 wire [63:0] payload={remote_op,8'd0,remote_a,remote_b,remote_c};
 wire local_stop=press[0] || (press[3] && active);
 wire location_ok=(&bounds_ready) && remote_a>=300 && remote_a<=max_x && remote_b>=300 && remote_b<=max_y;
 wire point_ok=(&bounds_ready) && remote_a<4 && remote_b>=300 && remote_b<=max_x && remote_c>=300 && remote_c<=max_y;
 reg [7:0] decision;
 always @*begin
  decision=0;
  if(remote_op==14)decision=(remote_a==16'h97F4 && remote_b==0 && remote_c==0)?0:5;
  else if(remote_op==15)begin
   if(remote_a>63 || remote_b!=0 || remote_c!=0)decision=2;
   else if(remote_a==0)decision=0;
   else if(|press)decision=4;
   else if(active && !manual_active)decision=1;
   else if(!motion_ready || !manual_range_ready)decision=3;
  end
  else if(remote_op==0 || remote_op==11 || remote_op==12)decision=0;
  else if(|press)decision=4;
  else if(active)decision=1;
  else case(remote_op)
   1:if(remote_a>5)decision=2;
   2:if(remote_a>5)decision=2;else if(!motion_ready)decision=3;
   3:if(!motion_ready || !range_ready)decision=3;
   4:if(!motion_ready || !manual_range_ready)decision=3;
   5:if(!location_ok)decision=2;
   6,8:if(!nav_ready)decision=3;
   7:if(!point_ok)decision=2;
   9:if(remote_a>1280 || remote_b>16384)decision=2;
   10:decision=0;
   16:if(remote_a<200 || remote_a>800 || remote_b<150 || remote_b>600 || remote_c<1 || remote_c>3)decision=2;
   13:if(remote_a<1 || remote_a>6 || remote_b!=0 || remote_c!=0)decision=2;
   default:decision=5;
  endcase
 end
 nav_buttons #(.CLK_HZ(CLK_HZ)) keys(.clk(clk),.rst_n(rst_n),.key_n(key_n),
  .press(press),.release_event(),.long_press(),.stable());
 reg [15:0] xs[0:3],ys[0:3];integer i;
 reg [15:0] available_x,available_y,cell_x,cell_y;
 wire [3:0] last_item=group_id==0?6:(group_id==1?5:(group_id==2?7:(group_id==3?4:(group_id==4?7:4))));
 function [15:0] adjust(input [15:0] value,low,high,input up);
 begin if(up)adjust=value+50>high?high:value+50;else adjust=value<=low+50?low:value-50;end endfunction
 assign points_x={xs[3],xs[2],xs[1],xs[0]};assign points_y={ys[3],ys[2],ys[1],ys[0]};
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin available_x<=1000;available_y<=1000;cell_x<=20;cell_y<=20;max_x<=1000;max_y<=1000;bounds_ready<=0;end
  else begin
   available_x<=width_mm>600?width_mm-295:16'd1000;available_y<=height_mm>600?height_mm-295:16'd1000;
   cell_x<=available_x/50;cell_y<=available_y/50;max_x<=cell_x*50;max_y<=cell_y*50;
   if(calibrated && width_mm>600 && height_mm>600)bounds_ready<={bounds_ready[1:0],1'b1};else bounds_ready<=0;
  end
 end
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   cached<=0;cached_seq<=0;cached_status<=0;cached_payload<=0;
   drive_valid<=0;drive_mask<=0;ack_valid<=0;ack_seq<=0;ack_status<=0;remote_owned<=0;seen_active<=0;lease_age<=0;
   page<=5;group_id<=0;menu_view<=1;cursor<=1;motion_selection<=0;slow_test<=0;
   demo_start<=0;single_start<=0;route_start<=0;stop_pulse<=0;calibrate_pulse<=0;
   target_x<=1000;target_y<=1000;edit_index<=0;gain_selection<=12;heading_kp<=320;heading_kd<=2560;speed_level<=4;stop_fb_mm<=400;stop_lr_mm<=250;cone_mode<=2;
   xs[0]<=500;ys[0]<=500;xs[1]<=1500;ys[1]<=500;xs[2]<=1500;ys[2]<=1500;xs[3]<=500;ys[3]<=1500;
  end else begin
   demo_start<=0;single_start<=0;route_start<=0;stop_pulse<=0;calibrate_pulse<=0;
   ack_valid<=0;drive_valid<=0;
   if(remote_owned)begin
    if(lease_age<CLK_HZ*2)lease_age<=lease_age+1'b1;
    if(active)seen_active<=1;
    if((seen_active && !active) || local_stop || (!active && press[2]))begin remote_owned<=0;seen_active<=0;end
    if(lease_age>=CLK_HZ*2)begin stop_pulse<=1;remote_owned<=0;seen_active<=0;end
   end
   if((&bounds_ready) && !active)begin
    if(target_x>max_x)target_x<=max_x;if(target_y>max_y)target_y<=max_y;
    for(i=0;i<4;i=i+1)begin if(xs[i]>max_x)xs[i]<=max_x;if(ys[i]>max_y)ys[i]<=max_y;end
   end
   if(press[0])begin
    stop_pulse<=1;
    if(page!=5)begin page<=5;menu_view<=1;cursor<=1;end
    else if(!menu_view)begin menu_view<=1;end
    else begin group_id<=group_id==5?4:0;cursor<=1;end
   end
   if(press[3] && active)stop_pulse<=1;
   if(!active && !press[0])begin
    if(page==1)begin
     if(press[1])gain_selection<=gain_selection==12?14:12;
     if(press[2]^press[3])begin
      if(gain_selection==12)begin
       if(press[2])heading_kp<=heading_kp>=1272?1280:heading_kp+8;
       else heading_kp<=heading_kp<=8?0:heading_kp-8;
      end else begin
       if(press[2])heading_kd<=heading_kd>=16128?16384:heading_kd+256;
       else heading_kd<=heading_kd<=256?0:heading_kd-256;
      end
     end
    end else if(page==5 && menu_view)begin
     if(press[1])cursor<=cursor==last_item?1:cursor+1'b1;
     if(press[2]^press[3])case(group_id)
      0:if(press[2])begin
       if(cursor<=4)begin group_id<=cursor[2:0];cursor<=1;end
       else if(cursor==5)page<=1;
       else menu_view<=0;
      end
      1:if(press[2])case(cursor)
       1:page<=4;2:page<=0;3:page<=2;4:page<=3;
       default:begin group_id<=0;cursor<=1;end
      endcase
      2:if(press[2])begin
       if(cursor==7)begin group_id<=0;cursor<=1;end
       else begin motion_selection<=cursor-1'b1;slow_test<=0;demo_start<=1;end
      end
      3:if(press[2])case(cursor)
       1:begin end // Live distances remain on this page.
       2:begin motion_selection<=0;slow_test<=1;demo_start<=1;end
       3:begin group_id<=4;cursor<=1;end
       default:begin group_id<=0;cursor<=1;end
      endcase
      4:case(cursor)
       1:if(press[2])calibrate_pulse<=1;
       2:if(&bounds_ready)target_x<=adjust(target_x,300,max_x,press[2]);
       3:if(&bounds_ready)target_y<=adjust(target_y,300,max_y,press[2]);
       4:if(press[2])single_start<=1;
       5:if(press[2])begin group_id<=5;cursor<=1;end
       6:if(press[2])route_start<=1;
       7:if(press[2])begin group_id<=0;cursor<=1;end
      endcase
      5:case(cursor)
       1:edit_index<=press[2]?edit_index+1'b1:edit_index-1'b1;
       2:if(&bounds_ready)xs[edit_index]<=adjust(xs[edit_index],300,max_x,press[2]);
       3:if(&bounds_ready)ys[edit_index]<=adjust(ys[edit_index],300,max_y,press[2]);
       4:if(press[2])begin group_id<=4;cursor<=5;end
      endcase
      default:begin group_id<=0;cursor<=1;end
     endcase
    end
   end
   if(remote_valid)begin
    // Heartbeats neither generate ACKs nor replace the last action's retry cache.
    if(remote_op==11)begin lease_age<=0;end
    else begin
     ack_valid<=1;ack_seq<=remote_seq;
     if(remote_op==0)begin
      stop_pulse<=1;remote_owned<=0;seen_active<=0;ack_status<=0;
      cached<=1;cached_seq<=remote_seq;cached_payload<=payload;cached_status<=0;
     end else if(cached && remote_seq==cached_seq)begin
      ack_status<=payload==cached_payload?cached_status:8'd6;
     end else begin
      cached<=1;cached_seq<=remote_seq;cached_payload<=payload;cached_status<=decision;
      ack_status<=decision;
      if(decision==0)begin
       lease_age<=0;
       case(remote_op)
        1:begin page<=remote_a[2:0];menu_view<=0;end
        2,3:begin
         page<=5;group_id<=remote_op==2?2:3;menu_view<=1;
         cursor<=remote_op==2?remote_a[3:0]+1'b1:4'd2;
         motion_selection<=remote_op==2?remote_a[2:0]:3'd0;
         slow_test<=remote_op==3;demo_start<=1;remote_owned<=1;seen_active<=0;
        end
        4:begin page<=5;group_id<=4;cursor<=1;menu_view<=1;calibrate_pulse<=1;end
        5:begin target_x<=remote_a;target_y<=remote_b;end
        6:begin page<=5;group_id<=4;cursor<=4;menu_view<=1;single_start<=1;remote_owned<=1;seen_active<=0;end
        7:begin xs[remote_a[1:0]]<=remote_b;ys[remote_a[1:0]]<=remote_c;end
        8:begin page<=5;group_id<=4;cursor<=6;menu_view<=1;route_start<=1;remote_owned<=1;seen_active<=0;end
        9:begin heading_kp<={16'd0,remote_a};heading_kd<={16'd0,remote_b};end
        10:begin page<=5;group_id<=0;cursor<=1;menu_view<=1;end
        13:speed_level<=remote_a[2:0];
        16:begin stop_fb_mm<=remote_a;stop_lr_mm<=remote_b;cone_mode<=remote_c[1:0];end
        15:begin drive_valid<=1;drive_mask<=remote_a[5:0];
         if(remote_a!=0)begin page<=5;group_id<=3;menu_view<=0;end
        end
        default:begin end
       endcase
      end
     end
    end
   end
  end
 end
endmodule
