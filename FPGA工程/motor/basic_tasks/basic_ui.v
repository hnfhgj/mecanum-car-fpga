// One task at a time: S1 back/stop, S2 next, S3 enter/+, S4 -/stop.
module basic_ui #(parameter integer CLK_HZ=50000000)(
 input wire clk,rst_n,active,input wire [3:0] key_n,
 input wire calibrated,input wire [15:0] width_mm,height_mm,
 output reg [2:0] page,group_id,output reg menu_view,
 output reg [3:0] cursor,output reg [2:0] motion_selection,output reg slow_test,
 output reg demo_start,single_start,route_start,stop_pulse,calibrate_pulse,
 output reg [15:0] target_x,target_y,output wire [63:0] points_x,points_y,
 output reg [1:0] edit_index,
 output reg [3:0] gain_selection,output reg [31:0] heading_kp,heading_kd);
 wire [3:0] press;
 nav_buttons #(.CLK_HZ(CLK_HZ)) keys(.clk(clk),.rst_n(rst_n),.key_n(key_n),
  .press(press),.release_event(),.long_press(),.stable());
 reg [15:0] xs[0:3],ys[0:3];integer i;
 reg [15:0] available_x,available_y,cell_x,cell_y,max_x,max_y;
 reg [2:0] bounds_ready;
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
   page<=5;group_id<=0;menu_view<=1;cursor<=1;motion_selection<=0;slow_test<=0;
   demo_start<=0;single_start<=0;route_start<=0;stop_pulse<=0;calibrate_pulse<=0;
   target_x<=1000;target_y<=1000;edit_index<=0;gain_selection<=12;heading_kp<=320;heading_kd<=2560;
   xs[0]<=500;ys[0]<=500;xs[1]<=1500;ys[1]<=500;xs[2]<=1500;ys[2]<=1500;xs[3]<=500;ys[3]<=1500;
  end else begin
   demo_start<=0;single_start<=0;route_start<=0;stop_pulse<=0;calibrate_pulse<=0;
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
  end
 end
endmodule
