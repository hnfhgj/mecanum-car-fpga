module square_ui #(parameter integer CLK_HZ=50000000)(
 input wire clk,rst_n,input wire [3:0] key_n,input wire active,calibrated,
 input wire [15:0] width_mm,height_mm,
 output reg [2:0] page,output reg menu_view,output reg [3:0] cursor,
 output reg [15:0] target_x,target_y,
 output reg calibrate_pulse,go_pulse,stop_pulse,
 output reg [3:0] gain_selection,output reg [31:0] heading_kp,heading_kd);
 wire [3:0] press,release_event,long_press;
 nav_buttons #(.CLK_HZ(CLK_HZ)) buttons(.clk(clk),.rst_n(rst_n),.key_n(key_n),.press(press),
  .release_event(release_event),.long_press(long_press),.stable());
 reg s1_long;
 // Keep targets on the 50mm input grid; their cell centres must retain 270mm.
 reg [15:0] available_x,available_y,cell_x,cell_y,max_x,max_y;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin available_x<=1000;available_y<=1000;cell_x<=20;cell_y<=20;max_x<=1000;max_y<=1000;end
  else begin
   available_x<=width_mm>600?width_mm-295:16'd1000;
   available_y<=height_mm>600?height_mm-295:16'd1000;
   cell_x<=available_x/50;cell_y<=available_y/50;
   max_x<=cell_x*50;max_y<=cell_y*50;
  end
 end
 function [15:0] adjust(input [15:0] value,step,low,high,input increase);
 begin if(increase)adjust=value+step>high?high:value+step;else adjust=value<=low+step?low:value-step;end endfunction
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin page<=5;menu_view<=1;cursor<=1;target_x<=1000;target_y<=1000;calibrate_pulse<=0;
   go_pulse<=0;stop_pulse<=0;gain_selection<=12;heading_kp<=320;heading_kd<=2560;s1_long<=0;
  end else begin
   calibrate_pulse<=0;go_pulse<=0;stop_pulse<=0;
   if(calibrated)begin if(target_x>max_x)target_x<=max_x;if(target_y>max_y)target_y<=max_y;end
   if(long_press[0])begin s1_long<=1;page<=page==5?0:5;menu_view<=1;if(active)stop_pulse<=1;end
   if(release_event[0])begin
    if(!s1_long)begin if(page==5)menu_view<=!menu_view;else page<=page==4?5:page+1'b1;end s1_long<=0;
   end
   if(active && press[3])stop_pulse<=1;
   if(page==1 && !active)begin
    if(press[1])gain_selection<=gain_selection==12?14:12;
    if(press[2]^press[3])begin
     if(gain_selection==12)heading_kp<=adjust(heading_kp[15:0],8,0,1280,press[2]);
     else heading_kd<=adjust(heading_kd[15:0],256,0,16384,press[2]);
    end
   end
   if(page==5)begin
    if(press[1])begin cursor<=cursor==4?1:cursor+1'b1;menu_view<=1;end
    if(!active && (press[2]^press[3]))case(cursor)
     1:if(press[2])calibrate_pulse<=1;
     2:target_x<=adjust(target_x,50,300,max_x,press[2]);
     3:target_y<=adjust(target_y,50,300,max_y,press[2]);
     4:if(press[2])go_pulse<=1;else stop_pulse<=1;
    endcase
   end
  end
 end
endmodule
