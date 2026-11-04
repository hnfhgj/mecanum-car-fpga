module nav_ui #(parameter integer CLK_HZ=50000000)(
 input wire clk,rst_n,input wire [3:0] key_n,input wire active,
 input wire feature_done,feature_ok,input wire [3:0] count,input wire [191:0] xs,ys,
 input wire learn_done,learn_ok,template_valid,input wire [15:0] width_mm,height_mm,
 output reg [2:0] page,output reg menu_view,output reg [3:0] cursor,
 output reg [3:0] o_id,x_id,y_id,frozen_count,output reg [191:0] frozen_xs,frozen_ys,
 output reg [15:0] diameter_mm,target_x,target_y,
 output reg learn_req,align_pulse,go_pulse,stop_pulse,clear_cal,
 output reg [3:0] gain_selection,output reg [31:0] heading_kp,heading_kd);
 wire [3:0] press,release_event,long_press;
 nav_buttons #(.CLK_HZ(CLK_HZ)) buttons(.clk(clk),.rst_n(rst_n),.key_n(key_n),.press(press),
  .release_event(release_event),.long_press(long_press),.stable());
 reg s1_long;reg [1:0] diameter_wait;
 wire [15:0] max_x=width_mm>=500?width_mm-200:16'd300;
 wire [15:0] max_y=height_mm>=500?height_mm-200:16'd300;
 function [15:0] adjust(input [15:0] value,input [15:0] step,low,high,input increase);
 begin if(increase)adjust=value+step>high?high:value+step;else adjust=value<=low+step?low:value-step;end endfunction
 function [3:0] id_adjust(input [3:0] value,input increase,input [3:0] total);
 begin if(total==0)id_adjust=0;else if(increase)id_adjust=value+1<total?value+1'b1:0;else id_adjust=value==0?total-1'b1:value-1'b1;end endfunction
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin page<=5;menu_view<=0;cursor<=1;o_id<=0;x_id<=1;y_id<=2;frozen_count<=0;frozen_xs<=0;frozen_ys<=0;
   diameter_mm<=100;target_x<=1000;target_y<=1000;learn_req<=0;align_pulse<=0;go_pulse<=0;stop_pulse<=0;clear_cal<=0;
   gain_selection<=12;heading_kp<=320;heading_kd<=2560;s1_long<=0;diameter_wait<=0;
  end else begin
   align_pulse<=0;go_pulse<=0;stop_pulse<=0;clear_cal<=0;
   if(feature_done && diameter_wait!=0)diameter_wait<=diameter_wait-1'b1;
   if(learn_done)learn_req<=0;
   if(template_valid)begin
    if(target_x>max_x)target_x<=max_x;if(target_y>max_y)target_y<=max_y;
   end
   if(long_press[0])begin s1_long<=1;page<=page==5?0:5;menu_view<=0;if(active)stop_pulse<=1;end
   if(release_event[0])begin
    if(!s1_long)begin
     if(page==5)menu_view<=!menu_view;
     else begin page<=page==4?5:page+1'b1;menu_view<=0;end
    end
    s1_long<=0;
   end
   if(active && press[3])stop_pulse<=1;
   if(page==1)begin
    if(press[1])gain_selection<=gain_selection==12?14:12;
    if(press[2]^press[3])begin
     if(gain_selection==12)heading_kp<=adjust(heading_kp[15:0],16'd8,16'd0,16'd1280,press[2]);
     else heading_kd<=adjust(heading_kd[15:0],16'd256,16'd0,16'd16384,press[2]);
    end
   end
   if(page==5)begin
    if(press[1])begin if(menu_view)cursor<=cursor==10?1:cursor+1'b1;menu_view<=1;end
    if(!active && (press[2]^press[3]))begin
     case(cursor)
      1:if(press[2] && feature_ok && count>=3 && diameter_wait==0)begin
       frozen_count<=count;frozen_xs<=xs;frozen_ys<=ys;menu_view<=0;stop_pulse<=1;
      end
      2:o_id<=id_adjust(o_id,press[2],frozen_count);
      3:x_id<=id_adjust(x_id,press[2],frozen_count);
      4:y_id<=id_adjust(y_id,press[2],frozen_count);
      5:begin diameter_mm<=adjust(diameter_mm,16'd10,16'd0,16'd400,press[2]);
       frozen_count<=0;clear_cal<=1;diameter_wait<=2;end
      6:if(press[2] && frozen_count>=3)begin learn_req<=1;clear_cal<=1;stop_pulse<=1;end
      7:if(press[2])align_pulse<=1;
      8:begin target_x<=adjust(target_x,16'd50,16'd200,max_x,press[2]);stop_pulse<=1;end
      9:begin target_y<=adjust(target_y,16'd50,16'd200,max_y,press[2]);stop_pulse<=1;end
      10:if(press[2])go_pulse<=1;else stop_pulse<=1;
      default:begin end
     endcase
    end
   end
  end
 end
endmodule
