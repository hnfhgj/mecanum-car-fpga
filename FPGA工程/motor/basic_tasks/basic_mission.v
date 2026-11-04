// Single target / four editable ordered targets. Never auto-starts on boot.
module basic_mission #(parameter integer CLK_HZ=50000000)(
 input wire clk,rst_n,start_single,start_route,stop,
 input wire [15:0] single_x,single_y,input wire [63:0] points_x,points_y,
 input wire can_go,nav_active,input wire [3:0] nav_status,
 output reg running,nav_go,goal_change,output reg [15:0] target_x,target_y,
 output reg [1:0] point_index,output reg route_mode,output reg [3:0] status);
 reg [2:0] state;reg [31:0] wait_age;reg [63:0] xs,ys;reg [15:0] sx,sy;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin state<=0;running<=0;nav_go<=0;goal_change<=0;target_x<=1000;target_y<=1000;
   point_index<=0;route_mode<=0;status<=0;wait_age<=0;xs<=0;ys<=0;sx<=0;sy<=0;
  end else begin
   nav_go<=0;goal_change<=0;
   if(stop)begin state<=0;running<=0;status<=7;end
   else case(state)
    0:if(start_single || start_route)begin
     xs<=points_x;ys<=points_y;sx<=single_x;sy<=single_y;
     route_mode<=start_route;point_index<=0;running<=1;status<=1;state<=1;
    end
    1:begin
     target_x<=route_mode?xs[point_index*16+:16]:sx;
     target_y<=route_mode?ys[point_index*16+:16]:sy;
     goal_change<=1;wait_age<=0;status<=1;state<=2;
    end
    2:state<=3; // Target registers and planner cancellation settle.
    3:begin
     if(can_go)begin nav_go<=1;status<=4;state<=4;end
     else if(wait_age>=CLK_HZ*5)begin running<=0;status<=6;state<=0;end
     else wait_age<=wait_age+1'b1;
    end
    4:state<=5; // Do not mistake the previous leg's ARRIVED for this leg.
    5:if(!nav_active)begin
     if(nav_status==5)begin
      if(route_mode && point_index<3)begin point_index<=point_index+1'b1;state<=1;end
      else begin running<=0;status<=5;state<=0;end
     end else if(nav_status==6 || nav_status==7)begin running<=0;status<=6;state<=0;end
    end
    default:begin running<=0;state<=0;status<=6;end
   endcase
  end
 end
endmodule
