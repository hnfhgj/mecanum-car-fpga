// Standalone map construction, not connected to actuator outputs.
// 64x64, 50mm cells. Only traversed ray cells are free; unknown remains blocked.
// The scanner must retain ownership from start until done. Output writes a
// complete map, then done; consumer must not plan until this atomic commit.
module square_map(input wire clk,rst_n,cancel,start,
 input wire [9:0] point_count,input wire signed [15:0] read_forward,read_left,
 output reg [8:0] read_addr,
 input wire signed [15:0] pose_x,pose_y,pose_yaw,
 input wire [15:0] width_mm,height_mm,
 output reg busy,done,valid,near_obstacle,
 output reg map_we,output reg [11:0] map_addr,output reg map_free);
 reg observed[0:4095],occupied[0:4095];
 reg occupied_read,occupied_read_valid,observed_read;
 reg [4:0] state;reg [12:0] cell_index;reg [9:0] point_index;
 reg signed [15:0] px0,py0,theta;reg [15:0] width,height;
 reg signed [16:0] wx,wy;reg signed [10:0] rx,ry,tx,ty,dx,dy,sx,sy;
 reg signed [12:0] error;
 wire signed [13:0] twice_error=$signed(error)*2;
 // 150mm fore/aft, 80mm left/right: circumradius=170mm, margin=100mm.
 // Inflate endpoints by 270mm plus the full 50mm-cell diagonal (71mm).
 reg signed [5:0] ox,oy;reg hit;
 reg project_start;wire project_done;wire signed [15:0] projected_x,projected_y;
 wire signed [31:0] world_x=$signed(projected_x)+$signed(px0);
 wire signed [31:0] world_y=$signed(projected_y)+$signed(py0);
 wire signed [31:0] distance_squared=$signed(projected_x)*$signed(projected_x)+$signed(projected_y)*$signed(projected_y);
 wire signed [31:0] gx=wx>=0?wx/50:-((-$signed(wx)+49)/50);
 wire signed [31:0] gy=wy>=0?wy/50:-((-$signed(wy)+49)/50);
 wire signed [7:0] nx=$signed({1'b0,cell_index[5:0]})+$signed(ox);
 wire signed [7:0] ny=$signed({1'b0,cell_index[11:6]})+$signed(oy);
 wire neighbour_valid=nx>=0 && nx<64 && ny>=0 && ny<64 &&
  ($signed(ox)*$signed(ox)+$signed(oy)*$signed(oy))<=47;
 wire [15:0] centre_x=cell_index[5:0]*50+25,centre_y=cell_index[11:6]*50+25;
 wire interior=centre_x>=270 && centre_y>=270 && centre_x+270<=width && centre_y+270<=height;
 square_project project(.clk(clk),.rst_n(rst_n),.cancel(cancel),.start(project_start),
  .forward_mm(read_forward),.left_mm(read_left),.angle_cdeg(theta),.busy(),.done(project_done),.x(projected_x),.y(projected_y));
 always @(posedge clk)begin
  if(!cancel && state==1)begin observed[cell_index[11:0]]<=0;occupied[cell_index[11:0]]<=0;end
  else if(!cancel)begin
   if(state==6 && !(rx==tx && ry==ty) && rx>=0 && rx<64 && ry>=0 && ry<64)observed[{ry[5:0],rx[5:0]}]<=1;
   if(state==12 && wx>70 && wy>70 && wx<$signed({1'b0,width})-70 &&
      wy<$signed({1'b0,height})-70 && tx>=0 && tx<64 && ty>=0 && ty<64)occupied[{ty[5:0],tx[5:0]}]<=1;
  end
  occupied_read<=occupied[{ny[5:0],nx[5:0]}];occupied_read_valid<=state==8 && neighbour_valid;
  observed_read<=observed[cell_index[11:0]];
 end
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin state<=0;busy<=0;done<=0;valid<=0;near_obstacle<=0;map_we<=0;map_addr<=0;map_free<=0;
   cell_index<=0;point_index<=0;read_addr<=0;px0<=0;py0<=0;theta<=0;width<=0;height<=0;wx<=0;wy<=0;
   rx<=0;ry<=0;tx<=0;ty<=0;dx<=0;dy<=0;sx<=0;sy<=0;error<=0;ox<=0;oy<=0;hit<=0;project_start<=0;
  end else begin
   done<=0;map_we<=0;project_start<=0;
   if(cancel)begin state<=0;busy<=0;valid<=0;end
   else case(state)
    0:if(start)begin busy<=1;valid<=0;near_obstacle<=0;px0<=pose_x;py0<=pose_y;
     theta<=pose_yaw-16'sd9000;width<=width_mm;height<=height_mm;cell_index<=0;state<=1;end
    1:begin
     if(cell_index==4095)begin point_index<=0;read_addr<=0;state<=2;end else cell_index<=cell_index+1'b1;end
    2:if(point_count==0)begin state<=0;busy<=0;done<=1;end
     else begin project_start<=1;state<=3;end
    3:if(project_done)begin
     wx<=world_x;wy<=world_y;state<=11;
     if(distance_squared<72900)near_obstacle<=1;
     // Frame edges have their own exact geometric clearance; don't inflate
     // their noisy endpoints as if they were interior objects.
    end
    11:begin tx<=gx;ty<=gy;rx<=px0/50;ry<=py0/50;state<=12;end
    12:state<=4;
    4:begin dx<=tx>=rx?tx-rx:rx-tx;dy<=ty>=ry?ry-ty:ty-ry;
     sx<=rx<tx?1:-1;sy<=ry<ty?1:-1;state<=5;end
    5:begin error<=dx+dy;state<=6;end
    6:begin
     if((rx==tx && ry==ty) || rx<0 || rx>=64 || ry<0 || ry>=64)state<=7;
     else begin
      if(twice_error>=dy)rx<=rx+sx;
      if(twice_error<=dx)ry<=ry+sy;
      if(twice_error>=dy && twice_error<=dx)error<=error+dy+dx;
      else if(twice_error>=dy)error<=error+dy;
      else if(twice_error<=dx)error<=error+dx;
     end
    end
    7:if(point_index+1>=point_count)begin cell_index<=0;ox<=-7;oy<=-7;hit<=0;state<=8;end
     else begin point_index<=point_index+1'b1;read_addr<=read_addr+1'b1;state<=2;end
    8:begin
     if(occupied_read_valid && occupied_read)hit<=1;
     if(ox==7)begin ox<=-7;if(oy==7)state<=17;else oy<=oy+1'b1;end else ox<=ox+1'b1;
    end
    17:begin if(occupied_read_valid && occupied_read)hit<=1;state<=9;end
    9:begin map_we<=1;map_addr<=cell_index[11:0];map_free<=observed_read && !hit && interior;
     if(cell_index==4095)state<=10;
     else begin cell_index<=cell_index+1'b1;ox<=-7;oy<=-7;hit<=0;state<=8;end
    end
    10:begin valid<=!near_obstacle;busy<=0;done<=1;state<=0;end
    default:begin state<=0;busy<=0;valid<=0;end
   endcase
  end
 end
endmodule
