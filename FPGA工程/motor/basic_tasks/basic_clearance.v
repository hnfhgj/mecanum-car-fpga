// Full-scan body-frame swept-circle corridors. Units mm, speeds counts/10ms.
// Range-dependent target reduction is separate from wheel PID/PWM limiting.
module basic_clearance #(parameter integer CLK_HZ=50000000)(
 input wire clk,rst_n,bad,point_valid,
 input wire signed [15:0] forward_mm,left_mm,
 input wire [15:0] angle_cdeg,range_mm,
 output wire fresh,near_stop,output reg scan_update,
 output reg [15:0] front_mm,back_mm,left_distance,right_mm,
 output wire [6:0] front_limit,back_limit,left_limit,right_limit);
 reg previous_valid,collect,live_stop,scan_stop;
 reg [15:0] previous_angle,minimum[0:3];reg [3:0] seen;
 reg [9:0] count;reg [31:0] age;integer k;
 wire wrap_scan=previous_valid && previous_angle>angle_cdeg+17'd18000;
 wire signed [31:0] f=forward_mm,l=left_mm;
 wire [31:0] af=f<0?-f:f,al=l<0?-l:l;
 wire good=range_mm>=50 && range_mm<=5000;
 function [6:0] limit(input [15:0] distance);
 begin
  if(distance<=270)limit=0;
  else if(distance<400)limit=15;
  else if(distance<550)limit=30;
  else if(distance<700)limit=45;
  else limit=60;
 end endfunction
 assign fresh=age<CLK_HZ/4;
 assign near_stop=live_stop || scan_stop;
 assign front_limit=fresh?limit(front_mm):0;
 assign back_limit=fresh?limit(back_mm):0;
 assign left_limit=fresh?limit(left_distance):0;
 assign right_limit=fresh?limit(right_mm):0;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin previous_valid<=0;collect<=0;live_stop<=0;scan_stop<=0;seen<=0;count<=0;
   previous_angle<=0;age<=32'hffffffff;front_mm<=0;back_mm<=0;left_distance<=0;right_mm<=0;scan_update<=0;
   for(k=0;k<4;k=k+1)minimum[k]<=16'hffff;
  end else begin
   scan_update<=0;if(age!=32'hffffffff)age<=age+1'b1;
   if(bad)begin previous_valid<=0;collect<=0;age<=32'hffffffff;live_stop<=0;scan_stop<=0;end
   else if(point_valid)begin
    previous_valid<=1;previous_angle<=angle_cdeg;
    if(wrap_scan)begin
     if(collect && count>=120 && (&seen))begin
      front_mm<=minimum[0];back_mm<=minimum[1];left_distance<=minimum[2];right_mm<=minimum[3];
      scan_stop<=live_stop;age<=0;scan_update<=1;
     end else age<=32'hffffffff;
     collect<=1;count<=0;seen<=0;live_stop<=0;
     for(k=0;k<4;k=k+1)minimum[k]<=16'hffff;
    end else if(collect && good)begin
     if(count<1023)count<=count+1'b1;
     if(al<=270 && f>0)begin seen[0]<=1;if(f<minimum[0])minimum[0]<=f;end
     if(al<=270 && f<0)begin seen[1]<=1;if(af<minimum[1])minimum[1]<=af;end
     if(af<=270 && l>0)begin seen[2]<=1;if(l<minimum[2])minimum[2]<=l;end
     if(af<=270 && l<0)begin seen[3]<=1;if(al<minimum[3])minimum[3]<=al;end
    end
    // Immediate stop latch is not delayed until the next full scan commit.
    if(good && range_mm<=270)live_stop<=1;
   end
  end
 end
endmodule
