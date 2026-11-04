// Streaming cluster extraction: only the first cluster is deferred for angular
// seam merging. Other clusters reuse one arithmetic unit as soon as they end.
module nav_features(input wire clk,rst_n,bad,input wire valid,
 input wire signed [15:0] x,y,input wire [15:0] angle,range_mm,diameter_mm,
 output wire ready,output reg done,scan_ok,output reg [3:0] count,
 output reg [191:0] xs,ys,output reg [31:0] scans);
 reg [3:0] state,candidate_count;reg [191:0] candidate_xs,candidate_ys;
 assign ready=state==0;
 reg previous_valid,collecting,active,over,have_first;
 reg [15:0] previous_angle,n,lo,hi;
 reg signed [31:0] sx,sy;reg [31:0] sr;
 reg signed [15:0] first_x,first_y,last_x,last_y;
 reg signed [31:0] saved_x,saved_y;
 reg [31:0] saved_r;reg [15:0] saved_n,saved_lo,saved_hi;
 reg signed [15:0] saved_fx,saved_fy,saved_lx,saved_ly;
 reg signed [31:0] work_x,work_y,avg_x,avg_y;reg [31:0] work_r;
 reg [15:0] work_n,work_lo,work_hi,mean_range;
 reg signed [15:0] work_fx,work_fy,work_lx,work_ly,center_x;
 reg [1:0] return_mode;
 reg math_start;reg [1:0] op;reg signed [63:0] a;reg signed [31:0] b;
 wire math_done;wire signed [63:0] result;
 nav_math math(.clk(clk),.rst_n(rst_n),.cancel(bad),.start(math_start),.op(op),.a(a),.b(b),
  .busy(),.done(math_done),.zero_divide(),.result(result));
 function [31:0] abs32(input signed [31:0] v);begin abs32=v<0?-v:v;end endfunction
 wire [31:0] gap=abs32($signed(x)-$signed(last_x))+abs32($signed(y)-$signed(last_y));
 wire good=range_mm>=50 && range_mm<=4000;
 wire wrap=previous_valid && previous_angle>angle+17'd18000;
 wire [16:0] delta=angle>=previous_angle?{1'b0,angle}-{1'b0,previous_angle}:17'd36000+angle-previous_angle;
 wire discontinuity=previous_valid && delta>500;
 wire adjacent=active && good && gap<=120;
 wire seam=abs32($signed(saved_fx)-$signed(last_x))+abs32($signed(saved_fy)-$signed(last_y))<=120;
 task seed;
 begin active<=good;sx<=x;sy<=y;sr<=range_mm;n<=good?16'd1:16'd0;
  first_x<=x;first_y<=y;last_x<=x;last_y<=y;lo<=range_mm;hi<=range_mm;end
 endtask
 task save_first;
 begin saved_x<=sx;saved_y<=sy;saved_r<=sr;saved_n<=n;saved_lo<=lo;saved_hi<=hi;
  saved_fx<=first_x;saved_fy<=first_y;saved_lx<=last_x;saved_ly<=last_y;have_first<=1;end
 endtask
 task load_current;
 begin work_x<=sx;work_y<=sy;work_r<=sr;work_n<=n;work_lo<=lo;work_hi<=hi;
  work_fx<=first_x;work_fy<=first_y;work_lx<=last_x;work_ly<=last_y;end
 endtask
 task load_first;
 begin work_x<=saved_x;work_y<=saved_y;work_r<=saved_r;work_n<=saved_n;work_lo<=saved_lo;work_hi<=saved_hi;
  work_fx<=saved_fx;work_fy<=saved_fy;work_lx<=saved_lx;work_ly<=saved_ly;end
 endtask
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin state<=0;done<=0;scan_ok<=0;count<=0;xs<=0;ys<=0;scans<=0;
   candidate_count<=0;candidate_xs<=0;candidate_ys<=0;previous_valid<=0;collecting<=0;active<=0;over<=0;have_first<=0;
   previous_angle<=0;n<=0;lo<=0;hi<=0;sx<=0;sy<=0;sr<=0;first_x<=0;first_y<=0;last_x<=0;last_y<=0;
   saved_x<=0;saved_y<=0;saved_r<=0;saved_n<=0;saved_lo<=0;saved_hi<=0;saved_fx<=0;saved_fy<=0;saved_lx<=0;saved_ly<=0;
   work_x<=0;work_y<=0;work_r<=0;work_n<=0;work_lo<=0;work_hi<=0;work_fx<=0;work_fy<=0;work_lx<=0;work_ly<=0;
   avg_x<=0;avg_y<=0;mean_range<=0;center_x<=0;return_mode<=0;math_start<=0;op<=0;a<=0;b<=0;
  end else begin
   done<=0;math_start<=0;
   if(bad)begin state<=0;collecting<=0;previous_valid<=0;active<=0;have_first<=0;over<=0;scan_ok<=0;
    candidate_count<=0;candidate_xs<=0;candidate_ys<=0;end
   else case(state)
    0:if(valid)begin
     previous_valid<=1;previous_angle<=angle;
     if(discontinuity)begin collecting<=0;active<=0;have_first<=0;over<=0;scan_ok<=0;
      candidate_count<=0;candidate_xs<=0;candidate_ys<=0;end
     else if(wrap)begin
      if(!collecting)begin collecting<=1;candidate_count<=0;candidate_xs<=0;candidate_ys<=0;have_first<=0;over<=0;end
      else begin
       if(active && have_first && seam)begin
        work_x<=sx+saved_x;work_y<=sy+saved_y;work_r<=sr+saved_r;work_n<=n+saved_n;
        work_lo<=lo<saved_lo?lo:saved_lo;work_hi<=hi>saved_hi?hi:saved_hi;
        work_fx<=first_x;work_fy<=first_y;work_lx<=saved_lx;work_ly<=saved_ly;
        return_mode<=1;state<=2;
       end else if(active)begin load_current();return_mode<=have_first?2:1;state<=2;end
       else if(have_first)begin load_first();return_mode<=1;state<=2;end
       else state<=14;
       have_first<=0;
      end
      seed();
     end else if(collecting)begin
      if(adjacent)begin sx<=sx+x;sy<=sy+y;sr<=sr+range_mm;
       if(n<1023)n<=n+1'b1;else over<=1;
       last_x<=x;last_y<=y;if(range_mm<lo)lo<=range_mm;if(range_mm>hi)hi<=range_mm;
      end else begin
       if(active)begin
        if(!have_first)save_first();else begin load_current();return_mode<=0;state<=2;end
       end
       seed();
      end
     end
    end
    2:begin
     if(work_n>=2 && work_n<=64 && work_hi-work_lo<=250 &&
      abs32($signed(work_fx)-$signed(work_lx))+abs32($signed(work_fy)-$signed(work_ly))<=450)state<=3;
     else state<=15;
    end
    3:begin op<=1;a<=work_x;b<={16'd0,work_n};math_start<=1;state<=4;end
    4:if(math_done)begin avg_x<=result;state<=5;end
    5:begin op<=1;a<=work_y;b<={16'd0,work_n};math_start<=1;state<=6;end
    6:if(math_done)begin avg_y<=result;state<=7;end
    7:begin op<=1;a<={32'd0,work_r};b<={16'd0,work_n};math_start<=1;state<=8;end
    8:if(math_done)begin mean_range<=result;state<=9;end
    9:begin op<=0;a<=avg_x;b<={16'd0,mean_range}+(diameter_mm>>1);math_start<=1;state<=10;end
    10:if(math_done)begin op<=1;a<=result;b<={16'd0,mean_range};math_start<=1;state<=11;end
    11:if(math_done)begin center_x<=result;op<=0;a<=avg_y;b<={16'd0,mean_range}+(diameter_mm>>1);math_start<=1;state<=12;end
    12:if(math_done)begin op<=1;a<=result;b<={16'd0,mean_range};math_start<=1;state<=13;end
    13:if(math_done)begin
     if(candidate_count<8)begin candidate_xs[candidate_count*16+:16]<=center_x;candidate_ys[candidate_count*16+:16]<=result[15:0];candidate_count<=candidate_count+1'b1;end
     else over<=1;
     state<=15;
    end
    15:begin
     if(return_mode==2)begin load_first();return_mode<=1;state<=2;end
     else state<=return_mode==1?14:0;
    end
    14:begin done<=1;scans<=scans+1'b1;scan_ok<=!over;count<=candidate_count;xs<=candidate_xs;ys<=candidate_ys;
     candidate_count<=0;candidate_xs<=0;candidate_ys<=0;over<=0;state<=0;
    end
    default:state<=0;
   endcase
  end
 end
endmodule
