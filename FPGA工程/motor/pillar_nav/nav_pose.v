// Three-landmark registration, with a learned (not assumed 2m) triangle.
// Manual freeze/O/X/Y selection removes initial data-association guesswork.
// Runtime correspondence uses all three pair distances and positive chirality;
// equally good competing constellations are rejected, never silently selected.
module nav_pose(input wire clk,rst_n,clear,
 input wire scan_done,scan_ok,input wire [3:0] count,input wire [191:0] xs,ys,
 input wire learn_req,input wire [3:0] frozen_count,o_id,x_id,y_id,
 input wire [191:0] frozen_xs,frozen_ys,
 output reg learn_done,learn_ok,template_valid,pose_update,pose_ok,busy,
 output reg signed [15:0] pose_x,pose_y,pose_yaw,
 output reg [15:0] width_mm,height_mm,bx_mm,quality,
 output reg ambiguous);
 reg signed [15:0] px[0:7],py[0:7];
 reg [15:0] len1,len2,len3;
 reg [1:0] leg;
 reg [2:0] endpoint0,endpoint1;
 reg [3:0] n;
 reg [2:0] mi,mj,i,j,k,oi,ai,bi,best_o,best_a,best_b;
 reg [7:0] state;reg learning,learn_seen;
 reg [15:0] length_oa,length_ob,length_ab,best,second,trial_score;
 reg signed [31:0] dx,dy,ax,ay,bx,by,ux,uy,ox,oy,world_x,world_y,world_bx,world_by;
 reg signed [63:0] temp,det_first;
 reg math_start;reg [1:0] op;reg signed [63:0] a;reg signed [31:0] b;
 wire math_done;wire signed [63:0] result;
 reg atan_start;wire atan_done;wire signed [15:0] axis_angle;
 integer q;
 function integer ix(input [3:0] p,input [3:0] t);begin ix=(p<<3)+t;end endfunction
 function [15:0] difference(input [15:0] p,input [15:0] t);begin difference=p>t?p-t:t-p;end endfunction
 function signed [31:0] sub16(input signed [15:0] p,input signed [15:0] t);
 begin sub16=$signed({{16{p[15]}},p})-$signed({{16{t[15]}},t});end endfunction
 wire [15:0] e1=difference(len1,length_oa);
 wire [15:0] e2=difference(len2,length_ob);
 wire [15:0] e3=difference(len3,length_ab);
 wire [17:0] score={2'd0,e1}+{2'd0,e2}+{2'd0,e3};
 nav_math math(.clk(clk),.rst_n(rst_n),.cancel(clear),.start(math_start),.op(op),.a(a),.b(b),
  .busy(),.done(math_done),.zero_divide(),.result(result));
 nav_atan2 atan(.clk(clk),.rst_n(rst_n),.start(atan_start),.in_x(ux<<<8),.in_y(uy<<<8),
  .done(atan_done),.angle(axis_angle),.magnitude());
 task next_triple;
 begin
  state<=7;
  if(k+1<n)k<=k+1'b1;
  else begin k<=0;if(j+1<n)j<=j+1'b1;
   else begin j<=0;if(i+1<n)i<=i+1'b1;else state<=15;end
  end
 end
 endtask
 task reject;
 begin
  busy<=0;state<=0;pose_ok<=0;
  if(learning)begin learn_done<=1;learn_ok<=0;end else pose_update<=1;
 end
 endtask
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin state<=0;busy<=0;template_valid<=0;learn_done<=0;learn_ok<=0;pose_update<=0;pose_ok<=0;
   pose_x<=0;pose_y<=0;pose_yaw<=0;width_mm<=2000;height_mm<=2000;bx_mm<=0;quality<=65535;ambiguous<=0;
   n<=0;mi<=0;mj<=1;i<=0;j<=0;k<=0;oi<=0;ai<=0;bi<=0;best_o<=0;best_a<=0;best_b<=0;
   learning<=0;learn_seen<=0;length_oa<=0;length_ob<=0;length_ab<=0;best<=65535;second<=65535;trial_score<=0;
   dx<=0;dy<=0;ax<=0;ay<=0;bx<=0;by<=0;ux<=0;uy<=0;ox<=0;oy<=0;
   world_x<=0;world_y<=0;world_bx<=0;world_by<=0;temp<=0;det_first<=0;
   math_start<=0;op<=0;a<=0;b<=0;atan_start<=0;len1<=0;len2<=0;len3<=0;leg<=0;endpoint0<=0;endpoint1<=0;
  end else begin
   math_start<=0;atan_start<=0;pose_update<=0;learn_done<=0;
   if(!learn_req)learn_seen<=0;
   if(clear)begin state<=0;busy<=0;template_valid<=0;pose_ok<=0;learn_ok<=0;end
   else case(state)
    0:begin
     if(learn_req && !learn_seen)begin
      learn_seen<=1;learning<=1;learn_ok<=0;template_valid<=0;pose_ok<=0;ambiguous<=0;
      if(frozen_count<3 || frozen_count>8 || o_id>=frozen_count || x_id>=frozen_count || y_id>=frozen_count ||
         o_id==x_id || o_id==y_id || x_id==y_id)begin learn_done<=1;end
      else begin
       n<=frozen_count;oi<=o_id;ai<=x_id;bi<=y_id;busy<=1;state<=1;
       for(q=0;q<8;q=q+1)begin px[q]<=frozen_xs[q*16+:16];py[q]<=frozen_ys[q*16+:16];end
      end
     end else if(scan_done && template_valid)begin
      learning<=0;ambiguous<=0;
      if(!scan_ok || count<3 || count>8)begin pose_ok<=0;pose_update<=1;end
      else begin n<=count;busy<=1;state<=1;
       for(q=0;q<8;q=q+1)begin px[q]<=xs[q*16+:16];py[q]<=ys[q*16+:16];end
      end
     end
    end
    // Compute only the three distances needed by the current hypothesis.
    // No multi-ported all-pairs RAM/mux fabric: ~2ms budget at 8 candidates.
    1:begin i<=0;j<=1;k<=2;best<=65535;second<=65535;
     if(learning)begin i<=oi;j<=ai;k<=bi;end
     state<=7;
    end
    7:begin
     if(i==j || i==k || j==k)next_triple();
     else begin leg<=0;endpoint0<=i;endpoint1<=j;state<=2;end
    end
    2:begin dx<=sub16(px[endpoint0],px[endpoint1]);dy<=sub16(py[endpoint0],py[endpoint1]);state<=3;end
    3:begin op<=0;a<=dx;b<=dx;math_start<=1;state<=4;end
    4:if(math_done)begin temp<=result;op<=0;a<=dy;b<=dy;math_start<=1;state<=5;end
    5:if(math_done)begin op<=2;a<=temp+result;b<=0;math_start<=1;state<=6;end
    6:if(math_done)begin
     case(leg)
      0:begin len1<=result[15:0];endpoint0<=i;endpoint1<=k;leg<=1;state<=2;end
      1:begin len2<=result[15:0];endpoint0<=j;endpoint1<=k;leg<=2;state<=2;end
      2:begin len3<=result[15:0];state<=learning?20:10;end
     endcase
    end
    10:begin
     if(i!=j && i!=k && j!=k && e1<=100 && e2<=100 && e3<=100 && score<=200)begin
      ax<=sub16(px[j],px[i]);ay<=sub16(py[j],py[i]);bx<=sub16(px[k],px[i]);by<=sub16(py[k],py[i]);
      trial_score<=score[15:0];state<=11;
     end else next_triple();
    end
    11:begin op<=0;a<=ax;b<=by;math_start<=1;state<=12;end
    12:if(math_done)begin det_first<=result;op<=0;a<=ay;b<=bx;math_start<=1;state<=13;end
    13:if(math_done)begin
     if(det_first>result)begin
      if(trial_score<best)begin second<=best;best<=trial_score;best_o<=i;best_a<=j;best_b<=k;end
      else if(trial_score<second)second<=trial_score;
     end
     next_triple();
    end
    15:begin
     quality<=best;
     if(best==65535 || {1'b0,second}<={1'b0,best}+17'd30)begin ambiguous<=best!=65535;reject();end
     else begin oi<=best_o;ai<=best_a;bi<=best_b;i<=best_o;j<=best_a;k<=best_b;state<=16;end
    end
    16:begin leg<=0;endpoint0<=i;endpoint1<=j;learning<=0;state<=17;end
    17:begin dx<=sub16(px[endpoint0],px[endpoint1]);dy<=sub16(py[endpoint0],py[endpoint1]);state<=18;end
    18:begin op<=0;a<=dx;b<=dx;math_start<=1;state<=19;end
    19:if(math_done)begin temp<=result;op<=0;a<=dy;b<=dy;math_start<=1;state<=41;end
    41:if(math_done)begin op<=2;a<=temp+result;b<=0;math_start<=1;state<=42;end
    42:if(math_done)begin
     case(leg)
      0:begin len1<=result[15:0];endpoint0<=i;endpoint1<=k;leg<=1;state<=17;end
      1:begin len2<=result[15:0];endpoint0<=j;endpoint1<=k;leg<=2;state<=17;end
      2:begin len3<=result[15:0];state<=20;end
     endcase
    end
    20:begin
     ax<=sub16(px[ai],px[oi]);ay<=sub16(py[ai],py[oi]);bx<=sub16(px[bi],px[oi]);by<=sub16(py[bi],py[oi]);
     ox<=px[oi];oy<=py[oi];state<=21;
    end
    21:begin op<=0;a<=ax;b<=by;math_start<=1;state<=22;end
    22:if(math_done)begin det_first<=result;op<=0;a<=ay;b<=bx;math_start<=1;state<=23;end
    23:if(math_done)begin
     if(det_first<=result || len1<500 || len1>3000 || len2<500 || len2>3500)reject();
     else state<=24;
    end
    24:begin op<=1;a<=$signed({{32{ax[31]}},ax})<<<14;b<={16'd0,len1};math_start<=1;state<=25;end
    25:if(math_done)begin ux<=result;op<=1;a<=$signed({{32{ay[31]}},ay})<<<14;b<={16'd0,len1};math_start<=1;state<=26;end
    26:if(math_done)begin uy<=result;state<=27;end
    27:begin op<=0;a<=ux;b<=ox;math_start<=1;state<=28;end
    28:if(math_done)begin temp<=result;op<=0;a<=uy;b<=oy;math_start<=1;state<=29;end
    29:if(math_done)begin world_x<=-((temp+result)>>>14);state<=30;end
    30:begin op<=0;a<=-uy;b<=ox;math_start<=1;state<=31;end
    31:if(math_done)begin temp<=result;op<=0;a<=ux;b<=oy;math_start<=1;state<=32;end
    32:if(math_done)begin world_y<=-((temp+result)>>>14);state<=33;end
    33:begin op<=0;a<=ux;b<=bx;math_start<=1;state<=34;end
    34:if(math_done)begin temp<=result;op<=0;a<=uy;b<=by;math_start<=1;state<=35;end
    35:if(math_done)begin world_bx<=(temp+result)>>>14;state<=36;end
    36:begin op<=0;a<=-uy;b<=bx;math_start<=1;state<=37;end
    37:if(math_done)begin temp<=result;op<=0;a<=ux;b<=by;math_start<=1;state<=38;end
    38:if(math_done)begin world_by<=(temp+result)>>>14;state<=39;end
    39:begin atan_start<=1;state<=40;end
    40:if(atan_done)begin
     if(learning)begin
      if(world_by<500 || world_by>3000 || world_bx>$signed({16'd0,len1})/4 || world_bx<-$signed({16'd0,len1})/4)reject();
      else begin template_valid<=1;length_oa<=len1;length_ob<=len2;length_ab<=len3;
       width_mm<=len1;height_mm<=world_by;bx_mm<=world_bx;learn_ok<=1;learn_done<=1;busy<=0;state<=0;end
     end else begin
      pose_update<=1;pose_ok<=world_x>=-200 && world_y>=-200 && world_x<=$signed({16'd0,width_mm})+200 && world_y<=$signed({16'd0,height_mm})+200;
      pose_x<=world_x;pose_y<=world_y;pose_yaw<=-axis_angle;busy<=0;state<=0;
     end
    end
    default:begin state<=0;busy<=0;end
   endcase
  end
 end
endmodule
