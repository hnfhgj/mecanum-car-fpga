// Adjustable independent body-axis cones. Defaults come from the UI registers.
module directional_clearance #(parameter integer CLK_HZ=50000000)(
 input wire clk,rst_n,bad,point_valid,input wire signed [15:0] forward_mm,left_mm,
 input wire [15:0] angle_cdeg,range_mm,output wire fresh,output wire [3:0] blocked,
 input wire [15:0] stop_fb_mm,stop_lr_mm,input wire [1:0] cone_mode);
 reg previous_valid,collect;reg [15:0] previous_angle;reg [3:0] live,committed;
 reg [9:0] count;reg [31:0] age;
 reg pv1,pv2,good1,good2,front1,left1;reg [15:0] angle1,angle2,af1,al1;reg [3:0] hit2;
 wire [15:0] af=forward_mm[15]?(~forward_mm+16'd1):forward_mm;
 wire [15:0] al=left_mm[15]?(~left_mm+16'd1):left_mm;
 wire [19:0] afx={4'd0,af1},alx={4'd0,al1};
 wire longitudinal=cone_mode==1?(alx<<4)<=((afx<<1)+afx):
  cone_mode==3?(alx<<4)<=((afx<<3)+afx):(alx<<3)<=((afx<<1)+afx);
 wire transverse=cone_mode==1?(afx<<4)<=((alx<<1)+alx):
  cone_mode==3?(afx<<4)<=((alx<<3)+alx):(afx<<3)<=((alx<<1)+alx);
 wire wrap_scan=previous_valid && previous_angle>angle2+17'd18000;
 assign fresh=age<CLK_HZ/4;
 assign blocked=fresh?(live|committed):4'b1111;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin previous_valid<=0;previous_angle<=0;collect<=0;count<=0;age<=32'hffffffff;live<=0;committed<=0;
   pv1<=0;pv2<=0;good1<=0;good2<=0;front1<=0;left1<=0;angle1<=0;angle2<=0;af1<=0;al1<=0;hit2<=0;
  end else begin
   pv1<=point_valid;pv2<=pv1;good1<=range_mm>=50 && range_mm<=5000;good2<=good1;
   angle1<=angle_cdeg;angle2<=angle1;af1<=af;al1<=al;front1<=forward_mm>0;left1<=left_mm>0;
   hit2<={transverse && !left1 && al1!=0 && al1<=stop_lr_mm,
          transverse && left1 && al1<=stop_lr_mm,
          longitudinal && !front1 && af1!=0 && af1<=stop_fb_mm,
          longitudinal && front1 && af1<=stop_fb_mm};
   if(age!=32'hffffffff)age<=age+1'b1;
   if(bad)begin previous_valid<=0;collect<=0;age<=32'hffffffff;live<=0;committed<=0;count<=0;pv1<=0;pv2<=0;end
   else if(pv2)begin
    previous_valid<=1;previous_angle<=angle2;
    if(wrap_scan)begin
     if(collect && count>=120)begin committed<=live;age<=0;end else age<=32'hffffffff;
     collect<=1;count<=0;live<=good2?hit2:4'd0;
    end else if(collect)begin
     if(count<1023)count<=count+1'b1;
     if(good2)live<=live|hit2;
    end
   end
  end
 end
endmodule
