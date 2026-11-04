// Native (forward,left) sensor point -> nominal world (+X right,+Y forward).
// angle_cdeg is the CCW yaw relative to nominal +Y, INCLUDING mounting offset.
module square_project(input wire clk,rst_n,cancel,start,
 input wire signed [15:0] forward_mm,left_mm,angle_cdeg,
 output reg busy,done,output reg signed [15:0] x,y);
 reg [3:0] state;reg signed [15:0] f,l,s,c;reg [9:0] phase;
 wire signed [15:0] sn,cs;
 reg request;reg signed [63:0] a,first;reg signed [31:0] b;
 wire md;wire signed [63:0] result;
 wire signed [31:0] angle_ext=angle_cdeg;
 wire [16:0] positive=angle_cdeg<0?angle_ext+36000:angle_ext;
 n10_sincos trig(.phase(phase),.sine(sn),.cosine(cs));
 nav_math mul(.clk(clk),.rst_n(rst_n),.cancel(cancel),.start(request),.op(2'd0),.a(a),.b(b),
  .busy(),.done(md),.zero_divide(),.result(result));
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin busy<=0;done<=0;state<=0;x<=0;y<=0;f<=0;l<=0;s<=0;c<=0;phase<=0;
   request<=0;a<=0;b<=0;first<=0;end
  else begin
   done<=0;request<=0;
   if(cancel)begin state<=0;busy<=0;end
   else case(state)
    0:if(start)begin busy<=1;f<=forward_mm;l<=left_mm;
     phase<=positive>=35975?0:(positive+17'd25)/17'd50;state<=1;end
    1:begin s<=sn;c<=cs;state<=2;end
    2:begin a<=-f;b<=s;request<=1;state<=3;end
    3:if(md)begin first<=result;a<=l;b<=c;request<=1;state<=4;end
    4:if(md)begin x<=(first-result)>>>14;a<=f;b<=c;request<=1;state<=5;end
    5:if(md)begin first<=result;a<=l;b<=s;request<=1;state<=6;end
    6:if(md)begin y<=(first-result)>>>14;done<=1;busy<=0;state<=0;end
    default:begin state<=0;busy<=0;end
   endcase
  end
 end
endmodule
