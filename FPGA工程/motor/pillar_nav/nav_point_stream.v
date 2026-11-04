// Elastic point FIFO absorbs the parser's packet bursts. Coordinates: +x=ray0,
// +y=left, a right-handed frame; native N10 angles increase clockwise.
module nav_point_stream(input wire clk,rst_n,bad,input wire point_valid,
 input wire [15:0] angle_cdeg,range_mm,
 input wire ready,output reg valid,output reg signed [15:0] x,y,
 output reg [15:0] angle,range_out,output reg overflow);
 reg [31:0] fifo[0:127];
 reg [6:0] wr,rd;
 reg [7:0] used;
 reg [3:0] state;
 reg [15:0] sample_range,sample_angle;
 reg [9:0] phase;
 wire signed [15:0] sine,cosine;
 wire pop=state==0 && used!=0 && !valid;
 wire push=point_valid && (used<128 || pop);
 reg math_start;reg signed [63:0] a;reg signed [31:0] b;
 wire done;wire signed [63:0] value;
 n10_sincos trig(.phase(phase),.sine(sine),.cosine(cosine));
 nav_math math(.clk(clk),.rst_n(rst_n),.cancel(bad),.start(math_start),.op(2'd0),
  .a(a),.b(b),.busy(),.done(done),.zero_divide(),.result(value));
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin wr<=0;rd<=0;used<=0;state<=0;valid<=0;x<=0;y<=0;angle<=0;
   range_out<=0;sample_range<=0;sample_angle<=0;phase<=0;math_start<=0;a<=0;b<=0;overflow<=0;end
  else begin
   overflow<=0;math_start<=0;
   if(bad)begin wr<=0;rd<=0;used<=0;state<=0;valid<=0;end
   else begin
    if(point_valid && !push)overflow<=1;
    case({push,pop}) 2'b10:used<=used+1'b1;2'b01:used<=used-1'b1;default:begin end endcase
    if(push)begin fifo[wr]<={angle_cdeg,range_mm};wr<=wr+1'b1;end
    if(valid && ready)valid<=0;
    case(state)
     0:if(pop)begin {sample_angle,sample_range}<=fifo[rd];rd<=rd+1'b1;state<=1;end
     1:begin phase<=sample_angle>=35975?10'd0:(sample_angle+16'd25)/16'd50;state<=2;end
     2:begin a<={48'd0,sample_range};b<=cosine;math_start<=1;state<=3;end
     3:if(done)begin x<=value>>>14;state<=4;end
     4:begin a<={48'd0,sample_range};b<=sine;math_start<=1;state<=5;end
     5:if(done)begin y<=-(value>>>14);angle<=sample_angle;range_out<=sample_range;valid<=1;state<=6;end
     6:if(valid && ready)state<=0;
     default:state<=0;
    endcase
   end
  end
 end
endmodule
