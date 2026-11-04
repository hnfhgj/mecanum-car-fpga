// N10 clockwise angle -> screen: 0 up, 90 right. Q14 trig, nearest 0.5 degree.
// Coordinates are cells; CELL_SHIFT=6 means 64 mm/cell and an 8.192 m square.
module n10_project_point #(
 parameter integer CELL_SHIFT=6,
 parameter [15:0] ROTATE_CDEG=0,
 parameter [15:0] MIN_MM=50, MAX_MM=6000
)(input wire clk,rst_n,start,input wire [15:0] angle_cdeg,distance_mm,
 output reg done,in_bounds,output reg [6:0] x,y);
 reg [2:0] state;
 reg [16:0] angle;
 reg [9:0] phase;
 reg [15:0] range_mm;
 wire signed [15:0] sine,cosine;
 reg signed [15:0] sine_q,cosine_q;
 reg signed [32:0] product_s,product_c;
 wire signed [32:0] cell_x=33'sd64+(product_s >>> (14+CELL_SHIFT));
 wire signed [32:0] cell_y=33'sd64-(product_c >>> (14+CELL_SHIFT));
 wire [16:0] normalized=angle>=36000 ? angle-17'd36000 : angle;
 wire [16:0] rounded_phase=(normalized+17'd25)/17'd50;
 n10_sincos lut(.phase(phase),.sine(sine),.cosine(cosine));
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin state<=0;angle<=0;phase<=0;range_mm<=0;sine_q<=0;cosine_q<=0;product_s<=0;product_c<=0;done<=0;in_bounds<=0;x<=0;y<=0;end
  else begin
   done<=0;
   case(state)
    0:if(start)begin angle<={1'b0,angle_cdeg}+{1'b0,ROTATE_CDEG};range_mm<=distance_mm;state<=1;end
    1:begin phase<=rounded_phase>=720 ? 10'd0 : rounded_phase[9:0];state<=2;end
    // Separate ROM selection from multiplication to meet the 50MHz clock.
    2:begin sine_q<=sine;cosine_q<=cosine;state<=3;end
    3:begin product_s<=$signed({1'b0,range_mm})*sine_q;product_c<=$signed({1'b0,range_mm})*cosine_q;state<=4;end
    4:begin
     x<=cell_x[6:0];y<=cell_y[6:0];
     in_bounds<=range_mm>=MIN_MM && range_mm<=MAX_MM && cell_x>=0 && cell_x<128 && cell_y>=0 && cell_y<128;
     done<=1;state<=0;
    end
    default:state<=0;
   endcase
  end
 end
endmodule
