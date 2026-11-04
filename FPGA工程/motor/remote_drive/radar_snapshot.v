// PC visualization: 180 native-clockwise 2-degree bins, 20mm quantization.
// Two RAM banks; the reader's bank is never overwritten while queued/sending.
module radar_snapshot #(parameter integer CLK_HZ=50000000)(
 input wire clk,rst_n,bad,point_valid,input wire [15:0] angle,range_mm,
 input wire take,read_busy,input wire [7:0] read_addr,
 output reg [7:0] read_data,output reg ready,output reg [15:0] scan_sequence);
 reg [7:0] memory[0:511];reg writer,reader,clearing,collect,previous_valid;
 reg [7:0] clear_index;reg [15:0] previous_angle;reg [9:0] count;reg [31:0] age;
 wire wrap_scan=previous_valid && previous_angle>angle+17'd18000;
 wire [7:0] bin=angle/16'd200;
 wire [15:0] quantized=range_mm/16'd20;
 wire good=range_mm>=50 && range_mm<=5100 && angle<36000;
 wire publish=wrap_scan && collect && count>=120 && !ready && !read_busy && age>=CLK_HZ/5;
 always @(posedge clk)begin
  read_data<=memory[{reader,read_addr}];
  if(rst_n)begin
   if(clearing)memory[{writer,clear_index}]<=0;
   else if(point_valid && collect && good && !wrap_scan)memory[{writer,bin}]<=quantized[7:0];
  end
 end
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin writer<=0;reader<=1;clearing<=1;clear_index<=0;collect<=0;previous_valid<=0;previous_angle<=0;count<=0;age<=0;ready<=0;scan_sequence<=0;end
  else begin
   if(age<CLK_HZ)age<=age+1'b1;
   if(take)ready<=0;
   if(clearing)begin if(clear_index==179)clearing<=0;else clear_index<=clear_index+1'b1;end
   if(bad)begin previous_valid<=0;collect<=0;ready<=0;clearing<=1;clear_index<=0;count<=0;end
   else if(point_valid)begin
    previous_valid<=1;previous_angle<=angle;
    if(wrap_scan)begin
     if(publish)begin reader<=writer;writer<=~writer;ready<=1;scan_sequence<=scan_sequence+1'b1;age<=0;end
     collect<=1;count<=0;clearing<=1;clear_index<=0;
    end else if(!clearing && collect && count<1023)count<=count+1'b1;
   end
  end
 end
endmodule
