// 128x128 bitmap + 32-pixel status sidebar, matching the existing TFT180.
// Bitmap zero means no observed endpoint, NOT known free space.
module nav_scan_renderer #(
 parameter integer CELL_SHIFT=6
)(input wire clk,rst_n,frame_start,
 input wire [7:0] pixel_x,input wire [6:0] pixel_y,
 input wire scan_bit,front_valid,online,
 input wire [31:0] packets,crc_errors,drops,scans,
 output wire [13:0] scan_addr,output reg [15:0] pixel);
 assign scan_addr={pixel_y,pixel_x[6:0]};
 reg [7:0] x;
 reg [6:0] y;
 reg live,has_scan;
 reg [15:0] packet_frame,crc_frame,drop_frame,scan_frame;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin x<=0;y<=0;live<=0;has_scan<=0;packet_frame<=0;crc_frame<=0;drop_frame<=0;scan_frame<=0;end
  else begin
   x<=pixel_x;y<=pixel_y;
   if(frame_start)begin live<=online;has_scan<=front_valid;packet_frame<=packets[15:0];
    crc_frame<=crc_errors[15:0];drop_frame<=drops[15:0];scan_frame<=scans[15:0];end
  end
 end
 function [7:0] hexchar(input [3:0] value);begin hexchar=value<10?"0"+value:"A"+value-10;end endfunction
 function [31:0] hexword(input [15:0] value);
  begin hexword={hexchar(value[15:12]),hexchar(value[11:8]),hexchar(value[7:4]),hexchar(value[3:0])};end
 endfunction
 reg [31:0] text;
 reg [7:0] character;
 wire font_bit;
 always @*begin
  case(y[6:3])
   0:text="N10 ";
   1:text=!live ? (has_scan?"OLD ":"WAIT") : (has_scan?"LIVE":"SYNC");
   2:text="SCAN";3:text=hexword(scan_frame);
   4:text="PKT ";5:text=hexword(packet_frame);
   6:text="CRC ";7:text=hexword(crc_frame);
   8:text="DROP";9:text=hexword(drop_frame);
   10:text=CELL_SHIFT==6?"64MM":"ZOOM";
   11:text="0 UP";12:text="90 R";13:text="S1> ";14:text="NO  ";15:text="SLAM";
   default:text="    ";
  endcase
  case(x[4:3])0:character=text[31:24];1:character=text[23:16];2:character=text[15:8];default:character=text[7:0];endcase
 end
 reg [7:0] x_q,character_q;reg [6:0] y_q;reg scan_q;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin x_q<=0;y_q<=0;character_q<=32;scan_q<=0;end
  else begin x_q<=x;y_q<=y;character_q<=character;scan_q<=scan_bit;end
 end
 tft180_font font(.ascii(character_q),.x(x_q[2:0]),.y(y_q[2:0]),.pixel(font_bit));
 always @*begin
  pixel=16'h0000;
  if(x_q<128)begin
   if(has_scan && scan_q)pixel=live?16'h07ff:16'h7bef;
   else if(x_q[3:0]==0 || y_q[3:0]==0)pixel=16'h1082;
   if(x_q==0 || x_q==127 || y_q==0 || y_q==127)pixel=16'h39e7;
   if((x_q>=61 && x_q<=67 && y_q==64)||(y_q>=61 && y_q<=67 && x_q==64))pixel=16'hf800;
   if((x_q==64 && y_q>=4 && y_q<=10)||(y_q==7 && (x_q==62||x_q==66)))pixel=16'hffe0;
  end else if(font_bit)begin
   if(y_q<8)pixel=16'h07ff;
   else if(y_q<16)pixel=live && has_scan?16'h07e0:16'hf800;
   else pixel=16'hffff;
  end
 end
endmodule
