// Radar stays native. S1 toggles the frozen-candidate overlay/configuration;
// status and numeric fields are captured atomically at the TFT frame boundary.
module nav_renderer(input wire clk,rst_n,frame_start,input wire [7:0] pixel_x,input wire [6:0] pixel_y,
 input wire [15:0] raw_pixel,input wire menu_view,input wire [3:0] cursor,status,
 input wire aligned,can_go,learn_ok,template_valid,ambiguous,
 input wire [3:0] live_count,frozen_count,o_id,x_id,y_id,
 input wire [191:0] live_xs,live_ys,frozen_xs,frozen_ys,
 input wire signed [15:0] pose_x,pose_y,input wire [15:0] target_x,target_y,width_mm,height_mm,diameter_mm,quality,
 output reg [15:0] pixel);
 reg [7:0] x;reg [6:0] y;
 reg menu,fr_aligned,fr_ready,fr_learn,fr_template,fr_ambiguous;
 reg [3:0] field,fr_status,n,fo,fa,fb;reg frozen;
 reg [191:0] xs,ys;
 reg [15:0] numbers[0:7];reg [19:0] bcd[0:7];
 reg [3:0] convert_index;reg [4:0] convert_bit;
 reg [15:0] binary;reg [19:0] work,adjusted;
 reg converting,negative_x,negative_y;integer m;
 always @*begin
  adjusted=work;
  for(m=0;m<5;m=m+1)if(work[m*4+:4]>=5)adjusted[m*4+:4]=work[m*4+:4]+4'd3;
 end
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin x<=0;y<=0;menu<=0;fr_aligned<=0;fr_ready<=0;fr_learn<=0;fr_template<=0;fr_ambiguous<=0;
   field<=0;fr_status<=0;n<=0;fo<=0;fa<=0;fb<=0;frozen<=0;xs<=0;ys<=0;convert_index<=0;convert_bit<=0;
   binary<=0;work<=0;converting<=0;negative_x<=0;negative_y<=0;
   for(m=0;m<8;m=m+1)begin numbers[m]<=0;bcd[m]<=0;end
  end else begin
   x<=pixel_x;y<=pixel_y;
   if(frame_start)begin
    menu<=menu_view;field<=cursor;fr_status<=status;fr_aligned<=aligned;fr_ready<=can_go;fr_learn<=learn_ok;fr_template<=template_valid;fr_ambiguous<=ambiguous;
    frozen<=frozen_count!=0;n<=frozen_count!=0?frozen_count:live_count;
    xs<=frozen_count!=0?frozen_xs:live_xs;ys<=frozen_count!=0?frozen_ys:live_ys;
    fo<=o_id;fa<=x_id;fb<=y_id;
    numbers[0]<=pose_x<0?-pose_x:pose_x;numbers[1]<=pose_y<0?-pose_y:pose_y;
    numbers[2]<=target_x;numbers[3]<=target_y;numbers[4]<=width_mm;numbers[5]<=height_mm;
    numbers[6]<=diameter_mm;numbers[7]<=quality;
    negative_x<=pose_x<0;negative_y<=pose_y<0;
    convert_index<=0;convert_bit<=0;work<=0;converting<=1;
    binary<=pose_x<0?-pose_x:pose_x;
   end else if(converting)begin
    work<={adjusted[18:0],binary[15]};binary<={binary[14:0],1'b0};
    if(convert_bit==15)begin
     bcd[convert_index]<={adjusted[18:0],binary[15]};
     convert_bit<=0;work<=0;
     if(convert_index==7)converting<=0;
     else begin convert_index<=convert_index+1'b1;binary<=numbers[convert_index+1'b1];end
    end else convert_bit<=convert_bit+1'b1;
   end
  end
 end
 function [7:0] hexchar(input [3:0] v);begin hexchar=v<10?8'd48+v:8'd65+v-10;end endfunction
 function [23:0] CM3(input [19:0] v);begin CM3={hexchar(v[15:12]),hexchar(v[11:8]),hexchar(v[7:4])};end endfunction
 reg [159:0] line;reg [7:0] character;wire glyph;
 reg [3:0] row;reg label_hit;integer j;
 integer cx,cy;reg signed [15:0] lx,ly;
 reg [2:0] font_x,font_y;
 reg [7:0] character_q;reg [2:0] font_x_q,font_y_q;
 reg menu_q,label_q,selected_q,frozen_q;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin character_q<=32;font_x_q<=0;font_y_q<=0;menu_q<=0;label_q<=0;selected_q<=0;frozen_q<=0;end
  else begin character_q<=character;font_x_q<=font_x;font_y_q<=font_y;
   menu_q<=menu;label_q<=label_hit;selected_q<=row>=3 && row<=12 && row==field+2;frozen_q<=frozen;end
 end
 tft180_font font(.ascii(character_q),.x(font_x_q),.y(font_y_q),.pixel(glyph));
 always @*begin
  row=y[6:3];line="                    ";
  case(row)
   0:begin
    case(fr_status)
     0:line="NAV: LEARN 3 POLES  ";1:line="NAV: ALIGN FRONT +X ";2:line="NAV: WAIT FIX / IMU ";
     3:line="NAV: READY          ";4:line="NAV: RUNNING        ";5:line="NAV: ARRIVED        ";
     6:line="NAV: LOST / STOPPED ";7:line="NAV: STOPPED        ";default:line="NAV: WAIT           ";
    endcase
   end
   1:begin line="P X:000 Y:000 CM    ";line[127-:24]=CM3(bcd[0]);line[79-:24]=CM3(bcd[1]);
    if(negative_x)line[135-:8]="-";if(negative_y)line[87-:8]="-";end
   2:begin line="T X:000 Y:000 CM    ";line[127-:24]=CM3(bcd[2]);line[79-:24]=CM3(bcd[3]);end
   3:begin line="1 FREEZE  N:0       ";line[63-:8]=hexchar(n);end
   4:begin line="2 ORIGIN O ID:0     ";line[47-:8]=hexchar(fo);end
   5:begin line="3 X AXIS A ID:1     ";line[47-:8]=hexchar(fa);end
   6:begin line="4 Y SIDE B ID:2     ";line[47-:8]=hexchar(fb);end
   7:begin line="5 DIAMETER:000 CM   ";line[71-:24]=CM3(bcd[6]);end
   8:begin line="6 LEARN 000X000 CM  ";line[95-:24]=CM3(bcd[4]);line[63-:24]=CM3(bcd[5]);end
   9:line="7 ALIGN FRONT TO +X ";
   10:begin line="8 GOAL X:000 CM     ";line[87-:24]=CM3(bcd[2]);end
   11:begin line="9 GOAL Y:000 CM     ";line[87-:24]=CM3(bcd[3]);end
   12:line=fr_ready?"A GO / S4 STOP      ":"A GO: NOT READY     ";
   13:line=fr_ambiguous?"AMBIGUOUS 3-POLE FIX":(!fr_template?"SELECT O A B; LEARN ":(!fr_aligned?"FACE +X THEN ALIGN  ":"UNIT:5CM TOL:7.5CM "));
   14:line="S2 NEXT S3+ S4-STOP ";
   15:line="S1 MAP  HOLD:PAGE   ";
   default:line="                    ";
  endcase
  character=x<160?line[(19-x[7:3])*8+:8]:8'd32;font_x=x[2:0];font_y=y[2:0];
  label_hit=0;cx=0;cy=0;lx=0;ly=0;
  if(!menu)for(j=0;j<8;j=j+1)begin
   lx=xs[j*16+:16];ly=ys[j*16+:16];cx=64-($signed(ly)>>>6);cy=64-($signed(lx)>>>6);
   if(j<n && cx>=0 && cx<120 && cy>=0 && cy<120 && x>=cx && x<cx+8 && y>=cy && y<cy+8)begin
    label_hit=1;character=hexchar(j);font_x=x-cx;font_y=y-cy;
   end
  end
  if(menu_q)pixel=glyph?(selected_q?16'hffe0:16'hffff):16'h0000;
  else pixel=label_q && glyph?(frozen_q?16'hffe0:16'h07e0):raw_pixel;
 end
endmodule
