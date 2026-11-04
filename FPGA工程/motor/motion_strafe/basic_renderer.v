// Twenty columns, sixteen rows; snapshot numbers at display frame boundary.
module basic_renderer(input wire clk,rst_n,frame_start,input wire [7:0] pixel_x,input wire [6:0] pixel_y,
 input wire [15:0] raw_pixel,input wire menu_view,input wire [2:0] group_id,input wire [3:0] cursor,
 input wire [3:0] demo_status,mission_status,input wire [1:0] edit_index,point_index,input wire route_mode,
 input wire camera_ready,imu_ready,range_ready,calibrated,near_stop,slow_seen,stop_seen,
 input wire signed [15:0] pose_x,pose_y,demo_yaw,demo_target,
 input wire [15:0] target_x,target_y,width_mm,height_mm,front_mm,back_mm,left_mm,right_mm,
 input wire [63:0] points_x,points_y,input wire [6:0] front_limit,output reg [15:0] pixel);
 reg [2:0] group_q;reg [1:0] edit_q,point_q;
 reg cam_q,imu_q,range_q,cal_q,near_q,slow_q,stop_q,route_q,yaw_negative,target_negative;
 reg [7:0] x;reg [6:0] y;
 reg menu,fr_aligned,fr_ready,fr_learn,fr_template,fr_ambiguous;
 reg [3:0] field,fr_status,n,fo,fa,fb;reg frozen;
 reg [191:0] xs,ys;
 reg [15:0] numbers[0:15];reg [19:0] bcd[0:15];
 reg [3:0] convert_index;reg [4:0] convert_bit;
 reg [15:0] binary;reg [19:0] work,adjusted;
 reg converting,negative_x,negative_y;integer m;
 always @*begin
  adjusted=work;
  for(m=0;m<5;m=m+1)if(work[m*4+:4]>=5)adjusted[m*4+:4]=work[m*4+:4]+4'd3;
 end
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin group_q<=0;edit_q<=0;point_q<=0;cam_q<=0;imu_q<=0;range_q<=0;cal_q<=0;
   near_q<=0;slow_q<=0;stop_q<=0;route_q<=0;yaw_negative<=0;target_negative<=0;x<=0;y<=0;menu<=0;fr_aligned<=0;fr_ready<=0;fr_learn<=0;fr_template<=0;fr_ambiguous<=0;
   field<=0;fr_status<=0;n<=0;fo<=0;fa<=0;fb<=0;frozen<=0;xs<=0;ys<=0;convert_index<=0;convert_bit<=0;
   binary<=0;work<=0;converting<=0;negative_x<=0;negative_y<=0;
   for(m=0;m<16;m=m+1)begin numbers[m]<=0;bcd[m]<=0;end
  end else begin
   x<=pixel_x;y<=pixel_y;
   if(frame_start)begin
    menu<=menu_view;field<=cursor;group_q<=group_id;edit_q<=edit_index;point_q<=point_index;
    cam_q<=camera_ready;imu_q<=imu_ready;range_q<=range_ready;cal_q<=calibrated;
    near_q<=near_stop;slow_q<=slow_seen;stop_q<=stop_seen;route_q<=route_mode;
    fr_status<=(group_id==2 || group_id==3)?demo_status:mission_status;
    numbers[0]<=pose_x<0?-pose_x:pose_x;numbers[1]<=pose_y<0?-pose_y:pose_y;
    numbers[2]<=target_x;numbers[3]<=target_y;numbers[4]<=width_mm;numbers[5]<=height_mm;
    numbers[6]<=front_mm;numbers[7]<=back_mm;numbers[8]<=left_mm;numbers[9]<=right_mm;
    numbers[10]<=demo_yaw<0?-demo_yaw:demo_yaw;numbers[11]<=demo_target<0?-demo_target:demo_target;
    numbers[12]<=points_x[edit_index*16+:16];numbers[13]<=points_y[edit_index*16+:16];
    numbers[14]<={9'd0,front_limit};numbers[15]<=0;
    yaw_negative<=demo_yaw<0;target_negative<=demo_target<0;
    convert_index<=0;convert_bit<=0;work<=0;converting<=1;
    binary<=pose_x<0?-pose_x:pose_x;
   end else if(converting)begin
    work<={adjusted[18:0],binary[15]};binary<={binary[14:0],1'b0};
    if(convert_bit==15)begin
     bcd[convert_index]<={adjusted[18:0],binary[15]};
     convert_bit<=0;work<=0;
     if(convert_index==15)converting<=0;
     else begin convert_index<=convert_index+1'b1;binary<=numbers[convert_index+1'b1];end
    end else convert_bit<=convert_bit+1'b1;
   end
  end
 end
 function [7:0] digit(input [3:0] v);begin digit=8'd48+v;end endfunction
 function [23:0] cm(input [19:0] v);begin cm={digit(v[15:12]),digit(v[11:8]),digit(v[7:4])};end endfunction
 function [23:0] deg(input [19:0] v);begin deg={digit(v[19:16]),digit(v[15:12]),digit(v[11:8])};end endfunction
 function [23:0] integer3(input [19:0] v);begin integer3={digit(v[11:8]),digit(v[7:4]),digit(v[3:0])};end endfunction
 reg [159:0] line;reg [7:0] character;reg [2:0] font_x,font_y;
 reg [7:0] character_q;reg [2:0] font_x_q,font_y_q;reg menu_q,selected_q;
 wire glyph;wire [3:0] row=y[6:3];
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin character_q<=32;font_x_q<=0;font_y_q<=0;menu_q<=0;selected_q<=0;end
  else begin character_q<=character;font_x_q<=font_x;font_y_q<=font_y;menu_q<=menu;
   selected_q<=row>=2 && row==field+1;end
 end
 tft180_font font(.ascii(character_q),.x(font_x_q),.y(font_y_q),.pixel(glyph));
 always @*begin
  line="                    ";
  case(group_q)
   0:case(row)
    0:begin line="BASIC TASK MENU     ";end
    2:begin line="1 SENSOR DISPLAY    ";end
    3:begin line="2 MOTION TESTS      ";end
    4:begin line="3 OBSTACLE TESTS    ";end
    5:begin line="4 NAVIGATION        ";end
    6:begin line="5 HEADING P/D       ";end
    7:begin line="6 LIVE RADAR        ";end
    8:begin line="BODY:30X16 CM       ";end
    11:begin line="NO AUTO START       ";end
    12:begin line="SELECT ONE TASK     ";end
    9:begin line="CAM:0 IMU:0 LDR:0   ";line[159-4*8-:8]=cam_q?"1":"0";line[159-10*8-:8]=imu_q?"1":"0";line[159-16*8-:8]=range_q?"1":"0";end
    10:begin line="FRAME:0 RANGE:0     ";line[159-6*8-:8]=cal_q?"1":"0";line[159-14*8-:8]=range_q?"1":"0";end
    default:begin end
   endcase
   1:case(row)
    0:begin line="TASK1 SENSORS       ";end
    2:begin line="1 CAMERA            ";end
    3:begin line="2 ANGLES/ENC/TOF    ";end
    4:begin line="3 RAW IMU           ";end
    5:begin line="4 CONTROL MONITOR   ";end
    6:begin line="5 BACK              ";end
    9:begin line="VIEW THEN S1 BACK   ";end
    default:begin end
   endcase
   2:case(row)
    0:begin line="TASK2 MOTION        ";end
    2:begin line="1 FORWARD 4S        ";end
    3:begin line="2 REVERSE 4S        ";end
    4:begin line="3 STRAFE LEFT 4S    ";end
    5:begin line="4 STRAFE RIGHT 4S   ";end
    6:begin line="5 SPIN LEFT 90DEG   ";end
    7:begin line="6 SPIN RIGHT 90DEG  ";end
    8:begin line="7 BACK              ";end
    10:begin line="STRAFE / SPIN90 DEG ";end
    11:begin line="4 WHEEL SPEED PID   ";end
    9:begin line="YAW:+DDD TO:+EEE    ";line[159-5*8-:24]=deg(bcd[10]);line[159-13*8-:24]=deg(bcd[11]);line[159-4*8-:8]=yaw_negative?"-":"+";line[159-12*8-:8]=target_negative?"-":"+";end
    default:begin end
   endcase
   3:case(row)
    0:begin line="TASK3 OBSTACLE      ";end
    2:begin line="1 LIVE DISTANCES    ";end
    3:begin line="2 APPROACH SLOW/STOP";end
    4:begin line="3 BYPASS: NAV MENU  ";end
    5:begin line="4 BACK              ";end
    11:begin line="700/550/400/270MM   ";end
    12:begin line="LIMIT 60/45/30/15/0 ";end
    6:begin line="F:FFF B:BBB CM      ";line[159-2*8-:24]=cm(bcd[6]);line[159-8*8-:24]=cm(bcd[7]);end
    7:begin line="L:LLL R:RRR CM      ";line[159-2*8-:24]=cm(bcd[8]);line[159-8*8-:24]=cm(bcd[9]);end
    8:begin line="FRONT LIMIT:SSS     ";line[159-12*8-:24]=integer3(bcd[14]);end
    9:begin line="SLOW:0 STOP:0       ";line[159-5*8-:8]=slow_q?"1":"0";line[159-12*8-:8]=stop_q?"1":"0";end
    10:begin line="RANGE:0 NEAR:0      ";line[159-6*8-:8]=range_q?"1":"0";line[159-13*8-:8]=near_q?"1":"0";end
    default:begin end
   endcase
   4:case(row)
    0:begin line="TASK4 NAVIGATION    ";end
    2:begin line="1 CALIBRATE FRAME   ";end
    5:begin line="4 GO SINGLE         ";end
    6:begin line="5 EDIT 4 POINTS     ";end
    7:begin line="6 RUN 4 POINTS      ";end
    8:begin line="7 BACK              ";end
    3:begin line="2 GOAL X:XXX CM     ";line[159-9*8-:24]=cm(bcd[2]);end
    4:begin line="3 GOAL Y:YYY CM     ";line[159-9*8-:24]=cm(bcd[3]);end
    9:begin line="POS XXX YYY CM      ";line[159-4*8-:24]=cm(bcd[0]);line[159-8*8-:24]=cm(bcd[1]);end
    10:begin line="TGT XXX YYY CM      ";line[159-4*8-:24]=cm(bcd[2]);line[159-8*8-:24]=cm(bcd[3]);end
    11:begin line="BOX WWW HHH CM      ";line[159-4*8-:24]=cm(bcd[4]);line[159-8*8-:24]=cm(bcd[5]);end
    12:begin line="WP:1/4  HOLD +Y     ";line[159-3*8-:8]=digit({2'd0,point_q}+1'b1);if(!route_q)line="SINGLE TARGET +Y    ";end
    default:begin end
   endcase
   5:case(row)
    0:begin line="EDIT ORDERED POINTS ";end
    5:begin line="4 BACK              ";end
    6:begin line="FIXED COUNT:4       ";end
    7:begin line="STEP:5CM            ";end
    8:begin line="EDIT THEN RUN 4     ";end
    10:begin line="BODY ENVELOPE:27CM  ";end
    2:begin line="1 POINT:1 OF 4      ";line[159-8*8-:8]=digit({2'd0,edit_q}+1'b1);end
    3:begin line="2 POINT X:XXX CM    ";line[159-10*8-:24]=cm(bcd[12]);end
    4:begin line="3 POINT Y:YYY CM    ";line[159-10*8-:24]=cm(bcd[13]);end
    default:begin end
   endcase
   default:begin end
  endcase
  if(row==1)case(fr_status)
   0:begin line="IDLE / SELECT       ";end
   1:begin line="WAIT READY / ARMING ";end
   2:begin line="NOT READY           ";end
   4:begin line="RUNNING             ";end
   5:begin line="ENDED / CHECK RESULT";end
   6:begin line="FAULT / NO FIX      ";end
   7:begin line="STOPPED             ";end
   8:begin line="OBSTACLE STOP       ";end
   default:begin end
  endcase
  if(row==13)begin line="S1 BACK / STOP      ";end
  if(row==14)begin line="S2 NEXT S3 ENTER/+  ";end
  if(row==15)begin line="S4 MINUS / STOP     ";end
  character=x<160?line[(19-x[7:3])*8+:8]:8'd32;font_x=x[2:0];font_y=y[2:0];
  pixel=menu_q?(glyph?(selected_q?16'hffe0:16'hffff):16'h0000):raw_pixel;
 end
endmodule
