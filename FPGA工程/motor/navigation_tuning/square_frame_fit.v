// Complete-scan ownership, orientation search and four supported wall bins.
// Initial calibration requires an unobstructed square and approximately +Y
// facing chassis. Fit precision is limited by 32mm bins and 0.5deg yaw steps.
// This core is standalone until square navigation integration is validated.
module square_frame_fit(input wire clk,rst_n,bad,calibrate,
 input wire point_valid,input wire signed [15:0] point_forward,point_left,
 input wire [15:0] point_angle,point_range,input wire signed [15:0] imu_yaw,
 input wire release_scan,input wire [8:0] read_addr,
 output reg signed [15:0] read_forward,read_left,
 output reg [9:0] point_count,output reg scan_ready,pose_update,pose_ok,calibrated,
 output reg signed [15:0] pose_x,pose_y,pose_yaw,correction,
 output reg [15:0] width_mm,height_mm,output reg [15:0] quality,output reg [3:0] debug_reason);
 reg signed [15:0] forwards[0:511],lefts[0:511];
 reg signed [15:0] fit_forwards[0:511],fit_lefts[0:511],fit_forward,fit_left;
 reg [6:0] state;reg collect,previous_valid,overflow,cal_pending,searching;
 reg [15:0] previous_angle;reg [9:0] captured,index;
 reg signed [15:0] yaw_reference,scan_yaw,trial,best_angle;
 reg [8:0] bin_index;
 reg [9:0] hist_x[0:255],hist_y[0:255];
 reg [7:0] hist_x_addr,hist_y_addr;
 reg hist_x_valid,hist_y_valid;
 reg [9:0] hist_x_read,hist_y_read;
 wire scanning_bins=state==24 || state==25 || state==4;
 reg [9:0] peaks[0:3];reg signed [15:0] locations[0:3];
 reg [15:0] best_score;reg signed [15:0] best_x,best_y;
 reg [15:0] best_w,best_h;
 reg project_start;wire project_done;wire signed [15:0] px,py;
 reg signed [15:0] project_f,project_l,project_angle;
 square_project project(.clk(clk),.rst_n(rst_n),.cancel(bad || calibrate),.start(project_start),
  .forward_mm(project_f),.left_mm(project_l),.angle_cdeg(project_angle),.busy(),.done(project_done),.x(px),.y(py));
 wire wrap_angle=previous_valid && previous_angle>point_angle+17'd18000;
 // Two synchronous 1W/1R replicas let synthesis infer BSRAM rather than a
 // large register file and asynchronous read multiplexers. No memory reset.
 always @(posedge clk)begin
  hist_x_read<=hist_x[scanning_bins?bin_index[7:0]:hist_x_addr];
  hist_y_read<=hist_y[scanning_bins?bin_index[7:0]:hist_y_addr];
  if(!bad && !calibrate)begin
   if(state==1)begin hist_x[bin_index[7:0]]<=0;hist_y[bin_index[7:0]]<=0;end
   else if(state==22)begin
    if(hist_x_valid)hist_x[hist_x_addr]<=hist_x_read+1'b1;
    if(hist_y_valid)hist_y[hist_y_addr]<=hist_y_read+1'b1;
   end
  end
  if(state==0 && !bad && !calibrate && point_valid && !wrap_angle && collect &&
     point_range>=50 && point_range<=5000 && captured<512)begin
   forwards[captured[8:0]]<=point_forward;lefts[captured[8:0]]<=point_left;
   fit_forwards[captured[8:0]]<=point_forward;fit_lefts[captured[8:0]]<=point_left;
  end
  read_forward<=forwards[read_addr];read_left<=lefts[read_addr];
  fit_forward<=fit_forwards[index[8:0]];fit_left<=fit_lefts[index[8:0]];
 end
 function signed [15:0] wrap(input signed [31:0] v);
 begin if(v>=18000)wrap=v-36000;else if(v< -18000)wrap=v+36000;else wrap=v;end endfunction
 function [31:0] ab(input signed [31:0] v);begin ab=v<0?-v:v;end endfunction
 wire signed [31:0] yaw_delta=$signed(scan_yaw)-$signed(yaw_reference);
 wire signed [31:0] bin_location=$signed({1'b0,bin_index[7:0]})*32-4080;
 wire signed [31:0] fit_w=$signed(locations[1])-$signed(locations[0]);
 wire signed [31:0] fit_h=$signed(locations[3])-$signed(locations[2]);
 wire [15:0] score=peaks[0]+peaks[1]+peaks[2]+peaks[3];
 wire geometry=fit_w>=800 && fit_w<=3000 && fit_h>=800 && fit_h<=3000 &&
  peaks[0]>=12 && peaks[1]>=12 && peaks[2]>=12 && peaks[3]>=12;
 wire consistent=!calibrated || (ab(fit_w-$signed({1'b0,width_mm}))<128 && ab(fit_h-$signed({1'b0,height_mm}))<128);
 wire x_negative=bin_location< -150 && (searching || ab(bin_location+$signed(pose_x))<224);
 wire x_positive=bin_location>150 && (searching || ab(bin_location-($signed({1'b0,width_mm})-$signed(pose_x)))<224);
 wire y_negative=bin_location< -150 && (searching || ab(bin_location+$signed(pose_y))<224);
 wire y_positive=bin_location>150 && (searching || ab(bin_location-($signed({1'b0,height_mm})-$signed(pose_y)))<224);
 integer k;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin state<=0;collect<=0;previous_valid<=0;overflow<=0;cal_pending<=0;searching<=0;
   previous_angle<=0;captured<=0;index<=0;yaw_reference<=0;scan_yaw<=0;trial<=0;best_angle<=0;bin_index<=0;
   best_score<=0;best_x<=0;best_y<=0;best_w<=0;best_h<=0;
   hist_x_addr<=0;hist_y_addr<=0;hist_x_valid<=0;hist_y_valid<=0;
   project_start<=0;project_f<=0;project_l<=0;project_angle<=0;
   point_count<=0;scan_ready<=0;pose_update<=0;pose_ok<=0;calibrated<=0;
   pose_x<=0;pose_y<=0;pose_yaw<=0;correction<=0;width_mm<=0;height_mm<=0;quality<=0;debug_reason<=0;
   for(k=0;k<4;k=k+1)begin peaks[k]<=0;locations[k]<=0;end
  end else begin
   project_start<=0;pose_update<=0;
   if(point_valid)begin previous_valid<=1;previous_angle<=point_angle;end
   if(calibrate)begin debug_reason<=1;cal_pending<=1;calibrated<=0;pose_ok<=0;scan_ready<=0;collect<=0;state<=0;end
   else if(bad)begin debug_reason<=7;collect<=0;previous_valid<=0;pose_ok<=0;scan_ready<=0;
    if(!calibrated)cal_pending<=1;state<=0;end
   else case(state)
    0:if(point_valid && (calibrated || cal_pending))begin
     if(wrap_angle)begin
      if(collect)begin
       collect<=0;point_count<=captured;scan_yaw<=imu_yaw;
       if(overflow || captured<120 || ab($signed(imu_yaw)-$signed(scan_yaw))>150)begin
        debug_reason<=overflow?5:(captured<120?6:4);pose_ok<=0;pose_update<=1;end
       else begin if(debug_reason<=2)debug_reason<=2;searching<=cal_pending;trial<=cal_pending?-16'sd2000:wrap(yaw_delta+$signed(correction));
        if(cal_pending)begin yaw_reference<=imu_yaw;cal_pending<=0;end
        best_score<=0;best_angle<=0;bin_index<=0;state<=1;end
      end else begin collect<=1;captured<=0;overflow<=0;scan_yaw<=imu_yaw;end
     end else if(collect && point_range>=50 && point_range<=5000)begin
      if(captured<512)begin captured<=captured+1'b1;end
      else overflow<=1;
     end
    end
    1:begin
     if(bin_index==255)begin index<=0;state<=20;end else bin_index<=bin_index+1'b1;end
    20:state<=2;
    2:begin project_f<=fit_forward;project_l<=fit_left;project_angle<=trial;project_start<=1;state<=3;end
    3:if(project_done)begin
     hist_x_addr<=px[12:5]^8'h80;hist_y_addr<=py[12:5]^8'h80;
     hist_x_valid<=px>=-4096 && px<4096;hist_y_valid<=py>=-4096 && py<4096;
     state<=21;
    end
    21:state<=22;
    22:begin
     if(index+1>=point_count)begin
      bin_index<=0;for(k=0;k<4;k=k+1)begin peaks[k]<=0;locations[k]<=0;end state<=24;
     end else begin index<=index+1'b1;state<=20;end
    end
    24:state<=25;
    25:state<=4;
    4:begin
     if(x_negative && hist_x_read>peaks[0])begin peaks[0]<=hist_x_read;locations[0]<=bin_location;end
     if(x_positive && hist_x_read>peaks[1])begin peaks[1]<=hist_x_read;locations[1]<=bin_location;end
     if(y_negative && hist_y_read>peaks[2])begin peaks[2]<=hist_y_read;locations[2]<=bin_location;end
     if(y_positive && hist_y_read>peaks[3])begin peaks[3]<=hist_y_read;locations[3]<=bin_location;end
     if(bin_index==255)state<=5;else begin bin_index<=bin_index+1'b1;state<=24;end
    end
    5:begin
     if(geometry && consistent && score>best_score)begin best_score<=score;best_angle<=trial;
      best_x<=-locations[0];best_y<=-locations[2];best_w<=fit_w;best_h<=fit_h;end
     if(searching && trial<2000)begin trial<=trial+16'sd50;bin_index<=0;state<=1;end else state<=6;
    end
    6:begin
     pose_update<=1;pose_ok<=best_score!=0;quality<=best_score;
     if(best_score!=0)begin debug_reason<=9;
      if(searching)begin correction<=best_angle;width_mm<=best_w;height_mm<=best_h;calibrated<=1;end
      pose_x<=best_x;pose_y<=best_y;pose_yaw<=wrap(32'sd9000+$signed(best_angle));
      scan_ready<=1;state<=7;
     end else begin debug_reason<=3;if(searching)cal_pending<=1;state<=0;end
    end
    7:if(release_scan)begin scan_ready<=0;state<=0;end
    default:state<=0;
   endcase
  end
 end
endmodule
