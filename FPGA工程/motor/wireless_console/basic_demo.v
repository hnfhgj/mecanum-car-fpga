// Four basic movements plus explicit approach/slow/stop demonstration.
// Heading hold remains the existing feedback PD; wheel outputs go to existing PID.
module basic_demo #(
 parameter integer MOVE_TICKS=400,SPIN_TICKS=1800,SLOW_TICKS=800,ARM_TICKS=300
)(input wire clk,rst_n,tick,start,stop,input wire [2:0] selection,
 input wire slow_test,imu_ready,heading_ready,wheel_fault,range_fresh,near_stop,
 input wire [6:0] front_limit,back_limit,
 input wire signed [15:0] relative_yaw,turn,
 output reg running,heading_enable,output reg signed [15:0] target_yaw,
 output reg signed [15:0] lf_target,rf_target,lb_target,rb_target,
 output reg [3:0] status,output reg slow_seen,stop_seen,input wire [2:0] speed_level);
 reg [2:0] kind;reg approach,started;reg [15:0] elapsed,arming;reg [5:0] settled;
 reg signed [15:0] base,lateral,rotation;
 wire signed [31:0] error=$signed(relative_yaw)-$signed(target_yaw);
 wire spinning=kind>=4;
 // Ordinary six motion tests ignore radar near-stop; explicit approach still uses it.
 wire obstacle=approach && range_fresh && near_stop;
 wire range_lost=approach && !range_fresh;
 wire [6:0] commanded_limit=approach?front_limit:7'd60;
 wire [15:0] move_speed,approach_speed;
 task_translation_scale move_scale(7'd90,speed_level,move_speed);
 task_translation_scale approach_scale(commanded_limit,speed_level,approach_speed);
 wire blocked=obstacle || (approach && front_limit==0);
 always @*begin
  base=0;lateral=0;rotation=0;
  if(running && started && heading_ready && !wheel_fault && !blocked && !range_lost)begin
   // Physical logical order LF/RF/LB/RB; positive lateral means LEFT.
   base=approach?$signed(approach_speed):(kind==0?$signed(move_speed):(kind==1?-$signed(move_speed):16'sd0));
   lateral=approach?16'sd0:(kind==2?$signed(move_speed):(kind==3?-$signed(move_speed):16'sd0));
   rotation=turn;
  end
 end
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin running<=0;heading_enable<=0;target_yaw<=0;lf_target<=0;rf_target<=0;lb_target<=0;rb_target<=0;
   kind<=0;approach<=0;started<=0;elapsed<=0;arming<=0;settled<=0;status<=0;slow_seen<=0;stop_seen<=0;
  end else begin
   lf_target<=base-lateral-rotation;rf_target<=base+lateral+rotation;lb_target<=base+lateral-rotation;rb_target<=base-lateral+rotation;
   if(start && !running)begin
    if(!imu_ready || wheel_fault || (slow_test && !range_fresh))status<=2;
    else begin
    kind<=selection;approach<=slow_test;running<=1;heading_enable<=1;started<=0;
    elapsed<=0;arming<=0;settled<=0;status<=1;slow_seen<=0;stop_seen<=0;
    case(selection)
     4:target_yaw<=9000;5:target_yaw<=-9000;default:target_yaw<=0;
    endcase
    end
   end
   if(running && tick)begin
    if(!started)begin
     if(heading_ready)begin started<=1;status<=4;end
     else if(arming>=ARM_TICKS)begin running<=0;heading_enable<=0;status<=6;end
     else arming<=arming+1'b1;
    end else begin
     elapsed<=elapsed+1'b1;
     if(approach && front_limit<60 && front_limit>0)slow_seen<=1;
     if(spinning && error<=200 && error>=-200)begin
      if(settled>=19)begin running<=0;heading_enable<=0;status<=5;end
      else settled<=settled+1'b1;
     end else settled<=0;
     if(!spinning && elapsed>=(approach?SLOW_TICKS:MOVE_TICKS))begin running<=0;heading_enable<=0;status<=5;end
     if(spinning && elapsed>=SPIN_TICKS)begin running<=0;heading_enable<=0;status<=6;end
    end
   end
   if(running && (wheel_fault || !imu_ready || range_lost || (started && !heading_ready)))begin
    running<=0;heading_enable<=0;status<=6;
   end
   if(running && blocked)begin running<=0;heading_enable<=0;status<=8;stop_seen<=1;end
   if(stop)begin running<=0;heading_enable<=0;status<=7;
    lf_target<=0;rf_target<=0;lb_target<=0;rb_target<=0;end
  end
 end
endmodule
