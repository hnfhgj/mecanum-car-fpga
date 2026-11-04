`timescale 1ns/1ps
module speed_tb;
reg clk=0;always #5 clk=~clk;
reg rst=0,start=0,stop=0;reg [2:0] selection=0,level=4;
reg [6:0] base=0;wire [15:0] scaled;
wire running,enable;wire signed [15:0] yaw,lf,rf,lb,rb;wire [3:0] status;
task_translation_scale scale(base,level,scaled);
basic_demo #(.MOVE_TICKS(10000)) demo(.clk(clk),.rst_n(rst),.tick(1'b1),.start(start),.stop(stop),.selection(selection),
 .slow_test(1'b0),.imu_ready(1'b1),.heading_ready(1'b1),.wheel_fault(1'b0),.range_fresh(1'b0),.near_stop(1'b1),
 .front_limit(7'd60),.back_limit(7'd60),.relative_yaw(16'd0),.turn(16'sd7),.running(running),.heading_enable(enable),
 .target_yaw(yaw),.lf_target(lf),.rf_target(rf),.lb_target(lb),.rb_target(rb),.status(status),.speed_level(level));
task launch(input [2:0] sel);begin
 stop=1;repeat(2)@(negedge clk);stop=0;selection=sel;start=1;@(negedge clk);start=0;repeat(8)@(negedge clk);
end endtask
function integer boosted(input integer value);begin
 case(value)0:boosted=0;15:boosted=35;30:boosted=70;45:boosted=105;60:boosted=140;90:boosted=210;default:boosted=value*7/4;endcase
end endfunction
initial begin
 for(integer l=1;l<=6;l=l+1)begin
  level=l;
  for(integer v=0;v<=127;v=v+1)begin base=v;#1;if(scaled!=(boosted(v)*l)/4)$fatal(1,"Scaling mismatch");end
 end
 #31;rst=1;
 for(integer l=1;l<=6;l=l+1)begin
  level=l;launch(0);
  if(lf!=210*l/4-7 || rf!=210*l/4+7 || lb!=lf || rb!=rf)$fatal(1,"Forward scale/mix");
  launch(1);if(lf!=-210*l/4-7 || rf!=-210*l/4+7)$fatal(1,"Reverse scale/mix");
  launch(2);if(lf!=-210*l/4-7 || rf!=210*l/4+7 || lb!=210*l/4-7 || rb!=-210*l/4+7)$fatal(1,"Pure left strafe mix");
  launch(3);if(lf!=210*l/4-7 || rf!=-210*l/4+7 || lb!=-210*l/4-7 || rb!=210*l/4+7)$fatal(1,"Pure right strafe mix");
  launch(4);if(yaw!=9000 || lf!=-7 || rf!=7 || lb!=-7 || rb!=7)$fatal(1,"Spin changed with speed");
  launch(5);if(yaw!=-9000 || lf!=-7 || rf!=7)$fatal(1,"Reverse spin changed");
 end
 stop=1;repeat(2)@(negedge clk);if(lf || rf || lb || rb)$fatal(1,"Stop output");
 $display("PASS speed: all six faster levels, all 0..127 inputs, four translations, unchanged +/-90 spin and heading, stop zero");$finish;
end
endmodule
