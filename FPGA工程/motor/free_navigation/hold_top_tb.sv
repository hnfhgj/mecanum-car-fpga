`timescale 1ns/1ps
module hold_top_tb;
reg clk=0;always #10 clk=~clk;
reg rst=0,rx=1;wire tx,lfp,rfp,lbp,rbp;tri tof_sda,cam_sda,cam_xclk;
free_navigation_top #(.USE_EXTERNAL_OSC(1)) dut(
 .clk(clk),.rst_n(rst),.lidar_rx(1'b1),.wireless_rx(rx),.wireless_rts(1'b0),.wireless_tx(tx),
 .key_n(4'b1111),.lf_ea(1'b0),.lf_eb(1'b0),.rf_ea(1'b0),.rf_eb(1'b0),.lb_ea(1'b0),.lb_eb(1'b0),.rb_ea(1'b0),.rb_eb(1'b0),
 .lf_pwm(lfp),.rf_pwm(rfp),.lb_pwm(lbp),.rb_pwm(rbp),.imu_miso(1'b0),.tof_sda(tof_sda),.tof_int(1'b1),
 .camera_pclk(1'b0),.camera_vsync(1'b0),.camera_href(1'b0),.camera_data(8'd0),.camera_xclk(cam_xclk),.camera_sda(cam_sda));
task byte_send(input [7:0] value);integer b;begin
 @(negedge clk);rx=0;repeat(434)@(negedge clk);
 for(b=0;b<8;b=b+1)begin rx=value[b];repeat(434)@(negedge clk);end
 rx=1;repeat(434)@(negedge clk);
end endtask
task command(input [7:0] seq,op,input [15:0] value);reg [7:0] sum;begin
 sum=1+seq+op+value[15:8]+value[7:0];
 byte_send('ha5);byte_send('h5a);byte_send(1);byte_send(seq);byte_send(op);
 byte_send(value[15:8]);byte_send(value[7:0]);repeat(4)byte_send(0);byte_send(sum);byte_send(13);byte_send(10);
 repeat(1000)@(negedge clk);
end endtask
initial begin
 #51;rst=1;repeat(1000)@(negedge clk);
 command(1,15,1);if(dut.active)$fatal(1,"Top accepted blind drive");
 // Isolate validated input/mapper/PID routing from the real sensor peripherals.
 force dut.imu_init=1;force dut.attitude_ready=1;force dut.imu_fresh=1;
 force dut.calibrated=0;force dut.pose_ok=0;
 force dut.attitude_valid=1;force dut.yaw=0;force dut.quiet=1;
 force dut.manual_range_fresh=1;force dut.manual_blocked=0;
 command(2,15,1);if(!dut.manual_active || dut.targets[0]!=210 || dut.targets[1]!=210 || dut.targets[2]!=210 || dut.targets[3]!=210)$fatal(1,"UART W did not reach four logical motor targets");
 force dut.manual_blocked=1;repeat(10)@(negedge clk);
 if(dut.targets[0] || dut.targets[1] || dut.targets[2] || dut.targets[3])$fatal(1,"Top front block not applied");
 command(3,15,5);if(dut.targets[0]!=-210 || dut.targets[1]!=210 || dut.targets[2]!=210 || dut.targets[3]!=-210)$fatal(1,"UART diagonal did not preserve free left escape");
 command(4,15,0);if(dut.active || lfp || rfp || lbp || rbp)$fatal(1,"UART release did not zero motor pipeline");
 command(5,15,32);if(dut.targets[0]!=110 || dut.targets[1]!=-110 || dut.targets[2]!=110 || dut.targets[3]!=-110)$fatal(1,"UART CW polarity");
 command(6,0,0);if(dut.active || lfp || rfp || lbp || rbp)$fatal(1,"UART STOP failed");
 $display("PASS held top: real 50MHz/115200 command UART to actual top motor target mux, no field/calibration/pose required, blind rejection, direction block/free escape, release, CW, STOP");$finish;
end
initial begin #100000000;$fatal(1,"Held top timeout");end
endmodule
