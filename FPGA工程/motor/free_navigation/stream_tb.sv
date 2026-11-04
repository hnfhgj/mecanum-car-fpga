`timescale 1ns/1ps
// End-to-end check of the real telemetry formatter tap against its wired UART.
module wireless_stream_tb;
 reg clk=0,rst=0;wire wired,wireless,valid;wire [7:0] stream;
 wire [7:0] a,b;wire av,bv,ae,be;integer an=0,bn=0,passed=0;
 reg found=0;reg [7:0] expected[0:95];
 initial begin string hextext="425404F1546042FF9C01F4232821E821E807D003E805DC07D009C409C43997F4E1005AFFC40050019000FA0200B40900";for(integer i=0;i<96;i=i+1)expected[i]=hextext[i];end
 reg [7:0] reference[0:1023];
 always #5 clk=~clk;
 basic_uart_debug #(.CLK_FREQ(10000),.BAUD(100),.SEND_MS(1),.DIAGNOSTIC_ENABLE(1)) source(
  .clk(clk),.rst_n(rst),.lf_count(16'd123),.rf_count(16'd456),.lb_count(16'd789),.rb_count(16'd999),
  .lf_speed(16'd21),.rf_speed(-16'd22),.lb_speed(16'd23),.rb_speed(-16'd24),
  .acc_x(16'd0),.acc_y(16'd0),.acc_z(16'd16384),.gyro_x(16'd1),.gyro_y(16'd2),.gyro_z(16'd3),
  .distance_mm(16'd357),.imu_init(1'b1),.imu_valid(1'b1),.tof_init(1'b1),.tof_valid(1'b1),
  .target_speed(16'd60),.run_mask(4'd0),.fault_mask(4'd0),
  .attitude_roll(16'd100),.attitude_pitch(-16'd200),.attitude_yaw(16'd300),.attitude_ready(1'b1),
  .heading_left(16'd60),.heading_right(16'd60),.heading_turn(16'd0),.heading_fault(1'b0),
  .calibration_mask(4'b1111),.encoder_sign_mask(4'b1001),.fault_reasons(16'd0),.diagnostic_stop(1'b1),
  .diagnostic_bits(384'h425404F1546042FF9C01F4232821E821E807D003E805DC07D009C409C43997F4E1005AFFC40050019000FA0200B40900),
  .scan_ready(1'b0),.scan_sequence(16'd0),.scan_data(8'd0),.stream_data(stream),.stream_valid(valid),.uart_tx(wired));
 wireless_uart_bridge #(.CLK_HZ(10000),.BAUD(100)) dut(clk,rst,stream,valid,1'b1,1'b0,wireless);
 n10_uart_rx #(.CLK_HZ(10000),.BAUD(100)) wired_rx(clk,rst,wired,a,av,ae);
 n10_uart_rx #(.CLK_HZ(10000),.BAUD(100)) wireless_rx(clk,rst,wireless,b,bv,be);
 always @(posedge clk)begin
  if(ae || be)$fatal(1,"Formatter UART framing failed");
  if(av)begin reference[an]=a;an=an+1;end
  if(bv)begin
   if(bn>=an || b!==reference[bn])$fatal(1,"Wireless differs from wired at byte %0d",bn);
   bn=bn+1;
   if(bn==442)begin
    for(integer i=0;i<442-7;i=i+1)begin
     if({reference[i],reference[i+1],reference[i+2],reference[i+3],reference[i+4],reference[i+5],reference[i+6]}=="IO HEX=")begin
      if(i+105!=442)$fatal(1,"Wrong BT04 line/frame length");
      for(integer j=0;j<96;j=j+1)begin
       if(reference[i+7+j]!==expected[j])$fatal(1,"BT04 hex mismatch %d",j);
      end
      if(reference[i+103]!=13 || reference[i+104]!=10)$fatal(1,"BT04 termination");
      found=1;
     end
    end
    if(!found)$fatal(1,"Missing BT04 line");
    if(dut.overflow)$fatal(1,"Normal stream overflow");
    passed=1;$display("PASS wired/wireless 442 real telemetry bytes match, including signed sensors and task frames");$finish;
   end
  end
 end
 initial begin #31;rst=1;#15000000;$fatal(1,"Real stream timeout");end
endmodule
