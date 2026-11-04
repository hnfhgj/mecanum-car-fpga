`timescale 1ns/1ps
module wireless_top_tb;
reg clk=0;always #10 clk=~clk;
reg rst=0,rx=1;wire tx,lfp,rfp,lbp,rbp;
tri tof_sda,cam_sda,cam_xclk;
wire [7:0] out;wire ov,oe;integer n=0;reg [7:0] actual[0:63];
free_navigation_top #(.USE_EXTERNAL_OSC(1)) dut(
 .clk(clk),.rst_n(rst),.lidar_rx(1'b1),.wireless_rx(rx),.wireless_rts(1'b0),.wireless_tx(tx),
 .key_n(4'b1111),.lf_ea(1'b0),.lf_eb(1'b0),.rf_ea(1'b0),.rf_eb(1'b0),.lb_ea(1'b0),.lb_eb(1'b0),.rb_ea(1'b0),.rb_eb(1'b0),
 .lf_pwm(lfp),.rf_pwm(rfp),.lb_pwm(lbp),.rb_pwm(rbp),.imu_miso(1'b0),.tof_sda(tof_sda),.tof_int(1'b1),
 .camera_pclk(1'b0),.camera_vsync(1'b0),.camera_href(1'b0),.camera_data(8'd0),.camera_xclk(cam_xclk),.camera_sda(cam_sda));
n10_uart_rx #(.BAUD(115200)) monitor(.clk(clk),.rst_n(rst),.rx(tx),.data(out),.valid(ov),.framing_error(oe));
always @(posedge clk)if(rst)begin
 if(lfp || rfp || lbp || rbp || dut.active)$fatal(1,"Boot/handshake caused motor activity");
 if(oe)$fatal(1,"Wireless TX framing error");
 if(ov)begin actual[n]=out;n=n+1;end
end
task byte_send(input [7:0] value);integer b;begin
 @(negedge clk);rx=0;repeat(434)@(negedge clk);
 for(b=0;b<8;b=b+1)begin rx=value[b];repeat(434)@(negedge clk);end
 rx=1;repeat(434)@(negedge clk);
end endtask
task hello(input [7:0] seq,input corrupt);reg [7:0] sum;begin
 sum=1+seq+12;
 byte_send('ha5);byte_send('h5a);byte_send(1);byte_send(seq);byte_send(12);
 repeat(6)byte_send(0);byte_send(corrupt?sum^1:sum);byte_send(13);byte_send(10);
end endtask
initial begin
 #51;rst=1;repeat(1000)@(negedge clk);
 hello(33,1);repeat(100000)@(negedge clk);if(n!=0)$fatal(1,"Corrupt frame ACK");
 hello(34,0);repeat(100000)@(negedge clk);
 if(n!=8 || {actual[0],actual[1],actual[2],actual[3],actual[4],actual[5],actual[6],actual[7]}!={"@22:00",8'd13,8'd10})$fatal(1,"Top HELLO ACK mismatch %d",n);
 if(dut.ui.remote_owned || dut.demo_running || dut.mission_running)$fatal(1,"Handshake owned task");
 n=0;
 byte_send('ha5);byte_send('h5a);byte_send(1);byte_send(35);byte_send(14);
 byte_send('h97);byte_send('hf4);repeat(4)byte_send(0);byte_send('hbd);byte_send(13);byte_send(10);
 repeat(100000)@(negedge clk);
 if(n!=8 || {actual[0],actual[1],actual[2],actual[3],actual[4],actual[5],actual[6],actual[7]}!={"@23:00",8'd13,8'd10})$fatal(1,"INFO wire ACK mismatch");
 if(dut.speed_level!=4)$fatal(1,"Top speed default");
 $display("PASS 97F4 TOP: actual 50MHz/115200 UART, bad frame ignored, HELLO ACK exact, boot/handshake zero PWM, no task");$finish;
end
initial begin #20000000;$fatal(1,"Top timeout");end
endmodule
