`timescale 1ns/1ps
module pc_control_tb;
 reg clk=0,rst=0,valid=0,fe=0,active=0,motion_ok=1,range_ok=1,nav_ok=1,calibrated=1;
 reg [7:0] byte_in=0;reg [3:0] keys=15;
 wire cv,frame,ack;wire [7:0] seq,op,ack_seq,status;wire [15:0] a,b,c,x,y;
 wire [2:0] page,group_id,selection;wire menu,slow,demo,single,route,stop,cal;
 wire [3:0] cursor,gain;wire [63:0] xs,ys;wire [1:0] index;wire [31:0] kp,kd;
 wire [15:0] fb,lr;wire [1:0] cone;wire [2:0] level;reg manual_active=0,manual_range=1;wire drive_valid;wire [5:0] drive_mask;
 integer passed=0,ndemo=0,nstop=0,ncal=0,nsingle=0,nroute=0,nack=0,ncommands=0;
 always #5 clk=~clk;
 pc_command_rx #(.CLK_HZ(1000)) parser(clk,rst,byte_in,valid,fe,cv,seq,op,a,b,c,frame);
 pc_ui #(.CLK_HZ(1000)) dut(clk,rst,active,keys,cv,seq,op,a,b,c,motion_ok,range_ok,nav_ok,
  ack,ack_seq,status,calibrated,16'd2000,16'd2000,page,group_id,menu,cursor,selection,slow,
  demo,single,route,stop,cal,x,y,xs,ys,index,gain,kp,kd,level,manual_active,manual_range,drive_valid,drive_mask,fb,lr,cone);
 always @(posedge clk)begin
  if(cv)ncommands=ncommands+1;if(ack)nack=nack+1;
  if(demo)ndemo=ndemo+1;if(stop)nstop=nstop+1;if(cal)ncal=ncal+1;
  if(single)nsingle=nsingle+1;if(route)nroute=nroute+1;
 end
 task byte_send(input [7:0] value);begin
  @(negedge clk);byte_in=value;valid=1;@(negedge clk);valid=0;
 end endtask
 task command(input [7:0] id,cmd,input [15:0] p,q,t,input corrupt);
 reg [7:0] sum;
 begin
  sum=1+id+cmd+p[15:8]+p[7:0]+q[15:8]+q[7:0]+t[15:8]+t[7:0];
  byte_send('hA5);byte_send('h5A);byte_send(1);byte_send(id);byte_send(cmd);
  byte_send(p[15:8]);byte_send(p[7:0]);byte_send(q[15:8]);byte_send(q[7:0]);byte_send(t[15:8]);byte_send(t[7:0]);
  byte_send(corrupt?sum^8'h01:sum);byte_send(13);byte_send(10);repeat(4)@(negedge clk);
 end endtask
 initial begin
  #31;rst=1;repeat(12)@(negedge clk);
  command(1,2,0,0,0,1);if(ncommands || ndemo || nack)$fatal(1,"Bad checksum executed");
  byte_send('hA5);byte_send('h5A);byte_send(1);repeat(60)@(negedge clk);
  if(frame)$fatal(1,"Incomplete frame did not expire");
  command(254,12,0,0,0,0);if(status || ndemo || nstop)$fatal(1,"Handshake caused an action");
  motion_ok=0;command(1,2,0,0,0,0);if(status!=3 || ndemo)$fatal(1,"Not-ready motion accepted");
  motion_ok=1;command(2,2,4,0,0,0);if(status || ndemo!=1 || selection!=4)$fatal(1,"Spin selection");
  active=1;command(0,11,0,0,0,0);command(2,2,4,0,0,0);
  if(status || ndemo!=1)$fatal(1,"Retransmitted action executed twice");
  command(2,2,5,0,0,0);if(status!=6 || ndemo!=1)$fatal(1,"Sequence conflict");
  command(3,5,500,600,0,0);if(status!=1 || x!=1000)$fatal(1,"Busy target edit accepted");
  command(4,0,0,0,0,0);if(status || nstop!=1)$fatal(1,"Remote stop failed");active=0;
  command(5,5,250,600,0,0);if(status!=2)$fatal(1,"Out of bounds accepted");
  command(6,5,500,600,0,0);if(status || x!=500 || y!=600)$fatal(1,"Target not applied");
  command(7,7,2,700,800,0);if(status || xs[47:32]!=700 || ys[47:32]!=800)$fatal(1,"Waypoint update failed");
  command(8,9,400,3000,0,0);if(status || kp!=400 || kd!=3000)$fatal(1,"Gains update failed");
  command(9,1,4,0,0,0);if(status || page!=4)$fatal(1,"TFT page select");
  command(10,10,0,0,0,0);if(status || page!=5 || !menu || group_id)$fatal(1,"Root menu");
  manual_range=0;command(11,4,0,0,0,0);if(status!=3 || ncal)$fatal(1,"Blind calibration accepted");manual_range=1;range_ok=0;
  command(12,4,0,0,0,0);if(status || ncal!=1)$fatal(1,"Calibration trigger");
  range_ok=1;nav_ok=0;command(13,6,0,0,0,0);if(status!=3 || nsingle)$fatal(1,"Blind nav accepted");nav_ok=1;
  command(14,6,0,0,0,0);if(status || nsingle!=1)$fatal(1,"Single goal start");
  command(15,8,0,0,0,0);if(status || nroute!=1)$fatal(1,"Route start");
  command(16,99,0,0,0,0);if(status!=5)$fatal(1,"Unknown command");
  command(17,2,0,0,0,0);active=1;
  repeat(1300)@(negedge clk);command(0,11,0,0,0,0);
  repeat(1300)@(negedge clk);if(nstop!=1)$fatal(1,"Heartbeat not extending lease");
  repeat(800)@(negedge clk);if(nstop!=2 || dut.remote_owned)$fatal(1,"Wireless loss did not stop owned task");
  active=0;repeat(10)@(negedge clk);
  // Physical starts are never assigned a PC lease.
  keys[2]=0;repeat(25)@(negedge clk);keys[2]=1;repeat(25)@(negedge clk);active=1;
  repeat(2100)@(negedge clk);if(nstop!=2)$fatal(1,"PC watchdog stopped local task");
  active=0;repeat(10)@(negedge clk);
  if(level!=4)$fatal(1,"Default speed changed");
  command(100,14,16'h97F4,0,0,0);if(status || dut.remote_owned)$fatal(1,"INFO caused action");
  command(101,14,16'h97E9,0,0,0);if(status!=5)$fatal(1,"Wrong firmware accepted");
  command(102,13,6,0,0,0);if(status || level!=6)$fatal(1,"Speed 150% not applied");
  active=1;command(103,13,1,0,0,0);if(status!=1 || level!=6)$fatal(1,"Busy speed edit accepted");
  command(104,14,16'h97F4,0,0,0);if(status)$fatal(1,"INFO blocked while active");
  active=0;command(105,13,0,0,0,0);if(status!=2 || level!=6)$fatal(1,"Zero speed accepted");
  command(106,13,7,0,0,0);if(status!=2)$fatal(1,"Bad speed accepted");
  command(107,13,1,1,0,0);if(status!=2)$fatal(1,"Extra speed payload accepted");
  command(108,13,1,0,0,0);if(status || level!=1)$fatal(1,"Speed 25% not applied");
  command(110,15,5,0,0,0);if(status || drive_mask!=5)$fatal(1,"W+A input rejected");
  active=1;manual_active=1;command(111,15,8,0,0,0);if(status || drive_mask!=8)$fatal(1,"Held update busy");
  manual_active=0;command(112,15,1,0,0,0);if(status!=1)$fatal(1,"Manual interrupted navigation");
  manual_range=0;command(113,15,0,0,0,0);if(status || drive_mask)$fatal(1,"Release blocked by readiness");
  active=0;command(114,15,1,0,0,0);if(status!=3)$fatal(1,"Blind manual accepted");
  command(115,15,64,0,0,0);if(status!=2)$fatal(1,"Bad key mask accepted");
  manual_range=1;active=0;manual_active=0;
  if(fb!=400 || lr!=250 || cone!=2)$fatal(1,"Obstacle defaults changed");
  command(116,16,500,300,3,0);if(status || fb!=500 || lr!=300 || cone!=3)$fatal(1,"Obstacle settings failed");
  command(117,16,199,300,3,0);if(status!=2 || fb!=500)$fatal(1,"Small front bound accepted");
  command(118,16,801,300,3,0);if(status!=2)$fatal(1,"Large front bound accepted");
  command(119,16,400,149,2,0);if(status!=2)$fatal(1,"Small side bound accepted");
  command(120,16,400,601,2,0);if(status!=2)$fatal(1,"Large side bound accepted");
  command(121,16,400,250,0,0);if(status!=2)$fatal(1,"Invalid cone accepted");
  active=1;command(122,16,400,250,2,0);if(status!=1 || fb!=500)$fatal(1,"Busy settings accepted");
  active=0;range_ok=0;begin integer old_demo,old_single,old_route;
  old_demo=ndemo;old_single=nsingle;old_route=nroute;command(123,4,0,0,0,0);
  if(status || ncal!=2 || ndemo!=old_demo || nsingle!=old_single || nroute!=old_route)$fatal(1,"Static calibration requires old wall clearance or started motion");end
  passed=1;$display("PASS PC protocol/UI: held updates, release while unready, exclusive tasks; INFO firmware, idle-only speed levels; checksum, timeout, dedup, conflicts, busy, readiness, targets, points, gains, heartbeat and local control");$finish;
 end
endmodule

module pc_motor_pipeline_tb;
 reg clk=0,rst=0,blocked=1;reg signed [15:0] selected=0,duty=0;
 wire signed [15:0] target,drive;integer passed=0;
 always #5 clk=~clk;
 pc_motor_pipeline dut(clk,rst,blocked,selected,duty,target,drive);
 initial begin
  #31;rst=1;selected=60;duty=441;repeat(2)@(negedge clk);
  if(target || drive)$fatal(1,"Idle pipeline enabled motor");
  blocked=0;@(negedge clk);if(target!=60 || drive!=-441)$fatal(1,"Forward pipeline polarity");
  selected=-60;duty=-500;@(negedge clk);if(target!=-60 || drive!=500)$fatal(1,"Reverse pipeline polarity");
  blocked=1;@(negedge clk);if(target || drive)$fatal(1,"Stop did not clear both stages");
  blocked=0;duty=882;@(negedge clk);if(drive!=-882)$fatal(1,"PWM magnitude changed");
  rst=0;#1;if(target || drive)$fatal(1,"Reset output not zero");
  passed=1;$display("PASS PC motor pipeline: idle/stop/reset zero, signed targets and original PWM polarity/magnitude");$finish;
 end
endmodule

module pc_bridge_tb;
 reg clk=0,rst=0,rx=1,rts=1,ack_valid=0;reg [7:0] ack_seq=0;
 wire tx,rv,re,cv,frame;wire [7:0] rd,seq,op,out;wire [15:0] a,b,c;wire ov,oe;
 integer passed=0,n=0,commands=0;reg [7:0] actual[0:31];
 always #5 clk=~clk;
 pc_wireless_bridge #(.CLK_HZ(20000),.BAUD(2000),.ADDRESS_BITS(5)) dut(
  clk,rst,8'd0,1'b0,rx,rts,tx,ack_valid,ack_seq,8'd0,frame,rd,rv,re);
 pc_command_rx #(.CLK_HZ(20000)) parser(clk,rst,rd,rv,re,cv,seq,op,a,b,c,frame);
 n10_uart_rx #(.CLK_HZ(20000),.BAUD(2000)) monitor(clk,rst,tx,out,ov,oe);
 always @(posedge clk)begin
  ack_valid<=cv;if(cv)begin ack_seq<=seq;commands=commands+1;end
  if(ov)begin actual[n]=out;n=n+1;end
  if(oe)$fatal(1,"ACK framing error");
 end
 task uart_byte(input [7:0] value);integer bitn;begin
  @(negedge clk);rx=0;repeat(10)@(negedge clk);
  for(bitn=0;bitn<8;bitn=bitn+1)begin rx=value[bitn];repeat(10)@(negedge clk);end
  rx=1;repeat(12)@(negedge clk);
 end endtask
 initial begin
  #31;rst=1;
  // Sequence=P, argument contains R/?; these must not trigger legacy raw commands.
  uart_byte('hA5);uart_byte('h5A);uart_byte(1);uart_byte('h50);uart_byte(9);
  uart_byte(0);uart_byte('h52);uart_byte(0);uart_byte('h3F);uart_byte(0);uart_byte(0);
  uart_byte(8'hEB);uart_byte(13);uart_byte(10);
  repeat(30)@(negedge clk);if(commands!=1 || dut.paused || n)$fatal(1,"Control frame/RTS isolation failed");
  rts=0;repeat(1100)@(negedge clk);
  if(n!=8 || actual[0]!="@" || actual[1]!="5" || actual[2]!="0" || actual[3]!=":" || actual[4]!="0" || actual[5]!="0" || actual[6]!=13 || actual[7]!=10)$fatal(1,"ACK wire format");
  passed=1;$display("PASS PC UART end-to-end: framed RX, payload P/R/? isolation, RTS-gated ACK bytes");$finish;
 end
endmodule
