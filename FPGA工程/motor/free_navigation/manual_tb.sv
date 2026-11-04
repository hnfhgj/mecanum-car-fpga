`timescale 1ns/1ps
module manual_tb;
reg clk=0;always #5 clk=~clk;
reg rst=0,cmd=0,stop=0,fresh=1;reg [5:0] mask=0;reg [3:0] blocked=0;reg [2:0] level=4;
wire active,lease;wire signed [15:0] f,l,u,lf,rf,lb,rb;
hold_drive #(.CLK_HZ(10000)) dut(.clk(clk),.rst_n(rst),.tick(1'b1),.stop(stop),.command_valid(cmd),.command_mask(mask),
 .imu_ready(1'b1),.range_fresh(fresh),.heading_ready(1'b1),.heading_fault(1'b0),.blocked(blocked),.speed_level(level),.heading_turn(16'sd0),
 .active(active),.lease_valid(lease),.forward_applied(f),.left_applied(l),.turn_applied(u),.lf_target(lf),.rf_target(rf),.lb_target(lb),.rb_target(rb));
task send(input [5:0] keys);begin mask=keys;cmd=1;@(negedge clk);cmd=0;repeat(4)@(negedge clk);end endtask
function integer diag(input integer v);begin case(v)
1:diag=36;2:diag=74;3:diag=110;5:diag=185;6:diag=222;default:diag=148;endcase end endfunction
initial begin
 #31;rst=1;send(1);if(f!=210 || lf!=210 || rf!=210)$fatal(1,"W forward");
 send(2);if(f!=-210)$fatal(1,"S reverse");
 send(4);if(l!=210 || lf!=-210 || rf!=210 || lb!=210 || rb!=-210)$fatal(1,"A left");
 send(8);if(l!=-210)$fatal(1,"D right");
 send(5);if(f!=148 || l!=148 || lf!=0 || rf!=296 || lb!=296 || rb!=0)$fatal(1,"Normalized diagonal");
 blocked=1;repeat(4)@(negedge clk);if(f!=0 || l!=210 || lf!=-210 || rf!=210)$fatal(1,"Front block inhibited free left component");
 send(1);if(lf || rf || lb || rb)$fatal(1,"Front block permitted W");
 send(2);if(f!=-210)$fatal(1,"Front block inhibited reverse escape");
 blocked=15;send(32);if(u!=-110 || lf!=110 || rf!=-110)$fatal(1,"CW side button");
 send(16);if(u!=110 || lf!=-110 || rf!=110)$fatal(1,"CCW side button");
 blocked=0;send(3);if(f || l || u || lf || rf)$fatal(1,"Opposite keys did not cancel");
 send(1);send(0);if(active || lf || rf || lb || rb)$fatal(1,"Release not zero");
 send(1);repeat(3010)@(negedge clk);if(lease || active || lf || rf)$fatal(1,"Key lease expiry failed");
 send(1);fresh=0;repeat(4)@(negedge clk);if(active || lf || rf)$fatal(1,"Stale radar not stopped");
 fresh=1;send(1);stop=1;repeat(3)@(negedge clk);if(active || lf || rf)$fatal(1,"STOP not immediate");
 stop=0;fresh=1;blocked=0;
 for(integer i=1;i<=6;i=i+1)begin
  level=i;send(1);if(f!=210*i/4 || l)$fatal(1,"Pipelined speed level %d",i);
  send(2);if(f!=-(210*i/4))$fatal(1,"Pipelined reverse level %d",i);
  send(5);if(f!=diag(i) || l!=f || lf || rb || rf!=2*f || lb!=2*f)$fatal(1,"Pipelined diagonal level %d",i);
 end
 $display("PASS manual: six speed/diagonal levels, WASD, independent block/escape, both turns, cancellation, release, 300ms lease, stale radar, STOP");$finish;
end
endmodule

module directional_tb;
reg clk=0;always #5 clk=~clk;
reg rst=0,pv=0,bad=0;reg signed [15:0] f=1000,l=1000;reg [15:0] angle=0,range=1400;
reg [15:0] fb=250,lr=180;reg [1:0] mode=1;wire fresh;wire [3:0] blocked;
directional_clearance #(.CLK_HZ(100000)) dut(clk,rst,bad,pv,f,l,angle,range,fresh,blocked,fb,lr,mode);
task point(input [15:0] a,input signed [15:0] x,y,input [15:0] r);begin
 @(negedge clk);angle=a;f=x;l=y;range=r;pv=1;@(negedge clk);pv=0;repeat(3)@(negedge clk);
end endtask
task scan;begin for(integer i=0;i<180;i=i+1)point(i*200,1000,1000,1400);end endtask
task open_scan;begin for(integer i=0;i<180;i=i+1)point(i*200,10000,0,12000);end endtask
task empty_scan;begin for(integer i=0;i<180;i=i+1)point(i*200,0,0,0);end endtask
initial begin
 #31;rst=1;scan();scan();point(0,1000,1000,1400);if(!fresh || blocked)$fatal(1,"Full scan did not enable clear directions");
 point(200,200,100,224);if(blocked)$fatal(1,"Side-front return blocked forward");
 point(400,200,0,200);if(blocked!=1)$fatal(1,"Front narrow cone mask");
 point(600,0,150,150);if(blocked!=5)$fatal(1,"Independent left cone");
 point(800,-200,0,200);if(blocked!=7)$fatal(1,"Independent back cone");
 point(1000,0,-150,150);if(blocked!=15)$fatal(1,"Independent right cone");
 scan();scan();scan();point(0,1000,1000,1400);if(blocked)$fatal(1,"Old obstacle never cleared");
 open_scan();open_scan();open_scan();point(0,10000,0,12000);if(!fresh || blocked)$fatal(1,"Open field falsely required near walls");
 empty_scan();empty_scan();empty_scan();point(0,0,0,0);if(!fresh || blocked)$fatal(1,"CRC-valid empty scan falsely required wall returns");
 fb=400;lr=250;mode=2;
 point(200,350,100,364);if(blocked!=1)$fatal(1,"Wider/longer front cone not effective");
 point(400,80,240,253);if(blocked!=5)$fatal(1,"New side range not effective");
 scan();scan();scan();point(0,1000,1000,1400);
 point(200,350,150,381);if(blocked)$fatal(1,"Medium cone includes wide side return");
 mode=3;point(400,350,150,381);if(blocked!=1)$fatal(1,"Wide cone mode ignored");
 scan();scan();scan();point(0,1000,1000,1400);
 fb=200;point(200,350,0,350);if(blocked)$fatal(1,"Configurable front limit ignored");
 repeat(25010)@(negedge clk);if(fresh || blocked!=15)$fatal(1,"Stale scan not inhibited");
 bad=1;@(negedge clk);if(fresh)$fatal(1,"Bad stream not invalidated");
 $display("PASS directional: open/empty scans need no walls, full-scan freshness, side-front excluded, four independent +/-10.6 degree cones, removal, stale/bad scan");$finish;
end
endmodule
