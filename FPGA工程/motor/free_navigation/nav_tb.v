`timescale 1ns/1ps
module basic_nav_tb;
 reg clk=0,rst=0,go=0,stop=0,tick=0,pu=0,rd=0,rangeok=1;
 reg [1:0] dir=1;reg [6:0] fl=60,bl=60,ll=60,rl=60;
 reg signed [15:0] x=900,y=700;reg [15:0] tx=1400,ty=1400;
 wire active,fresh,can;wire [3:0] status;wire signed [15:0] lf,rf,lb,rb;
 integer passed=0;reg [2:0] level=4;
 always #10 clk=~clk;
 basic_nav_drive #(.CLK_HZ(100000)) dut(clk,rst,1'b0,go,stop,tick,3'd5,1'b1,pu,1'b1,x,y,16'sd9000,
  tx,ty,16'd2000,16'd2000,rd,1'b1,1'b1,dir,rangeok,fl,bl,ll,rl,1'b0,1'b1,1'b0,16'sd0,16'sd0,
  active,fresh,can,status,lf,rf,lb,rb,level);
 task pose;begin pu=1;@(negedge clk);pu=0;repeat(200)@(negedge clk);end endtask
 task refresh;begin tick=1;@(negedge clk);tick=0;repeat(200)@(negedge clk);end endtask
 task route;begin rd=1;@(negedge clk);rd=0;repeat(200)@(negedge clk);end endtask
 initial begin
  #51;rst=1;repeat(5)pose();go=1;@(negedge clk);go=0;route();
  if(!active || lf!=140 || rf!=140)$fatal(1,"Forward cruise");
  fl=45;refresh();if(lf!=105 || rf!=105)$fatal(1,"Forward first slow");
  fl=30;refresh();if(lf!=70)$fatal(1,"Forward second slow");
  fl=15;refresh();if(lf!=35)$fatal(1,"Forward third slow");
  dir=0;rl=30;fl=60;route();if(lf!=70 || rf!=-70 || lb!=-70 || rb!=70)$fatal(1,"Right corridor selection");
  dir=2;ll=45;route();if(lf!=-105 || rf!=105)$fatal(1,"Left corridor selection");
  dir=3;bl=15;route();if(lf!=-35 || rf!=-35)$fatal(1,"Rear corridor selection");
  level=6;dir=1;fl=60;route();if(lf!=210 || rf!=210)$fatal(1,"Navigation 150% scale");
  level=1;refresh();if(lf!=35 || rf!=35)$fatal(1,"Navigation 25% scale");
  fl=0;refresh();if(lf || rf || lb || rb)$fatal(1,"Zero clearance bypassed by scale");
  fl=60;level=4;refresh();go=1;@(negedge clk);go=0;route();
  x=tx;y=ty;pose();route();if(lf || rf || lb || rb)$fatal(1,"Moved inside arrival tolerance after replan");
  repeat(3)pose();if(active || status!=5)$fatal(1,"Arrival confirmation");
  x=900;y=700;repeat(4)pose();go=1;@(negedge clk);go=0;dir=1;route();rangeok=0;
  repeat(4)@(negedge clk);if(active || lf || rf)$fatal(1,"Stale ranging not stopped");
  passed=1;$display("PASS faster basic nav distance-dependent speed in all four directions, arrival hold, stale-range stop");$finish;
 end
endmodule
