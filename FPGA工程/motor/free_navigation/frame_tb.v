`timescale 1ns/1ps
module square_frame_tb;
 reg clk=0,rst_n=0,bad=0,cal=0,pv=0,release_scan=0;
 reg signed [15:0] f=0,l=0,imu=0;reg [15:0] angle=0,range_mm=1000;
 wire [3:0] reason;wire [9:0] count;wire ready,update,ok,calibrated;
 wire signed [15:0] x,y,yaw,correction;wire [15:0] width,height,quality;
 integer passed=0,fd,ret,i,j,cycles,af,al,aa,ar,missing=0;
 always #10 clk=~clk;
 square_frame_fit dut(clk,rst_n,bad,cal,pv,f,l,angle,range_mm,imu,release_scan,9'd0,
  , ,count,ready,update,ok,calibrated,x,y,yaw,correction,width,height,quality,reason);
 task feed_scan;begin
  if(missing==2)fd=$fopen("square_nav/frame_live.txt","r");
  else fd=$fopen(missing?"square_nav/frame_missing.txt":"square_nav/frame_fixture.txt","r");
  if(!fd)$fatal(1,"Missing fixture");
  for(i=0;i<450;i=i+1)begin
   ret=$fscanf(fd,"%h %h %h %h\n",af,al,aa,ar);if(ret!=4)$fatal(1,"Bad fixture");
   @(negedge clk);f=af;l=al;angle=aa;range_mm=ar;pv=1;@(negedge clk);pv=0;
  end
  $fclose(fd);
 end endtask
 initial begin
  #51;rst_n=1;@(negedge clk);cal=1;@(negedge clk);cal=0;
  feed_scan();feed_scan();feed_scan();
  cycles=0;while(!update && cycles<7000000)begin @(negedge clk);cycles=cycles+1;end
  if(!update || !ok || !calibrated)$fatal(1,"Frame calibration failed");
  if(x<850 || x>950 || y<650 || y>750 || width<1950 || width>2050 || height<1150 || height>1250 || reason!=9 ||
     correction<650 || correction>750)$fatal(1,"Wrong frame geometry %d %d %d %d correction %d",x,y,width,height,correction);
  if(!ready || count<400)$fatal(1,"Scan ownership lost");
  // No capture may modify the retained scan until the map consumer releases it.
  feed_scan();if(!ready || !ok)$fatal(1,"Ownership not retained");
  @(negedge clk);release_scan=1;@(negedge clk);release_scan=0;missing=2;
  feed_scan();feed_scan();feed_scan();
  cycles=0;while(!update && cycles<7000000)begin @(negedge clk);cycles=cycles+1;end
  if(!update || !ok || !calibrated || x<950 || x>1050 || y<650 || y>750 || width<1950 || width>2050)
   $fatal(1,"Live frame with interior obstacle lost position %d %d",x,y);
  @(negedge clk);bad=1;@(negedge clk);bad=0;@(negedge clk);
  if(ok || ready)$fatal(1,"Corrupted scan retained valid pose/map");
  @(negedge clk);rst_n=0;@(negedge clk);rst_n=1;missing=1;
  cal=1;@(negedge clk);cal=0;feed_scan();feed_scan();feed_scan();
  cycles=0;while(!update && cycles<7000000)begin @(negedge clk);cycles=cycles+1;end
  if(!update || ok || calibrated)$fatal(1,"Incomplete frame accepted");
  passed=1;$display("PASS rectangle frame RTL geometry/orientation/scan ownership/bad stream");$finish;
 end
endmodule
