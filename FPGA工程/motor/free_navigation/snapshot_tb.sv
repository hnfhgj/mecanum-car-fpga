`timescale 1ns/1ps
module snapshot_tb;
reg clk=0;always #5 clk=~clk;
reg rst=0,pv=0,bad=0,take=0,busy=0;reg [15:0] angle=0,range=0;reg [7:0] address=90;
wire [7:0] data;wire ready;wire [15:0] seq;
radar_snapshot #(.CLK_HZ(1000)) dut(clk,rst,bad,pv,angle,range,take,busy,address,data,ready,seq);
task point(input integer i,input integer shift);begin
 @(negedge clk);angle=i*200;range=(i+1+shift)*20;pv=1;@(negedge clk);pv=0;
end endtask
task scan(input integer shift);begin
 point(0,shift);repeat(200)@(negedge clk);for(integer i=1;i<180;i=i+1)point(i,shift);
end endtask
task empty_scan;begin
 for(integer i=0;i<180;i=i+1)begin
  @(negedge clk);angle=i*200;range=0;pv=1;@(negedge clk);pv=0;
  if(i==0)repeat(200)@(negedge clk);
 end
end endtask
initial begin
 #31;rst=1;repeat(200)@(negedge clk);scan(0);scan(0);scan(0);repeat(3)@(negedge clk);
 if(!ready || seq!=1 || data!=91)$fatal(1,"Snapshot publish/RAM data %d %d %d",ready,seq,data);
 take=1;busy=1;@(negedge clk);take=0;
 scan(10);scan(10);scan(10);repeat(3)@(negedge clk);
 if(ready || seq!=1 || data!=91)$fatal(1,"Reader bank overwritten during sending");
 busy=0;scan(10);scan(10);repeat(3)@(negedge clk);
 if(!ready || seq!=2 || data!=101)$fatal(1,"Next snapshot not committed");
 bad=1;@(negedge clk);if(ready)$fatal(1,"Bad stream snapshot not invalidated");
 bad=0;empty_scan();empty_scan();empty_scan();repeat(3)@(negedge clk);
 if(!ready || seq!=3 || data!=0)$fatal(1,"Empty open-field scan did not publish valid empty snapshot");
 $display("PASS snapshot: valid empty open-field scan, 2-degree/20mm quantization, complete scan publication, immutable queued/reading bank, next snapshot, bad stream");$finish;
end
endmodule
