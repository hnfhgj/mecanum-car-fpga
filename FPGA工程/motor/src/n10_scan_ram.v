// Two independent scan bitmaps. Writer never touches the displayed bank.
// No memory reset: controller explicitly clears each bank before publication.
module n10_scan_ram(
 input wire clk,wr_en,wr_bank,input wire [13:0] wr_addr,input wire wr_bit,
 input wire rd_bank,input wire [13:0] rd_addr,output wire rd_bit
);
 (* ram_style="block",syn_ramstyle="block_ram" *) reg bank0[0:16383];
 (* ram_style="block",syn_ramstyle="block_ram" *) reg bank1[0:16383];
 reg q0,q1;
 always @(posedge clk)begin
  if(wr_en && !wr_bank)bank0[wr_addr]<=wr_bit;
  if(wr_en && wr_bank)bank1[wr_addr]<=wr_bit;
  q0<=bank0[rd_addr];q1<=bank1[rd_addr];
 end
 assign rd_bit=rd_bank?q1:q0;
endmodule
