// GW5AT-60 uses PLLA (UG306 table 5-11), not the 138K PLL primitive.
// 50MHz / 1 * 1 * 24 = 1200MHz VCO; /50 = 24MHz XCLK.
// The implementation tool calculates analog loop parameters.
module camera_clock_24(input wire clk, input wire rst_n,
 output wire xclk, output wire ready);
 wire locked;
 PLLA #(.FCLKIN("50.0"),.IDIV_SEL(1),.FBDIV_SEL(1),
  .MDIV_SEL(24),.ODIV0_SEL(50),.CLKFB_SEL("INTERNAL")) pll(
  .CLKIN(clk),.CLKFB(1'b0),.CLKOUT0(xclk),.LOCK(locked),
  .CLKOUT1(),.CLKOUT2(),.CLKOUT3(),.CLKOUT4(),.CLKOUT5(),.CLKOUT6(),
  .CLKFBOUT(),.MDRDO(),.RESET(~rst_n),.PLLPWD(1'b0),
  .RESET_I(1'b0),.RESET_O(1'b0),.PSSEL(3'b000),.PSDIR(1'b0),.PSPULSE(1'b0),
  .SSCPOL(1'b0),.SSCON(1'b0),.SSCMDSEL(7'b0),.SSCMDSEL_FRAC(3'b0),
  .MDWDI(8'b0),.MDCLK(1'b0),.MDOPC(2'b0),.MDAINC(1'b0));
 ov5640_reset_sync lock_release(clk,rst_n && locked,ready);
endmodule
