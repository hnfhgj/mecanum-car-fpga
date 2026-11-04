// Standalone camera pipeline; deliberately supplies no project/pin bindings.
// Default clk=50MHz, XCLK=25MHz. USE_EXTERNAL_OSC=1 holds XCLK low: main
// must leave that output physically UNCONNECTED for an onboard oscillator.
// Never drive against a module oscillator. Module variant/routing unknown.
// Reference setup assumes24MHz (its top has no FPGA XCLK output).
// PLL/exposure/PCLK values need board validation at25MHz.
// Reset: >=1ms low, >=20ms boot, >=5ms after 3008=82. PWDN active high=0.
// frame_valid falls on reset, configuration failure, or 1s without a valid
// completed frame; rd_pixel is zero when invalid. RAM itself is not reset.
// Display reader: pulse START before address fetches; pulse END after last
// fetch. Read at clk, one-cycle latency. Address = y*160+x, range0..20479.
// Do not issue overlapping STARTs. Bank only changes on an accepted START.
// Constraints for main: clk period20ns; generated XCLK divide_by2; PCLK is
// a separate source-synchronous input clock (period/input delays from actual
// sensor settings + PCB). Constrain data/HREF/VSYNC to PCLK rising edge.
// Mark two-stage reset/toggle/enable synchronizers; cut ONLY async paths to
// first stages, retain stage1->stage2 timing. ready_bank is bundled with req:
// hold until ACK; constrain its path to front_bank within one clk period.
// RAM clocks asynchronous; inspect dual-clock RAM inference and CDC report.
// Memory logical655360bits; expected40 BSRAM at1024x18 packing (20/bank).
// Existing motor/impl/pnr/project.rpt.txt reports6/118 BSRAM on GW5AT-60 B;
// capacity is sufficient (estimated46/118 combined). Actual inference and
// current whole-design packing must be confirmed in main's synthesis report.
module nav_camera #(
    parameter integer CLK_FREQ = 50_000_000,
    parameter integer I2C_FREQ = 100_000,
    parameter integer USE_EXTERNAL_OSC = 0,
    parameter integer MAX_RETRIES = 3,
    parameter integer STALE_CYCLES = CLK_FREQ
)(
    input wire clk, input wire rst_n,
    input wire pclk, input wire vsync, input wire href, input wire [7:0] data,
    output wire xclk, output wire reset_n, output wire pwdn,
    output wire scl, inout wire sda,
    output wire init_done, output wire fault, output wire frame_valid,
    input wire [14:0] rd_addr,
    input wire display_frame_start, input wire display_frame_end,
    output wire [15:0] rd_pixel
);
    wire crst_n;
    ov5640_reset_sync main_reset(clk,rst_n,crst_n);
    reg xclk_div;
    always @(posedge clk or negedge crst_n)
        if (!crst_n) xclk_div <= 0;
        else xclk_div <= ~xclk_div;
    assign xclk = USE_EXTERNAL_OSC ? 1'b0 : xclk_div;
    wire [2:0] config_status;
    ov5640_config #(.CLK_FREQ(CLK_FREQ),.I2C_FREQ(I2C_FREQ),
        .MAX_RETRIES(MAX_RETRIES)) config_controller(
        .clk(clk),.rst_n(crst_n),.reset_n(reset_n),.pwdn(pwdn),
        .scl(scl),.sda(sda),.init_done(init_done),.fault(fault),
        .status(config_status));
    nav_camera_capture #(.STALE_CYCLES(STALE_CYCLES)) capture(
        .clk(clk),.rst_n(rst_n),.enable(init_done && !fault),
        .pclk(pclk),.vsync(vsync),.href(href),.data(data),.rd_addr(rd_addr),
        .display_frame_start(display_frame_start),
        .display_frame_end(display_frame_end),.rd_pixel(rd_pixel),
        .frame_valid(frame_valid));
endmodule
