// Self-contained sensor-to-pixel path. No PC, WiFi, or external image memory.
module nav_lidar #(
 parameter integer CLK_HZ=50000000,CELL_SHIFT=6,
 parameter [15:0] ROTATE_CDEG=0
)(input wire clk,rst_n,rx,display_start,
 input wire [7:0] pixel_x,input wire [6:0] pixel_y,output wire [15:0] pixel,output wire online,
 output wire point_valid,stream_bad,output wire [15:0] point_angle,point_range_mm);
 wire [7:0] byte_data;
 wire byte_valid,byte_error,scan_bit,front_valid;
 wire [13:0] scan_addr;
 wire [31:0] packets,crc_errors,drops,scans,invalid_packets;
 n10_uart_rx #(.CLK_HZ(CLK_HZ)) receiver(.clk(clk),.rst_n(rst_n),.rx(rx),
  .data(byte_data),.valid(byte_valid),.framing_error(byte_error));
 nav_scan_core #(.CLK_HZ(CLK_HZ),.CELL_SHIFT(CELL_SHIFT),.ROTATE_CDEG(ROTATE_CDEG)) core(
  .clk(clk),.rst_n(rst_n),.byte_data(byte_data),.byte_valid(byte_valid),.byte_error(byte_error),
  .display_start(display_start),.display_addr(scan_addr),.display_bit(scan_bit),.front_valid(front_valid),.online(online),
  .packet_count(packets),.crc_errors(crc_errors),.drop_count(drops),.scan_count(scans),.invalid_packets(invalid_packets),.point_valid(point_valid),.stream_bad(stream_bad),.point_angle(point_angle),.point_range_mm(point_range_mm));
 nav_scan_renderer #(.CELL_SHIFT(CELL_SHIFT)) renderer(.clk(clk),.rst_n(rst_n),.frame_start(display_start),
  .pixel_x(pixel_x),.pixel_y(pixel_y),.scan_bit(scan_bit),.front_valid(front_valid),.online(online),
  .packets(packets),.crc_errors(crc_errors+invalid_packets),.drops(drops),.scans(scans),.scan_addr(scan_addr),.pixel(pixel));
endmodule
