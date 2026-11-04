// P4 bindings are kept in camera_build/camera_verified.cst.
// USE_EXTERNAL_OSC=1 is retained for isolated pipeline simulation only.
// Normal hardware mode uses a 24MHz PLLA and waits for its lock before SCCB.
module camera_dashboard_top #(
 parameter integer MOTORS_ENABLE=1,
 parameter integer USE_EXTERNAL_OSC=0
)(
 input wire clk,rst_n,
 input wire [3:0] key_n,
 input wire lf_ea,lf_eb,rf_ea,rf_eb,lb_ea,lb_eb,rb_ea,rb_eb,
 output wire lf_dir,lf_pwm,rf_dir,rf_pwm,lb_dir,lb_pwm,rb_dir,rb_pwm,
 output wire uart_tx,
 output wire imu_sck,imu_mosi,imu_cs_n,input wire imu_miso,
 output wire tof_scl,inout wire tof_sda,output wire tof_xs,input wire tof_int,
 output wire tft_scl,tft_sda,tft_reset,tft_dc,tft_cs_n,tft_bl,
 input wire camera_pclk,camera_vsync,camera_href,input wire [7:0] camera_data,
 inout wire camera_xclk,
 output wire camera_reset_n,camera_scl,inout wire camera_sda
);
 wire lf_dir_i,lf_pwm_i,rf_dir_i,rf_pwm_i,lb_dir_i,lb_pwm_i,rb_dir_i,rb_pwm_i;
 assign lf_dir=MOTORS_ENABLE ? lf_dir_i : 1'b0;
 assign lf_pwm=MOTORS_ENABLE ? lf_pwm_i : 1'b0;
 assign rf_dir=MOTORS_ENABLE ? rf_dir_i : 1'b0;
 assign rf_pwm=MOTORS_ENABLE ? rf_pwm_i : 1'b0;
 assign lb_dir=MOTORS_ENABLE ? lb_dir_i : 1'b0;
 assign lb_pwm=MOTORS_ENABLE ? lb_pwm_i : 1'b0;
 assign rb_dir=MOTORS_ENABLE ? rb_dir_i : 1'b0;
 assign rb_pwm=MOTORS_ENABLE ? rb_pwm_i : 1'b0;
 wire init_done,config_fault,frame_valid,frame_start,frame_end,xclk_internal;
 wire camera_ready;
 wire [14:0] rd_addr;
 wire [15:0] pixel;
 assign camera_xclk=USE_EXTERNAL_OSC ? 1'bz : xclk_internal;
 generate if(USE_EXTERNAL_OSC) begin: g_external
  assign camera_ready=rst_n;
  assign xclk_internal=1'b0;
 end else begin: g_clock
  camera_clock_24 u_clock(.clk(clk),.rst_n(rst_n),.xclk(xclk_internal),.ready(camera_ready));
 end endgenerate
 // PWDN is already pulled low on the board; do not create an unbound IO.
 ov5640_camera #(.USE_EXTERNAL_OSC(1)) u_camera(
  .clk(clk),.rst_n(camera_ready),.pclk(camera_pclk),.vsync(camera_vsync),.href(camera_href),.data(camera_data),
  .xclk(),.reset_n(camera_reset_n),.pwdn(),.scl(camera_scl),.sda(camera_sda),
  .init_done(init_done),.fault(config_fault),.frame_valid(frame_valid),
  .rd_addr(rd_addr),.display_frame_start(frame_start),.display_frame_end(frame_end),.rd_pixel(pixel));
 motor_top #(.DASHBOARD_ENABLE(1),.CAMERA_PAGE_ENABLE(1),.OUTPUTS_ENABLED(MOTORS_ENABLE),
  .NO_FEEDBACK_GUARD(MOTORS_ENABLE ? 1 : 0)) u_car(
  .clk(clk),.rst_n(rst_n),.key_n(key_n),
  .camera_valid(frame_valid),.camera_init_done(init_done),.camera_fault(config_fault),
  .camera_pixel(pixel),.camera_rd_addr(rd_addr),.camera_frame_start(frame_start),.camera_frame_end(frame_end),
  .lf_ea(lf_ea),.lf_eb(lf_eb),.rf_ea(rf_ea),.rf_eb(rf_eb),
  .lb_ea(lb_ea),.lb_eb(lb_eb),.rb_ea(rb_ea),.rb_eb(rb_eb),
  .lf_dir(lf_dir_i),.lf_pwm(lf_pwm_i),.rf_dir(rf_dir_i),.rf_pwm(rf_pwm_i),
  .lb_dir(lb_dir_i),.lb_pwm(lb_pwm_i),.rb_dir(rb_dir_i),.rb_pwm(rb_pwm_i),
  .uart_tx(uart_tx),.imu_sck(imu_sck),.imu_mosi(imu_mosi),.imu_cs_n(imu_cs_n),.imu_miso(imu_miso),
  .tof_scl(tof_scl),.tof_sda(tof_sda),.tof_xs(tof_xs),.tof_int(tof_int),
  .tft_scl(tft_scl),.tft_sda(tft_sda),.tft_reset(tft_reset),.tft_dc(tft_dc),.tft_cs_n(tft_cs_n),.tft_bl(tft_bl));
endmodule
