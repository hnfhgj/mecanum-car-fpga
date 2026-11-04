// P4 camera and N10 RX bindings are kept in constraints/lidar_board.cst.
// USE_EXTERNAL_OSC=1 is retained for isolated pipeline simulation only.
// Normal hardware mode uses a 24MHz PLLA and waits for its lock before SCCB.
module square_nav_top #(
 parameter integer MOTORS_ENABLE=1,
 parameter integer USE_EXTERNAL_OSC=0
)(
 input wire clk,rst_n,
 input wire lidar_rx,
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
 nav_camera #(.USE_EXTERNAL_OSC(1)) u_camera(
  .clk(clk),.rst_n(camera_ready),.pclk(camera_pclk),.vsync(camera_vsync),.href(camera_href),.data(camera_data),
  .xclk(),.reset_n(camera_reset_n),.pwdn(),.scl(camera_scl),.sda(camera_sda),
  .init_done(init_done),.fault(config_fault),.frame_valid(frame_valid),
  .rd_addr(rd_addr),.display_frame_start(frame_start),.display_frame_end(frame_end),.rd_pixel(pixel));

 // Navigation-specific control starts here; original motor_top is untouched.
 wire [3:0] faults,running;wire [15:0] reasons;
 wire [15:0] counts[0:3];wire signed [15:0] speeds[0:3],raw_speeds[0:3],duties[0:3],targets[0:3];
 wire [3:0] ticks;wire [3:0] sign_flags,cal_flags;
 wire [3:0] enc_a={rb_eb,lb_ea,rf_ea,lf_eb},enc_b={rb_ea,lb_eb,rf_eb,lf_ea};
 wire [3:0] pwm_i,dir_i;
 assign {rb_pwm_i,lb_pwm_i,rf_pwm_i,lf_pwm_i}=pwm_i;
 assign {rb_dir_i,lb_dir_i,rf_dir_i,lf_dir_i}=dir_i;
 wire [2:0] page;wire menu_view;wire [3:0] cursor,gain_selection;
 wire [31:0] heading_kp,heading_kd;
 wire [479:0] gains={heading_kd,32'd0,heading_kp,{4{32'd205,32'd123,32'd1229}}};
 wire [15:0] ax,ay,az,gx,gy,gz,tof_distance;
 wire imu_valid,imu_init,tof_valid,tof_init;
 wire [5:0] tof_state;wire [15:0] tof_retries;
 wire signed [15:0] roll,pitch,yaw;wire attitude_valid,attitude_ready;
 imu660ra imu(.clk(clk),.rst_n(rst_n),.acc_x(ax),.acc_y(ay),.acc_z(az),.gyro_x(gx),.gyro_y(gy),.gyro_z(gz),
  .data_valid(imu_valid),.init_done(imu_init),.spi_sck(imu_sck),.spi_mosi(imu_mosi),.spi_cs_n(imu_cs_n),.spi_miso(imu_miso));
 nav_attitude attitude(.clk(clk),.rst_n(rst_n),.sample_valid(imu_valid),.ax(ax),.ay(ay),.az(az),.gx(gx),.gy(gy),.gz(gz),
  .roll(roll),.pitch(pitch),.yaw(yaw),.angle_valid(attitude_valid),.ready(attitude_ready));
 dl1b #(.I2C_FREQ(100000)) tof(.clk(clk),.rst_n(rst_n),.distance_mm(tof_distance),.data_valid(tof_valid),.init_done(tof_init),
  .debug_state(tof_state),.retry_count(tof_retries),.scl(tof_scl),.sda(tof_sda),.xs(tof_xs),.int_n(tof_int));
 wire [7:0] lidar_x;wire [6:0] lidar_y;wire lidar_frame_start;
 wire [15:0] raw_pixel,nav_pixel,raw_angle,raw_range;wire raw_point,raw_bad,lidar_online;
 nav_lidar radar(.clk(clk),.rst_n(rst_n),.rx(lidar_rx),.display_start(lidar_frame_start),.pixel_x(lidar_x),.pixel_y(lidar_y),.pixel(raw_pixel),
  .online(lidar_online),.point_valid(raw_point),.stream_bad(raw_bad),.point_angle(raw_angle),.point_range_mm(raw_range));
 wire point_valid,point_ready,point_overflow;wire signed [15:0] point_x,point_y;wire [15:0] point_angle,point_range;
 reg raw_bad_q;
 nav_point_stream stream(.clk(clk),.rst_n(rst_n),.bad(raw_bad_q),.point_valid(raw_point),.angle_cdeg(raw_angle),.range_mm(raw_range),
  .ready(point_ready),.valid(point_valid),.x(point_x),.y(point_y),.angle(point_angle),.range_out(point_range),.overflow(point_overflow));
 wire calibrate_pulse,go_pulse,stop_pulse;
 wire [15:0] target_x,target_y,width_mm,height_mm,quality;
 wire pose_update,pose_ok,calibrated,scan_ready,release_scan;
 wire signed [15:0] pose_x,pose_y,pose_yaw,correction;
 wire signed [15:0] scan_forward,scan_left;wire [8:0] scan_addr;wire [9:0] scan_count;
 wire active,fresh,can_go;wire [3:0] nav_status;
 assign point_ready=1'b1;
 // Bound the packet validation path before distributing cancellation.
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)raw_bad_q<=0;else raw_bad_q<=raw_bad;
 end
 wire stream_bad=raw_bad_q || point_overflow;
 square_frame_fit localization(.clk(clk),.rst_n(rst_n),.bad(stream_bad),.calibrate(calibrate_pulse),
  .point_valid(point_valid),.point_forward(point_x),.point_left(point_y),.point_angle(point_angle),.point_range(point_range),
  .imu_yaw(yaw),.release_scan(release_scan),.read_addr(scan_addr),.read_forward(scan_forward),.read_left(scan_left),
  .point_count(scan_count),.scan_ready(scan_ready),.pose_update(pose_update),.pose_ok(pose_ok),.calibrated(calibrated),
  .pose_x(pose_x),.pose_y(pose_y),.pose_yaw(pose_yaw),.correction(correction),.width_mm(width_mm),.height_mm(height_mm),.quality(quality));
 square_ui ui(.clk(clk),.rst_n(rst_n),.key_n(key_n),.active(active),.calibrated(calibrated),
  .width_mm(width_mm),.height_mm(height_mm),.page(page),.menu_view(menu_view),.cursor(cursor),
  .target_x(target_x),.target_y(target_y),.calibrate_pulse(calibrate_pulse),.go_pulse(go_pulse),.stop_pulse(stop_pulse),
  .gain_selection(gain_selection),.heading_kp(heading_kp),.heading_kd(heading_kd));
 wire map_busy,map_done,map_valid,near_obstacle,map_we,map_free;wire [11:0] map_addr;
 reg scan_owned;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)scan_owned<=0;
  else if(stream_bad || calibrate_pulse || release_scan)scan_owned<=0;
  else if(scan_ready && !scan_owned)scan_owned<=1;
 end
 assign release_scan=map_done;
 square_map map_builder(.clk(clk),.rst_n(rst_n),.cancel(stream_bad || calibrate_pulse),
  .start(scan_ready && !scan_owned),.point_count(scan_count),.read_forward(scan_forward),.read_left(scan_left),.read_addr(scan_addr),
  .pose_x(pose_x),.pose_y(pose_y),.pose_yaw(pose_yaw),.width_mm(width_mm),.height_mm(height_mm),
  .busy(map_busy),.done(map_done),.valid(map_valid),.near_obstacle(near_obstacle),.map_we(map_we),.map_addr(map_addr),.map_free(map_free));
 wire route_busy,route_done,route_found,route_goal,write_rejected;wire [1:0] route_direction;
 wire [11:0] origin_cell=((pose_y/50)<<6)+(pose_x/50),goal_cell=((target_y/50)<<6)+(target_x/50);
 square_grid_route #(.EXPLORE(1)) planner(.clk(clk),.rst_n(rst_n),.cancel(stream_bad || calibrate_pulse || stop_pulse || go_pulse),
  .map_we(map_we),.map_addr(map_addr),.map_free(map_free),.start(map_done && map_valid),.origin(origin_cell),.goal(goal_cell),
  .busy(route_busy),.done(route_done),.found(route_found),.direction(route_direction),.write_rejected(write_rejected),
  .new_goal(go_pulse || calibrate_pulse),.goal_reached(route_goal));
 wire signed [15:0] turn,heading_yaw,heading_left,heading_right;wire heading_ready,heading_fault;
 wire quiet=$signed(gx)<=164 && $signed(gx)>=-164 && $signed(gy)<=164 && $signed(gy)>=-164 && $signed(gz)<=164 && $signed(gz)>=-164;
 heading_hold #(.MAX_TURN(80),.DYNAMIC_GAINS(1)) heading(.clk(clk),.rst_n(rst_n && calibrated && !calibrate_pulse),
  .tick(ticks[0]),.angle_valid(attitude_valid),.imu_ready(attitude_ready && imu_init),.quiet(quiet),.hold_turn(1'b0),
  .yaw(yaw),.base_speed(16'sd0),.target_yaw(-correction),.relative_yaw(heading_yaw),.turn(turn),.left_target(heading_left),.right_target(heading_right),
  .ready(heading_ready),.fault(heading_fault),.kp_live(heading_kp),.kd_live(heading_kd));
 square_drive control(.clk(clk),.rst_n(rst_n),.calibrate(calibrate_pulse),.go(go_pulse),.stop(stop_pulse),.page(page),
  .calibrated(calibrated),.pose_update(pose_update),.pose_ok(pose_ok && lidar_online),
  .pose_x(pose_x),.pose_y(pose_y),.pose_yaw(pose_yaw),.target_x(target_x),.target_y(target_y),.width_mm(width_mm),.height_mm(height_mm),
  .route_done(route_done),.route_found(route_found && !write_rejected),.route_goal(route_goal),.route_direction(route_direction),
  .near_obstacle(near_obstacle),.heading_ready(heading_ready && !heading_fault),.wheel_fault(|faults),.heading_turn(turn),.heading_error(heading_yaw+correction),
  .active(active),.fresh(fresh),.can_go(can_go),.status(nav_status),
  .lf_target(targets[0]),.rf_target(targets[1]),.lb_target(targets[2]),.rb_target(targets[3]));
 genvar w;
 generate for(w=0;w<4;w=w+1)begin: wheels
  quad_decoder encoder(.clk(clk),.rst_n(rst_n),.a(enc_a[w]),.b(enc_b[w]),.count(counts[w]));
  speed_measure measure(.clk(clk),.rst_n(rst_n),.count_in(counts[w]),.speed(raw_speeds[w]),.tick(ticks[w]));
  wheel_speed_loop #(.KP(1229),.KI(123),.KD(205),.MAX_DUTY(882),.START_RAMP_STEP(15),.START_TICKS(60),.FIXED_ENCODER_SIGN(1)) loop(
   .clk(clk),.rst_n(rst_n),.enable(1'b1),.tick(ticks[w]),.target(targets[w]),.raw_speed(raw_speeds[w]),.measured(speeds[w]),
   .command(duties[w]),.running(running[w]),.fault(faults[w]),.encoder_reverse(sign_flags[w]),.calibration_done(cal_flags[w]),
   .fault_reason(reasons[w*4+:4]),.kp_live(32'd1229),.ki_live(32'd123),.kd_live(32'd205));
  wire signed [15:0] drive=(|faults)?16'sd0:-duties[w];
  motor_driver #(.PHASE_OFFSET(w*735)) driver(.clk(clk),.rst_n(rst_n),.speed(drive),.dir(dir_i[w]),.pwm(pwm_i[w]));
 end endgenerate
 square_renderer overlay(.clk(clk),.rst_n(rst_n),.frame_start(lidar_frame_start),.pixel_x(lidar_x),.pixel_y(lidar_y),.raw_pixel(raw_pixel),
  .menu_view(menu_view),.cursor(cursor),.status(nav_status),.aligned(calibrated),.can_go(can_go),.learn_ok(calibrated),.template_valid(calibrated),.ambiguous(1'b0),
  .live_count(4'd0),.frozen_count(4'd0),.o_id(4'd0),.x_id(4'd0),.y_id(4'd0),.live_xs(192'd0),.live_ys(192'd0),
  .frozen_xs(192'd0),.frozen_ys(192'd0),.pose_x(pose_x),.pose_y(pose_y),.target_x(target_x),.target_y(target_y),
  .width_mm(width_mm),.height_mm(height_mm),.diameter_mm(correction<0?-correction:correction),.quality(quality),.pixel(nav_pixel));
 wire [7:0] flags={1'b0,route_busy,calibrated,calibrated,fresh,can_go,active,heading_ready};
 wire [167:0] diagnostics={24'h535101,flags,{4'd0,nav_status},4'd0,scan_count[3:0],{4'd0,cursor},pose_x,pose_y,pose_yaw,target_x,target_y,quality,16'd0};
 uart_enc_debug #(.DIAGNOSTIC_ENABLE(1)) serial(.clk(clk),.rst_n(rst_n),.lf_count(counts[0]),.rf_count(counts[1]),.lb_count(counts[2]),.rb_count(counts[3]),
  .lf_speed(speeds[0]),.rf_speed(speeds[1]),.lb_speed(speeds[2]),.rb_speed(speeds[3]),.acc_x(ax),.acc_y(ay),.acc_z(az),.gyro_x(gx),.gyro_y(gy),.gyro_z(gz),
  .distance_mm(tof_distance),.imu_init(imu_init),.imu_valid(imu_valid),.tof_init(tof_init),.tof_valid(tof_valid),.target_speed(16'd0),
  .run_mask(running),.fault_mask(faults),.attitude_roll(roll),.attitude_pitch(pitch),.attitude_yaw(yaw),.attitude_ready(attitude_ready),
  .heading_left(targets[0]),.heading_right(targets[1]),.heading_turn(turn),.heading_fault(heading_fault),.calibration_mask(cal_flags),.encoder_sign_mask(4'b1001),
  .fault_reasons(reasons),.diagnostic_stop(!active),.diagnostic_bits(diagnostics),.uart_tx(uart_tx));
 nav_tft_display #(.DASHBOARD_ENABLE(1),.TELEMETRY_PAGES(1),.CAMERA_PAGE_ENABLE(1),.LIDAR_PAGE_ENABLE(1)) display(
  .clk(clk),.rst_n(rst_n),.lidar_pixel(nav_pixel),.lidar_x(lidar_x),.lidar_y(lidar_y),.lidar_frame_start(lidar_frame_start),
  .lf_count(counts[0]),.rf_count(counts[1]),.lb_count(counts[2]),.rb_count(counts[3]),.lf_speed(speeds[0]),.rf_speed(speeds[1]),.lb_speed(speeds[2]),.rb_speed(speeds[3]),
  .acc_x(ax),.acc_y(ay),.acc_z(az),.gyro_x(gx),.gyro_y(gy),.gyro_z(gz),.distance_mm(tof_distance),.imu_init(imu_init),.tof_init(tof_init),
  .attitude_roll(roll),.attitude_pitch(pitch),.attitude_yaw(yaw),.attitude_ready(attitude_ready),.heading_turn(turn),.heading_left(targets[0]),.heading_right(targets[1]),
  .heading_fault(heading_fault),.outputs_enabled(MOTORS_ENABLE!=0),.lf_duty(duties[0]),.rf_duty(duties[1]),.lb_duty(duties[2]),.rb_duty(duties[3]),
  .fault_reasons(reasons),.tof_retries(tof_retries),.calibration_mask(cal_flags),.encoder_sign_mask(4'b1001),.encoder_inputs({rb_eb,rb_ea,lb_eb,lb_ea,rf_eb,rf_ea,lf_eb,lf_ea}),
  .tof_state(tof_state),.target_speed(16'd0),.closed_loop(1'b1),.fault_mask(faults),.page(page),.selection(gain_selection),.gains_flat(gains),
  .camera_valid(frame_valid),.camera_init_done(init_done),.camera_fault(config_fault),.camera_pixel(pixel),.camera_rd_addr(rd_addr),.camera_frame_start(frame_start),.camera_frame_end(frame_end),
  .tft_scl(tft_scl),.tft_sda(tft_sda),.tft_reset(tft_reset),.tft_dc(tft_dc),.tft_cs_n(tft_cs_n),.tft_bl(tft_bl));
endmodule
