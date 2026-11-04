// P4 camera and N10 RX bindings are kept in constraints/lidar_board.cst.
// USE_EXTERNAL_OSC=1 is retained for isolated pipeline simulation only.
// Normal hardware mode uses a 24MHz PLLA and waits for its lock before SCCB.
module free_navigation_top #(
 parameter integer MOTORS_ENABLE=1,
 parameter integer USE_EXTERNAL_OSC=0
)(
 input wire clk,rst_n,
 input wire lidar_rx,
 input wire wireless_rx,wireless_rts,output wire wireless_tx,
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
 wire [3:0] enc_a,enc_b,pwm_pins,dir_pins;
 wire [3:0] pwm_i,dir_i;
 wheel_channel_map physical_wheels(
  .raw_a({rb_ea,lb_ea,rf_ea,lf_ea}),.raw_b({rb_eb,lb_eb,rf_eb,lf_eb}),
  .logical_pwm(pwm_i),.logical_dir(dir_i),.encoder_a(enc_a),.encoder_b(enc_b),
  .pin_pwm(pwm_pins),.pin_dir(dir_pins));
 assign {rb_pwm_i,lb_pwm_i,rf_pwm_i,lf_pwm_i}=pwm_pins;
 assign {rb_dir_i,lb_dir_i,rf_dir_i,lf_dir_i}=dir_pins;
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
 wire calibrate_pulse,stop_pulse;
 wire [15:0] target_x,target_y,width_mm,height_mm,quality;
 wire pose_update,pose_ok,calibrated,scan_ready,release_scan;
 wire signed [15:0] pose_x,pose_y,pose_yaw,correction;
 wire signed [15:0] scan_forward,scan_left;wire [8:0] scan_addr;wire [9:0] scan_count;
 wire active,fresh,can_go;wire [3:0] nav_status;
 wire [2:0] group_id,motion_selection;wire slow_test,demo_start,single_start,route_start;
 wire [15:0] user_x,user_y;wire [63:0] points_x,points_y;wire [1:0] edit_index,point_index;
 wire mission_running,nav_active,nav_go,goal_change,route_mode,demo_running;
 wire [3:0] mission_status,demo_status;wire demo_heading_enable,slow_seen,stop_seen;
 wire signed [15:0] nav_targets[0:3],demo_targets[0:3];
 wire signed [15:0] demo_yaw,demo_turn,demo_target,demo_left,demo_right;
 wire demo_heading_ready,demo_heading_fault;
 wire range_fresh,range_near,range_update;
 wire [15:0] front_mm,back_mm,left_mm,right_mm;
 wire [6:0] front_limit,back_limit,left_limit,right_limit;
 reg [31:0] imu_age;
 wire imu_fresh=imu_age<12500000;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)imu_age<=32'hffffffff;
  else if(attitude_valid)imu_age<=0;
  else if(imu_age!=32'hffffffff)imu_age<=imu_age+1'b1;
 end
 wire [2:0] speed_level;
 wire quiet=$signed(gx)<=164 && $signed(gx)>=-164 && $signed(gy)<=164 && $signed(gy)>=-164 && $signed(gz)<=164 && $signed(gz)>=-164;
 wire stream_bad=raw_bad_q || point_overflow;
 wire [15:0] stop_fb_mm,stop_lr_mm;wire [1:0] cone_mode;wire [3:0] fit_reason;
 wire manual_active,manual_heading_enable,manual_heading_ready,manual_heading_fault,manual_lease,drive_valid,manual_range_fresh;
 wire [5:0] drive_mask,manual_requested;wire [3:0] manual_blocked;
 wire signed [15:0] manual_targets[0:3],manual_forward,manual_left,manual_rotation,manual_heading_turn;
 directional_clearance manual_ranges(.clk(clk),.rst_n(rst_n),.bad(stream_bad),.point_valid(point_valid),
  .forward_mm(point_x),.left_mm(point_y),.angle_cdeg(point_angle),.range_mm(point_range),.fresh(manual_range_fresh),.blocked(manual_blocked),.stop_fb_mm(stop_fb_mm),.stop_lr_mm(stop_lr_mm),.cone_mode(cone_mode));
 heading_hold #(.MAX_TURN(110),.DYNAMIC_GAINS(1),.QUIET_SAMPLES(1)) manual_heading(
  .clk(clk),.rst_n(rst_n && manual_heading_enable),.tick(ticks[0]),.angle_valid(attitude_valid),
  .imu_ready(attitude_ready && imu_init && imu_fresh),.quiet(quiet),.hold_turn(1'b0),.yaw(yaw),
  .base_speed(16'sd0),.target_yaw(16'sd0),.relative_yaw(),.turn(manual_heading_turn),.left_target(),.right_target(),
  .ready(manual_heading_ready),.fault(manual_heading_fault),.kp_live(heading_kp),.kd_live(heading_kd));
 hold_drive manual(.clk(clk),.rst_n(rst_n),.tick(ticks[0]),.stop(stop_pulse || calibrate_pulse || demo_running || mission_running),
  .command_valid(drive_valid),.command_mask(drive_mask),.imu_ready(attitude_ready && imu_init && imu_fresh),
  .range_fresh(manual_range_fresh),.heading_ready(manual_heading_ready),.heading_fault(manual_heading_fault),
  .blocked(manual_blocked),.speed_level(speed_level),.heading_turn(manual_heading_turn),.active(manual_active),
  .heading_enable(manual_heading_enable),.lease_valid(manual_lease),.requested(manual_requested),
  .forward_applied(manual_forward),.left_applied(manual_left),.turn_applied(manual_rotation),
  .lf_target(manual_targets[0]),.rf_target(manual_targets[1]),.lb_target(manual_targets[2]),.rb_target(manual_targets[3]));
 assign active=demo_running || mission_running || nav_active || manual_active;
 assign targets[0]=manual_active?manual_targets[0]:(demo_running?demo_targets[0]:(mission_running?nav_targets[0]:16'sd0));
 assign targets[1]=manual_active?manual_targets[1]:(demo_running?demo_targets[1]:(mission_running?nav_targets[1]:16'sd0));
 assign targets[2]=manual_active?manual_targets[2]:(demo_running?demo_targets[2]:(mission_running?nav_targets[2]:16'sd0));
 assign targets[3]=manual_active?manual_targets[3]:(demo_running?demo_targets[3]:(mission_running?nav_targets[3]:16'sd0));

 assign point_ready=1'b1;
 // Bound the packet validation path before distributing cancellation.
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)raw_bad_q<=0;else raw_bad_q<=raw_bad;
 end

 square_frame_fit localization(.clk(clk),.rst_n(rst_n),.bad(stream_bad),.calibrate(calibrate_pulse),
  .point_valid(point_valid),.point_forward(point_x),.point_left(point_y),.point_angle(point_angle),.point_range(point_range),
  .imu_yaw(yaw),.release_scan(release_scan),.read_addr(scan_addr),.read_forward(scan_forward),.read_left(scan_left),
  .point_count(scan_count),.scan_ready(scan_ready),.pose_update(pose_update),.pose_ok(pose_ok),.calibrated(calibrated),
  .pose_x(pose_x),.pose_y(pose_y),.pose_yaw(pose_yaw),.correction(correction),.width_mm(width_mm),.height_mm(height_mm),.quality(quality),.debug_reason(fit_reason));

 wire command_valid,control_frame_active,pc_rx_valid,pc_rx_error,ack_valid;
 wire [7:0] command_seq,command_op,pc_rx_data,ack_seq,ack_status;
 wire [15:0] command_a,command_b,command_c;
 // Framed wireless commands restored after independent hardware echo verification.
 pc_command_rx commands(.clk(clk),.rst_n(rst_n),.data(pc_rx_data),.valid(pc_rx_valid),.framing_error(pc_rx_error),
  .command_valid(command_valid),.seq(command_seq),.operation(command_op),.a(command_a),.b(command_b),.c(command_c),.in_frame(control_frame_active));
 pc_ui ui(.clk(clk),.rst_n(rst_n),.key_n(key_n),.active(active),.calibrated(calibrated),
  .remote_valid(command_valid),.remote_seq(command_seq),.remote_op(command_op),.remote_a(command_a),.remote_b(command_b),.remote_c(command_c),
  .motion_ready(attitude_ready && imu_init && imu_fresh),.range_ready(range_fresh),.nav_ready(can_go),
  .ack_valid(ack_valid),.ack_seq(ack_seq),.ack_status(ack_status),
  .width_mm(width_mm),.height_mm(height_mm),.page(page),.group_id(group_id),.menu_view(menu_view),.cursor(cursor),
  .motion_selection(motion_selection),.slow_test(slow_test),.demo_start(demo_start),
  .single_start(single_start),.route_start(route_start),.stop_pulse(stop_pulse),.calibrate_pulse(calibrate_pulse),
  .target_x(user_x),.target_y(user_y),.points_x(points_x),.points_y(points_y),.edit_index(edit_index),
  .gain_selection(gain_selection),.heading_kp(heading_kp),.heading_kd(heading_kd),.speed_level(speed_level),
  .manual_active(manual_active),.manual_range_ready(manual_range_fresh),.drive_valid(drive_valid),.drive_mask(drive_mask),.stop_fb_mm(stop_fb_mm),.stop_lr_mm(stop_lr_mm),.cone_mode(cone_mode));
 basic_clearance ranges(.clk(clk),.rst_n(rst_n),.bad(stream_bad),.point_valid(point_valid),
  .forward_mm(point_x),.left_mm(point_y),.angle_cdeg(point_angle),.range_mm(point_range),
  .fresh(range_fresh),.near_stop(range_near),.scan_update(range_update),.front_mm(front_mm),.back_mm(back_mm),
  .left_distance(left_mm),.right_mm(right_mm),.front_limit(front_limit),.back_limit(back_limit),
  .left_limit(left_limit),.right_limit(right_limit));
 basic_mission mission(.clk(clk),.rst_n(rst_n),.start_single(single_start && !demo_running),
  .start_route(route_start && !demo_running),.stop(stop_pulse || calibrate_pulse || demo_running),
  .single_x(user_x),.single_y(user_y),.points_x(points_x),.points_y(points_y),
  .can_go(can_go),.nav_active(nav_active),.nav_status(nav_status),.running(mission_running),.nav_go(nav_go),
  .goal_change(goal_change),.target_x(target_x),.target_y(target_y),.point_index(point_index),
  .route_mode(route_mode),.status(mission_status));
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
 square_grid_route #(.EXPLORE(1)) planner(.clk(clk),.rst_n(rst_n),.cancel(stream_bad || calibrate_pulse || stop_pulse || nav_go || goal_change || demo_running),
  .map_we(map_we),.map_addr(map_addr),.map_free(map_free),.start(map_done && map_valid),.origin(origin_cell),.goal(goal_cell),
  .busy(route_busy),.done(route_done),.found(route_found),.direction(route_direction),.write_rejected(write_rejected),
  .new_goal(nav_go || goal_change || calibrate_pulse),.goal_reached(route_goal));
 wire signed [15:0] turn,heading_yaw,heading_left,heading_right;wire heading_ready,heading_fault;

 heading_hold #(.MAX_TURN(110),.DYNAMIC_GAINS(1)) heading(.clk(clk),.rst_n(rst_n && calibrated && !calibrate_pulse),
  .tick(ticks[0]),.angle_valid(attitude_valid),.imu_ready(attitude_ready && imu_init && imu_fresh),.quiet(quiet),.hold_turn(1'b0),
  .yaw(yaw),.base_speed(16'sd0),.target_yaw(-correction),.relative_yaw(heading_yaw),.turn(turn),.left_target(heading_left),.right_target(heading_right),
  .ready(heading_ready),.fault(heading_fault),.kp_live(heading_kp),.kd_live(heading_kd));
 basic_nav_drive control(.clk(clk),.rst_n(rst_n),.calibrate(calibrate_pulse),.go(nav_go),.stop(stop_pulse || demo_running),.tick(ticks[0]),.page(page),
  .calibrated(calibrated),.pose_update(pose_update),.pose_ok(pose_ok && lidar_online),
  .pose_x(pose_x),.pose_y(pose_y),.pose_yaw(pose_yaw),.target_x(target_x),.target_y(target_y),.width_mm(width_mm),.height_mm(height_mm),
  .route_done(route_done),.route_found(route_found && !write_rejected),.route_goal(route_goal),.route_direction(route_direction),
  .clearance_fresh(range_fresh),.front_limit(front_limit),.back_limit(back_limit),.left_limit(left_limit),.right_limit(right_limit),
  .near_obstacle(near_obstacle || range_near),.heading_ready(heading_ready && !heading_fault),.wheel_fault(|faults),.heading_turn(turn),.heading_error(heading_yaw+correction),
  .speed_level(speed_level),.active(nav_active),.fresh(fresh),.can_go(can_go),.status(nav_status),
  .lf_target(nav_targets[0]),.rf_target(nav_targets[1]),.lb_target(nav_targets[2]),.rb_target(nav_targets[3]));
 heading_hold #(.MAX_TURN(110),.DYNAMIC_GAINS(1),.QUIET_SAMPLES(20)) demo_heading(
  .clk(clk),.rst_n(rst_n && demo_heading_enable),.tick(ticks[0]),.angle_valid(attitude_valid),
  .imu_ready(attitude_ready && imu_init && imu_fresh),.quiet(quiet),.hold_turn(1'b0),.yaw(yaw),
  .base_speed(16'sd0),.target_yaw(demo_target),.relative_yaw(demo_yaw),.turn(demo_turn),
  .left_target(demo_left),.right_target(demo_right),.ready(demo_heading_ready),.fault(demo_heading_fault),
  .kp_live(heading_kp),.kd_live(heading_kd));
 basic_demo demo(.clk(clk),.rst_n(rst_n),.tick(ticks[0]),.start(demo_start && !mission_running),
  .stop(stop_pulse || calibrate_pulse),.selection(motion_selection),.slow_test(slow_test),
  .imu_ready(attitude_ready && imu_init && imu_fresh && !demo_heading_fault),.heading_ready(demo_heading_ready),
  .wheel_fault(|faults),.range_fresh(range_fresh),.near_stop(range_near),.front_limit(front_limit),.back_limit(back_limit),
  .relative_yaw(demo_yaw),.turn(demo_turn),.running(demo_running),.heading_enable(demo_heading_enable),
  .target_yaw(demo_target),.lf_target(demo_targets[0]),.rf_target(demo_targets[1]),
  .lb_target(demo_targets[2]),.rb_target(demo_targets[3]),.speed_level(speed_level),.status(demo_status),.slow_seen(slow_seen),.stop_seen(stop_seen));
 genvar w;
 generate for(w=0;w<4;w=w+1)begin: wheels
  quad_decoder encoder(.clk(clk),.rst_n(rst_n),.a(enc_a[w]),.b(enc_b[w]),.count(counts[w]));
  speed_measure measure(.clk(clk),.rst_n(rst_n),.count_in(counts[w]),.speed(raw_speeds[w]),.tick(ticks[w]));
  wire signed [15:0] sampled_target,drive;
  pc_motor_pipeline pipeline(.clk(clk),.rst_n(rst_n),.blocked(!active || stop_pulse || calibrate_pulse || (|faults)),
   .selected_target(targets[w]),.pid_duty(duties[w]),.target(sampled_target),.drive(drive));
  basic_wheel_speed_loop #(.NO_FEEDBACK_GUARD(0),.KP(1229),.KI(123),.KD(205),.MAX_DUTY(882),.START_RAMP_STEP(15),.START_TICKS(60),.FIXED_ENCODER_SIGN(1)) loop(
   .clk(clk),.rst_n(rst_n),.enable(1'b1),.tick(ticks[w]),.target(sampled_target),.raw_speed(raw_speeds[w]),.measured(speeds[w]),
   .command(duties[w]),.running(running[w]),.fault(faults[w]),.encoder_reverse(sign_flags[w]),.calibration_done(cal_flags[w]),
   .fault_reason(reasons[w*4+:4]),.kp_live(32'd1229),.ki_live(32'd123),.kd_live(32'd205));
  motor_driver #(.PHASE_OFFSET(w*735)) driver(.clk(clk),.rst_n(rst_n),.speed(drive),.dir(dir_i[w]),.pwm(pwm_i[w]));
 end endgenerate
 basic_renderer overlay(.clk(clk),.rst_n(rst_n),.frame_start(lidar_frame_start),.pixel_x(lidar_x),.pixel_y(lidar_y),
  .raw_pixel(raw_pixel),.menu_view(menu_view),.group_id(group_id),.cursor(cursor),.demo_status(demo_status),
  .mission_status(mission_status),.edit_index(edit_index),.point_index(point_index),.route_mode(route_mode),
  .camera_ready(frame_valid && init_done),.imu_ready(attitude_ready && imu_init && imu_fresh),
  .range_ready(range_fresh),.calibrated(calibrated),.near_stop(range_near),.slow_seen(slow_seen),.stop_seen(stop_seen),
  .pose_x(pose_x),.pose_y(pose_y),.demo_yaw(demo_yaw),.demo_target(demo_target),
  .target_x(mission_running?target_x:user_x),.target_y(mission_running?target_y:user_y),
  .width_mm(width_mm),.height_mm(height_mm),.front_mm(front_mm),.back_mm(back_mm),.left_mm(left_mm),.right_mm(right_mm),
  .points_x(points_x),.points_y(points_y),.front_limit(front_limit),.pixel(nav_pixel));
 wire [7:0] flags={demo_running,mission_running,calibrated,range_fresh,fresh,can_go,active,heading_ready};
 wire [383:0] diagnostics={24'h425404,flags,{demo_status,mission_status},
  {group_id,route_mode,point_index,2'd0},{1'b0,speed_level,cursor},pose_x,pose_y,pose_yaw,target_x,target_y,front_mm,back_mm,left_mm,right_mm,width_mm,height_mm,
  {nav_status,slow_seen,stop_seen,range_near,imu_fresh},16'h97F4,
  {manual_active,manual_range_fresh,manual_lease,1'b0,manual_blocked},manual_forward,manual_left,manual_rotation,stop_fb_mm,stop_lr_mm,{6'd0,cone_mode},quality,{4'd0,fit_reason},{4'd0,(|faults),heading_fault,range_near,near_obstacle}};
 wire [7:0] scan_byte,cloud_addr;wire cloud_ready,scan_take,scan_busy;wire [15:0] scan_sequence;
 radar_snapshot cloud(.clk(clk),.rst_n(rst_n),.bad(stream_bad),.point_valid(point_valid),.angle(point_angle),.range_mm(point_range),
  .take(scan_take),.read_busy(scan_busy),.read_addr(cloud_addr),.read_data(scan_byte),.ready(cloud_ready),.scan_sequence(scan_sequence));
 wire [7:0] wireless_data;wire wireless_valid;
 pc_wireless_bridge wireless(.clk(clk),.rst_n(rst_n),.stream_data(wireless_data),.stream_valid(wireless_valid),
  .rx(wireless_rx),.rts(wireless_rts),.tx(wireless_tx),
  .ack_valid(ack_valid),.ack_seq(ack_seq),.ack_status(ack_status),.control_frame_active(control_frame_active),
  .rx_data(pc_rx_data),.rx_valid(pc_rx_valid),.rx_error(pc_rx_error));
 basic_uart_debug #(.DIAGNOSTIC_ENABLE(1)) serial(.clk(clk),.rst_n(rst_n),.lf_count(counts[0]),.rf_count(counts[1]),.lb_count(counts[2]),.rb_count(counts[3]),
  .lf_speed(speeds[0]),.rf_speed(speeds[1]),.lb_speed(speeds[2]),.rb_speed(speeds[3]),.acc_x(ax),.acc_y(ay),.acc_z(az),.gyro_x(gx),.gyro_y(gy),.gyro_z(gz),
  .distance_mm(tof_distance),.imu_init(imu_init),.imu_valid(imu_valid),.tof_init(tof_init),.tof_valid(tof_valid),.target_speed(16'd0),
  .run_mask(running),.fault_mask(faults),.attitude_roll(roll),.attitude_pitch(pitch),.attitude_yaw(yaw),.attitude_ready(attitude_ready),
  .heading_left(targets[0]),.heading_right(targets[1]),.heading_turn(demo_running?demo_turn:turn),.heading_fault(demo_running?demo_heading_fault:heading_fault),.calibration_mask(cal_flags),.encoder_sign_mask(4'b0110),
  .fault_reasons(reasons),.diagnostic_stop(!active),.diagnostic_bits(diagnostics),.scan_ready(cloud_ready),.scan_sequence(scan_sequence),.scan_data(scan_byte),.scan_take(scan_take),.scan_busy(scan_busy),.scan_addr(cloud_addr),.stream_data(wireless_data),.stream_valid(wireless_valid),.uart_tx(uart_tx));
 nav_tft_display #(.DASHBOARD_ENABLE(1),.TELEMETRY_PAGES(1),.CAMERA_PAGE_ENABLE(1),.LIDAR_PAGE_ENABLE(1)) display(
  .clk(clk),.rst_n(rst_n),.lidar_pixel(nav_pixel),.lidar_x(lidar_x),.lidar_y(lidar_y),.lidar_frame_start(lidar_frame_start),
  .lf_count(counts[0]),.rf_count(counts[1]),.lb_count(counts[2]),.rb_count(counts[3]),.lf_speed(speeds[0]),.rf_speed(speeds[1]),.lb_speed(speeds[2]),.rb_speed(speeds[3]),
  .acc_x(ax),.acc_y(ay),.acc_z(az),.gyro_x(gx),.gyro_y(gy),.gyro_z(gz),.distance_mm(tof_distance),.imu_init(imu_init),.tof_init(tof_init),
  .attitude_roll(roll),.attitude_pitch(pitch),.attitude_yaw(yaw),.attitude_ready(attitude_ready),.heading_turn(demo_running?demo_turn:turn),.heading_left(targets[0]),.heading_right(targets[1]),
  .heading_fault(demo_running?demo_heading_fault:heading_fault),.outputs_enabled(MOTORS_ENABLE!=0),.lf_duty(duties[0]),.rf_duty(duties[1]),.lb_duty(duties[2]),.rb_duty(duties[3]),
  .fault_reasons(reasons),.tof_retries(tof_retries),.calibration_mask(cal_flags),.encoder_sign_mask(4'b0110),.encoder_inputs({rb_eb,rb_ea,lb_eb,lb_ea,rf_eb,rf_ea,lf_eb,lf_ea}),
  .tof_state(tof_state),.target_speed(16'd0),.closed_loop(1'b1),.fault_mask(faults),.page(page),.selection(gain_selection),.gains_flat(gains),
  .camera_valid(frame_valid),.camera_init_done(init_done),.camera_fault(config_fault),.camera_pixel(pixel),.camera_rd_addr(rd_addr),.camera_frame_start(frame_start),.camera_frame_end(frame_end),
  .tft_scl(tft_scl),.tft_sda(tft_sda),.tft_reset(tft_reset),.tft_dc(tft_dc),.tft_cs_n(tft_cs_n),.tft_bl(tft_bl));
endmodule
