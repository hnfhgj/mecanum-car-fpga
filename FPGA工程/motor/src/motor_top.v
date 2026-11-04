//============================================================================
// motor_top.v —— 顶层：四轮麦轮电机 + 编码器速度闭环
//
// 每个轮子一条独立闭环链路：
//   编码器 A/B → quad_decoder(4×解码) → speed_measure(10ms测速)
//             → speed_pid(增量式PID) → motor_driver(PWM+DIR)
//
// 默认原地保持启动航向0度，航向PD外环生成左右轮速度差。
// TARGET_SPEED为平移基础速度（当前0），原麦轮接线和方向修正不变。
//
// USE_CLOSED_LOOP=0为有编码器反馈保护的固定TEST_SPEED短时开环诊断。
// 闭环默认持续运行，保留反馈异常保护；开环诊断10秒超时停机。
//============================================================================
module motor_top #(
    parameter integer DASHBOARD_ENABLE=0, // Legacy benches keep static gains/display.
    parameter integer OUTPUTS_ENABLED=1,
    parameter integer CAMERA_PAGE_ENABLE=0,
    parameter integer LIDAR_PAGE_ENABLE=0,
    parameter integer NO_FEEDBACK_GUARD=1,
    parameter integer USE_CLOSED_LOOP = 1,        // 四轮固定目标速度闭环
    parameter integer HEADING_ENABLE = 1,        // 航向外环；诊断仿真可单独关闭
    parameter integer HEADING_DIAGNOSTIC=0,      // 持续0度，不自动偏转/超时结束
    parameter integer HEADING_SOFT_START=1,     // 航向启动渐增PWM，不再固定15%冲击
    parameter signed [15:0] HEADING_PWM_LIMIT=1470, // 与已实测Flash相同50%，不是固定占空比
    parameter integer HEADING_FIXED_ENCODER=1, // 已验证A/B方向，不以手推运动重新学习
    parameter signed [15:0] TARGET_SPEED = 16'sd0, // 原地航向保持：平移速度0
    parameter signed [15:0] TEST_SPEED   = 16'sd441, // 441/2941≈15%，不可擅自提升
    parameter [3:0] TEST_WHEEL_MASK = 4'b1111, // 开环使能 bit0=LF,1=RF,2=LB,3=RB
    parameter signed [31:0] PID_KP=1229, PID_KI=123, PID_KD=205, // All four: Q12 .30/.03/.05
    parameter signed [15:0] MAX_DUTY=1470, // 约50%输出硬上限，不是固定输出
    parameter integer TRIANGLE_ENABLE=0,
    parameter integer OPEN_NO_FEEDBACK_TICKS=50, OPEN_TIMEOUT_TICKS=1000,
    parameter integer CLOSED_TIMEOUT_TICKS=0, // 0 disables diagnostic timeout, NOT feedback guards
    parameter [3:0] MOTOR_REVERSE = 4'b1111, // 用户确认四轮机械极性均反向
    parameter [3:0] ENCODER_REVERSE = 4'b1001 // Reverse LF/RB A/B in software, not motor DIR
)(
    input  wire clk,       // 系统时钟 50MHz
    input  wire rst_n,     // 复位，低有效
    input wire [3:0] key_n, // S1..S4; S0 remains reset.
    input wire lidar_rx,
    input wire camera_valid, camera_init_done, camera_fault,
    input wire [15:0] camera_pixel,
    output wire [14:0] camera_rd_addr,
    output wire camera_frame_start, camera_frame_end,

    // ---- 4 个正交编码器（A/B 各一）----
    input  wire lf_ea, lf_eb,   // 左前
    input  wire rf_ea, rf_eb,   // 右前
    input  wire lb_ea, lb_eb,   // 左后
    input  wire rb_ea, rb_eb,   // 右后

    // ---- 4 个电机（PWM + DIR）----
    output wire lf_dir, lf_pwm,
    output wire rf_dir, rf_pwm,
    output wire lb_dir, lb_pwm,
    output wire rb_dir, rb_pwm,

    // ---- 串口调试输出（与屏幕同时保留）----
    output wire       uart_tx,   // FPGA 引脚 M15 → 板上 CH340E RXD

    // ---- IMU660RA（SPI 接口）----
    output wire       imu_sck,   // SPI 时钟
    output wire       imu_mosi,  // SPI 主机出 → IMU SDI
    output wire       imu_cs_n,  // 片选，低有效
    input  wire       imu_miso,  // SPI 主机入 ← IMU SDO

    // ---- 逐飞 DL1B 激光测距（I2C 接口）----
    output wire       tof_scl,   // I2C 时钟
    inout  wire       tof_sda,   // I2C 数据（双向）
    output wire       tof_xs,    // XSHUT 复位脚
    input  wire       tof_int,   // 中断脚（低有效，可选）

    // ---- SPI TFT180：P7-31～36 ----
    output wire tft_scl, tft_sda, tft_reset, tft_dc, tft_cs_n, tft_bl
);

    wire [3:0] wheel_fault;
    wire [3:0] wheel_running;
    wire lf_tick;
    wire system_fault;
    wire [3:0] calibration_mask,encoder_sign_mask;
    wire [15:0] fault_reasons;
    // Independent input probe: observe transitions BEFORE encoder direction correction/decoding.
    reg [7:0] enc_meta,enc_sync,enc_previous;
    reg [15:0] edge_count[0:3],bad_count[0:3];
    integer probe_i;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            enc_meta<=0; enc_sync<=0; enc_previous<=0;
            for(probe_i=0;probe_i<4;probe_i=probe_i+1) begin edge_count[probe_i]<=0; bad_count[probe_i]<=0; end
        end else begin
            enc_meta<={rb_eb,rb_ea,lb_eb,lb_ea,rf_eb,rf_ea,lf_eb,lf_ea};
            enc_sync<=enc_meta; enc_previous<=enc_sync;
            for(probe_i=0;probe_i<4;probe_i=probe_i+1) begin
                if(enc_sync[probe_i*2 +: 2]!=enc_previous[probe_i*2 +: 2] && edge_count[probe_i]!=16'hffff)
                    edge_count[probe_i]<=edge_count[probe_i]+1'b1;
                if((enc_sync[probe_i*2 +: 2]^enc_previous[probe_i*2 +: 2])==2'b11 && bad_count[probe_i]!=16'hffff)
                    bad_count[probe_i]<=bad_count[probe_i]+1'b1;
            end
        end
    end
    wire [2:0] dashboard_page;
    wire [3:0] dashboard_selection;
    wire [479:0] dashboard_gains;
    generate if(DASHBOARD_ENABLE) begin: g_dashboard
        dashboard_keys #(.PAGE_COUNT(LIDAR_PAGE_ENABLE ? 6 : (CAMERA_PAGE_ENABLE ? 5 : 4)),
            .INITIAL_PAGE(LIDAR_PAGE_ENABLE ? 5 : 0),.WHEEL_KP(PID_KP),.WHEEL_KI(PID_KI),.WHEEL_KD(PID_KD)) u_keys(
            .clk(clk),.rst_n(rst_n),.key_n(key_n),.page(dashboard_page),
            .selection(dashboard_selection),.gains_flat(dashboard_gains));
    end else begin: g_legacy_dashboard
        assign dashboard_page=0;
        assign dashboard_selection=0;
        assign dashboard_gains={32'd2560,32'd0,32'd320,
            PID_KD,PID_KI,PID_KP,PID_KD,PID_KI,PID_KP,
            PID_KD,PID_KI,PID_KP,PID_KD,PID_KI,PID_KP};
    end endgenerate
    localparam integer HEADING_ACTIVE=HEADING_ENABLE && USE_CLOSED_LOOP && !TRIANGLE_ENABLE;
    localparam integer LOOP_START_STEP=(HEADING_ACTIVE && HEADING_SOFT_START) ? 15 : 0;
    localparam integer LOOP_START_TICKS=(HEADING_ACTIVE && HEADING_SOFT_START) ? 60 : 20;
    localparam integer LOOP_FIXED_ENCODER=HEADING_ACTIVE && HEADING_FIXED_ENCODER;
    localparam signed [15:0] LOOP_MAX_DUTY=(HEADING_ACTIVE && HEADING_DIAGNOSTIC && MAX_DUTY>441) ? 16'sd441 :
        ((HEADING_ACTIVE && MAX_DUTY>HEADING_PWM_LIMIT) ? HEADING_PWM_LIMIT : MAX_DUTY);
    reg [15:0] heading_test_ticks;
    reg heading_test_stop;
    wire signed [15:0] heading_test_target=(HEADING_DIAGNOSTIC && heading_test_ticks>=200 && heading_test_ticks<400) ? 16'sd200 : 16'sd0;
    wire signed [15:0] heading_yaw,heading_turn,left_target,right_target;
    wire heading_ready,heading_fault;
    // Calibration 0.3s; stop 0.7s; +0..3000..0 (10s); stop 1s;
    // -0..3000..0 (10s); then remain at zero, never automatically repeat.
    reg [15:0] profile_ticks;
    reg signed [15:0] profile_target;
    wire signed [15:0] speed_target=TRIANGLE_ENABLE ? profile_target : TARGET_SPEED;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin profile_ticks<=0; profile_target<=0; end
        else if (lf_tick && USE_CLOSED_LOOP && TRIANGLE_ENABLE && !system_fault) begin
            if (profile_ticks<2200) profile_ticks<=profile_ticks+1'b1;
            if (profile_ticks<30) profile_target<=16'sd700;
            else if (profile_ticks<100) profile_target<=0;
            else if (profile_ticks<600) profile_target<=profile_target+16'sd6;
            else if (profile_ticks<1100) profile_target<=profile_target-16'sd6;
            else if (profile_ticks<1200) profile_target<=0;
            else if (profile_ticks<1700) profile_target<=profile_target-16'sd6;
            else if (profile_ticks<2200) profile_target<=profile_target+16'sd6;
            else profile_target<=0;
        end
    end
    reg [31:0] closed_elapsed;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin heading_test_ticks<=0; heading_test_stop<=0; end
        else if(HEADING_ACTIVE && HEADING_DIAGNOSTIC && heading_ready && !system_fault) begin
            if(heading_yaw>=500 || heading_yaw<=-500) heading_test_stop<=1;
            if(lf_tick) begin
                if(heading_test_ticks>=999) heading_test_stop<=1;
                else heading_test_ticks<=heading_test_ticks+1'b1;
            end
        end
    end
    assign system_fault=(|wheel_fault) || heading_fault || heading_test_stop || (USE_CLOSED_LOOP && CLOSED_TIMEOUT_TICKS>0 && closed_elapsed>=CLOSED_TIMEOUT_TICKS);
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) closed_elapsed<=0;
        else if (USE_CLOSED_LOOP && CLOSED_TIMEOUT_TICKS>0 && lf_tick && closed_elapsed<CLOSED_TIMEOUT_TICKS)
            closed_elapsed<=closed_elapsed+1'b1;
    end
    // Unattended open-loop diagnostic: stop on missing feedback or after 10s.
    reg [31:0] open_elapsed;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) open_elapsed<=0;
        else if (!USE_CLOSED_LOOP && lf_tick && open_elapsed<OPEN_TIMEOUT_TICKS)
            open_elapsed<=open_elapsed+1'b1;
    end
    reg [3:0] first_fault;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) first_fault<=0;
        else if (first_fault==0 && system_fault) first_fault<=(heading_fault || heading_test_stop) ? 4'b1111 : wheel_fault;
    end
    // ------------------------------------------------------------------
    // 左前轮
    // ------------------------------------------------------------------
    wire [15:0]        lf_count;
    wire signed [15:0] lf_speed, lf_raw_speed, lf_duty;
    quad_decoder u_lf_dec (
        .clk(clk), .rst_n(rst_n),
        .a(ENCODER_REVERSE[0] ? lf_eb : lf_ea),
        .b(ENCODER_REVERSE[0] ? lf_ea : lf_eb), .count(lf_count));
    speed_measure u_lf_meas (
        .clk(clk), .rst_n(rst_n), .count_in(lf_count), .speed(lf_raw_speed), .tick(lf_tick));
    generate if (USE_CLOSED_LOOP) begin: g_lf_closed
        wheel_speed_loop #(.NO_FEEDBACK_GUARD(NO_FEEDBACK_GUARD),.DYNAMIC_GAINS(0),.KP(PID_KP), .KI(PID_KI), .KD(PID_KD), .MAX_DUTY(LOOP_MAX_DUTY), .START_RAMP_STEP(LOOP_START_STEP), .START_TICKS(LOOP_START_TICKS), .FIXED_ENCODER_SIGN(LOOP_FIXED_ENCODER)) u_loop(
            .kp_live(dashboard_gains[0 +: 32]),.ki_live(dashboard_gains[32 +: 32]),.kd_live(dashboard_gains[64 +: 32]),
            .clk(clk), .rst_n(rst_n), .enable(!system_fault), .tick(lf_tick),
            .target(left_target), .raw_speed(lf_raw_speed),
            .measured(lf_speed), .command(lf_duty),
            .running(wheel_running[0]), .fault(wheel_fault[0]), .encoder_reverse(encoder_sign_mask[0]),
            .calibration_done(calibration_mask[0]),.fault_reason(fault_reasons[3:0]));
    end else begin: g_lf_open
        assign lf_speed=lf_raw_speed;
        assign lf_duty=TEST_WHEEL_MASK[0] ? TEST_SPEED : 16'sd0;
        reg [31:0] quiet_ticks;
        reg no_feedback;
        always @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin quiet_ticks<=0; no_feedback<=0; end
            else if (lf_tick && TEST_WHEEL_MASK[0] && !system_fault) begin
                if (lf_raw_speed>=2 || lf_raw_speed<=-2) quiet_ticks<=0;
                else if (quiet_ticks>=OPEN_NO_FEEDBACK_TICKS-1) no_feedback<=1;
                else quiet_ticks<=quiet_ticks+1'b1;
            end
        end
        assign wheel_fault[0]=TEST_WHEEL_MASK[0] &&
            (no_feedback || open_elapsed>=OPEN_TIMEOUT_TICKS);
        assign wheel_running[0]=1'b0;
        assign calibration_mask[0]=0; assign encoder_sign_mask[0]=0; assign fault_reasons[3:0]=0;
    end endgenerate
    wire signed [15:0] lf_command = system_fault ? 16'sd0 : lf_duty;
    motor_driver #(.PHASE_OFFSET(0)) u_lf_drv (
        .clk(clk), .rst_n(rst_n), .speed(MOTOR_REVERSE[0] ? -lf_command : lf_command), .dir(lf_dir), .pwm(lf_pwm));

    // ------------------------------------------------------------------
    // 右前轮
    // ------------------------------------------------------------------
    wire [15:0]        rf_count;
    wire signed [15:0] rf_speed, rf_raw_speed, rf_duty;
    wire               rf_tick;
    quad_decoder u_rf_dec (
        .clk(clk), .rst_n(rst_n),
        .a(ENCODER_REVERSE[1] ? rf_eb : rf_ea),
        .b(ENCODER_REVERSE[1] ? rf_ea : rf_eb), .count(rf_count));
    speed_measure u_rf_meas (
        .clk(clk), .rst_n(rst_n), .count_in(rf_count), .speed(rf_raw_speed), .tick(rf_tick));
    generate if (USE_CLOSED_LOOP) begin: g_rf_closed
        wheel_speed_loop #(.NO_FEEDBACK_GUARD(NO_FEEDBACK_GUARD),.DYNAMIC_GAINS(0),.KP(PID_KP), .KI(PID_KI), .KD(PID_KD), .MAX_DUTY(LOOP_MAX_DUTY), .START_RAMP_STEP(LOOP_START_STEP), .START_TICKS(LOOP_START_TICKS), .FIXED_ENCODER_SIGN(LOOP_FIXED_ENCODER)) u_loop(
            .kp_live(dashboard_gains[96 +: 32]),.ki_live(dashboard_gains[128 +: 32]),.kd_live(dashboard_gains[160 +: 32]),
            .clk(clk), .rst_n(rst_n), .enable(!system_fault), .tick(rf_tick),
            .target(right_target), .raw_speed(rf_raw_speed),
            .measured(rf_speed), .command(rf_duty),
            .running(wheel_running[1]), .fault(wheel_fault[1]), .encoder_reverse(encoder_sign_mask[1]),
            .calibration_done(calibration_mask[1]),.fault_reason(fault_reasons[7:4]));
    end else begin: g_rf_open
        assign rf_speed=rf_raw_speed;
        assign rf_duty=TEST_WHEEL_MASK[1] ? TEST_SPEED : 16'sd0;
        reg [31:0] quiet_ticks;
        reg no_feedback;
        always @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin quiet_ticks<=0; no_feedback<=0; end
            else if (rf_tick && TEST_WHEEL_MASK[1] && !system_fault) begin
                if (rf_raw_speed>=2 || rf_raw_speed<=-2) quiet_ticks<=0;
                else if (quiet_ticks>=OPEN_NO_FEEDBACK_TICKS-1) no_feedback<=1;
                else quiet_ticks<=quiet_ticks+1'b1;
            end
        end
        assign wheel_fault[1]=TEST_WHEEL_MASK[1] &&
            (no_feedback || open_elapsed>=OPEN_TIMEOUT_TICKS);
        assign wheel_running[1]=1'b0;
        assign calibration_mask[1]=0; assign encoder_sign_mask[1]=0; assign fault_reasons[7:4]=0;
    end endgenerate
    wire signed [15:0] rf_command = system_fault ? 16'sd0 : rf_duty;
    motor_driver #(.PHASE_OFFSET(735)) u_rf_drv (
        .clk(clk), .rst_n(rst_n), .speed(MOTOR_REVERSE[1] ? -rf_command : rf_command), .dir(rf_dir), .pwm(rf_pwm));

    // ------------------------------------------------------------------
    // 左后轮
    // ------------------------------------------------------------------
    wire [15:0]        lb_count;
    wire signed [15:0] lb_speed, lb_raw_speed, lb_duty;
    wire               lb_tick;
    quad_decoder u_lb_dec (
        .clk(clk), .rst_n(rst_n),
        .a(ENCODER_REVERSE[2] ? lb_eb : lb_ea),
        .b(ENCODER_REVERSE[2] ? lb_ea : lb_eb), .count(lb_count));
    speed_measure u_lb_meas (
        .clk(clk), .rst_n(rst_n), .count_in(lb_count), .speed(lb_raw_speed), .tick(lb_tick));
    generate if (USE_CLOSED_LOOP) begin: g_lb_closed
        wheel_speed_loop #(.NO_FEEDBACK_GUARD(NO_FEEDBACK_GUARD),.DYNAMIC_GAINS(0),.KP(PID_KP), .KI(PID_KI), .KD(PID_KD), .MAX_DUTY(LOOP_MAX_DUTY), .START_RAMP_STEP(LOOP_START_STEP), .START_TICKS(LOOP_START_TICKS), .FIXED_ENCODER_SIGN(LOOP_FIXED_ENCODER)) u_loop(
            .kp_live(dashboard_gains[192 +: 32]),.ki_live(dashboard_gains[224 +: 32]),.kd_live(dashboard_gains[256 +: 32]),
            .clk(clk), .rst_n(rst_n), .enable(!system_fault), .tick(lb_tick),
            .target(left_target), .raw_speed(lb_raw_speed),
            .measured(lb_speed), .command(lb_duty),
            .running(wheel_running[2]), .fault(wheel_fault[2]), .encoder_reverse(encoder_sign_mask[2]),
            .calibration_done(calibration_mask[2]),.fault_reason(fault_reasons[11:8]));
    end else begin: g_lb_open
        assign lb_speed=lb_raw_speed;
        assign lb_duty=TEST_WHEEL_MASK[2] ? TEST_SPEED : 16'sd0;
        reg [31:0] quiet_ticks;
        reg no_feedback;
        always @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin quiet_ticks<=0; no_feedback<=0; end
            else if (lb_tick && TEST_WHEEL_MASK[2] && !system_fault) begin
                if (lb_raw_speed>=2 || lb_raw_speed<=-2) quiet_ticks<=0;
                else if (quiet_ticks>=OPEN_NO_FEEDBACK_TICKS-1) no_feedback<=1;
                else quiet_ticks<=quiet_ticks+1'b1;
            end
        end
        assign wheel_fault[2]=TEST_WHEEL_MASK[2] &&
            (no_feedback || open_elapsed>=OPEN_TIMEOUT_TICKS);
        assign wheel_running[2]=1'b0;
        assign calibration_mask[2]=0; assign encoder_sign_mask[2]=0; assign fault_reasons[11:8]=0;
    end endgenerate
    wire signed [15:0] lb_command = system_fault ? 16'sd0 : lb_duty;
    motor_driver #(.PHASE_OFFSET(1470)) u_lb_drv (
        .clk(clk), .rst_n(rst_n), .speed(MOTOR_REVERSE[2] ? -lb_command : lb_command), .dir(lb_dir), .pwm(lb_pwm));

    // ------------------------------------------------------------------
    // 右后轮
    // ------------------------------------------------------------------
    wire [15:0]        rb_count;
    wire signed [15:0] rb_speed, rb_raw_speed, rb_duty;
    wire               rb_tick;
    quad_decoder u_rb_dec (
        .clk(clk), .rst_n(rst_n),
        .a(ENCODER_REVERSE[3] ? rb_eb : rb_ea),
        .b(ENCODER_REVERSE[3] ? rb_ea : rb_eb), .count(rb_count));
    speed_measure u_rb_meas (
        .clk(clk), .rst_n(rst_n), .count_in(rb_count), .speed(rb_raw_speed), .tick(rb_tick));
    generate if (USE_CLOSED_LOOP) begin: g_rb_closed
        wheel_speed_loop #(.NO_FEEDBACK_GUARD(NO_FEEDBACK_GUARD),.DYNAMIC_GAINS(0),.KP(PID_KP), .KI(PID_KI), .KD(PID_KD), .MAX_DUTY(LOOP_MAX_DUTY), .START_RAMP_STEP(LOOP_START_STEP), .START_TICKS(LOOP_START_TICKS), .FIXED_ENCODER_SIGN(LOOP_FIXED_ENCODER)) u_loop(
            .kp_live(dashboard_gains[288 +: 32]),.ki_live(dashboard_gains[320 +: 32]),.kd_live(dashboard_gains[352 +: 32]),
            .clk(clk), .rst_n(rst_n), .enable(!system_fault), .tick(rb_tick),
            .target(right_target), .raw_speed(rb_raw_speed),
            .measured(rb_speed), .command(rb_duty),
            .running(wheel_running[3]), .fault(wheel_fault[3]), .encoder_reverse(encoder_sign_mask[3]),
            .calibration_done(calibration_mask[3]),.fault_reason(fault_reasons[15:12]));
    end else begin: g_rb_open
        assign rb_speed=rb_raw_speed;
        assign rb_duty=TEST_WHEEL_MASK[3] ? TEST_SPEED : 16'sd0;
        reg [31:0] quiet_ticks;
        reg no_feedback;
        always @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin quiet_ticks<=0; no_feedback<=0; end
            else if (rb_tick && TEST_WHEEL_MASK[3] && !system_fault) begin
                if (rb_raw_speed>=2 || rb_raw_speed<=-2) quiet_ticks<=0;
                else if (quiet_ticks>=OPEN_NO_FEEDBACK_TICKS-1) no_feedback<=1;
                else quiet_ticks<=quiet_ticks+1'b1;
            end
        end
        assign wheel_fault[3]=TEST_WHEEL_MASK[3] &&
            (no_feedback || open_elapsed>=OPEN_TIMEOUT_TICKS);
        assign wheel_running[3]=1'b0;
        assign calibration_mask[3]=0; assign encoder_sign_mask[3]=0; assign fault_reasons[15:12]=0;
    end endgenerate
    wire signed [15:0] rb_command = system_fault ? 16'sd0 : rb_duty;
    motor_driver #(.PHASE_OFFSET(2205)) u_rb_drv (
        .clk(clk), .rst_n(rst_n), .speed(MOTOR_REVERSE[3] ? -rb_command : rb_command), .dir(rb_dir), .pwm(rb_pwm));

    // ------------------------------------------------------------------
    // IMU660RA：6 轴 IMU（加速度 + 陀螺仪）
    // ------------------------------------------------------------------
    wire [15:0] acc_x, acc_y, acc_z;
    wire [15:0] gyro_x, gyro_y, gyro_z;
    wire imu_valid, imu_init;
    wire signed [15:0] attitude_roll,attitude_pitch,attitude_yaw;
    wire attitude_valid,attitude_ready;
    generate if(HEADING_ENABLE && USE_CLOSED_LOOP && !TRIANGLE_ENABLE) begin: g_heading
        reg encoder_start_done;
        always @(posedge clk or negedge rst_n) begin
            if(!rst_n) encoder_start_done<=0;
            else if(&wheel_running) encoder_start_done<=1;
        end
        // Bias calibration is performed by imu_attitude first, then another
        // 100 quiet frames (~0.51s) establish the physical startup heading.
        wire quiet=($signed(gyro_x)<=164 && $signed(gyro_x)>=-164 &&
                    $signed(gyro_y)<=164 && $signed(gyro_y)>=-164 &&
                    $signed(gyro_z)<=164 && $signed(gyro_z)>=-164);
        heading_hold #(.DYNAMIC_GAINS(DASHBOARD_ENABLE)) u_hold(.clk(clk),.rst_n(rst_n),.tick(lf_tick),
            .kp_live(dashboard_gains[384 +: 32]),.kd_live(dashboard_gains[448 +: 32]),
            .angle_valid(attitude_valid),.imu_ready(attitude_ready && imu_init),.quiet(quiet),
            .hold_turn(!encoder_start_done && heading_turn!=0),
            .yaw(attitude_yaw),.base_speed(speed_target),.relative_yaw(heading_yaw),
            .target_yaw(heading_test_target),
            .turn(heading_turn),.left_target(left_target),.right_target(right_target),
            .ready(heading_ready),.fault(heading_fault));
    end else begin: g_no_heading
        assign heading_yaw=attitude_yaw;
        assign heading_turn=0;
        assign left_target=speed_target;
        assign right_target=speed_target;
        assign heading_ready=attitude_ready;
        assign heading_fault=0;
    end endgenerate
    imu_attitude u_attitude(.clk(clk),.rst_n(rst_n),.sample_valid(imu_valid),
        .ax(acc_x),.ay(acc_y),.az(acc_z),.gx(gyro_x),.gy(gyro_y),.gz(gyro_z),
        .roll(attitude_roll),.pitch(attitude_pitch),.yaw(attitude_yaw),
        .angle_valid(attitude_valid),.ready(attitude_ready));
    imu660ra u_imu (
        .clk(clk), .rst_n(rst_n),
        .acc_x(acc_x), .acc_y(acc_y), .acc_z(acc_z),
        .gyro_x(gyro_x), .gyro_y(gyro_y), .gyro_z(gyro_z),
        .data_valid(imu_valid), .init_done(imu_init),
        .spi_sck(imu_sck), .spi_mosi(imu_mosi),
        .spi_cs_n(imu_cs_n), .spi_miso(imu_miso)
    );

    // ------------------------------------------------------------------
    // 逐飞 DL1B：激光测距（I2C）
    // ------------------------------------------------------------------
    wire [15:0] tof_distance;
    wire tof_valid, tof_init;
    wire [5:0] tof_state;
    wire [15:0] tof_retries;
    dl1b #(.I2C_FREQ(DASHBOARD_ENABLE ? 100_000 : 390_000)) u_tof (
        .clk(clk), .rst_n(rst_n),
        .distance_mm(tof_distance), .data_valid(tof_valid), .init_done(tof_init),
        .scl(tof_scl), .sda(tof_sda), .xs(tof_xs), .int_n(tof_int),
        .debug_state(tof_state),.retry_count(tof_retries)
    );

    // ------------------------------------------------------------------
    // 仅在六轴整帧读完后更新 UART 数据，避免混合两次 IMU 采样。
    reg [15:0] ax_frame, ay_frame, az_frame, gx_frame, gy_frame, gz_frame;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ax_frame <= 0; ay_frame <= 0; az_frame <= 0;
            gx_frame <= 0; gy_frame <= 0; gz_frame <= 0;
        end else if (imu_valid) begin
            ax_frame <= acc_x; ay_frame <= acc_y; az_frame <= acc_z;
            gx_frame <= gyro_x; gy_frame <= gyro_y; gz_frame <= gyro_z;
        end
    end

    // 串口调试：编码器、IMU、TOF、初始化/更新、PID状态，共五行。
    // ------------------------------------------------------------------
    uart_enc_debug #(.DIAGNOSTIC_ENABLE(DASHBOARD_ENABLE)) u_uart (
        .clk(clk), .rst_n(rst_n),
        .lf_count(lf_count), .rf_count(rf_count),
        .lb_count(lb_count), .rb_count(rb_count),
        .lf_speed(lf_speed), .rf_speed(rf_speed),
        .lb_speed(lb_speed), .rb_speed(rb_speed),
        .acc_x(ax_frame), .acc_y(ay_frame), .acc_z(az_frame),
        .gyro_x(gx_frame), .gyro_y(gy_frame), .gyro_z(gz_frame),
        .distance_mm(tof_distance),
        .imu_init(imu_init), .imu_valid(imu_valid),
        .tof_init(tof_init), .tof_valid(tof_valid),
        .target_speed(speed_target), .run_mask(wheel_running), .fault_mask(first_fault),
        .attitude_roll(attitude_roll),.attitude_pitch(attitude_pitch),.attitude_yaw(heading_yaw),
        .attitude_ready(heading_ready),
        .heading_left(left_target),.heading_right(right_target),.heading_turn(heading_turn),.heading_fault(heading_fault),
        .calibration_mask(calibration_mask),.encoder_sign_mask(encoder_sign_mask),
        .fault_reasons(fault_reasons),.diagnostic_stop(heading_test_stop),
        .diagnostic_bits({7'd0,enc_sync,edge_count[3],edge_count[2],edge_count[1],edge_count[0],
            bad_count[3],bad_count[2],bad_count[1],bad_count[0],tof_scl,tof_sda,tof_xs,tof_state,tof_retries}),
        .uart_tx(uart_tx)
    );

    wire [7:0] lidar_x;
    wire [6:0] lidar_y;
    wire [15:0] lidar_pixel;
    wire lidar_frame_start;
    generate if(LIDAR_PAGE_ENABLE)begin:g_lidar
        n10_lidar u_lidar(.clk(clk),.rst_n(rst_n),.rx(lidar_rx),.display_start(lidar_frame_start),
            .pixel_x(lidar_x),.pixel_y(lidar_y),.pixel(lidar_pixel));
    end else begin:g_no_lidar
        assign lidar_pixel=16'd0;
    end endgenerate
    tft180_display #(.DASHBOARD_ENABLE(DASHBOARD_ENABLE),.TELEMETRY_PAGES(DASHBOARD_ENABLE),.CAMERA_PAGE_ENABLE(CAMERA_PAGE_ENABLE),.LIDAR_PAGE_ENABLE(LIDAR_PAGE_ENABLE)) u_tft (
        .clk(clk), .rst_n(rst_n),
        .lidar_pixel(lidar_pixel),.lidar_x(lidar_x),.lidar_y(lidar_y),.lidar_frame_start(lidar_frame_start),
        .page(dashboard_page),.selection(dashboard_selection),.gains_flat(dashboard_gains),
        .camera_valid(camera_valid),.camera_init_done(camera_init_done),.camera_fault(camera_fault),
        .camera_pixel(camera_pixel),.camera_rd_addr(camera_rd_addr),
        .camera_frame_start(camera_frame_start),.camera_frame_end(camera_frame_end),
        .lf_count(lf_count), .rf_count(rf_count), .lb_count(lb_count), .rb_count(rb_count),
        .lf_speed(lf_speed), .rf_speed(rf_speed), .lb_speed(lb_speed), .rb_speed(rb_speed),
        .acc_x(ax_frame), .acc_y(ay_frame), .acc_z(az_frame),
        .gyro_x(gx_frame), .gyro_y(gy_frame), .gyro_z(gz_frame),
        .distance_mm(tof_distance), .imu_init(imu_init), .tof_init(tof_init),
        .attitude_roll(attitude_roll),.attitude_pitch(attitude_pitch),.attitude_yaw(heading_yaw),
        .attitude_ready(heading_ready),
        .heading_turn(heading_turn),.heading_left(left_target),.heading_right(right_target),
        .heading_fault(heading_fault),.outputs_enabled(OUTPUTS_ENABLED!=0),
        .lf_duty(lf_duty),.rf_duty(rf_duty),.lb_duty(lb_duty),.rb_duty(rb_duty),
        .fault_reasons(fault_reasons),.calibration_mask(calibration_mask),
        .encoder_sign_mask(encoder_sign_mask),.encoder_inputs(enc_sync),
        .tof_state(tof_state),.tof_retries(tof_retries),
        .target_speed(speed_target), .closed_loop(USE_CLOSED_LOOP != 0), .fault_mask(first_fault),
        .tft_scl(tft_scl), .tft_sda(tft_sda), .tft_reset(tft_reset),
        .tft_dc(tft_dc), .tft_cs_n(tft_cs_n), .tft_bl(tft_bl)
    );

endmodule
