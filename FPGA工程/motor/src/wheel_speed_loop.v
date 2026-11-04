// Bidirectional magnitude PI; calibrate encoder sign once, stop before reversal.
module wheel_speed_loop #(
    parameter signed [31:0] KP=2048, KI=410, KD=0,
    parameter signed [15:0] MAX_DUTY=1470, START_DUTY=441,
    parameter integer START_TICKS=20, NO_FEEDBACK_TICKS=50, MIN_FEEDBACK=2,
    parameter integer START_RAMP_STEP=0, // 0: legacy fixed pulse; otherwise PWM counts/tick
    parameter integer FIXED_ENCODER_SIGN=0, // 1: use the verified top-level A/B polarity
    parameter integer NO_FEEDBACK_GUARD=1, // Only disabled behind the dashboard's physical motor gate.
    parameter integer STOP_SPEED=30, STOP_TICKS=10, REVERSE_WAIT_TICKS=200,
    parameter integer DYNAMIC_GAINS=0
)(
    input wire clk, rst_n, enable, tick,
    input wire signed [15:0] target, raw_speed,
    output wire signed [15:0] measured, command,
    output reg running, fault, encoder_reverse,
    output wire calibration_done,
    output reg [3:0] fault_reason,
    input wire signed [31:0] kp_live, ki_live, kd_live
);
    reg calibrated, direction_negative, restarting;
    assign calibration_done=calibrated;
    reg [15:0] start_ticks, quiet_ticks, motion_ticks, reverse_ticks;
    reg [15:0] stopped_ticks, wait_ticks;
    reg signed [31:0] motion_sum;
    reg signed [15:0] startup_duty;
    localparam signed [15:0] START_LIMIT=(START_DUTY>MAX_DUTY) ? MAX_DUTY : START_DUTY;
    wire requested_negative=target[15];
    wire signed [15:0] magnitude=(target==16'sh8000) ? 16'sd32767 :
        (requested_negative ? -target : target);
    wire signed [31:0] raw_extended=raw_speed;
    wire signed [31:0] motion_next=motion_sum+raw_extended;
    wire moving=(raw_speed>=MIN_FEEDBACK || raw_speed<=-MIN_FEEDBACK);
    wire [15:0] motion_ticks_next=motion_ticks+(moving ? 16'd1 : 16'd0);
    wire sign_match=(requested_negative==direction_negative);
    wire pid_enable=enable && calibrated && running && !restarting && !fault && target!=0 && sign_match;
    wire signed [15:0] pid_out;
    assign measured=encoder_reverse ? -raw_speed : raw_speed;
    wire signed [15:0] directed_speed=direction_negative ? -measured : measured;
    wire stopped=(measured<=STOP_SPEED && measured>=-STOP_SPEED);
    wire signed [15:0] magnitude_command=calibrated ?
        (running && sign_match ? (restarting ? startup_duty : pid_out) : 16'sd0) :
        (start_ticks==0 || !sign_match ? 16'sd0 : startup_duty);
    assign command=(!enable || fault || target==0) ? 16'sd0 :
        (requested_negative ? -magnitude_command : magnitude_command);
    speed_pid #(.KP(KP), .KI(KI), .KD(KD), .MAX_OUT(MAX_DUTY), .MIN_OUT(16'sd0), .DYNAMIC_GAINS(DYNAMIC_GAINS)) u_pid(
        .clk(clk), .rst_n(rst_n), .enable(pid_enable), .tick(tick),
        .target(magnitude), .measured(directed_speed),
        .seed(target==0 || (calibrated && !restarting) ? 16'sd0 : startup_duty),
        .out(pid_out), .valid(), .kp_live(kp_live), .ki_live(ki_live), .kd_live(kd_live));
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            calibrated<=(FIXED_ENCODER_SIGN!=0); direction_negative<=0; restarting<=0;
            running<=0; fault<=0; encoder_reverse<=0; fault_reason<=0;
            start_ticks<=0; quiet_ticks<=0; motion_ticks<=0; reverse_ticks<=0;
            stopped_ticks<=0; wait_ticks<=0; motion_sum<=0;
            startup_duty<=(START_RAMP_STEP==0) ? START_LIMIT : 16'sd0;
        end else if (!enable) begin
            if(!fault) fault_reason<=1; // global disable, preserve primary cause
            fault<=1; running<=0;
        end else if (!fault && tick) begin
            if (stopped) begin
                if (stopped_ticks<STOP_TICKS) stopped_ticks<=stopped_ticks+1'b1;
            end else stopped_ticks<=0;
            if (target==16'sh8000) begin fault<=1; running<=0; fault_reason<=2; end
            else if (target==0) begin
                restarting<=0;
                running<=0; start_ticks<=0; quiet_ticks<=0; wait_ticks<=0;
                motion_ticks<=0; reverse_ticks<=0; motion_sum<=0;
                startup_duty<=(START_RAMP_STEP==0) ? START_LIMIT : 16'sd0;
            end else if (!calibrated) begin
                if (start_ticks!=0 && !sign_match) begin fault<=1; fault_reason<=3; end
                direction_negative<=requested_negative;
                start_ticks<=start_ticks+1'b1;
                motion_sum<=motion_next;
                motion_ticks<=motion_ticks_next;
                if (START_RAMP_STEP>0) begin
                    if ($signed(startup_duty)+START_RAMP_STEP>=START_LIMIT) startup_duty<=START_LIMIT;
                    else startup_duty<=startup_duty+START_RAMP_STEP;
                end
                // Stop ramping as soon as sufficient signed feedback is available.
                if (start_ticks>=START_TICKS-1 ||
                    (START_RAMP_STEP>0 && motion_ticks_next>=3 && (motion_next>=10 || motion_next<=-10))) begin
                    if (motion_ticks_next>=3 && (motion_next>=10 || motion_next<=-10)) begin
                        encoder_reverse<=(motion_next<0)^requested_negative;
                        calibrated<=1; running<=1; quiet_ticks<=0; reverse_ticks<=0;
                    end else begin fault<=1; running<=0; fault_reason<=4; end
                end
            end else if (!running || !sign_match) begin
                running<=0; quiet_ticks<=0; reverse_ticks<=0; restarting<=0;
                if (sign_match || (stopped && stopped_ticks>=STOP_TICKS)) begin
                    direction_negative<=requested_negative;
                    running<=1; wait_ticks<=0;
                    if(START_RAMP_STEP>0) begin
                        restarting<=1; startup_duty<=0; start_ticks<=0; motion_ticks<=0;
                    end
                end else if (wait_ticks>=REVERSE_WAIT_TICKS-1) begin fault<=1; running<=0; fault_reason<=5; end
                else wait_ticks<=wait_ticks+1'b1;
            end else if(restarting) begin
                // Re-use gentle breakaway on every zero-speed restart/reversal.
                // Encoder polarity is already known: never relearn it under load.
                start_ticks<=start_ticks+1'b1;
                if ($signed(startup_duty)+START_RAMP_STEP>=START_LIMIT) startup_duty<=START_LIMIT;
                else startup_duty<=startup_duty+START_RAMP_STEP;
                if(directed_speed>=MIN_FEEDBACK) begin
                    motion_ticks<=motion_ticks+1'b1; reverse_ticks<=0;
                    if(motion_ticks>=2) begin restarting<=0; quiet_ticks<=0; end
                end else begin
                    motion_ticks<=0;
                    if(directed_speed<=-MIN_FEEDBACK) begin
                        reverse_ticks<=reverse_ticks+1'b1;
                        if(reverse_ticks>=NO_FEEDBACK_TICKS-1) begin fault<=1; running<=0; fault_reason<=7; end
                    end else reverse_ticks<=0;
                end
                if(start_ticks>=START_TICKS-1) begin
                    if(NO_FEEDBACK_GUARD) begin fault<=1; running<=0; fault_reason<=6; end
                    else begin restarting<=0; quiet_ticks<=0; end
                end
            end else begin
                if (NO_FEEDBACK_GUARD && pid_out>=100 && !moving) begin
                    if (quiet_ticks>=NO_FEEDBACK_TICKS-1) begin fault<=1; running<=0; fault_reason<=6; end
                    else quiet_ticks<=quiet_ticks+1'b1;
                end else quiet_ticks<=0;
                // Low-speed stiction: bounded ramp before the existing dropout
                // timeout, not an unconditional minimum duty or disabled guard.
                if(NO_FEEDBACK_GUARD && START_RAMP_STEP>0 && pid_out>=100 && !moving && quiet_ticks==9) begin
                    restarting<=1; startup_duty<=pid_out; start_ticks<=0; motion_ticks<=0;
                end
                if (pid_out>=100 && directed_speed<=-MIN_FEEDBACK) begin
                    if (reverse_ticks>=NO_FEEDBACK_TICKS-1) begin fault<=1; running<=0; fault_reason<=7; end
                    else reverse_ticks<=reverse_ticks+1'b1;
                end else reverse_ticks<=0;
            end
        end
    end
endmodule
