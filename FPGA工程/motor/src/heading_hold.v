// Relative heading outer PD loop, 100Hz. Positive yaw/turn is LEFT (CCW).
// Mecanum, no lateral command: LF=LB=base-turn, RF=RB=base+turn.
// All units: yaw centidegrees; wheel target encoder counts per 10ms.
// No multipliers: FPGA attitude estimator already occupies all DSP blocks.
// Dynamic gains latch on an idle tick; turn updates 37 clocks later. Busy ticks
// are ignored. Gain edits preserve the heading reference and derivative filter.
module heading_hold #(
    parameter integer QUIET_SAMPLES=100,
    parameter integer MAX_TURN=300, SLEW=5, BRAKE_SLEW=20, MAX_SPEED=1000,
    // Legacy RATE_DIVISOR/BRAKE_SLEW retained for elaboration compatibility only.
    // Static D is now .625 (5/2048 applied to rate_q8); slew is symmetric.
    parameter integer RATE_DIVISOR=256,
    parameter integer DYNAMIC_GAINS=0
)(
    input wire clk,rst_n,tick,angle_valid,imu_ready,quiet,hold_turn,
    input wire signed [15:0] yaw,base_speed,target_yaw,
    output reg signed [15:0] relative_yaw,turn,
    output wire signed [15:0] left_target,right_target,
    output wire ready,
    output reg fault,
    // Q12 counts/centidegree (default 320) and counts/(degree/s) (2560).
    input wire signed [31:0] kp_live, kd_live
);
    reg armed;
    reg [15:0] quiet_count;
    reg signed [15:0] reference_yaw,previous_yaw;
    reg signed [31:0] rate_q8;
    reg [2:0] state;
    reg signed [31:0] heading_sample,error_sample,delta;
    reg signed [63:0] proportional,desired;
    reg signed [31:0] kp_sample,kd_sample;
    reg [5:0] multiply_bit;
    reg signed [63:0] p_shift,d_shift,p_acc,d_acc;
    wire signed [63:0] rate_ext=rate_q8;
    wire signed [63:0] p_next=p_acc+(kp_sample[multiply_bit] ?
        (multiply_bit==31 ? -p_shift : p_shift) : 64'sd0);
    wire signed [63:0] d_next=d_acc+(kd_sample[multiply_bit] ?
        (multiply_bit==31 ? -d_shift : d_shift) : 64'sd0);
    // Arithmetic shift alone rounds negatives down; gains require toward zero.
    function signed [63:0] trunc_shift;
        input signed [63:0] value;
        input integer bits;
        begin
            if(value<0) trunc_shift=-((-value)>>>bits);
            else trunc_shift=value>>>bits;
        end
    endfunction
    wire signed [31:0] yaw_ext=yaw,reference_ext=reference_yaw;
    wire signed [31:0] rel_ext=relative_yaw,prev_ext=previous_yaw;
    wire signed [31:0] target_ext=target_yaw;
    function signed [31:0] wrap;
        input signed [31:0] value;
        begin
            if(value>=18000) wrap=value-36000;
            else if(value< -18000) wrap=value+36000;
            else wrap=value;
        end
    endfunction
    function signed [15:0] limit_speed;
        input signed [31:0] value;
        begin
            if(value>MAX_SPEED) limit_speed=MAX_SPEED;
            else if(value< -MAX_SPEED) limit_speed=-MAX_SPEED;
            else limit_speed=value;
        end
    endfunction
    wire signed [31:0] base_ext=base_speed,turn_ext=turn;
    assign ready=armed && imu_ready && !fault;
    assign left_target=ready ? limit_speed(base_ext-turn_ext) : 16'sd0;
    assign right_target=ready ? limit_speed(base_ext+turn_ext) : 16'sd0;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            armed<=0; fault<=0; quiet_count<=0; reference_yaw<=0;
            relative_yaw<=0; previous_yaw<=0; turn<=0; rate_q8<=0;
            state<=0; heading_sample<=0; error_sample<=0; delta<=0; proportional<=0; desired<=0;
            kp_sample<=0; kd_sample<=0; multiply_bit<=0;
            p_shift<=0; d_shift<=0; p_acc<=0; d_acc<=0;
        end else if(fault || (armed && !imu_ready)) begin
            // Never automatically re-zero or restart following stale IMU data.
            fault<=1; turn<=0; state<=0;
        end else if(!armed) begin
            turn<=0; state<=0;
            if(!imu_ready || !quiet) quiet_count<=0;
            else if(angle_valid) begin
                if(quiet_count>=QUIET_SAMPLES-1) begin
                    reference_yaw<=yaw; relative_yaw<=0; previous_yaw<=0;
                    rate_q8<=0; armed<=1;
                end else quiet_count<=quiet_count+1'b1;
            end
        end else begin
            if(angle_valid) relative_yaw<=wrap(yaw_ext-reference_ext);
            // Pipeline the wrap, derivative filter, P/D and slew to keep 50MHz.
            case(state)
                0: if(tick) begin
                    heading_sample<=rel_ext; error_sample<=wrap(rel_ext-target_ext);
                    delta<=wrap(rel_ext-prev_ext); state<=1;
                    if(DYNAMIC_GAINS) begin kp_sample<=kp_live; kd_sample<=kd_live; end
                end
                1: begin
                    previous_yaw<=heading_sample;
                    rate_q8<=rate_q8+(((delta<<<8)-rate_q8)>>>2);
                    // P = -7.8125 counts/degree. Symmetric rounding to zero.
                    if(error_sample<=50 && error_sample>=-50) proportional<=0;
                    else if(error_sample>0)
                        proportional<=-(((error_sample<<<4)+(error_sample<<<2))>>>8);
                    else proportional<=(((-error_sample<<<4)+(-error_sample<<<2))>>>8);
                    state<=2;
                end
                2: begin
                    // D on measured yaw. Dynamic products take 32 shift/add clocks.
                    // At 100Hz: centidegree/sample == degree/second.
                    // Do not initiate the 15% encoder startup pulse for a
                    // one-count derivative artifact inside the heading deadband.
                    if(DYNAMIC_GAINS) begin
                        p_shift<=(error_sample<=50 && error_sample>=-50) ? 32'sd0 : error_sample;
                        d_shift<=rate_q8;
                        p_acc<=0; d_acc<=0; multiply_bit<=0; state<=5;
                    end else begin
                        if(error_sample<=50 && error_sample>=-50 && rate_q8<512 && rate_q8> -512)
                            desired<=0;
                        else desired<=proportional-trunc_shift((rate_ext<<<2)+rate_ext,11);
                        state<=3;
                    end
                end
                5: begin
                    p_acc<=p_next; d_acc<=d_next;
                    p_shift<=p_shift<<<1; d_shift<=d_shift<<<1;
                    if(multiply_bit==31) state<=6;
                    else multiply_bit<=multiply_bit+1'b1;
                end
                6: begin
                    if(error_sample<=50 && error_sample>=-50 && rate_q8<512 && rate_q8> -512)
                        desired<=0;
                    else desired<=-trunc_shift(p_acc,12)-trunc_shift(d_acc,20);
                    state<=3;
                end
                3: begin
                    if(desired>MAX_TURN) desired<=MAX_TURN;
                    else if(desired< -MAX_TURN) desired<=-MAX_TURN;
                    state<=4;
                end
                4: begin
                    // First encoder sign check requires a consistent direction.
                    // Sensor fault handling above always overrides this hold.
                    if(!hold_turn) begin
                        if(desired>$signed(turn_ext)+SLEW) turn<=turn+SLEW;
                        else if(desired<$signed(turn_ext)-SLEW) turn<=turn-SLEW;
                        else turn<=desired;
                    end
                    state<=0;
                end
                default: state<=0;
            endcase
        end
    end
endmodule
