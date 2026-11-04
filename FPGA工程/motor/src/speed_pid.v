// Incremental Q12 PID: wide pipelined arithmetic and anti-windup clamping.
// Static latency is unchanged; dynamic mode needs 37 clocks between ticks.
// Live gains are sampled together on an accepted tick; edits preserve u/e1/e2.
module speed_pid #(
    parameter signed [31:0] KP=2048, KI=410, KD=0,
    parameter signed [15:0] MAX_OUT=1470, MIN_OUT=-1470,
    parameter integer DYNAMIC_GAINS=0
)(
    input wire clk, rst_n, enable, tick,
    input wire signed [15:0] target, measured, seed,
    output reg signed [15:0] out,
    output reg valid,
    input wire signed [31:0] kp_live, ki_live, kd_live
);
    localparam signed [63:0] MAX_Q=$signed(MAX_OUT)*64'sd4096;
    localparam signed [63:0] MIN_Q=$signed(MIN_OUT)*64'sd4096;
    wire signed [63:0] seed_q=$signed(seed)*64'sd4096;
    wire signed [31:0] error_now=$signed(target)-$signed(measured);
    reg signed [31:0] e1, e2, error_sample, diff1, diff2;
    reg signed [63:0] p_term, i_term, d_term, delta, candidate, u;
    reg [2:0] phase;
    reg signed [31:0] kp_sample, ki_sample, kd_sample;
    reg [5:0] multiply_bit;
    reg signed [63:0] p_shift, i_shift, d_shift;
    reg signed [63:0] p_acc, i_acc, d_acc;
    // Two's-complement multiplier: bits 0..30 add, sign bit subtracts.
    wire signed [63:0] p_next=p_acc+(kp_sample[multiply_bit] ?
        (multiply_bit==31 ? -p_shift : p_shift) : 64'sd0);
    wire signed [63:0] i_next=i_acc+(ki_sample[multiply_bit] ?
        (multiply_bit==31 ? -i_shift : i_shift) : 64'sd0);
    wire signed [63:0] d_next=d_acc+(kd_sample[multiply_bit] ?
        (multiply_bit==31 ? -d_shift : d_shift) : 64'sd0);
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            e1<=0; e2<=0; error_sample<=0; diff1<=0; diff2<=0;
            p_term<=0; i_term<=0; d_term<=0; delta<=0; candidate<=0;
            u<=0; out<=0; phase<=0; valid<=0;
            kp_sample<=0; ki_sample<=0; kd_sample<=0; multiply_bit<=0;
            p_shift<=0; i_shift<=0; d_shift<=0; p_acc<=0; i_acc<=0; d_acc<=0;
        end else begin
            valid<=0;
            if (!enable) begin
                phase<=0; e1<=0; e2<=0;
                if (seed_q>MAX_Q) begin u<=MAX_Q; out<=MAX_OUT; end
                else if (seed_q<MIN_Q) begin u<=MIN_Q; out<=MIN_OUT; end
                else begin u<=seed_q; out<=seed; end
            end else begin
                case (phase)
                    0: if (tick) begin
                        error_sample<=error_now;
                        diff1<=error_now-e1;
                        diff2<=error_now-(e1<<<1)+e2;
                        e2<=e1; e1<=error_now; phase<=1;
                        if(DYNAMIC_GAINS) begin
                            kp_sample<=kp_live; ki_sample<=ki_live; kd_sample<=kd_live;
                        end
                    end
                    1: begin
                        if(DYNAMIC_GAINS) begin
                            p_shift<=diff1; i_shift<=error_sample; d_shift<=diff2;
                            p_acc<=0; i_acc<=0; d_acc<=0; multiply_bit<=0;
                            phase<=5;
                        end else begin
                            p_term<=KP*diff1; i_term<=KI*error_sample; d_term<=KD*diff2;
                            phase<=2;
                        end
                    end
                    5: begin
                        p_acc<=p_next; i_acc<=i_next; d_acc<=d_next;
                        p_shift<=p_shift<<<1; i_shift<=i_shift<<<1; d_shift<=d_shift<<<1;
                        if(multiply_bit==31) begin
                            p_term<=p_next; i_term<=i_next; d_term<=d_next; phase<=2;
                        end else multiply_bit<=multiply_bit+1'b1;
                    end
                    2: begin delta<=p_term+i_term+d_term; phase<=3; end
                    3: begin candidate<=u+delta; phase<=4; end
                    4: begin
                        if (candidate>MAX_Q) begin u<=MAX_Q; out<=MAX_OUT; end
                        else if (candidate<MIN_Q) begin u<=MIN_Q; out<=MIN_OUT; end
                        else begin u<=candidate; out<=candidate>>>12; end
                        valid<=1; phase<=0;
                    end
                    default: phase<=0;
                endcase
            end
        end
    end
endmodule
