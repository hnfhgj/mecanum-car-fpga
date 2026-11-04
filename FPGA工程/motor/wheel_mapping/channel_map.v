// Logical order everywhere downstream: LF, RF, LB, RB (bits 0..3).
// Pin order remains the historic CST order; no physical rewiring is required.
module wheel_channel_map(
 input wire[3:0]raw_a,raw_b,logical_pwm,logical_dir,
 output wire[3:0]encoder_a,encoder_b,pin_pwm,pin_dir);
 // Physical LF reads old RB, RF reads old LB, LB reads old RF, RB reads old LF.
 // Raw RF/LB (physical LB/RF) count negative for observed physical forward.
 assign encoder_a={raw_a[0],raw_b[1],raw_b[2],raw_a[3]};
 assign encoder_b={raw_b[0],raw_a[1],raw_a[2],raw_b[3]};
 // Old output slots0/1/2/3 physically drive LB/RB/LF/RF respectively.
 assign pin_pwm={logical_pwm[1],logical_pwm[0],logical_pwm[3],logical_pwm[2]};
 assign pin_dir={logical_dir[1],logical_dir[0],logical_dir[3],logical_dir[2]};
endmodule
