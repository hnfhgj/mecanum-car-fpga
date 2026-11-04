// Register task selection before the speed loop and polarity conversion before PWM.
// This cuts the measured 50MHz target->loop->driver critical paths.
module pc_motor_pipeline(input wire clk,rst_n,blocked,
 input wire signed [15:0] selected_target,pid_duty,
 output reg signed [15:0] target,drive);
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin target<=0;drive<=0;end
  else if(blocked)begin target<=0;drive<=0;end
  else begin target<=selected_target;drive<=-pid_duty;end
 end
endmodule
