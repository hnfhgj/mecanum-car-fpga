// Latest-state deadman input. Heartbeats never extend the 300ms key lease.
// Bits: W,S,A,D,CCW,CW. Invalid or opposite pairs cancel independently.
module hold_drive #(parameter integer CLK_HZ=50000000)(
 input wire clk,rst_n,tick,stop,command_valid,input wire [5:0] command_mask,
 input wire imu_ready,range_fresh,heading_ready,heading_fault,
 input wire [3:0] blocked,input wire [2:0] speed_level,
 input wire signed [15:0] heading_turn,
 output wire active,heading_enable,lease_valid,
 output wire [5:0] requested,
 output reg signed [15:0] forward_applied,left_applied,turn_applied,
 output reg signed [15:0] lf_target,rf_target,lb_target,rb_target);
 reg [5:0] held;reg [31:0] age;
 reg lease_q,sensors_ready_q,heading_ok_q;reg [3:0] blocked_q;
 reg signed [15:0] straight_pos,straight_neg,diagonal_pos,diagonal_neg;
 reg signed [15:0] f_axis,l_axis,u_axis,left_mix,right_mix,rotation_mix,f_mix,l_mix;
 reg axis_valid,mix_valid;
 assign requested=held;
 assign lease_valid=lease_q;
 assign active=lease_q && |held && sensors_ready_q && !stop;
 wire rotate=held[4]^held[5];
 assign heading_enable=active && !rotate;
 wire fw=held[0] && !held[1] && !blocked_q[0];
 wire bw=held[1] && !held[0] && !blocked_q[1];
 wire lt=held[2] && !held[3] && !blocked_q[2];
 wire rt=held[3] && !held[2] && !blocked_q[3];
 wire diagonal=(fw || bw) && (lt || rt);
 wire signed [15:0] pos_speed=diagonal?diagonal_pos:straight_pos;
 wire signed [15:0] neg_speed=diagonal?diagonal_neg:straight_neg;
 reg signed [15:0] f,l,u;
 always @*begin
  f=0;l=0;u=0;
  if(active && (rotate || heading_ok_q))begin
   f=fw?pos_speed:(bw?neg_speed:16'sd0);
   l=lt?pos_speed:(rt?neg_speed:16'sd0);
   u=rotate?(held[4]?16'sd110:-16'sd110):heading_turn;
   // A fully blocked/cancelled request must not move through heading correction.
   if(!rotate && !(fw || bw || lt || rt))u=0;
  end
 end
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin held<=0;age<=32'hffffffff;lf_target<=0;rf_target<=0;lb_target<=0;rb_target<=0;forward_applied<=0;left_applied<=0;turn_applied<=0;
   lease_q<=0;sensors_ready_q<=0;heading_ok_q<=0;blocked_q<=15;
   straight_pos<=210;straight_neg<=-210;diagonal_pos<=148;diagonal_neg<=-148;
   f_axis<=0;l_axis<=0;u_axis<=0;left_mix<=0;right_mix<=0;rotation_mix<=0;f_mix<=0;l_mix<=0;axis_valid<=0;mix_valid<=0;
  end
  else begin
   // Bound readiness fanout and break all steering carry chains at 50MHz.
   lease_q<=age<CLK_HZ*3/10;sensors_ready_q<=imu_ready && range_fresh;
   heading_ok_q<=heading_ready && !heading_fault;blocked_q<=blocked;
   // Exact floor(210*level/4) and floor(straight*181/256), including negatives.
   case(speed_level)
    1:begin straight_pos<=52;straight_neg<=-52;diagonal_pos<=36;diagonal_neg<=-36;end
    2:begin straight_pos<=105;straight_neg<=-105;diagonal_pos<=74;diagonal_neg<=-74;end
    3:begin straight_pos<=157;straight_neg<=-157;diagonal_pos<=110;diagonal_neg<=-110;end
    5:begin straight_pos<=262;straight_neg<=-262;diagonal_pos<=185;diagonal_neg<=-185;end
    6:begin straight_pos<=315;straight_neg<=-315;diagonal_pos<=222;diagonal_neg<=-222;end
    default:begin straight_pos<=210;straight_neg<=-210;diagonal_pos<=148;diagonal_neg<=-148;end
   endcase
   if(age!=32'hffffffff)age<=age+1'b1;
   if(command_valid)begin held<=command_mask;age<=0;end
   if(stop || !imu_ready || !range_fresh)begin held<=0;age<=32'hffffffff;end
   f_axis<=f;l_axis<=l;u_axis<=u;axis_valid<=active && (rotate || heading_ok_q);
   left_mix<=f_axis-l_axis;right_mix<=f_axis+l_axis;rotation_mix<=u_axis;
   f_mix<=f_axis;l_mix<=l_axis;mix_valid<=axis_valid;
   forward_applied<=f_mix;left_applied<=l_mix;turn_applied<=rotation_mix;
   lf_target<=left_mix-rotation_mix;rf_target<=right_mix+rotation_mix;lb_target<=right_mix-rotation_mix;rb_target<=left_mix+rotation_mix;
   if(stop || !active || !mix_valid)begin lf_target<=0;rf_target<=0;lb_target<=0;rb_target<=0;forward_applied<=0;left_applied<=0;turn_applied<=0;end
  end
 end
endmodule
