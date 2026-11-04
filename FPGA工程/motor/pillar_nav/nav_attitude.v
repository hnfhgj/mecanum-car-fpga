// Same Q20 quaternion complementary algorithm and calibration as imu_attitude.
// A three-cycle micro-engine reuses ONE 27x32 DSP multiplier for all products.
// Two replicated synchronous memories provide two read ports and one write.
module nav_attitude #(parameter integer CLK_FREQ=50000000,CAL_SAMPLES=256,
 parameter signed [31:0] CORRECTION_KP=681574)(
 input wire clk,rst_n,sample_valid,input wire signed [15:0] ax,ay,az,gx,gy,gz,
 output reg signed [15:0] roll,pitch,yaw,output reg angle_valid,output wire ready);
 localparam BOOT=0,CONSTANTS=1,IDLE=2,LOAD=3,FETCH=4,EXECUTE=5,COMMIT=6,
  READ_ROLL=7,START_ROLL=8,WAIT_ROLL=9,WAIT_PITCH=10,WAIT_YAW=11;
 reg [3:0] state;reg [6:0] boot_index;reg [2:0] load_index;reg [7:0] pc;
 reg [31:0] bank_a[0:127],bank_b[0:127];
 reg signed [31:0] ra,rb,exec_result;
 wire [31:0] instruction;wire [3:0] opcode=instruction[30:27];
 wire [6:0] destination=instruction[26:20];
 reg [6:0] address_a,address_b,write_address;reg write_enable;reg [31:0] write_data;
 reg acc_ok;
 wire signed [58:0] product=$signed(ra[26:0])*$signed(rb);
 nav_attitude_program program_rom(.pc(pc),.instruction(instruction));
 reg [15:0] cal_count;reg signed [31:0] sum_x,sum_y,sum_z,bias_x,bias_y,bias_z;
 reg calibrated,healthy;reg [31:0] us_div,sample_us,dt_latched;
 reg signed [15:0] ax_latched,ay_latched,az_latched,gx_latched,gy_latched,gz_latched;
 assign ready=healthy;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin us_div<=0;sample_us<=0;healthy<=0;end
  else begin
   healthy<=calibrated && sample_us<100000;
   if(us_div>=CLK_FREQ/1000000-1)begin us_div<=0;if(sample_us<1000000)sample_us<=sample_us+1'b1;end
   else us_div<=us_div+1'b1;
   if(sample_valid)begin sample_us<=0;us_div<=0;end
  end
 end
 always @*begin
  address_a=instruction[19:13];address_b=instruction[12:6];
  if(state==READ_ROLL || state==START_ROLL)begin address_a=46;address_b=45;end
  if(state==WAIT_ROLL)begin address_a=44;address_b=0;end
  if(state==WAIT_PITCH)begin address_a=66;address_b=67;end
  write_enable=0;write_address=0;write_data=0;
  case(state)
   BOOT:begin write_enable=1;write_address=boot_index;end
   CONSTANTS:begin
    write_enable=1;write_address={4'd0,load_index};
    case(load_index)
     0:write_data=1048576;1:write_data=3145728;2:write_data=CORRECTION_KP;
     3:write_data=1116;4:write_data=549756;5:write_data=6;6:write_data=0;7:write_data=1048576;
    endcase
   end
   LOAD:begin
    write_enable=1;
    case(load_index)
     0:begin write_address=14;write_data=$signed(ax_latched)<<<8;end
     1:begin write_address=15;write_data=$signed(ay_latched)<<<8;end
     2:begin write_address=16;write_data=$signed(az_latched)<<<8;end
     3:begin write_address=17;write_data=$signed(gx_latched)-bias_x;end
     4:begin write_address=18;write_data=$signed(gy_latched)-bias_y;end
     5:begin write_address=19;write_data=$signed(gz_latched)-bias_z;end
     6:begin write_address=20;write_data=dt_latched;end
     default:begin write_address=6;write_data=0;end
    endcase
   end
   COMMIT:if(opcode!=7)begin write_enable=1;write_address=destination;write_data=exec_result;end
   default:begin end
  endcase
 end
 // No array reset: explicit boot writes keep these inferable as block RAM.
 always @(posedge clk)begin
  if(write_enable)begin bank_a[write_address]<=write_data;bank_b[write_address]<=write_data;end
  ra<=bank_a[address_a];rb<=bank_b[address_b];
 end
 reg cordic_start;reg signed [31:0] cordic_x,cordic_y;
 wire cordic_done;wire signed [15:0] cordic_angle;wire signed [31:0] cordic_magnitude;
 reg signed [15:0] roll_work,pitch_work;
 attitude_cordic cordic(.clk(clk),.rst_n(rst_n),.start(cordic_start),.in_x(cordic_x),.in_y(cordic_y),
  .done(cordic_done),.angle(cordic_angle),.magnitude(cordic_magnitude));
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin state<=BOOT;boot_index<=0;load_index<=0;pc<=0;exec_result<=0;acc_ok<=0;
   cal_count<=0;sum_x<=0;sum_y<=0;sum_z<=0;bias_x<=0;bias_y<=0;bias_z<=0;calibrated<=0;
   dt_latched<=0;ax_latched<=0;ay_latched<=0;az_latched<=0;gx_latched<=0;gy_latched<=0;gz_latched<=0;
   roll<=0;pitch<=0;yaw<=0;angle_valid<=0;cordic_start<=0;cordic_x<=0;cordic_y<=0;roll_work<=0;pitch_work<=0;
  end else begin
   angle_valid<=0;cordic_start<=0;
   case(state)
    BOOT:if(boot_index==127)begin load_index<=0;state<=CONSTANTS;end else boot_index<=boot_index+1'b1;
    CONSTANTS:if(load_index==7)state<=IDLE;else load_index<=load_index+1'b1;
    IDLE:if(sample_valid)begin
     if(!calibrated)begin
      if(gx>164 || gx< -164 || gy>164 || gy< -164 || gz>164 || gz< -164)begin cal_count<=0;sum_x<=0;sum_y<=0;sum_z<=0;end
      else if(cal_count==CAL_SAMPLES-1)begin
       bias_x<=(sum_x+gx)/CAL_SAMPLES;bias_y<=(sum_y+gy)/CAL_SAMPLES;bias_z<=(sum_z+gz)/CAL_SAMPLES;calibrated<=1;
      end else begin sum_x<=sum_x+gx;sum_y<=sum_y+gy;sum_z<=sum_z+gz;cal_count<=cal_count+1'b1;end
     end else if(sample_us>=1000 && sample_us<=20000)begin
      dt_latched<=sample_us;ax_latched<=ax;ay_latched<=ay;az_latched<=az;gx_latched<=gx;gy_latched<=gy;gz_latched<=gz;
      load_index<=0;state<=LOAD;
     end
    end
    LOAD:if(load_index==6)begin pc<=0;state<=FETCH;end else load_index<=load_index+1'b1;
    FETCH:state<=EXECUTE;
    EXECUTE:begin
     case(opcode)
      0:exec_result<=ra;
      1:exec_result<=ra+rb;
      2:exec_result<=ra-rb;
      3:exec_result<=product>>>20;
      4:exec_result<=product[31:0];
      5:exec_result<=ra<<<1;
      6:exec_result<=ra>>>1;
      7:acc_ok<=ra>671089 && ra<1509949;
      default:exec_result<=0;
     endcase
     if(instruction[31] && !acc_ok)exec_result<=0;
     state<=COMMIT;
    end
    COMMIT:if(pc==137)state<=READ_ROLL;else begin pc<=pc+1'b1;state<=FETCH;end
    READ_ROLL:state<=START_ROLL;
    START_ROLL:begin cordic_x<=ra;cordic_y<=rb;cordic_start<=1;state<=WAIT_ROLL;end
    WAIT_ROLL:if(cordic_done)begin roll_work<=cordic_angle;cordic_x<=cordic_magnitude;cordic_y<=-ra;cordic_start<=1;state<=WAIT_PITCH;end
    WAIT_PITCH:if(cordic_done)begin pitch_work<=cordic_angle;cordic_x<=ra;cordic_y<=rb;cordic_start<=1;state<=WAIT_YAW;end
    WAIT_YAW:if(cordic_done)begin roll<=roll_work;pitch<=pitch_work;yaw<=cordic_angle;angle_valid<=1;state<=IDLE;end
    default:state<=BOOT;
   endcase
  end
 end
endmodule
