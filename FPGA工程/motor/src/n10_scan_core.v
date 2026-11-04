// Latest COMPLETE revolution only. A display-frame boundary transfers ownership.
// Invalid/zero/out-of-range distances are omitted, never drawn at the origin.
module n10_scan_core #(
 parameter integer CLK_HZ=50000000,CELL_SHIFT=6,
 parameter [15:0] ROTATE_CDEG=0,
 parameter [31:0] STALE_CYCLES=CLK_HZ/2
)(input wire clk,rst_n,input wire [7:0] byte_data,
 input wire byte_valid,byte_error,display_start,
 input wire [13:0] display_addr,output wire display_bit,
 output reg front_valid,output wire online,
 output wire [31:0] packet_count,crc_errors,drop_count,
 output reg [31:0] scan_count,invalid_packets);
 wire [7:0] queue_data;
 wire queue_available;
 reg queue_pop;
 reg [6:0] queue_index;
 n10_packet_queue queue(.clk(clk),.rst_n(rst_n),.byte_data(byte_data),.byte_valid(byte_valid),
  .byte_error(byte_error),.time_ms(32'd0),.flush(1'b0),.pop(queue_pop),.read_index(queue_index),
  .read_data(queue_data),.available(queue_available),.good_count(packet_count),.checksum_errors(crc_errors),.drop_count(drop_count));
 reg [31:0] age,seen_packets,seen_crc,seen_drops;
 reg online_old;
 assign online=packet_count!=0 && age<STALE_CYCLES;
 wire broken=(crc_errors!=seen_crc)||(drop_count!=seen_drops)||byte_error||(online_old && !online);
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin age<=STALE_CYCLES;seen_packets<=0;seen_crc<=0;seen_drops<=0;online_old<=0;end
  else begin
   seen_packets<=packet_count;seen_crc<=crc_errors;seen_drops<=drop_count;online_old<=online;
   if(packet_count!=seen_packets)age<=0;else if(age<STALE_CYCLES)age<=age+1'b1;
  end
 end
 localparam CLEAR=0,IDLE=1,LOAD_WAIT=2,LOAD_BYTE=3,DECODE=4,POINT=5,PROJECT=6,HIT=7,NEXT=8;
 reg [3:0] state;
 reg initializing,clear_second,wr_bank,front_bank,pending,collecting,previous_valid;
 reg [13:0] clear_addr;
 reg [7:0] packet[0:57];
 reg [5:0] load_index;
 reg [3:0] point_index;
 reg [15:0] angle,step;
 reg [3:0] remainder,fraction;
 reg [15:0] previous_angle;
 wire [15:0] start_angle={packet[5],packet[6]};
 wire [15:0] end_angle={packet[55],packet[56]};
 wire [16:0] span=end_angle>=start_angle ? {1'b0,end_angle}-{1'b0,start_angle} :
                            {1'b0,end_angle}+17'd36000-{1'b0,start_angle};
 wire [15:0] normalized_angle=angle>=36000?angle-16'd36000:angle;
 wire wraps=previous_valid && {1'b0,previous_angle}>{1'b0,normalized_angle}+17'd18000;
 wire [4:0] fraction_sum={1'b0,fraction}+{1'b0,remainder};
 wire [4:0] next_fraction=(fraction_sum >= 5'd15) ? (fraction_sum - 5'd15) : fraction_sum;
 wire [5:0] range_offset=6'd7+{2'b0,point_index}*6'd3;
 wire [15:0] point_range={packet[range_offset],packet[range_offset+1'b1]};
 reg projector_start;
 wire projector_done,in_bounds;
 wire [6:0] gx,gy;
 n10_project_point #(.CELL_SHIFT(CELL_SHIFT),.ROTATE_CDEG(ROTATE_CDEG)) projector(
  .clk(clk),.rst_n(rst_n),.start(projector_start),.angle_cdeg(normalized_angle),.distance_mm(point_range),
  .done(projector_done),.in_bounds(in_bounds),.x(gx),.y(gy));
 wire ram_write=state==CLEAR || (state==HIT && collecting && !pending && in_bounds);
 wire [13:0] ram_addr=state==CLEAR?clear_addr:{gy,gx};
 n10_scan_ram ram(.clk(clk),.wr_en(ram_write),.wr_bank(wr_bank),.wr_addr(ram_addr),.wr_bit(state!=CLEAR),
  .rd_bank(front_bank),.rd_addr(display_addr),.rd_bit(display_bit));
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   state<=CLEAR;initializing<=1;clear_second<=0;wr_bank<=0;front_bank<=0;clear_addr<=0;
   pending<=0;collecting<=0;previous_valid<=0;previous_angle<=0;front_valid<=0;
   load_index<=0;point_index<=0;angle<=0;step<=0;remainder<=0;fraction<=0;
   queue_index<=16;queue_pop<=0;projector_start<=0;scan_count<=0;invalid_packets<=0;
  end else begin
   queue_pop<=0;projector_start<=0;
   if(!initializing && broken)begin
    state<=CLEAR;clear_addr<=0;pending<=0;collecting<=0;previous_valid<=0;
   end else if(pending && display_start && !initializing)begin
    front_bank<=wr_bank;front_valid<=1;wr_bank<=!wr_bank;
    pending<=0;collecting<=0;previous_valid<=0;clear_addr<=0;state<=CLEAR;
   end else case(state)
    CLEAR:if(clear_addr==16383)begin
     clear_addr<=0;
     if(initializing && !clear_second)begin wr_bank<=1;clear_second<=1;end
     else begin initializing<=0;state<=IDLE;end
    end else clear_addr<=clear_addr+1'b1;
    IDLE:if(queue_available && !queue_pop)begin load_index<=0;queue_index<=16;state<=LOAD_WAIT;end
    LOAD_WAIT:state<=LOAD_BYTE;
    LOAD_BYTE:begin
     packet[load_index]<=queue_data;
     if(load_index==57)state<=DECODE;
     else begin load_index<=load_index+1'b1;queue_index<=queue_index+1'b1;state<=LOAD_WAIT;end
    end
    DECODE:begin
     queue_pop<=1;
     if(start_angle>36000 || end_angle>36000 || span==0 || span>3000)begin
      invalid_packets<=invalid_packets+1'b1;state<=CLEAR;clear_addr<=0;
      pending<=0;collecting<=0;previous_valid<=0;
     end else begin
      // DECODE rejects spans above 3000; this slice cannot discard a valid bit.
      angle<=start_angle;step<=span[15:0]/16'd15;remainder<=span%15;
      fraction<=0;point_index<=0;state<=POINT;
     end
    end
    POINT:begin
     previous_valid<=1;previous_angle<=normalized_angle;
     if(wraps && collecting)begin pending<=1;collecting<=0;scan_count<=scan_count+1'b1;state<=NEXT;end
     else if((wraps && !pending) || collecting)begin
      if(wraps)collecting<=1;
      projector_start<=1;state<=PROJECT;
     end else state<=NEXT;
    end
    PROJECT:if(projector_done)state<=HIT;
    HIT:state<=NEXT;
    NEXT:if(point_index==15)state<=IDLE;
     else begin
      point_index<=point_index+1'b1;
      angle<=angle+step+(fraction_sum>=15?16'd1:16'd0);
      fraction<=next_fraction[3:0];
      state<=POINT;
     end
    default:state<=CLEAR;
   endcase
  end
 end
endmodule
