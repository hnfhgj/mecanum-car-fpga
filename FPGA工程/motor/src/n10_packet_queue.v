// Sliding-window sync recovers from noise, damaged/truncated packets and A5 A5 5A.
// Whole frames are queued/dropped atomically. Never overwrites an in-flight frame.
module n10_packet_queue #(
    parameter integer DEPTH=8
)(input wire clk,rst_n, input wire [7:0] byte_data,
  input wire byte_valid, byte_error, input wire [31:0] time_ms,
  input wire flush, pop, input wire [6:0] read_index,
  output reg [7:0] read_data, output wire available,
  output reg [31:0] good_count, checksum_errors, drop_count);
    reg [7:0] window[0:57];
    reg [7:0] sum;
    reg [6:0] seen;
    reg [7:0] payload[0:DEPTH*58-1];
    reg [7:0] snapshot[0:57];
    reg copying;
    reg [5:0] copy_index;
    reg [31:0] pending_seq,pending_time;
    reg [31:0] seq_mem[0:DEPTH-1],time_mem[0:DEPTH-1];
    localparam integer PTR_W=(DEPTH<=2)?1:$clog2(DEPTH);
    localparam integer USED_W=$clog2(DEPTH+1);
    reg [PTR_W-1:0] wr_ptr,rd_ptr;
    reg [USED_W-1:0] used;
    wire candidate=byte_valid && !byte_error && seen>=57 &&
                   window[1]==8'ha5 && window[2]==8'h5a && window[3]==8'h3a;
    wire [7:0] expected=sum-window[0];
    wire accept=candidate && expected==byte_data;
    wire consume=pop && used!=0 && !flush;
    wire commit=copying && copy_index==57 && !flush;
    wire [USED_W:0] reserved_used={1'b0,used}+copying;
    wire reserve=accept && !flush && (!copying || commit) && (reserved_used<DEPTH || consume);
    wire [31:0] discarded=(accept && !reserve)+(flush?used+copying:0);
    assign available=(used!=0);
    integer j;
    // Registered read and a single byte write per cycle allow block-RAM mapping.
    // The SPI engine waits several clocks after changing read_index before sampling.
    always @(posedge clk) begin
      case(read_index)
        0:read_data<="N";1:read_data<="1";2:read_data<="0";3:read_data<="F";
        4:read_data<=1;5:read_data<=0;6:read_data<=58;7:read_data<=0;
        8:read_data<=seq_mem[rd_ptr][7:0];9:read_data<=seq_mem[rd_ptr][15:8];
        10:read_data<=seq_mem[rd_ptr][23:16];11:read_data<=seq_mem[rd_ptr][31:24];
        12:read_data<=time_mem[rd_ptr][7:0];13:read_data<=time_mem[rd_ptr][15:8];
        14:read_data<=time_mem[rd_ptr][23:16];15:read_data<=time_mem[rd_ptr][31:24];
        default: if(read_index<74)read_data<=payload[rd_ptr*58+read_index-16];else read_data<=0;
      endcase
    end
    always @(posedge clk or negedge rst_n) begin
      if(!rst_n) begin
        sum<=0;seen<=0;wr_ptr<=0;rd_ptr<=0;used<=0;
        copying<=0;copy_index<=0;pending_seq<=0;pending_time<=0;
        good_count<=0;checksum_errors<=0;drop_count<=0;
        for(j=0;j<58;j=j+1)window[j]<=0;
      end else begin
        if(byte_error) seen<=0;
        else if(byte_valid) begin
          for(j=0;j<57;j=j+1)window[j]<=window[j+1];
          window[57]<=byte_data;
          sum<=sum-window[0]+byte_data;
          if(seen<58)seen<=seen+1'b1;
        end
        if(candidate && !accept)checksum_errors<=checksum_errors+1'b1;
        if(accept) begin
          good_count<=good_count+1'b1;
        end
        if(discarded!=0)drop_count<=drop_count+discarded;
        if(copying && !flush)begin
          payload[wr_ptr*58+copy_index]<=snapshot[copy_index];
          if(copy_index==57)begin copying<=0;copy_index<=0;end
          else copy_index<=copy_index+1'b1;
        end
        if(commit)begin
          seq_mem[wr_ptr]<=pending_seq;time_mem[wr_ptr]<=pending_time;
          if(wr_ptr==DEPTH-1)wr_ptr<=0;else wr_ptr<=wr_ptr+1'b1;
        end
        if(reserve)begin
          for(j=0;j<57;j=j+1)snapshot[j]<=window[j+1];
          snapshot[57]<=byte_data;
          pending_seq<=good_count;pending_time<=time_ms;
          copying<=1;copy_index<=0;
        end
        if(consume)begin if(rd_ptr==DEPTH-1)rd_ptr<=0;else rd_ptr<=rd_ptr+1'b1;end
        case({commit,consume})
          2'b10:used<=used+1'b1;
          2'b01:used<=used-1'b1;
          default:;
        endcase
        if(flush)begin used<=0;wr_ptr<=0;rd_ptr<=0;copying<=0;copy_index<=0;end
      end
    end
endmodule
