// 8N1 receiver. RX is synchronized; stop-bit errors discard the byte.
module n10_uart_rx #(
    parameter integer CLK_HZ=50000000, BAUD=230400
)(input wire clk, rst_n, rx, output reg [7:0] data,
  output reg valid, output reg framing_error);
    localparam integer BIT_TICKS=(CLK_HZ+BAUD/2)/BAUD;
    reg r1,r2;
    reg [1:0] state;
    reg [31:0] timer;
    reg [2:0] bit_index;
    reg [7:0] shift;
    always @(posedge clk or negedge rst_n)
      if(!rst_n) begin r1<=1; r2<=1; end
      else begin r1<=rx; r2<=r1; end
    always @(posedge clk or negedge rst_n) begin
      if(!rst_n) begin state<=0;timer<=0;bit_index<=0;shift<=0;data<=0;valid<=0;framing_error<=0;end
      else begin
        valid<=0; framing_error<=0;
        case(state)
          0: if(!r2) begin timer<=BIT_TICKS/2-1;state<=1;end
          1: if(timer!=0) timer<=timer-1;
             else if(r2) state<=0;
             else begin timer<=BIT_TICKS-1;bit_index<=0;state<=2;end
          2: if(timer!=0) timer<=timer-1;
             else begin
               shift[bit_index]<=r2;timer<=BIT_TICKS-1;
               if(bit_index==7) state<=3; else bit_index<=bit_index+1'b1;
             end
          3: if(timer!=0) timer<=timer-1;
             else begin
               state<=0;
               if(r2) begin data<=shift;valid<=1;end else framing_error<=1;
             end
        endcase
      end
    end
endmodule
