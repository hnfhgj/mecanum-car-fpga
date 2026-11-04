// SEEKFREE wireless UART V2.4: 115200 8N1, RTS low permits a new byte.
// Protocol facts checked against the local SEEKFREE manual and wireless_uart.c.
// New independent RTL: complete-line telemetry queue; no motor command inputs.
module wireless_uart_bridge #(
 parameter integer CLK_HZ=50000000, BAUD=115200, ADDRESS_BITS=11
)(input wire clk,rst_n,input wire [7:0] stream_data,input wire stream_valid,
 input wire rx,rts,output wire tx);
 localparam integer DEPTH=1<<ADDRESS_BITS;
 reg [7:0] memory[0:DEPTH-1];
 reg [7:0] read_data;reg read_available;
 reg [ADDRESS_BITS-1:0] wr,rd,line_start;
 reg [ADDRESS_BITS:0] count,line_length;
 reg dropping,overflow,paused;
 reg rts1,rts2;
 wire [7:0] received;wire received_valid,framing_error;
 n10_uart_rx #(.CLK_HZ(CLK_HZ),.BAUD(BAUD)) receiver(
  .clk(clk),.rst_n(rst_n),.rx(rx),.data(received),.valid(received_valid),.framing_error(framing_error));
 reg [7:0] tx_data;reg send;wire busy;
 uart_tx #(.CLK_FREQ(CLK_HZ),.BAUD(BAUD)) transmitter(
  .clk(clk),.rst_n(rst_n),.data(tx_data),.send(send),.busy(busy),.tx(tx));
 reg pending,boundary;reg [3:0] reply_index;
 // Replies are inserted only between complete lines, never inside IO HEX frames.
 function [7:0] reply_byte;
  input [3:0] index;input lost;
  reg [103:0] text;
  begin text=lost?"WIRELESS DROP":"WIRELESS OK  ";
   if(index<=13)reply_byte=text[(13-index)*8+:8];
   else if(index==14)reply_byte=13;else reply_byte=10;
  end
 endfunction
 wire reply_start=pending && boundary && reply_index==0;
 wire ready=!busy && !send && !rts2;
 wire pop=ready && reply_index==0 && !reply_start && count!=0 && read_available;
 wire capacity=({1'b0,count}+{1'b0,line_length})<DEPTH;
 wire incoming=stream_valid && !paused;
 wire commit=incoming && !dropping && capacity && stream_data==10;
 // Separate unreset synchronous memory ports allow Gowin to infer SDPB RAM.
 always @(posedge clk)begin
  if(rst_n && incoming && !dropping && capacity)memory[wr]<=stream_data;
  read_data<=memory[rd];
 end
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   wr<=0;rd<=0;line_start<=0;count<=0;line_length<=0;dropping<=0;
   overflow<=0;paused<=0;rts1<=1;rts2<=1;tx_data<=0;send<=0;
   pending<=0;boundary<=1;reply_index<=0;read_available<=0;
  end else begin
   rts1<=rts;rts2<=rts1;send<=0;
   read_available<=count!=0 && !pop;
   // P discards the uncommitted line; R resumes at the next complete line.
   if(received_valid && received=="?")pending<=1;
   if(received_valid && received=="P")paused<=1;
   if(received_valid && received=="R")paused<=0;
   if(stream_valid && paused)begin
    wr<=line_start;line_length<=0;dropping<=stream_data!=10;
   end else if(incoming)begin
    if(dropping)begin if(stream_data==10)dropping<=0;end
    else if(!capacity)begin
     wr<=line_start;line_length<=0;dropping<=stream_data!=10;overflow<=1;
    end else begin
     wr<=wr+1'b1;
     if(stream_data==10)begin line_start<=wr+1'b1;line_length<=0;end
     else line_length<=line_length+1'b1;
    end
   end
   case({commit,pop})
    2'b10:count<=count+line_length+1'b1;
    2'b01:count<=count-1'b1;
    2'b11:count<=count+line_length;
    default:begin end
   endcase
   if(pop)begin tx_data<=read_data;send<=1;rd<=rd+1'b1;boundary<=read_data==10;end
   if(ready && reply_start)begin
    tx_data<=reply_byte(1,overflow);send<=1;reply_index<=2;pending<=0;
   end else if(ready && reply_index!=0)begin
    tx_data<=reply_byte(reply_index,overflow);send<=1;
    if(reply_index==15)begin reply_index<=0;boundary<=1;end
    else reply_index<=reply_index+1'b1;
   end
  end
 end
endmodule
