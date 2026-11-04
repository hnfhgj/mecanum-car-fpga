// Fixed 14-byte command: A5 5A 01 seq op aH aL bH bL cH cL sum 0D 0A.
// sum is the modulo-256 sum of bytes 2..10; no event before the entire frame.
module pc_command_rx #(parameter integer CLK_HZ=50000000)(
 input wire clk,rst_n,input wire [7:0] data,input wire valid,framing_error,
 output reg command_valid,output reg [7:0] seq,operation,
 output reg [15:0] a,b,c,output wire in_frame);
 reg [3:0] position;reg [7:0] checksum;reg checksum_ok;
 reg [31:0] age;
 assign in_frame=position!=0;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin position<=0;checksum<=0;checksum_ok<=0;age<=0;
   command_valid<=0;seq<=0;operation<=0;a<=0;b<=0;c<=0;end
  else begin
   command_valid<=0;
   if(position!=0 && age<CLK_HZ/20)age<=age+1'b1;
   if(framing_error || (position!=0 && age>=CLK_HZ/20))begin position<=0;age<=0;end
   else if(valid)begin
    age<=0;
    case(position)
     0:if(data==8'hA5)position<=1;
     1:if(data==8'h5A)begin position<=2;checksum<=0;end
       else position<=data==8'hA5 ? 1 : 0;
     2:if(data==1)begin checksum<=data;position<=3;end else position<=0;
     3:begin seq<=data;checksum<=checksum+data;position<=4;end
     4:begin operation<=data;checksum<=checksum+data;position<=5;end
     5:begin a[15:8]<=data;checksum<=checksum+data;position<=6;end
     6:begin a[7:0]<=data;checksum<=checksum+data;position<=7;end
     7:begin b[15:8]<=data;checksum<=checksum+data;position<=8;end
     8:begin b[7:0]<=data;checksum<=checksum+data;position<=9;end
     9:begin c[15:8]<=data;checksum<=checksum+data;position<=10;end
     10:begin c[7:0]<=data;checksum<=checksum+data;position<=11;end
     11:begin checksum_ok<=data==checksum;position<=12;end
     12:if(data==13)position<=13;else position<=0;
     13:begin command_valid<=checksum_ok && data==10;position<=0;end
     default:position<=0;
    endcase
   end
  end
 end
endmodule
