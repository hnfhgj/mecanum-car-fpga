module nav_buttons #(parameter integer CLK_HZ=50000000,DEBOUNCE_MS=20,LONG_MS=800)(
 input wire clk,rst_n,input wire [3:0] key_n,
 output reg [3:0] press,release_event,long_press,output reg [3:0] stable);
 localparam integer DB=(CLK_HZ/1000*DEBOUNCE_MS)<1?1:CLK_HZ/1000*DEBOUNCE_MS;
 localparam integer LT=(CLK_HZ/1000*LONG_MS)<1?1:CLK_HZ/1000*LONG_MS;
 localparam integer CW=$clog2(DB+1),LW=$clog2(LT+1);
 (* ASYNC_REG="TRUE" *) reg [3:0] meta,sync;
 reg [CW-1:0] db_count[0:3];reg [LW-1:0] held[0:3];reg [3:0] sent;
 integer i;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin meta<=15;sync<=15;stable<=15;press<=0;release_event<=0;long_press<=0;sent<=0;
   for(i=0;i<4;i=i+1)begin db_count[i]<=0;held[i]<=0;end
  end else begin
   meta<=key_n;sync<=meta;press<=0;release_event<=0;long_press<=0;
   for(i=0;i<4;i=i+1)begin
    if(sync[i]==stable[i])db_count[i]<=0;
    else if(db_count[i]>=DB-1)begin
     stable[i]<=sync[i];db_count[i]<=0;press[i]<=!sync[i];release_event[i]<=sync[i];
    end else db_count[i]<=db_count[i]+1'b1;
    if(stable[i])begin held[i]<=0;sent[i]<=0;end
    else if(!sent[i])begin
     if(held[i]>=LT-1)begin long_press[i]<=1;sent[i]<=1;end
     else held[i]<=held[i]+1'b1;
    end
   end
  end
 end
endmodule
