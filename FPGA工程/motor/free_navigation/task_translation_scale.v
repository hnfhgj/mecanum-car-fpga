// 25%-step translation scale. Shift/add only; no DSP, no heading scaling.
module task_translation_scale(input wire [6:0] base,input wire [2:0] level,output wire [15:0] scaled);
 reg [12:0] product;
 reg [12:0] value;
 // All built-in translation levels increase by 4/3 then 7/4; preserve zero stop.
 always @*case(base)
  0:value=0;15:value=35;30:value=70;45:value=105;60:value=140;90:value=210;
  default:value=({6'd0,base}*7)/4;
 endcase
 always @*case(level)
  1:product=value;
  2:product=value<<1;
  3:product=(value<<1)+value;
  5:product=(value<<2)+value;
  6:product=(value<<2)+(value<<1);
  default:product=value<<2;
 endcase
 assign scaled={5'd0,product[12:2]};
endmodule
