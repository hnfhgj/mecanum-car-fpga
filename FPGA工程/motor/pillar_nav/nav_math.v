// Shared sequential arithmetic. No inferred DSP multiplier or combinational divider.
module nav_math(input wire clk,rst_n,cancel,start,input wire [1:0] op,
 input wire signed [63:0] a,input wire signed [31:0] b,
 output reg busy,done,zero_divide,output reg signed [63:0] result);
 reg [1:0] operation;
 reg negative;
 reg [6:0] count;
 reg [63:0] accumulator,shift_a,numerator,quotient,radicand;
 reg [31:0] shift_b,denominator,root;
 reg [64:0] remainder;
 wire [63:0] product_next=accumulator+(shift_b[0]?shift_a:64'd0);
 wire [64:0] division_rem={remainder[63:0],numerator[63]};
 wire division_bit=division_rem>={33'd0,denominator};
 wire [63:0] division_q={quotient[62:0],division_bit};
 wire [64:0] sqrt_rem={remainder[62:0],radicand[63:62]};
 wire [64:0] sqrt_trial=({33'd0,root}<<2)|65'd1;
 wire sqrt_bit=sqrt_rem>=sqrt_trial;
 wire [31:0] sqrt_root={root[30:0],sqrt_bit};
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin busy<=0;done<=0;zero_divide<=0;result<=0;operation<=0;negative<=0;
   count<=0;accumulator<=0;shift_a<=0;shift_b<=0;numerator<=0;quotient<=0;
   denominator<=0;remainder<=0;root<=0;radicand<=0;end
  else begin
   done<=0;
   if(cancel)begin busy<=0;end
   else if(start && !busy)begin
    operation<=op;negative<=a[63]^b[31];count<=0;zero_divide<=0;
    accumulator<=0;shift_a<=a[63]?-a:a;shift_b<=b[31]?-b:b;
    numerator<=a[63]?-a:a;denominator<=b[31]?-b:b;
    quotient<=0;remainder<=0;root<=0;radicand<=a;busy<=1;
    if(op==1 && b==0)begin result<=0;zero_divide<=1;done<=1;busy<=0;end
   end else if(busy)begin
    count<=count+1'b1;
    case(operation)
     0:begin
      accumulator<=product_next;shift_a<=shift_a<<1;shift_b<=shift_b>>1;
      if(count==31)begin result<=negative?-$signed(product_next):$signed(product_next);busy<=0;done<=1;end
     end
     1:begin
      numerator<=numerator<<1;quotient<=division_q;
      remainder<=division_bit?division_rem-{33'd0,denominator}:division_rem;
      if(count==63)begin result<=negative?-$signed(division_q):$signed(division_q);busy<=0;done<=1;end
     end
     2:begin
      radicand<=radicand<<2;root<=sqrt_root;remainder<=sqrt_bit?sqrt_rem-sqrt_trial:sqrt_rem;
      if(count==31)begin result<={32'd0,sqrt_root};busy<=0;done<=1;end
     end
     default:begin result<=0;busy<=0;done<=1;end
    endcase
   end
  end
 end
endmodule
