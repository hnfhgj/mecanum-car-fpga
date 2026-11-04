// Sequential vectoring CORDIC: atan2(y,x), centidegrees and vector magnitude.
module nav_atan2(
    input wire clk,rst_n,start,
    input wire signed [31:0] in_x,in_y,
    output reg done,
    output reg signed [15:0] angle,
    output reg signed [31:0] magnitude
);
    reg busy;
    reg signed [34:0] x,y;
    reg signed [31:0] z;
    reg [4:0] iteration;
    reg signed [31:0] atan_step;
    // Angle-only CORDIC; magnitude is unused and no DSP is allocated.
    wire signed [47:0] z_ext=z;
    wire signed [47:0] scaled_angle=(z_ext<<<6)+(z_ext<<<5)+(z_ext<<<2);
    always @* begin
        case(iteration)
                0: atan_step=32'sd2949120;
                1: atan_step=32'sd1740967;
                2: atan_step=32'sd919879;
                3: atan_step=32'sd466945;
                4: atan_step=32'sd234379;
                5: atan_step=32'sd117304;
                6: atan_step=32'sd58666;
                7: atan_step=32'sd29335;
                8: atan_step=32'sd14668;
                9: atan_step=32'sd7334;
                10: atan_step=32'sd3667;
                11: atan_step=32'sd1833;
                12: atan_step=32'sd917;
                13: atan_step=32'sd458;
                14: atan_step=32'sd229;
                15: atan_step=32'sd115;
                16: atan_step=32'sd57;
                17: atan_step=32'sd29;
                18: atan_step=32'sd14;
                19: atan_step=32'sd7;
            default: atan_step=0;
        endcase
    end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin busy<=0; done<=0; angle<=0; magnitude<=0; x<=0; y<=0; z<=0; iteration<=0; end
        else begin
            done<=0;
            if(start && !busy) begin
                busy<=1; iteration<=0;
                if(in_x<0) begin
                    x<=-$signed({{3{in_x[31]}},in_x}); y<=-$signed({{3{in_y[31]}},in_y});
                    z<=in_y>=0 ? 32'sd11796480 : -32'sd11796480;
                end else begin x<=in_x; y<=in_y; z<=0; end
            end else if(busy) begin
                if(iteration==20) begin
                    angle<=scaled_angle>>>16;
                    magnitude<=0;
                    done<=1; busy<=0;
                end else begin
                    if(y>=0) begin x<=x+(y>>>iteration); y<=y-(x>>>iteration); z<=z+atan_step; end
                    else begin x<=x-(y>>>iteration); y<=y+(x>>>iteration); z<=z-atan_step; end
                    iteration<=iteration+1'b1;
                end
            end
        end
    end
endmodule
