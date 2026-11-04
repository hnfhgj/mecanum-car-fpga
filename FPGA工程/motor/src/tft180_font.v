// Compact 5x7 ASCII glyphs in an 8x8 cell; bit0 is the top row.
module tft180_font(
    input wire [7:0] ascii,
    input wire [2:0] x, y,
    output wire pixel
);
    reg [39:0] glyph;
    reg [7:0] column;
    always @* begin
        case (ascii)
            8'h30: glyph = 40'h3e5149453e;
            8'h31: glyph = 40'h00427f4000;
            8'h32: glyph = 40'h4261514946;
            8'h33: glyph = 40'h2141454b31;
            8'h34: glyph = 40'h1814127f10;
            8'h35: glyph = 40'h2745454539;
            8'h36: glyph = 40'h3c4a494930;
            8'h37: glyph = 40'h0171090503;
            8'h38: glyph = 40'h3649494936;
            8'h39: glyph = 40'h064949291e;
            8'h41: glyph = 40'h7e1111117e;
            8'h42: glyph = 40'h7f49494936;
            8'h43: glyph = 40'h3e41414122;
            8'h44: glyph = 40'h7f4141221c;
            8'h45: glyph = 40'h7f49494941;
            8'h46: glyph = 40'h7f09090901;
            8'h47: glyph = 40'h3e4149497a;
            8'h48: glyph = 40'h7f0808087f;
            8'h49: glyph = 40'h00417f4100;
            8'h4a: glyph = 40'h2040413f01;
            8'h4b: glyph = 40'h7f08142241;
            8'h4c: glyph = 40'h7f40404040;
            8'h4d: glyph = 40'h7f020c027f;
            8'h4e: glyph = 40'h7f0408107f;
            8'h4f: glyph = 40'h3e4141413e;
            8'h50: glyph = 40'h7f09090906;
            8'h51: glyph = 40'h3e4151215e;
            8'h52: glyph = 40'h7f09192946;
            8'h53: glyph = 40'h4649494931;
            8'h54: glyph = 40'h01017f0101;
            8'h55: glyph = 40'h3f4040403f;
            8'h56: glyph = 40'h1f2040201f;
            8'h57: glyph = 40'h3f4038403f;
            8'h58: glyph = 40'h6314081463;
            8'h59: glyph = 40'h0708700807;
            8'h5a: glyph = 40'h6151494543;
            8'h2b: glyph = 40'h08083e0808;
            8'h2d: glyph = 40'h0808080808;
            8'h3d: glyph = 40'h1414141414;
            8'h3a: glyph = 40'h0036360000;
            8'h2e: glyph = 40'h0060600000;
            8'h2f: glyph = 40'h2010080402;
            default: glyph = 40'd0;
        endcase
        case (x)
            0: column = glyph[39:32];
            1: column = glyph[31:24];
            2: column = glyph[23:16];
            3: column = glyph[15:8];
            4: column = glyph[7:0];
            default: column = 0;
        endcase
    end
    assign pixel = column[y];
endmodule
