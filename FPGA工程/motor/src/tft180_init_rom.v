// Register sequence adapted from SeekFree zf_device_tft180.c (GPL-3.0-or-later).
// Copyright (c) 2022 SEEKFREE. FPGA adaptation 2026.
// Landscape MADCTL=A0; module-specific region is set by tft180_display.
module tft180_init_rom(input wire [6:0] addr, output reg [8:0] entry);
    always @* begin
        case (addr)
            7'd0: entry = 9'h011;
            7'd1: entry = 9'h0b1;
            7'd2: entry = 9'h101;
            7'd3: entry = 9'h12c;
            7'd4: entry = 9'h12d;
            7'd5: entry = 9'h0b2;
            7'd6: entry = 9'h101;
            7'd7: entry = 9'h12c;
            7'd8: entry = 9'h12d;
            7'd9: entry = 9'h0b3;
            7'd10: entry = 9'h101;
            7'd11: entry = 9'h12c;
            7'd12: entry = 9'h12d;
            7'd13: entry = 9'h101;
            7'd14: entry = 9'h12c;
            7'd15: entry = 9'h12d;
            7'd16: entry = 9'h0b4;
            7'd17: entry = 9'h107;
            7'd18: entry = 9'h0c0;
            7'd19: entry = 9'h1a2;
            7'd20: entry = 9'h102;
            7'd21: entry = 9'h184;
            7'd22: entry = 9'h0c1;
            7'd23: entry = 9'h1c5;
            7'd24: entry = 9'h0c2;
            7'd25: entry = 9'h10a;
            7'd26: entry = 9'h100;
            7'd27: entry = 9'h0c3;
            7'd28: entry = 9'h18a;
            7'd29: entry = 9'h12a;
            7'd30: entry = 9'h0c4;
            7'd31: entry = 9'h18a;
            7'd32: entry = 9'h1ee;
            7'd33: entry = 9'h0c5;
            7'd34: entry = 9'h10e;
            7'd35: entry = 9'h036;
            7'd36: entry = 9'h1a0;
            7'd37: entry = 9'h0e0;
            7'd38: entry = 9'h10f;
            7'd39: entry = 9'h11a;
            7'd40: entry = 9'h10f;
            7'd41: entry = 9'h118;
            7'd42: entry = 9'h12f;
            7'd43: entry = 9'h128;
            7'd44: entry = 9'h120;
            7'd45: entry = 9'h122;
            7'd46: entry = 9'h11f;
            7'd47: entry = 9'h11b;
            7'd48: entry = 9'h123;
            7'd49: entry = 9'h137;
            7'd50: entry = 9'h100;
            7'd51: entry = 9'h107;
            7'd52: entry = 9'h102;
            7'd53: entry = 9'h110;
            7'd54: entry = 9'h0e1;
            7'd55: entry = 9'h10f;
            7'd56: entry = 9'h11b;
            7'd57: entry = 9'h10f;
            7'd58: entry = 9'h117;
            7'd59: entry = 9'h133;
            7'd60: entry = 9'h12c;
            7'd61: entry = 9'h129;
            7'd62: entry = 9'h12e;
            7'd63: entry = 9'h130;
            7'd64: entry = 9'h130;
            7'd65: entry = 9'h139;
            7'd66: entry = 9'h13f;
            7'd67: entry = 9'h100;
            7'd68: entry = 9'h107;
            7'd69: entry = 9'h103;
            7'd70: entry = 9'h110;
            7'd71: entry = 9'h0f0;
            7'd72: entry = 9'h101;
            7'd73: entry = 9'h0f6;
            7'd74: entry = 9'h100;
            7'd75: entry = 9'h03a;
            7'd76: entry = 9'h105;
            7'd77: entry = 9'h029;
            default: entry = 9'h029;
        endcase
    end
endmodule
