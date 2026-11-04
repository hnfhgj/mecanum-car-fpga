// Standalone board-key UI. S0 is the external active-low reset (rst_n).
// gains_flat[n*32 +: 32]: LF P/I/D, RF P/I/D, LB P/I/D, RB P/I/D,
//                       heading P/I/D. All gains are unsigned Q12.
// Wheel gains are display-only. Only heading P (12) and D (14) are editable.
module dashboard_keys #(
    parameter integer CLK_FREQ = 50000000,
    parameter integer DEBOUNCE_MS = 20,
    parameter integer PAGE_COUNT = 3,
    parameter [2:0] INITIAL_PAGE = 0,
    parameter [31:0] WHEEL_KP = 32'd1229,
    parameter [31:0] WHEEL_KI = 32'd123,
    parameter [31:0] WHEEL_KD = 32'd205,
    // Heading P operates on centidegrees; heading D on degrees/second.
    parameter [31:0] HEADING_KP = 32'd320,
    parameter [31:0] HEADING_KD = 32'd2560
) (
    input  wire         clk,
    input  wire         rst_n,
    input  wire [3:0]   key_n,       // [0]=S1, [1]=S2, [2]=S3, [3]=S4
    output reg  [2:0]   page,
    output reg  [3:0]   selection,
    output reg  [479:0] gains_flat
);
    // Divide after multiplying, so sub-kHz test clocks also work. The
    // 64-bit intermediate avoids overflow at normal board clock rates.
    localparam [63:0] DEBOUNCE_CALC = (64'd1 * CLK_FREQ * DEBOUNCE_MS) / 1000;
    localparam [63:0] DEBOUNCE_TICKS = DEBOUNCE_CALC < 1 ? 1 : DEBOUNCE_CALC;
    localparam integer COUNT_WIDTH = DEBOUNCE_TICKS <= 1 ? 1 : $clog2(DEBOUNCE_TICKS);

    (* ASYNC_REG = "TRUE" *) reg [3:0] key_meta;
    (* ASYNC_REG = "TRUE" *) reg [3:0] key_sync;
    reg [3:0] key_stable;
    reg [3:0] key_press;
    reg [COUNT_WIDTH-1:0] debounce_count [0:3];
    integer k;
    integer field;

    function [31:0] clamp_gain;
        input [31:0] value;
        input [31:0] maximum;
        begin
            clamp_gain = value > maximum ? maximum : value;
        end
    endfunction

    function [31:0] adjust_gain;
        input [31:0] value;
        input [31:0] step;
        input [31:0] maximum;
        input increase;
        begin
            if (increase)
                adjust_gain = value >= maximum - step ? maximum : value + step;
            else
                adjust_gain = value <= step ? 32'd0 : value - step;
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            key_meta <= 4'b1111;
            key_sync <= 4'b1111;
        end else begin
            key_meta <= key_n;
            key_sync <= key_meta;
        end
    end

    // Each key must disagree with its accepted state for a full interval.
    // Only accepted falling edges produce events; holding never repeats.
    // Reset starts in the released state: a key held through reset produces
    // exactly one event once it has subsequently passed the debounce filter.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            key_stable <= 4'b1111;
            key_press <= 4'b0000;
            for (k = 0; k < 4; k = k + 1)
                debounce_count[k] <= 0;
        end else begin
            key_press <= 4'b0000;
            for (k = 0; k < 4; k = k + 1) begin
                if (key_sync[k] == key_stable[k]) begin
                    debounce_count[k] <= 0;
                end else if (debounce_count[k] == DEBOUNCE_TICKS - 1) begin
                    debounce_count[k] <= 0;
                    key_stable[k] <= key_sync[k];
                    key_press[k] <= !key_sync[k];
                end else begin
                    debounce_count[k] <= debounce_count[k] + 1'b1;
                end
            end
        end
    end

    // A selected 32-bit slice changes on one clock edge. Consumers can
    // sample their complete P/I/D tuple on the following control-loop tick.
    // Simultaneous navigation and adjustment use the pre-event page/field.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            page <= INITIAL_PAGE < PAGE_COUNT ? INITIAL_PAGE : 3'd0;
            selection <= 4'd12;
            for (field = 0; field < 12; field = field + 3) begin
                gains_flat[field*32 +: 32] <= clamp_gain(WHEEL_KP, 32'd8192);
                gains_flat[(field+1)*32 +: 32] <= clamp_gain(WHEEL_KI, 32'd8192);
                gains_flat[(field+2)*32 +: 32] <= clamp_gain(WHEEL_KD, 32'd4096);
            end
            gains_flat[12*32 +: 32] <= clamp_gain(HEADING_KP, 32'd1280);
            gains_flat[13*32 +: 32] <= 32'd0;
            gains_flat[14*32 +: 32] <= clamp_gain(HEADING_KD, 32'd16384);
        end else begin
            if (key_press[0])
                page <= page == PAGE_COUNT-1 ? 2'd0 : page + 1'b1;

            if (page == 2'd1) begin
                // S2 selects only the two editable heading fields.
                if (key_press[1]) begin
                    selection <= selection == 4'd12 ? 4'd14 : 4'd12;
                end
                if (key_press[2] ^ key_press[3]) begin
                    if (selection == 4'd12)
                        gains_flat[12*32 +: 32] <= adjust_gain(
                            gains_flat[12*32 +: 32], 32'd8, 32'd1280, key_press[2]);
                    else if (selection == 4'd14)
                        gains_flat[14*32 +: 32] <= adjust_gain(
                            gains_flat[14*32 +: 32], 32'd256, 32'd16384, key_press[2]);
                end
            end
        end
    end
endmodule
