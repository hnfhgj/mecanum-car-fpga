// DVP contract: sample on PCLK rising edge; HREF active high; VSYNC active
// high blanking. Frame starts on VSYNC falling, completes on VSYNC rising.
// RGB565 first byte is bits 15:8, second byte bits 7:0 (4300=61).
// clk-side display pulses are one clk wide. START adopts only when idle
// (or coincident END); END unlocks the next START. Bank is frozen throughout
// the display transaction. rd_addr has one clk synchronous read latency.
// req toggles with ready_bank held stable until ack returns. ack is sent
// only after adoption, therefore writer can then use the former front bank.
module ov5640_capture #(
    parameter integer STALE_CYCLES = 50_000_000
)(
    input wire clk, input wire rst_n, input wire enable,
    input wire pclk, input wire vsync, input wire href, input wire [7:0] data,
    input wire [14:0] rd_addr,
    input wire display_frame_start, input wire display_frame_end,
    output wire [15:0] rd_pixel, output wire frame_valid
);
    wire crst_n, prst_n;
    ov5640_reset_sync creset(clk, rst_n, crst_n);
    ov5640_reset_sync preset(pclk, rst_n, prst_n);
    (* ASYNC_REG = "TRUE" *) reg [1:0] enable_p, ack_p, req_c;
    reg request, acknowledge, ready_bank, write_bank, pending;
    reg front_bank, have_front, display_active, request_seen;
    reg [31:0] age;
    reg vsync_d, href_d, active, malformed, half_byte;
    reg [7:0] high_byte;
    reg [9:0] line_bytes;
    reg [8:0] lines;
    reg [14:0] pixels;
    wire wr_en = active && !malformed && !pending && !vsync && href &&
                 half_byte && line_bytes < 320 && lines < 128 && pixels < 20480;
    wire [15:0] ram_pixel;
    ov5640_frame_ram ram(
        .wr_clk(pclk), .wr_en(wr_en), .wr_bank(write_bank),
        .wr_addr(pixels), .wr_pixel({high_byte,data}),
        .rd_clk(clk), .rd_bank(front_bank), .rd_addr(rd_addr), .rd_pixel(ram_pixel));
    assign frame_valid = have_front && enable && age < STALE_CYCLES;
    assign rd_pixel = frame_valid ? ram_pixel : 16'b0;
    always @(posedge pclk or negedge prst_n) begin
        if (!prst_n) begin
            enable_p <= 0; ack_p <= 0; request <= 0; ready_bank <= 1;
            write_bank <= 1; pending <= 0; vsync_d <= 0; href_d <= 0;
            active <= 0; malformed <= 0; half_byte <= 0;
            high_byte <= 0; line_bytes <= 0; lines <= 0; pixels <= 0;
        end else begin
            enable_p <= {enable_p[0],enable};
            ack_p <= {ack_p[0],acknowledge};
            vsync_d <= vsync; href_d <= href;
            if (pending && ack_p[1] == request) begin
                pending <= 0;
                write_bank <= ~write_bank;
            end
            if (!enable_p[1]) active <= 0;
            else if (vsync && !vsync_d) begin
                // Accept HREF falling on the same edge as VSYNC rising.
                if (active && !malformed && !href && !half_byte &&
                    pixels == 20480 &&
                    ((lines == 128 && !href_d) ||
                     (lines == 127 && href_d && line_bytes == 320))) begin
                    ready_bank <= write_bank;
                    request <= ~request;
                    pending <= 1;
                end
                active <= 0;
            end else if (!vsync && vsync_d) begin
                active <= !pending;
                malformed <= href; // Require blanking before first line.
                half_byte <= 0; line_bytes <= 0; lines <= 0; pixels <= 0;
            end else if (active && !vsync) begin
                if (href) begin
                    if (line_bytes >= 320 || lines >= 128) malformed <= 1;
                    if (line_bytes < 1023) line_bytes <= line_bytes + 1'b1;
                    if (!half_byte) high_byte <= data;
                    else if (wr_en) pixels <= pixels + 1'b1;
                    half_byte <= ~half_byte;
                end else if (href_d) begin
                    if (line_bytes != 320 || half_byte || lines >= 128)
                        malformed <= 1;
                    if (lines < 511) lines <= lines + 1'b1;
                    line_bytes <= 0; half_byte <= 0;
                end
            end
        end
    end
    always @(posedge clk or negedge crst_n) begin
        if (!crst_n) begin
            req_c <= 0; acknowledge <= 0; front_bank <= 0;
            have_front <= 0; display_active <= 0; request_seen <= 0;
            age <= STALE_CYCLES;
        end else begin
            req_c <= {req_c[0],request};
            // A newly completed valid frame refreshes age, even before adoption.
            // A queued frame cannot become fresh merely by adopting it late.
            request_seen <= req_c[1];
            if (req_c[1] != request_seen) age <= 0;
            else if (age < STALE_CYCLES) age <= age + 1'b1;
            if (display_frame_end) display_active <= 0;
            if (display_frame_start && (!display_active || display_frame_end)) begin
                display_active <= 1;
                if (req_c[1] != acknowledge) begin
                    front_bank <= ready_bank; // bundled data stable >2 clk edges
                    acknowledge <= req_c[1];
                    have_front <= 1;
                end
            end
        end
    end
endmodule
