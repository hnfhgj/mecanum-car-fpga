// Two independent simple dual-port RAMs: 2 * 20480 * 16 = 655360 bits.
// Both read ports are synchronous; no initialization/reset on memory or q.
// Ownership handshake in ov5640_capture prohibits read/write bank collisions.
module ov5640_frame_ram(
    input wire wr_clk, input wire wr_en, input wire wr_bank,
    input wire [14:0] wr_addr, input wire [15:0] wr_pixel,
    input wire rd_clk, input wire rd_bank, input wire [14:0] rd_addr,
    output wire [15:0] rd_pixel
);
    (* ram_style = "block", syn_ramstyle = "block_ram" *) reg [15:0] bank0 [0:20479];
    (* ram_style = "block", syn_ramstyle = "block_ram" *) reg [15:0] bank1 [0:20479];
    reg [15:0] q0, q1;
    always @(posedge wr_clk) begin
        if (wr_en && !wr_bank) bank0[wr_addr] <= wr_pixel;
        if (wr_en &&  wr_bank) bank1[wr_addr] <= wr_pixel;
    end
    always @(posedge rd_clk) begin
        if (rd_addr < 20480) begin
            q0 <= bank0[rd_addr];
            q1 <= bank1[rd_addr];
        end else begin
            q0 <= 16'b0;
            q1 <= 16'b0;
        end
    end
    assign rd_pixel = rd_bank ? q1 : q0;
endmodule
