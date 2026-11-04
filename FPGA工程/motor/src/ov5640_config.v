// SCCB 7-bit address 3c, 16-bit big-endian register addresses.
// Reuses i2c_master unchanged. No clock-stretch support in that engine.
// MAX_RETRIES counts retries AFTER the initial attempt, per register.
// status: 0 busy/ok, 1 NACK, 2 byte-engine timeout, 3 wrong chip ID.
// fault is sticky until reset; init_done is asserted only after all writes.
module ov5640_config #(
    parameter integer CLK_FREQ = 50_000_000,
    parameter integer I2C_FREQ = 100_000,
    parameter integer MAX_RETRIES = 3,
    parameter integer HW_RESET_CYCLES = CLK_FREQ / 1000,
    parameter integer BOOT_CYCLES = CLK_FREQ / 50,
    parameter integer SOFT_RESET_CYCLES = CLK_FREQ / 200,
    parameter integer BYTE_TIMEOUT_CYCLES = CLK_FREQ / 100,
    parameter integer RETRY_CYCLES = CLK_FREQ / 1000
)(
    input wire clk, input wire rst_n,
    output wire reset_n, output wire pwdn, output wire scl, inout wire sda,
    output reg init_done, output reg fault, output reg [2:0] status
);
    localparam HOLD=0, BOOT=1, LAUNCH=2, BYTE_WAIT=3, RETRY=4,
               RECOVER=5, SOFT_WAIT=6, DONE=7, FAILED=8;
    reg [3:0] state;
    reg [31:0] timer, retries;
    reg [1:0] operation; // 0=ID 300a, 1=ID 300b, 2=ROM write
    reg [2:0] phase;
    reg [7:0] index;
    reg txn_nack;
    wire [23:0] rom_value;
    ov5640_config_rom rom(index,rom_value);
    wire [15:0] address = operation == 0 ? 16'h300a :
                          operation == 1 ? 16'h300b : rom_value[23:8];
    wire reading = operation != 2;
    wire last = reading ? phase == 4 : phase == 3;
    wire first = phase == 0 || (reading && phase == 3);
    wire read_byte = reading && phase >= 3;
    wire [7:0] tx = phase == 1 ? address[15:8] :
                    phase == 2 ? address[7:0] : rom_value[7:0];
    wire byte_done, nack, busy;
    wire [7:0] rx;
    wire master_rst_n = rst_n && state != RECOVER && state != FAILED;
    i2c_master #(.CLK_FREQ(CLK_FREQ),.I2C_FREQ(I2C_FREQ)) bus(
        .clk(clk),.rst_n(master_rst_n),.start(state == LAUNCH),
        .tx_data(tx),.read_byte(read_byte),.first_byte(first),.last_byte(last),
        .dev_addr(7'h3c),.busy(busy),.rx_data(rx),.rx_done(byte_done),
        .nack(nack),.sda(sda),.scl(scl));
    assign reset_n = rst_n && state != HOLD;
    assign pwdn = 1'b0;
    wire phase_nack = nack && (!read_byte || first);
    wire id_bad = reading && rx != (operation == 0 ? 8'h56 : 8'h40);
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= HOLD; timer <= 0; retries <= 0; operation <= 0;
            phase <= 0; index <= 0; txn_nack <= 0;
            init_done <= 0; fault <= 0; status <= 0;
        end else begin
            case (state)
                HOLD: if (timer >= HW_RESET_CYCLES-1) begin
                    timer <= 0; state <= BOOT;
                end else timer <= timer + 1'b1;
                BOOT: if (timer >= BOOT_CYCLES-1) begin
                    timer <= 0; state <= LAUNCH;
                end else timer <= timer + 1'b1;
                LAUNCH: begin timer <= 0; state <= BYTE_WAIT; end
                BYTE_WAIT: begin
                    if (byte_done) begin
                        timer <= 0;
                        if (!last) begin
                            // Finish through STOP even after NACK, then retry.
                            txn_nack <= txn_nack | phase_nack;
                            phase <= phase + 1'b1; state <= LAUNCH;
                        end else if (txn_nack || phase_nack || id_bad) begin
                            status <= (txn_nack || phase_nack) ? 3'd1 : 3'd3;
                            phase <= 0; txn_nack <= 0;
                            if (retries >= MAX_RETRIES) begin
                                fault <= 1; state <= FAILED;
                            end else begin
                                retries <= retries + 1'b1; state <= RETRY;
                            end
                        end else begin
                            retries <= 0; phase <= 0; txn_nack <= 0; status <= 0;
                            if (operation == 0) begin
                                operation <= 1; state <= LAUNCH;
                            end else if (operation == 1) begin
                                operation <= 2; state <= LAUNCH;
                            end else if (index == 251) begin
                                init_done <= 1; state <= DONE;
                            end else begin
                                index <= index + 1'b1;
                                state <= index == 1 ? SOFT_WAIT : LAUNCH;
                            end
                        end
                    end else if (timer >= BYTE_TIMEOUT_CYCLES-1) begin
                        status <= 2; timer <= 0; state <= RECOVER;
                        phase <= 0; txn_nack <= 0;
                    end else timer <= timer + 1'b1;
                end
                RETRY: if (timer >= RETRY_CYCLES-1) begin
                    timer <= 0; state <= LAUNCH;
                end else timer <= timer + 1'b1;
                RECOVER: if (timer >= RETRY_CYCLES-1) begin
                    timer <= 0;
                    if (retries >= MAX_RETRIES) begin
                        fault <= 1; state <= FAILED;
                    end else begin
                        retries <= retries + 1'b1; state <= LAUNCH;
                    end
                end else timer <= timer + 1'b1;
                SOFT_WAIT: if (timer >= SOFT_RESET_CYCLES-1) begin
                    timer <= 0; state <= LAUNCH;
                end else timer <= timer + 1'b1;
                DONE: state <= DONE;
                FAILED: state <= FAILED;
                default: begin fault <= 1; state <= FAILED; end
            endcase
        end
    end
endmodule
