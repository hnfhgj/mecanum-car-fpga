//============================================================================
// imu660ra_config_rom.v -- IMU660RA (Bosch BMI270) 8192-byte config file ROM
//============================================================================
module imu660ra_config_rom(
    input  wire [12:0] addr,
    output reg  [7:0]  data
);
    reg [7:0] mem [0:8191];
    initial $readmemh("src/imu660ra_config.hex", mem);
    always @(*) data = mem[addr];
endmodule
