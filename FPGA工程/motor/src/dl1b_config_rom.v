//============================================================================
// dl1b_config_rom.v -- 逐飞 DL1B (VL53L1X) 135-byte 配置 ROM
//
// 内容与逐飞库 zf_device_config.a/.lib 里 dl1b_config_file[135] 完全一致，
// 上电初始化时从 I2C 寄存器 0x0001 开始连续写入 135 字节。
//============================================================================
module dl1b_config_rom(
    input  wire [7:0] addr,
    output reg  [7:0] data
);
    reg [7:0] mem [0:134];
    initial $readmemh("src/dl1b_config.hex", mem);
    always @(*) data = mem[addr];
endmodule
