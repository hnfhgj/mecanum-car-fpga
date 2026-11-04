//============================================================================
// imu660ra.v —— 逐飞 IMU660RA（内部 Bosch BMI270）6 轴 IMU 控制器
//
// SPI 模式 0，默认 1MHz，MSB 先发。参考逐飞库 zf_device_imu660ra.c：
//   - 上电后按初始化序列写寄存器 + 下载 8192 字节固件配置文件
//   - 初始化完成后周期读取加速度计(0x0C)和陀螺仪(0x12)，各 6 字节
//
// SPI 读写协议（BMI270）：
//   写寄存器：  [addr|0x00][data]
//   读寄存器：  [addr|0x80][dummy 1 字节][data N 字节]  —— 地址后第一字节是哑字节
//
// 输出：
//   acc_x/y/z, gyro_x/y/z  原始 16bit（LSB，小端拼接）
//   data_valid             每次读完 6 轴数据拉一拍
//   init_done              初始化完成
//============================================================================
module imu660ra #(
    parameter integer CLK_FREQ = 50_000_000,   // 系统时钟
    parameter integer SPI_FREQ = 1_000_000     // SPI 时钟（BMI270 固件下载需低速，1MHz 稳定）
)(
    input  wire        clk,
    input  wire        rst_n,          // 复位，低有效

    // ---- 输出：6 轴原始数据 ----
    output reg  [15:0] acc_x, acc_y, acc_z,     // 加速度计（LSB）
    output reg  [15:0] gyro_x, gyro_y, gyro_z,  // 陀螺仪（LSB）
    output reg         data_valid,              // 一次 6 轴读完整，拉一拍
    output reg         init_done,               // 初始化完成

    // ---- SPI 引脚 ----
    output wire        spi_sck,
    output wire        spi_mosi,
    output wire        spi_cs_n,
    input  wire        spi_miso
);

    // ---- 寄存器地址（BMI270）----
    localparam [7:0] A_CHIP_ID   = 8'h00;
    localparam [7:0] A_ACC       = 8'h0C;   // 加速度计数据起始
    localparam [7:0] A_GYRO      = 8'h12;   // 陀螺仪数据起始
    localparam [7:0] A_INT_STA   = 8'h21;
    localparam [7:0] A_ACC_CONF  = 8'h40;
    localparam [7:0] A_ACC_RNG   = 8'h41;
    localparam [7:0] A_GYR_CONF  = 8'h42;
    localparam [7:0] A_GYR_RNG   = 8'h43;
    localparam [7:0] A_INIT_CTRL = 8'h59;
    localparam [7:0] A_INIT_DATA = 8'h5E;
    localparam [7:0] A_PWR_CONF  = 8'h7C;
    localparam [7:0] A_PWR_CTRL  = 8'h7D;

    // ---- 状态机状态 ----
    localparam [5:0]
        S_PWRUP       = 6'd0,   // 上电等待 20ms
        S_RDID_A      = 6'd1,   // 读 CHIP_ID（自检）地址
        S_RDID_D      = 6'd2,   // 读 CHIP_ID 数据（哑+值）
        S_WPWRCONF_A  = 6'd3,
        S_WPWRCONF_D  = 6'd4,
        S_WAIT1       = 6'd5,   // 等 1ms
        S_WICTL0_A    = 6'd6,
        S_WICTL0_D    = 6'd7,
        S_DLCONF_A    = 6'd8,   // 发 INIT_DATA 地址
        S_DLCONF_D    = 6'd9,   // 逐字节下载 8192 字节
        S_WICTL1_A    = 6'd10,
        S_WICTL1_D    = 6'd11,
        S_WAIT20      = 6'd12,  // 等 20ms
        S_RDINT_A     = 6'd13,
        S_RDINT_D     = 6'd14,
        S_WPWRCTRL_A  = 6'd15,
        S_WPWRCTRL_D  = 6'd16,
        S_WACCCONF_A  = 6'd17,
        S_WACCCONF_D  = 6'd18,
        S_WGYRCONF_A  = 6'd19,
        S_WGYRCONF_D  = 6'd20,
        S_WACCRNG_A   = 6'd21,
        S_WACCRNG_D   = 6'd22,
        S_WGYRRNG_A   = 6'd23,
        S_WGYRRNG_D   = 6'd24,
        S_RUN_ACC_A   = 6'd25,  // 周期读加速度计地址
        S_RUN_ACC_D   = 6'd26,  // 收 6 字节加速度
        S_RUN_GYRO_A  = 6'd27,  // 周期读陀螺仪地址
        S_RUN_GYRO_D  = 6'd28,  // 收 6 字节陀螺仪
        S_WAIT2       = 6'd29,  // 等 1ms（INIT_CTRL=0 后、下载固件前）
        S_WAITWR      = 6'd30,  // 寄存器写之间的小延时（保证 CS 高电平时间）
        S_RDBACK_A    = 6'd31,  // 读回 ACC_RANGE（验证写是否生效）
        S_RDBACK_D    = 6'd32,  // 读回 ACC_RANGE 数据
        S_SAMPLE_WAIT = 6'd33;  // 200Hz sampling; allow sensor registers to update

    // ---- SPI 控制 ----
    reg  [7:0] tx_byte;
    reg        spi_start;
    reg        spi_last;
    wire       spi_busy;
    wire       spi_rx_done;
    wire [7:0] spi_rx_data;

    // ---- 固件 ROM ----
    reg  [13:0] rom_addr;   // 14 位，0..8191 为 ROM 地址，8192 表示发完
    wire [7:0]  rom_data;

    // ---- 字节计数 / 接收缓冲 ----
    reg  [3:0]  byte_cnt;
    reg  [7:0]  rx_buf [0:6];

    // ---- 延时 ----
    reg  [31:0] delay_timer;

    reg  [5:0]  state;
    reg  [5:0]  next_state;   // S_WAITWR 延时后要去的状态

    // ==================================================================
    spi_master #(.CLK_FREQ(CLK_FREQ), .SPI_FREQ(SPI_FREQ)) u_spi (
        .clk(clk), .rst_n(rst_n),
        .start(spi_start), .tx_data(tx_byte), .last(spi_last),
        .busy(spi_busy), .rx_data(spi_rx_data), .rx_done(spi_rx_done),
        .sck(spi_sck), .mosi(spi_mosi), .miso(spi_miso), .cs_n(spi_cs_n)
    );

    imu660ra_config_rom u_rom (
        .addr(rom_addr[12:0]),
        .data(rom_data)
    );

    // ==================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= S_PWRUP;
            next_state  <= S_PWRUP;
            tx_byte     <= 8'd0;
            spi_start   <= 1'b0;
            spi_last    <= 1'b0;
            rom_addr    <= 14'd0;
            byte_cnt    <= 4'd0;
            delay_timer <= 32'd0;
            init_done   <= 1'b0;
            data_valid  <= 1'b0;
            acc_x <= 16'd0; acc_y <= 16'd0; acc_z <= 16'd0;
            gyro_x <= 16'd0; gyro_y <= 16'd0; gyro_z <= 16'd0;
        end else begin
            spi_start  <= 1'b0;   // 默认拉低，需要时拉一拍
            data_valid <= 1'b0;

            case (state)
                // ---------------- 上电等待 20ms ----------------
                S_PWRUP: begin
                    delay_timer <= delay_timer + 1'b1;
                    if (delay_timer >= CLK_FREQ/1000*20) begin
                        state     <= S_RDID_A;
                        tx_byte   <= A_CHIP_ID | 8'h80;  // 读
                        spi_start <= 1'b1;
                        spi_last  <= 1'b0;
                        byte_cnt  <= 4'd0;
                    end
                end

                // ---------------- 读 CHIP_ID（自检） ----------------
                // 读 1 个寄存器 = 地址 + 2 个哑字节，值在第 2 个哑字节
                S_RDID_A: begin
                    if (spi_rx_done) begin
                        // 地址已发，发第 1 个哑字节
                        tx_byte   <= 8'h00;
                        spi_start <= 1'b1;
                        spi_last  <= 1'b0;
                        byte_cnt  <= 4'd0;
                        state     <= S_RDID_D;
                    end
                end
                S_RDID_D: begin
                    if (spi_rx_done) begin
                        if (byte_cnt == 4'd0) begin
                            // 第 1 个哑字节（丢弃），发第 2 个哑字节
                            byte_cnt  <= 4'd1;
                            tx_byte   <= 8'h00;
                            spi_start <= 1'b1;
                            spi_last  <= 1'b1;
                        end else begin
                            // 第 2 个哑字节收到的才是真正的 CHIP_ID
                            if (spi_rx_data == 8'h24) begin
                                state     <= S_WPWRCONF_A;
                                tx_byte   <= A_PWR_CONF;
                                spi_start <= 1'b1;
                                spi_last  <= 1'b0;
                            end else begin
                                state     <= S_RDID_A;
                                tx_byte   <= A_CHIP_ID | 8'h80;
                                spi_start <= 1'b1;
                                spi_last  <= 1'b0;
                            end
                        end
                    end
                end

                // ---------------- 写 PWR_CONF = 0x00 ----------------
                S_WPWRCONF_A: begin
                    if (spi_rx_done) begin
                        tx_byte   <= 8'h00;
                        spi_start <= 1'b1;
                        spi_last  <= 1'b1;
                        state     <= S_WPWRCONF_D;
                    end
                end
                S_WPWRCONF_D: begin
                    if (spi_rx_done) begin
                        delay_timer <= 32'd0;
                        state       <= S_WAIT1;
                    end
                end

                // ---------------- 等 1ms ----------------
                S_WAIT1: begin
                    delay_timer <= delay_timer + 1'b1;
                    if (delay_timer >= CLK_FREQ/1000) begin
                        state     <= S_WICTL0_A;
                        tx_byte   <= A_INIT_CTRL;
                        spi_start <= 1'b1;
                        spi_last  <= 1'b0;
                    end
                end

                // ---------------- 写 INIT_CTRL = 0x00 ----------------
                S_WICTL0_A: begin
                    if (spi_rx_done) begin
                        tx_byte   <= 8'h00;
                        spi_start <= 1'b1;
                        spi_last  <= 1'b1;   // 结束本笔写
                        state     <= S_WICTL0_D;
                    end
                end
                S_WICTL0_D: begin
                    if (spi_rx_done) begin
                        // 等 1ms 再下载固件（让传感器准备好接收配置）
                        delay_timer <= 32'd0;
                        state       <= S_WAIT2;
                    end
                end

                // ---------------- 等 1ms（下载固件前） ----------------
                S_WAIT2: begin
                    delay_timer <= delay_timer + 1'b1;
                    if (delay_timer >= CLK_FREQ/1000) begin
                        // 开始下载固件：发 INIT_DATA 地址
                        state     <= S_DLCONF_A;
                        tx_byte   <= A_INIT_DATA;
                        spi_start <= 1'b1;
                        spi_last  <= 1'b0;
                        rom_addr  <= 14'd0;
                    end
                end

                // ---------------- 寄存器写之间的小延时（保证 CS 高电平时间） ----------------
                S_WAITWR: begin
                    delay_timer <= delay_timer + 1'b1;
                    if (delay_timer >= CLK_FREQ/100000) begin   // 10us
                        state     <= next_state;
                        spi_start <= 1'b1;
                        spi_last  <= 1'b0;
                    end
                end

                // ---------------- 下载 8192 字节固件 ----------------
                S_DLCONF_A: begin
                    if (spi_rx_done) begin
                        // 地址已发，开始逐字节发固件
                        tx_byte   <= rom_data;
                        spi_last  <= (rom_addr == 14'd8191);   // 最后一字节才拉高 CS
                        spi_start <= 1'b1;
                        rom_addr  <= rom_addr + 1'b1;
                        state     <= S_DLCONF_D;
                    end
                end
                S_DLCONF_D: begin
                    if (spi_rx_done) begin
                        if (rom_addr < 14'd8192) begin
                            // 继续发下一字节
                            tx_byte   <= rom_data;
                            spi_last  <= (rom_addr == 14'd8191);
                            spi_start <= 1'b1;
                            rom_addr  <= rom_addr + 1'b1;
                        end else begin
                            // 8192 字节发完 → 延时后写 INIT_CTRL=0x01
                            delay_timer <= 32'd0;
                            next_state  <= S_WICTL1_A;
                            tx_byte     <= A_INIT_CTRL;
                            state       <= S_WAITWR;
                        end
                    end
                end

                // ---------------- 写 INIT_CTRL = 0x01 ----------------
                S_WICTL1_A: begin
                    if (spi_rx_done) begin
                        tx_byte   <= 8'h01;
                        spi_start <= 1'b1;
                        spi_last  <= 1'b1;
                        state     <= S_WICTL1_D;
                    end
                end
                S_WICTL1_D: begin
                    if (spi_rx_done) begin
                        delay_timer <= 32'd0;
                        state       <= S_WAIT20;
                    end
                end

                // ---------------- 等 20ms ----------------
                S_WAIT20: begin
                    delay_timer <= delay_timer + 1'b1;
                    if (delay_timer >= CLK_FREQ/1000*20) begin
                        state     <= S_RDINT_A;
                        tx_byte   <= A_INT_STA | 8'h80;
                        spi_start <= 1'b1;
                        spi_last  <= 1'b0;
                    end
                end

                // ---------------- 读 INTERNAL_STATUS，低4位=1 表示配置成功 ----------------
                S_RDINT_A: begin
                    if (spi_rx_done) begin
                        tx_byte   <= 8'h00;
                        spi_start <= 1'b1;
                        spi_last  <= 1'b0;
                        byte_cnt  <= 4'd0;
                        state     <= S_RDINT_D;
                    end
                end
                S_RDINT_D: begin
                    if (spi_rx_done) begin
                        if (byte_cnt == 4'd0) begin
                            byte_cnt  <= 4'd1;
                            tx_byte   <= 8'h00;
                            spi_start <= 1'b1;
                            spi_last  <= 1'b1;
                        end else if (spi_rx_data[3:0] == 4'h1) begin
                            // 配置成功，延时后写 PWR_CTRL
                            delay_timer <= 32'd0;
                            next_state  <= S_WPWRCTRL_A;
                            tx_byte     <= A_PWR_CTRL;
                            state       <= S_WAITWR;
                        end else begin
                            // 下载失败时重新初始化，不能把全零数据报告为就绪。
                            delay_timer <= 32'd0;
                            state <= S_PWRUP;
                        end
                    end
                end

                // ---------------- 写 PWR_CTRL = 0x0E ----------------
                S_WPWRCTRL_A: begin
                    if (spi_rx_done) begin
                        tx_byte   <= 8'h0E;
                        spi_start <= 1'b1;
                        spi_last  <= 1'b1;
                        state     <= S_WPWRCTRL_D;
                    end
                end
                S_WPWRCTRL_D: begin
                    if (spi_rx_done) begin
                        delay_timer <= 32'd0;
                        next_state  <= S_WACCCONF_A;
                        tx_byte     <= A_ACC_CONF;
                        state       <= S_WAITWR;
                    end
                end

                // ---------------- 写 ACC_CONF = 0xA7 ----------------
                S_WACCCONF_A: begin
                    if (spi_rx_done) begin
                        tx_byte   <= 8'hA7;
                        spi_start <= 1'b1;
                        spi_last  <= 1'b1;
                        state     <= S_WACCCONF_D;
                    end
                end
                S_WACCCONF_D: begin
                    if (spi_rx_done) begin
                        delay_timer <= 32'd0;
                        next_state  <= S_WGYRCONF_A;
                        tx_byte     <= A_GYR_CONF;
                        state       <= S_WAITWR;
                    end
                end

                // ---------------- 写 GYR_CONF = 0xA9 ----------------
                S_WGYRCONF_A: begin
                    if (spi_rx_done) begin
                        tx_byte   <= 8'hA9;
                        spi_start <= 1'b1;
                        spi_last  <= 1'b1;
                        state     <= S_WGYRCONF_D;
                    end
                end
                S_WGYRCONF_D: begin
                    if (spi_rx_done) begin
                        delay_timer <= 32'd0;
                        next_state  <= S_WACCRNG_A;
                        tx_byte     <= A_ACC_RNG;
                        state       <= S_WAITWR;
                    end
                end

                // ---------------- 写 ACC_RANGE = 0x02 ----------------
                S_WACCRNG_A: begin
                    if (spi_rx_done) begin
                        tx_byte   <= 8'h02;
                        spi_start <= 1'b1;
                        spi_last  <= 1'b1;
                        state     <= S_WACCRNG_D;
                    end
                end
                S_WACCRNG_D: begin
                    if (spi_rx_done) begin
                        delay_timer <= 32'd0;
                        next_state  <= S_WGYRRNG_A;
                        tx_byte     <= A_GYR_RNG;
                        state       <= S_WAITWR;
                    end
                end

                // ---------------- 写 GYR_RANGE = 0x00 ----------------
                S_WGYRRNG_A: begin
                    if (spi_rx_done) begin
                        tx_byte   <= 8'h00;
                        spi_start <= 1'b1;
                        spi_last  <= 1'b1;
                        state     <= S_WGYRRNG_D;
                    end
                end
                S_WGYRRNG_D: begin
                    if (spi_rx_done) begin
                        // 延时后读回 ACC_RANGE
                        delay_timer <= 32'd0;
                        next_state  <= S_RDBACK_A;
                        tx_byte     <= A_ACC_RNG | 8'h80;
                        state       <= S_WAITWR;
                        byte_cnt    <= 4'd0;
                    end
                end

                // ---------------- 读回 ACC_RANGE ----------------
                S_RDBACK_A: begin
                    if (spi_rx_done) begin
                        tx_byte   <= 8'h00;
                        spi_start <= 1'b1;
                        spi_last  <= 1'b0;
                        byte_cnt  <= 4'd0;
                        state     <= S_RDBACK_D;
                    end
                end
                S_RDBACK_D: begin
                    if (spi_rx_done) begin
                        if (byte_cnt == 4'd0) begin
                            byte_cnt  <= 4'd1;
                            tx_byte   <= 8'h00;
                            spi_start <= 1'b1;
                            spi_last  <= 1'b1;
                        end else if (spi_rx_data == 8'h02) begin
                            init_done <= 1'b1;
                            // 量程读回正确后进入运行态，先读加速度计
                            delay_timer <= 0;
                            next_state <= S_RUN_ACC_A;
                            state     <= S_WAITWR;
                            tx_byte   <= A_ACC | 8'h80;
                            spi_start <= 1'b0;
                            spi_last  <= 1'b0;
                            byte_cnt  <= 4'd0;
                        end else begin
                            delay_timer <= 32'd0;
                            state <= S_PWRUP;
                        end
                    end
                end

                // ---------------- 运行：读加速度计（地址+1哑+6数据） ----------------
                S_RUN_ACC_A: begin
                    if (spi_rx_done) begin
                        // 地址已发，发哑字节
                        tx_byte   <= 8'h00;
                        spi_start <= 1'b1;
                        spi_last  <= 1'b0;
                        byte_cnt  <= 4'd0;
                        state     <= S_RUN_ACC_D;
                    end
                end
                S_RUN_ACC_D: begin
                    if (spi_rx_done) begin
                        rx_buf[byte_cnt] <= spi_rx_data;
                        if (byte_cnt == 4'd6) begin
                            // 7 个哑字节收完；最后一个字节 spi_rx_data 即 rx_buf[6]
                            acc_x <= {rx_buf[2], rx_buf[1]};
                            acc_y <= {rx_buf[4], rx_buf[3]};
                            acc_z <= {spi_rx_data, rx_buf[5]};
                            delay_timer <= 0;
                            next_state <= S_RUN_GYRO_A;
                            state     <= S_WAITWR;
                            tx_byte   <= A_GYRO | 8'h80;
                            spi_start <= 1'b0;
                            spi_last  <= 1'b0;
                            byte_cnt  <= 4'd0;
                        end else begin
                            byte_cnt  <= byte_cnt + 1'b1;
                            tx_byte   <= 8'h00;
                            spi_start <= 1'b1;
                            spi_last  <= (byte_cnt == 4'd5);   // 最后一个哑字节拉高 CS
                        end
                    end
                end

                // ---------------- 运行：读陀螺仪（地址+1哑+6数据） ----------------
                S_RUN_GYRO_A: begin
                    if (spi_rx_done) begin
                        tx_byte   <= 8'h00;
                        spi_start <= 1'b1;
                        spi_last  <= 1'b0;
                        byte_cnt  <= 4'd0;
                        state     <= S_RUN_GYRO_D;
                    end
                end
                S_RUN_GYRO_D: begin
                    if (spi_rx_done) begin
                        rx_buf[byte_cnt] <= spi_rx_data;
                        if (byte_cnt == 4'd6) begin
                            gyro_x <= {rx_buf[2], rx_buf[1]};
                            gyro_y <= {rx_buf[4], rx_buf[3]};
                            gyro_z <= {spi_rx_data, rx_buf[5]};
                            data_valid <= 1'b1;
                            // Leave CS high between samples, rather than continuous polling.
                            delay_timer <= 0;
                            state     <= S_SAMPLE_WAIT;
                            tx_byte   <= A_ACC | 8'h80;
                            spi_start <= 1'b0;
                            spi_last  <= 1'b0;
                            byte_cnt  <= 4'd0;
                        end else begin
                            byte_cnt  <= byte_cnt + 1'b1;
                            tx_byte   <= 8'h00;
                            spi_start <= 1'b1;
                            spi_last  <= (byte_cnt == 4'd5);   // 最后一个哑字节拉高 CS
                        end
                    end
                end

                S_SAMPLE_WAIT: begin
                    delay_timer <= delay_timer + 1'b1;
                    if (delay_timer >= CLK_FREQ/200-1) begin
                        state <= S_RUN_ACC_A;
                        spi_start <= 1'b1;
                        spi_last <= 1'b0;
                    end
                end

                default: state <= S_PWRUP;
            endcase
        end
    end

endmodule
