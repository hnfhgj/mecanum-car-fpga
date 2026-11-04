//============================================================================
// dl1b.v —— 逐飞 DL1B（VL53L1X）激光测距模块驱动（I2C 400KHz）
//
// 参考逐飞库 zf_device_dl1b.c。设备 7bit 地址 0x29，16bit 大端寄存器地址。
//
// 初始化流程（对应 dl1b_init）：
//   1) XS(XSHUT) 引脚：高 50ms → 低 10ms → 高 50ms（复位）
//   2) 读 FIRMWARE__SYSTEM_STATUS(0x00E5)，bit0==1 表示 boot 完成，否则重新复位
//   3) 从 0x0001 起连续写 135 字节配置（dl1b_config_rom）
//   4) 轮询 GPIO__TIO_HV_STATUS(0x0031) 直到 bit0==0（首轮测距完成）
//
// 运行流程（对应 dl1b_get_distance）：
//   轮询 0x0031；bit0 置位(数据就绪) →
//     写 0x01 到 SYSTEM__INTERRUPT_CLEAR(0x0086)
//     读 RESULT__RANGE_STATUS(0x0089)，若==0x89 有效
//       读 FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0(0x0096) 2 字节 → 距离(mm)
//     距离超范围(>4000)判为无效，输出 8192
//
// 输出：
//   distance_mm   16bit 距离(mm)，无效时为 8192（对齐逐飞库）
//   data_valid    每读到一帧有效数据拉一拍
//   init_done     初始化完成
//============================================================================
module dl1b #(
    parameter integer CLK_FREQ = 50_000_000,
    parameter integer I2C_FREQ = 390_000,     // DL1B 最大 400KHz
    parameter integer MS_50     = 50,         // XS 复位高电平延时(ms)，仿真可改小
    parameter integer MS_10     = 10,         // XS 复位低电平延时(ms)
    parameter integer MS_1      = 1,          // 轮询间隔(ms)
    parameter integer RETRY_MS = 500,
    parameter integer BYTE_TIMEOUT_MS = 5
)(
    input  wire        clk,
    input  wire        rst_n,          // 复位，低有效

    output reg  [15:0] distance_mm,    // 距离(mm)
    output reg         data_valid,     // 一帧新数据，拉一拍
    output reg         init_done,      // 初始化完成

    // ---- I2C ----
    output wire        scl,
    inout  wire        sda,
    // ---- XS(XSHUT) 复位脚 ----
    output reg         xs,
    // ---- INT 中断脚（低有效，仅作轮询加速提示，实际靠轮询 0x0031）----
    input  wire        int_n,
    output wire [5:0] debug_state,
    output reg [15:0] retry_count
);

    // ---- 设备地址 / 寄存器 ----
    localparam [6:0]  DEV_ADDR = 7'h29;               // (0x52 >> 1)
    localparam [15:0] REG_BOOT  = 16'h00E5;           // FIRMWARE__SYSTEM_STATUS
    localparam [15:0] REG_CFG   = 16'h0001;           // 配置写入起始地址
    localparam [15:0] REG_TIO   = 16'h0031;           // GPIO__TIO_HV_STATUS
    localparam [15:0] REG_CLR   = 16'h0086;           // SYSTEM__INTERRUPT_CLEAR
    localparam [15:0] REG_RANGE = 16'h0089;           // RESULT__RANGE_STATUS
    localparam [15:0] REG_DIST  = 16'h0096;           // 距离结果(mm)

    // ---- 延时阈值 ----
    localparam [31:0] T_50MS = CLK_FREQ / 1000 * MS_50;
    localparam [31:0] T_10MS = CLK_FREQ / 1000 * MS_10;
    localparam [31:0] T_1MS  = CLK_FREQ / 1000 * MS_1;

    // ---- 状态机 ----
    localparam [5:0]
        S_XS_HI1  = 6'd0,   // XS 高 50ms
        S_XS_LO   = 6'd1,   // XS 低 10ms
        S_XS_HI2  = 6'd2,   // XS 高 50ms
        // boot 状态读（0x00E5）
        S_BOOT_W0 = 6'd3, S_BOOT_W1 = 6'd4, S_BOOT_W2 = 6'd5,
        S_BOOT_RS = 6'd6, S_BOOT_RD = 6'd7,
        // 写 135 字节配置
        S_CFG_W0  = 6'd8,  S_CFG_W1  = 6'd9,  S_CFG_W2  = 6'd10,
        S_CFG_D   = 6'd11,
        // 初始化轮询（0x0031 直到 bit0==0）
        S_POLL_W0 = 6'd12, S_POLL_W1 = 6'd13, S_POLL_W2 = 6'd14,
        S_POLL_RS = 6'd15, S_POLL_RD = 6'd16,
        // 运行：轮询间隔 + 读 0x0031
        S_RUN_WAIT= 6'd17,
        S_RPOLL_W0=6'd18, S_RPOLL_W1=6'd19, S_RPOLL_W2=6'd20,
        S_RPOLL_RS=6'd21, S_RPOLL_RD=6'd22,
        // 清中断（写 0x01 到 0x0086）
        S_CLR_W0  = 6'd23, S_CLR_W1  = 6'd24, S_CLR_W2  = 6'd25, S_CLR_D = 6'd26,
        // 读 range status（0x0089）
        S_RNG_W0  = 6'd27, S_RNG_W1  = 6'd28, S_RNG_W2  = 6'd29,
        S_RNG_RS  = 6'd30, S_RNG_RD  = 6'd31,
        // 读距离（0x0096，2 字节）
        S_DST_W0  = 6'd32, S_DST_W1  = 6'd33, S_DST_W2  = 6'd34,
        S_DST_RS  = 6'd35, S_DST_R1  = 6'd36, S_DST_R2  = 6'd37,
        S_RECOVER = 6'd38, S_RETRY = 6'd39;

    // ---- I2C 控制 ----
    reg  [7:0] i2c_tx;
    reg        i2c_start;
    reg        i2c_read;
    reg        i2c_first;
    reg        i2c_last;
    wire       i2c_busy;
    wire [7:0] i2c_rx;
    wire       i2c_rx_done;
    wire       i2c_nack;      // 写相位从机 NACK(1=无应答)

    // ---- 配置 ROM ----
    reg  [7:0] cfg_addr;    // 0..134
    wire [7:0] rom_data;

    // ---- 结果缓冲 ----
    reg  [7:0] dist_hi;

    // ---- 延时 ----
    reg  [31:0] delay_timer;

    // ---- INT 引脚两级同步 ----
    reg  [1:0]  int_sync;
    wire        int_ready;   // 1 = 中断拉低（数据就绪提示）

    reg  [5:0]  state;
    assign debug_state=state;
    reg [31:0] byte_timer;
    reg timeout_err;
    wire transaction_active=(state>=S_BOOT_W0 && state<=S_DST_R2 && state!=S_RUN_WAIT);
    // rx_done on a read-data phase includes our deliberate final NACK: do not treat that as a slave error.
    wire write_phase=!(state==S_BOOT_RD || state==S_POLL_RD || state==S_RPOLL_RD ||
        state==S_RNG_RD || state==S_DST_R1 || state==S_DST_R2);
    reg         nack_err;     // 设备地址无 ACK（传感器没接/没电/线反）
    reg         read_nack;    // 读地址无 ACK（从机没进入读模式）
    reg  [7:0]  boot_val;     // 0x00E5 原始值（调试）
    reg  [7:0]  poll_val;     // 0x0031 原始值（调试）

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)      int_sync <= 2'b11;
        else             int_sync <= {int_sync[0], int_n};
    end
    assign int_ready = ~int_sync[1];

    i2c_master #(.CLK_FREQ(CLK_FREQ), .I2C_FREQ(I2C_FREQ)) u_i2c (
        .clk(clk), .rst_n(rst_n && state!=S_RECOVER),
        .start(i2c_start), .tx_data(i2c_tx),
        .read_byte(i2c_read), .first_byte(i2c_first), .last_byte(i2c_last),
        .dev_addr(DEV_ADDR),
        .busy(i2c_busy), .rx_data(i2c_rx), .rx_done(i2c_rx_done), .nack(i2c_nack),
        .sda(sda), .scl(scl)
    );

    dl1b_config_rom u_rom (
        .addr(cfg_addr),
        .data(rom_data)
    );

    // ---- 调试：把初始化阶段编码进 distance_mm，串口可看卡在哪一步 ----
    // 8001=XS复位  8002=读boot状态  8003=写配置  8004=初始化轮询  (未init_done时)
    wire [3:0] init_stage;
    assign init_stage = (state <= S_XS_HI2) ? 4'd1 :
                        (state <= S_BOOT_RD) ? 4'd2 :
                        (state <= S_CFG_D)   ? 4'd3 :
                        (state <= S_POLL_RD) ? 4'd4 : 4'd5;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= S_XS_HI1;
            retry_count  <= 0; byte_timer <= 0; timeout_err <= 0;
            xs           <= 1'b1;
            i2c_tx       <= 8'd0;
            i2c_start    <= 1'b0;
            i2c_read     <= 1'b0;
            i2c_first    <= 1'b0;
            i2c_last     <= 1'b0;
            cfg_addr     <= 8'd0;
            dist_hi      <= 8'd0;
            delay_timer  <= 32'd0;
            distance_mm  <= 16'd8192;
            data_valid   <= 1'b0;
            init_done    <= 1'b0;
            nack_err     <= 1'b0;
            read_nack    <= 1'b0;
            boot_val     <= 8'd0;
            poll_val     <= 8'd0;
        end else begin
            i2c_start  <= 1'b0;   // 默认拉低，需要时拉一拍
            data_valid <= 1'b0;

            // 调试：未初始化完成时，distance_mm 显示当前阶段码（8001~8004）
            if (!init_done)
                distance_mm <= timeout_err ? 16'd8006 : (nack_err ? 16'd8005 : (16'd8000 + {12'd0, init_stage}));

            if(!transaction_active || i2c_rx_done || i2c_start) byte_timer<=0;
            else byte_timer<=byte_timer+1'b1;

            case (state)
                // ======================================================
                // XS 复位脉冲：高 50ms → 低 10ms → 高 50ms
                // ======================================================
                S_XS_HI1: begin
                    xs <= 1'b1;
                    delay_timer <= delay_timer + 1'b1;
                    if (delay_timer >= T_50MS) begin
                        delay_timer <= 32'd0;
                        state       <= S_XS_LO;
                    end
                end
                S_XS_LO: begin
                    xs <= 1'b0;
                    delay_timer <= delay_timer + 1'b1;
                    if (delay_timer >= T_10MS) begin
                        delay_timer <= 32'd0;
                        state       <= S_XS_HI2;
                    end
                end
                S_XS_HI2: begin
                    xs <= 1'b1;
                    delay_timer <= delay_timer + 1'b1;
                    if (delay_timer >= T_50MS) begin
                        // 复位完成，读 boot 状态：发 START+W
                        delay_timer <= 32'd0;
                        i2c_read    <= 1'b0;
                        i2c_first   <= 1'b1;
                        i2c_last    <= 1'b0;
                        i2c_start   <= 1'b1;
                        state       <= S_BOOT_W0;
                    end
                end

                // ======================================================
                // 读 boot 状态：写 0x00E5，读 1 字节
                // ======================================================
                S_BOOT_W0: if (i2c_rx_done) begin
                    if (i2c_nack) begin
                        nack_err <= 1'b1;      // 设备地址无 ACK → 传感器没响应
                    end else begin
                        i2c_tx <= REG_BOOT[15:8]; i2c_first <= 1'b0;
                        i2c_start <= 1'b1; state <= S_BOOT_W1;
                    end
                end
                S_BOOT_W1: if (i2c_rx_done) begin
                    i2c_tx <= REG_BOOT[7:0];
                    i2c_start <= 1'b1; state <= S_BOOT_W2;
                end
                S_BOOT_W2: if (i2c_rx_done) begin
                    i2c_read <= 1'b1; i2c_first <= 1'b1;
                    i2c_start <= 1'b1; state <= S_BOOT_RS;
                end
                S_BOOT_RS: if (i2c_rx_done) begin
                    if (i2c_nack) read_nack <= 1'b1;   // 读地址无 ACK
                    i2c_first <= 1'b0; i2c_last <= 1'b1;
                    i2c_start <= 1'b1; state <= S_BOOT_RD;
                end
                S_BOOT_RD: if (i2c_rx_done) begin   // 收到 boot 状态字节
                    boot_val <= i2c_rx;
                    if (i2c_rx[0]) begin            // bit0==1 → boot 完成
                        i2c_read <= 1'b0; i2c_first <= 1'b1; i2c_last <= 1'b0;
                        i2c_start <= 1'b1;
                        cfg_addr  <= 8'd0;
                        state     <= S_CFG_W0;
                    end else begin                  // 未完成 → 重新复位
                        delay_timer <= 32'd0;
                        state       <= S_XS_LO;
                    end
                end

                // ======================================================
                // 写 135 字节配置：写 0x0001 + 配置数据
                // ======================================================
                S_CFG_W0: if (i2c_rx_done) begin
                    i2c_tx <= REG_CFG[15:8]; i2c_first <= 1'b0;
                    i2c_start <= 1'b1; state <= S_CFG_W1;
                end
                S_CFG_W1: if (i2c_rx_done) begin
                    i2c_tx <= REG_CFG[7:0];
                    i2c_start <= 1'b1; state <= S_CFG_W2;
                end
                S_CFG_W2: if (i2c_rx_done) begin   // 发配置第 0 字节
                    i2c_tx <= rom_data;                 // rom_data = mem[cfg_addr=0]
                    i2c_last <= (cfg_addr == 8'd134);
                    i2c_start <= 1'b1;
                    cfg_addr  <= cfg_addr + 1'b1;
                    state     <= S_CFG_D;
                end
                S_CFG_D: if (i2c_rx_done) begin
                    if (cfg_addr < 8'd135) begin
                        i2c_tx <= rom_data;             // 发 mem[cfg_addr]
                        i2c_last <= (cfg_addr == 8'd134);
                        i2c_start <= 1'b1;
                        cfg_addr  <= cfg_addr + 1'b1;
                    end else begin                      // 135 字节发完 → 轮询就绪
                        delay_timer <= 32'd0;
                        i2c_read <= 1'b0; i2c_first <= 1'b1; i2c_last <= 1'b0;
                        i2c_start <= 1'b1;
                        state     <= S_POLL_W0;
                    end
                end

                // ======================================================
                // 初始化轮询：读 0x0031，直到 bit0==0
                // ======================================================
                S_POLL_W0: if (i2c_rx_done) begin
                    i2c_tx <= REG_TIO[15:8]; i2c_first <= 1'b0;
                    i2c_start <= 1'b1; state <= S_POLL_W1;
                end
                S_POLL_W1: if (i2c_rx_done) begin
                    i2c_tx <= REG_TIO[7:0];
                    i2c_start <= 1'b1; state <= S_POLL_W2;
                end
                S_POLL_W2: if (i2c_rx_done) begin
                    i2c_read <= 1'b1; i2c_first <= 1'b1;
                    i2c_start <= 1'b1; state <= S_POLL_RS;
                end
                S_POLL_RS: if (i2c_rx_done) begin
                    i2c_first <= 1'b0; i2c_last <= 1'b1;
                    i2c_start <= 1'b1; state <= S_POLL_RD;
                end
                S_POLL_RD: if (i2c_rx_done) begin
                    poll_val <= i2c_rx;
                    if (i2c_rx[0] == 1'b0) begin      // bit0==0 → 就绪，初始化完成
                        init_done <= 1'b1;
                        delay_timer <= 32'd0;
                        state <= S_RUN_WAIT;
                    end else begin                      // 未就绪 → 再轮询
                        i2c_read <= 1'b0; i2c_first <= 1'b1; i2c_last <= 1'b0;
                        i2c_start <= 1'b1;
                        state     <= S_POLL_W0;
                    end
                end

                // ======================================================
                // 运行：轮询 0x0031，数据就绪则读距离
                // ======================================================
                S_RUN_WAIT: begin
                    delay_timer <= delay_timer + 1'b1;
                    if (delay_timer >= T_1MS || int_ready) begin  // 1ms 或中断提示
                        delay_timer <= 32'd0;
                        i2c_read <= 1'b0; i2c_first <= 1'b1; i2c_last <= 1'b0;
                        i2c_start <= 1'b1;
                        state     <= S_RPOLL_W0;
                    end
                end
                S_RPOLL_W0: if (i2c_rx_done) begin
                    i2c_tx <= REG_TIO[15:8]; i2c_first <= 1'b0;
                    i2c_start <= 1'b1; state <= S_RPOLL_W1;
                end
                S_RPOLL_W1: if (i2c_rx_done) begin
                    i2c_tx <= REG_TIO[7:0];
                    i2c_start <= 1'b1; state <= S_RPOLL_W2;
                end
                S_RPOLL_W2: if (i2c_rx_done) begin
                    i2c_read <= 1'b1; i2c_first <= 1'b1;
                    i2c_start <= 1'b1; state <= S_RPOLL_RS;
                end
                S_RPOLL_RS: if (i2c_rx_done) begin
                    i2c_first <= 1'b0; i2c_last <= 1'b1;
                    i2c_start <= 1'b1; state <= S_RPOLL_RD;
                end
                S_RPOLL_RD: if (i2c_rx_done) begin
                    if (i2c_rx != 8'd0) begin          // 数据就绪
                        i2c_read <= 1'b0; i2c_first <= 1'b1; i2c_last <= 1'b0;
                        i2c_start <= 1'b1;
                        state     <= S_CLR_W0;
                    end else begin                      // 无数据 → 继续等
                        delay_timer <= 32'd0;
                        state       <= S_RUN_WAIT;
                    end
                end

                // ======================================================
                // 清中断：写 0x01 到 0x0086
                // ======================================================
                S_CLR_W0: if (i2c_rx_done) begin
                    i2c_tx <= REG_CLR[15:8]; i2c_first <= 1'b0;
                    i2c_start <= 1'b1; state <= S_CLR_W1;
                end
                S_CLR_W1: if (i2c_rx_done) begin
                    i2c_tx <= REG_CLR[7:0];
                    i2c_start <= 1'b1; state <= S_CLR_W2;
                end
                S_CLR_W2: if (i2c_rx_done) begin
                    i2c_tx <= 8'h01; i2c_last <= 1'b1;   // 数据 0x01，结束
                    i2c_start <= 1'b1; state <= S_CLR_D;
                end
                S_CLR_D: if (i2c_rx_done) begin
                    i2c_read <= 1'b0; i2c_first <= 1'b1; i2c_last <= 1'b0;
                    i2c_start <= 1'b1;
                    state     <= S_RNG_W0;
                end

                // ======================================================
                // 读 range status：写 0x0089，读 1 字节
                // ======================================================
                S_RNG_W0: if (i2c_rx_done) begin
                    i2c_tx <= REG_RANGE[15:8]; i2c_first <= 1'b0;
                    i2c_start <= 1'b1; state <= S_RNG_W1;
                end
                S_RNG_W1: if (i2c_rx_done) begin
                    i2c_tx <= REG_RANGE[7:0];
                    i2c_start <= 1'b1; state <= S_RNG_W2;
                end
                S_RNG_W2: if (i2c_rx_done) begin
                    i2c_read <= 1'b1; i2c_first <= 1'b1;
                    i2c_start <= 1'b1; state <= S_RNG_RS;
                end
                S_RNG_RS: if (i2c_rx_done) begin
                    i2c_first <= 1'b0; i2c_last <= 1'b1;
                    i2c_start <= 1'b1; state <= S_RNG_RD;
                end
                S_RNG_RD: if (i2c_rx_done) begin
                    if (i2c_rx == 8'h89) begin         // 有效 → 读距离
                        i2c_read <= 1'b0; i2c_first <= 1'b1; i2c_last <= 1'b0;
                        i2c_start <= 1'b1;
                        state     <= S_DST_W0;
                    end else begin                      // 无效 → 继续轮询
                        distance_mm <= 16'd8192;
                        delay_timer <= 32'd0;
                        state       <= S_RUN_WAIT;
                    end
                end

                // ======================================================
                // 读距离：写 0x0096，读 2 字节（大端）
                // ======================================================
                S_DST_W0: if (i2c_rx_done) begin
                    i2c_tx <= REG_DIST[15:8]; i2c_first <= 1'b0;
                    i2c_start <= 1'b1; state <= S_DST_W1;
                end
                S_DST_W1: if (i2c_rx_done) begin
                    i2c_tx <= REG_DIST[7:0];
                    i2c_start <= 1'b1; state <= S_DST_W2;
                end
                S_DST_W2: if (i2c_rx_done) begin
                    i2c_read <= 1'b1; i2c_first <= 1'b1;
                    i2c_start <= 1'b1; state <= S_DST_RS;
                end
                S_DST_RS: if (i2c_rx_done) begin
                    i2c_first <= 1'b0; i2c_last <= 1'b0;   // 第 1 字节
                    i2c_start <= 1'b1; state <= S_DST_R1;
                end
                S_DST_R1: if (i2c_rx_done) begin
                    dist_hi  <= i2c_rx;                     // 高字节
                    i2c_last <= 1'b1;                       // 第 2 字节(last)
                    i2c_start <= 1'b1; state <= S_DST_R2;
                end
                S_DST_R2: if (i2c_rx_done) begin
                    // 距离 = {dist_hi, i2c_rx}，超范围判无效
                    if ({dist_hi, i2c_rx} > 16'd4000)
                        distance_mm <= 16'd8192;
                    else begin
                        distance_mm <= {dist_hi, i2c_rx};
                        data_valid  <= 1'b1;
                    end
                    delay_timer <= 32'd0;
                    state       <= S_RUN_WAIT;
                end

                S_RECOVER: begin
                    xs<=0; i2c_start<=0; init_done<=0;
                    delay_timer<=delay_timer+1'b1;
                    if(delay_timer>=T_10MS) begin delay_timer<=0; state<=S_RETRY; end
                end
                S_RETRY: begin
                    xs<=0;
                    delay_timer<=delay_timer+1'b1;
                    if(delay_timer>=CLK_FREQ/1000*RETRY_MS) begin
                        delay_timer<=0; nack_err<=0; timeout_err<=0; read_nack<=0;
                        state<=S_XS_HI2;
                    end
                end
                default: state <= S_XS_HI1;
            endcase
            // Last assignments override the normal next-byte launch on any NACK/timeout.
            if(transaction_active && ((i2c_rx_done && write_phase && i2c_nack) ||
                byte_timer>=CLK_FREQ/1000*BYTE_TIMEOUT_MS)) begin
                nack_err<=i2c_rx_done && i2c_nack;
                timeout_err<=!(i2c_rx_done && i2c_nack);
                state<=S_RECOVER; delay_timer<=0; i2c_start<=0; init_done<=0; data_valid<=0;
                if(retry_count!=16'hffff) retry_count<=retry_count+1'b1;
            end
        end
    end

endmodule
