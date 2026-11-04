//============================================================================
// i2c_master.v —— I2C 主机（400KHz），字节级接口，支持写/读/重复 START
//
// 每次 start 拉一拍，完成一个「字节相位」。字节相位结束后 rx_done 拉一拍。
// 一个完整事务由多个字节相位拼接，由上层状态机控制：
//
//   写字节： read_byte=0，first=1 发 START+设备写地址，last=1 发完后发 STOP
//   读字节： read_byte=1，first=1 发(重复)START+设备读地址，非 first 收数据
//
// 典型「写寄存器地址 + 读 N 字节」：
//   byte0: read=0 first=1 last=0   tx={dev,0}   → START + 写地址
//   byte1: read=0 first=0 last=0   tx=reg_hi
//   byte2: read=0 first=0 last=0   tx=reg_lo
//   byte3: read=1 first=1 last=0   tx=忽略       → 重复 START + 读地址
//   byte4..: read=1 first=0        tx=忽略       收数据，最后 last=1 → NACK+STOP
//
// 设备地址 dev_addr 为 7bit（如 DL1B 为 0x29），主机内部自动左移拼 R/W 位。
//
// 引脚：SCL 推挽输出（单主机）；SDA 开漏模拟（拉低=0，三态=释放为 1），
// 由外部/内部上拉电阻提供高电平，从机 ACK 与读数据时拉低 SDA。
//
// 时序要点：
//   - 采样点放在 SCL 高电平「正中间」（cnt==HALF+HALF/2），而不是末尾，
//     给从机足够时间驱动 SDA 并稳定，避免采到 SCL 下降沿附近的不稳定电平。
//   - 重复 START 前加了 SCL 低建立时间（tSU;STA），否则从机会漏掉重复 START。
//============================================================================
module i2c_master #(
    parameter integer CLK_FREQ = 50_000_000,   // 系统时钟
    parameter integer I2C_FREQ = 390_000       // I2C 时钟（DL1B 最大 400KHz，留裕量）
)(
    input  wire       clk,
    input  wire       rst_n,          // 复位，低有效

    input  wire       start,          // 拉一拍：开始一个字节相位
    input  wire [7:0] tx_data,        // 要发送的字节（读相位时忽略）
    input  wire       read_byte,      // 1=读相位（收数据） 0=写相位（发数据）
    input  wire       first_byte,     // 1=本相位先发 START（首字节/重复 START）
    input  wire       last_byte,      // 1=本相位结束发 STOP（事务最后一个字节）
    input  wire [6:0] dev_addr,       // 7bit 设备地址

    output reg        busy,           // 正在收发中
    output reg  [7:0] rx_data,        // 读相位收到的字节
    output reg        rx_done,        // 拉一拍：一个字节相位完成
    output reg        nack,           // 写相位时：1=从机 NACK，0=ACK
    // ---- I2C 引脚 ----
    inout  wire       sda,
    output wire       scl
);

    // 半个 SCL 周期所需的系统时钟数
    localparam [15:0] HALF = CLK_FREQ / I2C_FREQ / 2;
    // 采样点：SCL 高电平的正中间（SCL 在 cnt==HALF 时拉高，采样放在 HALF+HALF/2）
    localparam [15:0] SAMPLE = HALF + HALF / 2;

    // ---- 状态 ----
    localparam [2:0]
        S_IDLE  = 3'd0,   // 等待 start
        S_START = 3'd1,   // START / 重复 START 条件
        S_BIT   = 3'd2,   // 8 个数据位
        S_ACK   = 3'd3,   // ACK/NACK 位
        S_STOP  = 3'd4,   // STOP 条件
        S_DONE  = 3'd5;   // 字节相位完成，拉 rx_done

    reg  [2:0]  state;
    reg  [15:0] cnt;          // 位内周期计数器（每 bit 2*HALF 个周期）
    reg  [3:0]  bit_idx;      // 0..7 当前数据位
    reg  [7:0]  sh;           // 移位寄存器（发：存 tx，收：移入）
    reg         rw;           // 读相位标志（锁存）
    reg         first_l;      // first_byte 锁存
    reg         last_l;       // last_byte 锁存
    reg  [6:0]  dev_l;        // dev_addr 锁存
    reg         scl_r;        // SCL 输出寄存器
    reg         sda_oe;       // 1=把 SDA 拉低，0=释放（三态）

    assign scl = scl_r;
    assign sda = sda_oe ? 1'b0 : 1'bz;   // 只拉低，不主动拉高（开漏模拟）
    wire sda_in = sda;

    // 本字节相位是「发」还是「收」：first 必发地址字节；非 first 且 read 才收
    wire send_mode = first_l | ~rw;
    // 发相位要发送的字节：first 发 {dev,rw}，否则发锁存的 tx_data
    wire [7:0] send_byte = first_l ? {dev_l, rw} : sh;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state   <= S_IDLE;
            cnt     <= 16'd0;
            bit_idx <= 4'd0;
            sh      <= 8'd0;
            rw      <= 1'b0;
            first_l <= 1'b0;
            last_l  <= 1'b0;
            dev_l   <= 7'd0;
            scl_r   <= 1'b1;      // 复位后总线空闲：SCL 高
            sda_oe  <= 1'b0;      // 释放 SDA
            busy    <= 1'b0;
            rx_data <= 8'd0;
            rx_done <= 1'b0;
            nack    <= 1'b0;
        end else begin
            rx_done <= 1'b0;

            case (state)
                // ----------------------------------------------------------
                S_IDLE: begin
                    if (start) begin
                        rw      <= read_byte;
                        first_l <= first_byte;
                        last_l  <= last_byte;
                        dev_l   <= dev_addr;
                        sh      <= tx_data;
                        bit_idx <= 4'd0;
                        cnt     <= 16'd0;
                        busy    <= 1'b1;
                        nack    <= 1'b0;
                        state   <= first_byte ? S_START : S_BIT;
                    end
                end

                // ----------------------------------------------------------
                // START / 重复 START：
                //   阶段1 SCL低+SDA高(建立 tSU;STA) → 阶段2 SCL高+SDA高
                //   → 阶段3 SDA低(START 沿) → 阶段4 SCL低 准备发数据
                // ----------------------------------------------------------
                S_START: begin
                    cnt <= cnt + 1'b1;
                    if (cnt < HALF) begin
                        scl_r  <= 1'b0;
                        sda_oe <= 1'b0;      // 阶段1：SCL 低 + SDA 高（建立时间）
                    end else if (cnt < 2*HALF) begin
                        scl_r  <= 1'b1;
                        sda_oe <= 1'b0;      // 阶段2：SCL 高 + SDA 高
                    end else if (cnt < 3*HALF) begin
                        scl_r  <= 1'b1;
                        sda_oe <= 1'b1;      // 阶段3：拉低 SDA → START 沿
                    end else begin
                        cnt     <= 16'd0;
                        scl_r   <= 1'b0;      // 阶段4：SCL 低，准备发数据
                        sda_oe  <= 1'b1;
                        state   <= S_BIT;
                    end
                end

                // ----------------------------------------------------------
                // 8 个数据位（每 bit 两个半周期：SCL 低换数据 → SCL 高中间采样）
                // ----------------------------------------------------------
                S_BIT: begin
                    cnt <= cnt + 1'b1;
                    if (cnt < HALF) begin
                        // SCL 低半周：写相位放数据位，读相位释放 SDA
                        scl_r <= 1'b0;
                        if (send_mode)
                            sda_oe <= ~send_byte[7 - bit_idx];
                        else
                            sda_oe <= 1'b0;   // 读相位：释放 SDA 让从机驱动
                    end else begin
                        // SCL 高半周
                        scl_r <= 1'b1;
                        if (cnt == SAMPLE) begin
                            // 高电平正中间采样一次
                            if (!send_mode)
                                sh <= {sh[6:0], sda_in};   // 收 1 位（先 MSB）
                        end
                        if (cnt == 2*HALF - 1) begin
                            cnt <= 16'd0;
                            if (bit_idx == 4'd7) begin
                                bit_idx <= 4'd0;
                                state   <= S_ACK;
                            end else
                                bit_idx <= bit_idx + 1'b1;
                        end
                    end
                end

                // ----------------------------------------------------------
                // ACK 位：写相位收从机 ACK；读相位发 ACK(0)/NACK(1)
                // ----------------------------------------------------------
                S_ACK: begin
                    cnt <= cnt + 1'b1;
                    if (cnt < HALF) begin
                        scl_r <= 1'b0;
                        if (send_mode)
                            sda_oe <= 1'b0;   // 写相位：释放 SDA 让从机回 ACK
                        else
                            // 读相位：最后字节发 NACK(释放)，否则发 ACK(拉低)
                            sda_oe <= last_l ? 1'b0 : 1'b1;
                    end else begin
                        scl_r <= 1'b1;
                        if (cnt == SAMPLE) begin
                            if (send_mode)
                                nack <= sda_in;   // 采样从机 ACK（0=ACK,1=NACK）
                        end
                        if (cnt == 2*HALF - 1) begin
                            cnt   <= 16'd0;
                            state <= last_l ? S_STOP : S_DONE;
                        end
                    end
                end

                // ----------------------------------------------------------
                // STOP：SCL 低时 SDA 低 → SCL 高 → SDA 高（SCL 高时上升沿）
                // ----------------------------------------------------------
                S_STOP: begin
                    cnt <= cnt + 1'b1;
                    if (cnt < HALF) begin
                        scl_r  <= 1'b0;
                        sda_oe <= 1'b1;      // SDA 低
                    end else if (cnt < 2*HALF) begin
                        scl_r  <= 1'b1;      // SCL 高，SDA 保持低
                        sda_oe <= 1'b1;
                    end else if (cnt < 3*HALF) begin
                        scl_r  <= 1'b1;
                        sda_oe <= 1'b0;      // 释放 SDA → STOP 沿
                    end else begin
                        cnt   <= 16'd0;
                        state <= S_DONE;
                    end
                end

                // ----------------------------------------------------------
                // 字节相位完成
                // ----------------------------------------------------------
                S_DONE: begin
                    rx_data <= send_mode ? 8'd0 : sh;   // 读相位：收满 8 位
                    rx_done <= 1'b1;
                    busy    <= 1'b0;
                    scl_r   <= last_l ? 1'b1 : 1'b0;   // STOP 后释放 SCL，否则保持低
                    sda_oe  <= 1'b0;                    // 释放 SDA
                    state   <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
