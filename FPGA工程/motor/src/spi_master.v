//============================================================================
// spi_master.v —— SPI 主机（模式 0：CPOL=0, CPHA=0），8bit，MSB 先发
//
// 每次 start 拉一拍，发送一个字节（tx_data），同时从 MISO 采样一个字节到 rx_data，
// 完成后 rx_done 拉一拍。last=1 表示这是本次事务的最后一个字节，发完自动把 CS 拉高。
//
// 时序（模式 0）：SCK 空闲低；MOSI 在 SCK 下降沿更新、上升沿稳定；MISO 在 SCK 上升沿采样。
// 与逐飞库 zf_driver_spi.c 的 SPI_MODE0 一致。
//============================================================================
module spi_master #(
    parameter integer CLK_FREQ = 50_000_000,   // 系统时钟
    parameter integer SPI_FREQ = 10_000_000    // SPI 时钟（IMU660RA 用 10MHz）
)(
    input  wire       clk,
    input  wire       rst_n,       // 复位，低有效
    input  wire       start,       // 拉一拍：开始发一个字节
    input  wire [7:0] tx_data,     // 要发送的字节
    input  wire       last,        // 1 = 发完这个字节后把 CS 拉高（事务结束）
    output reg        busy,        // 正在收发中
    output reg  [7:0] rx_data,     // 收到的字节
    output reg        rx_done,     // 拉一拍：一个字节收发完成
    // ---- SPI 引脚 ----
    output reg        sck,
    output reg        mosi,
    input  wire       miso,
    output reg        cs_n         // 片选，低有效
);

    // 半个 SCK 周期所需的系统时钟数
    localparam [15:0] HALF = CLK_FREQ / SPI_FREQ / 2;

    reg [15:0] cnt;
    reg [2:0]  bit_idx;    // 当前发送到第几位 0..7
    reg [7:0]  sh_out;     // 发送移位寄存器
    reg [7:0]  sh_in;      // 接收移位寄存器
    reg        last_latch;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sck      <= 1'b0;
            mosi     <= 1'b0;
            cs_n     <= 1'b1;
            busy     <= 1'b0;
            rx_done  <= 1'b0;
            rx_data  <= 8'd0;
            cnt      <= 16'd0;
            bit_idx  <= 3'd0;
            sh_out   <= 8'd0;
            sh_in    <= 8'd0;
            last_latch <= 1'b0;
        end else begin
            rx_done <= 1'b0;
            if (busy) begin
                if (cnt >= HALF - 1) begin
                    cnt <= 16'd0;
                    sck <= ~sck;                     // 翻转 SCK
                    if (sck == 1'b0) begin
                        // SCK 上升沿：采样 MISO（移位进 sh_in 最高位）
                        sh_in <= {sh_in[6:0], miso};
                    end else begin
                        // SCK 下降沿：更新 MOSI 到下一位
                        if (bit_idx == 3'd7) begin
                            busy   <= 1'b0;          // 8 位发完
                            rx_done <= 1'b1;
                            rx_data <= sh_in;        // 已采满 8 位（MSB 在前）
                            if (last_latch)
                                cs_n <= 1'b1;
                        end else begin
                            bit_idx <= bit_idx + 1'b1;
                            mosi    <= sh_out[6 - bit_idx];
                        end
                    end
                end else begin
                    cnt <= cnt + 1'b1;
                end
            end else if (start) begin
                busy       <= 1'b1;
                sck        <= 1'b0;
                cnt        <= 16'd0;
                bit_idx    <= 3'd0;
                sh_out     <= tx_data;
                sh_in      <= 8'd0;
                mosi       <= tx_data[7];   // 先放最高位
                cs_n       <= 1'b0;         // 拉低片选
                last_latch <= last;
            end
        end
    end

endmodule
