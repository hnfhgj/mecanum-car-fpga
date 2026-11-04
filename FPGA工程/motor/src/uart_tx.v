//============================================================================
// uart_tx.v —— 串口发送模块（8N1，LSB 先发）
//
// 接口：data + send（1 拍脉冲）→ 串口按波特率逐位发出；busy 高表示正在发送。
// 用法：等 busy=0 时给 send 拉一拍，加载 data；发完 1 字节后 busy 自动拉低。
// 电平：空闲高、起始位 0、停止位 1。
//
// 默认 50MHz / 115200 ≈ 434 个时钟周期一位，误差 <0.01%。
//============================================================================
module uart_tx #(
    parameter integer CLK_FREQ = 50_000_000,   // 系统时钟 50MHz
    parameter integer BAUD     = 115_200       // 波特率
)(
    input  wire       clk,
    input  wire       rst_n,       // 复位，低有效
    input  wire [7:0] data,        // 要发送的字节
    input  wire       send,        // 发送触发（1 拍脉冲）
    output reg        busy,        // 正在发送中
    output reg        tx           // 串口输出（接 CH340E RXD，FPGA 引脚 M15）
);

    localparam [15:0] BAUD_CNT = CLK_FREQ / BAUD - 1;   // 一位所需时钟数 - 1

    reg [15:0] cnt;        // 波特率分频计数
    reg [3:0]  bit_pos;    // 当前发送到第几位（0=起始位, 1..8=数据, 9=停止位）
    reg [9:0]  shreg;      // {停止位, data[7:0], 起始位}，LSB 先发

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx      <= 1'b1;        // 空闲高
            busy    <= 1'b0;
            cnt     <= 16'd0;
            bit_pos <= 4'd0;
            shreg   <= 10'h3FF;
        end else begin
            if (busy) begin
                if (cnt >= BAUD_CNT) begin
                    cnt <= 16'd0;
                    if (bit_pos >= 4'd10) begin   // 起始位+8 数据位+停止位 共 10 位发完
                        busy <= 1'b0;
                        tx   <= 1'b1;
                    end else begin
                        tx      <= shreg[bit_pos];
                        bit_pos <= bit_pos + 1'b1;
                    end
                end else begin
                    cnt <= cnt + 1'b1;
                end
            end else if (send) begin
                busy    <= 1'b1;
                cnt     <= 16'd0;
                bit_pos <= 4'd1;                  // 起始位已在此拍发出，下一个发数据位 0
                shreg   <= {1'b1, data, 1'b0};   // 停止位 + 8 位数据 + 起始位
                tx      <= 1'b0;                  // 先发起始位
            end
        end
    end

endmodule
