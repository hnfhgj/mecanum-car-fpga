//============================================================================
// speed_measure.v —— 编码器测速模块（M 法）
//
// 每 10ms 采样一次 quad_decoder 的计数值，输出这段时间的增量（即速度）。
//   speed = count_in - last_count   （单位：counts/10ms，2's complement）
// 同时输出 10ms 节拍 tick，供速度环 PID 复用，保证测速和控制同拍。
//============================================================================
module speed_measure #(
    parameter integer CLK_FREQ      = 50_000_000,  // 系统时钟 50MHz
    parameter integer MEAS_PERIOD_MS = 10,          // 测速周期 10ms
    parameter integer CNT_WIDTH      = 16
)(
    input  wire                  clk,
    input  wire                  rst_n,
    input  wire [CNT_WIDTH-1:0]  count_in,   // quad_decoder 的计数值
    output reg  signed [CNT_WIDTH-1:0] speed, // 每 10ms 增量（可正可负）
    output reg                   tick        // 10ms 节拍（高 1 拍）
);

    // 10ms 分频：50MHz * 10ms = 500000 个周期
    localparam [31:0] TICK_MAX = CLK_FREQ / 1000 * MEAS_PERIOD_MS - 1;

    reg [31:0] tick_cnt;
    reg [CNT_WIDTH-1:0] last_count;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tick_cnt   <= 32'd0;
            last_count <= {CNT_WIDTH{1'b0}};
            speed      <= {CNT_WIDTH{1'b0}};
            tick       <= 1'b0;
        end else begin
            tick <= 1'b0;
            if (tick_cnt >= TICK_MAX) begin
                tick_cnt   <= 32'd0;
                tick       <= 1'b1;
                speed      <= $signed(count_in) - $signed(last_count);
                last_count <= count_in;
            end else begin
                tick_cnt <= tick_cnt + 1'b1;
            end
        end
    end

endmodule
