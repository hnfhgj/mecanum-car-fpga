//============================================================================
// quad_decoder.v —— 正交编码器 4× 解码模块
//
// 输入编码器 A/B 两相（90° 相位差），输出有符号计数（正转加、反转减）。
// 4× 解码：A、B 的每个边沿都计一次数，分辨率是 MCU 单沿计数的 4 倍。
//
// 状态机（{A,B} 格雷码）：
//   正转：00 → 01 → 11 → 10 → 00
//   反转：00 → 10 → 11 → 01 → 00
// 对角跳变（如 00→11）视为抖动，忽略不计数。
//============================================================================
module quad_decoder #(
    parameter integer CNT_WIDTH = 16            // 计数位宽
)(
    input  wire                  clk,           // 系统时钟 50MHz
    input  wire                  rst_n,         // 复位，低有效
    input  wire                  a,             // 编码器 A 相
    input  wire                  b,             // 编码器 B 相
    output reg  [CNT_WIDTH-1:0] count          // 计数（2's complement，可正可负）
);

    // ---- 两拍同步 A/B 到 clk 域，消除亚稳态 + 毛刺 ----
    reg a1, a2, b1, b2;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            a1 <= 1'b0; a2 <= 1'b0;
            b1 <= 1'b0; b2 <= 1'b0;
        end else begin
            a1 <= a; a2 <= a1;
            b1 <= b; b2 <= b1;
        end
    end

    // ---- 4× 解码状态机 ----
    // cur = 当前同步后的 {a2,b2}，next = 新采到的 {a2,b2}
    reg [1:0] cur;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cur   <= 2'b00;
            count <= {CNT_WIDTH{1'b0}};
        end else begin
            cur <= {a2, b2};
            case ({a2, b2})
                2'b00: case (cur)
                    2'b10: count <= count + 1'b1;   // 反转路径 10→00
                    2'b01: count <= count - 1'b1;   // 正转路径 01→00
                    default: ;
                endcase
                2'b01: case (cur)
                    2'b00: count <= count + 1'b1;   // 正转 00→01
                    2'b11: count <= count - 1'b1;   // 反转 11→01
                    default: ;
                endcase
                2'b11: case (cur)
                    2'b01: count <= count + 1'b1;   // 正转 01→11
                    2'b10: count <= count - 1'b1;   // 反转 10→11
                    default: ;
                endcase
                2'b10: case (cur)
                    2'b11: count <= count + 1'b1;   // 正转 11→10
                    2'b00: count <= count - 1'b1;   // 反转 00→10
                    default: ;
                endcase
            endcase
        end
    end

endmodule
