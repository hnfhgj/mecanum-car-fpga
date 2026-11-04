//============================================================================
// motor_driver.v —— 单路电机驱动模块
//
// 与 MCU 端 motor.c 的 motor_set() 逻辑一一对应：
//   speed >= 0 : dir=0（正转），PWM 占空比 =  speed
//   speed <  0 : dir=1（反转），PWM 占空比 = -speed
//
// PWM 频率 17kHz，与 MCU 端 pwm_init(...,17000,...) 保持一致。
// 说明：motor.c 的 motor_set() 里左电机分支误用了 right_pwm 变量（疑似
//       复制粘贴 bug），本模块按正确逻辑实现（左右各用各的 speed）。
//============================================================================
module motor_driver #(
    parameter integer CLK_FREQ = 50_000_000,   // 系统时钟 50MHz
    parameter integer PWM_FREQ = 17_000,       // PWM 频率 17kHz
    parameter integer PHASE_OFFSET = 0        // Counter offset, 0 <= offset < PWM_PERIOD
)(
    input  wire               clk,     // 50MHz 时钟
    input  wire               rst_n,   // 复位，低有效
    input  wire signed [15:0] speed,   // 有符号转速：幅值 = 占空比计数值(0~PWM_PERIOD)
    output reg                dir,     // 方向：0=正转，1=反转
    output wire               pwm      // 17kHz PWM 输出
);

    // PWM 周期计数值 = 50MHz / 17kHz ≈ 2941
    localparam [15:0] PWM_PERIOD = CLK_FREQ / PWM_FREQ;

    reg  [15:0] cnt;
    wire [15:0] duty;

    // 方向：speed 符号位为 1（负数）时反转
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            dir <= 1'b0;
        else
            dir <= speed[15];
    end

    // 占空比 = |speed|
    assign duty = (speed < 16'sd0) ? (16'sd0 - speed) : speed;

    // PWM 计数器：0 ~ PWM_PERIOD-1 循环
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            cnt <= PHASE_OFFSET;
        else if (cnt >= PWM_PERIOD - 1'b1)
            cnt <= 16'd0;
        else
            cnt <= cnt + 1'b1;
    end

    // 计数小于占空比时输出高电平（占空比 = duty / PWM_PERIOD）
    assign pwm = rst_n && (cnt < duty);

endmodule
