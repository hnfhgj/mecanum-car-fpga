//============================================================================
// uart_enc_debug.v —— 编码器(位置+速度) + IMU + TOF 串口调试输出
//
// 每帧发送前等待 SEND_MS 毫秒；五行整帧数据使用同一时刻的快照：
//   LF=+12345 V=+00123 RF=-02345 V=-00045 LB=+00067 V=+00012 RB=-08910 V=-00123
//   AX=+01234 AY=-00567 AZ=+16200 GX=+00123 GY=-00045 GZ=+00012
//   TOF=01234
//   IMU_INIT=1 IMU_NEW=1 TOF_INIT=1 TOF_NEW=1
//
// 第一行：每轮 = 名称 + '=' + 符号 + 5 位计数 + 空格 + "V=" + 符号 + 5 位速度
// 第二行：IMU = AX/AY/AZ(加速度) + GX/GY/GZ(陀螺仪)，符号 + 5 位十进制
// 第三行：TOF = 距离(mm)，5 位十进制（无效时为 8192）
//
// 串口 115200 8N1，只发不收。接线：FPGA 引脚 M15 → 板上 CH340E RXD。
//============================================================================
module uart_enc_debug #(
    parameter integer CLK_FREQ = 50_000_000,   // 系统时钟 50MHz
    parameter integer BAUD     = 115_200,      // 波特率
    parameter integer SEND_MS  = 100,          // 每多少毫秒发一次
    parameter integer DIAGNOSTIC_ENABLE=0
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire [15:0] lf_count, rf_count, lb_count, rb_count,
    input  wire [15:0] lf_speed, rf_speed, lb_speed, rb_speed,
    input  wire [15:0] acc_x, acc_y, acc_z,       // 加速度计原始值
    input  wire [15:0] gyro_x, gyro_y, gyro_z,    // 陀螺仪原始值
    input  wire [15:0] distance_mm,               // TOF 距离(mm)
    input  wire        imu_init, imu_valid,
    input  wire        tof_init, tof_valid,
    input wire [15:0] target_speed,
    input wire [3:0] run_mask, fault_mask,
    input wire [15:0] attitude_roll,attitude_pitch,attitude_yaw,
    input wire attitude_ready,
    input wire [15:0] heading_left,heading_right,heading_turn,
    input wire heading_fault,
    input wire [3:0] calibration_mask,encoder_sign_mask,
    input wire [15:0] fault_reasons,
    input wire diagnostic_stop,
    input wire [167:0] diagnostic_bits,
    output wire        uart_tx
);

    reg [15:0] frame [0:21];
    reg frame_heading_fault;
    reg [3:0] frame_calibration,frame_encoder_sign;
    reg [15:0] frame_reasons;
    reg frame_diagnostic_stop;
    reg [167:0] frame_io;
    reg frame_attitude_ready;
    reg imu_pending, tof_pending;
    reg [3:0] frame_status, frame_run, frame_fault;
    integer k;

    // Shared binary-to-decimal conversion before serializing a snapshot.
    reg [19:0] decimal_frame [0:21];
    reg [19:0] decimal_work, decimal_adjusted;
    reg [15:0] binary_work;
    reg [4:0] convert_index;
    reg [4:0] convert_bit;
    integer nibble;
    always @* begin
        decimal_adjusted = decimal_work;
        for (nibble=0; nibble<5; nibble=nibble+1)
            if (decimal_work[nibble*4 +: 4] >= 5)
                decimal_adjusted[nibble*4 +: 4] = decimal_work[nibble*4 +: 4] + 4'd3;
    end

    // ---- 消息布局 ----
    localparam [7:0] ENC_LEN   = 8'd76;    // 编码器数据字节数
    localparam [7:0] IMU_START = 8'd78;    // IMU 行起始 idx
    localparam [7:0] IMU_END   = 8'd137;   // IMU 数据结束（不含 \r\n）
    localparam [7:0] TOF_START = 8'd139;   // TOF 行起始 idx
    localparam [7:0] TOF_END   = 8'd148;   // TOF 数据结束（不含 \r\n）
    localparam [7:0] STATUS_START = 8'd150;
    localparam [7:0] STATUS_END = 8'd191;
    localparam [7:0] PID_START = 8'd193, PID_END=8'd225;
    localparam [255:0] PID_TEXT="PID T=+00000 RUN=0000 FAULT=0000";
    localparam [8:0] MSG_LEN = DIAGNOSTIC_ENABLE ? 9'd388 : 9'd337;
    localparam [55:0] IO_TEXT="IO HEX=";
    localparam [279:0] DBG_TEXT="DBG CAL=0000 ER=0000 WHY=0000 LIM=0";
    localparam [271:0] HEAD_TEXT="HDG L=+00000 R=+00000 U=+00000 F=0";
    localparam [279:0] ATT_TEXT="ATT R=+00000 P=+00000 Y=+00000 OK=0";
    localparam [327:0] STATUS_TEXT = "IMU_INIT=0 IMU_NEW=0 TOF_INIT=0 TOF_NEW=0";

    reg  [8:0] idx;
    wire [8:0] io_i=idx-9'd337;
    wire [3:0] io_hex=frame_io[(41-(io_i-7))*4 +: 4];
    wire [8:0] idx_w = idx;
    wire [8:0] att_i=idx-9'd227;
    wire [8:0] head_i=idx-9'd264;
    wire [8:0] dbg_i=idx-9'd300;
    reg [19:0] head_digits;
    reg head_negative;
    reg [3:0] head_position;
    always @* begin
        if(head_i<13) begin head_digits=decimal_frame[19]; head_negative=frame[19][15]; head_position=head_i-6; end
        else if(head_i<22) begin head_digits=decimal_frame[20]; head_negative=frame[20][15]; head_position=head_i-15; end
        else begin head_digits=decimal_frame[21]; head_negative=frame[21][15]; head_position=head_i-24; end
    end
    reg [19:0] att_digits;
    reg att_negative;
    reg [3:0] att_digit_position;
    always @* begin
        if(att_i<13) begin att_digits=decimal_frame[16]; att_negative=frame[16][15]; att_digit_position=att_i-6; end
        else if(att_i<22) begin att_digits=decimal_frame[17]; att_negative=frame[17][15]; att_digit_position=att_i-15; end
        else begin att_digits=decimal_frame[18]; att_negative=frame[18][15]; att_digit_position=att_i-24; end
    end

    wire [1:0] wheel = idx_w / 8'd19;
    wire [4:0] sub   = idx_w % 8'd19;

    wire [7:0] imu_i     = idx_w - IMU_START;
    wire [2:0] imu_field = imu_i / 8'd10;
    wire [3:0] imu_sub   = imu_i % 8'd10;

    wire [7:0] tof_i     = idx_w - TOF_START;

    reg [19:0] cur_cdigits, cur_sdigits;
    reg        cur_cneg, cur_sneg;
    always @* begin
        case (wheel)
            2'd0: begin cur_cdigits = decimal_frame[0]; cur_sdigits = decimal_frame[4];
                         cur_cneg = frame[0][15]; cur_sneg = frame[4][15]; end
            2'd1: begin cur_cdigits = decimal_frame[1]; cur_sdigits = decimal_frame[5];
                         cur_cneg = frame[1][15]; cur_sneg = frame[5][15]; end
            2'd2: begin cur_cdigits = decimal_frame[2]; cur_sdigits = decimal_frame[6];
                         cur_cneg = frame[2][15]; cur_sneg = frame[6][15]; end
            default: begin cur_cdigits = decimal_frame[3]; cur_sdigits = decimal_frame[7];
                         cur_cneg = frame[3][15]; cur_sneg = frame[7][15]; end
        endcase
    end
    wire [7:0] c_sign = cur_cneg ? 8'h2D : 8'h2B;
    wire [7:0] s_sign = cur_sneg ? 8'h2D : 8'h2B;
    wire [7:0] c_d4 = {4'h3, cur_cdigits[19:16]};
    wire [7:0] c_d3 = {4'h3, cur_cdigits[15:12]};
    wire [7:0] c_d2 = {4'h3, cur_cdigits[11:8]};
    wire [7:0] c_d1 = {4'h3, cur_cdigits[7:4]};
    wire [7:0] c_d0 = {4'h3, cur_cdigits[3:0]};
    wire [7:0] s_d4 = {4'h3, cur_sdigits[19:16]};
    wire [7:0] s_d3 = {4'h3, cur_sdigits[15:12]};
    wire [7:0] s_d2 = {4'h3, cur_sdigits[11:8]};
    wire [7:0] s_d1 = {4'h3, cur_sdigits[7:4]};
    wire [7:0] s_d0 = {4'h3, cur_sdigits[3:0]};

    reg [19:0] imu_digits;
    reg        imu_neg;
    always @* begin
        case (imu_field)
            3'd0: begin imu_digits = decimal_frame[8];  imu_neg = frame[8][15];  end
            3'd1: begin imu_digits = decimal_frame[9];  imu_neg = frame[9][15];  end
            3'd2: begin imu_digits = decimal_frame[10]; imu_neg = frame[10][15]; end
            3'd3: begin imu_digits = decimal_frame[11]; imu_neg = frame[11][15]; end
            3'd4: begin imu_digits = decimal_frame[12]; imu_neg = frame[12][15]; end
            default: begin imu_digits = decimal_frame[13]; imu_neg = frame[13][15]; end
        endcase
    end
    wire [7:0]  imu_sign = imu_neg ? 8'h2D : 8'h2B;
    wire [7:0] imu_d4 = {4'h3, imu_digits[19:16]};
    wire [7:0] imu_d3 = {4'h3, imu_digits[15:12]};
    wire [7:0] imu_d2 = {4'h3, imu_digits[11:8]};
    wire [7:0] imu_d1 = {4'h3, imu_digits[7:4]};
    wire [7:0] imu_d0 = {4'h3, imu_digits[3:0]};
    wire [7:0] imu_n0 = (imu_field < 3'd3) ? 8'h41 : 8'h47;
    wire [7:0] imu_n1 = (imu_field % 3'd3 == 3'd0) ? 8'h58 :
                        (imu_field % 3'd3 == 3'd1) ? 8'h59 :
                                                     8'h5A;

    // TOF 距离（无符号，5 位十进制）
    wire [7:0] tof_d4 = {4'h3, decimal_frame[14][19:16]};
    wire [7:0] tof_d3 = {4'h3, decimal_frame[14][15:12]};
    wire [7:0] tof_d2 = {4'h3, decimal_frame[14][11:8]};
    wire [7:0] tof_d1 = {4'h3, decimal_frame[14][7:4]};
    wire [7:0] tof_d0 = {4'h3, decimal_frame[14][3:0]};

    reg [7:0] tx_byte;
    always @* begin
        if (idx < ENC_LEN) begin
            case (sub)
                5'd0 : tx_byte = (wheel[0] == 1'b0) ? 8'h4C : 8'h52;
                5'd1 : tx_byte = (wheel[1] == 1'b0) ? 8'h46 : 8'h42;
                5'd2 : tx_byte = 8'h3D;
                5'd3 : tx_byte = c_sign;
                5'd4 : tx_byte = c_d4;
                5'd5 : tx_byte = c_d3;
                5'd6 : tx_byte = c_d2;
                5'd7 : tx_byte = c_d1;
                5'd8 : tx_byte = c_d0;
                5'd9 : tx_byte = 8'h20;
                5'd10: tx_byte = 8'h56;
                5'd11: tx_byte = 8'h3D;
                5'd12: tx_byte = s_sign;
                5'd13: tx_byte = s_d4;
                5'd14: tx_byte = s_d3;
                5'd15: tx_byte = s_d2;
                5'd16: tx_byte = s_d1;
                5'd17: tx_byte = s_d0;
                default: tx_byte = 8'h20;
            endcase
        end else if (idx < IMU_START) begin
            tx_byte = (idx == 8'd76) ? 8'h0D : 8'h0A;
        end else if (idx < IMU_END) begin
            case (imu_sub)
                4'd0 : tx_byte = imu_n0;
                4'd1 : tx_byte = imu_n1;
                4'd2 : tx_byte = 8'h3D;
                4'd3 : tx_byte = imu_sign;
                4'd4 : tx_byte = imu_d4;
                4'd5 : tx_byte = imu_d3;
                4'd6 : tx_byte = imu_d2;
                4'd7 : tx_byte = imu_d1;
                4'd8 : tx_byte = imu_d0;
                default: tx_byte = 8'h20;
            endcase
        end else if (idx < TOF_END) begin
            if (idx < TOF_START)                          // IMU 行 \r\n (137,138)
                tx_byte = (idx == IMU_END) ? 8'h0D : 8'h0A;
            else begin                                    // TOF 行 (139..147)
                case (tof_i)
                    8'd0 : tx_byte = 8'h54;   // 'T'
                    8'd1 : tx_byte = 8'h4F;   // 'O'
                    8'd2 : tx_byte = 8'h46;   // 'F'
                    8'd3 : tx_byte = 8'h3D;   // '='
                    8'd4 : tx_byte = tof_d4;
                    8'd5 : tx_byte = tof_d3;
                    8'd6 : tx_byte = tof_d2;
                    8'd7 : tx_byte = tof_d1;
                    8'd8 : tx_byte = tof_d0;
                    default: tx_byte = 8'h20;
                endcase
            end
        end else if (idx < STATUS_START) begin
            tx_byte = (idx == TOF_END) ? 8'h0D : 8'h0A;   // TOF 行 \r\n (148,149)
        end else if (idx < STATUS_END) begin
            case (idx - STATUS_START)
                8'd9:  tx_byte = 8'h30 + {7'd0, frame_status[3]};
                8'd19: tx_byte = 8'h30 + {7'd0, frame_status[2]};
                8'd30: tx_byte = 8'h30 + {7'd0, frame_status[1]};
                8'd40: tx_byte = 8'h30 + {7'd0, frame_status[0]};
                default: tx_byte = STATUS_TEXT[(40 - (idx - STATUS_START))*8 +: 8];
            endcase
        end else if (idx < PID_START) begin
            tx_byte = (idx == STATUS_END) ? 8'h0D : 8'h0A;
        end else if (idx < PID_END) begin
            case (idx-PID_START)
                6: tx_byte=frame[15][15] ? "-" : "+";
                7: tx_byte={4'h3,decimal_frame[15][19:16]};
                8: tx_byte={4'h3,decimal_frame[15][15:12]};
                9: tx_byte={4'h3,decimal_frame[15][11:8]};
                10: tx_byte={4'h3,decimal_frame[15][7:4]};
                11: tx_byte={4'h3,decimal_frame[15][3:0]};
                17: tx_byte=frame_run[0] ? "1" : "0";
                18: tx_byte=frame_run[1] ? "1" : "0";
                19: tx_byte=frame_run[2] ? "1" : "0";
                20: tx_byte=frame_run[3] ? "1" : "0";
                28: tx_byte=frame_fault[0] ? "1" : "0";
                29: tx_byte=frame_fault[1] ? "1" : "0";
                30: tx_byte=frame_fault[2] ? "1" : "0";
                31: tx_byte=frame_fault[3] ? "1" : "0";
                default: tx_byte=PID_TEXT[(31-(idx-PID_START))*8 +: 8];
            endcase
        end else if(idx<227) tx_byte=(idx==PID_END) ? 8'h0D : 8'h0A;
        else if(idx<262) begin
            if((att_i>=6 && att_i<=11) || (att_i>=15 && att_i<=20) || (att_i>=24 && att_i<=29)) begin
                case(att_digit_position)
                    0: tx_byte=att_negative ? "-" : "+";
                    1: tx_byte={4'h3,att_digits[19:16]};
                    2: tx_byte={4'h3,att_digits[15:12]};
                    3: tx_byte={4'h3,att_digits[11:8]};
                    4: tx_byte={4'h3,att_digits[7:4]};
                    default: tx_byte={4'h3,att_digits[3:0]};
                endcase
            end else if(att_i==34) tx_byte=frame_attitude_ready ? "1" : "0";
            else tx_byte=ATT_TEXT[(34-att_i)*8 +: 8];
        end else if(idx<264) tx_byte=(idx==262) ? 8'h0D : 8'h0A;
        else if(idx<298) begin
            if((head_i>=6 && head_i<=11) || (head_i>=15 && head_i<=20) || (head_i>=24 && head_i<=29)) begin
                case(head_position)
                    0: tx_byte=head_negative ? "-" : "+";
                    1: tx_byte={4'h3,head_digits[19:16]};
                    2: tx_byte={4'h3,head_digits[15:12]};
                    3: tx_byte={4'h3,head_digits[11:8]};
                    4: tx_byte={4'h3,head_digits[7:4]};
                    default: tx_byte={4'h3,head_digits[3:0]};
                endcase
            end else if(head_i==33) tx_byte=frame_heading_fault ? "1" : "0";
            else tx_byte=HEAD_TEXT[(33-head_i)*8 +: 8];
        end else if(idx<300) tx_byte=(idx==298) ? 8'h0D : 8'h0A;
        else if(idx<335) begin
            if(dbg_i>=8 && dbg_i<=11) tx_byte="0"+frame_calibration[dbg_i-8];
            else if(dbg_i>=16 && dbg_i<=19) tx_byte="0"+frame_encoder_sign[dbg_i-16];
            else if(dbg_i>=25 && dbg_i<=28) tx_byte="0"+frame_reasons[(dbg_i-25)*4 +: 4];
            else if(dbg_i==34) tx_byte=frame_diagnostic_stop ? "1" : "0";
            else tx_byte=DBG_TEXT[(34-dbg_i)*8 +: 8];
        end else if(idx<337) tx_byte=(idx==335) ? 8'h0D : 8'h0A;
        else if(io_i<7) tx_byte=IO_TEXT[(6-io_i)*8 +: 8];
        else if(io_i<49) tx_byte=(io_hex<10) ? (8'h30+io_hex) : (8'h41+io_hex-10);
        else tx_byte=(io_i==49) ? 8'h0D : 8'h0A;
    end

    localparam [31:0] SEND_MAX = CLK_FREQ / 1000 * SEND_MS - 1;
    localparam [2:0] S_IDLE=0, S_SEND=1, S_WAIT=2, S_BCD_PREP=3, S_BCD_SHIFT=4, S_BCD_STORE=5;

    reg [31:0] timer;
    reg [2:0]  state;
    reg        tx_send;
    reg [7:0]  tx_data;
    wire       tx_busy;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            timer   <= 32'd0;
            idx     <= 8'd0;
            state   <= S_IDLE;
            tx_send <= 1'b0;
            tx_data <= 8'd0;
            imu_pending <= 1'b0;
            tof_pending <= 1'b0;
            frame_status <= 4'd0; frame_run<=0; frame_fault<=0;
            frame_attitude_ready<=0; frame_heading_fault<=0;
            frame_calibration<=0; frame_encoder_sign<=0; frame_reasons<=0; frame_diagnostic_stop<=0; frame_io<=0;
            decimal_work<=0; binary_work<=0; convert_index<=0; convert_bit<=0;
            for (k = 0; k < 22; k = k + 1) begin
                frame[k] <= 16'd0; decimal_frame[k] <= 20'd0;
            end
        end else begin
            tx_send <= 1'b0;
            if (imu_valid) imu_pending <= 1'b1;
            if (tof_valid) tof_pending <= 1'b1;
            case (state)
                S_IDLE: begin
                    if (timer >= SEND_MAX) begin
                        timer <= 32'd0;
                        idx   <= 8'd0;
                        convert_index <= 0;
                        state <= S_BCD_PREP;
                        frame[0] <= lf_count; frame[1] <= rf_count;
                        frame[2] <= lb_count; frame[3] <= rb_count;
                        frame[4] <= lf_speed; frame[5] <= rf_speed;
                        frame[6] <= lb_speed; frame[7] <= rb_speed;
                        frame[8] <= acc_x; frame[9] <= acc_y; frame[10] <= acc_z;
                        frame[11] <= gyro_x; frame[12] <= gyro_y; frame[13] <= gyro_z;
                        frame[14] <= distance_mm; frame[15]<=target_speed;
                        frame[16]<=attitude_roll; frame[17]<=attitude_pitch; frame[18]<=attitude_yaw;
                        frame_attitude_ready<=attitude_ready;
                        frame[19]<=heading_left; frame[20]<=heading_right; frame[21]<=heading_turn;
                        frame_heading_fault<=heading_fault;
                        frame_calibration<=calibration_mask; frame_encoder_sign<=encoder_sign_mask;
                        frame_reasons<=fault_reasons; frame_diagnostic_stop<=diagnostic_stop;
                        frame_io<=diagnostic_bits;
                        frame_run<=run_mask; frame_fault<=fault_mask;
                        frame_status <= {imu_init, imu_pending, tof_init, tof_pending | tof_valid};
                        // IMU 数据先在顶层锁存，当前拍 valid 归入下一帧。
                        imu_pending <= imu_valid;
                        tof_pending <= 1'b0;
                    end else
                        timer <= timer + 1'b1;
                end
                S_BCD_PREP: begin
                    binary_work <= (convert_index!=14 && frame[convert_index][15])
                        ? (~frame[convert_index]+16'd1) : frame[convert_index];
                    decimal_work<=0; convert_bit<=0; state<=S_BCD_SHIFT;
                end
                S_BCD_SHIFT: begin
                    decimal_work<={decimal_adjusted[18:0],binary_work[15]};
                    binary_work<={binary_work[14:0],1'b0};
                    if (convert_bit==15) state<=S_BCD_STORE;
                    else convert_bit<=convert_bit+1'b1;
                end
                S_BCD_STORE: begin
                    decimal_frame[convert_index]<=decimal_work;
                    if (convert_index==21) state<=S_SEND;
                    else begin convert_index<=convert_index+1'b1; state<=S_BCD_PREP; end
                end
                S_SEND: begin
                    if (!tx_busy) begin
                        if (idx >= MSG_LEN)
                            state <= S_IDLE;
                        else begin
                            tx_data <= tx_byte;
                            tx_send <= 1'b1;
                            idx     <= idx + 1'b1;
                            state   <= S_WAIT;
                        end
                    end
                end
                S_WAIT: begin
                    if (tx_busy)
                        state <= S_SEND;
                end
                default: state <= S_IDLE;
            endcase
        end
    end

    uart_tx #(.CLK_FREQ(CLK_FREQ), .BAUD(BAUD)) u_tx (
        .clk(clk), .rst_n(rst_n),
        .data(tx_data), .send(tx_send), .busy(tx_busy), .tx(uart_tx)
    );

endmodule
