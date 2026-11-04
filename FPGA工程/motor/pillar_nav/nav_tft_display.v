// SPI TFT180 dashboard: 160x128 landscape, RGB565, no framebuffer.
// SeekFree initialization; module-specific address offsets are configurable.
module nav_tft_display #(
    parameter integer CLK_FREQ = 50_000_000,
    parameter integer SPI_FREQ = 5_000_000,
    parameter integer RESET_MS = 10,
    parameter integer STARTUP_MS = 120,
    parameter integer SLEEP_MS = 120,
    parameter integer FRAME_GAP_MS = 100,
    // Diagnostic default for a native 128x160 memory base (no border offset).
    // SeekFree's +1/+2 assumes additional addressable border memory.
    parameter [7:0] X_OFFSET = 0, Y_OFFSET = 0,
    parameter integer DASHBOARD_ENABLE = 0,
    parameter integer TELEMETRY_PAGES = 0,
    parameter integer CAMERA_PAGE_ENABLE = 0,
    parameter integer LIDAR_PAGE_ENABLE = 0
)(
    input wire clk, rst_n,
    input wire [15:0] lidar_pixel,
    output wire [7:0] lidar_x, output wire [6:0] lidar_y,
    output reg lidar_frame_start,
    input wire [15:0] lf_count, rf_count, lb_count, rb_count,
    input wire [15:0] lf_speed, rf_speed, lb_speed, rb_speed,
    input wire [15:0] acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z,
    input wire [15:0] distance_mm,
    input wire imu_init, tof_init,
    input wire [15:0] attitude_roll, attitude_pitch, attitude_yaw,
    input wire attitude_ready,
    input wire [15:0] heading_turn, heading_left, heading_right,
    input wire heading_fault, outputs_enabled,
    input wire [15:0] lf_duty, rf_duty, lb_duty, rb_duty,
    input wire [15:0] fault_reasons, tof_retries,
    input wire [3:0] calibration_mask, encoder_sign_mask,
    input wire [7:0] encoder_inputs,
    input wire [5:0] tof_state,
    input wire [15:0] target_speed,
    input wire closed_loop,
    input wire [3:0] fault_mask,
    input wire [2:0] page,
    // Only 12 (heading P) and 14 (heading D) are editable selections.
    input wire [3:0] selection,
    // Little-endian words: LF P/I/D, RF P/I/D, LB P/I/D,
    // RB P/I/D, heading P/I/D. Unsigned Q12; heading I is locked zero.
    input wire [479:0] gains_flat,
    // Main must keep init_done/valid low until P4 camera pins are confirmed
    // and real capture initialization/frame completion has occurred.
    input wire camera_valid, camera_init_done, camera_fault,
    input wire [15:0] camera_pixel,
    output wire [14:0] camera_rd_addr,
    // Page-2 frame boundaries, including status-only frames. Main owns buffers.
    output reg camera_frame_start, camera_frame_end,
    output wire tft_scl, tft_sda, tft_cs_n,
    output reg tft_reset, tft_dc,
    output wire tft_bl
);
    localparam [4:0] RESET_LOW=0, STARTUP=1, INIT_SEND=2, INIT_WAIT=3,
        SLEEP_WAIT=4, FRAME_START=5, WINDOW_SEND=6, WINDOW_WAIT=7,
        PIXEL_SEND=8, PIXEL_WAIT=9, FRAME_GAP=10,
        BCD_PREP=11, BCD_SHIFT=12, BCD_STORE=13,
        PIXEL_FETCH=14, PIXEL_LATCH=15, GAIN_PREP=16,
        GAIN_SCALE=17, GAIN_BINARY=18, GAIN_SHIFT=19, GAIN_STORE=20, NAV_PIPE_WAIT=21;
    reg [4:0] state;
    reg [31:0] timer;
    reg [6:0] init_index;
    wire [8:0] init_entry;
    tft180_init_rom u_init(.addr(init_index), .entry(init_entry));
    reg [3:0] window_index;
    reg [8:0] window_entry;
    localparam [7:0] X_END = X_OFFSET + 8'd159;
    localparam [7:0] Y_END = Y_OFFSET + 8'd127;
    reg [7:0] px;
    reg [6:0] py;
    reg low_byte, ready;
    reg [15:0] frame [0:15];
    reg [19:0] decimal_frame [0:15];
    reg [19:0] decimal_work, decimal_adjusted;
    reg [15:0] binary_work;
    reg [3:0] convert_index;
    reg [4:0] convert_bit;
    reg frame_imu_init, frame_tof_init, frame_closed_loop;
    reg [3:0] frame_fault;
    reg frame_raw_imu, frame_heading_fault, frame_outputs;
    reg [15:0] frame_reasons, frame_retries;
    reg [3:0] frame_cal, frame_sign;
    reg [7:0] frame_inputs;
    reg [5:0] frame_tof_state;
    reg [2:0] frame_page;
    reg [3:0] frame_selection;
    reg frame_camera_valid, frame_camera_init, frame_camera_fault;
    reg [31:0] frame_gain [0:14];
    reg [31:0] gain_decimal [0:14];
    reg [48:0] scale_a, scale_b, scale_result;
    reg [31:0] gain_binary, gain_work, gain_adjusted;
    reg [15:0] pixel_latched;
    integer gain_nibble;
    assign lidar_x=px;
    assign lidar_y=py;
    wire lidar_page=LIDAR_PAGE_ENABLE && frame_page==5;
    wire camera_page = TELEMETRY_PAGES ? (CAMERA_PAGE_ENABLE && frame_page==4) : frame_page==2;
    wire camera_requested = TELEMETRY_PAGES ? (CAMERA_PAGE_ENABLE && page==4) : page==2;
    wire camera_image = DASHBOARD_ENABLE && camera_page &&
        frame_camera_valid && frame_camera_init && !frame_camera_fault;
    assign camera_rd_addr = ({8'd0,py} << 7) + ({8'd0,py} << 5) + {7'd0,px};
    integer k;
    integer nibble;
    reg [7:0] tx_data;
    reg tx_start, tx_last;
    wire tx_done;
    spi_master #(.CLK_FREQ(CLK_FREQ), .SPI_FREQ(SPI_FREQ)) u_spi(
        .clk(clk), .rst_n(rst_n), .start(tx_start), .tx_data(tx_data), .last(tx_last),
        .busy(), .rx_data(), .rx_done(tx_done), .sck(tft_scl), .mosi(tft_sda),
        .miso(1'b0), .cs_n(tft_cs_n));
    assign tft_bl = rst_n && ready;

    // Shared 16-step binary-to-decimal converter, once per value per frame.
    always @* begin
        decimal_adjusted = decimal_work;
        for (nibble=0; nibble<5; nibble=nibble+1)
            if (decimal_work[nibble*4 +: 4] >= 5)
                decimal_adjusted[nibble*4 +: 4] = decimal_work[nibble*4 +: 4] + 4'd3;
    end
    always @* begin
        gain_adjusted = gain_work;
        for (gain_nibble=0; gain_nibble<8; gain_nibble=gain_nibble+1)
            if (gain_work[gain_nibble*4 +: 4]>=5)
                gain_adjusted[gain_nibble*4 +: 4]=gain_work[gain_nibble*4 +: 4]+4'd3;
    end

    always @* begin
        case (window_index)
            0: window_entry = 9'h02a;
            1: window_entry = 9'h100;
            2: window_entry = {1'b1,X_OFFSET};
            3: window_entry = 9'h100;
            4: window_entry = {1'b1,X_END};
            5: window_entry = 9'h02b;
            6: window_entry = 9'h100;
            7: window_entry = {1'b1,Y_OFFSET};
            8: window_entry = 9'h100;
            9: window_entry = {1'b1,Y_END};
            default: window_entry = 9'h02c;
        endcase
    end

    function [7:0] number_char;
        input [19:0] digits;
        input [2:0] pos;
        input negative;
        begin
            case (pos)
                0: number_char = negative ? "-" : "+";
                1: number_char = {4'h3,digits[19:16]};
                2: number_char = {4'h3,digits[15:12]};
                3: number_char = {4'h3,digits[11:8]};
                4: number_char = {4'h3,digits[7:4]};
                5: number_char = {4'h3,digits[3:0]};
                default: number_char = " ";
            endcase
        end
    endfunction
    function [7:0] hex_char;
        input [3:0] value;
        begin hex_char=value<10 ? 8'd48+value : 8'd65+value-10; end
    endfunction
    // Four integer places plus three fractional places, leading spaces.
    function [63:0] gain_text;
        input [31:0] digits;
        begin
            gain_text={{4'h3,digits[27:24]},{4'h3,digits[23:20]},
                {4'h3,digits[19:16]},{4'h3,digits[15:12]},8'h2e,
                {4'h3,digits[11:8]},{4'h3,digits[7:4]},{4'h3,digits[3:0]}};
            if(digits[27:24]==0) begin
                gain_text[63:56]=" ";
                if(digits[23:20]==0) begin
                    gain_text[55:48]=" ";
                    if(digits[19:16]==0) gain_text[47:40]=" ";
                end
            end
        end
    endfunction
    wire [3:0] row = py[6:3];
    wire [4:0] col = px[7:3];
    reg [7:0] character;
    reg [3:0] shown_index;
    reg [111:0] label_text;
    reg [19:0] shown_digits;
    reg [159:0] dashboard_line;
    reg selected_cell, disabled_cell;
    reg [3:0] group_base;
    integer text_pos;
    always @* begin
        character=" "; shown_index=0; label_text=0; shown_digits=0;
        dashboard_line="                    ";
        selected_cell=0; disabled_cell=0; group_base=0;
        case(row)
            0: begin
                label_text="IMU ATTITUDE  ";
                if(col<14) character=label_text[(13-col)*8 +: 8];
            end
            3,6,9: begin
                case(row)
                    3: begin label_text="ROLL          "; shown_index=0; end
                    6: begin label_text="PITCH         "; shown_index=1; end
                    9: begin label_text="YAW           "; shown_index=2; end
                endcase
                shown_digits=decimal_frame[shown_index];
                if(col<5) character=label_text[(13-col)*8 +: 8];
                else if(col>=7 && col<=13 && frame_imu_init) begin
                    case(col-7)
                        0: character=frame[shown_index][15] ? "-" : "+";
                        1: character={4'h3,shown_digits[19:16]};
                        2: character={4'h3,shown_digits[15:12]};
                        3: character={4'h3,shown_digits[11:8]};
                        4: character=".";
                        5: character={4'h3,shown_digits[7:4]};
                        6: character={4'h3,shown_digits[3:0]};
                    endcase
                end else if(col>=7 && col<=13 && !frame_imu_init) character="-";
            end
            12: begin
                label_text="UNITS: DEG    ";
                if(col<14) character=label_text[(13-col)*8 +: 8];
            end
            14: begin
                label_text=frame_imu_init ? "READY         " : "KEEP STILL    ";
                if(col<14) character=label_text[(13-col)*8 +: 8];
            end
            15: begin
                label_text="YAW RELATIVE  ";
                if(col<14) character=label_text[(13-col)*8 +: 8];
            end
            default: character=" ";
        endcase
        if(DASHBOARD_ENABLE) begin
            case(frame_page)
                0: begin
                    case(row)
                        0: dashboard_line="MOTOR SENSOR        ";
                        2,3,4,5: begin
                            case(row)
                                2: begin dashboard_line="LF C      S         "; shown_index=3; end
                                3: begin dashboard_line="RF C      S         "; shown_index=4; end
                                4: begin dashboard_line="LB C      S         "; shown_index=5; end
                                5: begin dashboard_line="RB C      S         "; shown_index=6; end
                            endcase
                            for(text_pos=0;text_pos<5;text_pos=text_pos+1)
                                dashboard_line[159-(4+text_pos)*8 -: 8]=number_char(decimal_frame[shown_index],text_pos+1,1'b0);
                            for(text_pos=0;text_pos<6;text_pos=text_pos+1)
                                dashboard_line[159-(11+text_pos)*8 -: 8]=number_char(decimal_frame[shown_index+4],text_pos,frame[shown_index+4][15]);
                        end
                        7: begin
                            dashboard_line=frame_tof_init ? "TOF       MM READY  " : "TOF       MM INIT   ";
                            for(text_pos=0;text_pos<5;text_pos=text_pos+1)
                                dashboard_line[159-(4+text_pos)*8 -: 8]=number_char(decimal_frame[11],text_pos+1,1'b0);
                        end
                        8,9,10: begin
                            case(row)
                                8: begin dashboard_line="ROLL                "; shown_index=0; end
                                9: begin dashboard_line="PITCH               "; shown_index=1; end
                                10: begin dashboard_line="YAW                 "; shown_index=2; end
                            endcase
                            shown_digits=decimal_frame[shown_index];
                            dashboard_line[103:48]=frame_imu_init ?
                                {frame[shown_index][15] ? 8'h2d : 8'h2b,
                                 {4'h3,shown_digits[19:16]},{4'h3,shown_digits[15:12]},
                                 {4'h3,shown_digits[11:8]},8'h2e,
                                 {4'h3,shown_digits[7:4]},{4'h3,shown_digits[3:0]}} : "-------";
                        end
                        12: dashboard_line=frame_imu_init ? "IMU READY           " : "IMU KEEP STILL      ";
                        13: begin
                            dashboard_line="FAULT 0000          ";
                            for(text_pos=0;text_pos<4;text_pos=text_pos+1)
                                dashboard_line[111-text_pos*8 -: 8]=frame_fault[3-text_pos] ? "1" : "0";
                        end
                        15: dashboard_line="S1PAGE              ";
                    endcase
                end
                1: begin
                    case(row)
                        0: dashboard_line="PID GAINS           ";
                        2,3,4,5,6,7,8,9,10,11: begin
                            case(row)
                                2,3: begin group_base=0; dashboard_line="LF P                "; end
                                4,5: begin group_base=3; dashboard_line="RF P                "; end
                                6,7: begin group_base=6; dashboard_line="LB P                "; end
                                8,9: begin group_base=9; dashboard_line="RB P                "; end
                                10,11: begin group_base=12; dashboard_line="HD P                "; end
                            endcase
                            if(!row[0]) begin
                                dashboard_line[127:64]=gain_text(gain_decimal[group_base]);
                                disabled_cell=group_base!=12 && col>=3 && col<=11;
                                selected_cell=group_base==12 && frame_selection==12 && col>=3 && col<=11;
                            end else begin
                                dashboard_line="I         D         ";
                                dashboard_line[151:88]=gain_text(gain_decimal[group_base+1]);
                                dashboard_line[71:8]=gain_text(gain_decimal[group_base+2]);
                                disabled_cell=group_base!=12 || col<=8;
                                selected_cell=group_base==12 && frame_selection==14 && col>=10 && col<=18;
                            end
                        end
                        12: dashboard_line="WHEEL READ ONLY     ";
                        13: dashboard_line="HD I LOCKED 0       ";
                        14: dashboard_line="S1PAGE S2SEL        ";
                        15: dashboard_line="S3+ S4-             ";
                    endcase
                end
                2: if(row==6) begin
                    if(frame_camera_fault) dashboard_line="CAM FAULT           ";
                    else if(!frame_camera_init) dashboard_line="CAM INIT            ";
                    else if(!frame_camera_valid) dashboard_line="CAM NO FRAME        ";
                end
                default: if(row==0) dashboard_line="SELECT PAGE 0 1 2   ";
            endcase
            if(TELEMETRY_PAGES) begin
                if(frame_page==0 && row==7 && !frame_tof_init)
                    dashboard_line="TOF ----- MM INIT   ";
                if(frame_page==2) begin
                    case(row)
                        0: dashboard_line="IMU RAW DATA        ";
                        2,3,4,6,7,8: begin
                            case(row)
                                2: begin dashboard_line="AX                  "; shown_index=3; end
                                3: begin dashboard_line="AY                  "; shown_index=4; end
                                4: begin dashboard_line="AZ                  "; shown_index=5; end
                                6: begin dashboard_line="GX                  "; shown_index=6; end
                                7: begin dashboard_line="GY                  "; shown_index=7; end
                                8: begin dashboard_line="GZ                  "; shown_index=8; end
                            endcase
                            for(text_pos=0;text_pos<6;text_pos=text_pos+1)
                                dashboard_line[159-(5+text_pos)*8 -: 8]=number_char(decimal_frame[shown_index],text_pos,frame[shown_index][15]);
                        end
                        10: dashboard_line=frame_raw_imu ? "IMU INIT OK         " : "IMU INIT WAIT       ";
                        11: dashboard_line=frame_tof_init ? "TOF INIT OK         " : "TOF INIT WAIT       ";
                        12: begin
                            dashboard_line="ENC HEX 00          ";
                            dashboard_line[95:88]=hex_char(frame_inputs[7:4]);
                            dashboard_line[87:80]=hex_char(frame_inputs[3:0]);
                        end
                        13: begin
                            dashboard_line="TOF ST 00 R 0000    ";
                            dashboard_line[103:96]=hex_char({2'b0,frame_tof_state[5:4]});
                            dashboard_line[95:88]=hex_char(frame_tof_state[3:0]);
                            for(text_pos=0;text_pos<4;text_pos=text_pos+1)
                                dashboard_line[159-(12+text_pos)*8 -: 8]=hex_char(frame_retries[15-text_pos*4 -: 4]);
                        end
                        15: dashboard_line="S1PAGE RAW COUNTS   ";
                        default: dashboard_line="                    ";
                    endcase
                end
                if(frame_page==3) begin
                    case(row)
                        0: dashboard_line="HEADING CONTROL     ";
                        1: dashboard_line="TARGET YAW +000.00  ";
                        2: begin
                            dashboard_line="YAW                 ";
                            shown_digits=decimal_frame[2];
                            dashboard_line[103:48]={frame[2][15] ? 8'h2d : 8'h2b,
                                {4'h3,shown_digits[19:16]},{4'h3,shown_digits[15:12]},
                                {4'h3,shown_digits[11:8]},8'h2e,
                                {4'h3,shown_digits[7:4]},{4'h3,shown_digits[3:0]}};
                        end
                        3,4,5,6,7,8,9,10: begin
                            case(row)
                                3: dashboard_line="TURN                ";
                                4: dashboard_line="LEFT T              ";
                                5: dashboard_line="RIGHT T             ";
                                6: dashboard_line="BASE T              ";
                                7: dashboard_line="LF DUTY             ";
                                8: dashboard_line="RF DUTY             ";
                                9: dashboard_line="LB DUTY             ";
                                10: dashboard_line="RB DUTY             ";
                            endcase
                            shown_index=row;
                            for(text_pos=0;text_pos<6;text_pos=text_pos+1)
                                dashboard_line[159-(9+text_pos)*8 -: 8]=number_char(decimal_frame[shown_index],text_pos,frame[shown_index][15]);
                        end
                        11: begin
                            dashboard_line="READY 0 IMU 0 HF 0  ";
                            dashboard_line[111:104]=frame_imu_init ? "1" : "0";
                            dashboard_line[63:56]=frame_raw_imu ? "1" : "0";
                            dashboard_line[23:16]=frame_heading_fault ? "1" : "0";
                        end
                        12: begin
                            dashboard_line="FAULT 0000          ";
                            for(text_pos=0;text_pos<4;text_pos=text_pos+1)
                                dashboard_line[111-text_pos*8 -: 8]=frame_fault[3-text_pos] ? "1" : "0";
                        end
                        13: begin
                            dashboard_line="WHY 0000            ";
                            for(text_pos=0;text_pos<4;text_pos=text_pos+1)
                                dashboard_line[159-(4+text_pos)*8 -: 8]=hex_char(frame_reasons[15-text_pos*4 -: 4]);
                        end
                        14: begin
                            dashboard_line="CAL 0 ER 0 OUT 0    ";
                            dashboard_line[127:120]=hex_char(frame_cal);
                            dashboard_line[87:80]=hex_char(frame_sign);
                            dashboard_line[39:32]=frame_outputs ? "1" : "0";
                        end
                        15: dashboard_line="S1PAGE DUTY/2941    ";
                        default: dashboard_line="                    ";
                    endcase
                end
            end
            if(TELEMETRY_PAGES && CAMERA_PAGE_ENABLE && frame_page==4) begin
                dashboard_line="                    ";
                if(row==0) dashboard_line="CAMERA 160X128      ";
                if(row==6) begin
                    if(frame_camera_fault) dashboard_line="CAM CONFIG FAULT    ";
                    else if(!frame_camera_init) dashboard_line="CAM INITIALIZING    ";
                    else if(!frame_camera_valid) dashboard_line="CAM WAIT FRAME      ";
                end
                if(row==15) dashboard_line="S1PAGE RGB565       ";
            end
            if(col<20) character=dashboard_line[159-col*8 -: 8];
            else character=" ";
        end
    end
    wire font_pixel;
    // Split row/text selection from font decoding; FETCH already provides a
    // cycle for camera RAM, so this pipeline adds no pixel or SPI latency.
    reg [7:0] font_character;
    reg [2:0] font_x,font_y;
    reg [3:0] font_row;
    reg font_selected,font_disabled;
    tft180_font u_font(.ascii(font_character), .x(font_x), .y(font_y), .pixel(font_pixel));
    reg [15:0] color;
    always @* begin
        if (!font_pixel) color = 16'h0000;
        else if (font_row == 0) color = 16'h07ff;
        else if (font_row == 14 && !frame_imu_init) color = 16'hf800;
        else color = 16'hffff;
        if(DASHBOARD_ENABLE) begin
            if(font_selected && !font_disabled) color=font_pixel ? 16'h0000 : 16'hffe0;
            else if(!font_pixel) color=16'h0000;
            else if(font_disabled) color=16'h7bef;
            else if(font_row==0) color=16'h07ff;
            else if((frame_page==0 && ((font_row==13 && frame_fault!=0) || (font_row==12 && !frame_imu_init))) ||
                    (camera_page && frame_camera_fault)) color=16'hf800;
            else color=16'hffff;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state<=RESET_LOW; timer<=0; init_index<=0; window_index<=0;
            tft_reset<=0; tft_dc<=0; tx_start<=0; tx_last<=1; tx_data<=0;
            px<=0; py<=0; low_byte<=0; ready<=0;
            frame_imu_init<=0; frame_tof_init<=0; frame_closed_loop<=0; frame_fault<=0;
            frame_raw_imu<=0; frame_heading_fault<=0; frame_outputs<=0;
            frame_reasons<=0; frame_retries<=0; frame_cal<=0; frame_sign<=0;
            frame_inputs<=0; frame_tof_state<=0;
            decimal_work<=0; binary_work<=0; convert_index<=0; convert_bit<=0;
            frame_page<=0; frame_selection<=0;
            frame_camera_valid<=0; frame_camera_init<=0; frame_camera_fault<=0;
            lidar_frame_start<=0;
            camera_frame_start<=0; camera_frame_end<=0; pixel_latched<=0;
            font_character<=0; font_x<=0; font_y<=0; font_row<=0; font_selected<=0; font_disabled<=0;
            scale_a<=0; scale_b<=0; scale_result<=0;
            gain_binary<=0; gain_work<=0;
            for(k=0;k<15;k=k+1) begin frame_gain[k]<=0; gain_decimal[k]<=0; end
            for (k=0; k<16; k=k+1) begin frame[k]<=0; decimal_frame[k]<=0; end
        end else begin
            tx_start<=0;
            lidar_frame_start<=0;
            camera_frame_start<=0; camera_frame_end<=0;
            case (state)
                RESET_LOW: if (timer >= CLK_FREQ/1000*RESET_MS-1) begin
                    timer<=0; tft_reset<=1; state<=STARTUP;
                end else timer<=timer+1'b1;
                STARTUP: if (timer >= CLK_FREQ/1000*STARTUP_MS-1) begin
                    timer<=0; state<=INIT_SEND;
                end else timer<=timer+1'b1;
                INIT_SEND: begin
                    tx_data<=init_entry[7:0]; tft_dc<=init_entry[8]; tx_start<=1;
                    tx_last<=(init_index==0 || init_index==77); state<=INIT_WAIT;
                end
                INIT_WAIT: if (tx_done) begin
                    if (init_index==0) begin init_index<=1; timer<=0; state<=SLEEP_WAIT; end
                    else if (init_index==77) state<=FRAME_START;
                    else begin init_index<=init_index+1'b1; state<=INIT_SEND; end
                end
                SLEEP_WAIT: if (timer >= CLK_FREQ/1000*SLEEP_MS-1) begin
                    timer<=0; state<=INIT_SEND;
                end else timer<=timer+1'b1;
                FRAME_START: begin
                    frame[0]<=attitude_roll; frame[1]<=attitude_pitch; frame[2]<=attitude_yaw;
                    for(k=3;k<16;k=k+1) frame[k]<=0;
                    frame_imu_init<=attitude_ready;
                    if(DASHBOARD_ENABLE) begin
                        frame_page<=page; frame_selection<=selection;
                        // Independent scan RAM may publish on every TFT boundary,
                        // keeping the background radar current on the other pages.
                        lidar_frame_start<=LIDAR_PAGE_ENABLE;
                        frame[3]<=lf_count; frame[4]<=rf_count; frame[5]<=lb_count; frame[6]<=rb_count;
                        frame[7]<=lf_speed; frame[8]<=rf_speed; frame[9]<=lb_speed; frame[10]<=rb_speed;
                        frame[11]<=distance_mm;
                        frame_tof_init<=tof_init; frame_fault<=fault_mask;
                        frame_raw_imu<=imu_init; frame_heading_fault<=heading_fault;
                        frame_outputs<=outputs_enabled; frame_reasons<=fault_reasons;
                        frame_cal<=calibration_mask; frame_sign<=encoder_sign_mask;
                        frame_inputs<=encoder_inputs; frame_tof_state<=tof_state;
                        frame_retries<=tof_retries;
                        if(TELEMETRY_PAGES && page==2) begin
                            frame[3]<=acc_x; frame[4]<=acc_y; frame[5]<=acc_z;
                            frame[6]<=gyro_x; frame[7]<=gyro_y; frame[8]<=gyro_z;
                        end
                        if(TELEMETRY_PAGES && page==3) begin
                            frame[3]<=heading_turn; frame[4]<=heading_left; frame[5]<=heading_right;
                            frame[6]<=target_speed; frame[7]<=lf_duty; frame[8]<=rf_duty;
                            frame[9]<=lb_duty; frame[10]<=rb_duty;
                        end
                        frame_camera_valid<=camera_valid; frame_camera_init<=camera_init_done;
                        frame_camera_fault<=camera_fault;
                        camera_frame_start<=camera_requested;
                        for(k=0;k<15;k=k+1)
                            frame_gain[k]<=k==13 ? 32'd0 : gains_flat[k*32 +: 32];
                    end
                    px<=0; py<=0; low_byte<=0; window_index<=0;
                    convert_index<=0; state<=BCD_PREP;
                end
                BCD_PREP: begin
                    binary_work <= (frame[convert_index][15] &&
                        (DASHBOARD_ENABLE ? (convert_index<3 ||
                            (TELEMETRY_PAGES && frame_page==2 ? (convert_index>=3 && convert_index<=8) :
                            TELEMETRY_PAGES && frame_page==3 ? (convert_index>=3 && convert_index<=10) :
                            (convert_index>=7 && convert_index<=10))) : convert_index!=14))
                        ? (~frame[convert_index]+16'd1) : frame[convert_index];
                    decimal_work<=0; convert_bit<=0; state<=BCD_SHIFT;
                end
                BCD_SHIFT: begin
                    decimal_work<={decimal_adjusted[18:0],binary_work[15]};
                    binary_work<={binary_work[14:0],1'b0};
                    if (convert_bit==15) state<=BCD_STORE;
                    else convert_bit<=convert_bit+1'b1;
                end
                BCD_STORE: begin
                    decimal_frame[convert_index]<=decimal_work;
                    if (convert_index==15) begin
                        if(DASHBOARD_ENABLE && frame_page==1) begin convert_index<=0; state<=GAIN_PREP; end
                        else state<=WINDOW_SEND;
                    end
                    else begin convert_index<=convert_index+1'b1; state<=BCD_PREP; end
                end
                GAIN_PREP: begin
                    // Widen BEFORE shifting. Two registered partial sums bound the
                    // scaling path; no run-time multiply/divide or DSP required.
                    if(convert_index==12) begin
                        scale_a<=({17'd0,frame_gain[convert_index]}<<16)+({17'd0,frame_gain[convert_index]}<<15);
                        scale_b<=({17'd0,frame_gain[convert_index]}<<10)+({17'd0,frame_gain[convert_index]}<<9)+
                                 ({17'd0,frame_gain[convert_index]}<<7)+({17'd0,frame_gain[convert_index]}<<5);
                    end else begin
                        scale_a<=({17'd0,frame_gain[convert_index]}<<10);
                        scale_b<=({17'd0,frame_gain[convert_index]}<<4)+({17'd0,frame_gain[convert_index]}<<3);
                    end
                    state<=GAIN_SCALE;
                end
                GAIN_SCALE: begin
                    scale_result<=convert_index==12 ? scale_a+scale_b : scale_a-scale_b;
                    state<=GAIN_BINARY;
                end
                GAIN_BINARY: begin
                    gain_binary<=scale_result>>12; gain_work<=0; convert_bit<=0; state<=GAIN_SHIFT;
                end
                GAIN_SHIFT: begin
                    gain_work<={gain_adjusted[30:0],gain_binary[31]};
                    gain_binary<={gain_binary[30:0],1'b0};
                    if(convert_bit==31) state<=GAIN_STORE;
                    else convert_bit<=convert_bit+1'b1;
                end
                GAIN_STORE: begin
                    gain_decimal[convert_index]<=gain_work;
                    if(convert_index==14) state<=WINDOW_SEND;
                    else begin convert_index<=convert_index+1'b1; state<=GAIN_PREP; end
                end
                WINDOW_SEND: begin
                    tx_data<=window_entry[7:0]; tft_dc<=window_entry[8];
                    tx_last<=0; tx_start<=1; state<=WINDOW_WAIT;
                end
                WINDOW_WAIT: if (tx_done) begin
                    if (window_index==10) state<=PIXEL_FETCH;
                    else begin window_index<=window_index+1'b1; state<=WINDOW_SEND; end
                end
                // Address is stable for this clock; synchronous RAM returns its
                // word after this edge. Capture on the following edge.
                PIXEL_FETCH: begin
                    font_character<=character; font_x<=px[2:0]; font_y<=py[2:0];
                    font_row<=row; font_selected<=selected_cell; font_disabled<=disabled_cell;
                    state<=lidar_page?NAV_PIPE_WAIT:PIXEL_LATCH;
                end
                NAV_PIPE_WAIT: state<=PIXEL_LATCH;
                PIXEL_LATCH: begin
                    pixel_latched<=lidar_page ? lidar_pixel : (camera_image ? camera_pixel : color);
                    state<=PIXEL_SEND;
                end
                PIXEL_SEND: begin
                    tx_data<=low_byte ? pixel_latched[7:0] : pixel_latched[15:8]; tft_dc<=1;
                    tx_last<=low_byte && px==159 && py==127;
                    tx_start<=1; state<=PIXEL_WAIT;
                end
                PIXEL_WAIT: if (tx_done) begin
                    if (!low_byte) begin low_byte<=1; state<=PIXEL_SEND; end
                    else begin
                        low_byte<=0;
                        if (px==159 && py==127) begin
                            ready<=1; timer<=0; state<=FRAME_GAP;
                            camera_frame_end<=DASHBOARD_ENABLE && camera_page;
                        end
                        else begin
                            if (px==159) begin px<=0; py<=py+1'b1; end
                            else px<=px+1'b1;
                            state<=PIXEL_FETCH;
                        end
                    end
                end
                FRAME_GAP: if (lidar_page || timer >= CLK_FREQ/1000*FRAME_GAP_MS-1) begin
                    timer<=0; state<=FRAME_START;
                end else timer<=timer+1'b1;
                default: state<=RESET_LOW;
            endcase
        end
    end
endmodule
