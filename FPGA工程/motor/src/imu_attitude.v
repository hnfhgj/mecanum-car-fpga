// Six-axis quaternion complementary filter, adapted from fly_across pose.c.
// Q20; measured dt; gyro 16.4 LSB/(degree/s), accel 4096 LSB/g.
// No magnetometer: yaw is relative and will drift. Outputs: degree * 100.
module imu_attitude #(
    parameter integer CLK_FREQ=50000000,
    parameter integer CAL_SAMPLES=256,
    parameter signed [31:0] CORRECTION_KP=681574 // pose.c error_kp=.65, Ki=0
)(
    input wire clk,rst_n,sample_valid,
    input wire signed [15:0] ax,ay,az,gx,gy,gz,
    output reg signed [15:0] roll,pitch,yaw,
    output reg angle_valid,
    output wire ready
);
    localparam signed [31:0] ONE=1048576;
    function signed [31:0] mulq;
        // All enabled-path operands fit signed 27 bits (Q20 range +/-64).
        // Match the GW5AT native 27x36 multiplier instead of LUT multipliers.
        input signed [26:0] a;
        input signed [31:0] b;
        reg signed [58:0] p;
        begin p=a*b; mulq=p>>>20; end
    endfunction
    reg [7:0] state;
    reg [15:0] cal_count;
    reg signed [31:0] sum_x,sum_y,sum_z,bias_x,bias_y,bias_z;
    reg calibrated;
    reg [31:0] us_div,sample_us;
    reg signed [31:0] half_dt;
    reg healthy;
    wire signed [63:0] dt_product=$signed({1'b0,sample_us})*32'sd549756;
    // Register the freshness gate before it fans out to four motor fault paths.
    assign ready=healthy;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) healthy<=0;
        else healthy<=calibrated && sample_us<100000;
    end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin us_div<=0; sample_us<=0; end
        else begin
            if(us_div>=CLK_FREQ/1000000-1) begin
                us_div<=0;
                if(sample_us<1000000) sample_us<=sample_us+1'b1;
            end else us_div<=us_div+1'b1;
            if(sample_valid) begin sample_us<=0; us_div<=0; end
        end
    end
    reg signed [31:0] a[0:2],u[0:2],v[0:2],e[0:2],ef[0:2];
    reg signed [31:0] raw_g[0:2],rate[0:2],step_g[0:2];
    reg signed [31:0] q[0:3],nq[0:3];
    reg signed [31:0] qp[0:9],cross_p[0:5];
    reg signed [31:0] product_q[0:3],norm_product;
    reg output_products;
    reg signed [31:0] sq[0:3],norm2,inv,inv_sq,factor;
    reg signed [31:0] inv_square_operand,inv_update_operand,inv_normalize_operand;
    reg acc_ok;
    reg [1:0] inverse_iteration;
    reg signed [31:0] yaw_x,yaw_y;
    reg signed [15:0] roll_work,pitch_work;
    reg cordic_start;
    reg signed [31:0] cordic_x,cordic_y;
    wire cordic_done;
    wire signed [15:0] cordic_angle;
    wire signed [31:0] cordic_magnitude;
    attitude_cordic u_cordic(.clk(clk),.rst_n(rst_n),.start(cordic_start),
        .in_x(cordic_x),.in_y(cordic_y),.done(cordic_done),
        .angle(cordic_angle),.magnitude(cordic_magnitude));
    integer i;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            state<=0; cal_count<=0; calibrated<=0;
            sum_x<=0; sum_y<=0; sum_z<=0; bias_x<=0; bias_y<=0; bias_z<=0;
            roll<=0; pitch<=0; yaw<=0; angle_valid<=0;
            half_dt<=0; norm2<=ONE; inv<=ONE; inv_sq<=ONE; factor<=ONE;
            acc_ok<=0; inverse_iteration<=0; yaw_x<=ONE; yaw_y<=0;
            cordic_start<=0; cordic_x<=0; cordic_y<=0; roll_work<=0; pitch_work<=0;
            for(i=0;i<3;i=i+1) begin a[i]<=0;u[i]<=0;v[i]<=0;e[i]<=0;ef[i]<=0;raw_g[i]<=0;rate[i]<=0;step_g[i]<=0;end
            for(i=0;i<4;i=i+1) begin q[i]<=(i==0)?ONE:0; nq[i]<=0; sq[i]<=0; end
            for(i=0;i<10;i=i+1) qp[i]<=0;
            for(i=0;i<6;i=i+1) cross_p[i]<=0;
            for(i=0;i<4;i=i+1) product_q[i]<=0;
            norm_product<=0; output_products<=0;
            inv_square_operand<=ONE; inv_update_operand<=ONE; inv_normalize_operand<=ONE;
        end else begin
            angle_valid<=0; cordic_start<=0;
            case(state)
                0: if(sample_valid) begin
                    if(!calibrated) begin
                        // Require a quiet sensor; moving during calibration restarts averaging.
                        if(gx>164 || gx< -164 || gy>164 || gy< -164 || gz>164 || gz< -164) begin
                            cal_count<=0; sum_x<=0; sum_y<=0; sum_z<=0;
                        end else if(cal_count==CAL_SAMPLES-1) begin
                            bias_x<=(sum_x+gx)/CAL_SAMPLES;
                            bias_y<=(sum_y+gy)/CAL_SAMPLES;
                            bias_z<=(sum_z+gz)/CAL_SAMPLES;
                            calibrated<=1;
                        end else begin
                            sum_x<=sum_x+gx; sum_y<=sum_y+gy; sum_z<=sum_z+gz;
                            cal_count<=cal_count+1'b1;
                        end
                    end else if(sample_us>=1000 && sample_us<=20000) begin
                        a[0]<=$signed(ax)*32'sd256; a[1]<=$signed(ay)*32'sd256; a[2]<=$signed(az)*32'sd256;
                        raw_g[0]<=$signed(gx)-bias_x; raw_g[1]<=$signed(gy)-bias_y; raw_g[2]<=$signed(gz)-bias_z;
                        half_dt<=dt_product>>>20;
                        state<=1;
                    end
                end
                1: begin for(i=0;i<3;i=i+1) sq[i]<=mulq(a[i],a[i]); state<=2; end
                2: begin norm2<=sq[0]+sq[1]+sq[2]; inv<=ONE; inverse_iteration<=0; state<=26; end
                26: begin inv_square_operand<=inv; state<=3; end
                3: begin acc_ok<=norm2>671089 && norm2<1509949; inv_sq<=mulq(inv_square_operand,inv_square_operand); state<=4; end
                4: begin norm_product<=mulq(norm2,inv_sq); state<=24; end
                24: begin factor<=(3*ONE-norm_product)>>>1; inv_update_operand<=inv; state<=5; end
                5: begin
                    inv<=mulq(inv_update_operand,factor);
                    if(inverse_iteration==2 || !acc_ok) state<=6;
                    else begin inverse_iteration<=inverse_iteration+1'b1; state<=26; end
                end
                6: begin inv_normalize_operand<=inv; state<=27; end
                27: begin for(i=0;i<3;i=i+1) u[i]<=acc_ok ? mulq(a[i],inv_normalize_operand) : 0; state<=7; end
                7,16: begin
                    for(i=0;i<4;i=i+1) product_q[i]<=q[i];
                    output_products<=(state==16); state<=25;
                end
                25: begin
                    // Shared quaternion products, separated from sums/subtractions.
                    qp[0]<=mulq(product_q[0],product_q[0]); qp[1]<=mulq(product_q[1],product_q[1]);
                    qp[2]<=mulq(product_q[2],product_q[2]); qp[3]<=mulq(product_q[3],product_q[3]);
                    qp[4]<=mulq(product_q[1],product_q[3]); qp[5]<=mulq(product_q[0],product_q[2]);
                    qp[6]<=mulq(product_q[0],product_q[1]); qp[7]<=mulq(product_q[2],product_q[3]);
                    qp[8]<=mulq(product_q[0],product_q[3]); qp[9]<=mulq(product_q[1],product_q[2]);
                    state<=output_products ? 22 : 21;
                end
                21: begin
                    v[0]<=2*(qp[4]-qp[5]); v[1]<=2*(qp[6]+qp[7]);
                    v[2]<=qp[0]-qp[1]-qp[2]+qp[3]; state<=8;
                end
                8: begin
                    cross_p[0]<=mulq(u[1],v[2]); cross_p[1]<=mulq(u[2],v[1]);
                    cross_p[2]<=mulq(u[2],v[0]); cross_p[3]<=mulq(u[0],v[2]);
                    cross_p[4]<=mulq(u[0],v[1]); cross_p[5]<=mulq(u[1],v[0]);
                    state<=23;
                end
                23: begin
                    e[0]<=cross_p[0]-cross_p[1]; e[1]<=cross_p[2]-cross_p[3];
                    e[2]<=cross_p[4]-cross_p[5]; state<=9;
                end
                9: begin
                    for(i=0;i<3;i=i+1) ef[i]<=acc_ok ? ef[i]+mulq(e[i]-ef[i],half_dt*32'sd6) : 0;
                    state<=10;
                end
                10: begin
                    for(i=0;i<3;i=i+1) rate[i]<=raw_g[i]*32'sd1116+mulq(ef[i],CORRECTION_KP);
                    state<=11;
                end
                11: begin for(i=0;i<3;i=i+1) step_g[i]<=mulq(rate[i],half_dt); state<=12; end
                12: begin
                    nq[0]<=q[0]-mulq(q[1],step_g[0])-mulq(q[2],step_g[1])-mulq(q[3],step_g[2]);
                    nq[1]<=q[1]+mulq(q[0],step_g[0])+mulq(q[2],step_g[2])-mulq(q[3],step_g[1]);
                    nq[2]<=q[2]+mulq(q[0],step_g[1])-mulq(q[1],step_g[2])+mulq(q[3],step_g[0]);
                    nq[3]<=q[3]+mulq(q[0],step_g[2])+mulq(q[1],step_g[1])-mulq(q[2],step_g[0]);
                    state<=13;
                end
                13: begin for(i=0;i<4;i=i+1) sq[i]<=mulq(nq[i],nq[i]); state<=14; end
                14: begin factor<=(3*ONE-sq[0]-sq[1]-sq[2]-sq[3])>>>1; state<=15; end
                15: begin for(i=0;i<4;i=i+1) q[i]<=mulq(nq[i],factor); state<=16; end
                22: begin
                    v[0]<=2*(qp[4]-qp[5]); v[1]<=2*(qp[6]+qp[7]);
                    v[2]<=ONE-2*(qp[1]+qp[2]);
                    yaw_x<=ONE-2*(qp[2]+qp[3]); yaw_y<=2*(qp[8]+qp[9]);
                    state<=17;
                end
                17: begin cordic_x<=v[2]; cordic_y<=v[1]; cordic_start<=1; state<=18; end
                18: if(cordic_done) begin
                    roll_work<=cordic_angle; cordic_x<=cordic_magnitude; cordic_y<=-v[0];
                    cordic_start<=1; state<=19;
                end
                19: if(cordic_done) begin
                    pitch_work<=cordic_angle; cordic_x<=yaw_x; cordic_y<=yaw_y;
                    cordic_start<=1; state<=20;
                end
                20: if(cordic_done) begin
                    roll<=roll_work; pitch<=pitch_work; yaw<=cordic_angle;
                    angle_valid<=1; state<=0;
                end
                default: state<=0;
            endcase
        end
    end
endmodule
