// Asynchronous assertion, two-clock synchronous release. No RAM reset.
module ov5640_reset_sync(input wire clk, input wire rst_n, output wire out_n);
    (* ASYNC_REG = "TRUE" *) reg [1:0] sync;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) sync <= 2'b00;
        else sync <= {sync[0], 1'b1};
    assign out_n = sync[1];
endmodule
