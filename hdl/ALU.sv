module ALU (
    input clk, reset, flush, input valid_in, input [4:0] op, input [63:0] a, b,
    input [5:0] tag_in, input [4:0] rd_in, output ready_in, output reg valid_out,
    output reg [63:0] res_out, output reg [5:0] tag_out, output reg [4:0] rd_out, input ack_out
);
    assign ready_in = !valid_out || ack_out;
    always @(posedge clk) begin
        if (reset || flush) valid_out <= 0;
        else begin
            if (ack_out) valid_out <= 0;
            if (valid_in && ready_in) begin
                valid_out <= 1; tag_out <= tag_in; rd_out <= rd_in;
                case (op)
                    5'h00: res_out <= a & b; 5'h01: res_out <= a | b; 5'h02: res_out <= a ^ b; 5'h03: res_out <= ~a;
                    5'h18, 5'h19, 5'h10, 5'h13: res_out <= a + b;
                    5'h1a, 5'h1b: res_out <= a - b; 5'h1c: res_out <= a * b; 5'h1d: res_out <= a / b;
                    5'h04, 5'h05: res_out <= a >> b; 5'h06, 5'h07: res_out <= a << b;
                    5'h11: res_out <= a; 5'h12: res_out <= {a[63:12], b[11:0]}; default: res_out <= 64'b0;
                endcase
            end
        end
    end
endmodule