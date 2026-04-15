module branch_unit (
    input clk, reset, flush,
    
    input valid_in,
    input [4:0] op, input [63:0] pc, input [63:0] predicted_target,
    input [63:0] a_val, b_val, // rs and rt
    input [63:0] imm,
    input [5:0] tag_in, input [4:0] rd_in,
    output ready_in, 

    output reg valid_out,
    output reg mispredicted,
    output reg [63:0] correct_target,
    output reg actual_taken,
    output reg [63:0] branch_pc,
    
    output reg [63:0] res_out, // For Call instructions saving PC
    output reg [5:0] tag_out,
    output reg [4:0] rd_out,
    input ack_out
);
    assign ready_in = !valid_out || ack_out;

    always @(posedge clk) begin
        if (reset || flush) valid_out <= 0;
        else begin
            if (ack_out) valid_out <= 0;

            if (valid_in && ready_in) begin
                valid_out <= 1; tag_out <= tag_in; rd_out <= rd_in;
                branch_pc <= pc;
                res_out <= pc + 4; // Used for saving Return Address on Calls
                
                actual_taken = 0; correct_target = pc + 4;

                case (op)
                    5'h08: begin actual_taken = 1; correct_target = a_val; end // jmp
                    5'h09: begin actual_taken = 1; correct_target = (pc - 4) + a_val; end // jr
                    5'h0a: begin actual_taken = 1; correct_target = (pc - 4) + imm; end // jri
                    5'h0b: begin if (a_val != 0) begin actual_taken = 1; correct_target = a_val; end end // bnz
                    5'h0c: begin actual_taken = 1; correct_target = a_val; end // call
                    5'h0e: begin if (a_val > b_val) begin actual_taken = 1; correct_target = a_val; end end // bgt
                endcase

                mispredicted = (correct_target != predicted_target);
            end
        end
    end
endmodule