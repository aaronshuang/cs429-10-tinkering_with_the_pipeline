module register_file (
    input clk,
    input reset,
    // Dual Write Ports (From ROB Commits)
    input write_enable_1, input [4:0] rd_w1, input [63:0] data_1,
    input write_enable_2, input [4:0] rd_w2, input [63:0] data_2,
    // Instruction A Read Ports
    input [4:0] rd_A, rs_A, rt_A,
    output [63:0] rd_val_A, rs_val_A, rt_val_A,
    // Instruction B Read Ports
    input [4:0] rd_B, rs_B, rt_B,
    output [63:0] rd_val_B, rs_val_B, rt_val_B,
    // System Port
    output [63:0] r31_val
);
    reg [63:0] registers [0:31];
    assign rd_val_A = registers[rd_A];
    assign rs_val_A = registers[rs_A];
    assign rt_val_A = registers[rt_A];
    assign rd_val_B = registers[rd_B];
    assign rs_val_B = registers[rs_B];
    assign rt_val_B = registers[rt_B];
    assign r31_val  = registers[31];

    integer i;
    always @(posedge clk) begin
        if (reset) begin
            for (i = 0; i < 31; i = i + 1) registers[i] <= 64'b0;
            registers[31] <= 64'd524288;
        end else begin
            if (write_enable_1 && rd_w1 != 0) registers[rd_w1] <= data_1;
            if (write_enable_2 && rd_w2 != 0) registers[rd_w2] <= data_2;
        end
    end
endmodule