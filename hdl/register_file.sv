module register_file (
    input clk,
    input reset,
    
    // --- Single Write Port (From CDB or Memory) ---
    input write_enable,
    input [4:0] rd,
    input [63:0] data,
    
    // --- Instruction A Read Ports ---
    input [4:0] rd_A, rs_A, rt_A,
    output [63:0] rd_val_A, rs_val_A, rt_val_A,
    
    // --- Instruction B Read Ports ---
    input [4:0] rs_B, rt_B,
    output [63:0] rs_val_B, rt_val_B,
    
    // --- System Port ---
    output [63:0] r31_val
);
    reg [63:0] registers [0:31];

    // Instruction A
    assign rd_val_A = registers[rd_A];
    assign rs_val_A = registers[rs_A];
    assign rt_val_A = registers[rt_A];
    
    // Instruction B
    assign rs_val_B = registers[rs_B];
    assign rt_val_B = registers[rt_B];
    
    // System
    assign r31_val = registers[31];
    
    integer i;
    always @(posedge clk) begin
        if (reset) begin
            for (i = 0; i < 31; i = i + 1) begin
                registers[i] <= 64'b0;
            end
            registers[31] <= 64'd524288;
        end else if (write_enable && rd != 0) begin
            registers[rd] <= data;
        end
    end
endmodule