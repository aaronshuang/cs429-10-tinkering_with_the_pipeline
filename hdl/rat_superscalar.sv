module rat_superscalar (
    input clk, reset,
    
    // --- Instruction A Ports ---
    input [4:0] rs_A, rt_A, rd_A,
    output val_rs_A, val_rt_A,
    output [5:0] tag_rs_A, tag_rt_A,
    input alloc_A, input [5:0] alloc_tag_A,

    // --- Instruction B Ports ---
    input [4:0] rs_B, rt_B, rd_B,
    output val_rs_B, val_rt_B,
    output [5:0] tag_rs_B, tag_rt_B,
    input alloc_B, input [5:0] alloc_tag_B,

    // --- Common Data Bus (CDB) ---
    input cdb_valid,
    input [5:0] cdb_tag,
    input [4:0] cdb_rd
);
    reg valid [31:0];
    reg [5:0] tags [31:0];
    integer i;

    // ==========================================
    // INSTRUCTION A READS (Normal Lookup)
    // ==========================================
    assign val_rs_A = (rs_A == 0) ? 1'b1 : valid[rs_A];
    assign val_rt_A = (rt_A == 0) ? 1'b1 : valid[rt_A];
    assign tag_rs_A = tags[rs_A];
    assign tag_rt_A = tags[rt_A];

    // ==========================================
    // INSTRUCTION B READS (Intra-cycle Forwarding)
    // ==========================================
    // If B reads the register that A is writing IN THE SAME CYCLE, 
    // B must receive A's new allocated tag, not the old tag in the table!
    wire B_reads_A_rs = (alloc_A && rd_A == rs_B && rs_B != 0);
    wire B_reads_A_rt = (alloc_A && rd_A == rt_B && rt_B != 0);

    assign val_rs_B = B_reads_A_rs ? 1'b0 : (rs_B == 0 ? 1'b1 : valid[rs_B]);
    assign tag_rs_B = B_reads_A_rs ? alloc_tag_A : tags[rs_B];

    assign val_rt_B = B_reads_A_rt ? 1'b0 : (rt_B == 0 ? 1'b1 : valid[rt_B]);
    assign tag_rt_B = B_reads_A_rt ? alloc_tag_A : tags[rt_B];

    // ==========================================
    // WRITE LOGIC
    // ==========================================
    always @(posedge clk) begin
        if (reset) begin
            for (i=0; i<32; i=i+1) begin
                valid[i] <= 1'b1; tags[i] <= 6'b0;
            end
        end else begin
            // 1. CDB Broadcast (clears tag if it matches)
            if (cdb_valid && cdb_rd != 0) begin
                if (!valid[cdb_rd] && tags[cdb_rd] == cdb_tag) begin
                    valid[cdb_rd] <= 1'b1;
                end
            end

            // 2. Allocations (Instruction B overrides A if they write to the exact same register)
            if (alloc_A && rd_A != 0) begin
                valid[rd_A] <= 1'b0; tags[rd_A] <= alloc_tag_A;
            end
            if (alloc_B && rd_B != 0) begin
                valid[rd_B] <= 1'b0; tags[rd_B] <= alloc_tag_B;
            end
        end
    end
endmodule