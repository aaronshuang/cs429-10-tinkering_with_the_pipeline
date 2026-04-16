module rat_superscalar (
    input clk, reset, flush,
    input [4:0] rs_A, rt_A, rd_A, output val_rs_A, val_rt_A, val_rd_A, output [5:0] tag_rs_A, tag_rt_A, tag_rd_A, input alloc_A, input [5:0] alloc_tag_A, input [4:0] alloc_rd_A,
    input [4:0] rs_B, rt_B, rd_B, output val_rs_B, val_rt_B, val_rd_B, output [5:0] tag_rs_B, tag_rt_B, tag_rd_B, input alloc_B, input [5:0] alloc_tag_B, input [4:0] alloc_rd_B,
    input commit_A_valid, input [5:0] commit_A_tag, input [4:0] commit_A_rd,
    input commit_B_valid, input [5:0] commit_B_tag, input [4:0] commit_B_rd
);
    reg valid [31:0]; reg [5:0] tags [31:0]; integer i;

    assign val_rs_A = (rs_A == 0) ? 1'b1 : valid[rs_A]; assign val_rt_A = (rt_A == 0) ? 1'b1 : valid[rt_A]; assign val_rd_A = (rd_A == 0) ? 1'b1 : valid[rd_A];
    assign tag_rs_A = tags[rs_A]; assign tag_rt_A = tags[rt_A]; assign tag_rd_A = tags[rd_A];

    wire B_reads_A_rs = (alloc_A && rd_A == rs_B && rs_B != 0); wire B_reads_A_rt = (alloc_A && rd_A == rt_B && rt_B != 0); wire B_reads_A_rd = (alloc_A && rd_A == rd_B && rd_B != 0);
    assign val_rs_B = B_reads_A_rs ? 1'b0 : (rs_B == 0 ? 1'b1 : valid[rs_B]); assign tag_rs_B = B_reads_A_rs ? alloc_tag_A : tags[rs_B];
    assign val_rt_B = B_reads_A_rt ? 1'b0 : (rt_B == 0 ? 1'b1 : valid[rt_B]); assign tag_rt_B = B_reads_A_rt ? alloc_tag_A : tags[rt_B];
    assign val_rd_B = B_reads_A_rd ? 1'b0 : (rd_B == 0 ? 1'b1 : valid[rd_B]); assign tag_rd_B = B_reads_A_rd ? alloc_tag_A : tags[rd_B]; 

    always @(posedge clk) begin
        if (reset || flush) begin
            for (i=0; i<32; i=i+1) begin valid[i] <= 1'b1; tags[i] <= 6'b0; end
        end else begin
            if (commit_A_valid && commit_A_rd != 0) if (!valid[commit_A_rd] && tags[commit_A_rd] == commit_A_tag) valid[commit_A_rd] <= 1'b1;
            if (commit_B_valid && commit_B_rd != 0) if (!valid[commit_B_rd] && tags[commit_B_rd] == commit_B_tag) valid[commit_B_rd] <= 1'b1;
            if (alloc_A && alloc_rd_A != 0) begin valid[alloc_rd_A] <= 1'b0; tags[alloc_rd_A] <= alloc_tag_A; end
            if (alloc_B && alloc_rd_B != 0) begin valid[alloc_rd_B] <= 1'b0; tags[alloc_rd_B] <= alloc_tag_B; end
        end
    end
endmodule