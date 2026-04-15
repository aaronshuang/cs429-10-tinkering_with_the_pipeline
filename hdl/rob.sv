module reorder_buffer #(parameter DEPTH = 64) (
    input clk, reset, flush,
    input alloc_A, input [4:0] rd_A,
    input alloc_B, input [4:0] rd_B,
    output wire [5:0] tag_A, tag_B, output wire full, output wire empty,
    input cdb1_valid, input [5:0] cdb1_tag, input [63:0] cdb1_data,
    input cdb2_valid, input [5:0] cdb2_tag, input [63:0] cdb2_data,
    output reg commit_A_valid, output reg [4:0] commit_A_rd, output reg [63:0] commit_A_data, output reg [5:0] commit_A_tag,
    output reg commit_B_valid, output reg [4:0] commit_B_rd, output reg [63:0] commit_B_data, output reg [5:0] commit_B_tag
);
    reg valid [0:DEPTH-1]; reg ready [0:DEPTH-1]; reg [4:0] dest_reg [0:DEPTH-1]; reg [63:0] value [0:DEPTH-1];
    reg [5:0] head, tail; reg [6:0] count;
    
    assign full = (count >= DEPTH - 2);
    assign empty = (count == 0);
    assign tag_A = tail; 
    assign tag_B = (tail + 1) % DEPTH;

    // Fixed Yosys syntax error: Variables moved to module scope
    reg pop_A, pop_B, push_A, push_B;
    integer i;

    always @(posedge clk) begin
        if (reset || flush) begin
            head <= 0; tail <= 0; count <= 0; commit_A_valid <= 0; commit_B_valid <= 0;
            for(i=0; i<DEPTH; i=i+1) begin valid[i] <= 0; ready[i] <= 0; end
        end else begin
            pop_A = 0; pop_B = 0; push_A = 0; push_B = 0;
            commit_A_valid <= 0; commit_B_valid <= 0;
            
            // Retire Logic
            if (count > 0 && ready[head]) begin
                commit_A_valid <= 1; commit_A_rd <= dest_reg[head]; commit_A_data <= value[head]; commit_A_tag <= head;
                valid[head] <= 0; pop_A = 1;
                if (count > 1 && ready[(head+1)%DEPTH]) begin
                    commit_B_valid <= 1; commit_B_rd <= dest_reg[(head+1)%DEPTH]; commit_B_data <= value[(head+1)%DEPTH]; commit_B_tag <= (head+1)%DEPTH;
                    valid[(head+1)%DEPTH] <= 0; pop_B = 1;
                end
            end
            
            // Writeback Snooping
            if (cdb1_valid && valid[cdb1_tag]) begin ready[cdb1_tag] <= 1; value[cdb1_tag] <= cdb1_data; end
            if (cdb2_valid && valid[cdb2_tag]) begin ready[cdb2_tag] <= 1; value[cdb2_tag] <= cdb2_data; end
            
            // Issue Allocation
            if (alloc_A && !full) begin valid[tail] <= 1; ready[tail] <= 0; dest_reg[tail] <= rd_A; push_A = 1; end
            if (alloc_B && !full) begin valid[(tail+push_A)%DEPTH] <= 1; ready[(tail+push_A)%DEPTH] <= 0; dest_reg[(tail+push_A)%DEPTH] <= rd_B; push_B = 1; end
            
            head <= (head + pop_A + pop_B) % DEPTH;
            tail <= (tail + push_A + push_B) % DEPTH;
            count <= count + push_A + push_B - pop_A - pop_B;
        end
    end
endmodule