module reorder_buffer #(parameter DEPTH = 64) (
    input clk, reset, 
    output reg sys_flush, output reg [63:0] flush_target, // Driven by ROB now
    
    input alloc_A, input [4:0] rd_A,
    input alloc_B, input [4:0] rd_B,
    output wire [5:0] tag_A, tag_B, output wire full, output wire empty,
    
    input cdb1_valid, input [5:0] cdb1_tag, input [63:0] cdb1_data,
    input cdb2_valid, input [5:0] cdb2_tag, input [63:0] cdb2_data,
    
    input bu_valid, input [5:0] bu_tag, input bu_mispredicted, input [63:0] bu_target,
    input bu_taken, input [63:0] bu_pc,
    output reg bp_update_en, output reg [63:0] bp_update_pc, output reg bp_actual_taken, output reg [63:0] bp_actual_target,
    
    output reg commit_A_valid, output reg [4:0] commit_A_rd, output reg [63:0] commit_A_data, output reg [5:0] commit_A_tag,
    output reg commit_B_valid, output reg [4:0] commit_B_rd, output reg [63:0] commit_B_data, output reg [5:0] commit_B_tag,
    
    input [5:0] rtag_1, input [5:0] rtag_2, input [5:0] rtag_3, input [5:0] rtag_4, input [5:0] rtag_5, input [5:0] rtag_6,
    output wire rrdy_1, output wire rrdy_2, output wire rrdy_3, output wire rrdy_4, output wire rrdy_5, output wire rrdy_6,
    output wire [63:0] rval_1, output wire [63:0] rval_2, output wire [63:0] rval_3, output wire [63:0] rval_4, output wire [63:0] rval_5, output wire [63:0] rval_6
);
    reg valid [0:DEPTH-1]; reg ready [0:DEPTH-1]; reg [4:0] dest_reg [0:DEPTH-1]; reg [63:0] value [0:DEPTH-1];
    reg mispredict_array [0:DEPTH-1]; reg [63:0] target_array [0:DEPTH-1];
    reg taken_array [0:DEPTH-1]; reg [63:0] pc_array [0:DEPTH-1]; reg is_br_array [0:DEPTH-1];

    reg [5:0] head, tail; reg [6:0] count;
    assign full = (count >= DEPTH - 2); assign empty = (count == 0);
    assign tag_A = tail; assign tag_B = (tail + 1) % DEPTH;

    assign rrdy_1 = valid[rtag_1] && ready[rtag_1]; assign rval_1 = value[rtag_1];
    assign rrdy_2 = valid[rtag_2] && ready[rtag_2]; assign rval_2 = value[rtag_2];
    assign rrdy_3 = valid[rtag_3] && ready[rtag_3]; assign rval_3 = value[rtag_3];
    assign rrdy_4 = valid[rtag_4] && ready[rtag_4]; assign rval_4 = value[rtag_4];
    assign rrdy_5 = valid[rtag_5] && ready[rtag_5]; assign rval_5 = value[rtag_5];
    assign rrdy_6 = valid[rtag_6] && ready[rtag_6]; assign rval_6 = value[rtag_6];

    reg pop_A, pop_B, push_A, push_B; integer i;

    always @(posedge clk) begin
        sys_flush <= 0; bp_update_en <= 0; // Default off
        
        if (reset) begin 
            head <= 0; tail <= 0; count <= 0; commit_A_valid <= 0; commit_B_valid <= 0;
            for(i=0; i<DEPTH; i=i+1) begin valid[i] <= 0; ready[i] <= 0; is_br_array[i] <= 0; mispredict_array[i] <= 0; end
        end else begin
            pop_A = 0; pop_B = 0; push_A = 0; push_B = 0;
            commit_A_valid <= 0; commit_B_valid <= 0;
            
            // Snoop Branch Unit out-of-order
            if (bu_valid) begin
                mispredict_array[bu_tag] <= bu_mispredicted;
                target_array[bu_tag] <= bu_target;
                taken_array[bu_tag] <= bu_taken;
                pc_array[bu_tag] <= bu_pc;
                is_br_array[bu_tag] <= 1'b1;
            end
            
            // Retire Logic (In Order)
            if (count > 0 && ready[head]) begin
                if (is_br_array[head]) begin
                    bp_update_en <= 1; bp_update_pc <= pc_array[head]; bp_actual_taken <= taken_array[head]; bp_actual_target <= target_array[head];
                end
                
                if (mispredict_array[head]) begin
                    // Trigger Precise Exception Flush!
                    sys_flush <= 1; flush_target <= target_array[head];
                    head <= 0; tail <= 0; count <= 0;
                    for(i=0; i<DEPTH; i=i+1) begin valid[i] <= 0; ready[i] <= 0; is_br_array[i] <= 0; mispredict_array[i] <= 0; end
                end else begin
                    // Normal Commit
                    commit_A_valid <= 1; commit_A_rd <= dest_reg[head]; commit_A_data <= value[head]; commit_A_tag <= head;
                    valid[head] <= 0; is_br_array[head] <= 0; pop_A = 1;
                    
                    if (count > 1 && ready[(head+1)%DEPTH] && !mispredict_array[(head+1)%DEPTH]) begin
                        commit_B_valid <= 1; commit_B_rd <= dest_reg[(head+1)%DEPTH]; commit_B_data <= value[(head+1)%DEPTH]; commit_B_tag <= (head+1)%DEPTH;
                        valid[(head+1)%DEPTH] <= 0; is_br_array[(head+1)%DEPTH] <= 0; pop_B = 1;
                    end
                end
            end
            
            if (cdb1_valid && valid[cdb1_tag]) begin ready[cdb1_tag] <= 1; value[cdb1_tag] <= cdb1_data; end
            if (cdb2_valid && valid[cdb2_tag]) begin ready[cdb2_tag] <= 1; value[cdb2_tag] <= cdb2_data; end
            
            if (!(count > 0 && ready[head] && mispredict_array[head])) begin
                if (alloc_A && !full) begin valid[tail] <= 1; ready[tail] <= 0; dest_reg[tail] <= rd_A; is_br_array[tail] <= 0; mispredict_array[tail] <= 0; push_A = 1; end
                if (alloc_B && !full) begin valid[(tail+push_A)%DEPTH] <= 1; ready[(tail+push_A)%DEPTH] <= 0; dest_reg[(tail+push_A)%DEPTH] <= rd_B; is_br_array[(tail+push_A)%DEPTH] <= 0; mispredict_array[(tail+push_A)%DEPTH] <= 0; push_B = 1; end
                
                head <= (head + pop_A + pop_B) % DEPTH;
                tail <= (tail + push_A + push_B) % DEPTH;
                count <= count + push_A + push_B - pop_A - pop_B;
            end
        end
    end
endmodule