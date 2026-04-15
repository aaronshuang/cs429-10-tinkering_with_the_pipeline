module reorder_buffer #(parameter DEPTH = 64) (
    input clk, reset, 
    input flush, // We will use this when we add Branch Prediction

    // --- ISSUE STAGE (Allocate in-order) ---
    input alloc_A, input [4:0] rd_A,
    input alloc_B, input [4:0] rd_B,
    output wire [5:0] tag_A, tag_B,
    output wire full, // Stall fetch if we can't fit 2 instructions

    // --- WRITEBACK STAGE (Snoop CDB out-of-order) ---
    input cdb1_valid, input [5:0] cdb1_tag, input [63:0] cdb1_data,
    input cdb2_valid, input [5:0] cdb2_tag, input [63:0] cdb2_data,

    // --- RETIRE STAGE (Commit to RegFile in-order) ---
    output reg commit_A_valid, output reg [4:0] commit_A_rd, output reg [63:0] commit_A_data, output reg [5:0] commit_A_tag,
    output reg commit_B_valid, output reg [4:0] commit_B_rd, output reg [63:0] commit_B_data, output reg [5:0] commit_B_tag
);
    // ROB Data Structures
    reg valid [0:DEPTH-1];
    reg ready [0:DEPTH-1];
    reg [4:0] dest_reg [0:DEPTH-1];
    reg [63:0] value [0:DEPTH-1];

    reg [5:0] head, tail;
    reg [6:0] count;

    // We need 2 free slots to ensure dual-issue doesn't overflow
    assign full = (count >= DEPTH - 2);

    assign tag_A = tail;
    assign tag_B = (tail + 1) % DEPTH;

    wire [5:0] head_plus_1 = (head + 1) % DEPTH;

    integer i;
    always @(posedge clk) begin
        reg pop_A;
        reg pop_B;
        reg push_A;
        reg push_B;
        if (reset || flush) begin
            head <= 0; tail <= 0; count <= 0;
            commit_A_valid <= 0; commit_B_valid <= 0;
            for(i=0; i<DEPTH; i=i+1) begin
                valid[i] <= 0; ready[i] <= 0;
            end
        end else begin
            // ---------------------------------------------------
            // 1. RETIRE (Commit from Head in-order)
            // ---------------------------------------------------
            pop_A = 0;
            pop_B = 0;
            commit_A_valid <= 0; commit_B_valid <= 0;

            if (count > 0 && ready[head]) begin
                // The oldest instruction is finished! Commit it.
                commit_A_valid <= 1;
                commit_A_rd <= dest_reg[head];
                commit_A_data <= value[head];
                commit_A_tag <= head;
                valid[head] <= 0;
                pop_A = 1;

                // Can we commit the SECOND oldest instruction too?
                if (count > 1 && ready[head_plus_1]) begin
                    commit_B_valid <= 1;
                    commit_B_rd <= dest_reg[head_plus_1];
                    commit_B_data <= value[head_plus_1];
                    commit_B_tag <= head_plus_1;
                    valid[head_plus_1] <= 0;
                    pop_B = 1;
                end
            end

            // ---------------------------------------------------
            // 2. WRITEBACK (Snoop Dual CDB out-of-order)
            // ---------------------------------------------------
            if (cdb1_valid && valid[cdb1_tag]) begin
                ready[cdb1_tag] <= 1;
                value[cdb1_tag] <= cdb1_data;
            end
            if (cdb2_valid && valid[cdb2_tag]) begin
                ready[cdb2_tag] <= 1;
                value[cdb2_tag] <= cdb2_data;
            end

            // ---------------------------------------------------
            // 3. ISSUE (Allocate at Tail in-order)
            // ---------------------------------------------------
            push_A = 0;
            push_B = 0;

            if (alloc_A && !full) begin
                valid[tail] <= 1;
                ready[tail] <= 0; // Not done yet!
                dest_reg[tail] <= rd_A;
                push_A = 1;
            end
            if (alloc_B && !full) begin
                valid[tag_B] <= 1;
                ready[tag_B] <= 0;
                dest_reg[tag_B] <= rd_B;
                push_B = 1;
            end

            // ---------------------------------------------------
            // 4. Update Pointers
            // ---------------------------------------------------
            head <= (head + pop_A + pop_B) % DEPTH;
            tail <= (tail + push_A + push_B) % DEPTH;
            count <= count + push_A + push_B - pop_A - pop_B;
        end
    end
endmodule