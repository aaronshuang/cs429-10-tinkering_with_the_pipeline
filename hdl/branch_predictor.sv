module branch_predictor #(parameter ENTRIES = 64) (
    input clk, reset,

    // --- Read Port (Fetch Stage) ---
    input [63:0] fetch_pc,
    output wire predict_taken,
    output wire [63:0] predict_target,

    // --- Update Port (Execution Stage) ---
    input update_en,
    input [63:0] update_pc,
    input actual_taken,
    input [63:0] actual_target
);
    reg valid [0:ENTRIES-1];
    reg [55:0] tags [0:ENTRIES-1];
    reg [1:0] state [0:ENTRIES-1]; // 2-bit counter: 0=SNT, 1=WNT, 2=WT, 3=ST
    reg [63:0] targets [0:ENTRIES-1];

    wire [5:0] fetch_idx = fetch_pc[7:2];
    wire [55:0] fetch_tag = fetch_pc[63:8];

    wire [5:0] update_idx = update_pc[7:2];
    wire [55:0] update_tag = update_pc[63:8];

    // Prediction Logic
    wire is_match = valid[fetch_idx] && (tags[fetch_idx] == fetch_tag);
    assign predict_taken = is_match && (state[fetch_idx] >= 2'd2);
    assign predict_target = is_match ? targets[fetch_idx] : (fetch_pc + 4);

    integer i;
    always @(posedge clk) begin
        if (reset) begin
            for(i=0; i<ENTRIES; i=i+1) valid[i] <= 0;
        end else if (update_en) begin
            valid[update_idx] <= 1'b1;
            tags[update_idx] <= update_tag;
            targets[update_idx] <= actual_target;
            
            if (valid[update_idx] && tags[update_idx] == update_tag) begin
                // Update existing counter
                if (actual_taken && state[update_idx] != 2'd3) 
                    state[update_idx] <= state[update_idx] + 1;
                else if (!actual_taken && state[update_idx] != 2'd0) 
                    state[update_idx] <= state[update_idx] - 1;
            end else begin
                // Initialize new branch entry
                state[update_idx] <= actual_taken ? 2'd2 : 2'd1;
            end
        end
    end
endmodule   