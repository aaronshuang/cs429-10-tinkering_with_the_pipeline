module reservation_station #(parameter TAG = 6'd1) (
    input clk, reset,

    // Allocation (From Issue Stage)
    input alloc_en,
    input [4:0] op_in,
    input [4:0] rd_in,
    input val_vj, input [63:0] vj_in, input [5:0] qj_in,
    input val_vk, input [63:0] vk_in, input [5:0] qk_in,

    // CDB Snooping (Listen for finished tags)
    input cdb_valid,
    input [5:0] cdb_tag,
    input [63:0] cdb_data,

    // Dispatch (To Execution Unit)
    output wire ready_to_fire,
    output wire [4:0] op_out,
    output wire [63:0] vj_out, vk_out,
    output wire [5:0] tag_out,
    output wire [4:0] rd_out,
    input fire_ack, // Exec unit accepted the data

    output reg busy
);
    reg [4:0] op;
    reg [4:0] rd;
    reg qj_valid, qk_valid;
    reg [63:0] vj, vk;
    reg [5:0] qj, qk;

    assign ready_to_fire = busy & qj_valid & qk_valid;
    assign op_out = op;
    assign vj_out = vj;
    assign vk_out = vk;
    assign tag_out = TAG;
    assign rd_out = rd;

    always @(posedge clk) begin
        if (reset) busy <= 0;
        else begin
            // Free the RS once the execution unit takes the instruction
            if (fire_ack) busy <= 0;

            if (alloc_en) begin
                busy <= 1;
                op <= op_in; rd <= rd_in;
                qj_valid <= val_vj; vj <= vj_in; qj <= qj_in;
                qk_valid <= val_vk; vk <= vk_in; qk <= qk_in;
            end 

            // Snoop the Common Data Bus to capture missing operands
            if (busy && cdb_valid) begin
                if (!qj_valid && qj == cdb_tag) begin
                    vj <= cdb_data; qj_valid <= 1;
                end
                if (!qk_valid && qk == cdb_tag) begin
                    vk <= cdb_data; qk_valid <= 1;
                end
            end
        end
    end
endmodule