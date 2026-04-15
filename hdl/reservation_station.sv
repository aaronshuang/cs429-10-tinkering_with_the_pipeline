module reservation_station #(parameter TAG = 6'd1) (
    input clk, reset, flush, // ADDED FLUSH

    input alloc_en,
    input [4:0] op_in, input [4:0] rd_in,
    input [63:0] pc_in, input [63:0] pred_tgt_in, // ADDED
    input val_vj, input [63:0] vj_in, input [5:0] qj_in,
    input val_vk, input [63:0] vk_in, input [5:0] qk_in,

    input cdb1_valid, input [5:0] cdb1_tag, input [63:0] cdb1_data,
    input cdb2_valid, input [5:0] cdb2_tag, input [63:0] cdb2_data,

    output wire ready_to_fire,
    output wire [4:0] op_out, output wire [4:0] rd_out,
    output wire [63:0] pc_out, output wire [63:0] pred_tgt_out, // ADDED
    output wire [63:0] vj_out, vk_out,
    output wire [5:0] tag_out,
    input fire_ack, 
    output reg busy
);
    reg [4:0] op; reg [4:0] rd; reg [63:0] pc; reg [63:0] pred_tgt;
    reg qj_valid, qk_valid; reg [63:0] vj, vk; reg [5:0] qj, qk;

    assign ready_to_fire = busy & qj_valid & qk_valid;
    assign op_out = op; assign vj_out = vj; assign vk_out = vk;
    assign tag_out = TAG; assign rd_out = rd; 
    assign pc_out = pc; assign pred_tgt_out = pred_tgt;

    always @(posedge clk) begin
        if (reset || flush) busy <= 0; // FLUSH CLEARS RS
        else begin
            if (fire_ack) busy <= 0;

            if (alloc_en) begin
                busy <= 1;
                op <= op_in; rd <= rd_in; pc <= pc_in; pred_tgt <= pred_tgt_in;
                qj_valid <= val_vj; vj <= vj_in; qj <= qj_in;
                qk_valid <= val_vk; vk <= vk_in; qk <= qk_in;
            end 

            if (busy) begin
                if (!qj_valid) begin
                    if (cdb1_valid && qj == cdb1_tag) begin vj <= cdb1_data; qj_valid <= 1; end
                    else if (cdb2_valid && qj == cdb2_tag) begin vj <= cdb2_data; qj_valid <= 1; end
                end
                if (!qk_valid) begin
                    if (cdb1_valid && qk == cdb1_tag) begin vk <= cdb1_data; qk_valid <= 1; end
                    else if (cdb2_valid && qk == cdb2_tag) begin vk <= cdb2_data; qk_valid <= 1; end
                end
            end
        end
    end
endmodule