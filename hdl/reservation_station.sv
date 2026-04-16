module reservation_station (
    input clk, reset, flush,
    input alloc_en, input [4:0] op_in, input [4:0] rd_in, input [63:0] pc_in, input [63:0] pred_tgt_in, input [5:0] rob_tag_in, 
    input val_vj, input [63:0] vj_in, input [5:0] qj_in, 
    input val_vk, input [63:0] vk_in, input [5:0] qk_in,
    input val_vl, input [63:0] vl_in, input [5:0] ql_in, // 3rd Operand
    input cdb1_valid, input [5:0] cdb1_tag, input [63:0] cdb1_data,
    input cdb2_valid, input [5:0] cdb2_tag, input [63:0] cdb2_data,

    output wire ready_to_fire, output wire [4:0] op_out, output wire [4:0] rd_out, output wire [63:0] pc_out, output wire [63:0] pred_tgt_out,
    output wire [63:0] vj_out, vk_out, vl_out, output wire [5:0] tag_out, input fire_ack, output reg busy
);
    reg [4:0] op; reg [4:0] rd; reg [63:0] pc; reg [63:0] pred_tgt; reg [5:0] rob_tag;
    reg qj_valid, qk_valid, ql_valid; reg [63:0] vj, vk, vl; reg [5:0] qj, qk, ql;

    assign ready_to_fire = busy & qj_valid & qk_valid & ql_valid;
    assign op_out = op; assign rd_out = rd; assign pc_out = pc; assign pred_tgt_out = pred_tgt;
    assign vj_out = vj; assign vk_out = vk; assign vl_out = vl; assign tag_out = rob_tag;

    always @(posedge clk) begin
        if (reset || flush) busy <= 0;
        else begin
            if (fire_ack) busy <= 0;
            if (alloc_en) begin
                busy <= 1; op <= op_in; rd <= rd_in; pc <= pc_in; pred_tgt <= pred_tgt_in; rob_tag <= rob_tag_in;
                qj <= qj_in; qk <= qk_in; ql <= ql_in;
                
                if (!val_vj && cdb1_valid && qj_in == cdb1_tag) begin vj <= cdb1_data; qj_valid <= 1; end
                else if (!val_vj && cdb2_valid && qj_in == cdb2_tag) begin vj <= cdb2_data; qj_valid <= 1; end
                else begin vj <= vj_in; qj_valid <= val_vj; end

                if (!val_vk && cdb1_valid && qk_in == cdb1_tag) begin vk <= cdb1_data; qk_valid <= 1; end
                else if (!val_vk && cdb2_valid && qk_in == cdb2_tag) begin vk <= cdb2_data; qk_valid <= 1; end
                else begin vk <= vk_in; qk_valid <= val_vk; end

                if (!val_vl && cdb1_valid && ql_in == cdb1_tag) begin vl <= cdb1_data; ql_valid <= 1; end
                else if (!val_vl && cdb2_valid && ql_in == cdb2_tag) begin vl <= cdb2_data; ql_valid <= 1; end
                else begin vl <= vl_in; ql_valid <= val_vl; end
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
                if (!ql_valid) begin
                    if (cdb1_valid && ql == cdb1_tag) begin vl <= cdb1_data; ql_valid <= 1; end
                    else if (cdb2_valid && ql == cdb2_tag) begin vl <= cdb2_data; ql_valid <= 1; end
                end
            end
        end
    end
endmodule