`include "hdl/instruction_decoder.sv"
`include "hdl/register_file.sv"
`include "hdl/rat_superscalar.sv"
`include "hdl/reservation_station.sv"
`include "hdl/ALU.sv"
`include "hdl/FPU.sv"
`include "hdl/lsq.sv"
`include "hdl/rob.sv"
`include "hdl/branch_predictor.sv"
`include "hdl/branch_unit.sv"
`include "hdl/memory.sv"

module tinker_core (
    input clk, reset, output logic hlt
);
    localparam FETCH = 2'd0, ISSUE = 2'd1;
    reg [1:0] state; reg [63:0] pc; reg [31:0] IR_A, IR_B;

    wire bu_valid, bu_mispredicted; wire [63:0] bu_correct_tgt; 
    wire sys_flush = bu_valid && bu_mispredicted; 

    wire pred_taken; wire [63:0] pred_tgt;
    wire bu_actual_taken; wire [63:0] bu_branch_pc;

    branch_predictor #(64) bp_unit (.clk(clk), .reset(reset), .fetch_pc(pc), .predict_taken(pred_taken), .predict_target(pred_tgt), .update_en(bu_valid), .update_pc(bu_branch_pc), .actual_taken(bu_actual_taken), .actual_target(bu_correct_tgt));

    wire [4:0] op_A, rd_A, rs_A, rt_A; wire [11:0] imm_A; wire u_imm_A, u_fpu_A, br_A;
    instruction_decoder dec_A (.instruction(IR_A), .opcode(op_A), .rd(rd_A), .rs(rs_A), .rt(rt_A), .imm(imm_A), .use_immediate(u_imm_A), .use_fpu_instruction(u_fpu_A), .is_branch(br_A));

    wire [4:0] op_B, rd_B, rs_B, rt_B; wire [11:0] imm_B; wire u_imm_B, u_fpu_B, br_B;
    instruction_decoder dec_B (.instruction(IR_B), .opcode(op_B), .rd(rd_B), .rs(rs_B), .rt(rt_B), .imm(imm_B), .use_immediate(u_imm_B), .use_fpu_instruction(u_fpu_B), .is_branch(br_B));

    wire cdb1_valid, cdb2_valid; wire [5:0] cdb1_tag, cdb2_tag; wire [4:0] cdb1_rd, cdb2_rd; wire [63:0] cdb1_data, cdb2_data;
    
    wire [5:0] rob_tag_A, rob_tag_B; wire rob_full, rob_empty; reg alloc_A, alloc_B;
    wire com_A_vld, com_B_vld; wire [4:0] com_A_rd, com_B_rd; wire [63:0] com_A_dat, com_B_dat; wire [5:0] com_A_tag, com_B_tag;

    reorder_buffer #(64) rob_unit (
        .clk(clk), .reset(reset), .flush(sys_flush), 
        .alloc_A(alloc_A), .rd_A(rd_A), .alloc_B(alloc_B), .rd_B(rd_B), 
        .tag_A(rob_tag_A), .tag_B(rob_tag_B), .full(rob_full), .empty(rob_empty),
        .cdb1_valid(cdb1_valid), .cdb1_tag(cdb1_tag), .cdb1_data(cdb1_data), 
        .cdb2_valid(cdb2_valid), .cdb2_tag(cdb2_tag), .cdb2_data(cdb2_data), 
        .commit_A_valid(com_A_vld), .commit_A_rd(com_A_rd), .commit_A_data(com_A_dat), .commit_A_tag(com_A_tag), 
        .commit_B_valid(com_B_vld), .commit_B_rd(com_B_rd), .commit_B_data(com_B_dat), .commit_B_tag(com_B_tag)
    );

    wire [63:0] rs_val_A, rt_val_A, rd_val_A, rs_val_B, rt_val_B, rd_val_B, r31_val;
    register_file reg_file (.clk(clk), .reset(reset), .write_enable_1(com_A_vld), .data_1(com_A_dat), .rd_w1(com_A_rd), .write_enable_2(com_B_vld), .data_2(com_B_dat), .rd_w2(com_B_rd), .rd_A(rd_A), .rs_A(rs_A), .rt_A(rt_A), .rd_val_A(rd_val_A), .rs_val_A(rs_val_A), .rt_val_A(rt_val_A), .rs_B(rs_B), .rt_B(rt_B), .rs_val_B(rs_val_B), .rt_val_B(rt_val_B), .r31_val(r31_val));

    wire rs_val_A_rat, rt_val_A_rat, rd_val_A_rat, rs_val_B_rat, rt_val_B_rat, rd_val_B_rat;
    wire [5:0] rs_tag_A, rt_tag_A, rd_tag_A, rs_tag_B, rt_tag_B, rd_tag_B;

    rat_superscalar rat_unit (.clk(clk), .reset(reset), .flush(sys_flush), .rs_A(rs_A), .rt_A(rt_A), .rd_A(rd_A), .val_rs_A(rs_val_A_rat), .val_rt_A(rt_val_A_rat), .val_rd_A(rd_val_A_rat), .tag_rs_A(rs_tag_A), .tag_rt_A(rt_tag_A), .tag_rd_A(rd_tag_A), .alloc_A(alloc_A), .alloc_tag_A(rob_tag_A), .rs_B(rs_B), .rt_B(rt_B), .rd_B(rd_B), .val_rs_B(rs_val_B_rat), .val_rt_B(rt_val_B_rat), .val_rd_B(rd_val_B_rat), .tag_rs_B(rs_tag_B), .tag_rt_B(rt_tag_B), .tag_rd_B(rd_tag_B), .alloc_B(alloc_B), .alloc_tag_B(rob_tag_B), .commit_A_valid(com_A_vld), .commit_A_tag(com_A_tag), .commit_A_rd(com_A_rd), .commit_B_valid(com_B_vld), .commit_B_tag(com_B_tag), .commit_B_rd(com_B_rd));

    wire [63:0] imm_val_A = ((op_A == 5'h19 || op_A == 5'h1b) ? {52'b0, imm_A} : {{52{imm_A[11]}}, imm_A});
    wire vk_vld_A = u_imm_A ? 1'b1 : rt_val_A_rat; wire [63:0] vk_dat_A = u_imm_A ? imm_val_A : rt_val_A;
    wire [63:0] imm_val_B = ((op_B == 5'h19 || op_B == 5'h1b) ? {52'b0, imm_B} : {{52{imm_B[11]}}, imm_B});
    wire vk_vld_B = u_imm_B ? 1'b1 : rt_val_B_rat; wire [63:0] vk_dat_B = u_imm_B ? imm_val_B : rt_val_B;

    reg [4:0] rs1_op, rs1_rd, rs2_op, rs2_rd, rs3_op, rs3_rd, rs4_op, rs4_rd;
    reg rs1_vj_v, rs1_vk_v, rs2_vj_v, rs2_vk_v, rs3_vj_v, rs3_vk_v, rs4_vj_v, rs4_vk_v;
    reg [63:0] rs1_vj_d, rs1_vk_d, rs2_vj_d, rs2_vk_d, rs3_vj_d, rs3_vk_d, rs4_vj_d, rs4_vk_d;
    reg [5:0] rs1_qj, rs1_qk, rs2_qj, rs2_qk, rs3_qj, rs3_qk, rs4_qj, rs4_qk;
    reg [63:0] rs4_pc, rs4_pred;
    reg [5:0] rs1_tag, rs2_tag, rs3_tag, rs4_tag, lsq_alloc_tag; // Added explicit RS tags
    reg alu1_alloc, alu2_alloc, fpu1_alloc, lsq_alloc, bu_alloc;

    reg lsq_is_store; reg [4:0] lsq_rd; reg [63:0] lsq_imm;
    reg lsq_av_vld; reg [63:0] lsq_av_dat; reg [5:0] lsq_av_tag;
    reg lsq_dv_vld; reg [63:0] lsq_dv_dat; reg [5:0] lsq_dv_tag;

    wire rs_alu1_busy, rs_alu2_busy, rs_fpu1_busy, lsq_full, rs_bu_busy;
    wire alu1_fire, alu2_fire, fpu1_fire, bu_fire;
    wire [4:0] alu1_op, alu2_op, fpu1_op, bu_op, alu1_rd, alu2_rd, fpu1_rd, bu_rd;
    wire [63:0] alu1_vj, alu1_vk, alu2_vj, alu2_vk, fpu1_vj, fpu1_vk, bu_vj, bu_vk, bu_pc_out, bu_pred_out;
    wire [5:0] alu1_t, alu2_t, fpu1_t, bu_t; 
    wire alu1_ready, alu2_ready, fpu_ready, bu_ready;

    // Passing the dynamically allocated rob_tag_in properly
    reservation_station rs_alu_1 (.clk(clk), .reset(reset), .flush(sys_flush), .alloc_en(alu1_alloc), .op_in(rs1_op), .rd_in(rs1_rd), .pc_in(64'b0), .pred_tgt_in(64'b0), .rob_tag_in(rs1_tag), .val_vj(rs1_vj_v), .vj_in(rs1_vj_d), .qj_in(rs1_qj), .val_vk(rs1_vk_v), .vk_in(rs1_vk_d), .qk_in(rs1_qk), .cdb1_valid(cdb1_valid), .cdb1_tag(cdb1_tag), .cdb1_data(cdb1_data), .cdb2_valid(cdb2_valid), .cdb2_tag(cdb2_tag), .cdb2_data(cdb2_data), .ready_to_fire(alu1_fire), .op_out(alu1_op), .rd_out(alu1_rd), .pc_out(), .pred_tgt_out(), .vj_out(alu1_vj), .vk_out(alu1_vk), .tag_out(alu1_t), .fire_ack(alu1_fire && alu1_ready), .busy(rs_alu1_busy));
    reservation_station rs_alu_2 (.clk(clk), .reset(reset), .flush(sys_flush), .alloc_en(alu2_alloc), .op_in(rs2_op), .rd_in(rs2_rd), .pc_in(64'b0), .pred_tgt_in(64'b0), .rob_tag_in(rs2_tag), .val_vj(rs2_vj_v), .vj_in(rs2_vj_d), .qj_in(rs2_qj), .val_vk(rs2_vk_v), .vk_in(rs2_vk_d), .qk_in(rs2_qk), .cdb1_valid(cdb1_valid), .cdb1_tag(cdb1_tag), .cdb1_data(cdb1_data), .cdb2_valid(cdb2_valid), .cdb2_tag(cdb2_tag), .cdb2_data(cdb2_data), .ready_to_fire(alu2_fire), .op_out(alu2_op), .rd_out(alu2_rd), .pc_out(), .pred_tgt_out(), .vj_out(alu2_vj), .vk_out(alu2_vk), .tag_out(alu2_t), .fire_ack(alu2_fire && alu2_ready), .busy(rs_alu2_busy));
    reservation_station rs_fpu_1 (.clk(clk), .reset(reset), .flush(sys_flush), .alloc_en(fpu1_alloc), .op_in(rs3_op), .rd_in(rs3_rd), .pc_in(64'b0), .pred_tgt_in(64'b0), .rob_tag_in(rs3_tag), .val_vj(rs3_vj_v), .vj_in(rs3_vj_d), .qj_in(rs3_qj), .val_vk(rs3_vk_v), .vk_in(rs3_vk_d), .qk_in(rs3_qk), .cdb1_valid(cdb1_valid), .cdb1_tag(cdb1_tag), .cdb1_data(cdb1_data), .cdb2_valid(cdb2_valid), .cdb2_tag(cdb2_tag), .cdb2_data(cdb2_data), .ready_to_fire(fpu1_fire), .op_out(fpu1_op), .rd_out(fpu1_rd), .pc_out(), .pred_tgt_out(), .vj_out(fpu1_vj), .vk_out(fpu1_vk), .tag_out(fpu1_t), .fire_ack(fpu1_fire && fpu_ready), .busy(rs_fpu1_busy));
    reservation_station rs_branch(.clk(clk), .reset(reset), .flush(sys_flush), .alloc_en(bu_alloc),   .op_in(rs4_op), .rd_in(rs4_rd), .pc_in(rs4_pc), .pred_tgt_in(rs4_pred), .rob_tag_in(rs4_tag), .val_vj(rs4_vj_v), .vj_in(rs4_vj_d), .qj_in(rs4_qj), .val_vk(rs4_vk_v), .vk_in(rs4_vk_d), .qk_in(rs4_qk), .cdb1_valid(cdb1_valid), .cdb1_tag(cdb1_tag), .cdb1_data(cdb1_data), .cdb2_valid(cdb2_valid), .cdb2_tag(cdb2_tag), .cdb2_data(cdb2_data), .ready_to_fire(bu_fire),   .op_out(bu_op),   .rd_out(bu_rd),   .pc_out(bu_pc_out), .pred_tgt_out(bu_pred_out), .vj_out(bu_vj),   .vk_out(bu_vk),   .tag_out(bu_t),   .fire_ack(bu_fire && bu_ready),   .busy(rs_bu_busy));

    wire [1:0] lsq_tail; 
    wire lsq_vout; wire [5:0] lsq_tout; wire [4:0] lsq_rdout; wire [63:0] lsq_res; wire lsq_ack;
    wire mem_r, mem_w; wire [63:0] m_addr, m_wdata, m_rdata;

    lsq #(4) lsq_unit (
        .clk(clk), .reset(reset), .flush(sys_flush), .alloc_en(lsq_alloc), .is_store(lsq_is_store), .dest_rd(lsq_rd), .imm(lsq_imm),
        .addr_vld(lsq_av_vld), .addr_val(lsq_av_dat), .addr_tag(lsq_av_tag), .data_vld(lsq_dv_vld), .data_val(lsq_dv_dat), .data_tag(lsq_dv_tag),
        .alloc_tag(lsq_alloc_tag), .full(lsq_full), .current_tail(lsq_tail), // Changed to dynamic alloc_tag
        .cdb1_valid(cdb1_valid), .cdb1_tag(cdb1_tag), .cdb1_data(cdb1_data), .cdb2_valid(cdb2_valid), .cdb2_tag(cdb2_tag), .cdb2_data(cdb2_data),
        .mem_read(mem_r), .mem_write(mem_w), .mem_addr(m_addr), .mem_wdata(m_wdata), .mem_rdata(m_rdata),
        .cdb_valid(lsq_vout), .cdb_tag(lsq_tout), .cdb_rd(lsq_rdout), .cdb_data(lsq_res), .cdb_ack(lsq_ack)
    );
    memory memory (.clk(clk), .addr(m_addr), .write_data(m_wdata), .mem_write(mem_w), .mem_read(mem_r), .read_data(m_rdata));

    wire alu1_vout, alu2_vout, fpu_vout; wire [63:0] alu1_res, alu2_res, fpu_res;
    wire [5:0] alu1_tout, alu2_tout, fpu_tout; wire [4:0] alu1_rdout, alu2_rdout, fpu_rdout; wire alu1_ack, alu2_ack, fpu_ack;

    ALU alu_unit_1 (.clk(clk), .reset(reset), .flush(sys_flush), .valid_in(alu1_fire), .op(alu1_op), .a(alu1_vj), .b(alu1_vk), .tag_in(alu1_t), .rd_in(alu1_rd), .ready_in(alu1_ready), .valid_out(alu1_vout), .res_out(alu1_res), .tag_out(alu1_tout), .rd_out(alu1_rdout), .ack_out(alu1_ack));
    ALU alu_unit_2 (.clk(clk), .reset(reset), .flush(sys_flush), .valid_in(alu2_fire), .op(alu2_op), .a(alu2_vj), .b(alu2_vk), .tag_in(alu2_t), .rd_in(alu2_rd), .ready_in(alu2_ready), .valid_out(alu2_vout), .res_out(alu2_res), .tag_out(alu2_tout), .rd_out(alu2_rdout), .ack_out(alu2_ack));
    FPU fpu_unit   (.clk(clk), .reset(reset), .flush(sys_flush), .valid_in(fpu1_fire), .op(fpu1_op), .a(fpu1_vj), .b(fpu1_vk), .tag_in(fpu1_t), .rd_in(fpu1_rd), .ready_in(fpu_ready), .valid_out(fpu_vout), .res_out(fpu_res), .tag_out(fpu_tout), .rd_out(fpu_rdout), .ack_out(fpu_ack));
    
    wire [5:0] bu_tout; wire [4:0] bu_rdout; wire [63:0] bu_res; wire bu_ack;
    branch_unit branch_exec (.clk(clk), .reset(reset), .flush(sys_flush), .valid_in(bu_fire), .op(bu_op), .pc(bu_pc_out), .predicted_target(bu_pred_out), .a_val(bu_vj), .b_val(bu_vk), .imm(bu_vk), .tag_in(bu_t), .rd_in(bu_rd), .ready_in(bu_ready), .valid_out(bu_valid), .mispredicted(bu_mispredicted), .correct_target(bu_correct_tgt), .actual_taken(bu_actual_taken), .branch_pc(bu_branch_pc), .res_out(bu_res), .tag_out(bu_tout), .rd_out(bu_rdout), .ack_out(bu_ack));

    // Arbiter
    wire sel1_lsq = lsq_vout;
    wire sel1_bu  = !sel1_lsq & bu_valid;
    wire sel1_a1  = !sel1_lsq & !sel1_bu & alu1_vout;
    wire sel1_a2  = !sel1_lsq & !sel1_bu & !sel1_a1 & alu2_vout;
    wire sel1_fpu = !sel1_lsq & !sel1_bu & !sel1_a1 & !sel1_a2 & fpu_vout;

    assign cdb1_valid = sel1_lsq | sel1_bu | sel1_a1 | sel1_a2 | sel1_fpu;
    assign cdb1_tag   = sel1_lsq ? lsq_tout  : (sel1_bu ? bu_tout  : (sel1_a1 ? alu1_tout  : (sel1_a2 ? alu2_tout  : fpu_tout)));
    assign cdb1_rd    = sel1_lsq ? lsq_rdout : (sel1_bu ? bu_rdout : (sel1_a1 ? alu1_rdout : (sel1_a2 ? alu2_rdout : fpu_rdout)));
    assign cdb1_data  = sel1_lsq ? lsq_res   : (sel1_bu ? bu_res   : (sel1_a1 ? alu1_res   : (sel1_a2 ? alu2_res   : fpu_res)));

    wire in2_bu  = bu_valid & ~sel1_bu;
    wire in2_a1  = alu1_vout & ~sel1_a1;
    wire in2_a2  = alu2_vout & ~sel1_a2;
    wire in2_fpu = fpu_vout & ~sel1_fpu;

    wire sel2_bu  = in2_bu;
    wire sel2_a1  = !sel2_bu & in2_a1;
    wire sel2_a2  = !sel2_bu & !sel2_a1 & in2_a2;
    wire sel2_fpu = !sel2_bu & !sel2_a1 & !sel2_a2 & in2_fpu;

    assign cdb2_valid = sel2_bu | sel2_a1 | sel2_a2 | sel2_fpu;
    assign cdb2_tag   = sel2_bu ? bu_tout  : (sel2_a1 ? alu1_tout  : (sel2_a2 ? alu2_tout  : fpu_tout));
    assign cdb2_rd    = sel2_bu ? bu_rdout : (sel2_a1 ? alu1_rdout : (sel2_a2 ? alu2_rdout : fpu_rdout));
    assign cdb2_data  = sel2_bu ? bu_res   : (sel2_a1 ? alu1_res   : (sel2_a2 ? alu2_res   : fpu_res));

    assign lsq_ack  = sel1_lsq; 
    assign bu_ack   = sel1_bu | sel2_bu;
    assign alu1_ack = sel1_a1 | sel2_a1; 
    assign alu2_ack = sel1_a2 | sel2_a2; 
    assign fpu_ack  = sel1_fpu | sel2_fpu;

    wire backend_empty = (!rs_alu1_busy && !rs_alu2_busy && !rs_fpu1_busy && !rs_bu_busy && !lsq_vout && !alu1_vout && !alu2_vout && !fpu_vout && !bu_valid);
    wire is_mem_A = (op_A == 5'h10 || op_A == 5'h13); wire is_mem_B = (op_B == 5'h10 || op_B == 5'h13);

    always @(posedge clk) begin
        reg issued_A, issued_B;
        if (reset) begin 
            state <= FETCH; hlt <= 0; pc <= 64'h2000; 
            alloc_A <= 0; alloc_B <= 0; alu1_alloc <= 0; alu2_alloc <= 0; fpu1_alloc <= 0; lsq_alloc <= 0; bu_alloc <= 0; 
        end 
        else if (sys_flush) begin
            state <= FETCH; pc <= bu_correct_tgt; // Flush!
            alloc_A <= 0; alloc_B <= 0; alu1_alloc <= 0; alu2_alloc <= 0; fpu1_alloc <= 0; lsq_alloc <= 0; bu_alloc <= 0;
        end 
        else begin
            alloc_A <= 0; alloc_B <= 0; alu1_alloc <= 0; alu2_alloc <= 0; fpu1_alloc <= 0; lsq_alloc <= 0; bu_alloc <= 0;
            issued_A = 0; issued_B = 0;

            case (state)
                FETCH: begin
                    IR_A <= {memory.bytes[pc+3], memory.bytes[pc+2], memory.bytes[pc+1], memory.bytes[pc]};
                    IR_B <= {memory.bytes[pc+7], memory.bytes[pc+6], memory.bytes[pc+5], memory.bytes[pc+4]};
                    state <= ISSUE;
                end
                
                ISSUE: begin
                    // <--- FIXED: Safely wait for ROB to empty before asserting Halt
                    if (op_A == 5'h0f || op_B == 5'h0f) begin 
                        if (rob_empty && backend_empty) hlt <= 1'b1; 
                    end 
                    else if (!rob_full) begin
                        if (br_A) begin
                            if (!rs_bu_busy) begin 
                                bu_alloc <= 1; alloc_A <= 1; issued_A = 1;
                                rs4_op<=op_A; rs4_rd<=rd_A; rs4_vj_v<=rs_val_A_rat; rs4_vj_d<=rs_val_A; rs4_qj<=rs_tag_A; rs4_vk_v<=vk_vld_A; rs4_vk_d<=vk_dat_A; rs4_qk<=rt_tag_A;
                                rs4_pc<=pc; rs4_pred<=pred_tgt; rs4_tag<=rob_tag_A;
                            end
                        end else if (is_mem_A) begin
                            if (!lsq_full) begin
                                lsq_alloc <= 1; alloc_A <= 1; issued_A = 1; // Store AND Load allocate
                                lsq_is_store <= (op_A == 5'h13); lsq_imm <= imm_A; lsq_rd <= rd_A;
                                lsq_av_vld <= (op_A==5'h10) ? rs_val_A_rat : rd_val_A_rat; lsq_av_dat <= (op_A==5'h10) ? rs_val_A : rd_val_A; lsq_av_tag <= (op_A==5'h10) ? rs_tag_A : rd_tag_A;
                                lsq_dv_vld <= rs_val_A_rat; lsq_dv_dat <= rs_val_A; lsq_dv_tag <= rs_tag_A;
                                lsq_alloc_tag <= rob_tag_A;
                            end
                        end else if (!u_fpu_A) begin
                            if (!rs_alu1_busy) begin
                                alu1_alloc<=1; alloc_A<=1; issued_A=1;
                                rs1_op<=op_A; rs1_rd<=rd_A; rs1_vj_v<=rs_val_A_rat; rs1_vj_d<=rs_val_A; rs1_qj<=rs_tag_A; rs1_vk_v<=vk_vld_A; rs1_vk_d<=vk_dat_A; rs1_qk<=rt_tag_A;
                                rs1_tag<=rob_tag_A;
                            end else if (!rs_alu2_busy) begin
                                alu2_alloc<=1; alloc_A<=1; issued_A=1;
                                rs2_op<=op_A; rs2_rd<=rd_A; rs2_vj_v<=rs_val_A_rat; rs2_vj_d<=rs_val_A; rs2_qj<=rs_tag_A; rs2_vk_v<=vk_vld_A; rs2_vk_d<=vk_dat_A; rs2_qk<=rt_tag_A;
                                rs2_tag<=rob_tag_A;
                            end
                        end else begin
                            if (!rs_fpu1_busy) begin
                                fpu1_alloc<=1; alloc_A<=1; issued_A=1;
                                rs3_op<=op_A; rs3_rd<=rd_A; rs3_vj_v<=rs_val_A_rat; rs3_vj_d<=rs_val_A; rs3_qj<=rs_tag_A; rs3_vk_v<=vk_vld_A; rs3_vk_d<=vk_dat_A; rs3_qk<=rt_tag_A;
                                rs3_tag<=rob_tag_A;
                            end
                        end

                        if (issued_A && !is_mem_B && !rob_full) begin 
                            if (!pred_taken) begin
                                if (br_B) begin
                                    if (!rs_bu_busy && !bu_alloc) begin 
                                        bu_alloc<=1; alloc_B<=1; issued_B=1;
                                        rs4_op<=op_B; rs4_rd<=rd_B; rs4_vj_v<=rs_val_B_rat; rs4_vj_d<=rs_val_B; rs4_qj<=rs_tag_B; rs4_vk_v<=vk_vld_B; rs4_vk_d<=vk_dat_B; rs4_qk<=rt_tag_B;
                                        rs4_pc<=(pc+4); rs4_pred<=(pc+8); rs4_tag<=rob_tag_B;
                                    end
                                end else if (!u_fpu_B) begin
                                    if (!rs_alu1_busy && !alu1_alloc) begin 
                                        alu1_alloc<=1; alloc_B<=1; issued_B=1;
                                        rs1_op<=op_B; rs1_rd<=rd_B; rs1_vj_v<=rs_val_B_rat; rs1_vj_d<=rs_val_B; rs1_qj<=rs_tag_B; rs1_vk_v<=vk_vld_B; rs1_vk_d<=vk_dat_B; rs1_qk<=rt_tag_B;
                                        rs1_tag<=rob_tag_B;
                                    end else if (!rs_alu2_busy && !alu2_alloc) begin 
                                        alu2_alloc<=1; alloc_B<=1; issued_B=1;
                                        rs2_op<=op_B; rs2_rd<=rd_B; rs2_vj_v<=rs_val_B_rat; rs2_vj_d<=rs_val_B; rs2_qj<=rs_tag_B; rs2_vk_v<=vk_vld_B; rs2_vk_d<=vk_dat_B; rs2_qk<=rt_tag_B;
                                        rs2_tag<=rob_tag_B;
                                    end
                                end else begin
                                    if (!rs_fpu1_busy && !fpu1_alloc) begin 
                                        fpu1_alloc<=1; alloc_B<=1; issued_B=1;
                                        rs3_op<=op_B; rs3_rd<=rd_B; rs3_vj_v<=rs_val_B_rat; rs3_vj_d<=rs_val_B; rs3_qj<=rs_tag_B; rs3_vk_v<=vk_vld_B; rs3_vk_d<=vk_dat_B; rs3_qk<=rt_tag_B;
                                        rs3_tag<=rob_tag_B;
                                    end
                                end
                            end
                            if (pred_taken) pc <= pred_tgt;
                            else pc <= issued_B ? (pc + 8) : (pc + 4); 
                            state <= FETCH;
                        end else if (issued_A) begin
                            pc <= pred_taken ? pred_tgt : (pc + 4); 
                            state <= FETCH;
                        end
                    end
                end
            endcase
        end
    end
endmodule