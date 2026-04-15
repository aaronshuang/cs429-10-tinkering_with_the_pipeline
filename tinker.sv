`include "hdl/instruction_decoder.sv"
`include "hdl/register_file.sv"
`include "hdl/rat_superscalar.sv"
`include "hdl/reservation_station.sv"
`include "hdl/ALU.sv"
`include "hdl/FPU.sv"
`include "hdl/memory.sv"

module tinker_core (
    input clk,
    input reset,
    output logic hlt
);
    localparam FETCH = 3'd0, ISSUE = 3'd1, WAIT_DRAIN = 3'd2, EXEC_MEM = 3'd3;
    reg [2:0] state; 
    reg [63:0] pc;
    reg [31:0] IR_A, IR_B;

    // ==========================================
    // DUAL DECODERS
    // ==========================================
    wire [4:0] op_A, rd_A, rs_A, rt_A; wire [11:0] imm_A; wire u_imm_A, u_fpu_A, br_A;
    instruction_decoder dec_A (.instruction(IR_A), .opcode(op_A), .rd(rd_A), .rs(rs_A), .rt(rt_A), .imm(imm_A), .use_immediate(u_imm_A), .use_fpu_instruction(u_fpu_A), .is_branch(br_A));

    wire [4:0] op_B, rd_B, rs_B, rt_B; wire [11:0] imm_B; wire u_imm_B, u_fpu_B, br_B;
    instruction_decoder dec_B (.instruction(IR_B), .opcode(op_B), .rd(rd_B), .rs(rs_B), .rt(rt_B), .imm(imm_B), .use_immediate(u_imm_B), .use_fpu_instruction(u_fpu_B), .is_branch(br_B));

    // ==========================================
    // CDB & REGFILE WRITE MUX
    // ==========================================
    wire cdb_valid; wire [5:0] cdb_tag; wire [4:0] cdb_rd; wire [63:0] cdb_data;
    
    wire mem_wb_en = (state == EXEC_MEM && op_A == 5'h10); // Load instruction barrier finish
    wire rf_we = (cdb_valid && cdb_rd != 0) || (mem_wb_en && rd_A != 0);
    wire [4:0] rf_wa = mem_wb_en ? rd_A : cdb_rd;
    wire [63:0] rf_wd = mem_wb_en ? mem_read_data : cdb_data;

    // ==========================================
    // 4-PORT REGISTER FILE
    // ==========================================
    wire [63:0] rs_val_A, rt_val_A, rs_val_B, rt_val_B, rd_val_A, r31_val;
    register_file reg_file (
        .clk(clk), .reset(reset), .write_enable(rf_we), .data(rf_wd), .rd(rf_wa), 
        .rd_A(rd_A), .rs_A(rs_A), .rt_A(rt_A), .rd_val_A(rd_val_A), .rs_val_A(rs_val_A), .rt_val_A(rt_val_A), 
        .rs_B(rs_B), .rt_B(rt_B), .rs_val_B(rs_val_B), .rt_val_B(rt_val_B),
        .r31_val(r31_val)
    );

    // ==========================================
    // SUPERSCALAR REGISTER ALIAS TABLE (RAT)
    // ==========================================
    wire rs_val_A_rat, rt_val_A_rat, rs_val_B_rat, rt_val_B_rat;
    wire [5:0] rs_tag_A, rt_tag_A, rs_tag_B, rt_tag_B;
    reg alloc_A, alloc_B; reg [5:0] tag_alloc_A, tag_alloc_B;

    rat_superscalar rat_unit (
        .clk(clk), .reset(reset),
        .rs_A(rs_A), .rt_A(rt_A), .rd_A(rd_A), .val_rs_A(rs_val_A_rat), .val_rt_A(rt_val_A_rat), .tag_rs_A(rs_tag_A), .tag_rt_A(rt_tag_A), .alloc_A(alloc_A), .alloc_tag_A(tag_alloc_A),
        .rs_B(rs_B), .rt_B(rt_B), .rd_B(rd_B), .val_rs_B(rs_val_B_rat), .val_rt_B(rt_val_B_rat), .tag_rs_B(rs_tag_B), .tag_rt_B(rt_tag_B), .alloc_B(alloc_B), .alloc_tag_B(tag_alloc_B),
        .cdb_valid(cdb_valid), .cdb_tag(cdb_tag), .cdb_rd(cdb_rd)
    );

    // ==========================================
    // DISPATCH MULTIPLEXING LOGIC
    // ==========================================
    wire [63:0] imm_val_A = ((op_A == 5'h19 || op_A == 5'h1b) ? {52'b0, imm_A} : {{52{imm_A[11]}}, imm_A});
    wire vk_vld_A = u_imm_A ? 1'b1 : rt_val_A_rat;
    wire [63:0] vk_dat_A = u_imm_A ? imm_val_A : rt_val_A;

    wire [63:0] imm_val_B = ((op_B == 5'h19 || op_B == 5'h1b) ? {52'b0, imm_B} : {{52{imm_B[11]}}, imm_B});
    wire vk_vld_B = u_imm_B ? 1'b1 : rt_val_B_rat;
    wire [63:0] vk_dat_B = u_imm_B ? imm_val_B : rt_val_B;

    // Registers that catch the dispatched instruction and feed the RS
    reg [4:0] rs1_op, rs1_rd, rs2_op, rs2_rd, rs3_op, rs3_rd;
    reg rs1_vj_v, rs1_vk_v, rs2_vj_v, rs2_vk_v, rs3_vj_v, rs3_vk_v;
    reg [63:0] rs1_vj_d, rs1_vk_d, rs2_vj_d, rs2_vk_d, rs3_vj_d, rs3_vk_d;
    reg [5:0] rs1_qj, rs1_qk, rs2_qj, rs2_qk, rs3_qj, rs3_qk;
    reg alu1_alloc, alu2_alloc, fpu1_alloc;

    // ==========================================
    // RESERVATION STATIONS
    // ==========================================
    wire rs_alu1_busy, rs_alu2_busy, rs_fpu1_busy;
    wire alu1_fire, alu2_fire, fpu1_fire;
    wire [4:0] alu1_op, alu2_op, fpu1_op, alu1_rd, alu2_rd, fpu1_rd;
    wire [63:0] alu1_vj, alu1_vk, alu2_vj, alu2_vk, fpu1_vj, fpu1_vk;
    wire [5:0] alu1_t, alu2_t, fpu1_t;
    wire alu1_ready, alu2_ready, fpu_ready;

    reservation_station #(6'd1) rs_alu_1 (.clk(clk), .reset(reset), .alloc_en(alu1_alloc), .op_in(rs1_op), .rd_in(rs1_rd),
        .val_vj(rs1_vj_v), .vj_in(rs1_vj_d), .qj_in(rs1_qj), .val_vk(rs1_vk_v), .vk_in(rs1_vk_d), .qk_in(rs1_qk),
        .cdb_valid(cdb_valid), .cdb_tag(cdb_tag), .cdb_data(cdb_data),
        .ready_to_fire(alu1_fire), .op_out(alu1_op), .vj_out(alu1_vj), .vk_out(alu1_vk), .tag_out(alu1_t), .rd_out(alu1_rd),
        .fire_ack(alu1_fire && alu1_ready), .busy(rs_alu1_busy));

    reservation_station #(6'd2) rs_alu_2 (.clk(clk), .reset(reset), .alloc_en(alu2_alloc), .op_in(rs2_op), .rd_in(rs2_rd),
        .val_vj(rs2_vj_v), .vj_in(rs2_vj_d), .qj_in(rs2_qj), .val_vk(rs2_vk_v), .vk_in(rs2_vk_d), .qk_in(rs2_qk),
        .cdb_valid(cdb_valid), .cdb_tag(cdb_tag), .cdb_data(cdb_data),
        .ready_to_fire(alu2_fire), .op_out(alu2_op), .vj_out(alu2_vj), .vk_out(alu2_vk), .tag_out(alu2_t), .rd_out(alu2_rd),
        .fire_ack(alu2_fire && alu2_ready), .busy(rs_alu2_busy));

    reservation_station #(6'd3) rs_fpu_1 (.clk(clk), .reset(reset), .alloc_en(fpu1_alloc), .op_in(rs3_op), .rd_in(rs3_rd),
        .val_vj(rs3_vj_v), .vj_in(rs3_vj_d), .qj_in(rs3_qj), .val_vk(rs3_vk_v), .vk_in(rs3_vk_d), .qk_in(rs3_qk),
        .cdb_valid(cdb_valid), .cdb_tag(cdb_tag), .cdb_data(cdb_data),
        .ready_to_fire(fpu1_fire), .op_out(fpu1_op), .vj_out(fpu1_vj), .vk_out(fpu1_vk), .tag_out(fpu1_t), .rd_out(fpu1_rd),
        .fire_ack(fpu1_fire && fpu_ready), .busy(rs_fpu1_busy));

    // ==========================================
    // EXECUTION UNITS (True Dual ALU + 1 FPU)
    // ==========================================
    wire alu1_vout, alu2_vout, fpu_vout;
    wire [63:0] alu1_res, alu2_res, fpu_res;
    wire [5:0] alu1_tout, alu2_tout, fpu_tout;
    wire [4:0] alu1_rdout, alu2_rdout, fpu_rdout;
    wire alu1_ack, alu2_ack, fpu_ack;

    ALU alu_unit_1 (.clk(clk), .reset(reset), .valid_in(alu1_fire), .op(alu1_op), .a(alu1_vj), .b(alu1_vk), .tag_in(alu1_t), .rd_in(alu1_rd),
             .ready_in(alu1_ready), .valid_out(alu1_vout), .res_out(alu1_res), .tag_out(alu1_tout), .rd_out(alu1_rdout), .ack_out(alu1_ack));

    ALU alu_unit_2 (.clk(clk), .reset(reset), .valid_in(alu2_fire), .op(alu2_op), .a(alu2_vj), .b(alu2_vk), .tag_in(alu2_t), .rd_in(alu2_rd),
             .ready_in(alu2_ready), .valid_out(alu2_vout), .res_out(alu2_res), .tag_out(alu2_tout), .rd_out(alu2_rdout), .ack_out(alu2_ack));

    FPU fpu_unit (.clk(clk), .reset(reset), .valid_in(fpu1_fire), .op(fpu1_op), .a(fpu1_vj), .b(fpu1_vk), .tag_in(fpu1_t), .rd_in(fpu1_rd),
             .ready_in(fpu_ready), .valid_out(fpu_vout), .res_out(fpu_res), .tag_out(fpu_tout), .rd_out(fpu_rdout), .ack_out(fpu_ack));

    // ==========================================
    // CDB ARBITER (Priority Encoder & Collision Avoidance)
    // ==========================================
    assign cdb_valid = alu1_vout | alu2_vout | fpu_vout;
    
    // Output multiplexer based on priority (ALU1 > ALU2 > FPU)
    assign cdb_tag  = alu1_vout ? alu1_tout : (alu2_vout ? alu2_tout : fpu_tout);
    assign cdb_rd   = alu1_vout ? alu1_rdout : (alu2_vout ? alu2_rdout : fpu_rdout);
    assign cdb_data = alu1_vout ? alu1_res : (alu2_vout ? alu2_res : fpu_res);

    // Stalls lower-priority units if they try to write to the bus on the same exact cycle
    assign alu1_ack = alu1_vout;
    assign alu2_ack = alu2_vout & !alu1_vout;
    assign fpu_ack  = fpu_vout & !alu1_vout & !alu2_vout;

    // ==========================================
    // MEMORY UNIT & BARRIERS
    // ==========================================
    wire mem_read = (op_A == 5'h10 || op_A == 5'h0d);
    wire mem_write = (op_A == 5'h13 || op_A == 5'h0c);
    wire [63:0] mem_read_data;
    
    wire [63:0] load_store_addr = (op_A == 5'h13) ? (rd_val_A + imm_val_A) : (rs_val_A + imm_val_A);
    wire [63:0] stack_addr = r31_val - 8;
    wire [63:0] mem_addr = (op_A == 5'h0c || op_A == 5'h0d) ? stack_addr : load_store_addr;
    wire [63:0] mem_wdata = (op_A == 5'h0c) ? pc : rs_val_A;

    memory memory (.clk(clk), .addr(mem_addr), .write_data(mem_wdata), .mem_write(state == EXEC_MEM && mem_write), .mem_read(state == EXEC_MEM && mem_read), .read_data(mem_read_data));

    // ==========================================
    // FRONTEND FSM (FETCH & DUAL-ISSUE)
    // ==========================================
    wire backend_empty = (!rs_alu1_busy && !rs_alu2_busy && !rs_fpu1_busy && !alu1_vout && !alu2_vout && !fpu_vout);
    
    // We treat branches and memory ops as barriers that stall the OoO engine.
    wire barrier_A = (br_A || op_A == 5'h10 || op_A == 5'h13 || op_A == 5'h0d || op_A == 5'h0c);
    wire barrier_B = (br_B || op_B == 5'h10 || op_B == 5'h13 || op_B == 5'h0d || op_B == 5'h0c);

    always @(posedge clk) begin
        if (reset) begin
            state <= FETCH; hlt <= 0; pc <= 64'h2000;
            alloc_A <= 0; alloc_B <= 0; alu1_alloc <= 0; alu2_alloc <= 0; fpu1_alloc <= 0;
        end else begin
            alloc_A <= 0; alloc_B <= 0; alu1_alloc <= 0; alu2_alloc <= 0; fpu1_alloc <= 0;

            case (state)
                FETCH: begin
                    // Dual Fetch (8 Bytes)
                    IR_A <= {memory.bytes[pc+3], memory.bytes[pc+2], memory.bytes[pc+1], memory.bytes[pc]};
                    IR_B <= {memory.bytes[pc+7], memory.bytes[pc+6], memory.bytes[pc+5], memory.bytes[pc+4]};
                    state <= ISSUE;
                end
                
                ISSUE: begin
                    if (op_A == 5'h0f) begin hlt <= 1'b1; end 
                    
                    // --- ATTEMPT TO ISSUE INSTRUCTION A ---
                    else if (barrier_A) begin
                        if (backend_empty) state <= EXEC_MEM;
                        else state <= WAIT_DRAIN;
                    end 
                    else begin
                        reg issued_A = 0;
                        if (!u_fpu_A) begin
                            if (!rs_alu1_busy) begin
                                alu1_alloc<=1; alloc_A<=1; tag_alloc_A<=6'd1; issued_A=1;
                                rs1_op<=op_A; rs1_rd<=rd_A; rs1_vj_v<=rs_val_A_rat; rs1_vj_d<=rs_val_A; rs1_qj<=rs_tag_A; rs1_vk_v<=vk_vld_A; rs1_vk_d<=vk_dat_A; rs1_qk<=rt_tag_A;
                            end else if (!rs_alu2_busy) begin
                                alu2_alloc<=1; alloc_A<=1; tag_alloc_A<=6'd2; issued_A=1;
                                rs2_op<=op_A; rs2_rd<=rd_A; rs2_vj_v<=rs_val_A_rat; rs2_vj_d<=rs_val_A; rs2_qj<=rs_tag_A; rs2_vk_v<=vk_vld_A; rs2_vk_d<=vk_dat_A; rs2_qk<=rt_tag_A;
                            end
                        end else begin
                            if (!rs_fpu1_busy) begin
                                fpu1_alloc<=1; alloc_A<=1; tag_alloc_A<=6'd3; issued_A=1;
                                rs3_op<=op_A; rs3_rd<=rd_A; rs3_vj_v<=rs_val_A_rat; rs3_vj_d<=rs_val_A; rs3_qj<=rs_tag_A; rs3_vk_v<=vk_vld_A; rs3_vk_d<=vk_dat_A; rs3_qk<=rt_tag_A;
                            end
                        end

                        // --- ATTEMPT TO ISSUE INSTRUCTION B ---
                        // B can only issue if A successfully issued AND B is not a barrier.
                        if (issued_A && !barrier_B) begin
                            reg issued_B = 0;
                            if (!u_fpu_B) begin
                                if (!rs_alu1_busy && !alu1_alloc) begin // Verify A didn't just take ALU1
                                    alu1_alloc<=1; alloc_B<=1; tag_alloc_B<=6'd1; issued_B=1;
                                    rs1_op<=op_B; rs1_rd<=rd_B; rs1_vj_v<=rs_val_B_rat; rs1_vj_d<=rs_val_B; rs1_qj<=rs_tag_B; rs1_vk_v<=vk_vld_B; rs1_vk_d<=vk_dat_B; rs1_qk<=rt_tag_B;
                                end else if (!rs_alu2_busy && !alu2_alloc) begin // Verify A didn't just take ALU2
                                    alu2_alloc<=1; alloc_B<=1; tag_alloc_B<=6'd2; issued_B=1;
                                    rs2_op<=op_B; rs2_rd<=rd_B; rs2_vj_v<=rs_val_B_rat; rs2_vj_d<=rs_val_B; rs2_qj<=rs_tag_B; rs2_vk_v<=vk_vld_B; rs2_vk_d<=vk_dat_B; rs2_qk<=rt_tag_B;
                                end
                            end else begin
                                if (!rs_fpu1_busy && !fpu1_alloc) begin 
                                    fpu1_alloc<=1; alloc_B<=1; tag_alloc_B<=6'd3; issued_B=1;
                                    rs3_op<=op_B; rs3_rd<=rd_B; rs3_vj_v<=rs_val_B_rat; rs3_vj_d<=rs_val_B; rs3_qj<=rs_tag_B; rs3_vk_v<=vk_vld_B; rs3_vk_d<=vk_dat_B; rs3_qk<=rt_tag_B;
                                end
                            end

                            if (issued_B) begin
                                pc <= pc + 8; // Full Superscalar dual-issue success!
                                state <= FETCH;
                            end else begin
                                pc <= pc + 4; // Only A fired (Structural Hazard on B)
                                state <= FETCH;
                            end
                        end else if (issued_A) begin
                            pc <= pc + 4; // B was a barrier or A was the only one that fit
                            state <= FETCH;
                        end
                        // If !issued_A, both stall in ISSUE state until RS frees up
                    end
                end

                WAIT_DRAIN: begin
                    if (backend_empty) state <= EXEC_MEM;
                end

                EXEC_MEM: begin
                    // Executes Instruction A's Barrier (Branches & Memory)
                    if (br_A && op_A != 5'h0d) begin 
                        case (op_A)
                            5'h08: pc <= rd_val_A;
                            5'h09: pc <= (pc - 4) + rd_val_A;
                            5'h0a: pc <= (pc - 4) + imm_val_A;
                            5'h0b: if (rs_val_A != 0) pc <= rd_val_A;
                            5'h0c: pc <= rd_val_A; 
                            5'h0e: if (rs_val_A > rt_val_A) pc <= rd_val_A;
                        endcase
                    end else if (op_A == 5'h10) begin 
                        // Load: rf_we is true this cycle
                        pc <= pc + 4;
                    end else if (op_A == 5'h0d) begin 
                        // Return
                        pc <= mem_read_data;
                    end else begin
                        // Store
                        pc <= pc + 4;
                    end
                    state <= FETCH; 
                end
            endcase
        end
    end
endmodule