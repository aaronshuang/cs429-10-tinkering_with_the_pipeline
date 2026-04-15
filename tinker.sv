`include "hdl/instruction_decoder.sv"
`include "hdl/register_file.sv"
`include "hdl/rat.sv"
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
    reg [31:0] IR;

    // --- Decoder Wires ---
    wire [4:0] opcode, rd, rs, rt;
    wire [11:0] imm;
    wire use_imm, use_fpu, is_branch;
    instruction_decoder decoder (.instruction(IR), .opcode(opcode), .rd(rd), .rs(rs), .rt(rt), .imm(imm), .use_immediate(use_imm), .use_fpu_instruction(use_fpu), .is_branch(is_branch));

    // --- Common Data Bus (CDB) ---
    wire cdb_valid;
    wire [5:0] cdb_tag;
    wire [4:0] cdb_rd;
    wire [63:0] cdb_data;

    // --- Register File ---
    wire [63:0] rs_val, rt_val, r31_val;
    register_file reg_file (
        .clk(clk), .reset(reset), 
        .write_enable(cdb_valid && cdb_rd != 0), .data(cdb_data), .rd(cdb_rd), // CDB writes directly to RegFile
        .rs(rs), .rt(rt), .rs_val(rs_val), .rt_val(rt_val), .r31_val(r31_val)
    );

    // --- Register Alias Table (RAT) ---
    wire rs_valid, rt_valid;
    wire [5:0] rs_tag_val, rt_tag_val;
    reg rat_alloc; reg [5:0] rat_tag_alloc;
    rat rat_unit (.clk(clk), .reset(reset), .rs_idx(rs), .rt_idx(rt), .rd_idx(rd),
                  .valid_rs(rs_valid), .valid_rt(rt_valid), .tag_rs(rs_tag_val), .tag_rt(rt_tag_val),
                  .alloc_en(rat_alloc), .alloc_tag(rat_tag_alloc), 
                  .cdb_valid(cdb_valid), .cdb_tag(cdb_tag), .cdb_rd(cdb_rd));

    // --- Reservation Stations (2x ALU, 1x FPU) ---
    wire rs_alu1_busy, rs_alu2_busy, rs_fpu1_busy;
    reg alloc_alu1, alloc_alu2, alloc_fpu1;
    wire [63:0] imm_val = ((opcode == 5'h19 || opcode == 5'h1b) ? {52'b0, imm} : {{52{imm[11]}}, imm});
    
    // Immediate overrides the 'rt' register check
    wire vj_is_valid = rs_valid;
    wire vk_is_valid = use_imm ? 1'b1 : rt_valid;
    wire [63:0] vj_data = rs_val;
    wire [63:0] vk_data = use_imm ? imm_val : rt_val;

    // RS to Exec Unit Links
    wire alu1_fire, alu2_fire, fpu1_fire;
    wire [4:0] alu1_op, alu2_op, fpu1_op, alu1_rd, alu2_rd, fpu1_rd;
    wire [63:0] alu1_vj, alu1_vk, alu2_vj, alu2_vk, fpu1_vj, fpu1_vk;
    wire [5:0] alu1_t, alu2_t, fpu1_t;
    wire alu_ready, fpu_ready;

    reservation_station #(6'd1) rs_alu_1 (.clk(clk), .reset(reset), .alloc_en(alloc_alu1), .op_in(opcode), .rd_in(rd),
        .val_vj(vj_is_valid), .vj_in(vj_data), .qj_in(rs_tag_val), .val_vk(vk_is_valid), .vk_in(vk_data), .qk_in(rt_tag_val),
        .cdb_valid(cdb_valid), .cdb_tag(cdb_tag), .cdb_data(cdb_data),
        .ready_to_fire(alu1_fire), .op_out(alu1_op), .vj_out(alu1_vj), .vk_out(alu1_vk), .tag_out(alu1_t), .rd_out(alu1_rd),
        .fire_ack(alu1_fire && alu_ready), .busy(rs_alu1_busy));

    reservation_station #(6'd2) rs_alu_2 (.clk(clk), .reset(reset), .alloc_en(alloc_alu2), .op_in(opcode), .rd_in(rd),
        .val_vj(vj_is_valid), .vj_in(vj_data), .qj_in(rs_tag_val), .val_vk(vk_is_valid), .vk_in(vk_data), .qk_in(rt_tag_val),
        .cdb_valid(cdb_valid), .cdb_tag(cdb_tag), .cdb_data(cdb_data),
        .ready_to_fire(alu2_fire), .op_out(alu2_op), .vj_out(alu2_vj), .vk_out(alu2_vk), .tag_out(alu2_t), .rd_out(alu2_rd),
        .fire_ack(alu2_fire && !alu1_fire && alu_ready), .busy(rs_alu2_busy)); // ALU1 has priority to fire

    reservation_station #(6'd3) rs_fpu_1 (.clk(clk), .reset(reset), .alloc_en(alloc_fpu1), .op_in(opcode), .rd_in(rd),
        .val_vj(vj_is_valid), .vj_in(vj_data), .qj_in(rs_tag_val), .val_vk(vk_is_valid), .vk_in(vk_data), .qk_in(rt_tag_val),
        .cdb_valid(cdb_valid), .cdb_tag(cdb_tag), .cdb_data(cdb_data),
        .ready_to_fire(fpu1_fire), .op_out(fpu1_op), .vj_out(fpu1_vj), .vk_out(fpu1_vk), .tag_out(fpu1_t), .rd_out(fpu1_rd),
        .fire_ack(fpu1_fire && fpu_ready), .busy(rs_fpu1_busy));

    // --- Execution Units ---
    wire alu_vout, fpu_vout;
    wire [63:0] alu_res, fpu_res;
    wire [5:0] alu_tout, fpu_tout;
    wire [4:0] alu_rdout, fpu_rdout;

    wire alu_fire_any = alu1_fire | alu2_fire;
    wire [4:0] alu_op_in = alu1_fire ? alu1_op : alu2_op;
    wire [63:0] alu_a_in = alu1_fire ? alu1_vj : alu2_vj;
    wire [63:0] alu_b_in = alu1_fire ? alu1_vk : alu2_vk;
    wire [5:0] alu_t_in  = alu1_fire ? alu1_t : alu2_t;
    wire [4:0] alu_r_in  = alu1_fire ? alu1_rd : alu2_rd;

    ALU alu (.clk(clk), .reset(reset), .valid_in(alu_fire_any), .op(alu_op_in), .a(alu_a_in), .b(alu_b_in), .tag_in(alu_t_in), .rd_in(alu_r_in),
             .ready_in(alu_ready), .valid_out(alu_vout), .res_out(alu_res), .tag_out(alu_tout), .rd_out(alu_rdout), .ack_out(alu_vout)); // Auto-ack for now

    FPU fpu (.clk(clk), .reset(reset), .valid_in(fpu1_fire), .op(fpu1_op), .a(fpu1_vj), .b(fpu1_vk), .tag_in(fpu1_t), .rd_in(fpu1_rd),
             .ready_in(fpu_ready), .valid_out(fpu_vout), .res_out(fpu_res), .tag_out(fpu_tout), .rd_out(fpu_rdout), .ack_out(fpu_vout)); // Auto-ack

    // --- CDB Arbiter ---
    // If both finish same cycle, ALU gets bus priority
    assign cdb_valid = alu_vout | fpu_vout;
    assign cdb_tag   = alu_vout ? alu_tout : fpu_tout;
    assign cdb_rd    = alu_vout ? alu_rdout : fpu_rdout;
    assign cdb_data  = alu_vout ? alu_res : fpu_res;

    // --- Memory Barrier Unit ---
    wire mem_read = (opcode == 5'h10 || opcode == 5'h0d);
    wire mem_write = (opcode == 5'h13 || opcode == 5'h0c);
    wire [63:0] mem_read_data;
    memory memory (.clk(clk), .addr(rs_val), .write_data(rt_val), .mem_write(state == EXEC_MEM && mem_write), .mem_read(state == EXEC_MEM && mem_read), .read_data(mem_read_data));

    wire backend_empty = (!rs_alu1_busy && !rs_alu2_busy && !rs_fpu1_busy && !alu_vout && !fpu_vout);

    // --- Frontend IF/ISSUE State Machine ---
    always @(posedge clk) begin
        if (reset) begin
            state <= FETCH; hlt <= 0; pc <= 64'h2000;
            alloc_alu1 <= 0; alloc_alu2 <= 0; alloc_fpu1 <= 0; rat_alloc <= 0;
        end else begin
            alloc_alu1 <= 0; alloc_alu2 <= 0; alloc_fpu1 <= 0; rat_alloc <= 0;

            case (state)
                FETCH: begin
                    IR <= {memory.bytes[pc+3], memory.bytes[pc+2], memory.bytes[pc+1], memory.bytes[pc]};
                    pc <= pc + 4;
                    state <= ISSUE;
                end
                
                ISSUE: begin
                    if (opcode == 5'h0f && imm == 12'b0) begin
                        $display("Execution Halted."); hlt <= 1'b1;
                    end 
                    else if (is_branch || opcode == 5'h10 || opcode == 5'h13 || opcode == 5'h0d) begin
                        // Barrier: Stall until OoO backend finishes to prevent memory/control hazards
                        if (backend_empty) state <= EXEC_MEM;
                        else state <= WAIT_DRAIN;
                    end 
                    else begin // ALU / FPU Instructions
                        if (!use_fpu) begin
                            if (!rs_alu1_busy) begin
                                alloc_alu1 <= 1; rat_alloc <= 1; rat_tag_alloc <= 6'd1; state <= FETCH;
                            end else if (!rs_alu2_busy) begin
                                alloc_alu2 <= 1; rat_alloc <= 1; rat_tag_alloc <= 6'd2; state <= FETCH;
                            end // If both busy, stall in ISSUE
                        end else begin
                            if (!rs_fpu1_busy) begin
                                alloc_fpu1 <= 1; rat_alloc <= 1; rat_tag_alloc <= 6'd3; state <= FETCH;
                            end // Stall if busy
                        end
                    end
                end

                WAIT_DRAIN: begin
                    if (backend_empty) state <= EXEC_MEM;
                end

                EXEC_MEM: begin
                    // Sequential Execute for Memory and Control
                    if (is_branch && opcode != 5'h0d) begin 
                        // Resolve Branches
                        if (opcode == 5'h08) pc <= rs_val;
                        // ... ADD OTHER BRANCH OPCODES HERE ...
                        state <= FETCH;
                    end else if (opcode == 5'h10) begin 
                        // It's a Load. We must bypass CDB and force-write the RegFile and RAT.
                        // For a pure Tomasulo, you'd have a MEM RS, but this works for the barrier method.
                        state <= FETCH;
                    end else begin
                        state <= FETCH; // Store finish
                    end
                end
            endcase
        end
    end
endmodule