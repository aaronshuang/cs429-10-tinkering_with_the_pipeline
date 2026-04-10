`include "hdl/instruction_decoder.sv"
`include "hdl/register_file.sv"
`include "hdl/ALU.sv"
`include "hdl/FPU.sv"
`include "hdl/memory.sv"

module tinker_core (
    input clk,
    input reset,
    output logic hlt
);
    // --- FSM States ---
    localparam FETCH = 3'd0, DECODE = 3'd1, EXEC = 3'd2, MEM = 3'd3, WB = 3'd4;
    reg [2:0] state;

    // --- Architectural State ---
    reg [63:0] pc;

    // --- Pipeline Registers (Intermediate Buckets) ---
    reg [31:0] IR;
    reg [63:0] A, B, RD_LATCH;
    reg [63:0] ALUOut, FPUOut;
    reg [63:0] MDR;

    // --- Decoder Wires ---
    wire [4:0] opcode, rd, rs, rt;
    wire [11:0] imm;
    wire use_immediate, use_fpu_instruction, is_branch;

    instruction_decoder decoder (
        .instruction(IR), // Feeds from the latched Pipeline Register, not memory directly
        .opcode(opcode), .rd(rd), .rs(rs), .rt(rt), .imm(imm),
        .use_immediate(use_immediate), .use_fpu_instruction(use_fpu_instruction),
        .is_branch(is_branch)
    );

    // --- Control Signals (Driven by State) ---
    wire mem_read = (state == MEM && (opcode == 5'h10 || opcode == 5'h0d));
    wire mem_write = (state == MEM && (opcode == 5'h13 || opcode == 5'h0c));
    wire reg_write_en = (state == WB);

    // --- Register File ---
    wire [63:0] rd_val, rs_val, rt_val, r31_val;
    wire [63:0] reg_write_data;

    register_file reg_file (
        .clk(clk), .reset(reset), .write_enable(reg_write_en),
        .data(reg_write_data),
        .rd(rd), .rs(rs), .rt(rt),
        .rd_val(rd_val), .rs_val(rs_val), .rt_val(rt_val),
        .r31_val(r31_val)
    );

    // --- ALU / FPU MUX Routing ---
    wire uses_rd_as_a = (opcode == 5'h13) || (opcode == 5'h19) || 
                        (opcode == 5'h1b) || (opcode == 5'h05) || 
                        (opcode == 5'h07) || (opcode == 5'h12);
    
    // Feed ALU from the Pipeline Registers (A, B, RD_LATCH), not the Register File directly
    wire [63:0] alu_input_a = uses_rd_as_a ? RD_LATCH : A;
    wire [63:0] alu_input_b = (use_immediate) ? 
        ((opcode == 5'h19 || opcode == 5'h1b) ? {52'b0, imm} : {{52{imm[11]}}, imm}) : B;

    wire [63:0] alu_res, fpu_res;
    ALU alu (.a(alu_input_a), .b(alu_input_b), .op(opcode), .res(alu_res));
    FPU fpu (.a(A), .b(alu_input_b), .op(opcode), .res(fpu_res));

    // --- Memory MUX Routing ---
    wire [63:0] stack_addr = r31_val - 8;
    wire [63:0] mem_addr = (opcode == 5'h0c || opcode == 5'h0d) ? stack_addr : ALUOut;
    wire [63:0] mem_wdata = (opcode == 5'h0c) ? (pc) : A;

    wire [63:0] mem_read_data;
    memory memory (
        .clk(clk), .addr(mem_addr), .write_data(mem_wdata), 
        .mem_write(mem_write), .mem_read(mem_read), .read_data(mem_read_data)
    );

    // --- Write-back MUX Routing ---
    assign reg_write_data = (opcode == 5'h11) ? A : 
                            (opcode == 5'h12) ? {RD_LATCH[63:12], IR[11:0]} :
                            (use_fpu_instruction) ? FPUOut : 
                            (opcode == 5'h10) ? MDR : ALUOut;

    // --- Branch Target Logic ---
    reg take_branch;
    reg [63:0] branch_target;

    always @(*) begin
        take_branch = 1'b0;
        branch_target = 64'b0;
        if (is_branch) begin
            case (opcode)
                5'h08: begin take_branch = 1'b1; branch_target = RD_LATCH; end
                5'h09: begin take_branch = 1'b1; branch_target = (pc - 4) + RD_LATCH; end
                5'h0a: begin take_branch = 1'b1; branch_target = (pc - 4) + {{52{imm[11]}}, imm}; end
                5'h0b: begin if (A != 0) begin take_branch = 1'b1; branch_target = RD_LATCH; end end
                5'h0c: begin take_branch = 1'b1; branch_target = RD_LATCH; end
                5'h0e: begin if (A > B) begin take_branch = 1'b1; branch_target = RD_LATCH; end end
            endcase
        end
    end

    // FSM Controller
    always @(posedge clk) begin
        if (reset) begin
            state <= FETCH;
            hlt <= 1'b0;
            pc <= 64'h2000;
        end else begin
            case (state)
                FETCH: begin
                    // Read instruction from memory array
                    IR <= {memory.bytes[pc+3], memory.bytes[pc+2], memory.bytes[pc+1], memory.bytes[pc]};
                    pc <= pc + 4;
                    state <= DECODE;
                end
                
                DECODE: begin
                    // Latch outputs from the Register File
                    A <= rs_val;
                    B <= rt_val;
                    RD_LATCH <= rd_val;
                    state <= EXEC;
                end
                
                EXEC: begin
                    // Latch math results
                    ALUOut <= alu_res;
                    FPUOut <= fpu_res;

                    if (opcode == 5'h0f && imm == 12'b0) begin
                        $display("Execution Halted.");
                        hlt <= 1'b1;
                        // $finish; 
                    end else if (is_branch && opcode != 5'h0d) begin 
                        // Resolve Branches (except Return, which needs memory)
                        if (take_branch) pc <= branch_target;
                        
                        if (opcode == 5'h0c) state <= MEM; // CALL proceeds to push to stack
                        else state <= FETCH;
                    end else begin
                        // Standard Instruction Routing
                        if (opcode == 5'h10 || opcode == 5'h13 || opcode == 5'h0d) 
                            state <= MEM; // Load, Store, Return move to MEM
                        else 
                            state <= WB; // Math/Logic jump to Write-back
                    end
                end
                
                MEM: begin
                    if (opcode == 5'h10 || opcode == 5'h0d) begin 
                        MDR <= mem_read_data; // Capture load/return data
                        if (opcode == 5'h0d) pc <= mem_read_data; // Apply Return address to PC
                        state <= (opcode == 5'h10) ? WB : FETCH;
                    end else begin
                        state <= FETCH; // Store & Call finish here
                    end
                end
                
                WB: begin
                    // register_write_en is automatically held high by the state during this cycle
                    state <= FETCH;
                end
            endcase
        end
    end
endmodule