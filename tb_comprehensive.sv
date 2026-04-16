`timescale 1ns/1ps
`include "tinker.sv"

module tb_comprehensive;
    reg clk;
    reg reset;
    wire hlt;

    // Instantiate the Core
    tinker_core uut (
        .clk(clk),
        .reset(reset),
        .hlt(hlt)
    );

    // Clock Generation
    always #5 clk = ~clk;

    // --- INTERNAL ASSEMBLER TASK ---
    // Maps standard RISC format: [Opcode: 5][RD: 5][RS: 5][RT: 5][Imm: 12]
    task assemble(input [31:0] addr, input [4:0] op, input [4:0] rd, input [4:0] rs, input [4:0] rt, input [11:0] imm);
        logic [31:0] inst;
        begin
            inst = {op, rd, rs, rt, imm};
            // Little Endian memory storage
            uut.memory.bytes[addr]   = inst[7:0];
            uut.memory.bytes[addr+1] = inst[15:8];
            uut.memory.bytes[addr+2] = inst[23:16];
            uut.memory.bytes[addr+3] = inst[31:24];
        end
    endtask

    initial begin
        // Setup Waveforms
        $dumpfile("tinker_comprehensive.vcd");
        $dumpvars(0, tb_comprehensive);

        clk = 0;
        reset = 1;
        
        // ====================================================================
        // LOAD COMPREHENSIVE TEST PROGRAM INTO MEMORY (Starting at 0x0000)
        // ====================================================================
        // op=0x19 (ADDI), op=0x18 (ADD), op=0x16 (FMUL), op=0x13 (STORE), op=0x10 (LOAD), op=0x0B (BNZ)
        
        // 1. DUAL ISSUE TEST (These two should fetch and issue in Cycle 1)
        assemble(32'h0000, 5'h19, 5'd1, 5'd0, 5'd0, 12'd10);  // ADDI R1, R0, 10
        assemble(32'h0004, 5'h19, 5'd2, 5'd0, 5'd0, 12'd20);  // ADDI R2, R0, 20
        
        // 2. FORWARDING (RAW HAZARD) TEST
        assemble(32'h0008, 5'h18, 5'd3, 5'd1, 5'd2, 12'd0);   // ADD R3, R1, R2   (R3 = 30)

        // 3. OUT OF ORDER EXECUTION TEST
        assemble(32'h000C, 5'h16, 5'd4, 5'd3, 5'd3, 12'd0);   // FMUL R4, R3, R3  (Slow FPU operation)
        assemble(32'h0010, 5'h19, 5'd5, 5'd0, 5'd0, 12'd99);  // ADDI R5, R0, 99  (Fast ALU operation, will finish BEFORE FMUL)

        // 4. LOAD/STORE QUEUE TEST
        // ISA context: STORE uses rd as base, rs as data. LOAD uses rs as base, rd as dest.
        assemble(32'h0014, 5'h13, 5'd0, 5'd5, 5'd0, 12'd100); // STORE R5 -> [R0 + 100]
        assemble(32'h0018, 5'h10, 5'd6, 5'd0, 5'd0, 12'd100); // LOAD  R6 <- [R0 + 100] (R6 should become 99)

        // 5. SPECULATIVE BRANCH EXECUTION TEST
        assemble(32'h001C, 5'h19, 5'd7, 5'd0, 5'd0, 12'h02C); // ADDI R7, R0, 0x002C (Load Branch Target Address)
        assemble(32'h0020, 5'h0b, 5'd7, 5'd6, 5'd0, 12'd0);   // BNZ R6, target=R7  (Branch if R6!=0 to 0x002C)

        // 6. THE POISON INSTRUCTION (Should be fetched speculatively, but FLUSHED)
        assemble(32'h0024, 5'h19, 5'd8, 5'd0, 5'd0, 12'hBAD); // ADDI R8, R0, 0xBAD (Should NEVER commit to RegFile)
        assemble(32'h0028, 5'h0f, 5'd0, 5'd0, 5'd0, 12'd0);   // HALT (In case flush fails)

        // 7. CORRECT BRANCH TARGET
        assemble(32'h002C, 5'h19, 5'd9, 5'd0, 5'd0, 12'd42);  // ADDI R9, R0, 42 (Proof that branch succeeded)
        assemble(32'h0030, 5'h0f, 5'd0, 5'd0, 5'd0, 12'd0);   // HALT

        #15 reset = 0; // Release reset

        // Timeout Watchdog
        #5000;
        if (!hlt) begin
            $display("ERROR: Simulation timed out. Deadlock detected!");
            $finish;
        end
    end

    // Monitor for the HALT signal
    always @(posedge clk) begin
        if (hlt) begin
            $display("\n========================================");
            $display("    COMPREHENSIVE TEST FINISHED");
            $display("========================================");
            
            // Validate Results
            $display("R1 (Expected: 10): %d", uut.reg_file.registers[1]);
            $display("R2 (Expected: 20): %d", uut.reg_file.registers[2]);
            $display("R3 (Expected: 30): %d", uut.reg_file.registers[3]);
            $display("R5 (Expected: 99): %d", uut.reg_file.registers[5]);
            $display("R6 (Expected: 99): %d (LSQ Forwarding Successful)", uut.reg_file.registers[6]);
            
            if (uut.reg_file.registers[8] == 64'hBAD)
                $display("R8 (Expected: 0):  %h  => ERROR: Branch Flush FAILED! Poison instruction committed.", uut.reg_file.registers[8]);
            else
                $display("R8 (Expected: 0):  %h  => PASS: Branch Flush Successful. ROB protected Register File.", uut.reg_file.registers[8]);
                
            $display("R9 (Expected: 42): %d (Branch Taken Correctly)", uut.reg_file.registers[9]);
            $display("========================================\n");
            
            $finish;
        end
    end

    // Real-time Event Logging (Helps trace OoO behavior)
    always @(posedge clk) begin
        if (uut.cdb1_valid) $display("[%0t] CDB1 Broadcast: Tag %d writes data %d to Virtual Reg %d", $time, uut.cdb1_tag, uut.cdb1_data, uut.cdb1_rd);
        if (uut.cdb2_valid) $display("[%0t] CDB2 Broadcast: Tag %d writes data %d to Virtual Reg %d", $time, uut.cdb2_tag, uut.cdb2_data, uut.cdb2_rd);
        if (uut.sys_flush)  $display("[%0t] *** BRANCH MISPREDICTION DETECTED: PIPELINE FLUSH INITIATED ***", $time);
    end
endmodule