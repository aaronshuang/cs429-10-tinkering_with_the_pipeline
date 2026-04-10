`timescale 1ns / 1ps

module tb_tinker_2;

    reg clk;
    reg reset;

    tinker_core uut (.clk(clk), .reset(reset));

    always #5 clk = ~clk;

    integer passed_tests = 0;
    integer total_tests = 0;

    task assert_eq;
        input [63:0] expected;
        input [63:0] actual;
        input [255:0] test_name;
        begin
            total_tests = total_tests + 1;
            if (expected === actual) begin
                $display("[PASS] %s", test_name);
                passed_tests = passed_tests + 1;
            end else begin
                $display("[FAIL] %s | Expected: %h, Got: %h", test_name, expected, actual);
            end
        end
    endtask

    task write_inst;
        input [63:0] addr;
        input [4:0]  op;
        input [4:0]  rd;
        input [4:0]  rs;
        input [4:0]  rt;
        input [11:0] imm;
        reg   [31:0] inst;
        begin
            inst = {op, rd, rs, rt, imm};
            uut.memory.bytes[addr]   = inst[7:0];
            uut.memory.bytes[addr+1] = inst[15:8];
            uut.memory.bytes[addr+2] = inst[23:16];
            uut.memory.bytes[addr+3] = inst[31:24];
        end
    endtask

    initial begin
        $dumpfile("tinker_internals.vcd");
        $dumpvars(0, tb_tinker_2);
        
        clk = 0;
        reset = 1;

        write_inst(16'h2000, 5'h19, 5'd1, 5'd0, 5'd0, 12'd15);
        write_inst(16'h2004, 5'h13, 5'd1, 5'd2, 5'd0, 12'd8);
        write_inst(16'h2008, 5'h14, 5'd5, 5'd3, 5'd4, 12'd0);
        write_inst(16'h200C, 5'h0E, 5'd8, 5'd6, 5'd7, 12'd0);

        #12 reset = 0; 

        // Testing ADDI
        @(posedge clk);
        #1; 
        assert_eq(5'h19, uut.opcode, "Decoder Opcode Extraction (0x19)");
        assert_eq(5'd1, uut.rd, "Decoder Destination Register (r1)");
        assert_eq(12'd15, uut.imm, "Decoder Immediate Extraction (15)");
        assert_eq(1'b1, uut.use_immediate, "Decoder 'use_immediate' Flag Active");
        
        @(posedge clk);
        #1;
        assert_eq(64'd0, uut.alu_input_a, "ALU Input A correctly reads latched A (0)");
        assert_eq(64'd15, uut.alu_input_b, "ALU Input B correctly routes immediate (15)");
        assert_eq(64'd15, uut.alu_res, "ALU computes correct result (15)");
        
        @(posedge clk);
        #1;
        assert_eq(1'b1, uut.reg_write_en, "Decoder 'reg_write_en' Flag Active in WB");
        assert_eq(64'd15, uut.reg_write_data, "Write-back MUX routes ALU result");

        @(posedge clk); 
        uut.reg_file.registers[1] = 64'd15;
        uut.reg_file.registers[2] = 64'hDEADBEEF;

        // Testing STORE
        @(posedge clk);
        @(posedge clk);
        @(posedge clk);
        #1;
        assert_eq(1'b1, uut.mem_write, "Decoder 'mem_write' Flag Active");
        assert_eq(1'b0, uut.reg_write_en, "Decoder disables register write");
        assert_eq(64'd23, uut.mem_addr, "Memory MUX correctly calculates Address (15+8=23)");
        assert_eq(64'hDEADBEEF, uut.mem_wdata, "Memory MUX correctly routes r2 to wdata");

        @(posedge clk);
        uut.reg_file.registers[3] = 64'h3FF8000000000000;
        uut.reg_file.registers[4] = 64'h4000000000000000;

        // Testing ADDF
        @(posedge clk);
        @(posedge clk);
        @(posedge clk);
        #1;
        assert_eq(1'b1, uut.use_fpu_instruction, "Decoder flags FPU instruction");
        assert_eq(64'h400C000000000000, uut.fpu_res, "FPU correctly adds 1.5 + 2.0 = 3.5");
        assert_eq(64'h400C000000000000, uut.reg_write_data, "Write-back MUX properly prioritizes FPU res");

        @(posedge clk);
        uut.reg_file.registers[6] = 64'd100;
        uut.reg_file.registers[7] = 64'd50;
        uut.reg_file.registers[8] = 64'h3000;

        // Testing BRGT
        @(posedge clk);
        @(posedge clk);
        #1;
        assert_eq(1'b1, uut.is_branch, "Decoder flags instruction as a Branch");
        assert_eq(1'b1, uut.take_branch, "Branch Evaluator registers r6 > r7 (TRUE)");
        assert_eq(64'h3000, uut.branch_target, "Branch Evaluator calculates target address (0x3000)");

        @(posedge clk);
        #1;
        assert_eq(64'h3000, uut.pc, "Fetch Unit correctly applies branch target to PC");

        if (passed_tests == total_tests)
            $display("   ALL %0d INTERNAL DATAPATH TESTS PASSED!", total_tests);
        else
            $display("   FAILED %0d / %0d TESTS.", (total_tests - passed_tests), total_tests);

        $finish;
    end

endmodule