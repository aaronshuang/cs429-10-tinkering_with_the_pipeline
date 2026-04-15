module lsq #(parameter DEPTH = 4) (
    input clk, reset, flush,
    input alloc_en, input is_store, input [4:0] dest_rd, input [63:0] imm,
    input addr_vld, input [63:0] addr_val, input [5:0] addr_tag,
    input data_vld, input [63:0] data_val, input [5:0] data_tag, input [5:0] alloc_tag,
    output full, output [1:0] current_tail,
    input cdb1_valid, input [5:0] cdb1_tag, input [63:0] cdb1_data,
    input cdb2_valid, input [5:0] cdb2_tag, input [63:0] cdb2_data,
    output wire mem_read, output wire mem_write, output wire [63:0] mem_addr, output wire [63:0] mem_wdata,
    input [63:0] mem_rdata,
    output reg cdb_valid, output reg [5:0] cdb_tag, output reg [4:0] cdb_rd, output reg [63:0] cdb_data, input cdb_ack
);
    reg valid [0:DEPTH-1]; reg is_st [0:DEPTH-1]; reg [4:0] rd [0:DEPTH-1]; reg [63:0] offset [0:DEPTH-1];
    reg a_vld [0:DEPTH-1]; reg [63:0] a_val [0:DEPTH-1]; reg [5:0] a_tag [0:DEPTH-1];
    reg d_vld [0:DEPTH-1]; reg [63:0] d_val [0:DEPTH-1]; reg [5:0] d_tag [0:DEPTH-1]; reg [5:0] tag [0:DEPTH-1];
    reg [1:0] head, tail; reg [2:0] count; reg waiting_cdb;

    assign full = (count == DEPTH); assign current_tail = tail;
    wire head_is_st = is_st[head];
    wire head_ready = (count > 0) && a_vld[head] && (!head_is_st || d_vld[head]);
    wire [63:0] eff_addr = a_val[head] + offset[head];
    assign mem_addr = head_ready ? eff_addr : 64'b0;
    assign mem_wdata = head_ready ? d_val[head] : 64'b0;
    assign mem_read = head_ready && !head_is_st;
    assign mem_write = head_ready && head_is_st && !waiting_cdb;

    wire push = alloc_en && !full;
    wire pop_store = head_ready && !waiting_cdb && head_is_st;
    wire pop_load = waiting_cdb && cdb_ack;
    wire pop = pop_store || pop_load;

    integer i;
    always @(posedge clk) begin
        if (reset || flush) begin 
            head <= 0; tail <= 0; count <= 0; waiting_cdb <= 0; cdb_valid <= 0;
            for(i=0; i<DEPTH; i=i+1) valid[i] <= 0;
        end else begin
            if (push && !pop) count <= count + 1; else if (!push && pop) count <= count - 1;
            if (push) tail <= (tail + 1) % DEPTH; if (pop) head <= (head + 1) % DEPTH;
            if (cdb_ack) begin cdb_valid <= 0; waiting_cdb <= 0; end

            if (push) begin
                valid[tail] <= 1; is_st[tail] <= is_store; rd[tail] <= dest_rd; offset[tail] <= imm;
                a_vld[tail] <= addr_vld; a_val[tail] <= addr_val; a_tag[tail] <= addr_tag;
                d_vld[tail] <= data_vld; d_val[tail] <= data_val; d_tag[tail] <= data_tag; tag[tail] <= alloc_tag;
            end

            for (i=0; i<DEPTH; i=i+1) begin
                if (valid[i]) begin
                    if (!a_vld[i]) begin
                        if (cdb1_valid && a_tag[i] == cdb1_tag) begin a_val[i] <= cdb1_data; a_vld[i] <= 1; end
                        else if (cdb2_valid && a_tag[i] == cdb2_tag) begin a_val[i] <= cdb2_data; a_vld[i] <= 1; end
                    end
                    if (is_st[i] && !d_vld[i]) begin
                        if (cdb1_valid && d_tag[i] == cdb1_tag) begin d_val[i] <= cdb1_data; d_vld[i] <= 1; end
                        else if (cdb2_valid && d_tag[i] == cdb2_tag) begin d_val[i] <= cdb2_data; d_vld[i] <= 1; end
                    end
                end
            end

            if (head_ready && !head_is_st && !waiting_cdb) begin
                cdb_valid <= 1; cdb_data <= mem_rdata; cdb_tag <= tag[head]; cdb_rd <= rd[head];
                waiting_cdb <= 1; valid[head] <= 0;
            end else if (pop_store) begin
                valid[head] <= 0; 
            end
        end
    end
endmodule