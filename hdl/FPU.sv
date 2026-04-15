module FPU (
    input clk, reset, flush,
    input valid_in, input [4:0] op, input [63:0] a, b,
    input [5:0] tag_in, input [4:0] rd_in,
    output ready_in, 
    output reg valid_out, output reg [63:0] res_out, output reg [5:0] tag_out, output reg [4:0] rd_out, input ack_out 
);
    assign ready_in = 1'b1; 
    
    reg val_s1; reg [4:0] op_s1; reg [5:0] tag_s1; reg [4:0] rd_s1;
    reg stg1_sign_a, stg1_sign_b; reg [10:0] stg1_eff_exp_a, stg1_eff_exp_b; reg [55:0] stg1_frac_a, stg1_frac_b; reg [52:0] stg1_m_a, stg1_m_b;
    reg stg1_a_is_nan, stg1_b_is_nan, stg1_a_is_inf, stg1_b_is_inf, stg1_a_is_zero, stg1_b_is_zero;

    reg val_s2; reg [4:0] op_s2; reg [5:0] tag_s2; reg [4:0] rd_s2;
    reg stg2_sign_res; reg [10:0] stg2_exp_res; reg [56:0] stg2_frac_add_res; reg signed [12:0] stg2_signed_exp;
    reg [106:0] stg2_raw_mul_res; reg [56:0] stg2_raw_div_res; reg stg2_special_case; reg [63:0] stg2_special_res; reg [52:0] stg2_m_b; reg [107:0] stg2_div_num;

    reg val_s3; reg [4:0] op_s3; reg [5:0] tag_s3; reg [4:0] rd_s3;
    reg stg3_sign_res; reg [10:0] stg3_exp_res; reg [56:0] stg3_frac_add_res; reg signed [12:0] stg3_signed_exp;
    reg [106:0] stg3_raw_mul_res; reg [56:0] stg3_raw_div_res; reg stg3_special_case; reg [63:0] stg3_special_res; reg [52:0] stg3_m_b; reg [107:0] stg3_div_num;

    reg [10:0] v_exp_diff, v_exp_res; reg [56:0] v_frac_add_res; reg [55:0] v_shift_mask; reg [106:0] v_mul_shift_mask; reg [5:0] v_shift_amt;
    reg v_G, v_R, v_S, v_LSB, v_round_up; reg signed [12:0] v_signed_exp; integer i;

    always @(posedge clk) begin
        // Top-of-block declarations for Yosys
        reg [55:0] local_frac_a, local_frac_b;
        reg [106:0] local_raw_mul_res;
        reg [56:0] local_raw_div_res;

        if (reset || flush) begin
            val_s1 <= 0; val_s2 <= 0; val_s3 <= 0; valid_out <= 0; res_out <= 64'b0; tag_out <= 0; rd_out <= 0;
        end else begin
            if (ack_out) valid_out <= 0;

            val_s1 <= valid_in; op_s1 <= op; tag_s1 <= tag_in; rd_s1 <= rd_in;
            stg1_sign_a <= a[63]; stg1_sign_b <= (op == 5'h15) ? ~b[63] : b[63]; 
            stg1_eff_exp_a <= (a[62:52] == 0) ? 11'd1 : a[62:52]; stg1_eff_exp_b <= (b[62:52] == 0) ? 11'd1 : b[62:52];
            stg1_frac_a <= { (a[62:52] != 0), a[51:0], 3'b000 }; stg1_frac_b <= { (b[62:52] != 0), b[51:0], 3'b000 };
            stg1_m_a <= { (a[62:52] != 0), a[51:0] }; stg1_m_b <= { (b[62:52] != 0), b[51:0] };
            stg1_a_is_nan <= (a[62:52] == 11'h7FF) && (a[51:0] != 0); stg1_b_is_nan <= (b[62:52] == 11'h7FF) && (b[51:0] != 0);
            stg1_a_is_inf <= (a[62:52] == 11'h7FF) && (a[51:0] == 0); stg1_b_is_inf <= (b[62:52] == 11'h7FF) && (b[51:0] == 0);
            stg1_a_is_zero <= (a[62:52] == 0) && (a[51:0] == 0); stg1_b_is_zero <= (b[62:52] == 0) && (b[51:0] == 0);

            val_s2 <= val_s1; op_s2 <= op_s1; tag_s2 <= tag_s1; rd_s2 <= rd_s1;
            stg2_special_case <= 0; stg2_special_res <= 0; stg2_m_b <= stg1_m_b;
            
            if (op_s1 == 5'h14 || op_s1 == 5'h15) begin 
                local_frac_a = stg1_frac_a; local_frac_b = stg1_frac_b; v_exp_res = stg1_eff_exp_a;
                if (stg1_eff_exp_a > stg1_eff_exp_b) begin
                    v_exp_diff = stg1_eff_exp_a - stg1_eff_exp_b; v_exp_res  = stg1_eff_exp_a;
                    if (v_exp_diff > 55) local_frac_b = {55'b0, |local_frac_b};
                    else begin v_shift_mask = (56'd1 << v_exp_diff) - 56'd1; local_frac_b = (local_frac_b >> v_exp_diff) | {55'b0, |(local_frac_b & v_shift_mask)}; end
                end else if (stg1_eff_exp_b > stg1_eff_exp_a) begin
                    v_exp_diff = stg1_eff_exp_b - stg1_eff_exp_a; v_exp_res  = stg1_eff_exp_b;
                    if (v_exp_diff > 55) local_frac_a = {55'b0, |local_frac_a};
                    else begin v_shift_mask = (56'd1 << v_exp_diff) - 56'd1; local_frac_a = (local_frac_a >> v_exp_diff) | {55'b0, |(local_frac_a & v_shift_mask)}; end
                end
                
                if (stg1_sign_a == stg1_sign_b) begin stg2_frac_add_res <= local_frac_a + local_frac_b; stg2_sign_res <= stg1_sign_a; end 
                else begin
                    if (local_frac_a >= local_frac_b) begin stg2_frac_add_res <= local_frac_a - local_frac_b; stg2_sign_res <= stg1_sign_a; end 
                    else begin stg2_frac_add_res <= local_frac_b - local_frac_a; stg2_sign_res <= stg1_sign_b; end
                end
                stg2_exp_res <= v_exp_res;
            end
            else if (op_s1 == 5'h16) begin 
                stg2_sign_res <= stg1_sign_a ^ stg1_sign_b;
                if (stg1_a_is_nan || stg1_b_is_nan || (stg1_a_is_inf && stg1_b_is_zero) || (stg1_a_is_zero && stg1_b_is_inf)) begin stg2_special_case <= 1; stg2_special_res <= {1'b0, 11'h7FF, 52'h8000000000000}; end 
                else if (stg1_a_is_inf || stg1_b_is_inf) begin stg2_special_case <= 1; stg2_special_res <= {stg1_sign_a ^ stg1_sign_b, 11'h7FF, 52'b0}; end 
                else if (stg1_a_is_zero || stg1_b_is_zero) begin stg2_special_case <= 1; stg2_special_res <= {stg1_sign_a ^ stg1_sign_b, 63'b0}; end 
                else begin stg2_signed_exp <= stg1_eff_exp_a + stg1_eff_exp_b - 1023 + 1; stg2_raw_mul_res <= stg1_m_a * stg1_m_b; end
            end
            else if (op_s1 == 5'h17) begin 
                stg2_sign_res <= stg1_sign_a ^ stg1_sign_b;
                if (stg1_a_is_nan || stg1_b_is_nan || (stg1_a_is_inf && stg1_b_is_inf) || (stg1_a_is_zero && stg1_b_is_zero)) begin stg2_special_case <= 1; stg2_special_res <= {1'b0, 11'h7FF, 52'h8000000000000}; end 
                else if (stg1_a_is_inf || stg1_b_is_zero) begin stg2_special_case <= 1; stg2_special_res <= {stg1_sign_a ^ stg1_sign_b, 11'h7FF, 52'b0}; end 
                else if (stg1_b_is_inf || stg1_a_is_zero) begin stg2_special_case <= 1; stg2_special_res <= {stg1_sign_a ^ stg1_sign_b, 63'b0}; end 
                else begin stg2_signed_exp <= stg1_eff_exp_a - stg1_eff_exp_b + 1023; stg2_div_num <= {stg1_m_a, 55'b0}; stg2_raw_div_res <= {stg1_m_a, 55'b0} / stg1_m_b; end
            end

            val_s3 <= val_s2; op_s3 <= op_s2; tag_s3 <= tag_s2; rd_s3 <= rd_s2;
            stg3_sign_res <= stg2_sign_res; stg3_special_case <= stg2_special_case; stg3_special_res <= stg2_special_res; stg3_m_b <= stg2_m_b; stg3_div_num <= stg2_div_num;
            
            if (op_s2 == 5'h14 || op_s2 == 5'h15) begin 
                v_frac_add_res = stg2_frac_add_res; v_exp_res = stg2_exp_res;
                if (v_frac_add_res != 0) begin
                    if (v_frac_add_res[56]) begin v_frac_add_res = (v_frac_add_res >> 1) | {56'b0, v_frac_add_res[0]}; v_exp_res = v_exp_res + 1; end 
                    else begin
                        for (i = 55; i >= 0; i = i - 1) begin
                            if (v_frac_add_res[55] == 0 && v_exp_res > 0) begin v_frac_add_res = v_frac_add_res << 1; v_exp_res = v_exp_res - 1; end
                        end
                    end
                    if (v_exp_res == 0 && v_frac_add_res[55] == 1) v_exp_res = 0; 
                end
                stg3_frac_add_res <= v_frac_add_res; stg3_exp_res <= v_exp_res;
            end
            else if (op_s2 == 5'h16 && !stg2_special_case) begin 
                local_raw_mul_res = stg2_raw_mul_res; v_signed_exp = stg2_signed_exp;
                if (local_raw_mul_res != 0) begin
                    for (i = 105; i >= 0; i = i - 1) begin
                        if (local_raw_mul_res[105] == 0 && v_signed_exp > 1) begin local_raw_mul_res = local_raw_mul_res << 1; v_signed_exp = v_signed_exp - 1; end
                    end
                    if (v_signed_exp < 1) begin
                        v_shift_amt = 1 - v_signed_exp;
                        if (v_shift_amt > 106) local_raw_mul_res = 0;
                        else begin v_mul_shift_mask = (107'd1 << v_shift_amt) - 1; local_raw_mul_res = (local_raw_mul_res >> v_shift_amt) | {106'b0, |(local_raw_mul_res & v_mul_shift_mask)}; end
                        v_signed_exp = 0;
                    end else if (local_raw_mul_res[105] == 0) begin v_signed_exp = 0; end
                end
                stg3_raw_mul_res <= local_raw_mul_res; stg3_signed_exp <= v_signed_exp;
            end
            else if (op_s2 == 5'h17 && !stg2_special_case) begin 
                local_raw_div_res = stg2_raw_div_res; v_signed_exp = stg2_signed_exp;
                if (local_raw_div_res != 0) begin
                    for (i = 55; i >= 0; i = i - 1) begin
                        if (local_raw_div_res[55] == 0 && v_signed_exp > 1) begin local_raw_div_res = local_raw_div_res << 1; v_signed_exp = v_signed_exp - 1; end
                    end
                    if (v_signed_exp < 1) begin
                        v_shift_amt = 1 - v_signed_exp;
                        if (v_shift_amt > 56) local_raw_div_res = 0;
                        else begin v_shift_mask = (57'd1 << v_shift_amt) - 1; local_raw_div_res = (local_raw_div_res >> v_shift_amt) | {56'b0, |(local_raw_div_res & v_shift_mask)}; end
                        v_signed_exp = 0;
                    end else if (local_raw_div_res[55] == 0) begin v_signed_exp = 0; end
                end
                stg3_raw_div_res <= local_raw_div_res; stg3_signed_exp <= v_signed_exp;
            end

            if (val_s3) begin
                valid_out <= 1; tag_out <= tag_s3; rd_out <= rd_s3;    
                if (stg3_special_case) begin res_out <= stg3_special_res; end 
                else begin
                    if (op_s3 == 5'h14 || op_s3 == 5'h15) begin 
                        v_frac_add_res = stg3_frac_add_res; v_exp_res = stg3_exp_res;
                        if (v_frac_add_res == 0) res_out <= 64'b0;
                        else begin
                            v_LSB = v_frac_add_res[3]; v_G = v_frac_add_res[2]; v_R = v_frac_add_res[1]; v_S = v_frac_add_res[0];
                            v_round_up = v_G & (v_R | v_S | v_LSB);
                            if (v_round_up) begin
                                v_frac_add_res = v_frac_add_res + 4'b1000;
                                if (v_frac_add_res[56]) begin v_frac_add_res = v_frac_add_res >> 1; v_exp_res = v_exp_res + 1; end
                            end
                            res_out <= {stg3_sign_res, v_exp_res, v_frac_add_res[54:3]};
                        end
                    end 
                    else if (op_s3 == 5'h16) begin 
                        local_raw_mul_res = stg3_raw_mul_res; v_signed_exp = stg3_signed_exp;
                        v_LSB = local_raw_mul_res[53]; v_G = local_raw_mul_res[52]; v_R = local_raw_mul_res[51]; v_S = |local_raw_mul_res[50:0];
                        v_round_up = v_G & (v_R | v_S | v_LSB);
                        if (v_round_up) begin
                            local_raw_mul_res = local_raw_mul_res + (107'b1 << 53);
                            if (local_raw_mul_res[106]) begin local_raw_mul_res = local_raw_mul_res >> 1; v_signed_exp = v_signed_exp + 1; end
                        end
                        if (v_signed_exp >= 2047) v_exp_res = 11'h7FF; else v_exp_res = v_signed_exp[10:0];
                        res_out <= {stg3_sign_res, v_exp_res, local_raw_mul_res[104:53]};
                    end
                    else if (op_s3 == 5'h17) begin 
                        local_raw_div_res = stg3_raw_div_res; v_signed_exp = stg3_signed_exp;
                        v_LSB = local_raw_div_res[3]; v_G = local_raw_div_res[2]; v_R = local_raw_div_res[1]; 
                        v_S = |(stg3_div_num % stg3_m_b) | local_raw_div_res[0]; 
                        v_round_up = v_G & (v_R | v_S | v_LSB);
                        if (v_round_up) begin
                            local_raw_div_res = local_raw_div_res + 4'b1000;
                            if (local_raw_div_res[56]) begin local_raw_div_res = local_raw_div_res >> 1; v_signed_exp = v_signed_exp + 1; end
                        end
                        if (v_signed_exp >= 2047) v_exp_res = 11'h7FF; else v_exp_res = v_signed_exp[10:0];
                        res_out <= {stg3_sign_res, v_exp_res, local_raw_div_res[54:3]};
                    end else begin res_out <= 64'b0; end
                end
            end
        end
    end
endmodule