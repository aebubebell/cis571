`timescale 1ns / 1ps


module gp1(input wire a, b, output wire g, p);
    assign g = a & b; 
    assign p = a | b; 
endmodule

module gp4(input wire [3:0] gin, pin,
           input wire cin,
           output wire gout, pout,
           output wire [2:0] cout);

    wire [3:0] c_internal;
    assign c_internal[0] = cin;
    assign c_internal[1] = gin[0] | (pin[0] & c_internal[0]);
    assign c_internal[2] = gin[1] | (pin[1] & c_internal[1]);
    assign c_internal[3] = gin[2] | (pin[2] & c_internal[2]);

    assign cout[0] = c_internal[1];
    assign cout[1] = c_internal[2];
    assign cout[2] = c_internal[3];

    assign gout = gin[3] | (pin[3] & c_internal[3]);
    assign pout = pin[0] & pin[1] & pin[2] & pin[3];
endmodule

module gp8(input wire [7:0] gin, pin,
           input wire cin,
           output wire gout, pout,
           output wire [6:0] cout);  // Assuming cout is 7 bits for 8-bit gp calculation

    wire [7:0] carry_out;
    wire [7:0] gen_modified;


assign carry_out = {gin[6] | (pin[6] & carry_out[5]),
                    gin[5] | (pin[5] & carry_out[4]),
                    gin[4] | (pin[4] & carry_out[3]),
                    gin[3] | (pin[3] & carry_out[2]),
                    gin[2] | (pin[2] & carry_out[1]),
                    gin[1] | (pin[1] & carry_out[0]),
                    gin[0] | (pin[0] & cin),
                    1'b0}; // The last bit is dummy for alignment


assign cout = carry_out[6:0];


assign pout = pin[0] && pin[1] && pin[2] && pin[3] && pin[4] && pin[5] && pin[6] && pin[7];


integer j;
always @(*) begin
    gen_modified[7] = gin[7]; // Direct assignment for the highest bit
    for (j = 6; j >= 0; j = j - 1) begin
        gen_modified[j] = gin[j] && (&pin[j + 1:7]); 
    end
end


assign gout = |gen_modified;
endmodule


// 
module cla(
    input wire [31:0] a, b,
    input wire cin,
    output wire [31:0] sum
);
    wire [31:0] g, p, c;
    wire [7:0] gin, pin; 
    wire [3:0] g_inter, p_inter;
    wire [2:0] c_inter;
    wire gout_inter, pout_inter;
    wire [7:0] gen_modified; 


    generate
        genvar j;
        for (j = 0; j < 8; j++) begin : gen_logic
            if (j < 7) begin : less_than_seven
                assign gen_modified[j] = g[j] & (&p[j+1:7]);
            end else begin : equal_to_seven
                assign gen_modified[7] = g[7]; // Handle the last bit separately
            end
        end
    endgenerate

  
    assign sum[0] = a[0] ^ b[0] ^ cin;
    for (j = 1; j < 32; j++) begin : sum_calculation
        assign sum[j] = a[j] ^ b[j] ^ c[j-1];
    end

endmodule
