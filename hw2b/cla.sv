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

module gp8(
    input wire [7:0] gin, pin,
    input wire cin,
    output wire gout, pout,
    output wire [6:0] cout
);
    wire [7:0] c_internal;
    assign c_internal[0] = cin;
    genvar i;
    generate
        for (i = 0; i < 7; i = i + 1) begin : gen_gp8_carry
            assign c_internal[i + 1] = gin[i] | (pin[i] & c_internal[i]);
        end
    endgenerate

    assign cout = c_internal[1:7]; // Ensure correct bit ordering
    assign gout = gin[7] | (pin[7] & c_internal[7]);
    assign pout = &pin[0:7];
endmodule

// 
module cla(
    input wire [31:0] a, b,
    input wire cin,
    output wire [31:0] sum
);
    wire [31:0] g, p, c;
    wire [7:0] g8, p8;
    // Corrected: Define cout8 as a 4-bit wide signal to hold the output of gp8 instances.
    wire [3:0] cout8; // Adjusted from previous incorrect usage

    genvar i;

    // Instantiate gp1 modules for each bit
    generate
        for (i = 0; i < 32; i = i + 1) begin : gen_gp1
            gp1 gp1_inst(.a(a[i]), .b(b[i]), .g(g[i]), .p(p[i]));
        end
    endgenerate

    //
    gp8 gp8_inst0(.gin(g[7:0]), .pin(p[7:0]), .cin(cin), .gout(g8[0]), .pout(p8[0]), .cout({cout8[0]})); // Adjusted for bit width
    gp8 gp8_inst1(.gin(g[15:8]), .pin(p[15:8]), .cin(cout8[0]), .gout(g8[1]), .pout(p8[1]), .cout({cout8[1]}));
    gp8 gp8_inst2(.gin(g[23:16]), .pin(p[23:16]), .cin(cout8[1]), .gout(g8[2]), .pout(p8[2]), .cout({cout8[2]}));
    gp8 gp8_inst3(.gin(g[31:24]), .pin(p[31:24]), .cin(cout8[2]), .gout(g8[3]), .pout(p8[3]), .cout({cout8[3]}));

    // Correct carry calculation with named blocks and fix slice ranges
    assign c[0] = cin;
    generate
        for (i = 1; i < 32; i = i + 1) begin : carry_logic
            if (i % 8 == 0) begin : blk_carry_boundary
                assign c[i] = cout8[i/8-1];
            end else begin : blk_carry_internal
                assign c[i] = g[i-1] | (p[i-1] & c[i-1]);
            end
        end
    endgenerate

    // Calculate sum
    generate
        for (i = 0; i < 32; i = i + 1) begin : sum_calc
            assign sum[i] = a[i] ^ b[i] ^ c[i];
        end
    endgenerate

endmodule
