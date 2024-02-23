`timescale 1ns / 1ps

// Generate and Propagate for a single bit
module gp1(input wire a, b, output wire g, p);
    assign g = a & b; 
    assign p = a | b; 
endmodule

// Computes aggregate generate/propagate signals over a 4-bit window
module gp4(input wire [3:0] gin, pin, input wire cin,
           output wire gout, pout, output wire [2:0] cout);
    // Direct computation of intermediate carries and gout/pout
    wire [3:1] c; // Intermediate carries no need for c[0] as it is cin

    assign c[1] = gin[0] | (pin[0] & cin);
    assign c[2] = gin[1] | (pin[1] & c[1]);
    assign c[3] = gin[2] | (pin[2] & c[2]);

    assign cout[0] = c[1];
    assign cout[1] = c[2];
    assign cout[2] = c[3];

    assign gout = gin[3] | (pin[3] & c[3]);
    assign pout = &pin; // AND all propagate signals
endmodule

// Extends gp4 for an 8-bit window
module gp8(input wire [7:0] gin, pin, input wire cin,
           output wire gout, pout, output wire [6:0] cout);
    // Split the 8-bit block into two 4-bit blocks and calculate gout/pout for each
    wire [2:0] cout_low, cout_high;
    wire gout_low, pout_low, gout_high, pout_high;

    // Lower 4-bit block
    gp4 lower_block(.gin(gin[3:0]), .pin(pin[3:0]), .cin(cin),
                    .gout(gout_low), .pout(pout_low), .cout(cout_low));

    // Higher 4-bit block, feeding the carry out of lower as carry in
    gp4 higher_block(.gin(gin[7:4]), .pin(pin[7:4]), .cin(cout_low[2]),
                     .gout(gout_high), .pout(pout_high), .cout(cout_high));

    // Combine the carries and compute overall gout and pout
    assign cout[2:0] = cout_low;
    assign cout[6:3] = cout_high; // Adjusted to correctly assign the high carries

    assign cout[6] = gout_high | (pout_high & cout_low[2]); // Ensure correct carry logic
    assign gout = gout_high | (pout_high & gout_low); // Overall gout for 8 bits
    assign pout = pout_low & pout_high; // Overall pout for 8 bits
endmodule

// 32-bit Carry Lookahead Adder using gp1 and gp8 blocks
module cla(input wire [31:0] a, b, input wire cin, output wire [31:0] sum);
    wire [31:0] g, p; // Generate and propagate signals for each bit
    wire [32:0] c; // Carry bits, including cin at c[0]

    // Generate and propagate signals for each bit
    generate
        genvar i;
        for (i = 0; i < 32; i = i + 1) begin: gp_gen
            gp1 gp_unit(.a(a[i]), .b(b[i]), .g(g[i]), .p(p[i]));
        end
    endgenerate

    // Initialize the carry chain
    assign c[0] = cin;

    // Instantiate gp8 blocks for each 8-bit segment of the operands
    gp8 gp8_0(.gin(g[7:0]), .pin(p[7:0]), .cin(c[0]), .gout(), .pout(), .cout(c[1:7]));
    gp8 gp8_1(.gin(g[15:8]), .pin(p[15:8]), .cin(c[8]), .gout(), .pout(), .cout(c[9:15]));
    gp8 gp8_2(.gin(g[23:16]), .pin(p[23:16]), .cin(c[16]), .gout(), .pout(), .cout(c[17:23]));
    gp8 gp8_3(.gin(g[31:24]), .pin(p[31:24]), .cin(c[24]), .gout(), .pout(), .cout(c[25:31]));

    // Compute the sum
    assign sum = a ^ b ^ c[31:0]; // Use the computed carries to calculate the sum
endmodule
