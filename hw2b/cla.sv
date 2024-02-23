`timescale 1ns / 1ps

/**
 * @param a first 1-bit input
 * @param b second 1-bit input
 * @param g whether a and b generate a carry
 * @param p whether a and b would propagate an incoming carry
 */
module gp1(input wire a, b,
           output wire g, p);
   assign g = a & b;
   assign p = a | b;
endmodule

/**
 * Computes aggregate generate/propagate signals over a 4-bit window.
 * @param gin incoming generate signals
 * @param pin incoming propagate signals
 * @param cin the incoming carry
 * @param gout whether these 4 bits internally would generate a carry-out (independent of cin)
 * @param pout whether these 4 bits internally would propagate an incoming carry from cin
 * @param cout the carry outs for the low-order 3 bits
 */
module gp4(input wire [3:0] gin, pin,
           input wire cin,
           output wire gout, pout,
           output wire [2:0] cout);

   // Intermediate signals for carry calculation
    wire [2:0] c_intermediate;

    // Calculate intermediate carries
    assign c_intermediate[0] = gin[0] | (pin[0] & cin);
    assign c_intermediate[1] = gin[1] | (pin[1] & c_intermediate[0]);
    assign c_intermediate[2] = gin[2] | (pin[2] & c_intermediate[1]);

    // Output carries for the lower 3 bits
    assign cout = c_intermediate;

    // Compute aggregate generate and propagate signals
    assign gout = gin[3] | (pin[3] & c_intermediate[2]);
    assign pout = pin[0] & pin[1] & pin[2] & pin[3];


endmodule

/** Same as gp4 but for an 8-bit window instead */
module gp8(input wire [7:0] gin, pin,
           input wire cin,
           output wire gout, pout,
           output wire [6:0] cout);

   wire gout_low, pout_low, gout_high, pout_high;
    wire [2:0] cout_low, cout_high;

    // Lower 4-bit gp4 block
    gp4 lower_gp4(.gin(gin[3:0]), .pin(pin[3:0]), .cin(cin),
                  .gout(gout_low), .pout(pout_low), .cout(cout[2:0]));

    // Higher 4-bit gp4 block, with carry input from lower block
    gp4 higher_gp4(.gin(gin[7:4]), .pin(pin[7:4]), .cin(cout[2]),
                   .gout(gout_high), .pout(pout_high), .cout(cout[5:3]));

    // Calculate the carry out of the 7th bit
    assign cout[6] = gout_high | (pout_high & cout[2]);

    // Aggregate generate and propagate for the 8-bit block
    assign gout = gout_high | (pout_high & gout_low);
    assign pout = pout_low & pout_high;

endmodule

module cla(input wire [31:0] a, b,
           input wire cin,
           output wire [31:0] sum);

    wire [31:0] g, p;
    wire [32:0] c; // Extending carry to include cin at c[0]

    // Generate and propagate signals for each bit
    genvar i;
    generate
        for (i = 0; i < 32; i = i + 1) begin: gp_gen
            gp1 gp_unit(.a(a[i]), .b(b[i]), .g(g[i]), .p(p[i]));
        end
    endgenerate

    // Correcting bit range and assignment issues
    assign c[0] = cin;
    
    // Instantiating gp8 modules with corrected bit ranges and carry chain
    gp8 gp8_0(.gin(g[7:0]), .pin(p[7:0]), .cin(c[0]), .gout(), .pout(), .cout(c[1:7]));
    gp8 gp8_1(.gin(g[15:8]), .pin(p[15:8]), .cin(c[8]), .gout(), .pout(), .cout(c[9:15]));
    gp8 gp8_2(.gin(g[23:16]), .pin(p[23:16]), .cin(c[16]), .gout(), .pout(), .cout(c[17:23]));
    gp8 gp8_3(.gin(g[31:24]), .pin(p[31:24]), .cin(c[24]), .gout(), .pout(), .cout(c[25:31]));

    // Calculate the sum bits
    assign sum = a ^ b ^ c[31:0]; // Use only the relevant carry bits for sum calculation
endmodule
