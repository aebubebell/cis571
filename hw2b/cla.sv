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

    wire [3:0] g, p;
    wire [3:1] c;

    // Generate and propagate for each bit
    assign g = gin;
    assign p = pin;

    // Carry calculations
    assign c[1] = g[0] | (p[0] & cin);
    assign c[2] = g[1] | (p[1] & c[1]);
    assign c[3] = g[2] | (p[2] & c[2]);
    assign cout = c[3:1];

    // Overall generate and propagate
    assign gout = g[3] | (p[3] & c[3]);
    assign pout = p[3] & p[2] & p[1] & p[0];

endmodule


/** Same as gp4 but for an 8-bit window instead */
module gp8(input wire [7:0] gin, pin,
           input wire cin,
           output wire gout, pout,
           output wire [6:0] cout);

    wire [7:0] g, p;
    wire [7:1] c;

    // Generate and propagate for each bit
    assign g = gin;
    assign p = pin;

    // Carry calculations
    assign c[1] = g[0] | (p[0] & cin);
    assign c[2] = g[1] | (p[1] & c[1]);
    assign c[3] = g[2] | (p[2] & c[2]);
    assign c[4] = g[3] | (p[3] & c[3]);
    assign c[5] = g[4] | (p[4] & c[4]);
    assign c[6] = g[5] | (p[5] & c[5]);
    assign c[7] = g[6] | (p[6] & c[6]);
    assign cout = c[7:1];

    // Overall generate and propagate
    assign gout = g[7] | (p[7] & c[7]);
    assign pout = p[7] & p[6] & p[5] & p[4] & p[3] & p[2] & p[1] & p[0];

endmodule


`timescale 1ns / 1ps

module cla(
    input wire [31:0] a,
    input wire [31:0] b,
    input wire cin,
    output wire [31:0] sum
);
    wire [31:0] g, p; // Generate and propagate signals for each bit
    wire [7:0] gout, pout; // Output generate and propagate signals for 8-bit blocks
    wire [31:0] cout; // Carry out signals, including carry between 4-bit blocks

    // Instantiate gp1 modules for each bit to compute generate and propagate signals
    genvar i;
    generate
        for (i = 0; i < 32; i = i + 1) begin : gp1_blocks
            gp1 gp1_inst(.a(a[i]), .b(b[i]), .g(g[i]), .p(p[i]));
        end
    endgenerate

    // Instantiate gp4 modules for each 4-bit block to compute intermediate generate, propagate, and carry signals
    gp4 gp4_block0(.gin(g[3:0]), .pin(p[3:0]), .cin(cin), .gout(gout[0]), .pout(pout[0]), .cout(cout[2:0]));
    gp4 gp4_block1(.gin(g[7:4]), .pin(p[7:4]), .cin(cout[3]), .gout(gout[1]), .pout(pout[1]), .cout(cout[6:4]));
    gp4 gp4_block2(.gin(g[11:8]), .pin(p[11:8]), .cin(cout[7]), .gout(gout[2]), .pout(pout[2]), .cout(cout[10:8]));
    gp4 gp4_block3(.gin(g[15:12]), .pin(p[15:12]), .cin(cout[11]), .gout(gout[3]), .pout(pout[3]), .cout(cout[14:12]));
    gp4 gp4_block4(.gin(g[19:16]), .pin(p[19:16]), .cin(cout[15]), .gout(gout[4]), .pout(pout[4]), .cout(cout[18:16]));
    gp4 gp4_block5(.gin(g[23:20]), .pin(p[23:20]), .cin(cout[19]), .gout(gout[5]), .pout(pout[5]), .cout(cout[22:20]));
    gp4 gp4_block6(.gin(g[27:24]), .pin(p[27:24]), .cin(cout[23]), .gout(gout[6]), .pout(pout[6]), .cout(cout[26:24]));
    gp4 gp4_block7(.gin(g[31:28]), .pin(p[31:28]), .cin(cout[27]), .gout(gout[7]), .pout(pout[7]), .cout(cout[30:28]));

    //

    // Compute the sum for each bit, including the initial carry-in for the least significant bit
    assign sum = a ^ b ^ {gout[7] | (pout[7] & cout[27]), cout[30:0]};

endmodule
