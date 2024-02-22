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

// Compute intermediate generate and propagate signals
wire [3:0] g_intermediate, p_intermediate;
assign g_intermediate[0] = gin[0] | (pin[0] & cin);
assign p_intermediate[0] = pin[0];

genvar i;
generate
    for (i = 1; i < 4; i = i + 1) begin : gp4_logic
        assign g_intermediate[i] = gin[i] | (pin[i] & g_intermediate[i-1]);
        assign p_intermediate[i] = pin[i] & p_intermediate[i-1];
    end
endgenerate

// The carry out for the entire 4-bit block
assign gout = g_intermediate[3];
// Whether an incoming carry would be propagated through the entire 4-bit block
assign pout = p_intermediate[3];

// The carry outs for the low-order 3 bits
assign cout[0] = g_intermediate[0];
assign cout[1] = g_intermediate[1];
assign cout[2] = g_intermediate[2];

endmodule

/** Same as gp4 but for an 8-bit window instead */
module gp8(input wire [7:0] gin, pin,
           input wire cin,
           output wire gout, pout,
           output wire [6:0] cout);

// Compute intermediate generate and propagate signals
wire [7:0] g_intermediate, p_intermediate;
assign g_intermediate[0] = gin[0] | (pin[0] & cin);
assign p_intermediate[0] = pin[0];

generate
    for (i = 1; i < 8; i = i + 1) begin : gp8_logic
        assign g_intermediate[i] = gin[i] | (pin[i] & g_intermediate[i-1]);
        assign p_intermediate[i] = pin[i] & p_intermediate[i-1];
    end
endgenerate

// The carry out for the entire 8-bit block
assign gout = g_intermediate[7];
// Whether an incoming carry would be propagated through the entire 8-bit block
assign pout = p_intermediate[7];

// The carry outs for the internal bits
assign cout = g_intermediate[6:0];



endmodule

module cla
  (input wire [31:0]  a, b,
   input wire         cin,
   output wire [31:0] sum);

  wire [31:0] g, p, c;

// Generate and propagate signals for each bit
genvar i;
generate
    for (i = 0; i < 32; i = i + 1) begin : gen_prop
        gp1 gp_instance(a[i], b[i], g[i], p[i]);
    end
endgenerate

// use gp8 blocks to compute carries for each 8-bit segment
wire [3:0] gout, pout;
wire [28:0] cout; // Internal carries,  the size matches the bits between gp8 blocks

// Instantiate gp8 blocks or logic to compute the carries based on g and p

// Calculate the sum for each bit
assign sum = a ^ b ^ {cout[30:0], cin};


endmodule

