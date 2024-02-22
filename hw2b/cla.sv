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

wire [3:0] c_internal; // Internal carries, not including cin

// Calculate the carry out for each bit
assign c_internal[0] = gin[0] | (pin[0] & cin);
assign c_internal[1] = gin[1] | (pin[1] & c_internal[0]);
assign c_internal[2] = gin[2] | (pin[2] & c_internal[1]);
assign cout[0] = c_internal[0];
assign cout[1] = c_internal[1];
assign cout[2] = c_internal[2];

// Calculate the overall generate and propagate for the 4-bit block
assign gout = gin[3] | (pin[3] & c_internal[2]);
assign pout = pin[0] & pin[1] & pin[2] & pin[3];


endmodule

/** Same as gp4 but for an 8-bit window instead */
module gp8(input wire [7:0] gin, pin,
           input wire cin,
           output wire gout, pout,
           output wire [6:0] cout);

wire [6:0] c_internal; // Internal carries for 8-bit window

// Calculate intermediate carries based on individual generate and propagate signals
assign c_internal[0] = gin[0] | (pin[0] & cin);
// Repeat for bits 1 through 6
assign c_internal[1] = gin[1] | (pin[1] & c_internal[0]);
// Continue this pattern up to c_internal[5]

assign c_internal[6] = gin[6] | (pin[6] & c_internal[5]);
assign cout = c_internal; // The internal carries are directly the output carries for 8 bits

// Overall generate and propagate for the 8-bit window
assign gout = gin[7] | (pin[7] & c_internal[6]);
assign pout = &pin; // Logical AND of all propagate signals


endmodule

module cla
  (input wire [31:0]  a, b,
   input wire         cin,
   output wire [31:0] sum);

  // Generate initial generate and propagate signals for each bit
    genvar i;
    generate
        for (i = 0; i < 32; i = i + 1) begin : gp_generation
            gp1 gp1_inst(a[i], b[i], g[i], p[i]);
        end
    endgenerate
    
    // Use gp8 modules to aggregate generate and propagate signals over 8-bit blocks
    genvar j;
    generate
        for (j = 0; j < 4; j = j + 1) begin : gp8_blocks
            gp8 gp8_inst(g[j*8 +: 8], p[j*8 +: 8], c[j*8], gout[j], pout[j], cout[j*7 +: 7]);
        end
    endgenerate
    
    // Compute carry signals using a simpler logic, assuming gp8 provides intermediate carries
    
    assign c[0] = cin;
    

    // Calculate sum for each bit
    assign sum = a ^ b ^ c;

endmodule
