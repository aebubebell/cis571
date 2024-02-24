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


module gp4(input wire [3:0] gin, pin,
           input wire cin,
           output wire gout, pout,
           output wire [2:0] cout);
    // Internal carry signals
    wire [3:1] inter_carry;
    // Compute carries
    assign inter_carry[1] = gin[0] | (pin[0] & cin);
    assign inter_carry[2] = gin[1] | (pin[1] & inter_carry[1]);
    assign inter_carry[3] = gin[2] | (pin[2] & inter_carry[2]);

    // Assign outputs directly
    assign cout = inter_carry[3:1];
    assign gout = gin[3] | (pin[3] & inter_carry[3]);
    assign pout = &pin[3:0];
endmodule


module gp8(input wire [7:0] gin, pin,
           input wire cin,
           output wire gout, pout,
           output wire [6:0] cout);
    // Splitting the process into two 4-bit sections
    wire [3:0] g_mid, p_mid;
    wire [2:0] c_mid;
    wire mid_gout, mid_pout;

    // Process first 4 bits
    gp4 first_half(.gin(gin[3:0]), .pin(pin[3:0]), .cin(cin),
                   .gout(g_mid[0]), .pout(p_mid[0]), .cout(cout[2:0]));
    // Process next 4 bits based on carry out from first half
    gp4 second_half(.gin(gin[7:4]), .pin(pin[7:4]), .cin(cout[2]),
                    .gout(g_mid[1]), .pout(p_mid[1]), .cout(cout[5:3]));
                    
    // Combine middle outputs for final gout and pout
    assign gout = g_mid[1] | (p_mid[1] & g_mid[0]);
    assign pout = p_mid[0] & p_mid[1];
    // Manually handle the final carry out bit
    assign cout[6] = gin[6] | (pin[6] & cout[5]);
endmodule


module cla(input wire [31:0] a, b,
           input wire cin,
           output wire [31:0] sum);
    wire [31:0] g, p;
    wire [32:0] c;

    // Generate and propagate for each bit
    genvar i;
    generate
        for (i = 0; i < 32; i = i + 1) begin : gen_gp
            gp1 bit_gp(.a(a[i]), .b(b[i]), .g(g[i]), .p(p[i]));
        end
    endgenerate

    // Compute carries using gp8 modules, chaining carries
    assign c[0] = cin;
    gp8 first_8(.gin(g[7:0]), .pin(p[7:0]), .cin(c[0]), .gout(), .pout(), .cout(c[1:7]));
    gp8 second_8(.gin(g[15:8]), .pin(p[15:8]), .cin(c[8]), .gout(), .pout(), .cout(c[9:15]));
    gp8 third_8(.gin(g[23:16]), .pin(p[23:16]), .cin(c[16]), .gout(), .pout(), .cout(c[17:23]));
    gp8 fourth_8(.gin(g[31:24]), .pin(p[31:24]), .cin(c[24]), .gout(), .pout(), .cout(c[25:31]));

    // Final sum calculation
    assign sum = a ^ b ^ c[31:0];
endmodule
