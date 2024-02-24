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
module cla(input wire [31:0] a, b, input wire cin, output wire [31:0] sum);
    wire [31:0] g, p;
    wire [3:0] g_inter, p_inter; 
    wire [2:0] c_inter; 
    wire gout_inter, pout_inter; 
    wire [31:0] c; 
    // Instantiate gp1 modules for generate and propagate signals
    genvar i;
    generate
        for (i = 0; i < 32; i = i + 1) begin
            gp1 gp1_inst(.a(a[i]), .b(b[i]), .g(g[i]), .p(p[i]));
        end
    endgenerate

    // Instantiate gp8 modules for 8-bit segments of the operand
    gp8 m1(.gin(g[7:0]), .pin(p[7:0]), .cin(cin), .gout(g_inter[0]), .pout(p_inter[0]), .cout(c[6:0]));
    gp8 m2(.gin(g[15:8]), .pin(p[15:8]), .cin(c[7]), .gout(g_inter[1]), .pout(p_inter[1]), .cout(c[14:8]));
    gp8 m3(.gin(g[23:16]), .pin(p[23:16]), .cin(c[15]), .gout(g_inter[2]), .pout(p_inter[2]), .cout(c[22:16]));
    gp8 m4(.gin(g[31:24]), .pin(p[31:24]), .cin(c[23]), .gout(g_inter[3]), .pout(p_inter[3]), .cout(c[30:24]));

    // Corrected: Assign carries between 8-bit segments directly
    assign c[7] = c[7]; // Redundant but illustrates the carry flow
    assign c[15] = c[14];
    assign c[23] = c[22];

    // Calculate sum bits
    assign sum[0] = a[0] ^ b[0] ^ cin; // Direct calculation for the first bit
    generate
        for (i = 1; i < 32; i = i + 1) begin
            assign sum[i] = a[i] ^ b[i] ^ c[i-1]; // Use previous carry for each bit
        end
    endgenerate
endmodule
