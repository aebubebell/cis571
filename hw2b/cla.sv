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
    wire [3:0] cout8;

    // Instantiate gp1 modules for each bit
    genvar bit;
    generate
        for (bit = 0; bit < 32; bit = bit + 1) begin : gp1_instances
            gp1 gp1_inst(.a(a[bit]), .b(b[bit]), .g(g[bit]), .p(p[bit]));
        end
    endgenerate

    // Instantiate gp8 modules for each 8-bit segment
    gp8 gp8_inst0(.gin(g[7:0]), .pin(p[7:0]), .cin(cin), .gout(g8[0]), .pout(p8[0]), .cout(cout8[0]));
    gp8 gp8_inst1(.gin(g[15:8]), .pin(p[15:8]), .cin(cout8[0]), .gout(g8[1]), .pout(p8[1]), .cout(cout8[1]));
    gp8 gp8_inst2(.gin(g[23:16]), .pin(p[23:16]), .cin(cout8[1]), .gout(g8[2]), .pout(p8[2]), .cout(cout8[2]));
    gp8 gp8_inst3(.gin(g[31:24]), .pin(p[31:24]), .cin(cout8[2]), .gout(g8[3]), .pout(p8[3]), .cout(cout8[3]));

    // Correct
    generate
        for (bit = 1; bit < 32; bit = bit + 1) begin : carry_calculation
            if (bit % 8 == 0) begin : block_carry_boundary
                assign c[bit] = cout8[bit/8-1];
            end else begin : block_carry_internal
                assign c[bit] = g[bit-1] | (p[bit-1] & c[bit-1]);
            end
        end
    endgenerate

    assign c[0] = cin; // Set the initial carry

    // Calculate sum
    generate
        for (bit = 0; bit < 32; bit = bit + 1) begin : sum_calc
            assign sum[bit] = a[bit] ^ b[bit] ^ c[bit];
        end
    endgenerate

endmodule
