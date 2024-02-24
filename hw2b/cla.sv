`timescale 1ns / 1ps

// Basic Generate and Propagate Unit
module gp1(input wire a, b, output wire g, p);
    assign g = a & b; // Generate
    assign p = a | b; // Propagate
endmodule

module gp4(input wire [3:0] gin, pin,
           input wire cin,
           output wire gout, pout,
           output wire [2:0] cout);

    // Compute aggregate generate and propagate signals
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
           output wire [6:0] cout);

    wire [7:0] c_internal;
    assign c_internal[0] = cin;
    assign c_internal[1] = gin[0] | (pin[0] & c_internal[0]);
    // Repeat for bits 2 through 6
    genvar i;
    generate
        for (i = 1; i < 7; i = i + 1) begin : loop
            assign c_internal[i + 1] = gin[i] | (pin[i] & c_internal[i]);
        end
    endgenerate

    assign cout = c_internal[1:7]; // Assign the internal carries to cout

    assign gout = gin[7] | (pin[7] & c_internal[7]);
    assign pout = &pin; // AND all propagate signals together
endmodule


// 32-bit Carry Lookahead Adder
`timescale 1ns / 1ps

module cla(input wire [31:0] a, b,
           input wire cin,
           output wire [31:0] sum);

    wire [31:0] g, p; // Generate and propagate for each bit
    wire [31:0] c; // Carry for each bit position, c[0] is cin
    wire [7:0] g8, p8; // Generate and propagate for 8-bit segments
    wire [3:0] cout8; // Carry out between 8-bit segments
    wire gout, pout; // Overall generate and propagate

    // Instantiate gp1 modules for each bit
    genvar i;
    generate
        for (i = 0; i < 32; i = i + 1) begin : gp1_gen
            gp1 gp1_inst(.a(a[i]), .b(b[i]), .g(g[i]), .p(p[i]));
        end
    endgenerate

    // Instantiate gp8 modules for each 8-bit segment
    gp8 gp8_inst0(.gin(g[7:0]), .pin(p[7:0]), .cin(cin), .gout(g8[0]), .pout(p8[0]), .cout(cout8[0]));
    gp8 gp8_inst1(.gin(g[15:8]), .pin(p[15:8]), .cin(cout8[0]), .gout(g8[1]), .pout(p8[1]), .cout(cout8[1]));
    gp8 gp8_inst2(.gin(g[23:16]), .pin(p[23:16]), .cin(cout8[1]), .gout(g8[2]), .pout(p8[2]), .cout(cout8[2]));
    gp8 gp8_inst3(.gin(g[31:24]), .pin(p[31:24]), .cin(cout8[2]), .gout(g8[3]), .pout(p8[3]), .cout(cout8[3]));

    // Calculate carry for the entire 32-bit operation
    assign c[0] = cin;
    assign c[8] = cout8[0];
    assign c[16] = cout8[1];
    assign c[24] = cout8[2];
    // No need for c[32] as we don't have carry-out

    // Intermediate carries within 8-bit segments
    assign c[7:1] = {g[6:0] | (p[6:0] & c[0]), g[5:0] | (p[5:0] & c[0]), g[4:0] | (p[4:0] & c[0]), g[3:0] | (p[3:0] & c[0]), g[2:0] | (p[2:0] & c[0]), g[1:0] | (p[1:0] & c[0]), g[0] | (p[0] & c[0])};
    assign c[15:9] = {g[14:8] | (p[14:8] & c[8]), g[13:8] | (p[13:8] & c[8]), g[12:8] | (p[12:8] & c[8]), g[11:8] | (p[11:8] & c[8]), g[10:8] | (p[10:8] & c[8]), g[9:8] | (p[9:8] & c[8]), g[8] | (p[8] & c[8])};
    assign c[23:17] = {g[22:16] | (p[22:16] & c[16]), g[21:16] | (p[21:16] & c[16]), g[20:16] | (p[20:16] & c[16]), g[19:16] | (p[19:16] & c[16]), g[18:16] | (p[18:16] & c[16]), g[17:16] | (p[17:16] & c[16]), g[16] | (p[16] & c[16])};
    assign c[31:25] = {g[30:24] | (p[30:24] & c[24]), g[29:24] | (p[29:24] & c[24]), g[28:24] | (p[28:24] & c[24]), g[27:24] | (p[27:24] & c[24]), g[26:24] | (p[26:24] & c[24]), g[25:24] | (p[25:24] & c[24]), g[24] | (p[24] & c[24])};

    // Calculate sum
    generate
        for (i = 0; i < 32; i = i + 1) begin : sum_gen
            assign sum[i] = a[i] ^ b[i] ^ c[i];
        end
    endgenerate

endmodule

