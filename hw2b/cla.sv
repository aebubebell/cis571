`timescale 1ns / 1ps

// Basic Generate and Propagate Unit
module gp1(input wire a, b, output wire g, p);
    assign g = a & b; // Generate
    assign p = a | b; // Propagate
endmodule

// 4-bit Generate and Propagate Logic
module gp4(input wire [3:0] gin, pin, input wire cin,
           output wire gout, pout, output wire [2:0] cout);
    wire [3:1] c; // Intermediate carry signals
    
    // Calculate carries based on generate and propagate signals
    assign c[1] = gin[0] | (pin[0] & cin);
    assign c[2] = gin[1] | (pin[1] & c[1]);
    assign c[3] = gin[2] | (pin[2] & c[2]);

    // Output assignments
    assign cout = c[1:3]; // Carry out signals
    assign gout = gin[3] | (pin[3] & c[3]); // Final generate signal
    assign pout = &pin[3:0]; // Final propagate signal
endmodule

// 8-bit Generate and Propagate Logic
module gp8(input wire [7:0] gin, pin, input wire cin,
           output wire gout, pout, output wire [6:0] cout);
    wire [3:0] c_mid; // Intermediate carry signals for 8-bit operation
    wire g_low, p_low, g_high, p_high;

    // Lower 4 bits
    gp4 lower(.gin(gin[3:0]), .pin(pin[3:0]), .cin(cin),
              .gout(g_low), .pout(p_low), .cout(cout[2:0]));
    
    // Upper 4 bits
    gp4 upper(.gin(gin[7:4]), .pin(pin[7:4]), .cin(cout[2]),
              .gout(g_high), .pout(p_high), .cout(cout[5:3]));
    
    // Combine lower and upper parts
    assign cout[6] = g_high | (p_high & cout[2]);
    assign gout = g_high | (p_high & g_low);
    assign pout = p_low & p_high;
endmodule

// 32-bit Carry Lookahead Adder
module cla(input wire [31:0] a, b, input wire cin, output wire [31:0] sum);
    wire [31:0] g, p; // Generate and propagate signals for each bit
    wire [31:0] c;    // Carry signals, including carry-in and carry-out for each bit

    // Generate and propagate signals computation for each bit
    genvar i;
    for (i = 0; i < 32; i = i + 1) begin
        gp1 gp_unit(.a(a[i]), .b(b[i]), .g(g[i]), .p(p[i]));
    end

    // Compute carries for the entire 32-bit operand
    assign c[0] = cin; // Initial carry-in
    
    wire dummy_gout[0:3]; // If gout is not utilized in your logic
    wire dummy_pout[0:3]; // If pout is not utilized in your logic

    gp8 block0(.gin(g[7:0]), .pin(p[7:0]), .cin(cin), .gout(dummy_gout[0]), .pout(dummy_pout[0]), .cout(c[1:7]));
    gp8 block1(.gin(g[15:8]), .pin(p[15:8]), .cin(c[8]), .gout(dummy_gout[1]), .pout(dummy_pout[1]), .cout(c[9:15]));
    gp8 block2(.gin(g[23:16]), .pin(p[23:16]), .cin(c[16]), .gout(dummy_gout[2]), .pout(dummy_pout[2]), .cout(c[17:23]));
    gp8 block3(.gin(g[31:24]), .pin(p[31:24]), .cin(c[24]), .gout(dummy_gout[3]), .pout(dummy_pout[3]), .cout(c[25:31]));

    // Final sum calculation
    for (i = 0; i < 32; i = i + 1) begin
        assign sum[i] = a[i] ^ b[i] ^ c[i];
    end
endmodule
