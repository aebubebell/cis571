`timescale 1ns / 1ps

module gp1(input wire a, b, output wire g, p);
    assign g = a & b;
    assign p = a | b;
endmodule


module gp4(input wire [3:0] gin, pin, input wire cin,
           output wire gout, pout, output wire [2:0] cout);
   
    wire g_internal, p_internal;
    wire [3:0] c_internal; // Include an extra bit for internal consistency

    
    assign c_internal[0] = cin;
    assign c_internal[1] = gin[0] | (pin[0] & cin);
    assign c_internal[2] = gin[1] | (pin[1] & c_internal[1]);
    assign c_internal[3] = gin[2] | (pin[2] & c_internal[2]);

    
    assign cout = c_internal[1:3]; 
    assign gout = gin[3] | (pin[3] & c_internal[3]);
    assign pout = &pin[3:0]; 
endmodule


module gp8(input wire [7:0] gin, pin, input wire cin,
           output wire gout, pout, output wire [6:0] cout);
    wire [7:0] c_internal;

    assign c_internal[0] = cin;
  
    assign c_internal[1] = gin[0] | (pin[0] & cin);
    assign c_internal[2] = gin[1] | (pin[1] & c_internal[1]);
   
    assign c_internal[7] = gin[6] | (pin[6] & c_internal[6]); // Last carry calculation

    // Directly map the new internal carries to the outputs
    assign cout = c_internal[1:7]; // Adjusted indices for proper mapping
    assign gout = |(gin & (pin << 1)) | gin[7]; // Alternative expression for gout
    assign pout = &pin; // Simplified expression for pout
endmodule

module cla(input wire [31:0] a, b, input wire cin,
           output wire [31:0] sum);
    wire [31:0] g, p;
    wire [32:0] c; // Expanded to include cin directly

    // Instantiate gp1 for each pair of inputs
    genvar idx;
    generate
        for (idx = 0; idx < 32; idx = idx + 1) begin
            gp1 single_bit(.a(a[idx]), .b(b[idx]), .g(g[idx]), .p(p[idx]));
        end
    endgenerate

    // Sequentially apply gp8 modules for carry computation
    assign c[0] = cin;
    gp8 quarter1(.gin(g[7:0]), .pin(p[7:0]), .cin(c[0]), .gout(), .pout(), .cout(c[1:7]));

    gp8 quarter2(.gin(g[15:8]), .pin(p[15:8]), .cin(c[8]), .gout(), .pout(), .cout(c[9:15]));
    gp8 quarter3(.gin(g[23:16]), .pin(p[23:16]), .cin(c[16]), .gout(), .pout(), .cout(c[17:23]));
    gp8 quarter4(.gin(g[31:24]), .pin(p[31:24]), .cin(c[24]), .gout(), .pout(), .cout(c[25:31]));

    assign sum = a ^ b ^ c[31:0];
endmodule
