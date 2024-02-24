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

module gp8(input wire [7:0] gin, pin,
           input wire cin,
           output wire gout, pout,
           output wire [6:0] cout);  // Assuming cout is 7 bits for 8-bit gp calculation

    wire [7:0] carry_out;
    wire [7:0] gen_modified;


assign carry_out = {gin[6] | (pin[6] & carry_out[5]),
                    gin[5] | (pin[5] & carry_out[4]),
                    gin[4] | (pin[4] & carry_out[3]),
                    gin[3] | (pin[3] & carry_out[2]),
                    gin[2] | (pin[2] & carry_out[1]),
                    gin[1] | (pin[1] & carry_out[0]),
                    gin[0] | (pin[0] & cin),
                    1'b0}; // The last bit is dummy for alignment


assign cout = carry_out[6:0];


assign pout = pin[0] && pin[1] && pin[2] && pin[3] && pin[4] && pin[5] && pin[6] && pin[7];


integer j;
always @(*) begin
    gen_modified[7] = gin[7]; // Direct assignment for the highest bit
    for (j = 6; j >= 0; j = j - 1) begin
        gen_modified[j] = gin[j] && (&pin[j + 1:7]); 
    end
end


assign gout = |gen_modified;
endmodule


// 
module cla(input wire [31:0] a, b, input wire cin, output wire [31:0] sum);
    wire [31:0] gen_signals, prop_signals, carry_signals;
wire [3:0] gen_partial, prop_partial;
wire [2:0] carry_partial;
wire carry_out, prop_out;
genvar idx;

// Instantiate gp1 for each bit
generate
    for (idx = 0; idx < 32; idx++) begin : gp1_block
        gp1 unit(.a(a[idx]), .b(b[idx]), .g(gen_signals[idx]), .p(prop_signals[idx]));
    end
endgenerate

// Processing blocks using gp8
gp8 block1(
    .gin(gen_signals[7:0]), .pin(prop_signals[7:0]), .cin(cin),
    .gout(gen_partial[0]), .pout(prop_partial[0]), .cout(carry_signals[6:0])
);

gp8 block2(
    .gin(gen_signals[15:8]), .pin(prop_signals[15:8]), .cin(carry_partial[0]),
    .gout(gen_partial[1]), .pout(prop_partial[1]), .cout(carry_signals[14:8])
);

gp8 block3(
    .gin(gen_signals[23:16]), .pin(prop_signals[23:16]), .cin(carry_partial[1]),
    .gout(gen_partial[2]), .pout(prop_partial[2]), .cout(carry_signals[22:16])
);

gp8 block4(
    .gin(gen_signals[31:24]), .pin(prop_signals[31:24]), .cin(carry_partial[2]),
    .gout(gen_partial[3]), .pout(prop_partial[3]), .cout(carry_signals[30:24])
);

// Intermediate carry handling
gp4 inter_handler(
    .gin(gen_partial), .pin(prop_partial), .cin(cin),
    .gout(carry_out), .pout(prop_out), .cout(carry_partial)
);

// Direct assignments for certain carries
assign carry_signals[7] = carry_partial[0];
assign carry_signals[15] = carry_partial[1];
assign carry_signals[23] = carry_partial[2];

// Sum calculation loop
assign sum[0] = a[0] ^ b[0] ^ cin;
generate
    for (idx = 1; idx < 32; idx++) begin : sum_calculation
        assign sum[idx] = a[idx] ^ b[idx] ^ carry_signals[idx - 1];
    end
endgenerate

endmodule
