`timescale 1ns / 1ps

module gp1(input wire a, b, output wire g, p);
    assign g = a & b; 
    assign p = a | b; 
endmodule

module gp4(input wire [3:0] gin, pin,
           input wire cin,
           output wire gout, pout,
           output wire [2:0] cout);
    wire temp_c1, temp_c2, temp_c3;
wire propagate_out, generate_out;


assign temp_c1 = gin[0] | (cin & pin[0]);
assign temp_c2 = gin[1] | (temp_c1 & pin[1]);
assign temp_c3 = gin[2] | (temp_c2 & pin[2]);


assign cout[0] = temp_c1;
assign cout[1] = temp_c2;
assign cout[2] = temp_c3;


assign propagate_out = pin[0] & pin[1] & pin[2] & pin[3];
assign generate_out = gin[3] |
                      (pin[3] & gin[2]) |
                      (pin[3] & pin[2] & gin[1]) |
                      (pin[3] & pin[2] & pin[1] & gin[0]);

assign pout = propagate_out;
assign gout = generate_out;

endmodule

module gp8(input wire [7:0] gin, pin,
           input wire cin,
           output wire gout, pout,
           output wire [6:0] cout);
wire carry1, carry2, carry3, carry4, carry5, carry6, carry7;


assign carry1 = gin[0] | (cin & pin[0]);
assign carry2 = gin[1] | (carry1 & pin[1]);
assign carry3 = gin[2] | (carry2 & pin[2]);
assign carry4 = gin[3] | (carry3 & pin[3]);
assign carry5 = gin[4] | (carry4 & pin[4]);
assign carry6 = gin[5] | (carry5 & pin[5]);
assign carry7 = gin[6] | (carry6 & pin[6]);

assign cout[0] = carry1;
assign cout[1] = carry2;
assign cout[2] = carry3;
assign cout[3] = carry4;
assign cout[4] = carry5;
assign cout[5] = carry6;
assign cout[6] = carry7;


assign pout = &pin;


wire [7:0] new_g;
assign new_g[7] = gin[7];

genvar j;
for (j = 6; j >= 0; j = j - 1) begin
    
    assign new_g[j] = gin[j] & &pin[7:j+1];
end

// Assign gout using the newly defined new_g
assign gout = |new_g;

endmodule

module cla
  (input wire [31:0]  a, b,
   input wire         cin,
   output wire [31:0] sum);

wire [31:0] gen_signal, propagate, carry;
wire [3:0] gen_mid, prop_mid;
wire [2:0] carry_mid;
wire gen_out_mid, prop_out_mid;


generate
    genvar i;
    for (i = 0; i < 32; i = i + 1) begin : gp1_instances
        gp1 gp1_block(
            .a(a[i]),       
            .b(b[i]),       
            .g(gen_signal[i]), 
            .p(propagate[i]) 
        );
    end
endgenerate


gp8 module1(.gin(gen_signal[7:0]), .pin(propagate[7:0]), .cin(cin),
    .gout(gen_mid[0]), .pout(prop_mid[0]), .cout(carry[6:0]));

gp8 module2(.gin(gen_signal[15:8]), .pin(propagate[15:8]), .cin(carry_mid[0]),
    .gout(gen_mid[1]), .pout(prop_mid[1]), .cout(carry[14:8]));

gp8 module3(.gin(gen_signal[23:16]), .pin(propagate[23:16]), .cin(carry_mid[1]),
    .gout(gen_mid[2]), .pout(prop_mid[2]), .cout(carry[22:16]));

gp8 module4(.gin(gen_signal[31:24]), .pin(propagate[31:24]), .cin(carry_mid[2]),
    .gout(gen_mid[3]), .pout(prop_mid[3]), .cout(carry[30:24]));


gp4 module5(.gin(gen_mid), .pin(prop_mid), .cin(cin),
    .gout(gen_out_mid), .pout(prop_out_mid), .cout(carry_mid));

assign carry[7] = carry_mid[0];
assign carry[15] = carry_mid[1];
assign carry[23] = carry_mid[2];

genvar j;
assign sum[0] = a[0] ^ b[0] ^ cin;
for (j = 1; j < 32; j++) begin
    assign sum[j] = a[j] ^ b[j] ^ carry[j - 1];
end
endmodule
