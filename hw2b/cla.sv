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
           output wire [6:0] cout);
    wire [7:0] carry_out;
    wire [7:0] gen_modified;
    assign carry_out = {gin[6] | (pin[6] & carry_out[5]),
                        gin[5] | (pin[5] & carry_out[4]),
                        gin[4] | (pin[4] & carry_out[3]),
                        gin[3] | (pin[3] & carry_out[2]),
                        gin[2] | (pin[2] & carry_out[1]),
                        gin[1] | (pin[1] & carry_out[0]),
                        gin[0] | (pin[0] & cin),
                        1'b0};
    //assign cout = carry_out[6:0];
    assign pout = pin[0] && pin[1] && pin[2] && pin[3] && pin[4] && pin[5] && pin[6] && pin[7];
    integer j;
    always @* begin
  for (j = 0; j < 8; j = j + 1) begin
    if (j == 7) begin
      gen_modified[j] = gin[j];
    end
    else begin
        gen_modified[j] = (gin[j] && pin[7:(j-1)]);
    end
  end
end

    assign gout = |gen_modified;
endmodule

module cla
  (input wire [31:0]  a, b,
   input wire         cin,
   output wire [31:0] sum);
   wire [31:0] g, p, c;
   wire [3:0] g_inter, p_inter;
   wire [2:0] c_inter;
   wire gout_inter, pout_inter;
   gp1 gp1s[31:0] (.a(a), .b(b), .g(g), .p(p));
   gp8 m1(.gin(g[7:0]), .pin(p[7:0]), .cin(cin),
    .gout(g_inter[0]), .pout(p_inter[0]), .cout(c[6:0]));
   gp8 m2(.gin(g[15:8]), .pin(p[15:8]), .cin(c_inter[0]),
    .gout(g_inter[1]), .pout(p_inter[1]), .cout(c[14:8]));
   gp8 m3(.gin(g[23:16]), .pin(p[23:16]), .cin(c_inter[1]),
    .gout(g_inter[2]), .pout(p_inter[2]), .cout(c[22:16]));
   gp8 m4(.gin(g[31:24]), .pin(p[31:24]), .cin(c_inter[2]),
    .gout(g_inter[3]), .pout(p_inter[3]), .cout(c[30:24]));
   gp4 m5(.gin(g_inter), .pin(p_inter), .cin(cin),
   .gout(gout_inter), .pout(pout_inter), .cout(c_inter));
   assign c[7] = c_inter[0];
   assign c[15] = c_inter[1];
   assign c[23] = c_inter[2];
   genvar i;
   assign sum[0] = a[0] ^ b[0] ^ cin;
   for (i = 1; i < 32; i++) begin
      assign sum[i] = a[i] ^ b[i] ^ c[i - 1];
   end
endmodule
