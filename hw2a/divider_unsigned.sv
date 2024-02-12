/* Anthony Bell
    76981628 */

`timescale 1ns / 1ns

// quotient = dividend / divisor

module divider_unsigned (
    input  wire [31:0] i_dividend,
    input  wire [31:0] i_divisor,
    output wire [31:0] o_remainder,
    output wire [31:0] o_quotient
);

    wire [31:0] dividend[0:32];
    wire [31:0] remainder[0:32];
    wire [31:0] quotient[0:32];

    // Initialize the first stage inputs
    assign dividend[0] = i_dividend;
    assign remainder[0] = 0;
    assign quotient[0] = 0;

    // Generate 32 instances of divu_1iter
    genvar i;
    generate
        for (i = 0; i < 32; i = i + 1) begin : div_loop
            divu_1iter div_step(
                .i_dividend(dividend[i]),
                .i_divisor(i_divisor),
                .i_remainder(remainder[i]),
                .i_quotient(quotient[i]),
                .o_dividend(dividend[i+1]),
                .o_remainder(remainder[i+1]),
                .o_quotient(quotient[i+1])
            );
        end
    endgenerate

    // Connect the final stage outputs
    assign o_remainder = remainder[32];
    assign o_quotient = quotient[32];

endmodule




module divu_1iter (
    input  wire [31:0] i_dividend,
    input  wire [31:0] i_divisor,
    input  wire [31:0] i_remainder,
    input  wire [31:0] i_quotient,
    output wire [31:0] o_dividend,
    output wire [31:0] o_remainder,
    output wire [31:0] o_quotient
);

    // Temporary variables for internal calculations
    wire [31:0] next_remainder;
    wire [31:0] next_quotient;

    // Shift the remainder left by 1 bit and add the MSB of the dividend
    assign next_remainder = (i_remainder << 1) | (i_dividend >> 31);

    // If the updated remainder is greater than or equal to the divisor,
    // subtract the divisor from the remainder and set the LSB of the quotient to 1.
    // Otherwise, the remainder and quotient remain unchanged.
   assign next_quotient = (next_remainder >= i_divisor) ? (i_quotient << 1) | 32'h1 : (i_quotient << 1);


    // Update the remainder only if it's greater than or equal to the divisor
    assign o_remainder = (next_remainder >= i_divisor) ? next_remainder - i_divisor : next_remainder;

    // Shift the dividend left by 1 (discard the MSB, as it's already added to the remainder)
    assign o_dividend = i_dividend << 1;
    assign o_quotient = next_quotient;

endmodule
