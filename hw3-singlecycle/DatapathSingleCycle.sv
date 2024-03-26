`timescale 1ns / 1ns

// registers are 32 bits in RV32
`define REG_SIZE 31:0

// RV opcodes are 7 bits
`define OPCODE_SIZE 6:0

`include "../hw2a/divider_unsigned.sv"
`include "../hw2b/cla.sv"

module RegFile (
    input logic [4:0] rd,
    input logic [`REG_SIZE] rd_data,
    input logic [4:0] rs1,
    output logic [`REG_SIZE] rs1_data,
    input logic [4:0] rs2,
    output logic [`REG_SIZE] rs2_data,

    input logic clk,
    input logic we,
    input logic rst
);
  localparam int NumRegs = 32;
  logic [`REG_SIZE] regs[NumRegs];

  // Initialize register 0 to always be 0
  // assign rs1_data = rs1 == 0 ? 0 : regs[rs1];
  // assign rs2_data = rs2 == 0 ? 0 : regs[rs2];
genvar i;
for (i = 1; i < NumRegs; i = i + 1) begin
  always_ff @(posedge clk) begin
    if (rst) begin
      // Reset all registers except x0

        regs[i] <= 32'd0;
      
    end else if (we && rd == i) begin
      // Write to register (rd) if we is asserted and rd is not x0
      regs[rd] <= rd_data;
    end
  end
end

  assign regs[0] = 32'd0;
  assign rs1_data = regs[rs1];
  assign rs2_data = regs[rs2];
    
endmodule



module DatapathSingleCycle (
    input wire clk,
    input wire rst,
    output logic halt,
    output logic [`REG_SIZE] pc_to_imem,
    input wire [`REG_SIZE] insn_from_imem,
    // addr_to_dmem is a read-write port
    output wire [`REG_SIZE] addr_to_dmem,
    input logic [`REG_SIZE] load_data_from_dmem,
    output wire [`REG_SIZE] store_data_to_dmem,
    output wire [3:0] store_we_to_dmem
);

  // components of the instruction
  wire [6:0] insn_funct7;
  wire [4:0] insn_rs2;
  wire [4:0] insn_rs1;
  wire [2:0] insn_funct3;
  wire [4:0] insn_rd;
  wire [`OPCODE_SIZE] insn_opcode;

  logic [31:0] temp; // temporary adder 
  logic rf_we; // Register file write enable
  logic [31:0] rf_wdata; // Data to write to register file
  logic branch_taken; // Branch decision
  logic pc_update_request; // Program counter update request
  logic [31:0] pc_update_value; // Value to update the program counter with
  logic system_call_request; // System call request signal
  logic [31:0] rs1_data, rs2_data; // Data from source registers
  logic [31:0] current_pc; // Current program counter value

  // split R-type instruction - see section 2.2 of RiscV spec
  assign {insn_funct7, insn_rs2, insn_rs1, insn_funct3, insn_rd, insn_opcode} = insn_from_imem;

  // setup for I, S, B & J type instructions
  // I - short immediates and loads
  wire [11:0] imm_i;
  assign imm_i = insn_from_imem[31:20];
  wire [ 4:0] imm_shamt = insn_from_imem[24:20];

  // S - stores
  wire [11:0] imm_s;
  assign imm_s[11:5] = insn_funct7, imm_s[4:0] = insn_rd;

  // B - conditionals
  wire [12:0] imm_b;
  assign {imm_b[12], imm_b[10:5]} = insn_funct7, {imm_b[4:1], imm_b[11]} = insn_rd, imm_b[0] = 1'b0;

  // J - unconditional jumps
  wire [20:0] imm_j;
  assign {imm_j[20], imm_j[10:1], imm_j[11], imm_j[19:12], imm_j[0]} = {insn_from_imem[31:12], 1'b0};

  // U - 20-bit immediate
  wire [19:0] imm_u;
  assign imm_u = insn_from_imem[31:12];

  wire [`REG_SIZE] imm_i_sext = {{20{imm_i[11]}}, imm_i[11:0]};
  wire [`REG_SIZE] imm_s_sext = {{20{imm_s[11]}}, imm_s[11:0]};
  wire [`REG_SIZE] imm_b_sext = {{19{imm_b[12]}}, imm_b[12:0]};
  wire [`REG_SIZE] imm_j_sext = {{11{imm_j[20]}}, imm_j[20:0]};
  wire [`REG_SIZE] imm_u_sext = {{12{imm_u[19]}}, imm_u[19:0]};

  // opcodes - see section 19 of RiscV spec
  localparam bit [`OPCODE_SIZE] OpLoad = 7'b00_000_11;
  localparam bit [`OPCODE_SIZE] OpStore = 7'b01_000_11;
  localparam bit [`OPCODE_SIZE] OpBranch = 7'b11_000_11;
  localparam bit [`OPCODE_SIZE] OpJalr = 7'b11_001_11;
  localparam bit [`OPCODE_SIZE] OpMiscMem = 7'b00_011_11;
  localparam bit [`OPCODE_SIZE] OpJal = 7'b11_011_11;

  localparam bit [`OPCODE_SIZE] OpRegImm = 7'b00_100_11;
  localparam bit [`OPCODE_SIZE] OpRegReg = 7'b01_100_11;
  localparam bit [`OPCODE_SIZE] OpEnviron = 7'b11_100_11;

  localparam bit [`OPCODE_SIZE] OpAuipc = 7'b00_101_11;
  localparam bit [`OPCODE_SIZE] OpLui = 7'b01_101_11;

  wire insn_lui = insn_opcode == OpLui;
  wire insn_auipc = insn_opcode == OpAuipc;
  wire insn_jal = insn_opcode == OpJal;
  wire insn_jalr = insn_opcode == OpJalr;

  wire insn_beq = insn_opcode == OpBranch && insn_from_imem[14:12] == 3'b000;
  wire insn_bne = insn_opcode == OpBranch && insn_from_imem[14:12] == 3'b001;
  wire insn_blt = insn_opcode == OpBranch && insn_from_imem[14:12] == 3'b100;
  wire insn_bge = insn_opcode == OpBranch && insn_from_imem[14:12] == 3'b101;
  wire insn_bltu = insn_opcode == OpBranch && insn_from_imem[14:12] == 3'b110;
  wire insn_bgeu = insn_opcode == OpBranch && insn_from_imem[14:12] == 3'b111;

  wire insn_lb = insn_opcode == OpLoad && insn_from_imem[14:12] == 3'b000;
  wire insn_lh = insn_opcode == OpLoad && insn_from_imem[14:12] == 3'b001;
  wire insn_lw = insn_opcode == OpLoad && insn_from_imem[14:12] == 3'b010;
  wire insn_lbu = insn_opcode == OpLoad && insn_from_imem[14:12] == 3'b100;
  wire insn_lhu = insn_opcode == OpLoad && insn_from_imem[14:12] == 3'b101;

  wire insn_sb = insn_opcode == OpStore && insn_from_imem[14:12] == 3'b000;
  wire insn_sh = insn_opcode == OpStore && insn_from_imem[14:12] == 3'b001;
  wire insn_sw = insn_opcode == OpStore && insn_from_imem[14:12] == 3'b010;

  wire insn_addi = insn_opcode == OpRegImm && insn_from_imem[14:12] == 3'b000;
  wire insn_slti = insn_opcode == OpRegImm && insn_from_imem[14:12] == 3'b010;
  wire insn_sltiu = insn_opcode == OpRegImm && insn_from_imem[14:12] == 3'b011;
  wire insn_xori = insn_opcode == OpRegImm && insn_from_imem[14:12] == 3'b100;
  wire insn_ori = insn_opcode == OpRegImm && insn_from_imem[14:12] == 3'b110;
  wire insn_andi = insn_opcode == OpRegImm && insn_from_imem[14:12] == 3'b111;

  wire insn_slli = insn_opcode == OpRegImm && insn_from_imem[14:12] == 3'b001 && insn_from_imem[31:25] == 7'd0;
  wire insn_srli = insn_opcode == OpRegImm && insn_from_imem[14:12] == 3'b101 && insn_from_imem[31:25] == 7'd0;
  wire insn_srai = insn_opcode == OpRegImm && insn_from_imem[14:12] == 3'b101 && insn_from_imem[31:25] == 7'b0100000;

  wire insn_add = insn_opcode == OpRegReg && insn_from_imem[14:12] == 3'b000 && insn_from_imem[31:25] == 7'd0;
  wire insn_sub  = insn_opcode == OpRegReg && insn_from_imem[14:12] == 3'b000 && insn_from_imem[31:25] == 7'b0100000;
  wire insn_sll = insn_opcode == OpRegReg && insn_from_imem[14:12] == 3'b001 && insn_from_imem[31:25] == 7'd0;
  wire insn_slt = insn_opcode == OpRegReg && insn_from_imem[14:12] == 3'b010 && insn_from_imem[31:25] == 7'd0;
  wire insn_sltu = insn_opcode == OpRegReg && insn_from_imem[14:12] == 3'b011 && insn_from_imem[31:25] == 7'd0;
  wire insn_xor = insn_opcode == OpRegReg && insn_from_imem[14:12] == 3'b100 && insn_from_imem[31:25] == 7'd0;
  wire insn_srl = insn_opcode == OpRegReg && insn_from_imem[14:12] == 3'b101 && insn_from_imem[31:25] == 7'd0;
  wire insn_sra  = insn_opcode == OpRegReg && insn_from_imem[14:12] == 3'b101 && insn_from_imem[31:25] == 7'b0100000;
  wire insn_or = insn_opcode == OpRegReg && insn_from_imem[14:12] == 3'b110 && insn_from_imem[31:25] == 7'd0;
  wire insn_and = insn_opcode == OpRegReg && insn_from_imem[14:12] == 3'b111 && insn_from_imem[31:25] == 7'd0;

  wire insn_mul    = insn_opcode == OpRegReg && insn_from_imem[31:25] == 7'd1 && insn_from_imem[14:12] == 3'b000;
  wire insn_mulh   = insn_opcode == OpRegReg && insn_from_imem[31:25] == 7'd1 && insn_from_imem[14:12] == 3'b001;
  wire insn_mulhsu = insn_opcode == OpRegReg && insn_from_imem[31:25] == 7'd1 && insn_from_imem[14:12] == 3'b010;
  wire insn_mulhu  = insn_opcode == OpRegReg && insn_from_imem[31:25] == 7'd1 && insn_from_imem[14:12] == 3'b011;
  wire insn_div    = insn_opcode == OpRegReg && insn_from_imem[31:25] == 7'd1 && insn_from_imem[14:12] == 3'b100;
  wire insn_divu   = insn_opcode == OpRegReg && insn_from_imem[31:25] == 7'd1 && insn_from_imem[14:12] == 3'b101;
  wire insn_rem    = insn_opcode == OpRegReg && insn_from_imem[31:25] == 7'd1 && insn_from_imem[14:12] == 3'b110;
  wire insn_remu   = insn_opcode == OpRegReg && insn_from_imem[31:25] == 7'd1 && insn_from_imem[14:12] == 3'b111;

  wire insn_ecall = insn_opcode == OpEnviron && insn_from_imem[31:7] == 25'd0;
  wire insn_fence = insn_opcode == OpMiscMem;

  wire [31:0] lui_imm =  {insn_from_imem[31:12], 12'b0};

  wire [31:0] add_result, sub_result, addi_result, sll_result, slt_result, sltu_result, srl_result, sra_result, xor_result, or_result, and_result;

  // synthesis translate_off
  // this code is only for simulation, not synthesis
  `include "RvDisassembler.sv"
  string disasm_string;
  always_comb begin
    disasm_string = rv_disasm(insn_from_imem);
  end
  // HACK: get disasm_string to appear in GtkWave, which can apparently show only wire/logic...
  wire [(8*32)-1:0] disasm_wire;
  genvar i;
  for (i = 0; i < 32; i = i + 1) begin : gen_disasm
    assign disasm_wire[(((i+1))*8)-1:((i)*8)] = disasm_string[31-i];
  end
  // synthesis translate_on

  // program counter
  logic [`REG_SIZE] pcNext, pcCurrent;
  always @(posedge clk) begin
    if (rst) begin
      pcCurrent <= 32'd0;
    end else begin
      pcCurrent <= pcNext;
    end
  end
  assign pc_to_imem = pcCurrent;

  // cycle/insn_from_imem counters
  logic [`REG_SIZE] cycles_current, num_insns_current;
  always @(posedge clk) begin
    if (rst) begin
      cycles_current <= 0;
      num_insns_current <= 0;
    end else begin
      cycles_current <= cycles_current + 1;
      if (!rst) begin
        num_insns_current <= num_insns_current + 1;
      end
    end
  end

  logic illegal_insn;

  // Instantiate the RegFile module
  RegFile rf (
    .rd(insn_rd),                  // assuming insn_rd is the destination register ID
    .rd_data(rf_wdata),            // data to be written into the register file
    .rs1(insn_rs1),                // source register 1 ID
    .rs1_data(rs1_data),           // data read from source register 1
    .rs2(insn_rs2),                // source register 2 ID
    .rs2_data(rs2_data),           // data read from source register 2
    .clk(clk),                     // clock signal
    .we(rf_we),                    // write enable signal
    .rst(rst)                      // reset signal
  );

  cla cla_add(.a(rs1_data), .b(rs2_data), .cin(1'b0), .sum(add_result));
  cla cla_sub(.a(rs1_data), .b(~rs2_data), .cin(1'b1), .sum(sub_result));
  cla cla_addi(.a(rs1_data), .b(imm_i_sext), .cin(1'b0), .sum(addi_result));
  
  cla cla_slt(.a(rs1_data), .b(rs2_data), .cin(0), .sum(slt_result));
  cla cla_sltu(.a(rs1_data), .b(rs2_data), .cin(0), .sum(sltu_result));
  cla cla_srl(.a(rs1_data), .b({rs2_data[4:0], 27'b0}), .cin(0), .sum(srl_result));
  cla cla_sra(.a(rs1_data), .b({rs2_data[4:0], 27'b0}), .cin(0), .sum(sra_result));
  cla cla_xor(.a(rs1_data), .b(rs2_data), .cin(0), .sum(xor_result));
  cla cla_or(.a(rs1_data), .b(rs2_data), .cin(0), .sum(or_result));
  cla cla_and(.a(rs1_data), .b(rs2_data), .cin(0), .sum(and_result));

  always_comb begin
    illegal_insn = 1'b0;
    rf_we = 1'b0; 
    rf_wdata = 32'b0;
    branch_taken = 1'b0; // Signal to indicate if branch is taken
    pc_update_request = 1'b0; // Request to update the program counter
    pc_update_value = 32'b0; // New value for the program counter if branch is taken
    system_call_request = 1'b0; // Signal to indicate a system call request
    // branch_offset = 32'sd0;
    halt = 1'b0;
    pcNext = pcCurrent + 4;
    temp = 'd0;

    case (insn_opcode)
      OpLui: begin
        // TODO: start here by implementing lui
        rf_we = 1'b1;
        rf_wdata = lui_imm;
      end
      OpAuipc: begin // AUIPC
        rf_we = 1'b1;
        rf_wdata = pcCurrent + {{imm_u[19:0]}, 12'b0};
      end
      OpRegImm: begin
        case (insn_funct3)
          3'b000: begin // ADDI
            rf_we = 1'b1;
            rf_wdata = addi_result;
            // pcNext = pcCurrent + 4;
          end
          3'b010: begin // SLTI
            rf_we = 1'b1;
            rf_wdata = $signed(rs1_data) < $signed(imm_i_sext) ? 32'b1 : 32'b0;
            // pcNext = pcCurrent + 4;
          end
          3'b011: begin // SLTIU
            rf_we = 1'b1;
            rf_wdata = rs1_data < imm_i_sext ? 32'b1 : 32'b0;
            // pcNext = pcCurrent + 4;
          end
          3'b100: begin // XORI
            rf_we = 1'b1;
            rf_wdata = rs1_data ^ imm_i_sext;
            // pcNext = pcCurrent + 4;
          end
          3'b110: begin // ORI
            rf_we = 1'b1;
            rf_wdata = rs1_data | imm_i_sext;
            // pcNext = pcCurrent + 4;
          end
          3'b111: begin // ANDI
            rf_we = 1'b1;
            rf_wdata = rs1_data & imm_i_sext;
            // pcNext = pcCurrent + 4;
          end
          3'b001: begin // SLLI
            rf_we = 1'b1;
            rf_wdata = rs1_data << imm_i_sext[4:0]; 
            // pcNext = pcCurrent + 4;
          end
          3'b101: begin
            if (imm_i_sext[10] == 1'b0) begin // SRLI
              rf_we = 1'b1;
              rf_wdata = rs1_data >> imm_i_sext[4:0]; 
              // pcNext = pcCurrent + 4;
            end else begin // SRAI
              rf_we = 1'b1;
                rf_wdata = $signed(rs1_data) >>> imm_i_sext[4:0];
              // pcNext = pcCurrent + 4;
            end
          end
          default: 
            illegal_insn = 1'b1;
        endcase
      end
      OpRegReg: begin
        case (insn_funct3)
          3'b000: begin 
              if (insn_sub) begin // SUB
              rf_we = 1'b1;
              rf_wdata = sub_result;
            end else begin // ADD
              rf_we = 1'b1;
              rf_wdata = add_result;
            end
            // pcNext = pcCurrent + 4;
          end
          3'b001: begin // SLL
            rf_we = 1'b1;
              rf_wdata = rs1_data << (rs2_data[4:0]);
            // pcNext = pcCurrent + 4;
          end
          3'b010: begin // SLT
            rf_we = 1'b1;
              rf_wdata = $signed(rs1_data) < $signed(rs2_data) ? 1 : 0;
            // pcNext = pcCurrent + 4;
          end
          3'b011: begin // SLTU
            rf_we = 1'b1;
            rf_wdata = rs1_data < rs2_data ? 1 : 0;
            // pcNext = pcCurrent + 4;
          end
          3'b100: begin // XOR
            rf_we = 1'b1;
            rf_wdata = rs1_data ^ rs2_data;
            // pcNext = pcCurrent + 4;
          end
          3'b101: begin 
            if (insn_from_imem[30] == 0) begin // SRL
              rf_we = 1'b1;
                rf_wdata = rs1_data >> (rs2_data[4:0]);
              // pcNext = pcCurrent + 4;
            end else begin // SRA
              rf_we = 1'b1;
                rf_wdata = $signed(rs1_data) >>> $signed(rs2_data[4:0]);
              // pcNext = pcCurrent + 4;
            end
          end
          3'b110: begin // OR
            rf_we = 1'b1;
            rf_wdata = rs1_data | rs2_data;
            // pcNext = pcCurrent + 4;
          end
          3'b111: begin // AND
            rf_we = 1'b1;
            rf_wdata = rs1_data & rs2_data;
            // pcNext = pcCurrent + 4;
          end
        endcase
      end
      OpBranch: begin
        // Branch instructions
        case (insn_funct3)
          3'b000: begin // BEQ
            branch_taken = (rs1_data == rs2_data);
          end
          3'b001: begin // BNE
            branch_taken = (rs1_data != rs2_data);
          end
          3'b100: begin // BLT
            branch_taken = $signed(rs1_data) < $signed(rs2_data);
          end
          3'b101: begin // BGE
            branch_taken = $signed(rs1_data) >= $signed(rs2_data);
          end
          3'b110: begin // BLTU
            branch_taken = (rs1_data < rs2_data);
          end
          3'b111: begin // BGEU
            branch_taken = (rs1_data >= rs2_data);
          end
          default: 
            illegal_insn = 1'b1;
        endcase
        if (branch_taken) begin
          pc_update_request = 1'b1;
            pcNext = pcCurrent + imm_b_sext;
        end else begin
          pc_update_request = 1'b0; 
          pcNext = pcCurrent + 4;
        end
      end
      OpEnviron: begin
        halt = 1;
        pcNext = pcCurrent + 4;
      end
      default: begin
        illegal_insn = 1'b1;
        // pcNext = pcCurrent + 4;
      end
    OpJal: begin
      // JAL instruction
      rf_we = 1'b1; // Enable writing to the register file
      rf_wdata = pcCurrent + 4; // Return address is current PC + 4
      pcNext = pcCurrent + imm_j_sext; // Jump to target address
    end
    OpJalr: begin
      // JALR instruction
      rf_we = 1'b1; // Enable writing to the register file
      rf_wdata = pcCurrent + 4; // Return address is current PC + 4
      pcNext = (rs1_data + imm_i_sext) & ~32'b1; // Clear least significant bit to ensure alignment
    end
    OpMiscMem: begin
            // Check for insn_fence -> as nop in this context
            if (insn_fence) begin
                // No operation performed, just advance to the next instruction
                rf_we = 1'b0;
                branch_taken = 1'b0;
                pc_update_request = 1'b0;
                pcNext = pcCurrent + 4;
                
            end 
        end
    

    OpLoad: begin
      // Load instructions
      case (insn_funct3)
        3'b000: begin // LB
          rf_we = 1'b1;
          temp = rs1_data + imm_i_sext;
          case (temp[1:0])
            2'b00: rf_wdata = {{24{load_data_from_dmem[7]}}, load_data_from_dmem[7:0]};
            2'b01: rf_wdata = {{24{load_data_from_dmem[15]}}, load_data_from_dmem[15:8]};
            2'b10: rf_wdata = {{24{load_data_from_dmem[23]}}, load_data_from_dmem[23:16]};
            2'b11: rf_wdata = {{24{load_data_from_dmem[31]}}, load_data_from_dmem[31:24]};
            default: begin
                illegal_insn = 1'b1;
                rf_we = 1'b0;
              end
          endcase
            addr_to_dmem = {temp[31:2], 2'b00};
        end
        3'b001: begin // LH
          rf_we = 1'b1;
          temp = rs1_data + imm_i_sext;
          case (temp[1])
            1'b0: rf_wdata = {{16{load_data_from_dmem[15]}}, load_data_from_dmem[15:0]};
            1'b1: rf_wdata = {{16{load_data_from_dmem[31]}}, load_data_from_dmem[31:16]};
            default: begin
              illegal_insn = 1'b1;
              rf_we = 1'b0; // Do not write to register file on misaligned access
            end
          endcase
            addr_to_dmem = {temp[31:2], 2'b00};
        end
        3'b010: begin // LW
          rf_we = 1'b1;
          temp = rs1_data + imm_i_sext;
          //rf_wdata = load_data_from_dmem;
          //LW requires the address to be word-aligned
          if (temp[1:0] == 2'b00) begin
            rf_wdata = load_data_from_dmem;
          end else begin
            illegal_insn = 1'b1;
            rf_we = 1'b0; // Do not write to register file on misaligned access
          end
            addr_to_dmem = {temp[31:2], 2'b00};
        end
        3'b100: begin // LBU
          rf_we = 1'b1;
          temp = rs1_data + imm_i_sext;
          case (temp[1:0])
            2'b00: rf_wdata = {24'b0, load_data_from_dmem[7:0]};
            2'b01: rf_wdata = {24'b0, load_data_from_dmem[15:8]};
            2'b10: rf_wdata = {24'b0, load_data_from_dmem[23:16]};
            2'b11: rf_wdata = {24'b0, load_data_from_dmem[31:24]};
          endcase
          addr_to_dmem = {temp[31:2], 2'b00};
        end
        3'b101: begin // LHU
          rf_we = 1'b1;
          temp = rs1_data + imm_i_sext;
          case (temp[1:0])
            2'b00: rf_wdata = {16'b0, load_data_from_dmem[15:0]};
            2'b10: rf_wdata = {16'b0, load_data_from_dmem[31:16]};
            default: begin
              illegal_insn = 1'b1;
              rf_we = 1'b0; // Do not write to register file on misaligned access
            end 
          endcase
            addr_to_dmem = {temp[31:2], 2'b00};
        end
        default: illegal_insn = 1'b1;
      endcase
    end



  OpStore: begin
    // Store instructions
    case (insn_funct3)
      3'b000: begin // SB
        temp = rs1_data + imm_s_sext;
        store_data_to_dmem = {4{rs2_data[7:0]}};
        store_we_to_dmem = 4'b0001; // Enable writing the LSB
      end
      //addr_to_dmem = {temp[31:2], 2'b00};
      3'b001: begin // SH
        temp = rs1_data + imm_s_sext;
        store_data_to_dmem = {2{rs2_data[15:0]}};
        store_we_to_dmem = 4'b0011; // Enable writing the two LSBs
      end
        //addr_to_dmem = {temp[31:2], 2'b00};
      3'b010: begin // SW
        temp = rs1_data + imm_s_sext;
        store_data_to_dmem = rs2_data;
        store_we_to_dmem = 4'b1111; // Enable writing all bytes
        
      end
      default: illegal_insn = 1'b1;
      
    endcase
    addr_to_dmem = {temp[31:2], 2'b00};
  end
      
    endcase

  end

endmodule

/* A memory module that supports 1-cycle reads and writes, with one read-only port
 * and one read+write port.
 */
module MemorySingleCycle #(
    parameter int NUM_WORDS = 512
) (
    // rst for both imem and dmem
    input wire rst,

    // clock for both imem and dmem. See RiscvProcessor for clock details.
    input wire clock_mem,

    // must always be aligned to a 4B boundary
    input wire [`REG_SIZE] pc_to_imem,

    // the value at memory location pc_to_imem
    output logic [`REG_SIZE] insn_from_imem,

    // must always be aligned to a 4B boundary
    input wire [`REG_SIZE] addr_to_dmem,

    // the value at memory location addr_to_dmem
    output logic [`REG_SIZE] load_data_from_dmem,

    // the value to be written to addr_to_dmem, controlled by store_we_to_dmem
    input wire [`REG_SIZE] store_data_to_dmem,

    // Each bit determines whether to write the corresponding byte of store_data_to_dmem to memory location addr_to_dmem.
    // E.g., 4'b1111 will write 4 bytes. 4'b0001 will write only the least-significant byte.
    input wire [3:0] store_we_to_dmem
);

  // memory is arranged as an array of 4B words
  logic [`REG_SIZE] mem[NUM_WORDS];

  initial begin
    $readmemh("mem_initial_contents.hex", mem, 0);
  end

  always_comb begin
    // memory addresses should always be 4B-aligned
    assert (pc_to_imem[1:0] == 2'b00);
    assert (addr_to_dmem[1:0] == 2'b00);
  end

  localparam int AddrMsb = $clog2(NUM_WORDS) + 1;
  localparam int AddrLsb = 2;

  always @(posedge clock_mem) begin
    if (rst) begin
    end else begin
      insn_from_imem <= mem[{pc_to_imem[AddrMsb:AddrLsb]}];
    end
  end

  always @(negedge clock_mem) begin
    if (rst) begin
    end else begin
      if (store_we_to_dmem[0]) begin
        mem[addr_to_dmem[AddrMsb:AddrLsb]][7:0] <= store_data_to_dmem[7:0];
      end
      if (store_we_to_dmem[1]) begin
        mem[addr_to_dmem[AddrMsb:AddrLsb]][15:8] <= store_data_to_dmem[15:8];
      end
      if (store_we_to_dmem[2]) begin
        mem[addr_to_dmem[AddrMsb:AddrLsb]][23:16] <= store_data_to_dmem[23:16];
      end
      if (store_we_to_dmem[3]) begin
        mem[addr_to_dmem[AddrMsb:AddrLsb]][31:24] <= store_data_to_dmem[31:24];
      end
      // dmem is "read-first": read returns value before the write
      load_data_from_dmem <= mem[{addr_to_dmem[AddrMsb:AddrLsb]}];
    end
  end
endmodule

/*
This shows the relationship between clock_proc and clock_mem. The clock_mem is
phase-shifted 90° from clock_proc. You could think of one proc cycle being
broken down into 3 parts. During part 1 (which starts @posedge clock_proc)
the current PC is sent to the imem. In part 2 (starting @posedge clock_mem) we
read from imem. In part 3 (starting @negedge clock_mem) we read/write memory and
prepare register/PC updates, which occur at @posedge clock_proc.

        ____
 proc: |    |______
           ____
 mem:  ___|    |___
*/
module RiscvProcessor (
    input  wire  clock_proc,
    input  wire  clock_mem,
    input  wire  rst,
    output logic halt
);

  wire [`REG_SIZE] pc_to_imem, insn_from_imem, mem_data_addr, mem_data_loaded_value, mem_data_to_write;
  wire [3:0] mem_data_we;

  MemorySingleCycle #(
      .NUM_WORDS(8192)
  ) mem (
      .rst      (rst),
      .clock_mem (clock_mem),
      // imem is read-only
      .pc_to_imem(pc_to_imem),
      .insn_from_imem(insn_from_imem),
      // dmem is read-write
      .addr_to_dmem(mem_data_addr),
      .load_data_from_dmem(mem_data_loaded_value),
      .store_data_to_dmem (mem_data_to_write),
      .store_we_to_dmem  (mem_data_we)
  );

  DatapathSingleCycle datapath (
      .clk(clock_proc),
      .rst(rst),
      .pc_to_imem(pc_to_imem),
      .insn_from_imem(insn_from_imem),
      .addr_to_dmem(mem_data_addr),
      .store_data_to_dmem(mem_data_to_write),
      .store_we_to_dmem(mem_data_we),
      .load_data_from_dmem(mem_data_loaded_value),
      .halt(halt)
  );

endmodule
