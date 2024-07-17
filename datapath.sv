module datapath(
    input clk, rst
    input [31:0] instruction
);

// Module instantiation/connections

//Program Counter Register (holds the address of the next instruction to be executed)
wire pc_enable;
assign pc_enable = 1'b1; // PC always enabled
wire [63:0] pc_in;
wire [63:0] pc_out;

vDFFE #(64) PC(
    .clk(clk),
    .en(pc_enable),
    .in(pc_in),
    .out(pc_out)
);

// module PC(clk, en, in, out);
//   parameter n = 1;  // width
//   input clk, en ;
//   input [n-1:0] in ;
//   output reg [n-1:0] out ;
//   wire [n-1:0] next_out ;

//   assign next_out = en ? in : out;

//   always @(posedge clk)
//     out = next_out;  
// endmodule

// wire [31:0] instruction;
// InstructionMemory IM(
//     .address(pc_out),
//     .instruction(instruction)
// );

// reg [4:0] read_reg1 = instruction[19:15];
// reg [4:0] read_reg2 = instruction[24:20];
// reg [4:0] write_reg = instruction[11:7];
reg [63:0] read_data1, read_data2;
reg [63:0] imm_out;
reg [63:0] ALU_in2;
reg Branch, MemRead, MemtoReg, MemWrite, ALUsrc, RegWrite;
reg [1:0] ALUop;
reg [63:0] ALU_out;
reg Z;                            //WARNING: Z IS 2 BIT IN ALU, BUT TEXTBOOK USES 1 BIT FOR ZERO FLAG, UPDATE REQUIRED
reg [3:0] ALU_control_out;
reg [63:0] DM_out;
reg [63:0] write_data;
reg [63:0] SLA_out;
reg [63:0] PC_plus_4;
reg MUX64_PC_sel;

regfile REGFILE(
    .i(instruction),
    .write_data(write_data),
    .write(RegWrite),
    .clk(clk),
    .data_out1(read_data1),
    .data_out2(read_data2)
);

Control CONTROL(instruction, ALUop, Branch, MemRead, MemtoReg, MemWrite, ALUsrc, RegWrite);

ImmGen IMMGEN(instruction, imm_out);

MUX64 MUX64_REG_ALU(
    .in0(read_data2),
    .in1(imm_out),
    .sel(ALUsrc),
    .out(ALU_in2)
);

ALU ALU(read_data1, ALU_in2, ALU_control_out, ALU_out, Z);

ALUcontrol ALUCONTROL(instruction, ALUop, ALU_control_out);

DataMemory DM(
    .clk(clk),
    .address(ALU_out),
    .write_data(read_data2),
    .mem_read(MemRead),
    .mem_write(MemWrite),
    .read_data(DM_out)
);

MUX64 MUX64_DM(
    .in0(ALU_out),
    .in1(DM_out),
    .sel(MemtoReg),
    .out(write_data) 
);

MUX64 MUX64_PC(
    .in0(PC_plus_4),
    .in1(SLA_out),
    .sel(MUX64_PC_SEL),
    .out(pc_in)
);

ShiftLeftandAdd SLA(pc_out, imm_out, SLA_out);

assign PC_plus_4 = pc_out + 4;

assign MUX64_PC_sel = Branch & Z;

endmodule

// // Instruction memory module will read a 64bit address (PC) and output the corresponding instruction (RAM)
// module InstructionMemory (
//     input wire [63:0] address,     // Address input
//     output reg [31:0] instruction  // Instruction output
// );

//     // Instruction memory array (stores 1024 32-bit instructions)
//     reg [31:0] memory [0:1023];

    /*
    // Initialize the instruction memory with some values
    initial begin
        $readmemh("instructions.mem", memory);  // Load instructions from a file
    end
    */

//     // Output the instruction at the given address
//     always @(address) begin
//         instruction = memory[address[63:2]];    // Address is byte-aligned, so divide by 4
//     end
// endmodule

// The data memory module supports both read and write operations
// It has separate input ports for address, data to be written, and control signals for reading and writing
module DataMemory (
    input wire clk,                  // Clock signal
    input wire [63:0] address,       // Address input
    input wire [63:0] write_data,    // Data to write
    input wire mem_read,             // Memory read enable
    input wire mem_write,            // Memory write enable
    output reg [63:0] read_data      // Data read output
);

    // Data memory array (stores 1024 64-bit data)
    reg [63:0] memory [0:1023];

    // Read data
    always @(posedge clk) begin
        if (mem_read) begin
            read_data <= memory[address[63:3]];  // Address is byte-aligned, so divide by 8
        end
    end

    // Write data
    always @(posedge clk) begin
        if (mem_write) begin
            memory[address[63:3]] <= write_data;  // Address is byte-aligned, so divide by 8
        end
    end
endmodule

/* Immediate Generation Unit Module

 * Selects a 12-bit field for LOAD, STORE, and BRANCH IF EQUAL that is
 * sign-extended into a 64-bit result as output.

 * LW  = {offset[11:0],               src1[4:0], 3'b000, dest[4:0],      7'b0100000}
 * SW  = {offset[11:5],    src2[4:0], src1[4:0], 3'b000, dest[4:0],      7'b1000000}
 * BEQ = {offset[12,10:5], src2[4:0], src1[4:0], 3'b000, offset[4:1,11], 7'b1100000}
 */

/* Instruction Formats for RISC-V Architecture
 *
 * R-Type Instructions                                      I-Type Instructions
 * |31      25|24  20|19  15|14  12|11   7|6      0|        |31          20|19  15|14  12|11   7|6      0|
 * |  funct7  |  rs2 |  rs1 |func3 |  rd  | opcode |        |    imm[11:0] |  rs1 |func3 |  rd  | opcode |
 * Uses: R-R ops (add, sub, sll ...)                        Uses: Imm ops (addi, lw, srai ...)
 *
 * S-Type Instructions                                      B-Type Instructions
 * |31      25|24  20|19  15|14  12|11   7|6      0|        |31    25|24  20|19  15|14  12|11      7|6     0|
 * | imm[11:5]|  rs2 |  rs1 |func3 |imm[4:0]|opcode|        |imm[12| |  rs2 |  rs1 |func3 |imm[4:1| | opcode|
 * Uses: Store ops (sw, sh ...)                             |  10:5] |      |      |      |     11] |
 *                                                          Uses: Branch ops (beq, bne ...)
 *      
 * U-Type Instructions                                      J-Type Instructions
 * |31              12|11   7|6      0|                     |31             12|11   7|6      0|
 * |      imm[31:12]  |  rd  | opcode |                     |     imm[20|10:1]|  rd  | opcode |
 * Uses: Long jumps and large constants (lui, auipc ...)    |       |11|19:12]|      |        |
 *                                                          Uses: Jump operations (jal ...)
 */
module ImmGen(in, imm_out);
    input       [31:0] in;    // 32-bit Instruction input
    output reg  [63:0] imm_out;     // Sign extended 64-bit immediate 

    always_comb begin
        case(in[6:0])
            7'b1101111: imm_out = { {51{in[31]}}, in[31], in[19:12], in[20], in[30:21], 1'b0}; // J-Type
            7'b1100111: imm_out = { {52{in[31]}}, in[31:20] }; // I-Type JALR
            7'b0010011: imm_out = { {52{in[31]}}, in[31:20] }; // I-Type imm
            7'b0000011: imm_out = { {52{in[31]}}, in[31:20] }; // I-Type load instructions
            7'b0100011: imm_out = { {52{in[31]}}, in[31:25], in[11:7] }; // S-type
            7'b1100011: imm_out = { {51{in[31]}}, in[31], in[7], in[30:25], in[11:7], 1'b0 }; // B-Type
            7'b0110111: imm_out = { in[31], in[30:20], in[19:12], 12'b0}; // U-Type
            default: imm_out = 64'b0;
        endcase
    end
endmodule

/* Takes output of ImmGen, shifts by 1, and add to program counter */
module ShiftLeftandAdd(pc64, imm64, out64);
    input [63:0] pc64;
    input [63:0] imm64; // Gets shifted left by 1 and 0 fills in lsb.
    output [63:0] out64;
    wire [64:0] shift_output;
    assign shift_output = {imm64, 1'b0}; // Shift imm64 by 1 and fill lsb with 0.
    assign out64 = pc64 + shift_output[63:0]; // Adds shifted value to pc64.
endmodule

/* ALU Control Module
 *
 * Provides ALU with the right operation by judging 
 * the inputs from the Control Unit and Instructions. 
 * This is useful since it lessens the load on the Control Module
 * and reducing its latency thus decreasing clk cycle time. 
 */
module ALUcontrol(i, ALUop, opout);
/* Inputs towards the ALU:
 * 4'b0000: AND
 * 4'b0001: OR
 * 4'b0010: ADD
 * 4'b0110: SUB
 * 4'b0111: SLT (Set on Less Than): will output 1 if A < B
 * 4'b1100: NOR
 */
    input [31:0] i; // 32-bit instruction input 
    input [1:0] ALUop; // 2-bit ALUop from the Control Module
    output reg [3:0] opout; // 4-bit output towards the ALU
    wire [6:0] funct7;
    wire [2:0] funct3;
    assign funct7 = i[31:25]; // R-Type Instruction format
    assign funct3 = i[14:12];

    always_comb begin
        casex({ALUop, funct7, funct3})
            12'b00xxxxxxx000: opout = 4'b0010; // addi
            12'b00xxxxxxx111: opout = 4'b0000; // andi
            12'b00xxxxxxx110: opout = 4'b0001; // ori
            12'b00xxxxxxxxxx: opout = 4'b0010; // add
            12'bx1xxxxxxxxxx: opout = 4'b0110; // sub
            12'b1x0000000000: opout = 4'b0010; // add(r)
            12'b1x0100000000: opout = 4'b0110; // sub
            12'b1x0000000111: opout = 4'b0000; // and
            12'b1x0000000110: opout = 4'b0001; // or
            default: opout = 4'bxxxx;
        endcase
    end
endmodule


// Control unit for the datapath (OPCODE --> Control Signals)
module Control(i, ALUop, Branch, MemRead, MemtoReg, MemWrite, ALUsrc, RegWrite);
    input [31:0] i;
    output reg [1:0] ALUop;
    output reg Branch;
    output reg MemRead;
    output reg MemtoReg;
    output reg MemWrite;
    output reg ALUsrc;
    output reg RegWrite;
    
    always_comb begin
        case(i[6:0])
            7'b0010011: {ALUsrc, MemtoReg, RegWrite, MemRead, MemWrite, Branch, ALUop} = 8'b10100000; // I-type
            7'b0110011: {ALUsrc, MemtoReg, RegWrite, MemRead, MemWrite, Branch, ALUop} = 8'b00100010; // R-type
            7'b0000011: {ALUsrc, MemtoReg, RegWrite, MemRead, MemWrite, Branch, ALUop} = 8'b11110000; // ld (I-type LOAD)
            7'b0100011: {ALUsrc, MemtoReg, RegWrite, MemRead, MemWrite, Branch, ALUop} = 8'b1x001000; // sd (S-type)
            7'b1100011: {ALUsrc, MemtoReg, RegWrite, MemRead, MemWrite, Branch, ALUop} = 8'b0x000101; // beq (B-type)
            default: {ALUsrc, MemtoReg, RegWrite, MemRead, MemWrite, Branch, ALUop} = {8{1'bx}};
        endcase
    end
endmodule   

//register with load enable
module vDFFE(clk, en, in, out);
  parameter n = 1;  // width
  input clk, en ;
  input [n-1:0] in ;
  output reg [n-1:0] out ;
  wire [n-1:0] next_out ;

  assign next_out = en ? in : out;

  always @(posedge clk)
    out = next_out;  
endmodule

/* 64 bit mux */
module MUX64(in0, in1, sel, out);
    input [63:0] in0;
    input [63:0] in1;
    input sel;
    output [63:0] out;

    assign out = sel? in1 : in0;
endmodule