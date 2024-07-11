module datapath(
    input clk
);

// Module instantiation/connections

//Program Counter Register (holds the address of the next instruction to be executed)
wire pc_enable;
wire [63:0] pc_in;
wire [63:0] pc_out;
assign pc_enable = 1'b1;
vDFFE #(64) PC(
    .clk(clk),
    .en(pc_enable),
    .in(pc_in),
    .out(pc_out)
);

wire [31:0] IM_out;
InstructionMemory IM(
    .address(pc_out),
    .instruction(IM_out)
);

wire read_reg1 = IM_out[19:15];
wire read_reg2 = IM_out[24:20];
wire write_reg = IM_out[11:7];
wire opcode = IM_out[6:0];
wire [63:0] read_data1;
wire [63:0] read_data2;
regfile REGFILE(write_data,write_reg,RegWrite,read_reg1,read_reg2,clk,data_out1,data_out2);

wire [1:0] ALUop;
wire Branch, MemRead, MemtoReg, MemWrite, ALUsrc, RegWrite;
Control CONTROL(opcode, ALUop, Branch, MemRead, MemtoReg, MemWrite, ALUsrc, RegWrite);

wire [63:0] imm_out;
ImmGen IMMGEN(IM_out, imm_out);

wire [63:0] ALU_in2;
MUX64 MUX64_REG_ALU(
    .in0(read_data2),
    .in1(imm_out),
    .sel(ALUsrc),
    .out(ALU_in2)
);

wire [63:0] ALU_out;
wire Z;                            //WARNING: Z IS 2 BIT IN ALU, BUT TEXTBOOK USES 1 BIT FOR ZERO FLAG, UPDATE REQUIRED
ALU ALU(read_data1, ALU_in2, ALU_control_out, ALU_out, Z);

wire [3:0] ALU_control_out;
ALUcontrol ALUCONTROL(IM_out, ALUop, ALU_control_out);

wire [63:0] DM_out;
DataMemory DM(
    .clk(clk),
    .address(ALU_out),
    .write_data(read_data2),
    .mem_read(MemRead),
    .mem_write(MemWrite),
    .read_data(DM_out)
);

wire [63:0] write_data;
MUX64 MUX64_DM(
    .in0(ALU_out),
    .in1(DM_out),
    .sel(MemtoReg),
    .out(ALU_in2)
);

wire [63:0] SLA_out;
ShiftLeftandAdd SLA(pc_out, imm_out, SLA_out);

wire [63:0] PC_plus_4;
assign PC_plus_4 = pc_out + 4;

wire MUX64_PC_sel;
assign MUX64_PC_sel = Branch & Z[0];

MUX64 MUX64_PC(
    .in0(PC_plus_4),
    .in1(SLA_out),
    .sel(MUX64_PC_SEL),
    .out(pc_in)
);





// Instruction memory module will read a 64bit address (PC) and output the corresponding instruction (RAM)
module InstructionMemory (
    input wire [63:0] address,     // Address input
    output reg [31:0] instruction  // Instruction output
);

    // Instruction memory array (stores 1024 32-bit instructions)
    reg [31:0] memory [0:1023];

    /*
    // Initialize the instruction memory with some values (optional)
    initial begin
        $readmemh("instructions.mem", memory);  // Load instructions from a file
    end
    */

    // Output the instruction at the given address
    always @(address) begin
        instruction = memory[address[63:2]];    // Address is byte-aligned, so divide by 4
    end
endmodule

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
module ImmGen(in, imm_out);
    input   [31:0] in;    // 32-bit Instruction input
    output  [63:0] imm_out;     // Sign extended 64-bit immediate 

    case(in[6:0])
        7'b0100000: imm_out = { {52{in[31]}}, in[31:20] }; // LW
        7'b1000000: imm_out = { {52{in[31]}}, in[31:25], in[11:7] }; // SW
        7'b1100000: imm_out = { {52{in[31]}}, in[31:25], in[7], in[11:8] }; // BEQ
        default: imm_out = 32'b0;
    endcase
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
 */
module ALUcontrol(i, ALUop, opout);
    input [31:0] i; // 32-bit instruction input 
    input [1:0] ALUop; // 2-bit ALUop from the Control Module
    output [3:0] opout; // 4-bit output towards the ALU
    // opout: AND = 0000, OR  = 0001, add = 0010, sub = 0110
    wire [6:0] funct7;
    wire [2:0] funct3;
    assign funct7 = i[31:25]; // R-Type Instruction format
    assign funct3 = i[14:12];

    case({ALUop, funct7, funct3})
        12'b00xxxxxxxxxx: opout = 4'b0010;
        12'bx1xxxxxxxxxx: opout = 4'b0110;
        12'b1x0000000000: opout = 4'b0010;
        12'b1x0100000000: opout = 4'b0110;
        12'b1x0000000111: opout = 4'b0000;
        12'b1x0000000110: opout = 4'b0001;
        default: opout = 4'bxxxx;
    endcase
endmodule

// Control unit for the datapath (OPCODE --> Control Signals)
module Control(i, ALUop, Branch, MemRead, MemtoReg, MemWrite, ALUsrc, RegWrite);
    input [31:0] i;
    output [1:0] ALUop;
    output Branch;
    output MemRead;
    output MemtoReg;
    output MemWrite;
    output ALUsrc;
    output RegWrite;

    wire [7:0] output = {ALUsrc, MemtoReg, RegWrite, MemRead, MemWrite, Branch, ALUop};

    case(i[6:0])
        7'b0110011: output = 8'b00100010; // R-format
        7'b0000011: output = 8'b11110000; // ld
        7'b0100011: output = 8'b1x001000; // sd
        7'b1100011: output = 8'b0x000101; // beq
        default: output = {8{1'bx}};
    endcase
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

endmodule