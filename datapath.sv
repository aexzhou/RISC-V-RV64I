// 3    - loada
// 4    - loadb
// 5    - loadc
// 6    - asel
// 7    - bsel
// 9    - vsel
// 10   - loads
module datapath(
    //Inputs
    input [15:0] mdata, sximm8, sximm5,
    input [7:0] PC,
    input [2:0] writenum, readnum,
    input [1:0] shift, ALUop, vsel,
    input clk, write, asel, bsel, loada, loadb, loadc, loads,
    output reg [15:0] datapath_out, //this is C
    output reg [2:0] Z_out,
    input shift_ctrl
);
// Internal registers
reg [15:0] data_in, data_out, in, out, sout, Ain, Bin;
reg [15:0] regAout, regBout;
reg [2:0] Z; //vdffed to Z_out

reg [1:0] shift_ctrl_out;



// Module instantiation
regfile REGFILE(data_in, writenum, write, readnum, clk, data_out); //(1)
ALU ALU(Ain, Bin, ALUop, out, Z); //(2)
shifter SHIFTER(regBout, shift_ctrl_out, sout); //(8)

//(3)
reg [15:0] regAtemp;
assign regAtemp = regAout;
always_ff @(posedge clk)begin
    if(loada)
        regAout = data_out;
    else
        regAout = regAtemp;
end
/*
vDFFE #(16) regA(
    .clk(clk),
    .en(loada),
    .in(data_out),
    .out(regAout)
);
*/

//(4)
vDFFE #(16) regB(
    .clk(clk),
    .en(loadb),
    .in(data_out),
    .out(regBout)
);

//(5)
vDFFE #(16) regC(
    .clk(clk),
    .en(loadc),
    .in(out),
    .out(datapath_out)
);

//(6)
MUX16bit selA(
    .in0(regAout),
    .in1(16'd0),
    .sel(asel),
    .out(Ain)
);

//(7)
MUX16bit selB(
    .in0(sout),
    .in1(sximm5),
    .sel(bsel),
    .out(Bin)
);

//(9)
MUX16bit_vsel vSEL(
    .in0(datapath_out),
    .in1({8'b0,PC}),
    .in2(sximm8),
    .in3(mdata),
    .sel(vsel),
    .out(data_in)
);

//(10)
vDFFE #(3) status(
    .clk(clk),
    .en(loads),
    .in(Z),
    .out(Z_out)
);

//SPECIAL shift operation MUX specially designed for STR operations with negative imm5
//when ON, overide shift to 00 (ADD)

always_comb begin
    if(shift_ctrl)
        shift_ctrl_out = 2'b00;
    else 
        shift_ctrl_out = shift;
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
module Control(i);
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

//16-bit mux for vsel
module MUX16bit_vsel(in0,in1,in2,in3,sel,out);

input [15:0] in0;
input [15:0] in1;
input [15:0] in2;
input [15:0] in3;
input [1:0] sel;
output reg [15:0] out;

always @(*) begin 
    case(sel) 
    2'b00: out = in0;
    2'b01: out = in1;
    2'b10: out = in2;
    2'b11: out = in3;
    default: out = 16'bxxxxxxxxxxxxxxxx;
    endcase
end
endmodule