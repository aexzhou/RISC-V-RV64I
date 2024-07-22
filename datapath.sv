module datapath(
    input clk, rst
);

// IF|ID Pipeline Registers
reg [63:0] IFID_pc;
reg [32:0] IFID_i;
reg        RegWrite, MemtoReg, Branch, MemRead, MemWrite, ALUsrc; 
reg [1:0]  ALUop;                                            

// ID|EX Pipeline Registers
reg [63:0] IDEX_imm, IDEX_a, IDEX_b;
reg [4:0]  IDEX_Rs1, IDEX_Rs2, IDEX_Rd;
reg        IDEX_RegWrite, IDEX_MemtoReg, IDEX_Branch, IDEX_MemRead, IDEX_MemWrite, IDEX_ALUsrc; 
reg [1:0]  IDEX_ALUop;
reg [3:0]  IDEX_ALUcontrol;

// EX|M Pipeline Registers
reg [63:0] EXM_ALUout, EXM_muxb;
reg [4:0]  EXM_Rd;
reg        EXM_RegWrite, EXM_MemtoReg, EXM_Branch, EXM_MemRead, EXM_MemWrite;
reg        EXM_zflag;

// M|WB Pipeline Registers
reg [63:0] MWB_dout, MWB_aluout;
reg [4:0]  MWB_Rd;
reg        MWB_RegWrite, MWB_MemtoReg;

// General Wires, Regs
reg [63:0] WriteData;
reg [63:0] PC_plus_shimm;
reg        PC_Write, PC_src, IFID_Write, IF_Flush;
reg        zflag;
reg [3:0]  ALUctrl;
reg [63:0] ALUout;
 

/* * * Start of Datapath Logic: * * */

/*
 * (1) IF - Instruction Fetch Stage
 */
reg [63:0] iMem_out; 
reg [63:0] PC_in, PC_out, PC_incremented;
wire [64:0] IFID_pc_next;
wire [32:0] IFID_i_next;

assign PC_in = PC_src? PC_incremented : PC_plus_shimm; // MUX before PC

always_comb PC_incremented = PC_out + 4; // PC incrementer

vDFFE #(64) PC (clk, PC_Write, PC_in, PC_out); // PC Reg that's enabled by PC_Write

iMem IMEM(.address(PC_out), .instruction(iMem_out));

assign IFID_pc_next = IFID_Write? PC_out : IFID_pc; // Disable writing to IFID Registers in the event of a stall
assign IFID_i_next = IFID_Write? iMem_out : IFID_i; 

always @(posedge clk) begin  // IF ---> ID
    IFID_pc <= IFID_pc_next; // These must be non-blocking since they all happen in parallel (concurrently)
    IFID_i <= IFID_i_next;
end


/*
 * (2) ID - Instruction Decode Stage
 */
reg [63:0] imm, sh_imm; // 64 bit immediate, and sh_imm holds imm left shifted by 1
reg hazard_flag; // Controls the MUX after the Control Module
wire [7:0] control_out;

assign sh_imm = {imm[62:0],1'b0}; // Left Shift by 1
assign PC_plus_shimm = sh_imm + IFID_pc; // Adds to PC

ImmGen IMMGEN(IFID_i, imm); // Extracts a 64-bit sign-ext. immediate from the instruction

Control CONTROL(IFID_i, ALUop, Branch, MemRead, MemtoReg, MemWrite, ALUsrc, RegWrite);

regfile REGFILE(.i(IFID_i), .write_data(WriteData), .write(MWB_RegWrite), 
                .clk(clk), .rst(rst), .data_out1(IDEX_a), .data_out2(IDEX_b)); // Rd1 and Rd2 goes directly to IDEX_a and b

HazardDetectionUnit HAZDU(.IFID_i(IFID_i), .IDEX_Rd(IDEX_Rd), .MemRead(IDEX_MemRead),
                          .PC_Write(PC_Write), .IFID_Write(IFID_Write), .hazard_flag(hazard_flag));

assign control_out = {RegWrite, MemtoReg, Branch, MemRead, MemWrite, ALUsrc, ALUop};
assign {IDEX_RegWrite, // MUX to override Control Output with 0s when a Hazard occurs
        IDEX_MemtoReg, 
        IDEX_Branch, 
        IDEX_MemRead, 
        IDEX_MemWrite, IDEX_ALUsrc, IDEX_ALUop} = (hazard_flag)? 8'b0 : control_out; 

always @(posedge clk) begin    // ID ---> EX
    IDEX_Rs1 <= IFID_i[19:15]; // Rs1
    IDEX_Rs2 <= IFID_i[24:20]; // Rs2
    IDEX_Rd  <= IFID_i[11:7];  // Rd
    IDEX_ALUcontrol <= {IFID_i[30], IFID_i[14:12]}; // This is the input for the ALU Control module
    IDEX_imm <= imm;
    {IDEX_RegWrite, IDEX_MemtoReg, IDEX_Branch, IDEX_MemRead, 
    IDEX_MemWrite, IDEX_ALUsrc, IDEX_ALUop} <= {RegWrite, MemtoReg, Branch, MemRead, MemWrite, ALUsrc, ALUop}; // Control Signals
end

/*
 * (3) EX - Execution Stage
 */
reg [1:0] ForwardA, ForwardB;
reg [63:0] ALUa, ALUb, IDEX_muxb;

always_comb begin // MUX for Rd1
    case(ForwardA)
        2'b00: ALUa = IDEX_a;
        2'b01: ALUa = WriteData;
        2'b10: ALUa = EXM_ALUout;
        default: 64'bx;
    endcase
end

always_comb begin // MUX for Rd2
    case(ForwardB)
        2'b00: IDEX_muxb = IDEX_b;
        2'b01: IDEX_muxb = WriteData;
        2'b10: IDEX_muxb = EXM_ALUout;
        default: 64'bx;
    endcase
end

assign ALUb = (IDEX_ALUsrc)? IDEX_imm : IDEX_muxb; // MUX right before ALUb

ALUcontrol ALUCONTROL(IDEX_ALUcontrol, IDEX_ALUop, ALUctrl);

ALU ALU(ALUa, ALUb, ALUctrl, ALUout, zflag);

ForwardingUnit FWDU(IDEX_Rs1, IDEX_Rs2, EXM_Rd, EXM_RegWrite, MWB_Rd, MWB_RegWrite, ForwardA, ForwardB);

always @(posedge clk) begin    // EX ---> M
    EXM_zflag <= zflag;
    EXM_ALUout <= ALUout;
    EXM_muxb <= IDEX_muxb;
    EXM_Rd <= IDEX_Rd;
    {EXM_RegWrite, EXM_MemtoReg, EXM_Branch, 
    EXM_MemRead, EXM_MemWrite} <= {IDEX_RegWrite, IDEX_MemtoReg, IDEX_Branch, IDEX_MemRead, IDEX_MemWrite};
end

/*
 * (4) M - Memory Access Stage
 */
assign PC_src = EXM_Branch & EXM_zflag;

dataMem dMEM( .clk(clk), .rst(rst), .address(EXM_ALUout), .write_data(EXM_muxb),    
              .mem_read(EXM_MemRead), .mem_write(EXM_MemWrite), .read_data(MWB_dout)); // MWB_dout <= ...

always @(posedge clk) begin    // M ---> WB
    MWB_aluout <= EXM_aluout;
    MWB_Rd <= EXM_Rd;
    MWB_RegWrite <= EXM_RegWrite;
    MWB_MemtoReg <= EXM_MemtoReg;
end

/*
 * (5) WB - Write Back Stage
 */
assign WriteData = (MWB_MemtoReg)? MWB_aluout : MWB_dout;

endmodule



/* * * Start of Module Definitions: * * */



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

// Hazard Detection Unit to check Pipeline Hazards
module HazardDetectionUnit(
    input [63:0] IFID_i,
    input [4:0] IDEX_Rd,
    input MemRead,
    output PC_Write,
    output IFID_Write,
    output hazard_flag
);
    // HAZARD CODE TBCOMPLETED

endmodule

module ForwardingUnit(IDEX_Rs1, IDEX_Rs2, EXM_Rd, EXM_RegWrite, MWB_Rd, MWB_RegWrite, ForwardA, ForwardB);
    input [4:0] IDEX_Rs1, IDEX_Rs2, EXM_Rd, MWB_Rd;
    input EXM_RegWrite, MWB_RegWrite;
    output ForwardA, ForwardB;
    
    // Forwardingunit TBCompleted

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
