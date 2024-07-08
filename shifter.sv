/*
Shifter Module

Accomodates the following operations:

name | imm[11:5] | imm[4:0] | rs1[4:0] | funct3[2:0] | rd[4:0] | opcode[5:0]
-----|-----------|----------|----------|-------------|---------|-------------
SLLI |  0000000  |  shift   |   src    | 101         |  dest   |
SRLI |  0000000  |   amt    |   reg    | 110         |   reg   |
SRAI |  0100000  |          |          | 111         |         |

*/


module shifter(in,shift,sout);
    input [31:0] in; 
    input [1:0] shift;      // Shift operation
    output reg [31:0] sout; // Shifter Output

    always @(in or shift) begin
        case (shift)
            2'b00: sout = in;       // No shift
            2'b01: sout = in << 1;  // Shift left by 1
            2'b10: sout = in >> 1;  // Shift right by 1
            2'b11: sout = {in[31],in[31:1]}; //shifted right and set msb to in[31]
            default: sout = {32{1'bx}};
        endcase
    end
endmodule