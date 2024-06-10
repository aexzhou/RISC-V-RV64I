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


//register with load enable
module vDFFE(clk, en, in, out) ;
  parameter n = 1;  // width
  input clk, en ;
  input [n-1:0] in ;
  output reg [n-1:0] out ;
  wire [n-1:0] next_out ;

  assign next_out = en ? in : out;

  always @(posedge clk)
    out = next_out;  
endmodule

//16 bit mux
module MUX16bit(in0, in1, sel, out);
    input [15:0] in0;
    input [15:0] in1;
    input sel;
    output [15:0] out;

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