/*
 * 32 bit RISC-V 32I Register Module
 */

// Each register is 32 bits
// X0 is a zero-register 32'b0 
// 31 other general regisers, X1-X31, can hold any registers

module regfile(data_in,writenum,write,readnum,clk,data_out);
  input      [31:0] data_in;

  input       [4:0] writenum, readnum; // 5 bit read/write num to target all 32 registers (2^5)
  input             write, clk;
  output reg [31:0] data_out;

  reg [31:0] X [0:31] = '{default:32'b0}; // Array declaration of 32 of 32-bit registers
 
  // REGISTER LOGIC/WRITING LOGIC
  always @(posedge clk) begin
    if(write) begin
      if(writenum < 32 && writenum != 0) begin // Cannot write to X0 since X0 always = 0
        X[writenum] <= data_in; // Writenum maps to the index of the specific register to be written in the array.
      end else begin
        X <= '{default:32'b0}; // Reset Registers if writenum is out of range.
      end 
    end
  end

  always @(*)begin 
    if(readnum < 32) begin
      data_out = X[readnum]; // data_out recieves data stored in the <readnum>-th register of the register array.
    end else begin
      data_out = '{default:32'b0}; // data_out recieves all 0s if otherwise. 
    end 
  end
endmodule
