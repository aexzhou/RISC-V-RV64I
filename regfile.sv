/*
 * 32 bit RISC-V 32I Register Module
 */

// Each register is 64 bits
// X0 is a zero-register 64'b0 
// 31 other general regisers, X1-X31, can hold any registers

module regfile(write_data,writenum,write,readnum1,readnum2,clk,data_out1,data_out2);
  input      [31:0] write_data; //write data
  input       [4:0] writenum, readnum1, readnum2; // 5 bit read/write num to target all 32 registers (2^5)
  input             write, clk; //enable for write and clock
  output reg [31:0] data_out1;
  output reg [31:0] data_out2;

  reg [31:0] X [0:31] = '{default:32'b0}; // Array declaration of 32 of 32-bit registers
 
  // WRITING LOGIC
  always @(posedge clk) begin
    if(write) begin
      if(writenum < 32 && writenum != 0) begin // Cannot write to X0 since X0 always = 0
        X[writenum] <= data_in; // Writenum maps to the index of the specific register to be written in the array.
      end else begin
        X <= '{default:32'b0}; // Reset Registers if writenum is out of range.
      end 
    end
  end

  // READING LOGIC
  always @(*)begin 
    if(readnum1 < 32 && readnum2 < 32) begin
      data_out1 = X[readnum1]; // data_out recieves data stored in the <readnum>-th register of the register array.
      data_out2 = X[readnum2];
    end else begin
      data_out1 = '{default:32'b0}; // data_out recieves all 0s if otherwise. 
      data_out2 = '{default:32'b0};
    end 
  end
endmodule
