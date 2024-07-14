/*
 * 32 bit RISC-V 32I Register Module
 */

// Each register is 64 bits
// X0 is a zero-register 64'b0 
// 31 other general regisers, X1-X31, can hold any registers

module regfile(i,write_data,write,clk,data_out1,data_out2);
  input      [31:0] i; // Instruction Input
  input      [63:0] write_data; //write data
  input             write, clk; //enable for write and clock
  output reg [63:0] data_out1;
  output reg [63:0] data_out2;
  wire [4:0] readR1, readR2, writeR; // Register related instructions

  assign readR1 = i[19:15];
  assign readR2 = i[24:20];
  assign writeR = i[11:7];

  reg [63:0] X [0:31] = '{default:64'b0}; // 32 X 64-bit registers
 
  // WRITING LOGIC
  always @(posedge clk) begin
    if(write) begin
      if(writeR < 32 && writeR != 0) begin // Cannot write to X0 since X0 always = 0
        X[writeR] <= write_data; // Writenum maps to the index of the specific register to be written in the array.
      end else begin
        X <= '{default:64'b0}; // Reset Registers if writenum is out of range.
      end 
    end
  end

  // READING LOGIC
  always @(*)begin 
    if(readR1 < 32 && readR2 < 32) begin
      data_out1 = X[readR1]; // data_out recieves data stored in the <readnum>-th register of the register array.
      data_out2 = X[readR2];
    end else begin
      data_out1 = '{default:64'b0}; // data_out recieves all 0s if otherwise. 
      data_out2 = '{default:64'b0};
    end 
  end
endmodule
