`define MNONE 2'b00
`define MREAD 2'b01
`define MWRITE 2'b10
module lab7_top(KEY,SW,LEDR,HEX0,HEX1,HEX2,HEX3,HEX4,HEX5);

input [3:0] KEY;
input [9:0] SW;
output reg [9:0] LEDR; 
output [6:0] HEX0, HEX1, HEX2, HEX3, HEX4, HEX5;

wire [15:0] din, dout, write_data, read_data;
reg [8:0] PC_out, next_pc, add_out;
reg [7:0] read_address, write_address;
reg clk, reset, write, msel, mwrite, mread, tri_on;

wire[1:0] mem_cmd;
wire[8:0] mem_addr;

assign clk = ~KEY[0];
assign din = write_data;
assign write_address = mem_addr[7:0];
assign read_address = mem_addr[7:0];

//msel, mwrite, mread comparator
always_comb begin
  msel   = (mem_addr[8] == 1'b0) ? 1'b1 : 1'b0; //msel comparator
  mread  = (mem_cmd == `MREAD)   ? 1'b1 : 1'b0;
  mwrite = (mem_cmd == `MWRITE)  ? 1'b1 : 1'b0;
end
//Tri state logic
assign write = msel & mwrite;
assign tri_on = msel & mread;
assign read_data = tri_on ? dout : {16{1'bx}};

//Writing to LEDS: LED reg + LED comb logic
wire [7:0] LEDR_temp;
assign LEDR_temp = LEDR[7:0];
always_ff@(posedge clk) begin 
  if(mem_cmd == 2'b10 && mem_addr == 9'h100)begin 
    LEDR[7:0] = write_data[7:0]; 
  end else begin
    LEDR[7:0] = LEDR_temp;
  end
end


RAM MEM (
  .clk(clk),
  .read_address(read_address),
  .write_address(write_address),
  .write(write),
  .din(din),
  .dout(dout)
);

wire Z, N, V;

cpu CPU(
  .clk(~KEY[0]), // recall from Lab 4 that KEY0 is 1 when NOT pushed
  .reset(~KEY[1]), 
  .read_data(read_data),
  .write_data(write_data),
  .mem_cmd(mem_cmd),
  .mem_addr(mem_addr),
  .Z(Z),
  .N(N),
  .V(V)
);

assign HEX5[0] = ~Z;
assign HEX5[6] = ~N;
assign HEX5[3] = ~V;

// fill in sseg to display 4-bits in hexidecimal 0,1,2...9,A,B,C,D,E,F
sseg H0(write_data[3:0],   HEX0);
sseg H1(write_data[7:4],   HEX1);
sseg H2(write_data[11:8],  HEX2);
sseg H3(write_data[15:12], HEX3);
assign HEX4 = 7'b1111111;
assign {HEX5[2:1],HEX5[5:4]} = 4'b1111; // disabled
assign LEDR[8] = 1'b0;
endmodule
      
module vDFF(clk,D,Q);
  parameter n=1;
  input clk;
  input [n-1:0] D;
  output [n-1:0] Q;
  reg [n-1:0] Q;
  always @(posedge clk)
    Q <= D;
endmodule


//7 segment display
module sseg(in,segs);
  input [3:0] in;
  output reg [6:0] segs;

always @(*)begin
case(in)
  4'h0: segs = 7'b1000000;
  4'h1: segs = 7'b1111001;
  4'h2: segs = 7'b0100101;
  4'h3: segs = 7'b0110000;
  4'h4: segs = 7'b0011001;
  4'h5: segs = 7'b0010010;
  4'h6: segs = 7'b0000010;
  4'h7: segs = 7'b1111000;
  4'h8: segs = 7'b0000000;
  4'h9: segs = 7'b0010000;
  4'hA: segs = 7'b0001000;
  4'hb: segs = 7'b0000011;
  4'hC: segs = 7'b1000110;
  4'hd: segs = 7'b0100001;
  4'hE: segs = 7'b0000110;
  4'hF: segs = 7'b0001110;
  default: segs = 7'bxxxxxxx;
endcase
end
endmodule

module RAM(clk,read_address,write_address,write,din,dout);
  parameter data_width = 16; 
  parameter addr_width = 8;
  parameter filename = "data.txt";

  input clk;
  input [addr_width-1:0] read_address, write_address;
  input write;
  input [data_width-1:0] din;
  output [data_width-1:0] dout;
  reg [data_width-1:0] dout;

  reg [data_width-1:0] mem [2**addr_width-1:0];

  initial $readmemb(filename, mem);

  always @ (posedge clk) begin
    if (write)
      mem[write_address] <= din;
    dout <= mem[read_address]; 
// dout doesn't get din in this clock cycle 
                               
// (this is due to Verilog non-blocking assignment "<=")
  end 
endmodule