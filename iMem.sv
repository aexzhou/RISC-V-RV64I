import hidden_wires_pkg::*;
import hidden_clk_pkg::*;

/* Instruction memory module will read a 64bit address (PC) and output the corresponding instruction */
module iMem (
    input wire [63:0] address,     // Address input
    output reg [31:0] instruction  // Instruction output
);

    parameter WIDTH = 1024;

    reg [31:0] memory [0:WIDTH-1]; // Instruction memory array (stores <WIDTH> amount of 32-bit instructions)
    
    hidden_wires_pkg::hidden_wires_t        imem_wires;
    hidden_wires_pkg::hidden_wires_t        imem_flag;
    hidden_clk_pkg::hidden_clk_t        hidden_clk;

 

    // Feeding the instructions into memory on startup.
    wire [31:0] memory_temp [0:WIDTH-1];
    assign memory_temp = memory;
	 
    always @(posedge hidden_clk.clk) begin
        hidden_wires_pkg::connect(imem_wires, 1'b0);
        hidden_wires_pkg::connect(imem_flag, 1'b1); // Sets this flag on and send to Top Level if the last address space is used 
        hidden_clk_pkg::connect(hidden_clk, 1'b0); // Recieves a hidden Clock Signal from Top Level

        if(imem_wires.enable) memory[imem_wires.address] = imem_wires.data;
        else memory <= memory_temp;
    end

    // Output the instruction at the given address
    always @(address) begin
        if(address[63:2] == WIDTH) imem_flag = 1;
        else imem_flag = 0;
        instruction <= memory[address[63:2]];    // Address is byte-aligned, so divide by 4
    end
endmodule








    /*
    Initialize the instruction memory with some values in a file
    initial begin
        $readmemh("instructions.mem", memory);  // Load hex instructions from a file, use $readmemb for binary
    end
    */