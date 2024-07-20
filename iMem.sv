/* Instruction memory module will read a 64bit address (PC) and output the corresponding instruction */
module iMem (
    input wire [63:0] address,     // Address input
    output reg [31:0] instruction  // Instruction output
);

    parameter WIDTH = 256;

    reg [31:0] memory [0:WIDTH-1]; // Instruction memory array (stores <WIDTH> amount of 32-bit instructions)

    /*
    Initialize the instruction memory with some values in a file
    initial begin
        $readmemh("instructions.mem", memory);  // Load hex instructions from a file, use $readmemb for binary
    end
    */

    // Output the instruction at the given address
    always @(address) begin
        instruction = memory[address[63:2]];    // Address is byte-aligned, so divide by 4
    end
endmodule