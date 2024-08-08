// The data memory module supports both read and write operations
// It has separate input ports for address, data to be written, and control signals for reading and writing
module dataMem (
    input wire clk, rst,             
    input wire [63:0] address,       // Address input
    input wire [63:0] write_data,    // Data to write
    input wire mem_read,             // Memory read enable
    input wire mem_write,            // Memory write enable
    output reg [63:0] read_data      // Data read output
);
    parameter WIDTH = 1024;

    reg [63:0] memory [0:WIDTH-1]; // Data memory array (stores <WIDTH> amount of 64-bit data)

    /*
    The following code is only necessary if you wish to initialize the RAM 
    contents via an external file (use $readmemb for binary data) 
    */
    // initial
    //    $readmemh("<data_file_name>", <ram_name>, <begin_address>, <end_address>);
    
    // Read data
    always @(posedge clk) begin
        if (mem_read) begin
            read_data <= memory[address[63:3]];  // Address is byte-aligned, so divide by 8
        end
    end

    // Write data
    always @(posedge clk) begin
        if (mem_write) begin
            memory[address[63:3]] <= write_data;  // Address is byte-aligned, so divide by 8
        end
    end
endmodule