`timescale 1ps/1ps

module datapath_tb;
//Inputs to Datapath
reg [31:0] idata; // 32-bit Instruction

//Outputs from Datapath
reg [63:0] pc;
// Instantiation as an MUT Object
datapath MUT (
    .clk(clk),
    .instruction(idata),
    .pc(pc)
)

// Clock gen
initial begin
    clk = 0;
    forever #5 clk = ~clk; 
    // Delay = 10 for full clk cycle
end

//Reset 
task resetinputs;
    idata = 0; 
endtask

/* Datapath Testbench */
initial begin
/* Test 1:
 *
 * MOV R3, #42
 * MOV R5, #13
 * ADD R2, R5, R3
 */
    idata = 



end

endmodule