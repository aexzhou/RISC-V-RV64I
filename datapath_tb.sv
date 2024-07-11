`timescale 1ps/1ps

module datapath_tb;
// Inputs to Datapath
reg [31:0] idata; // 32-bit Instruction

// Error Detection
reg err = 1'b0; 

// Instantiation as an MUT Object
datapath MUT (
    .clk(clk),
    .instruction(idata),
    .pc(pc)
)

// Clock gen
initial begin
    clk = 0;
    forever #5 clk = ~clk; // Delay = 10 for full clk cycle
end

//Reset 
task resetinputs;
    idata = 0; 
endtask

/* Datapath Testbench */
initial begin
/* Test 1: 
 *
 * addi x1, x0, #5
 * addi x2, x0, #6
 * add x3, x1, x2
 */
    idata = 00000000010100000000000010010011;
    #10;
    idata = 00000000011000000000000100010011;
    #10;
    idata = 00000000001000001000000110110011;
    #10;

    if(MUT.REGFILE.X3 !== 64'd11) begin
        $display("TEST 1 FAILED"); err = 1'b1; $stop; 
    end else begin
        $display("TEST 1 PASSED");
    end
    
    if(MUT.PC !== 64'd11) begin
        $display("TEST 1 FAILED"); err = 1'b1; $stop; 
    end else begin
        $display("TEST 1 PASSED");
    end






end

endmodule