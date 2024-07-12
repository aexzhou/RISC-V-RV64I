`timescale 1ps/1ps

module datapath_tb;
// Inputs to Datapath
reg [31:0] idata; // 32-bit Instruction
reg clk;
// Error Detection
reg err = 1'b0; 

// Tracking Values
integer test_num = 0;
reg [63:0] data_ref = 64'd0;
reg [63:0] pc_ref = 64'd0;

// Instantiation as an MUT Object
datapath MUT (
    .clk(clk),
    .instruction(idata)
);

// Clock gen
initial begin
    clk = 0;
    forever #5 clk = ~clk; // Delay = 10 for full clk cycle
end

task testset;
    // MUT.pc_out = 0;
    pc_ref = 0; 
    test_num = test_num + 1; // Increments the test number
    #10;
endtask

/* Datapath Testbench */
initial begin
/* Test 1:
 *
 * addi x5, x0, #12
 */
    testset();
    idata = 32'b00000000110000000000001010010011; pc_ref = pc_ref + 4;
    #20;

    data_ref = 64'd12;
    if(MUT.REGFILE.X[5] !== data_ref) begin
        $error("TEST %d FAILED, expected: %h, got: %h",test_num,data_ref,MUT.REGFILE.X[5]); 
        err = 1'b1; $stop; 
    end else begin
        $display("TEST %d PASSED",test_num);
    end
    if(MUT.pc_out !== pc_ref) begin
        $error("TEST %d PC DID NOT INCREMENT, expected: %d, got: %d",test_num,pc_ref,MUT.pc_out); 
        err = 1'b1; $stop; 
    end 

/* Test 2:
 *
 * addi x1, x0, #5
 * addi x2, x0, #6
 * add x3, x1, x2
 */
    testset();
    idata = 32'b00000000010100000000000010010011; pc_ref = pc_ref + 4;
    #10;
    idata = 32'b00000000011000000000000100010011; pc_ref = pc_ref + 4;
    #10;
    idata = 32'b00000000001000001000000110110011; pc_ref = pc_ref + 4;
    #10;

    data_ref = 64'd11;
    if(MUT.REGFILE.X[3] !== data_ref) begin
        $error("TEST %d FAILED, expected: %h, got: %h",test_num,data_ref,MUT.REGFILE.X[3]); 
        err = 1'b1; $stop; 
    end else begin
        $display("TEST %d PASSED",test_num);
    end
    if(MUT.pc_out !== pc_ref) begin
        $error("TEST %d PC DID NOT INCREMENT, expected: %d, got: %d",test_num,pc_ref,MUT.pc_out); 
        err = 1'b1; $stop; 
    end 

/* Test 3:
 *
 * addi x6, x0, 15
 * addi x7, x0, 5
 * sub x4, x7, x6 
 */
    testset();
    idata = 32'b00000000111100000000001100010011; pc_ref = pc_ref + 4;
    #10;
    idata = 32'b00000000010100000000001110010011; pc_ref = pc_ref + 4;
    #10;
    idata = 32'b01000000011000111000001000110011; pc_ref = pc_ref + 4;
    #10;

    data_ref = 64'd10;
    if(MUT.REGFILE.X[4] !== data_ref) begin
        $error("TEST %d FAILED, expected: %h, got: %h",test_num,data_ref,MUT.REGFILE.X[4]); 
        err = 1'b1; $stop; 
    end else begin
        $display("TEST %d PASSED",test_num);
    end
    if(MUT.pc_out !== pc_ref) begin
        $error("TEST %d PC DID NOT INCREMENT, expected: %d, got: %d",test_num,pc_ref,MUT.pc_out); 
        err = 1'b1; $stop; 
    end

/* Test 4:
 *
 * addi x6, x0, 0x0F
 * addi x7, x0, 0x55
 * and x8, x7, x6
 */
    testset();
    idata = 32'b00000000111100000000000110010011; pc_ref = pc_ref + 4;
    #10;
    idata = 32'b00000101010100000000000110010011; pc_ref = pc_ref + 4;
    #10;
    idata = 32'b01000000011000111000001000110011; pc_ref = pc_ref + 4;
    #10;

    data_ref = 64'h5;
    if(MUT.REGFILE.X[8] !== data_ref) begin
        $error("TEST %d FAILED, expected: %h, got: %h",test_num,data_ref,MUT.REGFILE.X[8]); 
        err = 1'b1; $stop; 
    end else begin
        $display("TEST %d PASSED",test_num);
    end
    if(MUT.pc_out !== pc_ref) begin
        $error("TEST %d PC DID NOT INCREMENT, expected: %d, got: %d",test_num,pc_ref,MUT.pc_out); 
        err = 1'b1; $stop; 
    end

/* Test 5:
 *
 * addi x20, x0, 0xF0
 * addi x21, x0, 0x55
 * or x20, x21, x20
 */
    testset();
    idata = 32'b00000000111100000000001100010011; pc_ref = pc_ref + 4;
    #10;
    idata = 32'b00000000010100000000001110010011; pc_ref = pc_ref + 4;
    #10;
    idata = 32'b01000000011000111000001000110011; pc_ref = pc_ref + 4;
    #10;

    data_ref = 64'hf5;
    if(MUT.REGFILE.X[20] !== data_ref) begin
        $error("TEST %d FAILED, expected: %h, got: %h",test_num,data_ref,MUT.REGFILE.X[20]); 
        err = 1'b1; $stop; 
    end else begin
        $display("TEST %d PASSED",test_num);
    end
    if(MUT.pc_out !== pc_ref) begin
        $error("TEST %d PC DID NOT INCREMENT, expected: %d, got: %d",test_num,pc_ref,MUT.pc_out); 
        err = 1'b1; $stop; 
    end


end

endmodule