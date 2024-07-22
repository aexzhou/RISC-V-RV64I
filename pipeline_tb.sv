`timescale 1ps/1ps

module pipeline_tb;
// Inputs to Datapath
reg clk;
reg rst;

// Instantiation as an MUT Object
datapath MUT (
    .clk(clk),
    .rst(rst)
);

reg err = 1'b0; // Error Signal
integer test_num = 0; // Tracking Test Number
reg [63:0] data_ref = 64'd0;
reg [63:0] pc_ref = 64'd0;

initial begin // Clock gen
    clk = 0;
    forever #5 clk = ~clk; // Delay = 10 for full clk cycle
end

initial begin // Terminator
    #500;
    $display("Ending Testbench...");
    $stop;
end

task testset; // Run before starting a new test
    rst = 1;
    test_num = test_num + 1; // Increments the test number
    #10;
    rst = 0;
endtask

task testwait;
    #16;
endtask

initial begin
    $display("Loading instructions...");
    $readmemh("pipelinetest.txt", MUT.IMEM.memory);

    // 1
    testset();
    MUT.PC.out = 64'h0;
    testwait();
    data_ref = 64'd11;
    if(MUT.REGFILE.X[4] !== data_ref) $error("TEST %d FAILED, expected: %h, got: %h",test_num,data_ref,MUT.REGFILE.X[5]); err = 1'b1; $stop; 
    data_ref = 64'd12;
    if(MUT.REGFILE.X[3] !== data_ref) $error("TEST %d FAILED, expected: %h, got: %h",test_num,data_ref,MUT.REGFILE.X[5]); err = 1'b1; $stop; 
    $display("TEST %d PASSED",test_num);
    
    // 2
    testset();
    MUT.PC.out = 64'h0;
    testwait();
    data_ref = -64'd1;
    if(MUT.REGFILE.X[4] !== data_ref) $error("TEST %d FAILED, expected: %h, got: %h",test_num,data_ref,MUT.REGFILE.X[5]); err = 1'b1; $stop; 
    $display("TEST %d PASSED",test_num);
    
    // 3
    testset();
    MUT.PC.out = 64'h0;
    testwait();
    data_ref = -64'd1;
    if(MUT.REGFILE.X[1] !== data_ref) $error("TEST %d FAILED, expected: %h, got: %h",test_num,data_ref,MUT.REGFILE.X[5]); err = 1'b1; $stop; 
    $display("TEST %d PASSED",test_num);

    // 4
    testset();
    MUT.PC.out = 64'h0;
    testwait();
    data_ref = -64'd1;
    if(MUT.REGFILE.X[2] !== data_ref) $error("TEST %d FAILED, expected: %h, got: %h",test_num,data_ref,MUT.REGFILE.X[5]); err = 1'b1; $stop; 
    $display("TEST %d PASSED",test_num);

    // 5
    testset();
    MUT.PC.out = 64'h0;
    testwait();
    data_ref = 64'd11;
    if(MUT.REGFILE.X[4] !== data_ref) $error("TEST %d FAILED, expected: %h, got: %h",test_num,data_ref,MUT.REGFILE.X[5]); err = 1'b1; $stop; 
    $display("TEST %d PASSED",test_num);

    // 6
    testset();
    MUT.PC.out = 64'h0;
    testwait();
    data_ref = 64'd8;
    if(MUT.REGFILE.X[1] !== data_ref) $error("TEST %d FAILED, expected: %h, got: %h",test_num,data_ref,MUT.REGFILE.X[5]); err = 1'b1; $stop; 
    data_ref = 64'd9;
    if(MUT.REGFILE.X[2] !== data_ref) $error("TEST %d FAILED, expected: %h, got: %h",test_num,data_ref,MUT.REGFILE.X[5]); err = 1'b1; $stop; 
    $display("TEST %d PASSED",test_num);
end

endmodule