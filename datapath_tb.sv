`timescale 1ps/1ps

module datapath_tb;
//Inputs
reg [15:0] read_data, sximm8, sximm5, mdata;
reg [2:0] writenum, readnum;
reg [7:0] PC;
reg [1:0] shift, ALUop, vsel;
reg clk, write, asel, bsel, loada, loadb, loadc, loads;
reg shift_ctrl;

//Outputs
wire[15:0] datapath_out;
wire Z_out;

//Error Tracker
reg err;

//Instantiation under MUT
datapath mut (
    .clk(clk),
    .write(write),
    .mdata(mdata),
    .PC(PC),
    .ALUop(ALUop),
    .sximm8(sximm8),
    .sximm5(sximm5),
    .shift(shift),
    .readnum(readnum),
    .writenum(writenum),
    .asel(asel),
    .bsel(bsel),
    .vsel(vsel),
    .loada(loada),
    .loadb(loadb),
    .loadc(loadc),
    .loads(loads),
    .datapath_out(datapath_out),
    .Z_out(Z_out),
    .shift_ctrl(shift_ctrl)
);

//Clock generator with #5 time gap
initial begin
    clk = 0;
    forever #5 clk = ~clk;
end

//Reset 
task resetinputs;
    PC = 0;
    sximm8 = 0;
    sximm5 = 0;
    mdata = 0;
    writenum = 0;
    readnum = 0;
    shift = 0;
    ALUop = 0;
    write = 0;
    asel = 0;
    bsel = 0;
    vsel = 0;
    loada = 0;
    loadb = 0;
    loadc = 0;
    loads = 0;
endtask

//Testbench - all delays will be made #10
initial begin
    err = 0;
    shift_ctrl = 0; //no need here
    resetinputs();
    #10; //10

// ** Test 1: ------------------------------ // 
//      MOV R3, #42
//      MOV R5, #13
//      ADD R2, R5, R3

    vsel = 3;
    read_data = 42;
    writenum = 3;
    write = 1;
    #10; //20

    // MOV R5, #13
    vsel = 3;
    read_data = 13;
    writenum = 5;
    write = 1;
    #10; //30

    //ADD R2, R5, R3 -- OP dest, A, B
    readnum = 3; //load R3 into B
    loadb = 1;
    #10; //40
    loadb = 0;

    readnum = 5; //load R5 into A
    loada = 1;
    #10; //50
    loada = 0;

    shift = 0; //no shift
    asel = 0; //A out permitted
    bsel = 0; //B out permitted 
    ALUop = 0; //Add operation
    loadc = 1; //Allows load add result into C
    #10; //60

    writenum = 2; //gates to R2 opened 
    write = 1; //write mode activated
    vsel = 0; //sources from loadc and not datapath_in
    #10; //70

    resetinputs();
    #10;
    //## CHECK: 42 + 13 = 55
        readnum = 2;
        loadb = 1;
        #10;
        loadb = 0;

        shift = 0; 
        asel = 1;
        bsel = 0;
        ALUop = 0;
        loadc = 1;
        #10; 
    if(datapath_out == 55) begin
        $display("Test 1 Passed");
    end else begin
        $display("Test 1 Failed");
        err = 1;
    end
    #10;

    resetinputs();
    #10; //110

// ** Test 2: ------------------------------ //
//      MOV R1, #10
//      MOV R2, #20
//      AND R0, R2, R1, LSR #1
//              10100 & 00101 = 00100 -> 0000000000000100
//      MVN R4, R0
//              1111111111111011
//      Expected: 1111111111111011 (dec 65531)

    vsel = 3;
    read_data = 10;
    writenum = 1;
    write = 1;
    #10; 
    
    vsel = 3;
    read_data = 20;
    writenum = 2;
    write = 1;
    #10; 

    readnum = 1; 
    loadb = 1;
    #10; 
    loadb = 0;    

    readnum = 2;
    loada = 1;
    #10; 
    loada = 0;

    shift = 2; // div by 2
    asel = 0; 
    bsel = 0; 
    ALUop = 2; // &
    loadc = 1; 
    #10; 
    loadc = 0;

    writenum = 0; // write to R0
    write = 1; 
    vsel = 0; 
    #10; 
    write = 0;

    readnum = 0; //read from R0
    loadb = 1;
    #10; 
    loadb = 0;

    shift = 0;
    asel = 1; 
    bsel = 0; 
    ALUop = 3; //~B
    loadc = 1; //Allows load add result into C
    #10; 
    loadc = 0;    

    writenum = 4;  //write to R4
    write = 1; 
    vsel = 0; 
    #10; 

    resetinputs();
    #10;
    //## CHECK: R4 contains 65531
        readnum = 4; //220
        loadb = 1;
        #10;
        loadb = 0;

        shift = 0; 
        asel = 1;
        bsel = 0;
        ALUop = 0;
        loadc = 1;
        #10; 
    if(datapath_out == 65531) begin
        $display("Test 2 Passed");
    end else begin
        $display("Test 2 Failed");
        err = 1;
    end
    #10;

    resetinputs();
    #10; 

// ** Test 3: ------------------------------ //
//      MOV R4, # 65532 (1111 1111 1111 1100)
//      MOV R6, # 4 (100)
//      ADD R7, R4, R6

    vsel = 3;
    read_data = 16'b1111111111111100;
    writenum = 4;
    write = 1;
    #10; 
    
    vsel = 3;
    read_data = 4;
    writenum = 6;
    write = 1;
    #10; 

    readnum = 6; 
    loadb = 1;
    #10; 
    loadb = 0;    

    readnum = 4;
    loada = 1;
    #10; 
    loada = 0;

    shift = 0;
    asel = 0; 
    bsel = 0; 
    ALUop = 0; //ADD
    loadc = 1; 
    #10; 
    loadc = 0;

    writenum = 7;  //write to R7
    write = 1; 
    vsel = 0; 
    loads = 1;
    #10; 
 
    resetinputs();
    #10;
    //## CHECK: R7 contains 0 and Z_out is 1
        readnum = 7;
        loadb = 1;
        #10;
        loadb = 0;

        shift = 0; 
        asel = 1;
        bsel = 0;
        ALUop = 0;
        loadc = 1;
        #10; 
    if(datapath_out == 0 && Z_out == 1) begin
        $display("Test 3 Passed");
    end else begin
        $display("Test 3 Failed");
        err = 1;
    end
    #10;

    resetinputs();
    #10; 

// ** Test 4: ------------------------------ //
//      MOV R4, #32768 (1000 0000 0000 0000)
//      ASR R0, R4, #1
//      Expected: 1100 0000 0000 0000 in R0

    vsel = 3;
    read_data = 16'b1000000000000000;
    writenum = 4;
    write = 1;
    #10; 

    readnum = 4;
    loadb = 1;
    #10 
    loadb = 0;

    shift = 3;
    asel = 1; 
    bsel = 0; 
    ALUop = 0; //ADD
    loadc = 1; 
    #10; 
    loadc = 0;

    writenum = 0;  //write to R7
    write = 1; 
    vsel = 0; 
    #10; 

    resetinputs();
    #10;
    //## CHECK: R0 contains 16'b1100000000000000
        readnum = 0;
        loadb = 1;
        #10;
        loadb = 0;

        shift = 0; 
        asel = 1;
        bsel = 0;
        ALUop = 0;
        loadc = 1;
        #10; 
    if(datapath_out == 16'b1100000000000000) begin
        $display("Test 4 Passed");
    end else begin
        $display("Test 4 Failed");
        err = 1;
    end
    #10;
    resetinputs();
    #10;
end

//Simulation Stopper
initial begin
    #500;
    $stop;
end
endmodule