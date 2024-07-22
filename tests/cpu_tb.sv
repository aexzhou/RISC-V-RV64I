module cpu_tb();

    // DUT inputs
    reg clk, reset, addr_sel;
    reg [15:0] read_data;

    // DUT outputs
    reg [15:0] write_data;
    reg [8:0] mem_addr;
    reg [1:0] mem_cmd;
    reg N, V, Z;

    // Instantiate the DUT
    cpu DUT(
        .clk(clk),
        .reset(reset),
        .read_data(read_data),
        .write_data(write_data),
        .mem_cmd(mem_cmd),
        .mem_addr(mem_addr),
        .Z(Z),
        .V(V),
        .N(N)
    );

    // Error detection
    reg err = 1'b0; 
initial begin
    clk = 1'b0; #5;
    forever begin 
        clk = 1'b1; #5;
        clk = 1'b0; #5;
    end
end


initial begin 
// clk starts at 0
// assert reset 

reset = 1'b1;
#10;

// MOV R6, #3
reset = 1'b0;
#10;
read_data = 16'b1101011000000011;

// Setting the state machine in motion 
#40;
@(posedge DUT.PC); // halt till wait state //
if(cpu_tb.DUT.DP.REGFILE.R6 !== 16'h3) begin 
    err = 1'b1;

    $display("MOV R6, #3 was not successful");
    $stop;
end else begin 
    $display("MOV R6, #3 was successful");
end

@(negedge clk);
// MOV R1, R6, LSL#1
read_data = 16'b1100000000101110;
#10;


@(posedge DUT.PC);
if(cpu_tb.DUT.DP.REGFILE.R1 !== 16'h6) begin 
    err = 1'b1;
    $display("MOV R1, R6, LSL#1 was not successful");
    $stop;
end else begin 
    $display("MOV R1, R6, LSL#1 was successful");
end

@(negedge clk);
// MVN R3,R6
read_data = 16'b1011100001100110;
#10;

@(posedge DUT.PC);
if(cpu_tb.DUT.DP.REGFILE.R3 !== ~(cpu_tb.DUT.DP.REGFILE.R6)) begin 
    err = 1'b1;
    $display("MVN R3,R6 was not successful");
    $stop;
end else begin 
    $display("MVN R3,R6 was successful");
end

@(negedge clk);
// MVN R0,R1
read_data = 16'b1011100000000001;
#10;

@(posedge DUT.PC);
if(cpu_tb.DUT.DP.REGFILE.R0 !== ~(cpu_tb.DUT.DP.REGFILE.R1)) begin 
    err = 1'b1;
    $display("MVN R0,R1 was not successful");
    $stop;
end else begin 
    $display("MVN R0,R1 was successful");
end

@(negedge clk); 
// MOV R4, #2
read_data = 16'b1101010000000010;
#10;


@(posedge DUT.PC);
if(cpu_tb.DUT.DP.REGFILE.R4 !== 16'h2) begin 
    err = 1'b1;
    $display("MOV R4, #2 was not successful");
    $stop;
end else begin 
    $display("MOV R4, #2 was successful");
end

@(negedge clk);
// MVN R0,R4
read_data = 16'b1011100000000100;
#10;

@(posedge DUT.PC);
if(cpu_tb.DUT.DP.REGFILE.R0 !== ~(cpu_tb.DUT.DP.REGFILE.R4)) begin 
    err = 1'b1;
    $display("MVN R0,R1 was not successful");
    $stop;
end else begin 
    $display("MVN R0,R1 was successful");
end

@(negedge clk);
// AND R5, R4, R6
read_data = 16'b1011010010100110;
#10;

@(posedge DUT.PC);
if(cpu_tb.DUT.DP.REGFILE.R5 !== 16'h2) begin 
    err = 1'b1;
    $display("AND R5, R4, R6 was not successful");
    $stop;
end else begin 
    $display("AND R5, R4, R6 was successful");
end


@(negedge clk);
// AND R0, R1, R6
read_data = 16'b1011000100000110;
#10;

@(posedge DUT.PC);
if(cpu_tb.DUT.DP.REGFILE.R0 !== 16'h2) begin 
    err = 1'b1;
    $display("AND R5, R4, R6 was not successful");
    $stop;
end else begin 
    $display("AND R5, R4, R6 was successful");
end

@(negedge clk);
// CMP R5, R1, LSR#1
read_data = 16'b1010110100010001;
#10;

@(posedge DUT.PC);
if(N !== 1 && V !== 0 && Z !== 0) begin 
    err = 1'b1;
    $display("Negative flag detection was not successful");
    $stop;
end else begin 
    $display("Negative flag detection was successful");

end

@(negedge clk)
// ADD R7,R4,R6
read_data = 16'b1010010011100110;
#10;

@(posedge DUT.PC);
if(cpu_tb.DUT.DP.REGFILE.R7 !== 16'h5) begin 
    err = 1'b1;
    $display("ADD R7,R4,R6 was not successful");
    $stop;
end else begin 
    $display("ADD R7,R4,R6 was successful");
end

@(negedge clk)
// ADD R7,R4,R1
read_data = 16'b1010010011100001;
#10;

@(posedge DUT.PC);
if(cpu_tb.DUT.DP.REGFILE.R7 !== 16'h8) begin 
    err = 1'b1;
    $display("ADD R7,R4,R1 was not successful");
    $stop;
end else begin 
    $display("ADD R7,R4,R1 was successful");
end



//----------Overflow detection---------------// 

@(negedge clk)
// MOV R1, #100
read_data = 16'b1101000101100100;
#10;

@(posedge DUT.PC);
if(cpu_tb.DUT.DP.REGFILE.R1 !== 16'h64) begin 
    err = 1'b1;
    $display("MOV R1, #100 was not successful");
    $stop;
end else begin 
    $display("MOV R1, #100 was successful");
end

@(negedge clk)
// MOV R0, #50
read_data = 16'b1101000000110010;
#10;

@(posedge DUT.PC);
if(cpu_tb.DUT.DP.REGFILE.R0 !== 16'h32) begin 
    err = 1'b1;
    $display("MOV R0, #50 was not successful");
    $stop;
end else begin 
    $display("MOV R0, #50 was successful");
end

@(negedge clk)
// ADD R2,R0,R1
read_data = 16'b1010000001000001;
#10;

@(posedge DUT.PC); 
if(N !== 0 && V !== 1 && Z !== 0) begin 
    err = 1'b1;
    $display("Overflow detection was not successful");
    $stop;
end else begin 
    $display("Overflow detection was successful");

end

$stop; 
 end


endmodule
