module regfile_tb;

    reg [63:0] write_data;
    reg [4:0] writenum, readnum1, readnum2;
    reg write, clk;
    wire [63:0] data_out1, data_out2;

    // Instantiate the regfile
    regfile uut (
        .write_data(write_data),
        .writenum(writenum),
        .write(write),
        .readnum1(readnum1),
        .readnum2(readnum2),
        .clk(clk),
        .data_out1(data_out1),
        .data_out2(data_out2)
    );

    // Clock generation
    initial begin
        clk = 0;
        forever #5 clk = ~clk; // Toggle clock every 5 time units
    end

    initial begin
        // Monitor output
        $monitor("Time = %d, clk = %b, write = %b, writenum = %d, write_data = %d, readnum1 = %d, data_out1 = %d, readnum2 = %d, data_out2 = %d",
                 $time, clk, write, writenum, write_data, readnum1, data_out1, readnum2, data_out2);

        // Test writing to register X1
        #10;
        write = 1;
        writenum = 5'd1;
        write_data = 64'd42;
        #10;
        write = 0;
        
        // Test reading from register X1
        readnum1 = 5'd1;
        readnum2 = 5'd0; // Read from X0 which should always be 0
        #10;

        // Test writing to another register X2
        write = 1;
        writenum = 5'd2;
        write_data = 64'd84;
        #10;
        write = 0;

        // Test reading from both registers X1 and X2
        readnum1 = 5'd1;
        readnum2 = 5'd2;
        #10;

        // Test reading from an out-of-bounds register
        readnum1 = 5'd32;
        readnum2 = 5'd32;
        #10;

        // Test writing to register X0 (should not change)
        write = 1;
        writenum = 5'd0;
        write_data = 64'd100;
        #10;
        write = 0;

        // Test reading from X0 again (should still be 0)
        readnum1 = 5'd0;
        readnum2 = 5'd1;
        #10;

        // Finish simulation
        $stop;
    end
endmodule
