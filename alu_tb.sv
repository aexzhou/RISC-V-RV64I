module ALU_tb();

    //DUT inputs and outputs
    reg [63:0] in1, in2;
    reg [3:0] ALUop;
    reg [63:0] out;
    reg Z;

    // Instantiate the DUT object
    ALU DUT (
        .in1(in1),
        .in2(in2),
        .ALUop(ALUop),
        .out(out),
        .Z(Z)
    );

    // Clock gen
    initial begin
        clk = 0;
        forever #5 clk = ~clk; // Delay = 10 for full clk cycle
    end

    initial begin
        // Error detection
        reg err = 1'b0; 

        // Initialize inputs
        in1 = 64'd0;
        in2 = 64'd0;
        ALUop = 4'b0000;

        #10;

        // Test AND operation
        in1 = 64'd10; in2 = 64'd12; ALUop = 4'b0000; // 10 AND 12
        #10;
        if(out !== 64'd8) begin
            $error("AND operation failed, expected: %h, got: %h", 64'd8, out);
            err = 1'b1; $stop;
        end else begin
            $display("AND operation passed, expected: %h, got: %h", 64'd8, out);
        end

        
        // Test OR operation
        in1 = 64'd10; in2 = 64'd12; ALUop = 4'b0001; // 10 OR 12
        #10;
        if(out !== 64'd14) begin
            $error("OR operation failed, expected: %h, got: %h", 64'd14, out);
            err = 1'b1; $stop;
        end else begin
            $display("OR operation passed, expected: %h, got: %h", 64'd14, out);
        end
        
        // Test ADD operation
        in1 = 64'd10; in2 = 64'd12; ALUop = 4'b0010; // 10 + 12
        #10;
        if(out !== 64'd22) begin
            $error("ADD operation failed, expected: %h, got: %h", 64'd22, out);
            err = 1'b1; $stop;
        end else begin
            $display("ADD operation passed, expected: %h, got: %h", 64'd22, out);
        end
        
        // Test SUB operation
        in1 = 64'd15; in2 = 64'd10; ALUop = 4'b0110; // 15 - 10
        #10;
        if(out !== 64'd5) begin
            $error("SUB operation failed, expected: %h, got: %h", 64'd5, out);
            err = 1'b1; $stop;
        end else begin
            $display("SUB operation passed, expected: %h, got: %h", 64'd5, out);
        end
        
        // Test Zero flag
        in1 = 64'd10; in2 = 64'd10; ALUop = 4'b0110; // 10 - 10 (should set Z flag)
        #10;
        if(Z !== 1'b1) begin
            $error("Zero flag failed, expected: %h, got: %h", 1'b1, Z);
            err = 1'b1; $stop;
        end else begin
            $display("Zero flag passed, expected: %h, got: %h", 1'b1, Z);
        end

        // Test another Zero flag scenario
        in1 = 64'd0; in2 = 64'd0; ALUop = 4'b0000; // 0 AND 0 (should set Z flag)
        #10;
        if(Z !== 1'b1) begin
            $error("Zero flag failed, expected: %h, got: %h", 1'b1, Z);
            err = 1'b1; $stop;
        end else begin
            $display("Zero flag passed, expected: %h, got: %h", 1'b1, Z);
        end

        // Test default case
        ALUop = 4'b1111; // Invalid ALUop
        #10;
        if(out !== 64'hxxxxxxxxxxxxxxxx) begin
            $error("Default case failed, expected: %h, got: %h", 64'hxxxxxxxxxxxxxxxx, out);
            err = 1'b1; $stop;
        end else begin
            $display("Default case passed, expected: %h, got: %h", 64'hxxxxxxxxxxxxxxxx, out);
        end

        $stop;
    end

endmodule
