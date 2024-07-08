module shifter(in,shift,sout);
    input [15:0] in;
    input [1:0] shift;
    output reg [15:0] sout;

    always @(in or shift) begin
        case (shift)
            2'b00: sout = in;       // No shift
            2'b01: sout = in << 1;  // Shift left by 1
            2'b10: sout = in >> 1;  // Shift right by 1
            2'b11: sout = {in[15],in[15:1]}; //shifted right and set msb to in[15]
            default: sout = {16{1'bx}};
        endcase
    end
endmodule