// ALUop inputs
// 4'b0000: AND
// 4'b0001: OR
// 4'b0010: ADD
// 4'b0110: SUB



module ALU(Ain, Bin, ALUop, out, Z);
    input [31:0] Ain, Bin;
    input [3:0] ALUop;
    output reg [31:0] out;
    output reg [2:0] Z; //Z[2] is overflow, Z[1] Negative, Z[0] Zero

    //ADDSUB connection
    wire [31:0] s;
    wire ovf;

    AddSub addsub(
        .a(Ain),
        .b(Bin),
        .sub(ALUop[2]),
        .s(s),
        .ovf(ovf)
    );

    always @(*) begin
        case(ALUop)
            4'b0000: out = Ain && Bin;
            4'b0001: out = Ain || Bin;
            4'b0010: begin
                out = Ain + Bin;
                Z[2] = ovf; // Overflow flag, provided by addsub
            end
            4'b0110: begin
                out = Ain - Bin;
                Z[2] = ovf; // Overflow flag, provided by addsub
            end
            default: out = {32{1'bx}};
        endcase

        if(out == 0)begin   // Zero flag
            Z[0] = 1'b1;    
        end else begin
            Z[0] = 1'b0;
        end

        if(out[31] == 1'b1)begin    // Negative Flag
            Z[1] = 1'b1;
        end else begin
            Z[1] = 1'b0;
        end
    end
endmodule

// From Dally 10.3 and Slide Set 10, Slide 28
// add a+b or subtract a-b, check for overflow (AddSub is only used to calculate overflow)
module AddSub(a,b,sub,s,ovf) ;
    parameter n = 32 ;
    input [n-1:0] a, b ;
    input sub ;           // subtract if sub=1, otherwise add
    output [n-1:0] s ;
    output ovf ;          // 1 if overflow
    wire c1, c2 ;         // carry out of last two bits
    assign ovf = c1 ^ c2 ;  // overflow if signs don't match    

    // add non sign bits
    assign {c1,s[n-2:0]} = a[n-2:0] + (b[n-2:0] ^ {n-1{sub}}) + sub;
    // add sign bits
    assign {c2,s[n-1]} = a[n-1] + (b[n-1] ^ sub) + c1;
endmodule
