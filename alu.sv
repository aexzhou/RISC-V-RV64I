// ALUop inputs
// 4'b0000: AND
// 4'b0001: OR
// 4'b0010: ADD
// 4'b0110: SUB



module ALU(in1, in2, ALUop, out, Z);
    input [63:0] in1, in2;
    input [3:0] ALUop;
    output reg [63:0] out;
    output reg Z; // Zero flag

    // //ADDSUB connection
    // wire [31:0] s;
    // wire ovf;

    // AddSub addsub(
    //     .a(in1),
    //     .b(in2),
    //     .sub(ALUop[2]),
    //     .s(s),
    //     .ovf(ovf)
    // );

    always @(*) begin
        case(ALUop)
            4'b0000: out = in1 && in2;
            4'b0001: out = in1 || in2;
            4'b0010: begin
                out = in1 + in2;
            end
            4'b0110: begin
                out = in1 - in2;
            end
            default: out = {32{1'bx}};
        endcase

        if(out == 0)begin   // Zero flag
            Z = 1'b1;    
        end else begin
            Z = 1'b0;
        end

    end
endmodule

// // From Dally 10.3 and Slide Set 10, Slide 28
// // add a+b or subtract a-b, check for overflow (AddSub is only used to calculate overflow)
// module AddSub(a,b,sub,s,ovf) ;
//     parameter n = 32 ;
//     input [n-1:0] a, b ;
//     input sub ;           // subtract if sub=1, otherwise add
//     output [n-1:0] s ;
//     output ovf ;          // 1 if overflow
//     wire c1, c2 ;         // carry out of last two bits
//     assign ovf = c1 ^ c2 ;  // overflow if signs don't match    

//     // add non sign bits
//     assign {c1,s[n-2:0]} = a[n-2:0] + (b[n-2:0] ^ {n-1{sub}}) + sub;
//     // add sign bits
//     assign {c2,s[n-1]} = a[n-1] + (b[n-1] ^ sub) + c1;
// endmodule
