//states for the fsm
`define RST      4'd0
`define IF1      4'd1
`define IF2      4'd2
`define UpdatePC 4'd3 
`define DECODE   4'd4
`define LoadA    4'd5
`define LoadB    4'd6
`define LoadC    4'd7
`define MEMldr   4'd8
`define Loadstr  4'd9
`define MEMstr   4'd10
`define MEMbuff  4'd11
`define WRITE    4'd12

// {opcode,op} definitions for operations
`define MOVi 5'b11010
`define MOVr 5'b11000
`define ADD  5'b10100
`define CMP  5'b10101
`define AND  5'b10110
`define MVN  5'b10111
`define LDR  5'b01100
`define STR  5'b10000
`define HALT 5'b11100

module cpu(clk,reset,read_data,write_data,mem_cmd,mem_addr,Z,V,N);
input clk, reset;
input [15:0] read_data;
output [15:0] write_data;
output reg [8:0] mem_addr;
output reg [1:0] mem_cmd;
output Z,V,N;

reg [15:0]  ins_reg, ins_reg_temp, datapath_out, sximm5, sximm8, mdata;
reg [2:0]   opcode, Rn, Rd, Rm, readnum, writenum, Z_out;
reg [7:0]   imm8, PC;
reg [4:0]   imm5;
reg [1:0]   op, shift, ALUop, vsel, nsel;
reg         loada, loadb, loadc, loads, asel, bsel, write, w_out;
reg         reset_pc, load_pc, load_ir, load_addr, addr_sel;

reg [8:0]   next_pc, pc_out, pc_temp;
reg [8:0]   data_address_temp, data_address_out;

//fsm state declaration
reg [3:0] state;

assign PC = pc_out[7:0];

assign out = datapath_out;
assign w = w_out;

//Z_out Flag Assignment
assign Z = Z_out[0];
assign N = Z_out[1];
assign V = Z_out[2];

//CPU in/out assignments to adapt to datapath
assign mdata = read_data;
assign write_data = datapath_out;

//Instruction Register dff
assign ins_reg_temp = ins_reg;
always_ff @(posedge clk) begin
    if(load_ir) begin
        ins_reg = read_data;
    end else begin
        ins_reg = ins_reg_temp;
    end
end

//Program Counter MUX + incrementer
always_comb begin
    if(reset_pc)begin
        next_pc = 9'd0;
    end else begin
        next_pc = pc_out + 9'd1;
    end
end
//Program Counter dff
assign pc_temp = pc_out;
always_ff @(posedge clk) begin
    if(load_pc) begin
        pc_out = next_pc;
    end else begin
        pc_out = pc_temp;
    end
end

//Address Selecter MUX
always_comb begin
    if(addr_sel)begin
        mem_addr = pc_out;
    end else begin
        mem_addr = data_address_out;
    end
end
//Data Address dff
assign data_address_temp = data_address_out;
always_ff @(posedge clk) begin
    if(load_addr) begin
        data_address_out = datapath_out[8:0];
    end else begin
        data_address_out = data_address_temp;
    end
end

//Decoder
always_comb begin
    //to ctrler fsm
    opcode  = ins_reg[15:13];
    op      = ins_reg[12:11];
    //to datapath
    ALUop   = ins_reg[12:11];
    imm5    = ins_reg[4:0];
    imm8    = ins_reg[7:0];
    shift   = ins_reg[4:3];
    Rn      = ins_reg[10:8];
    Rd      = ins_reg[7:5];
    Rm      = ins_reg[2:0];

    //nsel MUX
    case(nsel)
        2'd0: begin
            readnum = Rn;
            writenum = Rn;
        end
        2'd1: begin
            readnum = Rd;
            writenum = Rd;
        end
        2'd2: begin
            readnum = Rm;
            writenum = Rm;
        end
        default begin
            readnum = 2'bxx;
            writenum = 2'bxx;
        end
    endcase

    // sign extending imm5 and imm8 based on their msb
    if(imm5[4] == 1'b1) begin
        sximm5 = {{11{1'b1}},imm5};
    end else begin
        sximm5 = {{11{1'b0}},imm5};
    end

    if(imm8[7] == 1'b1) begin
        sximm8 = {{8{1'b1}},imm8};
    end else begin
        sximm8 = {{8{1'b0}},imm8};
    end
end

//fsm states
always_ff @(posedge clk) begin
    if(reset)begin
        state <= `RST;
    end else begin
        case(state)
            `RST: state <= `IF1;

            `IF1: state <= `IF2;

            `IF2: state <= `UpdatePC;

            `UpdatePC: state <= `DECODE;

            `DECODE: begin
                case({opcode,op})
                    `MOVi: state <= `WRITE;
                    `MOVr: state <= `LoadB;
                    `ADD:  state <= `LoadA;
                    `CMP:  state <= `LoadA;
                    `AND:  state <= `LoadA;
                    `MVN:  state <= `LoadB;
                    `LDR:  state <= `LoadA;
                    `STR:  state <= `LoadA;
                    `HALT: state <= `DECODE;
                    default: state <= 3'bxxx;
                endcase
            end

            `LoadA: begin
                if({opcode,op} == `LDR)begin
                    state <= `LoadC;
                end else if ({opcode,op} == `STR) begin
                    state <= `LoadC;
                end else begin
                    state <= `LoadB;
                end
            end

            `LoadB: state <= `LoadC;
            
            `LoadC: begin
                case({opcode,op})
                    `MOVr: state <= `WRITE; 
                    `ADD:  state <= `WRITE;
                    `CMP:  state <= `IF1;
                    `AND:  state <= `WRITE;
                    `MVN:  state <= `WRITE;
                    `LDR:  state <= `MEMldr;
                    `STR:  state <= `MEMstr;
                    default: state <= 3'bxxx;
                endcase
            end
            
            `MEMldr: state <= `MEMbuff;

            `MEMstr: state <= `MEMbuff;

            `MEMbuff: begin
                if({opcode,op} == `LDR)begin
                    state <= `WRITE;
                end else begin
                    state <= `IF1;
                end
            end

            `WRITE: state <= `IF1;
            
        default: state <= 3'bxxx;
        endcase
    end
end

reg [17:0] fsm_out;
assign {reset_pc,addr_sel,load_ir,load_pc,load_addr,mem_cmd, write, loada, loadb, loadc, loads, asel, bsel, nsel, vsel} = fsm_out;
/*  ASSIGNMENT CONVENTION FOR FSM OUTPUT TO DATAPATH
    PURPOSE: avoides inferred latches
    REFERENCE: Google Sheets: FSM output assignment
    [TOGGLE AS NEEDED]
bit 
pos
17  - reset_pc
16  - addr_sel
15  - load_ir
14  - load_pc
13  - load_addr
12  - mem_cmd[1]
11  - mem_cmd[0]
10  - write
9   - loada
8   - loadb
7   - loadc
6   - loads
5   - asel
4   - bsel
3   - nsel[1]
2   - nsel[0]
1   - vsel[1]
0   - vsel[0]
*/

//fsm outputs
always_comb begin
    case(state)
        `RST: fsm_out = 18'b100100000000000000;
        `IF1: fsm_out = 18'b010000100000000000;
        `IF2: fsm_out = 18'b011000100000000000;
        `UpdatePC: fsm_out = 18'b000100000000000000;

        `LoadA: fsm_out = 18'b000000001000000000;

        `LoadB: fsm_out = 18'b000000000100001000;

        `LoadC: begin
            case({opcode,op})
                `MOVr: fsm_out = 18'b000000000010100000;
                `ADD:  fsm_out = 18'b000000000010000000;
                `CMP:  fsm_out = 18'b000000000001000000;
                `AND:  fsm_out = 18'b000000000010000000;
                `MVN:  fsm_out = 18'b000000000010000000;
                `LDR:  fsm_out = 18'b000000000010010000;
                `STR:  fsm_out = 18'b000000000110010100;
                default: fsm_out = 18'bxxxxxxxxxxxxxxxxxx;
            endcase
        end

        `MEMldr: fsm_out = 18'b000010100000000000;

        `MEMstr: fsm_out = 18'b000010000010100000;

        `MEMbuff: begin
            if({opcode,op} == `LDR) begin
                fsm_out = 18'b000000100000000000;
            end else begin
                fsm_out = 18'b000001000000000000;
            end
        end

        `WRITE: begin
            case({opcode,op})
                `MOVi: fsm_out = 18'b000000010000000010;
                `MOVr: fsm_out = 18'b000000010000000100;
                `ADD:  fsm_out = 18'b000000010000000100;
                `AND:  fsm_out = 18'b000000010000000100;
                `MVN:  fsm_out = 18'b000000010000000100;
                `LDR:  fsm_out = 18'b000000110000000111;
                default: fsm_out = 18'bxxxxxxxxxxxxxxxxxx;
            endcase
        end

    default: fsm_out = 18'bxxxxxxxxxxxxxxxxxx;
    endcase
end

reg shift_ctrl;
//shift control for neg imm5 for str and ldr
always_comb begin
    if({opcode,op} == `STR || {opcode,op} == `LDR)
        shift_ctrl = 1;
    else 
        shift_ctrl = 0;
end

// Datapath instantiation
datapath DP(
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

endmodule