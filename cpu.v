`timescale 1ns/100ps

`include "opcodes.v"
`include "constants.v"
`define WORD_SIZE 16

module cpu (
    output readM, // read from memory
    output writeM, // write to memory
    output [`WORD_SIZE-1:0] address, // current address for data
    inout [`WORD_SIZE-1:0] data, // data being input or output
    input inputReady, // indicates that data is ready from the input port
    input reset_n, // active-low RESET signal
    input clk, // clock signal
    
    // for debuging/testing purpose
    output [`WORD_SIZE-1:0] num_inst, // number of instruction during execution
    output [`WORD_SIZE-1:0] output_port, // this will be used for a "WWD" instruction
    output is_halted // 1 if the cpu is halted
);
    // ... fill in the rest of the code

control control(
    .reset_n(reset_n),
    .opcode(opcode),
    .func_code(func_code),
    .clk(clk),
    .ALUResult(ALUResult),

    .PCWriteCond(PCWriteCond),
    .PCWrite(PCWrite),
    .IorD(IorD),
    .MemRead(readM),
    .MemWrite(writeM),
    .MemtoReg(MemtoReg),
    .IRWrite(IRWrite),
    .PCSource(PCSource),
    .ALUOp(ALUOp),
    .ALUSrcA(ALUSrcA),
    .ALUSrcB(ALUSrcB),
    .RegWrite(RegWrite),
    .RegDst(RegDst),
    .isHalted(isHalted),
    .isWWD(isWWD),
    .done(done)
);

wire [3:0] opcode;
wire [5:0] func_code;
wire [`WORD_SIZE-1:0] ALUResult;
wire PCWriteCond;
wire PCWrite;
wire IorD;
wire [1:0] MemtoReg;
wire IRWrite;
wire [1:0] PCSource;
wire [3:0] ALUOp;
wire ALUSrcA;
wire [1:0] ALUSrcB;
wire RegWrite;
wire [1:0] RegDst;
wire isHalted;
assign is_halted = isHalted;
wire isWWD;
wire done;

datapath #(.WORD_SIZE (`WORD_SIZE))
datapath (
    .clk(clk),
    .reset_n(reset_n),

    .PCWriteCond(PCWriteCond),
    .PCWrite(PCWrite),
    .IorD(IorD),
    .MemRead(readM),
    .MemWrite(writeM),
    .MemtoReg(MemtoReg),
    .IRWrite(IRWrite),
    .PCSource(PCSource),
    .ALUOp(ALUOp),
    .ALUSrcA(ALUSrcA),
    .ALUSrcB(ALUSrcB),
    .RegWrite(RegWrite),
    .RegDst(RegDst),
    .isHalted(isHalted),
    .isWWD(isWWD),
    .done(done),

    .opcode(opcode),
    .func_code(func_code),
    .ALUResult(ALUResult),

    .inputReady(inputReady),
    .data(data),
    .address(address),
    .num_inst(num_inst),
    .output_port(output_port)
);

endmodule
