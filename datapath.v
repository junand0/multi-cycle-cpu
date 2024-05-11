`include "opcodes.v"

module datapath #(
    parameter WORD_SIZE = 16
)
(
    input clk,
    input reset_n,

    input PCWriteCond,
    input PCWrite,
    input IorD,
    input MemRead,
    input MemWrite,
    input [1:0] MemtoReg,
    input IRWrite,
    input [1:0] PCSource,
    input [3:0] ALUOp,
    input ALUSrcA,
    input [1:0] ALUSrcB,
    input RegWrite,
    input [1:0] RegDst,
    input isHalted,
    input isWWD,
    input done,

    output [3:0] opcode,
    output [5:0] func_code,
    output [15:0] ALUResult,

    input inputReady,
    inout [`WORD_SIZE-1:0] data,
    output [`WORD_SIZE-1:0] address,
    output [`WORD_SIZE-1:0] num_inst, // the number of insts done
    output [`WORD_SIZE-1:0] output_port
);

ALU ALU (
    .A(ALUinA),
    .B(ALUinB),
    .OP(ALUOp),
    .C(ALUResult)
);
wire [15:0] ALUinA;
wire [15:0] ALUinB;

RF RF(
    .write(RegWrite), // write enable
    .clk(clk), // clock
    .reset_n(reset_n), // reset
    .addr1(read_rf_Addr1), // read addr
    .addr2(read_rf_Addr2), // read addr
    .addr3(write_rf_Addr), // write addr
    .data1(read_rf_data1), // addr1에서 read한 data
    .data2(read_rf_data2), // addr2에서 read한 data
    .data3(write_rf_data) // addr3에 write할 data
);

wire RegWrite; // write enable
wire [1:0] read_rf_Addr1; // read addr
assign read_rf_Addr1 = rs;
wire [1:0] read_rf_Addr2; // read addr
assign read_rf_Addr2 = rt;
wire [1:0] write_rf_Addr; // write addr
wire [15:0] read_rf_data1; // addr1에서 read한 data
wire [15:0] read_rf_data2; // addr2에서 read한 data
wire [15:0] write_rf_data; // addr3에 write할 data

// to allocate value in always block
reg [1:0] write_rf_Addr_reg;
assign write_rf_Addr = write_rf_Addr_reg;
reg [15:0] write_rf_data_reg;
assign write_rf_data = write_rf_data_reg;

reg [`WORD_SIZE-1:0] num_inst_reg;
assign num_inst = num_inst_reg;
reg [`WORD_SIZE-1:0] address_reg;
assign address = address_reg;

// latch registers
reg [`WORD_SIZE-1:0] reg_A; // latch register to store value read from RF
reg [`WORD_SIZE-1:0] reg_B;
reg [`WORD_SIZE-1:0] ALUout; // latch register to store ALUresult
reg [`WORD_SIZE-1:0] MDR; // latch register to store MEM data
reg [`WORD_SIZE-1:0] IR; // latch register to store inst
reg [`WORD_SIZE-1:0] pc; // latch register to store pc

always @(*) begin
    if(reset_n && inputReady) begin
        MDR = data; // update MDR latch
    end
end

always @(posedge clk) begin
    if(reset_n) begin
        ALUout = ALUResult; // update ALUout latch
    end
end

always @(posedge clk) begin
    if(reset_n) begin
        reg_A = read_rf_data1; // update RF read data latch
        reg_B = read_rf_data2;
    end
end

wire [`WORD_SIZE-1:0] pc_selected; // selected value for pc
wire branch_true; // branch resolution result
assign branch_true = PCWrite || (PCWriteCond && ALUResult) ? 1 : 0;
assign pc_selected = (PCSource == 2'b00) ? ALUResult : 
                    (PCSource == 2'b01) ? ALUout : 
                    (PCSource == 2'b10) ? {pc[`WORD_SIZE-1:`WORD_SIZE-4], jmp_target} :
                    (PCSource == 2'b11) ? read_rf_data1 : pc;

always @(posedge clk) begin
    if(reset_n) begin
        if(branch_true) begin
            pc = pc_selected; // update pc latch
        end
    end
end

always @(*) begin
    if(reset_n) begin
        case(IorD) // inst pc or MEM address
            1'b0 : begin
                address_reg = pc;
            end
            1'b1 : begin
                address_reg = ALUout;
            end
        endcase
    end
end

assign data = MemWrite ? reg_B : `WORD_SIZE'bz; // input data to write into MEM

always @(*) begin
    if(reset_n) begin
        case(MemtoReg) // select data to write into RF
            2'b00 : begin
                write_rf_data_reg = ALUout;
            end
            2'b01 : begin
                write_rf_data_reg = MDR;
            end
            2'b10 : begin
                write_rf_data_reg = pc;
            end
        endcase
    end
end

always @(posedge inputReady) begin
    if(reset_n) begin
        if(IRWrite) begin
            IR = data; // update IR latch
        end
    end
end

// parsing inst (decode)
wire [1:0] rs;
wire [1:0] rt;
wire [1:0] rd;
wire [7:0] imm;
wire [11:0] jmp_target;

assign opcode = IR[`WORD_SIZE-1:`WORD_SIZE-4];
assign rs = IR[`WORD_SIZE-5:`WORD_SIZE-6];
assign rt = IR[`WORD_SIZE-7:`WORD_SIZE-8];
assign rd = IR[`WORD_SIZE-9:`WORD_SIZE-10];
assign func_code = IR[`WORD_SIZE-11:0];
assign imm = IR[7:0];
assign jmp_target = IR[`WORD_SIZE-5:0];

always @(*) begin
    if(reset_n) begin
        case(RegDst) // select address to write into RF
            2'b00 : begin
                write_rf_Addr_reg = rt;
            end
            2'b01 : begin
                write_rf_Addr_reg = rd;
            end
            2'b10 : begin
                write_rf_Addr_reg = 16'd2;
            end
        endcase
    end
end

// to allocate in always block
reg [`WORD_SIZE-1:0] ALUinA_reg;
assign ALUinA = ALUinA_reg;
reg [`WORD_SIZE-1:0] ALUinB_reg;
assign ALUinB = ALUinB_reg;

always @(*) begin
    if(reset_n) begin
        case(ALUSrcA) // select input of ALU
            1'b0 : begin
                ALUinA_reg = pc;
            end
            1'b1 : begin
                ALUinA_reg = reg_A;
            end
        endcase
        case(ALUSrcB)
            2'b00 : begin
                ALUinB_reg = reg_B;
            end
            2'b01 : begin
                ALUinB_reg = 16'd1;
            end
            2'b10 : begin
                ALUinB_reg = {{8{imm[7]}}, imm};
            end
        endcase
    end
end

// to allocate value in always block
reg [`WORD_SIZE-1:0] output_port_reg;
assign output_port = output_port_reg;

always @(posedge isWWD) begin
    output_port_reg = read_rf_data1;
end

always @(*) begin
    if(!reset_n) begin // reset
        address_reg <= 0;
        num_inst_reg <= 0;
        pc <= 0;
        IR <= 0;
        MDR <= 0;
        reg_A <= 0;
        reg_B <= 0;
        ALUout <= 0;
    end
end

always @(posedge done) begin
    if(reset_n) begin
        if(done) begin
            num_inst_reg <= num_inst + 1; // increment the number of completed inst when done signal is on
        end
    end
end
endmodule