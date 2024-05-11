`include "opcodes.v"

module control (
    input reset_n,
    input [3:0] opcode,
    input [5:0] func_code,
    input clk,
    input [15:0] ALUResult,
    input inputReady,

    output PCWriteCond, // when branch inst ex) BNE, BEQ, BLZ, BGZ
    output PCWrite, // write enable of pc latch register
    output IorD, // mux signal to select pc or ALUout into MEM addr
    output MemRead, // read enable of MEM
    output MemWrite, // write enable of MEM
    output [1:0] MemtoReg, // selec signal MDR value or ALUout or 2(for JAL)
    output IRWrite, // write enable of IR latch register
    output [1:0] PCSource, // select signal ALUresult or ALUout register or jump target
    output [3:0] ALUOp, // control signal of ALU operation
    output ALUSrcA, // mux signal to select ALU input
    output [1:0] ALUSrcB, // mux signal to select ALU input
    output RegWrite, // write enable of RF
    output [1:0] RegDst, // mux signal to select write register addr of RF
    output isHalted,
    output isWWD,
    output done // to verify whether inst completes or not
);
    // to allocate values in always statement
    reg PCWriteCond_reg;
    assign PCWriteCond = PCWriteCond_reg;
    reg PCWrite_reg;
    assign PCWrite = PCWrite_reg;
    reg IorD_reg;
    assign IorD = IorD_reg;
    reg MemRead_reg;
    assign MemRead = MemRead_reg;
    reg MemWrite_reg;
    assign MemWrite = MemWrite_reg;
    reg [1:0] MemtoReg_reg;
    assign MemtoReg = MemtoReg_reg;
    reg IRWrite_reg;
    assign IRWrite = IRWrite_reg;
    reg [1:0] PCSource_reg;
    assign PCSource = PCSource_reg;
    reg [3:0] ALUOp_reg;
    assign ALUOp = ALUOp_reg;
    reg ALUSrcA_reg;
    assign ALUSrcA = ALUSrcA_reg;
    reg [1:0] ALUSrcB_reg;
    assign ALUSrcB = ALUSrcB_reg;
    reg RegWrite_reg;
    assign RegWrite = RegWrite_reg;
    reg [1:0] RegDst_reg;
    assign RegDst = RegDst_reg;
    reg isHalted_reg;
    assign isHalted = isHalted_reg;
    reg isWWD_reg;
    assign isWWD = isWWD_reg;

    reg done_reg;
    assign done = done_reg;

    reg [2:0] nstate; // next state
    reg [2:0] state; // current state

wire Is_Itype; // is inst I-type?
assign Is_Itype = (0 <= opcode) && (opcode <= 4'b1000);
wire Is_Jtype; // is inst J-type?
assign Is_Jtype = (4'b1001 <= opcode) && (opcode <= 4'b1010);

always @(*) begin
    if(!reset_n) begin // reset
        state <= 0;
        nstate <= 0;
        MemRead_reg <= 0;
        MemWrite_reg <= 0;
    end
end

always @(posedge clk) begin
    if(reset_n) begin // state tansition
        state <= nstate;
    end
end

always @(posedge clk) begin
if (reset_n) begin
    case(nstate)
        3'b000 : begin // IF stage
            PCWriteCond_reg <= 0;
            PCWrite_reg <= 1;
            IorD_reg <= 0;
            MemRead_reg <= 1;
            MemWrite_reg <= 0;
            IRWrite_reg <= 1;
            PCSource_reg <= 0;
            ALUOp_reg <= 0;
            ALUSrcA_reg <= 0;
            ALUSrcB_reg <= 2'b01;
            RegWrite_reg <= 0;
            isHalted_reg <= 0;
            isWWD_reg <= 0;
            nstate <= 3'b001; // go to ID stage
            done_reg <= 0;
        end
        3'b001 : begin // ID stage
            PCWriteCond_reg <= 0;
            MemRead_reg <= 0;
            MemWrite_reg <= 0;
            IRWrite_reg <= 0;
            if(opcode == `OPCODE_Rtype) begin // control signal for Rtype insts in ID stage
                if(func_code >= 0 && func_code <= 3'b111) begin // ADD, SUB, ... SHR in ID stage
                    PCWrite_reg <= 0;
                    ALUOp_reg <= func_code;
                    RegWrite_reg <= 0;
                    RegDst_reg <= 2'b01;
                    isHalted_reg <= 0;
                    isWWD_reg <= 0;
                    nstate <= 3'b010; // go to EX stage
                end
                else if(func_code == `FUNC_JRL) begin // JRL in ID stage
                    PCWrite_reg <= 1;
                    MemtoReg_reg <= 2'b10;
                    PCSource_reg <= 2'b11;
                    RegWrite_reg <= 1;
                    RegDst_reg <= 2'b10;
                    isHalted_reg <= 0;
                    isWWD_reg <= 0;
                    nstate <= 3'b000; // back to IF stage
                    done_reg <= 1; // inst completes
                end
                else if(func_code == `FUNC_JPR) begin // JPR in ID stage
                    PCWrite_reg <= 1;
                    PCSource_reg <= 2'b11;
                    RegWrite_reg <= 0;
                    isHalted_reg <= 0;
                    isWWD_reg <= 0;
                    nstate <= 3'b000; // back to IF
                    done_reg <= 1; // inst completes
                end
                else if(func_code == `FUNC_HLT) begin // HLT in ID stage
                    PCWrite_reg <= 0;
                    RegWrite_reg <= 0;
                    isHalted_reg <= 1;
                    isWWD_reg <= 0;
                    nstate <= 3'b000; // back to IF
                    done_reg <= 1; // inst completes
                end
                else if(func_code == `FUNC_WWD) begin // WWD in ID stage
                    PCWrite_reg <= 0;
                    RegWrite_reg <= 0;
                    isHalted_reg <= 0;
                    isWWD_reg <= 1;
                    nstate <= 3'b000; // back to IF
                    done_reg <= 1; // inst completes
                end
            end
            else if(Is_Itype) begin // control signals for Itype in ID stage
                isHalted_reg <= 0;
                isWWD_reg <= 0;
                if(opcode >= 0 && opcode <= 4'b0011) begin // BNE, BEQ, BGZ, BLZ
                    PCWrite_reg <= 0;
                    ALUOp_reg <= 0; // ADD for caculating target branch
                    ALUSrcA_reg <= 0; // pc+1
                    ALUSrcB_reg <= 2'b10; // signed imm
                    RegWrite_reg <= 0;
                    nstate <= 3'b010; // go to EX 
                end
                else if(opcode == `OPCODE_ADI || 
                        opcode == `OPCODE_ORI || 
                        opcode == `OPCODE_LHI || 
                        opcode == `OPCODE_SWD || 
                        opcode == `OPCODE_LWD) begin // ADI, ORI, LHI, SWD, LWD
                    PCWrite_reg <= 0;
                    RegWrite_reg <= 0;
                    RegDst_reg <= 0;
                    nstate <= 3'b010; // go to EX
                end
            end
            else if(Is_Jtype) begin // Jtype in ID stage
                isHalted_reg <= 0;
                isWWD_reg <= 0;
                case(opcode)
                    `OPCODE_JMP : begin // JMP in ID stage
                        PCWrite_reg <= 1;
                        PCSource_reg <= 2'b10;
                        RegWrite_reg <= 0;
                        nstate <= 3'b000; // back to IF
                        done_reg <= 1; // inst completes
                    end
                    `OPCODE_JAL : begin // JAL in ID stage
                        PCWrite_reg <= 1;
                        MemtoReg_reg <= 2'b10;
                        RegWrite_reg <= 1;
                        PCSource_reg <= 2'b10;
                        RegDst_reg <= 2'b10;
                        nstate <= 3'b000; // back to IF
                        done_reg <= 1; // inst completes
                    end
                endcase
            end
        end
        3'b010 : begin // EX stage
            PCWrite_reg <= 0;
            MemRead_reg <= 0;
            MemWrite_reg <= 0;
            IRWrite_reg <= 0;
            RegWrite_reg <= 0;
            isHalted_reg <= 0;
            isWWD_reg <= 0;
            if(opcode == `OPCODE_Rtype) begin // R type in EX stage
                if(func_code >= 0 && func_code <= 3'b111) begin
                    PCWriteCond_reg <= 0;
                    ALUOp_reg <= func_code;
                    ALUSrcA_reg <= 1;
                    ALUSrcB_reg <= 0;
                    RegDst_reg <= 2'b01;
                    nstate <= 3'b100; // go to WB stage
                end
            end
            else if(Is_Itype) begin
                if(opcode == `OPCODE_BEQ) begin // BEQ in EX stage
                    PCWriteCond_reg <= 1;
                    PCSource_reg <= 2'b01;
                    ALUOp_reg <= 4'b1100; // operation of A == B ?
                    ALUSrcA_reg <= 1;
                    ALUSrcB_reg <= 2'b00;
                    nstate <= 3'b000; // back to IF
                    done_reg <= 1; // inst completes
                end
                else if(opcode == `OPCODE_BNE) begin // BNE in EX stage
                    PCWriteCond_reg <= 1;
                    PCSource_reg <= 2'b01;
                    ALUOp_reg <= 4'b1101; // operation of A != B ?
                    ALUSrcA_reg <= 1;
                    ALUSrcB_reg <= 2'b00;
                    nstate <= 3'b000; // back to IF
                    done_reg <= 1; // inst completes
                end
                else if(opcode == `OPCODE_BGZ) begin // BGZ in EX stage
                    PCWriteCond_reg <= 1;
                    PCSource_reg <= 2'b01;
                    ALUSrcA_reg <= 1;
                    ALUSrcB_reg <= 2'b00;
                    ALUOp_reg <= 4'b1010; // operation of A > 0 ?
                    nstate <= 3'b000; // back to IF
                    done_reg <= 1; // inst completes
                end
                else if(opcode == `OPCODE_BLZ) begin // BLZ in EX stage
                    PCWriteCond_reg <= 1;
                    PCSource_reg <= 2'b01;
                    ALUSrcA_reg <= 1;
                    ALUSrcB_reg <= 2'b00;
                    ALUOp_reg <= 4'b1110; // operation of A < 0 ?
                    nstate <= 3'b000; // back to IF
                    done_reg <= 1; // inst completes
                end
                else if(opcode >= 3'b100 && opcode <= 3'b110) begin // ADI, ORI, LHI in EX stage
                    PCWriteCond_reg <= 0;
                    PCSource_reg <= 2'b01;
                    ALUSrcA_reg <= 1;
                    ALUSrcB_reg <= 2'b10;
                    nstate <= 3'b100; // go to WB stage
                    case(opcode)
                        `OPCODE_ADI : begin
                            ALUOp_reg <= 0; // ADI
                        end
                        `OPCODE_ORI : begin
                            ALUOp_reg <= 4'b1000; // ORI
                        end
                        `OPCODE_LHI : begin
                            ALUOp_reg <= 4'b1001; // LHI
                        end
                    endcase
                end
                else if(opcode == `OPCODE_LWD || opcode == `OPCODE_SWD) begin // LWD, SWD in EX stage
                    PCWriteCond_reg <= 0;
                    ALUOp_reg <= 0; // ADD
                    ALUSrcA_reg <= 1;
                    ALUSrcB_reg <= 2'b10;
                    nstate <= 3'b011; // go to MEM stage
                end
            end
        end
        3'b011 : begin // MEM stage
            PCWriteCond_reg <= 0;
            PCWrite_reg <= 0;
            IorD_reg <= 1;
            IRWrite_reg <= 0;
            RegWrite_reg <= 0;
            isHalted_reg <= 0;
            isWWD_reg <= 0;
            if(opcode == `OPCODE_SWD) begin // SWD in MEM
                MemRead_reg <= 0;
                MemWrite_reg <= 1;
                done_reg <= 1; // inst completes
                nstate <= 0; // back to IF
            end
            else if(opcode == `OPCODE_LWD) begin // LWD in MEM
                MemRead_reg <= 1;
                MemWrite_reg <= 0;
                nstate <= 3'b100; // go to WB stage
            end
        end
        3'b100 : begin // WB stage
            PCWriteCond_reg <= 0;
            PCWrite_reg <= 0;
            MemRead_reg <= 0;
            MemWrite_reg <= 0;
            IRWrite_reg <= 0;
            RegWrite_reg <= 1;
            isHalted_reg <= 0;
            isWWD_reg <= 0;
            done_reg <= 1; // last stage -> inst completes
            nstate <= 0; // back to IF
            if(opcode == `OPCODE_Rtype) begin
                MemtoReg_reg <= 0;
                RegDst_reg <= 2'b01;
            end
            else if(Is_Itype && opcode != `OPCODE_LWD) begin
                MemtoReg_reg <= 0;
                RegDst_reg <= 0;
            end
            else if(opcode == `OPCODE_LWD) begin
                MemtoReg_reg <= 1;
                RegDst_reg <= 0;
            end
        end
    endcase
end
end
endmodule