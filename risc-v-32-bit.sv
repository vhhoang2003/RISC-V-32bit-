//=============================
// Program Counter
//=============================
module Program_Counter(
    input  logic        clk,
    input  logic        reset,
    input  logic [31:0] PC_in,
    output logic [31:0] PC_out
);
    always_ff @(posedge clk or posedge reset) begin
        if (reset) PC_out <= 32'b0;
        else       PC_out <= PC_in;
    end
endmodule

//=============================
// PC + 4
//=============================
module PC_plus4(
    input  logic [31:0] fromPC,
    output logic [31:0] NextPC
);
    assign NextPC = fromPC + 32'd4;
endmodule

//=============================
// Instruction Memory (64 words)
//=============================
module Instruction_Memory(
    input  logic        clk,
    input  logic        reset,
    input  logic [31:0] read_address,   // byte address
    output logic [31:0] instruction_out
);
    logic [31:0] I_Mem [0:63];
    integer i;

    // Optional: synchronous write not needed; only reset contents
    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            for (i = 0; i < 64; i = i + 1) begin
                I_Mem[i] <= 32'b0;
            end
        end
    end

    // Combinational read by word index
    assign instruction_out = I_Mem[read_address[31:2]];
endmodule

//=============================
// Register File (x0..x31)
//=============================
module Reg_File(
    input  logic        clk,
    input  logic        reset,
    input  logic        RegWrite,
    input  logic [4:0]  Rs1,
    input  logic [4:0]  Rs2,
    input  logic [4:0]  Rd,
    input  logic [31:0] Write_data,
    output logic [31:0] read_data1,
    output logic [31:0] read_data2
);
    logic [31:0] regfile [0:31];
    integer i;

    // Write port (x0 luôn 0)
    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            for (i = 0; i < 32; i = i + 1) regfile[i] <= 32'b0;
        end else if (RegWrite && (Rd != 5'd0)) begin
            regfile[Rd] <= Write_data;
        end
    end

    // Read ports (combinational)
    assign read_data1 = (Rs1 == 5'd0) ? 32'b0 : regfile[Rs1];
    assign read_data2 = (Rs2 == 5'd0) ? 32'b0 : regfile[Rs2];
endmodule

//=============================
// Immediate Generator (I / S / B)
//=============================
module ImmGen(
    input  logic [31:0] instruction,
    output logic [31:0] ImmExt
);
    logic [6:0] opcode;
    assign opcode = instruction[6:0];

	always_comb begin
		unique case (opcode)
			// I-type (load, ALU-I)
			7'b0000011, 7'b0010011:
				ImmExt = {{20{instruction[31]}}, instruction[31:20]};
			// S-type
			7'b0100011:
				ImmExt = {{20{instruction[31]}}, instruction[31:25], instruction[11:7]};
			// B-type
			7'b1100011:
				ImmExt = {{19{instruction[31]}},
						  instruction[31], instruction[7],
						  instruction[30:25], instruction[11:8],
						  1'b0};
			// J-type (JAL) – 21 bit immediate, LSB=0
			7'b1101111:
				ImmExt = {{11{instruction[31]}},
						  instruction[31],
						  instruction[19:12],
						  instruction[20],
						  instruction[30:21],
						  1'b0};
			// I-type JALR (opcode=1100111) dùng I-imm, đã cover ở trên nếu muốn gộp
			// U-type LUI/AUIPC
			7'b0110111, 7'b0010111:
				ImmExt = {instruction[31:12], 12'b0};
			default:
				ImmExt = 32'b0;
		endcase
	end

endmodule

//=============================
// Control Unit
//=============================
module Control_Unit(
    input  logic [6:0]  opcode,
    output logic        Branch,
    output logic        MemRead,
    output logic        MemtoReg,
    output logic [1:0]  ALUOp,
    output logic        MemWrite,
    output logic        ALUSrc,
    output logic        RegWrite,     // << thêm dấu phẩy ở đây
    // mới
    output logic [1:0]  PCSel,        // 00: normal/branch, 01: JAL, 10: JALR
    output logic [1:0]  ResultSrc,    // 00: ALU, 01: Mem, 10: PC+4
    output logic        ALUSrcA       // 0: A=Rs1, 1: A=PC  (AUIPC)
);
    always_comb begin
        // defaults
        Branch    = 1'b0;
        MemRead   = 1'b0;
        MemtoReg  = 1'b0;
        ALUOp     = 2'b00;
        MemWrite  = 1'b0;
        ALUSrc    = 1'b0;
        RegWrite  = 1'b0;
        PCSel     = 2'b00;
        ResultSrc = 2'b00;
        ALUSrcA   = 1'b0;

        unique case (opcode)
            // R-type
            7'b0110011: begin
                ALUSrc=0; RegWrite=1; ALUOp=2'b10;
                ResultSrc=2'b00; PCSel=2'b00;
            end
            // Load (LW)
            7'b0000011: begin
                ALUSrc=1; RegWrite=1; MemRead=1; ALUOp=2'b00;
                MemtoReg=1; ResultSrc=2'b01; PCSel=2'b00;   // WB = Mem
            end
            // Store (SW)
            7'b0100011: begin
                ALUSrc=1; MemWrite=1; ALUOp=2'b00; PCSel=2'b00;
            end
            // Branch (BEQ/BNE)
            7'b1100011: begin
                ALUSrc=0; Branch=1; ALUOp=2'b01; PCSel=2'b00;
            end
            // ALU‑I (ADDI, …)
            7'b0010011: begin
                ALUSrc=1; RegWrite=1; ALUOp=2'b10;
                ResultSrc=2'b00; PCSel=2'b00;
            end
            // JAL
            7'b1101111: begin
                RegWrite=1; ResultSrc=2'b10;      // rd = PC+4
                PCSel=2'b01; ALUOp=2'b00;         // PC <- PC + imm
            end
            // JALR
            7'b1100111: begin
                ALUSrc=1; RegWrite=1; ResultSrc=2'b10; // rd = PC+4
                PCSel=2'b10; ALUOp=2'b00;              // PC <- (Rs1+imm)&~1
            end
            // LUI
            7'b0110111: begin
                ALUSrc=1; RegWrite=1; ALUOp=2'b11;     // ALU out = imm (pass B)
                ResultSrc=2'b00; PCSel=2'b00;
            end
            // AUIPC
            7'b0010111: begin
                ALUSrc=1; ALUSrcA=1; RegWrite=1; ALUOp=2'b00; // A=PC, B=imm, ADD
                ResultSrc=2'b00; PCSel=2'b00;
            end
            default: ;
        endcase
    end
endmodule

//=============================
// ALU
//=============================
module ALU_unit(
    input  logic [31:0] A,
    input  logic [31:0] B,
    input  logic [3:0]  Control_in,
    output logic [31:0] ALU_result,
    output logic        zero
);
    always_comb begin
        unique case (Control_in)
            4'b0000: ALU_result = A & B;   // AND
            4'b0001: ALU_result = A | B;   // OR
            4'b0010: ALU_result = A + B;   // ADD
            4'b0110: ALU_result = A - B;   // SUB
			4'b0111: ALU_result = B; 	   // LUI
            default: ALU_result = 32'b0;
        endcase
    end
    assign zero = (ALU_result == 32'b0);
endmodule

//=============================
// ALU Control
//=============================
module ALU_Control(
    input  logic [1:0]  ALUOp,
    input  logic        funct7,      // instruction[30]
    input  logic [2:0]  funct3,      // instruction[14:12]
	input  logic        is_rtype, 
    output logic [3:0]  Control_out
);
    always_comb begin
        unique case (ALUOp)
            2'b00: Control_out = 4'b0010;   // ADD (load/store)
            2'b01: Control_out = 4'b0110;   // SUB (branch compare)
            2'b10: begin                    // R-type & I-type
					unique case (funct3)
						3'b000: Control_out = (is_rtype && funct7) ? 4'b0110  // SUB (R)
                                                               : 4'b0010; // ADD / ADDI
						3'b111: Control_out = 4'b0000; // AND
						3'b110: Control_out = 4'b0001; // OR
						default: Control_out = 4'b0010; // mặc định ADD
					endcase
				end
			2'b11: Control_out = 4'b0111;
            default: Control_out = 4'b0010;
        endcase
    end
endmodule

//=============================
// Data Memory (64 words)
//=============================
module Data_Memory(
    input  logic        clk,
    input  logic        reset,
    input  logic        MemWrite,
    input  logic        MemRead,
    input  logic [31:0] read_address,  // byte address
    input  logic [31:0] Write_data,
    output logic [31:0] MemData_out
);
    logic [31:0] D_Memory [0:63];
    integer i;

    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            for (i = 0; i < 64; i = i + 1) D_Memory[i] <= 32'b0;
        end else if (MemWrite) begin
            D_Memory[read_address[31:2]] <= Write_data;
        end
    end

    assign MemData_out = (MemRead) ? D_Memory[read_address[31:2]] : 32'b0;
endmodule

//=============================
// Simple 2:1 Mux (parametric width)
//=============================
module Mux2to1 #(
    parameter W = 32
)(
    input  logic        sel,
    input  logic [W-1:0] A,
    input  logic [W-1:0] B,
    output logic [W-1:0] Y
);
    assign Y = sel ? B : A; // sel==1 -> B
endmodule

//=============================
// Simple 3:1 Mux (parametric width)
//=============================
module Mux3to1 #(
    parameter W = 32
)(
    input  logic [1:0]   sel,   // 00:A, 01:B, 10:C
    input  logic [W-1:0] A,
    input  logic [W-1:0] B,
    input  logic [W-1:0] C,
    output logic [W-1:0] Y
);
    always_comb begin
        unique case (sel)
            2'b00: Y = A;
            2'b01: Y = B;
            2'b10: Y = C;
            default: Y = A;
        endcase
    end
endmodule

//=============================
// Adder (generic)
//=============================
module Adder(
    input  logic [31:0] in_1,
    input  logic [31:0] in_2,
    output logic [31:0] Sum_out
);
    assign Sum_out = in_1 + in_2;
endmodule

//=============================
// Branch comparator (BEQ/BNE)
//=============================
module Branch_Unit(
    input  logic        branch_en,        // Branch từ Control_Unit
    input  logic [2:0]  funct3,           // instruction[14:12]
    input  logic        zero,             // từ ALU (A-B==0)
    output logic        take_branch
);
    always_comb begin
        if (!branch_en) begin
            take_branch = 1'b0;
        end else begin
            unique case (funct3)
                3'b000: take_branch =  zero; // BEQ
                3'b001: take_branch = ~zero; // BNE
                default: take_branch = 1'b0; // các branch khác: chưa hỗ trợ
            endcase
        end
    end
endmodule

//=============================
// Top-level Datapath (single-cycle)
//=============================
module top(
    input logic clk,
    input logic reset_n
);
    // Wires
    logic [31:0] PC_top, instruction_top;
    logic [31:0] Rd1_top, Rd2_top, ImmExt_top;
    logic [31:0] ALU_B_mux_out;
    logic [31:0] branch_target, NextPC_top, PCin_top;
    logic [31:0] ALU_result_top, MemData_top, WriteBack_top;

    logic        RegWrite_top, ALUSrc_top, zero_top, Branch_top;
    logic        MemtoReg_top, MemWrite_top, MemRead_top;
    logic [1:0]  ALUOp_top;
    logic [3:0]  ALUcontrol_top;

	logic [1:0]  PCSel_top;        // từ Control_Unit
    logic [1:0]  ResultSrc_top;    // từ Control_Unit
    logic        ALUSrcA_top;      // từ Control_Unit
    logic        take_branch;
	
	logic [31:0] ALU_A_mux_out;
    logic [2:0]  funct3_top;
    logic [31:0] jalr_target_masked;
	
	
	assign funct3_top = instruction_top[14:12];
	
    // Program Counter
    Program_Counter uPC(
        .clk(clk), .reset(~reset_n), .PC_in(PCin_top), .PC_out(PC_top)
    );

    // PC + 4
    PC_plus4 uPC4(
        .fromPC(PC_top), .NextPC(NextPC_top)
    );

    // Instruction Memory
    Instruction_Memory uIMem(
        .clk(clk), .reset(~reset_n),
        .read_address(PC_top),
        .instruction_out(instruction_top)
    );

    // Control Unit
    Control_Unit uCtrl(
        .opcode(instruction_top[6:0]),
        .Branch(Branch_top), .MemRead(MemRead_top), .MemtoReg(MemtoReg_top),
        .ALUOp(ALUOp_top), .MemWrite(MemWrite_top),
        .ALUSrc(ALUSrc_top), .RegWrite(RegWrite_top),
        .PCSel(PCSel_top), .ResultSrc(ResultSrc_top), .ALUSrcA(ALUSrcA_top)
    );

    // Register File
    Reg_File uRF(
        .clk(clk), .reset(~reset_n), .RegWrite(RegWrite_top),
        .Rs1(instruction_top[19:15]),
        .Rs2(instruction_top[24:20]),
        .Rd (instruction_top[11:7]),
        .Write_data(WriteBack_top),
        .read_data1(Rd1_top), .read_data2(Rd2_top)
    );

    // ImmGen
    ImmGen uImm(
        .instruction(instruction_top),
        .ImmExt(ImmExt_top)
    );

    // ALU Control
    ALU_Control uALUC(
        .ALUOp(ALUOp_top),
        .funct7(instruction_top[30]),
        .funct3(instruction_top[14:12]),
		.is_rtype(instruction_top[6:0] == 7'b0110011),
        .Control_out(ALUcontrol_top)
    );
	
	// === A-input mux (A = Rs1 hoặc PC) ===
    Mux2to1 #(32) uALUAmux(
		.sel(ALUSrcA_top), 
		.A(Rd1_top), 
		.B(PC_top), 
		.Y(ALU_A_mux_out)
	);

    // ALU B-input mux (Reg vs Imm)
	Mux2to1 #(32) uALUBmux(
		.sel(ALUSrc_top), 
		.A(Rd2_top), 
		.B(ImmExt_top), 
		.Y(ALU_B_mux_out)
	);

    // ALU
    ALU_unit uALU(
        .A(ALU_A_mux_out), .B(ALU_B_mux_out), .Control_in(ALUcontrol_top),
        .ALU_result(ALU_result_top), .zero(zero_top)
    );


    // Branch target = PC + Imm (Imm đã có bit0=0 cho B-type)
    Adder uBranchAdd(
        .in_1(PC_top), .in_2(ImmExt_top), .Sum_out(branch_target)
    );

    // Branch decision
    Branch_Unit uBranch(
        .branch_en(Branch_top), .funct3(funct3_top), .zero(zero_top),
        .take_branch(take_branch)
    );

	// JALR target = (Rs1 + Imm) & ~1
    assign jalr_target_masked = (ALU_result_top & 32'hFFFF_FFFE);
	
    // Next PC mux
    always_comb begin
        unique case (PCSel_top)
            2'b00: PCin_top = (take_branch) ? branch_target : NextPC_top; // normal/branch
            2'b01: PCin_top = branch_target;                               // JAL (PC + imm)
            2'b10: PCin_top = jalr_target_masked;                          // JALR
            default: PCin_top = NextPC_top;
        endcase
    end

    // Data Memory
    Data_Memory uDMem(
        .clk(clk), .reset(~reset_n),
        .MemWrite(MemWrite_top), .MemRead(MemRead_top),
        .read_address(ALU_result_top),
        .Write_data(Rd2_top),
        .MemData_out(MemData_top)
    );

    // Writeback mux
    Mux3to1 #(32) uWBmux3(
        .sel(ResultSrc_top), .A(ALU_result_top), .B(MemData_top), .C(NextPC_top),
        .Y(WriteBack_top)
    );

endmodule
