//=============================
// Top-level Datapath (single-cycle)
//=============================
module top(
    input logic clk,
    input logic reset_n
);
    // Wires
    logic [31:0] PC_top;            // Program Counter
    logic [31:0] instruction_top;   // Current instruction
    logic [31:0] Rd1_top, Rd2_top;  // Register file outputs
    logic [31:0] ImmExt_top;        // Immediate value extended
    logic [31:0] ALU_B_mux_out;     // ALU B-input mux output
    logic [31:0] branch_target;     // Branch target address
    logic [31:0] PCin_top;          // Input to Program Counter
    logic [31:0] NextPC_top;        // PC + 4 output
    logic [31:0] ALU_result_top;    // ALU result
    logic [31:0] MemData_top;       // Data Memory output
    logic [31:0] WriteBack_top;     // Data to write back to register file

    logic        RegWrite_top;      // Register write enable
    logic        ALUSrc_top;        // ALU B-input select (Reg vs Imm)
    logic        zero_top;          // ALU zero flag
    logic        Branch_top;        // Branch enable
    logic        MemtoReg_top;      // Memory to register select
    logic        MemWrite_top;      // Memory write enable
    logic        MemRead_top;       // Memory read enable
    logic [1:0]  ALUOp_top;         // ALU operation select
    logic [3:0]  ALUcontrol_top;    // ALU control signals

	logic [1:0]  PCSel_top;         // From Control_Unit
    logic [1:0]  ResultSrc_top;     // From Control_Unit
    logic        ALUSrcA_top;       // From Control_Unit
    logic        take_branch;
	
	logic [31:0] ALU_A_mux_out;         // ALU A-input mux output
    logic [2:0]  funct3_top;            // From instruction (funct3 field)
    logic [31:0] jalr_target_masked;    // JALR target address (masked to even)
	
	
	assign funct3_top = instruction_top[14:12];
	
    // Program Counter
    Program_Counter uPC(
        .clk                (clk), 
        .reset_n            (reset_n), 
        .PC_in              (PCin_top), 
        .PC_out             (PC_top)
    );

    // PC + 4
    PC_plus4 uPC4(
        .fromPC             (PC_top), 
        .NextPC             (NextPC_top)
    );

    // Instruction Memory
    Instruction_Memory uIMem(
        .clk                (clk), 
        .reset_n            (reset_n),
        .read_address       (PC_top),
        .instruction_out    (instruction_top)
    );

    // Control Unit
    Control_Unit uCtrl(
        .opcode             (instruction_top[6:0]),
        .Branch             (Branch_top), 
        .MemRead            (MemRead_top), 
        .MemtoReg           (MemtoReg_top),
        .ALUOp              (ALUOp_top), 
        .MemWrite           (MemWrite_top),
        .ALUSrc             (ALUSrc_top), 
        .RegWrite           (RegWrite_top),
        .PCSel              (PCSel_top), 
        .ResultSrc          (ResultSrc_top), 
        .ALUSrcA            (ALUSrcA_top)
    );

    // Register File
    Reg_File uRF(
        .clk                (clk), 
        .reset_n            (reset_n), 
        .RegWrite           (RegWrite_top),
        .Rs1                (instruction_top[19:15]),
        .Rs2                (instruction_top[24:20]),
        .Rd                 (instruction_top[11:7]),
        .Write_data         (WriteBack_top),
        .read_data1         (Rd1_top), 
        .read_data2         (Rd2_top)
    );

    // ImmGen
    ImmGen uImm(
        .instruction        (instruction_top),
        .ImmExt             (ImmExt_top)
    );

    // ALU Control
    ALU_Control uALUC(
        .ALUOp              (ALUOp_top),
        .funct7             (instruction_top[30]),
        .funct3             (instruction_top[14:12]),
		.is_rtype           (instruction_top[6:0] == 7'b0110011),
        .Control_out        (ALUcontrol_top)
    );
	
	// === A-input mux (A = Rs1 or PC) ===
    Mux2to1 #(32) uALUAmux(
		.sel                (ALUSrcA_top), 
		.A                  (Rd1_top), 
		.B                  (PC_top), 
		.Y                  (ALU_A_mux_out)
	);

    // ALU B-input mux (Reg vs Imm)
	Mux2to1 #(32) uALUBmux(
		.sel                (ALUSrc_top), 
		.A                  (Rd2_top), 
		.B                  (ImmExt_top), 
		.Y                  (ALU_B_mux_out)
	);

    // ALU
    ALU_unit uALU(
        .A                  (ALU_A_mux_out), 
        .B                  (ALU_B_mux_out), 
        .Control_in         (ALUcontrol_top),
        .ALU_result         (ALU_result_top), 
        .zero               (zero_top)
    );


    // Branch target = PC + Imm (Imm had bit0=0 cho B-type)
    Adder uBranchAdd(
        .in_1               (PC_top), 
        .in_2               (ImmExt_top), 
        .Sum_out            (branch_target)
    );

    // Branch decision
    Branch_Unit uBranch(
        .branch_en          (Branch_top), 
        .funct3             (funct3_top), 
        .zero               (zero_top),
        .take_branch        (take_branch)
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
        .clk                (clk), 
        .reset_n            (reset_n),
        .MemWrite           (MemWrite_top), 
        .MemRead            (MemRead_top),
        .read_address       (ALU_result_top),
        .Write_data         (Rd2_top),
        .MemData_out        (MemData_top)
    );

    // Writeback mux
    Mux3to1 #(32) uWBmux3(
        .sel                (ResultSrc_top), 
        .A                  (ALU_result_top), 
        .B                  (MemData_top), 
        .C                  (NextPC_top),
        .Y                  (WriteBack_top)
    );

endmodule
