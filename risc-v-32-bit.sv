//==================================================================================//
// Beginning of RTL/RISC-V/risc-v-32-bit.sv                                         //
// This file contains the main components of a simple RISC-V 32-bit processor.      //
// It includes modules for the Program Counter, Instruction Memory, Control Unit,   //
// Register File, Immediate Generator, ALU Control, ALU, Data Memory, and various   //
// multiplexers. The design is intended for educational purposes and may not        //
// include all features of a complete RISC-V implementation.                        //
//==================================================================================//

//=============================
// Program Counter
//=============================
module Program_Counter(
    input  logic        clk,                // Clock signal
    input  logic        reset_n,            // Active-low reset
    input  logic [31:0] PC_in,              // Input for the next PC value
    output logic [31:0] PC_out              // Current PC value
);
    always_ff @(posedge clk or negedge reset_n) begin
        if (!reset_n) PC_out <= 32'b0;
        else       PC_out <= PC_in;
    end
endmodule

//=============================
// PC + 4
//=============================
module PC_plus4(
    input  logic [31:0] fromPC,             // Input PC value
    output logic [31:0] NextPC              // Output PC value (PC + 4)
);
    assign NextPC = fromPC + 32'd4;
endmodule

//=============================
// Instruction Memory (64 words)
//=============================
module Instruction_Memory(
    input  logic        clk,            // Clock signal
    input  logic        reset_n,        // kept for interface consistency; not used
    input  logic [31:0] read_address,   // byte address
    output logic [31:0] instruction_out // Instruction output (32 bits)
);
    // 64 x 32-bit IMEM 
    logic [31:0] I_Mem [0:63];

    // Combinational read by word index
    wire [31:0] word_index_full = read_address[31:2];
    wire        in_range        = (word_index_full < 64);

    assign instruction_out = in_range ? I_Mem[word_index_full] : 32'h0000_0013; // NOP on OOR

    // Simulation-only checks
    // synthesis translate_off
    always @* begin
        assert (read_address[1:0] == 2'b00)
          else $error("[IMEM] Misaligned fetch @ %h", read_address);
        if (!in_range)
          $warning("[IMEM] Fetch OOR: idx=%0d (depth=64). Returning NOP.", word_index_full);
    end
    // synthesis translate_on
endmodule
//=============================
// Register File (x0..x31)
//=============================
module Reg_File(
    input  logic        clk,            // Clock signal
    input  logic        reset_n,        // Active-low reset
    input  logic        RegWrite,       // Write enable
    input  logic [4:0]  Rs1,            // Source register 1
    input  logic [4:0]  Rs2,            // Source register 2
    input  logic [4:0]  Rd,             // Destination register
    input  logic [31:0] Write_data,     // Data to write
    output logic [31:0] read_data1,     // Read data from Rs1
    output logic [31:0] read_data2      // Read data from Rs2
);
    logic [31:0] regfile [0:31];
    integer i;

    // Write port (x0 always 0)
    always_ff @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            for (i = 0; i < 32; i = i + 1) regfile[i] <= 32'b0;
        end else if (RegWrite && (Rd != 5'd0)) begin
            regfile[Rd] <= Write_data;
        end
    end

    // Read ports (combination)
    assign read_data1 = (Rs1 == 5'd0) ? 32'b0 : regfile[Rs1];
    assign read_data2 = (Rs2 == 5'd0) ? 32'b0 : regfile[Rs2];
endmodule

//=============================
// Immediate Generator (I / S / B)
//=============================
module ImmGen(
    input  logic [31:0] instruction,        // Input instruction
    output logic [31:0] ImmExt              // Extended immediate value
);
    // Extract opcode once
    logic [6:0] opcode;
    assign opcode = instruction[6:0];

// Define the immediate extraction logic
    localparam logic [6:0]
        OPC_LOAD   = 7'b0000011,            // I-type (e.g., LW)
        OPC_ALUI   = 7'b0010011,            // I-type ALU (e.g., ADDI)
        OPC_STORE  = 7'b0100011,            // S-type (e.g., SW)
        OPC_BRANCH = 7'b1100011,            // B-type (e.g., BEQ)
        OPC_JAL    = 7'b1101111,            // J-type (JAL)
        OPC_JALR   = 7'b1100111,            // I-type (JALR)
        OPC_LUI    = 7'b0110111,            // U-type (LUI)
        OPC_AUIPC  = 7'b0010111;            // U-type (AUIPC)

    always_comb begin
        unique case (opcode)
            // I-type: LOAD, ALU-I, and JALR share the same immediate layout [31:20]
            OPC_LOAD, OPC_ALUI, OPC_JALR: begin
                // Sign-extend 12-bit imm: instruction[31:20]
                ImmExt = {{20{instruction[31]}}, instruction[31:20]};
            end

            // S-type: imm[11:5]=[31:25], imm[4:0]=[11:7]
            OPC_STORE: begin
                ImmExt = {{20{instruction[31]}}, instruction[31:25], instruction[11:7]};
            end

            // B-type: imm[12|10:5|4:1|11|0] = [31|30:25|11:8|7|0]
            OPC_BRANCH: begin
                ImmExt = {{19{instruction[31]}},
                          instruction[31], instruction[7],
                          instruction[30:25], instruction[11:8],
                          1'b0};
            end

            // J-type (JAL): imm[20|10:1|11|19:12|0] = [31|30:21|20|19:12|0]
            OPC_JAL: begin
                ImmExt = {{11{instruction[31]}},
                          instruction[31],
                          instruction[19:12],
                          instruction[20],
                          instruction[30:21],
                          1'b0};
            end

            // U-type (LUI/AUIPC): imm[31:12] << 12
            OPC_LUI, OPC_AUIPC: begin
                ImmExt = {instruction[31:12], 12'b0};
            end

            default: begin
                // For unsupported opcodes (SYSTEM, FENCE, etc.), return 0
                ImmExt = 32'b0;
            end
        endcase
    end

    // Synthesis assertion to ensure B-type and J-type immediates have LSB=0
    // synthesis translate_off
    always_comb begin
        if (opcode == OPC_BRANCH) begin
            assert (ImmExt[0] == 1'b0)
                else $error("B-type imm LSB should be 0");
        end
        if (opcode == OPC_JAL) begin
            assert (ImmExt[0] == 1'b0)
                else $error("J-type imm LSB should be 0");
        end
    end
    // synthesis translate_on

endmodule


//=============================
// Control Unit
//=============================
module Control_Unit(
    input  logic [6:0]  opcode,     // Input opcode from instruction
    output logic        Branch,     // Branch enable
    output logic        MemRead,    // Memory read enable
    output logic        MemtoReg,   // Memory to register
    output logic [1:0]  ALUOp,      // ALU operation select
    output logic        MemWrite,   // Memory write enable
    output logic        ALUSrc,     // ALU source select (0: Rs2, 1: Imm)
    output logic        RegWrite,   // Register write enable

    // Additional control signals for PC and result selection
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
    input  logic [31:0] A,              // First operand (Rs1 or PC)
    input  logic [31:0] B,              // Second operand (Rs2 or Imm)
    input  logic [3:0]  Control_in,     // ALU control signal
    output logic [31:0] ALU_result,     // ALU result

    // Zero flag for branch comparison
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
	input  logic        is_rtype,    // R-type instruction
    output logic [3:0]  Control_out  // ALU control output
);
    always_comb begin
        unique case (ALUOp)
            2'b00: Control_out = 4'b0010;   // ADD (load/store)
            2'b01: Control_out = 4'b0110;   // SUB (branch compare)
            2'b10: begin                    // R-type & I-type
					unique case (funct3)
						3'b000: Control_out = (is_rtype && funct7) ? 4'b0110    // SUB (R)
                                                               : 4'b0010;       // ADD / ADDI
						3'b111: Control_out = 4'b0000;          // AND
						3'b110: Control_out = 4'b0001;          // OR
						default: Control_out = 4'b0010;         // Default ADD
					endcase
				end
			2'b11: Control_out = 4'b0111;       // LUI (imm)
            default: Control_out = 4'b0010;     // Default to ADD
        endcase
    end
endmodule

//=============================
// Data Memory (64 words)
//=============================
module Data_Memory(
    input  logic        clk,            // Clock signal
    input  logic        reset_n,        // Active-low reset
    input  logic        MemWrite,       // Memory write enable
    input  logic        MemRead,        // Memory read enable
    input  logic [31:0] read_address,   // byte address
    input  logic [31:0] Write_data,     // Data to write
    output logic [31:0] MemData_out     // Data read from memory
);
    logic [31:0] D_Memory [0:63];
    integer i;

    always_ff @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            for (i = 0; i < 64; i = i + 1) D_Memory[i] <= 32'b0;                // Reset memory
        end else if (MemWrite) begin
            D_Memory[read_address[31:2]] <= Write_data;                         // Write data at word address
        end
    end

    assign MemData_out = (MemRead) ? D_Memory[read_address[31:2]] : 32'b0;      // Read data from memory
endmodule

//=============================
// Simple 2:1 Mux (parametric width)
//=============================
module Mux2to1 #(
    parameter W = 32
)(
    input  logic        sel,            // 0:A, 1:B
    input  logic [W-1:0] A,    
    input  logic [W-1:0] B,
    output logic [W-1:0] Y
);
    assign Y = sel ? B : A;             // sel==1 -> B
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
    input  logic        branch_en,        // Branch from Control_Unit
    input  logic [2:0]  funct3,           // instruction[14:12]
    input  logic        zero,             // from ALU (A-B==0)
    output logic        take_branch
);
    always_comb begin
        if (!branch_en) begin
            take_branch = 1'b0;
        end else begin
            unique case (funct3)
                3'b000: take_branch =  zero; // BEQ
                3'b001: take_branch = ~zero; // BNE
                default: take_branch = 1'b0; // Non supported
            endcase
        end
    end
endmodule


//=============================================
// End of RTL/RISC-V/risc-v-32-bit.sv
//=============================================