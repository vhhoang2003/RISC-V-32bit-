`timescale 1ns/1ps

module risc_v_tb;

  // ============================================================
  // Clock / Reset
  // ============================================================
  reg  clk;
  reg  reset_n;

  // Unit Under Test
  top uut (
    .clk    (clk),
    .reset_n(reset_n)
  );

  // 100 MHz clock (10 ns period)
  initial clk = 1'b0;
  always #5 clk = ~clk;

  // ============================================================
  // Plusargs & runtime options
  // ============================================================
  integer MAX_CYCLES;
  string  IMEM_FILE;      // Program hex file path
  string  DMEM_FILE;      // Optional data hex file path
  bit     QUIET;          // Reduce logs if set (+QUIET)

  // ============================================================
  // Simulation control
  // ============================================================
  initial begin
    // Optional VCD dump (+vcd)
    if ($test$plusargs("vcd")) begin
      $dumpfile("wave.vcd");
      $dumpvars(0, risc_v_tb);
      $display("[TB] VCD dump enabled -> wave.vcd");
    end

    // Max cycles (+MAX_CYCLES=...)
    if (!$value$plusargs("MAX_CYCLES=%d", MAX_CYCLES))
      MAX_CYCLES = 2000;

    // Hex file paths
    // NOTE: You can pass absolute path via +IMEM=<ABS_PATH_TO_HEX>
    // Example: +IMEM=C:/Users/vhhoa/RISC_V_SoC/RISC_V_SoC.srcs/sim_1/new/program.hex
    IMEM_FILE = "program.hex";
    void'($value$plusargs("IMEM=%s", IMEM_FILE));

    // Optional DMem preload (+DMEM=...)
    if ($value$plusargs("DMEM=%s", DMEM_FILE)) begin
      $display("[TB] Will preload DMem from: %s", DMEM_FILE);
    end

    QUIET = $test$plusargs("QUIET");

    $display("[TB] MAX_CYCLES=%0d", MAX_CYCLES);
    $display("[TB] Loading IMEM from: %s", IMEM_FILE);

    // Hold reset active-low during memory preload
    reset_n = 1'b0;

    // ------------------------------------------------------------
    // Preload memories (hierarchical access)
    // If your Instruction_Memory also does $readmemh internally,
    // you may remove the following IMEM preload to avoid double-load.
    // ------------------------------------------------------------
    begin : PRELOAD_BLOCK
      integer fd;

      // Check IMEM file exists before loading
      fd = $fopen(IMEM_FILE, "r");
      if (fd == 0) begin
        $fatal(1, "[TB] Cannot open IMEM file: %s (check path/permissions)", IMEM_FILE);
      end
      $fclose(fd);

      // Load IMEM
      $readmemh(IMEM_FILE, uut.uIMem.I_Mem);

      // Optionally load DMEM if provided
      if (DMEM_FILE.len() != 0) begin
        fd = $fopen(DMEM_FILE, "r");
        if (fd == 0) begin
          $fatal(1, "[TB] Cannot open DMEM file: %s (check path/permissions)", DMEM_FILE);
        end
        $fclose(fd);
        $readmemh(DMEM_FILE, uut.uDMem.D_Memory);
      end
    end

    // Give time for $readmemh to complete
    #1;

    // Quick peek: print first 8 IMEM words
    $display("[CHK] IMem[0..7] = %08h %08h %08h %08h %08h %08h %08h %08h",
             uut.uIMem.I_Mem[0], uut.uIMem.I_Mem[1], uut.uIMem.I_Mem[2], uut.uIMem.I_Mem[3],
             uut.uIMem.I_Mem[4], uut.uIMem.I_Mem[5], uut.uIMem.I_Mem[6], uut.uIMem.I_Mem[7]);

    // Warn if first 8 words are all NOP (likely wrong path or empty file)
    if (&{(uut.uIMem.I_Mem[0] == 32'h0000_0013),
           (uut.uIMem.I_Mem[1] == 32'h0000_0013),
           (uut.uIMem.I_Mem[2] == 32'h0000_0013),
           (uut.uIMem.I_Mem[3] == 32'h0000_0013),
           (uut.uIMem.I_Mem[4] == 32'h0000_0013),
           (uut.uIMem.I_Mem[5] == 32'h0000_0013),
           (uut.uIMem.I_Mem[6] == 32'h0000_0013),
           (uut.uIMem.I_Mem[7] == 32'h0000_0013)}) begin
      $display("[WARN] First 8 IMEM words are all NOP (00000013). Check program file/path/BOM.");
    end

    // Release reset and run
    reset_n = 1'b1;

    // Run for MAX_CYCLES cycles
    repeat (MAX_CYCLES) @(posedge clk);

    $display("[%0t] TIMEOUT after %0d cycles", $time, MAX_CYCLES);
    dump_regs();
    $finish;
  end

  // ============================================================
  // Watchers / Debug prints
  // ============================================================
  // PC + Instruction each cycle (unless QUIET)
  always @(posedge clk) begin
    if (reset_n && !QUIET) begin
      $display("[%0t] PC=%08h  INSTR=%08h", $time, uut.PC_top, uut.instruction_top);
    end
  end

  // Log register write-back (skip rd==x0)
  always @(posedge clk) begin
    if (reset_n && uut.RegWrite_top && (uut.instruction_top[11:7] != 5'd0)) begin
      $display("[%0t] RD[%0d] <= %08h",
               $time, uut.instruction_top[11:7], uut.WriteBack_top);
    end
  end

  // Data memory accesses
  always @(posedge clk) begin
    if (reset_n && uut.MemWrite_top && !QUIET) begin
      $display("[%0t] DMEM[0x%08h] <= %08h",
               $time, uut.ALU_result_top, uut.Rd2_top);
    end
    if (reset_n && uut.MemRead_top && !QUIET) begin
      $display("[%0t] DMEM[0x%08h] -> %08h",
               $time, uut.ALU_result_top, uut.MemData_top);
    end
  end

  // ============================================================
  // Assertions / early stop hooks
  // ============================================================
  // PC must be word-aligned (catch misaligned fetches)
  always @* begin
    if (reset_n) begin
      assert (uut.PC_top[1:0] == 2'b00)
        else $fatal("[TB] Misaligned PC: %08h", uut.PC_top);
    end
  end

  // Optional: stop if instruction becomes all zero (hole)
  always @(posedge clk) begin
    if (reset_n && (uut.instruction_top === 32'h0000_0000)) begin
      // Uncomment to stop early when fetching zeros
      // $display("[%0t] Hit 0x00000000 instruction (hole). Stopping.", $time);
      // dump_regs(); $finish;
    end
  end

  // ============================================================
  // Helpers
  // ============================================================
  // Dump the whole register file once
  task dump_regs;
    integer i;
    begin
      $display("==== Register file dump ====");
      for (i = 0; i < 32; i = i + 1) begin
        $display("x%0d = %08h", i, uut.uRF.regfile[i]);
      end
      $display("============================");
    end
  endtask

  // Dump a DMem word range (byte address = index << 2)
  task dump_dmem_range(input int lo_idx, input int hi_idx);
    int j;
    begin
      $display("==== DMem[%0d..%0d] dump ====", lo_idx, hi_idx);
      for (j = lo_idx; j <= hi_idx; j++) begin
        $display("[%0d] @0x%08h = %08h", j, (j<<2), uut.uDMem.D_Memory[j]);
      end
      $display("================================");
    end
  endtask

endmodule
