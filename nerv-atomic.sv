/*
 *  NERV -- Naive Educational RISC-V Processor
 *
 *  Copyright (C) 2020  Claire Xenia Wolf <claire@yosyshq.com>
 *
 *  Permission to use, copy, modify, and/or distribute this software for any
 *  purpose with or without fee is hereby granted, provided that the above
 *  copyright notice and this permission notice appear in all copies.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 *  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 *  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 *  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 *  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 *  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 *

 /* TODO's:

 PART 1:
 Need to do handshake with the multicore DRAM instead of using a counter
 to emulate memory access latency (for example, mem_wait_cycles is currently reset to
 a hardcoded value of 1 to simulate register based memory access delay.)

 PART 2:
 TODO

 PART 3:
 TODO

 */


module nerv #(
	parameter [31:0] RESET_ADDR = 32'h 0000_0000,
	parameter integer NUMREGS = 32
) (
	input clock,
	input reset,
	output trap,

	// we have 2 external memories
	// one is instruction memory
	output [31:0] imem_addr,
	input  [31:0] imem_data,

	// the other is data memory
	output        dmem_valid,
    output        dmem_wr_is_cond,
	output [31:0] dmem_addr,
	output [ 3:0] dmem_wstrb,
	output [31:0] dmem_wdata,
	input  [31:0] dmem_rdata,

    output dmem_resv,  // reservations
    input  dmem_cond   // conditional    
);
	// registers, instruction reg, program counter, next pc
	logic [31:0] regfile [0:NUMREGS-1];

	logic [31:0] pc;
	wire [31:0] insn;
	
	// instruction memory pointer
	assign imem_addr = pc;
	assign insn = imem_data;
	
	// Decode instruction fields
	wire [6:0] insn_funct7;
	wire [4:0] insn_rs2;
	wire [4:0] insn_rs1;
	wire [2:0] insn_funct3;
	wire [4:0] insn_rd;
	wire [6:0] insn_opcode;
	
	assign {insn_funct7, insn_rs2, insn_rs1, insn_funct3, insn_rd, insn_opcode} = insn;
	
	// Register file reads
	wire [31:0] rs1_value = !insn_rs1 ? 0 : regfile[insn_rs1];
	wire [31:0] rs2_value = !insn_rs2 ? 0 : regfile[insn_rs2];
	
	// Immediate values
	wire [11:0] imm_i = insn[31:20];
	wire [31:0] imm_i_sext = $signed(imm_i);
	
	// Opcodes
	localparam OPCODE_OP_IMM  = 7'b 00_100_11;
	localparam OPCODE_OP      = 7'b 01_100_11;
	localparam OPCODE_LUI     = 7'b 01_101_11;
	localparam OPCODE_AUIPC   = 7'b 00_101_11;
	localparam OPCODE_JAL     = 7'b 11_011_11;
	localparam OPCODE_JALR    = 7'b 11_001_11;
	localparam OPCODE_BRANCH  = 7'b 11_000_11;
	localparam OPCODE_LOAD    = 7'b 00_000_11;
	localparam OPCODE_STORE   = 7'b 01_000_11;
	localparam OPCODE_AMO     = 7'b 01_011_11;  // Atomic operations
	localparam OPCODE_SYSTEM  = 7'b 11_100_11;
	
	// AMO funct7[6:2] values for atomic instructions
	localparam AMO_LR = 5'b00010;
	localparam AMO_SC = 5'b00011;

	// Acquire access flag & release access flag for atomic instructions
	wire aq = insn_funct7[1];
	wire rl = insn_funct7[0];
	
	// Control signals
	logic illegalinsn;
	logic regwrite;
	logic [31:0] npc;
	logic [31:0] regfiledata;
	logic mem_valid;
	logic mem_wr_is_cond;
	logic [31:0] mem_addr;
	logic [3:0] mem_wstrb;
	logic [31:0] mem_wdata;
	logic mem_resv;
	
	// Trap signal (for ebreak)
	assign trap = (insn_opcode == OPCODE_SYSTEM) && (insn_funct3 == 3'b000) && (imm_i == 12'b000000000001);

	// Pipeline signals for zalrsc extension
	logic [7:0] mem_wait_cycles; // Assume memory responds within 256 cycles
	logic mem_wait;
	logic [4:0] mem_rd;
	logic mem_is_lr;
	logic mem_is_sc;
	logic mem_is_load;
	logic [2:0] mem_load_funct3;  // Store funct3 for load formatting
	
	// Combinational logic for control signals
	always_comb begin
		illegalinsn = 0;
		regwrite = 0;
		regfiledata = 32'h0;
		mem_valid = 0;
		mem_wr_is_cond = 0;
		mem_addr = 32'h0;
		mem_wstrb = 4'h0;
		mem_wdata = 32'h0;
		mem_resv = 0;

		if (mem_wait) begin
			npc = pc;  // Stall PC
		end else begin
			npc = pc + 4;

			case (insn_opcode)
				OPCODE_OP_IMM: begin
					// Immediate arithmetic operations (addi, slti, etc.)
					regwrite = 1;
					case (insn_funct3)
						3'b000: regfiledata = rs1_value + imm_i_sext;  // ADDI
						3'b010: regfiledata = $signed(rs1_value) < $signed(imm_i_sext) ? 1 : 0;  // SLTI
						3'b011: regfiledata = rs1_value < imm_i_sext ? 1 : 0;  // SLTIU
						3'b100: regfiledata = rs1_value ^ imm_i_sext;  // XORI
						3'b110: regfiledata = rs1_value | imm_i_sext;  // ORI
						3'b111: regfiledata = rs1_value & imm_i_sext;  // ANDI
						3'b001: regfiledata = rs1_value << imm_i[4:0];  // SLLI
						3'b101: begin
							if (insn_funct7[5]) 
								regfiledata = $signed(rs1_value) >>> imm_i[4:0];  // SRAI
							else
								regfiledata = rs1_value >> imm_i[4:0];  // SRLI
						end
					endcase
				end
				
				OPCODE_LOAD: begin
					// Load instructions (lw, lh, lb, etc.)
					mem_valid = 1;
					mem_addr = rs1_value + imm_i_sext;
					regwrite = 1;
					
					case (insn_funct3)
						3'b010: regfiledata = dmem_rdata;  // LW - load word
						3'b000: regfiledata = {{24{dmem_rdata[7]}}, dmem_rdata[7:0]};  // LB - load byte (sign-extended)
						3'b001: regfiledata = {{16{dmem_rdata[15]}}, dmem_rdata[15:0]};  // LH - load halfword (sign-extended)
						3'b100: regfiledata = {24'b0, dmem_rdata[7:0]};  // LBU - load byte unsigned
						3'b101: regfiledata = {16'b0, dmem_rdata[15:0]};  // LHU - load halfword unsigned
						default: illegalinsn = 1;
					endcase
				end
				
				OPCODE_AMO: begin
					// Only support lr.w & sc.w, RV32IA arch
					if (insn_funct3 == 3'b010) begin
						mem_valid = 1;
						mem_addr = rs1_value;
						regwrite = 1;
						
						case (insn_funct7[6:2])
							AMO_LR: begin
								// Load-Reserved (lr.w): read from memory, set reservation
								regfiledata = dmem_rdata;
								mem_wstrb = 4'b0000;  // Read operation
								mem_resv = 1;
							end
							
							AMO_SC: begin
								// Store-Conditional (sc.w): write if reservation valid
								regfiledata = {31'b0, dmem_cond};
								mem_wr_is_cond = 1;
								mem_wstrb = 4'b1111;  // Write full word
								mem_wdata = rs2_value;
							end
							
							default: illegalinsn = 1;
						endcase
					end else begin
						illegalinsn = 1;
					end
				end
				
				default: illegalinsn = 1;
			endcase
			
			// Check unaligned PC (I.e. incorrectly offset word)
			if ((npc & 32'b11) != 0) begin
				illegalinsn = 1;
				npc = pc & ~32'b11;
			end
		end
	end
	
	// Sequential register file and PC updates
	// Pipelined to handle atomic zalrsc based instructions (lr.w, sc.w) and regular loads
	always_ff @(posedge clock) begin
		if (reset) begin
			pc <= RESET_ADDR;
			mem_wait <= 0;
			mem_is_lr <= 0;
			mem_is_sc <= 0;
			mem_is_load <= 0;
			mem_rd <= 0;

			for (int i = 0; i < NUMREGS; i++) begin
				regfile[i] <= 32'h0;
			end
		end else if (!trap) begin

			// Case 1: memory is ready after waiting
			if (mem_wait) begin
				if (mem_wait_cycles == '1) begin
					$error("Memory operation timed out");
				end else if (mem_wait_cycles != 0) begin
					mem_wait_cycles <= mem_wait_cycles - 1;
				end else begin
					mem_wait_cycles <= '0;

					if (mem_is_lr) begin
						regfile[mem_rd] <= dmem_rdata;			// Complete lr.w
					end else if (mem_is_sc) begin
						regfile[mem_rd] <= {31'b0, dmem_cond};	// Complete sc.w
					end else if (mem_is_load) begin
						// Format the loaded data based on the load type
						case (mem_load_funct3)
							3'b010: regfile[mem_rd] <= dmem_rdata;  // LW
							3'b000: regfile[mem_rd] <= {{24{dmem_rdata[7]}}, dmem_rdata[7:0]};  // LB
							3'b001: regfile[mem_rd] <= {{16{dmem_rdata[15]}}, dmem_rdata[15:0]};  // LH
							3'b100: regfile[mem_rd] <= {24'b0, dmem_rdata[7:0]};  // LBU
							3'b101: regfile[mem_rd] <= {16'b0, dmem_rdata[15:0]};  // LHU
						endcase
					end

					mem_wait <= 0;
					mem_is_lr <= 0;
					mem_is_sc <= 0;
					mem_is_load <= 0;
					pc <= pc + 4;  								// Resume PC increment
				end
			end
			
			// Case 2: starting a new memory operation
			else if (mem_valid && (insn_opcode == OPCODE_AMO || insn_opcode == OPCODE_LOAD)) begin
				mem_wait <= 1;
				mem_wait_cycles <= 1;	// Wait 1 cycle before checking memory response (for multi-cycle SRAM latency)
				mem_rd <= insn_rd; 		// Defer insn_rd to be written to later (after waiting for memory)

				// Set flags for atomic operation (can't be done combinationally)
				if (insn_opcode == OPCODE_AMO) begin
					if (insn_funct7[6:2] == AMO_LR) begin
						mem_is_lr <= 1;
					end else if (insn_funct7[6:2] == AMO_SC) begin
						mem_is_sc <= 1;
					end
				end else if (insn_opcode == OPCODE_LOAD) begin
					mem_is_load <= 1;
					mem_load_funct3 <= insn_funct3;  // Store funct3 to format data later
				end
			end
			
			// Case 3: instruction doesn't touch memory
			else begin
				pc <= npc;
				
				// Write to register file
				if (regwrite && insn_rd != 0) begin
					regfile[insn_rd] <= regfiledata;
				end
			end
		end
	end

	// Memory interface signals
	assign dmem_valid = mem_valid;
	assign dmem_wr_is_cond = mem_wr_is_cond;
	assign dmem_addr = mem_addr;
	assign dmem_wstrb = mem_wstrb;
	assign dmem_wdata = mem_wdata;
	assign dmem_resv = mem_resv;

endmodule








/****************************************************************************************/

module multicore_memory #(
    parameter MEM_ADDR_WIDTH = 17,
    parameter string fname, 
    parameter RESV_BITS=12
) (
	input clock,
	input reset,

	
    input  [31:0] imem_addr0,
	output reg [31:0] imem_data0,

	input         dmem_valid0,
    input         dmem_wr_is_cond0,
	input  [31:0] dmem_addr0,
	input  [ 3:0] dmem_wstrb0,
	input  [31:0] dmem_wdata0,
	output [31:0] dmem_rdata0,

    input  [31:0] imem_addr1,
	output reg [31:0] imem_data1,

	input         dmem_valid1,
    input         dmem_wr_is_cond1,
	input  [31:0] dmem_addr1,
	input  [ 3:0] dmem_wstrb1,
	input  [31:0] dmem_wdata1,
	output [31:0] dmem_rdata1,

    input  [1:0] dmem_resv,  // reservations
    output [1:0] dmem_cond   // conditional        
);
    // THE ACTUAL MEMORY
    reg [7:0] mem [0:(1<<MEM_ADDR_WIDTH)-1];

    // Load firmware into memory
    initial begin
        $readmemh(fname, mem);
    end

    // Instruction memory for hart 0
    always @* begin
        imem_data0 = {mem[imem_addr0[MEM_ADDR_WIDTH-1:0] + 3],
                      mem[imem_addr0[MEM_ADDR_WIDTH-1:0] + 2],
                      mem[imem_addr0[MEM_ADDR_WIDTH-1:0] + 1],
                      mem[imem_addr0[MEM_ADDR_WIDTH-1:0] + 0]};
    end

    // Instruction memory for hart 1
    always @* begin
        imem_data1 = {mem[imem_addr1[MEM_ADDR_WIDTH-1:0] + 3],
                      mem[imem_addr1[MEM_ADDR_WIDTH-1:0] + 2],
                      mem[imem_addr1[MEM_ADDR_WIDTH-1:0] + 1],
                      mem[imem_addr1[MEM_ADDR_WIDTH-1:0] + 0]};
    end

    // ========================================================================
    // ATOMIC MEMORY OPERATIONS (LR/SC) IMPLEMENTATION
    // ========================================================================
    
    // Step 1: Track reservations for each hart
    // Each hart can have one active reservation on a memory address
    // We store the upper bits of the reserved address (using RESV_BITS parameter)
    reg [RESV_BITS-1:0] resv_addr0;  // Reserved address for hart 0
    reg [RESV_BITS-1:0] resv_addr1;  // Reserved address for hart 1
    reg resv_valid0;                  // Is hart 0's reservation valid?
    reg resv_valid1;                  // Is hart 1's reservation valid?
    
    // Step 2: Check if a memory access conflicts with existing reservations
    // A conflict occurs when one hart writes to an address that another hart has reserved
    wire resv_conflict0 = dmem_valid0 && (|dmem_wstrb0) && !dmem_wr_is_cond0 &&
                          resv_valid1 && (dmem_addr0[RESV_BITS+1:2] == resv_addr1);
    wire resv_conflict1 = dmem_valid1 && (|dmem_wstrb1) && !dmem_wr_is_cond1 &&
                          resv_valid0 && (dmem_addr1[RESV_BITS+1:2] == resv_addr0);
    
    // Step 3: Determine if SC (Store-Conditional) should succeed
    // SC succeeds (returns 0) if the reservation is still valid
    // SC fails (returns 1) if the reservation was invalidated
    reg dmem_cond0_reg;
    reg dmem_cond1_reg;
    assign dmem_cond = {dmem_cond1_reg, dmem_cond0_reg};
    
    // Step 4: Manage reservations and perform memory operations for hart 0
    reg [31:0] dmem_rdata0_reg;
    assign dmem_rdata0 = dmem_rdata0_reg;
    
    always @(posedge clock) begin
        if (reset) begin
            // Clear all reservations on reset
            resv_valid0 <= 0;
            dmem_cond0_reg <= 1;  // Failed by default
        end else begin
            // Check if hart 1 invalidated our reservation
            if (resv_conflict1) begin
                resv_valid0 <= 0;
            end
            
            if (dmem_valid0) begin
                // Always read the data from memory
                dmem_rdata0_reg <= {mem[dmem_addr0[MEM_ADDR_WIDTH-1:0] + 3],
                                    mem[dmem_addr0[MEM_ADDR_WIDTH-1:0] + 2],
                                    mem[dmem_addr0[MEM_ADDR_WIDTH-1:0] + 1],
                                    mem[dmem_addr0[MEM_ADDR_WIDTH-1:0] + 0]};
                
                if (dmem_resv[0]) begin
                    // This is a LR (Load-Reserved) instruction
                    // Set a reservation on this address
                    resv_addr0 <= dmem_addr0[RESV_BITS+1:2];
                    resv_valid0 <= 1;
                    dmem_cond0_reg <= 0;  // LR always succeeds
                    
                end else if (dmem_wr_is_cond0) begin
                    // This is a SC (Store-Conditional) instruction
                    // Only write if our reservation is still valid
                    if (resv_valid0 && (dmem_addr0[RESV_BITS+1:2] == resv_addr0)) begin
                        // Reservation is valid - SC succeeds!
                        if (dmem_wstrb0[0]) mem[dmem_addr0[MEM_ADDR_WIDTH-1:0] + 0] <= dmem_wdata0[7:0];
                        if (dmem_wstrb0[1]) mem[dmem_addr0[MEM_ADDR_WIDTH-1:0] + 1] <= dmem_wdata0[15:8];
                        if (dmem_wstrb0[2]) mem[dmem_addr0[MEM_ADDR_WIDTH-1:0] + 2] <= dmem_wdata0[23:16];
                        if (dmem_wstrb0[3]) mem[dmem_addr0[MEM_ADDR_WIDTH-1:0] + 3] <= dmem_wdata0[31:24];
                        dmem_cond0_reg <= 0;  // Return 0 (success)
                        resv_valid0 <= 0;     // Clear the reservation after successful SC
                    end else begin
                        // Reservation was lost - SC fails!
                        dmem_cond0_reg <= 1;  // Return 1 (failure)
                    end
                    
                end else if (|dmem_wstrb0) begin
                    // This is a normal store (not SC)
                    // Perform the write and clear our own reservation if we're writing to it
                    if (dmem_wstrb0[0]) mem[dmem_addr0[MEM_ADDR_WIDTH-1:0] + 0] <= dmem_wdata0[7:0];
                    if (dmem_wstrb0[1]) mem[dmem_addr0[MEM_ADDR_WIDTH-1:0] + 1] <= dmem_wdata0[15:8];
                    if (dmem_wstrb0[2]) mem[dmem_addr0[MEM_ADDR_WIDTH-1:0] + 2] <= dmem_wdata0[23:16];
                    if (dmem_wstrb0[3]) mem[dmem_addr0[MEM_ADDR_WIDTH-1:0] + 3] <= dmem_wdata0[31:24];
                    
                    // Clear our own reservation if we're writing to our reserved address
                    if (resv_valid0 && (dmem_addr0[RESV_BITS+1:2] == resv_addr0)) begin
                        resv_valid0 <= 0;
                    end
                end
            end
        end
    end
    
    // Step 5: Manage reservations and perform memory operations for hart 1
    // (Same logic as hart 0, but for the second core)
    reg [31:0] dmem_rdata1_reg;
    assign dmem_rdata1 = dmem_rdata1_reg;
    
    always @(posedge clock) begin
        if (reset) begin
            resv_valid1 <= 0;
            dmem_cond1_reg <= 1;
        end else begin
            if (resv_conflict0) begin
                resv_valid1 <= 0;
            end
            
            if (dmem_valid1) begin
                dmem_rdata1_reg <= {mem[dmem_addr1[MEM_ADDR_WIDTH-1:0] + 3],
                                    mem[dmem_addr1[MEM_ADDR_WIDTH-1:0] + 2],
                                    mem[dmem_addr1[MEM_ADDR_WIDTH-1:0] + 1],
                                    mem[dmem_addr1[MEM_ADDR_WIDTH-1:0] + 0]};
                
                if (dmem_resv[1]) begin
                    resv_addr1 <= dmem_addr1[RESV_BITS+1:2];
                    resv_valid1 <= 1;
                    dmem_cond1_reg <= 0;
                    
                end else if (dmem_wr_is_cond1) begin
                    if (resv_valid1 && (dmem_addr1[RESV_BITS+1:2] == resv_addr1)) begin
                        if (dmem_wstrb1[0]) mem[dmem_addr1[MEM_ADDR_WIDTH-1:0] + 0] <= dmem_wdata1[7:0];
                        if (dmem_wstrb1[1]) mem[dmem_addr1[MEM_ADDR_WIDTH-1:0] + 1] <= dmem_wdata1[15:8];
                        if (dmem_wstrb1[2]) mem[dmem_addr1[MEM_ADDR_WIDTH-1:0] + 2] <= dmem_wdata1[23:16];
                        if (dmem_wstrb1[3]) mem[dmem_addr1[MEM_ADDR_WIDTH-1:0] + 3] <= dmem_wdata1[31:24];
                        dmem_cond1_reg <= 0;
                        resv_valid1 <= 0;
                    end else begin
                        dmem_cond1_reg <= 1;
                    end
                    
                end else if (|dmem_wstrb1) begin
                    if (dmem_wstrb1[0]) mem[dmem_addr1[MEM_ADDR_WIDTH-1:0] + 0] <= dmem_wdata1[7:0];
                    if (dmem_wstrb1[1]) mem[dmem_addr1[MEM_ADDR_WIDTH-1:0] + 1] <= dmem_wdata1[15:8];
                    if (dmem_wstrb1[2]) mem[dmem_addr1[MEM_ADDR_WIDTH-1:0] + 2] <= dmem_wdata1[23:16];
                    if (dmem_wstrb1[3]) mem[dmem_addr1[MEM_ADDR_WIDTH-1:0] + 3] <= dmem_wdata1[31:24];
                    
                    if (resv_valid1 && (dmem_addr1[RESV_BITS+1:2] == resv_addr1)) begin
                        resv_valid1 <= 0;
                    end
                end
            end
        end
    end
    
endmodule
