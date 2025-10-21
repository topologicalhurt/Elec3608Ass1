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
 *  LR/SC MEMORY SEMANTICS:
 *  - Reads-before-writes ordering within same cycle
 *  - Concurrent LR+SC both succeed (only normal stores invalidate LR)
 *  - Hart 0 has priority for simultaneous SC conflicts
 *  - Reservation granularity: 4KB
 */


module nerv #(
	parameter [31:0] RESET_ADDR = 32'h 0000_0000,
	parameter integer NUMREGS = 32
) (
	input clock,
	input reset,
	output trap,

	output [31:0] imem_addr,
	input  [31:0] imem_data,

	output        dmem_valid,
    output        dmem_wr_is_cond,
	output [31:0] dmem_addr,
	output [ 3:0] dmem_wstrb,
	output [31:0] dmem_wdata,
	input  [31:0] dmem_rdata,

    output dmem_resv,
    input  dmem_cond
);
	logic [31:0] regfile [0:NUMREGS-1];
	logic [31:0] pc;
	wire [31:0] insn;
	
	assign imem_addr = pc;
	assign insn = imem_data;
	
	wire [6:0] insn_funct7;
	wire [4:0] insn_rs2;
	wire [4:0] insn_rs1;
	wire [2:0] insn_funct3;
	wire [4:0] insn_rd;
	wire [6:0] insn_opcode;
	
	assign {insn_funct7, insn_rs2, insn_rs1, insn_funct3, insn_rd, insn_opcode} = insn;
	
	wire [31:0] rs1_value = !insn_rs1 ? 0 : regfile[insn_rs1];
	wire [31:0] rs2_value = !insn_rs2 ? 0 : regfile[insn_rs2];
	
	wire [11:0] imm_i = insn[31:20];
	wire [31:0] imm_i_sext = $signed(imm_i);
	
	localparam OPCODE_OP_IMM  = 7'b 00_100_11;
	localparam OPCODE_OP      = 7'b 01_100_11;
	localparam OPCODE_LUI     = 7'b 01_101_11;
	localparam OPCODE_AUIPC   = 7'b 00_101_11;
	localparam OPCODE_JAL     = 7'b 11_011_11;
	localparam OPCODE_JALR    = 7'b 11_001_11;
	localparam OPCODE_BRANCH  = 7'b 11_000_11;
	localparam OPCODE_LOAD    = 7'b 00_000_11;
	localparam OPCODE_STORE   = 7'b 01_000_11;
	localparam OPCODE_AMO     = 7'b 01_011_11;
	localparam OPCODE_SYSTEM  = 7'b 11_100_11;
	
	localparam AMO_LR = 5'b00010;
	localparam AMO_SC = 5'b00011;

	wire aq = insn_funct7[1];
	wire rl = insn_funct7[0];
	
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
	
	assign trap = (insn_opcode == OPCODE_SYSTEM) && (insn_funct3 == 3'b000) && (imm_i == 12'b000000000001);

	// PIPELINE: Memory operations require PC stall and deferred writeback
	// Set MEM_WAIT_DEFAULT=0 for single-cycle / register based memory (no extra wait)
	// Set MEM_WAIT_DEFAULT>0 and define MEM_WAIT_MULTICYCLE for multi-cycle memory
	localparam MEM_WAIT_DEFAULT = 0;
	// `define MEM_WAIT_MULTICYCLE  // Uncomment for multi-cycle memory support

	`ifdef MEM_WAIT_MULTICYCLE
		logic [7:0] mem_wait_cycles;
	`endif
	logic mem_wait;
	logic [4:0] mem_rd;
	logic mem_is_lr;
	logic mem_is_sc;
	logic mem_is_load;
	logic [2:0] mem_load_funct3;
	
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
			npc = pc;
		end else begin
			npc = pc + 4;

			case (insn_opcode)
				OPCODE_OP_IMM: begin
					regwrite = 1;
					case (insn_funct3)
						3'b000: regfiledata = rs1_value + imm_i_sext;
						3'b010: regfiledata = $signed(rs1_value) < $signed(imm_i_sext) ? 1 : 0;
						3'b011: regfiledata = rs1_value < imm_i_sext ? 1 : 0;
						3'b100: regfiledata = rs1_value ^ imm_i_sext;
						3'b110: regfiledata = rs1_value | imm_i_sext;
						3'b111: regfiledata = rs1_value & imm_i_sext;
						3'b001: regfiledata = rs1_value << imm_i[4:0];
						3'b101: begin
							if (insn_funct7[5]) 
								regfiledata = $signed(rs1_value) >>> imm_i[4:0];
							else
								regfiledata = rs1_value >> imm_i[4:0];
						end
					endcase
				end
				
				OPCODE_LOAD: begin
					mem_valid = 1;
					mem_addr = rs1_value + imm_i_sext;
					regwrite = 1;
					
					case (insn_funct3)
						3'b010: regfiledata = dmem_rdata;
						3'b000: regfiledata = {{24{dmem_rdata[7]}}, dmem_rdata[7:0]};
						3'b001: regfiledata = {{16{dmem_rdata[15]}}, dmem_rdata[15:0]};
						3'b100: regfiledata = {24'b0, dmem_rdata[7:0]};
						3'b101: regfiledata = {16'b0, dmem_rdata[15:0]};
						default: illegalinsn = 1;
					endcase
				end
				
				OPCODE_AMO: begin
					if (insn_funct3 == 3'b010) begin
						mem_valid = 1;
						mem_addr = rs1_value;
						regwrite = 1;
						
						case (insn_funct7[6:2])
							AMO_LR: begin
								regfiledata = dmem_rdata;
								mem_wstrb = 4'b0000;
								mem_resv = 1;
							end
							
							AMO_SC: begin
								regfiledata = {31'b0, dmem_cond};
								mem_wr_is_cond = 1;
								mem_wstrb = 4'b1111;
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
			
			if ((npc & 32'b11) != 0) begin
				illegalinsn = 1;
				npc = pc & ~32'b11;
			end
		end
	end
	
	// PIPELINE STAGES:
	// Stage 1: Instruction decode (combinational)
	// Stage 2: Memory wait for LR/SC/LOAD operations (sequential)
	// Stage 3: Register writeback (sequential)
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

			if (mem_wait) begin
				
			`ifdef MEM_WAIT_MULTICYCLE
				// Multi-cycle wait: decrement counter until it reaches 0
				if (mem_wait_cycles == '1) begin
					$error("Memory operation timed out");
				end else if (mem_wait_cycles != 0) begin
					mem_wait_cycles <= mem_wait_cycles - 1;
				end else begin
					mem_wait_cycles <= '0;
			`endif

			// Complete memory operation and writeback
			if (mem_is_lr) begin
				regfile[mem_rd] <= dmem_rdata;
			end else if (mem_is_sc) begin
				regfile[mem_rd] <= {31'b0, dmem_cond};
			end else if (mem_is_load) begin
				case (mem_load_funct3)
					3'b010: regfile[mem_rd] <= dmem_rdata;
					3'b000: regfile[mem_rd] <= {{24{dmem_rdata[7]}}, dmem_rdata[7:0]};
					3'b001: regfile[mem_rd] <= {{16{dmem_rdata[15]}}, dmem_rdata[15:0]};
					3'b100: regfile[mem_rd] <= {24'b0, dmem_rdata[7:0]};
					3'b101: regfile[mem_rd] <= {16'b0, dmem_rdata[15:0]};
				endcase
			end

			mem_wait <= 0;
			mem_is_lr <= 0;
			mem_is_sc <= 0;
			mem_is_load <= 0;
			pc <= pc + 4;

			`ifdef MEM_WAIT_MULTICYCLE
				end
			`endif
			end
			
			else if (mem_valid && (insn_opcode == OPCODE_AMO || insn_opcode == OPCODE_LOAD)) begin
				mem_wait <= 1;

				`ifdef MEM_WAIT_MULTICYCLE
					mem_wait_cycles <= MEM_WAIT_DEFAULT;
				`endif

				mem_rd <= insn_rd;

				if (insn_opcode == OPCODE_AMO) begin
					if (insn_funct7[6:2] == AMO_LR) begin
						mem_is_lr <= 1;
					end else if (insn_funct7[6:2] == AMO_SC) begin
						mem_is_sc <= 1;
					end
				end else if (insn_opcode == OPCODE_LOAD) begin
					mem_is_load <= 1;
					mem_load_funct3 <= insn_funct3;
				end
			end
			
			else begin
				pc <= npc;
				
				if (regwrite && insn_rd != 0) begin
					regfile[insn_rd] <= regfiledata;
				end
			end
		end
	end

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
    parameter RESV_BITS = 12
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

    input  [1:0] dmem_resv,
    output [1:0] dmem_cond
);
    reg [7:0] mem [0:(1<<MEM_ADDR_WIDTH)-1];

    initial begin
        $readmemh(fname, mem);
    end

    wire [31:0] imem_addr [0:1];
    reg  [31:0] imem_data [0:1];
    
    assign imem_addr[0] = imem_addr0;
    assign imem_addr[1] = imem_addr1;
    assign imem_data0 = imem_data[0];
    assign imem_data1 = imem_data[1];
    
    genvar h;
    generate
        for (h = 0; h < 2; h = h + 1) begin : imem_gen
            always @* begin
                imem_data[h] = {mem[imem_addr[h][MEM_ADDR_WIDTH-1:0] + 3],
                                mem[imem_addr[h][MEM_ADDR_WIDTH-1:0] + 2],
                                mem[imem_addr[h][MEM_ADDR_WIDTH-1:0] + 1],
                                mem[imem_addr[h][MEM_ADDR_WIDTH-1:0] + 0]};
            end
        end
    endgenerate

    // DUAL-PORT MEMORY WITH PRIORITY: Hart 0 wins write conflicts
    wire        dmem_valid     [0:1];
    wire        dmem_wr_is_cond[0:1];
    wire [31:0] dmem_addr      [0:1];
    wire [3:0]  dmem_wstrb     [0:1];
    wire [31:0] dmem_wdata     [0:1];
    reg  [31:0] dmem_rdata_reg [0:1];
    
    assign dmem_valid[0]      = dmem_valid0;
    assign dmem_valid[1]      = dmem_valid1;
    assign dmem_wr_is_cond[0] = dmem_wr_is_cond0;
    assign dmem_wr_is_cond[1] = dmem_wr_is_cond1;
    assign dmem_addr[0]       = dmem_addr0;
    assign dmem_addr[1]       = dmem_addr1;
    assign dmem_wstrb[0]      = dmem_wstrb0;
    assign dmem_wstrb[1]      = dmem_wstrb1;
    assign dmem_wdata[0]      = dmem_wdata0;
    assign dmem_wdata[1]      = dmem_wdata1;
    assign dmem_rdata0        = dmem_rdata_reg[0];
    assign dmem_rdata1        = dmem_rdata_reg[1];
    
    wire [MEM_ADDR_WIDTH-3:0] write_addr [0:1];
    assign write_addr[0] = dmem_addr[0][MEM_ADDR_WIDTH-1:2];
    assign write_addr[1] = dmem_addr[1][MEM_ADDR_WIDTH-1:2];
    
    wire write_conflict_normal = dmem_valid[0] && dmem_valid[1] && 
                                 (|dmem_wstrb[0]) && (|dmem_wstrb[1]) &&
                                 (!dmem_wr_is_cond[0]) && (!dmem_wr_is_cond[1]) &&
                                 (write_addr[0] == write_addr[1]);

    wire hart_write_enable [0:1];
    assign hart_write_enable[0] = dmem_valid[0] && (|dmem_wstrb[0]);
    assign hart_write_enable[1] = dmem_valid[1] && (|dmem_wstrb[1]) && (!write_conflict_normal);
    
    // LR/SC reservation tracking
    reg [RESV_BITS-1:0] resv_addr  [0:1];
    reg                 resv_valid [0:1];
    reg                 dmem_cond_reg [0:1];
    
    assign dmem_cond = {dmem_cond_reg[1], dmem_cond_reg[0]};
    
	// 4KB reservation offset
    wire [RESV_BITS-1:0] resv_set [0:1];
    assign resv_set[0] = dmem_addr[0][11 + RESV_BITS:RESV_BITS];
    assign resv_set[1] = dmem_addr[1][11 + RESV_BITS:RESV_BITS];

    wire sc_conflict = dmem_valid[0] && dmem_valid[1] &&
                       dmem_wr_is_cond[0] && dmem_wr_is_cond[1] &&
                       resv_valid[0] && resv_valid[1] &&
                       resv_set == resv_addr &&
                       resv_set[0] == resv_set[1];
    
    // PIPELINE: 1) Read pre-cycle memory  2) Compute SC success  3) Apply writes  4) Update reservations
    generate
        for (genvar h = 0; h < 2; h++) begin : hart_gen
            localparam other = 1 - h;
            
			// Determine if this hart's SC succeeds
            wire hart_sc_succeeds = dmem_valid[h] && dmem_wr_is_cond[h] && resv_valid[h] &&
                                    (resv_set[h] == resv_addr[h]) && (h == 0 || !sc_conflict);
            
            // Determine if other hart writes to this hart's reserved block (normal store or successful SC)
            wire other_norm_store_to_this_block = dmem_valid[other] && (|dmem_wstrb[other]) && 
                                                   (!dmem_wr_is_cond[other]) && (resv_set[other] == resv_addr[h]);
            wire other_sc_to_this_block = hart_gen[other].hart_sc_succeeds && (resv_set[other] == resv_addr[h]);
            
            always @(posedge clock) begin
                if (reset) begin
                    resv_valid[h] <= 0;
                    dmem_cond_reg[h] <= 1;
                end else begin
                    // Phase 1: Reads (use pre-write memory state)
                    if (dmem_valid[h]) begin
                        dmem_rdata_reg[h] <= {mem[dmem_addr[h][MEM_ADDR_WIDTH-1:0] + 3],
                                              mem[dmem_addr[h][MEM_ADDR_WIDTH-1:0] + 2],
                                              mem[dmem_addr[h][MEM_ADDR_WIDTH-1:0] + 1],
                                              mem[dmem_addr[h][MEM_ADDR_WIDTH-1:0] + 0]};
                    end
                    
                    // Phase 2: Compute SC results (use pre-update reservation state)
                    if (dmem_valid[h] && dmem_wr_is_cond[h]) begin
                        dmem_cond_reg[h] <= hart_sc_succeeds ? 0 : 1;
                    end
                    
                    // Phase 3: Writes (apply after reads computed)
                    if ((dmem_valid[h] && dmem_wr_is_cond[h] && hart_sc_succeeds) ||
                        (hart_write_enable[h] && (!dmem_wr_is_cond[h]))) begin
                        if (dmem_wstrb[h][0]) mem[dmem_addr[h][MEM_ADDR_WIDTH-1:0] + 0] <= dmem_wdata[h][7:0];
                        if (dmem_wstrb[h][1]) mem[dmem_addr[h][MEM_ADDR_WIDTH-1:0] + 1] <= dmem_wdata[h][15:8];
                        if (dmem_wstrb[h][2]) mem[dmem_addr[h][MEM_ADDR_WIDTH-1:0] + 2] <= dmem_wdata[h][23:16];
                        if (dmem_wstrb[h][3]) mem[dmem_addr[h][MEM_ADDR_WIDTH-1:0] + 3] <= dmem_wdata[h][31:24];
                    end
                    
                    // Phase 4: Reservation management
                    if (dmem_valid[h]) begin

                        if (dmem_resv[h]) begin
                            // LR: Establish new reservation, invalidated only by concurrent normal store to same block
                            resv_addr[h] <= resv_set[h];
                            resv_valid[h] <= !other_norm_store_to_this_block;
                        end else if (dmem_wr_is_cond[h]) begin
                            // SC: Always clear own reservation
                            resv_valid[h] <= 0;
                        end else if (resv_valid[h] && (other_norm_store_to_this_block || other_sc_to_this_block)) begin
                            // Normal memory op (not LR, not SC): Invalidate if other hart writes to reserved block
                            resv_valid[h] <= 0;
                        end
                    end else if (resv_valid[h] && (other_norm_store_to_this_block || other_sc_to_this_block)) begin
                        // No operation by this hart: Invalidate if other hart writes to reserved block
                        resv_valid[h] <= 0;
                    end

                end
            end
        end
    endgenerate
    
endmodule
