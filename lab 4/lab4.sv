`ifndef _core_v
`define _core_v
`include "system.sv"
`include "base.sv"
`include "memory_io.sv"
`include "memory.sv"
`include "lab1.sv"

module core(
    input  logic       clk,
    input  logic       reset,
    input  logic      [`word_address_size-1:0] reset_pc,
    output memory_io_req   inst_mem_req,
    input  memory_io_rsp   inst_mem_rsp,
    output memory_io_req   data_mem_req,
    input  memory_io_rsp   data_mem_rsp
);

typedef enum {
    stage_fetch,
    stage_decode,
    stage_execute,
    stage_mem,
    stage_writeback
} stage;

stage current_stage;

logic [31:0] registers [31:0];
initial begin
    for (int i = 0; i < 32; i++) registers[i] = 32'b0;
end

// Decode fields (combinational in decode)
logic [31:0] instruction;
logic [4:0]  rd, rs1, rs2;
logic signed [31:0] imm;
logic [6:0]  opcode;
logic [2:0]  funct3;
logic [6:0]  funct7;

logic [31:0] rs1_data, rs2_data;
logic [31:0] alu_result;

// PC
word pc;

// NEW: Branch/Jump control signals
logic take_branch;
logic take_jump;
logic pc_src_is_reg;  // For JALR (rs1 + imm)
logic [31:0] next_pc;

// Pipeline registers
logic [6:0]  opcode_ex, opcode_mem, opcode_wb;
logic [2:0]  funct3_ex, funct3_mem;
logic [4:0]  rd_ex, rd_mem, rd_wb;

logic [31:0] rs2_data_ex, rs2_data_mem;     // store data forwarding
logic [31:0] alu_result_mem, alu_result_wb; // effective addr for mem, alu result for wb

logic [31:0] load_data_mem;  // value after sign/zero extend
logic [31:0] wb_value;       // what we actually write back
logic        wb_we;          // write enable in wb

// -------------------------
// Branch/Jump Logic (combinational in execute)
// -------------------------
always_comb begin
    take_branch = 1'b0;
    take_jump = 1'b0;
    pc_src_is_reg = 1'b0;
    
    if (current_stage == stage_execute) begin
        // Branch instructions (B-type)
        if (opcode_ex == 7'b1100011) begin
            take_branch = 1'b1;
            case (funct3_ex)
                3'b000: take_branch = (rs1_data == rs2_data);     // BEQ
                3'b001: take_branch = (rs1_data != rs2_data);     // BNE
                3'b100: take_branch = ($signed(rs1_data) < $signed(rs2_data));  // BLT
                3'b101: take_branch = ($signed(rs1_data) >= $signed(rs2_data)); // BGE
                3'b110: take_branch = (rs1_data < rs2_data);      // BLTU
                3'b111: take_branch = (rs1_data >= rs2_data);     // BGEU
                default: take_branch = 1'b0;
            endcase
        end
        
        // Jump instructions
        else if (opcode_ex == 7'b1101111) begin  // JAL
            take_jump = 1'b1;
        end
        else if (opcode_ex == 7'b1100111 && funct3_ex == 3'b000) begin  // JALR
            take_jump = 1'b1;
            pc_src_is_reg = 1'b1;
        end
    end
end

// Next PC calculation
always_comb begin
    next_pc = pc + 32'd4;  // Default: sequential PC
    
    if (current_stage == stage_execute) begin
        if (take_branch) begin
            next_pc = pc + imm;  // Note: imm already includes +1 shift from find_imm()
        end
        else if (take_jump) begin
            if (pc_src_is_reg) begin
                next_pc = (rs1_data & 32'hfffffffe) + imm;  // JALR: clear LSB of rs1
            end else begin
                next_pc = pc + imm;  // JAL
            end
        end
    end
end

// -------------------------
// Instruction memory request
// -------------------------
always_comb begin
    inst_mem_req.addr     = pc;
    inst_mem_req.data     = 32'b0;
    inst_mem_req.do_write = 4'b0000;

    if (current_stage == stage_fetch) begin
        inst_mem_req.valid   = 1'b1;
        inst_mem_req.do_read = 4'b1111;
    end else begin
        inst_mem_req.valid   = 1'b0;
        inst_mem_req.do_read = 4'b0000;
    end
end

// -------------------------
// Decode stage (combinational)
// -------------------------
always_comb begin
    // Safe defaults to avoid X-prop
    instruction = 32'b0;
    rd          = 5'b0;
    rs1         = 5'b0;
    rs2         = 5'b0;
    funct3      = 3'b0;
    funct7      = 7'b0;
    opcode      = 7'b0;
    imm         = '0;
    rs1_data    = 32'b0;
    rs2_data    = 32'b0;

    if (current_stage == stage_decode) begin
        instruction = inst_mem_rsp.data;
        rd     = instruction[11:7];
        rs1    = instruction[19:15];
        rs2    = instruction[24:20];
        funct3 = instruction[14:12];
        funct7 = instruction[31:25];
        opcode = instruction[6:0];

        print_instruction(pc, instruction);

        imm      = find_imm(opcode, instruction);
        rs1_data = (rs1 == 5'd0) ? 32'b0 : registers[rs1];
        rs2_data = (rs2 == 5'd0) ? 32'b0 : registers[rs2];
    end
end

// -------------------------
// Execute stage (combinational)
// Computes ALU result OR effective address for loads/stores
// -------------------------
always_comb begin
    alu_result = 32'b0;

    if (current_stage == stage_execute) begin
        // Loads/stores: compute effective address = rs1 + imm
        if (opcode == 7'b0000011 || opcode == 7'b0100011) begin
            alu_result = rs1_data + imm;
        end

        // Branches/Jumps: compute branch target (pc + imm) but don't write to rd here
        else if (opcode == 7'b1100011 || opcode == 7'b1101111 || opcode == 7'b1100111) begin
            alu_result = pc + imm;  // For JAL/JALR writeback (return address)
        end

        // R-type
        else if (opcode == 7'b0110011) begin
            case ({funct7, funct3})
                {7'b0000000, 3'b000}: alu_result = rs1_data + rs2_data;                         // add
                {7'b0100000, 3'b000}: alu_result = rs1_data - rs2_data;                         // sub
                {7'b0000000, 3'b001}: alu_result = rs1_data << rs2_data[4:0];                   // sll
                {7'b0000000, 3'b010}: alu_result = ($signed(rs1_data) < $signed(rs2_data)) ? 1 : 0; // slt
                {7'b0000000, 3'b011}: alu_result = (rs1_data < rs2_data) ? 1 : 0;               // sltu
                {7'b0000000, 3'b100}: alu_result = rs1_data ^ rs2_data;                         // xor
                {7'b0000000, 3'b101}: alu_result = rs1_data >> rs2_data[4:0];                   // srl
                {7'b0100000, 3'b101}: alu_result = $signed(rs1_data) >>> rs2_data[4:0];         // sra
                {7'b0000000, 3'b110}: alu_result = rs1_data | rs2_data;                         // or
                {7'b0000000, 3'b111}: alu_result = rs1_data & rs2_data;                         // and
                default:              alu_result = 32'b0;
            endcase
        end

        // I-type OP-IMM
        else if (opcode == 7'b0010011) begin
            case (funct3)
                3'b000: alu_result = rs1_data + imm;                                // addi
                3'b001: alu_result = rs1_data << imm[4:0];                          // slli
                3'b010: alu_result = ($signed(rs1_data) < $signed(imm)) ? 1 : 0;    // slti
                3'b011: alu_result = (rs1_data < $unsigned(imm)) ? 1 : 0;           // sltiu
                3'b100: alu_result = rs1_data ^ imm;                                // xori
                3'b101: alu_result = (imm[10]) ? ($signed(rs1_data) >>> imm[4:0])   // srai
                                              : (rs1_data >> imm[4:0]);             // srli
                3'b110: alu_result = rs1_data | imm;                                // ori
                3'b111: alu_result = rs1_data & imm;                                // andi
                default: alu_result = 32'b0;
            endcase
        end

        // U-type
        else if (opcode == 7'b0110111) begin
            alu_result = imm;                   // lui
        end else if (opcode == 7'b0010111) begin
            alu_result = pc + imm;              // auipc
        end
        else begin
            alu_result = 32'b0;
        end
    end
end

// -------------------------
// Data memory request (combinational)
// Driven only during MEM stage for load/store
// -------------------------
always_comb begin
    data_mem_req.addr     = alu_result_mem;
    data_mem_req.data     = 32'b0;
    data_mem_req.valid    = 1'b0;
    data_mem_req.do_read  = 4'b0000;
    data_mem_req.do_write = 4'b0000;

    if (current_stage == stage_mem) begin
        if (opcode_mem == 7'b0000011) begin
            // LOAD: always read a full 32-bit word; we'll select byte/half in core
            data_mem_req.valid   = 1'b1;
            data_mem_req.do_read = 4'b1111;
        end else if (opcode_mem == 7'b0100011) begin
            // STORE: set byte enables and pack data into the right lane(s)
            logic [1:0] a;
            a = alu_result_mem[1:0];

            data_mem_req.valid = 1'b1;

            case (funct3_mem)
                3'b000: begin // SB
                    case (a)
                        2'd0: begin data_mem_req.do_write = 4'b0001; data_mem_req.data = {24'b0, rs2_data_mem[7:0]}; end
                        2'd1: begin data_mem_req.do_write = 4'b0010; data_mem_req.data = {16'b0, rs2_data_mem[7:0], 8'b0}; end
                        2'd2: begin data_mem_req.do_write = 4'b0100; data_mem_req.data = {8'b0,  rs2_data_mem[7:0], 16'b0}; end
                        2'd3: begin data_mem_req.do_write = 4'b1000; data_mem_req.data = {rs2_data_mem[7:0], 24'b0}; end
                    endcase
                end

                3'b001: begin // SH
                    if (a[0] == 1'b0) begin
                        if (a[1] == 1'b0) begin
                            data_mem_req.do_write = 4'b0011; // bytes 0,1
                            data_mem_req.data     = {16'b0, rs2_data_mem[15:0]};
                        end else begin
                            data_mem_req.do_write = 4'b1100; // bytes 2,3
                            data_mem_req.data     = {rs2_data_mem[15:0], 16'b0};
                        end
                    end else begin
                        // misaligned halfword store: not handled (could trap in a full design)
                        data_mem_req.do_write = 4'b0000;
                        data_mem_req.valid    = 1'b0;
                    end
                end

                3'b010: begin // SW
                    if (a == 2'b00) begin
                        data_mem_req.do_write = 4'b1111;
                        data_mem_req.data     = rs2_data_mem;
                    end else begin
                        // misaligned word store: not handled
                        data_mem_req.do_write = 4'b0000;
                        data_mem_req.valid    = 1'b0;
                    end
                end

                default: begin
                    data_mem_req.valid    = 1'b0;
                    data_mem_req.do_write = 4'b0000;
                end
            endcase
        end
    end
end

// -------------------------
// MEM-stage load extraction (combinational)
// -------------------------
always_comb begin
    load_data_mem = data_mem_rsp.data;

    if (current_stage == stage_mem && opcode_mem == 7'b0000011) begin
        logic [1:0] a;
        logic [7:0]  b;
        logic [15:0] h;

        a = alu_result_mem[1:0];

        // little-endian byte selection from 32-bit word
        case (a)
            2'd0: b = data_mem_rsp.data[7:0];
            2'd1: b = data_mem_rsp.data[15:8];
            2'd2: b = data_mem_rsp.data[23:16];
            default: b = data_mem_rsp.data[31:24];
        endcase

        // halfword selection (aligned to 2 bytes)
        if (a[1] == 1'b0) h = data_mem_rsp.data[15:0];
        else             h = data_mem_rsp.data[31:16];

        case (funct3_mem)
            3'b000: load_data_mem = {{24{b[7]}}, b};   // LB (sign-extend)
            3'b001: load_data_mem = {{16{h[15]}}, h};  // LH (sign-extend)
            3'b010: load_data_mem = data_mem_rsp.data; // LW
            3'b100: load_data_mem = {24'b0, b};        // LBU (zero-extend)
            3'b101: load_data_mem = {16'b0, h};        // LHU (zero-extend)
            default: load_data_mem = 32'b0;
        endcase
    end
end

// -------------------------
// WB mux (combinational)
// -------------------------
always_comb begin
    wb_value = alu_result_wb;
    wb_we    = 1'b0;

    // JAL/JALR write back return address (pc+4)
    if (opcode_wb == 7'b1101111 || (opcode_wb == 7'b1100111 && funct3_ex == 3'b000)) begin
        wb_we    = (rd_wb != 5'd0);  // x0 destination doesn't write
        wb_value = alu_result_wb;
    end
    
    // Default: write back for ALU-type ops
    else if (opcode_wb == 7'b0110011 || opcode_wb == 7'b0010011 ||
             opcode_wb == 7'b0110111 || opcode_wb == 7'b0010111) begin
        wb_we    = (rd_wb != 5'd0);
        wb_value = alu_result_wb;
    end

    // Loads write back memory data
    if (opcode_wb == 7'b0000011) begin
        wb_we    = (rd_wb != 5'd0);
        wb_value = load_data_mem;  // Use captured load data
    end

    // Stores and branches do not write back
    if (opcode_wb == 7'b0100011 || opcode_wb == 7'b1100011) begin
        wb_we = 1'b0;
    end
end

// -------------------------
// Sequential stage machine
// -------------------------
always_ff @(posedge clk) begin
    if (reset) begin
        current_stage  <= stage_fetch;
        pc             <= reset_pc;

        rd_ex          <= 5'b0;
        rd_mem         <= 5'b0;
        rd_wb          <= 5'b0;

        opcode_ex      <= 7'b0;
        opcode_mem     <= 7'b0;
        opcode_wb      <= 7'b0;

        funct3_ex      <= 3'b0;
        funct3_mem     <= 3'b0;

        rs2_data_ex    <= 32'b0;
        rs2_data_mem   <= 32'b0;

        alu_result_mem <= 32'b0;
        alu_result_wb  <= 32'b0;

        for (int i = 0; i < 32; i++) registers[i] <= 32'b0;
    end else begin
        case (current_stage)

            stage_fetch: begin
                current_stage <= stage_decode;
            end

            stage_decode: begin
                current_stage <= stage_execute;

                rd_ex       <= rd;
                opcode_ex   <= opcode;
                funct3_ex   <= funct3;
                rs2_data_ex <= rs2_data;
            end

            stage_execute: begin
                // *** BRANCH/JUMP RESOLUTION: Update PC immediately if taken ***
                if (take_branch || take_jump) begin
                    pc <= next_pc;
                    current_stage <= stage_fetch;  // Go back to fetch immediately
                end else begin
                    current_stage  <= stage_mem;

                    rd_mem         <= rd_ex;
                    opcode_mem     <= opcode_ex;
                    funct3_mem     <= funct3_ex;
                    rs2_data_mem   <= rs2_data_ex;

                    alu_result_mem <= alu_result; // ALU result OR effective address
                end
            end

            stage_mem: begin
                // For loads/stores, wait for memory response (simple ready/valid)
                if (opcode_mem == 7'b0000011 || opcode_mem == 7'b0100011) begin
                    if (!data_mem_rsp.valid) begin
                        current_stage <= stage_mem; // stall
                    end else begin
                        current_stage <= stage_writeback;

                        rd_wb         <= rd_mem;
                        opcode_wb     <= opcode_mem;

                        // For loads, capture extended load value into alu_result_wb so WB path is simple
                        if (opcode_mem == 7'b0000011) alu_result_wb <= load_data_mem;
                        else                           alu_result_wb <= 32'b0; // store has no WB value
                    end
                end else begin
                    // Non-memory ops just pass through
                    current_stage <= stage_writeback;
                    rd_wb         <= rd_mem;
                    opcode_wb     <= opcode_mem;
                    alu_result_wb <= alu_result_mem;
                end
            end

            stage_writeback: begin
                if (wb_we && (rd_wb != 5'd0)) begin
                    registers[rd_wb] <= wb_value;
                end

                // *** SEQUENTIAL PC UPDATE ***
                if (!take_branch && !take_jump) begin
                    pc <= pc + 4;
                end
                current_stage <= stage_fetch;
            end

            default: begin
                current_stage <= stage_fetch;
            end
        endcase
    end
end

endmodule
`endif
