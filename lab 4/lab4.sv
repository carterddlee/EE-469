`ifndef _core_v
`define _core_v
`include "system.sv"
`include "base.sv"
`include "memory_io.sv"
`include "memory.sv"
`include "lab1.sv"

module core(
    input  logic                      clk,
    input  logic                      reset,
    input  logic [`word_address_size-1:0] reset_pc,
    output memory_io_req               inst_mem_req,
    input  memory_io_rsp               inst_mem_rsp,
    output memory_io_req               data_mem_req,
    input  memory_io_rsp               data_mem_rsp
);

typedef enum logic [2:0] {
    stage_fetch      = 3'd0,
    stage_decode     = 3'd1,
    stage_execute    = 3'd2,
    stage_mem        = 3'd3,
    stage_writeback  = 3'd4
} stage_t;

stage_t current_stage;

// Architectural state
word pc;
logic [31:0] registers [31:0];

// Decode
logic [31:0] instruction;
logic [4:0]  rd, rs1, rs2;
logic signed [31:0] imm;
logic [6:0]  opcode;
logic [2:0]  funct3;
logic [6:0]  funct7;
logic [31:0] rs1_data, rs2_data;

// EX latches
logic [6:0]   opcode_ex;
logic [2:0]   funct3_ex;
logic [6:0]   funct7_ex;
logic [4:0]   rd_ex;
logic [31:0]  rs1_ex_val, rs2_ex_val;
logic signed [31:0] imm_ex;
word          pc_ex;

// MEM/WB latches
logic [6:0]  opcode_mem, opcode_wb;
logic [2:0]  funct3_mem;
logic [4:0]  rd_mem, rd_wb;

logic [31:0] rs2_data_mem;

logic [31:0] alu_result;
logic [31:0] alu_result_mem, alu_result_wb;

// Load data pipeline
logic [31:0] load_data_mem;
logic [31:0] load_data_wb;

// WB
logic [31:0] wb_value;
logic        wb_we;

// Control flow (EX)
logic take_branch, take_jump;
logic [31:0] next_pc_ex;

// Flag: EX already redirected PC, so WB must not do pc+4
logic ex_took_cf;

// Temps (module scope)
logic [1:0]  a;
logic [7:0]  b;
logic [15:0] h;

// -------------------------
// Instruction memory request
// -------------------------
always @* begin
    inst_mem_req.addr     = pc;
    inst_mem_req.data     = 32'b0;
    inst_mem_req.valid    = 1'b0;
    inst_mem_req.do_read  = 4'b0000;
    inst_mem_req.do_write = 4'b0000;

    if (current_stage == stage_fetch) begin
        inst_mem_req.valid   = 1'b1;
        inst_mem_req.do_read = 4'b1111;
    end
end

// -------------------------
// Decode stage (combinational)
// -------------------------
always @* begin
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
// -------------------------
always @* begin
    alu_result  = 32'b0;
    take_branch = 1'b0;
    take_jump   = 1'b0;
    next_pc_ex  = pc_ex + 32'd4;

    if (current_stage == stage_execute) begin
        // LOAD/STORE effective address
        if (opcode_ex == 7'b0000011 || opcode_ex == 7'b0100011) begin
            alu_result = rs1_ex_val + imm_ex;
        end

        // BRANCH
        else if (opcode_ex == 7'b1100011) begin
            case (funct3_ex)
                3'b000: take_branch = (rs1_ex_val == rs2_ex_val);                    // BEQ
                3'b001: take_branch = (rs1_ex_val != rs2_ex_val);                    // BNE
                3'b100: take_branch = ($signed(rs1_ex_val) < $signed(rs2_ex_val));   // BLT
                3'b101: take_branch = ($signed(rs1_ex_val) >= $signed(rs2_ex_val));  // BGE
                3'b110: take_branch = (rs1_ex_val < rs2_ex_val);                     // BLTU
                3'b111: take_branch = (rs1_ex_val >= rs2_ex_val);                    // BGEU
                default: take_branch = 1'b0;
            endcase

            if (take_branch) next_pc_ex = pc_ex + imm_ex;
        end

        // JAL
        else if (opcode_ex == 7'b1101111) begin
            take_jump  = 1'b1;
            next_pc_ex = pc_ex + imm_ex;
            alu_result = pc_ex + 32'd4; // link = PC+4 [web:29]
        end

        // JALR (funct3=000), target = (rs1+imm) & ~1 [web:56]
        else if (opcode_ex == 7'b1100111 && funct3_ex == 3'b000) begin
            take_jump  = 1'b1;
            next_pc_ex = ((rs1_ex_val + imm_ex) & 32'hffff_fffe);
            alu_result = pc_ex + 32'd4; // link = PC+4 [web:29]
        end

        // R-type
        else if (opcode_ex == 7'b0110011) begin
            case ({funct7_ex, funct3_ex})
                {7'b0000000, 3'b000}: alu_result = rs1_ex_val + rs2_ex_val;                         // add
                {7'b0100000, 3'b000}: alu_result = rs1_ex_val - rs2_ex_val;                         // sub
                {7'b0000000, 3'b001}: alu_result = rs1_ex_val << rs2_ex_val[4:0];                   // sll
                {7'b0000000, 3'b010}: alu_result = ($signed(rs1_ex_val) < $signed(rs2_ex_val)) ? 32'd1 : 32'd0; // slt
                {7'b0000000, 3'b011}: alu_result = (rs1_ex_val < rs2_ex_val) ? 32'd1 : 32'd0;       // sltu
                {7'b0000000, 3'b100}: alu_result = rs1_ex_val ^ rs2_ex_val;                         // xor
                {7'b0000000, 3'b101}: alu_result = rs1_ex_val >> rs2_ex_val[4:0];                   // srl
                {7'b0100000, 3'b101}: alu_result = $signed(rs1_ex_val) >>> rs2_ex_val[4:0];         // sra
                {7'b0000000, 3'b110}: alu_result = rs1_ex_val | rs2_ex_val;                         // or
                {7'b0000000, 3'b111}: alu_result = rs1_ex_val & rs2_ex_val;                         // and
                default:              alu_result = 32'b0;
            endcase
        end

        // I-type OP-IMM
        else if (opcode_ex == 7'b0010011) begin
            case (funct3_ex)
                3'b000: alu_result = rs1_ex_val + imm_ex;                                // addi
                3'b001: alu_result = rs1_ex_val << imm_ex[4:0];                          // slli
                3'b010: alu_result = ($signed(rs1_ex_val) < $signed(imm_ex)) ? 32'd1 : 32'd0; // slti
                3'b011: alu_result = (rs1_ex_val < $unsigned(imm_ex)) ? 32'd1 : 32'd0;   // sltiu
                3'b100: alu_result = rs1_ex_val ^ imm_ex;                                // xori
                3'b101: alu_result = (imm_ex[10]) ? ($signed(rs1_ex_val) >>> imm_ex[4:0])
                                                  : (rs1_ex_val >> imm_ex[4:0]);         // srai/srli
                3'b110: alu_result = rs1_ex_val | imm_ex;                                // ori
                3'b111: alu_result = rs1_ex_val & imm_ex;                                // andi
                default: alu_result = 32'b0;
            endcase
        end

        // U-type
        else if (opcode_ex == 7'b0110111) begin
            alu_result = imm_ex;                    // lui
        end else if (opcode_ex == 7'b0010111) begin
            alu_result = pc_ex + imm_ex;            // auipc
        end
    end
end

// -------------------------
// Data memory request (MEM stage)
// -------------------------
always @* begin
    data_mem_req.addr     = alu_result_mem;
    data_mem_req.data     = 32'b0;
    data_mem_req.valid    = 1'b0;
    data_mem_req.do_read  = 4'b0000;
    data_mem_req.do_write = 4'b0000;

    a = alu_result_mem[1:0];

    if (current_stage == stage_mem) begin
        if (opcode_mem == 7'b0000011) begin
            data_mem_req.valid   = 1'b1;
            data_mem_req.do_read = 4'b1111;
        end else if (opcode_mem == 7'b0100011) begin
            data_mem_req.valid = 1'b1;
            case (funct3_mem)
                3'b000: begin // SB
                    case (a)
                        2'd0: begin data_mem_req.do_write = 4'b0001; data_mem_req.data = {24'b0, rs2_data_mem[7:0]}; end
                        2'd1: begin data_mem_req.do_write = 4'b0010; data_mem_req.data = {16'b0, rs2_data_mem[7:0], 8'b0}; end
                        2'd2: begin data_mem_req.do_write = 4'b0100; data_mem_req.data = {8'b0,  rs2_data_mem[7:0], 16'b0}; end
                        default: begin data_mem_req.do_write = 4'b1000; data_mem_req.data = {rs2_data_mem[7:0], 24'b0}; end
                    endcase
                end
                3'b001: begin // SH (aligned)
                    if (a[0] == 1'b0) begin
                        if (a[1] == 1'b0) begin
                            data_mem_req.do_write = 4'b0011;
                            data_mem_req.data     = {16'b0, rs2_data_mem[15:0]};
                        end else begin
                            data_mem_req.do_write = 4'b1100;
                            data_mem_req.data     = {rs2_data_mem[15:0], 16'b0};
                        end
                    end else begin
                        data_mem_req.valid = 1'b0;
                    end
                end
                3'b010: begin // SW (aligned)
                    if (a == 2'b00) begin
                        data_mem_req.do_write = 4'b1111;
                        data_mem_req.data     = rs2_data_mem;
                    end else begin
                        data_mem_req.valid = 1'b0;
                    end
                end
                default: data_mem_req.valid = 1'b0;
            endcase
        end
    end
end

// -------------------------
// MEM-stage load extraction
// -------------------------
always @* begin
    load_data_mem = data_mem_rsp.data;

    a = alu_result_mem[1:0];
    b = 8'b0;
    h = 16'b0;

    if (current_stage == stage_mem && opcode_mem == 7'b0000011) begin
        case (a)
            2'd0: b = data_mem_rsp.data[7:0];
            2'd1: b = data_mem_rsp.data[15:8];
            2'd2: b = data_mem_rsp.data[23:16];
            default: b = data_mem_rsp.data[31:24];
        endcase

        if (a[1] == 1'b0) h = data_mem_rsp.data[15:0];
        else             h = data_mem_rsp.data[31:16];

        case (funct3_mem)
            3'b000: load_data_mem = {{24{b[7]}}, b};   // LB
            3'b001: load_data_mem = {{16{h[15]}}, h};  // LH
            3'b010: load_data_mem = data_mem_rsp.data; // LW
            3'b100: load_data_mem = {24'b0, b};        // LBU
            3'b101: load_data_mem = {16'b0, h};        // LHU
            default: load_data_mem = 32'b0;
        endcase
    end
end

// -------------------------
// WB mux
// -------------------------
always @* begin
    wb_value = 32'b0;
    wb_we    = 1'b0;

    if (opcode_wb == 7'b0000011) begin
        wb_we    = (rd_wb != 5'd0);
        wb_value = load_data_wb;
    end else if (opcode_wb == 7'b1101111 || opcode_wb == 7'b1100111 ||
                 opcode_wb == 7'b0110011 || opcode_wb == 7'b0010011 ||
                 opcode_wb == 7'b0110111 || opcode_wb == 7'b0010111) begin
        wb_we    = (rd_wb != 5'd0);
        wb_value = alu_result_wb;
    end
end

// -------------------------
// Sequential stage machine
// -------------------------
integer i;
always_ff @(posedge clk) begin
    if (reset) begin
        current_stage <= stage_fetch;
        pc            <= reset_pc;
        ex_took_cf    <= 1'b0;

        opcode_ex  <= 7'd0; funct3_ex <= 3'd0; funct7_ex <= 7'd0; rd_ex <= 5'd0;
        rs1_ex_val <= 32'd0; rs2_ex_val <= 32'd0; imm_ex <= '0; pc_ex <= '0;

        opcode_mem <= 7'd0; funct3_mem <= 3'd0; rd_mem <= 5'd0;
        rs2_data_mem <= 32'd0; alu_result_mem <= 32'd0;

        opcode_wb <= 7'd0; rd_wb <= 5'd0;
        alu_result_wb <= 32'd0; load_data_wb <= 32'd0;

        for (i = 0; i < 32; i++) registers[i] <= 32'd0;
    end else begin
        case (current_stage)
            stage_fetch: begin
                current_stage <= stage_decode;
            end

            stage_decode: begin
                current_stage <= stage_execute;

                opcode_ex  <= opcode;
                funct3_ex  <= funct3;
                funct7_ex  <= funct7;
                rd_ex      <= rd;
                rs1_ex_val <= rs1_data;
                rs2_ex_val <= rs2_data;
                imm_ex     <= imm;
                pc_ex      <= pc;
            end

            stage_execute: begin
                if (take_branch || take_jump) begin
                    pc         <= next_pc_ex;
                    ex_took_cf <= 1'b1;
                    current_stage <= stage_fetch;
                end else begin
                    ex_took_cf     <= 1'b0;
                    current_stage  <= stage_mem;

                    rd_mem         <= rd_ex;
                    opcode_mem     <= opcode_ex;
                    funct3_mem     <= funct3_ex;
                    rs2_data_mem   <= rs2_ex_val;
                    alu_result_mem <= alu_result;
                end
            end

            stage_mem: begin
                if (opcode_mem == 7'b0000011 || opcode_mem == 7'b0100011) begin
                    if (!data_mem_rsp.valid) begin
                        current_stage <= stage_mem;
                    end else begin
                        current_stage <= stage_writeback;

                        rd_wb     <= rd_mem;
                        opcode_wb <= opcode_mem;

                        if (opcode_mem == 7'b0000011) begin
                            load_data_wb  <= load_data_mem;
                            alu_result_wb <= 32'd0;
                        end else begin
                            alu_result_wb <= 32'd0;
                        end
                    end
                end else begin
                    current_stage <= stage_writeback;

                    rd_wb        <= rd_mem;
                    opcode_wb    <= opcode_mem;
                    alu_result_wb<= alu_result_mem;
                end
            end

            stage_writeback: begin
                if (wb_we) registers[rd_wb] <= wb_value;

                // Important: don't clobber redirected pc
                if (!ex_took_cf) pc <= pc + 32'd4;
                ex_took_cf <= 1'b0;

                current_stage <= stage_fetch;
            end

            default: current_stage <= stage_fetch;
        endcase
    end
end

endmodule
`endif
