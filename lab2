`ifndef _core_v
`define _core_v
`include "system.sv"
`include "base.sv"
`include "memory_io.sv"
`include "memory.sv"
`include "lab1.sv"

module core(
    input logic       clk
    ,input logic      reset
    ,input logic      [`word_address_size-1:0] reset_pc
    ,output memory_io_req   inst_mem_req
    ,input  memory_io_rsp   inst_mem_rsp
    ,output memory_io_req   data_mem_req
    ,input  memory_io_rsp   data_mem_rsp
);

typedef enum {
    stage_fetch
    ,stage_decode
    ,stage_execute
    ,stage_mem
    ,stage_writeback
} stage;

stage current_stage;
logic [31:0] registers [31:0];

initial begin
    for (int i = 0; i < 32; i++) begin  
        registers[i] = 32'b0;
    end
end

logic [31:0] instruction;
logic [4:0] rd;
logic [4:0] rs1;
logic [4:0] rs2;
logic signed [31:0] imm;
logic [6:0] opcode;
logic [2:0] funct3;
logic [6:0] funct7;
string mnemonic;

logic [31:0] rs1_data;
logic [31:0] rs2_data;
logic [31:0] alu_result;

// Pipeline registers to preserve values across stages
logic [4:0] rd_ex;           // rd for execute stage
logic [4:0] rd_mem;          // rd for mem stage
logic [4:0] rd_wb;           // rd for writeback stage
logic [31:0] alu_result_mem; // alu_result for mem stage
logic [31:0] alu_result_wb;  // alu_result for writeback stage

word pc;

// Fetch stage
always_comb begin
    if (current_stage == stage_fetch) begin
        inst_mem_req.addr = pc;
        inst_mem_req.do_read = 4'b1111;
        inst_mem_req.valid = 1'b1;
    end else begin
        inst_mem_req.addr = pc;
        inst_mem_req.do_read = 4'b0000;
        inst_mem_req.valid = 1'b0;
    end
    inst_mem_req.do_write = 4'b0000;
    inst_mem_req.data = 32'b0;
end

// Decode stage
always_comb begin
    if (current_stage == stage_decode) begin
        instruction = inst_mem_rsp.data; 
        rd = instruction[11:7];
        rs1 = instruction[19:15];
        rs2 = instruction[24:20];
        funct3 = instruction[14:12];
        funct7 = instruction[31:25];
        opcode = instruction[6:0];
        print_instruction(pc, instruction);  
        imm = find_imm(opcode, instruction);
        rs1_data = (rs1 == 5'b0) ? 32'b0 : registers[rs1]; 
        rs2_data = (rs2 == 5'b0) ? 32'b0 : registers[rs2];
    end 
end

// Execute stage
always_comb begin
    alu_result = 32'b0;
    
    if (current_stage == stage_execute) begin
        // R‑format: opcode == 7'b0110011
        if (opcode == 7'b0110011) begin
            case ({funct7, funct3})
                {7'b0000000, 3'b000}: // add
                    alu_result = rs1_data + rs2_data;
                {7'b0100000, 3'b000}: // sub
                    alu_result = rs1_data - rs2_data;
                {7'b0000000, 3'b001}: // sll
                    alu_result = rs1_data << rs2_data[4:0];
                {7'b0000000, 3'b010}: // slt
                    alu_result = ($signed(rs1_data) < $signed(rs2_data)) ? 32'd1 : 32'd0;
                {7'b0000000, 3'b011}: // sltu
                    alu_result = (rs1_data < rs2_data) ? 32'd1 : 32'd0;
                {7'b0000000, 3'b100}: // xor
                    alu_result = rs1_data ^ rs2_data;
                {7'b0000000, 3'b101}: // srl
                    alu_result = rs1_data >> rs2_data[4:0];
                {7'b0100000, 3'b101}: // sra
                    alu_result = $signed(rs1_data) >>> rs2_data[4:0];
                {7'b0000000, 3'b110}: // or
                    alu_result = rs1_data | rs2_data;
                {7'b0000000, 3'b111}: // and
                    alu_result = rs1_data & rs2_data;
                default:
                    alu_result = 32'b0;
            endcase

        // I‑format arithmetic/logic + shifts: opcode == 7'b0010011
        end else if (opcode == 7'b0010011) begin
            case (funct3)
                3'b000: // addi
                    alu_result = rs1_data + imm;
                3'b001: // slli
                    alu_result = rs1_data << imm[4:0];
                3'b010: // slti
                    alu_result = ($signed(rs1_data) < $signed(imm)) ? 32'd1 : 32'd0;
                3'b011: // sltiu
                    alu_result = (rs1_data < $unsigned(imm)) ? 32'd1 : 32'd0;
                3'b100: // xori
                    alu_result = rs1_data ^ imm;
                3'b101: begin
                    if (imm[10]) begin // srai
                        alu_result = $signed(rs1_data) >>> imm[4:0];
                    end else begin   // srli
                        alu_result = rs1_data >> imm[4:0];
                    end
                end
                3'b110: // ori
                    alu_result = rs1_data | imm;
                3'b111: // andi
                    alu_result = rs1_data & imm;
                default:
                    alu_result = 32'b0;
            endcase

        // U‑format
        end else if (opcode == 7'b0110111) begin // lui
            alu_result = imm;

        end else if (opcode == 7'b0010111) begin // auipc
            alu_result = pc + imm;

        end else begin
            // unsupported instruction → 0
            alu_result = 32'b0;
        end
    end
end

// Sequential logic
always_ff @(posedge clk) begin
    if (reset) begin
        current_stage <= stage_fetch;
        pc <= reset_pc;
        rd_ex <= 5'b0;
        rd_mem <= 5'b0;
        rd_wb <= 5'b0;
        alu_result_mem <= 32'b0;
        alu_result_wb <= 32'b0;
        for (int i = 0; i < 32; i++) begin
            registers[i] <= 32'b0;
        end
    end else begin
        case (current_stage)
            stage_fetch:
                current_stage <= stage_decode;
            stage_decode: begin
                current_stage <= stage_execute;
                rd_ex <= rd;  // Capture rd for execute stage
            end
            stage_execute: begin
                current_stage <= stage_mem;
                rd_mem <= rd_ex;              // Forward rd to mem
                alu_result_mem <= alu_result; // Forward ALU result to mem
            end
            stage_mem: begin
                current_stage <= stage_writeback;
                rd_wb <= rd_mem;                 // Forward rd to writeback
                alu_result_wb <= alu_result_mem; // Forward ALU result to writeback
            end
            stage_writeback: begin
                // Register writeback using pipeline registers
                if (rd_wb != 5'b0) begin
                    registers[rd_wb] <= alu_result_wb;
                end
                // PC increment
                pc <= pc + 4;
                // Return to fetch
                current_stage <= stage_fetch;
            end
            default: begin
                $display("Should never get here");
                current_stage <= stage_fetch;
            end
        endcase
    end
end

endmodule

`endif
