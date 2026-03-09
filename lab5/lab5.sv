`ifndef _riscv_multicycle
`define _riscv_multicycle

`include "base.sv"
`include "system.sv"
`include "memory_io.sv"
`include "riscv.sv"

module core (
    input  logic                            clk,
    input  logic                            reset,
    input  logic [`word_address_size-1:0]   reset_pc,
    output memory_io_req                    inst_mem_req,
    input  memory_io_rsp                    inst_mem_rsp,
    output memory_io_req                    data_mem_req,
    input  memory_io_rsp                    data_mem_rsp
);

import riscv::*;

typedef struct packed {
    logic [`word_address_size-1:0]  pc;
    instr32                         instr;
    logic                           valid;
} if_id_t;

typedef struct packed {
    logic [`word_address_size-1:0]  pc;
    word                            rd1;
    word                            rd2;
    word                            imm;
    tag                             rs1;
    tag                             rs2;
    tag                             wbs;
    funct3                          f3;
    funct7                          f7;
    opcode_q                        op_q;
    logic                           wbv;
    logic                           valid;
} id_ex_t;

typedef struct packed {
    logic [`word_address_size-1:0]  pc;
    word                            exec_result;
    word                            next_pc;
    word                            rd2;
    tag                             wbs;
    funct3                          f3;
    opcode_q                        op_q;
    logic                           wbv;
    logic                           valid;
} ex_mem_t;

typedef struct packed {
    word    wbd;
    tag     wbs;
    logic   wbv;
    logic   valid;
} mem_wb_t;

if_id_t     if_id,  if_id_next;
id_ex_t     id_ex,  id_ex_next;
ex_mem_t    ex_mem, ex_mem_next;
mem_wb_t    mem_wb, mem_wb_next;

word_address pc;
word_address pc_next;

word reg_file[0:31];

logic stall_fetch;
assign stall_fetch = !inst_mem_rsp.valid;

logic stall_mem;
assign stall_mem = ex_mem.valid &&
                   (ex_mem.op_q == q_load || ex_mem.op_q == q_store) &&
                   !data_mem_rsp.valid;

logic stall;
assign stall = stall_fetch || stall_mem;

assign inst_mem_req.addr     = pc;
assign inst_mem_req.valid    = inst_mem_rsp.ready;
assign inst_mem_req.do_read  = inst_mem_rsp.ready ? 4'b1111 : 4'b0000;
assign inst_mem_req.do_write = 4'b0000;
assign inst_mem_req.data     = 32'b0;
assign inst_mem_req.dummy    = 3'b0;
assign inst_mem_req.user_tag = '0;

assign if_id_next.pc    = stall ? if_id.pc    : pc;
assign if_id_next.instr = stall ? if_id.instr : shuffle_store_data(inst_mem_rsp.data, inst_mem_rsp.addr);
assign if_id_next.valid = stall ? if_id.valid : inst_mem_rsp.valid;

tag          id_rs1, id_rs2, id_wbs;
funct3       id_f3;
funct7       id_f7;
opcode_q     id_op_q;
instr_format id_format;
word         id_imm;
logic        id_wbv;

assign id_rs1    = decode_rs1(if_id.instr);
assign id_rs2    = decode_rs2(if_id.instr);
assign id_wbs    = decode_rd(if_id.instr);
assign id_f3     = decode_funct3(if_id.instr);
assign id_op_q   = decode_opcode_q(if_id.instr);
assign id_format = decode_format(id_op_q);
assign id_imm    = decode_imm(if_id.instr, id_format);
assign id_wbv    = decode_writeback(id_op_q);
assign id_f7     = decode_funct7(if_id.instr, id_format);

assign id_ex_next.pc    = stall ? id_ex.pc    : if_id.pc;
assign id_ex_next.rs1   = stall ? id_ex.rs1   : id_rs1;
assign id_ex_next.rs2   = stall ? id_ex.rs2   : id_rs2;
assign id_ex_next.wbs   = stall ? id_ex.wbs   : id_wbs;
assign id_ex_next.f3    = stall ? id_ex.f3    : id_f3;
assign id_ex_next.f7    = stall ? id_ex.f7    : id_f7;
assign id_ex_next.op_q  = stall ? id_ex.op_q  : id_op_q;
assign id_ex_next.imm   = stall ? id_ex.imm   : id_imm;
assign id_ex_next.wbv   = stall ? id_ex.wbv   : id_wbv;
assign id_ex_next.valid = stall ? id_ex.valid  : if_id.valid;
assign id_ex_next.rd1   = stall ? id_ex.rd1   : ((id_rs1 == `tag_size'd0) ? `word_size'd0 : reg_file[id_rs1]);
assign id_ex_next.rd2   = stall ? id_ex.rd2   : ((id_rs2 == `tag_size'd0) ? `word_size'd0 : reg_file[id_rs2]);

ext_operand  ex_result_comb;
word_address ex_next_pc_comb;

assign ex_result_comb = execute(
    cast_to_ext_operand(id_ex.rd1),
    cast_to_ext_operand(id_ex.rd2),
    cast_to_ext_operand(id_ex.imm),
    id_ex.pc,
    id_ex.op_q,
    id_ex.f3,
    id_ex.f7);

assign ex_next_pc_comb = compute_next_pc(
    cast_to_ext_operand(id_ex.rd1),
    ex_result_comb,
    id_ex.imm,
    id_ex.pc,
    id_ex.op_q,
    id_ex.f3);

assign ex_mem_next.pc          = stall ? ex_mem.pc          : id_ex.pc;
assign ex_mem_next.exec_result = stall ? ex_mem.exec_result : ex_result_comb[`word_size-1:0];
assign ex_mem_next.next_pc     = stall ? ex_mem.next_pc     : ex_next_pc_comb;
assign ex_mem_next.rd2         = stall ? ex_mem.rd2         : id_ex.rd2;
assign ex_mem_next.wbs         = stall ? ex_mem.wbs         : id_ex.wbs;
assign ex_mem_next.f3          = stall ? ex_mem.f3          : id_ex.f3;
assign ex_mem_next.op_q        = stall ? ex_mem.op_q        : id_ex.op_q;
assign ex_mem_next.wbv         = stall ? ex_mem.wbv         : id_ex.wbv;
assign ex_mem_next.valid       = stall ? ex_mem.valid       : id_ex.valid;

assign data_mem_req.addr     = ex_mem.exec_result[`word_address_size-1:0];
assign data_mem_req.valid    = data_mem_rsp.ready && ex_mem.valid &&
                               (ex_mem.op_q == q_store || ex_mem.op_q == q_load);
assign data_mem_req.do_write = (ex_mem.op_q == q_store)
    ? shuffle_store_mask(memory_mask(cast_to_memory_op(ex_mem.f3)), ex_mem.exec_result)
    : 4'b0000;
assign data_mem_req.do_read  = (ex_mem.op_q == q_load)
    ? shuffle_store_mask(memory_mask(cast_to_memory_op(ex_mem.f3)), ex_mem.exec_result)
    : 4'b0000;
assign data_mem_req.data     = shuffle_store_data(ex_mem.rd2, ex_mem.exec_result);
assign data_mem_req.dummy    = 3'b0;
assign data_mem_req.user_tag = '0;

word load_data_latch;
always_ff @(posedge clk) begin
    if (data_mem_rsp.valid)
        load_data_latch <= data_mem_rsp.data;
end

word mem_load_word;
assign mem_load_word = data_mem_rsp.valid ? data_mem_rsp.data : load_data_latch;

word mem_wbd;
assign mem_wbd = (ex_mem.op_q == q_load)
    ? subset_load_data(shuffle_load_data(mem_load_word, ex_mem.exec_result), cast_to_memory_op(ex_mem.f3))
    : ex_mem.exec_result;

assign mem_wb_next.wbd   = stall_mem ? mem_wb.wbd   : mem_wbd;
assign mem_wb_next.wbs   = stall_mem ? mem_wb.wbs   : ex_mem.wbs;
assign mem_wb_next.wbv   = stall_mem ? mem_wb.wbv   : ex_mem.wbv;
assign mem_wb_next.valid = stall_mem ? mem_wb.valid : ex_mem.valid;

always_ff @(posedge clk) begin
    if (mem_wb.valid && mem_wb.wbv && (mem_wb.wbs != `tag_size'd0))
        reg_file[mem_wb.wbs] <= mem_wb.wbd;
end

assign pc_next = stall ? pc : (ex_mem.valid ? ex_mem.next_pc : pc + 4);

always_ff @(posedge clk) begin
    if (reset) begin
        pc     <= reset_pc;
        if_id  <= '0;
        id_ex  <= '0;
        ex_mem <= '0;
        mem_wb <= '0;
    end else begin
        pc     <= pc_next;
        if_id  <= if_id_next;
        id_ex  <= id_ex_next;
        ex_mem <= ex_mem_next;
        mem_wb <= mem_wb_next;
    end
end

endmodule
`endif
