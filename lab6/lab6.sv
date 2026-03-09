`ifndef __lab6_sv
`define __lab5_sv

// modularized pipelined (no hazard/mispredict handling)
`include "riscv.sv"

typedef struct packed {
    bool valid;
    riscv::word pc;
} fetch_set_pc_call_t;

typedef struct packed {
    bool _unused_;
} fetch_set_pc_return_t;

typedef struct packed {
    bool advance;
    bool flush;
} stage_signal_t;

typedef struct packed {
    bool valid;
    riscv::instr32 instruction;
    riscv::word pc;
} fetched_instruction_t;


module fetch(
    input logic       clk
    ,input logic      reset
    ,input logic      [`word_address_size-1:0] reset_pc

    // Stage control signalling
    ,input stage_signal_t       fetch_signal_in

    // Control function interfaces
    ,input fetch_set_pc_call_t    fetch_set_pc_call_in
    ,output fetch_set_pc_return_t  fetch_set_pc_return_out

    // Principle operation data path
    ,output memory_io_req           inst_mem_req
    ,input  memory_io_rsp           inst_mem_rsp
    ,output fetched_instruction_t   fetched_instruction_out
    );

import riscv::*;

word fetch_pc;
bool clear_fetch_stream;
word clear_to_this_pc;
word issued_fetch_pc;
bool issued;
instr32 latched_instruction_read;
bool latched_instruction_valid;
word latched_instruction_pc;

always @(*) begin
    inst_mem_req = memory_io_no_req;

    inst_mem_req.addr = fetch_pc;
    inst_mem_req.do_read[3:0] = 4'b1111;
    inst_mem_req.valid = inst_mem_rsp.ready
                        && fetch_signal_in.advance;
    inst_mem_req.user_tag = 0;

    fetched_instruction_out.valid = latched_instruction_valid;
    fetched_instruction_out.instruction = latched_instruction_read;
    fetched_instruction_out.pc = latched_instruction_pc;

    if (inst_mem_rsp.valid
        && fetch_signal_in.advance
        ) begin
        if (clear_fetch_stream &&
            inst_mem_rsp.addr != clear_to_this_pc) begin
            // discard

        end else begin
            word memory_read;
            memory_read = shuffle_store_data(inst_mem_rsp.data, inst_mem_rsp.addr);
            fetched_instruction_out.valid = true;
            fetched_instruction_out.instruction = memory_read[31:0];
            fetched_instruction_out.pc = inst_mem_rsp.addr;
        end
    end
end

always_ff @(posedge clk) begin
    if (reset) begin
        fetch_pc <= reset_pc;
        latched_instruction_valid <= false;
        clear_fetch_stream <= false;
        clear_to_this_pc <= 0;
    end else begin

        if (inst_mem_rsp.valid) begin
            issued <= false;
            if (clear_fetch_stream
                && inst_mem_rsp.addr != clear_to_this_pc) begin
                // do nthing
            end else begin
                clear_fetch_stream <= false;
                if (fetch_signal_in.advance) begin
                    word memory_read;
                    memory_read = shuffle_store_data(inst_mem_rsp.data, inst_mem_rsp.addr);
                    latched_instruction_pc <= inst_mem_rsp.addr;
                    latched_instruction_read <= memory_read[31:0];
                    latched_instruction_valid <= true;
                end
            end
        end

        if (inst_mem_req.valid) begin
            fetch_pc <= fetch_pc + 4;
        end

        //if (fetched_instruction_out.valid)
        //    $display("PC out: %x rsp.addr=%x", fetched_instruction_out.pc, inst_mem_rsp.addr);

        if (fetch_set_pc_call_in.valid) begin
            //$display("fetch_pc reset to: %x", fetch_set_pc_call_in.pc);
            fetch_pc <= fetch_set_pc_call_in.pc;
            latched_instruction_valid <= false;
            clear_fetch_stream <= true;
            clear_to_this_pc <= fetch_set_pc_call_in.pc;
        end else if (!fetch_signal_in.advance) begin
            // ut oh.  Fetch is only approximate, however
            // so we do our best to guess what is needed
            fetch_pc <= latched_instruction_pc + 4;
            clear_fetch_stream <= true;
            clear_to_this_pc <= latched_instruction_pc + 4;

        end
    end
end

endmodule

typedef struct packed {
    bool valid;
    riscv::tag rs1;
    riscv::tag rs2;
    riscv::word rd1;
    riscv::word rd2;
    riscv::word imm;
    riscv::tag wbs;
    bool wbv;
    riscv::funct3 f3;
    riscv::funct7 f7;
    riscv::opcode_q op_q;
    riscv::instr_format format;
    riscv::instr32 instruction;
    riscv::word pc;    
} decoded_instruction_t;

typedef struct packed {
    bool valid;
    bool wbv;
    riscv::tag wbs;
    riscv::word wbd;    
} writeback_instruction_t;

typedef struct packed {
    bool    valid;
    riscv::word    rd;
    riscv::tag     rs;
} reg_file_bypass_t;

module decode_and_writeback (
    input logic       clk
    ,input logic      reset

    // Stage control signalling
    ,input stage_signal_t   decode_signal_in
    ,input stage_signal_t   execute_signal_in
    ,input stage_signal_t   writeback_signal_in

    // Control function interfaces
    ,output reg_file_bypass_t reg_file_bypass_out

    // Principle operation data path
    ,input fetched_instruction_t fetched_instruction_in
    ,output decoded_instruction_t decoded_instruction_out

    ,input writeback_instruction_t writeback_instruction_in
    );
    
import riscv::*;

word    reg_file[0:31];

word    reg_file_bypass_rd;
tag     reg_file_bypass_rs;
bool    reg_file_bypass_valid;

always_comb begin
    reg_file_bypass_out.valid = reg_file_bypass_valid;
    reg_file_bypass_out.rd = reg_file_bypass_rd;
    reg_file_bypass_out.rs = reg_file_bypass_rs;
end

word comb_rd1;
word comb_rd2;
tag  comb_rs1;
tag  comb_rs2;

always_comb begin
    comb_rs1 = decode_rs1(fetched_instruction_in.instruction);
    comb_rs2 = decode_rs2(fetched_instruction_in.instruction);
    comb_rd1 = reg_file[comb_rs1];
    comb_rd2 = reg_file[comb_rs2];
end

tag last_decode_instruction_rs1;
tag last_decode_instruction_rs2;

initial begin
    for (int i = 0; i < 32; i++)
        reg_file[i] = `word_size'd0;
end

always_ff @(posedge clk) begin
    word    wbd;
    tag     rs1;
    tag     rs2;
    opcode_q op_q;
    instr_format format;
    tag read_reg_rs1;
    tag read_reg_rs2;
    
    rs1 = decode_rs1(fetched_instruction_in.instruction);
    rs2 = decode_rs2(fetched_instruction_in.instruction);
    op_q = decode_opcode_q(fetched_instruction_in.instruction);
    format = decode_format(op_q);

    if (reset)
        reg_file_bypass_valid <= false;

    if (reset || decode_signal_in.flush) begin
        decoded_instruction_out <= {($bits(decoded_instruction_t)){1'b0}};
        decoded_instruction_out.valid <= false;
    end else begin
        if (decode_signal_in.advance
            && fetched_instruction_in.valid) begin
            decoded_instruction_out.valid <= true;
            decoded_instruction_out.rs1 <= rs1;
            decoded_instruction_out.rs2 <= rs2;
            decoded_instruction_out.wbs <= decode_rd(fetched_instruction_in.instruction);
            decoded_instruction_out.f3 <= decode_funct3(fetched_instruction_in.instruction);
            decoded_instruction_out.op_q <= op_q;
            decoded_instruction_out.format <= format;
            decoded_instruction_out.imm <= decode_imm(fetched_instruction_in.instruction, format);
            decoded_instruction_out.wbv <= decode_writeback(op_q);
            decoded_instruction_out.f7 <= decode_funct7(fetched_instruction_in.instruction, format);
            decoded_instruction_out.pc <= fetched_instruction_in.pc;
            decoded_instruction_out.instruction <= fetched_instruction_in.instruction;
        end else begin //if (execute_signal_in.advance) begin
            // Either always or only on exec advance is fine.
            decoded_instruction_out <= {($bits(decoded_instruction_t)){1'b0}};
            decoded_instruction_out.valid <= false;
        end

    end

if (decode_signal_in.advance && fetched_instruction_in.valid) begin
    decoded_instruction_out.rd1 <= comb_rd1;
    decoded_instruction_out.rd2 <= comb_rd2;
end

    // Write back
    if (!reset
        && writeback_signal_in.advance
        && writeback_instruction_in.valid
        && writeback_instruction_in.wbv) begin
        reg_file[writeback_instruction_in.wbs] <= writeback_instruction_in.wbd;
        reg_file_bypass_rs <= writeback_instruction_in.wbs;
        reg_file_bypass_rd <= writeback_instruction_in.wbd;
        reg_file_bypass_valid <= true;
    end

end
endmodule


typedef struct packed {
    bool valid;
    riscv::word rd1;
    riscv::word rd2;
    riscv::tag rs1;
    riscv::tag rs2;
    riscv::funct3 f3;
    riscv::opcode_q op_q;
    writeback_instruction_t writeback_instruction;
} executed_instruction_t;

typedef struct packed {
    bool fetch_mispredict;
    bool wrong_pc;
    riscv::word correct_pc;    
}   pc_control_t;

module execute (
    input logic       clk
    ,input logic      reset
    ,input riscv::word       reset_pc
    // Stage control signalling
    ,input stage_signal_t   execute_signal_in
    ,input stage_signal_t   memory_signal_in

    // For detecting a branch mispredict
    ,input fetched_instruction_t fetched_instruction_in

    // For bypassing
    ,input reg_file_bypass_t reg_file_bypass_in
    ,input executed_instruction_t executed_instruction_in
    ,input writeback_instruction_t writeback_instruction_in

    // Datapath proper
    ,input decoded_instruction_t  decoded_instruction_in
    ,output executed_instruction_t executed_instruction_out

    ,output pc_control_t pc_control_out
    );

import riscv::*;

word pc;
ext_operand exec_result_comb;
word next_pc_comb;
word exec_bypassed_rd1_comb;
word exec_bypassed_rd2_comb;

always @(*) begin
    word rd1;
    word rd2;

    // TODO: Implement bypass logic
// Bypass priority: EX/MEM forwarding > MEM/WB forwarding > WB forwarding > register file
// RS1 forwarding
if (decoded_instruction_in.rs1 == 5'd0) begin
    rd1 = `word_size'd0;
end else if (executed_instruction_in.valid
    && executed_instruction_in.writeback_instruction.wbv
    && executed_instruction_in.writeback_instruction.valid
    && executed_instruction_in.rs1 != 5'd0
    && executed_instruction_in.writeback_instruction.wbs == decoded_instruction_in.rs1) begin
    rd1 = executed_instruction_in.writeback_instruction.wbd;
end else if (writeback_instruction_in.valid
    && writeback_instruction_in.wbv
    && writeback_instruction_in.wbs != 5'd0
    && writeback_instruction_in.wbs == decoded_instruction_in.rs1) begin
    rd1 = writeback_instruction_in.wbd;
end else if (reg_file_bypass_in.valid
    && reg_file_bypass_in.rs != 5'd0
    && reg_file_bypass_in.rs == decoded_instruction_in.rs1) begin
    rd1 = reg_file_bypass_in.rd;
end else begin
    rd1 = decoded_instruction_in.rd1;
end

// RS2 forwarding
if (decoded_instruction_in.rs2 == 5'd0) begin
    rd2 = `word_size'd0;
end else if (executed_instruction_in.valid
    && executed_instruction_in.writeback_instruction.wbv
    && executed_instruction_in.writeback_instruction.valid
    && executed_instruction_in.writeback_instruction.wbs != 5'd0
    && executed_instruction_in.writeback_instruction.wbs == decoded_instruction_in.rs2) begin
    rd2 = executed_instruction_in.writeback_instruction.wbd;
end else if (writeback_instruction_in.valid
    && writeback_instruction_in.wbv
    && writeback_instruction_in.wbs != 5'd0
    && writeback_instruction_in.wbs == decoded_instruction_in.rs2) begin
    rd2 = writeback_instruction_in.wbd;
end else if (reg_file_bypass_in.valid
    && reg_file_bypass_in.rs != 5'd0
    && reg_file_bypass_in.rs == decoded_instruction_in.rs2) begin
    rd2 = reg_file_bypass_in.rd;
end else begin
    rd2 = decoded_instruction_in.rd2;
end

    exec_bypassed_rd1_comb = rd1;
    exec_bypassed_rd2_comb = rd2;

    exec_result_comb = execute(
        cast_to_ext_operand(rd1),
        cast_to_ext_operand(rd2),
        cast_to_ext_operand(decoded_instruction_in.imm),
        decoded_instruction_in.pc,
        decoded_instruction_in.op_q,
        decoded_instruction_in.f3,
        decoded_instruction_in.f7);

    // "ground truth" for the next PC comes from this calculation.
    next_pc_comb = compute_next_pc(
        cast_to_ext_operand(rd1),
        exec_result_comb,
        decoded_instruction_in.imm,
        decoded_instruction_in.pc,
        decoded_instruction_in.op_q,
        decoded_instruction_in.f3);

    pc_control_out = {($bits(pc_control_t)){1'b0}};
    pc_control_out.fetch_mispredict = false;

    if (decoded_instruction_in.valid
        && next_pc_comb != fetched_instruction_in.pc) begin
        pc_control_out.fetch_mispredict = true;
        pc_control_out.correct_pc = next_pc_comb;
    end

    if (decoded_instruction_in.valid
        && decoded_instruction_in.pc != pc) begin
        pc_control_out.wrong_pc = true;
        pc_control_out.correct_pc = pc;
    end

end


always_ff @(posedge clk) begin
    if (reset) begin
        executed_instruction_out.valid <= false;
        pc <= reset_pc;
    end else begin
        if (decoded_instruction_in.valid && execute_signal_in.advance) begin
            if (decoded_instruction_in.pc == pc) begin
                executed_instruction_out.valid <= true;
                pc <= next_pc_comb;
            end else
                // Flush this instruction out
                executed_instruction_out.valid <= false;

            executed_instruction_out.rd1 <= exec_bypassed_rd1_comb;
            executed_instruction_out.rd2 <= exec_bypassed_rd2_comb;
            executed_instruction_out.rs1 <= decoded_instruction_in.rs1;
            executed_instruction_out.rs2 <= decoded_instruction_in.rs2;
            executed_instruction_out.writeback_instruction.wbs <= decoded_instruction_in.wbs;
            executed_instruction_out.writeback_instruction.wbv <= decoded_instruction_in.wbv;
            executed_instruction_out.writeback_instruction.wbd <= exec_result_comb[`word_size-1:0];
            executed_instruction_out.writeback_instruction.valid <= decoded_instruction_in.valid;
            executed_instruction_out.f3 <= decoded_instruction_in.f3;
            executed_instruction_out.op_q <= decoded_instruction_in.op_q;
        end else if (memory_signal_in.advance) begin
            executed_instruction_out <= {($bits(executed_instruction_t)){1'b0}};
            executed_instruction_out.valid <= false;
        end
    end
end

endmodule


typedef struct packed {
    bool valid;
    riscv::word pc;
    riscv::word exec_result;
    riscv::funct3 f3;
    riscv::opcode_q op_q;
    writeback_instruction_t writeback_instruction;
} memory_instruction_t;


module memory (
    input logic       clk
    ,input logic      reset

    // Stage control signalling
    ,input stage_signal_t   memory_signal_in
    ,input stage_signal_t   writeback_signal_in

    // For bypassing
    ,input reg_file_bypass_t reg_file_bypass_in
    ,input writeback_instruction_t writeback_instruction_in

    // Datapath proper
    ,output memory_io_req   data_mem_req
    ,input  memory_io_rsp   data_mem_rsp
    ,input executed_instruction_t  executed_instruction_in
    ,output memory_instruction_t memory_instruction_out

    );

import riscv::*;


always @(*) begin
    word  rd2;
    word  rd1;

    rd1 = executed_instruction_in.rd1;
    rd2 = executed_instruction_in.rd2;
    data_mem_req = memory_io_no_req;

    if (memory_signal_in.advance && executed_instruction_in.valid
        && (executed_instruction_in.op_q == q_store
         || executed_instruction_in.op_q == q_load
         || executed_instruction_in.op_q == q_amo)) begin
        data_mem_req.user_tag = 0;
        if (executed_instruction_in.op_q == q_store) begin
            data_mem_req.addr = executed_instruction_in.writeback_instruction.wbd[`word_address_size - 1:0];
            data_mem_req.valid = true;
            data_mem_req.do_write = shuffle_store_mask(memory_mask(
                cast_to_memory_op(executed_instruction_in.f3)), executed_instruction_in.writeback_instruction.wbd[`word_size - 1:0]);
            data_mem_req.data = shuffle_store_data(rd2, executed_instruction_in.writeback_instruction.wbd[`word_size - 1:0]);
        end
        else if (executed_instruction_in.op_q == q_load) begin  // q_load
            data_mem_req.addr = executed_instruction_in.writeback_instruction.wbd[`word_address_size - 1:0];
            data_mem_req.valid = true;
            data_mem_req.do_read = shuffle_store_mask(memory_mask(
                cast_to_memory_op(executed_instruction_in.f3)), executed_instruction_in.writeback_instruction.wbd[`word_size - 1:0]);
        end
/*        else if (executed_instruction_in.op_q == q_amo) begin
            data_mem_req.addr = rd1;
            data_mem_req.data = rd2;
            data_mem_req.valid = true;
            if (executed_instruction_in.f3 == f3_amo_d) begin
                data_mem_req.do_write = {(`word_size_bytes){1'b1}};
                data_mem_req.do_read = {(`word_size_bytes){1'b1}};
            end
        end
*/
    end
end

always_ff @(posedge clk) begin
    if (memory_signal_in.advance) begin
        memory_instruction_out <= {($bits(memory_instruction_t)){1'b0}};
        if (executed_instruction_in.valid) begin
            memory_instruction_out.writeback_instruction <= executed_instruction_in.writeback_instruction;
            memory_instruction_out.f3 <= executed_instruction_in.f3;
            memory_instruction_out.op_q <= executed_instruction_in.op_q;
            memory_instruction_out.valid <= executed_instruction_in.valid;
        end
    end else if (writeback_signal_in.advance)
        memory_instruction_out <= {($bits(memory_instruction_t)){1'b0}};

end

endmodule

module writeback(
    input stage_signal_t writeback_signal_in
    ,input memory_io_rsp data_mem_rsp
    ,input memory_instruction_t memory_instruction_in
    ,output writeback_instruction_t writeback_instruction_out
    );

import riscv::*;

always @(*) begin
    writeback_instruction_out = {($bits(writeback_instruction_t)){1'b0}};
    if (writeback_signal_in.advance && memory_instruction_in.valid) begin
        writeback_instruction_out = memory_instruction_in.writeback_instruction;
        if (memory_instruction_in.op_q == q_load || memory_instruction_in.op_q == q_amo) begin
            writeback_instruction_out.wbd = subset_load_data(
                                shuffle_load_data(data_mem_rsp.data, memory_instruction_in.writeback_instruction.wbd[`word_size - 1:0]),
                                cast_to_memory_op(memory_instruction_in.f3));
            writeback_instruction_out.valid = data_mem_rsp.valid & memory_instruction_in.valid;
        end
    end
end

endmodule

module control(
    input memory_io_rsp inst_mem_rsp
    ,input memory_io_rsp data_mem_rsp
    ,input pc_control_t pc_control_in
    ,input fetched_instruction_t fetched_instruction_in
    ,input decoded_instruction_t decoded_instruction_in
    ,input executed_instruction_t executed_instruction_in
    ,input memory_instruction_t memory_instruction_in
    ,output stage_signal_t  fetch_signal_out
    ,output stage_signal_t  decode_signal_out
    ,output stage_signal_t  execute_signal_out
    ,output stage_signal_t  memory_signal_out
    ,output stage_signal_t  writeback_signal_out
    ,output fetch_set_pc_call_t fetch_set_pc_call_out
    );

import riscv::*;

// Stall / Flush logic 
always_comb begin
    // Start with everything running smooth
    fetch_signal_out.advance = true;
    fetch_signal_out.flush = false;
    decode_signal_out.advance = true;
    decode_signal_out.flush = false;
    execute_signal_out.advance = true;
    execute_signal_out.flush = false;
    memory_signal_out.advance = true;
    memory_signal_out.flush = false;
    writeback_signal_out.advance = true;
    writeback_signal_out.flush = false;
    fetch_set_pc_call_out.valid = false;
    fetch_set_pc_call_out.pc = `word_size'd0;

    // --- Load-use hazard detection ---
    // If instruction in EX is a load, and the instruction in DE needs that register,
    // we must stall for one cycle.
    if (executed_instruction_in.valid
        && executed_instruction_in.op_q == riscv::q_load
        && executed_instruction_in.writeback_instruction.wbv
        && executed_instruction_in.writeback_instruction.wbs != 5'd0
        && (executed_instruction_in.writeback_instruction.wbs == decoded_instruction_in.rs1
         || executed_instruction_in.writeback_instruction.wbs == decoded_instruction_in.rs2)
        && decoded_instruction_in.valid) begin
        // Stall fetch and decode, bubble into execute
        fetch_signal_out.advance = false;
        decode_signal_out.advance = false;
        memory_signal_out.advance = true;
        writeback_signal_out.advance = true;
    end

    // --- Branch mispredict handling ---
    // pc_control tells us when the fetched instruction doesn't match
    // what execute expected to run next.
if (pc_control_in.fetch_mispredict) begin
    fetch_set_pc_call_out.valid = true;
    fetch_set_pc_call_out.pc = pc_control_in.correct_pc;
    fetch_signal_out.flush = true;
    decode_signal_out.flush = true;
    fetch_signal_out.advance = true;
    decode_signal_out.advance = true;
end
    // --- Memory stall (instruction memory not ready) ---
    if (!inst_mem_rsp.valid) begin
        fetch_signal_out.advance = false;
        decode_signal_out.advance = false;
    end

    // --- Memory stall (data memory not ready) ---
    if (memory_instruction_in.valid
        && (memory_instruction_in.op_q == riscv::q_load
         || memory_instruction_in.op_q == riscv::q_store
         || memory_instruction_in.op_q == riscv::q_amo)
        && !data_mem_rsp.valid) begin
        fetch_signal_out.advance = false;
        decode_signal_out.advance = false;
        memory_signal_out.advance = false;
        writeback_signal_out.advance = false;
    end

end

endmodule

module core #(
    parameter btb_enable = false          // use a BTB?
    ) (
    input logic       clk
    ,input logic      reset
    ,input logic      [`word_address_size-1:0] reset_pc
    ,output memory_io_req   inst_mem_req
    ,input  memory_io_rsp   inst_mem_rsp
    ,output memory_io_req   data_mem_req
    ,input  memory_io_rsp   data_mem_rsp);

import riscv::*;

/* verilator lint_off UNOPTFLAT */
stage_signal_t fetch_signal, decode_signal, execute_signal, memory_signal, writeback_signal;
/* verilator lint_on UNOPTFLAT */

// Interface function structs to fetch
fetch_set_pc_call_t fetch_set_pc_call;
fetch_set_pc_return_t fetch_set_pc_return;

// Chief output of fetch
fetched_instruction_t fetched_instruction;

// Instantiate the fetch module proper
fetch fetch_m(.clk(clk), .reset(reset)
    ,.reset_pc(reset_pc)
    ,.fetch_signal_in(fetch_signal)
    ,.fetch_set_pc_call_in(fetch_set_pc_call)
    ,.fetch_set_pc_return_out(fetch_set_pc_return)
    ,.inst_mem_req(inst_mem_req)
    ,.inst_mem_rsp(inst_mem_rsp)
    ,.fetched_instruction_out(fetched_instruction)
    );


reg_file_bypass_t reg_file_bypass;
decoded_instruction_t decoded_instruction;
writeback_instruction_t writeback_instruction;

// Instantiate the decode and writeback module proper
decode_and_writeback decode_and_writeback_m(.clk(clk), .reset(reset)
    // Stage control signalling
    ,.decode_signal_in(decode_signal)
    ,.execute_signal_in(execute_signal)
    ,.writeback_signal_in(writeback_signal)
    // Control function interfaces
    ,.reg_file_bypass_out(reg_file_bypass)

    // Principle operation data path
    ,.fetched_instruction_in(fetched_instruction)
    ,.decoded_instruction_out(decoded_instruction)
    ,.writeback_instruction_in(writeback_instruction)
    );

executed_instruction_t executed_instruction;
pc_control_t pc_control;

execute execute_m(
    .clk(clk), .reset(reset)
    ,.reset_pc(reset_pc)
    ,.execute_signal_in(execute_signal)
    ,.memory_signal_in(memory_signal)

    ,.fetched_instruction_in(fetched_instruction)
    ,.reg_file_bypass_in(reg_file_bypass)
    ,.executed_instruction_in(executed_instruction)
    ,.writeback_instruction_in(writeback_instruction)

    ,.decoded_instruction_in(decoded_instruction)
    ,.executed_instruction_out(executed_instruction)
    ,.pc_control_out(pc_control)
    );

memory_instruction_t memory_instruction;

memory memory_m(
    .clk(clk), .reset(reset)
    ,.memory_signal_in(memory_signal)
    ,.writeback_signal_in(writeback_signal)

    ,.writeback_instruction_in(writeback_instruction)
    ,.reg_file_bypass_in(reg_file_bypass)

    ,.data_mem_req(data_mem_req)
    ,.data_mem_rsp(data_mem_rsp)
    ,.executed_instruction_in(executed_instruction)
    ,.memory_instruction_out(memory_instruction)
    );

writeback writeback_m(
    .writeback_signal_in(writeback_signal)
    ,.data_mem_rsp(data_mem_rsp)
    ,.memory_instruction_in(memory_instruction)
    ,.writeback_instruction_out(writeback_instruction)
    );

control control_m(
    .inst_mem_rsp(inst_mem_rsp)
    ,.data_mem_rsp(data_mem_rsp)
    ,.pc_control_in(pc_control)
    ,.fetched_instruction_in(fetched_instruction)
    ,.decoded_instruction_in(decoded_instruction)
    ,.executed_instruction_in(executed_instruction)
    ,.memory_instruction_in(memory_instruction)
    ,.fetch_signal_out(fetch_signal)
    ,.decode_signal_out(decode_signal)
    ,.execute_signal_out(execute_signal)
    ,.memory_signal_out(memory_signal)
    ,.writeback_signal_out(writeback_signal)
    ,.fetch_set_pc_call_out(fetch_set_pc_call)
    );

endmodule

`endif
