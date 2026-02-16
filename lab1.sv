 function void print_instruction(logic [31:0] pc, logic [31:0] instruction);
    logic [4:0] rd;
    logic [4:0] rs1;
    logic [4:0] rs2;
    logic signed [31:0] imm;
    logic [6:0] opcode;
    logic [2:0] funct3;
    logic [6:0] funct7;
    
    string command;

    // Assign values after declaration
    rd = instruction[11:7];
    rs1 = instruction[19:15];
    rs2 = instruction[24:20];
    funct3 = instruction[14:12];
    funct7 = instruction[31:25];
    opcode = instruction[6:0];

    // Get instruction name and immediate directly using opcode
    command = find_instruct(opcode, funct3, funct7);
    imm = find_imm(opcode, instruction);

    $write("%08x: ", pc);
    $write("%08x   ", instruction);
    $write("%s\t", command);

    // Print operands based on opcode
    case (opcode)
        // R-type (OP)
        7'b0110011: begin
            $write("%s,%s,%s", regname(rd), regname(rs1), regname(rs2));
        end
        
        // I-type LOAD
        7'b0000011: begin
            $write("%s,%0d(%s)", regname(rd), imm, regname(rs1));
        end
        
        // I-type JALR
        7'b1100111: begin
            $write("%s,%s,%0d", regname(rd), regname(rs1), imm);
        end
        
        // I-type OP-IMM
        7'b0010011: begin
            $write("%s,%s,%0d", regname(rd), regname(rs1), imm);
        end
        
        // S-type
        7'b0100011: begin
            $write("%s,%0d(%s)", regname(rs2), imm, regname(rs1));
        end
        
        // B-type
        7'b1100011: begin
            $write("%s,%s,%0d", regname(rs1), regname(rs2), imm);
        end
        
        // U-type (LUI, AUIPC)
        7'b0110111,
        7'b0010111: begin
            $write("%s,0x%x", regname(rd), imm[31:12]);
        end
        
        // J-type (JAL)
        7'b1101111: begin
            $write("%s,%0d", regname(rd), imm);
        end
        
        default: begin
            $write("unknown");
        end
    endcase

    $write("\n");
endfunction


function string find_instruct(input logic [6:0] opcode, input logic [2:0] funct3, input logic [6:0] funct7);
    case (opcode)
        7'b0000011: begin  // LOAD
            case (funct3)
                3'b000: return "lb";
                3'b001: return "lh";
                3'b010: return "lw";
                3'b100: return "lbu";
                3'b101: return "lhu";
                default: return "unknown_load";
            endcase
        end

        7'b0010011: begin  // OP-IMM
            case (funct3)
                3'b000: return "addi";
                3'b010: return "slti";
                3'b011: return "sltiu";
                3'b100: return "xori";
                3'b110: return "ori";
                3'b111: return "andi";
                3'b001: begin
                    if (funct7 == 7'b0000000) return "slli";
                    else                      return "UNKNOWN";
                end
                3'b101: begin
                    if      (funct7 == 7'b0000000) return "srli";
                    else if (funct7 == 7'b0100000) return "srai";
                    else                           return "UNKNOWN";
                end
                default: return "UNKNOWN";
            endcase
        end

        7'b0010111: return "auipc";  // AUIPC

        7'b0100011: begin  // STORE
            case (funct3)
                3'b000: return "sb";
                3'b001: return "sh";
                3'b010: return "sw";
                default: return "unknown_store";
            endcase
        end

        7'b0110011: begin  // OP
            case (funct3)
                3'b000: begin
                    if      (funct7 == 7'b0000000) return "add";
                    else if (funct7 == 7'b0100000) return "sub";
                    else                           return "unknown_op";
                end
                3'b001: begin
                    if (funct7 == 7'b0000000) return "sll";
                    else                      return "unknown_op";
                end
                3'b010: begin
                    if (funct7 == 7'b0000000) return "slt";
                    else                      return "unknown_op";
                end
                3'b011: begin
                    if (funct7 == 7'b0000000) return "sltu";
                    else                      return "unknown_op";
                end
                3'b100: begin
                    if (funct7 == 7'b0000000) return "xor";
                    else                      return "unknown_op";
                end
                3'b101: begin
                    if      (funct7 == 7'b0000000) return "srl";
                    else if (funct7 == 7'b0100000) return "sra";
                    else                           return "unknown_op";
                end
                3'b110: begin
                    if (funct7 == 7'b0000000) return "or";
                    else                      return "unknown_op";
                end
                3'b111: begin
                    if (funct7 == 7'b0000000) return "and";
                    else                      return "unknown_op";
                end
                default: return "unknown_op";
            endcase
        end

        7'b0110111: return "lui";  // LUI

        7'b1100011: begin  // BRANCH
            case (funct3)
                3'b000: return "beq";
                3'b001: return "bne";
                3'b100: return "blt";
                3'b101: return "bge";
                3'b110: return "bltu";
                3'b111: return "bgeu";
                default: return "unknown_branch";
            endcase
        end

        7'b1100111: begin  // JALR
            if (funct3 == 3'b000) return "jalr";
            else                  return "unknown_jalr";
        end

        7'b1101111: return "jal";  // JAL

        default: return "UNKNOWN";
    endcase
endfunction


function logic [31:0] find_imm(
    input logic [6:0] opcode,
    input logic [31:0] instruction
);
    logic signed [31:0] imm;
    case (opcode)
        // I-type: LOAD, OP-IMM, JALR
        7'b0000011,
        7'b0010011,
        7'b1100111: begin
            imm = {{20{instruction[31]}}, instruction[31:20]};
        end
        
        // S-type: STORE
        7'b0100011: begin
            imm = {{20{instruction[31]}}, instruction[31:25], instruction[11:7]};
        end
        
        // B-type: BRANCH
        7'b1100011: begin
            imm = {{19{instruction[31]}}, instruction[31], instruction[7],
                   instruction[30:25], instruction[11:8], 1'b0};
        end
        
        // U-type: LUI, AUIPC
        7'b0110111,
        7'b0010111: begin
            imm = {instruction[31:12], 12'b0};
        end
        
        // J-type: JAL
        7'b1101111: begin
            imm = {{11{instruction[31]}}, instruction[31], instruction[19:12],
                   instruction[20], instruction[30:21], 1'b0};
        end
        
        default: imm = '0;
    endcase
    return imm;
endfunction


function string regname(input logic [4:0] r);
    case (r)
        5'd0:  return "zero";
        5'd1:  return "ra";
        5'd2:  return "sp";
        5'd3:  return "gp";
        5'd4:  return "tp";
        5'd5:  return "t0";
        5'd6:  return "t1";
        5'd7:  return "t2";
        5'd8:  return "s0";  
        5'd9:  return "s1";
        5'd10: return "a0";
        5'd11: return "a1";
        5'd12: return "a2";
        5'd13: return "a3";
        5'd14: return "a4";
        5'd15: return "a5";
        5'd16: return "a6";
        5'd17: return "a7";
        5'd18: return "s2";
        5'd19: return "s3";
        5'd20: return "s4";
        5'd21: return "s5";
        5'd22: return "s6";
        5'd23: return "s7";
        5'd24: return "s8";
        5'd25: return "s9";
        5'd26: return "s10";
        5'd27: return "s11";
        5'd28: return "t3";
        5'd29: return "t4";
        5'd30: return "t5";
        5'd31: return "t6";
        default: return "x?";
    endcase
endfunction
