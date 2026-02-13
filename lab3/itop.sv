`include "top.sv"

`timescale 1ns / 1ps

module itop();

logic clk = 0;
logic reset = 1;
logic halt;

top the_top(.clk(clk), .reset(reset), .halt(halt));

always #5 clk = ~clk;

initial begin
    $dumpfile("test.vcd");
    $dumpvars(0, itop);
    
    // Load your assembled program
    $readmemh("code0.hex", the_top.core.inst_mem.mem, 0, 31);  // First 32 instrs [web:80]
    
    reset = 1;
    #16 reset = 0;
    
    // Run ~100 cycles for load/store seq
    repeat(100) @(posedge clk);
    
    // Simple checks
    $display("a2(x5)=%h (exp 12345678)", the_top.core.registers[5]);
    $display("a3(x6)=%h (exp 5678)", the_top.core.registers[6]);
    $display("a4(x7)=%h (exp 78)", the_top.core.registers[7]);
    $display("t3(x28)=%h (sum check)", the_top.core.registers[28]);
    
    if (the_top.core.registers[5] == 32'h12345678 &&
        the_top.core.registers[6] == 32'h5678 &&
        the_top.core.registers[7] == 32'h78)
        $display("PASS: Load/Store works!");
    else
        $display("FAIL: Check registers in gtkwave");
        
    $finish;
end

endmodule
