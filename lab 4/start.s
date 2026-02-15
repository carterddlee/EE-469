.globl _start
.text
_start:
    # ---------- U-type ----------
    lui   x1, 1              # x1 = 0x00001000

    # ---------- I-type arith (small immediates only) ----------
    addi  x2, x1, 10         # x2 = x1 + 10
    addi  x3, x2, -5         # x3 = x2 - 5

    # ---------- I-type logic ----------
    xori  x4, x3, 15         # x4 = x3 ^ 0x0000000F
    ori   x5, x4, 32         # x5 = x4 | 0x00000020
    andi  x6, x5, 63         # x6 = x5 & 0x0000003F

    # ---------- I-type shifts ----------
    slli  x7, x6, 2          # x7 = x6 << 2
    srli  x8, x7, 1          # x8 = x7 >> 1 (logical)
    srai  x9, x7, 1          # x9 = x7 >> 1 (arith)

    # ---------- SLTI ----------
    slti  x10, x3, 20        # x10 = (x3 < 20) ? 1 : 0

    # ---------- R-type ----------
    add   x11, x3, x6        # R-type add
    sub   x12, x11, x3       # R-type sub
    xor   x13, x11, x6       # R-type xor
    or    x14, x11, x6       # R-type or
    and   x15, x11, x6       # R-type and
    sll   x16, x6, x10       # shift left
    srl   x17, x6, x10       # logical right
    sra   x18, x6, x10       # arithmetic right
    slt   x19, x3, x6        # signed compare
    sltu  x20, x3, x6        # unsigned compare

    # ---------- Branches ----------
    beq   x10, x10, 1f       # always taken
    j     fail
1:  bne   x19, x0, 2f        # taken if x19 != 0
    j     fail
2:  blt   x3, x6, 3f         # expect taken
    j     fail
3:  bge  x6, x3, 4f          # expect taken
    j     fail
4:  bltu x3, x6, 5f          # expect taken
    j     fail
5:  bgeu x6, x3, pass        # expect taken
    j     fail

fail:
    ebreak                    # signal FAIL (early halt)

pass:
    # ---------- Jumps ----------
    jal   x21, target        # x21 = return addr
target:
    jalr  x22, x21, 0        # jump back via x21

    ebreak                    # signal SUCCESS halt

    j _start                  # never reached in normal run
