#.extern main
.globl _start

.text

_start:
#
# Load/Store Tests (add after auipc/addi/adds)
#
    li      a0, 0x12345678    # Value for stores (word)
    li      a1, 0x10010000    # Base mem addr (data_mem writable)
    
    # Stores
    sw      a0, 0(a1)         # SW: store word @0x10010000
    sh      a0, 4(a1)         # SH: store half (0x5678) @0x10010004
    sb      a0, 7(a1)         # SB: store byte (0x78) @0x10010007
    
    # Loads (verify)
    lw      a2, 0(a1)         # LW: reload word → a2==0x12345678?
    lh      a3, 4(a1)         # LH: signed half → a3==0x5678?
    lbu     a4, 7(a1)         # LBU: zero-ext byte → a4==0x78?
    
    # Check: add to x28 (t3); inspect post-run
    add     t3, t3, a2
    add     t3, t3, a3
    add     t3, t3, a4
    
    j       _start

