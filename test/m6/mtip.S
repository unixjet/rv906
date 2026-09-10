#=============================================================================
# mtip.S - machine timer interrupt test (M6 directed test 2)
#=============================================================================
# Tests the CLINT mtimecmp tick path:
#   arm mtimecmp = time+1 -> mtime (tick every 100 clk) reaches it ->
#   mtip = (mtime >= mtimecmp) -> MTIP interrupt (cause 7|bit63).
# The handler also checks the `time` CSR (0xC01, M6 Task 3): at trap
# entry it must already read >= the armed compare value, and the tick
# must have advanced it past the pre-arm read.
#
# FAIL encoding: tohost = (2<<1)|1 -> "FAIL. test no. = 2".
#=============================================================================

#if __riscv_xlen == 64
# define STORE sd
# define LOAD ld
#else
# define STORE sw
# define LOAD lw
#endif

    .section ".text.init","ax",@progbits
    .globl _start
    .align 3

_start:
    # mtvec first: any early trap (e.g. the time read below) must land in
    # the handler, not the reset vector
    la   x12, handler
    csrw mtvec, x12

    #-------------------------------------------------------------------------
    # Arm the SBI-style timer: mtimecmp = time + 1 (next tick fires it)
    #-------------------------------------------------------------------------
    csrr x12, 0xc01              # time (CLINT mtime mirror)
    addi x16, x12, 1             # x16 = armed value
    li   x14, 0x02004000         # CLINT mtimecmp
    STORE x16, 0(x14)

    li   x12, 1<<7               # mie.MTIE
    csrw mie, x12
    csrrs x12, mstatus, x0
    or   x12, x12, 1<<3
    csrw mstatus, x12            # mstatus.MIE

spin:
    nop                           # trap may land on the nop or the branch
    j    spin

    #-------------------------------------------------------------------------
    # Post-mret PASS
    #-------------------------------------------------------------------------
done:
    li   x10, 0x7FFFF000
    li   x11, 1
    sd   x11, 0(x10)
    j    .

#=============================================================================
# handler - MTIP (cause 7|bit63). mtvec is BASE-aligned (CSR.v drops
# bits[1:0]), so the LABEL itself must be 4-byte aligned -- .align before
# the label, not after it.
#=============================================================================
    .align 2
handler:
    csrr x13, mcause
    li   x14, (1<<63) | 7
    bne  x13, x14, fail_test

    csrr x13, mtval
    bne  x13, x0, fail_test

    # mepc inside the spin loop. The RVC loop is {nop, j} (2+2 bytes), so
    # epc = ex2_next_pc lands at spin (the taken-jump target) or spin+2
    # (the j's own PC) -- accept the whole [spin, spin+4) window.
    csrr x13, mepc
    la   x14, spin
    bgeu x13, x14, 1f
    j    fail_test
1:  addi x14, x14, 4
    bltu x13, x14, 2f
    j    fail_test
2:
    # time CSR must have reached the armed value (the tick happened)
    csrr x13, 0xc01
    bltu x13, x16, fail_test     # time < armed -> tick never came

    # Disarm before returning (no immediate re-fire)
    li   x14, 0x02004000
    li   x12, -1
    STORE x12, 0(x14)

    la   x13, done
    csrw mepc, x13
    mret

fail_test:
    li x10, 0x7FFFF000
    li x11, (2<<1) | 1
    sd x11, 0(x10)
    j fail_test
