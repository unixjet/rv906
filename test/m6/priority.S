#=============================================================================
# priority.S - interrupt priority: MEIP beats MSIP and MTIP (test 6)
#=============================================================================
# Arms all three machine-level interrupt sources at once:
#   MSIP  <- CLINT msip store
#   MTIP  <- mtimecmp = time (>= compare fires at the next tick)
#   MEIP  <- UART IER -> PLIC source 7 (prio 1, enabled, threshold 0)
# All are enabled in mie; mstatus.MIE is the LAST write (after the three
# sources are armed), so the first interrupt taken must be MEIP
# (cause 11|bit63) -- the donor's claim ordering puts
# MEIP above MSIP above MTIP (CSR.v int_sel[12] > [11] > [10]).
# The handler then services all three (PLIC claim/deassert/complete,
# msip clear, timer disarm) and verifies mip is fully clear.
#
# FAIL encoding: tohost = (6<<1)|1 -> "FAIL. test no. = 6".
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
    #-------------------------------------------------------------------------
    # PLIC: prio[7]=1, enable source 7, threshold=0
    #-------------------------------------------------------------------------
    li   x12, 1
    li   x13, 0x0C00001C
    sw   x12, 0(x13)
    li   x12, 1<<7
    li   x13, 0x0C002000
    sw   x12, 0(x13)
    li   x12, 0
    li   x13, 0x0C200000
    sw   x12, 0(x13)

    la   x12, handler
    csrw mtvec, x12
    li   x12, (1<<11) | (1<<3) | (1<<7)   # MEIE | MSIE | MTIE
    csrw mie, x12
    csrr x15, mstatus                   # MIE still off while arming

    #-------------------------------------------------------------------------
    # Arm all three sources with MIE held off: if MIE were on, a claim could
    # land mid-arming (e.g. right after the MSIP store, before MTIP/MEIP are
    # yet asserted) and MSIP would legitimately win. MIE is the LAST write,
    # so the first claim sees all three pending.
    #-------------------------------------------------------------------------
    li   x14, 0x02000000
    li   x12, 1
    sd   x12, 0(x14)              # MSIP
    csrr x12, 0xc01               # MTIP: mtimecmp = time (fires next tick)
    li   x14, 0x02004000
    STORE x12, 0(x14)
    li   x12, 2                   # MEIP: UART IER=2 (THRE level)
    li   x13, 0x10000004
    sb   x12, 0(x13)
    or   x15, x15, 1<<3
    csrw mstatus, x15             # mstatus.MIE -- last write

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
# handler - the first interrupt must be MEIP (cause 11|bit63). mtvec is
# BASE-aligned: label itself must be 4-byte aligned.
#=============================================================================
    .align 2
handler:
    csrr x13, mcause
    li   x14, (1<<63) | 11
    bne  x13, x14, fail_test      # MEIP must outrank MSIP/MTIP

    csrr x13, mtval
    bne  x13, x0, fail_test

    # mepc inside the spin loop (RVC {nop, j} loop: epc in [spin, spin+4))
    csrr x13, mepc
    la   x14, spin
    bgeu x13, x14, 1f
    j    fail_test
1:  addi x14, x14, 4
    bltu x13, x14, 2f
    j    fail_test
2:
    # Service MEIP: claim -> deassert the UART level -> complete
    li   x14, 0x0C200004
    lw   x13, 0(x14)
    li   x15, 7
    bne  x13, x15, fail_test
    li   x16, 0x10000004
    li   x15, 0
    sb   x15, 0(x16)
    li   x15, 7
    sw   x15, 0(x14)

    # Service MSIP
    li   x14, 0x02000000
    sd   x0, 0(x14)

    # Service MTIP: disarm the timer
    li   x14, 0x02004000
    li   x12, -1
    STORE x12, 0(x14)

    # All three must be clear now
    csrr x15, mip
    li   x14, (1<<11) | (1<<3) | (1<<7)
    and  x15, x15, x14
    bnez x15, fail_test

    la   x13, done
    csrw mepc, x13
    mret

fail_test:
    li x10, 0x7FFFF000
    li x11, (6<<1) | 1
    sd x11, 0(x10)
    j fail_test
