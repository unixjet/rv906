#=============================================================================
# ssip_deleg.S - SSIP delegated to S-mode (M6 directed test 4)
#=============================================================================
# Tests the interrupt delegation path:
#   M sets PMP allow-all (S/U code must pass PMP), stvec, mideleg[1]
#   (SSIP delegable), mie.SSIE, mstatus.SIE, sets SSIP pending (mip[1]
#   is writable from M), then mret to S (MPP=S).
#   In S-mode the pending+enabled+delegated SSIP traps to stvec with
#   scause = 1|bit63 (delegated interrupts carry the same cause number;
#   the bit63 "interrupt" flag is in scause too).
#   The S handler clears SSIP via sip (in S-mode only SSIP is writable
#   through the sip view) and srets to `done` (runs in U: SPP=0).
#
# FAIL encoding: tohost = (4<<1)|1 -> "FAIL. test no. = 4".
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
    # PMP allow-all (S/U execution must not fault; M4 p-env convention)
    #-------------------------------------------------------------------------
    li   x12, (1<<53) - 1
    csrw pmpaddr0, x12
    li   x12, 0x1f               # NAPOT|R|W|X (A=0b11, cfg byte {L,A,X,W,R})
    csrw pmpcfg0, x12

    #-------------------------------------------------------------------------
    # S-mode vector + delegation + enables
    #-------------------------------------------------------------------------
    la   x12, s_handler
    csrw stvec, x12
    li   x12, 1<<1               # SSIP
    csrw mideleg, x12            # delegate SSIP to S
    li   x12, 1<<1               # mie.SSIE
    csrw mie, x12
    li   x12, 1<<1               # SSIP pending (M-mode: mip[1] writable)
    csrw mip, x12
    csrrs x12, mstatus, x0
    ori  x12, x12, 1<<1          # mstatus.SIE
    csrw mstatus, x12

    #-------------------------------------------------------------------------
    # Drop to S-mode: mret targets Mepc, so arm it explicitly (no trap has
    # run yet, so mepc is still its reset value)
    #-------------------------------------------------------------------------
    la   x12, s_spin
    csrw mepc, x12
    li   x12, (1<<12)|(1<<11)   # MPP[1:0] mask (resets to 2'b11 = M)
    csrc mstatus, x12           # MPP <- 00
    li   x12, 1<<11             # MPP[0] = 1
    csrs mstatus, x12           # MPP <- 01 (S)
    mret                         # -> s_spin

s_spin:
    nop                           # trap may land on the nop or the branch
    j    s_spin

    #-------------------------------------------------------------------------
    # Post-sret PASS (runs in U-mode; PMP allows all)
    #-------------------------------------------------------------------------
done:
    li   x10, 0x7FFFF000
    li   x11, 1
    sd   x11, 0(x10)
    j    .

#=============================================================================
# s_handler - S-mode trap (delegated SSIP, scause = 1|bit63). stvec is
# BASE-aligned, so the label itself must be 4-byte aligned.
#=============================================================================
    .align 2
s_handler:
    csrr x13, scause
    li   x14, (1<<63) | 1
    bne  x13, x14, fail_test

    csrr x13, stval
    bne  x13, x0, fail_test

    # sepc inside the S spin loop (RVC {nop, j} loop: epc in [s_spin, s_spin+4))
    csrr x13, sepc
    la   x14, s_spin
    bgeu x13, x14, 1f
    j    fail_test
1:  addi x14, x14, 4
    bltu x13, x14, 2f
    j    fail_test
2:
    # Clear SSIP (S-mode: the sip view writes SSIP only)
    csrci 0x104, 2               # sip &= ~SSIP

    la   x13, done
    csrw sepc, x13
    sret

fail_test:
    li x10, 0x7FFFF000
    li x11, (4<<1) | 1
    sd x11, 0(x10)
    j fail_test
