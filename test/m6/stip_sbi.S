#=============================================================================
# stip_sbi.S - SBI-model timer: M handler sets STIP -> S trap (test 5)
#=============================================================================
# Models the OpenSBI timer flow without a full SBI:
#   M arms mtimecmp = time+1 (the "SBI timer request") and drops to S
#   (MPP=S). The tick fires MTIP -> M handler (mtvec), which checks
#   mcause = 7|bit63, disarms the timer, and forwards the tick to the
#   guest by setting sip.STIP (stip_f, mip[5] writable from M). mret
#   returns to S, where the delegated STIP (mideleg[5]) traps to stvec
#   with scause = 5|bit63.
#   STIP is only clearable from M (the sip view in S touches SSIP only),
#   so the S handler requests the clear via ecall -- the SBI-style
#   service call -- and M clears it before returning.
#
# FAIL encoding: tohost = (5<<1)|1 -> "FAIL. test no. = 5".
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
    # PMP allow-all (S/U execution must not fault)
    #-------------------------------------------------------------------------
    li   x12, (1<<53) - 1
    csrw pmpaddr0, x12
    li   x12, 0x1f               # NAPOT|R|W|X (A=0b11, cfg byte {L,A,X,W,R})
    csrw pmpcfg0, x12

    #-------------------------------------------------------------------------
    # Vectors + delegation + enables
    #-------------------------------------------------------------------------
    la   x12, m_handler
    csrw mtvec, x12
    la   x12, s_handler
    csrw stvec, x12
    li   x12, 1<<5               # STIP
    csrw mideleg, x12            # delegate STIP to S
    li   x12, (1<<5) | (1<<7)    # mie.STIE | mie.MTIE
    csrw mie, x12
    csrrs x12, mstatus, x0
    ori  x12, x12, 1<<1          # mstatus.SIE
    csrw mstatus, x12

    #-------------------------------------------------------------------------
    # Arm the timer (SBI request) and drop to S ("guest"). mret targets
    # Mepc, so arm it explicitly (no trap has run yet, so mepc is 0).
    #-------------------------------------------------------------------------
    csrr x12, 0xc01              # time
    addi x13, x12, 1
    li   x14, 0x02004000
    STORE x13, 0(x14)
    la   x12, s_spin
    csrw mepc, x12
    li   x12, (1<<12)|(1<<11)   # MPP[1:0] mask (resets to 2'b11 = M)
    csrc mstatus, x12           # MPP <- 00
    li   x12, 1<<11             # MPP[0] = 1
    csrs mstatus, x12           # MPP <- 01 (S)
    mret                         # -> s_spin

s_spin:
    nop
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
# m_handler - M-mode: timer tick (MTIP) and the S-ecall clear request.
# mtvec/stvec are BASE-aligned: label itself must be 4-byte aligned.
#=============================================================================
    .align 2
m_handler:
    csrr x13, mcause
    li   x14, (1<<63) | 7
    beq  x13, x14, m_tick
    li   x14, 9
    bne  x13, x14, fail_test     # only MTIP and S-ecall may arrive here

    # S-ecall: service the tick-clear request. mepc = the ecall's own PC,
    # so it must not re-execute on resume: advance mepc past the (16- or
    # 32-bit) ecall before mret.
    li   x12, 0
    csrw mip, x12                 # clear STIP (and SSIP/SEIP, both 0 here)
    csrr x13, mepc
    lh   x14, 0(x13)
    andi x14, x14, 0x3            # RVC: low2 == 11 -> 32-bit, else 16-bit
    li   x15, 3
    beq  x14, x15, m_ecall32
    addi x13, x13, 2
    csrw mepc, x13
    mret
m_ecall32:
    addi x13, x13, 4
    csrw mepc, x13
    mret

m_tick:
    # Disarm the timer (no immediate re-fire)
    li   x14, 0x02004000
    li   x12, -1
    STORE x12, 0(x14)

    # SBI model: forward the tick to the guest as STIP
    li   x12, 1<<5
    csrw mip, x12                 # set mip.STIP
    mret                          # -> S; delegated STIP traps there

#=============================================================================
# s_handler - S-mode trap (delegated STIP, scause = 5|bit63)
#=============================================================================
    .align 2
s_handler:
    csrr x13, scause
    li   x14, (1<<63) | 5
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
    # Ask M (via ecall) to clear STIP BEFORE sret. STIP is only clearable
    # from M, and sret restores SIE=1 -- a sret with STIP still pending
    # would re-trap immediately at sepc (SIE=1), before any ecall could
    # run. The ecall is issued here while SIE=0 (cleared at trap entry);
    # M clears STIP and resumes us past the ecall, so the sret below
    # returns to s_clear_and_done with STIP already clear.
    la   x13, s_clear_and_done
    csrw sepc, x13
    ecall                         # -> M handler (cause 9|bit63)
    sret

s_clear_and_done:
    j    done

fail_test:
    li x10, 0x7FFFF000
    li x11, (5<<1) | 1
    sd x11, 0(x10)
    j fail_test
