#=============================================================================
# vectored.S - vectored interrupt tvec (mtvec[0]=1) (M6 directed test 7)
#=============================================================================
# With mtvec MODE = vectored (bit0 = 1), an interrupt of cause C enters
# at tvec_base + 4*C (donor aq_cp0_trap_csr.v:1346-1359; CSR.v vec_int_pc).
# MSIP (cause 3) must therefore enter at vec_base + 12.
#
# Layout (4-byte slots from vec_base):
#   +0   cause-0 slot: `j fail` -- any wrong-slot entry fails the test
#   +4   cause-1 slot: nop
#   +8   cause-2 slot: nop
#   +12  cause-3 slot: the real handler (MSIP)
#
# FAIL encoding: tohost = (7<<1)|1 -> "FAIL. test no. = 7".
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
    la   x12, vec_base
    ori  x12, x12, 1              # mtvec = base | 1 (vectored mode)
    csrw mtvec, x12
    li   x12, 1<<3                # mie.MSIE
    csrw mie, x12
    csrrs x12, mstatus, x0
    or   x12, x12, 1<<3
    csrw mstatus, x12             # mstatus.MIE

    li   x14, 0x02000000
    li   x12, 1
    sd   x12, 0(x14)              # assert MSIP -> cause 3

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

    #-------------------------------------------------------------------------
    # Vector table. The RTL computes slot C = (mtvec & ~3) + 4*C
    # (CSR.v vec_int_pc), so the base must be 4-byte aligned (256B used,
    # satisfying the spec's vectored-mode base alignment as well) and each
    # slot is forced to its 4-byte offset: RVC makes `j`/`nop` 2-byte
    # instructions, so .align 2 after each slot keeps the table slot-true.
    #-------------------------------------------------------------------------
    .align 8
vec_base:
    j    fail_test                # +0: cause-0 slot (wrong entry = fail)
    .align 2
    nop                           # +4: cause-1 slot
    .align 2
    nop                           # +8: cause-2 slot
    .align 2
vec_cause3:                       # +12: cause-3 slot (MSIP)
    csrr x13, mcause
    li   x14, (1<<63) | 3
    bne  x13, x14, fail_test
    csrr x13, mtval
    bne  x13, x0, fail_test
    # mepc inside the spin loop (RVC {nop, c.j} loop: epc in [spin, spin+4))
    csrr x13, mepc
    la   x14, spin
    bgeu x13, x14, 1f
    j    fail_test
1:  addi x14, x14, 4
    bltu x13, x14, 2f
    j    fail_test
2:
    li   x14, 0x02000000
    sd   x0, 0(x14)               # clear MSIP
    la   x13, done
    csrw mepc, x13
    mret

fail_test:
    li x10, 0x7FFFF000
    li x11, (7<<1) | 1
    sd x11, 0(x10)
    j fail_test
