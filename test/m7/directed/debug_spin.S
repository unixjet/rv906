#=============================================================================
# debug_spin.S - M7 Task 7 directed spin ELF (the C906_DEBUG_PATTERN.s
# analog, refs/openc906/smart_run/tests/cases/debug/C906_DEBUG_PATTERN.s:
# "main: ... LOOP: c.j LOOP"). An infinite, RVC-FREE spin loop at a
# known PC with known register setup before the loop, consumed by the
# extended --m7-debug e2e (halt -> dpc/dcsr; abstract GPR r/w; ITR;
# progbuf memory r/w; step; resume via resumereq AND dret).
#=============================================================================
# It NEVER exits (no tohost write): the testbench runs the JTAG smoke at
# reset release and exits before the normal run loop; if it ever DID
# enter the run loop the runner's timeout would catch it.
#
# .option norvc: the whole file is 32-bit -- the smoke's dpc-range and
# step checks assume every advance is exactly +4 (a compressed loop
# body, e.g. C.ADDI for x10, would make the loop PC range +4-ambiguous).
#
# Register setup (checked by the JTAG smoke, RVProcTest.cpp):
#   x8  = 0x5A5A0000   abstract GPR READ check (loop never touches x8)
#   x9  = 0xDEAD0000   pre-loop value; abstract GPR WRITE target
#   x10 = 0            loop counter, +1 per iteration (step-count oracle)
#   x13 = 0            ITR target (addi x13,x13,1)
# x6/x7 are left untouched: the donor's progbuf memory r/w uses them as
# scratch (JTAG_DRV.vh read/write_word_by_pb).
#
# LOOP lands at 0x80000020 (6 x 32-bit prologue instructions at the
# 0x80000000 reset vector + two .align-4 padding nops; confirmed by
# objdump of the built ELF):
#   0x80000020: 0x00150513  addi a0,a0,1   (loop counter++)
#   0x80000024: 0xffdff06f  j LOOP
# Loop region [0x80000020, 0x80000028); a haltreq dpc lands on one of
# the two loop PCs (the last-retired loop instruction).
#=============================================================================

    .option norvc

    .section ".text.init","ax",@progbits
    .globl _start
    .align 2

_start:
    lui  x8, 0x5a5a0        # x8 = 0x5a5a0000
    addi x8, x8, 0
    lui  x9, 0xdead0        # x9 = 0xdead0000
    addi x9, x9, 0
    li   x10, 0             # loop counter
    li   x13, 0             # ITR target

    .align 4
LOOP:
    addi x10, x10, 1       # 0x00128293  loop counter++
    j    LOOP              # 0xff9000ef  back jump, -8
