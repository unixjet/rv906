#=============================================================================
# debug_spin.S - M7 directed spin ELF.
#
# Task 7 base: an infinite, RVC-FREE spin loop at a known PC with known
# pre-loop register values, consumed by the --m7-debug e2e (halt ->
# dpc/dcsr; abstract GPR r/w; ITR; progbuf memory r/w; step; resume via
# resumereq AND dret).
#
# Task 8 extension: the loop body gains a deterministic load + store on a
# fixed memory word (B) and a minimal M-mode breakpoint handler, so the
# trigger e2e (mcontrol execute/load/store, action 1 and 0) has:
#   * an execute-trigger target (the loop head),
#   * a load-trigger target (lw x14,0(x12)) whose cancellation is visible
#     in x14,
#   * a store-trigger target (sw x15,0(x12)) whose suppression is visible
#     in the memory word (read back through the loop's own load), and
#   * a handler that skips the triggered instruction (mepc += 4; mret).
# It NEVER exits (no tohost write): the testbench runs the JTAG smoke at
# reset release and exits before the normal run loop; if it ever DID
# enter the run loop the runner's timeout would catch it.
#
# Task 9 extension (SBA e2e): the prologue additionally stores the known
# word 0x77777777 to tohost = 0x7FFFF000 (the ELF's tohost symbol, the
# UNCACHED aperture per common.ld:19) -- the SBA tohost-region read
# oracle (the SBA master bypasses the DCache, so an uncached word is
# what the SBA read sees bit-exact). The prologue is 15 instructions
# (60 bytes) and .align 4 pads to the 16-byte boundary, so LOOP =
# 0x80000040 and handler = 0x80000050 are UNCHANGED: the x8/x9/x12
# setups compress to single lui's and the x14 pre-init drops (x14 = 0
# at reset and the loop's lw sets it each lap before any check reads
# it; x14 doubles as the tohost-address scratch).
#
# .option norvc: the whole file is 32-bit -- the smoke's dpc-range and
# step checks assume every advance is exactly +4 (a compressed loop body
# would make the loop PC range +4-ambiguous).
#
# Register setup (checked by the JTAG smoke, RVProcTest.cpp):
#   x8  = 0x5A5A0000   abstract GPR READ check (loop never touches x8)
#   x9  = 0xDEAD0000   pre-loop value; abstract GPR WRITE target
#   x10 = 0            loop counter, +1 per iteration (step-count oracle)
#   x11 = 0x77777777   tohost word value (Task 9 SBA oracle)
#   x12 = 0x80001000   B: the loop's load/store memory word (Task 8)
#   x13 = 0            ITR target (addi x13,x13,1)
#   x14 = load target; the loop's `lw x14,0(x12)` (Task 8 load oracle);
#         also the prologue's tohost-address scratch (Task 9) -- 0 at
#         reset, set by the loop's lw before any check reads it
#   x15 = 0x11111111   C: the loop's store value (Task 8 store source)
#   x16 = &handler     mtvec (Task 8)
#   x17 = scratch      handler mepc scratch
# x6/x7 are left untouched: the donor's progbuf memory r/w uses them as
# scratch (JTAG_DRV.vh read/write_word_by_pb).
#
# Layout (objdump-verified after build; the 15-instruction prologue is
# padded by .align 4 to the 16-byte boundary -- see RVProcTest.cpp
# constants):
#   LOOP:      addi x10,x10,1   (loop head; execute-trigger target)
#   LOOP+4:    lw   x14,0(x12)  (load-trigger target)
#   LOOP+8:    sw   x15,0(x12)  (store-trigger target)
#   LOOP+12:   j    LOOP        (back jump)
#   handler:   csrr x17,mepc; addi x17,x17,4; csrw mepc,x17; mret
#   tohost:    0x7FFFF000 (UNCACHED aperture, common.ld:19) = 0x77777777
#              after the prologue; never written again (Task 9)
# Loop region [LOOP, LOOP+16); a free-running halt lands dpc on one of
# the four loop PCs. The loop converges B = x15 = C and x14 = C.
#=============================================================================

    .option norvc

    .section ".text.init","ax",@progbits
    .globl _start
    .align 2

_start:
    lui  x8, 0x5a5a0        # x8 = 0x5a5a0000
    lui  x9, 0xdead0        # x9 = 0xdead0000
    li   x10, 0             # loop counter
    li   x13, 0             # ITR target
    lui  x15, 0x11111       # x15 = C = 0x11111111 (store value)
    addiw x15, x15, 0x111
    lui  x12, 0x80001       # x12 = B = 0x80001000 (loop load/store word)
    la   x16, handler       # x16 = &handler (Task 8)
    csrw mtvec, x16         # mtvec = handler (Task 8)
    sw   x15, 0(x12)        # B = C (deterministic initial memory)
    lui  x11, 0x77777       # x11 = 0x77777777 (tohost word; Task 9)
    addiw x11, x11, 0x777
    lui  x14, 0x7ffff      # x14 = 0x7ffff000 = &tohost (Task 9)
    sw   x11, 0(x14)        # tohost = 0x77777777 (SBA read oracle)

    .align 4
LOOP:
    addi x10, x10, 1       # loop head; execute-trigger target
    lw   x14, 0(x12)       # load-trigger target; x14 <- B
    sw   x15, 0(x12)       # store-trigger target; B <- x15
    j    LOOP              # back jump

    .align 2
handler:
    csrr x17, mepc         # breakpoint trap: skip the triggered inst
    addi x17, x17, 4       # (mepc += 4) ...
    csrw mepc, x17
    mret                   # ... and resume. No disarm: an armed trigger
                           # keeps firing each lap (stable suppression /
                           # cancellation oracle for the Task 8 e2e).
