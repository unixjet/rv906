#=============================================================================
# wfi.S - WFI wake-on-MTIP test (M6 directed test 8)
#=============================================================================
# Tests that a legal WFI HOLDS the pipe (no issue) until an interrupt is
# pending-and-enabled, then retires as a no-op at which boundary the timer
# interrupt is taken (M6 Task 4, D-M4-7 discharged, D-M6-2 shape).
#
# Sequence:
#   1. mtvec = handler; mie.MTIE = 1; mstatus.MIE = 1
#   2. Read `time` (0xC01) -> t0; write mtimecmp = t0 + 2 (CLINT 0x4000)
#      -> mtip fires at exactly mtime == t0+2 (two rtc_ticks away, so mtip
#      is guaranteed 0 when wfi reaches EX1)
#   3. wfi  <- must HOLD until mtip
#   4. mtip fires -> wfi wakes, retires -> RTU takes the timer interrupt
#      at the wfi's OWN retire boundary (mepc == wfi+4 == post_wfi)
#
# Handler asserts, in order:
#   - mcause == (1<<63)|7 (MTIP, M-mode)
#   - mtval  == 0
#   - mepc   == post_wfi  (proves NO post-wfi instruction executed before
#                          the trap -- a no-op wfi would have let them run
#                          and the trap would land later, mepc != post_wfi)
#   - time   == t0 + 2    (proves the hart really waited for the tick;
#                          a pass-through wfi would have trapped/continued
#                          at t0+1 or earlier)
#   - clear mtip (mtimecmp = -1), mret -> post_wfi
#
# Post-mret: tohost = 1 (PASS). mret returns to mepc == post_wfi, so the
# PASS block is reachable exactly once.
#
# FAIL encoding: tohost = (testno<<1)|1 so the harness prints
# "FAIL. test no. = 8".
#=============================================================================

#define STACK_TOP _stack_top

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
    # Step 1: trap vector + interrupt enables
    #-------------------------------------------------------------------------
    la   x12, interrupt_handler
    csrw mtvec, x12

    li   x12, 1<<7            # mie.MTIE = 1
    csrw mie, x12

    csrrs x12, mstatus, x0
    or   x12, x12, 1<<3       # mstatus.MIE = 1
    csrw mstatus, x12

    #-------------------------------------------------------------------------
    # Step 2: schedule the timer tick two rtc_ticks out
    #-------------------------------------------------------------------------
    csrr x13, 0xC01           # x13 = t0 = live mtime
    mv   x15, x13             # keep t0 across the trap (handler re-checks)
    addi x13, x13, 2
    li   x14, 0x02004000      # CLINT mtimecmp
    STORE x13, 0(x14)         # mtimecmp = t0 + 2

    #-------------------------------------------------------------------------
    # Step 3: the instruction under test
    #-------------------------------------------------------------------------
    wfi                        # must hold until mtip (mtime == t0+2)
post_wfi:                      # mepc on the timer trap == &post_wfi

    #-------------------------------------------------------------------------
    # Step 4: post-mret PASS
    #-------------------------------------------------------------------------
    li   x10, 0x7FFFF000      # tohost (uncached aperture)
    li   x11, 1               # PASS
    sd   x11, 0(x10)
    j    .

#=============================================================================
# interrupt_handler - timer interrupt taken at the wfi's boundary
#=============================================================================
interrupt_handler:
    csrr x13, mcause
    li   x14, (1<<63) | 7     # MTIP interrupt, M-mode
    bne  x13, x14, fail_test

    csrr x13, mtval
    bne  x13, x0, fail_test   # interrupts carry zero tval

    csrr x13, mepc
    la   x14, post_wfi
    bne  x13, x14, fail_test  # trap must land EXACTLY at the wfi boundary

    csrr x13, 0xC01           # t_now: mtime at wake
    addi x14, x15, 2          # t0 + 2
    bne  x13, x14, fail_test  # wfi must have held until the 2nd tick

    # Clear mtip before returning (mtimecmp = -1)
    li   x13, 0x02004000
    li   x14, -1
    STORE x14, 0(x13)

    mret

#=============================================================================
# Test FAILED: tohost = (8<<1)|1 -> "FAIL. test no. = 8"
#=============================================================================
fail_test:
    li x10, 0x7FFFF000
    li x11, (8<<1) | 1
    sd x11, 0(x10)
    j fail_test
