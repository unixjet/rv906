#=============================================================================
# msip.S - M-mode self-interrupt test (first M6 directed interrupt test)
#=============================================================================
# Tests MSIP pin assertion via CLINT store triggers an M-mode interrupt
# (cause = 3 | bit63, epc = next-PC inside spin loop).
#
# Sequence:
#   1. Set mtvec to handler address
#   2. Enable interrupts: csrw mie, 1<<3; csrs mstatus, 1<<3
#   3. Store 1 to CLINT msip@0x02000000 -> msip pin asserted
#   4. Infinite spin loop (j 1b)
#
# Handler (taken from the spin loop, mepc == loop_start):
#   - Verify mcause == (1<<63)|3 (MSIP, M-mode)
#   - Verify mtval  == 0 (interrupts have zero tval)
#   - Verify mepc points INSIDE the spin-loop (proves interrupt taken FROM loop)
#   - Clear msip: sd x0, 0(0x02000000)
#   - Store PASS (tohost=1), then mret
#
# NOTE: tohost=1 is written INSIDE the handler, not after mret. mret returns to
# mepc == loop_start, which is an infinite spin, so a post-mret "pass" block
# would be unreachable on its own. The harness polls tohost and halts on the
# first nonzero write, so writing it in the handler is equivalent and robust.
# A stale claim after mret is harmless: msip is cleared first, and even a
# re-entry would re-pass every check (mepc stays inside the loop).
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
    # Save original mtvec for later restore (not strictly needed for this test)
    #-------------------------------------------------------------------------
    li x10, 0x02000000      # CLINT msip address (store 1 = assert msip pin)
    csrrw x11, mtvec, x0    # save old mtvec -> x11

    #-------------------------------------------------------------------------
    # Step 1: Set mtvec to handler
    #-------------------------------------------------------------------------
    la x12, interrupt_handler
    csrw mtvec, x12         # trap vector base = interrupt_handler

    #-------------------------------------------------------------------------
    # Step 2: Enable MSIP in MIE
    #-------------------------------------------------------------------------
    li x12, 1<<3            # mie.MSIP = 1
    csrw mie, x12           # enable msip interrupt

    #-------------------------------------------------------------------------
    # Step 3: Enable global interrupts (mstatus.MIE)
    #-------------------------------------------------------------------------
    csrrs x12, mstatus, x0  # read mstatus
    or x12, x12, 1<<3       # set MIE bit
    csrw mstatus, x12       # enable M-mode interrupts globally

    #-------------------------------------------------------------------------
    # Step 4: Assert msip by storing 1 to CLINT msip register
    #-------------------------------------------------------------------------
    # After AXI transaction completes, msip pin asserted -> will trigger on
    # NEXT retired instruction (epc = its next-PC, i.e. inside loop below)
    li x13, 1
    STORE x13, 0(x10)       # sd 1, 0x02000000 (assert msip pin)

    #-------------------------------------------------------------------------
    # Step 5: Spin forever waiting for interrupt
    #-------------------------------------------------------------------------
loop_start:                 # <--- mepc should be somewhere here when IRQ fires
    j loop_start

#=============================================================================
# interrupt_handler - M-mode trap handler (takes MSIP interrupt at cause 3|bit63)
#=============================================================================
interrupt_handler:
    #-------------------------------------------------------------------------
    # Verify we took MSIP interrupt (cause 3 with bit63 set for M-mode)
    #-------------------------------------------------------------------------
    csrr x13, mcause        # x13 = mcause
    li x14, (1<<63) | 3     # MSIP interrupt: [63]=1 (M-mode exception flag) | cause=3
    bne x13, x14, fail_test # FAIL if not MSIP

    #-------------------------------------------------------------------------
    # Verify mtval == 0 (interrupts always have zero tval per spec)
    #-------------------------------------------------------------------------
    csrr x13, mtval
    li x14, 0
    bne x13, x14, fail_test # FAIL if mtval != 0

    #-------------------------------------------------------------------------
    # Verify mepc points INSIDE the spin loop (prove IRQ taken FROM loop)
    # mepc must be >= loop_start && < loop_end
    #-------------------------------------------------------------------------
    csrr x13, mepc          # x13 = mepc (address that caused trap)

    # Check mepc >= loop_start (use sub-based comparison: mepc - loop_start >= 0)
    la x14, loop_start
    sub x13, x13, x14       # x13 = mepc - loop_start
    blt x13, x0, fail_test  # FAIL if mepc < loop_start (signed branch)

    # Check mepc < loop_end (loop_end = loop_start + small offset)
    # Since our loop body is 2 instructions (4 bytes), loop_end = loop_start + 8
    li x14, 8               # max offset in bytes
    blt x13, x14, check_mret_ok
fail_test_epc:
    j fail_test             # FAIL if mepc >= loop_start + 8 (outside loop)

check_mret_ok:
    #-------------------------------------------------------------------------
    # SUCCESS path: clear msip, report PASS, return
    #-------------------------------------------------------------------------
    # Clear msip: sd x0, 0x02000000
    li x13, 0x02000000
    sd x0, 0(x13)           # write 0 to msip -> deassert pin

    # Report PASS (tohost=1) before mret: mret returns to mepc == loop_start
    # (an infinite spin) so a post-mret pass block would be unreachable.
    li x10, 0x7FFFF000      # tohost address (uncached aperture)
    li x11, 1               # PASS = 1
    sd x11, 0(x10)          # -> harness polls tohost, halts on PASS

    # Return from trap: mret restores mepc and re-enables MIE
    mret

#=============================================================================
# Test FAILED: exit with tohost = 0x100 + test_number
#=============================================================================
fail_test:
    li x10, 0x7FFFF000      # tohost address
    li x11, 0x100 + 1       # FAIL = test number 1 shifted up
    sd x11, 0(x10)          # store FAIL result
    j fail_test             # infinite loop (test failed)
