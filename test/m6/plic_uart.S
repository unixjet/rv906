#=============================================================================
# plic_uart.S - UART IRQ -> PLIC source 7 -> MEIP test (M6 directed test 3)
#=============================================================================
# Tests the full external-interrupt path (M6 Task 6):
#   C++ UART model (IER=THRE) -> irq() level -> RVProcAXI G_io_pins_uart_irq
#   -> PLIC int_src[7] -> pending/enable/prio -> meip -> mip[11] -> MEIP
#   interrupt (cause 11|bit63) -> handler claims (0x200004 read == 7),
#   deasserts the source, completes (0x200004 write == 7), meip drops.
#
# Also exercises, for the first time, a 32-bit MMIO register file behind
# the 512->64 AXIWidthAdapter: the PLIC pair decode plus the M6 Task 6
# LSU fix (MMIO direct reads carry the access PA, see LSU.v MS_DIRECT_READ).
#
# Addresses (SoC map):
#   PLIC  0x0C000000:  prio[7]@0x1C, pending@0x1000, enable@0x2000,
#                      threshold@0x200000, claim/complete@0x200004
#   UART  0x10000000:  IER@+4 (bit1 = THRE, always on in this model)
#
# Handler epilogue redirects mepc to `done` (the interrupted spin-loop
# instruction is skipped -- the source has been serviced).
#
# FAIL encoding: tohost = (3<<1)|1 -> "FAIL. test no. = 3".
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
    # Step 1: PLIC setup (32-bit registers)
    #-------------------------------------------------------------------------
    li   x12, 1                   # prio[7] = 1
    li   x13, 0x0C00001C
    sw   x12, 0(x13)

    li   x12, 1<<7                # enable source 7
    li   x13, 0x0C002000
    sw   x12, 0(x13)

    li   x12, 0                   # threshold = 0 (prio 1 > 0)
    li   x13, 0x0C200000
    sw   x12, 0(x13)

    #-------------------------------------------------------------------------
    # Step 2: trap vector + interrupt enables
    #-------------------------------------------------------------------------
    la   x12, interrupt_handler
    csrw mtvec, x12

    li   x12, 1<<11               # mie.MEIE = 1
    csrw mie, x12

    csrrs x12, mstatus, x0
    or   x12, x12, 1<<3           # mstatus.MIE = 1
    csrw mstatus, x12

    #-------------------------------------------------------------------------
    # Step 3: assert the UART interrupt (IER = 2 -> THRE level)
    #-------------------------------------------------------------------------
    li   x12, 2
    li   x13, 0x10000004          # UART IER
    sb   x12, 0(x13)

spin:
    nop                           # trap may land on the nop or the branch
    j    spin

    #-------------------------------------------------------------------------
    # Step 4: post-mret PASS
    #-------------------------------------------------------------------------
done:
    li   x10, 0x7FFFF000          # tohost (uncached aperture)
    li   x11, 1                   # PASS
    sd   x11, 0(x10)
    j    .

#=============================================================================
# interrupt_handler - MEIP interrupt from the UART via PLIC source 7
#=============================================================================
interrupt_handler:
    .align 2
    csrr x13, mcause
    li   x14, (1<<63) | 11        # MEIP interrupt, M-mode
    bne  x13, x14, fail_test

    csrr x13, mtval
    bne  x13, x0, fail_test       # interrupts carry zero tval

    # mepc must be inside the spin loop (nop or branch)
    csrr x13, mepc
    la   x14, spin
    bne  x13, x14, 1f
    j    2f
1:  addi x14, x14, 4
    bne  x13, x14, fail_test
2:
    # Claim: the claim register read returns the source id (7)
    li   x14, 0x0C200004
    lw   x13, 0(x14)
    li   x15, 7
    bne  x13, x15, fail_test

    # Deassert the source (IER = 0) BEFORE completing, so the level
    # gateway cannot re-pend the source on the complete
    li   x16, 0x10000004          # UART IER
    li   x15, 0
    sb   x15, 0(x16)

    # Complete: write the claimed id back
    sw   x13, 0(x14)              # x13 still holds 7

    # MEIP must be clear now (no pending, enabled source)
    csrr x15, mip
    li   x14, 1<<11               # mip.MEIP mask
    and  x15, x15, x14
    bnez x15, fail_test

    # Skip the interrupted spin-loop instruction; land in `done`
    la   x13, done
    csrw mepc, x13

    mret

#=============================================================================
# Test FAILED: tohost = (3<<1)|1 -> "FAIL. test no. = 3"
#=============================================================================
fail_test:
    li x10, 0x7FFFF000
    li x11, (3<<1) | 1
    sd x11, 0(x10)
    j fail_test
