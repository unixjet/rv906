/* uncached.S - fetch through the uncached memory window
 *
 * M1 spec S4.2's "uncached-region fetch". rtl/RVProc.v's MMU stub derives
 * the cacheable attribute from byte-address bit 31 of the fetch VA (Task 6
 * finding, RVProc.v's own comment at the `mmu_ifu_prot` assignment):
 * addresses below 0x8000_0000 (bit 31 clear) are fetched UNCACHED --
 * ICache.v's `alloc_r_r` stays 0, so no tag/data array allocation happens
 * and every line is served fresh over AXI via the same critical-word bypass
 * path a cacheable miss uses (ICache.v SECTION REFILL), never a HIT.
 * common.ld places `.text.uncached` at 0x7FFF0000 (64KB below the reset
 * vector, still well inside `jal`'s +-1MB reach, and routed to the same wide
 * MEM AXI slave as 0x8000_0000 -- RVProc.v's header comment traces the
 * AXIAddrDecode.v default-slave reasoning).
 *
 * This test crosses into the uncached window, runs a genuine mix of RVC/
 * 32-bit instructions AND a conditional branch there (proving the direction
 * rule and IPACK's live boundary logic are unaffected by cacheability --
 * they are properties of PCGEN/IPACK, not of ICache's allocate decision),
 * then jumps back into the normal cached region to reach the sentinel --
 * exercising the transition back to a cacheable fetch too.
 */

#include "macros.h"

    M1_TEXT_START

    .option push
    .option rvc
    c.nop
    .option norvc
    addi  x1, x1, 1
    .option pop

    jal   x0, uncached_entry

    /* Landing point for the return from the uncached region. */
after_uncached:
    .option push
    .option rvc
    c.li  x12, 7
    .option norvc
    addi  x13, x13, 1
    .option pop

    SENTINEL

    M1_TOHOST_DATA

    /*=====================================================================
     * The uncached region itself (0x7FFF0000, common.ld/macros.h). Every
     * fetch here is a fresh, non-allocating AXI read (RVProc.v header) --
     * no line from this region is ever cached, so revisiting an address
     * would refetch it identically; nothing here needs to (or does) loop.
     *====================================================================*/
    M1_UNCACHED_TEXT

uncached_entry:
    .option push
    .option rvc
    c.nop
    .option norvc
    lui   x14, 0x2
    .option rvc
    c.li  x15, 3
    .option pop

    /* A conditional branch INSIDE the uncached region: direction is still
     * `^pc[7:4]` of this instruction's own (uncached-window) address, and
     * both oracles must agree regardless of which AXI slave answered the
     * fetch. uncached_br sits at a PAD_AT'd offset so the parity is known
     * by construction rather than left to the assembler. */
    PAD_AT(0x30, uncached_br)
    .option push
    .option norvc
    beq   x0, x0, uncached_br_target   /* pc[7:4]=3 (0b0011, even popcount)
                                           -> NOT taken: falls through */
    .option pop
uncached_br_fallthrough:
    .option push
    .option rvc
    c.mv  x16, x17
    .option pop
    jal   x0, uncached_straddle

    /* Unreachable on the correct path (the branch above is not taken) --
     * a wrong direction-rule evaluation would visibly diverge here instead
     * of coincidentally landing on the same PC either way. */
uncached_br_target:
    .option push
    .option rvc
    c.nop
    .option pop
    jal   x0, uncached_straddle

    /* A 32-bit instruction straddling the 64-byte cache-LINE-equivalent
     * boundary of this window too (the ICache's line granularity applies to
     * every AXI-backed address it fetches, cached or not -- see ICache.v's
     * refill/bypass path, which this window still exercises every time). */
    PAD_AT(0xBE, uncached_straddle)
    .option push
    .option norvc
    addi  x18, x18, 1            /* 0xBE..0xC1: crosses the 0xC0 boundary */
    .option pop

    jal   x0, after_uncached
