/* mixed.S - mixed stress: alignment, jumps, calls/returns, indirect jumps,
 * and dense branches interleaved in one stream
 *
 * M1 spec S4.2's "mixed stress (all of the above interleaved)". This file
 * combines a representative slice of every OTHER rung-1 test's mechanism
 * into a single stream, back to back with no isolation between them, so
 * that state one mechanism leaves behind (IPACK's entry0 carry, IBUF
 * occupancy, FetchSink's shadow-stack depth, an in-flight ICache refill) is
 * still live when the next mechanism starts -- exactly the kind of
 * interaction the dedicated single-mechanism files (rvc_mix.S, jal_chain.S,
 * callret.S, ind_jr.S, dense_br.S) cannot exercise by construction.
 *
 * DELIBERATELY OUT OF SCOPE HERE (documented, not an oversight): the
 * uncached-region excursion (uncached.S) and the fence.i host-patch
 * mechanism (fencei.S) are both testbench-driven (a distinct linker-script
 * region for the former, a `--fencei-patch=<addr>:<word>:<commit>` CLI flag
 * hand-computed against ONE specific file's layout for the latter) --
 * folding either into this file would mean re-deriving that same
 * hand-computed state against a much larger, harder-to-audit program for no
 * additional rung-1 coverage neither mechanism doesn't already get on its
 * own. Everything else lives here.
 */

#include "macros.h"

    M1_TEXT_START

    /*---------------------------------------------------------------------
     * Block 1: RVC/32-bit alignment mix, including a straddle across the
     * 4-byte ICache fetch-group boundary right at the very first fetch.
     *--------------------------------------------------------------------*/
    .option push
    .option rvc
    c.nop                       /* 0x000 */
    .option norvc
    addi  x1, x1, 1             /* 0x002: straddles the 4-byte boundary */
    .option rvc
    c.li  x2, 1
    c.mv  x3, x4
    .option norvc
    lui   x5, 0x1
    .option pop

    /*---------------------------------------------------------------------
     * Block 2: a short jal/c.j chain, forward only, phase-rotating.
     *--------------------------------------------------------------------*/
    .option push
    .option norvc
    .rept 16
    jal   x0, .+4
    .endr
    .option pop
    .option push
    .option rvc
    .rept 24
    c.j   .+2
    .endr
    .option pop

    /*---------------------------------------------------------------------
     * Block 3: dense conditional branches, forward-only shapes plus one
     * backward gadget (the standard M1 "jump in, take it back, escape
     * forward" trick -- jal_chain.S's header explains why).
     *--------------------------------------------------------------------*/
    .option push
    .option rvc
    .rept 32
    c.beqz x8, .+2
    .endr
    .option pop

    PAD_AT(0x180, mixbr_target)
    .option push
    .option norvc
    addi  x19, x19, 1
    jal   x0, mixbr_escape
    .option pop

    PAD_AT(0x1D0, mixbr_entry)   /* pc[7:4]=0xD (0b1101, 3 ones, odd) -> TAKEN */
    .option push
    .option norvc
    beq   x0, x0, mixbr_target  /* backward, TAKEN */
    .option pop

    PAD_AT(0x1E0, mixbr_escape)

    /*---------------------------------------------------------------------
     * Block 4: a call/return nest, 4 deep (same real-RAS-limit depth as
     * callret.S, exercised here with other state already in flight around
     * it rather than in isolation).
     *--------------------------------------------------------------------*/
    .option push
    .option norvc
    jal   x1, mixnest_start      /* push mix_esc -> depth 1 */
mix_esc:
    jal   x0, mix_after_nest
mixnest_start:
    .rept 3                      /* depths 2,3,4 */
    jal   x1, .+8
    jalr  x0, 0(x1)
    .endr
    jalr  x0, 0(x1)              /* deepest return: depth 4 here */
    .option pop

    /*---------------------------------------------------------------------
     * Block 5: an indirect jump and an indirect call, immediately after the
     * call/return nest above (so the shadow stack's just-drained state and
     * the indirect-jump redirect interact in the same window).
     *--------------------------------------------------------------------*/
mix_after_nest:
    PAD_AT(0x300, mix_ind_site)
    .option push
    .option rvc
    c.jr  x6
    .option pop
    JR_PAD(mix_ind_site, mix_ind_pad)
    .option push
    .option norvc
    jal   x0, mix_indcall_site
    .option pop

    PAD_AT(0x400, mix_indcall_site)
    .option push
    .option norvc
    jalr  x1, 0(x2)               /* pcall + ind_br: push, -> JR_TARGET */
mix_indcall_after:
    jal   x0, mix_tail
    .option pop
    JR_PAD(mix_indcall_site, mix_indcall_pad)
    .option push
    .option norvc
    jalr  x0, 0(x1)               /* preturn: pops mix_indcall_after */
    .option pop

    /*---------------------------------------------------------------------
     * Block 6: one more RVC/32-bit alignment run, ending in the sentinel --
     * proves the front end is still healthy after everything above.
     *--------------------------------------------------------------------*/
    PAD_AT(0x500, mix_tail)
    .rept 32
    .option push
    .option rvc
    c.li  x20, 1                /* 2 */
    .option norvc
    xori  x21, x21, 5           /* 4 */
    .option rvc
    c.li  x22, 2                /* 2 */
    .option norvc
    andi  x23, x23, 7           /* 4 */
    .option pop
    .endr

    SENTINEL

    M1_TOHOST_DATA
