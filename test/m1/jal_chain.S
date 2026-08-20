/* jal_chain.S - dense unconditional direct-jump chains, forward and backward
 *
 * M1 spec S4.2's "jal/c.j chains". `jal`/`c.j` are FetchSink's `ab_br`
 * ("always taken") class (rtl/FetchSink.v SECTION HELPERS): with every
 * predictor off (rung 1), FetchSink's fake BJU resolves EVERY one of these
 * unconditionally at commit time and redirects PCGEN via `iu_ifu_tar_pc_vld`
 * (rtl/IFU.v SECTION PCGEN, priority level "delayed change-flow"). So this
 * file is the redirect-path exerciser: one flush+refetch per jump, back to
 * back, with targets landing at every halfword phase of the ICache's 4-byte
 * fetch group (rv906 has no 16-byte predecode row the way C910 does -- see
 * rvc_mix.S's header -- so there are only two phases to matter, 0 and 2 mod
 * 4, both covered below by the stride choices).
 *
 * WHY EVERY BACKWARD JUMP HERE IS A DESCENDING CHAIN
 * --------------------------------------------------
 * M1 has no execute stage: control flow is a pure function of the PC (spec
 * S4.1). A backward unconditional jump therefore ALWAYS re-executes the same
 * path and hangs the test. The only way to fetch backwards and still finish
 * is a DESCENDING chain: enter at the highest element, and have each element
 * jump to the one below it, with the lowest element jumping forward past the
 * whole block. Every element then executes exactly once and the redirect
 * direction is genuinely negative. The same trick is reused in dense_br.S
 * and thrash.S; it is the standard M1 shape for "go backwards, once".
 */

#include "macros.h"

    M1_TEXT_START

    /*---------------------------------------------------------------------
     * 1. Maximum-density forward chain: every single instruction is a taken
     *    32-bit jump to the next one. 4-byte stride, so every target lands
     *    at phase 0 mod 4.
     *--------------------------------------------------------------------*/
    .option push
    .option norvc
    .rept 96
    jal   x0, .+4
    .endr
    .option pop

    /*---------------------------------------------------------------------
     * 2. Maximum-density forward chain in RVC: 2-byte stride, so targets
     *    alternate between phase 0 and phase 2 every single jump, including
     *    across 64-byte line boundaries.
     *--------------------------------------------------------------------*/
    .option push
    .option rvc
    .rept 128
    c.j   .+2
    .endr
    .option pop

    /*---------------------------------------------------------------------
     * 3. Forward chain that SKIPS: each jump steps over a dead instruction,
     *    so the fetch group crossing the jump always contains bytes that
     *    must be discarded after the redirect (IFU.v's ipack_buf_flush).
     *    6-byte stride mixes a 32-bit jump with a 16-bit dead slot and
     *    rotates the phase every block (gcd(6,4)=2).
     *--------------------------------------------------------------------*/
    .rept 64
    .option push
    .option norvc
    jal   x0, .+6               /* 4 bytes, target = the next jal */
    .option rvc
    c.nop                       /* 2 bytes, jumped over (never committed) */
    .option pop
    .endr

    /*---------------------------------------------------------------------
     * 4. Alternating jal / c.j with a 14-byte stride: both jump encodings
     *    and both instruction lengths inside one stride, phase rotating
     *    every block (gcd(14,4)=2).
     *--------------------------------------------------------------------*/
    .rept 48
    .option push
    .option norvc
    jal   x0, .+8               /* -> the c.j below */
    nop                         /* dead */
    .option rvc
    c.j   .+6                   /* -> the next block's jal */
    c.nop                       /* dead */
    c.nop                       /* dead */
    .option pop
    .endr

    /*---------------------------------------------------------------------
     * 5. DESCENDING 32-bit chain: 64 elements, 6-byte stride, entered at the
     *    top element and walked down to the bottom, which escapes forward.
     *--------------------------------------------------------------------*/
    .option push
    .option norvc
    jal   x0, bwd32_top         /* enter the chain at its HIGHEST element */
bwd32_bottom:
    jal   x0, bwd32_escape      /* element 0: the only forward jump */
    .option rvc
    c.nop
    .option norvc
    .rept 62
    jal   x0, .-6               /* element i -> element i-1 */
    .option rvc
    c.nop
    .option norvc
    .endr
bwd32_top:
    jal   x0, .-6               /* element 63 */
    .option rvc
    c.nop
    .option norvc
bwd32_escape:
    nop
    .option pop

    /*---------------------------------------------------------------------
     * 6. DESCENDING RVC chain: same shape with c.j and a 6-byte stride, so
     *    the negative CJ immediate (and its 11-bit sign extension) is what
     *    the fake BJU / golden ISS both have to reproduce.
     *--------------------------------------------------------------------*/
    .option push
    .option norvc
    jal   x0, bwdc_top
    .option rvc
bwdc_bottom:
    c.j   bwdc_escape
    c.nop
    c.nop
    .rept 62
    c.j   .-6
    c.nop
    c.nop
    .endr
bwdc_top:
    c.j   .-6
    c.nop
    c.nop
bwdc_escape:
    c.nop
    .option pop

    /*---------------------------------------------------------------------
     * 7. Long-range jumps: forward across several cache lines, landing at
     *    hand-chosen (deliberately non-line-aligned) offsets, then a
     *    descending pair whose elements are a whole 64-byte line apart, so
     *    each redirect also forces a different ICache set/line.
     *--------------------------------------------------------------------*/
    .option push
    .option norvc
    jal   x0, far_a
    .option pop

    PAD_AT(0xA00, far_a)
    .option push
    .option norvc
    jal   x0, far_b
    .option pop

    PAD_AT(0xB46, far_b)        /* deliberately not line-aligned */
    .option push
    .option norvc
    jal   x0, far_c
    .option pop

    PAD_AT(0xC1E, far_c)        /* offset 2 mod 4: entry0-carry + a jump */
    .option push
    .option norvc
    jal   x0, line_top
    .option pop

    /* Descending chain with one element per 64-byte line. */
    PAD_AT(0xD00, line_bottom)
    .option push
    .option norvc
    jal   x0, line_escape
    .option pop
    PAD_AT(0xD40, line_1)
    .option push
    .option norvc
    jal   x0, line_bottom
    .option pop
    PAD_AT(0xD80, line_2)
    .option push
    .option norvc
    jal   x0, line_1
    .option pop
    PAD_AT(0xDC0, line_3)
    .option push
    .option norvc
    jal   x0, line_2
    .option pop
    PAD_AT(0xE00, line_top)
    .option push
    .option norvc
    jal   x0, line_3
    .option pop

    PAD_AT(0xE40, line_escape)

    SENTINEL

    M1_TOHOST_DATA
