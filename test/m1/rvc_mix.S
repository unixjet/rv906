/* rvc_mix.S - RVC/32-bit alignment phases and length straddles
 *
 * M1 spec S4.2's "rvc-mix (alignment phases + straddle across the 32-bit
 * ICache-read boundary and the 64B line boundary)".
 *
 * WHAT THE HARDWARE DOES (rv906/C906, NOT C910 -- see IFU notes S3/S4.1/S4.2)
 * ----------------------------------------------------------------------
 * Unlike C910, C906's ICache has NO precomputed per-row predecode array: it
 * delivers exactly one 32-bit word (2 halfwords) to IPACK per cycle, and
 * IPACK computes RVC boundaries LIVE, per halfword, from that word's own
 * bits (`inst[1:0]==2'b11`) -- there is no bry0/bry1 "phase" to get wrong,
 * only entry0, a 1-halfword carry register that holds the low half of a
 * 32-bit instruction whose upper half hasn't arrived yet (rtl/IFU.v SECTION
 * IPACK; extraction note S4.2). So the interesting boundaries here are much
 * narrower than C910's 16-byte row:
 *   1. does a 32-bit instruction start at the LOW halfword (offset 0 mod 4)
 *      or the HIGH halfword (offset 2 mod 4) of the ICache's 4-byte fetch
 *      group -- offset 2 mod 4 is the entry0-carry case, completed by the
 *      NEXT cycle's fetch;
 *   2. a 32-bit instruction whose low half is the LAST halfword of a 64-byte
 *      cache LINE (offset 62 mod 64) -- its high half comes from the NEXT
 *      line, so on a cold cache it arrives a full refill later, potentially
 *      from a completely different AXI beat;
 *   3. a redirect landing on the UPPER halfword of an aligned 4-byte fetch
 *      group (an odd 2-byte offset) -- ICache's `icache_ipack_unalign`
 *      (icache_pa[1]) has to suppress that cycle's entry1 (the group's LOW
 *      half is NOT what was requested) without losing entry2 (which IS).
 * Phase 1 below walks case 1 systematically; phase 2 places case 2 by hand
 * with PAD_AT so the addresses are not at the mercy of the assembler; phase
 * 3 exercises case 3 via an explicit indirect redirect.
 *
 * Nothing here is a control transfer except the sentinel and phase 3's one
 * exercising jump, so the expected committed stream is pure address order
 * (with phase 3's one jump target spliced in) -- a divergence can only mean
 * a fetch group was split in the wrong place.
 */

#include "macros.h"

    M1_TEXT_START

    /*---------------------------------------------------------------------
     * Phase 1 (0x000..0x2FF): 6-byte blocks (c.nop + one 32-bit instruction).
     * gcd(6,4)=2, so the 32-bit instruction's start offset mod 4 alternates
     * every block (0, 2, 0, 2, ...) -- both entry-boundary phases, back to
     * back, across many 64-byte lines (0x2FF spans twelve lines).
     *--------------------------------------------------------------------*/
    .rept 128
    .option push
    .option rvc
    c.nop                       /* 2 */
    .option norvc
    addi  x1, x1, 1             /* 4 */
    .option pop
    .endr

    /*---------------------------------------------------------------------
     * Phase 2 (0x300..0x4FF): 10-byte blocks (three RVC + one 32-bit) --
     * gcd(10,4)=2 again, but a different absolute residue than phase 1's
     * blocks, and the 32-bit instruction sits in a different position
     * relative to its own block, so it visits offsets phase 1 never lands
     * the SAME way (interleaved with different neighbours on each side).
     *--------------------------------------------------------------------*/
    .rept 32
    .option push
    .option rvc
    c.nop                       /* 2 */
    .option norvc
    lui   x3, 0x1               /* 4 */
    .option rvc
    c.mv  x4, x5                /* 2 */
    c.li  x7, 3                 /* 2 */
    .option pop
    .endr

    /*---------------------------------------------------------------------
     * Phase 3 (0x500..): hand-placed 64-byte LINE straddles. Every PAD_AT
     * offset below has `off & 0x3F == 0x3E`: the 32-bit instruction's LOW
     * halfword is the very last halfword of one cache line, so its HIGH
     * halfword is the first halfword fetched from the NEXT line -- on a
     * cold cache the two halves come from two different refills. Gaps are
     * M1_FILL (`c.addi x2,0`), a 16-bit non-control instruction, so falling
     * through them is harmless.
     *--------------------------------------------------------------------*/
    PAD_AT(0x53E, strad_line_a)
    .option push
    .option norvc
    addi  x6, x6, 1             /* 0x53E..0x541: crosses the line at 0x540 */
    .option pop

    PAD_AT(0x57E, strad_line_b)
    .option push
    .option norvc
    addi  x7, x7, 2             /* 0x57E..0x581: crosses the line at 0x580 */
    .option pop

    /* Back-to-back: a 16-bit instruction as the LAST halfword of a line
     * (0x5BE), so the next line starts perfectly aligned, immediately
     * followed by ANOTHER line straddle one line later -- the entry0 carry
     * register is drained (no straddle) then reloaded (a fresh straddle) in
     * consecutive lines. */
    PAD_AT(0x5BE, last_hw_16)
    .option push
    .option rvc
    c.nop                       /* 0x5BE..0x5BF: fills the line exactly */
    .option norvc
    addi  x8, x8, 3             /* 0x5C0..0x5C3: aligned start of a new line */
    .option pop

    PAD_AT(0x5FE, strad_line_c)
    .option push
    .option norvc
    addi  x9, x9, 4             /* 0x5FE..0x601: crosses the line at 0x600 */
    .option rvc
    c.nop                       /* 0x602 */
    .option norvc
    addi  x10, x10, 5           /* 0x604 */
    .option pop

    /* Two line straddles one after another with only RVC in between, so the
     * entry0 carry register is loaded, drained, and reloaded across three
     * consecutive lines. */
    PAD_AT(0x63E, strad_line_d)
    .option push
    .option norvc
    addi  x11, x11, 6           /* 0x63E..0x641 */
    .option rvc
    c.nop                       /* 0x642 */
    .option pop

    PAD_AT(0x67E, strad_line_e)
    .option push
    .option norvc
    addi  x12, x12, 7           /* 0x67E..0x681 */
    .option pop

    /*---------------------------------------------------------------------
     * Phase 4 (0x700..): a redirect landing on the UPPER halfword of a
     * 4-byte fetch group (an odd-2-byte-aligned target -- legal per
     * IALIGN=16). Exercises ICache's `icache_ipack_unalign` path: the
     * fetch group's low halfword (whatever precedes the target in memory)
     * must NOT be delivered as a spurious entry1 -- only entry2 (the actual
     * target) may commit.
     *--------------------------------------------------------------------*/
    PAD_AT(0x700, unalign_site)
    .option push
    .option norvc
    jal   x0, unalign_target + 2  /* target = 0x706, offset 2 mod 4 */
    .option pop

    PAD_AT(0x704, unalign_target)
    .option push
    .option rvc
    c.nop                        /* 0x704: NEVER committed (skipped over) */
unalign_landing:
    c.li  x13, 5                 /* 0x706: the actual (unaligned) landing */
    .option pop

    /*---------------------------------------------------------------------
     * Phase 5 (0x740..): steady-state runs long enough to cross several
     * lines with no redirects -- pure RVC, then pure 32-bit, then an
     * instruction-for-instruction alternation, the three steady states of
     * the live boundary logic.
     *--------------------------------------------------------------------*/
    PAD_AT(0x740, rvc_run)
    .option push
    .option rvc
    .rept 128
    c.addi x2, 1                /* 2 bytes x 128 = 256 bytes, 4 lines */
    .endr
    .option pop

    .option push
    .option norvc
    .rept 64
    ori   x14, x14, 3           /* 4 bytes x 64 = 256 bytes, 4 lines */
    .endr
    .option pop

    .rept 64
    .option push
    .option rvc
    c.li  x15, 1                /* 2 */
    .option norvc
    xori  x16, x16, 5           /* 4 */
    .option rvc
    c.li  x17, 2                /* 2 */
    .option norvc
    andi  x18, x18, 7           /* 4 */
    .option pop
    .endr

    SENTINEL

    M1_TOHOST_DATA
