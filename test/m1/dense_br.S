/* dense_br.S - dense conditional branches: forward, backward, back to back
 *
 * M1 spec S4.2's "dense conditional branches (forward/backward, back-to-back,
 * more than one per fetch group where the 2-wide fetch bundle allows it)".
 * With every predictor off (rung 1), branch direction resolves entirely
 * through FetchSink's fake BJU (rtl/FetchSink.v: `taken = ^pc[7:4]`,
 * committed at the branch's own commit cycle) -- there is no BHT to train
 * yet (Task 8), so this file's rung-1 job is the redirect/squash-window
 * path: every taken branch is a full flush+refetch (IFU.v's `ipack_buf_
 * flush`/`ibuf_flush_en`, both driven directly off `iu_ifu_tar_pc_vld`),
 * and this is re-run unmodified once the BHT exists.
 *
 * THE DIRECTION RULE AND WHY BACKWARD BRANCHES NEED A GADGET
 * ----------------------------------------------------------
 * A conditional branch is TAKEN iff `^pc[7:4]` -- the parity of four bits of
 * its OWN address (spec S4.1, macros.h). Two consequences drive the layout:
 *
 *   1. Direction depends only on the PC, and M1 has no execute stage, so a
 *      taken BACKWARD branch would re-run the same path forever. Backward
 *      coverage therefore uses the same gadget as jal_chain.S: jump INTO the
 *      block past its target T, fall onto the branch, take it back to T, and
 *      let T's block jump forward past the branch. Every instruction runs
 *      once and the redirect really is negative.
 *   2. "More than one branch per fetch group": C906's ICache delivers only
 *      2 halfwords/cycle (IFU notes S4.1, NOT C910's 8), so the only way to
 *      get more than one CONDITIONAL branch resolved from the SAME cycle's
 *      fetch is two back-to-back COMPRESSED branches (4 bytes total, one
 *      32-bit-fetch-group). Region C below is exactly that.
 *
 * Everything outside the backward gadgets targets its own fall-through or a
 * label ahead of it, so it is safe whichever way the rule resolves; the
 * gadgets are the only places that need a hand-computed parity, and each one
 * says which nibble it relies on.
 */

#include "macros.h"

    M1_TEXT_START

    /*---------------------------------------------------------------------
     * Region A (0x000..0x1FF): one 32-bit branch per 8 bytes, target = 8
     * bytes ahead (skip the nop). The 8-byte stride flips pc[7:4] every
     * second block, so taken and not-taken alternate in long runs.
     *--------------------------------------------------------------------*/
    .option push
    .option norvc
    .rept 64
    beq   x0, x0, .+8
    nop
    .endr
    .option pop

    /*---------------------------------------------------------------------
     * Region B (0x200..0x2FF): one 32-bit branch every 4 bytes, each
     * targeting its own fall-through -- the densest legal 32-bit packing,
     * so every row's branch resolves independently back to back.
     *--------------------------------------------------------------------*/
    .option push
    .option norvc
    .rept 64
    beq   x0, x0, .+4
    .endr
    .option pop

    /*---------------------------------------------------------------------
     * Region C (0x300..0x3FF): TWO compressed conditionals per ICache fetch
     * group (4 bytes = one cycle's worth of fetch, IFU notes S4.1) -- the
     * one way to put more than one branch resolve in a single fetch group
     * on this front end. Each targets its own fall-through.
     *--------------------------------------------------------------------*/
    .option push
    .option rvc
    .rept 128
    c.beqz x8, .+2
    .endr
    .option pop

    /*---------------------------------------------------------------------
     * Region D (0x400..0x5FF): mixed lengths and directions inside one
     * fetch group -- both compressed branch encodings and both taken forms,
     * interleaved with 32-bit ones, so entry-boundary classification and
     * branch resolution have to agree about which halfwords are branches.
     *--------------------------------------------------------------------*/
    .rept 32
    .option push
    .option rvc
    c.beqz x8,  .+2             /* 2 */
    .option norvc
    beq   x0, x0, .+4           /* 4 */
    .option rvc
    c.bnez x9,  .+2             /* 2 */
    .option norvc
    bne   x0, x0, .+4           /* 4 */
    .option rvc
    c.beqz x10, .+2             /* 2 */
    c.bnez x11, .+2             /* 2 */
    .option pop
    .endr

    /*---------------------------------------------------------------------
     * Region E: conditionals that STRADDLE the 64-byte cache LINE boundary.
     * Each PAD_AT offset has `off & 0x3F == 0x3E`, so the branch's low
     * halfword is the last halfword of one line and its high halfword (and
     * the entry0 carry it rides in) comes from the next line/refill. Gaps
     * are M1_FILL and are executed (harmless c.addi x2,0).
     *--------------------------------------------------------------------*/
    PAD_AT(0x63E, strad_br_a)
    .option push
    .option norvc
    beq   x0, x0, .+4
    .option pop

    PAD_AT(0x67E, strad_br_b)
    .option push
    .option norvc
    bne   x0, x0, .+4
    .option pop

    PAD_AT(0x6BE, strad_br_c)
    .option push
    .option norvc
    beq   x0, x0, .+4
    .option pop

    PAD_AT(0x6FE, strad_br_d)
    .option push
    .option norvc
    bne   x0, x0, .+4
    .option pop

    /* A long forward TAKEN branch that crosses several cache lines, into
     * region F below (0x710 -> nibble 1 -> parity 1 -> taken). */
    PAD_AT(0x710, far_br)
    .option push
    .option norvc
    beq   x0, x0, g1_entry
    .option pop

    /*---------------------------------------------------------------------
     * Region F: the backward gadgets. Each is entered by a forward jump
     * landing ON the branch, so the branch's target block below it is
     * reached only by the backward redirect and escapes forward afterwards.
     *--------------------------------------------------------------------*/

    /* g1: branch at 0x870 -> pc[7:4] = 0x7 = 0b0111, parity 1 -> TAKEN. */
    PAD_AT(0x800, g1_target)
    .option push
    .option norvc
    addi  x20, x20, 1
    addi  x20, x20, 2
    jal   x0, g1_escape         /* forward, past the branch */
    .option pop

    PAD_AT(0x870, g1_entry)
    .option push
    .option norvc
    beq   x0, x0, g1_target     /* backward 0x70 bytes, TAKEN */
    .option pop

    PAD_AT(0x880, g1_escape)
    .option push
    .option norvc
    jal   x0, g2_entry
    .option pop

    /* g2: branch at 0x910 -> pc[7:4] = 0x1, parity 1 -> TAKEN, short hop. */
    PAD_AT(0x900, g2_target)
    .option push
    .option norvc
    addi  x21, x21, 1
    jal   x0, g2_escape
    .option pop

    PAD_AT(0x910, g2_entry)
    .option push
    .option norvc
    bne   x0, x0, g2_target     /* backward 0x10 bytes, TAKEN */
    .option pop

    PAD_AT(0x920, g2_escape)
    .option push
    .option norvc
    jal   x0, g3_entry
    .option pop

    /* g3: the COMPRESSED backward branch. c.beqz reaches +-256 bytes, so
     * the hop is 0x20; branch at 0xA20 -> pc[7:4] = 0x2, parity 1 -> TAKEN. */
    PAD_AT(0xA00, g3_target)
    .option push
    .option rvc
    c.addi x22, 1
    .option norvc
    jal   x0, g3_escape
    .option pop

    PAD_AT(0xA20, g3_entry)
    .option push
    .option rvc
    c.beqz x8, g3_target        /* backward 0x20 bytes, TAKEN */
    .option pop

    PAD_AT(0xA40, g3_escape)
    .option push
    .option norvc
    jal   x0, g4_entry
    .option pop

    /* g4: a long backward hop across two cache lines; branch at 0xB80 ->
     * pc[7:4] = 0x8 = 0b1000, parity 1 -> TAKEN. */
    PAD_AT(0xB00, g4_target)
    .option push
    .option norvc
    addi  x23, x23, 1
    jal   x0, g4_escape
    .option pop

    PAD_AT(0xB80, g4_entry)
    .option push
    .option norvc
    beq   x0, x0, g4_target     /* backward 0x80 bytes, TAKEN */
    .option pop

    PAD_AT(0xBC0, g4_escape)
    .option push
    .option norvc
    jal   x0, g5_entry
    .option pop

    /* g5: a backward branch that is NOT taken -- pc[7:4] = 0x0 at 0xC00,
     * parity 0. Exercises the negative B/CB immediate on both oracles
     * without ever redirecting, and falls straight through. */
    PAD_AT(0xC00, g5_entry)
    .option push
    .option norvc
    beq   x0, x0, g4_target     /* backward, NOT taken */
    bne   x0, x0, g1_target     /* backward, NOT taken */
    .option rvc
    c.beqz x8, g4_escape        /* backward 0x44 bytes, NOT taken (compressed) */
    .option norvc
    jal   x0, g6_entry
    .option pop

    /* g6: two conditionals in one row where the FIRST is taken backwards --
     * the redirect must discard the rest of the row before it is ever
     * resolved (the squash rule, spec S4.1). Row at 0xD10 -> pc[7:4]=0x1,
     * parity 1 -> TAKEN. */
    PAD_AT(0xD00, g6_target)
    .option push
    .option norvc
    addi  x24, x24, 1
    jal   x0, g6_escape
    .option pop

    PAD_AT(0xD10, g6_entry)
    .option push
    .option norvc
    beq   x0, x0, g6_target     /* TAKEN backwards; the rest of the row dies */
    bne   x0, x0, g6_target
    beq   x0, x0, g6_target
    bne   x0, x0, g6_target
    .option pop

    PAD_AT(0xD40, g6_escape)

    SENTINEL

    M1_TOHOST_DATA
