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
    .option push
    .option norvc
    jal   x0, bht_call_taken
    .option pop

    /*---------------------------------------------------------------------
     * Region G (0xD80+): shared-subroutine BHT training (plan Task 9.3).
     *
     * Regions A-F above each visit a distinct branch PC exactly ONCE --
     * every one of THEIR conditional branches is a cold, first-touch
     * lookup: the counter it hits starts at reset state, gets read once,
     * updated once, and is never read again by this test. Since
     * `taken = ^pc[7:4]` makes the correct answer a pure function of PC
     * (M1 spec's global contracts), no branch in a straight-line test can
     * ever be executed twice without an explicit shared-subroutine
     * construct -- the exact coverage gap rv12 found and fixed for C910's
     * BHT, re-checked explicitly here rather than assumed away (Task 9.3).
     * This region closes it.
     *
     * Two tiny subroutines, each with ONE conditional branch whose
     * direction is fixed by its own address's parity, are each called
     * back-to-back 24 times from a straight chain of `jal x1, sub` sites
     * (call, resolve the branch, `ret` straight back to the NEXT call
     * site -- RAS depth never exceeds 1, independent of callret.S's own
     * depth testing). BHT indexing is PURE GHR, no PC term at all
     * (extraction note S1.4) -- so repeating the SAME outcome every visit
     * shifts the SAME bit into the GHR every time. The read window that
     * matters is a contiguous 12 bits (`bht_vghr[11:0]`, rtl/BPU.v SECTION
     * BHT part (f)); after at most 12 identical shifts those 12 bits
     * converge to a fixed pattern and hold it, so every call from the
     * 13th onward reads+writes the EXACT SAME physical SRAM row/lane --
     * genuine training (0->1->2->3-saturate for the always-taken sub,
     * 3->2->1->0-saturate for the always-not-taken sub, each followed by
     * several steady-state re-reads confirming the saturated value holds).
     * 24 calls per sub leaves a comfortable margin over the 12-call
     * worst-case convergence bound. The two bursts run back-to-back on
     * purpose (GHR is one global register, not reset between them): the
     * not-taken burst starts from the taken burst's converged all-1s
     * state and has to re-converge to all-0s, exercising the counter
     * recovering from an opposite-trained state too.
     *--------------------------------------------------------------------*/
    PAD_AT(0xD80, bht_call_taken)
    .option push
    .option norvc
    .rept 24
    jal   x1, bht_sub_taken
    .endr
    jal   x0, bht_call_nottaken
    .option pop

    /* bht_sub_taken: branch at 0xE10 -> pc[7:4]=0x1=0b0001, parity 1 ->
     * TAKEN. The interposed `jal` after the branch is dead code on this
     * (fixed, always-taken) address but keeps the pattern reusable as-is
     * for bht_sub_nottaken below regardless of which parity that address
     * happens to have. */
    PAD_AT(0xE10, bht_sub_taken)
    .option push
    .option norvc
    beq   x0, x0, bht_sub_taken_ret
    jal   x0, bht_sub_taken_ret
bht_sub_taken_ret:
    jalr  x0, 0(x1)
    .option pop

    /* bht_sub_nottaken: branch at 0xE30 -> pc[7:4]=0x3=0b0011, parity 0 ->
     * NOT TAKEN; falls through into the `jal` below, which still lands on
     * the same ret label. */
    PAD_AT(0xE30, bht_sub_nottaken)
    .option push
    .option norvc
    beq   x0, x0, bht_sub_nottaken_ret
    jal   x0, bht_sub_nottaken_ret
bht_sub_nottaken_ret:
    jalr  x0, 0(x1)
    .option pop

    PAD_AT(0xE50, bht_call_nottaken)
    .option push
    .option norvc
    .rept 24
    jal   x1, bht_sub_nottaken
    .endr
    jal   x0, bht_train_done
    .option pop

    PAD_AT(0xEC0, bht_train_done)

    SENTINEL

    M1_TOHOST_DATA
