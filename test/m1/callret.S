/* callret.S - call/return nesting and the pcall/preturn classification corners
 *
 * M1 spec S4.2's "call/ret nests at and just past 4 deep -- the RAS's real
 * limit (S4.1) -- NOT past-12 the way rv12's C910 clone deliberately
 * overflowed a 12-entry RAS". rv906's plan explicitly diverges from rv12's
 * approach here (plan Task 6.1): C906's RAS is only 4 entries deep, and its
 * shipped depth-limitation behaviour (no content resync on mispredict, see
 * design doc S2.1/S4.1) only becomes gradeable once BPU.v's real RAS lands
 * in Task 7 -- this file is authored NOW, at the exact depth Task 7 needs,
 * and re-run unmodified at every later rung. At RUNG 1 there is no real RAS
 * at all yet (every predictor is off): calls/returns resolve purely through
 * FetchSink's own 16-entry shadow call stack (rtl/FetchSink.v SECTION SHADOW
 * CALL STACK) and the C++ golden ISS's matching 16-entry stack (m1_iss.h) --
 * both far deeper than 4, so depth 4-6 is well inside their shared "ground
 * truth" range and this file's rung-1 job is simply to prove push/pop
 * ordering and the pcall/preturn CLASSIFICATION corners are right, not the
 * RAS's own depth-limit fidelity (that grading hook is explicitly deferred
 * to Task 7.1, m1_iss.h's own header comment).
 *
 * CLASSIFICATION (x1-ONLY -- confirmed divergence from C910, rtl/FetchSink.v
 * TASK 4.1 FINDINGS item 2 / m1_iss.h's header, both read independently from
 * aq_iu_bju.v):
 *   pcall   = dst_reg == x1 (jal/jalr/c.jalr; c.jalr's rd is hardwired x1)
 *   preturn = jalr-family AND src_reg == x1 AND NOT(src_reg == dst_reg)
 * There is no C910-style x5 alternate link register, and -- unlike C910 --
 * preturn does NOT check the immediate at all: `jalr x5, 4(x1)` (rd=x5,
 * rs1=x1, imm=4) IS a preturn on C906, where it would NOT be on C910 (whose
 * predecode table additionally requires imm==0). Phase 3 below is a direct,
 * deliberate test of that divergence -- getting it wrong is exactly the kind
 * of bug two independently-derived oracles (FetchSink.v vs. the C++ ISS)
 * would still agree on by sharing a mistake, so it is worth pinning here.
 * The only case preturn excludes is rd==rs1==x1 (both link-shaped): C906 has
 * no x1/x5 dual pop-then-push the way C910's `c.jalr x5`/`jalr x5,0(x1)`
 * needs (rv906 has one link register, so the "looks like both" case is
 * always src==dst==x1, and the exclusion term kills preturn outright,
 * leaving a push-only call to JR_TARGET) -- phases 4/5 below.
 */

#include "macros.h"

    M1_TEXT_START

    /*---------------------------------------------------------------------
     * Phase 1 (0x000): POP ON EMPTY. Three different return encodings run
     * before anything has been pushed; each must fall through to its own
     * fall-through PC (spec S4.1's defined pop-on-empty rule).
     *--------------------------------------------------------------------*/
    .option push
    .option norvc
    jalr  x0, 0(x1)             /* 0x000: preturn via x1, stack empty */
    .option rvc
    c.jr  x1                    /* 0x004 */
    .option norvc
    jalr  x3, 4(x1)             /* 0x006: preturn (imm!=0, dst=x3 -- both
                                    irrelevant to C906's rule, see header) */
    .option pop

    /*---------------------------------------------------------------------
     * Phase 2 (0x00E): a nest 4 deep, then one MORE call/return pair (depth
     * 5) -- "at and just past" the real RAS's 4-entry limit. The outer call
     * primes the stack with esc1 so the outermost return escapes forward
     * instead of falling back into the chain (with no execute stage, a
     * return into the chain body would re-run it forever).
     *--------------------------------------------------------------------*/
    .option push
    .option norvc
    jal   x1, nest_start        /* push esc1 -> stack depth 1 */
esc1:
    jal   x0, phase3            /* reached by the outermost `ret` */

nest_start:
    .rept 5                     /* pushes bring depth to 2,3,4,5,6 in turn
                                    (esc1 already primed depth 1) -- hits the
                                    RAS's real 4-entry limit exactly and goes
                                    2 past it, per the header note */
    jal   x1, .+8               /* call the next frame, push its `ret` */
    jalr  x0, 0(x1)             /* that frame's `ret` */
    .endr
    jalr  x0, 0(x1)             /* kicks off the unwind: stack is at its
                                    deepest (6) right before this pops */
    .option pop

    /*---------------------------------------------------------------------
     * Phase 3 (0x300): the divergence-from-C910 case, `jalr x5, 4(x1)`
     * (rd=x5, rs1=x1, imm=4). On C906 this is a preturn (pop) despite the
     * non-zero immediate and non-link destination register.
     *--------------------------------------------------------------------*/
    PAD_AT(0x300, phase3)
    .option push
    .option norvc
    jal   x1, cA_site           /* push cA_after */
    .option pop

    PAD_AT(0x340, cA_site)
    .option push
    .option norvc
    jalr  x5, 4(x1)             /* preturn: pops cA_after (imm/rd ignored) */
cA_after:
    jal   x0, dual32_site
    .option pop

    /*---------------------------------------------------------------------
     * Phase 4 (0x400): the x1-only "dual" case, 32-bit form: `jalr x1,0(x1)`
     * (rd==rs1==x1). Excluded from preturn by the src==dst term -- a CALL
     * ONLY, target = JR_TARGET(pc), and the fall-through is still pushed;
     * the landing pad returns through it.
     *--------------------------------------------------------------------*/
    PAD_AT(0x400, dual32_site)
    .option push
    .option norvc
    jalr  x1, 0(x1)             /* pcall only -> JR_TARGET(0x400)=0x440;
                                    push dual32_after */
dual32_after:
    jal   x0, dualc_site            /* reached by the pad's return */
    .option pop

    JR_PAD(dual32_site, dual32_pad)
    .option push
    .option norvc
    jalr  x0, 0(x1)             /* preturn: pops dual32_after */
    .option pop

    /*---------------------------------------------------------------------
     * Phase 5 (0x500): the RVC form of the dual case, `c.jalr x1` (rd is
     * hardwired x1 by the encoding, rs1=x1) -- same push-only/JR_TARGET
     * behaviour as phase 4, compressed.
     *--------------------------------------------------------------------*/
    PAD_AT(0x500, dualc_site)
    .option push
    .option rvc
    c.jalr x1                   /* pcall only -> JR_TARGET(0x500)=0x540;
                                    push dualc_after */
dualc_after:
    .option norvc
    jal   x0, indcall_site
    .option pop

    JR_PAD(dualc_site, dualc_pad)
    .option push
    .option rvc
    c.jr  x1                    /* preturn: pops dualc_after */
    .option pop

    /*---------------------------------------------------------------------
     * Phase 6 (0x600): an INDIRECT CALL through a non-x1 register --
     * pcall (dst==x1) AND ind_br (src!=x1) on the SAME instruction: pushes
     * AND targets JR_TARGET(pc), never the popped value.
     *--------------------------------------------------------------------*/
    PAD_AT(0x600, indcall_site)
    .option push
    .option norvc
    jalr  x1, 0(x2)              /* pcall + ind_br -> JR_TARGET(0x600)=0x640;
                                     push indcall_after */
indcall_after:
    jal   x0, done
    .option pop

    JR_PAD(indcall_site, indcall_pad)
    .option push
    .option norvc
    jalr  x0, 0(x1)              /* preturn: pops indcall_after */
    .option pop

    PAD_AT(0x700, done)

    SENTINEL

    M1_TOHOST_DATA
