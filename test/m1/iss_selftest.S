/* iss_selftest.S - the golden fragment for the ISS's own unit test (plan
 * Task 5.3). Assembled here and disassembled with objdump to get REAL
 * addresses/encodings (not hand-guessed ones); the expected committed
 * stream embedded in m1_iss.h's selftest() is derived from that real
 * disassembly by applying the plan's "Global contracts" BY HAND -- direction
 * = ^pc[7:4], JR_TARGET(pc), x1-only pcall/preturn -- never by running the
 * ISS itself. See m1_iss.h's selftest() comment block for the derivation.
 *
 * Coverage: RVC/32-bit mix with a straddle across a 4-byte boundary; all
 * four (direction x encoding) branch combinations -- B-type taken/not-taken,
 * CB-type (c.beqz/c.bnez) taken/not-taken; a direct jump (c.j / jal x0); a
 * call/return nest two deep (x1-only); a return with an EMPTY shadow stack;
 * a plain indirect jump (c.jr through a non-x1 register) landing at its
 * JR_TARGET pad; an INDIRECT CALL through a non-x1 register (rd==x1,
 * rs1!=x1 -- pushes AND uses JR_TARGET, a case that cannot occur on
 * rv12/C910's x1/x5 dual-link convention); and rv906's x1-only "dual" case
 * `jalr x1, 0(x1)` (rd==rs1==x1), which C906's own aq_iu_bju.v excludes from
 * preturn, so it must classify as push-only with a JR_TARGET destination.
 */

#include "macros.h"

M1_TEXT_START

    /* ---- RVC/32-bit mix, with a straddle across the 0x004 boundary ---- */
    c.nop
    addi   x5, x5, 1
    c.li   a0, 0
    lui    a1, 1
    c.nop

    /* ---- four branch combinations, each pinned to a chosen pc[7:4] ----
     * n = (pc>>4)&0xF, taken = parity(n): n=0 even(NOT), n=1 odd(TAKEN),
     * n=2 odd(TAKEN), n=3 even(NOT). .balign 256 gives a known n=0 base;
     * each branch + its tiny filler fits well inside 16 bytes, so four
     * consecutive `.balign 16` calls advance n by exactly 1 each time. The
     * jump ahead of it is load-bearing: `.balign` alone would leave a long
     * run of alignment-fill NOPs that the CPU would otherwise commit one at
     * a time falling through from the prologue. */
    jal    x0, br4
    .balign 256
br4:
    c.bnez a0, branch_wrong_target /* CB-type, n=0 -> NOT taken: falls
                                       through into br1 (below). The taken
                                       target is a shared, otherwise
                                       unreachable trap -- if this branch's
                                       direction were (bug) evaluated taken,
                                       the committed stream would visibly
                                       diverge from the not-taken golden
                                       entries instead of coincidentally
                                       landing on the same PC either way. */
    c.nop                          /* COMMITTED (br4's fallthrough) */
    .balign 16
br1:
    beq    x0, x0, br2             /* B-type,  n=1 -> TAKEN: jump into br2
                                       so it is visited too, instead of
                                       skipping straight to post_branches */
    c.nop                          /* never committed */
    .balign 16
br2:
    c.beqz s0, br3                 /* CB-type, n=2 -> TAKEN: jump into br3
                                       for the same reason */
    c.nop                          /* never committed */
    .balign 16
br3:
    bne    x0, x0, branch_wrong_target /* B-type, n=3 -> NOT taken (falls
                                       through to post_branches -- see the
                                       br4 comment above for why the taken
                                       target must differ from the
                                       fallthrough address) */
post_branches:
    c.j    jal_site

    /* Shared trap for br3/br4's taken-but-should-never-be-taken target.
     * `post_branches`'s c.j is unconditional (ab_br is always taken), so
     * this is unreachable on the correct path -- reached only if the ISS's
     * direction rule is buggy. */
branch_wrong_target:
    c.nop

    /* ---- direct jump (ab_br), rd=x0 -> never pushes ---- */
jal_site_filler:
    c.nop                          /* never committed */
    c.nop
jal_site:
    jal    x0, direct_target
direct_target_filler:
    c.nop                          /* never committed */
direct_target:
    c.nop

    /* ---- call/return nest, two deep, x1-only ---- */
call1:
    jal    x1, callee1             /* pcall: push after_call1 */
after_call1:
    jal    x0, after_nest          /* hop over the callee bodies */

callee1_filler:
    c.nop
callee1:
    c.nop
call2:
    jal    x1, callee2             /* pcall: push after_call2 */
after_call2:
    c.jr   x1                      /* preturn: pop -> after_call1 */

callee2_filler:
    c.nop
callee2:
    c.nop
    c.jr   x1                      /* preturn: pop -> after_call2 */

    /* ---- return with an EMPTY shadow stack: pop-on-empty falls through -
     * forced 32-bit (not c.jr) for coverage of the 32-bit jalr preturn
     * path too. ---- */
after_nest:
    .option push
    .option norvc
    jalr   x0, 0(x1)
    .option pop
after_empty_ret:
    c.nop

    /* ---- plain indirect jump: c.jr through a non-x1 register ---- */
ind_jr_site:
    c.jr   x6
    JR_PAD(ind_jr_site, ind_jr_pad)
ind_jr_pad:
    c.nop

    /* ---- indirect CALL through a non-x1 register: pcall (dst==x1) AND
     * ind_br (src!=x1) on the SAME instruction -- forced 32-bit so it
     * cannot auto-compress to c.jalr x2. ---- */
ind_call_site:
    .option push
    .option norvc
    jalr   x1, 0(x2)
    .option pop
after_ind_call:
    c.nop
    jal    x0, past_ind_call_pad
    JR_PAD(ind_call_site, ind_call_pad)
ind_call_pad:
    c.jr   x1                      /* preturn: pop -> after_ind_call */
past_ind_call_pad:
    c.nop

    /* ---- x1-only "dual" case: jalr x1,x1,0 (rd==rs1==x1). Excluded from
     * preturn by aq_iu_bju.v's src_dst_reg_equal term -- push-only, target
     * = JR_TARGET. Forced 32-bit so it cannot auto-compress to c.jalr x1. */
dual_site:
    .option push
    .option norvc
    jalr   x1, 0(x1)
    .option pop
after_dual:
    c.nop
    jal    x0, past_dual_pad
    JR_PAD(dual_site, dual_pad)
dual_pad:
    c.jr   x1                      /* preturn: pop -> after_dual */
past_dual_pad:
    c.nop

    SENTINEL

M1_TOHOST_DATA
