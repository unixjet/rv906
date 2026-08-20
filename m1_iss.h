//=============================================================================
// m1_iss.h - the C++ golden fetch-ISS for rv906 M1 (plan Task 5.1)
//=============================================================================
// THE SECOND ORACLE. Written from the SPEC TEXT
// (docs/superpowers/specs/2026-08-20-m1-ifu-design.md S4.1, the plan's
// "Global contracts" section) plus an INDEPENDENT reading of C906's own
// gen_rtl/iu/rtl/aq_iu_bju.v -- NOT from rtl/FetchSink.v. FetchSink.v and
// this file are cross-checking oracles built from the same spec by two
// different paths; a bug shared between them would validate nothing, so
// nothing in here may be "fixed" by looking at what the RTL does -- a
// disagreement is a bug report, not a patch target. (FetchSink.v's own
// classification comment block was read AFTER this file's decode logic was
// derived, purely to sanity-check that two independent readings of
// aq_iu_bju.v agree -- they do, see the note below.)
//
// WHAT IT MODELS
// --------------
// M1 has no execute stage, so the architectural instruction stream is
// defined by the harness contract, not by data flow. C906 delivers exactly
// ONE instruction per cycle to IDU (design doc S4.1) -- unlike C910/rv12's
// own M1 clone's 3-slot bundle -- so the committed stream here is a flat
// sequence, never slotted. Every control transfer has a deterministic,
// PREDICTOR-INDEPENDENT outcome:
//
//   conditional branch   taken <=> ^pc[7:4] (parity of the byte-address
//                        bits 7..4 of the BRANCH's own PC); taken target =
//                        the B/CB immediate, not-taken target = the
//                        fall-through. The actual register operands are
//                        decorative -- nothing in M1 ever reads them.
//   jal / c.j            target = the J/CJ immediate. Always "taken".
//   call (pcall)         a jal/jalr/c.jalr whose DESTINATION register is x1
//                        pushes the fall-through PC onto a 16-entry shadow
//                        call stack.
//   return (preturn)     a jalr-family instruction whose SOURCE register is
//                        x1, provided it is not also the pcall "dual" case
//                        below: target = shadow-stack pop; pop on an EMPTY
//                        stack returns the fall-through PC (spec S4.1,
//                        defined and deterministic).
//   other jalr-family    target = JR_TARGET(pc), a fixed formula of the
//                        jump's own PC. M1 has no register file to compute
//                        a real ALU target from, so this is the model's
//                        necessary substitute for every jalr-family
//                        instruction that is not a preturn: a plain
//                        indirect jump, an indirect CALL through a non-x1
//                        register (pushes AND uses JR_TARGET), or rv906's
//                        x1-only "dual" case below. The .S tests place
//                        landing pads exactly there.
//
// CLASSIFICATION (independently derived from aq_iu_bju.v:637-667 --
// bju_link_vld_raw, bju_ret_vld_raw, bju_src_dst_reg_equal -- cross-checked
// against the FUNC_* bit-11 table in idu/rtl/aq_idu_cfig.h:402-450 for what
// "func11" means for JAL/JALR/C.JALR vs C.J/C.JR; independently confirmed to
// match the separate reading recorded in rtl/FetchSink.v's own Task 4.1
// completion note):
//
//   pcall   = dst_reg == x1  AND (jal | jalr | c.jalr)
//   preturn = jalr-family AND src_reg == x1 AND NOT(src_reg == dst_reg)
//             (dst_reg is IMPLICIT x0 for c.jr and IMPLICIT x1 for c.jalr --
//             the encoding hard-wires it, there is no dst field to read)
//   else, if jalr-family at all: JR_TARGET, no push
//
// C906 IS X1-ONLY: unlike C910 (rv12's own M1 clone), aq_iu_bju.v tests
// dst_preg/src0_reg against 5'b1 (x1) exactly, nowhere against x5 -- there
// is no alternate link register. `jal x5, ...` / `jalr x0, x5, 0` do NOT
// touch the shadow stack on this core. A direct, hand-checked consequence:
// `jalr x1, x1, 0` (dst==src==x1) satisfies the pcall condition but is
// EXCLUDED from preturn by the `NOT(src==dst)` term (aq_iu_bju.v's own
// bju_src_dst_reg_equal wire), so it classifies as a push-only call whose
// target is JR_TARGET, never a same-cycle pop-then-push the way C910's
// x1/x5 dual convention (`c.jalr x5`) forces rv12's ISS to model. Checking
// every (src_reg, dst_reg) combination the two conditions can take by hand:
// pcall requires dst==x1, preturn requires src==x1 AND src!=dst -- so if
// both dst==x1 (pcall) AND src==x1 with src!=dst (preturn) were to hold
// simultaneously, src==x1==dst contradicts src!=dst. They are therefore
// MUTUALLY EXCLUSIVE by construction, not by a runtime check -- C906 never
// needs rv12's atomic pop-then-push case.
//
// SHADOW CALL STACK: 16 entries (plan "Global contracts" -- deeper than the
// real 4-entry RAS on purpose, to exercise past the real limit, but not
// absurdly so).
//
// RAS-FAITHFUL GRADING HOOK -- NOT IMPLEMENTED HERE (see RasFaithfulView
// below): the plan's "RAS depth-limitation contract" (design doc S2.1/S4.1)
// requires a SEPARATE comparison path mirroring aq_ifu_ras.v's 4-entry,
// pointer-only misprediction resync (no content resync) once BPU.v's real
// RAS exists, to grade rung-2+ tests near depth 4 against the RAS's actual
// shipped behaviour rather than this general-purpose 16-entry stack. The
// plan explicitly scopes that to Task 7.1 and says not to block Task 5 on
// aq_ifu_ras.v internals not yet needed until rung 2 -- this file lays the
// 16-entry stack down (the rung-1 oracle) and marks the hook; it does not
// fill it in.
//
// INSTRUCTION LENGTH / OPCODE PAYLOAD CONVENTION
// ----------------------------------------------
// RVC-aware: a halfword with [1:0] != 2'b11 is a 16-bit instruction.
// FetchSink/verisim.h deliver a raw 32-bit window for every commit (the low
// halfword is the instruction; for a 16-bit instruction the upper halfword
// is whatever follows in memory -- a don't-care), so Entry::opcode only
// ever carries the bits that matter: the low 16 for an RVC entry, the full
// 32 otherwise. The online checker (RVProcTest.cpp) is responsible for
// masking FetchSink's cmt_opcode the same way before comparing.
//
// LAZINESS
// --------
// next() reads the image through the caller's fetch callback AT THE MOMENT
// each instruction is produced, not up front -- so a self-modifying-code
// test (Task 6's fence.i mechanism) sees whatever the host has patched into
// ExtMem by the time the ISS gets there.
//=============================================================================

#ifndef M1_ISS_H
#define M1_ISS_H

#include <stdint.h>
#include <stdio.h>
#include <functional>
#include <vector>

namespace m1 {

//-----------------------------------------------------------------------------
// Contract constants (plan "Global contracts")
//-----------------------------------------------------------------------------
static const uint64_t RESET_VECTOR       = 0x0000000080000000ULL;
static const uint32_t SENTINEL_OPCODE    = 0x0000006FU;   // jal x0, 0
static const size_t   SHADOW_STACK_DEPTH = 16;             // 16, not rv12's 64

// JR_TARGET(pc): a 64-byte-aligned landing pad 64..256 bytes ahead of the
// jump. pc is the BYTE address of the indirect jump itself.
//   (pc & ~0x3F) + 0x40 + (((pc >> 6) & 0x3) << 6)
static inline uint64_t jr_target(uint64_t pc)
{
    return (pc & ~0x3FULL) + 0x40ULL + (((pc >> 6) & 0x3ULL) << 6);
}

// Conditional-branch direction rule: taken = ^pc[7:4] (parity of the nibble
// at byte-address bits 7..4).
static inline bool branch_taken(uint64_t pc)
{
    uint32_t nib = (uint32_t)((pc >> 4) & 0xFULL);
    nib ^= nib >> 2;
    nib ^= nib >> 1;
    return (nib & 1U) != 0U;
}

//-----------------------------------------------------------------------------
// Immediate decoders (RISC-V ISA encodings; sign-extended to 64 bits). One
// correct transcription of the ISA manual's bit tables -- expect this to
// look like any other independent implementation of the same tables.
//-----------------------------------------------------------------------------
static inline int64_t sext(uint64_t v, unsigned bits)
{
    const uint64_t m = 1ULL << (bits - 1);
    return (int64_t)((v ^ m) - m);
}

// B-type (conditional branch): imm[12|10:5|4:1|11], LSB implicit 0.
static inline int64_t imm_b(uint32_t i)
{
    uint64_t v = (((uint64_t)(i >> 31) & 0x1U)  << 12) |
                 (((uint64_t)(i >> 7)  & 0x1U)  << 11) |
                 (((uint64_t)(i >> 25) & 0x3FU) << 5)  |
                 (((uint64_t)(i >> 8)  & 0xFU)  << 1);
    return sext(v, 13);
}

// J-type (jal): imm[20|10:1|11|19:12], LSB implicit 0.
static inline int64_t imm_j(uint32_t i)
{
    uint64_t v = (((uint64_t)(i >> 31) & 0x1U)   << 20) |
                 (((uint64_t)(i >> 12) & 0xFFU)  << 12) |
                 (((uint64_t)(i >> 20) & 0x1U)   << 11) |
                 (((uint64_t)(i >> 21) & 0x3FFU) << 1);
    return sext(v, 21);
}

// CJ (c.j): offset[11|4|9:8|10|6|7|3:1|5], LSB implicit 0.
static inline int64_t imm_cj(uint32_t h)
{
    uint64_t v = (((uint64_t)(h >> 12) & 0x1U) << 11) |
                 (((uint64_t)(h >> 11) & 0x1U) << 4)  |
                 (((uint64_t)(h >> 9)  & 0x3U) << 8)  |
                 (((uint64_t)(h >> 8)  & 0x1U) << 10) |
                 (((uint64_t)(h >> 7)  & 0x1U) << 6)  |
                 (((uint64_t)(h >> 6)  & 0x1U) << 7)  |
                 (((uint64_t)(h >> 3)  & 0x7U) << 1)  |
                 (((uint64_t)(h >> 2)  & 0x1U) << 5);
    return sext(v, 12);
}

// CB (c.beqz/c.bnez): offset[8|4:3|7:6|2:1|5], LSB implicit 0.
static inline int64_t imm_cb(uint32_t h)
{
    uint64_t v = (((uint64_t)(h >> 12) & 0x1U) << 8) |
                 (((uint64_t)(h >> 10) & 0x3U) << 3) |
                 (((uint64_t)(h >> 5)  & 0x3U) << 6) |
                 (((uint64_t)(h >> 3)  & 0x3U) << 1) |
                 (((uint64_t)(h >> 2)  & 0x1U) << 5);
    return sext(v, 9);
}

//-----------------------------------------------------------------------------
// One committed instruction.
//-----------------------------------------------------------------------------
struct Entry {
    uint64_t pc;
    uint32_t opcode;    // low 16 bits only when rvc; full 32 otherwise
    bool     rvc;

    Entry() : pc(0), opcode(0), rvc(false) {}
};

//-----------------------------------------------------------------------------
// RAS-FAITHFUL GRADING HOOK -- filled in now that BPU.v's real RAS exists to
// mirror (plan Task 7.1; see rtl/BPU.v's RAS section for the aq_ifu_ras.v
// pointer-resync findings this reproduces, and rtl/FetchSink.v's own
// "SECTION RAS-FAITHFUL GRADING MODEL" for the identical model built there).
// DIAGNOSTIC ONLY: the online checker's committed-stream comparison is
// already invariant to RAS prediction accuracy (design doc S4.1's
// "predictors change WHEN, never WHICH"), so this is not consulted by
// next()'s own walk (which keeps using the unbounded `stack` above,
// unconditionally, at every rung -- exactly as it always has). It exists so
// a caller CAN cross-check, instruction by instruction, whether the real
// 4-entry/pointer-only-resync RAS would have predicted a given return
// correctly -- useful for confirming callret.S's Phase 2 (6 unreturned
// calls stacked, 2 past the 4-entry limit) actually exercises the shipped
// limitation the way it is designed to.
//
// Mirrors aq_ifu_ras.v exactly: ONE one-hot 4-bit pointer (push rotates -1
// mod 4 / right-rotate, pop rotates +1 mod 4 / left-rotate), one physical
// 4-entry content array, NO content resync on misprediction, NO
// empty-stack special case (an under-flowed pop just reads whatever is
// physically in the pointed-to entry, zero if never written -- unlike
// Iss::pop()'s own fall-through substitute above, which models FetchSink's
// ACTUAL architectural behavior, not the predictor's). Driven by the same
// push/pop CALL SITES as the unbounded `stack` (see push()/pop() below),
// i.e. the confirmed/committed view -- it cannot see genuine speculative
// wrong-path RAS corruption, which only BPU.v's own ID-stage stream could;
// this is a best-effort cross-check, not a substitute for reading BPU.v's
// trace directly.
//-----------------------------------------------------------------------------
struct RasFaithfulView {
    uint64_t entry[4];
    unsigned pop_idx;   // 0..3, the one-hot pointer's bit position

    RasFaithfulView() : pop_idx(0) { entry[0] = entry[1] = entry[2] = entry[3] = 0; }

    // Returns the target the real RAS would have predicted for THIS pop,
    // sampled before the pointer rotates (aq_ifu_ras.v's own read-then-
    // rotate order), then advances the pointer.
    uint64_t pop()
    {
        uint64_t predicted = entry[pop_idx];
        pop_idx = (pop_idx + 1) & 0x3U;   // left-rotate: +1 mod 4
        return predicted;
    }

    // Pushes fall_through into the slot the pointer is about to rotate
    // INTO (right-rotate: -1 mod 4, i.e. +3 mod 4), then advances the
    // pointer there -- matching aq_ifu_ras.v's "write target = the slot
    // the pointer is rotating into" (BPU notes S3.2).
    void push(uint64_t fall_through)
    {
        pop_idx = (pop_idx + 3U) & 0x3U;   // right-rotate: -1 mod 4
        entry[pop_idx] = fall_through;
    }
};

//-----------------------------------------------------------------------------
// The ISS itself. Iterator style: next() produces one committed
// instruction, so a multi-million-instruction stream never has to be
// materialised.
//-----------------------------------------------------------------------------
struct Iss {
    // Halfword fetch, byte address -> the 16 bits at that address. Supplied
    // by the harness (ExtMem pages) or by the selftest (a literal image).
    typedef std::function<uint16_t(uint64_t)> ReadHalf;

    ReadHalf              rd_half;
    uint64_t              pc;
    std::vector<uint64_t> stack;        // shadow call stack, newest at back()
    uint64_t              count;        // instructions produced so far
    uint64_t              prefix_count; // instructions produced BEFORE the sentinel
    bool                  sentinel_seen;
    bool                  depth_warned;
    RasFaithfulView        ras_view;      // Task 7.1: driven alongside `stack`
                                           // below, diagnostic only (see its
                                           // own header comment)
    bool                   ras_faithful_mispredict; // valid only right after
                                                     // a pop() that popped
    uint64_t               ras_faithful_pred_pc;    // the model's own guess
                                                     // for that same pop

    Iss() : pc(RESET_VECTOR), count(0), prefix_count(0),
            sentinel_seen(false), depth_warned(false),
            ras_faithful_mispredict(false), ras_faithful_pred_pc(0) {}

    void reset(ReadHalf reader, uint64_t start_pc)
    {
        rd_half = reader;
        pc = start_pc;
        stack.clear();
        count = 0;
        prefix_count = 0;
        sentinel_seen = false;
        depth_warned = false;
        ras_view = RasFaithfulView();
        ras_faithful_mispredict = false;
        ras_faithful_pred_pc = 0;
    }

    //-- shadow call stack -----------------------------------------------------
    void push(uint64_t fall_through)
    {
        if (stack.size() >= SHADOW_STACK_DEPTH && !depth_warned) {
            printf("[iss] WARNING: shadow call stack past the %u-entry "
                   "contract depth at pc=%016llx -- rung-1 grading is exact "
                   "here (FetchSink's own stack is the same size), but this "
                   "is NOT what the real 4-entry RAS would do; see the "
                   "RAS-faithful grading hook (Task 7.1)\n",
                   (unsigned)SHADOW_STACK_DEPTH, (unsigned long long)pc);
            depth_warned = true;
        }
        stack.push_back(fall_through);
        ras_view.push(fall_through);
    }

    // Pop-on-empty returns the fall-through PC (spec S4.1, verbatim). The
    // RAS-faithful model has no such rule (real hardware has no empty
    // detection either) -- it is polled unconditionally so
    // ras_faithful_mispredict/ras_faithful_pred_pc are always set from
    // THIS pop, whether or not the real 16-entry ground-truth stack was
    // itself empty.
    uint64_t pop(uint64_t fall_through)
    {
        uint64_t faithful = ras_view.pop();
        uint64_t actual;
        if (stack.empty()) {
            actual = fall_through;
        } else {
            actual = stack.back();
            stack.pop_back();
        }
        ras_faithful_mispredict = (faithful != actual);
        ras_faithful_pred_pc    = faithful;
        return actual;
    }

    //-- the walk ---------------------------------------------------------------
    // Produces the next committed instruction. Always succeeds: after the
    // sentinel it keeps returning the sentinel, which is what the
    // sentinel's own self-jump architecturally means.
    Entry next()
    {
        Entry e;
        uint16_t h0 = rd_half(pc);
        e.pc  = pc;
        e.rvc = ((h0 & 0x3U) != 0x3U);
        e.opcode = e.rvc ? (uint32_t)h0
                          : ((uint32_t)h0 | ((uint32_t)rd_half(pc + 2) << 16));

        if (!sentinel_seen && !e.rvc && e.opcode == SENTINEL_OPCODE) {
            sentinel_seen = true;
            prefix_count = count;
        }
        count++;
        pc = e.rvc ? next_pc_rvc(e.pc, h0) : next_pc_32(e.pc, e.opcode);
        return e;
    }

private:
    //-- 32-bit instructions -----------------------------------------------------
    uint64_t next_pc_32(uint64_t cur, uint32_t inst)
    {
        const uint64_t fall = cur + 4;
        const uint32_t op   = inst & 0x7FU;

        if (op == 0x63U) {                        // BRANCH (B-type)
            return branch_taken(cur) ? (uint64_t)((int64_t)cur + imm_b(inst))
                                      : fall;
        }
        if (op == 0x6FU) {                        // JAL (J-type)
            const uint32_t rd = (inst >> 7) & 0x1FU;
            if (rd == 1U)                          // pcall: direct call
                push(fall);
            return (uint64_t)((int64_t)cur + imm_j(inst));
        }
        if (op == 0x67U) {                        // JALR
            const uint32_t rd  = (inst >> 7)  & 0x1FU;
            const uint32_t rs1 = (inst >> 15) & 0x1FU;
            const bool pcall   = (rd == 1U);
            // func11 (aq_idu_cfig.h) is unconditionally 1 for every 32-bit
            // jalr, so aq_iu_bju.v's src_dst_reg_equal term reduces here to
            // plain rs1 == rd (see the classification note above).
            const bool preturn = (rs1 == 1U) && (rd != rs1);
            if (pcall)
                push(fall);
            if (preturn)
                return pop(fall);
            return jr_target(cur);                 // every other jalr-family
        }
        return fall;                               // not a control transfer
    }

    //-- 16-bit (RVC) instructions -------------------------------------------
    uint64_t next_pc_rvc(uint64_t cur, uint16_t h)
    {
        const uint64_t fall = cur + 2;
        const uint32_t q    = h & 0x3U;
        const uint32_t f3   = (h >> 13) & 0x7U;

        if (q == 0x1U) {
            if (f3 == 0x5U)                        // c.j
                return (uint64_t)((int64_t)cur + imm_cj(h));
            if (f3 == 0x6U || f3 == 0x7U)           // c.beqz / c.bnez
                return branch_taken(cur)
                           ? (uint64_t)((int64_t)cur + imm_cb(h))
                           : fall;
            return fall;                            // f3==1 is c.addiw (RV64)
        }
        if (q == 0x2U && f3 == 0x4U) {
            const uint32_t rs1 = (h >> 7) & 0x1FU;
            const uint32_t rs2 = (h >> 2) & 0x1FU;
            const uint32_t b12 = (h >> 12) & 0x1U;
            if (rs2 == 0U && rs1 != 0U) {
                if (b12 == 0U) {                    // c.jr rs1: dst implicit x0
                    if (rs1 == 1U)                  // preturn (dst==x0 != x1,
                        return pop(fall);           // so never excluded)
                    return jr_target(cur);           // plain indirect jump
                }
                // c.jalr rs1: dst implicit x1 -> UNCONDITIONALLY a pcall.
                // Never also a preturn, even when rs1==x1 (rv906's x1-only
                // "dual" case): dst==src excludes it, mirroring
                // aq_iu_bju.v's own bju_src_dst_reg_equal exclusion.
                push(fall);
                return jr_target(cur);
            }
            return fall;                             // c.mv/c.add/c.ebreak
        }
        return fall;
    }
};

//=============================================================================
// ISS selftest (plan Task 5.3) -- the oracle's own gate, run with NO RTL.
//
// Golden fragment: test/m1/iss_selftest.S, assembled with test/m1/Makefile
// and disassembled with objdump (riscv-none-elf-objdump -d -M no-aliases)
// to read off REAL addresses and encodings -- not hand-guessed ones. The
// EXPECTED table below (SELFTEST_GOLDEN) was derived from that real
// disassembly BY HAND, applying the contract rules above one instruction at
// a time; it was never produced by running this ISS and copying its output.
//
// Coverage: RVC/32-bit mix with an unaligned-to-4 straddle (the prologue's
// closing `jal` starts at .+0x0a); all four (encoding x direction)
// conditional-branch combinations -- B-type taken/not-taken, CB-type
// (c.beqz/c.bnez) taken/not-taken, each pinned to a chosen pc[7:4] value and
// each with a taken-target that is DELIBERATELY DIFFERENT from its own
// fall-through address (so a direction-rule bug is visible, not masked by a
// coincidence); a direct jump (c.j and 32-bit jal, rd=x0, no push); a
// call/return nest two deep (x1-only); a return with an EMPTY shadow stack
// (pop-on-empty falls through); a plain indirect jump (c.jr through a
// non-x1 register) landing at its JR_TARGET pad; an INDIRECT CALL through a
// non-x1 register (rd==x1, rs1!=x1 -- pushes AND uses JR_TARGET, a
// combination that cannot occur on rv12/C910's x1/x5 dual-link convention);
// and rv906's x1-only "dual" case `jalr x1, 0(x1)` (rd==rs1==x1), which
// aq_iu_bju.v excludes from preturn, so it must classify as push-only with a
// JR_TARGET destination -- another case with no C910 analogue.
//=============================================================================

struct SelftestWord { uint64_t off; uint32_t val; unsigned bytes; };

// Sparse memory image: every halfword the walk below actually reads, and
// nothing else (a stray read of an unlisted offset returns 0x0000 -- the
// canonical illegal RVC encoding -- so an ISS bug that strays off the
// expected path produces an obviously wrong entry instead of silently
// matching by luck).
static const SelftestWord SELFTEST_IMAGE[] = {
    { 0x000, 0x0001,     2 },   // c.nop
    { 0x002, 0x0285,     2 },   // c.addi t0,1
    { 0x004, 0x4501,     2 },   // c.li a0,0
    { 0x006, 0x6585,     2 },   // c.lui a1,1
    { 0x008, 0x0001,     2 },   // c.nop
    { 0x00a, 0x0f60006f, 4 },   // jal x0,+0xf6      -> 0x100 (straddles +0x4)
    { 0x100, 0xe91d,     2 },   // c.bnez a0,+0x36   -> 0x136 (NOT taken, n=0)
    { 0x102, 0x0001,     2 },   // c.nop (br4's fall-through)
    { 0x104, 0x00000013, 4 },   // addi x0,x0,0 (alignment filler, committed)
    { 0x108, 0x00000013, 4 },   // addi x0,x0,0
    { 0x10c, 0x00000013, 4 },   // addi x0,x0,0
    { 0x110, 0x00000863, 4 },   // beq x0,x0,+0x10   -> 0x120 (TAKEN, n=1)
    { 0x120, 0xc801,     2 },   // c.beqz s0,+0x10   -> 0x130 (TAKEN, n=2)
    { 0x130, 0x00001363, 4 },   // bne x0,x0,+0x06   -> 0x136 (NOT taken, n=3;
                                 //   falls through to 0x134)
    { 0x134, 0xa021,     2 },   // c.j +0x08         -> 0x13c
    { 0x13c, 0x0060006f, 4 },   // jal x0,+0x06      -> 0x142
    { 0x142, 0x0001,     2 },   // c.nop
    { 0x144, 0x00a000ef, 4 },   // jal x1,+0xa       -> 0x14e, push 0x148
    { 0x14e, 0x0001,     2 },   // c.nop (callee1)
    { 0x150, 0x008000ef, 4 },   // jal x1,+0x8       -> 0x158, push 0x154
    { 0x158, 0x0001,     2 },   // c.nop (callee2)
    { 0x15a, 0x8082,     2 },   // c.jr x1           preturn: pop -> 0x154
    { 0x154, 0x8082,     2 },   // c.jr x1           preturn: pop -> 0x148
    { 0x148, 0x0140006f, 4 },   // jal x0,+0x14      -> 0x15c
    { 0x15c, 0x00008067, 4 },   // jalr x0,0(x1)     preturn on EMPTY stack
                                 //   -> fall-through 0x160
    { 0x160, 0x0001,     2 },   // c.nop
    { 0x162, 0x8302,     2 },   // c.jr x6           ind_br: JR_TARGET(0x162)=0x1c0
    { 0x1c0, 0x0001,     2 },   // c.nop (ind_jr_pad)
    { 0x1c2, 0x000100e7, 4 },   // jalr x1,0(x2)     pcall (push 0x1c6) AND
                                 //   ind_br (rs1=x2!=x1): JR_TARGET(0x1c2)=0x2c0
    { 0x2c0, 0x8082,     2 },   // c.jr x1           preturn: pop -> 0x1c6
    { 0x1c6, 0x0001,     2 },   // c.nop
    { 0x1c8, 0x0fa0006f, 4 },   // jal x0,+0xfa      -> 0x2c2
    { 0x2c2, 0x0001,     2 },   // c.nop
    { 0x2c4, 0x000080e7, 4 },   // jalr x1,0(x1)     rv906 "dual" case: pcall
                                 //   (push 0x2c8) but EXCLUDED from preturn
                                 //   (src==dst) -> JR_TARGET(0x2c4)=0x3c0
    { 0x3c0, 0x8082,     2 },   // c.jr x1           preturn: pop -> 0x2c8
    { 0x2c8, 0x0001,     2 },   // c.nop
    { 0x2ca, 0x0f80006f, 4 },   // jal x0,+0xf8      -> 0x3c2
    { 0x3c2, 0x0001,     2 },   // c.nop
    { 0x3c4, 0x0000006f, 4 },   // jal x0,0          SENTINEL
};

// The hand-derived committed stream (offsets from RESET_VECTOR). Column 3 is
// rvc; column 2 is the low 16 bits for an rvc entry, the full 32-bit word
// otherwise (Entry::opcode's own convention, see the header comment).
static const struct { uint64_t off; uint32_t opcode; bool rvc; } SELFTEST_GOLDEN[] = {
    { 0x000, 0x0001,     true  },
    { 0x002, 0x0285,     true  },
    { 0x004, 0x4501,     true  },
    { 0x006, 0x6585,     true  },
    { 0x008, 0x0001,     true  },
    { 0x00a, 0x0f60006f, false },  // jal x0 -> 0x100
    { 0x100, 0xe91d,     true  },  // c.bnez NOT taken -> fall through
    { 0x102, 0x0001,     true  },
    { 0x104, 0x00000013, false },
    { 0x108, 0x00000013, false },
    { 0x10c, 0x00000013, false },
    { 0x110, 0x00000863, false },  // beq TAKEN -> 0x120
    { 0x120, 0xc801,     true  },  // c.beqz TAKEN -> 0x130
    { 0x130, 0x00001363, false },  // bne NOT taken -> fall through to 0x134
    { 0x134, 0xa021,     true  },  // c.j -> 0x13c
    { 0x13c, 0x0060006f, false },  // jal x0 -> 0x142
    { 0x142, 0x0001,     true  },
    { 0x144, 0x00a000ef, false },  // call -> 0x14e, stack [0x148]
    { 0x14e, 0x0001,     true  },
    { 0x150, 0x008000ef, false },  // call -> 0x158, stack [0x148,0x154]
    { 0x158, 0x0001,     true  },
    { 0x15a, 0x8082,     true  },  // ret -> 0x154, stack [0x148]
    { 0x154, 0x8082,     true  },  // ret -> 0x148, stack []
    { 0x148, 0x0140006f, false },  // jal x0 -> 0x15c
    { 0x15c, 0x00008067, false },  // ret on EMPTY stack -> fall through 0x160
    { 0x160, 0x0001,     true  },
    { 0x162, 0x8302,     true  },  // ind_br -> JR_TARGET = 0x1c0
    { 0x1c0, 0x0001,     true  },
    { 0x1c2, 0x000100e7, false },  // ind call (rs1=x2) -> JR_TARGET = 0x2c0,
                                   //   stack [0x1c6]
    { 0x2c0, 0x8082,     true  },  // ret -> 0x1c6, stack []
    { 0x1c6, 0x0001,     true  },
    { 0x1c8, 0x0fa0006f, false },  // jal x0 -> 0x2c2
    { 0x2c2, 0x0001,     true  },
    { 0x2c4, 0x000080e7, false },  // dual case (rs1==rd==x1) -> JR_TARGET =
                                   //   0x3c0, stack [0x2c8] (push, no pop)
    { 0x3c0, 0x8082,     true  },  // ret -> 0x2c8, stack []
    { 0x2c8, 0x0001,     true  },
    { 0x2ca, 0x0f80006f, false },  // jal x0 -> 0x3c2
    { 0x3c2, 0x0001,     true  },
    { 0x3c4, 0x0000006f, false },  // SENTINEL
};

// Returns true on success and prints ISS-SELFTEST-PASS.
static inline bool selftest()
{
    static std::vector<uint16_t> img;
    const size_t img_halfwords = 0x400 / 2;
    img.assign(img_halfwords, 0x0000);   // illegal RVC = "unexpectedly read"
    for (size_t i = 0; i < sizeof(SELFTEST_IMAGE) / sizeof(SELFTEST_IMAGE[0]); i++) {
        const SelftestWord &w = SELFTEST_IMAGE[i];
        img[w.off / 2] = (uint16_t)(w.val & 0xFFFFU);
        if (w.bytes == 4)
            img[w.off / 2 + 1] = (uint16_t)(w.val >> 16);
    }

    Iss iss;
    iss.reset([](uint64_t a) -> uint16_t {
                  uint64_t off = a - RESET_VECTOR;
                  if (off >= 0x400 || (off & 1))
                      return 0x0000;
                  return img[off / 2];
              },
              RESET_VECTOR);

    const size_t n = sizeof(SELFTEST_GOLDEN) / sizeof(SELFTEST_GOLDEN[0]);
    bool ok = true;
    for (size_t i = 0; i < n; i++) {
        Entry e = iss.next();
        const uint64_t want_pc  = RESET_VECTOR + SELFTEST_GOLDEN[i].off;
        const uint32_t want_op  = SELFTEST_GOLDEN[i].opcode;
        const bool     want_rvc = SELFTEST_GOLDEN[i].rvc;
        const bool op_ok = want_rvc ? ((e.opcode & 0xFFFFU) == (want_op & 0xFFFFU))
                                     : (e.opcode == want_op);
        if (e.pc != want_pc || !op_ok || e.rvc != want_rvc) {
            printf("[iss-selftest] entry %u MISMATCH: got pc=%016llx op=%08x "
                   "rvc=%d, want pc=%016llx op=%08x rvc=%d\n",
                   (unsigned)i, (unsigned long long)e.pc, e.opcode, (int)e.rvc,
                   (unsigned long long)want_pc, want_op, (int)want_rvc);
            ok = false;
            break;
        }
    }

    if (ok && !iss.sentinel_seen) {
        printf("[iss-selftest] the walk never reached the sentinel\n");
        ok = false;
    }
    if (ok && iss.prefix_count != n - 1) {
        printf("[iss-selftest] prefix_count=%llu, expected %u\n",
               (unsigned long long)iss.prefix_count, (unsigned)(n - 1));
        ok = false;
    }
    // Past the sentinel the stream is the sentinel repeated (it jumps to
    // itself), which is what the online checker's end-of-run rule relies on.
    if (ok) {
        Entry e = iss.next();
        const uint64_t want_pc = RESET_VECTOR + 0x3c4;
        if (e.pc != want_pc || e.opcode != SENTINEL_OPCODE) {
            printf("[iss-selftest] the sentinel is not a self-loop: "
                   "pc=%016llx op=%08x\n",
                   (unsigned long long)e.pc, e.opcode);
            ok = false;
        }
    }

    // Direct checks on the two contract formulas, independent of the walk
    // (jr_target values cross-checked by hand against the same assembled
    // fragment's JR_PAD placements, and confirmed a second, completely
    // independent way -- reworking the formula's arithmetic from scratch --
    // in this task's own completion notes).
    if (ok) {
        struct { uint64_t pc, tgt; } jr[] = {
            { 0x80000000ULL, 0x80000040ULL },
            { 0x80000162ULL, 0x800001C0ULL },   // ind_jr_site
            { 0x800001C2ULL, 0x800002C0ULL },   // ind_call_site
            { 0x800002C4ULL, 0x800003C0ULL },   // dual_site
            { 0x800000BFULL, 0x80000140ULL },   // unaligned pc, sanity check
        };
        for (size_t i = 0; i < sizeof(jr) / sizeof(jr[0]); i++) {
            if (jr_target(jr[i].pc) != jr[i].tgt) {
                printf("[iss-selftest] JR_TARGET(%016llx) = %016llx, expected %016llx\n",
                       (unsigned long long)jr[i].pc,
                       (unsigned long long)jr_target(jr[i].pc),
                       (unsigned long long)jr[i].tgt);
                ok = false;
            }
        }
        struct { uint64_t pc; bool taken; } dir[] = {
            { 0x80000100ULL, false },  // n=0
            { 0x80000110ULL, true  },  // n=1
            { 0x80000120ULL, true  },  // n=2
            { 0x80000130ULL, false },  // n=3
            { 0x80000040ULL, true  },  // n=4
            { 0x800000F0ULL, false },  // n=15
        };
        for (size_t i = 0; i < sizeof(dir) / sizeof(dir[0]); i++) {
            if (branch_taken(dir[i].pc) != dir[i].taken) {
                printf("[iss-selftest] direction rule at %016llx = %d, expected %d\n",
                       (unsigned long long)dir[i].pc,
                       (int)branch_taken(dir[i].pc), (int)dir[i].taken);
                ok = false;
            }
        }
    }

    if (ok)
        printf("ISS-SELFTEST-PASS (%u committed entries checked)\n", (unsigned)n);
    else
        printf("ISS-SELFTEST-FAIL\n");
    return ok;
}

} // namespace m1

#endif // M1_ISS_H
