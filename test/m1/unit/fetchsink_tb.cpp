//=============================================================================
// fetchsink_tb.cpp - standalone unit bench for rtl/FetchSink.v (M1 plan
// task 4.3)
//=============================================================================
// Verilates FetchSink.v + rvproc_pkg.sv alone (no IFU, no ICache, no SoC)
// and drives the frozen ifu_idu_id_inst/_vld/_bht_pred ports directly with
// hand-built single-instruction delivery sequences, exactly the way
// icache_tb.cpp (plan task 2.2) drove ICache.v's fetch port directly.
// Pattern (raw Verilator C++ main, obj-dir-per-unit Makefile, tick()-based
// clocking, check()/test_result() bookkeeping) copied from that file and,
// stylistically, from rv12's own FetchSink unit bench; the DUT CONTRACT
// itself (x1-only pcall/preturn classification, the single-instruction
// interface, the 2-cycle squash window) is rv906/C906-specific and is NOT
// copied from rv12/C910's numbers -- see rtl/FetchSink.v's header "TASK 4.1
// FINDINGS" for what was actually confirmed from aq_iu_bju.v.
//
// This bench is a WHITE-BOX test of FetchSink's OWN stated contract, run in
// isolation -- it does not prove the 2-cycle squash window is the RIGHT
// length for IFU.v's real timing (that is plan Task 6's integration
// bring-up gate); it proves FetchSink honors the window IT documents.
//
// Build/run: make -C test/m1/unit fetchsink && bin/unit/fetchsink_tb
// Prints one line per test and ends with UNIT-PASS or UNIT-FAIL.
//=============================================================================

#include <verilated.h>
#include "VFetchSink.h"
#include "VFetchSink___024root.h"
#include "VFetchSink_FetchSink.h"

#include <cstdio>
#include <cstdint>
#include <cstring>

// FetchSink.v's `verilator public` internal registers (cmt_*/resolve_*/
// perr_code/cfg_*) live on the nested per-instance class Verilator generates
// even when FetchSink IS the top module (VFetchSink_FetchSink, reached via
// rootp->FetchSink) -- only the module's PORTS flatten onto VFetchSink
// itself. Mirrors verisim.h's FSINK() macro for the same reason.
#define FS(d) ((d)->rootp->FetchSink)

//-----------------------------------------------------------------------------
// Geometry / constants (mirrors rvproc_pkg.sv)
//-----------------------------------------------------------------------------
static const unsigned PC_WIDTH   = 40;
static const uint64_t PC_MASK    = (PC_WIDTH >= 64) ? ~0ULL : ((1ULL << PC_WIDTH) - 1);
static const uint64_t RESET_VECTOR = 0x80000000ULL;   // FetchSink.v's default parameter
static const uint64_t TOHOST_ADDR  = 0x0000000090001000ULL;  // rvproc_pkg.sv ADDR_TOHOST

//-----------------------------------------------------------------------------
// Instruction encodings used by this bench (RV64GC, hand-encoded and
// cross-checked against the standard B/J/CB/CJ immediate layouts -- see the
// derivation in the plan Task 4.3 completion note; every constant below is
// double-checked against FetchSink.v's own fs_decode() formulas by hand,
// not just asserted).
//-----------------------------------------------------------------------------
static const uint32_t NOP             = 0x00000013u;   // addi x0,x0,0
static const uint32_t SENTINEL        = 0x0000006Fu;   // jal x0,0
static const uint32_t JAL_PLAIN_P8    = 0x0080006Fu;   // jal x0,+8      (ab_br, no link)
static const uint32_t JAL_CALL_P8     = 0x008000EFu;   // jal x1,+8      (pcall, imm target)
static const uint32_t BEQ_P16         = 0x00208863u;   // beq x1,x2,+16  (con_br)
static const uint32_t JALR_RET        = 0x00008067u;   // jalr x0,x1,0   (preturn)
static const uint32_t JALR_IND        = 0x00028067u;   // jalr x0,x5,0   (ind_br, rs1!=x1)
static const uint32_t JALR_IND_CALL   = 0x000280E7u;   // jalr x1,x5,0   (pcall + ind target)
static const uint32_t JALR_DUAL       = 0x000080E7u;   // jalr x1,x1,0   (pcall only, NOT preturn)
static const uint16_t C_J_P2_LO       = 0xA009u;        // c.j +2 (ab_br, compressed)

// JR_TARGET(pc), plan "Global contracts" -- independent C++ reimplementation
// (must NOT be copied from FetchSink.v's jr_target() function: this bench is
// checking that function, so it needs its own independent formula, same
// discipline the plan requires between FetchSink and the C++ fetch-ISS).
static uint64_t jr_target(uint64_t pc) {
    uint64_t blocks = ((pc >> 6) & 0x3ULL) + 1ULL;
    return (pc & ~0x3FULL) + (blocks << 6);
}

static bool direction(uint64_t pc) {
    unsigned nib = (unsigned)((pc >> 4) & 0xFu);
    return (__builtin_parity(nib) & 1u) != 0;
}

//-----------------------------------------------------------------------------
// DUT plumbing
//-----------------------------------------------------------------------------
static VFetchSink *dut = nullptr;
static uint64_t g_cycles = 0;

static void tie_idle_inputs(void) {
    dut->ifu_idu_id_inst     = 0;
    dut->ifu_idu_id_inst_vld = 0;
    dut->ifu_idu_id_bht_pred = 0;
    dut->ifu_iu_chgflw_vld   = 0;
    dut->ifu_iu_chgflw_pc    = 0;
    dut->ifu_cp0_icache_inv_done = 0;
    dut->bht_cp0_inv_done        = 0;
    dut->axi_d_awready = 0;
    dut->axi_d_wready  = 0;
    dut->axi_d_bvalid  = 0;
    dut->axi_d_bresp   = 0;
    dut->axi_d_arready = 0;
    dut->axi_d_rvalid  = 0;
    for (int i = 0; i < 16; i++) dut->axi_d_rdata[i] = 0;
    dut->axi_d_rresp = 0;
    dut->axi_d_rlast = 0;
}

//-----------------------------------------------------------------------------
// Behavioural D-side AXI write slave: captures the tohost write (awaddr,
// wdata[63:0]) and completes it with BVALID one cycle later. Modeled on
// icache_tb.cpp's AxiSlave -- this is the consumer side of the SAME write
// FSM lifted from TestMaster.v into FetchSink.v.
//-----------------------------------------------------------------------------
struct AxiWriteSlave {
    bool aw_seen = false, w_seen = false, b_pending = false;
    uint64_t captured_addr = 0;
    uint64_t captured_data = 0;
    int      writes = 0;

    void sample(VFetchSink *d) {
        if (d->axi_d_awvalid && d->axi_d_awready) {
            aw_seen = true;
            captured_addr = d->axi_d_awaddr;
        }
        if (d->axi_d_wvalid && d->axi_d_wready) {
            w_seen = true;
            captured_data = (uint64_t)d->axi_d_wdata[0] |
                             ((uint64_t)d->axi_d_wdata[1] << 32);
        }
        if (aw_seen && w_seen && !b_pending) {
            b_pending = true;
            writes++;
        }
    }
    void drive(VFetchSink *d) {
        d->axi_d_awready = 1;
        d->axi_d_wready  = 1;
        d->axi_d_bvalid  = b_pending ? 1 : 0;
        d->axi_d_bresp   = 0;
        if (b_pending) {
            // Response accepted the same cycle BVALID is asserted (bready is
            // tied 1 by FetchSink.v) -- clear on the NEXT sample.
            aw_seen = false;
            w_seen  = false;
            b_pending = false;
        }
    }
};
static AxiWriteSlave g_axi;

static void tick(void) {
    dut->eval();
    g_axi.sample(dut);
    dut->clk = 1;
    dut->eval();                 // registers commit here
    g_axi.drive(dut);
    dut->eval();
    dut->clk = 0;
    dut->eval();
    g_cycles++;
}

static void reset_dut(void) {
    dut->clk   = 0;
    dut->rst_n = 0;
    tie_idle_inputs();
    for (int i = 0; i < 5; i++) tick();
    dut->rst_n = 1;
    for (int i = 0; i < 5; i++) tick();
}

//-----------------------------------------------------------------------------
// Result bookkeeping
//-----------------------------------------------------------------------------
static int g_fail  = 0;
static int g_local = 0;

static void check(bool cond, const char *what, uint64_t got = 0, uint64_t exp = 0) {
    if (!cond) {
        g_local++;
        if (g_fail < 30)
            printf("    FAIL %-56s got=0x%llx exp=0x%llx (cycle %llu)\n", what,
                   (unsigned long long)got, (unsigned long long)exp,
                   (unsigned long long)g_cycles);
        g_fail++;
    }
}

static void test_result(const char *name) {
    printf("[fetchsink_tb] %-56s %s\n", name, g_local ? "FAIL" : "PASS");
    g_local = 0;
}

//-----------------------------------------------------------------------------
// Delivery primitive: present ONE instruction for exactly one cycle (the
// frozen contract: C906 delivers one instruction/cycle, FetchSink.v
// SECTION RESOLVE) and read back every registered output the cycle it
// updates (posedge-registered, so it is already visible right after tick()
// returns -- see FetchSink.v's own "registered resolve" discipline).
//-----------------------------------------------------------------------------
struct CommitResult {
    bool     cmt_valid = false;
    uint64_t cmt_pc = 0;
    uint32_t cmt_opcode = 0;
    uint32_t cmt_count = 0;
    bool     tar_pc_vld = false;
    uint64_t tar_pc = 0;
    bool     pc_mispred = false, bht_mispred = false, br_vld = false;
    bool     bht_taken = false, link_vld = false, ret_vld = false;
    unsigned bht_pred = 0;
    bool     resolve_event = false;
    unsigned resolve_kind = 0;
    unsigned perr_code = 0;
    bool     idu_stall = false;
};

static CommitResult deliver(uint32_t inst, unsigned bht_pred_in = 0) {
    dut->ifu_idu_id_inst     = inst;
    dut->ifu_idu_id_inst_vld = 1;
    dut->ifu_idu_id_bht_pred = bht_pred_in;
    tick();
    dut->ifu_idu_id_inst_vld = 0;
    dut->ifu_idu_id_inst     = 0;

    CommitResult r;
    r.cmt_valid    = FS(dut)->cmt_valid != 0;
    r.cmt_pc       = FS(dut)->cmt_pc;
    r.cmt_opcode   = FS(dut)->cmt_opcode;
    r.cmt_count    = FS(dut)->cmt_count;
    r.tar_pc_vld   = dut->iu_ifu_tar_pc_vld != 0;
    r.tar_pc       = dut->iu_ifu_tar_pc;
    r.pc_mispred   = dut->iu_ifu_pc_mispred != 0;
    r.bht_mispred  = dut->iu_ifu_bht_mispred != 0;
    r.br_vld       = dut->iu_ifu_br_vld != 0;
    r.bht_taken    = dut->iu_ifu_bht_taken != 0;
    r.link_vld     = dut->iu_ifu_link_vld != 0;
    r.ret_vld      = dut->iu_ifu_ret_vld != 0;
    r.bht_pred     = dut->iu_ifu_bht_pred;
    r.resolve_event= FS(dut)->resolve_event != 0;
    r.resolve_kind = FS(dut)->resolve_kind;
    r.perr_code    = FS(dut)->perr_code;
    r.idu_stall    = dut->idu_ifu_id_stall != 0;
    return r;
}

// Idle for N cycles (no delivery), returning the LAST cycle's readback --
// used to probe the squash window (nothing should commit while it holds).
static CommitResult idle_probe(uint32_t maybe_inst = NOP) {
    dut->ifu_idu_id_inst     = maybe_inst;
    dut->ifu_idu_id_inst_vld = 1;   // offer data even during squash: FetchSink
                                    // must ignore it, IFU-side realism aside
    dut->ifu_idu_id_bht_pred = 0;
    tick();
    dut->ifu_idu_id_inst_vld = 0;
    dut->ifu_idu_id_inst     = 0;
    CommitResult r;
    r.cmt_valid  = FS(dut)->cmt_valid != 0;
    r.tar_pc_vld = dut->iu_ifu_tar_pc_vld != 0;
    r.cmt_count  = FS(dut)->cmt_count;
    return r;
}

// Genuinely idle: inst_vld held LOW (nothing offered at all). Used to check
// that FetchSink never commits on its own with no delivery in flight.
static CommitResult tick_no_deliver(void) {
    dut->ifu_idu_id_inst     = 0;
    dut->ifu_idu_id_inst_vld = 0;
    dut->ifu_idu_id_bht_pred = 0;
    tick();
    CommitResult r;
    r.cmt_valid  = FS(dut)->cmt_valid != 0;
    r.tar_pc_vld = dut->iu_ifu_tar_pc_vld != 0;
    r.cmt_count  = FS(dut)->cmt_count;
    return r;
}

//-----------------------------------------------------------------------------
// Test driver state: mirrors FetchSink's own arch_pc so the test always
// knows what PC the NEXT delivered instruction lands at, without needing a
// PC field on the interface (there isn't one -- FetchSink.v TASK 4.1
// FINDINGS item... see SECTION RESOLVE header).
//-----------------------------------------------------------------------------
static uint64_t sim_pc = RESET_VECTOR;

static void advance_pc(uint64_t next) { sim_pc = next & PC_MASK; }

// Deliver filler NOPs (each len 4, never a control transfer) until sim_pc's
// direction rule matches `want_taken`. Bounded: the nibble pc[7:4] cycles
// through all 16 values every 16 NOPs, so this always terminates quickly.
static void advance_to_direction(bool want_taken) {
    for (int guard = 0; guard < 64 && direction(sim_pc) != want_taken; guard++) {
        CommitResult r = deliver(NOP);
        check(r.cmt_valid, "advance_to_direction: filler NOP commits");
        check(!r.tar_pc_vld, "advance_to_direction: filler NOP never redirects");
        advance_pc(sim_pc + 4);
    }
    check(direction(sim_pc) == want_taken, "advance_to_direction: reached wanted direction");
}

// Drain the documented 2-cycle post-redirect squash window (FetchSink.v
// TASK 4.1 FINDINGS item 3): offer data every cycle, confirm NONE of it
// commits, until the window elapses.
static void settle_squash(void) {
    for (int i = 0; i < 2; i++) {
        CommitResult r = idle_probe(NOP);
        check(!r.cmt_valid, "settle_squash: squash window rejects delivered data");
    }
}

//=============================================================================
// Tests
//=============================================================================

static void test_reset_idle(void) {
    check(FS(dut)->cmt_valid == 0, "quiescent after reset: cmt_valid low");
    check(dut->iu_ifu_tar_pc_vld == 0, "quiescent after reset: tar_pc_vld low");
    check(dut->iu_ifu_br_vld == 0, "quiescent after reset: br_vld low");
    check(dut->idu_ifu_id_stall == 0, "quiescent after reset: stall low (cfg_sink_stall=0)");
    for (int i = 0; i < 8; i++) {
        CommitResult r = tick_no_deliver();
        check(!r.cmt_valid, "quiescent: no spurious commit with inst_vld held low",
              0, 0);
    }
    test_result("T1 reset/idle quiescent state");
}

static void test_correct_prediction_commits(void) {
    uint64_t start_pc = sim_pc;
    for (int i = 1; i <= 5; i++) {
        CommitResult r = deliver(NOP);
        check(r.cmt_valid, "sequential commit: cmt_valid asserted", r.cmt_valid, 1);
        check(r.cmt_pc == sim_pc, "sequential commit: cmt_pc matches tracked PC",
              r.cmt_pc, sim_pc);
        check(r.cmt_opcode == NOP, "sequential commit: cmt_opcode matches delivered inst",
              r.cmt_opcode, NOP);
        check(!r.tar_pc_vld, "sequential commit: no redirect for a plain instruction");
        check(!r.br_vld, "sequential commit: no br_vld for a plain instruction");
        advance_pc(sim_pc + 4);
    }
    check(sim_pc == start_pc + 20, "sequential commit: PC advanced exactly 4*5",
          sim_pc, start_pc + 20);
    test_result("T2 back-to-back correct-prediction commits, no squash");
}

static void test_cond_branch_taken_mispredict_squash(void) {
    advance_to_direction(true);
    uint64_t br_pc = sim_pc;
    // Predict NOT-TAKEN (bht_pred[1]=0); actual direction is taken here, so
    // this must mispredict and redirect to br_pc+16.
    CommitResult r = deliver(BEQ_P16, /*bht_pred=*/0);
    check(r.cmt_valid && r.cmt_pc == br_pc, "cond-br-taken: the branch itself commits",
          r.cmt_pc, br_pc);
    check(r.br_vld, "cond-br-taken: br_vld fires for every resolved conditional");
    check(r.bht_taken, "cond-br-taken: bht_taken reflects the actual direction");
    check(r.bht_mispred, "cond-br-taken: bht_mispred (actual != predicted)");
    check(r.tar_pc_vld, "cond-br-taken: redirect asserted");
    check(r.tar_pc == br_pc + 16, "cond-br-taken: tar_pc == pc + B-immediate",
          r.tar_pc, br_pc + 16);
    check(!r.link_vld && !r.ret_vld, "cond-br-taken: not classified pcall/preturn");

    advance_pc(br_pc + 16);   // FetchSink's arch_pc already snapped here
    settle_squash();

    // Resume: the next delivered instruction must be interpreted at the
    // REDIRECTED pc, proving the squash window didn't lose or shift it.
    CommitResult r2 = deliver(NOP);
    check(r2.cmt_valid && r2.cmt_pc == sim_pc,
          "cond-br-taken: resumes exactly at the redirect target", r2.cmt_pc, sim_pc);
    advance_pc(sim_pc + 4);

    test_result("T3 conditional branch taken -> mispredict -> squash -> resume");
}

static void test_cond_branch_not_taken_correct(void) {
    advance_to_direction(false);
    uint64_t br_pc = sim_pc;
    // Predict NOT-TAKEN (bht_pred[1]=0), matching the actual direction here.
    CommitResult r = deliver(BEQ_P16, /*bht_pred=*/0);
    check(r.cmt_valid && r.cmt_pc == br_pc, "cond-br-not-taken: commits", r.cmt_pc, br_pc);
    check(r.br_vld, "cond-br-not-taken: br_vld still fires (every resolved conditional)");
    check(!r.bht_taken, "cond-br-not-taken: bht_taken false");
    check(!r.bht_mispred, "cond-br-not-taken: correctly predicted, no mispredict");
    check(!r.tar_pc_vld, "cond-br-not-taken: no redirect needed");
    advance_pc(br_pc + 4);   // fall-through, no squash

    // No squash window: the very next cycle must accept a new instruction.
    CommitResult r2 = deliver(NOP);
    check(r2.cmt_valid && r2.cmt_pc == sim_pc,
          "cond-br-not-taken: next instruction commits with NO gap", r2.cmt_pc, sim_pc);
    advance_pc(sim_pc + 4);

    test_result("T4 conditional branch correctly predicted not-taken, no squash");
}

static uint64_t g_call_fallthrough = 0;   // shared with test_preturn_pop_matches_call

static void test_jal_call_pushes_fallthrough(void) {
    uint64_t call_pc = sim_pc;
    CommitResult r = deliver(JAL_CALL_P8);
    check(r.cmt_valid && r.cmt_pc == call_pc, "jal-call: commits at call_pc", r.cmt_pc, call_pc);
    check(r.link_vld, "jal-call: classified pcall (rd==x1)");
    check(!r.ret_vld, "jal-call: not classified preturn");
    check(!r.pc_mispred, "jal-call: pc_mispred is a jalr-only signal, false for jal");
    check(r.tar_pc_vld && r.tar_pc == call_pc + 8,
          "jal-call: redirects to pc + J-immediate (NOT the shadow stack)", r.tar_pc, call_pc + 8);

    g_call_fallthrough = call_pc + 4;   // what SHOULD be sitting on the shadow stack
    advance_pc(call_pc + 8);
    settle_squash();
    test_result("T5 jal call (rd==x1): commits, pushes fall-through, targets the immediate");
}

static void test_preturn_pop_matches_call(void) {
    uint64_t ret_pc = sim_pc;
    CommitResult r = deliver(JALR_RET);
    check(r.cmt_valid && r.cmt_pc == ret_pc, "preturn: commits at ret_pc", r.cmt_pc, ret_pc);
    check(r.ret_vld, "preturn: classified preturn (rs1==x1, rd!=x1)");
    check(!r.link_vld, "preturn: not also classified pcall");
    check(!r.pc_mispred, "preturn: pc_mispred false (src0_reg==x1)");
    check(r.tar_pc_vld && r.tar_pc == g_call_fallthrough,
          "preturn: pops the EARLIER call's fall-through, not JR_TARGET",
          r.tar_pc, g_call_fallthrough);

    advance_pc(g_call_fallthrough);
    settle_squash();
    test_result("T6 preturn pops exactly the fall-through the matching call pushed");
}

static void test_pop_on_empty_returns_fallthrough(void) {
    // The shadow stack is empty here (T6's pop consumed the only entry).
    uint64_t ret_pc = sim_pc;
    CommitResult r = deliver(JALR_RET);
    check(r.ret_vld, "pop-on-empty: still classified preturn");
    check(r.tar_pc_vld && r.tar_pc == ret_pc + 4,
          "pop-on-empty: returns the instruction's OWN fall-through",
          r.tar_pc, ret_pc + 4);
    advance_pc(ret_pc + 4);
    settle_squash();
    test_result("T7 preturn on an empty shadow stack returns the fall-through");
}

static void test_ind_br_jr_target(void) {
    uint64_t jr_pc = sim_pc;
    CommitResult r = deliver(JALR_IND);
    check(r.cmt_valid && r.cmt_pc == jr_pc, "ind_br: commits at jr_pc", r.cmt_pc, jr_pc);
    check(!r.ret_vld, "ind_br: not classified preturn (rs1==x5, not x1)");
    check(!r.link_vld, "ind_br: not classified pcall (rd==x0)");
    check(r.pc_mispred, "ind_br: pc_mispred fires (jalr-family, rs1 != x1)");
    uint64_t exp = jr_target(jr_pc);
    check(r.tar_pc_vld && r.tar_pc == exp,
          "ind_br: redirects to JR_TARGET(pc), not a register value", r.tar_pc, exp);
    advance_pc(exp);
    settle_squash();
    test_result("T8 indirect jump through a non-x1 register uses JR_TARGET(pc)");
}

static void test_dual_case_jalr_x1_x1(void) {
    uint64_t pc = sim_pc;
    CommitResult r = deliver(JALR_DUAL);
    check(r.link_vld, "dual-case jalr x1,x1: classified pcall");
    check(!r.ret_vld, "dual-case jalr x1,x1: EXCLUDED from preturn (aq_iu_bju.v src_dst_reg_equal)");
    check(!r.pc_mispred, "dual-case jalr x1,x1: pc_mispred false (src0_reg==x1)");
    uint64_t exp = jr_target(pc);
    check(r.tar_pc_vld && r.tar_pc == exp,
          "dual-case jalr x1,x1: targets JR_TARGET(pc), not a stack pop", r.tar_pc, exp);
    uint64_t pushed = pc + 4;   // this instruction's own fall-through
    advance_pc(exp);
    settle_squash();

    // Prove the push actually happened: a later return must pop THIS
    // instruction's fall-through, not the empty-stack fallback.
    uint64_t ret_pc = sim_pc;
    CommitResult r2 = deliver(JALR_RET);
    check(r2.ret_vld, "dual-case follow-up: classified preturn");
    check(r2.tar_pc_vld && r2.tar_pc == pushed,
          "dual-case follow-up: pops the dual-case instruction's OWN fall-through",
          r2.tar_pc, pushed);
    advance_pc(pushed);
    settle_squash();
    (void)ret_pc;
    test_result("T9 jalr x1,x1 dual case: push-only call, JR_TARGET, never a preturn");
}

static void test_indirect_call_via_register(void) {
    uint64_t pc = sim_pc;
    CommitResult r = deliver(JALR_IND_CALL);
    check(r.link_vld, "indirect call (jalr x1,x5): classified pcall (rd==x1)");
    check(!r.ret_vld, "indirect call: not classified preturn (rs1==x5)");
    check(r.pc_mispred, "indirect call: pc_mispred fires (rs1 != x1)");
    uint64_t exp = jr_target(pc);
    check(r.tar_pc_vld && r.tar_pc == exp,
          "indirect call: targets JR_TARGET(pc), the documented Task 4.1 substitute",
          r.tar_pc, exp);
    advance_pc(exp);
    settle_squash();
    test_result("T10 indirect call through a register (rd==x1, rs1!=x1)");
}

static void test_compressed_ab_br(void) {
    uint32_t inst = 0xDEAD0000u | C_J_P2_LO;   // garbage upper half, C.J in [15:0]
    uint64_t pc = sim_pc;
    CommitResult r = deliver(inst);
    check(r.cmt_valid && r.cmt_pc == pc, "compressed c.j: commits at pc", r.cmt_pc, pc);
    check(r.cmt_opcode == inst, "compressed c.j: opcode exported verbatim (garbage upper half too)",
          r.cmt_opcode, inst);
    check(r.tar_pc_vld && r.tar_pc == pc + 2,
          "compressed c.j: redirects to pc + CJ-immediate (2-byte length)", r.tar_pc, pc + 2);
    check(!r.link_vld && !r.ret_vld, "compressed c.j: not pcall/preturn");
    advance_pc(pc + 2);
    settle_squash();
    test_result("T11 compressed instruction (is32=0): length + immediate decode");
}

static void test_sentinel_tohost(void) {
    uint64_t pc = sim_pc;
    CommitResult r = deliver(SENTINEL);
    check(r.cmt_valid && r.cmt_pc == pc && r.cmt_opcode == SENTINEL,
          "sentinel: commits like any other instruction", r.cmt_opcode, SENTINEL);
    check(!r.tar_pc_vld, "sentinel: redirect suppressed (sim is ending)");

    bool wrote = false;
    for (int i = 0; i < 200 && !wrote; i++) {
        tick();
        if (g_axi.writes > 0) wrote = true;
    }
    check(wrote, "sentinel: a tohost write eventually happens");
    check(g_axi.captured_addr == TOHOST_ADDR, "sentinel: write lands at TOHOST_ADDR",
          g_axi.captured_addr, TOHOST_ADDR);
    check(g_axi.captured_data == 1, "sentinel: tohost value is 1 (PASS convention)",
          g_axi.captured_data, 1);
    test_result("T12 sentinel (jal x0,0) reports tohost=1");
}

static void test_max_insts_budget(void) {
    // Fresh instance: the budget/termination FSM is terminal once entered.
    VFetchSink *saved = dut;
    dut = new VFetchSink;
    AxiWriteSlave saved_axi = g_axi;
    g_axi = AxiWriteSlave();

    reset_dut();
    FS(dut)->cfg_max_insts = 3;
    FS(dut)->cfg_sink_stall = 0;

    for (int i = 0; i < 3; i++) {
        CommitResult r = deliver(NOP);
        check(r.cmt_valid, "budget: filler instruction commits", r.cmt_valid, 1);
    }
    check(FS(dut)->perr_code == 1, "budget: perr_code = 1 (--max-insts exhausted)",
          FS(dut)->perr_code, 1);

    bool wrote = false;
    for (int i = 0; i < 200 && !wrote; i++) {
        tick();
        if (g_axi.writes > 0) wrote = true;
    }
    check(wrote, "budget: a tohost write eventually happens");
    check(g_axi.captured_addr == TOHOST_ADDR, "budget: write lands at TOHOST_ADDR",
          g_axi.captured_addr, TOHOST_ADDR);
    check(g_axi.captured_data == 3, "budget: tohost value is 3 (protocol/budget convention)",
          g_axi.captured_data, 3);

    test_result("T13 --max-insts budget exhaustion reports tohost=3");

    dut->final();
    delete dut;
    dut = saved;
    g_axi = saved_axi;
}

static void test_sink_stall_mode(void) {
    VFetchSink *saved = dut;
    dut = new VFetchSink;
    reset_dut();
    FS(dut)->cfg_max_insts = 1000000;

    FS(dut)->cfg_sink_stall = 0;
    bool saw_stall_off = false;
    for (int i = 0; i < 64; i++) {
        idle_probe(0);
        if (dut->idu_ifu_id_stall) saw_stall_off = true;
    }
    check(!saw_stall_off, "sink-stall off: idu_ifu_id_stall never asserted");

    FS(dut)->cfg_sink_stall = 1;
    bool saw_stall_on = false;
    for (int i = 0; i < 64; i++) {
        idle_probe(0);
        if (dut->idu_ifu_id_stall) saw_stall_on = true;
    }
    check(saw_stall_on, "sink-stall on: idu_ifu_id_stall pulses over 64 cycles");

    test_result("T14 --sink-stall pseudo-random stall mode (config bank readback)");

    dut->final();
    delete dut;
    dut = saved;
}

//=============================================================================
// main
//=============================================================================
int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    dut = new VFetchSink;

    reset_dut();
    FS(dut)->cfg_max_insts  = 1000000;
    FS(dut)->cfg_sink_stall = 0;

    test_reset_idle();
    test_correct_prediction_commits();
    test_cond_branch_taken_mispredict_squash();
    test_cond_branch_not_taken_correct();
    test_jal_call_pushes_fallthrough();
    test_preturn_pop_matches_call();
    test_pop_on_empty_returns_fallthrough();
    test_ind_br_jr_target();
    test_dual_case_jalr_x1_x1();
    test_indirect_call_via_register();
    test_compressed_ab_br();
    test_sentinel_tohost();          // terminal for this dut instance

    test_max_insts_budget();         // own fresh instance
    test_sink_stall_mode();          // own fresh instance

    printf("[fetchsink_tb] %llu cycles, %d failure(s)\n",
           (unsigned long long)g_cycles, g_fail);
    printf("%s\n", g_fail ? "UNIT-FAIL" : "UNIT-PASS");

    dut->final();
    delete dut;
    return g_fail ? 1 : 0;
}
