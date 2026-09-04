//=============================================================================
// rtu_tb.cpp - standalone unit bench for rtl/RTU.v (M2 plan task 4.3)
//=============================================================================
// Verilates RTU.v + rvproc_pkg.sv alone (no IDU, no IU, no LSU, no CSR) and
// drives the frozen iu_rtu_*/lsu_rtu_*/cp0_rtu_* ports directly with
// hand-scripted, per-cycle stimulus standing in for those five real
// producers, the same tick()-based clocking and check()/test_result()
// bookkeeping pattern as test/m2/unit/csr_tb.cpp (Task 2) and iu_tb.cpp
// (Task 3).
//
// This is a WHITE-BOX test of RTU.v's OWN documented contract (its
// header's port-list amendments, LSU-writeback-path resolution, and known
// scope gaps) run in isolation. It does not exercise a real IDU/IU/LSU/CSR
// (none of Tasks 5/6's real bodies exist yet, and IU.v/CSR.v are tested
// against THEIR OWN contracts in iu_tb.cpp/csr_tb.cpp, not re-verified
// here) -- it proves RTU.v honors the interface + timing contract ITS OWN
// header documents, driven by a script that stands in for every producer.
//
// KNOWN LIMITATION (flagged, not silently worked around): the exception-
// priority chain's legs 1 (pending-breakpoint), 2 (interrupt), and 4
// (ebreak/debug breakpoint) have NO corresponding input port anywhere on
// RTU.v's frozen list (no DTU, no cp0_rtu_int_vld -- see RTU.v's header).
// This bench can therefore only exercise the priority order between leg 3
// (LSU async bus error, a real port) and leg 5 (the synchronous CP0/LSU
// exception, real ports) -- legs 1/2/4 are provably, structurally dead
// (there is nothing to poke to make them fire), which is exactly the
// "wired but structurally never fire" property task 4.1 asks for, not a
// bench gap.
//
// Build/run: make -C test/m2/unit rtu && bin/unit/rtu_tb
// Prints one line per test and ends with UNIT-PASS or UNIT-FAIL.
//=============================================================================

#include <verilated.h>
#include "VRTU.h"
#include "VRTU___024root.h"
#include "VRTU_RTU.h"

#include <cstdio>
#include <cstdint>
#include <cstring>

//-----------------------------------------------------------------------------
// DUT plumbing
//-----------------------------------------------------------------------------
static VRTU *dut = nullptr;
static uint64_t g_cycles = 0;

// RTU.v's dbg_onehot_violation/dbg_fwd_collision are internal (non-port)
// `verilator public` wires -- only PORTS flatten directly onto VRTU; an
// internal public signal is reached via rootp->RTU->, matching
// test/m1/unit/fetchsink_tb.cpp's own FS(dut) macro precedent exactly.
#define RTUP(d) ((d)->rootp->RTU)

static void tie_idle_inputs(void) {
    // ALU
    dut->iu_rtu_ex1_alu_cmplt      = 0;
    dut->iu_rtu_ex1_alu_cmplt_dp   = 0;
    dut->iu_rtu_ex1_alu_data       = 0;
    dut->iu_rtu_ex1_alu_inst_len   = 1;
    dut->iu_rtu_ex1_alu_inst_split = 0;
    dut->iu_rtu_ex1_alu_preg       = 0;
    dut->iu_rtu_ex1_alu_wb_dp      = 0;
    dut->iu_rtu_ex1_alu_wb_vld     = 0;
    // BJU
    dut->iu_rtu_ex1_bju_cmplt        = 0;
    dut->iu_rtu_ex1_bju_cmplt_dp     = 0;
    dut->iu_rtu_ex1_bju_data         = 0;
    dut->iu_rtu_ex1_bju_inst_len     = 1;
    dut->iu_rtu_ex1_bju_preg         = 0;
    dut->iu_rtu_ex1_bju_wb_dp        = 0;
    dut->iu_rtu_ex1_bju_wb_vld       = 0;
    dut->iu_rtu_ex1_branch_inst      = 0;
    dut->iu_rtu_ex1_cur_pc           = 0x1000;
    dut->iu_rtu_ex1_next_pc          = 0x1004;
    dut->iu_rtu_ex2_bju_ras_mispred  = 0;
    dut->iu_rtu_depd_lsu_chgflow_vld = 0;
    dut->iu_rtu_depd_lsu_chgflow_next_pc = 0;
    // MUL
    dut->iu_rtu_ex1_mul_cmplt    = 0;
    dut->iu_rtu_ex1_mul_cmplt_dp = 0;
    dut->iu_rtu_ex3_mul_data     = 0;
    dut->iu_rtu_ex3_mul_preg     = 0;
    dut->iu_rtu_ex3_mul_wb_vld   = 0;
    // DIV
    dut->iu_rtu_ex1_div_cmplt    = 0;
    dut->iu_rtu_ex1_div_cmplt_dp = 0;
    dut->iu_rtu_div_data         = 0;
    dut->iu_rtu_div_preg         = 0;
    dut->iu_rtu_div_wb_dp        = 0;
    dut->iu_rtu_div_wb_vld       = 0;
    // LSU
    dut->lsu_rtu_ex1_cmplt        = 0;
    dut->lsu_rtu_ex1_cmplt_dp     = 0;
    dut->lsu_rtu_wb_data          = 0;
    dut->lsu_rtu_wb_preg          = 0;
    dut->lsu_rtu_wb_vld           = 0;
    dut->lsu_rtu_ex2_data         = 0;
    dut->lsu_rtu_ex2_data_vld     = 0;
    dut->lsu_rtu_ex2_dest_reg     = 0;
    dut->lsu_rtu_expt_vld         = 0;
    dut->lsu_rtu_expt_vec         = 0;
    dut->lsu_rtu_tval             = 0;
    dut->lsu_rtu_async_expt_vld   = 0;
    dut->lsu_rtu_async_ld_inst    = 0;
    // CP0
    dut->cp0_rtu_ex1_cmplt_dp  = 0;
    dut->cp0_rtu_ex1_wb_data   = 0;
    dut->cp0_rtu_ex1_wb_preg   = 0;
    dut->cp0_rtu_ex1_wb_vld    = 0;
    dut->cp0_rtu_ex1_expt_vld  = 0;
    dut->cp0_rtu_ex1_expt_int  = 0;
    dut->cp0_rtu_ex1_expt_vec  = 0;
    dut->cp0_rtu_ex1_chgflw    = 0;
    dut->cp0_rtu_ex1_chgflw_pc = 0;
    dut->cp0_rtu_trap_pc       = 0x80000010ULL;   // mtvec-equivalent target, arbitrary nonzero
}

static void tick(void) {
    dut->eval();
    dut->clk = 1;
    dut->eval();   // registers commit here
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
// Result bookkeeping (mirrors csr_tb.cpp/iu_tb.cpp exactly)
//-----------------------------------------------------------------------------
static int g_fail  = 0;
static int g_local = 0;

static void check(bool cond, const char *what, uint64_t got = 0, uint64_t exp = 0) {
    if (!cond) {
        g_local++;
        if (g_fail < 40)
            printf("    FAIL %-56s got=0x%llx exp=0x%llx (cycle %llu)\n", what,
                   (unsigned long long)got, (unsigned long long)exp,
                   (unsigned long long)g_cycles);
        g_fail++;
    }
}

static void test_result(const char *name) {
    printf("[rtu_tb] %-56s %s\n", name, g_local ? "FAIL" : "PASS");
    g_local = 0;
}

//=============================================================================
// Tests
//=============================================================================

static void test_reset_state(void) {
    check(dut->rtu_idu_wb0_vld == 0, "reset: wb0 quiescent");
    check(dut->rtu_idu_wb1_vld == 0, "reset: wb1 quiescent");
    check(dut->rtu_idu_fwd0_vld == 0, "reset: fwd0 quiescent");
    check(dut->rtu_idu_fwd1_vld == 0, "reset: fwd1 quiescent");
    check(dut->rtu_idu_fwd2_vld == 0, "reset: fwd2 quiescent");
    check(dut->rtu_yy_xx_expt_vld == 0, "reset: expt_vld quiescent");
    check(dut->rtu_ifu_chgflw_vld == 0, "reset: chgflw_vld quiescent");
    check(dut->rtu_ifu_flush_fe == 0, "reset: flush_fe quiescent");
    check(dut->rtu_idu_pipeline_empty == 1, "reset: pipeline reports empty");
    check(dut->rtu_idu_commit == 1, "reset: commit asserted (nothing flushing)");
    check(dut->rtu_idu_commit_for_bju == 1, "reset: commit_for_bju asserted");
    check(dut->rtu_iu_mul_wb_grant == 1, "reset: mul_wb_grant asserted (EX1 group idle)");
    check(dut->rtu_iu_div_wb_grant == 1, "reset: div_wb_grant asserted (EX1 group idle)");
    check(RTUP(dut)->dbg_onehot_violation == 0, "reset: onehot assertion quiescent");
    check(RTUP(dut)->dbg_fwd_collision == 0, "reset: fwd-collision assertion quiescent");
    test_result("T1 reset state: everything quiescent");
}

//-----------------------------------------------------------------------------
// T2: ALU create/complete/commit -- EX1 group member, fwd0 same cycle,
// wb0 one cycle later (task 4.3's "scripted create/complete/commit" for
// the ALU fake producer port).
//-----------------------------------------------------------------------------
static void test_alu_create_complete_commit(void) {
    tie_idle_inputs();
    dut->iu_rtu_ex1_alu_cmplt    = 1;
    dut->iu_rtu_ex1_alu_cmplt_dp = 1;
    dut->iu_rtu_ex1_alu_wb_dp    = 1;
    dut->iu_rtu_ex1_alu_wb_vld   = 1;
    dut->iu_rtu_ex1_alu_data     = 0x1234;
    dut->iu_rtu_ex1_alu_preg     = 5;
    dut->eval();
    check(dut->rtu_idu_fwd0_vld && dut->rtu_idu_fwd0_data == 0x1234 && dut->rtu_idu_fwd0_reg == 5,
          "ALU: fwd0 combinational the SAME cycle as dispatch", dut->rtu_idu_fwd0_data, 0x1234);
    check(!RTUP(dut)->dbg_onehot_violation, "ALU: single source, no onehot violation");
    tick();
    check(dut->rtu_idu_wb0_vld && dut->rtu_idu_wb0_data == 0x1234 && dut->rtu_idu_wb0_reg == 5,
          "ALU: wb0 fires exactly ONE cycle later (retire register)", dut->rtu_idu_wb0_data, 0x1234);
    tie_idle_inputs();
    tick();
    check(!dut->rtu_idu_wb0_vld, "ALU: wb0 clears once idle (no queue, no skid)");
    test_result("T2 ALU create/complete/commit");
}

//-----------------------------------------------------------------------------
// T3: BJU create/complete/commit -- a JAL-style op (writes link reg) and a
// conditional branch (never writes a reg, but still retires / sets
// inst_branch bookkeeping).
//-----------------------------------------------------------------------------
static void test_bju_create_complete_commit(void) {
    tie_idle_inputs();
    dut->iu_rtu_ex1_bju_cmplt    = 1;
    dut->iu_rtu_ex1_bju_cmplt_dp = 1;
    dut->iu_rtu_ex1_bju_wb_dp    = 1;
    dut->iu_rtu_ex1_bju_wb_vld   = 1;
    dut->iu_rtu_ex1_bju_data     = 0x2008;   // link address
    dut->iu_rtu_ex1_bju_preg     = 1;         // x1 (ra)
    dut->iu_rtu_ex1_branch_inst  = 1;
    dut->eval();
    check(dut->rtu_idu_fwd0_vld && dut->rtu_idu_fwd0_data == 0x2008 && dut->rtu_idu_fwd0_reg == 1,
          "BJU (JAL): fwd0 combinational, link address");
    tick();
    check(dut->rtu_idu_wb0_vld && dut->rtu_idu_wb0_data == 0x2008 && dut->rtu_idu_wb0_reg == 1,
          "BJU (JAL): wb0 one cycle later");
    tie_idle_inputs();
    tick();

    // Conditional branch: cmplt fires, wb_dp/wb_vld never do.
    dut->iu_rtu_ex1_bju_cmplt    = 1;
    dut->iu_rtu_ex1_bju_cmplt_dp = 1;
    dut->iu_rtu_ex1_branch_inst  = 1;
    dut->eval();
    check(!dut->rtu_idu_fwd0_vld, "BJU (cond branch): no fwd0 -- conditional branches never write a reg");
    tick();
    check(!dut->rtu_idu_wb0_vld, "BJU (cond branch): no wb0 either, but it still retired (no exception/flush)");
    check(dut->rtu_yy_xx_expt_vld == 0, "BJU (cond branch): plain retire, no trap");
    test_result("T3 BJU create/complete/commit (JAL writes, cond-branch doesn't)");
}

//-----------------------------------------------------------------------------
// T4: MULT create/complete/commit -- the documented EX1-early-accept
// (cmplt) vs EX3-late-writeback (ex3_mul_wb_vld) split (IU.v's own header,
// confirmed against the real donor aq_iu_mul.v's shape). cmplt retires
// the instruction's PC bookkeeping several cycles before its GPR value
// actually lands on wb0/fwd1 -- exactly the RTU.v header's documented,
// carried-forward characteristic, not something this bench "fixes".
//-----------------------------------------------------------------------------
static void test_mult_create_complete_commit(void) {
    tie_idle_inputs();
    dut->iu_rtu_ex1_mul_cmplt    = 1;   // EX1 early-accept ("create")
    dut->iu_rtu_ex1_mul_cmplt_dp = 1;
    dut->eval();
    check(!RTUP(dut)->dbg_onehot_violation, "MUL: single source, no onehot violation");
    tick();   // retire-packet latches; MUL's value is NOT ready yet
    check(!dut->rtu_idu_wb0_vld, "MUL: wb0 NOT valid yet (EX3 hasn't produced data)");

    tie_idle_inputs();   // "well-behaved dispatcher": cmplt was a one-shot accept
    tick();
    check(!dut->rtu_idu_wb0_vld, "MUL: still not valid one cycle later (still iterating)");

    // Now EX3 produces the actual result ("complete").
    dut->iu_rtu_ex3_mul_wb_vld = 1;
    dut->iu_rtu_ex3_mul_data   = 0xABCD;
    dut->iu_rtu_ex3_mul_preg   = 7;
    dut->eval();
    check(dut->rtu_idu_fwd1_vld && dut->rtu_idu_fwd1_data == 0xABCD && dut->rtu_idu_fwd1_reg == 7,
          "MUL: fwd1 combinational the cycle EX3 resolves");
    check(dut->rtu_iu_mul_wb_grant == 1, "MUL: wb grant asserted (nothing else contending for rbus)");
    tick();   // "commit"
    check(dut->rtu_idu_wb0_vld && dut->rtu_idu_wb0_data == 0xABCD && dut->rtu_idu_wb0_reg == 7,
          "MUL: wb0 fires one cycle after EX3's wb_vld (not one cycle after cmplt)");
    tie_idle_inputs();
    tick();
    check(!dut->rtu_idu_wb0_vld, "MUL: wb0 clears once idle");
    test_result("T4 MULT create(EX1)/complete(EX3)/commit(wb0), decoupled timing");
}

//-----------------------------------------------------------------------------
// T5: DIV create/complete/commit -- IU.v's own documented, donor-confirmed
// (aq_iu_div.v:755) LEVEL echo of dispatch-select: cmplt/cmplt_dp is held
// across the entire multi-cycle busy span, not a single pulse. This test
// deliberately holds cmplt for MULTIPLE cycles (mirroring iu_tb.cpp's own
// div_op() driving pattern) and confirms the one-hot bus stays clean
// throughout (only DIV's own bit is ever set) while wb0 stays 0 until the
// real result (div_wb_dp/vld) appears.
//-----------------------------------------------------------------------------
static void test_div_create_complete_commit(void) {
    tie_idle_inputs();
    dut->iu_rtu_ex1_div_cmplt    = 1;
    dut->iu_rtu_ex1_div_cmplt_dp = 1;
    dut->eval();
    check(!RTUP(dut)->dbg_onehot_violation, "DIV: single source (held level), no onehot violation");
    tick();
    check(!dut->rtu_idu_wb0_vld, "DIV: wb0 not valid mid-iteration (cycle 1)");
    dut->eval();
    check(!RTUP(dut)->dbg_onehot_violation, "DIV: still no onehot violation while cmplt held high");
    tick();
    check(!dut->rtu_idu_wb0_vld, "DIV: wb0 not valid mid-iteration (cycle 2)");

    // Now the real result appears.
    dut->iu_rtu_div_wb_dp  = 1;
    dut->iu_rtu_div_wb_vld = 1;
    dut->iu_rtu_div_data   = 0x77;
    dut->iu_rtu_div_preg   = 9;
    dut->eval();
    check(dut->rtu_iu_div_wb_grant == 1, "DIV: wb grant asserted");
    tick();
    check(dut->rtu_idu_wb0_vld && dut->rtu_idu_wb0_data == 0x77 && dut->rtu_idu_wb0_reg == 9,
          "DIV: wb0 fires one cycle after div_wb_dp/vld assert");
    tie_idle_inputs();
    tick();
    check(!dut->rtu_idu_wb0_vld, "DIV: wb0 clears once idle");
    test_result("T5 DIV create/complete/commit, held-level cmplt stays one-hot clean");
}

//-----------------------------------------------------------------------------
// T6: CP0 create/complete/commit -- a plain CSRRW-style op (wb_vld, no
// exception), and confirms the donor's own fwd0 asymmetry (CP0's old-CSR
// write is NOT forwarded via fwd0, only ALU/BJU/LSU-ex1 are) is carried
// forward unchanged.
//-----------------------------------------------------------------------------
static void test_cp0_create_complete_commit(void) {
    tie_idle_inputs();
    dut->cp0_rtu_ex1_cmplt_dp = 1;
    dut->cp0_rtu_ex1_wb_vld   = 1;
    dut->cp0_rtu_ex1_wb_data  = 0x55;
    dut->cp0_rtu_ex1_wb_preg  = 3;
    dut->eval();
    check(!dut->rtu_idu_fwd0_vld,
          "CP0: NOT forwarded via fwd0 -- donor's own asymmetry (aq_rtu_rbus.v's fwd0 mux never "
          "handles CP0's leg), carried forward unchanged");
    check(!RTUP(dut)->dbg_onehot_violation, "CP0: single source, no onehot violation");
    tick();
    check(dut->rtu_idu_wb0_vld && dut->rtu_idu_wb0_data == 0x55 && dut->rtu_idu_wb0_reg == 3,
          "CP0: wb0 DOES fire (architectural write happens; only the forward is skipped)");
    tie_idle_inputs();
    tick();
    check(!dut->rtu_idu_wb0_vld, "CP0: wb0 clears once idle");
    test_result("T6 CP0 create/complete/commit + fwd0 asymmetry carried forward");
}

//-----------------------------------------------------------------------------
// T7: CP0 ecall exception -- epc == cur_pc (synchronous exception), tval
// stays 0 (vec 11 is not in the mtval allowlist).
//-----------------------------------------------------------------------------
static void test_cp0_ecall_exception(void) {
    tie_idle_inputs();
    dut->cp0_rtu_ex1_cmplt_dp = 1;
    dut->cp0_rtu_ex1_expt_vld = 1;
    dut->cp0_rtu_ex1_expt_vec = 11;   // ecall
    dut->iu_rtu_ex1_cur_pc  = 0x3000;
    dut->iu_rtu_ex1_next_pc = 0x3004;
    tick();
    check(dut->rtu_yy_xx_expt_vld == 1, "ecall: expt_vld fires");
    check(dut->rtu_yy_xx_expt_vec == 11, "ecall: vec == 11", dut->rtu_yy_xx_expt_vec, 11);
    check(dut->rtu_yy_xx_expt_int == 0, "ecall: not an interrupt");
    check(dut->rtu_cp0_epc == 0x3000, "ecall: epc == cur_pc (synchronous exception)", dut->rtu_cp0_epc, 0x3000);
    check(dut->rtu_cp0_tval == 0, "ecall: tval == 0 (vec 11 not in {1,2,4,5,6,7,12,13,15} allowlist)");
    test_result("T7 CP0 ecall: epc=cur_pc, tval=0 (not allowlisted)");
}

//-----------------------------------------------------------------------------
// T8: mtval allowlist quirk carried forward unchanged (task 4.1: {1,2,4,5,
// 6,7,12,13,15}). vec=2 (illegal instruction, via LSU's tval port standing
// in for "some producer's tval") IS allowlisted -> tval populated; vec=3
// (ebreak-shaped) is NOT allowlisted -> tval stays 0.
//-----------------------------------------------------------------------------
static void test_mtval_allowlist(void) {
    tie_idle_inputs();
    dut->lsu_rtu_ex1_cmplt_dp = 1;
    dut->lsu_rtu_expt_vld     = 1;
    dut->lsu_rtu_expt_vec     = 2;      // illegal instruction -- IS allowlisted
    dut->lsu_rtu_tval         = 0xDEAD;
    tick();
    check(dut->rtu_yy_xx_expt_vec == 2, "allowlist: vec==2 propagated");
    check(dut->rtu_cp0_tval == 0xDEAD, "allowlist: vec 2 IS allowlisted -- tval populated",
          dut->rtu_cp0_tval, 0xDEAD);

    tie_idle_inputs();
    dut->lsu_rtu_ex1_cmplt_dp = 1;
    dut->lsu_rtu_expt_vld     = 1;
    dut->lsu_rtu_expt_vec     = 3;      // NOT in the allowlist
    dut->lsu_rtu_tval         = 0xBEEF;
    tick();
    check(dut->rtu_cp0_tval == 0, "allowlist: vec==3 is NOT allowlisted -- tval reads 0 despite nonzero input",
          dut->rtu_cp0_tval, 0);
    test_result("T8 mtval allowlist {1,2,4,5,6,7,12,13,15} carried forward unchanged");
}

//-----------------------------------------------------------------------------
// T8b (M4 Task 6): CP0 fetch-fault exception (vec 12 pgflt / vec 1 accflt) --
// tval must equal epc (the faulting fetch PC), since CSR.v has no dedicated
// tval port for CP0-sourced faults; RTU.v reuses iu_rtu_ex1_cur_pc directly.
// A non-fetch-fault CP0 exception (e.g. ecall, vec 11, T7 above) must NOT
// pick up cur_pc as tval -- it stays 0 (not allowlisted).
//-----------------------------------------------------------------------------
static void test_cp0_fetch_fault_tval(void) {
    tie_idle_inputs();
    dut->cp0_rtu_ex1_cmplt_dp = 1;
    dut->cp0_rtu_ex1_expt_vld = 1;
    dut->cp0_rtu_ex1_expt_vec = 12;   // fetch page fault
    dut->iu_rtu_ex1_cur_pc  = 0x80001004;
    dut->iu_rtu_ex1_next_pc = 0x80001008;
    tick();
    check(dut->rtu_yy_xx_expt_vec == 12, "fetch pgflt: vec == 12 propagated");
    check(dut->rtu_cp0_epc == 0x80001004, "fetch pgflt: epc == faulting fetch PC",
          dut->rtu_cp0_epc, 0x80001004);
    check(dut->rtu_cp0_tval == 0x80001004, "fetch pgflt: tval == epc (no dedicated CP0 tval port)",
          dut->rtu_cp0_tval, 0x80001004);

    tie_idle_inputs();
    dut->cp0_rtu_ex1_cmplt_dp = 1;
    dut->cp0_rtu_ex1_expt_vld = 1;
    dut->cp0_rtu_ex1_expt_vec = 1;    // fetch access fault
    dut->iu_rtu_ex1_cur_pc  = 0x80002008;
    dut->iu_rtu_ex1_next_pc = 0x8000200c;
    tick();
    check(dut->rtu_yy_xx_expt_vec == 1, "fetch accflt: vec == 1 propagated");
    check(dut->rtu_cp0_tval == 0x80002008, "fetch accflt: tval == epc",
          dut->rtu_cp0_tval, 0x80002008);
    test_result("T8b CP0 fetch-fault (vec 12/1): tval == epc, reusing iu_rtu_ex1_cur_pc");
}

//-----------------------------------------------------------------------------
// T9: LSU wb1 -- a PURE combinational passthrough (no register at all),
// unlike every other write port. fwd2 uses the SEPARATE ex2_data_vld/
// dest_reg/data bus.
//-----------------------------------------------------------------------------
static void test_lsu_wb1_and_fwd2(void) {
    tie_idle_inputs();
    dut->lsu_rtu_wb_vld  = 1;
    dut->lsu_rtu_wb_data = 0x9999;
    dut->lsu_rtu_wb_preg = 4;
    dut->eval();
    check(dut->rtu_idu_wb1_vld && dut->rtu_idu_wb1_data == 0x9999 && dut->rtu_idu_wb1_reg == 4,
          "LSU wb1: combinational passthrough, SAME cycle -- no register stage");
    tie_idle_inputs();
    dut->eval();
    check(!dut->rtu_idu_wb1_vld, "LSU wb1: clears immediately (combinational), no lingering registered value");

    dut->lsu_rtu_ex2_data_vld = 1;
    dut->lsu_rtu_ex2_data     = 0x1111;
    dut->lsu_rtu_ex2_dest_reg = 8;
    dut->eval();
    check(dut->rtu_idu_fwd2_vld && dut->rtu_idu_fwd2_data == 0x1111 && dut->rtu_idu_fwd2_reg == 8,
          "LSU fwd2: separate bus from wb1, also combinational");
    test_result("T9 LSU wb1 (pure combinational passthrough) + fwd2 (separate bus)");
}

//-----------------------------------------------------------------------------
// T10: retire-blocking -- nothing completes, EX2 holds (0/cycle on any
// stall, matching RTU note S2's "at most 1/cycle, 0/cycle on any stall,
// no queue").
//-----------------------------------------------------------------------------
static void test_retire_blocking(void) {
    tie_idle_inputs();
    for (int i = 0; i < 5; i++) {
        tick();
        check(!dut->rtu_idu_wb0_vld, "retire-blocking: wb0 never spuriously fires while idle");
        check(!dut->rtu_idu_wb1_vld, "retire-blocking: wb1 never spuriously fires while idle");
        check(!dut->rtu_yy_xx_expt_vld, "retire-blocking: no spurious trap while idle");
        check(!dut->rtu_ifu_chgflw_vld, "retire-blocking: no spurious redirect while idle");
        check(dut->rtu_idu_pipeline_empty == 1, "retire-blocking: pipeline stays reported-empty");
        check(dut->rtu_idu_commit == 1, "retire-blocking: commit stays asserted");
    }
    test_result("T10 retire-blocking: nothing completes -> EX2 holds quiescent, no queue");
}

//-----------------------------------------------------------------------------
// T11: flush path -- the delayed/LSU-dependent BJU mispredict leg
// (`retire_bju_flush_req`), which forces the flush FSM WITHOUT going
// through retire at all (RTU note S6) -- and confirms RTU asserts NO
// redirect target for this leg (IU.v already redirected IFU directly via
// `iu_ifu_tar_pc_vld`/`_pc`, per this file's header/RTU.v's header).
//-----------------------------------------------------------------------------
static void test_flush_bju_depd_lsu(void) {
    tie_idle_inputs();
    dut->iu_rtu_depd_lsu_chgflow_vld     = 1;
    dut->iu_rtu_depd_lsu_chgflow_next_pc = 0x5000;
    dut->eval();
    check(!dut->rtu_yy_xx_expt_vld, "depd-lsu bju flush: not a trap");
    tick();   // IDLE -> FE, purely from retire_bju_flush_req, no retire_vld needed
    check(dut->rtu_idu_flush_fe == 1, "depd-lsu bju flush: FE reached one cycle after trigger");
    check(dut->rtu_ifu_flush_fe == 1, "depd-lsu bju flush: IFU FE-kill pulse asserted too");
    check(dut->rtu_ifu_chgflw_vld == 0,
          "depd-lsu bju flush: RTU asserts NO redirect target -- IU already redirected IFU directly");
    tie_idle_inputs();
    tick();   // FE -> BE (cpu_no_op: nothing retiring, wb idle)
    check(dut->rtu_idu_flush_wbt == 1, "depd-lsu bju flush: BE reached");
    tick();   // BE -> IDLE
    check(dut->rtu_idu_flush_fe == 0 && dut->rtu_idu_flush_wbt == 0,
          "depd-lsu bju flush: FSM returns to IDLE");
    test_result("T11 flush path: LSU-dependent BJU mispredict bypasses retire, no RTU redirect target");
}

//-----------------------------------------------------------------------------
// T12: flush path -- mret (CSR-serializing changeflow). Redirect target
// mux uses `ex2_next_pc` (== mepc, the return address CSR.v computed) --
// NOT the trap PC -- and fires IMMEDIATELY at retire (no FE wait, since
// this is not a trap). `rtu_lsu_expt_exit` fires later, gated on the
// flush FSM reaching BE.
//-----------------------------------------------------------------------------
static void test_flush_mret(void) {
    tie_idle_inputs();
    dut->cp0_rtu_ex1_cmplt_dp  = 1;
    dut->cp0_rtu_ex1_chgflw    = 1;
    dut->cp0_rtu_ex1_chgflw_pc = 0x9000;   // mepc
    dut->iu_rtu_ex1_cur_pc  = 0x4000;
    dut->iu_rtu_ex1_next_pc = 0x4004;      // irrelevant here -- chgflw_pc wins the next_pc mux
    tick();   // ex2 latches: ex2_inst_chgflw=1, ex2_next_pc=0x9000
    tie_idle_inputs();

    check(dut->rtu_ifu_chgflw_vld == 1, "mret: chgflw_vld fires IMMEDIATELY at retire (not trap-gated)");
    check(dut->rtu_ifu_chgflw_pc == 0x9000,
          "mret: redirect target == ex2_next_pc (mepc), not cp0_rtu_trap_pc", dut->rtu_ifu_chgflw_pc, 0x9000);
    check(dut->rtu_yy_xx_expt_vld == 0, "mret: not a trap");
    check(dut->rtu_lsu_expt_exit == 0, "mret: expt_exit not yet -- flush FSM hasn't reached BE");

    tick();   // IDLE -> FE (retire_inst_flush_fe_set fired last cycle on ex2_inst_chgflw)
    check(dut->rtu_idu_flush_fe == 1, "mret: FE reached");
    check(dut->rtu_lsu_expt_exit == 0, "mret: still not at BE");

    tick();   // FE -> BE
    check(dut->rtu_idu_flush_wbt == 1, "mret: BE reached");
    check(dut->rtu_lsu_expt_exit == 1, "mret: expt_exit fires at BE (retire_xret_vld && flush_be)");

    tick();   // BE -> IDLE
    check(dut->rtu_lsu_expt_exit == 0, "mret: expt_exit clears, FSM back to IDLE");
    test_result("T12 flush path: mret redirect uses ex2_next_pc, expt_exit gated on flush BE");
}

//-----------------------------------------------------------------------------
// T13: flush path -- a taken exception. Redirect target mux uses
// `cp0_rtu_trap_pc` (mtvec) -- NOT ex2_next_pc -- and is DELAYED until the
// flush FSM reaches FE (unlike mret's immediate redirect), because
// `retire_chgflw_vld`'s trap leg is `retire_trap_chgflw_vld && retire_flush_fe`.
// `rtu_lsu_expt_ack` fires later, gated on BE. mepc/mtval readback via
// rtu_cp0_epc/tval is also confirmed here.
//-----------------------------------------------------------------------------
static void test_flush_taken_exception(void) {
    tie_idle_inputs();
    dut->cp0_rtu_ex1_cmplt_dp = 1;
    dut->cp0_rtu_ex1_expt_vld = 1;
    dut->cp0_rtu_ex1_expt_vec = 2;    // illegal instruction
    dut->iu_rtu_ex1_cur_pc  = 0x6000;
    dut->iu_rtu_ex1_next_pc = 0x6004;
    tick();   // ex2 latches; retire_trap_vld fires THIS cycle (combinational off ex2)
    tie_idle_inputs();

    check(dut->rtu_yy_xx_expt_vld == 1, "taken exception: trap declared at retire");
    check(dut->rtu_cp0_epc == 0x6000, "taken exception: epc == cur_pc");
    check(dut->rtu_ifu_chgflw_vld == 0,
          "taken exception: redirect NOT yet asserted -- waits for flush FE, unlike mret");

    tick();   // IDLE -> FE; retire_trap_chgflw_vld latches to 1 on this same edge
    check(dut->rtu_idu_flush_fe == 1, "taken exception: FE reached");
    check(dut->rtu_ifu_chgflw_vld == 1, "taken exception: redirect NOW asserted (trap leg, gated on flush_fe)");
    check(dut->rtu_ifu_chgflw_pc == 0x80000010ULL,
          "taken exception: redirect target == cp0_rtu_trap_pc (mtvec), not ex2_next_pc",
          dut->rtu_ifu_chgflw_pc, 0x80000010ULL);
    check(dut->rtu_lsu_expt_ack == 0, "taken exception: expt_ack not yet -- waits for BE");

    tick();   // FE -> BE
    check(dut->rtu_idu_flush_wbt == 1, "taken exception: BE reached");
    check(dut->rtu_lsu_expt_ack == 1, "taken exception: expt_ack fires at BE");

    tick();   // BE -> IDLE
    check(dut->rtu_lsu_expt_ack == 0, "taken exception: expt_ack clears, FSM back to IDLE");
    test_result("T13 flush path: taken exception redirect uses cp0_rtu_trap_pc, gated on flush FE/BE");
}

//-----------------------------------------------------------------------------
// T14: exception priority matrix -- force the LSU async bus error (leg 3)
// and a CP0 synchronous exception (leg 5) simultaneously; confirm the
// FIXED order (leg 3 wins) exactly as the donor codes it
// (aq_rtu_retire.v:481-501's if/else-if chain).
//-----------------------------------------------------------------------------
static void test_exception_priority_matrix(void) {
    tie_idle_inputs();
    // Latch the leg-5 (sync) candidate into ex2 first.
    dut->cp0_rtu_ex1_cmplt_dp = 1;
    dut->cp0_rtu_ex1_expt_vld = 1;
    dut->cp0_rtu_ex1_expt_vec = 2;    // illegal instruction candidate
    tick();
    // Now, at THIS cycle (ex2_retire_vld already 1 from the tick above),
    // ALSO assert the leg-3 (async, live-wire) candidate.
    dut->cp0_rtu_ex1_cmplt_dp = 0;
    dut->cp0_rtu_ex1_expt_vld = 0;
    dut->lsu_rtu_async_expt_vld = 1;
    dut->lsu_rtu_async_ld_inst  = 1;   // load -> vec 5, else vec 7
    dut->eval();
    check(dut->rtu_yy_xx_expt_vld == 1, "priority matrix: trap still declared with both candidates present");
    check(dut->rtu_yy_xx_expt_vec == 5,
          "priority matrix: LSU async (leg 3) beats CP0 sync (leg 5): vec==5 not 2",
          dut->rtu_yy_xx_expt_vec, 5);

    // Flip to a store-side async error: vec should become 7, still winning over leg 5.
    dut->lsu_rtu_async_ld_inst = 0;
    dut->eval();
    check(dut->rtu_yy_xx_expt_vec == 7,
          "priority matrix: async store-side vec==7, still beats leg 5", dut->rtu_yy_xx_expt_vec, 7);
    test_result("T14 exception priority matrix: leg3(async) > leg5(sync), fixed order confirmed");
}

//-----------------------------------------------------------------------------
// T15: rbus arbitration matrix -- force EX1-group (ALU) + DIV + MUL-EX3 to
// all want the bus the same cycle; confirm EX1-group > DIV > MUL-EX3
// exactly (RTU note S3), by peeling sources off one at a time.
//-----------------------------------------------------------------------------
static void test_rbus_arbitration_matrix(void) {
    // All three contending: EX1-group (ALU) must win.
    tie_idle_inputs();
    dut->iu_rtu_ex1_alu_wb_dp  = 1;
    dut->iu_rtu_ex1_alu_wb_vld = 1;
    dut->iu_rtu_ex1_alu_data   = 0xA1;
    dut->iu_rtu_ex1_alu_preg   = 10;
    dut->iu_rtu_div_wb_dp  = 1;
    dut->iu_rtu_div_wb_vld = 1;
    dut->iu_rtu_div_data   = 0xD1;
    dut->iu_rtu_div_preg   = 11;
    dut->iu_rtu_ex3_mul_wb_vld = 1;
    dut->iu_rtu_ex3_mul_data   = 0xE1;
    dut->iu_rtu_ex3_mul_preg   = 12;
    dut->eval();
    check(dut->rtu_iu_div_wb_grant == 0, "rbus matrix: DIV NOT granted while EX1-group wants the bus");
    check(dut->rtu_iu_mul_wb_grant == 0, "rbus matrix: MUL NOT granted while EX1-group wants the bus");
    tick();
    check(dut->rtu_idu_wb0_vld && dut->rtu_idu_wb0_data == 0xA1 && dut->rtu_idu_wb0_reg == 10,
          "rbus matrix: EX1-group (ALU) wins over DIV and MUL-EX3", dut->rtu_idu_wb0_data, 0xA1);

    // Only DIV + MUL contending (EX1-group absent): DIV must win.
    tie_idle_inputs();
    dut->iu_rtu_div_wb_dp  = 1;
    dut->iu_rtu_div_wb_vld = 1;
    dut->iu_rtu_div_data   = 0xD2;
    dut->iu_rtu_div_preg   = 13;
    dut->iu_rtu_ex3_mul_wb_vld = 1;
    dut->iu_rtu_ex3_mul_data   = 0xE2;
    dut->iu_rtu_ex3_mul_preg   = 14;
    dut->eval();
    check(dut->rtu_iu_div_wb_grant == 1, "rbus matrix: DIV granted (EX1-group absent)");
    check(dut->rtu_iu_mul_wb_grant == 0, "rbus matrix: MUL still NOT granted (DIV outranks it)");
    tick();
    check(dut->rtu_idu_wb0_vld && dut->rtu_idu_wb0_data == 0xD2 && dut->rtu_idu_wb0_reg == 13,
          "rbus matrix: DIV wins over MUL-EX3 when EX1-group is absent", dut->rtu_idu_wb0_data, 0xD2);

    // Only MUL contending: MUL must win.
    tie_idle_inputs();
    dut->iu_rtu_ex3_mul_wb_vld = 1;
    dut->iu_rtu_ex3_mul_data   = 0xE3;
    dut->iu_rtu_ex3_mul_preg   = 15;
    dut->eval();
    check(dut->rtu_iu_mul_wb_grant == 1, "rbus matrix: MUL granted when it's the only contender");
    tick();
    check(dut->rtu_idu_wb0_vld && dut->rtu_idu_wb0_data == 0xE3 && dut->rtu_idu_wb0_reg == 15,
          "rbus matrix: MUL-EX3 wins when alone", dut->rtu_idu_wb0_data, 0xE3);
    test_result("T15 rbus arbitration matrix: EX1-group > DIV > MUL-EX3, confirmed by peeling");
}

//-----------------------------------------------------------------------------
// T16 (task 4.2/4.3): mutation-style check that the one-hot completion
// assertion actually fires when deliberately violated, then clears once
// reverted to clean stimulus.
//-----------------------------------------------------------------------------
static void test_onehot_mutation(void) {
    tie_idle_inputs();
    dut->iu_rtu_ex1_alu_cmplt_dp = 1;
    dut->eval();
    check(!RTUP(dut)->dbg_onehot_violation, "onehot mutation: baseline (ALU alone) is clean");

    // Deliberately violate: TWO sources' cmplt_dp asserted the same cycle
    // (something IDU's real single-issue dispatch should never allow --
    // this is the mutation).
    dut->iu_rtu_ex1_bju_cmplt_dp = 1;
    dut->eval();
    check(RTUP(dut)->dbg_onehot_violation == 1,
          "onehot mutation: ALU+BJU cmplt_dp simultaneously -> assertion FIRES");

    // Revert to clean stimulus.
    dut->iu_rtu_ex1_bju_cmplt_dp = 0;
    dut->eval();
    check(RTUP(dut)->dbg_onehot_violation == 0, "onehot mutation: reverted -- assertion CLEARS");
    test_result("T16 one-hot assertion: fires on deliberate violation, clears on revert");
}

//-----------------------------------------------------------------------------
// T17 (task 4.2/4.3): mutation-style check for the fwd0/fwd1/fwd2
// destination-register collision assertion, including the x0 exception
// (two producers both targeting x0 is harmless and must NOT trip it).
//-----------------------------------------------------------------------------
static void test_fwd_collision_mutation(void) {
    tie_idle_inputs();
    dut->iu_rtu_ex1_alu_wb_dp = 1;
    dut->iu_rtu_ex1_alu_preg  = 5;
    dut->iu_rtu_ex3_mul_wb_vld = 1;
    dut->iu_rtu_ex3_mul_preg   = 6;
    dut->eval();
    check(!RTUP(dut)->dbg_fwd_collision, "fwd collision mutation: baseline (distinct regs 5 vs 6) is clean");

    // Deliberately violate: fwd0 and fwd1 both target x5 the same cycle.
    dut->iu_rtu_ex3_mul_preg = 5;
    dut->eval();
    check(RTUP(dut)->dbg_fwd_collision == 1,
          "fwd collision mutation: fwd0/fwd1 both target x5 -> assertion FIRES");

    // Revert to distinct registers.
    dut->iu_rtu_ex3_mul_preg = 6;
    dut->eval();
    check(!RTUP(dut)->dbg_fwd_collision, "fwd collision mutation: reverted -- assertion CLEARS");

    // x0 exception: both producers targeting x0 must NOT trip the flag.
    dut->iu_rtu_ex1_alu_preg = 0;
    dut->iu_rtu_ex3_mul_preg = 0;
    dut->eval();
    check(!RTUP(dut)->dbg_fwd_collision,
          "fwd collision mutation: fwd0/fwd1 BOTH targeting x0 is harmless -- assertion stays clear");
    test_result("T17 fwd-collision assertion: fires on deliberate violation, x0 exempted, clears on revert");
}

//=============================================================================
// main
//=============================================================================
int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    dut = new VRTU;

    reset_dut();

    test_reset_state();
    test_alu_create_complete_commit();
    test_bju_create_complete_commit();
    test_mult_create_complete_commit();
    test_div_create_complete_commit();
    test_cp0_create_complete_commit();
    test_cp0_ecall_exception();
    test_mtval_allowlist();
    test_cp0_fetch_fault_tval();
    test_lsu_wb1_and_fwd2();
    test_retire_blocking();
    test_flush_bju_depd_lsu();
    test_flush_mret();
    test_flush_taken_exception();
    test_exception_priority_matrix();
    test_rbus_arbitration_matrix();
    test_onehot_mutation();
    test_fwd_collision_mutation();

    printf("[rtu_tb] %llu cycles, %d failure(s)\n",
           (unsigned long long)g_cycles, g_fail);
    printf("%s\n", g_fail ? "UNIT-FAIL" : "UNIT-PASS");

    dut->final();
    delete dut;
    return g_fail ? 1 : 0;
}
