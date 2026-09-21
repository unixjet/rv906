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
    // M6 Task 7: the replying LSU op's own PC/next-PC. Idle default mirrors
    // the IU display default below (0x1000/0x1004) so solo-LSU dp rows that
    // don't care about epc see a sane value.
    dut->lsu_rtu_ex1_cur_pc       = 0x1000;
    dut->lsu_rtu_ex1_next_pc      = 0x1004;
    dut->lsu_rtu_wb_data          = 0;
    dut->lsu_rtu_wb_preg          = 0;
    dut->lsu_rtu_wb_vld           = 0;
    dut->lsu_rtu_wb_dst_frf       = 0;
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
    // M6 Task 2: drive CSR's registered interrupt-claim export (active-LOW: 1=no claim)
    dut->cp0_rtu_int_sel       = 0;              // no interrupt sources claimed at reset
    dut->cp0_rtu_int_b         = 1;              // active-low valid term tied off at idle
    // FALU (M5 Task 4b: EX1-group rbus arbiter's 3rd leg + registered wbf0)
    dut->fpu_rtu_ex1_falu_fdata = 0;
    dut->fpu_rtu_ex1_falu_xdata = 0;
    dut->fpu_rtu_ex1_falu_fflags = 0;   // M5 Task 8: no accrued flags
    dut->fpu_rtu_ex1_falu_fvld  = 0;
    dut->fpu_rtu_ex1_falu_xvld  = 0;
    dut->fpu_rtu_ex1_falu_preg  = 0;
    // M7 Task 1: DTU/debug ports, idle = "no debugger attached, no halt".
    dut->dtu_rtu_sync_halt_req  = 0;
    dut->dtu_rtu_resume_req     = 0;
    dut->dtu_rtu_step_en        = 0;
    dut->dtu_rtu_int_mask       = 0;
    dut->dtu_rtu_ebreak_action  = 0;
    dut->dtu_rtu_dpc            = 0;
    dut->cp0_rtu_ebreak_halt    = 0;
    dut->cp0_rtu_ex1_inst_dret  = 0;
    dut->ifu_rtu_reset_halt_req = 0;
    // M7 Task 2: trigger halt_info carriers + DTU pending level, idle =
    // "no trigger match" (all zeros -> no leg-4 trap, no trigger halt).
    dut->iu_rtu_ex1_halt_info   = 0;
    dut->lsu_rtu_ex1_halt_info  = 0;
    dut->dtu_rtu_pending_halt   = 0;
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
    check(dut->rtu_idu_wbf1_vld == 0, "reset: wbf1 quiescent");
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
    // M7 Task 1: debug state quiescent at reset (off-path identity).
    check(dut->rtu_yy_xx_dbgon == 0, "reset: dbgon quiescent (off-path identity)");
    check(dut->rtu_dtu_halt_ack == 0, "reset: halt_ack quiescent");
    check(dut->rtu_cp0_exit_debug == 0, "reset: exit_debug quiescent");
    check(dut->rtu_dtu_retire_debug_expt_vld == 0, "reset: retire_debug_expt quiescent");
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
    // M6 Task 7: solo-LSU dp -- the epc must come from the LSU's OWN
    // replying-op pc (0x3000), NOT the decayed IU display (0x1000).
    dut->lsu_rtu_ex1_cur_pc   = 0x3000;
    dut->lsu_rtu_ex1_next_pc  = 0x3004;
    tick();
    check(dut->rtu_yy_xx_expt_vec == 2, "allowlist: vec==2 propagated");
    check(dut->rtu_cp0_tval == 0xDEAD, "allowlist: vec 2 IS allowlisted -- tval populated",
          dut->rtu_cp0_tval, 0xDEAD);
    check(dut->rtu_cp0_epc == 0x3000, "solo-LSU dp: epc == replying op's pc (LSU export, not IU display)",
          dut->rtu_cp0_epc, 0x3000);

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
// T9b (M5 Task 4c, D8): lsu_rtu_wb_dst_frf steers the SAME lsu_rtu_wb_* payload
// to wbf1 instead of wb1 -- an FLW/FLD completion must NOT also fire wb1 (its
// dst0_reg index was never allocated in the GPR scoreboard), and a plain
// GPR-destined completion (dst_frf=0, T9 above) must NOT also fire wbf1.
//-----------------------------------------------------------------------------
static void test_lsu_wbf1_dst_frf_routing(void) {
    tie_idle_inputs();
    dut->lsu_rtu_wb_vld     = 1;
    dut->lsu_rtu_wb_data    = 0x3f800000;
    dut->lsu_rtu_wb_preg    = 5;
    dut->lsu_rtu_wb_dst_frf = 1;
    dut->eval();
    check(dut->rtu_idu_wbf1_vld && dut->rtu_idu_wbf1_data == 0x3f800000 && dut->rtu_idu_wbf1_reg == 5,
          "LSU wbf1: combinational passthrough when dst_frf==1, SAME cycle");
    check(!dut->rtu_idu_wb1_vld, "LSU wbf1: dst_frf==1 does NOT also fire wb1 (GPR)");

    tie_idle_inputs();
    dut->eval();
    check(!dut->rtu_idu_wbf1_vld, "LSU wbf1: clears immediately (combinational), no lingering registered value");

    dut->lsu_rtu_wb_vld     = 1;
    dut->lsu_rtu_wb_data    = 0x9999;
    dut->lsu_rtu_wb_preg    = 4;
    dut->lsu_rtu_wb_dst_frf = 0;
    dut->eval();
    check(dut->rtu_idu_wb1_vld, "LSU wb1: dst_frf==0 fires wb1 (GPR) as before");
    check(!dut->rtu_idu_wbf1_vld, "LSU wb1: dst_frf==0 does NOT also fire wbf1 (FRF)");

    test_result("T9b LSU wbf1 dst_frf routing (M5 Task 4c, D8): wb1/wbf1 are mutually exclusive");
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

//-----------------------------------------------------------------------------
// T18: M5 Task 4b's 3rd EX1-group rbus leg -- fpu_rtu_ex1_falu_xvld/_preg/
// _xdata (GPR-destined FALU results: fcmp/fclass, RTU.v:518-538). The OUTER
// arbiter (ex1-group vs DIV vs MUL-EX3, RTU.v:549-568) mirrors T15's
// peel-one-leg-at-a-time pattern and IS a priority chain -- the FALU xvld
// leg must win it over DIV/MUL exactly like the ALU/BJU legs already do in
// T15, and must gate div_wb_grant/mul_wb_grant (RTU.v:541-542, ex1_wb_dp is
// leg-agnostic). The INNER ex1_wb_src_vld case (RTU.v:526-536) is NOT a
// priority chain, just a one-hot mux over 3 mutually-exclusive producers
// (assumed exclusive by single-issue dispatch, undefended in RTL) -- the
// second sub-test below confirms that asserting two of its legs at once (a
// structurally-unreachable combo) hits the case default for preg/data
// while vld (a separate plain OR) still fires -- a latent landmine if that
// combo were ever reachable, recorded here rather than fixed.
//-----------------------------------------------------------------------------
static void test_falu_xvld_arbiter_leg(void) {
    // FALU alone vs DIV/MUL contending: FALU must win, grants must drop.
    tie_idle_inputs();
    dut->fpu_rtu_ex1_falu_xvld = 1;
    dut->fpu_rtu_ex1_falu_preg = 20;
    dut->fpu_rtu_ex1_falu_xdata = 0xF1;
    dut->iu_rtu_div_wb_dp  = 1;
    dut->iu_rtu_div_wb_vld = 1;
    dut->iu_rtu_div_data   = 0xD9;
    dut->iu_rtu_div_preg   = 21;
    dut->iu_rtu_ex3_mul_wb_vld = 1;
    dut->iu_rtu_ex3_mul_data   = 0xE9;
    dut->iu_rtu_ex3_mul_preg   = 22;
    dut->eval();
    check(dut->rtu_iu_div_wb_grant == 0, "FALU leg: DIV NOT granted while FALU xvld wants the bus");
    check(dut->rtu_iu_mul_wb_grant == 0, "FALU leg: MUL NOT granted while FALU xvld wants the bus");
    tick();
    check(dut->rtu_idu_wb0_vld && dut->rtu_idu_wb0_data == 0xF1 && dut->rtu_idu_wb0_reg == 20,
          "FALU leg: FALU xvld wins over DIV and MUL-EX3", dut->rtu_idu_wb0_data, 0xF1);

    // The 3-way ex1_wb_src_vld case decodes only the one-hot patterns
    // 3'b001/3'b010/3'b100 -- it is a mux, not a priority encoder, so
    // asserting two legs at once (a structurally-unreachable combo under
    // single-issue dispatch) hits the default arm for preg/data (both go
    // to 0). Note ex1_wb_vld itself is a PLAIN OR of the four raw *_vld
    // signals (RTU.v:538), independent of that case -- so vld still fires
    // even though the data it's paired with is garbage. This is a latent
    // landmine if the combo were ever reachable, but it structurally isn't
    // (same "not defended against" property already noted for T15's
    // outer arbiter) -- recorded here, not fixed, since fixing an
    // unreachable path is out of scope.
    tie_idle_inputs();
    dut->fpu_rtu_ex1_falu_xvld  = 1;
    dut->fpu_rtu_ex1_falu_preg  = 23;
    dut->fpu_rtu_ex1_falu_xdata = 0xF2;
    dut->cp0_rtu_ex1_wb_vld  = 1;
    dut->cp0_rtu_ex1_wb_data = 0xC1;
    dut->cp0_rtu_ex1_wb_preg = 24;
    dut->eval();
    tick();
    check(dut->rtu_idu_wb0_vld == 1 && dut->rtu_idu_wb0_data == 0 && dut->rtu_idu_wb0_reg == 0,
          "FALU leg: simultaneous FALU+CP0 (structurally unreachable) hits the case default for data, vld still ORs true",
          dut->rtu_idu_wb0_data, 0);

    // FALU alone: div/mul grants held at 1 when it retreats.
    tie_idle_inputs();
    dut->eval();
    check(dut->rtu_iu_div_wb_grant == 1, "FALU leg: DIV grant restored once FALU xvld retreats");
    check(dut->rtu_iu_mul_wb_grant == 1, "FALU leg: MUL grant restored once FALU xvld retreats");
    test_result("T18 FALU xvld arbiter leg (M5 Task 4b): wins EX1-group case, gates div/mul grants");
}

//-----------------------------------------------------------------------------
// T19: M5 Task 4b's registered wbf0 output (FRF-destined FALU results:
// fadd/fsub/fminmax/fsgnj/f2f-convert, RTU.v:955-963). Driven directly by
// fpu_rtu_ex1_falu_fvld/_preg/_fdata with NO arbiter (FALU is the only wbf0
// producer until FMAU/FDSU land) -- one cycle of register delay, mirroring
// the existing wb0_vld_r/preg_r/data_r pattern exercised by T2/T15. Also
// confirms wbf0 is independent of the xvld leg tested in T18 (driving both
// simultaneously must not cross-contaminate either output).
//-----------------------------------------------------------------------------
static void test_wbf0_register(void) {
    tie_idle_inputs();
    dut->fpu_rtu_ex1_falu_fvld  = 1;
    dut->fpu_rtu_ex1_falu_preg  = 8;
    dut->fpu_rtu_ex1_falu_fdata = 0x1122334455667788ULL;
    dut->eval();
    check(!dut->rtu_idu_wbf0_vld, "wbf0: not yet valid same cycle as fvld (registered, not combinational)");
    tick();
    check(dut->rtu_idu_wbf0_vld && dut->rtu_idu_wbf0_reg == 8 &&
          dut->rtu_idu_wbf0_data == 0x1122334455667788ULL,
          "wbf0: fires exactly ONE cycle after fvld", dut->rtu_idu_wbf0_data, 0x1122334455667788ULL);
    tie_idle_inputs();
    tick();
    check(!dut->rtu_idu_wbf0_vld, "wbf0: clears once idle (no queue, no skid)");

    // wbf0 (fvld/fdata) and the xvld arbiter leg (xvld/xdata) driven
    // together must land on their own independent outputs, unmixed.
    tie_idle_inputs();
    dut->fpu_rtu_ex1_falu_fvld  = 1;
    dut->fpu_rtu_ex1_falu_preg  = 9;
    dut->fpu_rtu_ex1_falu_fdata = 0xAAAA;
    dut->fpu_rtu_ex1_falu_xvld  = 1;
    dut->fpu_rtu_ex1_falu_preg  = 9;   // shared preg port, per RTU.v's port list
    dut->fpu_rtu_ex1_falu_xdata = 0xBBBB;
    dut->eval();
    tick();
    check(dut->rtu_idu_wbf0_vld && dut->rtu_idu_wbf0_data == 0xAAAA,
          "wbf0: fvld/fdata land on wbf0 unmixed with the concurrent xvld leg", dut->rtu_idu_wbf0_data, 0xAAAA);
    check(dut->rtu_idu_wb0_vld && dut->rtu_idu_wb0_data == 0xBBBB,
          "wbf0: concurrent xvld/xdata land on wb0 unmixed with wbf0", dut->rtu_idu_wb0_data, 0xBBBB);
    test_result("T19 wbf0 registered writeback (M5 Task 4b): one-cycle delay, independent of xvld leg");
}

//-----------------------------------------------------------------------------
// T20: M5 Task 8 -- the FPU joins the one-hot completion bus. An FP op
// completing in EX1 (fvld OR xvld, FPU.v SECTION 13 -- both pulse only on
// the cycle the result mux actually outputs, incl. FDSU's completion cycle)
// must: (a) assert the pcgen trigger rtu_iu_ex1_cmplt the SAME cycle with
// inst_len==1 (FP ops are always 32-bit; RVC has no F/D encodings),
// (b) register into the EX1->EX2 retire packet -> rtu_cp0_inst_retire ONE
// cycle later, and (c) carry the EX1-cycle flags onto the new
// rtu_cp0_fflags/rtu_cp0_fs_dirty_updt outputs CSR.v consumes for the D7
// sticky OR-in + mstatus.FS dirty.
//-----------------------------------------------------------------------------
static void test_fpu_cmplt_retire_heartbeat(void) {
    // fvld leg (FRF-destined result).
    tie_idle_inputs();
    dut->fpu_rtu_ex1_falu_fvld   = 1;
    dut->fpu_rtu_ex1_falu_fflags = 0b01010;   // NX|DZ
    dut->eval();
    check(dut->rtu_iu_ex1_cmplt == 1, "FPU cmplt: pcgen trigger asserted same cycle as fvld");
    check(dut->rtu_iu_ex1_inst_len == 1, "FPU cmplt: inst_len==1 (32-bit; no FP in RVC)");
    check(!RTUP(dut)->dbg_onehot_violation, "FPU cmplt: single source, one-hot clean");
    tick();
    check(dut->rtu_cp0_inst_retire == 1,
          "FPU cmplt: rtu_cp0_inst_retire fires ONE cycle later (EX2 retire packet)");
    check(dut->rtu_cp0_fflags == 0b01010,
          "FPU cmplt: rtu_cp0_fflags carries the EX1-cycle flags", dut->rtu_cp0_fflags, 0b01010);
    check(dut->rtu_cp0_fs_dirty_updt == 1,
          "FPU cmplt: rtu_cp0_fs_dirty_updt fires on retire (EX2)");
    tie_idle_inputs();
    tick();
    check(dut->rtu_cp0_inst_retire == 0 && dut->rtu_cp0_fs_dirty_updt == 0,
          "FPU cmplt: the retire pulse is one-cycle, clears when idle");

    // xvld leg (GPR-destined fcmp/fclass) drives the same retire path.
    tie_idle_inputs();
    dut->fpu_rtu_ex1_falu_xvld   = 1;
    dut->fpu_rtu_ex1_falu_fflags = 0b00100;   // OF
    dut->eval();
    check(dut->rtu_iu_ex1_cmplt == 1, "FPU cmplt: xvld alone also triggers pcgen");
    check(dut->rtu_iu_ex1_inst_len == 1, "FPU cmplt: xvld completer also 32-bit");
    tick();
    check(dut->rtu_cp0_fs_dirty_updt == 1,
          "FPU cmplt: xvld retire also pulses rtu_cp0_fs_dirty_updt");
    check(dut->rtu_cp0_fflags == 0b00100,
          "FPU cmplt: xvld retire also carries its flags", dut->rtu_cp0_fflags, 0b00100);
    test_result("T20 FPU cmplt leg (M5 Task 8): fvld/xvld -> retire heartbeat + fflags/FS-dirty to CSR");
}

//=============================================================================
// M7 Task 1 -- core-side debug halt machinery (donor aq_rtu_retire.v).
// Timing map for the t1 (retire-boundary) halt:
//   T   : cp0_rtu_ex1_cmplt_dp=1 (retiring inst in EX1), iu_rtu_ex1_cur_pc=X
//   T+1 : ex2_retire_vld=1; dtu_rtu_sync_halt_req=1 -> halt_req_t1=1:
//         rtu_dtu_halt_ack=1, rtu_dtu_halt_cause=3, rtu_dtu_dpc=X (donor
//         aq_rtu_retire.v:1200-1202: cur_pc for the sync leg); flush IDLE->FE
//         (retire_inst_flush_fe_set = || halt_req); dbg_mode_on_after_req=1
//   T+2 : flush FE->BE (cpu_no_op)
//   T+3 : flush BE, dbg_mode_on set at this edge -> rtu_yy_xx_dbgon=1 at T+4
//=============================================================================

static void test_m7_dm_sync_halt_t1(void) {
    tie_idle_inputs();
    // T: a CP0 op retires through EX1 (pc 0x7000).
    dut->cp0_rtu_ex1_cmplt_dp = 1;
    dut->iu_rtu_ex1_cur_pc    = 0x7000;
    dut->iu_rtu_ex1_next_pc   = 0x7004;
    dut->eval();
    check(dut->rtu_dtu_halt_ack == 0, "t1: no ack yet (pre-retire)");
    tick();   // -> T+1: ex2_retire_vld=1
    dut->cp0_rtu_ex1_cmplt_dp = 0;   // one-cycle retire pulse; let ex2 fall at T+2
                                     // so cpu_no_op frees the flush FE->BE (S6)

    // T+1: the DM's sync-halt request lands on the retire boundary.
    dut->dtu_rtu_sync_halt_req = 1;
    dut->eval();
    check(dut->rtu_dtu_halt_ack == 1, "t1: halt_ack pulses at the retire boundary");
    check(dut->rtu_dtu_halt_cause == 3, "t1: cause == 3 (dm_sync)", dut->rtu_dtu_halt_cause, 3);
    check(dut->rtu_dtu_dpc == 0x7000, "t1: dpc latch source == retiring inst's cur_pc",
          dut->rtu_dtu_dpc, 0x7000);
    check(dut->rtu_yy_xx_dbgon == 0, "t1: dbgon not yet (flush not at BE)");
    tick();   // -> T+2: flush FE
    tie_idle_inputs();
    check(dut->rtu_idu_flush_fe == 1, "t1: flush FE reached");
    tick();   // -> T+3: flush BE
    check(dut->rtu_idu_flush_wbt == 1, "t1: flush BE reached");
    tick();   // -> T+4: dbg_mode_on set
    check(dut->rtu_yy_xx_dbgon == 1, "t1: rtu_yy_xx_dbgon=1 once flush completes",
          dut->rtu_yy_xx_dbgon, 1);
    for (int i = 0; i < 3; i++) tick();   // settle idle
    check(dut->rtu_yy_xx_dbgon == 1, "t1: dbgon holds (no exit requested)");
    test_result("T21 M7 dm-sync halt (t1): ack/cause3/dpc=cur_pc, dbgon after flush BE");

    // Teardown: resume out of debug (also exercised fully in T24).
    dut->dtu_rtu_resume_req = 1;
    dut->eval();
    tick();
    tie_idle_inputs();
    for (int i = 0; i < 2; i++) tick();
}

static void test_m7_ebreak_reset_halt_t0(void) {
    // (a) EBREAK-with-action: timing-0, cause 1, no retire needed at all.
    tie_idle_inputs();
    dut->cp0_rtu_ebreak_halt = 1;
    dut->eval();
    check(dut->rtu_dtu_halt_ack == 1, "t0 ebreak: halt_ack IMMEDIATE (no retire boundary)");
    check(dut->rtu_dtu_halt_cause == 1, "t0 ebreak: cause == 1 (ebreak)",
          dut->rtu_dtu_halt_cause, 1);
    check(dut->rtu_yy_xx_dbgon == 0, "t0 ebreak: dbgon not yet");
    tick();   // flush IDLE->FE at this edge
    tie_idle_inputs();
    tick();   // FE->BE
    tick();   // BE->IDLE, dbg_mode_on set at this edge
    check(dut->rtu_yy_xx_dbgon == 1, "t0 ebreak: dbgon=1 after flush");
    // Resume out.
    dut->dtu_rtu_resume_req = 1;
    dut->eval();
    tick();
    tie_idle_inputs();
    for (int i = 0; i < 2; i++) tick();
    check(dut->rtu_yy_xx_dbgon == 0, "t0 ebreak: resumed out of debug");

    // (b) RESET-halt from the IFU: timing-0, cause 5.
    tie_idle_inputs();
    dut->ifu_rtu_reset_halt_req = 1;
    dut->eval();
    check(dut->rtu_dtu_halt_ack == 1, "t0 reset: halt_ack immediate");
    check(dut->rtu_dtu_halt_cause == 5, "t0 reset: cause == 5 (reset)",
          dut->rtu_dtu_halt_cause, 5);
    tick();
    tie_idle_inputs();
    tick();
    tick();
    check(dut->rtu_yy_xx_dbgon == 1, "t0 reset: dbgon=1 after flush");
    dut->dtu_rtu_resume_req = 1;
    dut->eval();
    tick();
    tie_idle_inputs();
    for (int i = 0; i < 2; i++) tick();
    check(dut->rtu_yy_xx_dbgon == 0, "t0 reset: resumed out of debug");
    test_result("T22 M7 timing-0 halts: ebreak(cause1) + reset(cause5), immediate ack");
}

static void test_m7_step_halt_and_int_mask(void) {
    // (a) Single-step: timing-1, cause 4.
    tie_idle_inputs();
    dut->cp0_rtu_ex1_cmplt_dp = 1;
    dut->iu_rtu_ex1_cur_pc    = 0x8000;
    dut->iu_rtu_ex1_next_pc   = 0x8004;
    tick();   // -> ex2_retire_vld
    dut->cp0_rtu_ex1_cmplt_dp = 0;   // one-cycle retire pulse; ex2 falls at T+2
    dut->dtu_rtu_step_en = 1;
    dut->eval();
    check(dut->rtu_dtu_halt_ack == 1, "step: halt_ack at retire boundary");
    check(dut->rtu_dtu_halt_cause == 4, "step: cause == 4 (step)",
          dut->rtu_dtu_halt_cause, 4);
    tick();
    tie_idle_inputs();
    tick();
    tick();
    check(dut->rtu_yy_xx_dbgon == 1, "step: dbgon=1 after flush");
    dut->dtu_rtu_resume_req = 1;
    dut->eval();
    tick();
    tie_idle_inputs();
    for (int i = 0; i < 2; i++) tick();
    check(dut->rtu_yy_xx_dbgon == 0, "step: resumed out of debug");

    // (b) dcsr.int_mask (step && !stepie) blocks the interrupt-claim retire.
    // Baseline: an interrupt claim (cp0_rtu_int_b=0) on a normal ALU retire
    // takes the interrupt.
    tie_idle_inputs();
    dut->cp0_rtu_int_b = 0;   // a claim is present
    dut->iu_rtu_ex1_alu_cmplt    = 1;
    dut->iu_rtu_ex1_alu_cmplt_dp = 1;
    tick();   // -> ex2_retire_vld
    dut->iu_rtu_ex1_alu_cmplt    = 0;
    dut->iu_rtu_ex1_alu_cmplt_dp = 0;
    dut->eval();
    check(dut->rtu_yy_xx_expt_vld == 1, "int_mask=0: interrupt claim taken at retire",
          dut->rtu_yy_xx_expt_vld, 1);
    // Now with the DTU's step-interrupt mask asserted: no interrupt.
    tie_idle_inputs();
    dut->dtu_rtu_int_mask = 1;
    dut->cp0_rtu_int_b = 0;
    dut->iu_rtu_ex1_alu_cmplt    = 1;
    dut->iu_rtu_ex1_alu_cmplt_dp = 1;
    tick();
    dut->iu_rtu_ex1_alu_cmplt    = 0;
    dut->iu_rtu_ex1_alu_cmplt_dp = 0;
    dut->eval();
    check(dut->rtu_yy_xx_expt_vld == 0, "int_mask=1: interrupt claim masked (single-step)",
          dut->rtu_yy_xx_expt_vld, 0);
    tie_idle_inputs();
    test_result("T23 M7 step halt (cause4) + dcsr.int_mask blocks interrupt retire");
}

static void test_m7_exit_debug_resume_and_dret(void) {
    // Enter debug via the ebreak timing-0 halt.
    tie_idle_inputs();
    dut->cp0_rtu_ebreak_halt = 1;
    dut->eval();
    tick();
    tie_idle_inputs();
    tick();
    tick();
    check(dut->rtu_yy_xx_dbgon == 1, "exit: entered debug (prerequisite)");

    // (a) RESUME exit: rtu_cp0_exit_debug pulses AND a redirect to dpc.
    // The donor's exit_debug chgflw leg fires for resume too (not just dret):
    // aq_rtu_retire.v:1008 ORs retire_exit_debug into retire_chgflw_vld and
    // :1032 points retire_chgflw_pc at dtu_rtu_dpc for either leg.
    dut->dtu_rtu_dpc = 0x4000;   // debugger's resume target
    dut->dtu_rtu_resume_req = 1;
    dut->eval();
    check(dut->rtu_cp0_exit_debug == 1, "resume: rtu_cp0_exit_debug pulses");
    check(dut->rtu_ifu_chgflw_vld == 1, "resume: redirect asserted (exit_debug leg)");
    check(dut->rtu_ifu_chgflw_pc == 0x4000, "resume: redirect target == dpc",
          dut->rtu_ifu_chgflw_pc, 0x4000);
    tick();
    tie_idle_inputs();
    check(dut->rtu_yy_xx_dbgon == 0, "resume: dbgon cleared");
    for (int i = 0; i < 3; i++) tick();

    // (b) DRET exit: re-enter debug, then retire a dret -> redirect to
    // dtu_rtu_dpc (the debugger-set resume PC), dbgon cleared.
    dut->cp0_rtu_ebreak_halt = 1;
    dut->eval();
    tick();
    tie_idle_inputs();
    tick();
    tick();
    check(dut->rtu_yy_xx_dbgon == 1, "dret exit: re-entered debug");

    dut->dtu_rtu_dpc          = 0x8000;   // debugger's resume target
    dut->cp0_rtu_ex1_cmplt_dp = 1;        // the dret retires
    dut->cp0_rtu_ex1_inst_dret = 1;
    tick();   // -> ex2_retire_vld && ex2_inst_dret
    dut->eval();
    check(dut->rtu_cp0_exit_debug == 1, "dret exit: rtu_cp0_exit_debug pulses");
    check(dut->rtu_ifu_chgflw_vld == 1, "dret exit: redirect asserted");
    check(dut->rtu_ifu_chgflw_pc == 0x8000, "dret exit: redirect target == dtu_rtu_dpc",
          dut->rtu_ifu_chgflw_pc, 0x8000);
    tick();
    tie_idle_inputs();
    check(dut->rtu_yy_xx_dbgon == 0, "dret exit: dbgon cleared");
    for (int i = 0; i < 3; i++) tick();
    check(dut->rtu_ifu_chgflw_vld == 0, "dret exit: redirect is one-shot");
    test_result("T24 M7 exit-debug: resume (redirect to dpc) + dret (redirect to dpc)");
}

static void test_m7_debug_mode_exception(void) {
    // Enter debug.
    tie_idle_inputs();
    dut->cp0_rtu_ebreak_halt = 1;
    dut->eval();
    tick();
    tie_idle_inputs();
    tick();
    tick();
    check(dut->rtu_yy_xx_dbgon == 1, "dbg-expt: entered debug (prerequisite)");

    // An ecall retires WHILE in debug: no architectural trap (retire_trap_vld
    // is gated by !dbg_mode_on), but the DM sees retire_debug_expt_vld.
    dut->cp0_rtu_ex1_cmplt_dp = 1;
    dut->cp0_rtu_ex1_expt_vld = 1;
    dut->cp0_rtu_ex1_expt_vec = 11;   // ecall
    dut->iu_rtu_ex1_cur_pc    = 0x9000;
    tick();   // -> ex2_retire_vld
    dut->cp0_rtu_ex1_cmplt_dp = 0;
    dut->cp0_rtu_ex1_expt_vld = 0;
    dut->eval();
    check(dut->rtu_yy_xx_expt_vld == 0, "dbg-expt: NO architectural trap while dbgon",
          dut->rtu_yy_xx_expt_vld, 0);
    check(dut->rtu_dtu_retire_debug_expt_vld == 1,
          "dbg-expt: rtu_dtu_retire_debug_expt_vld pulses for the DM");
    tie_idle_inputs();
    // Resume out of debug for the next scenario (t1 halts only ack outside
    // debug -- dbg_mode_on_after_req gates them).
    dut->dtu_rtu_resume_req = 1;
    dut->eval();
    tick();
    tie_idle_inputs();
    for (int i = 0; i < 3; i++) tick();
    check(dut->rtu_yy_xx_dbgon == 0, "halt-defer: back out of debug (prerequisite)");

    // A concurrent t1 HALT request suppresses a pending sync trap: the halt
    // takes the retire boundary, the trap is deferred (retire_trap_vld is
    // gated by !halt_req).
    dut->cp0_rtu_ex1_cmplt_dp = 1;
    dut->cp0_rtu_ex1_expt_vld = 1;
    dut->cp0_rtu_ex1_expt_vec = 2;    // illegal instruction
    tick();   // -> ex2_retire_vld
    dut->cp0_rtu_ex1_cmplt_dp = 0;
    dut->cp0_rtu_ex1_expt_vld = 0;
    dut->dtu_rtu_sync_halt_req = 1;
    dut->eval();
    check(dut->rtu_dtu_halt_ack == 1, "halt-defer: halt taken at the boundary");
    check(dut->rtu_yy_xx_expt_vld == 0, "halt-defer: pending trap suppressed by the halt");
    tick();
    tie_idle_inputs();
    tick();
    tick();
    check(dut->rtu_yy_xx_dbgon == 1, "halt-defer: halted (dbgon=1)");
    // Resume out.
    dut->dtu_rtu_resume_req = 1;
    dut->eval();
    tick();
    tie_idle_inputs();
    for (int i = 0; i < 3; i++) tick();
    test_result("T25 M7 exception-in-debug: no arch trap + retire_debug_expt_vld; halt defers trap");
}

//=============================================================================
// M7 Task 2 -- trigger halt/trap legs (RTU.v leg 4 + halt section).
//
// halt_info encoding (rvproc_pkg.sv TDT_HINFO_*): CANCEL[0] MATCH[1]
// LDST[2] CHAIN[3] ACTION[4] ACTION01[5] TIMING[6] PENDING_HALT[7]
// CAUSE[11:8] TRIGGER[21:12]. Bundles driven on iu_rtu_ex1_halt_info are
// latched into ex2_halt_info at the dp cycle, exactly as the IFU/LSU
// carriers are in the full core.
//
//   t0 bp     = 0x3   (CANCEL|MATCH, ACTION=0, TIMING=0)
//   t0 halt   = 0x13  (CANCEL|MATCH|ACTION)
//   t1 bp     = 0x42  (MATCH|TIMING, ACTION=0, CANCEL=0)
//   t1 halt   = 0x52  (MATCH|ACTION|TIMING, CANCEL=0)
//=============================================================================

static void test_m7_trigger_breakpoint_t0(void) {
    // Baseline: a plain ALU wb instruction commits wb0 one cycle later.
    tie_idle_inputs();
    dut->iu_rtu_ex1_alu_cmplt    = 1;
    dut->iu_rtu_ex1_alu_cmplt_dp = 1;
    dut->iu_rtu_ex1_alu_wb_dp    = 1;
    dut->iu_rtu_ex1_alu_wb_vld   = 1;
    dut->iu_rtu_ex1_alu_data     = 0x1234;
    dut->iu_rtu_ex1_alu_preg     = 5;
    dut->iu_rtu_ex1_cur_pc       = 0xA000;
    dut->iu_rtu_ex1_next_pc      = 0xA004;
    tick();
    check(dut->rtu_idu_wb0_vld && dut->rtu_idu_wb0_data == 0x1234,
          "t0 bp baseline: untriggered ALU wb commits wb0");
    tie_idle_inputs();
    tick();

    // Triggered: the SAME instruction with a t0 breakpoint bundle.
    tie_idle_inputs();
    dut->iu_rtu_ex1_alu_cmplt    = 1;
    dut->iu_rtu_ex1_alu_cmplt_dp = 1;
    dut->iu_rtu_ex1_alu_wb_dp    = 1;
    dut->iu_rtu_ex1_alu_wb_vld   = 1;
    dut->iu_rtu_ex1_alu_data     = 0x1234;
    dut->iu_rtu_ex1_alu_preg     = 5;
    dut->iu_rtu_ex1_cur_pc       = 0xA000;
    dut->iu_rtu_ex1_next_pc      = 0xA004;
    dut->iu_rtu_ex1_halt_info    = 0x3;   // CANCEL|MATCH, ACTION=0
    tick();   // -> T+1: ex2_retire_vld, leg 4 fires
    dut->iu_rtu_ex1_alu_cmplt    = 0;
    dut->iu_rtu_ex1_alu_cmplt_dp = 0;
    dut->iu_rtu_ex1_alu_wb_dp    = 0;
    dut->iu_rtu_ex1_alu_wb_vld   = 0;
    dut->iu_rtu_ex1_halt_info    = 0;
    dut->eval();
    check(dut->rtu_yy_xx_expt_vld == 1, "t0 bp: trap declared at retire",
          dut->rtu_yy_xx_expt_vld, 1);
    check(dut->rtu_yy_xx_expt_vec == 3, "t0 bp: vec 3 (breakpoint)",
          dut->rtu_yy_xx_expt_vec, 3);
    check(dut->rtu_cp0_epc == 0xA000, "t0 bp: epc == trigger's cur_pc",
          dut->rtu_cp0_epc, 0xA000);
    check(dut->rtu_cp0_tval == 0, "t0 bp: tval == 0", dut->rtu_cp0_tval, 0);
    check(dut->rtu_dtu_halt_ack == 0, "t0 bp: NO halt (action 0)");
    check(dut->rtu_idu_wb0_vld == 0, "t0 bp: wb0 CANCELLED (CANCEL bit)",
          dut->rtu_idu_wb0_vld, 0);
    check(dut->rtu_dtu_retire_halt_info == 0x3,
          "t0 bp: rtu_dtu_retire_halt_info feeds back the latched bundle",
          dut->rtu_dtu_retire_halt_info, 0x3);
    // Settle the flush (the trap redirects), then resume if any halt —
    // none here; just drain the flush FSM.
    tie_idle_inputs();
    for (int i = 0; i < 5; i++) tick();
    test_result("T26 M7 trigger breakpoint (t0 action0): vec3/epc=cur_pc/tval0, wb0 cancelled, no halt");
}

static void test_m7_trigger_halt_t0(void) {
    // A t0 action-1 trigger HALTS (cause 2) instead of trapping.
    tie_idle_inputs();
    dut->cp0_rtu_ex1_cmplt_dp = 1;
    dut->iu_rtu_ex1_cur_pc    = 0xB000;
    dut->iu_rtu_ex1_next_pc   = 0xB004;
    dut->iu_rtu_ex1_halt_info = 0x13;   // CANCEL|MATCH|ACTION
    tick();   // -> ex2_retire_vld, halt_req_trigger_t0
    dut->cp0_rtu_ex1_cmplt_dp = 0;
    dut->iu_rtu_ex1_halt_info = 0;
    dut->eval();
    check(dut->rtu_dtu_halt_ack == 1, "t0 halt: halt_ack at the retire boundary");
    check(dut->rtu_dtu_halt_cause == 2, "t0 halt: cause == 2 (trigger)",
          dut->rtu_dtu_halt_cause, 2);
    check(dut->rtu_dtu_dpc == 0xB000, "t0 halt: dpc == trigger's cur_pc",
          dut->rtu_dtu_dpc, 0xB000);
    check(dut->rtu_yy_xx_expt_vld == 0, "t0 halt: NO architectural trap (halt instead)",
          dut->rtu_yy_xx_expt_vld, 0);
    check(dut->rtu_yy_xx_dbgon == 0, "t0 halt: dbgon not yet (flush not at BE)");
    tick();
    tie_idle_inputs();
    tick();
    tick();
    check(dut->rtu_yy_xx_dbgon == 1, "t0 halt: dbgon=1 after flush BE");
    // Resume out.
    dut->dtu_rtu_resume_req = 1;
    dut->eval();
    tick();
    tie_idle_inputs();
    for (int i = 0; i < 3; i++) tick();
    check(dut->rtu_yy_xx_dbgon == 0, "t0 halt: resumed out of debug");
    test_result("T27 M7 trigger halt (t0 action1): cause2, dpc=cur_pc, no trap, dbgon after flush");
}

static void test_m7_trigger_timing1(void) {
    // (a) t1 action-1 (after-completion): the instruction's side effects
    // commit (CANCEL=0 -> wb0 NOT cancelled), THEN the halt takes the
    // boundary (cause 2).
    tie_idle_inputs();
    dut->iu_rtu_ex1_alu_cmplt    = 1;
    dut->iu_rtu_ex1_alu_cmplt_dp = 1;
    dut->iu_rtu_ex1_alu_wb_dp    = 1;
    dut->iu_rtu_ex1_alu_wb_vld   = 1;
    dut->iu_rtu_ex1_alu_data     = 0x9999;
    dut->iu_rtu_ex1_alu_preg     = 6;
    dut->iu_rtu_ex1_cur_pc       = 0xC000;
    dut->iu_rtu_ex1_next_pc      = 0xC004;
    dut->iu_rtu_ex1_halt_info    = 0x52;   // MATCH|ACTION|TIMING, CANCEL=0
    tick();   // -> ex2_retire_vld, halt_req_trigger_t1
    dut->iu_rtu_ex1_alu_cmplt    = 0;
    dut->iu_rtu_ex1_alu_cmplt_dp = 0;
    dut->iu_rtu_ex1_alu_wb_dp    = 0;
    dut->iu_rtu_ex1_alu_wb_vld   = 0;
    dut->iu_rtu_ex1_halt_info    = 0;
    dut->eval();
    check(dut->rtu_dtu_halt_ack == 1, "t1 halt: halt_ack at the boundary");
    check(dut->rtu_dtu_halt_cause == 2, "t1 halt: cause == 2 (trigger)",
          dut->rtu_dtu_halt_cause, 2);
    check(dut->rtu_idu_wb0_vld == 1 && dut->rtu_idu_wb0_data == 0x9999,
          "t1 halt: side effect COMMITTED (CANCEL=0 -> wb0 not cancelled)");
    check(dut->rtu_yy_xx_expt_vld == 0, "t1 halt: no architectural trap");
    tick();
    tie_idle_inputs();
    tick();
    tick();
    check(dut->rtu_yy_xx_dbgon == 1, "t1 halt: dbgon=1 after flush");
    dut->dtu_rtu_resume_req = 1;
    dut->eval();
    tick();
    tie_idle_inputs();
    for (int i = 0; i < 3; i++) tick();
    check(dut->rtu_yy_xx_dbgon == 0, "t1 halt: resumed out of debug");

    // (b) t1 action-0 (after-completion breakpoint): the instruction
    // commits (wb0 fires), THEN the trap (vec 3, epc=cur_pc) lands.
    tie_idle_inputs();
    dut->iu_rtu_ex1_alu_cmplt    = 1;
    dut->iu_rtu_ex1_alu_cmplt_dp = 1;
    dut->iu_rtu_ex1_alu_wb_dp    = 1;
    dut->iu_rtu_ex1_alu_wb_vld   = 1;
    dut->iu_rtu_ex1_alu_data     = 0x7777;
    dut->iu_rtu_ex1_alu_preg     = 7;
    dut->iu_rtu_ex1_cur_pc       = 0xC100;
    dut->iu_rtu_ex1_next_pc      = 0xC104;
    dut->iu_rtu_ex1_halt_info    = 0x42;   // MATCH|TIMING, ACTION=0
    tick();
    dut->iu_rtu_ex1_alu_cmplt    = 0;
    dut->iu_rtu_ex1_alu_cmplt_dp = 0;
    dut->iu_rtu_ex1_alu_wb_dp    = 0;
    dut->iu_rtu_ex1_alu_wb_vld   = 0;
    dut->iu_rtu_ex1_halt_info    = 0;
    dut->eval();
    check(dut->rtu_yy_xx_expt_vld == 1, "t1 bp: trap at the boundary",
          dut->rtu_yy_xx_expt_vld, 1);
    check(dut->rtu_yy_xx_expt_vec == 3, "t1 bp: vec 3 (breakpoint)",
          dut->rtu_yy_xx_expt_vec, 3);
    check(dut->rtu_cp0_epc == 0xC100, "t1 bp: epc == trigger's cur_pc",
          dut->rtu_cp0_epc, 0xC100);
    check(dut->rtu_idu_wb0_vld == 1 && dut->rtu_idu_wb0_data == 0x7777,
          "t1 bp: side effect COMMITTED first (CANCEL=0)");
    check(dut->rtu_dtu_halt_ack == 0, "t1 bp: no halt (action 0)");
    tie_idle_inputs();
    for (int i = 0; i < 5; i++) tick();
    test_result("T28 M7 trigger timing-1: halt(cause2)+bp(vec3) after side effects commit");
}

static void test_m7_trigger_cause_priority_and_pending(void) {
    // (a) Cause priority: a t0 trigger halt (cause 2) and an ebreak halt
    // (cause 1) land on the SAME retire boundary -> trigger wins (RTU
    // cause ladder: trigger > ebreak > reset > dm_sync > step).
    //
    // NOTE the stimulus timing: cp0_rtu_ebreak_halt is a level, not a
    // latched record -- in the full core the CSR only asserts it when the
    // ebreak itself retires (RTU.v:1045-1048), i.e. at the same boundary.
    // Driving it BEFORE the boundary would take the ebreak halt t0-style
    // on the earlier edge and set dbg_mode_on_after_req, masking the
    // trigger leg -- not the scenario under test.
    tie_idle_inputs();
    dut->cp0_rtu_ex1_cmplt_dp = 1;
    dut->iu_rtu_ex1_cur_pc    = 0xD000;
    dut->iu_rtu_ex1_next_pc   = 0xD004;
    dut->iu_rtu_ex1_halt_info = 0x13;
    tick();
    dut->cp0_rtu_ex1_cmplt_dp = 0;
    dut->cp0_rtu_ebreak_halt  = 1;   // ebreak level lands on the SAME boundary
    dut->eval();
    check(dut->rtu_dtu_halt_ack == 1, "priority: halt taken at the boundary");
    check(dut->rtu_dtu_halt_cause == 2, "priority: trigger(2) beats ebreak(1)",
          dut->rtu_dtu_halt_cause, 2);
    dut->cp0_rtu_ebreak_halt  = 0;
    dut->iu_rtu_ex1_halt_info = 0;
    tick();   // -> dbg_mode_on_after_req latched at the boundary edge, flush FE
    tie_idle_inputs();
    tick();   // -> flush BE
    tick();   // -> dbgon=1
    check(dut->rtu_yy_xx_dbgon == 1, "priority: halted in debug after flush");
    dut->dtu_rtu_resume_req = 1;
    dut->eval();
    tick();
    tie_idle_inputs();
    for (int i = 0; i < 3; i++) tick();
    check(dut->rtu_yy_xx_dbgon == 0, "priority: resumed out of debug");

    // (b) The DTU pending-halt LEVEL: a normal retire boundary with
    // dtu_rtu_pending_halt=1 (no match bits) -> halt (cause 2) and the
    // ack releases the level.
    tie_idle_inputs();
    dut->cp0_rtu_ex1_cmplt_dp = 1;
    dut->iu_rtu_ex1_cur_pc    = 0xE000;
    dut->iu_rtu_ex1_next_pc   = 0xE004;
    dut->dtu_rtu_pending_halt = 1;
    tick();
    dut->cp0_rtu_ex1_cmplt_dp = 0;
    dut->eval();
    check(dut->rtu_dtu_halt_ack == 1, "pending: halt at the boundary");
    check(dut->rtu_dtu_halt_cause == 2, "pending: cause == 2 (trigger class)",
          dut->rtu_dtu_halt_cause, 2);
    check(dut->rtu_dtu_pending_ack == 1, "pending: rtu_dtu_pending_ack releases the level");
    dut->dtu_rtu_pending_halt = 0;   // the DTU drops the level on the ack
    tick();
    check(dut->rtu_dtu_pending_ack == 0, "pending: ack falls with the level");
    tie_idle_inputs();
    tick();
    tick();
    dut->dtu_rtu_resume_req = 1;
    dut->eval();
    tick();
    tie_idle_inputs();
    for (int i = 0; i < 3; i++) tick();
    check(dut->rtu_yy_xx_dbgon == 0, "pending: resumed out of debug");
    test_result("T29 M7 trigger cause priority (trigger>ebreak) + pending-halt level ack");
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
    test_lsu_wbf1_dst_frf_routing();
    test_retire_blocking();
    test_flush_bju_depd_lsu();
    test_flush_mret();
    test_flush_taken_exception();
    test_exception_priority_matrix();
    test_rbus_arbitration_matrix();
    test_onehot_mutation();
    test_fwd_collision_mutation();
    test_falu_xvld_arbiter_leg();
    test_wbf0_register();
    test_fpu_cmplt_retire_heartbeat();

    // M7 Task 1: core-side debug halt machinery.
    test_m7_dm_sync_halt_t1();
    test_m7_ebreak_reset_halt_t0();
    test_m7_step_halt_and_int_mask();
    test_m7_exit_debug_resume_and_dret();
    test_m7_debug_mode_exception();

    // M7 Task 2: trigger halt/trap legs (halt_info from the DTU).
    test_m7_trigger_breakpoint_t0();
    test_m7_trigger_halt_t0();
    test_m7_trigger_timing1();
    test_m7_trigger_cause_priority_and_pending();

    printf("[rtu_tb] %llu cycles, %d failure(s)\n",
           (unsigned long long)g_cycles, g_fail);
    printf("%s\n", g_fail ? "UNIT-FAIL" : "UNIT-PASS");

    dut->final();
    delete dut;
    return g_fail ? 1 : 0;
}
