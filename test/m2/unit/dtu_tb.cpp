//=============================================================================
// dtu_tb.cpp - standalone unit bench for rtl/DTU.v (M7 Task 1)
//=============================================================================
// Verilates DTU.v + rvproc_pkg.sv alone (top-module DTU) and drives the
// frozen cp0/tdt_dm/rtu ports directly with hand-scripted stimulus, the same
// tick()/check()/test_result() bookkeeping pattern as rtu_tb.cpp / csr_tb.cpp.
//
// This is a WHITE-BOX test of DTU.v's OWN documented contract (dcsr/dpc/
// dscratch0/dscratch1 storage, the ebreak-action + int_mask + wake_up
// control outputs, the itr injection channel, the DM-side same-clock
// handshake + havereset FSM, D-M7-1/D-M7-6) run in isolation. It stands in
// for the DM (Task 3) and the RTU/CSR/IFU neighbours.
//
// Build/run: make -C test/m2/unit dtu && bin/unit/dtu_tb
// Prints one line per test and ends with UNIT-PASS or UNIT-FAIL.
//=============================================================================

#include <verilated.h>
#include "VDTU.h"

#include <cstdio>
#include <cstdint>
#include <cstring>

//-----------------------------------------------------------------------------
// CSR addresses + dcsr field bits (mirror DTU.v SECTION DCSR / rvproc_pkg.sv)
//-----------------------------------------------------------------------------
static const uint32_t CSR_DCSR      = 0x7B0;
static const uint32_t CSR_DPC       = 0x7B1;
static const uint32_t CSR_DSCRATCH0 = 0x7B2;
static const uint32_t CSR_DSCRATCH1 = 0x7B3;

// Trigger CSR addresses (rvproc_pkg.sv, M7 Task 2)
static const uint32_t CSR_TSELECT  = 0x7A0;
static const uint32_t CSR_TDATA1   = 0x7A1;
static const uint32_t CSR_TDATA2   = 0x7A2;
static const uint32_t CSR_TDATA3   = 0x7A3;
static const uint32_t CSR_TINFO    = 0x7A4;
static const uint32_t CSR_TCONTROL = 0x7A5;
static const uint32_t CSR_MCONTEXT = 0x7A8;
static const uint32_t CSR_SCONTEXT = 0x7AA;

// dcsr layout (0.13, DTU.v): xdebugver[31:28]=0100, ebreakm[15], ebreaks[13],
// ebreaku[12], stepie[11], stopcount[10], cause[8:6], mprven[4], step[2],
// prv[1:0].
static const int  DCSR_STEP      = 2;
static const int  DCSR_MPRVEN    = 4;
static const int  DCSR_CAUSE_LO  = 6;
static const int  DCSR_STOPCOUNT = 10;
static const int  DCSR_STEPIE    = 11;
static const int  DCSR_EBREAKU   = 12;
static const int  DCSR_EBREAKS   = 13;
static const int  DCSR_EBREAKM   = 15;
static const uint64_t DCSR_XDEBUGVER = (0x4ULL << 28);

// cp0_yy_priv_mode values (rvproc_pkg.sv)
static const uint32_t PRIV_U = 0, PRIV_S = 1, PRIV_M = 3;

//-----------------------------------------------------------------------------
// DUT plumbing
//-----------------------------------------------------------------------------
static VDTU *dut = nullptr;
static uint64_t g_cycles = 0;

static void tie_idle_inputs(void) {
    dut->cp0_dtu_addr        = 0;
    dut->cp0_dtu_wdata       = 0;
    dut->cp0_dtu_wreg        = 0;
    dut->cp0_dtu_rreg        = 0;
    dut->cp0_yy_priv_mode    = PRIV_M;
    // DM side
    dut->tdt_dm_dtu_halt_req     = 0;
    dut->tdt_dm_dtu_resume_req   = 0;
    dut->tdt_dm_dtu_halt_on_reset = 0;
    dut->tdt_dm_dtu_ack_havereset = 1;   // drain the havereset pulse
    dut->tdt_dm_dtu_itr          = 0;
    dut->tdt_dm_dtu_itr_vld      = 0;
    dut->tdt_dm_dtu_wr_vld       = 0;
    dut->tdt_dm_dtu_wr_flg       = 0;
    dut->tdt_dm_dtu_wdata        = 0;
    // RTU side
    dut->rtu_dtu_dpc        = 0;
    dut->rtu_dtu_halt_ack   = 0;
    dut->rtu_dtu_halt_cause = 0;
    dut->rtu_dtu_retire_vld = 0;
    dut->rtu_dtu_retire_debug_expt_vld = 0;
    dut->rtu_yy_xx_dbgon    = 0;
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
// Result bookkeeping
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
    printf("[dtu_tb] %-56s %s\n", name, g_local ? "FAIL" : "PASS");
    g_local = 0;
}

//-----------------------------------------------------------------------------
// Primitives
//-----------------------------------------------------------------------------
// Combinational read of dtu_cp0_rdata off a given address (the read mux in
// DTU.v is a pure case on cp0_dtu_addr, independent of the rreg strobe).
static uint64_t dtu_read(uint32_t addr) {
    dut->cp0_dtu_addr = addr;
    dut->cp0_dtu_rreg = 1;
    dut->eval();
    uint64_t v = dut->dtu_cp0_rdata;
    dut->cp0_dtu_rreg = 0;
    return v;
}

// One cp0 write to a debug CSR. Gated inside DTU.v on rtu_yy_xx_dbgon, so the
// caller sets that first. Commits on the tick, then deasserts the strobe.
static void dtu_cp0_write(uint32_t addr, uint64_t val) {
    dut->cp0_dtu_addr  = addr;
    dut->cp0_dtu_wdata = val;
    dut->cp0_dtu_wreg  = 1;
    tick();
    dut->cp0_dtu_wreg  = 0;
}

//=============================================================================
// Tests
//=============================================================================

static void test_reset_state(void) {
    reset_dut();
    // havereset: the ack_havereset is tied 1 in tie_idle, so the pulse has
    // drained by end-of-reset; but the DTU->DM response outputs are all 0
    // at this point (no dbgon, no request).
    check(dut->dtu_tdt_dm_halted == 0, "reset: halted=0 (not in debug)");
    check(dut->dtu_rtu_int_mask == 0, "reset: int_mask=0 (no step)");
    check(dut->dtu_rtu_ebreak_action == 0, "reset: ebreak_action=0");
    check(dut->dtu_rtu_step_en == 0, "reset: step_en=0");
    check(dut->dtu_cp0_wake_up == 0, "reset: wake_up=0 (no request/step)");
    check(dut->dtu_ifu_debug_inst_vld == 0, "reset: no itr injection");
    check(dut->dtu_tdt_dm_wr_ready == 0, "reset: wr_ready quiescent");
    check(dut->dtu_rtu_sync_halt_req == 0, "reset: sync_halt passthrough=0");
    check(dut->dtu_rtu_pending_tval == 0, "reset: pending_tval tied 0 (Task 2)");
    // dcsr reset value: xdebugver=0100, everything else 0.
    check(dtu_read(CSR_DCSR) == DCSR_XDEBUGVER, "reset: dcsr == xdebugver (0x40000000)",
          dtu_read(CSR_DCSR), DCSR_XDEBUGVER);
    check(dtu_read(CSR_DPC) == 0, "reset: dpc == 0");
    check(dtu_read(CSR_DSCRATCH0) == 0, "reset: dscratch0 == 0");
    check(dtu_read(CSR_DSCRATCH1) == 0, "reset: dscratch1 == 0");
    check(dut->dtu_hpcp_dcsr_stopcount == 0, "reset: stopcount=0");
    test_result("T1 reset state: quiescent + dcsr xdebugver");
}

static void test_dcsr_rw_debug(void) {
    reset_dut();
    // In debug mode, a dcsr write commits the RW fields and the control
    // outputs track them.
    dut->rtu_yy_xx_dbgon = 1;
    dut->eval();
    // Set step=1, mprven=1, stepie=1, ebreaku/s/m=1, stopcount=1.
    uint64_t w = DCSR_XDEBUGVER
               | (1ULL << DCSR_STEP) | (1ULL << DCSR_MPRVEN)
               | (1ULL << DCSR_STOPCOUNT) | (1ULL << DCSR_STEPIE)
               | (1ULL << DCSR_EBREAKU) | (1ULL << DCSR_EBREAKS)
               | (1ULL << DCSR_EBREAKM);
    dtu_cp0_write(CSR_DCSR, w);
    check(dtu_read(CSR_DCSR) == w, "dcsr: RW fields persist", dtu_read(CSR_DCSR), w);
    check(dut->dtu_rtu_step_en == 1, "dcsr: step_en follows step");
    check(dut->dtu_cp0_dcsr_mprven == 1, "dcsr: mprven exported");
    check(dut->dtu_hpcp_dcsr_stopcount == 1, "dcsr: stopcount exported");
    // stepie=1 -> int_mask = step && !stepie = 0.
    check(dut->dtu_rtu_int_mask == 0, "dcsr: stepie=1 -> int_mask=0");

    // Clear stepie -> int_mask = 1.
    dtu_cp0_write(CSR_DCSR, (w & ~(1ULL << DCSR_STEPIE)));
    check(dut->dtu_rtu_int_mask == 1, "dcsr: stepie=0 -> int_mask=1 (step && !stepie)");
    dut->rtu_yy_xx_dbgon = 0;
    test_result("T2 dcsr RW in debug: fields persist, step/mprven/stopcount/int_mask track");
}

static void test_dcsr_write_gated_by_dbgon(void) {
    // A dcsr write OUTSIDE debug mode is dropped (donor ctrl.v:314-317 gate).
    reset_dut();
    dut->rtu_yy_xx_dbgon = 0;
    dtu_cp0_write(CSR_DCSR, DCSR_XDEBUGVER | (1ULL << DCSR_STEP));
    check(dtu_read(CSR_DCSR) == DCSR_XDEBUGVER,
          "dcsr !dbgon: write dropped, step stays 0", dtu_read(CSR_DCSR), DCSR_XDEBUGVER);
    check(dut->dtu_rtu_step_en == 0, "dcsr !dbgon: step_en stays 0");
    test_result("T3 dcsr write gated on dbgon (dropped outside debug)");
}

static void test_dcsr_prv_cause_latch(void) {
    // dcsr.prv/cause latch from the RTU at halt_ack (donor ctrl.v:353-370).
    // First set the current priv to S and a cause, then fire halt_ack.
    reset_dut();
    dut->cp0_yy_priv_mode  = PRIV_S;
    dut->rtu_dtu_halt_cause = 5;   // reset cause
    dut->rtu_dtu_halt_ack  = 1;
    tick();
    dut->rtu_dtu_halt_ack  = 0;
    dut->cp0_yy_priv_mode  = PRIV_M;
    uint64_t dcsr = dtu_read(CSR_DCSR);
    check((dcsr & 3) == PRIV_S, "dcsr.prv latched to S at halt_ack", dcsr & 3, PRIV_S);
    check(((dcsr >> DCSR_CAUSE_LO) & 7) == 5, "dcsr.cause latched to 5 at halt_ack",
          (dcsr >> DCSR_CAUSE_LO) & 7, 5);
    check(dut->dtu_cp0_dcsr_prv == PRIV_S, "dcsr.prv exported to cp0 (S)");

    // The donor ALSO allows a debug-mode dcsr write to set prv (ctrl.v:360-361)
    // -- a lower-priority write arm than the halt_ack latch. Confirm it can
    // change prv from the latched S to U.
    dut->rtu_yy_xx_dbgon = 1;
    dtu_cp0_write(CSR_DCSR, (dtu_read(CSR_DCSR) & ~3ULL) | PRIV_U);
    check((dtu_read(CSR_DCSR) & 3) == PRIV_U, "dcsr.prv writable in debug (donor write arm)",
          dtu_read(CSR_DCSR) & 3, PRIV_U);
    dut->rtu_yy_xx_dbgon = 0;
    test_result("T4 dcsr.prv/cause latch at halt_ack + debug write arm");
}

static void test_dpc(void) {
    // dpc: halt_ack latch from rtu_dtu_dpc, then a debug-mode cp0 write.
    reset_dut();
    dut->rtu_dtu_dpc = 0x1000;
    dut->rtu_dtu_halt_ack = 1;
    tick();
    dut->rtu_dtu_halt_ack = 0;
    check(dtu_read(CSR_DPC) == 0x1000, "dpc latched from rtu_dtu_dpc at halt_ack",
          dtu_read(CSR_DPC), 0x1000);
    check(dut->dtu_rtu_dpc == 0x1000, "dpc exported to rtu");

    // Debug-mode cp0 write redirects dpc.
    dut->rtu_yy_xx_dbgon = 1;
    dtu_cp0_write(CSR_DPC, 0x9000);
    check(dtu_read(CSR_DPC) == 0x9000, "dpc redirect via cp0 write in debug",
          dtu_read(CSR_DPC), 0x9000);

    // The donor does NOT auto-increment dpc on a debug-mode retire (only the
    // halt_ack latch advances it). Confirm no spurious +4.
    dut->rtu_dtu_retire_vld = 1;
    tick();
    dut->rtu_dtu_retire_vld = 0;
    check(dtu_read(CSR_DPC) == 0x9000, "dpc NOT auto-incremented on retire (clone-faithful)",
          dtu_read(CSR_DPC), 0x9000);
    dut->rtu_yy_xx_dbgon = 0;
    test_result("T5 dpc: halt_ack latch + cp0 redirect, no auto-increment");
}

static void test_dscratch(void) {
    reset_dut();
    dut->rtu_yy_xx_dbgon = 1;
    // cp0 write both scratch regs.
    dtu_cp0_write(CSR_DSCRATCH0, 0xAAAA);
    dtu_cp0_write(CSR_DSCRATCH1, 0xBBBB);
    check(dtu_read(CSR_DSCRATCH0) == 0xAAAA, "dscratch0 cp0 write", dtu_read(CSR_DSCRATCH0), 0xAAAA);
    check(dtu_read(CSR_DSCRATCH1) == 0xBBBB, "dscratch1 cp0 write", dtu_read(CSR_DSCRATCH1), 0xBBBB);

    // DM write to dscratch0 (wr_flg=01, dbgon) -- donor ctrl.v:318.
    dut->tdt_dm_dtu_wr_vld = 1;
    dut->tdt_dm_dtu_wr_flg = 1;   // 01 = write dscratch0
    dut->tdt_dm_dtu_wdata  = 0xCCCC;
    tick();
    dut->tdt_dm_dtu_wr_vld = 0;
    check(dtu_read(CSR_DSCRATCH0) == 0xCCCC, "dscratch0 DM write (wr_flg=01)",
          dtu_read(CSR_DSCRATCH0), 0xCCCC);
    check(dtu_read(CSR_DSCRATCH1) == 0xBBBB, "dscratch1 unaffected by DM dscratch0 write");

    // DM write OUTSIDE debug is dropped.
    dut->rtu_yy_xx_dbgon = 0;
    dut->tdt_dm_dtu_wr_vld = 1;
    dut->tdt_dm_dtu_wr_flg = 1;
    dut->tdt_dm_dtu_wdata  = 0x1111;
    tick();
    dut->tdt_dm_dtu_wr_vld = 0;
    check(dtu_read(CSR_DSCRATCH0) == 0xCCCC, "dscratch0 DM write dropped !dbgon");
    dut->rtu_yy_xx_dbgon = 0;
    test_result("T6 dscratch0/1: cp0 + DM writes, DM gated on dbgon");
}

static void test_ebreak_action(void) {
    // ebreak_action = per-priv ebreakX (donor ctrl.v:377-379). Set all three
    // ebreak bits, then sweep priv.
    reset_dut();
    dut->rtu_yy_xx_dbgon = 1;
    dut->eval();
    dtu_cp0_write(CSR_DCSR, DCSR_XDEBUGVER
                 | (1ULL << DCSR_EBREAKU) | (1ULL << DCSR_EBREAKS)
                 | (1ULL << DCSR_EBREAKM));
    dut->cp0_yy_priv_mode = PRIV_U;
    dut->eval();
    check(dut->dtu_rtu_ebreak_action == 1, "ebreak_action U (ebreaku set)");
    dut->cp0_yy_priv_mode = PRIV_S;
    dut->eval();
    check(dut->dtu_rtu_ebreak_action == 1, "ebreak_action S (ebreaks set)");
    dut->cp0_yy_priv_mode = PRIV_M;
    dut->eval();
    check(dut->dtu_rtu_ebreak_action == 1, "ebreak_action M (ebreakm set)");

    // Clear ebreaku -> U no longer actions.
    dtu_cp0_write(CSR_DCSR, DCSR_XDEBUGVER | (1ULL << DCSR_EBREAKS) | (1ULL << DCSR_EBREAKM));
    dut->cp0_yy_priv_mode = PRIV_U;
    dut->eval();
    check(dut->dtu_rtu_ebreak_action == 0, "ebreak_action U cleared (ebreaku=0)");
    dut->cp0_yy_priv_mode = PRIV_M;
    dut->eval();
    check(dut->dtu_rtu_ebreak_action == 1, "ebreak_action M still set");
    dut->rtu_yy_xx_dbgon = 0;
    test_result("T7 ebreak_action per-priv mapping (U/S/M)");
}

static void test_wake_up(void) {
    // wake_up = tdt_dm_dtu_halt_req || (step && !dbgon)  (D-M7-7 reduced).
    reset_dut();
    // (a) no request, no step -> 0.
    check(dut->dtu_cp0_wake_up == 0, "wake_up idle = 0");
    // (b) DM halt request -> 1 (and passes through to rtu).
    dut->tdt_dm_dtu_halt_req = 1;
    dut->eval();
    check(dut->dtu_cp0_wake_up == 1, "wake_up on DM halt_req");
    check(dut->dtu_rtu_sync_halt_req == 1, "sync_halt_req passthrough = halt_req");
    dut->tdt_dm_dtu_halt_req = 0;
    dut->eval();
    // (c) step=1 & !dbgon -> 1. (write step in debug, then drop dbgon).
    dut->rtu_yy_xx_dbgon = 1;
    dtu_cp0_write(CSR_DCSR, (1ULL << DCSR_STEP) | DCSR_XDEBUGVER);
    dut->rtu_yy_xx_dbgon = 0;
    dut->eval();
    check(dut->dtu_cp0_wake_up == 1, "wake_up on step && !dbgon");
    dut->rtu_yy_xx_dbgon = 1;
    dut->eval();
    check(dut->dtu_cp0_wake_up == 0, "wake_up cleared when dbgon (step term masked)");
    dut->rtu_yy_xx_dbgon = 0;
    test_result("T8 wake_up = DM halt_req || (step && !dbgon)");
}

static void test_itr_injection(void) {
    // itr -> dtu_ifu_debug_inst/_vld (same-cycle live passthrough in rv906).
    reset_dut();
    dut->tdt_dm_dtu_itr     = 0xDEADBEEF;
    dut->tdt_dm_dtu_itr_vld = 1;
    dut->eval();
    check(dut->dtu_ifu_debug_inst_vld == 1, "itr: debug_inst_vld live with itr_vld");
    check(dut->dtu_ifu_debug_inst == 0xDEADBEEF, "itr: debug_inst == itr payload",
          dut->dtu_ifu_debug_inst, 0xDEADBEEF);
    tick();
    dut->tdt_dm_dtu_itr_vld = 0;
    dut->eval();
    check(dut->dtu_ifu_debug_inst_vld == 0, "itr: vld deasserts when itr_vld falls");

    // itr_done: one cycle after a debug-mode retire.
    dut->rtu_yy_xx_dbgon    = 1;
    dut->rtu_dtu_retire_vld = 1;
    tick();
    dut->rtu_dtu_retire_vld = 0;
    check(dut->dtu_tdt_dm_itr_done == 1, "itr_done pulses 1 cycle after a debug retire");
    tick();
    check(dut->dtu_tdt_dm_itr_done == 0, "itr_done is a 1-cycle pulse");
    dut->rtu_yy_xx_dbgon = 0;
    test_result("T9 itr injection (debug_inst/_vld) + itr_done");
}

static void test_wr_rx_dscratch0(void) {
    // DM read dscratch0 (wr_flg=00): wr_ready pulses 1 cycle after wr_vld,
    // rx_data carries the latched dscratch0.
    reset_dut();
    dut->rtu_yy_xx_dbgon = 1;
    dtu_cp0_write(CSR_DSCRATCH0, 0x5555);
    dut->rtu_yy_xx_dbgon = 0;

    dut->tdt_dm_dtu_wr_vld = 1;
    dut->tdt_dm_dtu_wr_flg = 0;   // 00 = read dscratch0
    tick();   // latches dscratch0 into rx_data; wr_ready set at this edge
    dut->tdt_dm_dtu_wr_vld = 0;
    check(dut->dtu_tdt_dm_wr_ready == 1, "wr: wr_ready pulses 1 cycle after wr_vld");
    check(dut->dtu_tdt_dm_rx_data == 0x5555, "wr: rx_data == dscratch0 readback",
          dut->dtu_tdt_dm_rx_data, 0x5555);
    tick();
    check(dut->dtu_tdt_dm_wr_ready == 0, "wr: wr_ready is a 1-cycle pulse");

    // wr_flg 10/11 (latest_pc/satp) are DROPPED per D-M7-6 -> rx_data = 0.
    dut->tdt_dm_dtu_wr_vld = 1;
    dut->tdt_dm_dtu_wr_flg = 2;   // 10 = latest_pc (dropped)
    tick();
    dut->tdt_dm_dtu_wr_vld = 0;
    check(dut->dtu_tdt_dm_rx_data == 0, "wr: wr_flg=10 (latest_pc) rx_data=0 (D-M7-6)");
    test_result("T10 DM wr/rx: dscratch0 readback + dropped 10/11 arms");
}

static void test_halted_and_havereset(void) {
    // halted = dbgon (donor cdc:392-410).
    reset_dut();
    check(dut->dtu_tdt_dm_halted == 0, "halted=0 when !dbgon");
    dut->rtu_yy_xx_dbgon = 1;
    dut->eval();
    check(dut->dtu_tdt_dm_halted == 1, "halted=1 when dbgon");
    dut->rtu_yy_xx_dbgon = 0;

    // havereset FSM: manual reset with ack=0 held, so the pulse must assert
    // and STAY until the DM acks (tie_idle forces ack=1, so do the reset by
    // hand here to keep ack=0 through it).
    tie_idle_inputs();
    dut->tdt_dm_dtu_ack_havereset = 0;
    dut->clk = 0; dut->rst_n = 0;
    for (int i = 0; i < 5; i++) tick();
    dut->rst_n = 1;
    for (int i = 0; i < 5; i++) tick();
    check(dut->dtu_tdt_dm_havereset == 1, "havereset asserts after reset (DM not acked)");
    // Ack it: FSM -> PENDING, havereset deasserts within a couple of cycles.
    dut->tdt_dm_dtu_ack_havereset = 1;
    int dropped = 0;
    for (int i = 0; i < 4 && !dropped; i++) {
        tick();
        if (dut->dtu_tdt_dm_havereset == 0) dropped = 1;
    }
    check(dropped == 1, "havereset deasserts after ack_havereset");
    test_result("T11 halted=dbgon + havereset FSM (assert-until-ack)");
}

static void test_dm_passthrough(void) {
    // The D-M7-1 direct passthroughs: sync_halt_req, resume_req, halt_on_reset.
    reset_dut();
    dut->tdt_dm_dtu_halt_req     = 1;
    dut->tdt_dm_dtu_resume_req   = 1;
    dut->tdt_dm_dtu_halt_on_reset = 1;
    dut->eval();
    check(dut->dtu_rtu_sync_halt_req == 1, "sync_halt_req = tdt_dm halt_req");
    check(dut->dtu_rtu_resume_req == 1, "resume_req = tdt_dm resume_req");
    check(dut->dtu_ifu_halt_on_reset == 1, "halt_on_reset = tdt_dm halt_on_reset");
    dut->tdt_dm_dtu_halt_req = 0;
    dut->tdt_dm_dtu_resume_req = 0;
    dut->tdt_dm_dtu_halt_on_reset = 0;
    dut->eval();
    check(dut->dtu_rtu_sync_halt_req == 0 && dut->dtu_rtu_resume_req == 0
          && dut->dtu_ifu_halt_on_reset == 0, "passthroughs deassert with the request");
    test_result("T12 D-M7-1 DM passthroughs (sync_halt/resume/halt_on_reset)");
}

//=============================================================================
// M7 Task 2 -- trigger storage (tselect/tdata1/2/3/tinfo/tcontrol/
// mcontext/scontext). The new IFU/LSU/RTU trigger-match ports are left
// unconnected in this bench: Verilator 2-state leaves them 0, so the
// comparators see no accesses and no matches (the OFF-path identity the
// M7 gate set relies on).
//=============================================================================

// mcontrol tdata1 field encodings (standard 0.13 layout, DTU.v localparams)
static const uint64_t MTYPE    = 0x2ULL << 60;   // type = 2 (mcontrol)
static const uint64_t MT_MMODE = 0x1ULL << 6;
static const uint64_t MT_EXE   = 0x1ULL << 2;
static const uint64_t MT_LD    = 0x1ULL << 0;
static const uint64_t MT_ST    = 0x1ULL << 1;

static void test_trigger_reset_state(void) {
    reset_dut();
    check(dtu_read(CSR_TSELECT)  == 0, "trig reset: tselect == 0", dtu_read(CSR_TSELECT), 0);
    check(dtu_read(CSR_TDATA1)   == 0, "trig reset: tdata1(slot0) == 0", dtu_read(CSR_TDATA1), 0);
    check(dtu_read(CSR_TDATA2)   == 0, "trig reset: tdata2 == 0");
    check(dtu_read(CSR_TDATA3)   == 0, "trig reset: tdata3 == 0");
    check(dtu_read(CSR_TCONTROL) == 0, "trig reset: tcontrol == 0 (MTE/MPTE clear)",
          dtu_read(CSR_TCONTROL), 0);
    check(dtu_read(CSR_MCONTEXT) == 0, "trig reset: mcontext == 0");
    check(dtu_read(CSR_SCONTEXT) == 0, "trig reset: scontext == 0");
    // tinfo: slot 0 is an mcontrol slot (tinfo[9:4]=001000 -> 0x10),
    // slots 8/9 are iie (0x30).
    check(dtu_read(CSR_TINFO) == 0x10, "trig reset: tinfo(slot0) == 0x10 (mcontrol)",
          dtu_read(CSR_TINFO), 0x10);
    test_result("T13 trigger reset: all storage 0, tinfo 0x10");
}

static void test_tselect_clamp(void) {
    reset_dut();
    // 0..8 pass through; 9 and above (or any high bit) clamp to 9.
    dtu_cp0_write(CSR_TSELECT, 0x4);
    check(dtu_read(CSR_TSELECT) == 4, "tselect: 4 passes", dtu_read(CSR_TSELECT), 4);
    dtu_cp0_write(CSR_TSELECT, 0x8);
    check(dtu_read(CSR_TSELECT) == 8, "tselect: 8 (iie slot) passes", dtu_read(CSR_TSELECT), 8);
    dtu_cp0_write(CSR_TSELECT, 0x9);
    check(dtu_read(CSR_TSELECT) == 9, "tselect: 9 clamps to 9", dtu_read(CSR_TSELECT), 9);
    dtu_cp0_write(CSR_TSELECT, 0xA);
    check(dtu_read(CSR_TSELECT) == 9, "tselect: 10 clamps to 9", dtu_read(CSR_TSELECT), 9);
    dtu_cp0_write(CSR_TSELECT, 0xFFFF);
    check(dtu_read(CSR_TSELECT) == 9, "tselect: 0xFFFF clamps to 9", dtu_read(CSR_TSELECT), 9);
    // tinfo follows the selected slot: 0x30 for iie slots 8/9.
    dtu_cp0_write(CSR_TSELECT, 0x9);
    check(dtu_read(CSR_TINFO) == 0x30, "tinfo(iie slot 9) == 0x30", dtu_read(CSR_TINFO), 0x30);
    dtu_cp0_write(CSR_TSELECT, 0x0);
    test_result("T14 tselect WARL clamp at 9 + tinfo slot select");
}

static void test_tdata1_readback(void) {
    reset_dut();
    // (a) Bit-exact readback for the stock-breakpoint.S encodings
    // (2 << (XLEN-4) | M | {EXE,LD,ST}):
    // 0x2000000000000044 / 0x2000000000000041 / 0x2000000000000042.
    const uint64_t exe = MTYPE | MT_MMODE | MT_EXE;   // 0x2000000000000044
    const uint64_t ld  = MTYPE | MT_MMODE | MT_LD;    // 0x2000000000000041
    const uint64_t st  = MTYPE | MT_MMODE | MT_ST;    // 0x2000000000000042
    dtu_cp0_write(CSR_TSELECT, 0x0);
    dtu_cp0_write(CSR_TDATA1, exe);
    check(dtu_read(CSR_TDATA1) == exe, "tdata1: execute enc bit-exact",
          dtu_read(CSR_TDATA1), exe);
    dtu_cp0_write(CSR_TDATA1, ld);
    check(dtu_read(CSR_TDATA1) == ld, "tdata1: load enc bit-exact", dtu_read(CSR_TDATA1), ld);
    dtu_cp0_write(CSR_TDATA1, st);
    check(dtu_read(CSR_TDATA1) == st, "tdata1: store enc bit-exact", dtu_read(CSR_TDATA1), st);

    // (b) A second slot is independent.
    dtu_cp0_write(CSR_TSELECT, 0x1);
    dtu_cp0_write(CSR_TDATA1, st);
    check(dtu_read(CSR_TDATA1) == st, "tdata1: slot1 stores independently", dtu_read(CSR_TDATA1), st);
    dtu_cp0_write(CSR_TSELECT, 0x0);
    check(dtu_read(CSR_TDATA1) == st, "tdata1: slot0 unchanged by slot1 write",
          dtu_read(CSR_TDATA1), st);

    // (c) WARL: unsupported type (4) -> type field reads back 0 (slot
    // disabled; a match requires type==2, D-M7-5).
    dtu_cp0_write(CSR_TDATA1, (0x4ULL << 60) | MT_MMODE | MT_EXE);
    check((dtu_read(CSR_TDATA1) >> 60) == 0,
          "tdata1: type 4 unsupported -> type field reads 0",
          dtu_read(CSR_TDATA1) >> 60, 0);
    dtu_cp0_write(CSR_TDATA1, (0x5ULL << 60) | MT_MMODE | MT_EXE);
    check((dtu_read(CSR_TDATA1) >> 60) == 0,
          "tdata1: type 5 unsupported -> type field reads 0",
          dtu_read(CSR_TDATA1) >> 60, 0);

    // (d) WARL: match > 5 clamps to 0; action > 1 clamps to 0; timing is
    // forced 0 for an execute trigger.
    dtu_cp0_write(CSR_TDATA1, MTYPE | MT_MMODE | MT_EXE | (0x9ULL << 7));
    check(((dtu_read(CSR_TDATA1) >> 7) & 0xFULL) == 0, "tdata1: match 9 clamps to 0",
          (dtu_read(CSR_TDATA1) >> 7) & 0xFULL, 0);
    dtu_cp0_write(CSR_TDATA1, MTYPE | MT_MMODE | MT_EXE | (0x3ULL << 12));
    check(((dtu_read(CSR_TDATA1) >> 12) & 0x3FULL) == 0, "tdata1: action 3 clamps to 0",
          (dtu_read(CSR_TDATA1) >> 12) & 0x3FULL, 0);
    dtu_cp0_write(CSR_TDATA1, MTYPE | MT_MMODE | MT_EXE | (1ULL << 18));
    check(((dtu_read(CSR_TDATA1) >> 18) & 1ULL) == 0, "tdata1: timing forced 0 for execute",
          (dtu_read(CSR_TDATA1) >> 18) & 1ULL, 0);
    // but timing IS kept for a load trigger.
    dtu_cp0_write(CSR_TDATA1, MTYPE | MT_MMODE | MT_LD | (1ULL << 18));
    check(((dtu_read(CSR_TDATA1) >> 18) & 1ULL) == 1, "tdata1: timing kept for load",
          (dtu_read(CSR_TDATA1) >> 18) & 1ULL, 1);

    // (e) tdata2/tdata3 plain storage.
    dtu_cp0_write(CSR_TDATA2, 0x80000040);
    check(dtu_read(CSR_TDATA2) == 0x80000040, "tdata2 plain storage", dtu_read(CSR_TDATA2), 0x80000040);
    dtu_cp0_write(CSR_TDATA3, 0xDEADBEEF);
    check(dtu_read(CSR_TDATA3) == 0xDEADBEEF, "tdata3 plain storage", dtu_read(CSR_TDATA3), 0xDEADBEEF);
    dtu_cp0_write(CSR_TSELECT, 0x0);
    test_result("T15 tdata1 bit-exact + WARL (type/match/action/timing) + tdata2/3");
}

static void test_tcontrol_mcontext_scontext(void) {
    reset_dut();
    // tcontrol: MTE (bit 3) + MPTE (bit 7) stored and read back.
    dtu_cp0_write(CSR_TCONTROL, (1 << 3));
    check(dtu_read(CSR_TCONTROL) == (1 << 3), "tcontrol: MTE stored",
          dtu_read(CSR_TCONTROL), (1 << 3));
    dtu_cp0_write(CSR_TCONTROL, (1 << 7));
    check(dtu_read(CSR_TCONTROL) == (1 << 7), "tcontrol: MPTE stored",
          dtu_read(CSR_TCONTROL), (1 << 7));
    dtu_cp0_write(CSR_TCONTROL, 0);

    // mcontext: low 13 bits plain storage.
    dtu_cp0_write(CSR_MCONTEXT, 0x1FFF);
    check(dtu_read(CSR_MCONTEXT) == 0x1FFF, "mcontext: low 13b stored",
          dtu_read(CSR_MCONTEXT), 0x1FFF);
    dtu_cp0_write(CSR_MCONTEXT, 0x12345);
    check(dtu_read(CSR_MCONTEXT) == (0x12345 & 0x1FFF), "mcontext: high bits dropped",
          dtu_read(CSR_MCONTEXT), 0x12345 & 0x1FFF);

    // scontext: low 34 bits plain storage.
    dtu_cp0_write(CSR_SCONTEXT, 0x3FFFFFFF);
    check(dtu_read(CSR_SCONTEXT) == 0x3FFFFFFF, "scontext: low 34b stored",
          dtu_read(CSR_SCONTEXT), 0x3FFFFFFF);
    dtu_cp0_write(CSR_SCONTEXT, 0xFFFFFFFFFFFF);
    check(dtu_read(CSR_SCONTEXT) == (0xFFFFFFFFFFFFULL & 0x3FFFFFFFFULL),
          "scontext: high bits dropped", dtu_read(CSR_SCONTEXT), 0xFFFFFFFFFFFFULL & 0x3FFFFFFFFULL);
    test_result("T16 tcontrol MTE/MPTE + mcontext(13b)/scontext(34b) storage");
}

//=============================================================================
// main
//=============================================================================
int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    dut = new VDTU;

    reset_dut();

    test_reset_state();
    test_dcsr_rw_debug();
    test_dcsr_write_gated_by_dbgon();
    test_dcsr_prv_cause_latch();
    test_dpc();
    test_dscratch();
    test_ebreak_action();
    test_wake_up();
    test_itr_injection();
    test_wr_rx_dscratch0();
    test_halted_and_havereset();
    test_dm_passthrough();
    test_trigger_reset_state();
    test_tselect_clamp();
    test_tdata1_readback();
    test_tcontrol_mcontext_scontext();

    printf("[dtu_tb] %llu cycles, %d failure(s)\n",
           (unsigned long long)g_cycles, g_fail);
    printf("%s\n", g_fail ? "UNIT-FAIL" : "UNIT-PASS");

    dut->final();
    delete dut;
    return g_fail ? 1 : 0;
}
