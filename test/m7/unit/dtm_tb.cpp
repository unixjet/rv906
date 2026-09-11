//=============================================================================
// dtm_tb.cpp - standalone unit bench for rtl/TDT_DTM.v (M7 Task 4)
//=============================================================================
// Verilates TDT_DTM.v alone (TAP-5 + IDCODE/DTMCS/DMI/BYPASS TDRs + the
// DMI-to-APB bridge with the tck<->pclk pulse syncs) and drives the JTAG
// pads (tck/tms/tdi) from C++ with an event-driven tck, plus a fake APB
// slave on the apbm_* master port. Same row/print/exit-code idiom as
// test/m2/unit/csr_tb.cpp.
//
// Two clocks (test strategy note 2026-09-11-m7-debug-test-strategy.md S2a):
//   - pclk  : free-running, toggled every bench tick ("fast pclk" -- the
//             7-idle-TCK budget then always satisfies the donor's
//             Freq.pclk/Freq.tck > 8/3 constraint, tdt_dmi_define.h:309).
//   - tck   : event-driven via tck_edge(): set TMS/TDI, eval, tck=1, eval,
//             tck=0, eval, then sample TDO (TDO is a negedge-registered
//             output, tdt_dtm_chain.v:109-122, so the value after the
//             tck=0 eval is the bit the next posedge will shift out --
//             exactly what JTAG_DRV.vh's shift_dr samples on posedge).
// Every tck_edge() interleaves pclk ticks so the 4-FF pulse syncs always
// have dst-clock edges to cross (Verilated comb-eval never crosses a sync
// unless the destination clock actually steps).
//
// TDD row order: TAP/DR rows (T1-T3, T7, T9, T10) first, then the DMI
// bridge rows (T4-T6, T8).
//
// Build/run: make -C test/m7/unit dtm && bin/unit/dtm_tb
// Prints one line per test and ends with UNIT-PASS or UNIT-FAIL.
//=============================================================================

#include <verilated.h>
#include "VTDT_DTM.h"

#include <cstdio>
#include <cstdint>

//-----------------------------------------------------------------------------
// JTAG constants (donor tdt_dtm_chain.v:42-45, tdt_dtm_idr.v:45-47)
//-----------------------------------------------------------------------------
static const uint8_t  IR_IDCODE  = 0x01;
static const uint8_t  IR_DMI_ACC = 0x02;
static const uint8_t  IR_DTMCS   = 0x10;
static const uint8_t  IR_DMI     = 0x11;
static const uint8_t  IR_UNKNOWN = 0x1F;   // any unlisted opcode -> 1-bit bypass

static const uint32_t IDCODE_VAL = 0x10000B6Fu;  // tdt_dtm_idr.v:45
static const int      DMI_DR_LEN = 44;           // {addr[9:0], data[31:0], op[1:0]}

// DMI op field (tdt_dtm_idr.v:152: ^op == read|write)
static const uint8_t  DMI_OP_NOP   = 0x0;
static const uint8_t  DMI_OP_READ  = 0x1;
static const uint8_t  DMI_OP_WRITE = 0x2;

//-----------------------------------------------------------------------------
// DUT plumbing
//-----------------------------------------------------------------------------
static VTDT_DTM *dut = nullptr;
static uint64_t g_ticks = 0;

//-----------------------------------------------------------------------------
// Fake APB slave (C++): responds on the access phase (psel&penable) with a
// configurable pready/prdata; records the last transaction and counts them.
//-----------------------------------------------------------------------------
struct FakeApb {
    bool     hold_pready_low = false;  // T8: stall the access phase
    uint32_t prdata          = 0;
    unsigned txn_count       = 0;
    bool     last_pwrite     = false;
    uint32_t last_paddr      = 0;
    uint32_t last_pwdata     = 0;
    // Debug observability of the APB phase (not a DUT port -- the bench
    // watches the DUT's own apbm_* outputs, which IS the contract surface).
    bool     saw_setup       = false;
    bool     saw_access      = false;
};
static FakeApb apb;

// Drive the slave side of the APB from the DUT's current master outputs,
// then eval so pready/prdata settle combinationally for this state. A
// transaction is counted ONCE per access phase, on the first eval where the
// donor's completion condition (psel&penable&pready, tdt_apb_master.v:216)
// holds -- the level can persist across several evals within one pclk
// cycle, so we edge-detect it.
static void apb_slave_respond(void) {
    static bool prev_completion = false;
    bool in_access = dut->apbm_psel && dut->apbm_penable;
    if (in_access) {
        apb.saw_access = true;
        dut->apbm_pready = apb.hold_pready_low ? 0 : 1;
        dut->apbm_prdata = apb.prdata;
    } else {
        dut->apbm_pready = 0;
        dut->apbm_prdata = 0;
    }
    dut->apbm_pslverr = 0;
    bool completion = in_access && dut->apbm_pready;
    if (completion && !prev_completion) {
        apb.txn_count++;
        apb.last_pwrite = dut->apbm_pwrite != 0;
        apb.last_paddr  = dut->apbm_paddr;
        apb.last_pwdata = dut->apbm_pwdata;
    }
    prev_completion = completion;
    if (dut->apbm_psel && !dut->apbm_penable) apb.saw_setup = true;
}

//-----------------------------------------------------------------------------
// Clock primitives
//-----------------------------------------------------------------------------
// One full pclk cycle with the APB slave combinationally answering.
static void pclk_tick(void) {
    dut->eval();
    apb_slave_respond();
    dut->pclk = 1;
    dut->eval();          // pclk posedge: bridge FSM/syncs commit
    apb_slave_respond();
    dut->pclk = 0;
    dut->eval();
    apb_slave_respond();
    g_ticks++;
}

// One full tck cycle: TMS/TDI were set by the caller BEFORE the call (they
// are sampled at the posedge). Interleaves pclk so the pulse syncs cross.
// Returns the TDO value after the falling edge (the bit that was presented
// for THIS shift cell -- TDO is negedge-registered, so the post-fall value
// is what a posedge sampler would see during the NEXT tck high phase; see
// the header note).
static int tck_edge(void) {
    pclk_tick();          // let pclk-domain state settle before the tck edge
    dut->eval();
    dut->tck = 1;
    dut->eval();          // tck posedge: TAP FSM, shifter, TDRs commit
    pclk_tick();
    dut->tck = 0;
    dut->eval();          // tck negedge: TDO register commits
    pclk_tick();
    return dut->tdo;
}

//-----------------------------------------------------------------------------
// TAP navigation helpers (all start/end in Run-Test/Idle unless noted)
//-----------------------------------------------------------------------------
// From IDLE: 5+ cycles of TMS=1 forces Test-Logic-Reset from any state.
static void goto_tlr(void) {
    dut->tms = 1; dut->tdi = 0;
    for (int i = 0; i < 6; i++) tck_edge();
}
// TLR -> IDLE.
static void tlr_to_idle(void) {
    dut->tms = 0; tck_edge();
}
// IDLE -> IDLE (one idle TCK cycle).
static void idle_cycle(int n = 1) {
    dut->tms = 0; dut->tdi = 0;
    for (int i = 0; i < n; i++) tck_edge();
}

// Shift a 5-bit IR, LSB first, leave the TAP in IDLE.
// IDLE -> Select-DR -> Select-IR -> Capture-IR -> Shift-IR -> 5 cells ->
// Exit1-IR -> Update-IR -> IDLE (JTAG_DRV.vh:628-649 verbatim pattern).
// TDO note (see shift_dr): TDO is negedge-registered, so the bit for cell i
// is read from the CURRENT tdo BEFORE clocking cell i. The IR path's own
// shift-out is not consumed by any test here (T9 reads it via shift_dr-style
// code), so this helper only drives TDI.
static void shift_ir(uint8_t ir) {
    dut->tms = 1; dut->tdi = 0; tck_edge();   // -> Select-DR-Scan
    dut->tms = 1;               tck_edge();   // -> Select-IR-Scan
    dut->tms = 0;               tck_edge();   // -> Capture-IR
    dut->tms = 0;               tck_edge();   // -> Shift-IR
    for (int i = 0; i < 4; i++) {
        dut->tms = 0;
        dut->tdi = (ir >> i) & 1;
        tck_edge();                            // shift bits 0..3, stay Shift-IR
    }
    dut->tms = 1;
    dut->tdi = (ir >> 4) & 1;
    tck_edge();                                // bit 4, -> Exit1-IR
    dut->tms = 1;               tck_edge();   // -> Update-IR
    dut->tms = 0;               tck_edge();   // -> IDLE
}

// Shift `len` DR bits, LSB first, simultaneously capturing TDO. Caller is in
// IDLE; returns to IDLE afterwards. In-bits from `in` (LSB first), out-bits
// returned (LSB first).
//
// TDO TIMING (the load-bearing convention of this bench): TDO is a
// negedge-tck register (tdt_dtm_chain.v:109-122) loaded from shifter[0]
// while in SHIFT_DR. The edge that moves the TAP Capture-DR -> Shift-DR
// loads TDO = captured bit 0 at its negedge (no shift happened yet -- the
// shifter only shifts on posedges where cur_st == SHIFT_DR). So after the
// entry edge, TDO already shows bit 0; cell i's output bit is the CURRENT
// tdo BEFORE the cell-i posedge, and cell i's input bit is clocked in AT
// that same posedge. We therefore read dut->tdo first, then drive TDI/TMS
// and clock. This matches JTAG_DRV.vh:706-716, which samples TDO on the
// posedge after driving the inputs at the preceding negedge.
static uint64_t shift_dr(int len, uint64_t in) {
    uint64_t out = 0;
    dut->tms = 1; dut->tdi = 0; tck_edge();   // -> Select-DR-Scan
    dut->tms = 0;               tck_edge();   // -> Capture-DR (loads DR)
    dut->tms = 0;               tck_edge();   // -> Shift-DR; TDO now = bit 0
    for (int i = 0; i < len; i++) {
        if (dut->tdo) out |= (1ULL << i);      // sample bit i (pre-edge)
        dut->tms = (i == len - 1) ? 1 : 0;     // exit after the last cell
        dut->tdi = (in >> i) & 1;
        tck_edge();                            // clock cell i
    }
    dut->tms = 1; dut->tdi = 0; tck_edge();   // -> Update-DR
    dut->tms = 0;               tck_edge();   // -> IDLE
    return out;
}

// Read-only DR scan (TDI held 0).
static uint64_t shift_dr_out(int len) { return shift_dr(len, 0); }

// The 7-idle-TCK budget between DMI ops (dtmcs.idle=7, tdt_dmi_define.h:309).
// With the bench's fast pclk (3 pclk ticks per tck edge) the pulse syncs
// always complete well inside this; the donor contract is what makes the
// scan-out of the NEXT DMI op carry the result of the PREVIOUS one.
static void dmi_idle_budget(void) { idle_cycle(7); }

//-----------------------------------------------------------------------------
// Result bookkeeping (mirrors test/m2/unit/csr_tb.cpp)
//-----------------------------------------------------------------------------
static int g_fail  = 0;
static int g_local = 0;

static void check(bool cond, const char *what, uint64_t got = 0, uint64_t exp = 0) {
    if (!cond) {
        g_local++;
        if (g_fail < 40)
            printf("    FAIL %-60s got=0x%llx exp=0x%llx (tick %llu)\n", what,
                   (unsigned long long)got, (unsigned long long)exp,
                   (unsigned long long)g_ticks);
        g_fail++;
    }
}

static void test_result(const char *name) {
    printf("[dtm_tb] %-60s %s\n", name, g_local ? "FAIL" : "PASS");
    g_local = 0;
}

//-----------------------------------------------------------------------------
// Reset: preset_n (pclk domain) + a TLR sequence (tck domain; D-M7-2: no
// trst_n, the TAP resets via 5x TMS=1).
//-----------------------------------------------------------------------------
static void reset_dut(void) {
    dut->tck = 0; dut->tms = 0; dut->tdi = 0;
    dut->pclk = 0; dut->preset_n = 0;
    dut->apbm_pready = 0; dut->apbm_prdata = 0; dut->apbm_pslverr = 0;
    for (int i = 0; i < 5; i++) pclk_tick();
    dut->preset_n = 1;
    for (int i = 0; i < 5; i++) pclk_tick();
    goto_tlr();          // TAP into Test-Logic-Reset
    tlr_to_idle();       // -> Run-Test/Idle
}

//=============================================================================
// TDD Phase 1 -- TAP / DR rows (no APB traffic)
//=============================================================================

// T1: TLR then a plain IDCODE scan -- proves the 5xTMS=1 reset + basic walk.
static void test_t1_tlr_then_scan(void) {
    goto_tlr();                     // 6x TMS=1 from anywhere
    tlr_to_idle();
    shift_ir(IR_IDCODE);
    uint64_t idcode = shift_dr_out(32);
    check(idcode == IDCODE_VAL, "T1: TAP still works after TLR (IDCODE scan)",
          idcode, IDCODE_VAL);
    test_result("T1 TLR (6x TMS=1) -> IDLE -> IDCODE scan works");
}

// T2: IDCODE = 0x1000_0B6F (tdt_dtm_idr.v:45).
static void test_t2_idcode(void) {
    shift_ir(IR_IDCODE);
    uint64_t idcode = shift_dr_out(32);
    check((idcode & 0xFFFFFFFFULL) == IDCODE_VAL,
          "T2: IDCODE == 0x10000B6F (version 1, partnum 0, JEP106 0xB6F)",
          idcode & 0xFFFFFFFFULL, IDCODE_VAL);
    test_result("T2 IDCODE DR reads 0x10000B6F");
}

// T3: DTMCS = {idle[14:12]=7, dmistat[11:10]=0, abits[9:4]=10, version[3:0]=1}.
static void test_t3_dtmcs(void) {
    shift_ir(IR_DTMCS);
    uint64_t dtmcs = shift_dr_out(32) & 0xFFFFFFFFULL;
    check((dtmcs & 0xF) == 1,        "T3: dtmcs.version == 1 (spec 0.13)", dtmcs & 0xF, 1);
    check(((dtmcs >> 4) & 0x3F) == 10, "T3: dtmcs.abits == 10", (dtmcs >> 4) & 0x3F, 10);
    check(((dtmcs >> 10) & 0x3) == 0,  "T3: dtmcs.dmistat == 0 (no error/busy)", (dtmcs >> 10) & 0x3, 0);
    check(((dtmcs >> 12) & 0x7) == 7,  "T3: dtmcs.idle == 7", (dtmcs >> 12) & 0x7, 7);
    test_result("T3 DTMCS: version=1, abits=10, dmistat=0, idle=7");
}

// T7: DMI_ACC (IR=02) is a 1-bit bypass register (tdt_dtm_chain.v:75), as
// is any unknown IR (:82). Shift 1 bit out, 1 bit in, then read it back.
static void test_t7_dmi_acc_bypass(void) {
    shift_ir(IR_DMI_ACC);
    uint64_t b0 = shift_dr(1, 0);       // shift 1 bit out (mode, reset=0)
    check(b0 == 0, "T7: DMI_ACC bypass bit reads 0 at reset", b0, 0);
    shift_ir(IR_DMI_ACC);
    shift_dr(1, 1);                     // shift a 1 in (UPDATE_DR sets mode=1)
    shift_ir(IR_DMI_ACC);
    uint64_t b1 = shift_dr(1, 0);       // shift it back out
    check(b1 == 1, "T7: DMI_ACC bypass bit shifted in and back out (mode=1)", b1, 1);
    // Restore mode=0 so later DMI scans use the full 44-bit shift (the
    // donor's idr_dmi_mode selects a 34-bit shift when mode=1).
    shift_ir(IR_DMI_ACC);
    shift_dr(1, 0);

    // Unknown IR -> 1-bit bypass (default arm, tdt_dtm_chain.v:82). The
    // bypass register has no storage outside the shift chain and Capture-DR
    // loads all-zeros for an unlisted IR (tdt_dtm_idr.v:206), so the bypass
    // property is observable WITHIN one scan: cell 0 shifts out the captured
    // 0, cell 1 shifts out the TDI bit injected at cell 0.
    shift_ir(IR_UNKNOWN);
    uint64_t ub = shift_dr(2, 0x2);          // in: bit0=0, bit1=1; out: [captured0, in-bit0... ]
    check((ub & 0x1) == 0, "T7: unknown IR (0x1F) bypass cell 0 shifts out captured 0",
          ub & 0x1, 0);
    check(((ub >> 1) & 0x1) == 0, "T7: unknown IR bypass cell 1 shifts out cell-0 TDI (0)",
          (ub >> 1) & 0x1, 0);
    shift_ir(IR_UNKNOWN);
    uint64_t ub2 = shift_dr(2, 0x1);         // in: bit0=1 -> must appear at cell 1
    check(((ub2 >> 1) & 0x1) == 1, "T7: unknown IR bypass cell 1 shifts out cell-0 TDI (1)",
          (ub2 >> 1) & 0x1, 1);
    test_result("T7 DMI_ACC + unknown IR: 1-bit bypass chains");
}

// T9: TAP state spot-checks, driven as a DIRECTED WALK with observable
// side effects per state (the TAP state itself is not a port -- each arm
// below proves the FSM took the named transition by what the TDRs/TDO do).
static void test_t9_tap_walk(void) {
    goto_tlr();
    tlr_to_idle();

    // --- DR path: IDLE -(0)-> IDLE -(1)-> Select-DR -(0)-> Capture-DR
    //     -(0)-> Shift-DR -(1)-> Exit1-DR -(1)-> Update-DR -(0)-> IDLE ---
    // Preload IDCODE so Capture-DR loads a known pattern, then walk with a
    // PAUSE detour: Capture -> Shift -> Exit1 -> Pause -> Exit2 -> Update.
    shift_ir(IR_IDCODE);
    dut->tms = 1; tck_edge();               // IDLE -> Select-DR-Scan
    dut->tms = 0; tck_edge();               // -> Capture-DR (IDCODE loaded)
    dut->tms = 0; tck_edge();               // -> Shift-DR; TDO now = bit 0
    uint64_t walk_out = 0;
    // shift 4 bits with TMS=0 (stay Shift-DR), watching LSB-first IDCODE;
    // sample TDO BEFORE each clock (negedge-registered, see shift_dr).
    for (int i = 0; i < 4; i++) {
        if (dut->tdo) walk_out |= (1ULL << i);
        dut->tms = 0; dut->tdi = 0;
        tck_edge();
    }
    check(walk_out == (IDCODE_VAL & 0xF),
          "T9: IDLE->SelDR->Capture->Shift: 4 LSBs of IDCODE shifted out LSB-first",
          walk_out, IDCODE_VAL & 0xF);
    dut->tms = 1; dut->tdi = 0; tck_edge(); // -> Exit1-DR (5th bit shifts too)
    dut->tms = 0;               tck_edge(); // -> Pause-DR (NOT Update)
    dut->tms = 1;               tck_edge(); // -> Exit2-DR
    dut->tms = 1;               tck_edge(); // -> Update-DR
    dut->tms = 0;               tck_edge(); // -> IDLE
    // The pause detour must NOT have corrupted the shifter: a fresh IDCODE
    // scan still reads the constant (Capture reloads it).
    shift_ir(IR_IDCODE);
    check(shift_dr_out(32) == IDCODE_VAL,
          "T9: Exit1->Pause->Exit2->Update detour leaves the TAP functional");

    // --- TMS=0 stays in IDLE (self-loop) ---
    int tdo_idle = 0;
    for (int i = 0; i < 3; i++) { dut->tms = 0; dut->tdi = 0; tdo_idle |= tck_edge() << i; }
    // Still in IDLE: an immediately-following IR shift must work from IDLE.
    shift_ir(IR_IDCODE);
    check(shift_dr_out(32) == IDCODE_VAL,
          "T9: TMS=0 self-loops in Run-Test/Idle (subsequent walk still works)");

    // --- IR path: IDLE -> Select-DR -> Select-IR -> Capture-IR ->
    //     Shift-IR -> Exit1-IR -> Pause-IR -> Exit2-IR -> Update-IR -> IDLE
    // Capture-IR loads the 5-bit IR register into the shifter (donor
    // tdt_dtm_chain.v:71-72); after T1/T2/etc the IR holds IDCODE=0x01.
    shift_ir(IR_IDCODE);
    dut->tms = 1; tck_edge();               // -> Select-DR-Scan
    dut->tms = 1; tck_edge();               // -> Select-IR-Scan
    dut->tms = 0; tck_edge();               // -> Capture-IR (IR value loaded)
    dut->tms = 0; tck_edge();               // -> Shift-IR; TDO now = IR bit 0
    uint64_t ir_out = 0;
    for (int i = 0; i < 4; i++) {
        if (dut->tdo) ir_out |= (1ULL << i);
        dut->tms = 0; dut->tdi = 0;
        tck_edge();
    }
    check(ir_out == (IR_IDCODE & 0xF),
          "T9: IR path Capture-IR->Shift-IR shifts the current IR (0x01) LSB-first",
          ir_out, IR_IDCODE & 0xF);
    dut->tms = 1; dut->tdi = 0; tck_edge(); // 5th IR bit -> Exit1-IR
    dut->tms = 0;               tck_edge(); // -> Pause-IR
    dut->tms = 1;               tck_edge(); // -> Exit2-IR
    dut->tms = 1;               tck_edge(); // -> Update-IR
    dut->tms = 0;               tck_edge(); // -> IDLE
    test_result("T9 TAP walk: DR path w/ Pause-DR detour, IDLE self-loop, IR path w/ Pause-IR");
}

// T10: with TMS=0 in Run-Test/Idle, tdo == 1 and stable across tck edges
// (the OFF-path JTAG-idle check: donor tdt_dtm_chain.v:120-121 drives 1
// whenever the TAP is not shifting).
static void test_t10_tdo_idle_stable(void) {
    goto_tlr();
    tlr_to_idle();
    bool all_one = true;
    for (int i = 0; i < 10; i++) {
        dut->tms = 0; dut->tdi = 0;
        if (!tck_edge()) all_one = false;
    }
    check(all_one, "T10: tdo==1 across 10 idle tck edges (Run-Test/Idle)");
    // Also in TLR itself.
    dut->tms = 1;
    for (int i = 0; i < 6; i++) { if (!tck_edge()) all_one = false; }
    check(all_one, "T10: tdo==1 across Test-Logic-Reset too");
    tlr_to_idle();
    test_result("T10 TDO idle value 1, stable in IDLE and TLR");
}

//=============================================================================
// TDD Phase 2 -- DMI bridge rows
//=============================================================================

// Compose a 44-bit DMI scan-in word: {addr[9:0], data[31:0], op[1:0]}
// (tdt_dtm_idr.v:115-143; op is the LSB pair, shifted first).
static uint64_t dmi_word(uint16_t addr, uint32_t data, uint8_t op) {
    return ((uint64_t)(addr & 0x3FF) << 34) | ((uint64_t)data << 2) | (op & 0x3);
}

// One full DMI operation with the donor's idle budget afterwards so the
// CDC handshake completes before the next scan (JTAG_DRV.vh:1084-1088's
// rwDTMReg passes idle_cycle_num=dtmcs.idle into shift_dr).
static uint64_t dmi_scan(uint16_t addr, uint32_t data, uint8_t op) {
    shift_ir(IR_DMI);
    uint64_t out = shift_dr(DMI_DR_LEN, dmi_word(addr, data, op));
    dmi_idle_budget();
    return out;
}

// T5: DMI read -- the first scan issues the request; the SECOND scan (a
// nop poll, JTAG_DRV.vh:1121-1136's busy-poll shape) returns the data.
static void test_t5_dmi_read(void) {
    apb.prdata = 0xDEADBEEF;
    apb.hold_pready_low = false;
    unsigned before = apb.txn_count;

    uint64_t out1 = dmi_scan(0x10, 0, DMI_OP_READ);   // issue the read
    // The APB transaction must have happened: pwrite=0, paddr=0x10<<2=0x40.
    check(apb.txn_count == before + 1, "T5: exactly one APB transaction for the read",
          apb.txn_count, before + 1);
    check(apb.saw_setup && apb.saw_access, "T5: APB went through SETUP and ACCESS phases");
    check(!apb.last_pwrite, "T5: pwrite==0 (read)", apb.last_pwrite, 0);
    check(apb.last_paddr == 0x40, "T5: paddr == dmi_addr<<2 == 0x40",
          apb.last_paddr, 0x40);
    // First scan-out: op field should read 0 (success) once the idle budget
    // let the response land (donor: capture happens with op_stat, :182).
    check((out1 & 0x3) == 0, "T5: scan-out op field == 0 (previous-op success)",
          out1 & 0x3, 0);
    // The returned read data is in the data field of the NEXT scan
    // (data <= rdata at ready, tdt_dtm_idr.v:131-133; a poll scan captures it).
    uint64_t out2 = dmi_scan(0x10, 0, DMI_OP_NOP);    // poll: no new op
    uint32_t rdata = (uint32_t)((out2 >> 2) & 0xFFFFFFFFULL);
    check(rdata == 0xDEADBEEF, "T5: poll scan-out data field == 0xDEADBEEF",
          rdata, 0xDEADBEEF);
    check((out2 & 0x3) == 0, "T5: poll scan-out op == 0 (success)", out2 & 0x3, 0);
    test_result("T5 DMI read: pwrite=0, paddr=0x40, data field returns 0xDEADBEEF");
}

// T6: DMI write -- pwrite=1, paddr/pwdata match.
static void test_t6_dmi_write(void) {
    unsigned before = apb.txn_count;
    uint64_t out1 = dmi_scan(0x04, 0x12345678, DMI_OP_WRITE);
    check(apb.txn_count == before + 1, "T6: exactly one APB transaction for the write",
          apb.txn_count, before + 1);
    check(apb.last_pwrite, "T6: pwrite==1 (write)", apb.last_pwrite, 1);
    check(apb.last_paddr == 0x10, "T6: paddr == 0x04<<2 == 0x10", apb.last_paddr, 0x10);
    check(apb.last_pwdata == 0x12345678, "T6: pwdata == 0x12345678",
          apb.last_pwdata, 0x12345678);
    check((out1 & 0x3) == 0, "T6: scan-out op field == 0 (success)", out1 & 0x3, 0);
    test_result("T6 DMI write: pwrite=1, paddr=0x10, pwdata=0x12345678");
}

// T8: in-flight protection -- with pready held low, a second DMI op scanned
// through the TAP must NOT issue a second APB transaction (donor
// tdt_dtm_idr.v:145-166: no wr_vld while dmi_req_running), and the mid-
// flight capture reports op==2'b11 (busy).
static void test_t8_inflight_busy(void) {
    apb.hold_pready_low = true;            // stall the access phase
    unsigned before = apb.txn_count;

    // Issue a read; the APB transaction starts but never completes.
    shift_ir(IR_DMI);
    shift_dr(DMI_DR_LEN, dmi_word(0x20, 0, DMI_OP_READ));
    idle_cycle(4);                          // let the request cross to pclk
    // The bridge is now stuck in APB_ACCESS with pready=0.
    check(apb.txn_count == before, "T8: stalled access: no completed transaction yet",
          apb.txn_count, before);

    // Scan a SECOND DMI op while the first is in flight: the capture must
    // report busy (op==2'b11, donor :168,:182) and no new request may fire.
    shift_ir(IR_DMI);
    uint64_t busy_out = shift_dr(DMI_DR_LEN, dmi_word(0x24, 0xA5A5A5A5, DMI_OP_WRITE));
    idle_cycle(4);
    check((busy_out & 0x3) == 0x3, "T8: mid-flight DMI capture reports op==2'b11 (busy)",
          busy_out & 0x3, 0x3);
    check(apb.txn_count == before,
          "T8: no second APB transaction while the first is stalled",
          apb.txn_count, before);
    // The stalled transaction is still the FIRST one (paddr unchanged).
    check(dut->apbm_paddr == 0x80, "T8: APB still presenting the FIRST op (paddr=0x20<<2=0x80)",
          dut->apbm_paddr, 0x80);

    // Release pready: the first op completes with exactly one transaction.
    apb.hold_pready_low = false;
    apb.prdata = 0xCAFEBABE;
    dmi_idle_budget();                      // ready crosses back to tck
    check(apb.txn_count == before + 1, "T8: after release, exactly ONE transaction completed",
          apb.txn_count, before + 1);
    check(!apb.last_pwrite && apb.last_paddr == 0x80,
          "T8: the completed transaction is the original read (0x80)");
    test_result("T8 in-flight: busy op==11 on capture, no 2nd APB txn, 1st completes on release");
}

// T4: dmireset (dtmcs bit 16) clears the DMI request engine: after a busy
// (op_stat==11 from T8's mid-flight capture... but T8 ended clean), force
// a busy state here and recover via dmireset. Donor :95-104,:175-176.
static void test_t4_dmireset(void) {
    // Re-create a busy op_stat: stall an op, capture mid-flight (->11),
    // then complete the APB side so dmi_req_running clears but op_stat
    // stays 11 (donor :170-180: only dmireset/dmihardreset clear op_stat).
    apb.hold_pready_low = true;
    shift_ir(IR_DMI);
    shift_dr(DMI_DR_LEN, dmi_word(0x30, 0, DMI_OP_READ));
    idle_cycle(4);
    shift_ir(IR_DMI);
    uint64_t busy_out = shift_dr(DMI_DR_LEN, dmi_word(0, 0, DMI_OP_NOP));
    check((busy_out & 0x3) == 0x3, "T4: setup: mid-flight capture -> op_stat busy (11)",
          busy_out & 0x3, 0x3);
    apb.hold_pready_low = false;
    dmi_idle_budget();                       // first op completes

    // dtmcs must now show dmistat==11.
    shift_ir(IR_DTMCS);
    uint64_t dtmcs_busy = shift_dr_out(32);
    check(((dtmcs_busy >> 10) & 0x3) == 0x3, "T4: dtmcs.dmistat == 3 (busy) before dmireset",
          (dtmcs_busy >> 10) & 0x3, 0x3);

    // Write dtmcs with bit16=1 (dmireset): op_stat clears (donor :175-176).
    shift_ir(IR_DTMCS);
    shift_dr(32, (1ULL << 16));
    idle_cycle(2);

    shift_ir(IR_DTMCS);
    uint64_t dtmcs_clear = shift_dr_out(32);
    check(((dtmcs_clear >> 10) & 0x3) == 0, "T4: dtmcs.dmistat == 0 after dmireset write",
          (dtmcs_clear >> 10) & 0x3, 0);

    // And the engine accepts a fresh op (proves full recovery, not just the
    // status bit).
    apb.prdata = 0x11223344;
    unsigned before = apb.txn_count;
    dmi_scan(0x10, 0, DMI_OP_READ);
    check(apb.txn_count == before + 1, "T4: DMI engine accepts a new op after dmireset",
          apb.txn_count, before + 1);
    test_result("T4 dmireset: busy op_stat recovers, dtmcs.dmistat 3 -> 0, engine live");
}

//=============================================================================
int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    dut = new VTDT_DTM;

    reset_dut();

    // Phase 1: TAP / DR rows.
    test_t1_tlr_then_scan();
    test_t2_idcode();
    test_t3_dtmcs();
    test_t7_dmi_acc_bypass();
    test_t9_tap_walk();
    test_t10_tdo_idle_stable();

    // Phase 2: DMI bridge rows.
    test_t5_dmi_read();
    test_t6_dmi_write();
    test_t8_inflight_busy();
    test_t4_dmireset();

    dut->final();
    delete dut;

    if (g_fail) {
        printf("[dtm_tb] UNIT-FAIL (%d checks failed)\n", g_fail);
        return 1;
    }
    printf("[dtm_tb] UNIT-PASS\n");
    return 0;
}
