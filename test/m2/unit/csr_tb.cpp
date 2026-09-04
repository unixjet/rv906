//=============================================================================
// csr_tb.cpp - standalone unit bench for rtl/CSR.v (M2 plan task 2.2)
//=============================================================================
// Verilates CSR.v + rvproc_pkg.sv alone (no IDU, no IU, no RTU, no SoC) and
// drives the frozen idu_cp0_ex1_*/rtu_yy_xx_*/rtu_cp0_* ports directly with
// hand-scripted single-instruction stimulus, the same tick()-based clocking
// and check()/test_result() bookkeeping pattern as test/m1/unit/
// fetchsink_tb.cpp and icache_tb.cpp.
//
// This is a WHITE-BOX test of CSR.v's OWN stated contract (contract 7's
// exact minimal CSR set, plus the trap-entry-capture/mret/RMW mechanics
// CSR.v's own header documents) run in isolation. It does not exercise
// RTU's real arbiter/flush-FSM timing (RTU.v's real body is plan Task 4) --
// it proves CSR.v honors the interface contract IT documents, driven by a
// script that stands in for RTU/IDU.
//
// Build/run: make -C test/m2/unit csr && bin/unit/csr_tb
// Prints one line per test and ends with UNIT-PASS or UNIT-FAIL.
//=============================================================================

#include <verilated.h>
#include "VCSR.h"

#include <cstdio>
#include <cstdint>
#include <cstring>

//-----------------------------------------------------------------------------
// Geometry / constants (mirrors rvproc_pkg.sv)
//-----------------------------------------------------------------------------
static const unsigned PC_WIDTH = 40;
static const uint64_t PC_MASK  = (1ULL << PC_WIDTH) - 1;
static const uint64_t RESET_VECTOR = 0x80000000ULL;

// CSR addresses (rvproc_pkg.sv)
static const uint32_t CSR_MSTATUS   = 0x300;
static const uint32_t CSR_MISA      = 0x301;
static const uint32_t CSR_MIE       = 0x304;
static const uint32_t CSR_MTVEC     = 0x305;
static const uint32_t CSR_MSCRATCH  = 0x340;
static const uint32_t CSR_MEPC      = 0x341;
static const uint32_t CSR_MCAUSE    = 0x342;
static const uint32_t CSR_MTVAL     = 0x343;
static const uint32_t CSR_MIP       = 0x344;
static const uint32_t CSR_MCYCLE    = 0xB00;
static const uint32_t CSR_MINSTRET  = 0xB02;
static const uint32_t CSR_MVENDORID = 0xF11;
static const uint32_t CSR_MARCHID   = 0xF12;
static const uint32_t CSR_MIMPID    = 0xF13;
static const uint32_t CSR_MHARTID   = 0xF14;
static const uint32_t CSR_MXSTATUS  = 0x7C0;
static const uint32_t CSR_MHCR      = 0x7C1;

// MHCR bit positions (rvproc_pkg.sv)
static const int MHCR_IE_BIT   = 0;
static const int MHCR_DE_BIT   = 1;
static const int MHCR_WA_BIT   = 2;
static const int MHCR_WB_BIT   = 3;
static const int MHCR_RSE_BIT  = 4;
static const int MHCR_BPE_BIT  = 5;
static const int MHCR_BTBE_BIT = 6;
static const int MHCR_WBR_BIT  = 8;
static const int MXSTATUS_MM   = 15;

// CP0 FUNC one-hot values (rvproc_pkg.sv, confirmed bit-exact against
// aq_idu_cfig.h:453-474 -- see that file's header comment).
static const uint32_t CP0_FUNC_ECALL   = 0x00012;
static const uint32_t CP0_FUNC_EBREAK  = 0x00022;
static const uint32_t CP0_FUNC_MRET    = 0x00042;
static const uint32_t CP0_FUNC_FENCE   = 0x00028;
static const uint32_t CP0_FUNC_FENCEI  = 0x00024;
static const uint32_t CP0_FUNC_CSRRW   = 0x00011;
static const uint32_t CP0_FUNC_CSRRS   = 0x00021;
static const uint32_t CP0_FUNC_CSRRC   = 0x00041;
static const uint32_t CP0_FUNC_CSRRWI  = 0x00211;
static const uint32_t CP0_FUNC_CSRRSI  = 0x00221;
static const uint32_t CP0_FUNC_CSRRCI  = 0x00241;
static const uint32_t CP0_FUNC_SFENCE  = 0x00044;   // M4 Task 7 (rvproc_pkg.sv)

//-----------------------------------------------------------------------------
// DUT plumbing
//-----------------------------------------------------------------------------
static VCSR *dut = nullptr;
static uint64_t g_cycles = 0;

static void tie_idle_inputs(void) {
    dut->idu_cp0_ex1_sel        = 0;
    dut->idu_cp0_ex1_func       = 0;
    dut->idu_cp0_ex1_opcode     = 0;
    dut->idu_cp0_ex1_illegal    = 0;
    dut->idu_cp0_ex1_fetch_pgflt  = 0;
    dut->idu_cp0_ex1_fetch_accflt = 0;
    dut->idu_cp0_ex1_src0_data  = 0;
    dut->idu_cp0_ex1_src1_data  = 0;
    dut->idu_cp0_ex1_dst0_reg   = 0;
    dut->iu_cp0_ex1_cur_pc      = 0;
    dut->rtu_yy_xx_expt_vld     = 0;
    dut->rtu_yy_xx_expt_int     = 0;
    dut->rtu_yy_xx_expt_vec     = 0;
    dut->rtu_yy_xx_flush_fe     = 0;
    dut->rtu_yy_xx_flush        = 0;
    dut->rtu_cp0_epc            = 0;
    dut->rtu_cp0_tval           = 0;
    dut->ifu_cp0_icache_inv_done= 0;
    dut->lsu_cp0_stb_empty      = 1;   // LSU quiescent so FENCE/FENCE.I complete
    dut->lsu_cp0_clean_done     = 0;
    dut->mmu_cp0_sfence_done    = 0;
    dut->bht_cp0_inv_done       = 0;
    dut->mtip = 0;
    dut->msip = 0;
    dut->meip = 0;
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
// Result bookkeeping (mirrors fetchsink_tb.cpp/icache_tb.cpp exactly)
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
    printf("[csr_tb] %-56s %s\n", name, g_local ? "FAIL" : "PASS");
    g_local = 0;
}

//-----------------------------------------------------------------------------
// Dispatch primitive: present ONE CP0 instruction for exactly one cycle
// (CSR.v's own documented contract: no FSM, single-cycle unconditional
// completion) and read back every EX1-completion output the same cycle
// (combinational off this cycle's dispatch, per the "SECTION EX1
// COMPLETION" comment in CSR.v).
//-----------------------------------------------------------------------------
struct DispatchResult {
    bool     wb_vld = false;
    uint64_t wb_data = 0;
    unsigned wb_preg = 0;
    bool     cmplt_dp = false;
    bool     expt_vld = false;
    bool     expt_int = false;
    unsigned expt_vec = 0;
    bool     chgflw = false;
    uint64_t chgflw_pc = 0;
};

// func: the CP0_FUNC_* one-hot value. csr_addr: goes in src1_data[11:0].
// rs1_or_uimm: for register forms, goes in src0_data (the VALUE IDU already
// read out of the GPR file); for immediate forms, goes in opcode[19:15] (5
// bits) exactly like real hardware encodes it. `rs1_reg_field` is the raw
// 5-bit rs1/uimm FIELD CSR.v itself reads off opcode[19:15] to detect the
// RISC-V-mandated "CSRRS/CSRRC with rs1==x0 skip the write" rule (CSR.v's
// own `rs1_is_x0` wire) -- defaults to a nonzero placeholder ("not x0") so
// every ordinary register-form RMW test in this file actually writes,
// unless a test explicitly passes 0 to model a real `rs1==x0` dispatch.
static DispatchResult dispatch(uint32_t func, uint32_t csr_addr = 0,
                                uint64_t rs1_or_uimm = 0, unsigned dst_reg = 0,
                                bool illegal = false, bool imm_form = false,
                                unsigned rs1_reg_field = 1,
                                bool fetch_pgflt = false, bool fetch_accflt = false) {
    dut->idu_cp0_ex1_sel       = 1;
    dut->idu_cp0_ex1_func      = func;
    dut->idu_cp0_ex1_illegal   = illegal ? 1 : 0;
    dut->idu_cp0_ex1_fetch_pgflt  = fetch_pgflt  ? 1 : 0;
    dut->idu_cp0_ex1_fetch_accflt = fetch_accflt ? 1 : 0;
    dut->idu_cp0_ex1_src1_data = csr_addr;
    dut->idu_cp0_ex1_dst0_reg  = dst_reg;
    if (imm_form) {
        dut->idu_cp0_ex1_src0_data = 0;
        dut->idu_cp0_ex1_opcode    = (uint32_t)((rs1_or_uimm & 0x1F) << 15);
    } else {
        dut->idu_cp0_ex1_src0_data = rs1_or_uimm;
        dut->idu_cp0_ex1_opcode    = (uint32_t)((rs1_reg_field & 0x1F) << 15);
    }

    DispatchResult r;
    dut->eval();   // combinational EX1-completion outputs, THIS cycle
    r.wb_vld    = dut->cp0_rtu_ex1_wb_vld != 0;
    r.wb_data   = dut->cp0_rtu_ex1_wb_data;
    r.wb_preg   = dut->cp0_rtu_ex1_wb_preg;
    r.cmplt_dp  = dut->cp0_rtu_ex1_cmplt_dp != 0;
    r.expt_vld  = dut->cp0_rtu_ex1_expt_vld != 0;
    r.expt_int  = dut->cp0_rtu_ex1_expt_int != 0;
    r.expt_vec  = dut->cp0_rtu_ex1_expt_vec;
    r.chgflw    = dut->cp0_rtu_ex1_chgflw != 0;
    r.chgflw_pc = dut->cp0_rtu_ex1_chgflw_pc;

    tick();   // commit the flops this dispatch drives (RMW write, trap swap, etc.)

    dut->idu_cp0_ex1_sel = 0;
    dut->idu_cp0_ex1_illegal = 0;
    dut->idu_cp0_ex1_fetch_pgflt  = 0;
    dut->idu_cp0_ex1_fetch_accflt = 0;
    return r;
}

// Plain read: csrrs rd, addr, x0 -- a REAL rs1==x0 dispatch (rs1_reg_field=0
// explicitly), so CSR.v's own rs1_is_x0 skip-write rule applies and this is
// a genuine no-side-effect read of every CSR in this file, including the
// free-running counters (does not spuriously reset mcycle's per-cycle
// increment -- see test_mcycle_free_running()'s header comment for why
// that distinction is load-bearing).
static uint64_t csr_read(uint32_t addr) {
    DispatchResult r = dispatch(CP0_FUNC_CSRRS, addr, /*rs1=*/0, /*dst=*/1,
                                 /*illegal=*/false, /*imm_form=*/false, /*rs1_reg_field=*/0);
    return r.wb_data;
}

static void csr_write(uint32_t addr, uint64_t value) {
    dispatch(CP0_FUNC_CSRRW, addr, value, /*dst=*/0);
}

static DispatchResult tick_no_dispatch(void) {
    dut->idu_cp0_ex1_sel = 0;
    DispatchResult r;
    dut->eval();
    r.wb_vld   = dut->cp0_rtu_ex1_wb_vld != 0;
    r.expt_vld = dut->cp0_rtu_ex1_expt_vld != 0;
    r.chgflw   = dut->cp0_rtu_ex1_chgflw != 0;
    tick();
    return r;
}

//=============================================================================
// Tests
//=============================================================================

static void test_reset_state(void) {
    check(dut->cp0_rtu_ex1_wb_vld == 0, "reset: wb_vld quiescent");
    check(dut->cp0_rtu_ex1_expt_vld == 0, "reset: expt_vld quiescent");
    check(dut->cp0_rtu_ex1_chgflw == 0, "reset: chgflw quiescent");
    // M4: mstatus reset now carries the donor's full RV64 shape: SXL/UXL=2'b10
    // (bits 35:32 = 0xA), MPP=11 (bits 12:11), SPP=1 (bit 8) -- donor
    // aq_cp0_trap_csr.v:499-501,654. 0xA00001900.
    check(csr_read(CSR_MSTATUS) == 0xA00001900ULL,
          "reset: mstatus reads SXL/UXL=10,MPP=11,SPP=1 (0xA00001900)",
          csr_read(CSR_MSTATUS), 0xA00001900ULL);
    check(csr_read(CSR_MHCR) == 0x108,
          "reset: MHCR reads wb=1(bit3)+wbr=1(bit8)=0x108, rest 0", csr_read(CSR_MHCR), 0x108);
    check(csr_read(CSR_MXSTATUS) == (1ULL << MXSTATUS_MM),
          "reset: MXSTATUS.mm reads 1 (contract 3)", csr_read(CSR_MXSTATUS), 1ULL << MXSTATUS_MM);
    check(csr_read(CSR_MTVEC) == 0, "reset: mtvec reads 0");
    check(csr_read(CSR_MEPC) == 0, "reset: mepc reads 0");
    check(csr_read(CSR_MCAUSE) == 0, "reset: mcause reads 0");
    check(csr_read(CSR_MTVAL) == 0, "reset: mtval reads 0");
    check(csr_read(CSR_MSCRATCH) == 0, "reset: mscratch reads 0");
    check(csr_read(CSR_MIE) == 0, "reset: mie reads 0");
    check(dut->cp0_lsu_wa == 0, "reset: cp0_lsu_wa (MHCR.wa) is 0 (contract 6)");
    check(dut->cp0_lsu_dcache_en == 0, "reset: cp0_lsu_dcache_en (MHCR.de) is 0");
    check(dut->cp0_ifu_icache_en == 0, "reset: cp0_ifu_icache_en (MHCR.ie) is 0");
    check(dut->cp0_lsu_mm == 1, "reset: cp0_lsu_mm (MXSTATUS.mm passthrough) is 1");
    check(dut->cp0_xx_mrvbr == RESET_VECTOR,
          "reset: cp0_xx_mrvbr == RESET_VECTOR", dut->cp0_xx_mrvbr, RESET_VECTOR);
    test_result("T1 reset state matches contract 7/6/3 exactly");
}

static void test_misa_and_ids_readonly(void) {
    uint64_t misa_before = csr_read(CSR_MISA);
    check(misa_before == 0x8000000000001104ULL,
          "misa: MXL=64,I|M|C == 0x8000000000001104", misa_before, 0x8000000000001104ULL);
    csr_write(CSR_MISA, 0xFFFFFFFFFFFFFFFFULL);
    check(csr_read(CSR_MISA) == misa_before, "misa: write is ignored (RO)");

    uint64_t vendorid = csr_read(CSR_MVENDORID);
    csr_write(CSR_MVENDORID, 0xDEADBEEFULL);
    check(csr_read(CSR_MVENDORID) == vendorid, "mvendorid: write ignored (RO)");
    csr_write(CSR_MARCHID, 0x1234ULL);
    check(csr_read(CSR_MARCHID) == 0, "marchid: write ignored (RO), reads 0");
    csr_write(CSR_MIMPID, 0x1234ULL);
    check(csr_read(CSR_MIMPID) == 0, "mimpid: write ignored (RO), reads 0");
    csr_write(CSR_MHARTID, 0x1234ULL);
    check(csr_read(CSR_MHARTID) == 0, "mhartid: write ignored (RO), reads 0");
    test_result("T2 misa/mvendorid/marchid/mimpid/mhartid: RO hardwired constants");
}

static void test_csrrw_rmw(void) {
    uint64_t old = csr_read(CSR_MSCRATCH);
    check(old == 0, "csrrw: mscratch starts at 0");
    DispatchResult r = dispatch(CP0_FUNC_CSRRW, CSR_MSCRATCH, 0x1122334455667788ULL, /*dst=*/5);
    check(r.wb_vld && r.wb_data == 0, "csrrw: old value (0) rides wb_data, dst!=x0", r.wb_data, 0);
    check(r.wb_preg == 5, "csrrw: wb_preg == dst0_reg", r.wb_preg, 5);
    check(r.cmplt_dp, "csrrw: cmplt_dp asserted (one-hot completion source)");
    check(!r.expt_vld && !r.chgflw, "csrrw: no exception, no changeflow");
    check(csr_read(CSR_MSCRATCH) == 0x1122334455667788ULL,
          "csrrw: new value stored verbatim", csr_read(CSR_MSCRATCH), 0x1122334455667788ULL);
    test_result("T3 CSRRW: plain write, old value on wb_data (RMW form 1/6)");
}

static void test_csrrs_rmw(void) {
    csr_write(CSR_MSCRATCH, 0x00F0ULL);
    DispatchResult r = dispatch(CP0_FUNC_CSRRS, CSR_MSCRATCH, 0x000FULL, /*dst=*/2);
    check(r.wb_data == 0x00F0ULL, "csrrs: old value on wb_data", r.wb_data, 0x00F0ULL);
    check(csr_read(CSR_MSCRATCH) == 0x00FFULL,
          "csrrs: new value == old | rs1", csr_read(CSR_MSCRATCH), 0x00FFULL);
    test_result("T4 CSRRS: set-bits RMW, rdata|rs1 (RMW form 2/6)");
}

static void test_csrrc_rmw(void) {
    csr_write(CSR_MSCRATCH, 0x00FFULL);
    DispatchResult r = dispatch(CP0_FUNC_CSRRC, CSR_MSCRATCH, 0x000FULL, /*dst=*/2);
    check(r.wb_data == 0x00FFULL, "csrrc: old value on wb_data", r.wb_data, 0x00FFULL);
    check(csr_read(CSR_MSCRATCH) == 0x00F0ULL,
          "csrrc: new value == old & ~rs1", csr_read(CSR_MSCRATCH), 0x00F0ULL);
    test_result("T5 CSRRC: clear-bits RMW, rdata&~rs1 (RMW form 3/6)");
}

static void test_csrrwi_rmw(void) {
    csr_write(CSR_MSCRATCH, 0xFFFFFFFFFFFFFFFFULL);
    DispatchResult r = dispatch(CP0_FUNC_CSRRWI, CSR_MSCRATCH, /*uimm=*/0x15,
                                 /*dst=*/3, /*illegal=*/false, /*imm_form=*/true);
    check(r.wb_data == 0xFFFFFFFFFFFFFFFFULL, "csrrwi: old value on wb_data");
    check(csr_read(CSR_MSCRATCH) == 0x15ULL,
          "csrrwi: new value == zero-extended 5-bit uimm from opcode[19:15]",
          csr_read(CSR_MSCRATCH), 0x15ULL);
    test_result("T6 CSRRWI: immediate write, uimm from opcode[19:15] (RMW form 4/6)");
}

static void test_csrrsi_rmw(void) {
    csr_write(CSR_MSCRATCH, 0x10ULL);
    dispatch(CP0_FUNC_CSRRSI, CSR_MSCRATCH, /*uimm=*/0x03, /*dst=*/0,
             /*illegal=*/false, /*imm_form=*/true);
    check(csr_read(CSR_MSCRATCH) == 0x13ULL,
          "csrrsi: new value == old | uimm", csr_read(CSR_MSCRATCH), 0x13ULL);
    test_result("T7 CSRRSI: immediate set-bits (RMW form 5/6)");
}

static void test_csrrci_rmw(void) {
    csr_write(CSR_MSCRATCH, 0x13ULL);
    dispatch(CP0_FUNC_CSRRCI, CSR_MSCRATCH, /*uimm=*/0x03, /*dst=*/0,
             /*illegal=*/false, /*imm_form=*/true);
    check(csr_read(CSR_MSCRATCH) == 0x10ULL,
          "csrrci: new value == old & ~uimm", csr_read(CSR_MSCRATCH), 0x10ULL);
    test_result("T8 CSRRCI: immediate clear-bits (RMW form 6/6)");
}

static void test_mret_pop(void) {
    // Arrange: MIE=1, MPIE=0 via csrrw mstatus (bit3=MIE, bit7=MPIE).
    // M4: keep MPP=M(11) so mret stays in M-mode (full-CSRRW would otherwise
    // clear MPP to 0 and drop the bench to U-mode for later tests).
    csr_write(CSR_MSTATUS, (1ULL << 3) | (3ULL << 11));
    check((csr_read(CSR_MSTATUS) & 0x88) == 0x08, "mret setup: MIE=1,MPIE=0 written");
    csr_write(CSR_MEPC, 0x80001000ULL);

    DispatchResult r = dispatch(CP0_FUNC_MRET, 0, 0, 0);
    check(r.chgflw, "mret: chgflw asserted");
    check(r.chgflw_pc == 0x80001000ULL, "mret: chgflw_pc == mepc",
          r.chgflw_pc, 0x80001000ULL);
    check(!r.wb_vld, "mret: no GPR writeback (not a CSR op)");
    check(!r.expt_vld, "mret: not an exception");

    uint64_t mstatus_after = csr_read(CSR_MSTATUS);
    check(((mstatus_after >> 3) & 1) == 0,
          "mret: MIE <= old MPIE (was 0, so MIE now reads 0)",
          (mstatus_after >> 3) & 1, 0);
    check(((mstatus_after >> 7) & 1) == 1,
          "mret: MPIE <= 1 unconditionally", (mstatus_after >> 7) & 1, 1);
    test_result("T9a mret issues chgflw to mepc");
}

static void test_mret_pop_bit_semantics(void) {
    // Precise MIE<=MPIE / MPIE<=1 semantics, isolated from T9a's setup.
    // M4: keep MPP=M(11) so mret stays in M-mode.
    csr_write(CSR_MSTATUS, (1ULL << 7) | (3ULL << 11));   // MPIE=1, MIE=0
    check(((csr_read(CSR_MSTATUS) >> 7) & 1) == 1, "mret bit-semantics: MPIE=1 written");
    check(((csr_read(CSR_MSTATUS) >> 3) & 1) == 0, "mret bit-semantics: MIE=0 written");

    dispatch(CP0_FUNC_MRET, 0, 0, 0);
    uint64_t after = csr_read(CSR_MSTATUS);
    check(((after >> 3) & 1) == 1, "mret: MIE <= MPIE (was 1) -> MIE now 1",
          (after >> 3) & 1, 1);
    check(((after >> 7) & 1) == 1, "mret: MPIE <= 1 unconditionally", (after >> 7) & 1, 1);
    test_result("T9b mret: MIE<=MPIE, MPIE<=1 (exact donor semantics, trap_csr.v:675-681)");
}

static void test_trap_entry_capture(void) {
    // Precondition: MIE=1 so the swap is observable.
    csr_write(CSR_MSTATUS, (1ULL << 3));   // MIE=1, MPIE=0
    check(((csr_read(CSR_MSTATUS) >> 3) & 1) == 1, "trap setup: MIE=1 written");

    dut->rtu_yy_xx_expt_vld = 1;
    dut->rtu_yy_xx_expt_int = 0;
    dut->rtu_yy_xx_expt_vec = 2;         // illegal instruction (in the tval allowlist)
    dut->rtu_cp0_epc  = 0x80002004ULL;
    dut->rtu_cp0_tval = 0xDEADBEEFULL;
    dut->idu_cp0_ex1_sel = 0;
    dut->eval();
    tick();
    dut->rtu_yy_xx_expt_vld = 0;

    uint64_t mstatus_after = csr_read(CSR_MSTATUS);
    check(((mstatus_after >> 7) & 1) == 1, "trap entry: MPIE <= old MIE (was 1)",
          (mstatus_after >> 7) & 1, 1);
    check(((mstatus_after >> 3) & 1) == 0, "trap entry: MIE <= 0", (mstatus_after >> 3) & 1, 0);
    check(csr_read(CSR_MEPC) == 0x80002004ULL, "trap entry: mepc <= rtu_cp0_epc",
          csr_read(CSR_MEPC), 0x80002004ULL);
    check(csr_read(CSR_MCAUSE) == 2, "trap entry: mcause == vec 2 (illegal inst), int bit 0",
          csr_read(CSR_MCAUSE), 2);
    check(csr_read(CSR_MTVAL) == 0xDEADBEEFULL,
          "trap entry: mtval <= rtu_cp0_tval (vec 2 IS in the allowlist)",
          csr_read(CSR_MTVAL), 0xDEADBEEFULL);
    test_result("T10 trap-entry capture: MIE/MPIE swap + mepc/mcause/mtval (vec-in-allowlist)");
}

static void test_trap_entry_mtval_non_allowlist(void) {
    dut->rtu_yy_xx_expt_vld = 1;
    dut->rtu_yy_xx_expt_int = 0;
    dut->rtu_yy_xx_expt_vec = 3;          // breakpoint -- NOT in the allowlist
    dut->rtu_cp0_epc  = 0x80003000ULL;
    dut->rtu_cp0_tval = 0xCAFEF00DULL;    // nonzero, must be SUPPRESSED
    dut->idu_cp0_ex1_sel = 0;
    dut->eval();
    tick();
    dut->rtu_yy_xx_expt_vld = 0;

    check(csr_read(CSR_MTVAL) == 0,
          "non-allowlist trap (vec 3): mtval forced to 0 despite nonzero rtu_cp0_tval",
          csr_read(CSR_MTVAL), 0);
    check(csr_read(CSR_MCAUSE) == 3, "non-allowlist trap: mcause == 3", csr_read(CSR_MCAUSE), 3);
    check(csr_read(CSR_MEPC) == 0x80003000ULL, "non-allowlist trap: mepc still captured");
    test_result("T11 trap-entry mtval: non-allowlist vec forces mtval=0");
}

static void test_trap_entry_int_bit(void) {
    dut->rtu_yy_xx_expt_vld = 1;
    dut->rtu_yy_xx_expt_int = 1;           // interrupt, not a synchronous exception
    dut->rtu_yy_xx_expt_vec = 7;           // MTI
    dut->rtu_cp0_epc  = 0x80004000ULL;
    dut->rtu_cp0_tval = 0;
    dut->idu_cp0_ex1_sel = 0;
    dut->eval();
    tick();
    dut->rtu_yy_xx_expt_vld = 0;

    uint64_t mcause = csr_read(CSR_MCAUSE);
    check((mcause >> 63) == 1, "trap entry: mcause interrupt bit (bit63) set", mcause >> 63, 1);
    check((mcause & 0x1F) == 7, "trap entry: mcause vec == 7 (MTI)", mcause & 0x1F, 7);
    test_result("T12 trap-entry mcause: interrupt bit captured from rtu_yy_xx_expt_int");
}

static void test_ecall_ebreak_illegal(void) {
    DispatchResult r_ecall = dispatch(CP0_FUNC_ECALL, 0, 0, 0);
    check(r_ecall.expt_vld && r_ecall.expt_vec == 11 && !r_ecall.expt_int,
          "ecall: expt_vld, vec=11 (M-mode ecall), not an interrupt",
          r_ecall.expt_vec, 11);
    check(!r_ecall.wb_vld && !r_ecall.chgflw, "ecall: no wb, no chgflw");

    DispatchResult r_ebreak = dispatch(CP0_FUNC_EBREAK, 0, 0, 0);
    check(r_ebreak.expt_vld && r_ebreak.expt_vec == 3,
          "ebreak: expt_vld, vec=3 (breakpoint)", r_ebreak.expt_vec, 3);

    DispatchResult r_illegal = dispatch(CP0_FUNC_CSRRW, CSR_MSCRATCH, 0, 0, /*illegal=*/true);
    check(r_illegal.expt_vld && r_illegal.expt_vec == 2,
          "illegal (idu_cp0_ex1_illegal=1): expt_vld, vec=2, regardless of func",
          r_illegal.expt_vec, 2);
    check(!r_illegal.wb_vld, "illegal: no GPR writeback even though func looked like CSRRW");
    test_result("T13 ecall/ebreak/illegal: expt_vld + correct vec, no side effects");
}

static void test_fetch_fault_priority(void) {
    // pgflt alone -> vec 12, not an interrupt, no wb/chgflw.
    DispatchResult r_pg = dispatch(/*func=*/0, 0, 0, 0, /*illegal=*/false,
                                    /*imm_form=*/false, /*rs1_reg_field=*/1,
                                    /*fetch_pgflt=*/true, /*fetch_accflt=*/false);
    check(r_pg.expt_vld && r_pg.expt_vec == 12 && !r_pg.expt_int,
          "fetch pgflt: expt_vld, vec=12 (fetch page fault)", r_pg.expt_vec, 12);
    check(!r_pg.wb_vld && !r_pg.chgflw, "fetch pgflt: no wb, no chgflw");

    // accflt alone -> vec 1.
    DispatchResult r_acc = dispatch(/*func=*/0, 0, 0, 0, /*illegal=*/false,
                                     /*imm_form=*/false, /*rs1_reg_field=*/1,
                                     /*fetch_pgflt=*/false, /*fetch_accflt=*/true);
    check(r_acc.expt_vld && r_acc.expt_vec == 1,
          "fetch accflt: expt_vld, vec=1 (fetch access fault)", r_acc.expt_vec, 1);

    // both asserted (shouldn't happen from IDU, but the priority mux must be
    // deterministic) -- donor priority pgflt(12) > accflt(1).
    DispatchResult r_both = dispatch(/*func=*/0, 0, 0, 0, /*illegal=*/false,
                                      /*imm_form=*/false, /*rs1_reg_field=*/1,
                                      /*fetch_pgflt=*/true, /*fetch_accflt=*/true);
    check(r_both.expt_vec == 12, "fetch pgflt+accflt both set: pgflt wins (donor priority)",
          r_both.expt_vec, 12);

    // fetch pgflt + idu_cp0_ex1_illegal both set (the synthetic-NOP injection
    // never sets illegal, but the priority mux must still resolve correctly
    // if it ever did) -- donor priority pgflt(12) > illegal(2).
    DispatchResult r_pg_ill = dispatch(CP0_FUNC_CSRRW, CSR_MSCRATCH, 0, 0,
                                        /*illegal=*/true, /*imm_form=*/false,
                                        /*rs1_reg_field=*/1, /*fetch_pgflt=*/true);
    check(r_pg_ill.expt_vec == 12, "fetch pgflt + illegal both set: pgflt wins over illegal",
          r_pg_ill.expt_vec, 12);
    test_result("T13b fetch-fault priority: vec 12 > vec 1 > vec 2 (illegal), no side effects");
}

static void test_fence_no_op(void) {
    // Plain FENCE: with LSU quiescent (tie_idle_inputs drives
    // lsu_cp0_stb_empty=1) it completes immediately -- cmplt_dp asserted,
    // no wb/expt/chgflw. (FENCE.I got a real serialize+clean+invalidate in
    // Task 10 for rv64ui-p-fence_i, so it is exercised separately below.)
    DispatchResult r_fence  = dispatch(CP0_FUNC_FENCE, 0, 0, 0);
    check(!r_fence.wb_vld && !r_fence.expt_vld && !r_fence.chgflw,
          "fence: no wb/expt/chgflw (plain no-GPR-result completion)");
    // TASK 4 FIX (rtl/CSR.v): cmplt_dp is RTU's one-hot RETIRE-heartbeat
    // leg, not a GPR-writeback-select bit -- it must fire for ANY
    // non-flushed CP0 dispatch, or RTU's retire register would never latch
    // these instructions and they could never retire.
    check(r_fence.cmplt_dp, "fence: cmplt_dp asserted (retire heartbeat, not gated on wb)");

    // FENCE.I (Task 10): a real donor-faithful serialize: wait LSU quiescent
    // (FENC), run the D-cache clean walk (CDCA), run the I-cache INV_ALL
    // (IICA), then complete with cmplt_dp + a front-end changeflow (refetch).
    // Drive the two handshake-done pulses and hold ex1_sel the whole time,
    // exactly as IDU does while the fence is held in EX1.
    dut->idu_cp0_ex1_sel       = 1;
    dut->idu_cp0_ex1_func      = CP0_FUNC_FENCEI;
    dut->idu_cp0_ex1_illegal   = 0;
    dut->idu_cp0_ex1_src1_data = 0;
    dut->idu_cp0_ex1_dst0_reg  = 0;
    dut->idu_cp0_ex1_src0_data = 0;
    dut->idu_cp0_ex1_opcode    = 0;
    dut->lsu_cp0_stb_empty     = 1;
    dut->eval();
    check(dut->cp0_rtu_ex1_cmplt_dp == 0, "fence.i held: cmplt_dp low while serialize pending");
    check(dut->cp0_rtu_ex1_chgflw  == 0, "fence.i held: no chgflw until FI_CMPLT");
    tick();                                   // FI_IDLE -> FI_CLEAN
    check(dut->cp0_lsu_dcache_clean == 1, "fence.i: D-cache clean walk requested (FI_CLEAN)");
    dut->lsu_cp0_clean_done = 1;
    dut->eval();
    tick();                                   // FI_CLEAN -> FI_INV
    dut->lsu_cp0_clean_done = 0;
    check(dut->cp0_ifu_icache_inv_req == 1, "fence.i: I-cache INV_ALL requested (FI_INV)");
    dut->ifu_cp0_icache_inv_done = 1;
    dut->eval();
    tick();                                   // FI_INV -> FI_CMPLT
    dut->ifu_cp0_icache_inv_done = 0;
    dut->eval();
    check(dut->cp0_rtu_ex1_cmplt_dp == 1, "fence.i: cmplt_dp asserted at FI_CMPLT (retire heartbeat)");
    check(dut->cp0_rtu_ex1_chgflw  == 1, "fence.i: changeflow at FI_CMPLT (front-end refetch)");
    check(dut->cp0_rtu_ex1_wb_vld  == 0, "fence.i: no GPR writeback");
    check(dut->cp0_rtu_ex1_expt_vld== 0, "fence.i: no exception");
    tick();                                   // FI_CMPLT -> FI_IDLE
    dut->idu_cp0_ex1_sel = 0;
    dut->idu_cp0_ex1_illegal = 0;
    test_result("T14 FENCE/FENCE.I: fence completes; fence.i serializes clean+inv+refetch");
}

// sfence.vma sequencer (M4 Task 7; donor aq_cp0_fence_inst.v FNC_IDLE->
// FNC_CMMU->FNC_IICA->FNC_CMPLT, :146-194 -- full donor citation and the
// three rv906-specific deviations (STB-drain wait, near-1-cycle MMU ack,
// wait-then-wipe mid-walk hazard) are documented at CSR.v's sfence_state
// block itself). This bench drives CSR.v's cp0_mmu_sfence_vld/
// mmu_cp0_sfence_done handshake ports directly, the same way
// test_fence_no_op() stands in for FENCE.I's lsu_cp0_clean_done/
// ifu_cp0_icache_inv_done handshakes -- MMU.v's own sfence_apply/
// sfence_pend_r logic is exercised separately in mmu_tb.cpp.
static void test_sfence_launch_ack_complete(void) {
    dut->idu_cp0_ex1_sel       = 1;
    dut->idu_cp0_ex1_func      = CP0_FUNC_SFENCE;
    dut->idu_cp0_ex1_illegal   = 0;
    dut->idu_cp0_ex1_src1_data = 0;
    dut->idu_cp0_ex1_dst0_reg  = 0;
    dut->idu_cp0_ex1_src0_data = 0;
    dut->idu_cp0_ex1_opcode    = 0;
    dut->lsu_cp0_stb_empty     = 1;
    dut->mmu_cp0_sfence_done   = 0;
    dut->eval();
    check(dut->cp0_mmu_sfence_vld == 1,
          "sfence: launch pulses cp0_mmu_sfence_vld same cycle (SF_IDLE->SF_WAIT)");
    check(dut->cp0_rtu_ex1_cmplt_dp == 0, "sfence: held (cmplt_dp low) while MMU ack pending");
    check(dut->cp0_rtu_ex1_chgflw  == 0,
          "sfence: no chgflw ever (donor FNC_CMPLT->FNC_IDLE has no PC-redirect output)");
    tick();                                    // SF_IDLE -> SF_WAIT
    check(dut->cp0_mmu_sfence_vld == 0,
          "sfence: cp0_mmu_sfence_vld is a one-cycle launch pulse, not held through SF_WAIT");
    dut->mmu_cp0_sfence_done = 1;               // MMU acks (near-1-cycle, D11 single flop-array TLB)
    dut->eval();
    check(dut->cp0_rtu_ex1_cmplt_dp == 0,
          "sfence: still held the cycle MMU asserts done (SF_WAIT, transitions next edge)");
    tick();                                    // SF_WAIT -> SF_CMPLT
    dut->mmu_cp0_sfence_done = 0;
    dut->eval();
    check(dut->cp0_rtu_ex1_cmplt_dp == 1, "sfence: cmplt_dp asserted at SF_CMPLT");
    check(dut->cp0_rtu_ex1_chgflw  == 0, "sfence: no chgflw at completion either");
    check(dut->cp0_rtu_ex1_wb_vld  == 0, "sfence: no GPR writeback");
    check(dut->cp0_rtu_ex1_expt_vld== 0, "sfence: not an exception (legal M-mode sfence)");
    tick();                                    // SF_CMPLT -> SF_IDLE
    dut->idu_cp0_ex1_sel = 0;
    test_result("T23a sfence.vma: M-mode launch/ack/complete handshake, one-cycle vld pulse, no chgflw");
}

static void test_sfence_stb_wait(void) {
    // Deliberate rv906 deviation from the donor (no STB-drain precondition
    // in aq_cp0_fence_inst.v's FNC_CMMU): rv906's PTW servant probes the
    // D-cache array with no STB-forwarding path (D3), so sfence must wait
    // for the STB to drain before the MMU invalidate can be trusted coherent.
    dut->idu_cp0_ex1_sel       = 1;
    dut->idu_cp0_ex1_func      = CP0_FUNC_SFENCE;
    dut->idu_cp0_ex1_illegal   = 0;
    dut->idu_cp0_ex1_src1_data = 0;
    dut->idu_cp0_ex1_dst0_reg  = 0;
    dut->idu_cp0_ex1_src0_data = 0;
    dut->idu_cp0_ex1_opcode    = 0;
    dut->lsu_cp0_stb_empty     = 0;             // STB has pending stores
    dut->mmu_cp0_sfence_done   = 0;
    dut->eval();
    check(dut->cp0_mmu_sfence_vld == 0,
          "sfence: no MMU pulse while STB non-empty (quiescence wait)");
    check(dut->cp0_rtu_ex1_cmplt_dp == 0, "sfence: held while STB non-empty");
    tick();
    check(dut->cp0_mmu_sfence_vld == 0, "sfence: still waiting (STB still non-empty)");
    dut->lsu_cp0_stb_empty = 1;                 // STB drains
    dut->eval();
    check(dut->cp0_mmu_sfence_vld == 1,
          "sfence: launches the same cycle the STB empties, no extra delay");
    tick();                                     // SF_IDLE -> SF_WAIT
    dut->mmu_cp0_sfence_done = 1;
    dut->eval();
    tick();                                     // SF_WAIT -> SF_CMPLT
    dut->mmu_cp0_sfence_done = 0;
    dut->eval();
    check(dut->cp0_rtu_ex1_cmplt_dp == 1, "sfence: completes normally after the STB-drain wait");
    tick();
    dut->idu_cp0_ex1_sel = 0;
    test_result("T23b sfence.vma: quiescence wait on lsu_cp0_stb_empty before MMU launch");
}

static void test_sfence_mid_walk_hold(void) {
    // Models the mid-walk hazard resolution (MMU.v's sfence_apply only
    // fires once ptw_st==PTW_IDLE): from CSR.v's side this is just an
    // arbitrarily long mmu_cp0_sfence_done delay -- the sequencer must hold
    // (not time out, not double-pulse) for as long as it takes.
    dut->idu_cp0_ex1_sel       = 1;
    dut->idu_cp0_ex1_func      = CP0_FUNC_SFENCE;
    dut->idu_cp0_ex1_illegal   = 0;
    dut->idu_cp0_ex1_src1_data = 0;
    dut->idu_cp0_ex1_dst0_reg  = 0;
    dut->idu_cp0_ex1_src0_data = 0;
    dut->idu_cp0_ex1_opcode    = 0;
    dut->lsu_cp0_stb_empty     = 1;
    dut->mmu_cp0_sfence_done   = 0;
    dut->eval();
    check(dut->cp0_idu_fencei_full == 1,
          "sfence: cp0_idu_fencei_full asserted (IDU dispatch stall) while sequencer active");
    tick();                                     // SF_IDLE -> SF_WAIT
    for (int i = 0; i < 5; i++) {               // a PTW walk still in flight
        dut->mmu_cp0_sfence_done = 0;
        dut->eval();
        check(dut->cp0_rtu_ex1_cmplt_dp == 0,
              "sfence: hold persists arbitrarily long until MMU ack (mid-walk hazard)");
        check(dut->cp0_idu_fencei_full == 1,
              "sfence: dispatch stays stalled throughout the wait");
        tick();
    }
    dut->mmu_cp0_sfence_done = 1;               // walk finishes, MMU wipes and acks
    dut->eval();
    tick();                                     // SF_WAIT -> SF_CMPLT
    dut->mmu_cp0_sfence_done = 0;
    dut->eval();
    check(dut->cp0_rtu_ex1_cmplt_dp == 1,
          "sfence: completes once the in-flight walk finishes and MMU finally acks");
    tick();
    dut->idu_cp0_ex1_sel = 0;
    test_result("T23c sfence.vma: hold survives a multi-cycle MMU ack delay (mid-walk hazard)");
}

static void test_sfence_tvm_illegal(void) {
    // Reach S-mode: write mstatus.MPP=S(2'b01, bits[12:11]) + TVM=1(bit20)
    // from M-mode, then mret pops pm_r <= mpp_field.
    csr_write(CSR_MSTATUS, (1ULL << 11) | (1ULL << 20));
    DispatchResult r_mret = dispatch(CP0_FUNC_MRET, 0, 0, 0);
    check(r_mret.chgflw, "sfence/tvm setup: mret to S-mode issues chgflw");
    check(dut->cp0_yy_priv_mode == 1, "sfence/tvm setup: now in S-mode (PRIV_S=01)",
          dut->cp0_yy_priv_mode, 1);

    DispatchResult r = dispatch(CP0_FUNC_SFENCE, 0, 0, 0);
    check(r.expt_vld, "sfence: S-mode with TVM=1 raises an exception (illegal instruction)");
    check(r.expt_vec == 2, "sfence: TVM-blocked sfence traps as illegal instruction (vec 2)",
          r.expt_vec, 2);
    check(dut->cp0_mmu_sfence_vld == 0,
          "sfence: TVM-blocked sfence never pulses the MMU (no invalidate on an illegal op)");
    check(!r.chgflw, "sfence: illegal-instruction trap carries no CSR-side chgflw");

    // Restore M-mode for any later test: mret only ever narrows privilege
    // (mret_priv_illegal requires pm_r==M to begin with), so the only way
    // back to M from S is a real trap. medeleg_reg defaults to 0 (never
    // written in this bench), so any vec forces pm_wdata=PRIV_M
    // unconditionally (donor: non-delegated trap always captures to M).
    dut->idu_cp0_ex1_sel    = 0;
    dut->rtu_yy_xx_expt_vld = 1;
    dut->rtu_yy_xx_expt_vec = 2;
    dut->rtu_yy_xx_expt_int = 0;
    dut->eval();
    tick();
    dut->rtu_yy_xx_expt_vld = 0;
    check(dut->cp0_yy_priv_mode == 3, "sfence/tvm teardown: non-delegated trap forces pm_r back to M",
          dut->cp0_yy_priv_mode, 3);
    csr_write(CSR_MSTATUS, (uint64_t)3 << 11);   // MPP=M(11), TVM=0 (clean slate for later tests)
    check(((csr_read(CSR_MSTATUS) >> 20) & 1) == 0, "sfence/tvm teardown: TVM back to 0");
    test_result("T23d sfence.vma: S-mode + TVM=1 traps illegal, no MMU invalidate pulse");
}

static void test_mie_mip_masking(void) {
    dut->mtip = 1; dut->msip = 0; dut->meip = 0;
    uint64_t mip = csr_read(CSR_MIP);
    check(mip == (1ULL << 7), "mip: MTIP alone reflects mtip pin", mip, 1ULL << 7);

    dut->mtip = 0; dut->msip = 1; dut->meip = 0;
    mip = csr_read(CSR_MIP);
    check(mip == (1ULL << 3), "mip: MSIP alone reflects msip pin", mip, 1ULL << 3);

    dut->mtip = 0; dut->msip = 0; dut->meip = 1;
    mip = csr_read(CSR_MIP);
    check(mip == (1ULL << 11), "mip: MEIP alone reflects meip pin", mip, 1ULL << 11);

    dut->mtip = 1; dut->msip = 1; dut->meip = 1;
    mip = csr_read(CSR_MIP);
    check(mip == ((1ULL << 11) | (1ULL << 7) | (1ULL << 3)),
          "mip: all three asserted simultaneously", mip,
          (1ULL << 11) | (1ULL << 7) | (1ULL << 3));

    csr_write(CSR_MIP, 0);   // mip is read-only: this must be silently ignored
    check(csr_read(CSR_MIP) == ((1ULL << 11) | (1ULL << 7) | (1ULL << 3)),
          "mip: write is a no-op (fully read-only)");

    dut->mtip = 0; dut->msip = 0; dut->meip = 0;

    csr_write(CSR_MIE, (1ULL << 7) | (1ULL << 3) | (1ULL << 11));
    check(csr_read(CSR_MIE) == ((1ULL << 11) | (1ULL << 7) | (1ULL << 3)),
          "mie: real R/W flop, stores written value verbatim");
    test_result("T15 mie/mip: mip read-only wires from pins, mie real R/W");
}

static void test_mhcr_fanout(void) {
    uint64_t all_bits = (1ULL << MHCR_IE_BIT) | (1ULL << MHCR_DE_BIT) | (1ULL << MHCR_WA_BIT)
                      | (1ULL << MHCR_RSE_BIT) | (1ULL << MHCR_BPE_BIT) | (1ULL << MHCR_BTBE_BIT);
    csr_write(CSR_MHCR, all_bits);

    check(dut->cp0_ifu_icache_en == 1, "MHCR fan-out: cp0_ifu_icache_en <= MHCR.ie");
    check(dut->cp0_lsu_dcache_en == 1, "MHCR fan-out: cp0_lsu_dcache_en <= MHCR.de");
    check(dut->cp0_lsu_wa == 1, "MHCR fan-out: cp0_lsu_wa <= MHCR.wa");
    check(dut->cp0_ifu_ras_en == 1, "MHCR fan-out: cp0_ifu_ras_en <= MHCR.rse");
    check(dut->cp0_ifu_bht_en == 1, "MHCR fan-out: cp0_ifu_bht_en <= MHCR.bpe");
    check(dut->cp0_ifu_btb_en == 1, "MHCR fan-out: cp0_ifu_btb_en <= MHCR.btbe");

    uint64_t mhcr_val = csr_read(CSR_MHCR);
    check(((mhcr_val >> MHCR_WB_BIT) & 1) == 1, "MHCR readback: wb hardwired 1");
    check(((mhcr_val >> MHCR_WBR_BIT) & 1) == 1, "MHCR readback: wbr hardwired 1");
    check((mhcr_val & 0x77) == (all_bits & 0x77),
          "MHCR readback: the 6 writable bits reflect the write (bit3=WB excluded, hardwired)",
          mhcr_val & 0x77, all_bits & 0x77);

    csr_write(CSR_MHCR, 0);   // clear back down for later tests
    check(dut->cp0_ifu_icache_en == 0, "MHCR fan-out: clears back to 0");

    test_result("T16 MHCR fan-out: 6 writable bits drive IFU/BPU/LSU 1:1, wb/wbr RO");
}

static void test_mxstatus_mm_rw_unconsumed(void) {
    check(dut->cp0_lsu_mm == 1, "MXSTATUS.mm pre-test: still reset value 1");
    csr_write(CSR_MXSTATUS, 0);   // mm <= 0
    check(csr_read(CSR_MXSTATUS) == 0, "MXSTATUS.mm: write 0 reads back 0");
    check(dut->cp0_lsu_mm == 0, "MXSTATUS.mm: cp0_lsu_mm output tracks the flop");
    // "unconsumed by CSR.v itself" (contract 3): CSR.v stores/exports the
    // value but never branches its OWN behavior on it -- there is nothing
    // else in this bench to check beyond store-and-export, which the above
    // already proves.
    csr_write(CSR_MXSTATUS, 1ULL << MXSTATUS_MM);
    check(dut->cp0_lsu_mm == 1, "MXSTATUS.mm: write 1 restores cp0_lsu_mm == 1");
    test_result("T17 MXSTATUS.mm: real R/W flop, value stored+exported, never consumed here");
}

static void test_mtvec_direct_mode_only(void) {
    csr_write(CSR_MTVEC, 0x80005001ULL);   // attempt vectored mode (bit0=1)
    uint64_t mtvec = csr_read(CSR_MTVEC);
    check((mtvec & 0x3) == 0, "mtvec: mode bits forced 0 even though write set bit0",
          mtvec & 0x3, 0);
    check((mtvec & ~0x3ULL) == (0x80005001ULL & ~0x3ULL),
          "mtvec: base bits stored (accepted-but-ignored only on the mode bit)");
    check((uint64_t)dut->cp0_rtu_trap_pc == (mtvec & PC_MASK),
          "mtvec: cp0_rtu_trap_pc mirrors mtvec's direct-mode value",
          dut->cp0_rtu_trap_pc, mtvec & PC_MASK);
    test_result("T18 mtvec: direct-mode only, cp0_rtu_trap_pc tracks it");
}

static void test_mepc_lsb_forced_zero(void) {
    csr_write(CSR_MEPC, 0x80006003ULL);   // odd address
    check((csr_read(CSR_MEPC) & 1) == 0,
          "mepc: LSB forced 0 on software write", csr_read(CSR_MEPC) & 1, 0);
    check((csr_read(CSR_MEPC) & ~1ULL) == (0x80006003ULL & ~1ULL),
          "mepc: remaining bits stored verbatim");

    dut->rtu_yy_xx_expt_vld = 1;
    dut->rtu_yy_xx_expt_int = 0;
    dut->rtu_yy_xx_expt_vec = 2;
    dut->rtu_cp0_epc  = 0x80007005ULL;   // odd trap-entry epc too
    dut->rtu_cp0_tval = 0;
    dut->idu_cp0_ex1_sel = 0;
    dut->eval();
    tick();
    dut->rtu_yy_xx_expt_vld = 0;
    check((csr_read(CSR_MEPC) & 1) == 0,
          "mepc: LSB forced 0 on trap-entry capture too", csr_read(CSR_MEPC) & 1, 0);
    test_result("T19 mepc: LSB forced 0 on both software write and trap-entry capture");
}

static void test_mcycle_free_running(void) {
    // csr_read() itself performs a real dispatch+tick internally (a genuine
    // rs1==x0 CSRRS, which per the fix above does NOT suppress the
    // increment). Between the value c0 samples (combinationally, BEFORE
    // that call's own tick) and the value c1 samples (same: before ITS OWN
    // tick), exactly 11 clock edges elapse: the 1 edge inside c0's own
    // dispatch, plus the 10 idle ticks -- c1's own trailing edge happens
    // AFTER c1 is sampled, so it is not part of this delta.
    uint64_t c0 = csr_read(CSR_MCYCLE);
    for (int i = 0; i < 10; i++) tick_no_dispatch();
    uint64_t c1 = csr_read(CSR_MCYCLE);
    check(c1 > c0, "mcycle: increments every cycle unconditionally", c1, c0);
    check(c1 - c0 == 11, "mcycle: increments by exactly 1/cycle (1 read-dispatch + 10 idle)",
          c1 - c0, 11);

    // A software write fully overrides the counter for that cycle (mutually
    // exclusive with the auto-increment arm, CSR.v's own always-block) --
    // immediately readable as exactly the written value, no extra +1 sneaks
    // in from the write cycle itself.
    csr_write(CSR_MCYCLE, 0x1000ULL);
    uint64_t c2 = csr_read(CSR_MCYCLE);
    check(c2 == 0x1000ULL, "mcycle: software write lands exactly, no extra increment",
          c2, 0x1000ULL);
    test_result("T20 mcycle: free-running, increments every cycle, R/W");
}

static void test_minstret_rw_no_spurious_increment(void) {
    csr_write(CSR_MINSTRET, 0x77ULL);
    uint64_t before = csr_read(CSR_MINSTRET);
    check(before == 0x77ULL, "minstret: write/read-back verbatim", before, 0x77ULL);
    for (int i = 0; i < 20; i++) tick_no_dispatch();
    // Documented interim scope (this file's header + CSR.v's own "MCYCLE /
    // MINSTRET" section comment): the real retire-commit pulse does not
    // exist on CSR.v's port list yet (deferred to plan Task 4 / RTU.v, per
    // Task 1.1's own explicit "confirmed in Task 4, not guessed here").
    // minstret is therefore R/W-correct but must NOT silently auto-
    // increment from cycles passing the way mcycle does -- that is the one
    // property this bench CAN honestly verify today.
    uint64_t after = csr_read(CSR_MINSTRET);
    check(after == before,
          "minstret: does NOT spuriously increment merely from cycles passing "
          "(retire-commit pulse deferred to Task 4 -- see header note)",
          after, before);
    test_result("T21 minstret: R/W correct; auto-increment deferred to Task 4 (documented gap)");
}

static void test_flush_suppresses_dispatch(void) {
    // A dispatch arriving the same cycle rtu_yy_xx_flush_fe/_flush is high
    // must be treated as cancelled -- CSR.v must NOT declare a new
    // exception/changeflow/writeback for it (this file's DECODE section:
    // mirrors the donor's own `!iui_cancel` qualifier).
    dut->rtu_yy_xx_flush_fe = 1;
    DispatchResult r = dispatch(CP0_FUNC_ECALL, 0, 0, 0);
    check(!r.expt_vld, "flush_fe held: an ecall dispatched this cycle is suppressed, no expt_vld");
    dut->rtu_yy_xx_flush_fe = 0;

    dut->rtu_yy_xx_flush = 1;
    DispatchResult r2 = dispatch(CP0_FUNC_MRET, 0, 0, 0);
    check(!r2.chgflw, "flush held: an mret dispatched this cycle is suppressed, no chgflw");
    dut->rtu_yy_xx_flush = 0;

    dut->rtu_yy_xx_flush_fe = 1;
    DispatchResult r3 = dispatch(CP0_FUNC_CSRRW, CSR_MSCRATCH, 0xABCDULL, 1);
    check(!r3.wb_vld, "flush_fe held: a csrrw dispatched this cycle is suppressed, no wb_vld");
    dut->rtu_yy_xx_flush_fe = 0;
    check(csr_read(CSR_MSCRATCH) != 0xABCDULL,
          "flush_fe held: the suppressed csrrw's write never actually landed");

    test_result("T22 rtu_yy_xx_flush_fe/_flush suppress same-cycle CSR dispatch");
}

//=============================================================================
// Mutation-check discipline note (plan task 2.2): the mutation itself is
// applied by hand to rtl/CSR.v (NOT left as code here), the bench re-run to
// confirm a FAIL, then the mutation reverted before committing -- the same
// discipline test/m1/unit/icache_tb.cpp's own development followed. See the
// Task 2 completion report for the exact mutation applied (swapping the
// MIE/MPIE trap-entry assignment order) and the confirmed FAIL it produced
// in test_trap_entry_capture()/test_mret_pop_bit_semantics() before revert.
//=============================================================================

//=============================================================================
// main
//=============================================================================
int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    dut = new VCSR;

    reset_dut();

    test_reset_state();
    test_misa_and_ids_readonly();
    test_csrrw_rmw();
    test_csrrs_rmw();
    test_csrrc_rmw();
    test_csrrwi_rmw();
    test_csrrsi_rmw();
    test_csrrci_rmw();
    test_mret_pop();
    test_mret_pop_bit_semantics();
    test_trap_entry_capture();
    test_trap_entry_mtval_non_allowlist();
    test_trap_entry_int_bit();
    test_ecall_ebreak_illegal();
    test_fetch_fault_priority();
    test_fence_no_op();
    test_sfence_launch_ack_complete();
    test_sfence_stb_wait();
    test_sfence_mid_walk_hold();
    test_sfence_tvm_illegal();
    test_mie_mip_masking();
    test_mhcr_fanout();
    test_mxstatus_mm_rw_unconsumed();
    test_mtvec_direct_mode_only();
    test_mepc_lsb_forced_zero();
    test_mcycle_free_running();
    test_minstret_rw_no_spurious_increment();
    test_flush_suppresses_dispatch();

    printf("[csr_tb] %llu cycles, %d failure(s)\n",
           (unsigned long long)g_cycles, g_fail);
    printf("%s\n", g_fail ? "UNIT-FAIL" : "UNIT-PASS");

    dut->final();
    delete dut;
    return g_fail ? 1 : 0;
}
