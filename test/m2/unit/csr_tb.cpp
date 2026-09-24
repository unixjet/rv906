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
static const uint32_t CSR_MIDELEG   = 0x303;   // M6 Task 1: claim matrix
static const uint32_t CSR_MIE       = 0x304;
static const uint32_t CSR_MTVEC     = 0x305;
static const uint32_t CSR_STVEC     = 0x105;   // M6 Task 1: vectored stvec row
static const uint32_t CSR_MSCRATCH  = 0x340;
static const uint32_t CSR_MEPC      = 0x341;
static const uint32_t CSR_MCAUSE    = 0x342;
static const uint32_t CSR_MTVAL     = 0x343;
static const uint32_t CSR_MIP       = 0x344;
static const uint32_t CSR_SCAUSE    = 0x142;   // M6 Task 1: delegated trap check
static const uint32_t CSR_MCYCLE    = 0xB00;
static const uint32_t CSR_MINSTRET  = 0xB02;
static const uint32_t CSR_TIME      = 0xC01;   // M6 Task 3: CLINT mtime mirror
static const uint32_t CSR_MVENDORID = 0xF11;
static const uint32_t CSR_MARCHID   = 0xF12;
static const uint32_t CSR_MIMPID    = 0xF13;
static const uint32_t CSR_MHARTID   = 0xF14;
static const uint32_t CSR_MXSTATUS  = 0x7C0;
static const uint32_t CSR_MHCR      = 0x7C1;
// M5 Task 1 (rvproc_pkg.sv:327-329) -- the FP CSR file.
static const uint32_t CSR_FFLAGS    = 0x001;
static const uint32_t CSR_FRM       = 0x002;
static const uint32_t CSR_FCSR      = 0x003;

// M7 Task 1 (rvproc_pkg.sv) -- the debug-mode CSR file, storage in DTU.v.
static const uint32_t CSR_DCSR      = 0x7B0;
static const uint32_t CSR_DPC       = 0x7B1;
static const uint32_t CSR_DSCRATCH0 = 0x7B2;
static const uint32_t CSR_DSCRATCH1 = 0x7B3;
// M7 Task 2 (rvproc_pkg.sv) -- the trigger CSR file, storage in DTU.v.
static const uint32_t CSR_TSELECT   = 0x7A0;
static const uint32_t CSR_TDATA1    = 0x7A1;
static const uint32_t CSR_TDATA2    = 0x7A2;
static const uint32_t CSR_TDATA3    = 0x7A3;
static const uint32_t CSR_TINFO     = 0x7A4;
static const uint32_t CSR_TCONTROL  = 0x7A5;
static const uint32_t CSR_MCONTEXT  = 0x7A8;
static const uint32_t CSR_SCONTEXT  = 0x7AA;
// dcsr bit positions (0.13 layout, DTU.v SECTION DCSR):
// [31:28]xdebugver, [15]ebreakm, [13]ebreaks, [12]ebreaku, [11]stepie,
// [10]stopcount, [8:6]cause, [4]mprven, [2]step, [1:0]prv.
static const int  DCSR_STEP      = 2;
static const int  DCSR_MPRVEN    = 4;
static const int  DCSR_CAUSE_LO  = 6;
static const int  DCSR_STOPCOUNT = 10;
static const int  DCSR_STEPIE    = 11;
static const int  DCSR_EBREAKU   = 12;
static const int  DCSR_EBREAKS   = 13;
static const int  DCSR_EBREAKM   = 15;
static const uint64_t DCSR_XDEBUGVER_013 = (0x4ULL << 28);

// mstatus field bit positions used by the M5 FP tests (CSR.v layout):
// [63]SD, [14:13]FS (00=Off,01=Clean,10=Initial,11=Dirty), [12:11]MPP.
static const int MSTATUS_SD    = 63;
static const int MSTATUS_FS_LO = 13;   // LSB of the 2-bit FS field [14:13]
static const uint64_t MSTATUS_MPP_M = (3ULL << 11);   // keep MPP=M across mstatus writes

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
static const uint32_t CP0_FUNC_DRET    = 0x00202;   // M7 Task 1 (rvproc_pkg.sv)

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
    dut->rtu_cp0_fflags         = 0;   // M5 Task 8: no FP-op accrual pending
    dut->rtu_cp0_fs_dirty_updt  = 0;   // M5 Task 8: no FP-instruction retire
    dut->ifu_cp0_icache_inv_done= 0;
    dut->lsu_cp0_stb_empty      = 1;   // LSU quiescent so FENCE/FENCE.I complete
    dut->lsu_cp0_clean_done     = 0;
    dut->mmu_cp0_sfence_done    = 0;
    dut->bht_cp0_inv_done       = 0;
    dut->mtip = 0;
    dut->msip = 0;
    dut->meip = 0;
    dut->mtime = 0;   // M6 Task 3: CLINT mtime mirror (driven per-test below)
    // M7 Task 1: cp0<->DTU debug ports. All tied to "no debugger attached":
    // not in debug mode, no exit-debug, DTU returns reset values, no ebreak
    // action, no WFI wake. Individual debug tests drive these per-test.
    dut->dtu_cp0_rdata          = 0;
    dut->dtu_cp0_dcsr_prv       = 0;
    dut->dtu_cp0_dcsr_mprven    = 0;
    dut->dtu_cp0_wake_up        = 0;
    dut->dtu_rtu_ebreak_action  = 0;
    dut->rtu_yy_xx_dbgon        = 0;
    dut->rtu_cp0_exit_debug     = 0;
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

// M5 Task 8: stand in for RTU's EX2-registered FP-retire pulse (RTU.v's
// ex2_fpu_retire/ex2_fpu_fflags, wired to these donor-named pins in
// RVProc.v; donor aq_rtu_wb.v:268-269). One cycle: assert, commit the flop,
// deassert. Used to simulate "an FP op retired this cycle" without a real
// FPU in this bench's scope.
static void fp_retire(uint32_t flags) {
    dut->rtu_cp0_fflags        = flags & 0x1F;
    dut->rtu_cp0_fs_dirty_updt = 1;
    dut->eval();
    tick();
    dut->rtu_cp0_fs_dirty_updt = 0;
    dut->rtu_cp0_fflags        = 0;
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
    // M6 Task 5 (D-M6-6): IMACFDSU = I|M|A|F|D|C|S|U. The pre-M6 0x..._112C
    // had a bit4/bit5 F/G transposition (set G, omitted F); corrected here.
    check(misa_before == 0x800000000014111DULL,
          "misa: MXL=64,IMACFDSU == 0x800000000014111D (M6 Task 5; F=bit4 not bit5)", misa_before, 0x800000000014111DULL);
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

static void test_csr_qual_illegal_no_wb(void) {
    // M6 Task 7 (Class B, donor aq_cp0_iui.v:809 `!iui_expt_vld &&
    // !iui_cancel`): an instruction that LOOKS like a legal CSR op but
    // fails CSR.v's OWN access qualification must raise the
    // illegal-instruction trap AND suppress the GPR writeback -- the
    // readout must not clobber the destination register. rv64mi-p-csr
    // TEST 14 is the e2e pin (U-mode `csrrw a0, cycle, zero` must leave
    // a0 at its sentinel); this row pins the unit-level contract on both
    // qualification wires that can fire on a well-formed CSR
    // instruction (csr_ro_write, csr_priv_bad).
    //
    // Reach U-mode: MPP=U(00) + mret (mstatus reset is MPP=11, SPP=1).
    csr_write(CSR_MSTATUS, 0);
    DispatchResult r_mret_u = dispatch(CP0_FUNC_MRET, 0, 0, 0);
    check(r_mret_u.chgflw, "qual setup: mret to U-mode issues chgflw");
    check(dut->cp0_yy_priv_mode == 0, "qual setup: now in U-mode (PRIV_U=00)",
          dut->cp0_yy_priv_mode, 0);

    // 1) U-mode RO-write of `cycle` (0xC00): addr[9:8]=00 (user-visible,
    //    priv OK) but addr[11:10]=11 (read-only) -> csr_ro_write.
    DispatchResult r_ro = dispatch(CP0_FUNC_CSRRW, 0xC00, 1, /*dst=*/10);
    check(r_ro.expt_vld && r_ro.expt_vec == 2,
          "U csrrw cycle: csr_ro_write raises illegal-instruction (vec 2)",
          r_ro.expt_vec, 2);
    check(!r_ro.wb_vld,
          "U csrrw cycle: GPR writeback SUPPRESSED (donor :809) -- the "
          "readout must not clobber the destination");
    check(r_ro.cmplt_dp,
          "U csrrw cycle: cmplt_dp still heartbeats (the gate is on "
          "wb_vld only, NOT the retire leg -- RTU must still latch the "
          "trap-bearing EX1 slot)");

    // 2) Contrast: a LEGAL U-mode read of the same CSR (csrrs a0, cycle,
    //    x0: real rs1==x0, no write) must not be over-suppressed.
    DispatchResult r_rd = dispatch(CP0_FUNC_CSRRS, 0xC00, 0, /*dst=*/10,
                                   /*illegal=*/false, /*imm_form=*/false,
                                   /*rs1_reg_field=*/0);
    check(!r_rd.expt_vld, "U csrrs cycle,x0: legal U-mode read, no exception");
    check(r_rd.wb_vld, "U csrrs cycle,x0: writeback still fires (no over-suppression)");

    // Back to M: a non-delegated trap always captures to M (medeleg=0
    // in this bench), same teardown pattern as T23d.
    dut->idu_cp0_ex1_sel    = 0;
    dut->rtu_yy_xx_expt_vld = 1;
    dut->rtu_yy_xx_expt_vec = 2;
    dut->rtu_yy_xx_expt_int = 0;
    dut->eval();
    tick();
    dut->rtu_yy_xx_expt_vld = 0;
    check(dut->cp0_yy_priv_mode == 3, "qual teardown: non-delegated trap back to M",
          dut->cp0_yy_priv_mode, 3);

    // 3) S-mode access to an M-only CSR (mscratch, 0x340: addr[9:8]=11)
    //    -> csr_priv_bad -> illegal, writeback suppressed likewise.
    csr_write(CSR_MSTATUS, (uint64_t)1 << 11);      // MPP=S(01)
    dispatch(CP0_FUNC_MRET, 0, 0, 0);
    check(dut->cp0_yy_priv_mode == 1, "qual setup: now in S-mode (PRIV_S=01)",
          dut->cp0_yy_priv_mode, 1);
    DispatchResult r_priv = dispatch(CP0_FUNC_CSRRW, 0x340, 1, /*dst=*/10);
    check(r_priv.expt_vld && r_priv.expt_vec == 2,
          "S csrrw mscratch: csr_priv_bad raises illegal-instruction (vec 2)",
          r_priv.expt_vec, 2);
    check(!r_priv.wb_vld, "S csrrw mscratch: GPR writeback SUPPRESSED (donor :809)");
    check(r_priv.cmplt_dp, "S csrrw mscratch: cmplt_dp still heartbeats (ungated)");

    // Teardown: trap back to M, MPP restored to M for any later test.
    dut->rtu_yy_xx_expt_vld = 1;
    dut->rtu_yy_xx_expt_vec = 2;
    dut->rtu_yy_xx_expt_int = 0;
    dut->eval();
    tick();
    dut->rtu_yy_xx_expt_vld = 0;
    check(dut->cp0_yy_priv_mode == 3, "qual teardown: back to M", dut->cp0_yy_priv_mode, 3);
    csr_write(CSR_MSTATUS, (uint64_t)3 << 11);      // MPP=M(11), clean slate
    test_result("T13c CSR access qualification: illegal op traps AND suppresses GPR writeback");
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
    // M6 Task 1 REWRITE: mtvec now stores mode bit 0 (donor
    // aq_cp0_trap_csr.v:936-956 -- `mtvec_value = {base, 1'b0, mode[0]}`,
    // only bit 0 architecturally visible; bit 1 accepted-but-masked). The
    // M2-era "mode forced 0" contract this row used to pin is superseded;
    // the vectored redirect itself is exercised in
    // test_int_claim_tvec_vectored() below.
    csr_write(CSR_MTVEC, 0x80005001ULL);   // vectored mode (bit0=1)
    uint64_t mtvec = csr_read(CSR_MTVEC);
    check((mtvec & 0x1) == 1, "mtvec: mode bit 0 now STORED (donor :936-944)",
          mtvec & 0x1, 1);
    check((mtvec & 0x2) == 0, "mtvec: mode bit 1 stays masked at read",
          mtvec & 0x2, 0);
    check((mtvec & ~0x3ULL) == (0x80005001ULL & ~0x3ULL),
          "mtvec: base bits stored verbatim");
    // Direct-mode value on the trap-PC mux: no trap is being taken here, so
    // regs_intr (mcause capture flop) is 0 and the mux stays at plain base
    // regardless of the mode bit.
    check((uint64_t)dut->cp0_rtu_trap_pc == (mtvec & ~0x1ULL & PC_MASK),
          "mtvec: cp0_rtu_trap_pc at plain base while no interrupt is being taken",
          dut->cp0_rtu_trap_pc, mtvec & ~0x1ULL & PC_MASK);
    // Back to direct mode for the later tests.
    csr_write(CSR_MTVEC, 0x80005000ULL);
    check((csr_read(CSR_MTVEC) & 0x3) == 0, "mtvec: back to direct mode");
    test_result("T18 mtvec: mode bit 0 stored (donor :936-956), bit 1 masked, mux direct until intr");
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

static void test_time_csr_d_m8_5(void) {
    // M8 T2 (D-M8-5): the `time` (0xC01) read arm was re-pointed from the
    // CLINT mtime mirror to mcycle_reg -- `time` now advances at exactly the
    // cycle rate. DONOR DEVIATION (the donor's `time` reads the SoC mtime,
    // aq_hpcp_top.v:2655): cross-checking `time`-derived windows against
    // donor cycle counts would carry rtc_tick phase; a per-cycle `time`
    // keeps the M8 parity windows deterministic. The mtime pin is now UNUSED
    // by CSR.v (kept wired for structural fidelity; the CLINT mtime MMIO
    // 0x0200BFF8 is unaffected). This pins the new semantics:
    //  (a) time == mcycle (the same free-running counter as the 0xB00 alias)
    //  (b) time free-runs 1/cycle and does NOT track the mtime pin
    //  (c) driving the mtime pin has NO effect on the time read
    uint64_t t0 = csr_read(CSR_TIME);
    uint64_t m0 = csr_read(CSR_MCYCLE);   // one dispatch edge after t0
    check(m0 == t0 + 1, "time (D-M8-5): same counter as the 0xB00 mcycle alias",
          m0, t0 + 1);

    dut->mtime = 0x1122334455667788ULL;
    uint64_t t1 = csr_read(CSR_TIME);
    check(t1 != 0x1122334455667788ULL,
          "time (D-M8-5): does NOT mirror the driven mtime pin",
          t1, 0x1122334455667788ULL);
    check(t1 == t0 + 2, "time (D-M8-5): advances 1/cycle (2 dispatch edges)",
          t1, t0 + 2);

    for (int i = 0; i < 10; i++) tick_no_dispatch();
    uint64_t t2 = csr_read(CSR_TIME);
    check(t2 - t1 == 11, "time (D-M8-5): per-cycle free-run (1 read + 10 idle)",
          t2 - t1, 11);

    test_result("T27 time (D-M8-5): per-cycle mcycle alias; mtime pin no longer mirrored");
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
// M5 Task 1 (untested until now): fflags/frm/fcsr storage + FS Clean/
// Initial -> Dirty on explicit FP-CSR write.
// M5 Task 8 (D7): sticky OR-in accrual at FP-op retire + FS dirty on
// FP-instruction retire alone + explicit-write-wins priority.
//
// NOTE ON ORDERING: T24a MUST run first (from a clean reset) -- it is the
// OFF-path sanity check and must observe the FP state with the new
// rtu_cp0_* inputs never driven. Every later T24* test sets up its own
// mstatus.FS precondition, so their relative order is free.
//=============================================================================

static void test_fp_offpath_quiescent(void) {
    // From reset, FS==Off(00), so the first legal FP-CSR touch is an
    // mstatus write moving FS to Clean(01) (mstatus itself is not FP-gated).
    // No rtu_cp0_* drive, no FP-CSR write: everything stays at reset.
    csr_write(CSR_MSTATUS, (1ULL << MSTATUS_FS_LO) | MSTATUS_MPP_M);   // FS=01, MPP=M
    check(((csr_read(CSR_MSTATUS) >> MSTATUS_FS_LO) & 3) == 1,
          "offpath: FS=Clean(01) written via mstatus");
    for (int i = 0; i < 10; i++) tick_no_dispatch();
    check(csr_read(CSR_FFLAGS) == 0, "offpath: fflags still 0 (no accrual pulse ever driven)");
    check(csr_read(CSR_FRM) == 0, "offpath: frm still 0");
    check(csr_read(CSR_FCSR) == 0, "offpath: fcsr still 0");
    check(((csr_read(CSR_MSTATUS) >> MSTATUS_FS_LO) & 3) == 1,
          "offpath: FS still Clean(01) -- no FP retire fired, no auto-dirty");
    check((csr_read(CSR_MSTATUS) >> MSTATUS_SD) == 0,
          "offpath: SD (mstatus[63]) still 0");
    test_result("T24a M5 OFF-path: new rtu_cp0_* inputs at 0 leave fflags/frm/fcsr/FS at reset");
}

static void test_fcsr_rw_storage(void) {
    // FS=Initial(10) so FP-CSR access is legal (FS==Off is illegal, T24c).
    csr_write(CSR_MSTATUS, (2ULL << MSTATUS_FS_LO) | MSTATUS_MPP_M);
    check(((csr_read(CSR_MSTATUS) >> MSTATUS_FS_LO) & 3) == 2,
          "setup: FS=Initial(10) written");

    DispatchResult r = dispatch(CP0_FUNC_CSRRW, CSR_FFLAGS, 0x1, /*dst=*/1);
    check(!r.expt_vld, "fflags csrrw: legal while FS!=Off (no illegal-instruction trap)");
    check(csr_read(CSR_FFLAGS) == 0x1, "fflags: 0x1 (NX) stored verbatim",
          csr_read(CSR_FFLAGS), 0x1);

    dispatch(CP0_FUNC_CSRRW, CSR_FRM, 0x3, /*dst=*/0);   // RMM
    check(csr_read(CSR_FRM) == 0x3, "frm: 0x3 (RMM) stored", csr_read(CSR_FRM), 0x3);

    // An fcsr write updates BOTH fields at once (donor
    // aq_cp0_float_csr.v: fcsr_local_en arm in each register's always block).
    // fcsr = {frm[7:5], fflags[4:0]}: frm=RDN(2), fflags=NX(1) -> (2<<5)|1.
    dispatch(CP0_FUNC_CSRRW, CSR_FCSR, (0x2ULL << 5) | 0x1, /*dst=*/0);
    check(csr_read(CSR_FFLAGS) == 0x1, "fcsr write: fflags[4:0] updated (0x1)");
    check(csr_read(CSR_FRM) == 0x2, "fcsr write: frm[7:5] updated (RDN=2)");
    check(csr_read(CSR_FCSR) == 0x41, "fcsr readback == {frm,fflags} == 0x41",
          csr_read(CSR_FCSR), 0x41);
    test_result("T24b fflags/frm/fcsr: basic R/W storage; fcsr write updates both fields");
}

static void test_fs_dirty_on_csr_write(void) {
    // Clean -> Dirty on an explicit FP-CSR write (donor fs_dirty_upd,
    // aq_cp0_trap_csr.v:562-569) + SD aggregation into mstatus[63].
    csr_write(CSR_MSTATUS, (1ULL << MSTATUS_FS_LO) | MSTATUS_MPP_M);   // FS=Clean(01)
    check(((csr_read(CSR_MSTATUS) >> MSTATUS_FS_LO) & 3) == 1, "setup: FS=Clean(01)");
    dispatch(CP0_FUNC_CSRRW, CSR_FFLAGS, 0x0, /*dst=*/0);
    check(((csr_read(CSR_MSTATUS) >> MSTATUS_FS_LO) & 3) == 3,
          "fs: Clean(01)->Dirty(11) on explicit fflags write");
    check((csr_read(CSR_MSTATUS) >> MSTATUS_SD) == 1,
          "sd: mstatus[63]==1 once FS is Dirty");

    // An frm write dirties too.
    csr_write(CSR_MSTATUS, (1ULL << MSTATUS_FS_LO) | MSTATUS_MPP_M);   // back to Clean
    dispatch(CP0_FUNC_CSRRW, CSR_FRM, 0x0, /*dst=*/0);
    check(((csr_read(CSR_MSTATUS) >> MSTATUS_FS_LO) & 3) == 3,
          "fs: Clean->Dirty on frm write");

    // FS==Off: any FP-CSR access is illegal (M5 Task 1 gating, donor
    // aq_cp0_regs.v:1101-1104) and must neither reach storage nor dirty FS.
    csr_write(CSR_MSTATUS, MSTATUS_MPP_M);                            // FS=Off(00)
    DispatchResult r = dispatch(CP0_FUNC_CSRRW, CSR_FFLAGS, 0x4, /*dst=*/0);
    check(r.expt_vld && r.expt_vec == 2,
          "fs off: fflags write traps illegal instruction (vec 2)", r.expt_vec, 2);
    check(((csr_read(CSR_MSTATUS) >> MSTATUS_FS_LO) & 3) == 0,
          "fs off: FS stays Off(00)");
    csr_write(CSR_MSTATUS, (1ULL << MSTATUS_FS_LO) | MSTATUS_MPP_M);  // Clean, un-gate reads
    check(csr_read(CSR_FFLAGS) == 0x0,
          "fs off: the illegal write never reached storage (fflags still 0)");
    test_result("T24c mstatus.FS: Clean->Dirty on FP-CSR write, SD aggregates, FS-off access illegal");
}

static void test_fflags_accrual_sticky(void) {
    // D7: on FP-op retire, fflags <= fflags | retired_fflags (sticky OR,
    // donor aq_cp0_float_csr.v:234-238). Two back-to-back accruals with
    // disjoint flag patterns must ACCUMULATE, not overwrite.
    csr_write(CSR_MSTATUS, (1ULL << MSTATUS_FS_LO) | MSTATUS_MPP_M);  // FS=Clean(01)
    dispatch(CP0_FUNC_CSRRW, CSR_FFLAGS, 0x0, /*dst=*/0);            // known start: 0
    check(csr_read(CSR_FFLAGS) == 0, "accrual setup: fflags cleared to 0");

    fp_retire(0b01000);   // FP op #1 retires with NX
    check(csr_read(CSR_FFLAGS) == 0b01000,
          "accrual 1: fflags == 0 (pre) | 0x8 (retired) == 0x8",
          csr_read(CSR_FFLAGS), 0b01000);

    fp_retire(0b00100);   // FP op #2 retires with OF, back-to-back
    check(csr_read(CSR_FFLAGS) == 0b01100,
          "accrual 2: 0x8 | 0x4 -- sticky OR accumulates (not overwrite)",
          csr_read(CSR_FFLAGS), 0b01100);
    test_result("T24d fflags accrual: sticky OR-in at retire (two back-to-back FP retires)");
}

static void test_fs_dirty_on_fp_retire(void) {
    // The "FS dirty wiring" half: an FP-instruction retire ALONE (no
    // fflags/frm/fcsr write at all) drives Clean/Initial -> Dirty,
    // including the SD aggregation.
    csr_write(CSR_MSTATUS, (1ULL << MSTATUS_FS_LO) | MSTATUS_MPP_M);  // FS=Clean(01)
    check(((csr_read(CSR_MSTATUS) >> MSTATUS_FS_LO) & 3) == 1, "setup: FS=Clean(01)");
    fp_retire(0);
    check(((csr_read(CSR_MSTATUS) >> MSTATUS_FS_LO) & 3) == 3,
          "fs: Clean(01)->Dirty(11) on FP-instruction retire alone (no CSR write)");
    check((csr_read(CSR_MSTATUS) >> MSTATUS_SD) == 1,
          "sd: mstatus[63]==1 after the retire dirties FS");

    // Initial(10) transitions too.
    csr_write(CSR_MSTATUS, (2ULL << MSTATUS_FS_LO) | MSTATUS_MPP_M);  // FS=Initial(10)
    fp_retire(0);
    check(((csr_read(CSR_MSTATUS) >> MSTATUS_FS_LO) & 3) == 3,
          "fs: Initial(10)->Dirty(11) on retire too");

    // Already-Dirty(11) stays Dirty; Off(00) is NEVER auto-dirtied
    // (donor's fs==2'b11 / fs==2'b00 exclusion, aq_cp0_trap_csr.v:562-569).
    fp_retire(0);
    check(((csr_read(CSR_MSTATUS) >> MSTATUS_FS_LO) & 3) == 3,
          "fs: already-Dirty(11) stays Dirty on further retires");
    csr_write(CSR_MSTATUS, MSTATUS_MPP_M);                          // FS=Off(00)
    fp_retire(0);
    check(((csr_read(CSR_MSTATUS) >> MSTATUS_FS_LO) & 3) == 0,
          "fs: Off(00) excluded -- a retire cannot auto-dirty it");
    // (Architecturally an FP op cannot complete while FS==Off -- every FP
    // opcode traps illegal first -- but the RTL exclusion matches the
    // donor and is pinned here.)
    test_result("T24e mstatus.FS: FP-instruction retire alone drives Clean/Initial->Dirty, Off excluded");
}

static void test_fflags_explicit_write_wins(void) {
    // D7 priority: an explicit csrw to fflags in the SAME cycle as a
    // retiring FP op's accrual must WIN (the fflags always block's
    // if-elsif order puts the accrual arm last).
    csr_write(CSR_MSTATUS, (1ULL << MSTATUS_FS_LO) | MSTATUS_MPP_M);  // FS=Clean(01)
    dispatch(CP0_FUNC_CSRRW, CSR_FFLAGS, 0b01010, /*dst=*/0);        // pre: 0x5
    check(csr_read(CSR_FFLAGS) == 0b01010, "setup: fflags pre-armed at 0x5");

    // Same cycle: explicit csrrw fflags=0x2 AND a retire carrying 0x4.
    dut->rtu_cp0_fflags        = 0b00100;
    dut->rtu_cp0_fs_dirty_updt = 1;
    dispatch(CP0_FUNC_CSRRW, CSR_FFLAGS, 0b00010, /*dst=*/0);
    dut->rtu_cp0_fs_dirty_updt = 0;
    dut->rtu_cp0_fflags        = 0;

    check(csr_read(CSR_FFLAGS) == 0b00010,
          "explicit write wins: result == written 0x2, NOT (0x5|0x4)=0x7 (accrual arm lost)",
          csr_read(CSR_FFLAGS), 0b00010);
    check(((csr_read(CSR_MSTATUS) >> MSTATUS_FS_LO) & 3) == 3,
          "fs: dirty either way (the explicit write OR the retire both qualify)");
    test_result("T24f fflags D7 priority: explicit same-cycle csrw beats the accrual OR-in");
}

//=============================================================================
// M6 Task 1: interrupt claim (int_sel[14:0]) + REGISTERED export + vectored
// tvec. CSR.v computes the donor's 15-term claim (aq_cp0_trap_csr.v:1269-
// 1338) and exports it registered + active-low (rv12 template -- see CSR.v's
// cp0_rtu_int_sel port comment for the donor-vs-rv12 decision). The export
// is one cycle behind the combinational claim, so every read here is: drive
// the state, tick, then sample cp0_rtu_int_sel / cp0_rtu_int_b -- exactly
// the values RTU.v consumes once Task 2 wires them.
//
// int_sel bit map pinned by these rows (must match RTU.v's casez arms
// 1:1, which are aq_rtu_int.v:53-74 verbatim):
//   [12] meip (cause 11)  [9] seip nodeleg  [7] stip nodeleg  [3] seip deleg
//   [11] msip (cause 3)   [8] ssip nodeleg  ...                [2] ssip deleg
//   [10] mtip (cause 7)                                        [1] stip deleg
//=============================================================================

// Enter privilege mode pm (3=M, 1=S, 0=U) from M via mstatus.MPP + mret
// (mie/mip/mideleg/mstatus are all M-only CSRs, so every claim cell
// configures from M first). mret pops MIE<=MPIE, so the post-drop MIE rides
// MPIE; SIE is untouched by mret (only sret pops it) and rides the mstatus
// write directly.
static void enter_priv(unsigned pm, bool mie_after, bool sie_after) {
    uint64_t mstatus = ((uint64_t)pm << 11)                 // MPP
                     | (mie_after ? (1ULL << 7) : 0)        // MPIE -> MIE after
                     | (sie_after ? (1ULL << 1) : 0);       // SIE (survives mret)
    csr_write(CSR_MSTATUS, mstatus);
    dispatch(CP0_FUNC_MRET, 0, 0, 0);
}

// Return to M the only way privilege ever widens: a non-delegated trap
// (these rows never write medeleg, so any vec captures to M).
static void return_to_m(void) {
    dut->idu_cp0_ex1_sel    = 0;
    dut->rtu_yy_xx_expt_vld = 1;
    dut->rtu_yy_xx_expt_int = 0;
    dut->rtu_yy_xx_expt_vec = 2;
    dut->eval();
    tick();
    dut->rtu_yy_xx_expt_vld = 0;
    dut->eval();
}

static uint16_t claim_sel(void)      { dut->eval(); return (uint16_t)dut->cp0_rtu_int_sel; }
static bool     claim_active_b(void) { dut->eval(); return dut->cp0_rtu_int_b != 0; }

// One claim source's geometry: mie bit, pending source (mip pin vs mip CSR
// flop), nodeleg/deleg int_sel bit positions, and (S-trio) its mideleg bit.
struct ClaimSrc {
    const char *name;
    unsigned    mie_bit;        // mie_reg bit
    bool        pin;            // true: driven by the mtip/msip/meip pin
    unsigned    pin_idx;        // 0=msip 1=mtip 2=meip (pin sources only)
    unsigned    mip_bit;        // mip flop bit (flop sources only)
    unsigned    nodeleg_bit;    // int_sel bit, nodeleg/M-target slot
    unsigned    deleg_bit;      // int_sel bit, deleg/S-target slot
    unsigned    mideleg_bit;    // mideleg bit (S-trio; 0 for M-trio)
    bool        m_trio;         // M-target source (not delegable)
};

static void set_pin(const ClaimSrc &s, bool v) {
    switch (s.pin_idx) {
        case 0:  dut->msip = v ? 1 : 0; break;
        case 1:  dut->mtip = v ? 1 : 0; break;
        default: dut->meip = v ? 1 : 0; break;
    }
}

// Drive ONE pending+enabled source and check the export for a
// privilege / global-enable / delegation cell.
static void claim_cell(const ClaimSrc &s, unsigned priv, bool en, bool deleg) {
    csr_write(CSR_MIE, 1ULL << s.mie_bit);
    if (s.pin) set_pin(s, true);
    else       csr_write(CSR_MIP, 1ULL << s.mip_bit);
    csr_write(CSR_MIDELEG, deleg ? (1ULL << s.mideleg_bit) : 0);
    enter_priv(priv, en, en);
    tick_no_dispatch();   // settle the registered export on the new pm

    // Expected claim, donor aq_cp0_trap_csr.v:1269-1338 verbatim:
    //  M trio:    pm != M || MIE               (delegation impossible)
    //  S nodeleg: pm==M&&MIE || pm==S || pm==U  (NO SIE term -- donor quirk:
    //             a pending non-delegated source in S targets M unconditionally)
    //  S deleg:   pm==S&&SIE || pm==U           (never claims from M)
    bool expect;
    if (s.m_trio)       expect = (priv != 3) || en;
    else if (priv == 3) expect = !deleg && en;
    else if (priv == 1) expect = deleg ? en : true;
    else                expect = true;   // U: both arms bare
    unsigned expect_bit = s.m_trio ? s.nodeleg_bit
                        : (deleg ? s.deleg_bit : s.nodeleg_bit);

    char what[128];
    uint16_t got = claim_sel();
    uint16_t exp = expect ? (uint16_t)(1u << expect_bit) : 0;
    snprintf(what, sizeof(what),
             "claim %s priv=%u en=%d deleg=%d: int_sel",
             s.name, priv, (int)en, (int)deleg);
    check(got == exp, what, got, exp);
    snprintf(what, sizeof(what),
             "claim %s priv=%u en=%d deleg=%d: active-low export",
             s.name, priv, (int)en, (int)deleg);
    check(claim_active_b() == !expect, what, claim_active_b(), !expect);

    // Teardown: back to M, clear the pending source for the next cell.
    return_to_m();
    if (s.pin) set_pin(s, false);
    else       csr_write(CSR_MIP, 0);
}

static void test_int_claim_matrix(void) {
    // Dark check with nothing pending (the M6 OFF-path invariant: whatever
    // mie holds, no pending source means no claim, active-low stays idle).
    check(claim_sel() == 0, "claim: int_sel dark while nothing pending",
          claim_sel(), 0);
    check(claim_active_b(), "claim: active-low export idle (1) while nothing pending");

    static const ClaimSrc srcs[6] = {
        // name    mie pin idx mip ndel deleg mideleg m_trio
        {"meip",   11, true,  2,  0, 12,  0,  0, true },
        {"msip",    3, true,  0,  0, 11,  0,  0, true },
        {"mtip",    7, true,  1,  0, 10,  0,  0, true },
        {"seip",    9, false, 0,  9,  9,  3,  9, false},
        {"ssip",    1, false, 0,  1,  8,  2,  1, false},
        {"stip",    5, false, 0,  5,  7,  1,  5, false},
    };
    static const unsigned privs[3] = {3, 1, 0};   // M, S, U

    for (int i = 0; i < 6; i++) {
        for (int p = 0; p < 3; p++) {
            for (int e = 0; e < 2; e++) {
                claim_cell(srcs[i], privs[p], e != 0, false);
                if (!srcs[i].m_trio)
                    claim_cell(srcs[i], privs[p], e != 0, true);
            }
        }
    }
    csr_write(CSR_MIDELEG, 0);
    csr_write(CSR_MIE, 0);
    csr_write(CSR_MSTATUS, (uint64_t)3 << 11);
    test_result("T25 M6 claim matrix: 6 sources x M/S/U x deleg x global-en (int_sel + active-low)");
}

static void test_int_claim_tvec_vectored(void) {
    const uint64_t mbase = 0x80005000ULL;
    const uint64_t sbase = 0x80006000ULL;

    // (1) M-mode vectored: mtvec[0]=1 + interrupt cause 3 -> base + 4*3
    //     (donor aq_cp0_trap_csr.v:1355-1359).
    csr_write(CSR_MTVEC, mbase | 1ULL);
    dut->idu_cp0_ex1_sel    = 0;
    dut->rtu_yy_xx_expt_vld = 1;
    dut->rtu_yy_xx_expt_int = 1;
    dut->rtu_yy_xx_expt_vec = 3;              // MSIP cause
    dut->rtu_cp0_epc        = 0x80007000ULL;
    dut->rtu_cp0_tval       = 0;
    dut->eval();
    tick();
    dut->rtu_yy_xx_expt_vld = 0;
    dut->eval();
    check(dut->cp0_rtu_trap_pc == ((mbase + 12) & PC_MASK),
          "vectored: M interrupt cause 3 -> trap_pc = mtvec_base + 12",
          dut->cp0_rtu_trap_pc, (mbase + 12) & PC_MASK);

    // (2) Same vectored mtvec, SYNCHRONOUS exception -> plain base (the
    //     donor's redirect arm is interrupts only, :1358-1359).
    dut->rtu_yy_xx_expt_vld = 1;
    dut->rtu_yy_xx_expt_int = 0;
    dut->rtu_yy_xx_expt_vec = 2;
    dut->eval();
    tick();
    dut->rtu_yy_xx_expt_vld = 0;
    dut->eval();
    check(dut->cp0_rtu_trap_pc == (mbase & PC_MASK),
          "vectored: sync exception ignores the mode bit -> plain base",
          dut->cp0_rtu_trap_pc, mbase & PC_MASK);

    // (3) Direct-mode mtvec (bit0=0) + interrupt -> plain base.
    csr_write(CSR_MTVEC, mbase);
    dut->rtu_yy_xx_expt_vld = 1;
    dut->rtu_yy_xx_expt_int = 1;
    dut->rtu_yy_xx_expt_vec = 7;              // MTIP cause
    dut->eval();
    tick();
    dut->rtu_yy_xx_expt_vld = 0;
    dut->eval();
    check(dut->cp0_rtu_trap_pc == (mbase & PC_MASK),
          "vectored: direct-mode mtvec + interrupt -> plain base",
          dut->cp0_rtu_trap_pc, mbase & PC_MASK);

    // (4) Delegated S-mode interrupt through stvec[0]=1: mideleg[1] (SSIP)
    //     + trap from S with int cause 1 -> stvec_base + 4.
    csr_write(CSR_MIDELEG, 1ULL << 1);
    csr_write(CSR_STVEC, sbase | 1ULL);
    enter_priv(1, false, false);               // S-mode, MIE/SIE off
    dut->rtu_yy_xx_expt_vld = 1;
    dut->rtu_yy_xx_expt_int = 1;
    dut->rtu_yy_xx_expt_vec = 1;               // SSIP cause, delegated
    dut->eval();
    tick();
    dut->rtu_yy_xx_expt_vld = 0;
    dut->eval();
    check(dut->cp0_yy_priv_mode == 1,
          "vectored stvec setup: still S-mode after the delegated trap",
          dut->cp0_yy_priv_mode, 1);
    check(dut->cp0_rtu_trap_pc == ((sbase + 4) & PC_MASK),
          "vectored: delegated S interrupt cause 1 -> trap_pc = stvec_base + 4",
          dut->cp0_rtu_trap_pc, (sbase + 4) & PC_MASK);
    uint64_t scause = csr_read(CSR_SCAUSE);
    check(scause == ((1ULL << 63) | 1ULL),
          "vectored: scause == int|cause 1 for the delegated trap",
          scause, (1ULL << 63) | 1ULL);

    // Teardown: back to M (the delegated trap left us in S), clean state.
    return_to_m();
    csr_write(CSR_MIDELEG, 0);
    csr_write(CSR_MTVEC, 0);
    csr_write(CSR_STVEC, 0);
    csr_write(CSR_MSTATUS, (uint64_t)3 << 11);
    test_result("T26 M6 vectored tvec: intr && tvec[0] -> base+4*cause (mtvec + delegated stvec)");
}

//=============================================================================
// M7 Task 1 -- core-side debug: CSR.v's cp0<->dtu port. The DTU module is
// NOT instantiated in this bench (top-module CSR), so dtu_cp0_rdata is a
// driven input and cp0_dtu_addr/wdata/wreg/rreg are observed outputs. These
// rows test CSR.v's OWN documented contract: (a) the read mux routes
// 0x7B0-0x7B3 to the DTU's rdata bus, (b) a 0x7B0-0x7B3 write strobes
// cp0_dtu_wreg with the right addr/wdata, (c) ebreak becomes a debug HALT
// (cp0_rtu_ebreak_halt, no vec-3) when (dbgon || ebreak_action), (d) dret
// declares cp0_rtu_ex1_inst_dret when dbgon, (e) mret/sret/wfi/ecall are
// gated OFF in debug mode.
//=============================================================================

// Probe a CSRRW write and capture the combinational cp0_dtu_* port the same
// cycle (CSR.v asserts these off the live decode, before any tick).
struct DebugWriteProbe {
    bool     wreg = false;
    uint32_t addr = 0;
    uint64_t wdata = 0;
};
static DebugWriteProbe debug_write_probe(uint32_t csr_addr, uint64_t value) {
    dut->idu_cp0_ex1_sel       = 1;
    dut->idu_cp0_ex1_func      = CP0_FUNC_CSRRW;
    dut->idu_cp0_ex1_illegal   = 0;
    dut->idu_cp0_ex1_src1_data = csr_addr;
    dut->idu_cp0_ex1_src0_data = value;
    dut->idu_cp0_ex1_dst0_reg  = 0;
    dut->eval();
    DebugWriteProbe p;
    p.wreg  = dut->cp0_dtu_wreg != 0;
    p.addr  = dut->cp0_dtu_addr;
    p.wdata = dut->cp0_dtu_wdata;
    tick();
    dut->idu_cp0_ex1_sel = 0;
    return p;
}

static void test_debug_csr_read_routing(void) {
    // Drive the DTU's rdata bus to a sentinel; all four debug CSRs must read
    // it back through the CSR read mux.
    const uint64_t S0 = 0x1111111111111111ULL;
    const uint64_t S1 = 0x2222222222222222ULL;
    dut->dtu_cp0_rdata = S0;
    check(csr_read(CSR_DCSR) == S0, "dcsr read routed to dtu_cp0_rdata", csr_read(CSR_DCSR), S0);
    dut->dtu_cp0_rdata = S1;
    check(csr_read(CSR_DPC) == S1, "dpc read routed to dtu_cp0_rdata", csr_read(CSR_DPC), S1);
    dut->dtu_cp0_rdata = S0;
    check(csr_read(CSR_DSCRATCH0) == S0, "dscratch0 read routed to dtu_cp0_rdata", csr_read(CSR_DSCRATCH0), S0);
    check(csr_read(CSR_DSCRATCH1) == S0, "dscratch1 read routed to dtu_cp0_rdata", csr_read(CSR_DSCRATCH1), S0);

    // A non-debug CSR must NOT read the DTU bus (its own storage wins).
    dut->dtu_cp0_rdata = 0xDEADBEEFDEADBEEFULL;
    csr_write(CSR_MSCRATCH, 0x4242);
    check(csr_read(CSR_MSCRATCH) == 0x4242, "mscratch read NOT hijacked by dtu_cp0_rdata",
          csr_read(CSR_MSCRATCH), 0x4242);
    dut->dtu_cp0_rdata = 0;   // teardown
    test_result("T27 M7 debug CSR read routing: 0x7B0-0x7B3 -> dtu_cp0_rdata");
}

static void test_debug_csr_write_strobe(void) {
    // A write to a debug CSR strobes cp0_dtu_wreg with the right addr/wdata.
    DebugWriteProbe p = debug_write_probe(CSR_DCSR, 0xA5A5);
    check(p.wreg, "dcsr write: cp0_dtu_wreg strobes", p.wreg);
    check(p.addr == CSR_DCSR, "dcsr write: cp0_dtu_addr == 0x7B0", p.addr, CSR_DCSR);
    check(p.wdata == 0xA5A5, "dcsr write: cp0_dtu_wdata == 0xA5A5", p.wdata, 0xA5A5);

    p = debug_write_probe(CSR_DSCRATCH1, 0x1234);
    check(p.wreg && p.addr == CSR_DSCRATCH1 && p.wdata == 0x1234,
          "dscratch1 write: wreg/addr/wdata correct", p.addr, CSR_DSCRATCH1);

    // A write to a NON-debug CSR must NOT strobe the DTU port.
    p = debug_write_probe(CSR_MSCRATCH, 0x99);
    check(!p.wreg, "mscratch write: cp0_dtu_wreg does NOT strobe", p.wreg);
    test_result("T28 M7 debug CSR write strobe: 0x7B0-0x7B3 -> cp0_dtu_wreg/addr/wdata");
}

static void test_debug_ebreak_halt(void) {
    // (1) dbgon=0, action=0 -> classic vec-3 breakpoint, no halt (pre-M7).
    dut->rtu_yy_xx_dbgon = 0;
    dut->dtu_rtu_ebreak_action = 0;
    DispatchResult r1 = dispatch(CP0_FUNC_EBREAK, 0, 0, 0);
    check(r1.expt_vld && r1.expt_vec == 3, "ebreak !dbgon !action: vec-3 exception", r1.expt_vec, 3);
    check(dut->cp0_rtu_ebreak_halt == 0, "ebreak !dbgon !action: NO ebreak_halt");

    // (2) dbgon=1 -> ebreak is a HALT, NOT a vec-3 exception.
    dut->rtu_yy_xx_dbgon = 1;
    DispatchResult r2 = dispatch(CP0_FUNC_EBREAK, 0, 0, 0);
    check(dut->cp0_rtu_ebreak_halt == 1, "ebreak dbgon: cp0_rtu_ebreak_halt fires");
    check(!r2.expt_vld, "ebreak dbgon: NO vec-3 exception (it is a halt)");
    dut->rtu_yy_xx_dbgon = 0;

    // (3) dbgon=0, action=1 (dcsr.ebreakX for current priv) -> also a HALT.
    dut->dtu_rtu_ebreak_action = 1;
    DispatchResult r3 = dispatch(CP0_FUNC_EBREAK, 0, 0, 0);
    check(dut->cp0_rtu_ebreak_halt == 1, "ebreak action: cp0_rtu_ebreak_halt fires");
    check(!r3.expt_vld, "ebreak action: NO vec-3 exception");
    dut->dtu_rtu_ebreak_action = 0;   // teardown
    test_result("T29 M7 ebreak->debug-halt conversion (dbgon || ebreak_action)");
}

static void test_debug_dret_and_xret_gating(void) {
    // dret declares cp0_rtu_ex1_inst_dret ONLY in debug mode.
    dut->rtu_yy_xx_dbgon = 1;
    dispatch(CP0_FUNC_DRET, 0, 0, 0);
    // cp0_rtu_ex1_inst_dret is combinational off is_dret; re-dispatch and read
    // it on the eval cycle (dispatch already ticked, so re-probe).
    dut->idu_cp0_ex1_sel = 1;
    dut->idu_cp0_ex1_func = CP0_FUNC_DRET;
    dut->idu_cp0_ex1_illegal = 0;
    dut->eval();
    check(dut->cp0_rtu_ex1_inst_dret == 1, "dret dbgon: cp0_rtu_ex1_inst_dret fires");
    check(dut->cp0_rtu_ex1_expt_vld == 0, "dret dbgon: no exception");
    dut->idu_cp0_ex1_sel = 0;
    tick();

    // dret OUTSIDE debug: is_dret is gated off (IDU would flag it illegal).
    // With illegal=1 (as IDU sets it) -> vec-2; the dret flag must stay 0.
    dut->rtu_yy_xx_dbgon = 0;
    dut->idu_cp0_ex1_sel = 1;
    dut->idu_cp0_ex1_func = CP0_FUNC_DRET;
    dut->idu_cp0_ex1_illegal = 1;
    dut->eval();
    check(dut->cp0_rtu_ex1_inst_dret == 0, "dret !dbgon: cp0_rtu_ex1_inst_dret gated OFF");
    check(dut->cp0_rtu_ex1_expt_vld == 1 && dut->cp0_rtu_ex1_expt_vec == 2,
          "dret !dbgon illegal: vec-2 (IDU-flagged illegal)", dut->cp0_rtu_ex1_expt_vec, 2);
    dut->idu_cp0_ex1_sel = 0;
    dut->idu_cp0_ex1_illegal = 0;
    tick();

    // mret in debug mode must be gated OFF (no chgflw, no pm pop).
    dut->rtu_yy_xx_dbgon = 1;
    dut->idu_cp0_ex1_sel = 1;
    dut->idu_cp0_ex1_func = CP0_FUNC_MRET;
    dut->idu_cp0_ex1_illegal = 0;
    dut->eval();
    check(dut->cp0_rtu_ex1_chgflw == 0, "mret dbgon: chgflw gated OFF (xret not declared in debug)");
    dut->idu_cp0_ex1_sel = 0;
    tick();
    dut->rtu_yy_xx_dbgon = 0;   // teardown
    test_result("T30 M7 dret declare + xret/ebreak gating in debug mode");
}

// M7 Task 2 -- trigger CSR routing (0x7A0-0x7AA): the read mux arms and the
// write strobe. The DTU module is NOT instantiated in this bench, so this
// tests CSR.v's OWN contract (the trigger CSRs route exactly like the
// 0x7B0-0x7B3 debug CSRs); the DTU-side storage behavior is covered by
// dtu_tb's T13-T16.
static void test_trigger_csr_read_routing(void) {
    const uint64_t S0 = 0x5555555555555555ULL;
    const uint64_t S1 = 0x6666666666666666ULL;
    dut->dtu_cp0_rdata = S0;
    check(csr_read(CSR_TSELECT)  == S0, "tselect read routed to dtu_cp0_rdata", csr_read(CSR_TSELECT), S0);
    check(csr_read(CSR_TDATA1)   == S0, "tdata1 read routed to dtu_cp0_rdata", csr_read(CSR_TDATA1), S0);
    check(csr_read(CSR_TDATA2)   == S0, "tdata2 read routed to dtu_cp0_rdata", csr_read(CSR_TDATA2), S0);
    check(csr_read(CSR_TDATA3)   == S0, "tdata3 read routed to dtu_cp0_rdata", csr_read(CSR_TDATA3), S0);
    check(csr_read(CSR_TINFO)    == S0, "tinfo read routed to dtu_cp0_rdata", csr_read(CSR_TINFO), S0);
    dut->dtu_cp0_rdata = S1;
    check(csr_read(CSR_TCONTROL) == S1, "tcontrol read routed to dtu_cp0_rdata", csr_read(CSR_TCONTROL), S1);
    check(csr_read(CSR_MCONTEXT) == S1, "mcontext read routed to dtu_cp0_rdata", csr_read(CSR_MCONTEXT), S1);
    check(csr_read(CSR_SCONTEXT) == S1, "scontext read routed to dtu_cp0_rdata", csr_read(CSR_SCONTEXT), S1);
    // A non-trigger CSR must NOT read the DTU bus.
    dut->dtu_cp0_rdata = 0xDEADBEEFDEADBEEFULL;
    csr_write(CSR_MSCRATCH, 0x4343);
    check(csr_read(CSR_MSCRATCH) == 0x4343, "mscratch read NOT hijacked by trigger CSR arms",
          csr_read(CSR_MSCRATCH), 0x4343);
    dut->dtu_cp0_rdata = 0;   // teardown
    test_result("T31 M7 trigger CSR read routing: 0x7A0-0x7AA -> dtu_cp0_rdata");
}

static void test_trigger_csr_write_strobe(void) {
    DebugWriteProbe p;
    p = debug_write_probe(CSR_TSELECT, 0x1);
    check(p.wreg && p.addr == CSR_TSELECT && p.wdata == 0x1,
          "tselect write: wreg/addr/wdata correct", p.addr, CSR_TSELECT);
    p = debug_write_probe(CSR_TDATA1, 0x2000000000000044ULL);
    check(p.wreg && p.addr == CSR_TDATA1 && p.wdata == 0x2000000000000044ULL,
          "tdata1 write: wreg/addr/wdata correct", p.wdata, 0x2000000000000044ULL);
    p = debug_write_probe(CSR_TDATA2, 0x80000040);
    check(p.wreg && p.addr == CSR_TDATA2 && p.wdata == 0x80000040,
          "tdata2 write: wreg/addr/wdata correct", p.addr, CSR_TDATA2);
    p = debug_write_probe(CSR_TCONTROL, 0x8);
    check(p.wreg && p.addr == CSR_TCONTROL && p.wdata == 0x8,
          "tcontrol write: wreg/addr/wdata correct", p.addr, CSR_TCONTROL);
    p = debug_write_probe(CSR_MCONTEXT, 0x1FFF);
    check(p.wreg && p.addr == CSR_MCONTEXT && p.wdata == 0x1FFF,
          "mcontext write: wreg/addr/wdata correct", p.addr, CSR_MCONTEXT);
    p = debug_write_probe(CSR_SCONTEXT, 0x3FFFF);
    check(p.wreg && p.addr == CSR_SCONTEXT && p.wdata == 0x3FFFF,
          "scontext write: wreg/addr/wdata correct", p.addr, CSR_SCONTEXT);
    // A non-trigger CSR write must NOT strobe the DTU port.
    p = debug_write_probe(CSR_MSCRATCH, 0x77);
    check(!p.wreg, "mscratch write: cp0_dtu_wreg does NOT strobe for non-trigger CSR", p.wreg);
    test_result("T32 M7 trigger CSR write strobe: 0x7A0-0x7AA -> cp0_dtu_wreg/addr/wdata");
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
    // M5 Task 8: OFF-path sanity check -- must run before any test that
    // drives the new rtu_cp0_* pins or touches the FP CSR state.
    test_fp_offpath_quiescent();
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
    test_csr_qual_illegal_no_wb();
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
    test_time_csr_d_m8_5();
    test_minstret_rw_no_spurious_increment();
    test_flush_suppresses_dispatch();

    // M5 Task 1/8: FP CSR storage + FS dirty tracking + fflags accrual.
    test_fcsr_rw_storage();
    test_fs_dirty_on_csr_write();
    test_fflags_accrual_sticky();
    test_fs_dirty_on_fp_retire();
    test_fflags_explicit_write_wins();

    // M6 Task 1: interrupt claim matrix + vectored tvec.
    test_int_claim_matrix();
    test_int_claim_tvec_vectored();

    // M7 Task 1: core-side debug CSR routing + ebreak/dret/xret gating.
    test_debug_csr_read_routing();
    test_debug_csr_write_strobe();
    test_debug_ebreak_halt();
    test_debug_dret_and_xret_gating();

    // M7 Task 2: trigger CSR (0x7A0-0x7AA) routing to the DTU.
    test_trigger_csr_read_routing();
    test_trigger_csr_write_strobe();

    printf("[csr_tb] %llu cycles, %d failure(s)\n",
           (unsigned long long)g_cycles, g_fail);
    printf("%s\n", g_fail ? "UNIT-FAIL" : "UNIT-PASS");

    dut->final();
    delete dut;
    return g_fail ? 1 : 0;
}
