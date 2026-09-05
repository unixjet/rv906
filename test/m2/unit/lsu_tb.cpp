//=============================================================================
// lsu_tb.cpp - standalone unit bench for rtl/LSU.v (M2 plan task 6.4)
//=============================================================================
// Verilates LSU.v + the DCache.v it instantiates internally + SRAM.v +
// rvproc_pkg.sv (no IDU, no RTU, no CSR, no MMU.v, no SoC) and drives the
// frozen idu_lsu_ex1_*/rtu_lsu_*/cp0_lsu_*/mmu_lsu_* ports directly, plus a
// behavioural AXI D-side slave backed by a golden byte map (an
// unordered_map, not a flat array, since test addresses are deliberately
// scattered across the address space to keep different tests' cache sets
// independent). This is a WHITE-BOX test of LSU.v's OWN stated contract
// (AG/DC/DA, the STB, the single-outstanding-miss + victim-writeback FRZ
// path, the D-side AXI master) run in isolation -- it does not exercise
// IDU's or RTU's real timing, but it DOES verify LSU.v honors the exact
// interlock contract those units are documented (LSU.v's own header) to
// rely on: `idu_lsu_ex1_sel` (not `_dp_sel`) gates every side effect.
//
// Build/run: make -C test/m2/unit lsu && bin/unit/lsu_tb
// Prints one line per test and ends with UNIT-PASS or UNIT-FAIL.
//=============================================================================

#include <verilated.h>
#include "VLSU.h"

#include <cstdio>
#include <cstdint>
#include <cstring>
#include <unordered_map>
#include <unordered_set>

//-----------------------------------------------------------------------------
// FUNC encodings (rvproc_pkg.sv LSU_FUNC_*, confirmed bit-exact).
//-----------------------------------------------------------------------------
static const uint32_t F_LB  = 0x00302;
static const uint32_t F_LH  = 0x00306;
static const uint32_t F_LW  = 0x0030a;
static const uint32_t F_LD  = 0x0030e;
static const uint32_t F_LBU = 0x00300;
static const uint32_t F_LHU = 0x00304;
static const uint32_t F_LWU = 0x00308;
static const uint32_t F_SB  = 0x00301;
static const uint32_t F_SH  = 0x00305;
static const uint32_t F_SW  = 0x00309;
static const uint32_t F_SD  = 0x0030d;
// M5 Task 4c: LOAD-FP/STORE-FP (rvproc_pkg.sv LSU_FUNC_F*) -- reuses the
// same EU_LSU AG/DC/LFB machinery; only the dst0_frf tag (threaded via
// idu_lsu_ex1_dst0_frf/lsu_rtu_wb_dst_frf) and the FLW NaN-boxing formatter
// differ from the plain LW/SW/LD/SD path this bench already exercises.
static const uint32_t F_FLW = 0x00208;
static const uint32_t F_FLD = 0x0020c;
static const uint32_t F_FSW = 0x00209;
static const uint32_t F_FSD = 0x0020d;

//-----------------------------------------------------------------------------
// DUT plumbing
//-----------------------------------------------------------------------------
static VLSU *dut = nullptr;
static uint64_t g_cycles = 0;

static int g_cp0_lsu_mm         = 1;   // reset default per contract 3
static int g_cp0_lsu_wa         = 0;   // reset default per contract 6
static int g_cp0_lsu_dcache_en  = 1;   // this bench always runs "post-boot"
// M3b Task D: MHINT prefetch controls + the IU's EX1 PC broadcast (the PFB
// trainer's PC tag). pref_en defaults to the MHINT reset value (0 = off), so
// every pre-Task-D test runs with the prefetcher fully disabled.
static int g_cp0_lsu_pref_en    = 0;
static int g_cp0_lsu_pref_dist  = 2;   // MHINT reset value
static int g_cp0_lsu_amr        = 0;   // M3b Task E: MHINT.amr reset value
static uint16_t g_ex1_pc        = 0;
// AXI read latency (cycles from AR accept to R valid). Default 2 keeps the
// legacy tests' timing; the non-blocking / hit-under-miss tests raise it so
// a refill is observably "in flight" while other ops proceed.
static int g_axi_rd_lat         = 2;
// M4 Task 5: mmu_lsu_pa_vld stall injection (SECTION AG WAIT-STATE test) --
// while >0, drive_mmu() holds mmu_lsu_pa_vld=0 (a translation-in-flight
// walk) instead of its default always-same-cycle-1 answer; decremented once
// per tick() (not per drive_mmu() call -- drive_mmu() runs twice a cycle).
// Defaults to 0 so every pre-Task-5 test's timing is bit-identical.
static int  g_pa_vld_stall_cycles     = 0;
static bool g_mmu_inject_page_fault   = false;
static bool g_mmu_inject_access_fault = false;
// M4 Task 5: RTU front-end flush broadcast, driven by drive_csr() every
// cycle from this latch -- a test pulses it with flush_pulse() below.
static bool g_flush_fe = false;

//-----------------------------------------------------------------------------
// Golden memory (sparse -- test addresses are deliberately scattered).
//-----------------------------------------------------------------------------
static std::unordered_map<uint64_t, uint8_t> g_mem;

static uint8_t mem_rd(uint64_t addr)
{
    auto it = g_mem.find(addr);
    return (it == g_mem.end()) ? 0 : it->second;
}
static void mem_wr(uint64_t addr, uint8_t v) { g_mem[addr] = v; }

static uint64_t mem_read64(uint64_t addr)
{
    uint64_t v = 0;
    for (int i = 0; i < 8; i++) v |= (uint64_t)mem_rd(addr + i) << (i * 8);
    return v;
}

//-----------------------------------------------------------------------------
// Behavioural AXI D-side slave: AWREADY/WREADY/ARREADY hardwired 1 (no
// backpressure -- this bench is not stress-testing AXI arbitration), a
// fixed 2-cycle latency to BVALID/RVALID, always a full 64B-aligned single
// beat (matches LSU.v's own convention: contract 17's 16-beat cap is
// trivially satisfied, and writes are byte-selected via wstrb).
//-----------------------------------------------------------------------------
struct AxiDSlave {
    bool     b_pending = false; int b_cnt = 0;
    bool     r_pending = false; int r_cnt = 0; uint64_t r_addr = 0;
    int      reads = 0, writes = 0;
    uint64_t last_awaddr = 0, last_araddr = 0;
    // Cycle stamps of the most recent AR/AW acceptance (T10 decoupling check:
    // a decoupled dirty-victim writeback must show the refill READ accepted
    // before the victim WRITE, the reverse of the old inline flow).
    uint64_t last_read_cycle = 0, last_write_cycle = 0;
    // Every accepted AR address (T11: prove prefetch reads land AHEAD of
    // demand -- on lines the demand sequence never asked for).
    std::unordered_set<uint64_t> read_addrs;

    void reset() { b_pending = false; r_pending = false; reads = 0; writes = 0;
                   last_read_cycle = 0; last_write_cycle = 0; read_addrs.clear(); }

    void step(VLSU *d)
    {
        if (!b_pending && d->axi_d_awvalid && d->axi_d_wvalid) {
            uint64_t addr = d->axi_d_awaddr;
            uint64_t strb = d->axi_d_wstrb;
            for (int i = 0; i < 64; i++) {
                if ((strb >> i) & 1ULL) {
                    uint32_t word = d->axi_d_wdata[i / 4];
                    uint8_t byte = (uint8_t)((word >> ((i % 4) * 8)) & 0xFF);
                    mem_wr(addr + i, byte);
                }
            }
            last_awaddr = addr;
            writes++;
            last_write_cycle = g_cycles;
            b_pending = true; b_cnt = 2;
        } else if (b_pending && b_cnt > 0) {
            b_cnt--;
        }

        if (!r_pending && d->axi_d_arvalid) {
            r_addr = d->axi_d_araddr;
            last_araddr = r_addr;
            reads++;
            last_read_cycle = g_cycles;
            read_addrs.insert(r_addr);
            r_pending = true; r_cnt = g_axi_rd_lat;
        } else if (r_pending && r_cnt > 0) {
            r_cnt--;
        }
    }

    void drive(VLSU *d)
    {
        d->axi_d_awready = 1;
        d->axi_d_wready  = 1;
        bool bv = (b_pending && b_cnt == 0);
        d->axi_d_bvalid  = bv ? 1 : 0;
        d->axi_d_bresp   = 0;
        if (bv) b_pending = false;

        d->axi_d_arready = 1;
        bool rv = (r_pending && r_cnt == 0);
        d->axi_d_rvalid  = rv ? 1 : 0;
        d->axi_d_rresp   = 0;
        d->axi_d_rlast   = rv ? 1 : 0;
        if (rv) {
            for (int wi = 0; wi < 16; wi++) {
                uint32_t w = 0;
                for (int b = 0; b < 4; b++)
                    w |= ((uint32_t)mem_rd(r_addr + wi * 4 + b)) << (b * 8);
                d->axi_d_rdata[wi] = w;
            }
            r_pending = false;
        }
    }
};
static AxiDSlave g_slave;

//-----------------------------------------------------------------------------
// MMU stub: identity map + a simplified PMA rule matching contract 5's
// shape (cacheable iff PA >= 0x8000_0000). `lsu_mmu_va` is the PAGE NUMBER
// (LSU.v drives `ag_addr[63:12]`, exactly the donor's own split,
// aq_lsu_ag.v:1566) -- the SAME convention the I-side uses -- so this
// mirrors what a real MMU.v would answer combinationally, same cycle:
// page in, page out, PMA attribute from the page-aligned PA.
//-----------------------------------------------------------------------------
static void drive_mmu(void)
{
    uint64_t ppn = dut->lsu_mmu_va;   // PAGE NUMBER (ag_addr[63:12]), not a byte VA
    // Identity map: page in, page out (rtl/MMU.v D-side, donor aq_lsu_ag.v:201).
    dut->mmu_lsu_pa           = (uint32_t)(ppn & 0x0FFFFFFFULL);
    // PMA cacheability: reconstruct the page-aligned PA and check the
    // DRAM range (contract 5) -- mirrors pma_cacheable() in rtl/MMU.v.
    bool cacheable            = ((ppn << 12) >= 0x0000000080000000ULL);
    dut->mmu_lsu_ca           = cacheable ? 1 : 0;
    dut->mmu_lsu_so           = cacheable ? 0 : 1;
    dut->mmu_lsu_buf          = cacheable ? 1 : 0;
    dut->mmu_lsu_sec          = 0;
    dut->mmu_lsu_sh           = 0;
    // M4 Task 5: pa_vld stall / fault injection (see g_pa_vld_stall_cycles's
    // own header comment). Every pre-Task-5 test leaves both at their
    // defaults (0/false), so this reproduces the old always-1/never-fault
    // body exactly.
    if (g_pa_vld_stall_cycles > 0) {
        dut->mmu_lsu_pa_vld       = 0;
        dut->mmu_lsu_page_fault   = 0;
        dut->mmu_lsu_access_fault = 0;
    } else {
        dut->mmu_lsu_pa_vld       = 1;
        dut->mmu_lsu_page_fault   = g_mmu_inject_page_fault   ? 1 : 0;
        dut->mmu_lsu_access_fault = g_mmu_inject_access_fault ? 1 : 0;
    }
}

static void drive_csr(void)
{
    dut->cp0_lsu_dcache_en = g_cp0_lsu_dcache_en;
    dut->cp0_lsu_mm        = g_cp0_lsu_mm;
    dut->cp0_lsu_wa        = g_cp0_lsu_wa;
    dut->cp0_lsu_dcache_pref_en   = g_cp0_lsu_pref_en;
    dut->cp0_lsu_dcache_pref_dist = g_cp0_lsu_pref_dist;
    dut->cp0_lsu_amr     = g_cp0_lsu_amr;
    dut->iu_lsu_ex1_cur_pc = g_ex1_pc;   // M3b Task D: PFB trainer's PC tag
    dut->rtu_lsu_expt_ack  = 0;
    dut->rtu_lsu_expt_exit = 0;
    dut->rtu_yy_xx_flush_fe = g_flush_fe ? 1 : 0;   // M4 Task 5 (D1)
}

static void idle_issue(void)
{
    dut->idu_lsu_ex1_dp_sel     = 0;
    dut->idu_lsu_ex1_sel        = 0;
    dut->idu_lsu_ex1_raw_vld    = 0;   // M4 Task 5 fix: mirrors sel (see do_op)
    dut->idu_lsu_ex1_func       = 0;
    dut->idu_lsu_ex1_src0_data  = 0;
    dut->idu_lsu_ex1_src0_ready = 1;
    dut->idu_lsu_ex1_src1_data  = 0;
    dut->idu_lsu_ex1_src1_ready = 1;
    dut->idu_lsu_ex1_src2_data  = 0;
    dut->idu_lsu_ex1_src2_ready = 1;
    dut->idu_lsu_ex1_dst0_reg   = 0;
    dut->idu_lsu_ex1_dst0_frf   = 0;
}

static void tick(void)
{
    // ORDER MATTERS (a real bug found while bisecting T4/T5): drive_mmu()
    // reads the DUT's combinational lsu_mmu_va, which only settles to the
    // value implied by THIS cycle's freshly-driven idu_lsu_ex1_* inputs
    // AFTER an eval(). Driving the MMU response before that eval made the
    // stub answer one cycle stale (the PREVIOUS cycle's VA -- usually 0),
    // so every real operation's AG latched ag_pa = {0,12'b0} = 0 and
    // mmu_lsu_ca = 0: all addresses silently squashed to 0 and every
    // "cacheable" access degraded to an uncached direct AXI hit on the
    // line at PA 0. Eval first, then let the stub answer the VA it
    // actually sees.
    drive_csr();
    dut->eval();
    drive_mmu();
    dut->eval();
    g_slave.step(dut);
    dut->clk = 1;
    dut->eval();
    drive_csr();
    drive_mmu();
    dut->eval();
    g_slave.drive(dut);
    dut->eval();
    dut->clk = 0;
    dut->eval();
    g_cycles++;
    // M4 Task 5: decrement once per full tick (drive_mmu() itself runs
    // twice a cycle -- see its own header note), so N settle-ticks of
    // pa_vld=0 means exactly N cycles, not N/2.
    if (g_pa_vld_stall_cycles > 0) g_pa_vld_stall_cycles--;
}

static void settle(int n) { for (int i = 0; i < n; i++) { idle_issue(); tick(); } }

static void reset_dut(void)
{
    dut->clk   = 0;
    dut->rst_n = 0;
    idle_issue();
    drive_csr();
    dut->mmu_lsu_pa = 0; dut->mmu_lsu_pa_vld = 0; dut->mmu_lsu_ca = 0; dut->mmu_lsu_so = 1;
    dut->mmu_lsu_buf = 0; dut->mmu_lsu_sec = 0; dut->mmu_lsu_sh = 0;
    dut->mmu_lsu_page_fault = 0; dut->mmu_lsu_access_fault = 0;
    dut->axi_d_awready = 0; dut->axi_d_wready = 0; dut->axi_d_bvalid = 0; dut->axi_d_bresp = 0;
    dut->axi_d_arready = 0; dut->axi_d_rvalid = 0; dut->axi_d_rresp = 0; dut->axi_d_rlast = 0;
    for (int i = 0; i < 16; i++) dut->axi_d_rdata[i] = 0;
    // M4 Task 5: the PTW servant channel and the flush broadcast -- idle by
    // default, a test drives them directly.
    dut->mmu_lsu_data_req      = 0;
    dut->mmu_lsu_data_req_addr = 0;
    dut->mmu_lsu_data_req_size = 0;
    dut->rtu_yy_xx_flush_fe    = 0;
    g_pa_vld_stall_cycles      = 0;
    g_mmu_inject_page_fault    = false;
    g_mmu_inject_access_fault  = false;
    g_flush_fe                 = false;
    g_slave.reset();
    for (int i = 0; i < 5; i++) tick();
    dut->rst_n = 1;
    for (int i = 0; i < 5; i++) tick();
}

//=============================================================================
// Result bookkeeping
//=============================================================================
static int g_fail  = 0;
static int g_local = 0;

static void check(bool cond, const char *what, uint64_t got = 0, uint64_t exp = 0)
{
    if (!cond) {
        g_local++;
        if (g_fail < 60)
            printf("    FAIL %-64s got=0x%llx exp=0x%llx (cycle %llu)\n", what,
                   (unsigned long long)got, (unsigned long long)exp,
                   (unsigned long long)g_cycles);
        g_fail++;
    }
}

static void test_result(const char *name)
{
    printf("[lsu_tb] %-56s %s\n", name, g_local ? "FAIL" : "PASS");
    g_local = 0;
}

//=============================================================================
// LSU operation driver.
//=============================================================================
struct LsuResult {
    bool     cmplt = false, wb_vld = false, expt_vld = false;
    uint64_t wb_data = 0;
    unsigned wb_preg = 0, expt_vec = 0;
    uint64_t tval = 0;
    int      cycles = 0;
    bool     timed_out = false;
    bool     wb_dst_frf = false;   // M5 Task 4c: lsu_rtu_wb_dst_frf
};

// Issues one EX1 dispatch (a one-cycle pulse, matching how a real IDU
// hands off) and waits for lsu_rtu_ex1_cmplt_dp, capturing the completion
// bus at that exact cycle. `sel`/`dp_sel` are exposed separately so the
// STB-create-vs-flush interlock test can drive the ungated `dp_sel` alone.
static LsuResult do_op(uint32_t func, uint64_t src0, uint64_t src1, uint64_t src2,
                       unsigned dst0, bool sel = true, bool dp_sel = true, int guard = 500,
                       bool dst0_frf = false)
{
    int waited = 0;
    while (dut->lsu_idu_full && waited < guard) { idle_issue(); tick(); waited++; }

    dut->idu_lsu_ex1_dp_sel     = dp_sel ? 1 : 0;
    dut->idu_lsu_ex1_sel        = sel ? 1 : 0;
    // M4 Task 5 fix: `idu_lsu_ex1_raw_vld` is `idu_lsu_ex1_sel` minus its own
    // `!lsu_idu_full` gate (see LSU.v's `ag_raw_ready`) -- since this
    // harness only ever drives `sel` once `lsu_idu_full` has already
    // dropped (the wait loop above), `sel`'s driven value already equals
    // what the real, ungated raw signal would read: mirror it exactly.
    dut->idu_lsu_ex1_raw_vld    = sel ? 1 : 0;
    dut->idu_lsu_ex1_func       = func;
    dut->idu_lsu_ex1_src0_data  = src0;
    dut->idu_lsu_ex1_src0_ready = 1;
    dut->idu_lsu_ex1_src1_data  = src1;
    dut->idu_lsu_ex1_src1_ready = 1;
    dut->idu_lsu_ex1_src2_data  = src2;
    dut->idu_lsu_ex1_src2_ready = 1;
    dut->idu_lsu_ex1_dst0_reg   = dst0;
    dut->idu_lsu_ex1_dst0_frf   = dst0_frf ? 1 : 0;
    tick();
    idle_issue();

    LsuResult r;
    for (int i = 0; i < guard; i++) {
        if (dut->lsu_rtu_ex1_cmplt_dp) {
            r.cmplt      = true;
            r.wb_vld     = dut->lsu_rtu_wb_vld != 0;
            r.wb_data    = dut->lsu_rtu_wb_data;
            r.wb_preg    = dut->lsu_rtu_wb_preg;
            r.expt_vld   = dut->lsu_rtu_expt_vld != 0;
            r.expt_vec   = dut->lsu_rtu_expt_vec;
            r.tval       = dut->lsu_rtu_tval;
            r.cycles     = i + 1;
            r.wb_dst_frf = dut->lsu_rtu_wb_dst_frf != 0;
            tick();
            return r;
        }
        tick();
    }
    r.timed_out = true;
    return r;
}

static inline uint64_t LD(uint64_t addr) { return do_op(F_LD, addr, 0, 0, 5).wb_data; }

// M4 Task 5: like do_op, but correctly models IDU HOLDING the EX1-resident
// operands unchanged across a stall, instead of do_op's single-cycle-sel
// pattern (which only matches a REAL protocol when the op is accepted the
// very cycle it is offered -- true for every op before Task 5, false for a
// DTLB miss). D1: "IDU keeps driving idu_lsu_ex1_ex1_src*_data live from
// EX1 registers" while `adv` is held off by `lsu_idu_full`; sel/dp_sel drop
// (both gated on !lsu_idu_full in the real IDU) but the data does not.
static LsuResult do_op_held(uint32_t func, uint64_t src0, uint64_t src1, uint64_t src2,
                            unsigned dst0, int guard = 500)
{
    int waited = 0;
    while (dut->lsu_idu_full && waited < guard) { idle_issue(); tick(); waited++; }

    dut->idu_lsu_ex1_func       = func;
    dut->idu_lsu_ex1_src0_data  = src0;
    dut->idu_lsu_ex1_src0_ready = 1;
    dut->idu_lsu_ex1_src1_data  = src1;
    dut->idu_lsu_ex1_src1_ready = 1;
    dut->idu_lsu_ex1_src2_data  = src2;
    dut->idu_lsu_ex1_src2_ready = 1;
    dut->idu_lsu_ex1_dst0_reg   = dst0;
    dut->idu_lsu_ex1_dp_sel     = 1;
    dut->idu_lsu_ex1_sel        = 1;
    dut->idu_lsu_ex1_raw_vld    = 1;   // M4 Task 5 fix: mirrors sel below
    tick();

    LsuResult r;
    for (int i = 0; i < guard; i++) {
        // sel/dp_sel drop the moment lsu_idu_full reads 1 (the real gate);
        // every OTHER field (func/src*/dst0) stays exactly as first driven.
        if (dut->lsu_idu_full) {
            dut->idu_lsu_ex1_dp_sel  = 0;
            dut->idu_lsu_ex1_sel     = 0;
            dut->idu_lsu_ex1_raw_vld = 0;
        }
        if (dut->lsu_rtu_ex1_cmplt_dp) {
            r.cmplt    = true;
            r.wb_vld   = dut->lsu_rtu_wb_vld != 0;
            r.wb_data  = dut->lsu_rtu_wb_data;
            r.wb_preg  = dut->lsu_rtu_wb_preg;
            r.expt_vld = dut->lsu_rtu_expt_vld != 0;
            r.expt_vec = dut->lsu_rtu_expt_vec;
            r.tval     = dut->lsu_rtu_tval;
            r.cycles   = i + 1;
            tick();
            idle_issue();
            return r;
        }
        tick();
    }
    r.timed_out = true;
    idle_issue();
    return r;
}

// M4 Task 5: PTW servant probe/answer driver -- asserts group 5's
// mmu_lsu_data_req/_addr/_size (as MMU.v's own walker would) and waits for
// lsu_mmu_data_vld, then drops the request the cycle after (MMU.v's own
// "return to idle is an acknowledge" protocol, SECTION PTW SERVANT).
struct PtwResult {
    bool     cmplt = false, bus_error = false;
    uint64_t data = 0;
    int      cycles = 0;
    bool     timed_out = false;
};

static PtwResult ptw_probe(uint64_t addr, int guard = 200)
{
    int waited = 0;
    while (dut->lsu_idu_full && waited < guard) { idle_issue(); tick(); waited++; }

    dut->mmu_lsu_data_req      = 1;
    dut->mmu_lsu_data_req_addr = addr;
    dut->mmu_lsu_data_req_size = 1;

    PtwResult r;
    for (int i = 0; i < guard; i++) {
        idle_issue();
        tick();
        if (dut->lsu_mmu_data_vld) {
            r.cmplt     = true;
            r.data      = dut->lsu_mmu_data;
            r.bus_error = dut->lsu_mmu_bus_error != 0;
            r.cycles    = i + 1;
            dut->mmu_lsu_data_req = 0;
            tick();
            return r;
        }
    }
    r.timed_out = true;
    dut->mmu_lsu_data_req = 0;
    return r;
}

// Issue one EX1 dispatch and return immediately WITHOUT waiting for
// completion (non-blocking). Used by the hit-under-miss tests to put a
// missing load in flight and then observe other ops proceeding.
static void issue_only(uint32_t func, uint64_t src0, uint64_t src1, uint64_t src2,
                       unsigned dst0, bool dst0_frf = false)
{
    int waited = 0;
    while (dut->lsu_idu_full && waited < 500) { idle_issue(); tick(); waited++; }
    dut->idu_lsu_ex1_dp_sel     = 1;
    dut->idu_lsu_ex1_sel        = 1;
    dut->idu_lsu_ex1_raw_vld    = 1;   // M4 Task 5 fix: mirrors sel
    dut->idu_lsu_ex1_func       = func;
    dut->idu_lsu_ex1_src0_data  = src0;
    dut->idu_lsu_ex1_src0_ready = 1;
    dut->idu_lsu_ex1_src1_data  = src1;
    dut->idu_lsu_ex1_src1_ready = 1;
    dut->idu_lsu_ex1_src2_data  = src2;
    dut->idu_lsu_ex1_src2_ready = 1;
    dut->idu_lsu_ex1_dst0_reg   = dst0;
    dut->idu_lsu_ex1_dst0_frf   = dst0_frf ? 1 : 0;
    tick();
    idle_issue();
}

// Poll lsu_rtu_ex1_cmplt_dp until a completion lands on `preg` (or any preg
// if preg<0), returning the captured bus. guard bounds the wait.
static LsuResult wait_completion(int preg, int guard = 600)
{
    LsuResult r;
    for (int i = 0; i < guard; i++) {
        if (dut->lsu_rtu_ex1_cmplt_dp &&
            (preg < 0 || (unsigned)preg == dut->lsu_rtu_wb_preg)) {
            r.cmplt    = true;
            r.wb_vld   = dut->lsu_rtu_wb_vld != 0;
            r.wb_data  = dut->lsu_rtu_wb_data;
            r.wb_preg  = dut->lsu_rtu_wb_preg;
            r.expt_vld = dut->lsu_rtu_expt_vld != 0;
            r.expt_vec = dut->lsu_rtu_expt_vec;
            r.tval     = dut->lsu_rtu_tval;
            r.cycles   = i + 1;
            r.wb_dst_frf = dut->lsu_rtu_wb_dst_frf != 0;
            tick();
            return r;
        }
        tick();
    }
    r.timed_out = true;
    return r;
}

//=============================================================================
// Tests
//=============================================================================

// T1: load/store byte-width matrix with sign/zero extension.
static void test_byte_width_matrix(void)
{
    const uint64_t A = 0x0000000080010000ULL;

    LsuResult sd = do_op(F_SD, A, 0, 0x1122334455667788ULL, 0);
    check(sd.cmplt && !sd.expt_vld, "SD completes cleanly", sd.cmplt && !sd.expt_vld, 1);
    settle(20);
    LsuResult ld = do_op(F_LD, A, 0, 0, 5);
    check(ld.wb_vld, "LD asserts wb_vld", ld.wb_vld, 1);
    check(ld.wb_data == 0x1122334455667788ULL, "LD reads back the exact SD pattern",
          ld.wb_data, 0x1122334455667788ULL);

    do_op(F_SB, A, 8, 0x00000000000000FFULL, 0);
    settle(20);
    LsuResult lb  = do_op(F_LB,  A, 8, 0, 5);
    LsuResult lbu = do_op(F_LBU, A, 8, 0, 5);
    check(lb.wb_data == 0xFFFFFFFFFFFFFFFFULL, "LB sign-extends 0xFF to -1", lb.wb_data, 0xFFFFFFFFFFFFFFFFULL);
    check(lbu.wb_data == 0x00000000000000FFULL, "LBU zero-extends 0xFF", lbu.wb_data, 0xFFULL);

    do_op(F_SH, A, 16, 0x0000000000008000ULL, 0);
    settle(20);
    LsuResult lh  = do_op(F_LH,  A, 16, 0, 5);
    LsuResult lhu = do_op(F_LHU, A, 16, 0, 5);
    check(lh.wb_data  == 0xFFFFFFFFFFFF8000ULL, "LH sign-extends 0x8000", lh.wb_data, 0xFFFFFFFFFFFF8000ULL);
    check(lhu.wb_data == 0x0000000000008000ULL, "LHU zero-extends 0x8000", lhu.wb_data, 0x8000ULL);

    do_op(F_SW, A, 24, 0x0000000080000000ULL, 0);
    settle(20);
    LsuResult lw  = do_op(F_LW,  A, 24, 0, 5);
    LsuResult lwu = do_op(F_LWU, A, 24, 0, 5);
    check(lw.wb_data  == 0xFFFFFFFF80000000ULL, "LW sign-extends 0x80000000", lw.wb_data, 0xFFFFFFFF80000000ULL);
    check(lwu.wb_data == 0x0000000080000000ULL, "LWU zero-extends 0x80000000", lwu.wb_data, 0x80000000ULL);

    test_result("T1 byte-width matrix: sign/zero extension");
}

// T2: misaligned access always traps, regardless of a scripted
// MXSTATUS.mm value (contract 3).
static void test_misalign_traps(void)
{
    const uint64_t A = 0x0000000080020001ULL;   // misaligned for H/W/D
    for (int mm = 0; mm <= 1; mm++) {
        g_cp0_lsu_mm = mm;
        LsuResult rl = do_op(F_LH, A, 0, 0, 5);
        check(rl.cmplt, "misaligned LH completes (traps, doesn't hang)", rl.cmplt, 1);
        check(rl.expt_vld, "misaligned LH raises an exception", rl.expt_vld, 1);
        check(rl.expt_vec == 4, "misaligned LOAD cause is 4", rl.expt_vec, 4);
        check(!rl.wb_vld, "misaligned load asserts no GPR writeback", rl.wb_vld, 0);
        check(rl.tval == A, "tval is the faulting address", rl.tval, A);

        LsuResult rs = do_op(F_SW, A + 2, 0, 0x1234, 0);   // misaligned word store
        check(rs.expt_vld, "misaligned SW raises an exception", rs.expt_vld, 1);
        check(rs.expt_vec == 6, "misaligned STORE cause is 6", rs.expt_vec, 6);
    }
    g_cp0_lsu_mm = 1;   // restore reset default
    test_result("T2 misaligned access always traps, regardless of MXSTATUS.mm");
}

// T3: STB byte-granular forward -- hit, no-hit, partial-overlap.
static void test_stb_forward(void)
{
    const uint64_t A = 0x0000000080030000ULL;
    do_op(F_SD, A, 0, 0x0000000000000000ULL, 0);
    settle(20);

    do_op(F_SB, A, 8, 0xEE, 0);
    LsuResult noHit = do_op(F_LD, A, 0, 0, 5);
    check(noHit.wb_data == 0, "STB no-hit: an unrelated doubleword's store doesn't forward",
          noHit.wb_data, 0);
    settle(20);

    do_op(F_SD, A, 0, 0x1122334455667788ULL, 0);
    LsuResult fullHit = do_op(F_LD, A, 0, 0, 5);
    check(fullHit.wb_data == 0x1122334455667788ULL, "STB full hit forwards the exact store data",
          fullHit.wb_data, 0x1122334455667788ULL);
    settle(20);

    do_op(F_SB, A, 0, 0x99, 0);
    LsuResult partial = do_op(F_LD, A, 0, 0, 5);
    check((partial.wb_data & 0xFF) == 0x99, "partial overlap: the new byte0 forwards",
          partial.wb_data & 0xFF, 0x99);
    check(((partial.wb_data >> 8) & 0x00FFFFFFFFFFFFFFULL) == (0x1122334455667788ULL >> 8),
          "partial overlap: the other 7 (already-drained) bytes still read the prior SD",
          partial.wb_data >> 8, 0x1122334455667788ULL >> 8);
    settle(20);

    test_result("T3 STB byte-granular forward: hit / no-hit / partial-overlap");
}

// T4: the STB-create-vs-flush interlock (contract 4) -- the exact
// verification the plan calls out by name: a flush arriving the same
// cycle an STB-create would otherwise fire must suppress the create.
// LSU.v's only knob for this in a standalone bench is exactly the shape a
// real same-cycle flush produces in the IDU/RTU integration: idu_lsu_ex1_
// dp_sel (ungated) asserted while idu_lsu_ex1_sel (rtu_idu_commit-gated)
// is held low.
static void test_stb_create_vs_flush_interlock(void)
{
    const uint64_t A = 0x0000000080040000ULL;
    settle(10);
    check(dut->lsu_idu_full == 0, "LSU idle before the interlock probe", dut->lsu_idu_full, 0);

    dut->idu_lsu_ex1_dp_sel     = 1;
    dut->idu_lsu_ex1_sel        = 0;
    dut->idu_lsu_ex1_func       = F_SD;
    dut->idu_lsu_ex1_src0_data  = A;
    dut->idu_lsu_ex1_src0_ready = 1;
    dut->idu_lsu_ex1_src1_data  = 0;
    dut->idu_lsu_ex1_src1_ready = 1;
    dut->idu_lsu_ex1_src2_data  = 0xDEADBEEFDEADBEEFULL;
    dut->idu_lsu_ex1_src2_ready = 1;
    dut->idu_lsu_ex1_dst0_reg   = 0;
    bool full_seen = false;
    for (int i = 0; i < 10; i++) { if (dut->lsu_idu_full) full_seen = true; tick(); }
    check(!full_seen, "dp_sel-alone (sel=0) never even makes LSU busy -- AG never issues",
          full_seen, 0);
    idle_issue();
    settle(5);

    LsuResult after = do_op(F_LD, A, 0, 0, 5);
    check(after.wb_data == 0, "no STB entry (and no DCache write) was created for the "
          "dp_sel-only (flushed) store -- the address still reads as never-written",
          after.wb_data, 0);

    LsuResult real = do_op(F_SD, A, 0, 0xDEADBEEFDEADBEEFULL, 0, /*sel=*/true, /*dp_sel=*/true);
    check(real.cmplt, "the SAME store, issued for real (sel=1), completes normally", real.cmplt, 1);
    settle(20);
    LsuResult confirm = do_op(F_LD, A, 0, 0, 5);
    check(confirm.wb_data == 0xDEADBEEFDEADBEEFULL,
          "and its data IS visible once genuinely issued", confirm.wb_data, 0xDEADBEEFDEADBEEFULL);

    test_result("T4 STB-create-vs-flush interlock: dp_sel alone creates no side effect");
}

// T5: single-outstanding-miss refill end-to-end, including a dirty victim
// being written back FIRST (contract 11).
static void test_miss_refill_dirty_victim_writeback(void)
{
    g_cp0_lsu_wa = 1;   // write-allocate on, so store misses allocate too
    const uint64_t SET_BASE = 0x0000000080052000ULL;
    const uint64_t STRIDE   = 0x2000ULL;   // same set (index), different tag
    uint64_t lines[5];
    for (int i = 0; i < 5; i++) lines[i] = SET_BASE + (uint64_t)i * STRIDE;

    uint64_t patterns[4] = {
        0x1111111111111111ULL, 0x2222222222222222ULL,
        0x3333333333333333ULL, 0x4444444444444444ULL,
    };
    for (int i = 0; i < 4; i++) {
        LsuResult r = do_op(F_SD, lines[i], 0, patterns[i], 0);
        check(r.cmplt && !r.expt_vld, "store to a fresh line completes (cold miss+refill+alloc)",
              r.cmplt && !r.expt_vld, 1);
        settle(25);   // let this store's STB entry actually drain -- the
                       // line must be genuinely dirty in the array before
                       // the next miss's eviction decision reads it.
    }

    int writes_before = g_slave.writes;
    LsuResult r5 = do_op(F_LD, lines[4], 0, 0, 5);
    check(r5.cmplt, "5th (eviction-forcing) load completes", r5.cmplt, 1);
    check(r5.wb_data == 0, "the newly-refilled 5th line reads as zero (never written)",
          r5.wb_data, 0);
    check(g_slave.writes > writes_before, "the eviction produced a real AXI write "
          "(the dirty victim's writeback)", (uint64_t)g_slave.writes, (uint64_t)(writes_before + 1));

    uint64_t got0 = mem_read64(lines[0]);
    check(got0 == patterns[0], "golden memory now holds line0's exact dirty data "
          "(writeback landed BEFORE the new line was read in)", got0, patterns[0]);

    int reads_before = g_slave.reads;
    LsuResult reload = do_op(F_LD, lines[0], 0, 0, 5);
    check(reload.wb_data == patterns[0], "reloading the evicted line reads back correctly from memory",
          reload.wb_data, patterns[0]);
    check(g_slave.reads > reads_before, "reloading line0 required a fresh AXI read (it was truly evicted)",
          (uint64_t)g_slave.reads, (uint64_t)(reads_before + 1));

    g_cp0_lsu_wa = 0;   // restore reset default
    test_result("T5 single-outstanding-miss refill + dirty-victim writeback ordering");
}

// T6 (M3b Task A): hit-under-miss. A missing load A is deferred into the LFB
// and its refill runs in the background; the LSU must FREE UP (non-blocking)
// so a cache-HIT load B can complete while A's refill is still in flight.
// Asserts: (a) the LSU frees shortly after A's miss (NOT held for the whole
// refill -- that is the blocking behavior this replaces), (b) A's refill read
// was actually issued, (c) B completes with correct data before A does, and
// (d) A completes with correct data once its refill lands.
static void test_hit_under_miss(void)
{
    g_cp0_lsu_wa = 0;
    const uint64_t LINE_A = 0x0000000080060000ULL;   // cold line -> A misses
    const uint64_t LINE_B = 0x0000000080061000ULL;   // warmed line -> B hits
    const uint64_t VAL_A  = 0xAAAAAAAAAAAAAAAAULL;
    const uint64_t VAL_B  = 0xBBBBBBBBBBBBBBBBULL;
    for (int i = 0; i < 8; i++) {
        mem_wr(LINE_A + i, (uint8_t)(VAL_A >> (i * 8)));
        mem_wr(LINE_B + i, (uint8_t)(VAL_B >> (i * 8)));
    }

    // Warm B's line so B will be a cache hit.
    LsuResult warm = do_op(F_LD, LINE_B, 0, 0, 6);
    check(warm.cmplt && warm.wb_data == VAL_B, "warm B's line (hit thereafter)",
          warm.cmplt && warm.wb_data == VAL_B, 1);
    settle(5);

    g_axi_rd_lat = 40;   // slow refill so the in-flight window is observable

    // Issue A: cold miss. A non-blocking LSU defers it and frees up fast.
    int reads_before = g_slave.reads;
    issue_only(F_LD, LINE_A, 0, 0, 5);
    int freed_in = 0;
    while (dut->lsu_idu_full && freed_in < 60) { idle_issue(); tick(); freed_in++; }
    bool freed = !dut->lsu_idu_full;
    check(freed && freed_in < 15, "LSU frees quickly after deferring A's miss (non-blocking)",
          (uint64_t)(freed ? freed_in : 999), 15);

    // Issue B: cache hit. Completes while A's refill is still in flight.
    issue_only(F_LD, LINE_B, 0, 0, 6);
    LsuResult b = wait_completion(6, 100);
    check(b.cmplt && b.wb_data == VAL_B, "B (hit) completes while A's refill in flight (hit-under-miss)",
          b.cmplt && b.wb_data == VAL_B, 1);

    // A completes once its refill lands, with correct data.
    LsuResult a = wait_completion(5, 600);
    check(a.cmplt && a.wb_data == VAL_A, "A completes with correct data after refill",
          a.cmplt && a.wb_data == VAL_A, 1);

    // By now A's refill read has definitely been issued to AXI (it follows the
    // dirty-victim peek/writeback, so it is checked here rather than right
    // after issue). Exactly one new read: A's line refill.
    check(g_slave.reads == reads_before + 1, "A's refill read was issued to AXI",
          (uint64_t)g_slave.reads, (uint64_t)(reads_before + 1));

    g_axi_rd_lat = 2;   // restore
    settle(10);
    test_result("T6 hit-under-miss: hit load completes during an outstanding miss refill");
}

// T7 (M3b Task A rework): multi-miss tracking. Two distinct-line misses are
// deferred back-to-back into the now-8-entry LFB. rv906's AXI is genuinely
// single-outstanding (Adaptation Decision #1), so only one entry can ever be
// "active" at a time and activation order == creation order -- completion
// order is therefore forced to equal issue order. Asserts: (a) both misses
// defer and free up quickly (neither blocks on the other), (b) the two
// completions land in issue order (C before D), each with correct data.
static void test_multi_miss_tracking(void)
{
    g_cp0_lsu_wa = 0;
    const uint64_t LINE_C = 0x0000000080070000ULL;
    const uint64_t LINE_D = 0x0000000080071000ULL;
    const uint64_t VAL_C  = 0xCCCCCCCCCCCCCCCCULL;
    const uint64_t VAL_D  = 0xDDDDDDDDDDDDDDDDULL;
    for (int i = 0; i < 8; i++) {
        mem_wr(LINE_C + i, (uint8_t)(VAL_C >> (i * 8)));
        mem_wr(LINE_D + i, (uint8_t)(VAL_D >> (i * 8)));
    }

    g_axi_rd_lat = 40;   // slow refill so both misses are genuinely in flight together

    issue_only(F_LD, LINE_C, 0, 0, 5);
    int freed_c = 0;
    while (dut->lsu_idu_full && freed_c < 60) { idle_issue(); tick(); freed_c++; }
    check(!dut->lsu_idu_full && freed_c < 15, "LSU frees quickly after deferring C (1st miss)",
          (uint64_t)(dut->lsu_idu_full ? 999 : freed_c), 15);

    issue_only(F_LD, LINE_D, 0, 0, 6);
    int freed_d = 0;
    while (dut->lsu_idu_full && freed_d < 60) { idle_issue(); tick(); freed_d++; }
    check(!dut->lsu_idu_full && freed_d < 15,
          "LSU frees quickly after deferring D (2nd miss, while C's refill still in flight)",
          (uint64_t)(dut->lsu_idu_full ? 999 : freed_d), 15);

    LsuResult first = wait_completion(-1, 600);
    check(first.cmplt && first.wb_preg == 5 && first.wb_data == VAL_C,
          "1st completion is C (issue order == completion order, single-outstanding engine)",
          first.cmplt ? first.wb_preg : 999, 5);

    LsuResult second = wait_completion(-1, 600);
    check(second.cmplt && second.wb_preg == 6 && second.wb_data == VAL_D,
          "2nd completion is D, with correct data", second.cmplt ? second.wb_preg : 999, 6);

    g_axi_rd_lat = 2;
    settle(10);
    test_result("T7 multi-miss tracking: two outstanding misses complete in issue order");
}

// T8 (M3b Task A rework): miss-to-in-flight-line replay. A second access to
// the SAME line as an already in-flight miss must not corrupt the first or
// deadlock -- it replays (parks at ST_DCS via lfb_addr_hit, LSU.v's OR-tree
// across all entries, mirroring donor dc_hit_lfb_idx/_addr,
// aq_lsu_lfb.v:722-726) until that entry fully drains, then proceeds.
// Asserts: no deadlock (both complete), correct data for both, and the
// second access's completion is strictly ordered after the first's (proof
// it genuinely stalled/replayed rather than being tracked as a second,
// concurrently-active entry for the same line).
static void test_miss_inflight_replay(void)
{
    g_cp0_lsu_wa = 0;
    const uint64_t LINE_E = 0x0000000080072000ULL;
    const uint64_t VAL_E  = 0xEEEEEEEEEEEEEEEEULL;
    for (int i = 0; i < 8; i++) mem_wr(LINE_E + i, (uint8_t)(VAL_E >> (i * 8)));

    g_axi_rd_lat = 40;   // slow refill so the 2nd access genuinely overlaps the 1st's in-flight miss

    issue_only(F_LD, LINE_E, 0, 0, 5);
    int freed = 0;
    while (dut->lsu_idu_full && freed < 60) { idle_issue(); tick(); freed++; }
    check(!dut->lsu_idu_full, "LSU frees after deferring the 1st access to E",
          dut->lsu_idu_full ? 1 : 0, 0);

    settle(3);   // let the 1st access's entry genuinely start refilling before the 2nd probes the same line
    issue_only(F_LD, LINE_E, 0, 0, 6);

    LsuResult first = wait_completion(-1, 600);
    check(first.cmplt && first.wb_preg == 5 && first.wb_data == VAL_E,
          "1st access to E completes first, with correct data",
          first.cmplt ? first.wb_preg : 999, 5);

    LsuResult second = wait_completion(-1, 600);
    check(second.cmplt && second.wb_preg == 6 && second.wb_data == VAL_E,
          "2nd access to the same in-flight line completes after (replayed), with correct data",
          second.cmplt ? second.wb_preg : 999, 6);

    g_axi_rd_lat = 2;
    settle(10);
    test_result("T8 miss-to-in-flight-line replay: 2nd access to same line stalls then completes correctly");
}

// T9 (M3b Task A rework): LFB-full backpressure. Fill all 8 LFB entries with
// distinct-line misses, then issue a 9th. The 9th must be genuinely HELD
// (lsu_idu_full stays asserted) until a slot frees via drain -- the same
// circular-FIFO lfb_full test as the donor (aq_lsu_lfb.v:718-719) -- not
// silently corrupt or skip ahead. All 9 then complete with correct data.
static void test_lfb_full_backpressure(void)
{
    g_cp0_lsu_wa = 0;
    const int N = 9;
    uint64_t lines[N];
    uint64_t vals[N];
    for (int i = 0; i < N; i++) {
        lines[i] = 0x0000000080080000ULL + (uint64_t)i * 0x1000ULL;
        vals[i]  = 0x1100000000000000ULL * (uint64_t)(i + 1) + (uint64_t)i;
        for (int b = 0; b < 8; b++) mem_wr(lines[i] + b, (uint8_t)(vals[i] >> (b * 8)));
    }

    g_axi_rd_lat = 200;   // service time per entry >> the few cycles needed to issue all 8

    // Fill all 8 LFB slots -- each of the first 8 misses must defer and free
    // quickly (LFB has room), matching T6/T7's non-blocking behavior.
    for (int i = 0; i < 8; i++) {
        issue_only(F_LD, lines[i], 0, 0, (unsigned)(i + 10));
        int freed_in = 0;
        while (dut->lsu_idu_full && freed_in < 60) { idle_issue(); tick(); freed_in++; }
        check(!dut->lsu_idu_full && freed_in < 15,
              "LSU frees quickly deferring miss into an LFB slot (slots 0-7)",
              (uint64_t)(dut->lsu_idu_full ? 999 : freed_in), 15);
    }

    // The 9th miss finds the LFB genuinely full: it must park (lsu_idu_full
    // stays asserted for a while) rather than proceed immediately, until the
    // head entry (slot 0) drains and frees a slot. From here on, poll in a
    // SINGLE continuous loop and record every completion the instant it
    // happens: entries 0-7 keep draining in the background while we wait for
    // the 9th to unpark, so a busy-wait loop followed by separate per-preg
    // wait_completion() calls would miss (swallow) those one-cycle
    // completion pulses -- exactly what the first draft of this test did.
    issue_only(F_LD, lines[8], 0, 0, 18);

    bool     got[N]      = { false };
    uint64_t got_data[N] = { 0 };
    int      held_for    = -1;
    for (int cyc = 0; cyc < 6000; cyc++) {
        if (held_for < 0 && !dut->lsu_idu_full) held_for = cyc;
        if (dut->lsu_rtu_ex1_cmplt_dp) {
            unsigned preg = dut->lsu_rtu_wb_preg;
            for (int i = 0; i < N; i++) {
                unsigned want = (unsigned)(i < 8 ? i + 10 : 18);
                if (preg == want && !got[i]) { got[i] = true; got_data[i] = dut->lsu_rtu_wb_data; }
            }
        }
        idle_issue();
        tick();
        bool all_done = true;
        for (int i = 0; i < N; i++) if (!got[i]) { all_done = false; break; }
        if (all_done) break;
    }

    check(held_for >= 0, "the 9th miss eventually unparks once a slot drains",
          held_for < 0 ? 1 : 0, 0);
    check(held_for > 30, "the 9th miss was genuinely held back (LFB-full backpressure), "
          "not deferred immediately like the first 8", (uint64_t)(held_for < 0 ? 0 : held_for), 31);

    // All 9 complete with correct data.
    for (int i = 0; i < N; i++) {
        check(got[i] && got_data[i] == vals[i], "each of the 9 misses completes with correct data",
              got[i] ? got_data[i] : 0xdeadULL, vals[i]);
    }

    g_axi_rd_lat = 2;
    settle(10);
    test_result("T9 LFB-full backpressure: 9th miss held until a slot frees, all 9 complete correctly");
}

// T10 (M3b Task B/C): single-entry victim buffer decouples the dirty-victim
// writeback from the refill (donor aq_lsu_rdl.v CHECK->WVB + aq_lsu_vb.v
// :135-148). A load that misses on a set whose 4 ways all hold DIRTY lines
// must evict one of them. Asserts:
//   (a) DECOUPLING: the refill READ is accepted by the slave BEFORE the
//       victim writeback is even accepted -- the old inline flow issued the
//       write first and held the refill until the writeback's B response, so
//       this ordering is the observable signature of the VB handoff;
//   (b) exactly one writeback + one refill read for the one eviction;
//   (c) the evicted line's dirty data reaches golden memory intact;
//   (d) a reload of the evicted address misses again (fresh AXI read) and
//       returns the written-back value.
// Uses set index 1 (base bit [12:6] = 1) so the setup starts from an empty
// set regardless of what earlier tests left in index 0.
static void test_vb_decoupled_dirty_writeback(void)
{
    g_cp0_lsu_wa = 1;   // write-allocate so the setup stores allocate + dirty
    const uint64_t SET_BASE = 0x0000000080090040ULL;   // index 1, clean slate
    const uint64_t STRIDE   = 0x2000ULL;               // same index, new tag
    uint64_t lines[5];
    for (int i = 0; i < 5; i++) lines[i] = SET_BASE + (uint64_t)i * STRIDE;

    uint64_t patterns[4] = {
        0xA5A5A5A5A5A5A5A5ULL, 0x5A5A5A5A5A5A5A5AULL,
        0x0F0F0F0F0F0F0F0FULL, 0xF0F0F0F0F0F0F0F0ULL,
    };
    for (int i = 0; i < 4; i++) {
        LsuResult r = do_op(F_SD, lines[i], 0, patterns[i], 0);
        check(r.cmplt && !r.expt_vld, "setup store dirties a line (cold miss+refill+alloc)",
              r.cmplt && !r.expt_vld, 1);
        settle(25);   // let each store drain so the line is genuinely dirty
                      // in the array before the eviction decision reads it
    }
    int leaked = 0;
    for (int i = 0; i < 4; i++) if (mem_read64(lines[i]) == patterns[i]) leaked++;
    check(leaked == 0, "all 4 dirty lines still only in the cache (no premature writeback)",
          (uint64_t)leaked, 0);

    int reads_before  = g_slave.reads;
    int writes_before = g_slave.writes;

    LsuResult r5 = do_op(F_LD, lines[4], 0, 0, 5);
    check(r5.cmplt, "eviction-forcing load completes", r5.cmplt, 1);
    check(r5.wb_data == 0, "the newly-refilled line reads as zero (never written)",
          r5.wb_data, 0);
    settle(30);   // let the VB drain fully (writeback retires on B response)

    check(g_slave.writes == writes_before + 1, "exactly one writeback for the one eviction",
          (uint64_t)g_slave.writes, (uint64_t)(writes_before + 1));
    check(g_slave.reads == reads_before + 1, "exactly one refill read for the miss",
          (uint64_t)g_slave.reads, (uint64_t)(reads_before + 1));
    check(g_slave.last_read_cycle < g_slave.last_write_cycle,
          "DECOUPLED: refill read accepted BEFORE the victim writeback "
          "(old inline flow wrote back first, blocking the refill)",
          g_slave.last_read_cycle, g_slave.last_write_cycle);

    // Exactly one of the four dirty lines landed in golden memory -- the
    // evicted one (which way the round-robin picked is deliberately not
    // assumed here).
    int evicted = -1;
    for (int i = 0; i < 4; i++) if (mem_read64(lines[i]) == patterns[i]) evicted = i;
    check(evicted >= 0, "the evicted line's dirty data reached memory intact",
          (uint64_t)(evicted >= 0 ? 1 : 0), 1);

    if (evicted >= 0) {
        int reads_before2 = g_slave.reads;
        LsuResult reload = do_op(F_LD, lines[evicted], 0, 0, 5);
        check(reload.wb_data == patterns[evicted],
              "reloading the evicted address returns the written-back value",
              reload.wb_data, patterns[evicted]);
        check(g_slave.reads == reads_before2 + 1,
              "the reload missed again (line was truly evicted, fresh AXI read)",
              (uint64_t)g_slave.reads, (uint64_t)(reads_before2 + 1));
    }

    g_cp0_lsu_wa = 0;   // restore reset default
    test_result("T10 VB decoupling: refill read ahead of dirty-victim writeback");
}

// T11 (M3b Task D): PFB stride prefetch (donor aq_lsu_pfb_top/aq_lsu_pfb,
// gated by MHINT.pref_en). With prefetch enabled, a striding load sequence
// (same PC, +1-line stride) trains a PC entry and, once confirmed, the PFB
// must prefetch AHEAD of demand:
//   (a) AXI reads appear for lines the demand sequence never requested
//       (past the last demanded line),
//   (b) a later demand load to an already-prefetched line completes as a
//       cache HIT (correct data, no new AXI read),
//   (c) with pref_en=0 the SAME sequence issues exactly the demand reads --
//       no prefetch traffic at all.
static void test_pfb_stride_prefetch(void)
{
    g_cp0_lsu_pref_en   = 1;
    g_cp0_lsu_pref_dist = 2;      // reset default: lookahead = stride << 2
    g_ex1_pc            = 0x1234; // all training loads share one PC

    const uint64_t BASE   = 0x00000000800A0800ULL;   // sets 32..47 (unused so far)
    const int NLINES = 16;
    uint64_t line[NLINES];
    for (int i = 0; i < NLINES; i++) {
        line[i] = BASE + (uint64_t)i * 64;
        uint64_t pat = 0xA5A5A5A5A5A50000ULL | (uint64_t)i;
        for (int b = 0; b < 8; b++) mem_wr(line[i] + b, (uint8_t)(pat >> (b * 8)));
    }

    // ---- Phase A: pref_en=1 -- demand loads 0..7, prefetch runs ahead ----
    std::unordered_set<uint64_t> reads_before = g_slave.read_addrs;
    for (int i = 0; i < 8; i++) {
        LsuResult r = do_op(F_LD, line[i], 0, 0, 5);
        check(r.cmplt && r.wb_data == (0xA5A5A5A5A5A50000ULL | (uint64_t)i),
              "demand load completes with correct data (prefetch enabled)",
              r.wb_data, 0xA5A5A5A5A5A50000ULL | (uint64_t)i);
        settle(6);
    }
    settle(120);   // let every in-flight prefetch fill drain

    // At least one AXI read landed PAST the demand sequence's last line.
    bool ahead = false;
    uint64_t ahead_addr = 0;
    for (uint64_t a : g_slave.read_addrs) {
        if (reads_before.count(a)) continue;          // only this phase's reads
        if (a >= line[8] && a < BASE + (uint64_t)NLINES * 64) { ahead = true; ahead_addr = a; }
    }
    check(ahead, "prefetcher read at least one line AHEAD of the demand sequence",
          (uint64_t)ahead, 1);

    // ---- Phase B: demand load to an already-prefetched line is a HIT ----
    if (ahead) {
        int reads_before_hit = g_slave.reads;
        LsuResult h = do_op(F_LD, ahead_addr, 0, 0, 5);
        int idx = (int)((ahead_addr - BASE) / 64);
        check(h.cmplt && h.wb_data == (0xA5A5A5A5A5A50000ULL | (uint64_t)idx),
              "demand load to a prefetched line returns correct data",
              h.wb_data, 0xA5A5A5A5A5A50000ULL | (uint64_t)idx);
        check(g_slave.reads == reads_before_hit,
              "... and it was a cache HIT (no new AXI read needed)",
              (uint64_t)g_slave.reads, (uint64_t)reads_before_hit);
        settle(10);
    }

    // ---- Phase C: pref_en=0 -- identical sequence, zero prefetch ----
    g_cp0_lsu_pref_en = 0;
    settle(10);   // clearing pref_en flushes the PFB (donor pfb_top.v:363-367)
    // In-flight prefetch LFB entries survive that flush and drain their
    // refills (the donor kills the PFB trackers, not pending fills) -- and
    // Phase B's own load retrained the still-enabled PFB one last time, so
    // wait long enough for every queued entry to drain (they activate in
    // FIFO order, ~13 cycles apart -- a flat 300 covers a full LFB) before
    // counting.
    settle(300);

    const uint64_t BASE2 = 0x00000000800A1000ULL;    // sets 64..71 (clean)
    for (int i = 0; i < 8; i++) {
        uint64_t l2 = BASE2 + (uint64_t)i * 64;
        uint64_t pat = 0x5A5A5A5A5A5A0000ULL | (uint64_t)i;
        for (int b = 0; b < 8; b++) mem_wr(l2 + b, (uint8_t)(pat >> (b * 8)));
    }
    int reads_phase_c = g_slave.reads;
    for (int i = 0; i < 8; i++) {
        LsuResult r = do_op(F_LD, BASE2 + (uint64_t)i * 64, 0, 0, 5);
        check(r.cmplt && r.wb_data == (0x5A5A5A5A5A5A0000ULL | (uint64_t)i),
              "pref_en=0: demand load completes with correct data",
              r.wb_data, 0x5A5A5A5A5A5A0000ULL | (uint64_t)i);
        settle(6);
    }
    settle(40);
    check(g_slave.reads == reads_phase_c + 8,
          "pref_en=0: exactly the 8 demand reads -- no prefetch traffic",
          (uint64_t)g_slave.reads, (uint64_t)(reads_phase_c + 8));

    g_ex1_pc = 0;
    test_result("T11 PFB stride prefetch: trains, prefetches ahead, hits; off=quiet");
}

// T12 (M3b Task E): AMR streaming-store write-allocate disabler (donor
// aq_lsu_amr.v, gated by MHINT.amr). A contiguous streaming-sd sequence
// (stride 8 == store size) trains the detector; once confirmed (amr=2'b01:
// 4 lines), write-allocate is disabled and subsequent store misses write
// straight through WITHOUT allocating:
//   (a) with amr=2'b01 only the training window's lines are allocated
//       (exactly one refill read per allocated line, 5 lines),
//   (b) a line inside the allocated region hits with correct data,
//   (c) a line past the training window was never allocated: a load there
//       misses (fresh read) but still returns the direct-written data,
//   (d) with amr=0 the SAME sequence allocates every line (10 reads).
static void test_amr_streaming_store(void)
{
    g_cp0_lsu_pref_en = 0;      // PFB off: stores don't train it anyway
    g_cp0_lsu_wa      = 1;      // write-allocate on, so AMR has something to disable
    g_cp0_lsu_amr     = 1;      // 2'b01: 4-line confirmation threshold
    g_ex1_pc          = 0x5678;

    const uint64_t BASE3  = 0x00000000800C0000ULL;
    const int NSTORES = 80;     // 10 lines of sd (8 stores per line)
    // No golden-memory pre-write: the stores themselves must populate the
    // data (allocated lines via refill+merge, non-allocated lines via the
    // AMR direct write), so a broken write path cannot hide behind a stale
    // pre-seeded value.

    // ---- amr=2'b01: only the training window allocates ----
    int reads_before = g_slave.reads;
    for (int i = 0; i < NSTORES; i++) {
        LsuResult r = do_op(F_SD, BASE3 + (uint64_t)i * 8, 0,
                            0x1111111111111111ULL + (uint64_t)i, 0);
        check(r.cmplt && !r.expt_vld, "streaming sd completes (amr on)",
              r.cmplt && !r.expt_vld, 1);
        settle(3);
    }
    settle(60);
    // amr=2'b01: line_cnt_done fires once line_cnt reaches 3 (donor
    // threshold, amr.v:281,289) -- after the 3rd line-completion event
    // (store 25), so the FSM is in FUNC from store 26 onward. First-of-line
    // stores 0, 8, 16, 24 (lines 0..3) all miss while still in CALS/CHCK
    // and allocate; line 4's first store (32) already sees FUNC and writes
    // straight through -> exactly 4 refill reads.
    check(g_slave.reads == reads_before + 4,
          "amr on: only the training window's 4 lines were allocated",
          (uint64_t)g_slave.reads, (uint64_t)(reads_before + 4));

    // Allocated region: line 2 hits with correct data, no new read.
    int rb = g_slave.reads;
    LsuResult h = do_op(F_LD, BASE3 + 16 * 8, 0, 0, 5);   // line 2, dword 0
    check(h.cmplt && h.wb_data == 0x1111111111111111ULL + 16,
          "allocated line hits with correct data",
          h.wb_data, 0x1111111111111111ULL + 16);
    check(g_slave.reads == rb, "... as a cache HIT (no read)",
          (uint64_t)g_slave.reads, (uint64_t)rb);
    settle(5);

    // Non-allocated region: line 7 missed the allocation window; a load
    // misses again (fresh read) but returns the direct-written data.
    rb = g_slave.reads;
    LsuResult m = do_op(F_LD, BASE3 + 56 * 8, 0, 0, 5);   // line 7, dword 0
    check(m.cmplt && m.wb_data == 0x1111111111111111ULL + 56,
          "non-allocated line still returns the direct-written data",
          m.wb_data, 0x1111111111111111ULL + 56);
    check(g_slave.reads == rb + 1,
          "... via a fresh read (line was never allocated)",
          (uint64_t)g_slave.reads, (uint64_t)(rb + 1));
    settle(5);

    // ---- amr=0 control: identical sequence allocates every line ----
    g_cp0_lsu_amr = 0;
    settle(10);
    const uint64_t BASE4 = 0x00000000800D0000ULL;
    reads_before = g_slave.reads;
    for (int i = 0; i < NSTORES; i++) {
        LsuResult r = do_op(F_SD, BASE4 + (uint64_t)i * 8, 0,
                            0x2222222222222222ULL + (uint64_t)i, 0);
        check(r.cmplt && !r.expt_vld, "streaming sd completes (amr off)",
              r.cmplt && !r.expt_vld, 1);
        settle(3);
    }
    settle(60);
    check(g_slave.reads == reads_before + 10,
          "amr off: all 10 lines allocated (one read each)",
          (uint64_t)g_slave.reads, (uint64_t)(reads_before + 10));

    g_cp0_lsu_wa  = 0;   // restore reset defaults
    g_cp0_lsu_amr = 0;
    g_ex1_pc      = 0;
    test_result("T12 AMR streaming stores: wa disabled after training; off=allocate");
}

// T13 (M3b Task F): STB-forward-under-miss. A store to a line whose miss is
// still in flight parks behind the deferred load's LFB entry (replay, donor
// lfb.v:722-726), the deferred load completes with the refill data, then
// the store retries, refills and lands in the STB -- and a subsequent load
// returns the stored data (STB forward / drained array), all with exactly
// the expected bus traffic.
static void test_stb_forward_under_miss(void)
{
    g_cp0_lsu_pref_en = 0;
    g_cp0_lsu_wa      = 1;
    const uint64_t LINE = 0x00000000800E0000ULL;   // cold line, set 0 tag space
    const uint64_t SVAL = 0xFEEDFACEFEEDFACEULL;

    int reads_before = g_slave.reads;

    // Load misses and defers into the LFB (non-blocking).
    issue_only(F_LD, LINE, 0, 0, 5);

    // Store to the SAME line while the miss is in flight: it probes, misses,
    // sees the in-flight LFB entry (lfb_addr_hit) and parks behind it.
    issue_only(F_SD, LINE, 0, SVAL, 0);

    // The deferred load completes first, with memory's (zero) data.
    LsuResult a = wait_completion(5, 600);
    check(a.cmplt && a.wb_data == 0,
          "deferred load completes with refill data while store parked",
          a.wb_data, 0);

    // The parked store retries once the entry drains: refills the line
    // (redundant but correct -- replay-not-merge) and lands in the STB.
    LsuResult s = wait_completion(0, 600);
    check(s.cmplt, "parked store retries and completes after the entry drains",
          (uint64_t)s.cmplt, 1);

    // Load again: the stored data must be visible (STB forward or drained
    // array), with no further bus read.
    int rb = g_slave.reads;
    LsuResult b = do_op(F_LD, LINE, 0, 0, 5);
    check(b.cmplt && b.wb_data == SVAL,
          "subsequent load returns the store's data (forward under miss)",
          b.wb_data, SVAL);
    check(g_slave.reads == rb, "... as a hit (no new read)",
          (uint64_t)g_slave.reads, (uint64_t)rb);

    // Exactly two reads for the whole sequence: the load's refill and the
    // store's retry refill.
    check(g_slave.reads == reads_before + 2,
          "exactly two refill reads (load miss + store retry)",
          (uint64_t)g_slave.reads, (uint64_t)(reads_before + 2));

    settle(20);
    g_cp0_lsu_wa = 0;
    test_result("T13 STB forward under miss: store parks, retries, data visible");
}

// T14 (M4 Task 5, D1): a DTLB miss (mmu_lsu_pa_vld=0 for N cycles) parks
// the op via ag_wait_r instead of entering the DC pipe; once the walk lands
// (pa_vld=1, same VA held throughout by IDU's own EX1-register hold,
// modeled here by do_op_held), the op issues and completes exactly as it
// would have on an immediate answer.
static void test_ag_wait_state_dtlb_miss(void)
{
    const uint64_t A = 0x00000000800F1000ULL;   // fresh line, unused elsewhere

    g_pa_vld_stall_cycles = 4;
    LsuResult sd = do_op_held(F_SD, A, 0, 0xCAFEBABECAFEBABEULL, 0);
    check(sd.cmplt && !sd.expt_vld, "SD completes after a 4-cycle DTLB-miss wait",
          sd.cmplt && !sd.expt_vld, 1);
    check((uint64_t)sd.cycles >= 4, "completion took at least the stalled cycles",
          (uint64_t)sd.cycles, 4);
    settle(20);

    g_pa_vld_stall_cycles = 3;
    LsuResult ld = do_op_held(F_LD, A, 0, 0, 5);
    check(ld.cmplt && !ld.expt_vld && ld.wb_data == 0xCAFEBABECAFEBABEULL,
          "LD after its own DTLB-miss wait reads back the exact SD pattern",
          ld.wb_data, 0xCAFEBABECAFEBABEULL);
    settle(20);
    test_result("T14 AG wait-state: DTLB miss parks the op, issues once pa_vld lands");
}

// T15 (M4 Task 5, S12): a DTLB PAGE_FAULT/ACCESS_FAULT rides the SAME cycle
// pa_vld=1 arrives -- trap-at-issue, same shape as the existing misalign
// fix: vec 13/15 (page fault load/store), vec 5/7 (access fault load/
// store), tval=VA, and -- the correctness property that matters most here
// -- the faulting access must never touch the array (a faulting STORE must
// never actually write memory).
static void test_mmu_fault_trap_at_issue(void)
{
    const uint64_t A = 0x00000000800F2000ULL;   // fresh line, unused elsewhere

    g_mmu_inject_page_fault = true;
    LsuResult ld = do_op(F_LD, A, 0, 0, 5);
    check(ld.cmplt && ld.expt_vld && !ld.wb_vld,
          "DTLB page-fault load traps at issue, no writeback",
          ld.cmplt && ld.expt_vld && !ld.wb_vld, 1);
    check(ld.expt_vec == 13, "page-fault LOAD takes vec 13", ld.expt_vec, 13);
    check(ld.tval == A, "tval is the faulting VA", ld.tval, A);
    g_mmu_inject_page_fault = false;
    settle(10);

    g_mmu_inject_page_fault = true;
    LsuResult sd = do_op(F_SD, A, 0, 0x1122334455667788ULL, 0);
    check(sd.cmplt && sd.expt_vld, "DTLB page-fault STORE traps at issue",
          sd.cmplt && sd.expt_vld, 1);
    check(sd.expt_vec == 15, "page-fault STORE takes vec 15", sd.expt_vec, 15);
    g_mmu_inject_page_fault = false;
    settle(10);

    // The faulted store above must never have actually written memory --
    // read it back (now with a clean translation) and confirm it's still 0.
    LsuResult ld2 = do_op(F_LD, A, 0, 0, 5);
    check(ld2.cmplt && !ld2.expt_vld && ld2.wb_data == 0,
          "the faulted store never actually wrote memory", ld2.wb_data, 0);

    g_mmu_inject_access_fault = true;
    LsuResult ld3 = do_op(F_LD, A, 8, 0, 5);
    check(ld3.cmplt && ld3.expt_vld && ld3.expt_vec == 5,
          "DTLB access-fault LOAD takes vec 5", ld3.expt_vec, 5);
    g_mmu_inject_access_fault = false;
    settle(10);

    test_result("T15 MMU DTLB fault traps at issue (vec 13/15/5), never touches the array");
}

// T16 (M4 Task 5, D3): the PTW servant. The load-bearing property (design
// doc S4/D3): PROBE THE ARRAY FIRST. A PTE-shaped value stored through the
// normal pipe is DIRTY in the array (not yet written back) -- the servant
// must answer with that dirty value, not a stale bus read. A cold line
// falls through to exactly one direct AXI read.
static void test_ptw_servant(void)
{
    const uint64_t LINE     = 0x00000000800B0000ULL;   // fresh line, cold
    const uint64_t PROBE_A  = LINE + 0x10;              // dwoff=2 within the line
    const uint64_t PTE_VAL  = 0x00000000200000CFULL;    // a plausible leaf PTE (V,R,W,X,A,D)

    // wa=1 so the store actually ALLOCATES the line into the array (contract
    // 6's reset default is wa=0 -- a cold store-miss there bypasses the
    // array with a direct AXI write, which would make this "hit" scenario
    // a miss instead and defeat the whole point of the test).
    g_cp0_lsu_wa = 1;
    LsuResult sd = do_op(F_SD, PROBE_A, 0, PTE_VAL, 0);
    check(sd.cmplt && !sd.expt_vld, "PTE-shaped store completes", sd.cmplt, 1);
    settle(5);
    g_cp0_lsu_wa = 0;   // restore reset default

    int reads_before = g_slave.reads;
    PtwResult hit = ptw_probe(PROBE_A);
    check(hit.cmplt && !hit.timed_out, "PTW probe of a dirty line completes", hit.cmplt, 1);
    check(hit.data == PTE_VAL,
          "PTW probe returns the DIRTY array value, not a stale bus read",
          hit.data, PTE_VAL);
    check(!hit.bus_error, "array-hit answer carries no bus error", hit.bus_error, 0);
    check((uint64_t)g_slave.reads == (uint64_t)reads_before,
          "the array hit never touched the AXI bus",
          (uint64_t)g_slave.reads, (uint64_t)reads_before);

    // Cold line, backed by a known value in golden memory only -- the
    // servant must fall through to a direct AXI read.
    const uint64_t LINE2    = 0x00000000800F0000ULL;   // fresh line, unused elsewhere
    const uint64_t PROBE_A2 = LINE2 + 0x18;             // dwoff=3
    const uint64_t MEM_VAL  = 0x0123456789ABCDEFULL;
    for (int i = 0; i < 8; i++) mem_wr(PROBE_A2 + i, (uint8_t)(MEM_VAL >> (i * 8)));

    reads_before = g_slave.reads;
    PtwResult miss = ptw_probe(PROBE_A2);
    check(miss.cmplt && !miss.timed_out, "PTW probe of a cold line completes", miss.cmplt, 1);
    check(miss.data == MEM_VAL, "PTW bus-read fallback returns the correct doubleword",
          miss.data, MEM_VAL);
    check(!miss.bus_error, "clean bus read carries no error", miss.bus_error, 0);
    check((uint64_t)g_slave.reads == (uint64_t)(reads_before + 1),
          "the array miss fell through to exactly one AXI read",
          (uint64_t)g_slave.reads, (uint64_t)(reads_before + 1));

    settle(10);
    test_result("T16 PTW servant: array-probe-first coherence, then bus-read fallback");
}

// T17 (M4 Task 5, D1): RTU's flush broadcast clears ag_wait_r. A DTLB miss
// that never gets answered (a huge stall) parks the op; the flush must
// drop it outright (it never touched the array/STB -- still ST_IDLE) so a
// fresh, unrelated op issues cleanly afterward with no ghost completion.
static void test_ag_wait_flush(void)
{
    const uint64_t A = 0x0000000080050000ULL;

    g_pa_vld_stall_cycles = 500;   // never answers on its own
    while (dut->lsu_idu_full) { idle_issue(); tick(); }
    dut->idu_lsu_ex1_func       = F_LD;
    dut->idu_lsu_ex1_src0_data  = A;
    dut->idu_lsu_ex1_src0_ready = 1;
    dut->idu_lsu_ex1_src1_data  = 0;
    dut->idu_lsu_ex1_src1_ready = 1;
    dut->idu_lsu_ex1_src2_data  = 0;
    dut->idu_lsu_ex1_src2_ready = 1;
    dut->idu_lsu_ex1_dst0_reg   = 5;
    dut->idu_lsu_ex1_dp_sel     = 1;
    dut->idu_lsu_ex1_sel        = 1;
    dut->idu_lsu_ex1_raw_vld    = 1;   // M4 Task 5 fix: mirrors sel
    tick();
    idle_issue();

    settle(5);
    check(dut->lsu_idu_full != 0, "op is parked (lsu_idu_full) mid DTLB-miss wait",
          (uint64_t)dut->lsu_idu_full, 1);

    g_flush_fe = true;
    tick();
    g_flush_fe = false;

    settle(5);
    check(dut->lsu_idu_full == 0, "ag_wait_r cleared by the flush -- LSU is free again",
          (uint64_t)dut->lsu_idu_full, 0);

    g_pa_vld_stall_cycles = 0;
    settle(5);

    LsuResult ld = do_op(F_LD, A, 8, 0, 6);
    check(ld.cmplt && !ld.expt_vld, "a fresh op after the flush issues cleanly",
          ld.cmplt && !ld.expt_vld, 1);

    test_result("T17 AG wait-state: RTU flush drops a parked DTLB-miss op");
}

// T18 (M5 Task 4c): FLW/FLD dst0_frf tag threading + FLW NaN-boxing, through
// BOTH completion paths -- the fast-reply hit path (da_final, LSU.v:2453)
// and the deferred-LFB-miss path (lfb_wb_data, LSU.v:2497) -- plus a plain
// LW contrast on the identical bit pattern proving the NaN-boxing formatter
// is gated strictly on dst0_frf (D5), not merely on word size.
static void test_fp_load_dst_frf_and_nanbox(void)
{
    const uint64_t LINE_HIT = 0x00000000800a0000ULL;   // warmed -> fast-reply hits
    const uint64_t W_OFF    = 0x00, D_OFF = 0x08;
    const uint32_t W_PAT    = 0x3f800000UL;             // 1.0f; bit31=0 so the
                                                          // LW-vs-FLW top-32 contrast
                                                          // is unambiguous
    const uint64_t D_PAT    = 0x400921FB54442D18ULL;    // pi as a double bit pattern

    for (int i = 0; i < 4; i++) mem_wr(LINE_HIT + W_OFF + i, (uint8_t)(W_PAT >> (i * 8)));
    for (int i = 0; i < 8; i++) mem_wr(LINE_HIT + D_OFF + i, (uint8_t)(D_PAT >> (i * 8)));

    // Warm the line (plain LD) so the FLW/FLD/LW reads below are cache hits,
    // exercising the fast-reply da_final path.
    do_op(F_LD, LINE_HIT, 0, 0, 6);
    settle(3);

    LsuResult flw = do_op(F_FLW, LINE_HIT, W_OFF, 0, 5, true, true, 500, /*dst0_frf=*/true);
    check(flw.cmplt && !flw.expt_vld, "flw (hit): completes cleanly",
          flw.cmplt && !flw.expt_vld, 1);
    check(flw.wb_dst_frf, "flw (hit): lsu_rtu_wb_dst_frf==1 (D8 tag threads to the wb bus)",
          flw.wb_dst_frf, 1);
    check(flw.wb_data == (0xffffffff00000000ULL | W_PAT),
          "flw (hit): NaN-boxed (D5) -- upper 32 bits forced to 0xffffffff",
          flw.wb_data, 0xffffffff00000000ULL | W_PAT);

    LsuResult lw = do_op(F_LW, LINE_HIT, W_OFF, 0, 5, true, true, 500, /*dst0_frf=*/false);
    check(!lw.wb_dst_frf, "lw (hit) contrast: lsu_rtu_wb_dst_frf==0", lw.wb_dst_frf, 0);
    check(lw.wb_data == (uint64_t)W_PAT,
          "lw (hit) contrast: sign-extended (bit31=0), NOT NaN-boxed -- same bit "
          "pattern as flw above, proving the formatter gates on dst0_frf (D5), not size",
          lw.wb_data, (uint64_t)W_PAT);

    LsuResult fld = do_op(F_FLD, LINE_HIT, D_OFF, 0, 7, true, true, 500, /*dst0_frf=*/true);
    check(fld.cmplt && !fld.expt_vld, "fld (hit): completes cleanly",
          fld.cmplt && !fld.expt_vld, 1);
    check(fld.wb_dst_frf, "fld (hit): lsu_rtu_wb_dst_frf==1", fld.wb_dst_frf, 1);
    check(fld.wb_data == D_PAT,
          "fld (hit): full 64-bit passthrough, no NaN-boxing at double width",
          fld.wb_data, D_PAT);

    // Deferred-LFB-miss path: a cold line forces the non-blocking LSU to
    // defer the FLW into the LFB (M3b Task A rework); the dst0_frf tag must
    // survive that detour (lfb_dst_frf[]) and the NaN-boxing formatter must
    // re-fire from lfb_wb_data (LSU.v:2497), not just from da_final.
    const uint64_t LINE_MISS = 0x0000000080100000ULL;   // cold -> genuine miss (untouched
                                                          // by any earlier test in this file --
                                                          // 0x800a1000 collided with T11's
                                                          // still-resident BASE2 cache line, and
                                                          // 0x800b0000 collided with T16's
                                                          // still-resident PTE-servant line
                                                          // (test_ptw_servant's wa=1 store-alloc
                                                          // at 0x800b0010); every address up to
                                                          // 0x800f2000 is touched somewhere in
                                                          // this file, so this constant must stay
                                                          // above that high-water mark)
    for (int i = 0; i < 4; i++) mem_wr(LINE_MISS + i, (uint8_t)(W_PAT >> (i * 8)));

    issue_only(F_FLW, LINE_MISS, 0, 0, 5, /*dst0_frf=*/true);
    LsuResult flw_miss = wait_completion(5, 300);
    check(flw_miss.cmplt && !flw_miss.expt_vld, "flw (LFB-deferred miss): completes cleanly",
          flw_miss.cmplt && !flw_miss.expt_vld, 1);
    check(flw_miss.wb_dst_frf,
          "flw (LFB-deferred miss): dst0_frf tag survives the LFB detour (lfb_dst_frf[])",
          flw_miss.wb_dst_frf, 1);
    check(flw_miss.wb_data == (0xffffffff00000000ULL | W_PAT),
          "flw (LFB-deferred miss): NaN-boxed from lfb_wb_data, same as the hit path",
          flw_miss.wb_data, 0xffffffff00000000ULL | W_PAT);

    settle(10);
    test_result("T18 FLW/FLD dst0_frf tag + NaN-boxing (M5 Task 4c, D5/D8): fast-reply and LFB-deferred paths both correct");
}

//=============================================================================
// main
//=============================================================================
int main(int argc, char **argv)
{
    Verilated::commandArgs(argc, argv);
    dut = new VLSU;

    reset_dut();

    test_byte_width_matrix();
    test_misalign_traps();
    test_stb_forward();
    test_stb_create_vs_flush_interlock();
    test_miss_refill_dirty_victim_writeback();
    test_hit_under_miss();   // M3b Task A: hit-under-miss (non-blocking LFB)
    test_multi_miss_tracking();     // M3b Task A rework: 8-entry LFB, multi-miss
    test_miss_inflight_replay();    // M3b Task A rework: same-line replay, no corruption
    test_lfb_full_backpressure();   // M3b Task A rework: 8-entry LFB-full backpressure
    test_vb_decoupled_dirty_writeback();   // M3b Task B/C: single-entry VB decoupling
    test_pfb_stride_prefetch();            // M3b Task D: PFB stride prefetch + MHINT
    test_amr_streaming_store();            // M3b Task E: AMR write-allocate disabler
    test_stb_forward_under_miss();         // M3b Task F: store-under-miss consolidation
    test_ag_wait_state_dtlb_miss();        // M4 Task 5, D1: AG wait-state on a DTLB miss
    test_mmu_fault_trap_at_issue();        // M4 Task 5, S12: DTLB fault traps at issue
    test_ptw_servant();                    // M4 Task 5, D3: PTW servant, array-probe-first
    test_ag_wait_flush();                  // M4 Task 5, D1: RTU flush drops a parked op
    test_fp_load_dst_frf_and_nanbox();     // M5 Task 4c, D5/D8: dst0_frf tag + FLW NaN-boxing

    printf("[lsu_tb] %llu cycles, %d failure(s)\n",
           (unsigned long long)g_cycles, g_fail);
    printf("%s\n", g_fail ? "UNIT-FAIL" : "UNIT-PASS");

    dut->final();
    delete dut;
    return g_fail ? 1 : 0;
}
