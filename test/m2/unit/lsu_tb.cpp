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

//-----------------------------------------------------------------------------
// DUT plumbing
//-----------------------------------------------------------------------------
static VLSU *dut = nullptr;
static uint64_t g_cycles = 0;

static int g_cp0_lsu_mm         = 1;   // reset default per contract 3
static int g_cp0_lsu_wa         = 0;   // reset default per contract 6
static int g_cp0_lsu_dcache_en  = 1;   // this bench always runs "post-boot"

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

    void reset() { b_pending = false; r_pending = false; reads = 0; writes = 0; }

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
            b_pending = true; b_cnt = 2;
        } else if (b_pending && b_cnt > 0) {
            b_cnt--;
        }

        if (!r_pending && d->axi_d_arvalid) {
            r_addr = d->axi_d_araddr;
            last_araddr = r_addr;
            reads++;
            r_pending = true; r_cnt = 2;
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
// shape (cacheable iff PA >= 0x8000_0000) -- lsu_mmu_va is the VPN (an
// OUTPUT of LSU.v, already computed by AG), so this mirrors what a real
// MMU.v would answer combinationally, same cycle.
//-----------------------------------------------------------------------------
static void drive_mmu(void)
{
    uint64_t va = dut->lsu_mmu_va;   // byte VA (LSU drives ag_addr[51:0])
    // page number = va[39:12] -- must match rtl/MMU.v's D-side identity
    // map (the ITLB port takes a VPN directly, but the DTLB port takes the
    // full byte VA and extracts the page number itself).
    dut->mmu_lsu_pa           = (uint32_t)((va >> 12) & 0x0FFFFFFFULL);
    dut->mmu_lsu_pa_vld       = 1;
    bool cacheable            = (va >= 0x0000000080000000ULL);
    dut->mmu_lsu_ca           = cacheable ? 1 : 0;
    dut->mmu_lsu_so           = cacheable ? 0 : 1;
    dut->mmu_lsu_buf          = cacheable ? 1 : 0;
    dut->mmu_lsu_sec          = 0;
    dut->mmu_lsu_sh           = 0;
    dut->mmu_lsu_page_fault   = 0;
    dut->mmu_lsu_access_fault = 0;
}

static void drive_csr(void)
{
    dut->cp0_lsu_dcache_en = g_cp0_lsu_dcache_en;
    dut->cp0_lsu_mm        = g_cp0_lsu_mm;
    dut->cp0_lsu_wa        = g_cp0_lsu_wa;
    dut->rtu_lsu_expt_ack  = 0;
    dut->rtu_lsu_expt_exit = 0;
}

static void idle_issue(void)
{
    dut->idu_lsu_ex1_dp_sel     = 0;
    dut->idu_lsu_ex1_sel        = 0;
    dut->idu_lsu_ex1_func       = 0;
    dut->idu_lsu_ex1_src0_data  = 0;
    dut->idu_lsu_ex1_src0_ready = 1;
    dut->idu_lsu_ex1_src1_data  = 0;
    dut->idu_lsu_ex1_src1_ready = 1;
    dut->idu_lsu_ex1_src2_data  = 0;
    dut->idu_lsu_ex1_src2_ready = 1;
    dut->idu_lsu_ex1_dst0_reg   = 0;
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
};

// Issues one EX1 dispatch (a one-cycle pulse, matching how a real IDU
// hands off) and waits for lsu_rtu_ex1_cmplt_dp, capturing the completion
// bus at that exact cycle. `sel`/`dp_sel` are exposed separately so the
// STB-create-vs-flush interlock test can drive the ungated `dp_sel` alone.
static LsuResult do_op(uint32_t func, uint64_t src0, uint64_t src1, uint64_t src2,
                       unsigned dst0, bool sel = true, bool dp_sel = true, int guard = 500)
{
    int waited = 0;
    while (dut->lsu_idu_full && waited < guard) { idle_issue(); tick(); waited++; }

    dut->idu_lsu_ex1_dp_sel     = dp_sel ? 1 : 0;
    dut->idu_lsu_ex1_sel        = sel ? 1 : 0;
    dut->idu_lsu_ex1_func       = func;
    dut->idu_lsu_ex1_src0_data  = src0;
    dut->idu_lsu_ex1_src0_ready = 1;
    dut->idu_lsu_ex1_src1_data  = src1;
    dut->idu_lsu_ex1_src1_ready = 1;
    dut->idu_lsu_ex1_src2_data  = src2;
    dut->idu_lsu_ex1_src2_ready = 1;
    dut->idu_lsu_ex1_dst0_reg   = dst0;
    tick();
    idle_issue();

    LsuResult r;
    for (int i = 0; i < guard; i++) {
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
            return r;
        }
        tick();
    }
    r.timed_out = true;
    return r;
}

static inline uint64_t LD(uint64_t addr) { return do_op(F_LD, addr, 0, 0, 5).wb_data; }

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

    printf("[lsu_tb] %llu cycles, %d failure(s)\n",
           (unsigned long long)g_cycles, g_fail);
    printf("%s\n", g_fail ? "UNIT-FAIL" : "UNIT-PASS");

    dut->final();
    delete dut;
    return g_fail ? 1 : 0;
}
