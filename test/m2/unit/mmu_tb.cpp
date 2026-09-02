//=============================================================================
// mmu_tb.cpp - standalone unit bench for rtl/MMU.v (M2 plan task 6.4)
//=============================================================================
// Verilates MMU.v + rvproc_pkg.sv alone and drives both independent port
// groups (IFU's ITLB request, LSU's DTLB request) directly. MMU.v is a pure
// combinational identity-map + PMA/sysmap stub (contract 2/contract 5) --
// no clock is functionally required, but this bench still ticks a clock per
// the project's unit-bench convention (harmless, since every output here is
// a same-cycle function of its inputs).
//
// Build/run: make -C test/m2/unit mmu && bin/unit/mmu_tb
// Prints one line per test and ends with UNIT-PASS or UNIT-FAIL.
//=============================================================================

#include <verilated.h>
#include "VMMU.h"

#include <cstdio>
#include <cstdint>
#include <cstring>

static const unsigned MMU_PA_WIDTH = 28;

static VMMU *dut = nullptr;
static uint64_t g_cycles = 0;

static void tick(void)
{
    dut->eval();
    dut->clk = 1;
    dut->eval();
    dut->clk = 0;
    dut->eval();
    g_cycles++;
}

static void tie_idle_inputs(void)
{
    dut->ifu_mmu_abort     = 0;
    dut->ifu_mmu_va        = 0;
    dut->ifu_mmu_va_vld    = 0;
    dut->lsu_mmu_va        = 0;
    dut->lsu_mmu_va_vld    = 0;
    dut->lsu_mmu_priv_mode = 3;   // M-mode
    dut->lsu_mmu_st_inst   = 0;
    // M4 Task 4: satp Mode=0 (bare) keeps mmu_en=0 for BOTH ports, so T1-T9
    // above stay on the identity+PMA path untouched by anything below.
    dut->cp0_mmu_satp_data  = 0;
    dut->cp0_mmu_satp_wen   = 0;
    dut->cp0_mmu_mxr        = 0;
    dut->cp0_mmu_sum        = 0;
    dut->cp0_yy_priv_mode   = 3;   // M-mode
    dut->lsu_mmu_data       = 0;
    dut->lsu_mmu_data_vld   = 0;
    dut->lsu_mmu_bus_error  = 0;
    dut->lsu_mmu_abort      = 0;
    dut->pmp_mmu_fetch_deny = 0;
    dut->pmp_mmu_data_deny  = 0;
}

static void reset_dut(void)
{
    dut->clk   = 0;
    dut->rst_n = 0;
    tie_idle_inputs();
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
        if (g_fail < 40)
            printf("    FAIL %-52s got=0x%llx exp=0x%llx (cycle %llu)\n", what,
                   (unsigned long long)got, (unsigned long long)exp,
                   (unsigned long long)g_cycles);
        g_fail++;
    }
}

static void test_result(const char *name)
{
    printf("[mmu_tb] %-56s %s\n", name, g_local ? "FAIL" : "PASS");
    g_local = 0;
}

//=============================================================================
// Helpers: an "address" here is a full 40-bit byte address. BOTH request
// ports carry the PAGE NUMBER, not the byte address -- the I-side's
// `ifu_mmu_va` is `icache_rd_addr[63:12]` (ICache.v's own convention,
// confirmed against icache.v's ports) and the D-side's `lsu_mmu_va` is
// `ag_pipe_addr[63:12]` (aq_lsu_ag.v:1566; LSU.v drives `ag_addr[63:12]`
// since the 2026-08-23 donor-faithful fix). The response is likewise a page
// number (`mmu_lsu_pa` is 28 bits, aq_lsu_ag.v:201), which the requester
// reassembles into a full PA as `{pa_ppn, addr[11:0]}` (aq_lsu_ag.v:1446).
//=============================================================================
static inline uint64_t vpn_of(uint64_t addr) { return addr >> 12; }

struct DtlbResp {
    uint32_t pa;
    bool pa_vld, ca, so, buf, sec, sh, page_fault, access_fault;
};

static DtlbResp query_dtlb(uint64_t addr, bool st_inst = false)
{
    dut->lsu_mmu_va        = vpn_of(addr);   // PAGE NUMBER (aq_lsu_ag.v:1566), not the byte VA
    dut->lsu_mmu_va_vld    = 1;
    dut->lsu_mmu_priv_mode = 3;
    dut->lsu_mmu_st_inst   = st_inst ? 1 : 0;
    dut->eval();
    DtlbResp r;
    r.pa            = (uint32_t)(dut->mmu_lsu_pa & ((1u << MMU_PA_WIDTH) - 1));
    r.pa_vld        = dut->mmu_lsu_pa_vld != 0;
    r.ca            = dut->mmu_lsu_ca != 0;
    r.so            = dut->mmu_lsu_so != 0;
    r.buf           = dut->mmu_lsu_buf != 0;
    r.sec           = dut->mmu_lsu_sec != 0;
    r.sh            = dut->mmu_lsu_sh != 0;
    r.page_fault    = dut->mmu_lsu_page_fault != 0;
    r.access_fault  = dut->mmu_lsu_access_fault != 0;
    tick();
    dut->lsu_mmu_va_vld = 0;
    tick();
    return r;
}

struct ItlbResp {
    uint32_t pa;
    bool pa_vld, access_fault;
    uint8_t prot;
};

static ItlbResp query_itlb(uint64_t addr)
{
    dut->ifu_mmu_abort  = 0;
    dut->ifu_mmu_va     = vpn_of(addr);
    dut->ifu_mmu_va_vld = 1;
    dut->eval();
    ItlbResp r;
    r.pa            = (uint32_t)(dut->mmu_ifu_pa & ((1u << MMU_PA_WIDTH) - 1));
    r.pa_vld        = dut->mmu_ifu_pa_vld != 0;
    r.access_fault  = dut->mmu_ifu_access_fault != 0;
    r.prot          = (uint8_t)(dut->mmu_ifu_prot & 0x1F);
    tick();
    dut->ifu_mmu_va_vld = 0;
    tick();
    return r;
}

static uint64_t reconstruct_pa(uint64_t va, uint32_t pa_ppn)
{
    return ((uint64_t)pa_ppn << 12) | (va & 0xFFFULL);
}

//=============================================================================
// Tests
//=============================================================================

// T1: DTLB identity-map correctness across a spread of addresses.
static void test_dtlb_identity_map(void)
{
    const uint64_t addrs[] = {
        0x0000000000000000ULL, 0x0000000000001234ULL, 0x0000000080000000ULL,
        0x0000000080001008ULL, 0x00000000FFFFFFFFULL, 0x0000000012345678ULL,
    };
    for (uint64_t a : addrs) {
        DtlbResp r = query_dtlb(a);
        check(r.pa_vld, "dtlb pa_vld always 1", r.pa_vld, 1);
        check(!r.page_fault, "dtlb page_fault always 0", r.page_fault, 0);
        check(!r.access_fault, "dtlb access_fault always 0", r.access_fault, 0);
        uint64_t recon = reconstruct_pa(a, r.pa);
        check(recon == a, "dtlb identity map: pa==va", recon, a);
        check(!r.sec, "dtlb sec always 0", r.sec, 0);
        check(!r.sh, "dtlb sh always 0", r.sh, 0);
    }
    test_result("T1 DTLB identity-map correctness");
}

// T2: ITLB identity-map correctness + prot field shape.
static void test_itlb_identity_map(void)
{
    const uint64_t addrs[] = {
        0x0000000080000000ULL, 0x0000000080010000ULL, 0x00000000FFFFF000ULL,
    };
    for (uint64_t a : addrs) {
        ItlbResp r = query_itlb(a);
        check(r.pa_vld, "itlb pa_vld always 1", r.pa_vld, 1);
        check(!r.access_fault, "itlb access_fault always 0", r.access_fault, 0);
        uint64_t recon = reconstruct_pa(a, r.pa);
        check(recon == a, "itlb identity map: pa==va", recon, a);
        // prot[4]=pgflt must be 0 (no fault-capable MMU in M2)
        check(((r.prot >> 4) & 1) == 0, "itlb prot[4]=pgflt is 0", (r.prot >> 4) & 1, 0);
        // prot[0]=sec must be 0
        check((r.prot & 1) == 0, "itlb prot[0]=sec is 0", r.prot & 1, 0);
    }
    test_result("T2 ITLB identity-map correctness + prot shape");
}

// T3: DRAM region (0x8000_0000-0xFFFF_FFFF): cacheable, bufferable, not
// strongly-ordered.
static void test_pma_dram(void)
{
    const uint64_t addrs[] = {
        0x0000000080000000ULL, 0x0000000090000000ULL, 0x00000000FFFFFFFFULL,
        0x0000000080000FFFULL,
    };
    for (uint64_t a : addrs) {
        DtlbResp r = query_dtlb(a);
        check(r.ca, "DRAM region is cacheable", r.ca, 1);
        check(!r.so, "DRAM region is not strongly-ordered", r.so, 0);
        check(r.buf, "DRAM region is bufferable", r.buf, 1);
    }
    test_result("T3 PMA: DRAM region cacheable+bufferable");
}

// T4: CLINT region (0x0200_0000-0x0200_FFFF): uncached, strongly ordered.
static void test_pma_clint(void)
{
    const uint64_t addrs[] = {
        0x0000000002000000ULL, 0x0000000002008000ULL, 0x000000000200FFFFULL,
    };
    for (uint64_t a : addrs) {
        DtlbResp r = query_dtlb(a);
        check(!r.ca, "CLINT region is uncached", r.ca, 0);
        check(r.so, "CLINT region is strongly-ordered", r.so, 1);
        check(!r.buf, "CLINT region is not bufferable", r.buf, 0);
    }
    test_result("T4 PMA: CLINT region uncached+strongly-ordered");
}

// T5: PLIC region (0x0C00_0000-0x0CFF_FFFF): uncached, strongly ordered.
static void test_pma_plic(void)
{
    const uint64_t addrs[] = {
        0x000000000C000000ULL, 0x000000000C800000ULL, 0x000000000CFFFFFFULL,
    };
    for (uint64_t a : addrs) {
        DtlbResp r = query_dtlb(a);
        check(!r.ca, "PLIC region is uncached", r.ca, 0);
        check(r.so, "PLIC region is strongly-ordered", r.so, 1);
    }
    test_result("T5 PMA: PLIC region uncached+strongly-ordered");
}

// T6: UART region (0x1000_0000-0x1000_FFFF): uncached, strongly ordered.
static void test_pma_uart(void)
{
    const uint64_t addrs[] = {
        0x0000000010000000ULL, 0x0000000010008000ULL, 0x000000001000FFFFULL,
    };
    for (uint64_t a : addrs) {
        DtlbResp r = query_dtlb(a);
        check(!r.ca, "UART region is uncached", r.ca, 0);
        check(r.so, "UART region is strongly-ordered", r.so, 1);
    }
    test_result("T6 PMA: UART region uncached+strongly-ordered");
}

// T7: everything else in 0x0000_0000-0x7FFF_FFFF not otherwise covered:
// uncached/reserved (deliberate divergence from AXICrossbar's
// DEFAULT_SLAVE=SI_MEM convenience, contract 5).
static void test_pma_reserved(void)
{
    const uint64_t addrs[] = {
        0x0000000000000000ULL, 0x0000000000010000ULL, 0x0000000004000000ULL,
        0x0000000020000000ULL, 0x0000000050000000ULL, 0x000000007FFFFFFFULL,
    };
    for (uint64_t a : addrs) {
        DtlbResp r = query_dtlb(a);
        check(!r.ca, "reserved region is uncached", r.ca, 0);
    }
    test_result("T7 PMA: everything else below 0x8000_0000 is uncached");
}

// T8: the relocated tohost aperture (0x7FFF_F000, contract 6) reads
// uncached -- the whole point of the tohost-relocation fix (design doc S8).
static void test_pma_tohost(void)
{
    const uint64_t ADDR_TOHOST = 0x000000007FFFF000ULL;
    DtlbResp r = query_dtlb(ADDR_TOHOST, /*st_inst=*/true);
    check(!r.ca, "relocated tohost aperture is uncached", r.ca, 0);
    check(r.so, "relocated tohost aperture is strongly-ordered", r.so, 1);
    test_result("T8 PMA: relocated tohost aperture reads uncached");
}

// T9: exact region boundaries -- one byte on each side of DRAM's lower edge.
static void test_pma_boundaries(void)
{
    DtlbResp below = query_dtlb(0x000000007FFFFFFFULL);   // just below DRAM
    DtlbResp at    = query_dtlb(0x0000000080000000ULL);   // DRAM's own base
    check(!below.ca, "0x7FFF_FFFF (below DRAM) is uncached", below.ca, 0);
    check(at.ca, "0x8000_0000 (DRAM base) is cacheable", at.ca, 1);

    DtlbResp clint_below = query_dtlb(0x0000000001FFFFFFULL);
    DtlbResp clint_at    = query_dtlb(0x0000000002000000ULL);
    DtlbResp clint_above = query_dtlb(0x0000000002010000ULL);
    // Both neighbors of CLINT fall into the "everything else" reserved
    // bucket (contract 5) -- uncached, and therefore ALSO strongly-ordered
    // (so=!ca everywhere in this table; there is no "uncached but weakly
    // ordered" row anywhere in contract 5's map).
    check(!clint_below.ca, "just below CLINT (reserved bucket) is uncached", clint_below.ca, 0);
    check(clint_below.so, "just below CLINT (reserved bucket) is strongly-ordered",
          clint_below.so, 1);
    check(clint_at.so, "CLINT base is strongly-ordered", clint_at.so, 1);
    check(!clint_above.ca, "just above CLINT is uncached (reserved bucket, not CLINT itself)",
          clint_above.ca, 0);
    check(clint_above.so, "just above CLINT is still strongly-ordered (reserved bucket)",
          clint_above.so, 1);

    test_result("T9 PMA region boundaries exact");
}

//=============================================================================
// M4 Task 4: TLB + PTW translation rows. The bench plays the PTW's memory
// servant directly (group 5's frozen six-wire channel, mmu_lsu_data_req/
// _addr/_size -> lsu_mmu_data/_vld/_bus_error) and the PMP channels
// (mmu_pmp_*_pa/vld -> pmp_mmu_*_deny), exactly as rv12's own mmu_tb does
// until its LSU servant/PMP instance land (this task's own gate: LSU.v's
// real servant is Task 5, real PMP.v wiring is Task 5/6).
//=============================================================================
#include <map>
#include <set>

static std::map<uint64_t, uint64_t> g_mem;        // byte addr (8B-aligned) -> PTE
static std::set<uint64_t>           g_bus_err;    // addrs that answer with bus_error
static uint32_t                     g_next_pt_ppn;

static uint64_t pte_encode(uint32_t ppn28, bool r, bool w, bool x, bool u,
                            bool g, bool a, bool d)
{
    uint64_t pte = 1;                       // V
    if (r) pte |= 1ULL << 1;
    if (w) pte |= 1ULL << 2;
    if (x) pte |= 1ULL << 3;
    if (u) pte |= 1ULL << 4;
    if (g) pte |= 1ULL << 5;
    if (a) pte |= 1ULL << 6;
    if (d) pte |= 1ULL << 7;
    pte |= ((uint64_t)ppn28 << 10);
    return pte;
}

// Installs a 3-level chain rooted at satp_ppn for vpn27, with the leaf
// planted at `level` (1=1G at the root table, 2=2M at the mid table,
// 3=4K at the leaf table) -- mirrors MMU.v SECTION 6.2's own address
// formulas (fst/scd/thd_addr) exactly so the servant answers at the SAME
// addresses the walker computes.
static void install_leaf(uint32_t satp_ppn, uint32_t vpn27, int level,
                          uint32_t leaf_ppn, bool r, bool w, bool x, bool u,
                          bool g = false, bool a = true, bool d = true)
{
    uint32_t vpn2 = (vpn27 >> 18) & 0x1FF;
    uint32_t vpn1 = (vpn27 >> 9)  & 0x1FF;
    uint32_t vpn0 =  vpn27        & 0x1FF;

    uint64_t l1_addr = ((uint64_t)satp_ppn << 12) + (uint64_t)vpn2 * 8;
    if (level == 1) {
        g_mem[l1_addr] = pte_encode(leaf_ppn, r, w, x, u, g, a, d);
        return;
    }
    uint32_t l2_ppn;
    if (g_mem.count(l1_addr) && (g_mem[l1_addr] & 0xF) == 0x1)
        l2_ppn = (uint32_t)(g_mem[l1_addr] >> 10);
    else {
        l2_ppn = g_next_pt_ppn++;
        g_mem[l1_addr] = pte_encode(l2_ppn, false, false, false, false, false, true, false);
    }
    uint64_t l2_addr = ((uint64_t)l2_ppn << 12) + (uint64_t)vpn1 * 8;
    if (level == 2) {
        g_mem[l2_addr] = pte_encode(leaf_ppn, r, w, x, u, g, a, d);
        return;
    }
    uint32_t l3_ppn;
    if (g_mem.count(l2_addr) && (g_mem[l2_addr] & 0xF) == 0x1)
        l3_ppn = (uint32_t)(g_mem[l2_addr] >> 10);
    else {
        l3_ppn = g_next_pt_ppn++;
        g_mem[l2_addr] = pte_encode(l3_ppn, false, false, false, false, false, true, false);
    }
    uint64_t l3_addr = ((uint64_t)l3_ppn << 12) + (uint64_t)vpn0 * 8;
    g_mem[l3_addr] = pte_encode(leaf_ppn, r, w, x, u, g, a, d);
}

static uint64_t make_satp(uint32_t root_ppn, uint16_t asid = 0)
{
    return (1ULL << 63) | ((uint64_t)asid << 44) | (uint64_t)root_ppn;
}

// One cycle of the servant loop: sample the walker's memory request (if
// any) combinationally, answer it same-cycle, then clock. 0-cycle-latency
// PTE reads keep the walk bounded and the test loop simple; the FSM's own
// PMP->DATA->CHK ladder still runs one state per posedge regardless.
static void tick_servant(void)
{
    dut->eval();
    if (dut->mmu_lsu_data_req) {
        uint64_t addr = (uint64_t)dut->mmu_lsu_data_req_addr;
        if (g_bus_err.count(addr)) {
            dut->lsu_mmu_bus_error = 1;
            dut->lsu_mmu_data_vld  = 0;
            dut->lsu_mmu_data      = 0;
        } else {
            dut->lsu_mmu_data      = g_mem[addr];   // 0 (invalid PTE) if absent
            dut->lsu_mmu_data_vld  = 1;
            dut->lsu_mmu_bus_error = 0;
        }
    } else {
        dut->lsu_mmu_data_vld  = 0;
        dut->lsu_mmu_bus_error = 0;
    }
    dut->eval();
    dut->clk = 1; dut->eval();
    dut->clk = 0; dut->eval();
    g_cycles++;
}

static void flush_tlb(void)
{
    dut->cp0_mmu_satp_wen = 1;
    tick_servant();
    dut->cp0_mmu_satp_wen = 0;
}

struct WalkResult {
    bool pa_vld = false, page_fault = false, access_fault = false;
    uint32_t pa = 0;
    int cycles = 0;
    bool timed_out = false;
};

static WalkResult query_dtlb_mmu(uint64_t va_byte, bool st_inst, uint64_t satp,
                                  uint8_t priv, bool mxr = false, bool sum = false)
{
    dut->cp0_mmu_satp_data  = satp;
    dut->cp0_mmu_mxr        = mxr;
    dut->cp0_mmu_sum        = sum;
    dut->lsu_mmu_priv_mode  = priv;
    dut->lsu_mmu_va         = va_byte >> 12;
    dut->lsu_mmu_va_vld     = 1;
    dut->lsu_mmu_st_inst    = st_inst ? 1 : 0;
    WalkResult r;
    for (int i = 0; i < 64; i++) {
        tick_servant();
        r.cycles++;
        dut->eval();
        if (dut->mmu_lsu_pa_vld) {
            r.pa_vld       = true;
            r.page_fault   = dut->mmu_lsu_page_fault != 0;
            r.access_fault = dut->mmu_lsu_access_fault != 0;
            r.pa           = (uint32_t)(dut->mmu_lsu_pa & ((1u << MMU_PA_WIDTH) - 1));
            break;
        }
    }
    if (!r.pa_vld) r.timed_out = true;
    dut->lsu_mmu_va_vld = 0;
    tick_servant();
    return r;
}

static WalkResult query_itlb_mmu(uint64_t va_byte, uint64_t satp, uint8_t priv,
                                  bool mxr = false, bool sum = false)
{
    dut->cp0_mmu_satp_data = satp;
    dut->cp0_mmu_mxr       = mxr;
    dut->cp0_mmu_sum       = sum;
    dut->cp0_yy_priv_mode  = priv;
    dut->ifu_mmu_va        = va_byte >> 12;
    dut->ifu_mmu_va_vld    = 1;
    WalkResult r;
    for (int i = 0; i < 64; i++) {
        tick_servant();
        r.cycles++;
        dut->eval();
        if (dut->mmu_ifu_pa_vld) {
            r.pa_vld       = true;
            r.page_fault   = ((dut->mmu_ifu_prot >> 4) & 1) != 0;
            r.access_fault = dut->mmu_ifu_access_fault != 0;
            r.pa           = (uint32_t)(dut->mmu_ifu_pa & ((1u << MMU_PA_WIDTH) - 1));
            break;
        }
    }
    if (!r.pa_vld) r.timed_out = true;
    dut->ifu_mmu_va_vld = 0;
    dut->cp0_yy_priv_mode = 3;   // restore M-mode default for later mach-path tests
    tick_servant();
    return r;
}

static void reset_pt_model(void)
{
    g_mem.clear();
    g_bus_err.clear();
    g_next_pt_ppn = 0x200;
}

// T10: 4K leaf, clean RWX-user page, S-mode load -- first access WALKS
// (mmu_lsu_data_req pulses), second access to the same page HITS without
// walking (the TLB actually cached the refill).
static void test_dtlb_4k_walk_and_hit(void)
{
    reset_pt_model();
    flush_tlb();
    const uint32_t satp_ppn = 0x10;
    const uint32_t vpn      = 0x123;
    const uint32_t leaf_ppn = 0x8ABC;
    install_leaf(satp_ppn, vpn, 3, leaf_ppn, true, true, false, true);
    uint64_t satp = make_satp(satp_ppn);
    uint64_t va   = (uint64_t)vpn << 12 | 0x40;

    // mmu_lsu_pa is a PAGE NUMBER (contract 2), not a byte address -- the
    // requester splices in the page offset itself. U-mode load against the
    // page's own U=1 bit (an S-mode load here would need SUM=1 too, T13's
    // own point -- kept orthogonal to this walk-then-hit mechanic).
    WalkResult r1 = query_dtlb_mmu(va, false, satp, 0 /*U*/);
    check(!r1.timed_out, "4K walk completes within bound", r1.timed_out, 0);
    check(r1.pa_vld, "4K walk: pa_vld", r1.pa_vld, 1);
    check(!r1.page_fault, "4K walk: no page fault", r1.page_fault, 0);
    check(!r1.access_fault, "4K walk: no access fault", r1.access_fault, 0);
    check(r1.pa == leaf_ppn, "4K walk: PPN reassembled correctly", r1.pa, leaf_ppn);
    check(r1.cycles > 1, "4K walk: took multiple cycles (a real walk happened)",
          r1.cycles, 0);

    // Break the page-table memory so a SECOND walk (if one happened) would
    // page-fault -- proves the second access is served from the TLB, not
    // re-walked.
    g_mem.clear();
    WalkResult r2 = query_dtlb_mmu(va, false, satp, 0 /*U*/);
    check(r2.pa_vld && !r2.page_fault && !r2.access_fault,
          "4K hit: served from TLB after memory is pulled out from under it",
          r2.pa_vld && !r2.page_fault && !r2.access_fault, 1);
    check(r2.cycles == 1, "4K hit: single-cycle (no walk)", r2.cycles, 1);
    test_result("T10 DTLB 4K walk-then-hit");
}

// T11: 2M superpage leaf at level 2 -- the splice must supply the
// request's own low-9 VPN bits into the returned PA.
static void test_dtlb_2m_superpage(void)
{
    reset_pt_model();
    flush_tlb();
    const uint32_t satp_ppn = 0x11;
    const uint32_t vpn2m    = 0x40200;      // VPN[8:0] free for the splice probe
    const uint32_t frame_ppn = 0x9000;      // 2M-aligned (low 9 PPN bits 0)
    install_leaf(satp_ppn, vpn2m, 2, frame_ppn, true, true, true, false);
    uint64_t satp = make_satp(satp_ppn);
    const uint32_t sub_vpn = vpn2m | 0x15;  // same 2M frame, nonzero low bits
    uint64_t va = (uint64_t)sub_vpn << 12 | 0x8;

    WalkResult r = query_dtlb_mmu(va, false, satp, 1 /*S*/);
    check(r.pa_vld && !r.page_fault && !r.access_fault,
          "2M superpage walk succeeds", r.pa_vld && !r.page_fault, 1);
    uint32_t expect_ppn = frame_ppn | 0x15;   // PPN only (contract 2)
    check(r.pa == expect_ppn, "2M superpage: sub-frame VPN spliced into PPN",
          r.pa, expect_ppn);
    test_result("T11 DTLB 2M superpage splice");
}

// T12: 1G superpage leaf at level 1.
static void test_dtlb_1g_superpage(void)
{
    reset_pt_model();
    flush_tlb();
    const uint32_t satp_ppn = 0x12;
    const uint32_t vpn1g    = 0x40000;      // VPN[17:0] free for the splice probe
    const uint32_t frame_ppn = 0xA00000;    // 1G-aligned (low 18 PPN bits 0)
    install_leaf(satp_ppn, vpn1g, 1, frame_ppn, true, false, true, false);
    uint64_t satp = make_satp(satp_ppn);
    const uint32_t sub_vpn = vpn1g | 0x2A1;
    uint64_t va = (uint64_t)sub_vpn << 12;

    WalkResult r = query_dtlb_mmu(va, false, satp, 1 /*S*/);
    check(r.pa_vld && !r.page_fault && !r.access_fault,
          "1G superpage walk succeeds", r.pa_vld && !r.page_fault, 1);
    uint32_t expect_ppn = frame_ppn | 0x2A1;   // PPN only (contract 2)
    check(r.pa == expect_ppn, "1G superpage: sub-frame VPN spliced into PPN",
          r.pa, expect_ppn);
    test_result("T12 DTLB 1G superpage splice");
}

// T13: U/S permission crossing -- a U-mode load to a supervisor-only (U=0)
// page page-faults; the same page is fine from S-mode.
static void test_dtlb_priv_cross(void)
{
    reset_pt_model();
    flush_tlb();
    const uint32_t satp_ppn = 0x13;
    const uint32_t vpn = 0x300;
    install_leaf(satp_ppn, vpn, 3, 0x8100, true, true, false, false /*U=0*/);
    uint64_t satp = make_satp(satp_ppn);
    uint64_t va = (uint64_t)vpn << 12;

    WalkResult ru = query_dtlb_mmu(va, false, satp, 0 /*U*/);
    check(ru.pa_vld && ru.page_fault, "U-mode load to a supervisor-only page faults",
          ru.pa_vld && ru.page_fault, 1);

    flush_tlb();
    WalkResult rs = query_dtlb_mmu(va, false, satp, 1 /*S*/);
    check(rs.pa_vld && !rs.page_fault, "S-mode load to the same page succeeds",
          rs.pa_vld && !rs.page_fault, 1);
    test_result("T13 DTLB U/S privilege crossing");
}

// T14: A=0 always faults (no HW A-bit update, spec S5); D=0 store faults
// (D-M4-1, donor comments this arm out -- rv906 enables it).
static void test_dtlb_ad_bits(void)
{
    reset_pt_model();
    flush_tlb();
    const uint32_t satp_ppn = 0x14;
    // U=0 (supervisor-only) -- keeps this test orthogonal to the U/S
    // crossing + SUM logic (T13's own point).
    install_leaf(satp_ppn, 0x10, 3, 0x8200, true, true, false, false,
                 /*g=*/false, /*a=*/false, /*d=*/true);
    uint64_t satp = make_satp(satp_ppn);
    WalkResult ra = query_dtlb_mmu(0x10ULL << 12, false, satp, 1);
    check(ra.pa_vld && ra.page_fault, "A=0 load faults", ra.pa_vld && ra.page_fault, 1);

    reset_pt_model();
    flush_tlb();
    install_leaf(satp_ppn, 0x11, 3, 0x8210, true, true, false, false,
                 /*g=*/false, /*a=*/true, /*d=*/false);
    WalkResult rd_ld = query_dtlb_mmu(0x11ULL << 12, false, satp, 1);
    check(rd_ld.pa_vld && !rd_ld.page_fault, "D=0 load does NOT fault",
          rd_ld.pa_vld && !rd_ld.page_fault, 1);
    flush_tlb();
    WalkResult rd_st = query_dtlb_mmu(0x11ULL << 12, true, satp, 1);
    check(rd_st.pa_vld && rd_st.page_fault, "D=0 store DOES fault (D-M4-1)",
          rd_st.pa_vld && rd_st.page_fault, 1);
    test_result("T14 DTLB A/D bit faults");
}

// T15: an invalid PTE (V=0) at the final level faults; a walk through a
// non-existent table (bus error mid-walk) reports ACCESS fault, not page
// fault.
static void test_dtlb_invalid_and_buserr(void)
{
    reset_pt_model();
    flush_tlb();
    const uint32_t satp_ppn = 0x15;
    uint64_t satp = make_satp(satp_ppn);
    // VPN 0x20: nothing installed at all -> root PTE reads 0 (V=0).
    WalkResult ri = query_dtlb_mmu(0x20ULL << 12, false, satp, 1);
    check(ri.pa_vld && ri.page_fault && !ri.access_fault,
          "unmapped (V=0 root PTE) faults as a PAGE fault",
          ri.pa_vld && ri.page_fault && !ri.access_fault, 1);

    reset_pt_model();
    flush_tlb();
    install_leaf(satp_ppn, 0x21, 3, 0x8300, true, true, false, true);
    // Force a bus error at the ROOT read.
    g_bus_err.insert((uint64_t)satp_ppn << 12);
    WalkResult rb = query_dtlb_mmu(0x21ULL << 12, false, satp, 1);
    check(rb.pa_vld && rb.access_fault && !rb.page_fault,
          "a bus error mid-walk reports ACCESS fault", rb.pa_vld && rb.access_fault, 1);
    test_result("T15 DTLB invalid-PTE and bus-error faults");
}

// T16: a PMP deny during the walk (poked directly on the data channel,
// SECTION 6.4/7's live per-level check) reports access fault.
static void test_dtlb_pmp_deny_during_walk(void)
{
    reset_pt_model();
    flush_tlb();
    const uint32_t satp_ppn = 0x16;
    install_leaf(satp_ppn, 0x30, 3, 0x8400, true, true, false, true);
    uint64_t satp = make_satp(satp_ppn);

    dut->pmp_mmu_data_deny = 1;   // deny every data-channel PMP check
    WalkResult r = query_dtlb_mmu(0x30ULL << 12, false, satp, 1);
    dut->pmp_mmu_data_deny = 0;
    check(r.pa_vld && r.access_fault && !r.page_fault,
          "PMP deny mid-walk reports ACCESS fault", r.pa_vld && r.access_fault, 1);
    test_result("T16 DTLB PMP deny mid-walk");
}

// T17: non-canonical VA (VA[63:39] doesn't sign-extend VA[38]) page-faults
// immediately, WITHOUT starting a walk (mmu_lsu_data_req never pulses).
static void test_dtlb_noncanonical_va(void)
{
    reset_pt_model();
    flush_tlb();
    const uint32_t satp_ppn = 0x17;
    uint64_t satp = make_satp(satp_ppn);
    uint64_t bad_va = 0x0001000000001000ULL;   // VA[63:39] != sign-ext(VA[38])

    dut->cp0_mmu_satp_data = satp;
    dut->lsu_mmu_priv_mode = 1;
    dut->lsu_mmu_va        = bad_va >> 12;
    dut->lsu_mmu_va_vld    = 1;
    dut->lsu_mmu_st_inst   = 0;
    dut->eval();
    bool req_pulsed = dut->mmu_lsu_data_req != 0;
    bool pa_vld_now = dut->mmu_lsu_pa_vld != 0;
    bool pf_now     = dut->mmu_lsu_page_fault != 0;
    tick_servant();
    dut->lsu_mmu_va_vld = 0;
    tick_servant();
    check(!req_pulsed, "non-canonical VA never starts a walk", req_pulsed, 0);
    check(pa_vld_now && pf_now, "non-canonical VA page-faults same cycle",
          pa_vld_now && pf_now, 1);
    test_result("T17 DTLB non-canonical VA");
}

// T18: satp write flushes the whole TLB (a cached hit stops hitting; the
// next access re-walks).
static void test_satp_write_flush(void)
{
    reset_pt_model();
    flush_tlb();
    const uint32_t satp_ppn = 0x18;
    install_leaf(satp_ppn, 0x50, 3, 0x8500, true, true, false, true);
    uint64_t satp = make_satp(satp_ppn);
    WalkResult r1 = query_dtlb_mmu(0x50ULL << 12, false, satp, 1);
    check(r1.pa_vld && r1.cycles > 1, "first access walks", r1.pa_vld && r1.cycles > 1, 1);

    flush_tlb();
    g_mem.clear();   // pull the page table out from under a cached entry
    WalkResult r2 = query_dtlb_mmu(0x50ULL << 12, false, satp, 1);
    check(r2.pa_vld && r2.page_fault,
          "post-flush access re-walks (and now faults on the cleared table)",
          r2.pa_vld && r2.page_fault, 1);
    test_result("T18 satp-write flushes the TLB");
}

// T19: ITLB translation basics + D-M4-10 (SUM never excuses a supervisor
// FETCH from a U page, even with SUM=1 -- donor lets it through, rv906
// diverges to match rv12's identical fix and the spec).
static void test_itlb_walk_and_sum(void)
{
    reset_pt_model();
    flush_tlb();
    const uint32_t satp_ppn = 0x19;
    install_leaf(satp_ppn, 0x60, 3, 0x8600, true, false, true, true /*U=1*/);
    uint64_t satp = make_satp(satp_ppn);
    uint64_t va = 0x60ULL << 12;

    WalkResult ru = query_itlb_mmu(va, satp, 0 /*U*/);
    check(ru.pa_vld && !ru.page_fault, "U-mode fetch from its own U page succeeds",
          ru.pa_vld && !ru.page_fault, 1);

    flush_tlb();
    WalkResult rs_nosum = query_itlb_mmu(va, satp, 1 /*S*/, false, false);
    check(rs_nosum.pa_vld && rs_nosum.page_fault,
          "S-mode fetch from a U page faults with SUM=0",
          rs_nosum.pa_vld && rs_nosum.page_fault, 1);

    flush_tlb();
    WalkResult rs_sum = query_itlb_mmu(va, satp, 1 /*S*/, false, true);
    check(rs_sum.pa_vld && rs_sum.page_fault,
          "D-M4-10: S-mode fetch from a U page STILL faults with SUM=1 "
          "(SUM never excuses execution)",
          rs_sum.pa_vld && rs_sum.page_fault, 1);
    test_result("T19 ITLB walk + D-M4-10 SUM-never-excuses-fetch");
}

//=============================================================================
// main
//=============================================================================
int main(int argc, char **argv)
{
    Verilated::commandArgs(argc, argv);
    dut = new VMMU;

    reset_dut();

    test_dtlb_identity_map();
    test_itlb_identity_map();
    test_pma_dram();
    test_pma_clint();
    test_pma_plic();
    test_pma_uart();
    test_pma_reserved();
    test_pma_tohost();
    test_pma_boundaries();

    test_dtlb_4k_walk_and_hit();
    test_dtlb_2m_superpage();
    test_dtlb_1g_superpage();
    test_dtlb_priv_cross();
    test_dtlb_ad_bits();
    test_dtlb_invalid_and_buserr();
    test_dtlb_pmp_deny_during_walk();
    test_dtlb_noncanonical_va();
    test_satp_write_flush();
    test_itlb_walk_and_sum();

    printf("[mmu_tb] %llu cycles, %d failure(s)\n",
           (unsigned long long)g_cycles, g_fail);
    printf("%s\n", g_fail ? "UNIT-FAIL" : "UNIT-PASS");

    dut->final();
    delete dut;
    return g_fail ? 1 : 0;
}
