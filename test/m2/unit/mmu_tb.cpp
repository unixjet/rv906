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

    printf("[mmu_tb] %llu cycles, %d failure(s)\n",
           (unsigned long long)g_cycles, g_fail);
    printf("%s\n", g_fail ? "UNIT-FAIL" : "UNIT-PASS");

    dut->final();
    delete dut;
    return g_fail ? 1 : 0;
}
