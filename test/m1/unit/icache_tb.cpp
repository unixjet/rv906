//=============================================================================
// icache_tb.cpp - standalone unit bench for rtl/ICache.v (M1 plan task 2.2)
//=============================================================================
// Verilates ICache.v + SRAM.v + rvproc_pkg.sv alone (no IFU, no SoC) and
// drives the frozen fetch/refill/invalidate ports directly, with a
// behavioural AXI slave in front of a golden byte array. Pattern copied from
// rv12's test/m1/unit/icache_tb.cpp (raw Verilator C++ main, obj-dir-per-unit
// Makefile, tick()-based clocking, check()/test_result() bookkeeping) -- see
// that file for the harness shape this one follows. The DUT contract itself
// is rv906/C906-specific and NOT copied from rv12/C910's numbers:
//   - 256 sets (not 512), 8-bit index addr[13:6] (not 9-bit addr[14:6])
//   - ICache.v's request port is UNIFIED (pcgen_icache_va + ctrl_icache_req_
//     vld drives hit-check AND autonomously triggers a refill on a miss --
//     there is no separate "IP-stage miss request" port like rv12's
//     ipctrl_l1_refill_miss_req, because C906's real aq_ifu_icache.v is one
//     monolithic module, not split across IF/IP module pairs, IFU notes S0).
//   - ICache.v also owns the MMU-facing port group directly (IFU notes S1
//     SEAM NOTE), so THIS bench plays the M1 MMU stub itself (zero-latency
//     bare physical mapping, matching RVProc.v's own stub) rather than
//     driving a pre-translated ptag input the way rv12's bench did.
//   - No predecode array exists on this port list at all (IFU notes S3: RVC
//     boundaries are computed LIVE downstream in IPACK, not stored in
//     ICache). The plan's "live RVC-boundary detection vs a C++
//     reimplementation" bullet is therefore tested here as a DATA-INTEGRITY
//     check: reconstruct the byte stream ICache delivers word-by-word, walk
//     it with the C906 live-detection rule (inst[1:0]==2'b11 -> 4B, else
//     2B), and confirm the walk over the DELIVERED bytes matches the SAME
//     walk over the ORIGINAL golden bytes -- i.e. the boundary rule, applied
//     live downstream the way IPACK will apply it in Task 3, sees exactly
//     the same instruction stream ICache's real HW would hand it. There is
//     no predecode port to cross-check because C906 has none (see the
//     ICache.v header's IWPE-resolution note for the analogous "nothing to
//     build" finding on cp0_ifu_iwpe).
//
// Build/run:  make -C test/m1/unit icache && bin/unit/icache_tb
// Prints one line per test and ends with UNIT-PASS or UNIT-FAIL.
//=============================================================================

#include <verilated.h>
#include "VICache.h"

#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <random>
#include <vector>

//-----------------------------------------------------------------------------
// Geometry (mirrors rvproc_pkg.sv; the bench must know it to build addresses)
//-----------------------------------------------------------------------------
static const unsigned LINE_BYTES = 64;
static const unsigned SETS       = 256;                       // ICACHE_SETS
static const unsigned SET_SPAN   = 1u << 14;                   // 2^14 = 16KB: addr[13:6] repeats every 16KB (see T3)
static const unsigned WAY_BYTES  = SETS * LINE_BYTES;          // 16 KB per way

static const uint64_t MEM_BASE = 0x0000000080000000ULL;
static const size_t   MEM_SIZE = 512 * 1024;                   // several set-aliasing periods, margin past 0x40000

//=============================================================================
// Golden memory
//=============================================================================
static std::vector<uint8_t> g_mem(MEM_SIZE, 0);

static inline uint8_t mem_rd(uint64_t addr)
{
    uint64_t off = addr - MEM_BASE;
    return (off < MEM_SIZE) ? g_mem[off] : 0;
}

static inline void mem_wr(uint64_t addr, uint8_t v)
{
    uint64_t off = addr - MEM_BASE;
    if (off < MEM_SIZE) g_mem[off] = v;
}

static inline uint32_t mem_rd_word(uint64_t addr)
{
    return (uint32_t)mem_rd(addr) | ((uint32_t)mem_rd(addr + 1) << 8) |
           ((uint32_t)mem_rd(addr + 2) << 16) | ((uint32_t)mem_rd(addr + 3) << 24);
}

//=============================================================================
// Live RVC-boundary walk (C906: inst[1:0]==2'b11 -> 32-bit, else 16-bit --
// NOT a precomputed predecode array, IFU notes S3). Independent
// reimplementation, used only by test_rvc_boundary_live() below.
//=============================================================================
static std::vector<unsigned> walk_boundaries(const uint8_t *bytes, size_t len)
{
    std::vector<unsigned> lens;
    size_t pos = 0;
    while (pos + 2 <= len) {
        uint16_t hw = (uint16_t)(bytes[pos] | (bytes[pos + 1] << 8));
        bool is32 = (hw & 0x3u) == 0x3u;
        unsigned l = is32 ? 4u : 2u;
        if (is32 && pos + 4 > len) break;   // straddles the end of the window
        lens.push_back(l);
        pos += l;
    }
    return lens;
}

//=============================================================================
// DUT plumbing
//=============================================================================
static VICache *dut = nullptr;
static uint64_t g_cycles = 0;

// M1 MMU stub state (bench-controlled equivalent of RVProc.v's zero-latency
// bare physical mapping): mmu_ifu_prot[4:0] = {pgflt,supv,ca,ba,sec}, pinned
// in ICache.v's own header from icache.v's actual consumers.
static bool g_mmu_fault    = false;
static uint8_t g_mmu_prot  = 0x06;     // pgflt=0,supv=0,ca=1,ba=1,sec=0

static void drive_mmu(void)
{
    uint64_t va = dut->ifu_mmu_va;
    dut->mmu_ifu_pa           = (uint32_t)(va & 0x0FFFFFFFULL);   // bare mapping, low 28b
    dut->mmu_ifu_pa_vld       = dut->ifu_mmu_va_vld;
    dut->mmu_ifu_access_fault = g_mmu_fault ? 1 : 0;
    dut->mmu_ifu_prot         = g_mmu_prot;
}

//=============================================================================
// Behavioural AXI read slave: ONE 512b single-beat read per request, always
// (deviation 1 -- rv906's bus works in 512b/64B granules regardless of
// cacheable, ICache.v header). Single outstanding request by construction.
//=============================================================================
struct AxiSlave {
    bool     busy = false, rvalid = false, rerr = false;
    int      cnt = 0;
    uint64_t addr = 0;

    int      delay      = 2;
    bool     inject_err = false;

    int      reads = 0;
    uint64_t a_addr = 0;
    unsigned a_size = 0, a_len = 0, a_burst = 0, a_cache = 0, a_prot = 0;

    void reset() { busy = rvalid = rerr = false; cnt = 0; reads = 0; }

    void sample(VICache *d)
    {
        if (rvalid && d->axi_i_rready) {
            rvalid = false;
            busy   = false;
        } else if (!busy && d->axi_i_arvalid && d->axi_i_arready) {
            busy    = true;
            addr    = d->axi_i_araddr;
            a_addr  = addr;
            a_size  = d->axi_i_arsize;
            a_len   = d->axi_i_arlen;
            a_burst = d->axi_i_arburst;
            a_cache = d->axi_i_arcache;
            a_prot  = d->axi_i_arprot;
            cnt     = delay;
            reads++;
        } else if (busy && !rvalid && cnt > 0) {
            cnt--;
        }
    }

    void drive(VICache *d)
    {
        if (busy && !rvalid && cnt == 0) {
            rvalid = true;
            rerr   = inject_err;
            put_data(d);
        }
        d->axi_i_arready = (!busy && !rvalid) ? 1 : 0;
        d->axi_i_rvalid  = rvalid ? 1 : 0;
        d->axi_i_rlast   = rvalid ? 1 : 0;
        d->axi_i_rresp   = rerr ? 0x2 : 0x0;
    }

    void put_data(VICache *d)
    {
        for (int wi = 0; wi < 16; wi++)
            d->axi_i_rdata[wi] = mem_rd_word(addr + (uint64_t)wi * 4);
    }
};
static AxiSlave g_slave;

//=============================================================================
// Clocking
//=============================================================================
static void tick(void)
{
    drive_mmu();
    dut->eval();
    g_slave.sample(dut);
    dut->clk = 1;
    dut->eval();                 // registers commit here (icache_rd_addr, FSM, ...)
    drive_mmu();                 // ifu_mmu_va just changed -- re-settle same cycle
    dut->eval();
    g_slave.drive(dut);
    dut->eval();
    dut->clk = 0;
    dut->eval();
    g_cycles++;
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
        if (g_fail < 20)
            printf("    FAIL %-46s got=0x%llx exp=0x%llx (cycle %llu)\n", what,
                   (unsigned long long)got, (unsigned long long)exp,
                   (unsigned long long)g_cycles);
        g_fail++;
    }
}

static void test_result(const char *name)
{
    printf("[icache_tb] %-50s %s\n", name, g_local ? "FAIL" : "PASS");
    g_local = 0;
}

//=============================================================================
// Port drivers
//=============================================================================
struct FetchResult {
    bool     vld = false;      // icache_ipack_inst_vld pulsed within the guard
    uint32_t inst = 0;
    bool     acc_err = false, pgflt = false, unalign = false;
    int      cycles = 0;       // ticks until the pulse
};

// Holds pcgen_icache_va steady (as a real PCGEN would while not granted) and
// keeps ctrl_icache_req_vld asserted every cycle until icache_ipack_inst_vld
// pulses (either the hit path or the miss->refill->bypass path) or the guard
// expires. This is the ONE fetch primitive: ICache.v's request port is
// unified (see file header), so there is no separate "issue" vs "miss
// request" call the way rv12's split IF/IP interface needed.
//
// Cross-checks the pulse against icache_pcgen_addr (= the registered fetch
// address the pulse actually belongs to) before accepting it: a fetch() call
// that starts while an EARLIER call's miss is still draining (bench holds
// ctrl_icache_req_vld across calls, exactly as a real PCGEN would hold a
// not-yet-granted PC) must not mistake that stale transaction's bypass for
// its own answer just because SOME pulse happened to arrive first.
//
// Deasserts ctrl_icache_req_vld before returning control to the caller: a
// real PCGEN would move on to a NEW address (whatever comes next) rather
// than re-requesting this one forever, and leaving the request asserted
// with a now-stale address across a test-helper call boundary would let
// this line silently re-fetch itself (or, worse, get evicted and refilled)
// during some LATER unrelated helper's ticks (e.g. invalidate_all()'s own
// settle loop), corrupting that helper's AXI-traffic bookkeeping. This is a
// bench hygiene fix, not an RTL change -- see the M2 bug this exact pattern
// caused, found and fixed while writing this bench.
static FetchResult fetch(uint64_t addr, int guard = 400)
{
    const uint64_t addr40 = addr & 0xFFFFFFFFFFULL;   // PC_WIDTH = 40
    FetchResult r;
    for (int i = 0; i < guard; i++) {
        dut->pcgen_icache_va         = addr;
        dut->pcgen_icache_seq_tag    = 0;    // iwpe-only input, unused (ICache.v header)
        dut->pcgen_icache_chgflw_vld = 0;    // iwpe-only input, unused (ICache.v header)
        dut->ctrl_icache_req_vld     = 1;
        dut->ctrl_icache_abort       = 0;
        tick();
        if (dut->icache_ipack_inst_vld && (uint64_t)dut->icache_pcgen_addr == addr40) {
            r.vld     = true;
            r.inst    = dut->icache_ipack_inst;
            r.acc_err = dut->icache_ipack_acc_err != 0;
            r.pgflt   = dut->icache_ipack_pgflt != 0;
            r.unalign = dut->icache_ipack_unalign != 0;
            r.cycles  = i + 1;
            dut->ctrl_icache_req_vld = 0;
            tick();
            return r;
        }
    }
    dut->ctrl_icache_req_vld = 0;
    tick();
    return r;   // r.vld == false: timed out
}

static void idle_fetch_ports(void)
{
    dut->pcgen_icache_va         = 0;
    dut->pcgen_icache_seq_tag    = 0;
    dut->pcgen_icache_chgflw_vld = 0;
    dut->ctrl_icache_req_vld     = 0;
    dut->ctrl_icache_abort       = 0;
}

static void invalidate_all(void)
{
    dut->cp0_ifu_icache_inv_req  = 1;
    dut->cp0_ifu_icache_inv_addr = 0;
    dut->cp0_ifu_icache_inv_type = 0;
    tick();
    dut->cp0_ifu_icache_inv_req = 0;

    int guard = 2000;
    bool done = false;
    while (guard-- > 0) {
        if (dut->ifu_cp0_icache_inv_done) { done = true; tick(); break; }
        tick();
    }
    check(done, "invalidate reported done", 0, 0);
    for (int i = 0; i < 4; i++) tick();   // settle past the FSM's return to idle
}

static void reset_dut(void)
{
    dut->clk   = 0;
    dut->rst_n = 0;
    idle_fetch_ports();
    dut->cp0_ifu_icache_en          = 1;
    dut->cp0_ifu_iwpe               = 0;   // M1 setting: no tag-hit buffer exists (ICache.v header)
    dut->cp0_ifu_icache_pref_en     = 0;
    dut->cp0_ifu_icache_inv_addr    = 0;
    dut->cp0_ifu_icache_inv_req     = 0;
    dut->cp0_ifu_icache_inv_type    = 0;
    dut->mmu_ifu_access_fault       = 0;
    dut->mmu_ifu_pa                 = 0;
    dut->mmu_ifu_pa_vld              = 0;
    dut->mmu_ifu_prot                = g_mmu_prot;
    dut->axi_i_arready               = 0;
    dut->axi_i_rvalid                = 0;
    dut->axi_i_rresp                 = 0;
    dut->axi_i_rlast                 = 0;
    for (int i = 0; i < 16; i++) dut->axi_i_rdata[i] = 0;
    g_slave.reset();

    for (int i = 0; i < 5; i++) tick();
    dut->rst_n = 1;
    for (int i = 0; i < 5; i++) tick();
}

//=============================================================================
// Helpers shared by the tests
//=============================================================================
static void fill_random_bytes(uint64_t addr, unsigned len, std::mt19937 &rng)
{
    for (unsigned i = 0; i < len; i++) mem_wr(addr + i, (uint8_t)(rng() & 0xFFu));
}

// Fetch every word of a resident line and confirm it equals golden memory.
static bool fetch_and_check_line(uint64_t line_addr, const char *what)
{
    bool ok = true;
    for (unsigned off = 0; off < LINE_BYTES; off += 4) {
        FetchResult f = fetch(line_addr + off);
        if (!f.vld) { check(false, what, 0, 0); ok = false; continue; }
        uint32_t exp = mem_rd_word(line_addr + off);
        if (f.inst != exp) { check(false, what, f.inst, exp); ok = false; }
    }
    return ok;
}

//=============================================================================
// Tests
//=============================================================================
static const uint64_t LINE_A = MEM_BASE + 0x1000;                  // set S
static const uint64_t LINE_B = LINE_A + SET_SPAN;                  // same set S, different tag
static const uint64_t LINE_C = LINE_A + 2u * SET_SPAN;              // same set S, third tag
static const uint64_t LINE_NOALIAS = LINE_A + 0x2000;               // addr[13] flipped -> different set
static const uint64_t LINE_UNCACHED = MEM_BASE + 0x30000;
static const uint64_t LINE_ERR      = MEM_BASE + 0x34000;

// T1: cold miss -> refill (bypass) -> normal hit, plus AXI request shape.
static void test_miss_refill_hit(void)
{
    std::mt19937 rng(0xC906u);
    fill_random_bytes(LINE_A, LINE_BYTES, rng);
    invalidate_all();

    g_slave.reads = 0;
    FetchResult f0 = fetch(LINE_A + 0x18);        // third word of the line
    check(f0.vld, "cold fetch eventually delivers (bypass)", f0.vld, 1);
    check(f0.inst == mem_rd_word(LINE_A + 0x18), "bypass data equals memory",
          f0.inst, mem_rd_word(LINE_A + 0x18));
    check(!f0.acc_err && !f0.pgflt, "no fault on a clean refill", 0, 0);

    check(g_slave.reads == 1, "one AXI read per line", g_slave.reads, 1);
    check(g_slave.a_addr == LINE_A, "araddr is the 64B line base", g_slave.a_addr, LINE_A);
    check(g_slave.a_size == 6, "arsize = 6 (64B)", g_slave.a_size, 6);
    check(g_slave.a_len == 0, "arlen = 0 (single beat)", g_slave.a_len, 0);
    check(g_slave.a_burst == 1, "arburst = INCR", g_slave.a_burst, 1);
    check(g_slave.a_cache == 0xF, "arcache = {ca,ca,1,ba} = 1111", g_slave.a_cache, 0xF);
    check(g_slave.a_prot == 0x6, "arprot = {1,1,supv} = 110", g_slave.a_prot, 0x6);

    // A fresh probe of the SAME line must now hit fast, through the array
    // (not a second bypass -- no new AXI transaction).
    g_slave.reads = 0;
    FetchResult f1 = fetch(LINE_A);
    check(f1.vld, "second fetch hits", f1.vld, 1);
    check(f1.inst == mem_rd_word(LINE_A), "array data equals memory", f1.inst, mem_rd_word(LINE_A));
    check(g_slave.reads == 0, "no bus traffic on the array hit", g_slave.reads, 0);
    check(f1.cycles <= 8, "array hit resolves quickly (no refill wait)", (uint64_t)f1.cycles, 8);

    fetch_and_check_line(LINE_A, "every word of the filled line reads back correctly");

    test_result("T1 cold miss -> refill (bypass) -> array hit");
}

// T2: the alias tag fills way1, the FIFO bit round-robins, and a third alias
// evicts way0 -- both-ways fill + FIFO replacement, verified through actual
// eviction (not just the tag bit). The FIFO pointer is a single SHARED bit
// per set that flips on every refill regardless of which tag is involved
// (icache.v:547, "same scheme as C910" per the design doc) -- it is a pure
// round-robin, not an occupancy-aware LRU/victim-select. That means a FOURTH
// alias miss in this set evicts way1 (line B) even though way0 (line C) was
// the most recently touched -- this test demonstrates that real, shipped
// behavior explicitly rather than assuming a smarter policy.
static void test_fifo_replacement(void)
{
    std::mt19937 rng(0xFEED1u);
    invalidate_all();
    fill_random_bytes(LINE_A, LINE_BYTES, rng);
    fill_random_bytes(LINE_B, LINE_BYTES, rng);
    fill_random_bytes(LINE_C, LINE_BYTES, rng);

    FetchResult a = fetch(LINE_A);
    check(a.vld && a.inst == mem_rd_word(LINE_A), "way0 fill (line A)", a.inst, mem_rd_word(LINE_A));

    FetchResult b = fetch(LINE_B);
    check(b.vld && b.inst == mem_rd_word(LINE_B), "way1 fill (line B, same set)", b.inst, mem_rd_word(LINE_B));

    // both survive
    g_slave.reads = 0;
    FetchResult a2 = fetch(LINE_A);
    FetchResult b2 = fetch(LINE_B);
    check(a2.vld && a2.inst == mem_rd_word(LINE_A), "line A still resident", a2.inst, mem_rd_word(LINE_A));
    check(b2.vld && b2.inst == mem_rd_word(LINE_B), "line B still resident", b2.inst, mem_rd_word(LINE_B));
    check(g_slave.reads == 0, "no refills needed for two resident ways", g_slave.reads, 0);

    // third alias must evict the round-robin victim (way0, filled first)
    g_slave.reads = 0;
    FetchResult c = fetch(LINE_C);
    check(c.vld && c.inst == mem_rd_word(LINE_C), "way0 replaced by line C", c.inst, mem_rd_word(LINE_C));
    check(g_slave.reads == 1, "line C required a fresh refill", g_slave.reads, 1);

    // line B (way1) must survive C's fill untouched -- check THIS before
    // touching line A again, since refetching A is itself another miss in
    // the same set and would advance the round-robin pointer again.
    g_slave.reads = 0;
    FetchResult b3 = fetch(LINE_B);
    check(b3.vld && b3.inst == mem_rd_word(LINE_B), "line B (way1) survived line C's fill", b3.inst, mem_rd_word(LINE_B));
    check(g_slave.reads == 0, "line B needed no refill after C's fill", g_slave.reads, 0);

    // line A was evicted by C -> a fresh refill is required
    g_slave.reads = 0;
    FetchResult a3 = fetch(LINE_A);
    check(a3.vld && a3.inst == mem_rd_word(LINE_A), "line A refetch delivers correct data (post-eviction)",
          a3.inst, mem_rd_word(LINE_A));
    check(g_slave.reads == 1, "line A had to be refilled (it was evicted)", g_slave.reads, 1);

    // the round-robin pointer kept moving: A's refill above used way1 (the
    // pointer's next slot after C used way0), so it evicted line B in turn.
    // This is the shipped one-bit-round-robin behavior, not a bug.
    g_slave.reads = 0;
    FetchResult b4 = fetch(LINE_B);
    check(b4.vld && b4.inst == mem_rd_word(LINE_B), "line B refetch delivers correct data (evicted by A's refill)",
          b4.inst, mem_rd_word(LINE_B));
    check(g_slave.reads == 1, "line B needed a refill -- round-robin evicted it, as designed",
          g_slave.reads, 1);

    test_result("T2 both-ways fill + FIFO round-robin replacement");
}

// T3: 256-set indexing (not 512, per rv12's C910 clone) -- addr+16KB (2^14)
// MUST alias into the same set (addr[13:6] is the whole index, addr[14] and
// up are tag-only); addr+8KB (2^13) MUST land in a different set and coexist.
static void test_alias_free_indexing(void)
{
    std::mt19937 rng(0xA11A5u);
    invalidate_all();
    fill_random_bytes(LINE_A, LINE_BYTES, rng);
    fill_random_bytes(LINE_B, LINE_BYTES, rng);           // = LINE_A + 16KB, same set
    fill_random_bytes(LINE_NOALIAS, LINE_BYTES, rng);     // = LINE_A + 8KB, different set

    FetchResult a  = fetch(LINE_A);
    FetchResult n  = fetch(LINE_NOALIAS);
    FetchResult b  = fetch(LINE_B);                       // this is the SAME-set alias of A

    check(a.vld && a.inst == mem_rd_word(LINE_A), "line A resident", a.inst, mem_rd_word(LINE_A));
    check(n.vld && n.inst == mem_rd_word(LINE_NOALIAS), "8KB-away line resident", n.inst, mem_rd_word(LINE_NOALIAS));
    check(b.vld && b.inst == mem_rd_word(LINE_B), "16KB-away line resident", b.inst, mem_rd_word(LINE_B));

    // A and the 8KB-away line must BOTH still be resident together (different
    // sets, no eviction) -- re-probe without touching B again.
    g_slave.reads = 0;
    FetchResult a2 = fetch(LINE_A);
    FetchResult n2 = fetch(LINE_NOALIAS);
    check(a2.vld && a2.inst == mem_rd_word(LINE_A), "8KB-apart line did not evict A", a2.inst, mem_rd_word(LINE_A));
    check(n2.vld && n2.inst == mem_rd_word(LINE_NOALIAS), "and vice versa", n2.inst, mem_rd_word(LINE_NOALIAS));
    check(g_slave.reads == 0, "no refills -- genuinely different sets", g_slave.reads, 0);

    // A and B (16KB apart) DO alias -- filling B must have used the second
    // way of A's set (from T2's mechanics), so both remain resident together
    // too (this confirms the alias exists at 16KB, the 256-set signature).
    g_slave.reads = 0;
    FetchResult a3 = fetch(LINE_A);
    FetchResult b3 = fetch(LINE_B);
    check(a3.vld && a3.inst == mem_rd_word(LINE_A), "16KB-alias A still resident (2nd way)", a3.inst, mem_rd_word(LINE_A));
    check(b3.vld && b3.inst == mem_rd_word(LINE_B), "16KB-alias B still resident (2nd way)", b3.inst, mem_rd_word(LINE_B));
    check(g_slave.reads == 0, "both halves of the 16KB alias coexist in 2 ways", g_slave.reads, 0);

    test_result("T3 256-set indexing: 16KB aliases, 8KB does not");
}

// T4: live RVC-boundary detection (inst[1:0]==2'b11) applied to the byte
// stream ICache delivers, vs. the SAME walk over the golden memory -- see
// file header for why this is a data-integrity check rather than a
// predecode-port cross-check (C906 has no predecode array).
static void test_rvc_boundary_live(void)
{
    std::mt19937 rng(0x1DC0DEu);
    invalidate_all();

    const uint64_t base = MEM_BASE + 0x40000;
    const unsigned NBYTES = 4 * LINE_BYTES;               // 4 lines, random insn mix
    std::vector<uint8_t> golden(NBYTES);
    for (unsigned i = 0; i < NBYTES; ) {
        unsigned pick = rng() % 3u;
        uint16_t hw;
        if (pick == 0)      hw = (uint16_t)((rng() & 0xFFFFu) | 0x0003u);   // force 32-bit half
        else if (pick == 1) hw = (uint16_t)((rng() & 0xFFFCu) | 0x0001u);   // force 16-bit (RVC quad1)
        else                hw = (uint16_t)((rng() & 0xFFFCu) | 0x0000u);   // force 16-bit (RVC quad0)
        golden[i]     = (uint8_t)(hw & 0xFFu);
        golden[i + 1] = (uint8_t)((hw >> 8) & 0xFFu);
        mem_wr(base + i, golden[i]);
        mem_wr(base + i + 1, golden[i + 1]);
        i += 2;
    }

    std::vector<uint8_t> delivered(NBYTES);
    for (unsigned off = 0; off < NBYTES; off += 4) {
        FetchResult f = fetch(base + off);
        check(f.vld, "boundary-test word delivered", f.vld, 1);
        delivered[off]     = (uint8_t)(f.inst & 0xFFu);
        delivered[off + 1] = (uint8_t)((f.inst >> 8) & 0xFFu);
        delivered[off + 2] = (uint8_t)((f.inst >> 16) & 0xFFu);
        delivered[off + 3] = (uint8_t)((f.inst >> 24) & 0xFFu);
    }

    std::vector<unsigned> lens_golden    = walk_boundaries(golden.data(), NBYTES);
    std::vector<unsigned> lens_delivered = walk_boundaries(delivered.data(), NBYTES);

    check(lens_delivered.size() == lens_golden.size(),
          "live boundary walk: same instruction count",
          lens_delivered.size(), lens_golden.size());
    size_t n = std::min(lens_delivered.size(), lens_golden.size());
    unsigned mismatches = 0;
    for (size_t i = 0; i < n; i++)
        if (lens_delivered[i] != lens_golden[i]) mismatches++;
    check(mismatches == 0, "live boundary walk: every length matches", mismatches, 0);
    check(delivered == std::vector<uint8_t>(golden), "delivered bytes equal golden bytes (raw)", 0, 0);

    test_result("T4 live RVC-boundary detection vs. C++ reimplementation");
}

// T5: INV_ALL clears every valid+FIFO bit; a resident line must re-miss and
// go back to the bus afterwards.
static void test_invalidate_all(void)
{
    std::mt19937 rng(0x9A11u);
    invalidate_all();
    fill_random_bytes(LINE_A, LINE_BYTES, rng);

    FetchResult before = fetch(LINE_A);
    check(before.vld, "line resident before fence.i", before.vld, 1);
    g_slave.reads = 0;
    FetchResult before2 = fetch(LINE_A);
    check(before2.vld && g_slave.reads == 0, "confirmed resident (no refill)", g_slave.reads, 0);

    uint64_t c0 = g_cycles;
    invalidate_all();
    uint64_t span = g_cycles - c0;
    check(span >= SETS, "walk visits at least SETS cycles worth of sets", span, SETS);

    g_slave.reads = 0;
    FetchResult after = fetch(LINE_A);
    check(after.vld, "line still delivers data (re-miss + refill)", after.vld, 1);
    check(g_slave.reads == 1, "INV_ALL forced a fresh bus read", g_slave.reads, 1);
    check(after.inst == mem_rd_word(LINE_A), "data after re-miss equals memory", after.inst, mem_rd_word(LINE_A));

    test_result("T5 INV_ALL then re-miss");
}

// T6: an uncacheable fetch (mmu_ifu_prot[2]=0) still gets its data via the
// bypass path but never allocates into the array.
static void test_uncacheable(void)
{
    std::mt19937 rng(0x5555u);
    invalidate_all();
    fill_random_bytes(LINE_UNCACHED, LINE_BYTES, rng);

    uint8_t save_prot = g_mmu_prot;
    g_mmu_prot = (uint8_t)(save_prot & ~0x04u);     // clear the cacheable bit only

    g_slave.reads = 0;
    FetchResult f = fetch(LINE_UNCACHED + 0x10);
    check(f.vld, "uncached fetch delivers data", f.vld, 1);
    check(f.inst == mem_rd_word(LINE_UNCACHED + 0x10), "uncached data equals memory",
          f.inst, mem_rd_word(LINE_UNCACHED + 0x10));
    check(g_slave.reads == 1, "one AXI read for the uncached fetch", g_slave.reads, 1);
    check(g_slave.a_cache == 0x3, "arcache = {0,0,1,ba} (not allocating)", g_slave.a_cache, 0x3);

    g_mmu_prot = save_prot;                          // restore cacheable for the re-probe

    g_slave.reads = 0;
    FetchResult again = fetch(LINE_UNCACHED + 0x10);
    check(again.vld, "re-probe (now cacheable) still delivers data", again.vld, 1);
    check(g_slave.reads == 1, "no allocation happened -- this is a fresh miss",
          g_slave.reads, 1);

    test_result("T6 uncacheable fetch: bypass data, no allocation");
}

// T7: an AXI bus error faults the fetch (acc_err) and does not allocate.
static void test_bus_error(void)
{
    std::mt19937 rng(0xE44u);
    invalidate_all();
    fill_random_bytes(LINE_ERR, LINE_BYTES, rng);

    g_slave.inject_err = true;
    g_slave.reads = 0;
    FetchResult f = fetch(LINE_ERR);
    check(f.vld, "errored fetch still reports (with acc_err)", f.vld, 1);
    check(f.acc_err, "rresp error raises acc_err", f.acc_err, 1);
    g_slave.inject_err = false;

    g_slave.reads = 0;
    FetchResult retry = fetch(LINE_ERR);
    check(retry.vld && !retry.acc_err, "retry (no error) succeeds cleanly", retry.acc_err, 0);
    check(g_slave.reads == 1, "retry required a fresh bus read (no bad allocation)",
          g_slave.reads, 1);
    check(retry.inst == mem_rd_word(LINE_ERR), "retry data equals memory",
          retry.inst, mem_rd_word(LINE_ERR));

    test_result("T7 AXI rresp error -> acc_err, no allocation");
}

//=============================================================================
// main
//=============================================================================
int main(int argc, char **argv)
{
    Verilated::commandArgs(argc, argv);
    dut = new VICache;

    std::mt19937 seed_rng(0xBEEF);
    fill_random_bytes(MEM_BASE, MEM_SIZE, seed_rng);

    reset_dut();
    invalidate_all();          // the boot array clear (aq_ifu_vec.v RESET)

    test_miss_refill_hit();
    test_fifo_replacement();
    test_alias_free_indexing();
    test_rvc_boundary_live();
    test_invalidate_all();
    test_uncacheable();
    test_bus_error();

    printf("[icache_tb] %llu cycles, %d failure(s)\n",
           (unsigned long long)g_cycles, g_fail);
    printf("%s\n", g_fail ? "UNIT-FAIL" : "UNIT-PASS");

    dut->final();
    delete dut;
    return g_fail ? 1 : 0;
}
