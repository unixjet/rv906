//=============================================================================
// dcache_tb.cpp - standalone unit bench for rtl/DCache.v (M2 plan task 6.4)
//=============================================================================
// Verilates DCache.v + SRAM.v + rvproc_pkg.sv alone (no LSU, no AXI -- this
// module has no AXI ports at all, see DCache.v's own header) and drives the
// frozen request/response/invalidate ports directly with a scripted driver,
// mirroring test/m1/unit/icache_tb.cpp's overall harness shape (tick(),
// check()/test_result() bookkeeping) even though there is no golden AXI
// memory here -- DCache.v never touches memory itself; LSU.v's own refill/
// victim FSM does that externally and just writes results in through this
// module's plain array interface (dc_req_wr=1/alloc=1). This bench verifies
// exactly that interface: tag compare, per-way valid/dirty tracking, the
// way-select-or-hit-way response mux (used for both ordinary hits and
// LSU's victim-peek mechanism), invalidate, and the IDLE->DCS->FRZ->REPLY
// FSM's one genuine collision (an invalidate arriving the same cycle as a
// new request).
//
// Build/run: make -C test/m2/unit dcache && bin/unit/dcache_tb
// Prints one line per test and ends with UNIT-PASS or UNIT-FAIL.
//=============================================================================

#include <verilated.h>
#include "VDCache.h"

#include <cstdio>
#include <cstdint>
#include <cstring>

static const unsigned WAYS  = 4;
static const unsigned TAGW  = 27;
static const unsigned IDXW  = 7;

static VDCache *dut = nullptr;
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

static void idle_ports(void)
{
    dut->dc_req_vld       = 0;
    dut->dc_req_index     = 0;
    dut->dc_req_tag       = 0;
    dut->dc_req_wr        = 0;
    dut->dc_req_way_sel   = 0;
    for (int i = 0; i < 16; i++) dut->dc_req_wdata[i] = 0;
    dut->dc_req_wstrb     = 0;
    dut->dc_req_dirty_set = 0;
    dut->dc_req_alloc     = 0;
    dut->dc_inv_vld       = 0;
    dut->dc_inv_index     = 0;
    dut->dc_inv_way_sel   = 0;
}

static void reset_dut(void)
{
    dut->clk   = 0;
    dut->rst_n = 0;
    idle_ports();
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
            printf("    FAIL %-56s got=0x%llx exp=0x%llx (cycle %llu)\n", what,
                   (unsigned long long)got, (unsigned long long)exp,
                   (unsigned long long)g_cycles);
        g_fail++;
    }
}

static void test_result(const char *name)
{
    printf("[dcache_tb] %-54s %s\n", name, g_local ? "FAIL" : "PASS");
    g_local = 0;
}

//=============================================================================
// Request/response driver.
//=============================================================================
struct DcResp {
    bool     vld = false;
    uint8_t  hit_way = 0, way_vld = 0, way_dirty = 0;
    uint32_t victim_tag = 0;
    uint64_t rdata_lo = 0;   // low 64 bits of the responded line (identity marker)
    int      cycles = 0;
};

// marker: a 64-bit value placed at word[0]/word[1] of the 512b line so a
// simple 64-bit compare is enough to prove the right data came back.
static void set_marker(uint64_t marker)
{
    for (int i = 0; i < 16; i++) dut->dc_req_wdata[i] = 0;
    dut->dc_req_wdata[0] = (uint32_t)(marker & 0xFFFFFFFFu);
    dut->dc_req_wdata[1] = (uint32_t)(marker >> 32);
}

// Issues one request (one-cycle pulse, matching LSU.v's own usage pattern)
// and waits for dc_resp_vld, capturing the response at that exact cycle.
// If inv_same_cycle is set, dc_inv_vld/inv_index/inv_way_sel also fire on
// the SAME cycle as dc_req_vld -- the one genuine port collision this
// module's FRZ state exists for.
static DcResp submit(unsigned index, unsigned tag, bool wr, unsigned way_sel,
                      uint64_t wdata_marker, uint64_t wstrb, bool dirty_set, bool alloc,
                      bool inv_same_cycle = false, unsigned inv_index = 0, unsigned inv_way_sel = 0,
                      int guard = 30)
{
    dut->dc_req_vld       = 1;
    dut->dc_req_index     = index;
    dut->dc_req_tag       = tag;
    dut->dc_req_wr        = wr ? 1 : 0;
    dut->dc_req_way_sel   = way_sel;
    set_marker(wdata_marker);
    dut->dc_req_wstrb     = wstrb;
    dut->dc_req_dirty_set = dirty_set ? 1 : 0;
    dut->dc_req_alloc     = alloc ? 1 : 0;
    if (inv_same_cycle) {
        dut->dc_inv_vld     = 1;
        dut->dc_inv_index   = inv_index;
        dut->dc_inv_way_sel = inv_way_sel;
    }
    tick();
    dut->dc_req_vld = 0;
    dut->dc_inv_vld = 0;

    DcResp r;
    for (int i = 0; i < guard; i++) {
        if (dut->dc_resp_vld) {
            r.vld        = true;
            r.hit_way    = dut->dc_resp_hit_way;
            r.way_vld    = dut->dc_resp_way_vld;
            r.way_dirty  = dut->dc_resp_way_dirty;
            r.victim_tag = dut->dc_resp_victim_tag;
            r.rdata_lo   = ((uint64_t)dut->dc_resp_rdata[1] << 32) | dut->dc_resp_rdata[0];
            r.cycles     = i + 1;
            tick();
            return r;
        }
        tick();
    }
    return r;   // timed out
}

static DcResp read_req(unsigned index, unsigned tag, unsigned way_sel = 0)
{
    return submit(index, tag, /*wr=*/false, way_sel, 0, 0, false, false);
}

static DcResp write_req(unsigned index, unsigned tag, unsigned way_sel, uint64_t marker,
                         uint64_t wstrb, bool dirty_set, bool alloc)
{
    return submit(index, tag, /*wr=*/true, way_sel, marker, wstrb, dirty_set, alloc);
}

static bool invalidate(unsigned index, unsigned way_sel, int guard = 20)
{
    dut->dc_inv_vld     = 1;
    dut->dc_inv_index   = index;
    dut->dc_inv_way_sel = way_sel;
    tick();
    dut->dc_inv_vld = 0;
    for (int i = 0; i < guard; i++) {
        if (dut->dc_inv_done) { tick(); return true; }
        tick();
    }
    return false;
}

static unsigned onehot(unsigned way) { return 1u << way; }

//=============================================================================
// Tests
//=============================================================================

// T1: a fresh line write (alloc=1) then a plain read at the same index/tag
// hits in the written way, with the right data and dirty=0 (a clean fill).
static void test_alloc_then_hit(void)
{
    invalidate(5, 0xF);
    DcResp w = write_req(5, 0x1000001, onehot(0), 0xAAAABBBBCCCCDDDDULL, 0xFFFFFFFFFFFFFFFFULL,
                          /*dirty_set=*/false, /*alloc=*/true);
    check(w.vld, "alloc write acked", w.vld, 1);

    DcResp r = read_req(5, 0x1000001);
    check(r.vld, "read after alloc acked", r.vld, 1);
    check(r.hit_way == onehot(0), "hits in way0", r.hit_way, onehot(0));
    check(r.rdata_lo == 0xAAAABBBBCCCCDDDDULL, "data equals what was written",
          r.rdata_lo, 0xAAAABBBBCCCCDDDDULL);
    check((r.way_dirty & onehot(0)) == 0, "clean fill: way0 not dirty", r.way_dirty & onehot(0), 0);
    check(r.cycles == 1, "clean hit resolves in 1 cycle (IDLE->DCS, combinational response)", (uint64_t)r.cycles, 1);

    test_result("T1 alloc write -> plain read hits with correct data");
}

// T2: fill all 4 ways of one set with 4 distinct tags; all 4 remain
// independently resident (no self-eviction inside DCache.v -- it never
// replaces on its own, contract 9/11's "LSU owns replacement policy").
static void test_four_way_fill(void)
{
    const unsigned idx = 9;
    invalidate(idx, 0xF);
    uint32_t tags[4]    = {0x2000001, 0x2000002, 0x2000003, 0x2000004};
    uint64_t markers[4] = {0x1111111111111111ULL, 0x2222222222222222ULL,
                            0x3333333333333333ULL, 0x4444444444444444ULL};
    for (unsigned w = 0; w < WAYS; w++)
        write_req(idx, tags[w], onehot(w), markers[w], 0xFFFFFFFFFFFFFFFFULL, false, true);

    for (unsigned w = 0; w < WAYS; w++) {
        DcResp r = read_req(idx, tags[w]);
        check(r.hit_way == onehot(w), "each way independently hits", r.hit_way, onehot(w));
        check(r.rdata_lo == markers[w], "each way's data survives the other 3 fills",
              r.rdata_lo, markers[w]);
        check(r.way_vld == 0xF, "all 4 ways report valid once full", r.way_vld, 0xF);
    }

    // A 5th, different tag at the same (full) set misses -- exactly the
    // signal a real requester uses to decide it must evict.
    DcResp miss = read_req(idx, 0x2000005);
    check(miss.hit_way == 0, "5th distinct tag at a full set misses", miss.hit_way, 0);
    check(miss.way_vld == 0xF, "miss response still reports the full way_vld vector",
          miss.way_vld, 0xF);

    test_result("T2 4-way fill: independent residency + miss when full");
}

// T3: overwriting a way (alloc=1, new tag) evicts the old tag -- the old
// tag now misses, the new tag hits in the same way.
static void test_eviction_by_overwrite(void)
{
    const unsigned idx = 12;
    invalidate(idx, 0xF);
    write_req(idx, 0x3000001, onehot(2), 0xAAAAAAAAAAAAAAAAULL, 0xFFFFFFFFFFFFFFFFULL, false, true);
    DcResp before = read_req(idx, 0x3000001);
    check(before.hit_way == onehot(2), "old tag hits before eviction", before.hit_way, onehot(2));

    write_req(idx, 0x3000009, onehot(2), 0xBBBBBBBBBBBBBBBBULL, 0xFFFFFFFFFFFFFFFFULL, false, true);
    DcResp after_old = read_req(idx, 0x3000001);
    check(after_old.hit_way == 0, "old tag misses after eviction", after_old.hit_way, 0);
    DcResp after_new = read_req(idx, 0x3000009);
    check(after_new.hit_way == onehot(2), "new tag hits in the same way", after_new.hit_way, onehot(2));
    check(after_new.rdata_lo == 0xBBBBBBBBBBBBBBBBULL, "new tag's data is the new write",
          after_new.rdata_lo, 0xBBBBBBBBBBBBBBBBULL);

    test_result("T3 eviction by overwrite (alloc=1, new tag)");
}

// T4: dirty tracking -- a clean alloc, then a store-hit update (alloc=0)
// with dirty_set=1 marks the way dirty; the tag is unchanged.
static void test_dirty_tracking(void)
{
    const unsigned idx = 20;
    invalidate(idx, 0xF);
    write_req(idx, 0x4000001, onehot(1), 0x1234ULL, 0xFFFFFFFFFFFFFFFFULL, /*dirty=*/false, /*alloc=*/true);
    DcResp r0 = read_req(idx, 0x4000001);
    check((r0.way_dirty & onehot(1)) == 0, "clean fill: not dirty yet", r0.way_dirty & onehot(1), 0);

    // store-hit update: same way, same tag, alloc=0, dirty_set=1.
    // dc_req_wstrb is one bit PER BYTE across the whole 64-byte line (not
    // per bit) -- 0x1 enables only byte 0, leaving byte 1 (and the rest)
    // untouched.
    write_req(idx, 0x4000001, onehot(1), 0x5678ULL, 0x0000000000000001ULL, /*dirty=*/true, /*alloc=*/false);
    DcResp r1 = read_req(idx, 0x4000001);
    check(r1.hit_way == onehot(1), "still hits same way after store-hit update", r1.hit_way, onehot(1));
    check((r1.way_dirty & onehot(1)) != 0, "store-hit update marks the way dirty",
          (r1.way_dirty & onehot(1)) != 0, 1);
    check((r1.rdata_lo & 0xFF) == 0x78, "wstrb-masked byte 0 updated", r1.rdata_lo & 0xFF, 0x78);
    check(((r1.rdata_lo >> 8) & 0xFF) == 0x12, "byte 1 untouched by the partial-wstrb write "
          "(only byte0 was masked in)", (r1.rdata_lo >> 8) & 0xFF, 0x12);

    test_result("T4 dirty tracking + byte-granular wstrb on a store-hit update");
}

// T5: the way-select "peek" mechanism (dc_req_way_sel driven on a READ) --
// LSU's victim-writeback path uses exactly this to read out a dirty
// eviction candidate's tag+data before overwriting it, regardless of
// whether that way is the tag-compare hit.
static void test_way_peek(void)
{
    const unsigned idx = 30;
    invalidate(idx, 0xF);
    write_req(idx, 0x5000001, onehot(0), 0xA0A0A0A0A0A0A0A0ULL, 0xFFFFFFFFFFFFFFFFULL, true, true);
    write_req(idx, 0x5000002, onehot(3), 0xB0B0B0B0B0B0B0B0ULL, 0xFFFFFFFFFFFFFFFFULL, false, true);

    // Read with a DIFFERENT tag (a genuine miss for the compare) but peek
    // way3 explicitly -- the peek must return way3's real tag+data anyway.
    DcResp peek = read_req(idx, 0x5000099, onehot(3));
    check(peek.hit_way == 0, "peek request itself still misses the (unrelated) compare tag",
          peek.hit_way, 0);
    check(peek.victim_tag == 0x5000002u, "peek returns way3's real tag", peek.victim_tag, 0x5000002u);
    check(peek.rdata_lo == 0xB0B0B0B0B0B0B0B0ULL, "peek returns way3's real data",
          peek.rdata_lo, 0xB0B0B0B0B0B0B0B0ULL);
    check((peek.way_dirty & onehot(0)) != 0, "way0's dirty bit visible in the same response",
          (peek.way_dirty & onehot(0)) != 0, 1);

    test_result("T5 way-select peek mechanism (victim-writeback readout)");
}

// T6: invalidate clears valid (the previously-hit tag now misses).
static void test_invalidate_clears_valid(void)
{
    const unsigned idx = 40;
    invalidate(idx, 0xF);
    write_req(idx, 0x6000001, onehot(0), 0x1ULL, 0xFFFFFFFFFFFFFFFFULL, false, true);
    write_req(idx, 0x6000002, onehot(1), 0x2ULL, 0xFFFFFFFFFFFFFFFFULL, false, true);
    DcResp before = read_req(idx, 0x6000001);
    check(before.hit_way == onehot(0), "way0 resident before invalidate", before.hit_way, onehot(0));

    bool done = invalidate(idx, onehot(0));   // invalidate ONLY way0
    check(done, "invalidate reports done", done, 1);

    DcResp after0 = read_req(idx, 0x6000001);
    check(after0.hit_way == 0, "way0's tag misses after its own invalidate", after0.hit_way, 0);
    DcResp after1 = read_req(idx, 0x6000002);
    check(after1.hit_way == onehot(1), "way1 (not invalidated) still resident", after1.hit_way, onehot(1));
    check((after1.way_vld & onehot(0)) == 0, "way0's valid bit reads 0 post-invalidate",
          after1.way_vld & onehot(0), 0);

    test_result("T6 invalidate clears exactly the targeted way's valid bit");
}

// T7: the one genuine port collision -- dc_inv_vld arriving the SAME cycle
// as a new dc_req_vld. The FSM must take exactly one extra cycle (IDLE->
// FRZ->DCS, 2 cycles, vs. the clean IDLE->DCS 1 cycle -- the response is
// combinational at DCS, matching the donor SRAM's 1-cycle read latency) and
// BOTH operations must still complete correctly.
static void test_frz_collision(void)
{
    const unsigned idx = 50;
    invalidate(idx, 0xF);
    write_req(idx, 0x7000001, onehot(0), 0xCAFEULL, 0xFFFFFFFFFFFFFFFFULL, false, true);

    // Baseline: a clean, non-colliding read takes 1 cycle (combinational
    // response at DCS).
    DcResp baseline = read_req(idx, 0x7000001);
    check(baseline.cycles == 1, "baseline (no collision) resolves in 1 cycle",
          (uint64_t)baseline.cycles, 1);

    // Now issue a read at idx, WHILE simultaneously invalidating a
    // DIFFERENT set (idx2) the same cycle -- a genuine port collision.
    const unsigned idx2 = 51;
    DcResp collided = submit(idx, 0x7000001, /*wr=*/false, 0, 0, 0, false, false,
                              /*inv_same_cycle=*/true, idx2, 0xF);
    check(collided.vld, "colliding request still eventually completes", collided.vld, 1);
    check(collided.cycles == 2, "FRZ collision adds exactly one cycle (2 total, not 1)",
          (uint64_t)collided.cycles, 2);
    check(collided.hit_way == onehot(0), "colliding request's own answer is still correct",
          collided.hit_way, onehot(0));

    test_result("T7 invalidate-vs-request collision: FRZ adds exactly 1 cycle");
}

// MUTATION CHECK 1 (of this task's required >=2 across all three benches):
// this test is self-documenting proof the bench actually distinguishes the
// FRZ path from the clean path -- see the completion report for the
// temporarily-broken-then-reverted mutation this test caught (removing the
// "state <= dc_inv_vld ? ST_FRZ : ST_DCS" branch, collapsing straight to
// ST_DCS, made T7's cycle-count check fail as expected).

//=============================================================================
// main
//=============================================================================
int main(int argc, char **argv)
{
    Verilated::commandArgs(argc, argv);
    dut = new VDCache;

    reset_dut();

    test_alloc_then_hit();
    test_four_way_fill();
    test_eviction_by_overwrite();
    test_dirty_tracking();
    test_way_peek();
    test_invalidate_clears_valid();
    test_frz_collision();

    printf("[dcache_tb] %llu cycles, %d failure(s)\n",
           (unsigned long long)g_cycles, g_fail);
    printf("%s\n", g_fail ? "UNIT-FAIL" : "UNIT-PASS");

    dut->final();
    delete dut;
    return g_fail ? 1 : 0;
}
