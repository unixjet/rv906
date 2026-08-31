// 7 Load/Store Unit (LSU) -- Non-blocking D-cache implementation

This document describes the **non-blocking** D-cache implementation in rv906's LSU module, which replaces the earlier blocking "single outstanding miss" path with a C906-inspired design featuring:
- **LFB (Load-Fill Buffer):** 8-entry multi-miss tracking with deferred load refills that don't block the main pipe
- **RDL + VB:** Replacement/victim engine decoupled from refill via single-entry victim buffer
- **PFB (Prefetch Buffer):** PC-indexed stride prefetcher for read-ahead
- **AMR (Adaptive Memory Restrictor):** Streaming-store detection to disable write-allocate when beneficial

## Architecture overview

```
┌─────────┐     ┌───────────────┐     ┌─────────────────────────────────────┐
│ IDU     │──→──│ LSU           │──→──│ AXI crossbar / memory slave         │
└─────────┘     └───────────────┘     └─────────────────────────────────────┘
                ↑      │
        ┌──────────────┴──────────────────────────────────────────────────┐
        │                                                                  │
   ┌────▼─────┐                                                         ┌─▼─┐
   │  STB     │   ←── dirty stores waiting for LFB drain ───────────────►│DC │
   │(store buf)│                                                             │ach│
   └──────────┘   ←── prefetch requests allocate here ────────────────────┘ └───┘
       ▲                                                                        ▲
       │                                        ┌───────────┐                  │
       ├───────┐                                │           │                  │
       │       ▼                                │    LFB    │◄─────┐         │
       │  ┌─────────────────────────────────────┤ (8 entries) │          │         │
       │  │                                     └───────────┘      │         │
       │  │                                                        │         │
       │  │                                  ┌─────────────────────┘         │
       │  │                                  │                               │
       │  └───────┐                          │                               │
       │          ▼                          ▼                               │
       │  ┌──────────────────┐  ┌───────────────────────────────────────────┐│
       │  │  PFB (prefetch) │  │  VB (victim buffer)                       ││
       │  │ 5 entries       │  │ Single entry: decouples writeback from    ││
       │  └─────────────────┘  │            demand-refill critical path    ││
       └──────────────────────┼────────────────────────────────────────────┘│
                              │                                              │
               ┌──────────────┴──────────────────────────────────────────────┘
               │
          ┌────▼────┐
          │  AMR    │ → gates dcache_wa after stride pattern detected
          │streaming│
          │detector│
          └─────────┘
```

## Key behaviors

### 1. Demand misses are deferred into the LFB (non-blocking)

When a cacheable load/store misses:
- **Loads:** Deferral creates an LFB entry (`E_PENDING`). The main FSM returns to `ST_IDLE` immediately (the miss runs in background). Cache hits continue to complete normally while the miss is in flight (**hit-under-miss**).
- **Stores with WA (write-allocate):** Create LFB + STB together; store data waits until LFB drains.
- **Stores without WA:** Go straight to direct AXI path (no allocation).

The LFB tracks up to 8 outstanding misses. Each entry has its own tag/index, state machine, and request context. Refills commit directly to the D-cache array; no second array probe is needed.

### 2. Background refill does NOT wait on dirty-victim writeback

The classic bottleneck for demand misses is the dirty-victim eviction writeback. In the original single-outstanding path, every miss blocked on the victim writeback finishing before issuing its refill read.

The new path uses a **single-entry VB (victim buffer)**:
1. When victim-select finds a dirty way, MS_VPEEK_ISSUE/WAIT hands off the victim data to the VB and moves **straight** to MS_REFILL_READ.
2. The refill read goes out immediately; the victim writeback drains asynchronously whenever axi_w_free cycles appear.
3. If the VB is occupied (single entry), re-evict retries at MS_IDLE (WVB stall).

**Benefit:** The demand-miss latency now includes only the refill-read round trip, not the victim writeback time. The VB ensures global ordering (fence waits for undrained VB via `!vb_vld` gate on `lsu_cp0_stb_empty`).

### 3. Prefetch buffer (PFB) trains on strided loads

The PFB detects regular load patterns and prefetches lines ahead of demand:
- **Training:** Loads to the same PC with consistent stride (|stride| < 1KB) train a tracker. After 3 matches (4-line stride) or 15 matches (global), it enters FUNC state.
- **Prefetch:** Issues LFB-create requests for upcoming lines at lookahead distance = stride × dist (MHINT.pref_dist). Prefetches saturate at 64-line distance (suspension).
- **Suppression:** Line must not be already in LFB/STB/VB/DC (redundant fills suppressed).
- **Control:** MHINT CSR bit2 enables/disables (clear flushes all entries). Default OFF (M2 base stays unchanged).

### 4. AMR streaming-store detector disables write-allocate

For memset-like streaming writes, allocating a line you'll evict moments later is wasteful. The AMR detects store-size-strided streams and disables WA once confirmed:
- **Training:** Stores with matching byte-counts and strides count toward line completion.
- **Confirmation:** After `N` lines (threshold from MHINT.amr), the FSM enters FUNC state.
- **Disable:** FUNC raises `amr_dc_wa_dis`, making `dcache_wa = cp0_wa & !amr_dc_wa_dis`. Subsequent store misses bypass allocation, write directly through.
- **Exit:** Stride break causes confidence decay; loss of confidence exits FUNC.

Default threshold = 4 lines (MHINT.amr = 2'b01). AMR is stored but consumer-controlled until M4 privilege exposes control.

## Bus serialization adaptation

rv906's SoC-level AXI is single-port, single-outstanding:
- All transactions (reads AND writes) serialize on `AXICrossbar.v sr_active`.
- No ARID/ARID arbitration (requests have no outstanding IDs).
- Multiple LFB entries can track multiple misses, but only ONE fills at a time.

This differs from the donor (which would support parallel refs on multi-port memory). However, it's functionally equivalent: the donor LFB on a single-outstanding bus also issues one refill at a time. Our simpler circular-FIFO head/tail pointer (vs donor's round-robin create ptr) captures this degenerate behavior faithfully.

### Adaptation decision #1: Bus serialization
**File:** `rtl/AXICrossbar.v` (sr_active), `rtl/LSU.v` (MS_REFILL_READ sequentialism)
**Source:** Single-port slave on all slaves; verified no arid/rid registers anywhere in crossbar/vlram.
**Impact:** No functional change on correct code paths; simplifies arbitration logic.

### Adaptation decision #2: 512-bit single-beat refill
**File:** `rtl/LSU.v` (arlen=0, arsize=6); reader reads whole line.
**Source:** Donor uses 4×128b WRAP (4 beats).
**Impact:** Critical-word-first degenerates (whole line arrives as one beat, so no special handling needed).

### Adaptation decision #3: PIPT/no alias
**File:** `rtl/DCache.v` header notes "single bank, PA-based tag", `rtl/LSU.v` (VB only dirty-writeback role).
**Source:** Donor has virtual-indexed alias-handling (ALIAS read-back path, vb_pfb hit check for alias).
**Impact:** VB does NOT need alias lookup or per-entry tagging beyond the tag/index used by normal misses.

## Direct-store handling

Uncached (direct) stores use a dedicated AXI write path separate from the FRZ refill:
- Direct stores claim axi_w_free; VB handles background writebacks (mutually exclusive gates: FRZ claims when !vb_vld, VB claims when vb_vld==1).
- FENCE.I clean-walk also claims (but never concurrently thanks to lsu_cp0_stb_empty gating).

No explicit ownership-tag register is needed; mutual exclusion relies on the three mutually-exclusive preconditions (see VB section above).

## Integration points

### From IDU/RTU
- `idu_lsu_ex1_sel/sel`: LSUs side effects gated by `_sel` (not `_dp_sel`) — LPF contract.
- `rtu_iu_ex2_dest_reg`: Store destination forwarded back for merge.
- PC broadcast to PFB trainer (`iu_lsu_ex1_cur_pc`).

### From CSR
- `cp0_lsu_dcache_en`: Enables/disables entire D-cache access (boot-time zero, set after init).
- `cp0_lsu_wa`: Global WA enable (Task E: gated further by AMR).
- `cp0_lsu_dcache_pref_en` / `pref_dist`: PFB controls (CSR 0x7C5 bits 2, [14:13]).
- `cp0_lsu_amr`: AMR threshold control (CSR 0x7C5 bits [4:3]).

## Test coverage (lsu_tb.cpp T1–T13)

| Test | Scenario                                          | Validation                         |
|------|---------------------------------------------------|------------------------------------|
| T5   | Dirty-victim writeback ordering                   | Writeback fires first, then refill |
| T6   | Hit-under-miss (HIT load during MISS refill)      | Pipe frees, hit completes          |
| T7   | Multi-miss tracking                                 | Two misses tracked, correct order  |
| T8   | Same-line replay (load→store→load sequence)       | Replay blocks, final data correct  |
| T9   | LFB-full backpressure                             | 9th miss parked, others succeed    |
| T10  | VB decoupling (refill ahead of writeback)         | Read accepted before write         |
| T11  | PFB ahead-prefetch                                | Prefetched line hit, no extra read |
| T12  | AMR wa disable                                    | Training window allocates, rest pass-through |
| T13  | Store under miss (parks, retries, drains forward)| Data visible after retry           |

Full acceptance gates passed: `make verisim` clean, unit suite UNIT-SUITE-PASS, sweep 86/87 (documented failure `rv64ui-p-ma_data`), atomic suite 19/19.

---

**References:** 
- Plan: `docs/plans/2026-08-31-m3b-full-lsu.md` (task breakdown, donor file mappings)
- Verification: `docs/08-verification.md` (§8.14 M3b acceptance criteria)
