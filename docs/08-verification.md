// 8 Verification gates (M3b complete)

## §8.14 M3b full LSU acceptance (§7.4 "Complete LSU")

After implementing the full non-blocking LSU (Tasks A–G: LFB rework, RDL + VB decoupling, PFB stride prefetch + MHINT, AMR streaming-store disabler), verify against the following gates:

### Gate chain (must pass all):

```bash
# 1. Clean Verilog build
make verisim 2>&1 | grep -icE "%error|error:"    # should be 0

# 2. Unit suite (lsu_tb.cpp T1–T13)
make -C test/m2/unit run                          # UNIT-SUITE-PASS

# 3. Full regression sweep (rv64ui-p / rv64um-p / rv64uc-p / rv64ua-p)
bash test/m2/run_all.sh                           # 86/87 (only documented rv64ui-p-ma_data)

# 4. Atomic operation tests
bash /tmp/run_atomic.sh                           # PASS=19 FAIL=0
```

### Expected results (green = pass, red = known failure):

| Test suite               | Pass   | Fail   | Notes                               |
|--------------------------|--------|--------|-------------------------------------|
| `make verisim`           | 0 errors | —    | No new Verilator warnings           |
| Unit suite (T1–T13)      | PASS   | —      | 13 directed tests covering all paths|
| Sweep (build)            | 86     | 1      | `rv64ui-p-ma_data` documented       |
| RV64U atomic (lrsc + 18) | 19     | 0      | All atomic ops correct              |

### Directed-test coverage (lsu_tb.cpp):

| Test | Scenario                                          | Block(s) validated                      |
|------|---------------------------------------------------|-----------------------------------------|
| T5   | Single-outstanding miss + dirty-victim writeback  | FRZ path, victim-select                 |
| T6   | Hit-under-miss (HIT load during MISS refill)      | LFB deferral, hit path                  |
| T7   | Multi-miss tracking (two misses)                  | LFB multi-entry tracking                |
| T8   | Miss-to-in-flight-line replay                     | LFB addr_hit, replay logic              |
| T9   | LFB-full backpressure                             | Backpressure on full LFB                |
| T10  | VB decoupled dirty-writeback (refill→writeback)   | VB handoff, refilling before eviction   |
| T11  | PFB stride prefetch ahead + hit                   | PC-stride training, lookahead, hitpath  |
| T12  | AMR wa disable after stride training              | AMR FSM FUNC state, gate dcache_wa      |
| T13  | STB forward under miss                            | Store parks, retries, drains to cache   |

### Gate-level behavior verification:

1. **LFB 8-entry non-blocking:** Deferred loads do NOT block the main pipe; hits complete while a miss is in flight. Multiple misses tracked, completion order matches issue order (bus serialization).
2. **Dirty-victim decoupling:** When a demand miss evicts a dirty victim, the refill read happens BEFORE the writeback (not blocked on it); fence waits for undrained VB writeback (`!vb_vld`).
3. **Prefetch:** With pref_en=1, strided stores train the PFB which prefetches lines ahead of demand; prefetched lines hit. With pref_en=0, no prefetch traffic.
4. **AMR write-allocate disable:** After `N` line-completion events (threshold from MHINT.amr), subsequent store misses bypass allocation and write directly through.
5. **STB consolidation:** Stores that replay behind a deferred load retry after drain, land in STB, and forward data to subsequent loads (no redundant cache writes).

### Known adaptation differences (documented):

- Bus serialized via single-port AXI (donor would support parallel refs on multi-port memory); this does NOT change functional behavior but simplifies implementation.
- 512-bit single-beat refill (vs donor's 4×128b WRAP); critical-word-first degenerates into whole-line return.
- PIPT D-cache (single-bank, no aliasing); VB only handles dirty-victim writeback, not alias read-back.

---

**Verification log (this session):** `git status` shows clean working tree after committing all Task F changes. All gates verified green in the preceding commands. M3b fully implemented and tested.
