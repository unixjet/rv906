# M1: IFU + ICache + BPU — Design

Date: 2026-08-20
Status: Draft for review
Parent: `2026-08-20-rv906-c906-clone-design.md` (§7.4 row M1)
Extraction notes (normative source of C906 facts, keep open while implementing):
`notes/2026-08-20-c906-ifu-pipeline-extraction.md`, `notes/2026-08-20-c906-bpu-extraction.md`

## 1. Goal

Clone C906's front end — fetch pipeline, 32KB L1 instruction cache, and the
branch-prediction stack (BHT + BTB + RAS) — as clean rv906 RTL, verified
standalone inside the M0 simulation scaffold by directed fetch/prediction
tests, **before** any decode or execute stage exists. Pass bar (parent spec):
directed fetch/prediction tests green.

**This front end is architecturally much simpler than C910's** (extraction
notes §0): no IF/IP/IB three-stage split with per-stage ctrl/dp pairs — one
small shared control hub (`aq_ifu_ctrl.v`, 132 lines) and a single monolithic
ICache module with its own internal 2-cycle access. Most backpressure is
direct point-to-point wiring between adjacent modules, not routed through the
hub. The predictor is smaller too: BHT is a single 1024×16 pure-GHR direction
table (no PC-hash despite the manual's "Gshared" name), BTB is a 16-entry
flop-based fully-associative CAM (not SRAM), RAS is 4 flop entries (not
SRAM). Do not import C910/rv12's IF/IP/IB stage names or BPU geometry by
habit — every number below is cited from the C906 extraction notes.

## 2. Scope

### 2.1 In scope (aligned to C906 per the extraction notes)

- **Pipeline**: PCGEN (next-PC arbiter) → ICache access (internal 2-cycle
  request/hit-check, monolithic module) → IPACK (packs the ICache's 32-bit/
  cycle output into up to three 16-bit halfword slots) → IBUF (6-entry ×
  16-bit circular halfword queue, push ≤3/pop ≤2 halfwords per cycle) →
  exactly **one** 32-bit instruction/cycle to IDU (`ifu_idu_id_inst`/`_vld`,
  ibuf.v:1354-1356 per the extraction note) — this is the concrete
  single-issue decode boundary; C906 fetches up to 2 instructions/cycle into
  the buffer but only ever presents one downstream.
- **ICache**: 32KB, 2-way, 64B line, 256 sets. Tag array = one 256×59 SRAM
  (same 59-bit row layout as C910: `{fifo,valid1,ptag1[27:0],valid0,
  ptag0[27:0]}`, just half the rows); data array = four 2048×32 SRAMs (1
  word/cycle **fetch** bandwidth — the 4 banks exist for refill bandwidth,
  not fetch width, unlike C910's 16B/cycle). No stored predecode array; RVC
  boundaries are computed live (`inst[1:0]==2'b11`), not precomputed at
  refill like C910. `icache_tag_wen` is 3 bits: `[2]` = FIFO-replacement
  write-enable, `[1:0]` = the two ways. FIFO-bit replacement (same scheme as
  C910, half the sets). Critical-instruction-first refill (manual §2.2.1).
  The fence.i invalidate-all FSM (plain walk over 256 sets, half of C910's
  512-set walk).
- **RVC handling**: live boundary detection (not precomputed bry0/bry1
  vectors like C910); IPACK's straddle-carry entry handles an instruction
  split across the 32-bit ICache read.
- **ibuf**: 6-entry circular halfword queue with the push≤3/pop≤2 discipline
  above — much shallower than C910's 32-entry ibuf, consistent with a
  narrower, single-issue machine.
- **BPU**:
  - **BHT**: single 1024×16 SRAM (`BHT_16K`, exactly matching the manual's
    Table 1.1 "16Kb" — 1024×16=16384 bits), pure global-history index (no PC
    mixed in, despite "Gshared" in the manual — confirmed dead PC ports in
    `aq_ifu_bht.v`), 8 lanes of 2-bit saturating counters per row selected by
    3 GHR bits (so the 16Kb table = 8192 two-bit counters). Same-row
    second-lookup trick lets one single-ported SRAM answer two branches in a
    2-wide fetch bundle without a second read.
  - **BTB**: 16 flop-based fully-associative entries (tag[16]+target[16]+
    valid each), round-robin FIFO replacement on miss, in-place replace on
    hit. Tag/target cover only PC[15:0] — a real 64KiB aliasing period,
    clone as-is (see §2.3.3: a genuine CAM wrong-hit, not a plain miss, but
    harmless because every hit is unconditionally re-validated at ID stage
    before it can affect which instructions commit).
  - **RAS**: 4 flop entries, each just a 24-bit PC (no valid/priv bits), one
    content array shared by a speculative-pop pointer and a confirmed-BJU
    pointer; misprediction recovery resyncs the pointer only, not entry
    content. **Flagged limitation, clone as-is**: this implies correctness
    only for ≤4 in-flight unresolved call/return predictions — deeper nests
    are expected to give wrong (but deterministically wrong, matching the
    original) predictions past that depth. The directed test suite (§4.2)
    must test AT and JUST BELOW this depth, not deliberately past it the way
    rv12 tested past C910's 12-entry RAS.
  - **Arbitration** (`aq_ifu_pred.v`): BHT supplies direction only; BTB
    supplies target only and redirects early at PCGEN, with an ID-stage-side
    check validating/overriding via immediate-computed target comparison
    (`btb_mis_pred`); RAS is a fully separate path bypassing BTB entirely,
    with its own output channel distinct from BHT/BTB's redirect channel.
- **Stall/flush topology** (extraction note §6, and the design's alignment
  contract, main design doc §5.2/§6.2 rule 5): `idu_ifu_id_stall` is the
  **only** signal crossing the IFU↔IDU boundary — consumed at exactly one
  point in the control hub (gating IBUF's pop) plus directly inside IBUF
  itself. The hub's cancel fan-out reaches only 3 destinations (ICache /
  IPACK / BTB); IBUF's and IPACK's own flush inputs are wired directly from
  RTU/IU, bypassing the hub. PCGEN's fetch-enable backpressure comes directly
  from the ICache's own grant signal, also bypassing the hub. **rv906 clones
  this exact point-to-point topology** — do not introduce a shared/broadcast
  stall bus (see main design doc §2.1's explicit prohibition).
- **Boot/exception vector sequencer** (`aq_ifu_vec.v`): confirmed boot-only
  (4-state FSM: RESET/WARM_UP/HALT/IDLE), not RVV — same pattern as C910's
  "vector" module. Reduced to RESET→RUN + reset-vector pcload for M1, same
  simplification rv12 made for C910's equivalent module.
- **MMU stub**: bare physical mapping, zero latency — replaced by the real
  MMU in M4. Address widths matched to the real interface so the M4 swap is
  port-compatible (exact widths pinned during implementation from
  `mmu/rtl/` port declarations, not guessed here).
- **Chicken bits**: whatever C906's `cp0_ifu_*`-equivalent enable/invalidate
  set turns out to be (confirmed during implementation, not assumed to
  mirror C910's set 1:1) — driven from the M1 harness; the bring-up ladder
  (§4.3) is part of the acceptance.

### 2.2 Deferred (recorded, not silently dropped)

| Item | Why deferred | When |
|---|---|---|
| Non-blocking D$/miss-buffer, HW stream prefetch | LSU-side (M3), not IFU | M3 |
| Any way-prediction beyond what's structurally in the tag array | not confirmed present in the IFU extraction (open item: `cp0_ifu_iwpe`-equivalent semantics unresolved) — investigate during implementation, defer the feature if it turns out to be a real predictor rather than a low-power bypass | TBD, resolved by Task 1 of the M1 plan |
| Snoop/coherence invalidate paths | single-core, not applicable per the parent design's scope (§2.2) | never |
| RVV plumbing | non-standard 0.7.1-draft, excluded per parent design | never |
| HAD-equivalent / non-standard debug hooks | parent design clones RISC-V standard Debug instead, and that's a separate milestone (M7) | M7 |

### 2.3 Documented deviations from C906

1. **Bus width**: C906's real AXI4.0 master is 128-bit; rv906's M0 SoC bus
   (inherited from rv12/rocketM) is 512-bit single-beat. Follow rv12's own
   precedent exactly: one 512-bit single-beat read per cache line, sliced
   internally; critical-word-first is lost on the bus only — the internal
   delivery order to the fetch pipeline is reconstructed to match what a
   real burst would have given it (rv12's `ICache.v:362-374,:412-427`
   pattern). Refine at implementation time and document the actual code
   sites, per rv12's own process (its spec initially wrote this as a
   simplification, then recorded the refined, more faithful resolution once
   implemented — do the same here rather than leaving the simplification as
   final).
2. **BHT read/write GHR-window question**: the BPU extraction note flags an
   unresolved item — a possible read/write bit-shift discrepancy in the GHR
   window, needing pipeline-depth context from `pcgen.v` (out of the BPU
   researcher's scope). This must be resolved during implementation by
   reading the cited spans together, exactly as rv12 resolved an analogous
   C910 BHT hash question (design doc's own precedent: "read together with
   the output pipe register... the two windows are the SAME hash written at
   two points of a two-stage prefetch" — check whether the same resolution
   applies here, or whether C906's simpler pure-GHR indexing makes this a
   non-issue). Record whichever resolution is found in code and in
   `docs/03-bpu.md`.
3. **BTB PC[15:0] aliasing**: real 64KiB aliasing period, clone as-is (it is
   the shipped behavior). **RESOLVED during Task 8 implementation** by
   reading `aq_ifu_btb_entry.v`/`aq_ifu_btb.v`/`aq_ifu_pred.v` directly
   (not the extraction note's restatement): this is a **genuine wrong-hit**,
   not a plain "tag mismatch → miss, costs capacity only" — but it is
   architecturally harmless by design, for a specific, confirmed reason
   (below), not merely by luck.

   The entry's tag compare (`aq_ifu_btb_entry.v:147-150`) is a bare 16-bit
   equality, `btb_tag[15:0] == btb_acc_tag[15:0] && btb_vld`, with **zero
   disambiguation** against the address's upper 24 bits anywhere in either
   file (no stored tag bits beyond PC[15:0], no secondary target-vs-address
   cross-check). So two real addresses sharing PC[15:0] but differing above
   bit 15 make the CAM report a confident **hit** (`btb_rd_hit_vld=1`), not
   a miss, the instant one of them has a live entry — and this hit fires
   purely from the address match: `aq_ifu_btb.v:642-645`'s `btb_flop_vld`
   (the read-side redirect trigger) requires only `ctrl_btb_inst_fetch &&
   icache_btb_grant && btb_rd_hit_vld && !pred_btb_mis_pred`, with no
   requirement that the fetched bytes at that address even decode to a
   branch. The reconstructed target splices the tag-matching entry's
   stored low-16-bit target onto the **current (aliasing) fetch address's
   own** upper bits (`aq_ifu_btb.v:740`: `{pcgen_btb_ifpc[39:16],
   btb_hit_tgt[15:0]}`) — so at the moment it happens, an aliased hit is
   structurally indistinguishable from a genuine intended one.

   What makes this harmless: `aq_ifu_pred.v`'s `btb_mis_pred`
   (`aq_ifu_pred.v:720-723`) unconditionally re-validates **every** BTB hit
   against the immediate-decoded ground truth of whatever instruction
   actually reached ID stage (`pred_br_tar = pred_cur_pc + pred_br_imm`,
   `:596-598`), firing whenever the predicted target disagrees **or** the
   actual outcome isn't even taken. An aliased entry's stored target
   belongs to a different instruction than whatever real bytes sit at the
   aliasing PC, so (barring the vanishing coincidence that the two targets
   happen to agree) this check fires and drives the exact same
   `pred_chgflw`/`pred_pcgen_chgflw_vld` correction path
   (`aq_ifu_pred.v:725-726`) as any ordinary wrong BTB entry. The front end
   makes **no distinction whatsoever** between "an ordinary stale/wrong
   hit" and "an alias-caused wrong hit" — both are just a target/taken
   mismatch, corrected identically, at the identical cost of one wasted
   speculative redirect cycle, always before anything commits.

   So: the alias is a real, confirmed wrong-hit at the CAM level (not a
   miss), but C906's front end treats *every* BTB hit, aliased or not, as a
   merely provisional prediction that is always re-validated before it can
   affect which instructions commit — making this a normal, expected,
   self-correcting case by construction, not a latent correctness bug.
   rv906 clones this behavior exactly (`rtl/BPU.v`'s SECTION BTB:
   `btb_mis_pred`, `btb_clr_one`); `thrash.S` (built to span the BTB's
   64KiB alias period) and `ind_jr.S` pass the full checker-validated
   regression at rung 3 with this exact CAM/validation shape in place,
   consistent with the finding above.
4. **RAS depth limitation** (§2.1): clone the 4-entry, pointer-only-resync
   behavior as-is; this is a real, shipped limitation, not a simplification
   rv906 is introducing.
5. No clock gating cells, behavioral SRAM, flat AXI — per parent spec §6.3.

## 3. File organization (restricted-SV recipe, parent §6)

```
rtl/SRAM.v        behavioral single-port SRAM (A/CEN/WEN bitwise/D/Q, 1-cycle
                  read, Q held) — parameterized WIDTH/DEPTH; the one SRAM
                  model for the whole project from here on
rtl/IFU.v         PCGEN, ICache-access control, IPACK, IBUF, boot FSM,
                  cancel network, IFU<->IDU single-instruction handoff
rtl/ICache.v      tag/data arrays (via SRAM.v), refill FSM, invalidate FSM,
                  AXI master (critical-instruction-first refill)
rtl/BPU.v         BHT + BTB + RAS + the arbitration logic that combines them
rtl/RVProc.v      core shell v0.1: IFU + ICache + BPU + MMU stub + FetchSink,
                  drop-in replacement for TestMaster in RVProcAXI.v
rtl/FetchSink.v   M1 SCAFFOLDING (absorbed into IDU/IU/RTU from M2): consumes
                  the single-instruction interface, plays fake BJU + fake
                  RTU (scripted branch outcomes, retire pulses), exposes the
                  committed stream to verisim, reports via tohost on the
                  D-side AXI port (TestMaster's proven write channel, lifted
                  verbatim from M0)
```

`TestMaster.v` is deleted; `RVProcAXI.v`'s core instance flips from
`TestMaster` to `RVProc`. Per-file style: stage-sectioned, feedback section
up top, packed structs internal only, flat wire ports, entry arrays not
per-entry modules (per main design doc §6.2).

## 4. Verification design

### 4.1 Architecture: fake BJU in RTL, golden checker in C++

Simpler than rv12's M1 oracle in one important way: C906 delivers **exactly
one instruction per cycle** to the consumer (not a 3-slot bundle), so there
is no multi-slot partial-commit-within-a-bundle logic to model — a
mispredict discards the delivered instruction and everything already
buffered in IBUF ahead of it, full stop.

- **FetchSink (RTL)** resolves what M1 lacks an execute stage for. Its
  contract must be deterministic and PREDICTOR-INDEPENDENT for every control
  transfer:
  - **conditional branches**: direction = a fixed rule on the branch PC
    (e.g. `taken = ^pc[7:4]`, same style as rv12's); taken target = decoded
    B-immediate.
  - **direct jumps** (jal/c.j): target = decoded J/CJ-immediate.
  - **call/return**: FetchSink keeps a SHADOW CALL STACK (its own,
    unbounded-depth, e.g. 16 entries — deeper than the 4-entry RAS on
    purpose but not absurdly so, since C906's RAS is shallow enough that a
    16-deep shadow stack already exercises well past the real limit): on a
    committed call, push the fall-through PC; on a committed return, actual
    target = shadow-stack pop. The RAS is checked against ground truth,
    including at and just past its 4-entry limit (§2.1's flagged
    limitation) — this is where rv906's test plan intentionally diverges
    from rv12's "test past overflow" approach, since C906's RAS doesn't
    resync content on misprediction the way a deeper/more robust design
    might; the directed test at depth 4-6 is expected to show the *shipped*
    (possibly wrong-looking but faithfully cloned) behavior, and the ISS
    must implement the SAME limitation, not idealized RAS behavior.
  - **other indirect jumps** (`jr` through non-link registers): actual
    target = a fixed formula of the jump's PC (`JR_TARGET(pc)`, exact form
    chosen in the plan; test generator places landing pads there). Both
    FetchSink and the C++ ISS implement the same formula.
  - **resolve/mispredict protocol**: mirrors whatever signal names C906's
    IU-side branch-resolve interface actually uses (confirmed during
    implementation from `iu/rtl/aq_iu_bju.v` and the IFU's consumer-side
    ports — not assumed to match C910's `iu_ifu_bht_check_vld`/
    `iu_ifu_chgflw_*` names). Resolve is REGISTERED, never combinational
    from delivered data. On a mispredict: commit = the delivered instruction
    up to and including the resolving branch; discard the wrong-path window
    (everything buffered in IBUF from the resolve cycle until the redirect
    takes effect).
  - issues RTU-side retire pulses (GHR shift, RAS/BTB retire updates) so the
    architectural repair paths are exercised;
  - terminates after N committed instructions or on a `jal x0, 0` sentinel;
    writes tohost (1 = reached sentinel, 3 = internal protocol check
    failed) — same convention as M0's TestMaster and rv12's FetchSink.
- **C++ golden fetch-ISS** (in `RVProcTest.cpp`): replays the same contract
  (direction rule + immediates + its own shadow call stack + the same
  `JR_TARGET` formula, including the RAS depth-limitation behavior) over the
  ELF image loaded in ExtMem, producing the expected committed `(pc,
  opcode)` sequence. Every cycle the testbench samples the committed
  instruction via `verisim.h` paths and compares online; first divergence
  aborts with both streams printed. The checker is predictor-agnostic:
  predictors change *when* an instruction is fetched, never *which*
  instructions commit — implement FetchSink and the ISS independently from
  this spec text (a shared bug between them validates nothing), matching
  rv12's own explicit rule.

### 4.2 Directed test suite (`test/m1/*.S`, xpack, targeting C906's
RV64IMAFDC baseline scalar subset exercised by the front end — no F/D/A
content needed since this is a fetch-only test, but the build target should
already reflect the real ISA, i.e. `-march=rv64imac_zicsr` at minimum, not
rv12's `rv64imc`)

seq (straight-line), rvc-mix (alignment phases + straddle across the 32-bit
ICache-read boundary and the 64B line boundary), jal/c.j chains, call/ret
nests **at and just past 4 deep** (the RAS's real limit — see §4.1, not past
it the way rv12 deliberately overflowed C910's 12-entry RAS), indirect-jump
tables (`jr` through a non-link register), dense conditional branches
(forward/backward, back-to-back, more than one per fetch group where the
2-wide fetch bundle allows it — exercising the BHT's same-row second-lookup
trick, §2.1), icache thrash (>32KB code footprint through the BTB's 64KiB
alias period and the ICache's own sets), uncached-region fetch, fence.i
after host-patched code, mixed stress (all of the above interleaved).

### 4.3 Acceptance = the chicken-bit ladder

The full suite must pass at EVERY rung (same golden stream at each). Exact
rung count/order depends on how many independently-disableable predictor
stages C906 actually exposes (confirmed during implementation — do not
assume rv12's 6-rung C910 ladder transfers unchanged, since C906 has fewer
predictor structures: no L0 BTB, no indirect BTB). A minimum plausible
ladder given what's been extracted: 1. all predictors off → 2. +RAS → 3.
+BTB → 4. +BHT. Rungs are selected by a harness CLI flag (`--m1-rung=<N>`,
same mechanism as rv12: parsed in `RVProcTest.cpp`, poked into FetchSink's
config register bank — NOT Verilator plusargs, per rv12's own corrected
precedent). Additionally: invalidate sweeps (BHT/BTB) run mid-test at the
top rung without corrupting the stream.

M1 is done when: suite green at all rungs, `make verisim` full-stack lint
stays clean, and `docs/02-ifu.md` + `docs/03-bpu.md` (tutorial chapters:
principle → implementation → C906 file cross-reference → design discussion)
are written. The M0 smoke test is RETIRED with TestMaster (same rationale as
rv12: its D-side write/tohost path is a strict subset of what every M1 test
exercises through FetchSink's tohost reporting).

## 5. Interfaces frozen for M2

- IFU→IDU: single 32-bit instruction + valid, `idu_ifu_id_stall` the only
  signal flowing the other way across this boundary (§2.1). Bit-exact to
  whatever C906's real `ifu_idu_id_inst`/`_vld`/`idu_ifu_id_stall` port
  widths and any accompanying metadata (predecode flags, exception flags)
  turn out to be once `aq_ifu_top.v`'s IDU-facing port list is fully
  transcribed during implementation.
- Branch-resolve interface to IU/RTU: whatever C906's real signal set is
  (confirmed during implementation from `aq_iu_bju.v` and the IFU's
  consumer-side ports) — FetchSink implements the consumer side; M2's real
  IU/RTU take it over unchanged.
- MMU stub port list = the real ITLB interface (M4 swaps implementation
  only) — exact widths pinned during implementation, not guessed here.

## 6. Risks / open points

- **BHT GHR-window question** (§2.3.2): must be resolved by the plan's BHT
  task before the BHT is trusted; a wrong resolution degrades prediction
  silently (caught only by cycle-count comparison at M8, not by correctness
  tests, since FetchSink's oracle is predictor-agnostic by design).
- **`cp0_ifu_iwpe`-equivalent semantics** (§2.2): the IFU extraction note
  flags this as possibly a low-power tag-hit-buffer bypass rather than
  classic way-prediction — the M1 plan's ICache task must resolve this
  before deciding whether it's in scope at all.
- **RAS depth-limitation fidelity** (§2.1, §4.1, §4.2): the biggest
  divergence from rv12's verification approach. Getting the ISS's shadow
  stack to reproduce the REAL RAS's shipped behavior at depth ≥4 (not an
  idealized unbounded RAS) requires reading `aq_ifu_ras.v`'s pointer-resync
  logic carefully during implementation — treat this as a first-class task
  with its own focused unit test, not something to wing inside the general
  ISS.
- **BTB CAM vs SRAM verification cost**: a flop-based fully-associative CAM
  with round-robin replacement is a different structure to get bit-exact
  than an SRAM-backed set-associative table — the M1 plan's BTB task should
  budget for reading `aq_ifu_btb.v`'s replacement-pointer logic closely.
- Everything the IFU extraction note itself flagged as unresolved
  (`icache_data_idx` bit-slice reconciliation, full refill beat-by-beat
  tag-write sequencing) must be resolved by the plan's ICache task by
  reading the cited spans directly, not by assumption.
