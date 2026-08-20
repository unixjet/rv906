# 3. The Branch Predictor: BHT, BTB and RAS

This chapter describes `rtl/BPU.v`, C906's branch-prediction stack as built
in M1. Where `docs/02-ifu.md` covers the fetch pipeline and the instruction
cache, this chapter covers the three predictor structures that hang off
that pipeline — BHT, BTB, RAS — and the arbitration logic that combines
them, all of which live inside `aq_ifu_pred.v`'s scope in the real chip and
inside `BPU.v` here.

**How to read it.** Section 1 is the principle: why C906's predictor set is
so much smaller and shallower than C910's, and what that trades away.
Section 2 covers each structure's real storage shape (SRAM vs. CAM vs.
flops) and the arbitration that combines their outputs into the two redirect
channels PCGEN consumes. Section 3 is the C906 file cross-reference table.
Section 4 is the design discussion — the four genuine findings this
milestone made by reading the RTL directly rather than trusting the
extraction notes' restatement, plus the full deferral list.

**Normative documents.** The contract is
`docs/superpowers/specs/2026-08-20-m1-ifu-design.md`, specifically §2.1 (RAS
depth limitation), §2.3.2 (BHT GHR-window finding) and §2.3.3 (BTB PC[15:0]
aliasing finding) — both §2.3.2 and §2.3.3 were corrected **in place** during
Tasks 8 and 9 with the as-implemented findings, following the same
precedent rv12 set for its own C910 spec. The C906 facts rest on
`docs/superpowers/specs/notes/2026-08-20-c906-bpu-extraction.md`. Donor
citations below are relative to
`refs/openc906/C906_RTL_FACTORY/gen_rtl/ifu/rtl/`.

---

## 1. Principle: three small, independent structures

### 1.1 Why C906's predictor is shallower than C910's

C910 is a wide, out-of-order, speculative machine; the deeper it can look
ahead the more work it can overlap, so it pays for five predictor
structures at increasing depth and cost (L0 BTB, BTB+BHT, RAS, indirect BTB,
plus an exact backstop). C906 is single-issue and in-order — a mispredicted
branch stalls the *entire* machine behind it either way, so there is
correspondingly less to buy by predicting further ahead or predicting more
kinds of transfer. The extraction notes' headline finding (BPU notes S0)
makes this concrete: C906's predictor is **three small, independent,
differently-sized structures**, not one unified table family the way
C910's BHT/BTB families are. There is no L0 BTB and no indirect BTB at all
— confirmed absent from the module list, not merely disabled.

The total non-SRAM predictor state makes the size difference vivid: BTB's
16 flop entries (528 b) + RAS's 4 flop entries (96 b) + two 14-bit GHRs
(28 b) = **676 bits**, against C910's ~1504 bits of flop state in just one
of its several structures (BPU notes S7). The one SRAM-backed structure,
BHT, is a single 1024×16 array — "16Kb" in the manual is the literal raw
bit count of that one table, not a family aggregate (BPU notes S6).

### 1.2 What each structure is actually built from

| Structure | Storage | Why that shape |
|---|---|---|
| BHT | 1 SRAM (`aq_spsram_1024x16`) | direction is looked up on every branch every cycle; an SRAM amortizes that cost across 8192 counters |
| BTB | 16 flops, fully-associative CAM | small enough that a parallel compare across all 16 entries is cheaper than an indexed SRAM lookup, and a CAM never aliases capacity away the way an indexed table does |
| RAS | 4 flops, one physical array | call/return nesting rarely goes more than a few deep in practice; 4 entries trades rare deep-nest mispredictions for a tiny, fast structure |

BTB and RAS are **pure flop arrays with zero SRAM** — confirmed by grepping
both files for `aq_spsram_*` instantiations and finding none (BPU notes
S2.1/S3.1). `rtl/BPU.v` clones this exactly: no set-associative SRAM BTB or
copy-back-repair RAS the way C910's clone builds, because C906's real
structures are not shaped that way.

### 1.3 The two-channel arbitration

C906's `aq_ifu_pred.v` produces **two separate redirect channels**, not one
(BPU notes S4.3):

- **`chgflw`** ("change-flow"): the BHT/BTB combination. BTB supplies a
  target only; BHT supplies direction only; the two combine at ID-stage
  time via `pred_chgflw = btb_pred_tar_vld ? (btb_mis_pred && ...) :
  pred_br_taken` (`aq_ifu_pred.v:725-726`) — if BTB had a valid prediction
  and it agreed with the immediate-decoded ground truth, nothing new
  redirects; if it disagreed or had nothing, ID-stage's own BHT-direction +
  computed-target drives the correction directly.
- **`curflw`** ("current-flow"): RAS returns, plus the same-bundle
  delay-replay mechanism (§2.2 below). **RAS bypasses BTB entirely** — the
  two structures never compare notes on the same instruction (BPU notes
  S4.1).

`BPU.v` keeps this exact split: `pred_pcgen_chgflw_vld/_pc` and
`pred_pcgen_curflw_vld/_pc` are two distinct output ports, PCGEN priority
levels 3 and 5 respectively (`docs/02-ifu.md` §1.3).

---

## 2. Implementation (`rtl/BPU.v`, 1182 lines)

Section map, in the order the real RTL's own structures were brought up
(Tasks 7-9, RAS→BTB→BHT — deliberately not BHT-first the way a "biggest
structure first" instinct might suggest, because RAS is bypassed by
everything else and could be verified in isolation soonest):

| Region | Lines |
|---|---|
| header, C906 file map, seam notes | `BPU.v:1-183` |
| SECTION: RAS (+ pcall/preturn classification) | `BPU.v:184-571` |
| SECTION: BTB | `BPU.v:572-814` |
| SECTION: BHT | `BPU.v:815-1182` |

`aq_ifu_pre_decd.v` (branch/jump/link/return classification + immediate
decode) is **not** instantiated in `aq_ifu_top.v` at all — it lives inside
`aq_ifu_pred.v` (IFU pipeline notes S1), i.e. structurally inside this
module's scope, not the pipeline's. `BPU.v` therefore owns *all*
classification and immediate-decode logic; `IFU.v` forwards only the raw
fetched-bundle view (`ipack_pred_inst0/1`) and the current ID-stage PC.

### 2.1 RAS (`BPU.v:184-571`)

Four flop entries, each just a 24-bit PC — no valid bit, no privilege field
(`aq_ifu_ras_entry.v:36`, confirmed). **One** physical content array shared
by two independent one-hot 4-bit pointers:

- `ras_pop` (speculative): rotates on every predicted call/return, and
  **snaps back to `ras_bju`** — the pointer only, never entry content — on
  a flush or a BJU misprediction.
- `ras_bju` (confirmed): rotates only on IU-resolved `iu_ifu_link_vld`/
  `iu_ifu_ret_vld` events.

This pointer-only resync (no copy-back repair the way C910's 12+6-entry RAS
does) is the real, shipped RAS depth-limitation contract: correctness is
only guaranteed for ≤4 in-flight unresolved call/return predictions. See
§4.3 for the full finding and how the test suite exercises it.

**Classification** (pcall/preturn/ind_br) lives here, not in the pipeline,
per §2 above — and it is a genuine, confirmed divergence from C910's
register convention, discussed in §4.4.

### 2.2 BTB (`BPU.v:572-814`)

16 flop entries, each `{valid, tag[15:0], target[15:0]}` — a real parallel
CAM (`aq_ifu_btb_entry.v` instantiated ×16, each with its own tag compare),
not an indexed SRAM lookup. Tag/target cover only PC[15:0]: a genuine
64 KiB aliasing period, cloned as-is (§4.2). Round-robin FIFO allocation on
a miss; in-place replace on a tag hit; no confidence/counter field at all —
direction always comes from BHT, never from the BTB entry itself.

`BPU.v` collapses the real 2-stage CAM pipeline (PCGEN-time speculative
compare → ID-stage validation) into a single ID-stage-time lookup, a
deliberate, documented rv906 simplification discussed at length in
`docs/02-ifu.md` §5.4 — the reasoning is an IFU-side pipeline-timing
argument, not a BPU-side one, so it lives in that chapter.

### 2.3 BHT (`BPU.v:815-1182`)

A single `1024×16` SRAM, indexed **purely by global history** — the two
PC-derived ports wired into the real module (`pred_bht_pc`,
`iu_ifu_bht_cur_pc`) are confirmed dead code (grepped with zero hits outside
the port list, BPU notes S1.4), so despite the manual's "Gshared" name, this
is not McFarling's PC-xor-GHR gshare. `BPU.v` does not even carry a signal
shaped like `pred_bht_pc` on its frozen ports, for that reason.

**Two 14-bit GHRs**, not C910's spec/arch/checkpoint-FIFO triple:
`bht_ghr` (architectural, shifts in the *actual* resolved outcome on every
`iu_ifu_br_vld`) and `bht_vghr` (speculative, normally shifts in the
*predicted* outcome, but reloads from `{bht_ghr[12:0], iu_ifu_bht_taken}` on
a misprediction — resyncing from the architectural register plus the
just-resolved outcome, not from its own wrong history).

**Prediction**: row = `bht_vghr[11:2]`, lane = `bht_vghr[2:0]` (bit 2 is
shared between row and lane, a deliberate 1-bit fold) — a contiguous 12-bit
read window. Each row holds 8 lanes of 2-bit saturating counters, split
across two SRAM byte-planes (`A`=taken plane, `B`=not-taken plane); the
update case table is a textbook increment-on-taken/decrement-on-not-taken
counter with saturation at 0/3 (`BPU.v:915-933`).

**The same-row second-lookup trick** (BPU notes S1.6): the array is
single-ported but `aq_ifu_pred.v` needs a prediction for a *second* branch
in the same 2-wide bundle without a second SRAM read. `BPU.v` reuses the
already-fetched row and substitutes the first branch's own not-yet-resolved
predicted outcome into the lane-select bit, exactly matching
`aq_ifu_bht.v:310-314`.

**Update**: row = `bht_ref_vghr[13:4]`, lane = `bht_ref_vghr[2:0]` —
deliberately captured from `bht_ghr` (architectural) at resolve time, one
cycle in a 3-state refill FSM (IDLE→READ1→READ2→WRTE for a mispredict
recovery, or IDLE→UPD for an ordinary resolve). Why this write window's
bit-slice differs from the read window's is a genuine, as-shipped finding —
§4.1 below.

**Invalidate**: a 3-state sweep (`BHT_INV_IDLE/WRTE/READ`) over all 1024
rows, 1024 cycles — sized to the active `BHT_16K` config, the same
sweep-invalidate style BTB uses for its instant, all-16-entries clear
(§2.2), just proportional to a much larger array.

---

## 3. C906 file cross-reference table

| rv906 file / section | Real C906 file(s) | What it clones |
|---|---|---|
| `BPU.v` SECTION RAS | `aq_ifu_ras.v` (267 lines), `aq_ifu_ras_entry.v` (112 lines) | 4-entry flop stack, dual one-hot pointer (`ras_pop`/`ras_bju`), pointer-only misprediction resync |
| `BPU.v` SECTION RAS (classification) | `aq_iu_bju.v:637-667`, `aq_idu_cfig.h:402-450` | pcall/preturn/ind_br classification — x1-only, confirmed divergence from C910 (§4.4) |
| `BPU.v` SECTION BTB | `aq_ifu_btb.v` (753 lines), `aq_ifu_btb_entry.v` (163 lines) | 16-entry fully-associative CAM, PC[15:0] tag/target, round-robin FIFO allocation |
| `BPU.v` SECTION BHT | `aq_ifu_bht.v` (549 lines), `aq_ifu_bht_array.v` (123 lines) | 1024×16 SRAM, pure-GHR index, 8-lane 2-bit saturating counters, same-row second-lookup trick, two-GHR update scheme |
| `BPU.v` (arbitration, spread across all three sections) | `aq_ifu_pred.v` (819 lines) | Two-channel (`chgflw`/`curflw`) redirect split, delay/replay for a not-taken-then-taken pair in one bundle, chicken-bit gating |
| `rvproc_pkg.sv` BHT/BTB/RAS constants | `cpu_cfig.h:135,344-358`, `aq_ifu_bht_array.v:102`, `aq_ifu_btb.v:154`, `aq_ifu_btb_entry.v:89` | Geometry confirmed against the SRAM primitive/entry port widths directly |

---

## 4. Design discussion

### 4.1 The BHT read/write GHR-window finding (Task 9, spec §2.3.2)

The extraction notes flagged an open question: the prediction-read index
(`bht_vghr[11:2]`) and the counter-update index (`bht_ref_vghr[13:4]`) are
different bit-slices of what looked like it might be "the same hash at two
pipeline stages" — the same *shape* of anomaly rv12 found and resolved for
C910's analogous BHT, where the two windows turned out to be one hash
viewed one pipeline stage apart. The plan explicitly warned against
assuming that resolution transfers, since C906's indexing is simpler
(pure GHR, no PC-hash) — Task 9 re-derived the answer from `aq_ifu_bht.v`
directly rather than reusing C910's verdict.

**The verdict is different this time: this is a real, as-shipped
discrepancy, not a pipeline-depth artifact.** The full derivation lives at
`BPU.v:935-984`; in outline:

- `bht_ref_vghr` is captured, on a non-blocking read, from the
  *architectural* `bht_ghr` at the exact moment a branch resolves — i.e.
  from *before* this branch's own outcome shifts into it on that same edge.
- Because this front end resolves branches strictly in program order,
  single-issue, every branch strictly older than the one resolving has
  *already both predicted and resolved* by the time it does. So on the
  happy path (no older branch mispredicted), `bht_ghr` at a branch's own
  resolve holds the **identical** 14-bit value that `bht_vghr` held when
  that same branch was predicted.
- That equality holds for *any* constant number of pipeline stages between
  prediction and resolve — the stage count cancels out of the argument
  entirely. So a fixed pipeline latency cannot be the source of a
  systematic 2-bit-shifted, bit-3-skipping re-slice of one identical
  register: if this were "the same hash at two pipeline stages," read and
  write would slice the *same* bit positions of that value, and they
  provably do not (read: bits `[11:0]`; write: bits `{0,1,2}∪{4..13}`,
  skipping bit 3 and reaching two bits further back than read ever
  touches).
- `aq_ifu_pcgen.v` was read directly, as the plan required, to check for a
  reconciling mechanism there — grepped case-insensitively for
  `"bht"`/`"ghr"`, zero matches anywhere in the file. There is no
  PCGEN-side pipeline-depth term resolving this the way one existed for
  C910's analogous question.

**Why it is harmless**, by the same structural argument as the BTB
aliasing finding (§4.2): C906's front end treats every predictor output as
provisional, always re-validated before it can affect which instructions
commit (design doc S4.1, "predictors change WHEN, never WHICH"). The
discrepancy can only misdirect a branch's own counter update to a different
physical row/lane than the one that predicted it — extra destructive
aliasing on top of what pure-GHR indexing (zero PC disambiguation) already
accepts by design. It degrades prediction accuracy/cycle count only, never
the committed instruction stream. `rtl/BPU.v`'s SECTION BHT part (e) carries
this same finding at the code site; rv906 clones the discrepancy exactly as
shipped rather than "fixing" the two windows to agree.

The full regression matrix's own cycle-count data is consistent with this:
`dense_br` at rung 4 (BHT on) takes *more* cycles than at rung 1 (2837 vs.
2801, `--sink-stall` off) while every rung commits the identical 809
instructions — exactly the "accuracy/cycle-count only, never correctness"
signature the finding predicts, measured rather than merely argued.

### 4.2 The BTB PC[15:0] aliasing finding (Task 8, spec §2.3.3)

The design doc's open question: does a tag mismatch on an 8-alias address
(two real addresses sharing PC[15:0] but differing above bit 15) show up as
a plain miss (costing capacity only), or can the CAM produce a genuine
wrong-hit? Task 8 read `aq_ifu_btb_entry.v`/`aq_ifu_btb.v`/`aq_ifu_pred.v`
directly to settle it.

**Verdict: a genuine wrong-hit, not a plain miss** — but architecturally
harmless by a specific, confirmed reason, not merely by luck. The entry's
tag compare (`aq_ifu_btb_entry.v:147-150`) is a bare 16-bit equality with
**zero disambiguation** against the address's upper 24 bits anywhere in
either file — no stored tag bits beyond PC[15:0], no secondary
target-vs-address cross-check. So two real addresses aliasing on PC[15:0]
make the CAM report a confident hit the instant one of them has a live
entry, purely from the address match, with no requirement that the fetched
bytes even decode to a branch. The reconstructed target splices the
tag-matching entry's stored low-16-bit target onto the *current (aliasing)*
fetch address's own upper bits — at the moment it happens, an aliased hit
is structurally indistinguishable from a genuine intended one.

**What makes it harmless**: `btb_mis_pred` (`BPU.v:686-706`,
`aq_ifu_pred.v:720-723`) unconditionally re-validates *every* BTB hit
against the immediate-decoded ground truth of whatever instruction actually
reached ID stage. An aliased entry's stored target belongs to a different
instruction than whatever real bytes sit at the aliasing PC, so — barring
the vanishing coincidence that the two targets happen to agree — this check
fires and drives the exact same correction path as any ordinary wrong BTB
entry (`btb_clr_one`, `BPU.v:758`). The front end makes **no distinction
whatsoever** between "an ordinary stale/wrong hit" and "an alias-caused
wrong hit" — both are just a target/taken mismatch, corrected identically,
at the cost of one wasted speculative redirect cycle, always before
anything commits.

`test/m1/thrash.S` is built specifically to span the BTB's 64 KiB alias
period (96 KB total footprint, 1.5× the alias period) and passes the full
checker-validated regression at every rung with this exact CAM/validation
shape in place, consistent with the finding.

### 4.3 The RAS depth-limitation behavior (Task 7, spec §2.1/§4.1)

The real `aq_ifu_ras.v` is 4 flop entries sharing **one** physical content
array between a speculative pointer and a confirmed pointer; misprediction
recovery resyncs the *pointer* only (`ras_pop <= ras_bju`), never entry
*content* — a materially different, shallower and simpler structure than
C910's 12+6-entry copy-back-repair RAS. This implies correctness only for
≤4 in-flight unresolved call/return predictions; deeper nests are expected
to give wrong (but deterministically wrong, matching the original)
predictions past that depth.

This is why the M1 plan **deliberately diverges from rv12's own C910 test
strategy**: rv12 tested C910's RAS *past* overflow (25/31-deep nests against
a 12-entry stack) specifically to prove the copy-back repair still worked at
depth. Testing C906's RAS the same way would prove nothing useful — past
depth 4, the real hardware is *expected* to mispredict, and there is no
repair mechanism whose correctness to demonstrate. `test/m1/callret.S`
instead nests **at and just past** 4 deep (design doc §4.1/§4.2), and the
depth-limitation behavior itself has to be modeled in both oracles, not
idealized away:

- **`FetchSink.v`'s RAS-faithful grading model** (`FetchSink.v:424-475`) is
  a *second*, diagnostic-only shadow structure — separate from the
  general-purpose 16-entry correctness oracle used for FetchSink's actual
  commit/squash decisions — that mirrors `aq_ifu_ras.v`'s algorithm exactly:
  one one-hot 4-bit pointer, one physical 4-entry content array, no content
  resync, no empty-stack special case. It exists to *confirm* the real
  4-entry/pointer-only-resync limitation is actually being exercised the
  way `callret.S`'s past-depth-4 nesting intends, not to re-decide
  correctness — the online checker's committed-stream comparison is already
  invariant to RAS prediction accuracy by design, since FetchSink's own
  resolve (§4.4) always corrects a RAS misprediction before commit.
- A documented limitation of this cross-check, carried at the code site and
  repeated in `docs/08-verification.md`: it is built on FetchSink's own
  *committed* push/pop events, so it structurally **cannot see genuine
  speculative/wrong-path RAS activity** — `BPU.v`'s own trace is the only
  ground truth for that. This is a best-effort cross-check, not a
  substitute for reading the real RAS's trace directly.

RAS has **no invalidate signal at all** in the real RTL (grepped, zero hits
for `ras_inv`/`ras_clr` across both RAS files) — only full reset and a
flush/mispredict resync affect its state. `rtl/BPU.v` clones this absence;
there is no RAS-specific pulse in `--inv-test`'s repertoire, matching the
real chip.

### 4.4 A genuine C906/C910 divergence: x1-only link/return classification

While implementing FetchSink's resolve logic (Task 4.1), reading
`aq_iu_bju.v:637-667` directly (cross-checked against the `FUNC_*` encoding
table in `aq_idu_cfig.h:402-450`) surfaced a real, confirmed divergence from
C910's register-set convention that the plan's own caution ("do not assume
C910's exact register-set convention transfers unchanged") anticipated:

```
bju_link_vld_raw (~pcall)   = dst_preg==x1 && uncond_sel && func[11]
bju_ret_vld_raw  (~preturn) = src0_reg==x1 && inst_jalr &&
                               !(src0_reg==dst_preg && func[11])
```

**C906 checks `dst_preg`/`src0_reg` against `x1` exactly** — there is no x5
alternate link register the way C910's convention (and rv12's own C910
clone, `(x[11:7]==5'd1)||(x[11:7]==5'd5)`) allows. `jal x5, ...` or
`jalr x0, x5, 0` do **not** push or pop the shadow call stack on C906; only
x1 does. This shows up structurally in the RAS section (`BPU.v:184-190`):
the pcall/preturn classification is derived from the IU/BJU file, not from
`aq_idu_id_decd.v`'s opcode table (whose only contribution is emitting the
`func[11]` bit BJU later tests).

One consequence follows for free: the `jalr x1, x1, 0` dual case
(`rd==rs1==x1`) is explicitly excluded from preturn by BJU's own
`!(src0_reg==dst_preg && func[11])` term — real hardware treats it as a
plain call (RAS push only), never a push-then-pop. Because C906 has only
one link register (not C910's x1/x5 pair), this is the *only* way a single
`jalr` could appear to satisfy both push and pop conditions at once, and the
real RTL's own exclusion term already prevents it. `FetchSink.v`'s shadow
call stack (§ SECTION SHADOW CALL STACK) therefore never needs the
same-cycle "push-and-pop" case rv12's C910 clone had to build for
`c.jalr x5` — `do_push`/`do_pop` are mutually exclusive by construction here,
not by a runtime check.

### 4.5 The full deferral list

| Item | Why it is safe to defer | When |
|---|---|---|
| Real production power-gating / clock-gating (`cp0_ifu_icg_en` and analogues) | The umbrella spec forbids clock-gating cells entirely (S6.3); every enable folds into the flop's own update condition instead, which is unconditionally correct, not a staged simplification | never (structural project rule, not a milestone item) |
| Priv-mode gating on RAS/BTB | No privilege levels below M-mode exist yet in M1 | M2+ |
| RAS invalidate | Confirmed absent from the real RTL (§4.3) — nothing to build | never |
| Way-prediction-adjacent low-power bypass consumers on the predictor side | Covered in `docs/02-ifu.md` §3.6/§5.6 (an ICache-side item, not a BPU one) | pre-M8, if ever |
| Performance tuning of the predictor set (accuracy, table sizing) | M8's job per the parent design; M1's bar is correctness only | M8 |

---

## 5. Verification

The BPU's own correctness gate is the same online checker described in
`docs/02-ifu.md` §6 and `docs/08-verification.md`: the committed stream from
`FetchSink.v` is compared against the independently-written golden ISS every
cycle, at every rung of the chicken-bit ladder (1=all off, 2=+RAS, 3=+BTB,
4=+BHT). No BPU-specific unit bench exists separately from the full-stack
suite — RAS/BTB/BHT are exercised through the same directed tests every
other rung uses, which is the entire point of a predictor-agnostic oracle:
one suite gates every predictor configuration without being rewritten for
it.
