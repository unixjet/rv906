# 2. The Front End: IFU and ICache

This chapter describes the instruction-fetch half of milestone M1:
`rtl/IFU.v` (the fetch pipeline), `rtl/ICache.v` (the 32 KB instruction
cache) and `rtl/SRAM.v` (the behavioral memory model both cache and
predictor sit on). The branch predictor that hangs off this pipeline
(`rtl/BPU.v`) has its own chapter, `docs/03-bpu.md`, because C906 keeps BHT,
BTB and RAS almost entirely independent of the pipeline that feeds them —
splitting the two chapters mirrors the RTL's own module boundary.

**How to read it.** Section 1 is the principle: what a front end has to do
and why C906's version ends up shaped the way it does — noticeably flatter
and shallower than C910's, because C906 is a single-issue in-order machine,
not a three-wide out-of-order one. Section 2 is the fetch pipeline. Section 3
is the instruction cache. Section 4 is the C906 file cross-reference table.
Section 5 is the design discussion: the one deviation the SoC bus forces, the
deviations rv906 chose deliberately, the bugs found while bringing this up,
and what stayed deliberately unbuilt. Section 6 points at the verification
chapter.

**Normative documents.** The contract is
`docs/superpowers/specs/2026-08-20-m1-ifu-design.md`; the C906 facts it rests
on were extracted into
`docs/superpowers/specs/notes/2026-08-20-c906-ifu-pipeline-extraction.md` and
`notes/2026-08-20-c906-bpu-extraction.md`. Donor citations below are relative
to `refs/openc906/C906_RTL_FACTORY/gen_rtl/ifu/rtl/`. The verification
harness that proves all of this has its own chapter, `docs/08-verification.md`.

**One convention that differs from rv12/C910.** C906's PC is a **byte**
address throughout the front end — `cp0_xx_mrvbr[39:0]`, `pcgen_ifpc[39:0]`
— not the halfword convention C910 uses. `PC_WIDTH = 40` in `rvproc_pkg.sv`
is therefore a byte width with no analogue to rv12's `VPC_WIDTH`. Every
index quoted below (BHT rows, BTB tags, ICache sets) is quoted in byte-PC
bits for this same reason.

---

## 1. Principle: a flat, single-issue front end

### 1.1 What C906 does not need to do

C906 is a five-stage, single-issue, in-order machine (design doc S1). Its
decoder wants exactly **one** instruction a cycle, in program order. That
one fact removes most of the complexity a wider front end carries:

- No three-instruction bundle to assemble, classify and partially squash —
  IFU notes S2 item 6 confirms the IFU→IDU handoff is a single
  `ifu_idu_id_inst[31:0]`/`_inst_vld` pair, with no second or third slot
  anywhere in the port list.
- No wide predecode array pre-computed at refill time — the cache delivers
  only one 32-bit word (two halfwords) to the pack stage per cycle, so
  finding RVC boundaries live, per halfword, on that one word's own bits
  costs nothing extra and there is no serial-scan hazard to buy off (IFU
  notes S3, "no stored predecode array").
- No elaborate multi-level redirect priority ladder — C906 has no L0 BTB and
  no indirect BTB at all (design doc S1), so the redirect chain PCGEN
  arbitrates is four levels shorter than C910's ten.

What remains is still a real pipelined machine with a real correctness
problem: variable-length instructions, a cache that is not free, and control
transfers discovered and resolved late. The answer is the same shape as
every front end — guess early, correct late, oldest correct answer wins —
just built with far fewer moving parts.

### 1.2 Why C906's stage boundaries fall where they do

Unlike C910's IF/IP/IB three-stage split with dedicated per-stage `*ctrl`/
`*dp` module pairs, C906 has **no separate stage modules** at all (IFU notes
S0). The instantiation order in the donor's own `aq_ifu_top.v:459-840` is
flat: `pcgen → ctrl → icache → btb → ipack → ibuf → pred → vec`, one
instance of each. The "stages" below are register boundaries inside that
flat graph, not module boundaries:

| Stage | What becomes available | Where it lives |
|---|---|---|
| **PCGEN** | a next-PC candidate | `aq_ifu_pcgen.v`, one register (`pcgen_ifpc`) plus a combinational priority mux |
| **ICache access** | the tag/data SRAM outputs, one cycle later | `aq_ifu_icache.v`, a single monolithic module with its **own internal** 2-cycle request/hit-check pipeline (IFU notes S2 item 3) — not two separate pipe-registered stages the way C910 splits IF and IP |
| **IPACK** | the 32-bit fetched word, re-sliced into halfwords | `aq_ifu_ipack.v`, purely combinational, live RVC-boundary detection |
| **IBUF** | a classified 16- or 32-bit instruction, ready to deliver | `aq_ifu_ibuf.v`, a 6-entry circular halfword queue that presents exactly **one** 32-bit instruction/cycle to IDU |

rv906 keeps this flat shape: `IFU.v` hoists PCGEN + the stall/cancel hub
(`ctrl`) + IPACK + IBUF + the reduced boot FSM as **sections** of one file,
exactly as `aq_ifu_top.v` instantiates them, rather than inventing stage
modules the donor does not have. `ICache.v` keeps the whole tag/data/
hit-check/refill/invalidate/AXI-master function in **one** module, matching
`aq_ifu_icache.v`'s own boundary — a smaller seam decision than rv12's C910
clone needed, because C906 draws that boundary itself.

### 1.3 The redirect ladder is four levels shorter

C906's next-PC arbiter (`aq_ifu_pcgen.v:237-253`, IFU notes S7) has seven
priority levels, not C910's ten — there is no L0 BTB level and no
indirect-BTB level because those structures do not exist in this front end
(design doc S1). In priority order:

| # | Level | Raised when |
|---|---|---|
| 1 | boot | reset-vector load |
| 2 | RTU / IU-BJU | retire-stage flush, or a branch/jump resolve |
| 3 | predictor "current-flow" | a same-cycle RAS-return or delay-replay correction, held only if ICache didn't grant |
| 4 | IPACK reissue | IBUF-stall-triggered same-PC refetch |
| 5 | predictor "change-flow" (BTB/BHT) | held only if ICache didn't grant |
| 6 | sequential | normal +4-byte increment, the common case |
| 7 | hold | nothing granted |

Levels 2 and 6 are the rung-1 engine: with every predictor off, a retire/BJU
resolve is the only way a taken control transfer is ever discovered, and
sequential increment is everything else. Section 5.1 below discusses the one
place rv906 had to depart from a literal transcription of this mux to make
rung 1 a working machine rather than a livelock.

### 1.4 The cancel network is three destinations, not ten

C906's stall/cancel hub, `aq_ifu_ctrl.v`, is 132 lines total (IFU notes S5.1)
— an order of magnitude smaller than C910's equivalent. Its cancel fan-out
reaches exactly **three** destinations: ICache, IPACK, BTB. IBUF and IPACK's
own internal buffer get their flush wired **directly** from RTU/IU,
bypassing the hub entirely; PCGEN's fetch-enable backpressure comes directly
from ICache's own grant signal, also bypassing the hub. rv906 clones this
exact point-to-point topology rather than inventing a shared/broadcast stall
bus — see §2.2 below for the actual wires.

---

## 2. The fetch pipeline (`rtl/IFU.v`, 841 lines)

The file's own section map, in instantiation order:

| Region | What it is |
|---|---|
| header, C906 file map, seam notes | `IFU.v:1-178` |
| SECTION: BOOT | `IFU.v:179-198` |
| SECTION: PCGEN | `IFU.v:199-434` |
| SECTION: CTRL | `IFU.v:435-479` |
| SECTION: IPACK | `IFU.v:480-685` |
| SECTION: IBUF | `IFU.v:686-841` |

Compare this to `aq_ifu_top.v`'s 863 lines of pure glue plus its five
sub-modules — the donor spreads the same logic across six files; rv906
makes each donor sub-block a section of one file, so every wire that used
to cross a module boundary stays internal and every wire that still crosses
one (IFU↔ICache, IFU↔BPU) keeps its donor name verbatim.

### 2.1 Boot (`IFU.v:179-198`)

`aq_ifu_vec.v`'s 4-state FSM (RESET/WARM_UP/HALT/IDLE) reduces to a single
one-shot pulse: `boot_rst_vld` fires for exactly one cycle out of reset and
forces the reset-vector load. There is no WARM_UP fan-out to model — no
IDU/IU/RTU/DTU exist yet in M1 to warm up or halt for — and no per-unit
handshake, since the donor's own boot module has no exception-vector
redirect logic either (confirmed: exception PC redirects come from RTU
directly into PCGEN's mux, not through `aq_ifu_vec.v`, IFU notes S6).

### 2.2 PCGEN (`IFU.v:199-434`) — the redirect mux and the cancel network

PCGEN owns `pcgen_ifpc[39:0]` (or a 64-bit host wire carrying the same 40
meaningful bits, per `rvproc_pkg.sv`'s note on `pcgen_icache_va[63:0]`) and
either advances it by 4 bytes or replaces it from the redirect ladder
(§1.3). The cancel network is copied from `aq_ifu_ctrl.v:93-129` term for
term:

- `ctrl_inst_fetch` → `ctrl_icache_req_vld`/`ctrl_btb_inst_fetch`: fetch is
  enabled whenever IBUF says it has room and boot/debug/low-power are all
  clear.
- `ctrl_if_stall = pred_ctrl_stall || icache_ctrl_stall` → **only**
  `ctrl_btb_stall`. The donor's analogous `ctrl_pcgen_stall` term is
  commented out in the real RTL — `aq_ifu_ctrl.v` does not gate PCGEN at
  all. PCGEN's own backpressure comes straight from ICache's
  `icache_pcgen_grant`, bypassing this hub entirely (IFU notes S5.1).
- `ctrl_if_cancel = rtu_ifu_flush_fe || pcgen_ctrl_chgflw_vld` fans out to
  exactly 3 destinations (ICache abort, IPACK cancel, BTB chgflw) — versus
  C910's ten-signal cancel fan-out, because there are only three downstream
  modules here that need a "cancel" concept at all. IBUF's flush is wired
  directly from RTU/IU instead, skipping this hub (IFU notes S5.1/S5.3).
- `ctrl_ibuf_pop_en = !idu_ifu_id_stall` is the **single point** where
  `idu_ifu_id_stall` — the only signal that ever crosses the IFU↔IDU
  boundary — is consumed (IFU notes S5.2).

### 2.3 IPACK (`IFU.v:480-685`) — halfword re-slicing, no predecode array

Three flopped 16-bit entries (`entry0/1/2`, mirroring `aq_ifu_ipack_entry.v`
×3). `entry0` is a **carry register**, not a fresh fetch — it exists purely
to hold the leftover low halfword of a 32-bit instruction whose upper half
lands in the next cycle's ICache word (the h0 straddle). Boundary detection
is `inst[1:0]==2'b11` computed live on each halfword (`ipack.v:373-380`),
never a stored bit — there is nothing to precompute against, since the
ICache only ever hands IPACK one 32-bit word (two halfwords) per cycle
(§1.1).

### 2.4 IBUF (`IFU.v:686-841`) — 6-entry queue, one instruction out

`aq_ifu_ibuf.v`'s 6-entry × 16-bit circular register queue: push ≤3
halfwords per cycle (the carry plus up to two new ones), pop ≤2 (one 32-bit
or 16-bit instruction). rv906's IBUF, unlike C910's 32-entry one-hot-pointer
buffer, uses a **binary** occupancy pointer (documented deviation, IFU.v's
own SECTION IBUF header note) — a design choice with a real, found
consequence discussed in §5.5.

The single-issue boundary is explicit in the port list: `ifu_idu_id_inst`
and `_inst_vld` are the only instruction-data signals crossing to IDU, with
no per-slot valid array — this is C906's concrete "fetch up to 2, decode 1"
asymmetry made structural.

---

## 3. The instruction cache (`rtl/ICache.v`, 610 lines)

32 KB, 2-way, 64-byte line, 256 sets — exactly half of C910's 64 KB/512-set
cache, same 59-bit tag-row layout, half the rows. The file's section map:

| Region | Lines |
|---|---|
| header, port list, C906 file map | `ICache.v:1-303` |
| SECTION: INVALIDATE | `ICache.v:304-348` |
| SECTION: ARRAYS | `ICache.v:349-453` |
| SECTION: HIT | `ICache.v:454-469` |
| SECTION: REFILL | `ICache.v:470-580` |
| SECTION: OUTPUT | `ICache.v:581-610` |

The rv906 module boundary matches the donor's exactly (§1.2): the MMU-facing
port group (`ifu_mmu_*`/`mmu_ifu_*`) is a port of *this* module, not of
`IFU.v`, because `aq_ifu_icache.v`'s own port list puts it there
(`icache.v:729`, `mmu_ifu_pa` consumed directly inside the cache) — a
structural fact for C906, not the arbitrary seam choice it was for C910.

### 3.1 Geometry and the tag row

Set index = byte-PC bits `[13:6]` (256 sets); data index = `[14:2]` (2048
rows, `ICACHE_DATA_IDX_W=11`); tag = `PA[39:12]` (28 bits). Fetch bandwidth
is **one 32-bit word per cycle** — a materially different, narrower fetch
bandwidth than C910's 16 bytes/cycle, not a simplification (`rvproc_pkg.sv`
flags this explicitly): the four data banks exist for refill bandwidth
(128 b/beat AXI on the real chip), not fetch width. The 59-bit tag row —
`{fifo, valid1, ptag1[27:0], valid0, ptag0[27:0]}` — is the identical layout
C910 uses, just at 256 rows instead of 512. `icache_tag_wen[2:0]` is 3 bits
wide for a 2-way array because bit `[2]` is a separate, shared FIFO-pointer
write-enable, not a third way (IFU notes S3).

### 3.2 Refill: one 512-bit beat, deviation 1

`SECTION: REFILL` (`ICache.v:470-580`) is where the SoC bus width forces the
one deviation from C906 that is not a choice. The donor issues 4×128-bit
WRAP bursts per line; rv906's SoC bus (inherited from the M0 scaffold) is
512 bits wide and single-beat, so `ICache.v` issues **one 512-bit read per
64-byte line** and slices it internally into 4 FILL cycles:

```
donor:  IDLE -> REQ -> WFD1 -> WFD2 -> WFD3 -> WFD4
rv906:  IDLE -> R_REQ -> R_WFD -> R_FILL (x4, off one register)
```

What survives: critical-word-first delivery **inside** the cache. The line
is written and bypassed to IPACK starting at the row that actually missed
(`miss_pa_r[5:2]` seeds the bypass word index, `ICache.v:509`), so the fetch
pipeline sees the same delivery order it would from a real WRAP burst — only
the bus transaction shape changed, not the microarchitecture. Full
discussion, including the two structural consequences (no refill abort,
an errored refill ends after one row), is in §5.2.

### 3.3 Live RVC boundaries, no predecode array

Unlike C910's precomputed `bry0`/`bry1` phase vectors stored at refill time,
C906's ICache stores **no predecode bits at all**. Boundary detection
(`inst[1:0]==2'b11`) happens live, per halfword, in IPACK (§2.3) — only
possible because the fetch width here is already just one word (two
halfwords) per cycle, so there is no multi-halfword interleaving hazard that
would otherwise force precomputing boundaries the way C910's wider fetch
does.

### 3.4 fence.i: the 256-set invalidate walk

`SECTION: INVALIDATE` (`ICache.v:304-348`), the donor's `INV_ALL` path
(`icache.v:1176-1301`). One tag row per cycle, 256 cycles, clearing both
valid bits and the FIFO replacement bit for every set — half of C910's
512-cycle walk, since C906 has half the sets. The same machinery serves
boot: `SRAM.v` has no reset, so the tag array is garbage on power-up and the
walk must complete before the first real fetch (§2.1).

### 3.5 The uncached path

Cacheability is one address bit, derived in `RVProc.v`'s MMU stub from
byte-address bit 31 of the fetch VA (a Task 6 bring-up finding, recorded at
the `mmu_ifu_prot` assignment site). An uncached fetch takes the identical
refill FSM with `alloc_r_r=0`: no tag/data array write happens, but the data
still reaches IPACK over the same critical-word bypass path a cacheable miss
uses — never a hit, one beat per fetch, exactly like `test/m1/uncached.S`
exercises at `0x7FFF_0000`.

### 3.6 Way prediction — resolved, and not built

The extraction notes flagged `cp0_ifu_iwpe` as an open item: is it real way
prediction, or a low-power bypass? Task 2 resolved this by reading
`icache.v:566-664` directly (`ICache.v:145-190`'s header note carries the
finding at the code site): the bit gates a **single-entry tag-hit buffer**
(`tag_hit_vld`/`direct_sel`/`cen_mask_vld`) that lets a repeated access to
the same line skip the tag/data SRAM read entirely. Nothing is *guessed* —
the "way" bypassed is already known from the previous access, not predicted
— so this is not way prediction in any sense, not even the C910-style
speculative-way-read rv12's clone modeled. It is a low-power bypass with no
functional effect on which way is selected. `cp0_ifu_iwpe` stays a frozen,
tied-0 port; the bypass logic itself was never built, because building it
would buy nothing this milestone needs (no low-power modeling is in scope)
and there is no way-prediction table to defer the way rv12 deferred C910's
`iwpe`. The honest clone of "iwpe=0" here is "the bypass does not exist in
this build," not "the bypass exists and is disabled."

---

## 4. C906 file cross-reference table

| rv906 file / section | Real C906 file(s) | What it clones |
|---|---|---|
| `IFU.v` SECTION BOOT | `aq_ifu_vec.v` (291 lines) | Reset→run one-shot pulse only; WARM_UP/HALT fan-out dropped (no consumers exist in M1) |
| `IFU.v` SECTION PCGEN | `aq_ifu_pcgen.v` (332 lines) | Next-PC register + 7-level redirect priority mux (`pcgen.v:237-253`) |
| `IFU.v` SECTION CTRL | `aq_ifu_ctrl.v` (132 lines) | The whole stall/cancel hub, near-verbatim |
| `IFU.v` SECTION IPACK | `aq_ifu_ipack.v` + `_entry.v` ×3 (500 lines) | 3-entry halfword re-slicer, live boundary detection, straddle carry |
| `IFU.v` SECTION IBUF | `aq_ifu_ibuf.v` + `_entry.v` ×6 + `_pop_entry.v` ×2 (1373 lines) | 6-entry circular queue, push≤3/pop≤2, single 32-bit instruction to IDU |
| `ICache.v` SECTION ARRAYS | `aq_ifu_icache_tag_array.v`, `aq_ifu_icache_data_array.v` | 1× 256×59 tag SRAM, 4× 2048×32 data SRAMs |
| `ICache.v` SECTION HIT | `aq_ifu_icache.v:741-812` | Tag compare, way select, 32-bit word mux |
| `ICache.v` SECTION REFILL | `aq_ifu_icache.v:844-1016` (refill FSM), `:1363-1367` (AXI) | Miss FSM, critical-word bypass, byte-swizzle; bus width is deviation 1 (§5.2) |
| `ICache.v` SECTION INVALIDATE | `aq_ifu_icache.v:1176-1301` | fence.i / icache.iall 256-set walk |
| `ICache.v` (way-pred header note) | `aq_ifu_icache.v:566-664` | Resolved: low-power tag-hit bypass, not way prediction — not built (§3.6) |
| `rvproc_pkg.sv` M1 constants | `cpu_cfig.h:135,142,344-358`, `aq_ifu_icache_tag_array.v:111`, `aq_ifu_icache_data_array.v:245,269,293,317` | ICache/BHT/BTB/RAS geometry, confirmed against the SRAM primitives' own port widths |
| `rtl/SRAM.v` | `aq_spsram_256x59`, `aq_spsram_1024x16`, `aq_spsram_2048x32` | Behavioral single-port model of the `aq_spsram_*` contract (1-cycle read, `q` held on `cen_n`) |

---

## 5. Design discussion

### 5.1 The rung-1 gating decision

C906's own IP/IB-equivalent redirect levels are, in the real RTL, wired to
fire on an invalid predictor's *default* output rather than being gated on
"a real prediction exists" — the same pattern rv12 found and fixed in
C910. With every predictor disabled (rung 1), a literal transcription of the
donor's redirect terms would self-redirect toward whatever an inactive
BTB/RAS defaults its target to, and rely entirely on a correctness backstop
to break the resulting loop. rv906 checked this explicitly during Task 3
(as the plan required, rather than assuming either C910's answer or its
opposite) and gates every predictor-sourced redirect level on the
predictor's own valid bit, exactly as C910's clone does. This is what makes
rung 1 — every predictor off — a real, working configuration rather than a
livelock, and it is the reason FetchSink's resolve/squash logic (its own
chapter's concern, but load-bearing here) is what actually corrects every
taken direct transfer at rung 1.

### 5.2 Deviation 1: the bus width, and what survives it

C906's real AXI4.0 master issues 4×128-bit WRAP bursts per 64-byte line.
rv906's inherited SoC bus is 512 bits and single-beat, so `ICache.v` issues
one 512-bit read per line instead (§3.2). Two structural consequences follow
from that choice, both intentional and both visible at the refill FSM's own
site (`ICache.v:470-475`'s header comment):

- **No refill abort on a redirect.** A single-beat AXI transaction cannot be
  cancelled mid-flight once the address handshake completes, so a
  mid-refill redirect always lets the fill finish; this differs from the
  donor, which can drop a WRAP burst partway through. Cost: occasionally
  filling a line the fetch no longer needs — never a correctness issue,
  since the array write only ever installs data that was legitimately
  fetched.
- **An errored refill ends after one row.** `fill_last_seq` collapses to
  `2'd0` when the read comes back with an AXI error (`ICache.v:497`), so
  there is nothing further to drain — the donor's four-beat error path has
  no equivalent here because there was only ever one beat.

Critical-word-first is preserved **inside** the cache regardless: the 4
logical rows are written and bypassed to IPACK starting at the row that
missed and wrapping (`miss_pa_r[5:2]` seeds `fill_seq_r`), so the fetch
pipeline's own view of delivery order is unaffected by the bus-width change
— only the wire transaction shape moved, exactly the deviation the design
doc records and rv12 set the precedent for.

### 5.3 The IBUF's binary pointer, and the push-count gap it inherited

C906's real `aq_ifu_ibuf.v` uses one-hot push/pop pointers over its 6
physical entries and computes its own create-side push count directly from
`h0_vld`/`h1_16bit_vld`/`h2_16bit_vld`/entry-valids inside `ibuf.v` itself
(extraction note S9, flagged there as "not fully traced"). rv906's IBUF
instead uses a binary occupancy pointer (documented deviation, `IFU.v`'s own
SECTION IBUF header) and, rather than reimplementing `ibuf.v`'s fuller
create-side arithmetic, trusts IPACK's three named retire-count flags
(`ipack_one_16bit_vld`/`ipack_secnd_vld`/`ipack_all_vld`,
`ipack.v:399-414`) as its sole push-count source.

This mattered in practice. The real `aq_ifu_ipack.v` source contains a
**commented-out** fourth term:

```verilog
//assign ipack_one_32bit_vld = !h0_vld && h1_32bit_vld && entry2_vld
//                           || h0_vld && entry1_vld && h2_32bit_vld;
//assign ipack_retire_two = ipack_one_32bit_vld || ipack_two_16bit_vld;
```

— i.e. real silicon's `ibuf.v` covers a case IPACK's three named flags do
not: a **standalone 32-bit instruction with nothing else valid the same
cycle**, the ordinary shape of any straight-line run of non-RVC code. Before
this was found (Task 6 bring-up, `iss_selftest.S`'s `pc=0x104` `addi
x0,x0,0`), that instruction was silently dropped: entry1/entry2 were
overwritten by the next cycle's fetch before ever reaching IBUF, and the
eventual push carried whatever wrong-path halfwords happened to be sitting
there by then. The fix reinstates the real RTL's own commented-out formula,
both OR-terms, folded into the existing `ipack_ibuf_inst_two` push-count
slot (`IFU.v:585-660`, documented at the code site as BUG FIX #3). The
general lesson, consistent with what rv12 found in its own IBUF work: a
donor's own commented-out code is sometimes the missing case, not dead
weight — worth reading, not just skipping.

### 5.4 The BTB's collapsed 2-stage CAM pipeline

The real `aq_ifu_btb.v` runs its 16-entry CAM as a 2-stage pipeline: the tag
compare fires speculatively at PCGEN time against `pcgen_btb_ifpc`, is
flopped one cycle into `btb_entry_hit_flop`, and is validated/written back
at ID-stage time once the *same* instruction arrives there — a scheme that
relies on a fixed, small number of cycles between PCGEN and ID-stage set by
the real chip's fixed-latency ICache access and one-hot IBUF timing.

rv906's IBUF clone does not have that fixed-latency property: entries can
stay valid-but-unretired for many consecutive cycles under `--sink-stall` or
backpressure (§2.4/§3.6). Transplanting the real 2-stage pipeline as-is
would silently misalign the PCGEN-time tag-compare address against whatever
ID-stage bundle eventually shows up beside it, for no observable benefit —
the M1 oracle only cares about *which* instructions commit, never *when* a
predictor fires (design doc S4.1). `BPU.v`'s BTB section (its own chapter,
`docs/03-bpu.md` §5.1, documents the CAM itself) therefore does the tag
compare, hit-target reconstruction, ID-stage validation and write-back all
**together, same-cycle**, keyed off the ID-stage bundle's own PC — the
identical CAM contents, hit decision and FIFO-replacement rule, one pipeline
stage later than real silicon. `pcgen_btb_ifpc` remains a frozen, genuinely
unused port for this reason, documented at the code site rather than
quietly dropped.

### 5.5 Bugs found during bring-up (Task 6, Task 9)

Four bring-up bugs are documented at their code sites in `IFU.v` and
`FetchSink.v`, because each is the kind of thing that only shows up once the
whole stack runs together and is worth a permanent record for whoever next
touches this file:

- **BUG FIX #1** (`IFU.v:287-312`): `boot_rst_vld` is high the same cycle
  `ctrl_icache_req_vld` first reads 1 (nothing gates fetch-enable during
  reset the way the real `vec_ctrl_reset_mask` does), so the combinational
  fetch address needed its own `boot_rst_vld` priority term threaded through
  `pcgen_fetch_pc` — without it, ICache's very first request read address 0
  instead of the reset vector, one register stage before the sequential
  reset-vector load itself landed.
- **BUG FIX #2** (`IFU.v:315-346`): an earlier fix's own extra
  `else if (boot_rst_vld)` branch in the *sequential* PCGEN update clobbered
  a legitimate same-cycle `icache_pcgen_grant` advance, permanently stalling
  `pcgen_ifpc` at the reset vector. The correct fix needed no separate
  sequential priority level at all, once BUG FIX #1's combinational fix was
  in place.
- **BUG FIX #4** (`IFU.v:377-434`): `pcgen_pipe_ifpc` (BPU's ID-stage PC
  view) was latching "whatever PCGEN most recently requested" on every
  grant, while IPACK's entries — the data BPU actually classifies — can lag
  several grants behind under backpressure. Found running `dense_br.S`
  region C (two back-to-back compressed branches, 128 iterations) at every
  rung: the committed stream silently dropped whole instructions starting
  ~90 iterations in.
- **FetchSink's stall port** (`FetchSink.v:849-863`): `idu_ifu_id_stall` was
  computed but never actually driven onto the port, so `--sink-stall`
  silently had no effect on IFU's own IBUF pop gating — a dropped-
  instruction bug caught by the FetchSink unit bench's own `T14`
  (`test_sink_stall_mode`) before it ever reached the full-stack suite.

### 5.6 What is deferred, and why it is safe

| Item | Why it is safe to defer | When |
|---|---|---|
| Way-prediction / low-power tag-hit bypass (`cp0_ifu_iwpe`) | Resolved (§3.6) to be a low-power bypass, not a predictor; no functional table to build or disable — the bypass itself was never implemented, since M1 models no low-power behavior | pre-M8, if ever |
| ICache next-line prefetch (`cp0_ifu_icache_pref_en`) | Performance only; `cp0_ifu_icache_pref_en=0` is a supported real configuration | pre-M8 |
| Non-blocking D$/miss-buffer, HW stream prefetch | LSU-side, not IFU | M3 |
| Snoop/coherence invalidate paths | Single-core, not applicable to this design's scope | never |
| RVV plumbing | Excluded per the parent design | never |
| HAD-equivalent/non-standard debug hooks | The parent design clones RISC-V standard Debug instead | M7 |

---

## 6. Verification

The full M1 harness — the two-oracle architecture, the CLI flags, the
directed test suite and the full regression matrix — has its own chapter:
`docs/08-verification.md`. The short version: `rtl/FetchSink.v` and the
C++ golden fetch-ISS (`m1_iss.h`, via `RVProcTest.cpp`) are independently
written from the same spec text and compared online every cycle; the
predictor chicken-bit ladder (rungs 1-4) is layered on top, and the same
committed stream is required at every rung.
