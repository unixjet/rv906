# C906 BPU Extraction Notes (M1 working material)

Source: `refs/openc906/C906_RTL_FACTORY/gen_rtl/ifu/rtl/` (file:line refs relative). Config
header: `../../cpu/rtl/cpu_cfig.h` (relative to the same `gen_rtl` root). PC is a **byte
address**, 40 bits (matches sibling's `2026-08-20-c906-ifu-pipeline-extraction.md`, which owns
pcgen/icache/ibuf/ipack; this note covers only bht/bht_array/btb/btb_entry/ras/ras_entry/pred
and the 3 SRAM primitives, per scope).

**Headline finding**: C906's predictor is three *small, independent, differently-sized*
structures, not one unified table family. BHT is the only SRAM-backed one (and its "16Kb" is
an exact bit count, not a family/aggregate — see §6). BTB and RAS are both **pure flop arrays
with zero SRAM**, despite `aq_spsram_256x59`/`aq_spsram_2048x32` being handed to me as
candidates — those two belong to the ICache tag/data arrays, not the BPU (§5). BHT indexing
uses **only the GHR, no PC contribution at all** — the two PC-derived ports wired into the BHT
module (`pred_bht_pc`, `iu_ifu_bht_cur_pc`) are provably dead code (§1.4). So "Gshared" in the
manual is a marketing label for a global-history 2-bit-counter table, not McFarling's
PC-xor-GHR gshare.

## 0. Inventory

| Structure | Kind | Storage | Entries | Field widths | Region limit |
|---|---|---|---|---|---|
| BHT | global-history 2-bit sat. counter table | 1x SRAM (`aq_spsram_1024x16`) | 1024 rows x 8 lanes/row = 8192 counters | 2b/counter (split across 2 byte-planes) | n/a (direction only) |
| BTB | fully-assoc. target cache | 16 flops (`aq_ifu_btb_entry` x16) | 16 | tag 16b + target 16b + valid 1b = 33b/entry | 64KiB (tag/tgt = PC[15:0] only) |
| RAS | return-address stack | 4 flops (`aq_ifu_ras_entry` x4) | 4 | pc 24b/entry, no valid/priv bits | 16MiB (tgt reassembled with id-stage PC[39:24]) |

## 1. BHT (`aq_ifu_bht.v`, 549 lines + `aq_ifu_bht_array.v`, 123 lines)

### 1.1 Config resolution ("16Kb" question — see §6 for the full arithmetic)

`aq_ifu_bht.v:130-132`: `IDX_WIDTH = `BHT_INDEX_WIDTH`, `HIS_WIDTH = `BHT_INDEX_WIDTH+4`,
`DATA_WIDTH = 16`. `BHT_INDEX_WIDTH` is defined in `cpu_cfig.h:344-358` as one of
`{7,8,9,10}` gated by `BHT_2K`/`BHT_4K`/`BHT_8K`/`BHT_16K`. **`cpu_cfig.h:135` unconditionally
defines `BHT_16K`** (not inside any alternate-selecting `ifdef`/`else`) → active
`BHT_INDEX_WIDTH = 10`, `HIS_WIDTH = 14`. This matches the port widths directly:
`bht_idx[9:0]` (10b, `aq_ifu_bht_array.v:34,54`), `bht_ghr`/`bht_vghr` regs `[13:0]` (14b,
`aq_ifu_bht.v:66,76`).

### 1.2 SRAM array

`aq_ifu_bht_array.v:102-110` instantiates exactly **one** `aq_spsram_1024x16` (depth 1024,
width 16, confirmed by the primitive's own ports `A[9:0]`/`D,Q,WEN[15:0]` in
`aq_spsram_1024x16.v:28-34`). Commented-out sibling options at same file
`aq_ifu_bht_array.v:98-101` (`aq_spsram_128x16`/`256x16`/`512x16`/`1024x16`) confirm this is a
depth-configurable *family of one array*, not multiple arrays — the "16K" config just widens
the address bus of the same single SRAM.

### 1.3 GHR

Two 14-bit registers, both clocked on branch-resolution events, no third copy (unlike C910's
vghr/rtughr/checkpoint-FIFO triple):
- `bht_ghr[13:0]` (architectural) — shifts in `iu_ifu_bht_taken` on every `iu_ifu_br_vld`
  (resolved branch), zeroed by `cp0_ifu_bht_inv` or reset (`aq_ifu_bht.v:196-206`).
- `bht_vghr[13:0]` (speculative) — on `iu_ifu_bht_mispred`, reloads from
  `{bht_ghr[12:0], iu_ifu_bht_taken}` (recovery); else on `pred_bht_br_vld` (a branch reached
  ID stage) shifts in `bht_pred_taken` (the just-made prediction) (`aq_ifu_bht.v:208-220`).

### 1.4 Indexing — GHR only, no PC (confirmed dead ports)

`bht_idx[9:0]` (`aq_ifu_bht.v:253-257`), four cases, all pure windows of a GHR register, none
reference PC:
- invalidate sweep: `bht_inv_cnt[9:0]`
- miss-pred refill read1: `bht_ghr[13:4]`
- miss-pred refill read2: `bht_ghr[12:3]`
- miss-pred refill write / normal update: `bht_ref_vghr[13:4]`
- **normal prediction read (default case): `bht_vghr[11:2]`**

Within-row lane select is a separate 3-bit field, also pure GHR: read lane =
`bht_vghr[2:0]` → `bht_sel_way[7:0] = 8'b1 << bht_vghr[2:0]` (`aq_ifu_bht.v:305`); write lane =
`bht_ref_vghr[2:0]` (`bht_upd_idx`, `:455`). **Bit 2 of vghr is used by both the index and the
lane select** (deliberate 1-bit fold, `:257` vs `:305`) — so the effective read address space
is a contiguous 12-bit window `vghr[11:0]` (10 idx bits + 3 lane bits, one bit shared).

Two ports are wired all the way from `aq_ifu_pred.v` into the BHT module specifically to carry
PC information (`pred_bht_pc[2:0]`, computed at `aq_ifu_pred.v:514-516` from `pred_idpc`/
`pred_h0_pc`; and `iu_ifu_bht_cur_pc[39:0]`) — **grep confirms both appear only in the port
list and `&Force` synthesis-tool comments in `aq_ifu_bht.v`, never in any `assign`/`always`
block** (checked via `grep -n` for each name across the whole file). These are dead inputs.
**Conclusion: this BHT is indexed purely by (a window of) global history, with zero PC
contribution — not a PC-xor-GHR gshare.** `pred_bht_pc`'s computation looks like leftover
plumbing for a per-slot PC-offset scheme that was superseded by the vghr-lane-select mechanism
actually wired up.

**MUST-VERIFY / read vs. write window mismatch** (same class of anomaly the C910 note flagged
for its BHT, `2026-08-18-c910-bpu-extraction.md` §1): read window is `vghr[11:2]`(idx) ∪
`vghr[2:0]`(lane) = contiguous bits `[11:0]`. Write window is `ref_vghr[13:4]`(idx) ∪
`ref_vghr[2:0]`(lane) = bits `[13:4]` ∪ `[2:0]`, i.e. **skips bit 3** and extends 2 bits higher
than the read window. Plausible explanation (not confirmed from files in scope): `ref_vghr` is
loaded from the *architectural* `bht_ghr` at resolve time, which — assuming the fetch-to-resolve
pipeline is a small constant number of cycles/predictions deep — could legitimately sit ~2
history-shifts "ahead of" the speculative `vghr` value the branch was predicted with, making
the index windows consistent in absolute history-age terms even though their bit-slices look
shifted. Confirming this needs the pipeline depth from `pcgen`/`ipack`/`ibuf` (sibling's
scope), not something resolvable from bht.v alone.

### 1.5 Counter is a standard 2-bit saturating counter, split across 2 SRAM byte-planes

Read output `bht_dout_rslt[15:0]`: high byte `[15:8]` and low byte `[7:0]` are two independent
1-bit-per-lane planes selected by the same one-hot `bht_sel_way[7:0]`
(`bht_sel_result[1]`/`[0]`, `aq_ifu_bht.v:307-308`). Prediction direction = `bht_sel_result[1]`
only (`bht_pred_taken`, `:316`) — the high-byte ("A") plane. The update case table
(`iu_ifu_bht_pred[1:0]` = captured `{A,B}` at prediction time, `iu_ifu_bht_taken` = resolved
outcome, `:457-509`) decodes to, treating `{A,B}` as a 2-bit number 0..3:

| state | meaning | +taken | +not-taken |
|---|---|---|---|
| 0 = `(A,B)=(0,0)` | strong not-taken | → 1 | → 0 (no-op, `upd_en=0`) |
| 1 = `(0,1)` | weak not-taken | → 2 | → 0 |
| 2 = `(1,0)` | weak taken | → 3 | → 1 |
| 3 = `(1,1)` | strong taken | → 3 (no-op) | → 2 |

i.e. a textbook saturating up/down 2-bit counter (increment on taken, decrement on not-taken,
saturate at 0/3), just stored as two separate 1-bit byte-planes (`A`=MSB="taken" plane,
`B`=LSB="ntaken" plane) instead of one 2-bit field — confirmed purely from the case table at
`aq_ifu_bht.v:461-507`, re-derived by hand (not asserted from any comment; the file has no
comment naming this a saturating counter).

Update round-trip: `aq_ifu_pred.v:787-788` forwards `bht_pred_rslt` (captured at prediction
time) to `ibuf` as `pred_ibuf_br_taken{0,1}`; IU returns it at resolve time as
`iu_ifu_bht_pred[1:0]` into `aq_ifu_bht.v`'s update case table (`:458-461`) — this is how the
"predicted state" half of the update decision reaches the BHT despite the SRAM itself never
being re-read at resolve time for that purpose (the mispred-path re-reads are for refill/replay,
not for recovering the predicted state — see §1.6).

### 1.6 Same-row two-branch trick (why `bht_pred_mem_taken` exists)

The array is single-ported (one `aq_ifu_bht_array` instance) but `aq_ifu_pred.v` fetches up to
2 instructions/cycle and needs a prediction for a **second** branch in the same bundle without
a second SRAM read. `aq_ifu_bht.v:310-314`: `bht_mem_idx[2:0] = {bht_vghr[1:0],
bht_pred_taken}` — reuses the *same* already-fetched 16-bit row, but picks a different one of
the 8 lanes by substituting the first branch's own (not-yet-resolved) predicted outcome for the
lane's low bit. `bht_pred_mem_taken` is this second lookup's direction bit, output to
`aq_ifu_pred.v` and consumed at `aq_ifu_pred.v:550-551` (`pred_delay_br1_taken`) exactly when
instr0 is a branch predicted not-taken and instr1 is also a branch — i.e. it answers "what
would the BHT have said for instr1, assuming instr0's prediction becomes part of history" using
data already sitting in the just-read row. This resolves what was otherwise an unlabeled,
easily-misread signal.

### 1.7 Invalidate

`cp0_ifu_bht_inv` drives a 3-state FSM (IDLE/WRTE/READ, `aq_ifu_bht.v:319-370`) that sweeps all
`2^IDX_WIDTH = 1024` rows over 1024 cycles (`bht_inv_cnt[9:0]` full-count check, `:375-382`) —
same sweep-invalidate style as C910's BHT/BTB, sized to the active 1024-row config.

## 2. BTB (`aq_ifu_btb.v`, 753 lines + `aq_ifu_btb_entry.v`, 163 lines) — NOT SRAM-backed

### 2.1 Structure — fully-associative flop CAM, 16 entries

`aq_ifu_btb.v:216` `parameter ENTRY_NUM = 16`; 16 instances of `aq_ifu_btb_entry`
(`:220-583`), each with its own tag/target/valid flops (`aq_ifu_btb_entry.v:55-57`). **No
SRAM instantiation anywhere in either file** — grepping both files for `aq_spsram_` returns
nothing (checked). `BTB_ADDR_WIDTH = 16` (`aq_ifu_btb.v:154`, and again per-entry at
`aq_ifu_btb_entry.v:89`).

### 2.2 Tag / target fields and the 64KiB region limit

Per entry: `btb_tag[15:0]`, `btb_tgt[15:0]`, `btb_vld` (`aq_ifu_btb_entry.v:55-57`) = 33 bits.
Tag compare is a literal equality on the **low 16 bits of the PC only**:
`btb_rd_acc_tag = pcgen_btb_ifpc[15:0]` (read) / `btb_wr_acc_tag = pred_btb_cur_pc[15:0]`
(write) (`aq_ifu_btb.v:637-638`); hit = `btb_tag == acc_tag && btb_vld`
(`aq_ifu_btb_entry.v:147-150`). Reconstructed target reuses the **current fetch PC's** upper
24 bits: `btb_pcgen_tar_pc[39:0] = {pcgen_btb_ifpc[39:16], btb_hit_tgt[15:0]}`
(`aq_ifu_btb.v:740`). Both facts together mean: (a) any two branches whose addresses agree in
bits `[15:0]` but differ above bit 15 alias in this fully-associative CAM (64KiB aliasing
period, tag doesn't cover the rest of the 40-bit PC at all); (b) predicted targets are
constrained to the same 64KiB-aligned region as the branch's own PC. `pred_btb_cur_pc[39:0]`
itself is PC-aligned to 4 bytes by the caller (`aq_ifu_pred.v:804`: `{...[39:2], 2'b0}`), so
only 14 bits of the 16-bit tag actually vary in practice.

### 2.3 Replacement / update

Round-robin one-hot FIFO pointer `btb_fifo[15:0]` advances only on an allocate-new-entry
update (`btb_entry_upd_vld && !btb_wr_hit_vld`, `aq_ifu_btb.v:604-614`) — a tag *hit* on write
instead does an in-place replace of the SAME entry (`btb_entry_replace =
btb_entry_upd_vld && btb_wr_hit_vld`, `:616-623`), so there is no true "replacement policy"
beyond simple round-robin allocation on miss. No confidence/counter field at all — a valid tag
match always yields a target (direction comes entirely from BHT, §4).

### 2.4 Invalidate — single-cycle, not a sweep

Two invalidate paths, both instantaneous (all 16 flop entries in one cycle, since it's flops
not SRAM): global `cp0_ifu_btb_clr` clears all 16 unconditionally
(`btb_entry_clr[15:0] |= {16{cp0_ifu_btb_clr}}`, `:596`); per-entry `btb_clr_one` clears just
the entry that mispredicted (`pred_btb_mis_pred && cp0_ifu_btb_en && btb_wr_hit_vld`, `:593`).
Contrast with BHT's 1024-cycle sweep (§1.7) — BTB's small flop array makes a combinational
full clear cheap.

## 3. RAS (`aq_ifu_ras.v`, 267 lines + `aq_ifu_ras_entry.v`, 112 lines) — NOT SRAM-backed, 4 entries

### 3.1 Structure

`aq_ifu_ras.v:137` `parameter ENTRY_NUM = 4`; 4 instances of `aq_ifu_ras_entry`, each storing
only a 24-bit PC (`aq_ifu_ras_entry.v:36,49,90,106` — `entry_pc[23:0]`), **no valid bit, no
privilege field** (unlike C910's `{pc,filled,priv}` RAS entries). This is dramatically smaller
than C910's 12-entry spec + 6-entry retire-mirror design — a flat 4-slot stack.

### 3.2 One storage array, two one-hot pointers (no separate spec/arch content copies)

Two independent one-hot 4-bit pointer registers, both rotate-shift, no counter/binary pointer:
- `ras_pop[3:0]` — the **speculative** pointer, used both to select the read (`ras_tar_pc`
  case mux, `aq_ifu_ras.v:238-244`) and to select which physical entry the NEXT push writes
  (`entry3_upd = pred_ras_link_vld && ras_pop[0]`, etc., `:251-254` — write target is the
  slot the pointer is rotating *into*). Advances (right-rotate, push) on `pred_ras_link_vld`
  (predicted call, `:209-210`); advances (left-rotate, pop) on
  `pred_ras_ret_vld && !ras_cur_st` (predicted return, `:211-212`); **snaps back to
  `ras_bju`** on `rtu_ifu_flush_fe || iu_ifu_bht_mispred || (iu_ifu_pc_mispred &&
  !iu_ifu_link_vld)` (`:207-208`).
- `ras_bju[3:0]` — the **confirmed** pointer, rotates the same way but only on
  IU-resolved events: `iu_ifu_link_vld` (confirmed call, right-rotate, `:221-222`) /
  `iu_ifu_ret_vld` (confirmed return, left-rotate, `:223-224`).

Critically, **there is only one physical content array** — recovery on misprediction/flush
copies the *pointer* (`ras_pop <= ras_bju`) but there is no mirrored/checkpointed copy of
entry *contents* to restore from (contrast C910's explicit arch-mirror `rtu_entryN` reload,
`ct_ifu_ras.v:886-891`). **Flag (inference, not confirmed as a functional bug from this file
alone)**: with only `ENTRY_NUM=4` physical slots and a mod-4 rotating pointer, if more than 4
speculative calls/returns are in flight past the last confirmed one before a misprediction is
caught, the physical slot the recovered `ras_bju` pointer lands on could already have been
overwritten by intervening (possibly wrong-path) speculative pushes — the design only
guarantees correctness within a window of ≤4 in-flight unresolved link/return events. Verifying
whether the surrounding pipeline can exceed that window is out of BPU-file scope.

### 3.3 Push data and the 16MiB region limit

Push value = low 24 bits of (current ID-stage PC + 2 or 4, i.e. return address after the call):
`pred_ras_link_pc[23:0] = pred_cur_pc[23:0] + ras_link_offset[23:0]` where
`ras_link_offset = pred_inst0_32 ? 4 : 2` (`aq_ifu_pred.v:625-628`). Reconstructed target on
pop reuses the **current ID-stage PC's** upper 16 bits, not a stored tag:
`pred_ras_tar[39:0] = {pred_idpc[39:24], ras_pred_tar_pc[23:0]}` (`aq_ifu_pred.v:656`) —
predicted return addresses are constrained to the same 16MiB-aligned region as the PC at the
point of return, a materially looser constraint than BTB's 64KiB (§2.2), consistent with
call/return typically spanning less code distance than arbitrary branch targets but still a
real, quantifiable limit. **No RAS-specific invalidate signal exists at all** — grep for
`ras_inv`/`ras_clr` across both RAS files returns nothing; only `cpurst_b` (full reset) and
`rtu_ifu_flush_fe`/`pcgen_pred_flush_vld` (via the RAS-stall FSM in `aq_ifu_pred.v:663-671`,
§3.4) affect its state.

### 3.4 RAS-busy stall FSM (in `aq_ifu_pred.v`, not `aq_ifu_ras.v`)

2-state FSM `RAS_IDLE`/`RAS_WAIT` (`aq_ifu_pred.v:660-693`): a predicted return
(`pred_ras_ret_vld`) moves to `RAS_WAIT` and stalls further ID-stage prediction
(`pred_ret_stall = ras_cur_st==RAS_WAIT && pred_ras_ret_vld`, `:696`, folded into
`pred_id_stall`, `:741`) until IU confirms the return (`iu_ifu_ret_vld`) — i.e. only one
predicted return can be in flight through the ID stage at a time, unlike BHT/BTB which pipeline
freely.

## 4. Prediction arbitration (`aq_ifu_pred.v`, 819 lines) — how BHT/BTB/RAS combine

This module lives at the ID (decode) stage, working on a fetched 2-instruction bundle
(`ipack_pred_inst0`=32b, `ipack_pred_inst1`=16b — the second slot is only ever a 16-bit/RVC
half). Predecode (branch/jump/link/return classification, immediates) comes from
`aq_ifu_pre_decd` (`:421-438`, a separate small module outside the scope list — not
characterized further here beyond its outputs' names).

### 4.1 Two independent, differently-timed contributions

- **BHT → direction only.** `pred_inst0_taken = pred_br_vld0 && bht_pred_rslt[1] ||
  pred_jmp_vld0` (`:587`) — BHT's high-byte plane bit (§1.5) gates whether instr0's branch is
  taken; unconditional jumps (`pred_jmp_vld0`) are always taken regardless of BHT. Same pattern
  for instr1 (`:589-591`), consuming the second-lookup trick's result via `bht_pred_rslt`
  (already covers instr1 in the two-branch case, §1.6) — note `bht_pred_mem_taken` itself feeds
  the *delay* logic (§4.2), not this direction mux directly.
- **BTB → target only, produced earlier (PCGEN stage) and validated here.** BTB's CAM lookup
  and redirect happen upstream in `aq_ifu_btb.v` against `pcgen_btb_ifpc` (i.e. before this
  module runs); `aq_ifu_pred.v` only re-derives the "correct" target from the decoded
  immediate (`pred_br_tar[39:0] = pred_cur_pc + pred_br_imm`, `:596-598`) and **compares it
  against what BTB already predicted**: `btb_mis_pred = (pred_br_tar != btb_pred_tar_pc ||
  !pred_br_taken) && btb_pred_tar_vld && ipack_pred_inst0_vld` (`:720-723`). The final
  change-of-flow decision is BTB-first, ID-stage-corrects-on-disagreement:
  `pred_chgflw = btb_pred_tar_vld ? (btb_mis_pred && !pred_delay_br_raw) : pred_br_taken`
  (`:725-726`) — if BTB had a valid prediction and it was right, ID stage does *not* re-issue a
  redirect (fetch is already following it); if BTB had no entry, ID stage's own
  BHT-direction + immediate-computed target drives the redirect directly.
- **RAS → its own path, bypasses BTB entirely.** Returns never go through the
  `btb_mis_pred`/`pred_br_tar` comparison at all — `pred_ras_ret_vld0/1` (`:617-618`) feed a
  separate output channel (`pred_pcgen_curflw_*`, §4.3) using `pred_ras_tar` directly
  (`:776`). Calls (`pred_ras_link_vld`) push into RAS independently of any BTB read/write.
  BTB and RAS never compare notes on the same instruction.

### 4.2 Delay/replay for a not-taken-then-taken pair in one bundle

`pred_delay_br_raw = pred_br_vld0 && !bht_pred_rslt[1] && pred_br_vld1` (`:546`) — when instr0
is a branch predicted not-taken and instr1 is also a branch, the bundle can't be fully resolved
this cycle (single BHT direction-mux path is occupied by instr0); `pred_delay_br1_taken`
(`:550-552`) uses the §1.6 same-row second lookup (`bht_pred_mem_taken`) to decide whether to
splice in a same-cycle-deferred redirect (`delay_chgflw`/`chgflw_pc_ff`, `:562-581`) the
following cycle rather than waiting a full extra fetch round-trip.

### 4.3 Two output redirect channels, not one

`pred_pcgen_chgflw_*` ("change-flow": BHT/BTB branch redirects, driven from `pred_chgflw_fin`,
`:769-773`) is distinct from `pred_pcgen_curflw_*` ("current-flow": RAS returns and the
delay-slot replay from §4.2, `:774-777`). Exact pipeline-timing semantics of this split
(what "current" vs "change" mean to `pcgen`) are `pcgen.v` territory (sibling scope) — noted
here only because it's the visible seam where RAS's redirect and BHT/BTB's redirect diverge
structurally, confirming §4.1's "RAS bypasses BTB" finding at the output-port level too.

### 4.4 Chicken bits

Three independent enables, one per structure, no interlocks between them observed in this
file: `cp0_ifu_bht_en` (gates BHT SRAM `CEN`, `aq_ifu_bht.v:233-244`), `cp0_ifu_btb_en` (gates
all BTB read/write/clear side effects, `aq_ifu_btb.v:593-702`), `cp0_ifu_ras_en` (gates RAS
push validity and target selection — when off, `pred_ras_tar` degenerates to
`pred_idpc[39:0]` i.e. RAS is simply not consulted, `aq_ifu_pred.v:614,656`). Plus
`cp0_ifu_bht_inv` (BHT-only 1024-cycle sweep, §1.7) and `cp0_ifu_btb_clr` (BTB-only, instant,
§2.4). No equivalent RAS invalidate (§3.3).

## 5. SRAM primitive resolution (the three files handed to me as candidates)

Grep of every `.v` file in the IFU rtl dir for `aq_spsram_` instantiations (not comments):

- `aq_spsram_1024x16` → **only** `aq_ifu_bht_array.v:102` (the BHT, and nothing else in the
  IFU). This is the *only* SRAM primitive of the three actually used by the branch predictor.
- `aq_spsram_256x59` → **only** `aq_ifu_icache_tag_array.v:111` — the ICache tag array
  (`ICACHE_32K` config, `I_TAG_INDEX_WIDTH=8` per `cpu_cfig.h:375`... actually resolves to 256
  sets via the `256x59` instance name itself). **Not part of the BPU.**
- `aq_spsram_2048x32` → **only** `aq_ifu_icache_data_array.v` (4 instances,
  `x_aq_spsram_2048x32_{0,1,2,3}`, one per way/bank) — the ICache data array. **Not part of
  the BPU.**

This matches the sibling pipeline note's independent finding (`2026-08-20-c906-ifu-pipeline-
extraction.md` §1) that `aq_spsram_1024x16` is used "only by `aq_ifu_bht_array.v:102`". Net:
**BHT is the only SRAM-backed predictor structure; BTB and RAS are pure flop arrays** (§2, §3).

## 6. Resolving the "BHT 16Kb" question

Manual Table 1.1 lists "BHT 16Kb" as a config parameter. RTL arithmetic, fully chained:
`cpu_cfig.h:135` defines `BHT_16K` unconditionally (active config) → `cpu_cfig.h:357`
(`` `ifdef BHT_16K``) sets `BHT_INDEX_WIDTH = 10` → `aq_ifu_bht.v:130`
(`IDX_WIDTH = `BHT_INDEX_WIDTH`) → `bht_idx[IDX_WIDTH-1:0]` = 10 bits → the single SRAM
instantiated is `aq_spsram_1024x16` (`aq_ifu_bht_array.v:102`, depth 1024 = 2^10, width 16,
confirmed against the primitive's own port declarations `aq_spsram_1024x16.v:28,31`).

**Depth x width = 1024 x 16 = 16,384 bits = 16 Kibibit, exactly matching "16Kb."** This is not
an approximation and not a family-of-tables aggregate — it is the literal total bit capacity
of the single BHT SRAM array, and the RTL's own config-macro name (`BHT_16K`) directly encodes
this same number. The design-doc's open question ("does 16Kb correspond to a single table or a
family of tables") is answered: **a single table**, and the match is exact. (The commented-out
`128x16`/`256x16`/`512x16` alternates at `aq_ifu_bht_array.v:98-100` confirm this is a
depth-configurable single array, i.e. the `BHT_2K`/`4K`/`8K`/`16K` macros in `cpu_cfig.h` are
four size options for that *same* one table, not four different tables.)

Each of the 1024 rows holds 8 independent 2-bit saturating counters (one per 3-bit GHR lane,
§1.5), so in counter-count terms this is 1024 x 8 = 8192 total 2-bit counters — but the "16Kb"
figure in the manual is the raw SRAM bit count (1024 x 16b), not a counter count (which would
read "16K counters" = 32Kbit, a different number). Worth flagging so nobody double-converts
this later: **16Kb = 16384 raw bits = 8192 two-bit counters**, both true simultaneously, "16Kb"
in the manual refers to the former.

## 7. Complexity / storage totals

File line counts: `aq_ifu_bht.v` 549, `aq_ifu_bht_array.v` 123, `aq_ifu_btb.v` 753,
`aq_ifu_btb_entry.v` 163, `aq_ifu_ras.v` 267, `aq_ifu_ras_entry.v` 112, `aq_ifu_pred.v` 819 →
~2786 total (predictor-proper files only; excludes `aq_ifu_pre_decd.v` which is shared/sibling
territory).

Storage: BHT SRAM 1024x16 = 16,384 bits (16Kb, §6). BTB flops: 16 entries x 33b (16 tag + 16
target + 1 valid) = 528b, plus a 16b round-robin FIFO pointer = 544b total. RAS flops: 4
entries x 24b (pc only) = 96b, plus two 4b one-hot pointers (`ras_pop`,`ras_bju`) = 104b total.
GHR: 2x14b (`bht_ghr`,`bht_vghr`) = 28b. **Total flop-based predictor state (excluding the BHT
SRAM): 544 + 104 + 28 = 676 bits** — for comparison, C910's flop-based L0-BTB+RAS+GHR/path
state alone was ~1504b (template §9), so C906's entire non-SRAM predictor state is smaller than
just one of C910's several flop structures. This is consistent with C906 being T-Head's
smaller/simpler in-order core.

## 8. Read-directly spans

BHT idx/lane derivation (all cases) `aq_ifu_bht.v:246-316`; BHT update case table
`:457-509`; BHT GHR update recurrences `:196-220`; BHT invalidate FSM `:319-382`. BTB entry
hit/update `aq_ifu_btb_entry.v:117-150`; BTB FIFO/replace `aq_ifu_btb.v:604-623`; BTB
target reconstruction `:740`. RAS pointer recurrences (both `ras_pop` and `ras_bju`)
`aq_ifu_ras.v:203-227`; RAS entry write ports `:251-255`. Arbitration: BTB-vs-ID-stage
mispredict check `aq_ifu_pred.v:720-733`; RAS/BTB channel split `:713,738-739,769-777`;
same-row second-branch trick consumption `:546-556`.

## 9. Open items not resolved from files in scope

- The read/write GHR-window bit-shift-by-2-with-a-gap (§1.4 MUST-VERIFY) needs pipeline depth
  from `pcgen.v`/`ibuf.v` (sibling scope) to confirm it's an intentional pipeline-latency
  compensation rather than an inconsistency.
- RAS's ≤4-in-flight-speculative-event correctness window (§3.2) is inferred from the
  single-content-array + pointer-only-recovery structure; whether the surrounding pipeline can
  actually exceed 4 in-flight link/return predictions before misprediction detection was not
  checked (would require `ibuf.v`/`ipack.v` depth, sibling scope).
- `aq_ifu_pre_decd.v` (branch/jump/link/return classification feeding `pred_br_vld0/1`,
  `pred_link_vld0/1`, `pred_ret_vld0/1`, `pred_jmp_vld0/1`) was not read — it was not in the
  file list handed to me for this note, and the sibling pipeline note also declined it as
  out-of-scope. Its classification rules (e.g. which registers count as "link"/"return" per
  the RVI/RVC ABI hint bits) are unverified here.
