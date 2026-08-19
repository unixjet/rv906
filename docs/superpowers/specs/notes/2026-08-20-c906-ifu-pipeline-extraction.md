# C906 IFU Pipeline Extraction Notes (M1 working material)

Source: `refs/openc906/C906_RTL_FACTORY/gen_rtl/ifu/rtl/` (all file:line refs relative to it,
unless another dir is named). Scope per assignment: pipeline datapath + ICache (pcgen, ctrl,
icache[+tag/data array], pre_decd, ibuf, ipack[+entry], vec, SRAM primitives). Predictor
tables (bht/bht_array/btb/btb_entry/ras/ras_entry) are a sibling researcher's scope; `pred.v`
and `pre_decd.v` are read here only far enough to see how the pipeline consumes their outputs.

Global: `cpu_cfig.h:142` selects `` `define ICACHE_32K `` (confirms manual's 32KB). PC is a
**byte address**, 40 bits (`cp0_xx_mrvbr[39:0]`, `pcgen_ifpc[39:0]`) — no half-word PC
convention here, unlike C910. Config header: `../../cpu/rtl/cpu_cfig.h`.

## 0. Architecture shape vs. C910 (read this first)

C906's IFU is **flat and shallow** compared to C910's IF/IP/IB three-stage split with
separate `*ctrl`/`*dp` module pairs per stage. There is exactly one small shared control hub
(`aq_ifu_ctrl.v`, 132 lines total) instead of per-stage ctrl modules, and the ICache itself
(`aq_ifu_icache.v`) is a single monolithic module containing its own 2-cycle
request/hit-check pipeline rather than being split across IF-stage and IP-stage module pairs.
Most backpressure is **direct point-to-point wiring between adjacent modules** (e.g.
`icache_pcgen_grant`, `ibuf_ipack_stall`), not routed through the ctrl hub — see §6.

## 1. Module graph (instantiated flat in `aq_ifu_top.v`, 863 lines of pure glue)

Instantiation order in `aq_ifu_top.v:459-840`, one instance of each (no unrolling, unlike
C910's generator-expanded RTL):

- `aq_ifu_pcgen` (`x_aq_ifu_pcgen`, top.v:460) — 12461 B / 332 lines. Next-PC arbiter.
- `aq_ifu_ctrl` (`x_aq_ifu_ctrl`, top.v:504) — 4144 B / 132 lines. Fetch-enable + cancel hub.
- `aq_ifu_icache` (`x_aq_ifu_icache`, top.v:527) — 51505 B / 1391 lines. Whole ICache: tag/data
  SRAM wrappers, hit judge, refill FSM, prefetch FSM, CP0 invalidate/read-line FSM, AXI (BIU)
  master. Instantiates `aq_ifu_icache_tag_array` (icache.v:690) and
  `aq_ifu_icache_data_array` (icache.v:702).
- `aq_ifu_btb` (`x_aq_ifu_btb`, top.v:601) — 28836 B. L0 BTB (sibling scope) — only its
  interface to pcgen/ctrl is used here (§8).
- `aq_ifu_ipack` (`x_aq_ifu_ipack`, top.v:634) — 21106 B / 500 lines. Packs the ICache's
  32-bit/cycle output into up to three 16-bit halfword slots (`aq_ifu_ipack_entry`,
  5222 B, instantiated x3: entry0/1/2, ipack.v:298-362).
- `aq_ifu_ibuf` (`x_aq_ifu_ibuf`, top.v:682) — 59742 B / 1373 lines. 6-entry × 16-bit
  circular halfword queue (`aq_ifu_ibuf_entry`, 11159 B, x6, ibuf.v:388-644) plus a 2-deep
  "pop" staging pair (`aq_ifu_ibuf_pop_entry`, 8147 B, x2, ibuf.v:1202-1253) that assembles
  the single 32-bit instruction delivered to IDU each cycle.
- `aq_ifu_pred` (`x_aq_ifu_pred`, top.v:730) — 30285 B. BHT/RAS branch predictor (sibling
  scope) — consumes `aq_ifu_pre_decd`-style immediate/branch-type decode internally and the
  `ipack_pred_inst{0,1}` bus (§8); only its pcgen/ipack/ibuf/ctrl-facing ports are read here.
- `aq_ifu_vec` (`x_aq_ifu_vec`, top.v:816) — 8519 B / 291 lines. Boot/reset/warm-up sequencer
  (§7) — **confirmed NOT RVV**: no vector-register, `vsetvli`, `vl`/`vsew`/`vlmul` signal
  anywhere in this file; it is a 4-state FSM (`RESET`,`WARM_UP`,`HALT`,`IDLE`,
  vec.v:157-160) driving reset-invalidate handshake and per-unit `warm_up` pulses.

`aq_ifu_pre_decd.v` (7949 B, 220 lines) is **not instantiated inside `aq_ifu_top.v` at all** —
grep of `top.v` shows no `aq_ifu_pre_decd` instance. It must be instantiated inside
`aq_ifu_pred.v` (sibling scope) directly on the `ipack_pred_inst0/1` bus that `aq_ifu_ipack`
exposes (ipack.v:456-465); this note only characterizes what it computes (§4.2), not where
it lives structurally.

SRAM primitives actually instantiated in the IFU dir: `aq_spsram_256x59` (tag array, §3),
`aq_spsram_2048x32` (data array ×4, §3), and `aq_spsram_1024x16`, which is used **only** by
`aq_ifu_bht_array.v:102` (BHT table, `` `define BHT_16K `` at cpu_cfig.h:135) — out of my
scope, noted here only to close out the "which array uses which SRAM" question the task
asked for.

## 2. Pipeline stages (actual, not forced into C910's IF/IP/IB shape)

There is no separate IF/IP/IB stage split. The real stage boundaries, by register:

1. **PCGEN** (`aq_ifu_pcgen.v`) — combinational priority mux + one flop (`pcgen_ifpc`,
   pcgen.v:100,237-253) holding the fetch VA. Sends `pcgen_icache_va[63:0]` +
   `pcgen_icache_seq_tag[33:0]` (= `pcgen_ifpc[39:6]`, the line tag, pcgen.v:311) to ICache
   every cycle it is granted.
2. **ICache access, cycle A** (`aq_ifu_icache.v`) — `icache_rd_cen = icache_rd_req &&
   !icache_stall` (icache.v:525) drives tag/data SRAM address ports combinationally; also
   issues the (stub) MMU translation request this same cycle.
3. **ICache hit-check, cycle B** — at the clock edge, `icache_rd_vld<=1` and
   `icache_rd_addr<=icache_va` latch (icache.v:749-776, gated by `icache_rd_clk`). In cycle
   B, tag/data SRAM `Q` outputs and the (stubbed, same-cycle) `mmu_ifu_pa` are combinationally
   compared (`icache_way0_hit`/`icache_way1_hit`, icache.v:781-784) and muxed into one 32-bit
   word, `icache_ipack_inst[31:0]` (icache.v:1327-1328). **So the ICache is architecturally a
   2-cycle access (address-issue cycle + tag-compare/output cycle) implemented as ONE module
   with ONE internal flop stage**, not two separate pipe-register stages with independent
   stall control as in C910's IF+IP.
4. **IPACK** (`aq_ifu_ipack.v`) — purely combinational re-slicing of the single 32-bit
   `icache_ipack_inst` into up to three 16-bit "entries" (§4) with no PC or predecode array
   of its own; boundary detection is `inst[1:0]==2'b11` computed live on each halfword
   (ipack.v:373-380), not a precomputed/stored predecode bit as in C910.
5. **IBUF** (`aq_ifu_ibuf.v`) — 6-entry × 16-bit circular register queue (§5) plus the
   2-deep pop-staging pair that emits exactly one 32-bit `ifu_idu_id_inst` per cycle
   (ibuf.v:1354-1356).
6. **IDU boundary** — `ifu_idu_id_inst[31:0]` / `_inst_vld` (top.v ports) is the sole
   instruction-data handoff; there is no second or third instruction slot anywhere in the
   `ifu_idu_*` port list (`aq_ifu_top.v:80-101,219-240`). Confirmed single-issue delivery.

## 3. ICache geometry: 32KB, 2-way, 64B line, 256 sets, tag 59b, VA-indexed/PA-tagged

- Config macro `ICACHE_32K` (cpu_cfig.h:142) sets `I_TAG_INDEX_WIDTH=8`,
  `I_DATA_INDEX_WIDTH=11` (cpu_cfig.h:375-376). Cache-size/index-width table is spelled out
  in-RTL: `icache.v:528-533` ("32KB: tag index: addr[13:6], data index: addr[14:2]").
- **Tag array**: `aq_ifu_icache_tag_array.v` instantiates `aq_spsram_256x59`
  (icache_tag_array.v:111) — confirmed 256 sets × 59-bit row from the SRAM primitive itself
  (`aq_spsram_256x59.v:28,31,34`: `A[7:0]`, `D[58:0]`, `Q[58:0]`). 256 sets × 2 ways × 64B
  = 32768B = 32KB, matching the manual exactly (vs. C910's 512 sets/64KB — confirms the
  size-scaling arithmetic the task asked me to verify, not assume).
- **Tag row format, 59 bits** (icache.v:742-746): `{fifo(1), way1_vld(1), way1_ptag[27:0],
  way0_vld(1), way0_ptag[27:0]}` = 1+1+28+1+28 = 59. `ptag = PA[39:12]` (28 bits) —
  `icache_pa[39:0] = {mmu_ifu_pa[27:0], icache_rd_addr[11:0]}` (icache.v:729), and
  `mmu_ifu_pa` is a 28-bit port (`aq_ifu_top.v:185` `input [27:0] mmu_ifu_pa`). This is the
  **same 59-bit tag-row layout as C910** (1 replacement bit + 2×(valid+28b ptag)), just at
  256 rows instead of 512.
- **Tag write-enable is 3 bits, not 2** (`icache_tag_wen[2:0]`, icache.v:673,
  icache_tag_array.v:34): `icache_tag_bwen_b[58:0] = ~{wen[2] (fifo, 1b), {29{wen[1]}} (way1,
  29b), {29{wen[0]}} (way0, 29b)}` (icache_tag_array.v:98-101). So bit widths are `[2]`=FIFO
  replacement-bit write-enable, `[1]`=way1 (valid+ptag) write-enable, `[0]`=way0. **This
  clarifies the task's "known fact" about `icache_tag_wen[1:0]`**: the 2-way-ness lives in
  bits `[1:0]` exactly as stated, but the port itself is 3 bits wide because bit `[2]` is a
  separate, shared FIFO-pointer write-enable, not a third way. 2-way-set-associativity is
  fully confirmed: `icache_way0_hit`/`icache_way1_hit` (icache.v:781-784),
  `icache_hit_inst = hit1&way1_inst | hit0&way0_inst` (icache.v:805-806).
- **Replacement policy**: single shared FIFO bit per set (`icache_tag_dout[58]`,
  icache.v:742), latched into `icache_refill_fifo` at miss-detect time
  (icache.v:830-836: `icache_refill_fifo <= icache_tag_fifo`). Refill writes the way
  indicated by that latched bit (`refill_tag_wen[1:0] = {icache_refill_fifo,
  !icache_refill_fifo}` inside the `{3{refill_icache_req}}&{...}` term, icache.v:544), and
  on the **first** refill beat (`refill_icache_init`, not the last beat as in C910) also
  flips the stored FIFO bit for next time: `refill_tag_din[58] = refill_icache_init ?
  !refill_icache_fifo : !pf_icache_fifo` (icache.v:547). The full beat-by-beat write
  sequencing (which of the 4 beats actually re-writes the way/valid fields) is intricate
  enough that I did not fully unwind it — treat `icache.v:542-564` as a read-directly span
  before implementing refill.
- **Data array**: `aq_ifu_icache_data_array.v` instantiates `aq_spsram_2048x32` **four
  times** (banks 0-3, data_array.v:245,269,293,317), each 2048 rows × 32 bits
  (`aq_spsram_2048x32.v:28,31,34`: `A[10:0]`). 4 × 2048 × 32b = 262144b = 32768B = 32KB
  total data storage across **both ways combined** — way selection is folded into the bank
  index (`icache_data_idx_high/low`, data_array.v:200-207), not a separate way dimension.
  This 32KB-total-including-both-ways arithmetic is exactly consistent with the tag-array
  sizing above.
- **Data output width per cycle: only 32 bits (one word), not a wide cache-line fetch.**
  `icache_hit_inst[31:0]` (icache.v:805) is the entire per-cycle ICache-to-IPACK bus
  (`icache_ipack_inst[31:0]`, icache.v:1327). Bank select for that one word is
  `icache_pa[3:2]` (icache.v:795-803: `icache_data_32`/`icache_data_10` mux by `pa[2]`, then
  `icache_way{0,1}_inst` mux by `pa[3]`). **This is a materially different fetch bandwidth
  from C910**, which read a full 128-bit (16B) row per cycle across 4 banks in parallel; here
  the 4 banks exist to hold one 64B line's worth of data per way (4 banks × 4B = 16B... 
  actually 4 banks × 4B/bank = 16B per way per "column", ×4 columns via the 11-bit index's
  low bits = 64B/line), but only ONE of those 4B words is read out per cycle — the wide
  layout is for write-side refill bandwidth (128b/beat from `biu_ifu_rdata[127:0]`,
  icache.v:1015), not read-side fetch bandwidth. Fetch bandwidth to IPACK is 4B (2
  halfwords)/cycle, not 16B/cycle.
- **No stored predecode array.** Unlike C910 (`icache_predecd_array0/1`), C906's ICache has
  no third SRAM array for RVC-boundary bits. Boundary detection (`inst[1:0]==2'b11`) is
  computed live per-halfword in IPACK/IBUF (§4), which is only possible because the fetch
  width is already just 1-2 halfwords/cycle — there's no multi-halfword interleaving hazard
  that would require precomputed boundary bits the way C910's 8-halfwords/cycle fetch did.
- **Refill FSM**: `IDLE(000)->REQ(001)->INIT(010)->WFC(011)`, with a separate `WFPA(100)`
  wait-for-physical-address state (icache.v:845-936). 4 beats × 128b via AXI
  (`ifu_biu_arlen=2'b11` when cacheable, icache.v:1363), **not** explicitly critical-word/
  critical-instruction-first by address reordering in this FSM — the beat order is
  whatever the bus returns; `req_cnt` is seeded from `icache_miss_addr[5:4]`
  (icache.v:976, i.e. the requested word's position within the 64B/4-beat line) so the
  **AXI burst starts at the missing word's own beat** (via `WRAP` burst type,
  `ifu_biu_arburst=2'b10` cacheable, icache.v:1365) rather than always beat 0 — this **is**
  the critical-beat-first mechanism the manual refers to, implemented via AXI WRAP-burst
  addressing rather than an internal reorder buffer. Refill data before the tag/valid write
  completes is bypassed straight to IPACK: `icache_bypass_vld = refill_icache_init &&
  !refill_data_abort` (icache.v:1324), `icache_ipack_inst = icache_bypass_vld ?
  icache_bypass_inst : icache_hit_inst` (icache.v:1327-1328).
- **Prefetch**: separate FSM `PF_IDLE->PF_READ->PF_CHK->PF_REQ->PF_WFC0..3`
  (icache.v:1025-1119), next-line-only (`pf_chk_addr = {..., addr[11:6]+1, 6'b0}`,
  icache.v:1131), gated by `cp0_ifu_icache_pref_en`, id `rd_id=1` vs demand `rd_id=0`
  (`ifu_biu_arid = !icache_req`, icache.v:1362).
- **Invalidation**: `IOP_IDLE/WRTE/READ/FLOP` FSM (icache.v:1176-1250) shared between
  `fence.i`/`icache.iall` (`cp0_ifu_icache_inv_req`) and CP0 diagnostic cache-line reads
  (`cp0_ifu_icache_read_req`). `inv_cnt_max=9'hFF` (256, icache.v:1279) confirms 256 sets
  again for the invalidate-all walk.

## 4. Fetch/decode width asymmetry (fetch up to 2/cycle, single-issue delivery)

### 4.1 ICache -> IPACK: one 32-bit word (2 halfwords) per cycle
`icache_ipack_inst[31:0]` is the entire ICache output bus (§3). IPACK treats it as exactly
two 16-bit halfwords: `entry1_upd_inst = icache_ipack_inst[15:0]`,
`entry2_upd_inst = icache_ipack_inst[31:16]` (ipack.v:277-278).

### 4.2 IPACK: 3 halfword entries, RVC-boundary carry logic
`aq_ifu_ipack.v` holds 3 flopped 16-bit entries (`entry0/1/2`, via `aq_ifu_ipack_entry.v`
x3). `entry0` is a **carry register**, not a new fetch: `entry0_upd_inst =
entry2_inst[15:0]` (ipack.v:276) — it exists purely to hold the leftover low halfword of a
32-bit instruction that straddled the previous cycle's `entry2`/this cycle's `entry1`
boundary. Boundary/length classification is all combinational on the entries themselves
(ipack.v:373-380):
- `h0_vld = entry0_vld && entry0_inst[1:0]==2'b11` (carry is low half of a 32b inst)
- `h1_16bit_vld` / `h1_32bit_vld` = entry1 is a complete 16b inst / starts a 32b inst
- `h2_16bit_vld` / `h2_32bit_vld` = same for entry2

Retire-count flags to IBUF: `ipack_ibuf_inst_one` (exactly one 16-bit inst this cycle,
ipack.v:399-402), `ipack_ibuf_inst_two` (two 16-bit insts, ipack.v:480-482),
`ipack_ibuf_inst_all` (a completed straddling 32b inst **plus** a trailing 16b inst,
ipack.v:412-414/483) — i.e. IPACK can present **up to 2 instructions' worth of halfwords**
per cycle to IBUF, matching the manual's "fetch up to 2 instructions/cycle", bounded by the
2-halfword/cycle ICache supply.

`aq_ifu_pre_decd.v` (fed by `ipack_pred_inst0[31:0]`/`ipack_pred_inst1[15:0]`, the same
first/second-instruction view IPACK exposes to the predictor, ipack.v:456-465) decodes
branch/jump/jalr/ret validity + immediates for **up to 2 instructions per cycle**
(`pred_br_vld0/1`, `pred_jmp_vld0/1`, `pred_ret_vld0/1`, pre_decd.v:205-216) — this is the
predictor's view of the same 2-wide fetch group, done combinationally on raw bits with no
separate stored array (§3).

### 4.3 IBUF: 6-entry halfword queue, push up to 3/pop up to 2 halfwords, but exactly ONE 32-bit instruction to IDU per cycle
`parameter ENTRY_NUM = 6` (ibuf.v:384): a 6-entry × 16-bit **register-based circular
buffer** (not SRAM — each `aq_ifu_ibuf_entry` is a flop, ibuf_entry.v). One-hot push
pointers `push0/push1/push2` (create up to 3 entries/cycle: the carry + 2 new halfwords,
ibuf.v:1013,1033,1058) and one-hot pop pointers `pop0/pop1` (retire up to 2 halfwords/cycle
i.e. one 32-bit instruction, ibuf.v:660-713). Occupancy: `ibuf_vld_num[2:0]` = popcount of
the 6 entry-valid bits (ibuf.v:1131-1136); `ibuf_full` at count==6 (ibuf.v:1138).

**The single-issue boundary is explicit in the port list**: `ifu_idu_id_inst_vld` and
`ifu_idu_id_inst[31:0]` (ibuf.v:1354-1356) are the *only* instruction-valid/data signals to
IDU — there is no `ifu_idu_id_inst1`, no per-slot valid array, unlike C910's
`ifu_idu_id_ib_inst{0,1,2}`. The queue's 6-halfword/3-instruction-ish depth and up-to-3-push/
up-to-2-pop width is entirely an IFU-internal elasticity buffer; IDU only ever sees one
packaged instruction (16 or 32-bit, reassembled from `pop_entry0_inst`+`pop_entry1_inst`,
ibuf.v:1325) per cycle. **This is the concrete RTL mechanism behind the "IFU fetches 2,
IDU decodes 1" asymmetry** the task asked me to locate.

## 5. IFU boundary stall/flush signal topology (directly informs the M1 stall network)

### 5.1 The stall hub, `aq_ifu_ctrl.v` (132 lines total, quoted almost in full)
Three outputs, all pure combinational OR/AND of inputs, no state:
- `ctrl_inst_fetch = ibuf_ctrl_inst_fetch && !(cp0_ifu_in_lpmd||cp0_ifu_lpmd_req) &&
  !rtu_ifu_dbg_mask && !vec_ctrl_reset_mask` (ctrl.v:93-96) -> `ctrl_icache_req_vld`,
  `ctrl_btb_inst_fetch` (ctrl.v:117,128).
- `ctrl_if_stall = pred_ctrl_stall || icache_ctrl_stall` (ctrl.v:101) -> **only**
  `ctrl_btb_stall` (ctrl.v:127). Note: the analogous `ctrl_pcgen_stall` assignment is
  **commented out** (ctrl.v:124) — `aq_ifu_ctrl.v` does **not** gate PCGEN at all; PCGEN's
  own backpressure comes directly from ICache's `icache_pcgen_grant`/`_gate` (§5.3), bypassing
  this hub entirely.
- `ctrl_if_cancel = rtu_ifu_flush_fe || pcgen_ctrl_chgflw_vld` (ctrl.v:106) -> fans out to
  `ctrl_icache_abort`, `ctrl_ipack_cancel`, `ctrl_btb_chgflw_vld` (ctrl.v:118,121,129) — **3
  destinations**, much smaller than C910's 10-signal cancel fan-out (`ifctrl_cancel`/
  `ipctrl_cancel`/etc.) because there are only 3 downstream modules that need a "cancel"
  concept here (ICache, IPACK, BTB). IBUF gets its own separate flush input directly
  (`pcgen_ibuf_chgflw_vld`, top.v:716, and `rtu_ifu_flush_fe` again directly, top.v:722) —
  **not** routed through `ctrl_if_cancel`.
- `ctrl_ibuf_pop_en = !idu_ifu_id_stall` (ctrl.v:113) — **the single point where the
  IDU->IFU stall signal is consumed**, directly gating `pop_entry0/1_retire_en`
  (ibuf.v:1318-1319), i.e. the last-mile pointer-advance on the 32-bit-instruction handoff to
  IDU.

### 5.2 Confirmed: `idu_ifu_id_stall` is the only signal crossing the IFU<->IDU boundary
`aq_ifu_top.v` port list: exactly one input from IDU, `idu_ifu_id_stall` (top.v:167,319), and
it fans out to exactly two consumers inside the IFU: `aq_ifu_ctrl` (-> `ctrl_ibuf_pop_en`,
§5.1) and directly into `aq_ifu_ibuf` itself (top.v:698, used again inside `ibuf.v` at
create-time arbitration, e.g. ibuf.v:1265-1310, to decide whether newly-arriving IPACK
halfwords bypass straight into the pop-staging registers or must queue). There is no
`ifu_idu_*_stall` signal going the other direction in the port list (`aq_ifu_top.v:80-101`)
— backpressure is one-directional (IDU tells IFU to hold); IFU communicates "nothing to
send" purely via `ifu_idu_id_inst_vld=0`. This is the exact point-to-point boundary crossing
the design doc's §5.2/§6.2 rule 5 describes, confirmed structurally in RTL.

### 5.3 Everything else is direct point-to-point wiring (no shared bus/hub)
- PCGEN <-> ICache: `icache_pcgen_grant` / `icache_pcgen_grant_gate` (icache.v:1346-1347,
  pcgen.v used at :243-292) directly gate whether `pcgen_ifpc` advances. `ref_rdy = ref_fsm_idle
  && pf_fsm_idle && !icache_miss_req && !icache_stall` (icache.v:939) is the ultimate root of
  this grant — i.e. PCGEN stalls whenever ICache is mid-refill/mid-prefetch/mid-miss/
  mid-invalidate, entirely without going through `aq_ifu_ctrl`.
- IPACK <-> IBUF: `ibuf_ipack_stall` (= `ibuf_stall`, ibuf.v:1340/1194) tells IPACK to hold
  its entries; IPACK's `ipack_pcgen_reissue = ibuf_ipack_stall && icache_inst_vld`
  (ipack.v:491) propagates that stall **all the way back to PCGEN** as a same-PC refetch
  request (pcgen.v:245-246,267-268: `ipack_pcgen_reissue && icache_pcgen_inst_vld` reissues
  `icache_pcgen_addr`) — this is the point-to-point chain analogous to what the design doc
  calls out for the IDU<->IFU link, but internal to IFU's own sub-stages.
- Pred <-> IPACK/IBUF: `pred_ipack_mask`, `pred_ipack_delay_stall`, `pred_ipack_ret_stall`
  (consumed at ipack.v:253,256,262,265,268,293,380,401,414 etc.) and `pred_ibuf_chgflw_vld0`/
  `pred_ibuf_br_taken{0,1}` (consumed in ibuf.v create-side logic) are all direct pred->ipack
  or pred->ibuf wires, not routed through `aq_ifu_ctrl`.
- Flush sources, by destination:
  - ICache abort: `ctrl_icache_abort` (`=ctrl_if_cancel`, §5.1) OR `cp0_ifu_lpmd_req`
    (icache.v:725-726, low-power-mode).
  - IPACK cancel: `ctrl_ipack_cancel` (`=ctrl_if_cancel`) plus its own `pred_ipack_mask` gate
    (ipack.v:253).
  - IBUF flush: `rtu_ifu_flush_fe || pcgen_ibuf_chgflw_vld` (ibuf.v:657, note: NOT
    `ctrl_if_cancel` — IBUF is wired directly to RTU's flush and to PCGEN's own
    `pcgen_ibuf_chgflw_vld = rtu_ifu_chgflw_vld || iu_ifu_tar_pc_vld` (pcgen.v:304-305),
    skipping the ctrl hub).
  - IPACK's internal entry buffer flush: `ipack_buf_flush = rtu_ifu_flush_fe ||
    iu_ifu_tar_pc_vld || rtu_ifu_chgflw_vld` (ipack.v:251-252) — again wired directly to RTU/
    IU, not through `ctrl_if_cancel`.

### 5.4 Full external port inventory relevant to stall/flush (`aq_ifu_top.v`)
- From IDU: `idu_ifu_id_stall` (top.v:167) — only backpressure signal, §5.2.
- From RTU: `rtu_ifu_chgflw_vld`/`rtu_ifu_chgflw_pc[39:0]` (redirect, e.g. exception/trap
  target), `rtu_ifu_flush_fe` (front-end flush), `rtu_ifu_dbg_mask` (halts fetch, ctrl.v:95),
  `rtu_yy_xx_dbgon` (debug-mode broadcast, gates IBUF debug-inst injection).
  (top.v:123-126,189-193)
  Output to RTU-side/IU: `ifu_rtu_reset_halt_req` (vec.v:279, boot-halt request),
  `ifu_rtu_warm_up` + 5 other `ifu_*_warm_up` pulses (vec.v, §7).
- From IU/BJU: `iu_ifu_tar_pc_vld`(+`_gate`)/`iu_ifu_tar_pc[63:0]` (branch-resolution
  redirect, priority above BTB/pred in pcgen, §8), `iu_ifu_pc_mispred`(+`_gate`),
  `iu_ifu_bht_mispred`(+`_gate`), `iu_ifu_br_vld`/`iu_ifu_link_vld`/`iu_ifu_ret_vld`
  (+`_gate` each) — all predictor-update/mispredict signals (sibling scope for the tables,
  but the mispredict redirect path through pcgen is in scope, §8).
  Output to IU: `ifu_iu_chgflw_vld`/`ifu_iu_chgflw_pc[39:0]` (pcgen.v:325-326, forwards RTU's
  redirect onward to IU), `ifu_iu_ex1_pc_pred[39:0]` (from pred, predicted PC for IU's
  branch-resolution compare).
- No stall signal to IDU (§5.2) and no stall signal to RTU/IU (redirects flow one-way from
  them into IFU).

## 6. Boot / vector sequencer (`aq_ifu_vec.v`, 291 lines) — confirmed NOT RVV

4-state FSM `RESET(01)->WARM_UP(11)->{HALT(10)|IDLE(00)}` (vec.v:157-160,182-204).
`RESET`: waits for `cp0_ifu_rst_inv_done` (cache-invalidate-done from CP0) before leaving
reset; asserts `vec_pcgen_rst_vld` (= `vec_rst_inv_req`, a one-shot pulse,
vec.v:230,260) which forces `pcgen_ifpc <= cp0_xx_mrvbr[39:0]` in PCGEN (pcgen.v:239-240) —
i.e. **this module is what loads the reset vector into PCGEN**, matching C910's analogous
"vector" module pattern exactly. `WARM_UP`: 3-bit counter to `warm_up_cnt==3'b111`
(vec.v:236-243) then either `HALT` (if `dtu_ifu_halt_on_reset`) or `IDLE`; drives one-shot
`ifu_{idu,vidu,iu,vpu,lsu,cp0,rtu}_warm_up` pulses (vec.v:282-288) to every other unit.
`vec_ctrl_reset_mask = vec_sm_reset || vec_sm_warm_up` (vec.v:264) blocks fetch via
`aq_ifu_ctrl`'s `ctrl_inst_fetch` term (ctrl.v:96) during boot. **No exception-vector
redirect logic lives here** (unlike C910's vector module, which also handled the trap-vector
PCLOAD state) — exception PC redirects in C906 instead come from RTU's
`rtu_ifu_chgflw_pc`/`_vld` directly into PCGEN's delayed-change-flow mux (pcgen.v:212-219),
so `aq_ifu_vec.v` is reset/warm-up-only, an even narrower scope than C910's equivalent.
Confirmed absence of RVV: no `vl`/`vsew`/`vlmul`/`vsetvli`/vector-register token anywhere in
this file (grepped).

## 7. PCGEN redirect priority (`aq_ifu_pcgen.v:237-253`, the `pcgen_ifpc` update mux)

In priority order (first matching `else if` wins), from the always-block at pcgen.v:237-253:
1. `vec_pcgen_rst_vld` — boot reset vector (§6), highest.
2. `pcgen_delay_chgflw_vld` = `rtu_ifu_chgflw_vld || iu_ifu_tar_pc_vld ||
   pred_pcgen_chgflw_vld` (pcgen.v:212-214) — RTU flush/exception, OR IU/BJU
   branch-resolution mispredict, OR the predictor's own "final" (BHT+RAS-resolved) redirect;
   RTU wins ties over the other two via `pcgen_br_chgflw_vld = (iu..||pred..) &&
   !rtu_ifu_chgflw_vld` (pcgen.v:208-209).
3. `pcgen_chgflw_cur && !icache_pcgen_grant` = a same-cycle predictor correction
   (`pred_pcgen_curflw_vld`, RAS-return or immediate-branch-taken redirect, §8) held because
   ICache didn't grant this cycle.
4. `ipack_pcgen_reissue && icache_pcgen_inst_vld` — IBUF-stall-triggered same-PC refetch
   (§5.3).
5. `pcgen_chgflw_btb && !icache_pcgen_grant` — L0 BTB target, held because ICache didn't
   grant.
6. `icache_pcgen_grant` — normal sequential increment, `pcgen_ifpc_inc = {fetch_pc[63:2],
   2'b0} + 4` (pcgen.v:277) i.e. **+4 bytes/cycle nominal advance**, one 32-bit-word request
   per grant (consistent with the 1-word/cycle ICache read port, §3).
7. else hold (`pcgen_ifpc <= pcgen_ifpc`).

Notably, `pcgen_chgflw_cur`/`pcgen_chgflw_btb` (items 3 and 5) only take effect **when ICache
did NOT grant** — when it did grant, the increment path (item 6) always wins even in the
same cycle a BTB/pred redirect is asserted, because `pcgen_fetch_pc` (pcgen.v:281-283) is
already muxed to the redirect target *before* the increment adder consumes it
(`pcgen_ifpc_inc` is computed from `pcgen_fetch_pc`, not from `pcgen_ifpc` directly) — i.e.
redirects are folded into the SAME increment/grant cycle rather than needing an extra "hold"
cycle, unlike C910's separate hold states. This single-cycle-redirect-into-increment
interaction (pcgen.v:277-283) is subtle enough to flag as a read-directly span before
reimplementing.

## 8. Branch-prediction <-> pipeline-control interface (internals are sibling scope)

Only the consumption-facing signals of `aq_ifu_pred.v` (pred.v ports + pred.v:766-783,
the "Rename for Output" section):
- `pred_ctrl_stall = pred_id_stall` (pred.v:766) — an internal pred-pipeline stall,
  routed only to `ctrl_btb_stall` (§5.1), not to PCGEN/ICache/IPACK.
- `pred_pcgen_chgflw_vld/_pc = pred_chgflw_fin` / `pred_chgflw_fin_tar` (pred.v:769,773) —
  the "final", presumably BHT+RAS-resolved, delayed redirect (PCGEN priority level 2, §7).
- `pred_pcgen_curflw_vld/_pc = pred_curflw` / muxed on `pred_ras_ret_chgflw` (pred.v:774,776)
  — a same-cycle correction (PCGEN priority level 3, §7), sourced from either a RAS return
  target or (implicitly, by exclusion) an immediate branch/BTB-adjacent correction.
- `pred_ipack_ret_stall`/`pred_ipack_delay_stall`/`pred_ipack_mask` = `pred_ret_stall`/
  `pred_delay_br_raw`/`pred_delay_br_raw` (pred.v:781-783) — gate IPACK's entry-creation and
  retire logic (§4.2/§5.1) when the predictor needs a cycle to resolve a branch before
  IPACK is allowed to commit instructions past it.
- `pred_ibuf_chgflw_vld0`, `pred_ibuf_br_taken{0,1}[1:0]`, `pred_ibuf_halt_info{0,1}` feed
  IBUF's entry-creation logic directly (ibuf.v create-side assigns, §4.3) — per-instruction
  taken/not-taken prediction bits riding alongside each halfword into the queue, ultimately
  surfacing as `ifu_idu_id_bht_pred[1:0]` (ibuf.v:1357-1358) to IDU.
- Consumes from IPACK: `ipack_pred_inst0[31:0]`/`_vld`, `ipack_pred_inst1[15:0]`/`_vld`,
  `ipack_pred_h0_create`/`_h0_vld`, `ipack_pred_unalign` (ipack.v:456-472) — the up-to-2-
  instruction view described in §4.2, plus straddle-state flags.

I did not open `aq_ifu_bht.v`/`aq_ifu_bht_array.v`/`aq_ifu_btb.v`/`aq_ifu_btb_entry.v`/
`aq_ifu_ras.v`/`aq_ifu_ras_entry.v` beyond what's cited above (sibling's scope) except to
confirm the `aq_spsram_1024x16` -> BHT-array binding in §1.

## 9. Read-directly spans (too intricate to fully resolve at this pass)

- Tag/valid write sequencing across the 4 refill beats and the FIFO-bit flip timing:
  `icache.v:542-564,830-837,1003-1016`.
  Prefetch tag/data write aliasing with demand refill: `icache.v:1148-1168`.
- Direct-select / low-power tag-hit buffer bypass path (`direct_sel`, `buf_hit_tag`,
  `cen_mask_vld`): `icache.v:566-664` — a hit-buffer that lets a repeated access to the same
  line skip the tag/data SRAM read entirely; only active when `cp0_ifu_iwpe` is set
  (way-prediction enable — **should be 0 for M1**, mirroring C910's `cp0_ifu_iwpe=0` M1
  setting, since the C906 encoding of this bit gates the exact same kind of low-power
  hit-buffer bypass, not way prediction per se here — confirm against the C906 manual/CP0
  spec before wiring it up, I have not independently verified the bit's semantics beyond
  what `icache.v` shows).
- Invalidate-all vs. per-line-by-VA vs. per-line-by-PA FSM interactions and the alias-set
  walk: `icache.v:1170-1301`.
- IPACK's create/retire boolean conditions (`entry{0,1,2}_create_en`, `ipack_acc_err{0,1,2}`,
  `ipack_pgflt{0,1,2}`): `ipack.v:258-293,427-450` — dense combinational logic covering every
  straddle/alignment case; I traced the *shape* of the mechanism (§4.2) but not every case.
  IBUF's mirrored create-side arbitration between "IPACK bypass" and "queued" paths:
  `ibuf.v:1259-1313`.
- PCGEN's redirect-into-same-cycle-increment interaction: `pcgen.v:277-283` (flagged in §7).

## 10. Open items / not independently verified

- The exact semantics of `cp0_ifu_iwpe` in C906 (way-prediction-enable per the task's framing,
  vs. what looks in RTL like a tag-hit-buffer/low-power bypass enable, `icache.v:596-602`) —
  worth double-checking against the C906 user manual before assuming it maps 1:1 to C910's
  way-predictor bit.
- I did not verify the `TDT_HINFO_WIDTH` numeric value (halt-info field width riding through
  IPACK/IBUF entries, `` `define `` chain rooted in `../../dtu/rtl/aq_dtu_cfig.h:27`,
  `` `TDT_HINFO_TRIGGER + 1 ``) — debug/trigger-related, likely droppable for M1 same as
  C910's HAD/debug path, but I have not traced `TDT_HINFO_TRIGGER`'s value.
- `icache_data_idx`'s bit-slice reconciliation between the in-RTL comment
  ("32KB: ... data index: addr[14:2]", 13 bits) and the actual `[13:0]` port width /
  `I_DATA_INDEX_WIDTH=11` used inside the data-array module is not fully reconciled bit-for-
  bit in this pass — the aggregate 32KB-capacity arithmetic is solid (§3) and I'm confident
  in that, but the exact index-bit-to-bank mapping deserves a closer read
  (`icache_data_array.v:200-241`) before implementation.
