# C906 LSU Base Path + CP0/CSR Extraction Notes (M2 working material)

Sources: `refs/openc906/C906_RTL_FACTORY/gen_rtl/lsu/rtl/` (33 files, 24,030 lines, of which
`aq_vlsu_*`/`aq_vlsu_*_entry` = 8 files / ~5,132 lines are the **RVV vector load/store engine,
out of scope** — leaving ~18,900 lines of scalar LSU+DCache), `refs/openc906/.../mmu/rtl/`
(17 files, real uTLB/JTLB/PTW — read only far enough to answer the TLB-scope question, not
implemented in depth) and `refs/openc906/.../cp0/rtl/` (15 files, 9,989 lines). Extracted
2026-08-20. All file:line refs are relative to `lsu/rtl/`, `mmu/rtl/`, or `cp0/rtl/` as named.

Global: `` `define PA_WIDTH 40 `` (cpu_cfig.h:440) — same 40-bit byte-address PA as the IFU
note. LSU is built non-vector/`DPLEN_64` (`aq_lsu_cfig.h:34-46`): `LSU_DATAW=64`,
`LSU_BYTEW=8`. All load/store data paths in this note are the **64-bit-wide, DPLEN_64**
variant; a `DPLEN_128` config exists in the same header (dead code for this build).

# (A) LSU BASE PATH + DCACHE

## A1. Module graph (`aq_lsu_top.v`, instantiation order at top.v:1102-2258)

`aq_lsu_ag` (1797L, address-gen) -> `aq_lsu_dc` (2648L, tag compare + DC/DA stages) ->
`aq_lsu_lfb` (2000L + `aq_lsu_lfb_entry` 692L x8, line-fill/miss buffer) -> `aq_lsu_stb`
(1689L + `aq_lsu_stb_entry` 732L x4, store buffer) -> `aq_lsu_rdl` (778L, the module that
actually **writes** refill/store/dirty/invalidate data into the DCache SRAMs — a small FSM,
not a load-side "read/align" stage despite the name) -> `aq_lsu_icc` (750L, **I-cache
coherence**: `fence.i`/self-modifying-code snoop into ICache, not a DTLB thing) -> `aq_lsu_vb`
(615L, victim buffer for dirty-evict writeback) -> `aq_dcache_top`/arrays (see A4) ->
`aq_lsu_pfb_top`+`aq_lsu_pfb` (683L+595L, HW next-line prefetch — **M3-deferred**, matches the
task's "HW prefetch deferred to M3") -> `aq_lsu_amr` (367L, **write-Allocate-cancel-on-Miss
Region** policy for scattered store misses — NOT atomics, despite the name's similarity to
"AMO"; see A6) -> `aq_lsu_arb` (669L, cache-port arbiter) -> `aq_lsu_mcic` (284L, CP0
diagnostic cache-line read/invalidate access, "M-mode Cache Info Channel"-ish) -> `aq_lsu_lm`
(167L, **LR/SC reservation monitor**, see A6) -> `aq_lsu_amo_alu` (327L, AMO ALU) ->
`aq_lsu_dtif` (565L, **Debug/Trigger Unit interface** — hardware watchpoints on ld/st
addresses, ports named `dtu_*`/`*_dtu_*`; nothing to do with the DTLB, droppable for M2 like
the IFU note's HAD/debug path).

M2-core vs M3-deferred (by the task's framing): AG, DC, DCache arrays/arb, STB
(4-entry, no MSHR-scale buffering), LM, AMO_ALU are needed for a **blocking** base path.
LFB (8-entry, `aq_lsu_lfb.v:666 parameter DEPTH=8`) is the miss-fill engine; a full
non-blocking implementation is M3 scope, but **some minimal single-outstanding-miss stand-in
is unavoidable even for M2** (a load miss must eventually get its line back) — same
conclusion the rv12 C910 note reached for RB+LFB. PFB (prefetch) is cleanly M3. ICC/VB/MCIC
are needed only once fence.i-vs-dcache-coherence and dirty-evict are exercised; can likely be
stubbed (write-through-only, no eviction needed if the DRAM aperture is small enough to never
evict) for the very first M2 cut, but flag as a real correctness risk if test programs write
enough distinct cache lines to force replacement.

## A2. Pipeline stages: AG -> DC -> DA (3 cycles, cache hit, no arbitration stall)

- **AG**: `ag_addr[63:0] = ({64{ag_pipe_base_sel}} & ag_pipe_src0_data[63:0]) +
  ag_pipe_addr_offset[63:0]` (ag.v:1248) — **one 64-bit adder, fully combinational, no
  separate AGU pipe stage**; base+immediate is computed in the same cycle AG issues the
  uTLB request and the DCache tag/data SRAM read. AG also does misalign detection
  combinationally the same cycle (A5).
- Same cycle, AG issues a real MMU/uTLB request: `lsu_mmu_va[51:0] = ag_pipe_addr[63:12]`,
  `lsu_mmu_va_vld`, `lsu_mmu_priv_mode`, `lsu_mmu_st_inst` (ag.v:1564-1570) — see A7 for what
  answers this.
- **DC**: a 4-state FSM `IDLE(00)->DCS(01)->{FRZ(10)|REPLY(11)}` (dc.v:1349-1416).
  `dc_req_vld = (cur_state==DCS)` is the cycle the tag/data SRAM outputs are actually compared
  (A4); on a resolved hit with the writeback port free, `DCS->REPLY` next cycle; if the
  writeback port is busy, `DCS->FRZ->REPLY` (dc.v:1378-1393) — i.e. FRZ is a pure arbitration
  stall, not part of the nominal hit latency.
- **DA**: a registered stage clocked by `da_clk` (dc.v:2339-2419) that does byte-rotate +
  sign/zero-extend (A5) and asserts `lsu_rtu_wb_vld = da_inst_vld & !da_vfls & !da_expt_vld`
  (dc.v:2417-2419) to RTU/IDU.
- **Net: AG(cyc0, addr+SRAM-req issued) -> DC(cyc1, tag compare resolves hit/miss) ->
  DA(cyc2, data aligned + writeback)** = 3 cycles address-gen-to-data-return for a hit, one
  extra cycle (FRZ) if the DA/writeback port collides with another requester (e.g. an LFB
  refill completing the same cycle).

## A3. DCache geometry — 32KB, 4-way, 64B line, 128 sets, VIPT-with-alias, PIPT tag

Config: `` `define DCACHE_32K `` (cpu_cfig.h:150) selects `D_TAG_INDEX_WIDTH=6`,
`D_TAG_TAG_WIDTH=28`, `D_DATA_INDEX_WIDTH=10` (cpu_cfig.h:400-403).

- **Tag array**: two instances of `aq_dcache_tag_array` (`x_aq_dcache_tag_array_bank0/1`,
  dcache_top.v:150-162,208-220), each internally two `aq_spsram_64x58` (64 rows, `A=tag_idx
  [11:6]`, tag_array.v:150-182). Each 58-bit row packs **two way-slots** of `{valid(1),
  tag(28)}` (tag_array.v:79-93,143-147), so each outer bank holds 4 way-slots -> **8 total
  way-slots** across bank0 (ways 0-3) + bank1 (ways 4-7): `way0..3_tag = dcache_tag_dout_bank0
  [...]`, `way4..7_tag = dcache_tag_dout_bank1[...]` (dc.v:1175-1191). Tag compare:
  `wayN_tag_hit = (wayN_tag==dcache_acc_tag) & wayN_tag_vld & dcache_acc_ca`, tag =
  `dc_pa[39:12]` (28b, matches `D_TAG_TAG_WIDTH`) (dc.v:1195-1207) — **PIPT tag**.
- **Which 4-way group is "yours" is picked by `dc_virt_idx[0]`** = `addr[12]`
  (`ag_ptw_virt_idx[1:0]=mcic_dc_req?mcic_dc_addr[13:12]:ag_dc_virt_idx[1:0]`, dc.v:1336) —
  a bit **outside the 12-bit page offset**, i.e. genuinely virtual until translation
  resolves it: `dc_tag_hit[3:0] = dc_virt_idx[0] ? dc_hit_way[7:4] : dc_hit_way[3:0]`
  (dc.v:1213). Both bank0 and bank1 are read every cycle at the **same** low-6-bit
  `tag_idx[11:6]`; the *other* group is checked as a **VIPT synonym/alias detector**:
  `dc_alias_way`/`dc_alias_hit_raw = |(dc_hit_way_group[1:0] & ~(1<<virt_idx[0]))`
  (dc.v:1220-1224) catches the case where the same physical line got cached under the
  *other* value of VA[12] by an earlier access with a different virtual alias. `dc_alias_hit`
  is OR'd into "present" for allocate-suppression (`dc_pfb_ld_miss=!(dc_cache_hit|
  dc_alias_hit)`, dc.v:1892; similarly dc.v:1811,1924) so a synonym doesn't trigger a
  spurious second fill, but does not by itself resolve/merge the duplicate copy — I did not
  fully trace what happens if BOTH copies later diverge (a real correctness question for
  M4+ if VA aliasing is ever exercised); **read-directly before wiring this up**: dc.v
  :1195-1270, rdl.v alias-consumer at rdl.v:645-656.
- **Data array**: four `aq_dcache_data_array` = `aq_spsram_1024x64` (1024 rows x 64b,
  dcache_top.v:349-446), `way0..3 = dcache_data_dout_bank0..3` directly (dc.v:1170-1173).
  Row address = `{virt_idx[0], pa[11:3]}` (`dc_arb_data_idx[13:0]={virt_idx[1:0],pa[11:0]}`,
  dc.v:1634, only bits `[12:3]` of that bus reach the SRAM's `A` port per
  dcache_data_array.v:103) — i.e. the data array does **not** replicate storage for the
  alias case the way tag does; it speculatively fetches from your **own** VA[12] group
  and falls back (extra cycle via the FRZ/miss-retry path) if the alias group turns out to
  be the real hit. `data_idx[5:3]` selects the doubleword within the line (=64B/8dw line).
- **Dirty/replacement array**: `aq_spsram_128x8` (dcache_dirty_array.v:91, `A=dirty_idx
  [12:6]`, **128 rows** — independently confirms 128 sets), 8 bits/row (per-way
  dirty/replacement-state bits, exact per-way breakdown not fully unwound).
- **Net geometry: 32KB total = 128 sets x 4-way x 64B**, `PA[39:12]` tag, index =
  `{VA[12] (alias bit), PA[11:6]}` (7 bits), VIPT with the alias-check above. **This is a
  materially different associativity from the ICache's 2-way** (M1 finding) despite both
  being 32KB in this build — confirmed independently by public XuanTie C906 documentation
  describing the L1 D$ as "VIPT, four-way set-associative" vs. the I$'s two-way (see Sources
  below) — the task's instruction not to assume DCache mirrors ICache turned out to matter.
- Write policy: write-back (implied by dirty array + VB eviction path); write-allocate is
  policy-gated by `aq_lsu_amr` (A1) which can cancel allocation for a store miss under a
  configurable heuristic (`cp0_lsu_amr[1:0]`, amr.v:39, plus `cp0_lsu_dcache_en` gate).

## A4. Store buffer / store-to-load forwarding

`aq_lsu_stb.v`, **`parameter DEPTH = 4`** (stb.v:528) — 4-entry store buffer, each entry a
full `aq_lsu_stb_entry` (732L) instance carrying `{pa, data, bytes_vld[7:0], way, alias_idx,
attr, lock, ...}` (create-side signal list, stb.v:389-412).

- **Load forwarding is byte-granular, not line-granular**: `dc_hit_stb_bytes =
  |(dc_xx_bytes_vld[7:0] & stb_entry_bytes_vld[7:0])` (stb_entry.v:612) checks the incoming
  load's byte mask against each STB entry's valid-byte mask (after an address/index match
  elsewhere in the entry, `dc_hit_stb_full`); `stb_dc_ld_fwd_vld = |(dc_hit_stb_full[3:0])`
  (stb.v:590), `stb_dc_fwd_vld = |stb_entry_fwd_vld[3:0]` (stb.v:604). A load that partially
  overlaps an outstanding store's bytes gets a real forward path (not just a stall-until-
  drain), though I did not trace the exact partial-overlap merge logic byte-by-byte — flag
  `aq_lsu_stb_entry.v` around line 612 as read-directly before implementing the merge case.
- **No RTU-commit gate found anywhere in the scalar LSU.** I grepped the entire
  `lsu/rtl/*.v` (excluding vlsu) for `rtu_yy_xx_commit`/`rtu_lsu_commit` and got zero hits.
  This is a genuine, load-bearing difference from C910: the rv12 note's central M2 warning
  for C910 was "stores never write the DCache from the pipe... write happens only on
  `rtu_yy_xx_commitN && iid match`" because C910 is out-of-order and stores must sit in the
  SQ until the OoO core actually retires them. **C906 is single-issue in-order**, so by the
  time a store reaches DC/STB there is nothing older left to roll back except the store's
  own fault (checked via `dc_expt_vld` before the STB entry is even created) — the STB's own
  age-ordered create/drain FSM (stb.v, driven by `stb_create_ptr`/`stb_rdl_req`/
  `rdl_stb_grant`) is the only ordering mechanism, and `aq_lsu_rdl.v` is what actually issues
  the write into `aq_dcache_top`'s data/dirty arrays once STB grants it (rdl.v:330-341,
  619-639 `rdl_stb_cmplt`/`rdl_dc_sel`). **This means M2 can very likely skip building any
  RTU-commit-gated store-write mechanism at all** for a single-issue in-order LSU — a real
  simplification opportunity vs. what rv12 had to build for C910, but flagging it as a
  design decision (not yet decided) since it depends on rv906's own pipeline actually being
  strictly in-order end-to-end with no store speculation introduced elsewhere (e.g. branch
  misprediction squash timing needs to still precede any STB-create for a store past a
  mispredicted branch — I did not check the squash-vs-STB-create interlock, only that no
  RTU-commit signal exists).

## A5. Byte/halfword/word/doubleword handling + sign-extension

Decoded control (`SIGN`, `size[1:0]` where `BYTE=00,HALF=01,WORD=10,DWORD=11`) is produced by
AG/IDU and threaded through as `ag_ptw_func[SIGN]` -> `dc_sign_ext <= ag_ptw_func[SIGN]`
(dc.v:1445) and `dc_size[1:0]`.

- **Rotate**: `data_align[63:0]` = byte-granular right-rotate of the 64-bit doubleword by
  `data_shift[2:0]` (0-7 bytes), an 8-way `casez` (dc.v:2065-2081). `dc_rdata_shift[3:0] =
  dc_sc_inst ? 4'b0 : dc_data_shift[3:0]` (dc.v:2063) — SC skips the rotate since it returns
  a fixed success/fail code, not memory data.
- **Sign/zero-extend**: `case({sign_ext,size[1:0]})` — BYTE/HALF/WORD each have an explicit
  zero-extend arm (`sign_ext=0`, for LBU/LHU/LWU) and sign-extend arm (`sign_ext=1`, for
  LB/LH/LW); DWORD falls to `default` (pass-through, no extension needed) (dc.v:2083-2098).
  This is the actual LB/LBU/LH/LHU/LW/LWU/LD/(LWU as WORD+zero) implementation — clean,
  single case statement, no separate rot_data-style helper module (unlike C910's dedicated
  `rot_data.v` per the rv12 note; here it's inline in `aq_lsu_dc.v`).
  There are also `data_align_vls_ext`/`data_align_fls_ext` variants for vector-load-strided
  and FP-load element extension (dc.v:2103-2145) — vlsu/FP scope, not read further.
- Store-side byte placement (the mirror-image left-rotate + byte-write-enable mask
  construction for `bytes_vld[7:0]`) lives in `aq_lsu_ag.v`'s store-address-generation path
  and `aq_lsu_stb_entry.v`'s create-side logic; I did not trace it symbol-by-symbol given time
  budget, but the STB's `stb_create_bytes_vld[7:0]`/`stb_create_shift[3:0]` signals
  (stb.v:394,406) are the obvious entry points.

## A6. Misaligned access handling — HW-split under a chicken bit, else trap

- Misalign is detected purely from the low address bits vs. size:
  `ag_pipe_misalign_unmask` = `addr[0]` (HALF) / `|addr[1:0]` (WORD) / `|addr[2:0]` (DWORD)
  (ag.v:1071-1080).
- **`ag_pipe_unalign_permit = cp0_lsu_mm && ag_pipe_inst_unalign && ...`** (ag.v:1103-1106) —
  gated by a CP0 chicken bit. `cp0_lsu_mm` is **MXSTATUS.mm, bit 15 of the *MXSTATUS* CSR**
  (a separate custom C-SKY/T-Head CSR from MHCR — cp0/ext_csr.v:550-569), **reset value 1**
  (`mm<=1'b1` on reset, ext_csr.v:554) — misaligned accesses are HW-handled by default out of
  reset, opposite of MHCR's cache-enables which reset to 0.
- If permitted **and** the access actually crosses an 8-byte boundary
  (`ag_pipe_va_add_unalign[3] = ({1'b0,addr[2:0]}+{1'b0,size[2:0]})[3]`, ag.v:1100-1101 —
  `ag_pipe_boundary_unmask`), AG runs a 2-pass FSM (`UNALIGN_IDLE` + states around
  ag.v:1126-1140) that re-issues the access with an 8-byte-rounded address
  (`ag_pipe_addr = unalign_cur_state ? {ag_addr[63:3],3'b0} : ag_addr`, ag.v:1251-1253) —
  same 2-pass-split shape as C910's boundary_stall mechanism per the rv12 note, just at an
  8-byte (not 16-byte) granularity, matching this LSU's 64-bit (not 128-bit) datapath.
- If **not** permitted (or crosses a page for the no-page case): `ag_pipe_misalign_no_page =
  ag_pipe_misalign_unmask && ... && !ag_pipe_unalign_permit` (ag.v:1118-1120) — this is the
  hardware trap path (misaligned load/store address exception, RISC-V cause 4/6), matching
  the task's framing that a trap path here needs RTU/CP0 support even in "minimal CSR" M2
  scope. **Since `mm` resets to 1, the default out-of-reset behavior is HW-handled
  misalignment**, not trap-on-misalign — M2 should decide explicitly whether to keep that
  default or force `mm=0` for a simpler-to-verify trap-only M2 cut.

## A7. Address translation status — a real MMU exists; the base LSU path unconditionally talks to it, but M2 can legitimately stub it

- AG issues a real request/response protocol to a **separate MMU unit** (`refs/openc906/
  .../mmu/rtl/`, `aq_mmu_top.v` + `aq_mmu_utlb.v`/`aq_mmu_jtlb.v`/`aq_mmu_ptw.v` — real uTLB,
  JTLB, and a hardware page-table walker, structurally analogous to C910's separate MMU):
  outputs `lsu_mmu_va[51:0]`/`_va_vld`/`_priv_mode`/`_st_inst`/`_abort` (ag.v:1564-1570),
  inputs `mmu_lsu_pa[27:0]`/`_pa_vld`/`_ca`/`_so`/`_buf`/`_sec`/`_sh`/`_page_fault`/
  `_access_fault` (ag.v:198-206,597-605). **This is not skippable/optional at the protocol
  level** — every load/store's AG stage genuinely issues this request and blocks
  (`ag_pipe_tlb_miss`/`ag_self_stall`, ag.v:1053-1056) on a valid response before the DCache
  tag/data request goes out (`ag_pipe_ca = mmu_ca || ag_pipe_dca_pa`, ag.v:1444, feeds
  `ag_dc_ca`).
- However, **nothing in the AG<->MMU protocol requires the MMU side to be a real Sv39
  TLB+PTW**: it's a clean request(VA,priv,st)/response(PA,ca/so/buf/sec/sh,fault) interface.
  M2 can implement a **combinational identity-map stub** satisfying this exact protocol
  (`pa=va[39:12]`, `pa_vld=1`, `page_fault=access_fault=0`, `ca`/`so` from the sysmap lookup
  below) — the same approach rv12 prescribed for C910's M2 MMU stub — and defer the real
  uTLB/JTLB/PTW (mmu/rtl/) to M4's Sv39 bring-up. **Answering the task's scope question
  directly: the base LSU pipe does need to unconditionally exercise an MMU-shaped
  request/response interface, but does not need the real translation hardware behind it for
  M2** — physical=virtual is a valid M2 assumption as long as the stub still honors the
  protocol's timing (same-ish-cycle combinational response) so AG's stall logic isn't
  perturbed.
- PMP is a separate unit (`gen_rtl/pmp/`, not read) that would also need at least a
  pass-through stub; not investigated further here.

## A8. Atomics (LR/SC/AMO) — live in LSU, not IU (cross-ref for IU extraction)

- **LR/SC reservation**: `aq_lsu_lm.v` (167L) — a **single reservation register**, not a
  reservation-set: 2-state FSM `LM_OPEN/LM_EXCL` (lm.v:89-143) storing the exact `{addr[39:0],
  size[1:0]}` of the LR (`lm_addr<=lm_req_addr`, `lm_size<=lm_req_size` on `lm_set`,
  lm.v:152-156). `lm_pass = (state==LM_EXCL) && (addr matches) && (size matches)`
  (lm.v:159-161) — an SC only succeeds if the address **and** size exactly match the prior
  LR's reservation (no coarse line-granularity aliasing built in at this register). Set/clear
  from `aq_lsu_dc.v`: `lm_set = dc_inst_vld & dc_lr_inst & !dc_ld_reply` (an LR that actually
  completed this cycle, not a replayed miss); `lm_clr = dc_inst_vld & (dc_sc_inst |
  dc_amo_inst) & !dc_reply` (dc.v:1618-1621) — **any** SC or AMO clears the reservation,
  matching the RISC-V requirement that an intervening store-class op to any address
  invalidates the reservation. SC's return value is generated right here:
  `dc_sc_rdata = dc_lm_pass ? 0 : 1` (dc.v:1941) — 0=success, 1=fail, standard RISC-V
  convention.
- **AMO RMW sequencing**: an AMO's memory operand is read exactly like a load (rides the
  `LSU_LD_AMO_INST`/`LSU_LD_AMO_FUNC` fields of the load bus, `aq_lsu_cfig.h:31-32/40-44`)
  through DC/DA, landing as `da_amo_alu_src0[63:0]` (the aligned old value). Its RS2 operand
  is staged in the STB, tagged by id (`stb_amo_alu_src1[63:0]`, matched via
  `da_amo_stb_id`/`amo_alu_stb_id[1:0]`). `aq_lsu_amo_alu.v` (a standalone module,
  instantiated once in `aq_lsu_top.v:2241`, fed by DA+STB, not folded into DC) computes
  `amo_alu_stb_rst[63:0]` (add/xor/or/and/min/max/minu/maxu/swap, with `dw_sign_ext`/
  `wd_sign_ext`/`_unsign_ext` variants for 32- vs 64-bit AMO, amo_alu.v:107-201) and returns
  it to the **same** STB entry by id, from which it is written to the cache exactly like an
  ordinary store (via `aq_lsu_rdl.v`, same path as A4). So the RMW is: DC/DA read (cycle N) ->
  ALU compute (registered, `amo_rst_ff`, one extra cycle) -> STB entry updated in place ->
  drains through the normal store-write path. This is a genuine load-then-store pair
  internally, not a single atomic bus transaction — cross-reference this against whatever the
  IU extraction found for AMO decode/dispatch, since the actual RMW execution is entirely
  LSU-resident.
- Write-allocate-cancel (`aq_lsu_amr.v`) is unrelated to atomics despite the superficially
  similar module-name; see A3.

# (B) CP0/CSR BASICS

## B1. Structure (`aq_cp0_top.v` instantiates `aq_cp0_iui` + `aq_cp0_regs` + `aq_cp0_special`,
top.v:699-966; `aq_cp0_regs.v` in turn instantiates the per-CSR-group submodules)

`aq_cp0_iui` (869L, CSR-instruction execution: decodes csrrw/csrrs/csrrc(+I forms) and drives
the read/write bus) -> `aq_cp0_regs` (2242L, wiring hub) -> instantiates `aq_cp0_info_csr`
(469L: mhartid/mvendorid/marchid/mimpid/misa), `aq_cp0_trap_csr` (1439L: **mstatus, mie, mip,
mtvec, mscratch, mepc, mcause, mtval** — the exact M2-minimal set, all present as real M-mode
CSRs plus their S-mode shadow bits), `aq_cp0_prtc_csr` (190L), `aq_cp0_hpcp_csr` (285L:
mcycle/minstret **interface only**, see B2), `aq_cp0_float_csr` (441L: fcsr, FP scope), and
`aq_cp0_ext_csr` (1276L: custom T-Head CSRs — MHCR, MXSTATUS, MCOR, etc.) (regs.v:1854-2067).
`aq_cp0_fence_inst.v`/`aq_cp0_cache_inst.v`/`aq_cp0_vector_inst.v` (247L/84L/62L) and
`aq_cp0_lpmd.v` (233L, WFI) sit alongside, not traced in depth.

- **RMW**: `iui_csrrw_rs1=rs1`, `iui_csrrs_rs1=rdata|rs1`, `iui_csrrc_rs1=rdata&~rs1`, muxed
  by `iui_inst_csr_func[2:0]` into `iui_csr_wdata` (iui.v:494-500) — same three-op RMW shape
  as C910.
- **Bus-based generic CSR access + dedicated point-to-point fan-out (hybrid, same as C910)**:
  each CSR group has its own `xxx_local_en` decode line off a CSR-address bus (e.g.
  `mxstatus_local_en`, `mhcr_local_en`, ext_csr.v:558,681) gating a write of the common
  `iui_regs_wdata[63:0]` bus into that group's registers — this is the generic
  read/write-by-address path IDU dispatches csrrw/csrrs/csrrc through. Separately, values
  needed every cycle by other units are dedicated single-purpose wires, not read through the
  bus: `regs_iui_mepc[39:0]` (trap_csr.v:1382, for `mret`), `cp0_ifu_icache_en`/
  `cp0_lsu_dcache_en`/`cp0_lsu_mm` (ext_csr.v:1180,1220-1221), `cp0_hpcp_mcntwen`
  (hpcp_csr.v:280), etc.
- I did not find an explicit multi-state EX1/EX2/EX3 CSR-execute FSM in `aq_cp0_iui.v` the way
  the rv12 C910 note found in `ct_cp0_iui.v` (no `commit`-gated write pattern grepped there
  either) — C906's CSR write appears to be more directly combinational/single-cycle off the
  RMW bus, consistent with C906 being in-order (same "no commit gate needed" story as A4).
  Flag as **not fully verified** — I did not trace `aq_cp0_iui.v`'s full state machine (if
  any) line-by-line; read-directly before implementing if exact timing matters.

## B2. The M2-minimal CSR set, what's really HW vs. derived

- **mstatus** (trap_csr.v:729-732): full RV64 layout present (SD, MPV, SXL/UXL, TVM/TW/TSR,
  MXR/SUM/MPRV, MPP, SPP, MPIE/SPIE, MIE/SIE) — more than M2 needs; M2 can implement only
  MIE/MPIE/MPP as real state and tie the rest, mirroring rv12's C910 recommendation, since
  S-mode is out of scope alongside the MMU.
- **mie/mip**: `mip` bits `meip=biu_cp0_me_int`, `mtip=biu_cp0_mt_int`, `msip=biu_cp0_ms_int`
  (trap_csr.v:1234-1236) are **read-only wires sourced from the BIU** (i.e. ultimately
  CLINT/PLIC-equivalent external pins), exactly like C910's finding. `meip_en=meie&&meip`
  etc. (trap_csr.v:1272-1274); global qualification `meip_vld = (pm!=M || mie_bit) &&
  meip_en` (trap_csr.v:1302-1304); priority-encoded into `cp0_rtu_xx_int_b`/`_vec` for RTU
  (not traced further given time budget, but same shape as C910 per rv12 note B4).
- **mtvec/mscratch/mepc/mcause/mtval**: `mscratch` is a plain flopped 64-bit R/W register
  (trap_csr.v:222,1001-1008) — trivial. `mepc` stored without its LSB
  (`regs_iui_mepc={mepc_reg[38:0],1'b0}`, trap_csr.v:1382) — enforces 2-byte alignment,
  consistent with the IFU note's confirmed **byte-address** (not half-word) PC convention.
  mtvec/mcause/mtval not traced symbol-by-symbol but are present in the same file per the
  in-RTL CSR table comment at trap_csr.v:430-449.
- **mcycle/minstret are NOT real registers in CP0** — `aq_cp0_hpcp_csr.v` takes
  `hpcp_cp0_data[63:0]` as an **input** (hpcp_csr.v:41) and just decodes/muxes it for CSR
  reads; the actual counters live in the separate `gen_rtl/pmu/` unit (not read). **Same
  architecture as C910** (rv12 note B2) — M2 should implement two local 64-bit counters
  directly in the M2 CSR unit and keep the `hpcp_cp0_data`-shaped port stub for a later PMU
  milestone.
- **mvendorid/marchid/mimpid/mhartid/misa** (info_csr.v): `mvendorid=64'h5B7` (T-Head's real
  JEDEC-ish ID, hardwired), `marchid=mimpid=0`, `mhartid={61'b0,biu_cp0_coreid[2:0]}`
  (info_csr.v:103-131), `misa` = MXL=64-bit + a hardwired extension bitmask including
  `misa_fd=1` (F/D) and `misa_vector=1'b0` (**RVV disabled in this specific integer-focused
  read**, info_csr.v:150-155) — confirms this build's `misa` would need `A` added for M2's
  eventual atomics support (currently baked in as constants, not writable).
- **MHCR** (custom, ext_csr.v): `ie`(bit0,icache-en)/`de`(bit1,dcache-en)/`wa`(bit2,write-
  allocate)/`rse`(bit4)/`bpe`(bit5)/`btbe`(bit6) **all reset to 0** (ext_csr.v:696-721); `wb`
  (write-back) and `wbr` hardwired 1 (RO) (ext_csr.v:670,694) — **boot code must explicitly
  write MHCR to turn on I$/D$**, exactly matching the M1 IFU finding for `icache_en` and
  confirming the same is true for `dcache_en`. Since `dc_cache_hit = dc_cache_hit_raw &
  cp0_lsu_dcache_en` (dc.v:1264), **every** load/store is forced to miss until boot code sets
  `de=1`.
- **MXSTATUS.mm** (custom, separate CSR from MHCR, ext_csr.v:550-585): the
  misalign-permit bit consumed by LSU AG (A6), **resets to 1** — opposite polarity from
  MHCR's other bits. Worth flagging precisely because it's easy to mis-file this bit as "part
  of MHCR" (as the rv12 C910 note's summary did, bundling it there) — in this C906 RTL it is
  a genuinely separate CSR.

## B3. Uncached/device-memory (MMIO) path — a real, hardwired, address-range PMA table, independent of Sv39

`gen_rtl/mmu/rtl/aq_mmu_sysmap.v` (206L) + `aq_mmu_sysmap_hit.v` (45L) + `sysmap.h` (43L) —
an **8-region fixed physical-address-range table**, keyed purely on `mmu_sysmap_pa[27:0]`
(the 28-bit PPN, i.e. `PA[39:12]`), each region giving a 5-bit `{StrongOrder, Cacheable,
Bufferable, Shareable, Security}` flag (`sysmap.h:16-17,19-41`):

```
SYSMAP_BASE_ADDR0=28'h8ffff  FLG0=5'b01111
SYSMAP_BASE_ADDR1=28'hbffff  FLG1=5'b10011
...
SYSMAP_BASE_ADDR7=28'hfffffff FLG7=5'b01111
```

These are **`` `define `` constants — hardwired at RTL-compile time, not CP0-programmable
registers** — comparators in `aq_mmu_sysmap.v` bucket the incoming PPN into the first region
whose upper bound it's `<=` (ascending thresholds, `aq_mmu_sysmap_hit.v:39-42`), and the
result (`sysmap_mmu_flg[4:0]`) feeds into the MMU's overall `ca`/`so`/`buf`/`sh`/`sec` output
to LSU (`mmu_lsu_ca` etc., A7) **entirely independent of the page-table/TLB path** — it fires
whether or not a real Sv39 walk happened. This directly answers the task's question: yes,
there is a genuine fixed-address-range cacheability/MMIO determination mechanism, it's
**physical-address-based**, and it does **not** require Sv39 PMA bits to exist. For M2:
1) these specific thresholds encode T-Head's *own* reference SoC's memory map (roughly a
~2.25GB cacheable low region, then several small strongly-ordered/uncached windows up near
the top of a 40-bit PA space) — **rv906's own memory map (with its own CLINT/PLIC/UART
addresses) will need this table's constants edited to match**, since they're compile-time
`define`s, not something software reprograms; 2) the M2 MMU stub (A7) needs to either
instantiate a same-shaped sysmap lookup, or fold an equivalent simpler "is this address in
the DRAM aperture" range check directly into the stub, to give LSU a real `ca`/`so` signal
for MMIO test programs without waiting on M4's Sv39 bring-up.

# Cross-cutting notes / open items

1. **No RTU-commit gate on stores anywhere in the scalar LSU** (A4) — likely a genuine,
   safe simplification for M2 given C906/rv906 are single-issue in-order, but flagged as a
   design decision, not yet confirmed against rv906's own squash/flush timing.
2. **DCache is 4-way, ICache is 2-way** — do not carry over ICache way-count assumptions;
   independently corroborated by public C906 documentation (VIPT 4-way D$, VIPT 2-way I$).
3. **VIPT alias-detection (dc_virt_idx[0]=addr[12]) is real HW in this design** (A3) but is
   very likely **safely elidable for M2** if rv906's M2 test programs never create two
   different VA mappings to the same PA (trivially true under an identity/fixed-offset
   MMU stub) — recommend explicitly deferring the alias-merge behavior to whenever M4's Sv39
   might actually produce synonyms, while still budgeting the SRAM area/bit layout for it if
   the DCache module structure is meant to carry forward unchanged.
4. **MXSTATUS.mm is a separate CSR from MHCR**, resets to 1 (misalign HW-handled by default)
   — the opposite of MHCR's caches-off-by-default reset. Decide explicitly whether M2 keeps
   HW misalign handling on by default or forces trap-on-misalign for a simpler-to-verify cut.
5. **AG↔MMU is a real, unconditionally-exercised protocol, but the MMU behind it can be an
   M2-only combinational identity-map + sysmap-lookup stub** (A7) — physical=virtual is a
   legitimate M2 assumption; defer uTLB/JTLB/PTW (mmu/rtl/) to M4.
6. Read-directly spans flagged inline: dc.v:1195-1270 (alias detect) + rdl.v:645-656 (alias
   consumer); stb_entry.v:~612 (partial-byte-overlap forward merge); aq_cp0_iui.v (CSR-write
   timing, no FSM found but not exhaustively traced); dc.v beat-by-beat LFB refill interaction
   with the alias/way-select muxes (not opened, M3 territory per A1).
7. LR/SC atomics and AMO ALU are 100% LSU-resident (A6/A8) — cross-check against the IU
   extraction's decode/dispatch findings for AMO/LR/SC opcodes; the actual RMW execution
   RTL described here is authoritative for "where does the work happen."
