# 7. The Load/Store Unit: LSU, DCache, MMU

This chapter describes `rtl/LSU.v` (the load/store pipeline + store buffer),
`rtl/DCache.v` (the 32 KB 4-way data cache), and `rtl/MMU.v` (the M2
address-translation stub). The LSU executes loads and stores dispatched from
`IDU.v` (chapter 4), talks to the DCache, and owns the D-side AXI master and
the store buffer (STB).

**How to read it.** Section 1 is the principle. Section 2 is the LSU
pipeline, section 3 the DCache, section 4 the MMU stub. Section 5 is the C906
cross-reference. Section 6 is the design discussion.

**Normative documents.** Contract
`docs/superpowers/specs/2026-08-20-m2-integer-design.md`; C906 facts in
`notes/2026-08-20-c906-lsu-base-cp0-extraction.md`. Donor citations relative
to `refs/openc906/C906_RTL_FACTORY/gen_rtl/lsu/rtl/`.

---

## 1. Principle: one outstanding miss, store buffer decouples stores

C906 is in-order with a write-back data cache. The LSU

1. computes the address in AG (one 64-bit adder, combinational misalign
   detect, MMU request),
2. reads the DCache tag+data in DCS,
3. on a hit, returns the data (load) or updates the line (store); on a
   miss, handles the refill (load) or, for a store with write-allocate
   disabled, writes straight to memory,
4. decouples stores from the pipeline with a 4-entry store buffer (STB).

rv906 models a **single outstanding miss** (design doc S2.2): no LFB array,
one miss in flight at a time. Stores buffer in the STB and drain when the LSU
is idle.

---

## 2. The LSU pipeline (`rtl/LSU.v`)

States: `ST_IDLE` (AG) → `ST_DCS` (DCache response) → `ST_FRZ` (miss
refill / victim write-back / direct AXI) → `ST_REPLY` (byte-rotate +
sign/zero-extend + STB forward, completion).

- **AG**: one adder, combinational `ag_misalign`, MMU-stub request
  (page-number out, physical page number back, reassembled PA), byte
  position/mask math.
- **Misalignment is trap-only in M2** (contract 3): `ag_misalign` raises the
  misaligned-address exception; the donor's two-pass HW-split is deferred to
  M4. `cp0_lsu_mm` (MXSTATUS.mm) is **not consulted** for this decision.
  (`rv64ui-p-ma_data` exercises the HW-split and is the documented M2
  exception — see chapter 8.)
- **Store buffer (STB)**, 4 entries: stores buffer and drain when idle
  (`issue_drain`). Byte-granular forward merges a pending store into a
  later overlapping load. A store that hits an existing STB entry merges
  (byte-mask OR) instead of allocating.
- **Store-miss write-back** (wa=0): a store miss with write-allocate
  disabled writes straight to memory (direct AXI), sized and addressed to the
  *store* (not the line) — `awsize` = store size, `awaddr` = the store
  address (Task 10 fix, see §6).
- **FENCE.I clean walk** (Task 10): on FENCE.I, after the STB drains, the
  LSU walks every set, writes back each valid+dirty line (reusing the
  victim-write-back AXI path), and invalidates it
  (`cp0_lsu_dcache_clean`/`lsu_cp0_clean_done` handshake with CSR.v chapter 6).
- **DCache response is combinational at DCS** (matches the donor SRAM's
  1-cycle read latency): `dc_resp_vld = (state == ST_DCS)`, response muxed
  combinationally. The `dcache_tb` timing checks encode this.

---

## 3. The DCache (`rtl/DCache.v`)

32 KB, 4-way, 64-byte lines. Tag + data + dirty SRAM arrays. A request is
accepted in `ST_IDLE`, compared/read in `ST_DCS` (combinational response),
with `ST_FRZ` reserved for the invalidate-vs-request collision (adds exactly
one cycle). A way-select peek lets the LSU read out a victim's tag+data for
write-back. Invalidate clears exactly the targeted way's valid bit.

---

## 4. The MMU stub (`rtl/MMU.v`)

M2 has no virtual memory (M4). The MMU is an identity-map stub: it answers
the AG's page-number request with the same page number (`pa = va`), reports
the PMA attributes (cacheable/bufferable/strongly-ordered) from a sysmap
lookup, and never faults. The protocol (page-number out, physical page number
+ attributes back) is the real donor interface so M4 can fill it in without
rewiring.

---

## 5. C906 file cross-reference

| rv906 | C906 donor file(s) |
|---|---|
| `rtl/LSU.v` AG | `aq_lsu_ag.v` |
| `rtl/LSU.v` DC/DA pipeline | `aq_lsu_dc.v`, `aq_lsu_da.v` |
| `rtl/LSU.v` STB | `aq_lsu_stb.v` |
| `rtl/LSU.v` victim write-back | `aq_lsu_vb.v` |
| `rtl/DCache.v` | `aq_dcache_top.v` + tag/data/dirty arrays |
| `rtl/MMU.v` | `aq_mmu_*` (protocol only; M2 is identity-map) |

---

## 6. Design discussion: deviations, findings, and what stayed unbuilt

**Deliberate deviations (documented, design-doc-sanctioned):**

- **Single outstanding miss** (design doc S2.2): no LFB array; one miss in
  flight. Sufficient for M2's tests; M3 adds the fuller store/miss buffering.
- **Misalignment trap-only** (contract 3): HW-split deferred to M4.
  `rv64ui-p-ma_data` is the documented M2 exception.
- **MMU identity-map** (M4 fills in translation).

**Bugs found during M2 bring-up (fixed in this milestone):**

1. **Sub-word store-miss wrote the whole beat** (Task 10, `rv64ui-p-sb/sh`).
   The direct store-miss AXI write hard-coded `awsize=6` (64 B) and the line
   base address, clobbering neighbouring bytes. Fixed to `awsize` = store
   size and `awaddr` = the store address (donor `aq_lsu_stb.v:895-898`).
2. **STB needed a per-entry size** for the drain path (the drain doesn't
   re-latch `dc_size_r`), so a drained wa=0 store writes back at the right
   size. Added `stb_size[0:3]`.
3. **DCache response timing** made combinational at DCS to match the donor
   SRAM 1-cycle latency; `dcache_tb` T1/T7 encode the cycle counts.

**Deliberately unbuilt in M2:** HW misaligned split (M4), hit-under-miss /
multi-outstanding misses (M3), the full LFB, and hardware prefetch.

---

*Next: chapter 8 (`08-verification.md`) covers the harness that proves all of
this.*
