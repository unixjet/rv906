# M5: Scalar FPU (F/D) — design doc

## Status: Exploration complete; decisions settled; implementation starting at Task 1

## Context

M4 (privilege M/S/U + Sv39 MMU + PMP + counter CSRs) is complete: all ten
tasks committed (`1e47c25`..`0a1643b`), working tree clean, both OFF-path
and ON-path acceptance gates green (`docs/08-verification.md` §8.15).

M5 is the next milestone per the design doc (`docs/superpowers/specs/
2026-08-20-rv906-c906-clone-design.md` §7.4 row M5): *"scalar FPU (F/D,
half-precision transfers)"*, acceptance *"rv64uf/ud pass"*. The standing
directive is "do as C906 does" (gate-level faithful clone, donor RTL is
the spec, cite `file:line` for every clone-fidelity decision).

The pipeline was pre-architected for this: the design doc already lists
the EU stage as *"IU (ALU/MULT/DIV/BJU) and FPU (FALU/FMAU/FDSU) in
parallel"* (design doc lines 48/141-142/173) and CSR.v already carries a
storage-only `mstatus.FS` field with a comment pointing straight at this
milestone (`rtl/CSR.v:635-636`: *"FS: M4 keeps it storage-only (no FPU
until M5)..."*).

## What M5 builds

1. **FPU cluster** (`rtl/FPU.v`, new) — FALU (add/sub/compare/min-max/
   sign-inject/classify/FP↔FP-precision-convert), FMAU (multiply/FMA,
   shares one datapath across MUL/MADD/MSUB/NMADD/NMSUB per the donor),
   FDSU (iterative div/sqrt). Single-issue, in-order: at most one
   instance of each unit, no dual-pipe, no forwarding network (nothing
   to forward across in an in-order single-issue pipe).
2. **FP register file (FRF)** — 32×64b, inline inside `rtl/IDU.v`
   alongside the existing integer GPR block (mirrors rv906's own
   established convention, not the donor's — see D2), no hardwired-zero
   register, 2 write ports (FPU result + LSU load result).
3. **FP CSR state** — `fflags`(0x001)/`frm`(0x002)/`fcsr`(0x003) storage
   in `CSR.v`; activation of the existing `mstatus.FS`/`sstatus.FS`
   field (illegal-gating when Off, Clean→Dirty tracking, SD aggregation
   into `mstatus[63]`/`sstatus[63]`).
4. **Decode** — LOAD-FP/STORE-FP/OP-FP/MADD/MSUB/NMSUB/NMADD opcode
   majors in `IDU.v`, dispatched to a new `EU_FPU` class alongside the
   existing `EU_ALU/EU_MUL/EU_DIV/EU_BJU/EU_LSU/EU_CP0` classes.
5. **LSU integration** — FLW/FSW/FLD/FSD ride the existing AG/DC pipe
   unchanged structurally; write-back destination becomes the FRF
   instead of the GPR; FLW result is NaN-boxed at the LSU write-back
   formatter (rv12 precedent).
6. **misa F/D bits** — flipped from 0→1 only at the swap commit (THE
   SWAP, same sequencing rule M4 used: new decode/CSR acceptance flips
   at exactly one commit, not incrementally).

## Donor sources

`refs/openc906/C906_RTL_FACTORY/gen_rtl/`:
- `vdsp/rtl/aq_vpu_top.v` — combined vector+FP cluster top (donor has no
  separate scalar-FP-only top; FP execution units live inside the
  vector/DSP cluster because C906 bundles V+F/D together).
- `vfalu/rtl/` (`aq_falu_top.v`, `aq_fspu_top.v`, `aq_fcnvt_*.v`,
  `aq_fadd_*.v`) — add/sub/sign-inject/classify/convert.
- `vfdsu/rtl/` (`aq_fdsu_scalar_ctrl.v`, `aq_fdsu_round.v`,
  `aq_fdsu_denorm_shift.v`) — iterative radix-4 SRT div/sqrt.
- `vfmau/rtl/aq_vfmau_*.v` — shared multiply/FMA datapath.
- `vidu/rtl/aq_vidu_vid_gpr_fp.v` (+`_reg_fp.v`) — separate 32×64b FRF,
  parallel structure to `idu/rtl/aq_idu_id_gpr.v`.
- `cp0/rtl/aq_cp0_float_csr.v` — fflags/frm/fcsr storage + accrual.
- `cp0/rtl/aq_cp0_trap_csr.v:282,561-569,585-586` — mstatus.FS field,
  dirty-update gating, FS-off illegal gating.
- `idu/rtl/aq_idu_id_decd.v:989,682-684,1660-1719` — OP-FP/LOAD-FP/
  STORE-FP opcode decode; `idu/rtl/aq_idu_cfig.h:510-971` — FP func
  encodings incl. half/bf16.

## Sibling rv12 precedent (M5 already complete there)

rv12 (C910 clone, out-of-order superscalar) completed its own M5 across
14+ tasks (`rv12` git log, tasks 5-14 + fix rounds). Its `rtl/FPU.v`
(1612 lines) is a **two-pipe, five-execution-unit** cluster with a
dependency-tracking `FRFDepCell`, an 8-source forwarding network in
`FRF.v`, and issue-queue/rename machinery in `VIQ.v`/`ISU.v` — all of
that is OOO-superscalar shape and explicitly **does not transfer** to
rv906's single-issue in-order pipe. What **does** transfer as reference:
- `rv12/rtl/CSR.v` fcsr/frm/fflags addresses, `mstatus.FS`/`sstatus.FS`
  2-bit-field-with-two-views semantics, dirty-bit-on-write gating,
  FS-off illegal gating — all architecture-independent CSR semantics.
- `rv12/rtl/IDU.v` decode table shape (opcode/funct3/funct7/fmt
  extraction) for LOAD-FP/STORE-FP/OP-FP/MADD family.
- `rv12/rtl/FPUDiv.v` SRT digit-recurrence arithmetic core (the math
  itself, not its dual-pipe write-back-steal handshake).
- `rv12/rtl/LSU.v` NaN-boxing-at-write-back-formatter pattern for FLW.
- `rv12/test/m5/build/*.elf` test list: `{fadd,fclass,fcmp,fcvt,fcvt_w,
  fdiv,fmadd,fmin,ldst,move,recoding,structural}` × `{p,v}` ×
  `{rv64uf,rv64ud}` = 48 ELFs — the exact upstream riscv-tests target
  set, reusable verbatim.
- rv12's design doc gotchas (architecture-independent): NaN-box on every
  producer AND check on every single-precision consumer (no asymmetry);
  full IEEE gradual underflow with tininess-after-rounding, no
  flush-to-zero, `UF⇒NX` always asserted together; explicit `csrw` to
  fcsr/frm/fflags must win over stale pending accrual; `fld` never gets
  a NaN-box check (double has no box concept); divider early-exit set is
  `{zero, inf, NaN, overflow, deep-underflow}`, no power-of-two shortcut.

## rv906 current state (verified, pre-M5)

- `misa_value` (`rtl/CSR.v:927`) has M/I/C set, **F(bit5)/D(bit3) = 0**.
- `mstatus.FS`/`sstatus.FS` exist as storage-only 2-bit fields
  (`rtl/CSR.v:635-636,654,660`) — write-accepted, read-back, functionally
  inert. This is the only pre-existing FP hook in CSR.v.
- No `fflags`/`frm`/`fcsr` storage anywhere in `CSR.v` (file header
  `CSR.v:31` explicitly lists `fcsr` as "ABSENT, not stubbed" as of M4).
- No FLEN/FP-width constants in `rvproc_pkg.sv`.
- `IDU.v` has zero decode arms for LOAD-FP/STORE-FP/OP-FP/MADD family —
  those opcodes fall through to the generic illegal-instruction default.
- `IU.v`/`RTU.v`/`LSU.v` have zero FP-related identifiers — no frozen
  ports, no dead scaffolding to work around.
- **Register file precedent**: rv906's integer architectural register
  file is `reg [63:0] gpr_r [0:31]` **inline inside `IDU.v`**
  (`IDU.v:1174-1184`, "SECTION GPR", donor citation
  `aq_idu_id_gpr_gated_reg.v:86-109`), not a standalone module — 3 read
  ports via `gpr_read()` with same-cycle read-during-write merge, 2
  write ports one-hot decoded, x0 hardwired to 0. This is the structural
  precedent for the new FRF (D2).
- `test/m2/env/riscv_test.h:22` already vendors `RVTEST_RV64UF`
  (upstream boilerplate, unused); `test/m2/env/encoding.h` already
  vendors full FP instruction encodings. No rv64uf/ud entries in any
  Makefile/run script yet.

## Design decisions

**D1 — Single-issue in-order FPU cluster, no dual-pipe/forwarding.**
The donor and rv12 both build multi-pipe structures (donor: two lanes in
`aq_vpu_top.v` for its dual-issue vector context; rv12: two pipes for
OOO superscalar issue). rv906 dispatches one instruction at a time, so
`FPU.v` gets exactly one FALU/FMAU/FDSU instance, a simple EX-stage
handshake with the existing dispatch/retire machinery (same shape as
`IU.v`'s ALU/MUL/DIV), and a busy/stall signal for the iterative divider
instead of any cross-pipe forwarding network.

**D2 — FRF inline in `IDU.v`, not a standalone module.**
The donor's FP register file (`aq_vidu_vid_gpr_fp.v`) lives in a
separate `vidu` cluster because C906 bundles the vector engine and FPU
together behind one combined issue/decode/GPR structure; the split
exists to serve the vector unit, not the FPU itself. rv906 has no vector
extension, so there's no reason to introduce a second top-level
decode/GPR cluster — the FRF is added as a second `reg [63:0] frf_r
[0:31]` array in `IDU.v`'s existing GPR section, sized like the GPR but
with no hardwired-zero entry (per spec, `f0` is a real register).

**D3 — No vector extension, no BF16.**
The donor's `cp0_vpu_xx_bf16` config bit and the vector-cluster context
that motivates it don't exist in rv906's scope; BF16 is dropped
entirely (not part of stock RV64GC).

**D4 — Half-precision (Zfh) full arithmetic is deferred; M5 targets
F/D only.**
The milestone's actual pass criterion is *"rv64uf/ud pass"* — it does
not mention a Zfh test suite, and rv12's own M5 (the sibling precedent)
shipped the same 48-ELF F/D-only list with no zfh tests. The donor
implements full FP16 compute (`FADDH`/`FMULH`/etc., `aq_idu_cfig.h:
657-971`) because C906 targets it as a product feature, but that's
extra scope beyond this milestone's acceptance bar. **Decision: decode
`fmt==01` (FP16) far enough to correctly recognize it as
not-yet-implemented (illegal instruction) rather than mis-decoding it as
double or single; do not build FP16 arithmetic paths.** This satisfies
"half-precision transfers" in spirit only if a later milestone or fix
round adds FLH/FSH/FMV.H.X — tracked as a documented deferral
(D-M5-1), not silently dropped. Re-evaluate after F/D acceptance is
green if time budget allows.

**D5 — NaN-boxing: producer-side box, load-path box, consumer-side
check.**
Every FALU/FMAU/FDSU single-precision result output is boxed (upper 32
bits forced to 1s) at the producing stage, mirroring the donor's
write-side boxing (`aq_fadd_double_add.v:1404-1408` et al.) rather than
centralizing it in the FRF. `FLW` is boxed at the LSU write-back
formatter (rv12 `LSU.v:2150-2151` precedent) — `FLD` is never boxed
(double has no box concept). Every single-precision consumer port
checks its FRF-read input for a valid box and substitutes the canonical
quiet NaN if unboxed, per the RV spec's mandated behavior (rv12 P8
precedent, no donor-side asymmetry to replicate — apply the stricter
correct behavior).

**D6 — FP CSR state lives in `CSR.v`, not a separate module.**
Matches rv906's own convention (all of M4's privilege/PMP/counter CSR
state is consolidated in `CSR.v`, unlike the donor's separate
`aq_cp0_float_csr.v`). Add `fflags`/`frm`/`fcsr` storage, `mstatus.FS`/
`sstatus.FS` full semantics: FS==Off (`2'b00`) makes any FP opcode or
`fflags`/`frm`/`fcsr` CSR access illegal (mirrors the existing `misa.F`-
gated illegal check pattern already used for `misa.C`); any FP-producing
instruction commit or explicit write to `fflags`/`frm`/`fcsr` while
FS∈{Initial,Clean} drives FS→Dirty; `mstatus[63]`/`sstatus[63]` (SD) is
the OR of {FS==Dirty, VS==Dirty (n/a, no vector), XS==Dirty (n/a)} —
for rv906 this collapses to `SD = (FS==2'b11)`.

**D7 — fflags accrual: sticky OR-in at retire, explicit write wins.**
On any FPU-op retire, `fcsr_fflags <= fcsr_fflags | retired_fflags`
(sticky OR, matches donor `aq_cp0_float_csr.v:234-238`). Because rv906
is in-order single-issue, there is no reorder window and therefore none
of rv12's OOO stale-accrual-ordering hazard — but the same rule still
applies structurally: an explicit `csrw`/`csrs`/`csrc` to `fflags`/
`fcsr` in the same cycle as a retiring FP op's accrual must win (write
observed on top of, not clobbered by, the accrual), since the write and
the accrual are almost always different cycles in-order anyway; document
the priority explicitly in the RTL comment regardless.

**D8 — FP load/store reuses the LSU pipe unchanged in structure.**
FLW/FSW/FLD/FSD decode as `EU_LSU`-class ops exactly like existing
integer loads/stores (same AG/DC pipeline, same STB/LFB/VB machinery,
"format-blind" — no FP-specific change to the memory pipeline itself).
The only new plumbing: a destination-register-file selector (FRF vs
GPR) carried alongside the existing writeback payload, and the FLW
NaN-boxing step in the write-back formatter (D5).

**D9 — Divider: adapt the SRT math, discard the multi-pipe steal
logic.**
`FPU.v`'s FDSU sub-block reuses the SRT digit-recurrence algorithm and
early-exit conditions from rv12's `FPUDiv.v` (itself following the
donor's `aq_fdsu_scalar_ctrl.v` radix-4 iteration counts: double≈29
rounds, single≈15, per `aq_fdsu_scalar_ctrl.v:417-424`), but replaces
rv12's two-named-pipe write-back-steal handshake with a single busy
flop that stalls new dispatch to FDSU until the current op completes
(no dual-pipe to arbitrate between).

**D10 — Full IEEE conformance: all 5 rounding modes, full subnormals,
no flush-to-zero.**
Matches the donor (`aq_fdsu_round.v:889-920` implements RNE/RTZ/RDN/
RUP/RMM in hardware; `aq_fdsu_denorm_shift.v` handles gradual
underflow) and rv12's corrected precedent (tininess-after-rounding,
`UF⇒NX` always asserted together). No shortcuts.

**D11 — misa F/D bits flip only at the swap commit.**
Same sequencing rule as M4's THE SWAP: every earlier task adds CSR
storage / decode arms / the FPU cluster itself while `misa.F`/`misa.D`
stay 0 and any not-yet-wired FP opcode still traps illegal, so the OFF
path (the entire pre-M5 battery) never regresses mid-implementation.
The swap commit flips `misa.F`/`misa.D`→1 and is the ONLY commit where
new FP-opcode acceptance goes live end-to-end.

**D12 — Test infra**: new `test/m5/` mirroring `test/m4/`'s shape
(Makefile with explicit p/v lists, env override only if FP tests need
one beyond the existing `test/m2/env`), reusing rv12's 48-ELF list
verbatim as the upstream riscv-tests target set (`{fadd,fclass,fcmp,
fcvt,fcvt_w,fdiv,fmadd,fmin,ldst,move,recoding,structural}` ×
`{rv64uf,rv64ud}` × `{p,v}`).

## Task decomposition

Every RTL task ends with the standard gates: `make verisim` clean,
`make -C test/m2/unit run` (UNIT-SUITE-PASS), `bash test/m2/run_all.sh`
(86/87), `bash /tmp/run_atomic.sh` (19/0), `bash /tmp/run_vsuite_full.sh`
(85/86) — the OFF path must stay bit-identical throughout, since
`misa.F`/`misa.D` stay 0 until the swap task.

| # | Task | Contents | Gate focus |
|---|---|---|---|
| 0 | Design doc (this doc) | Extraction notes + decisions D1-D12 | docs |
| 1 | CSR FP state | `fflags`/`frm`/`fcsr` storage, `mstatus.FS`/`sstatus.FS` full semantics (dirty tracking, SD bit), illegal-gating logic written but **not yet reachable** (no FP opcode decode exists yet) | csr_tb rows; OFF-path unaffected |
| 2 | rvproc_pkg + FRF | FP width/opcode/func constants in `rvproc_pkg.sv`; 32-entry FRF added inline in `IDU.v` next to GPR (no hardwired-zero), read/write port plumbing, not yet connected to any consumer | idu_tb rows |
| 3 | FPU.v skeleton + FALU | New `rtl/FPU.v`: FALU sub-block (add/sub, compare/min-max, sign-inject, classify, FP-precision-convert), wired into `RVProc.v` but IDU decode not yet routing to it | fpu_tb starts |
| 4 | Decode + FALU end-to-end | IDU decode arms for OP-FP add/sub/cmp/minmax/sgnj/classify/fcvt.f2f; LOAD-FP/STORE-FP decode + LSU FRF-destination plumbing + FLW NaN-boxing; dispatch routes to FPU/FRF end-to-end (still gated illegal since misa.F/D=0) | idu_tb/lsu_tb rows |
| 5 | FMAU | Multiply/FMA sub-block (shared datapath for FMUL/FMADD/FMSUB/FNMADD/FNMSUB), decode arms, dispatch wiring | fpu_tb FMA rows |
| 6 | FDSU | Div/sqrt SRT FSM (D9), decode arms, busy/stall dispatch gating | fpu_tb div/sqrt rows |
| 7 | Int↔FP convert + FMV | FCVT.{W,WU,L,LU}.{S,D}, FCVT.{S,D}.{W,WU,L,LU}, FMV.{X.W,W.X,X.D,D.X}, FCVT.S.D/FCVT.D.S | fpu_tb convert rows |
| 8 | fflags accrual + FS dirty wiring | Retire-time sticky OR-in (D7), FS-off illegal gating now reachable but still inert (misa.F/D=0 keeps everything illegal pre-swap) | csr_tb accrual rows |
| 9 | THE SWAP | `misa.F`/`misa.D` 0→1; this is the ONE commit where FP-opcode acceptance goes live | G1 OFF-path identity re-check |
| 10 | test/m5 infra | Makefile (p/v lists, 48 ELFs per D12), directed unit coverage as needed | build only |
| 11 | Acceptance + close-out | rv64uf-p/rv64ud-p/rv64uf-v/rv64ud-v suites, full M2-M4 battery re-run, docs (07-lsu.md or new 09-fpu.md, 08-verification.md §8.16, this doc's Status→COMPLETE) | all gates |

## Deviation ledger

- D-M5-1: half-precision (Zfh) arithmetic deferred — only recognized
  enough in decode to trap correctly as illegal, per D4. Milestone
  acceptance ("rv64uf/ud pass") does not require it; revisit later.
- D-M5-2: BF16 dropped entirely (donor's vector-context-only feature,
  D3).
- D-M5-3: FRF implemented inline in `IDU.v` rather than as a donor-
  mirroring separate module, since rv906 has no vector cluster to
  motivate the split (D2).
- D-M5-4: no dual-pipe/forwarding network — single-issue in-order needs
  none (D1).

## Files expected to change

- New: `rtl/FPU.v`, `test/m5/` (Makefile + any env override + fp test
  sources if any directed tests are needed beyond the upstream list).
- Major edits: `rtl/CSR.v` (fcsr/frm/fflags, FS semantics, misa F/D
  swap), `rtl/IDU.v` (FRF, FP decode arms, dispatch to EU_FPU),
  `rtl/RVProc.v` (FPU.v instance wiring), `rtl/LSU.v` (FRF destination
  selector + FLW NaN-boxing at write-back), `rtl/rvproc_pkg.sv` (FP
  constants).
- Small edits: `rtl/RTU.v` (fflags retire-time capture wiring, if not
  already reachable through existing writeback payload plumbing).
- Test infra: `test/m2/unit/Makefile` (FPU_RTL grows FPU.v; new
  `fpu_tb.cpp`), `test/m2/unit/idu_tb.cpp` / `lsu_tb.cpp` (new FP rows).

## Verification plan

**Two-sided acceptance (M4 precedent):**
1. **OFF path (G1):** the entire pre-M5 battery (unit suite, sweep
   86/87, atomics 19/19, v-suite 85/86, M4's directed si/mi/mmu 30/30 +
   ON-path v-suite 85/86) stays bit-identical through every task, since
   `misa.F`/`misa.D` = 0 gates every new FP opcode as illegal until the
   swap task.
2. **ON path:** after the swap, `rv64uf-p`/`rv64ud-p`/`rv64uf-v`/
   `rv64ud-v` (48 ELFs per D12) pass.

**Standard gates every task:** `make verisim` clean; `make -C
test/m2/unit run` → UNIT-SUITE-PASS; `bash test/m2/run_all.sh` → 86/87;
`/tmp/run_atomic.sh` → 19/0; `/tmp/run_vsuite_full.sh` → 85/86 (OFF
path); after M4's swap, also re-run `test/m4/run_directed.sh` (30/30)
and the ON-path v-suite (85/86) to confirm M4 stays intact through M5's
changes.
