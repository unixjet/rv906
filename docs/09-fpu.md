// 9 FPU (M5 complete)

Scalar single/double-precision FPU (`rtl/FPU.v`), per design doc §7.4 row M5
("scalar FPU (F/D, half-precision transfers)", acceptance "rv64uf/ud pass").

## Architecture overview

`FPU.v` is a **flattened transcription of the rv12 FPUAlu algorithm sections
(2–11, plus FMAU/FDSU 12–14)** — the C910 scalar algorithms, minus the
flop-chain pipeline: rv906 is single-issue in-order, so each sub-unit's
arithmetic runs combinationally from EX1 latched operands and retires at
EX2, mirroring how `IU.v`'s ALU consumes `idu_iu_ex1_*` (design doc lines
48/141-173 pre-architected the "EU stage: IU and FPU in parallel" slot;
deviation D1 in the header comment). Diffing against
`../rv12/rtl/FPUAlu.v` section by section is the intended maintenance path.

Three sub-units, one each (no dual-pipe, no forwarding network):

- **FALU** (fadd/fsub/fmin/fmax/fsgnj{,n,x}/fclass/feq/flt/fle + FCNVT
  f2f `fcvt.{s.d,d.s}`) — normalise, align, add/subtract, round (all six
  rm modes + RNI/RTZ/RZ for conversions), canonical-NaN generation.
- **FMAU** (SECTION 12) — fused multiply-add: `fmul.{s,d}` (addend
  don't-care) plus R4 `fmadd/fmsub/fnmsub/fnmadd.{s,d}` sharing one
  VLS-precision product datapath; sign/magnitude combination for the
  four N/S variants; `au_xvld`-guarded int-conversion saturation terms
  are reused here for f2i.
- **FDSU** (SECTION 14) — iterative restore-division and digit-by-digit
  sqrt, one iteration per cycle, `busy`/`full` backpressure to the IDU
  hold logic (`idu_fpu_ex1_fdsu_sel` + ctrl `eu_full` term).

Pipeline position: IDU decodes an FP op into the `idu_fpu_ex1_*` bundle
(func word, fmt, rm, three source data words + source-kind bits) → FPU
computes → EX2 retire packet carries result + `fflags[4:0]` → RTU
latches it into `ex2_fpu_*` and (a) writes the destination FP register,
(b) sticky-ORs `fflags` and sets `mstatus.FS` dirty.

**Operand sourcing** (`rtl/IDU.v`, `dis_gpr_fsrc0`): FP registers live in
the same 32-register file as GPRs (no separate F-register file); each
operand is GPR-sourced (`fmv.w.x/d.x`, i2f, fclass on register...) or
FP-boxed per the decode arm's source-kind bits. The `FUNC_SPU_MV` /
`FUNC_MAU_NEG` bits alias at bit 17 — every consumer must AND-gate on a
second discriminating bit (the "reused bits stay purely AND-gated" rule,
`rvproc_pkg.sv:888-914`); M5 Task 11 hit this once (fnmadd/fnmsub read
fsrc0 from the GPR file, fixed by AND-gating with `FUNC_SPU_SGN`).

**FP write-back tracking (FWBT)**: 32-entry scoreboard section in
`IDU.v` (~:1702-1850) tracks in-flight FP results so same-register
sources resolve to the freshest committed value (EX1 forwarding with
lf0/lf1 updates), same discipline as the GPR forward paths.

## Key behaviors

- **Rounding DYN (rm=111)** resolves to the `frm` CSR at one point in
  FPU.v: `rm_eff = (idu_fpu_ex1_rm == 3'b111) ? cp0_fpu_frm : ...` —
  R4 FMA ops have no rm field in the instruction, so their latched
  `rm==111` always routes through `frm` (RISC-V spec; rv12 has the same
  single resolution point). All seven internal rm consumers use `rm_eff`.
- **NaN-boxing**: a single-precision op whose 64-bit FP source has
  non-all-ones high 32 bits canonicalizes to qNaN (`0x7fc00000`) instead
  of using the raw low half (`a_cnan/b_cnan` → `spu_a_s32/spu_b_s32`,
  FPU.v:612-628, 678-679). Enable is `box_check_en = FUNC_B_SINGLE`, so
  **every single-precision decode arm sets `FUNC_B_SINGLE`** (18 arms;
  Task 11 BUG 4 — it was missing on all of them, which let
  `fsgnj.s` on a broken-box qNaN-double pass the raw low 32 bits).
  f2i singles carry an equivalent term in the converter itself, so
  `fcvt.w.s/wu.s` needed no decode change.
- **fflags same-cycle retire**: `csr_read_mux`'s fflags/fcsr view
  forwards the in-flight EX2 retire packet —
  `fflags_rd = fflags_reg | (rtu_cp0_fs_dirty_updt ? rtu_cp0_fflags : 0)`
  (CSR.v ~:1274-1293) — so a `csrr fflags` in EX1 of the same cycle an
  FP op retires at EX2 sees the just-accrued bits. Same-cycle CSR-write
  priority is preserved (local_en arm precedes the accrual arm).
- **FDSU exclusion**: fused FMA ops set FUSED/SUB ctrl bits that alias
  `FUNC_FDSU_DIV/SQRT`, so both the FDSU select and the ctrl `eu_full`
  hold term exclude them with `!ex1_func_r[FUNC_MAU_MUL]` (IDU.v:1993,
  2184) — without this, `fmadd` hangs the FDSU path.
- **misa.F/D** flipped 0→1 at the Task 9 swap commit (`55cb318`);
  `mstatus.FS` accrues dirty on any FP retire (Task 8).

## Integration points

- `RVProc.v`: `u_fpu` between the IDU EU dispatch and the EX2 retire
  broadcast; `cp0_fpu_frm` wire from CSR to FPU.
- `IDU.v`: FP decode arms (R-type FP + R4 FMA + FMA-routed fmul),
  FWBT scoreboard, FDSU full/hold, `dis_gpr_fsrc0` operand sourcing.
- `CSR.v`: `frm`/`fflags`/`fcsr` storage + forwarded read views,
  `rtu_cp0_fs_dirty_updt` accrual, FS dirty.
- `RTU.v`: EX2 FP retire capture (`rtu_cp0_fflags`/`rtu_cp0_fs_dirty_updt`).

## Test coverage

- `test/m2/unit/fpu_tb.cpp` T1–T22: FADD.S/D value+flags+NaN-box, FSPU
  sign-inject/fclass, FCNVT f2f/i2f/f2i (all boundaries, RNE ties,
  saturation), FMV bit-moves, FMAU.D/S all four fused variants + plain
  fmul (native-cast oracle sweeps), FDSU busy/full timing, FDIV/FSQRT
  abnormal-input sets. **UNIT-SUITE-PASS** (13 benches).
- `test/m5/run_all.sh`: **46/46** — rv64uf-p/v + rv64ud-p/v
  (fadd fclass fcmp fcvt fcvt_w fdiv fmadd fmin ldst move recoding;
  structural on -p only). No standalone fmul ELF (fmul lives in
  fadd.S).

## Task 11 bug ledger (all Category A — rv906-own, no donor scalar FPU)

The public C906 factory ships no scalar FP unit (only vector
vfalu/vfdsu/vfmau), so clone-discipline classification is A: spec/rv12-
algorithm fidelity.

| # | Symptom | Root cause | Fix |
|---|---------|-----------|-----|
| FDSU hang | every fused FMA hangs | FUSED(5)/SUB(7) alias FUNC_FDSU_DIV/SQRT | `!FUNC_MAU_MUL` exclusion, IDU.v:1993/2184 |
| BUG 1 | fnmadd/fnmsub wrong values | bit-17 FUNC_SPU_MV==FUNC_MAU_NEG alias, `dis_gpr_fsrc0` read ungated | AND-gate with FUNC_SPU_SGN, IDU.v:2110 |
| BUG 2 | FMA rounding ignored DYN | rm=111 latched, never resolved | `rm_eff` via `cp0_fpu_frm`, FPU.v:350 |
| BUG 3 | fcmp/fcvt_w fflags 0 | csrr in EX1 misses same-cycle EX2 accrual | read view forwards retire packet, CSR.v |
| BUG 4 | ud-move test 40 raw bits | 18 single arms missing FUNC_B_SINGLE → box check off | arms set bit, IDU.v (no-op for boxed data) |
