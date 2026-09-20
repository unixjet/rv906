# M8-T6 Donor Runs — smart_run case verdicts under iverilog (csr, interrupt, MMU, exception, coremark)

Date: 2026-09-21 (machine local time JST/UTC+9; UTC timestamps in logs).
Spec: `2026-09-21-m8-crosscheck-design.md` §1, §2.2, §3 D-M8-2/D-M8-6, §4, task table T6, R1/R2.
Predecessor: `2026-09-21-m8-t1-dryrun-results.md` (T1, committed 81351a5).

## Verdict

Four of the five donor-side cases ran to a verdict: **csr, interrupt, MMU =
TEST PASS**; **exception = TEST FAIL** (the predicted R1 outcome — its
pagefault subtest faulting instructions are commented out in the donor source
while the checks stay live; it drops out of the parity set per the D-M8-2
consequence clause). **coremark is BLOCKED at test-build**: a fourth
donor-side patch is required (`-mtune=c906` is rejected by standard gcc
`cc1` for `.c` files — a design-doc premise error, §5 below) and it was not
applied because it lies outside the three approved patches; it is recorded
here with exact before/after text for the user's decision. The final parity
set declaration is therefore **{csr, interrupt, MMU} now, coremark pending**
the patch-4 decision.

## 1. Environment and reuse check

| Item | Value |
|---|---|
| Donor tree | `refs/openc906/` (same clone as T1; no donor RTL or tb touched) |
| iverilog | 12.0 (same as T1) |
| Toolchain | `/opt/riscv/bin` riscv64-unknown-elf-gcc 15.1.0 (`TOOL_EXTENSION`) |
| `xuantie_core.vvp` | **reused, not recompiled** — verified size 57,808,961 bytes (exact T1 artifact, mtime 2026-09-21 04:37, predates this session); no donor RTL changed since T1 |
| Env | `CODE_BASE_PATH=.../refs/openc906/C906_RTL_FACTORY`, `TOOL_EXTENSION=/opt/riscv/bin`, `SHELL=/bin/bash` on every make (T1 runtime requirements) |
| Per-case sequence | `make buildcase CASE=<c> SIM=iverilog SHELL=/bin/bash` then `cd work && vvp xuantie_core.vvp` |

## 2. Per-case results

| Case | Built? | Simulated? | Verdict (run_case.report, byte-exact, 9 bytes, no trailing newline) | $finish line (vvp stdout) | Cycles (simtime/100000) | vvp wall time |
|---|---|---|---|---|---|---|
| `csr` | yes | yes | `TEST PASS` | `../logical/tb/tb.v:269: $finish called at 273650 (100ps)` | **2736.5** | 117 s |
| `interrupt` | yes | yes | `TEST PASS` | `../logical/tb/tb.v:269: $finish called at 371650 (100ps)` | **3716.5** | 147 s |
| `MMU` | yes (after patch 1) | yes | `TEST PASS` | `../logical/tb/tb.v:269: $finish called at 455050 (100ps)` | **4550.5** | 177 s |
| `exception` | yes (after patch 2) | yes | `TEST FAIL` | `../logical/tb/tb.v:281: $finish called at 1329350 (100ps)` | 13293.5 (failed) | 431 s |
| `coremark` | **no** | — | not run | — | — | build failed in seconds |

Console (tb 0x10015000 watch) printed nothing for any case; every report
contains only the verdict string. `coremark` first error, verbatim
(`work/coremark_build.case.log`):

```
cc1: error: unknown cpu 'c906' for '-mtune'
make[2]: *** [Makefile:68: core_list_join.o] Error 1
```

## 3. Reproduce check (T1 baseline) — PASS

Same `xuantie_core.vvp` + same (unpatched) case sources ⇒ must match T1
exactly. Measured:

| Case | T1 verdict/cycles | T6 verdict/cycles | Match |
|---|---|---|---|
| `csr` | TEST PASS / 2736.5 (116 s wall) | TEST PASS / 2736.5 (117 s wall) | **exact** |
| `interrupt` | TEST PASS / 3716.5 (147 s wall) | TEST PASS / 3716.5 (147 s wall) | **exact** |

Sim-time (the cycle column, R7) is bit-identical; wall time is within
run-to-run noise. Nothing changed in the flow.

## 4. Donor-side patches applied (md5-ledgered; baseline = T1's post-patch state)

Only files under `smart_run/tests/` were touched. No donor RTL, no tb.
Before-images saved at `/tmp/m8_t6_{mmu,exception}_before.s`,
`/tmp/m8_t6_makefile_before.mk` (the pre-T6 Makefile = T1's final state).

### Patch 1 — `tests/cases/MMU/C906_mmu_basic.s` (6 sites)

`mxstatus` → `0x7c0` at exactly 6 sites (grep count 6 before, 0 after;
`0x7c0` count 6 after) in the two macro bodies:

```diff
 .macro MXSTATUS_THEADISAEE IMM
   #write cskyisaee
   li    x9,0x400000
-  csrc  mxstatus,x9
-  csrr  x9,mxstatus
+  csrc  0x7c0,x9
+  csrr  x9,0x7c0
   li    x10, \IMM
   slli  x10,x10,22
   or    x9,x9,x10
-  csrw  mxstatus,x9
+  csrw  0x7c0,x9
 .endm
```
and identically in `MXSTATUS_MAEE` (`:242-251`). No other line changed
(full-file diff reviewed).

md5: `7458e3ddca358db8daf5cd8e5b765516` → `676f6d9a065d72e61f6639e4c0542b8a`

### Patch 2 — `tests/cases/exception/C906_Exception.s` (2 lines)

```diff
       .option norvc
-      dcache.ciall
+      #dcache.ciall
 .global DATA_CACHE_DIS
 DATA_CACHE_DIS:
-      csrci mhcr,0x2
+      csrci 0x7c1,0x2
```

No other line changed (full-file diff reviewed).

md5: `087aa075932ebb1bd7faa2446176a5e1` → `145a107f5dece8cbcd0b78dfc6b5bb96`

### Patch 3 — `tests/lib/Makefile` line 50 (coremark CFLAGS, 4 vendor flags)

Dropped exactly `-msignedness-cmpiv -mno-thread-jumps1
-mno-iv-adjust-addr-cost -mno-expand-split-imm`; everything else on the line
byte-identical (verified with `cat -A` and full-file diff), incl.
`-DITERATIONS=10000`:

```diff
 ifeq (${CASENAME}, coremark)
-  CFLAGS +=  -c -O3 -mtune=c906 -static -funroll-all-loops -finline-limit=500 -fgcse-sm -fno-schedule-insns --param max-rtl-if-conversion-unpredictable-cost=100 -msignedness-cmpiv -fno-code-hoisting -mno-thread-jumps1 -mno-iv-adjust-addr-cost -mno-expand-split-imm -DITERATIONS=10000
+  CFLAGS +=  -c -O3 -mtune=c906 -static -funroll-all-loops -finline-limit=500 -fgcse-sm -fno-schedule-insns --param max-rtl-if-conversion-unpredictable-cost=100 -fno-code-hoisting -DITERATIONS=10000
 else
   CFLAGS += -c -O2
 endif
```

md5: `9e245e1542f6f23104533468ef8480d4` (T1 final) → `86c235ed3f8b0c7895751520dde19bbc`

### NOT applied — proposed patch 4 (coremark build blocker, pending user decision)

The coremark build (post patch 3) fails because **`-mtune=c906` is rejected
by standard gcc `cc1` for `.c` files**:

```
cc1: error: unknown cpu 'c906' for '-mtune'
```

- Probed on **both** toolchains: `/opt/riscv` 15.1.0 and xpack 15.2.0
  (rv906's, `test/m6/Makefile:31`) both emit the identical error on a `.c`
  probe; the full remaining coremark CFLAGS (minus `-mtune=c906` and the 4
  vendor flags) compiles clean on xpack (exit 0).
- Why T1's "-mtune=c906 accepted" measurement missed it: the flag sits at
  `tests/lib/Makefile:33` (`FLAG_MARCH = -mtune=c906`) and again at `:50`.
  For `.s` cases (csr/interrupt/MMU/exception) the gcc driver path is
  cpp→as — `cc1` is never invoked, so the flag is inert and the builds
  succeeded. Only `.c` compilation (coremark) reaches `cc1`, which rejects
  it. Design doc §2.3/§4.5 measured only `.s` builds (see §5, deviation D1).
- No T-Head toolchain exists on the machine to accept it (`/opt` inventory:
  /opt/riscv 15.1.0, xpack 15.2.0, and an RV32-only c2rtl 9.2.0).

Proposed change (same file, 2 lines, functionally neutral — `-mtune` is a
tuning hint with no effect on generated-code semantics; the rv906 side hits
the identical wall, so T3's `test/m8/Makefile` coremark line simply omits
`-mtune=c906` — identical-both-sides):

```diff
-FLAG_MARCH = -mtune=c906 
+FLAG_MARCH =  # -mtune=c906 (T6: vendor tune rejected by standard gcc cc1, see M8-T6 note)
```
(line 33; the trailing `#`-comment form follows T1's crt0 `#mxstatus`
traceability convention; make evaluates the value as empty) and on line 50
delete the `-mtune=c906 ` token (post-patch-3 state). Before md5 for the
file: `86c235ed3f8b0c7895751520dde19bbc`.

This goes beyond the three approved patches, so it was **not** applied; the
auto-mode change guard refused it as out-of-fence, which is the correct
behavior. **Decision needed:** approve patch 4 (recommended) or direct an
alternative (e.g. a toolchain wrapper) — the latter is discouraged: it would
hide the effective CFLAGS from the ledger and would not mirror cleanly on
the rv906 side.

## 5. Deviations from the design doc

- **D1 — §2.3/§4.5 premise error (measured, this session):** "the remaining
  CFLAGS, incl. … are accepted by /opt/riscv 15.1.0" was measured against
  `.s` builds only; `-mtune=c906` (present in `FLAG_MARCH` at
  `Makefile:33` *and* at `:50`) is rejected by `cc1` on `.c` files on both
  toolchains (verbatim errors above). Consequence: coremark needs a 4th
  ledgered donor patch (proposed, §4) and T3's rv906 coremark CFLAGS line
  must omit `-mtune=c906` (identical-both-sides, recorded).
- **D2 — data-driven, not a deviation: exception drops out.** Donor verdict
  is TEST FAIL ⇒ per D-M8-2 consequence clause the case drops from the
  parity set and T4 skips it (no port to a failing oracle).

### exception failure analysis (why it is source-inherent, R1)

- The FAIL came through the magic-fail path (x3 = 0x2382348720,
  `__fail`) at `tb.v:281`, not the watchdog (the "meeting max simulation
  time, stop!" banner at `tb.v:192` was never printed; 13293.5 cycles ≪ the
  7 M-cycle watchdog). So the case *ran* and reached its own
  `TEST_FAIL` label (`:510`, `jr __fail`) — either an explicit
  `bne …,TEST_FAIL` check or an unhandled trap into crt0's
  `vector_table`→`__fail` scaffold. No console output was produced, so the
  failing subtest is not identifiable from the run alone.
- The §4.2 source-level observation is confirmed in the donor source
  (unmodified by T6's patches): the pagefault subtests' **faulting
  instructions are commented out** (`:475-495`: `jalr x0,x1,0x0`,
  `lw x3,0x0(x1)`, `sw x3,0x0(x1)` all `#`-commented) while their **checks
  remain live** (`INS_PAGEFAULT_BEG` checks `mepc==0xfff00000` and
  `mcause==0xc`; `LOAD/STORE_PAGEFAULT` check `mtval==0xfff00000`,
  `mcause==0xd/0xf`). With no fault to take, `mepc` still holds the
  previous subtest's value, so the first `bne x1,x3,TEST_FAIL` at
  `INS_PAGEFAULT_BEG` fires. This is independent of T6's two exception
  patches: commenting out `dcache.ciall` (a performance hint in M-mode on
  the donor) and `mhcr`→`0x7c1` (the identical CSR numerically) cannot
  affect mepc/mcause/mtval — i.e. the case would fail on the donor even if
  buildable as-shipped. **T6's donor run is the confirmation the design doc
  required (§4.2): the donor verdict is unaffected-by-usable; the case is
  unsalvageable on the donor without editing the checks themselves, which
  is out of scope (donor sources are the oracle).**

## 6. coremark measurement

- First run: **not reached** — build failure (patch 3 applied, error in §2).
- Iteration trim (`core_main.c:178` `results[0].iterations= 2;` → `1;`):
  **not applied** (no sim run occurred). `core_main.c` baseline md5
  `5cc9d76be5086852d941165b049485df` recorded for when it does run.
- Resume procedure once patch 4 (or an alternative) is approved:
  1. Apply patch 4 (§4), re-record Makefile md5.
  2. `make buildcase CASE=coremark SIM=iverilog SHELL=/bin/bash`.
  3. `cd work && vvp xuantie_core.vvp` in the background with a **~2 h
     wall-time guard** (measured donor pace ≈ 23–25 cyc/s from csr/interrupt;
     est. 1–6 h for 2 iterations). If no `$finish` within ~2 h wall: kill,
     apply the identical-both-sides trim `results[0].iterations= 1;`
     (md5-ledgered), rebuild, re-run. Record which setting produced the
     final verdict; T4 must apply the same on the rv906 side.
  4. Capture `run_case.report` + the `$finish` line as in §2.

## 7. FINAL PARITY SET declaration (T6 output)

Donor-PASS cases measured in T6:

| Case | Donor verdict | In final parity set? |
|---|---|---|
| `csr` | TEST PASS (2736.5 cyc) | **yes** |
| `interrupt` | TEST PASS (3716.5 cyc) | **yes** |
| `MMU` | TEST PASS (4550.5 cyc) | **yes** |
| `exception` | TEST FAIL (R1, source-inherent) | **no — dropped** (recorded; T4 skips the port) |
| `coremark` | BLOCKED (build, patch 4 pending) | **pending** — enters the set if it PASSes on the donor after patch 4 + run (with iterations 2, or 1 iff the trim was applied) |

**Declared now: final parity set = {csr, interrupt, MMU} ∪ {coremark:
pending}.** T4 ports csr + MMU (and interrupt is T5); the exception port is
cancelled; coremark is ported only against a measured donor PASS. T7's
table covers exactly the donor-PASS set that exists at T7 time.

## 8. Artifacts left on disk

- Donor tree: the three patched files (md5s in §4), `work/` holding the
  last-built case (`exception`) artifacts + the reused `xuantie_core.vvp`
  (untouched), `work/coremark_build.case.log` (the build error).
- /tmp (ephemeral): `m8_t6_{mmu,exception}_before.s`,
  `m8_t6_makefile_{before,after_p3}.mk`, `m8_t6_md5_{before,after}.txt`,
  `m8_t6_run_{csr,interrupt,MMU,exception}.log` (full vvp transcripts),
  `m8_t6_report_{csr,interrupt,MMU,exception}.txt` (byte-exact reports),
  `m8_t6_build_*.log`, `m8_t6_run_case.sh` (per-case runner),
  `tune_probe.c` / `tune_probe*.o` (the -mtune probes).
