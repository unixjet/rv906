# M8-T6 Donor Runs — smart_run case verdicts under iverilog (csr, interrupt, MMU, exception, coremark)

Date: 2026-09-21 (machine local time JST/UTC+9; UTC timestamps in logs).
Spec: `2026-09-21-m8-crosscheck-design.md` §1, §2.2, §3 D-M8-2/D-M8-6, §4, task table T6, R1/R2.
Predecessor: `2026-09-21-m8-t1-dryrun-results.md` (T1, committed 81351a5).

## Verdict

All five donor-side cases ran to a verdict: **csr, interrupt, MMU, coremark
= TEST PASS**; **exception = TEST FAIL** (the predicted R1 outcome - its
pagefault subtest faulting instructions are commented out in the donor
source while the checks stay live; it drops out of the parity set per the
D-M8-2 consequence clause). coremark needed two further controller-approved
Makefile patches (patch 4: drop `-mtune=c906`; patch 5: four GCC-15
acceptance tokens, CFLAGS-only so the D-M8-1 `.c` source-identity claim
stands) plus the identical-both-sides iteration trim (patch 6,
`iterations=2->1`), applied after the ~2 h wall guard tripped on the
2-iteration run. The trimmed run **PASSed at 435,894.5 cycles** (13,441 s
wall; section 6). **FINAL PARITY SET (T6 output): {csr, interrupt, MMU,
coremark}** - these are the cases T4 ports and T7 compares.

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
| `coremark` | yes (patches 3+4+5) | yes (trimmed run, iterations=1) | `TEST PASS` | `../logical/tb/tb.v:269: $finish called at 43589450 (100ps)` | **435,894.5** | 13,441 s (3 h 44 min) |

Console (tb 0x10015000 watch) printed nothing for any case; every report
contains only the verdict string. `coremark` first error, verbatim
(`work/coremark_build.case.log`):

Before patch 4 (the `-mtune` wall, resolved by patch 4):

```
cc1: error: unknown cpu 'c906' for '-mtune'
make[2]: *** [Makefile:68: core_list_join.o] Error 1
```

After patch 4 (the second wall, first error, verbatim):

```
core_main.c: In function 'iterate':
core_main.c:79:3: error: implicit declaration of function 'printf' [-Wimplicit-function-declaration]
   79 |   printf ("\nVCUNT_SIM: CoreMark has been run %d times, one times cost %d cycles !\n",iterations,vcycles);
```

(Both walls above are cleared: patch 4 resolved the `-mtune` error, patch 5
the GCC-15 strictness errors; the coremark build then succeeded - section
4/6.)


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

### Patch 4 — `tests/lib/Makefile` lines 33 + 50 (drop `-mtune=c906`) — **APPLIED (controller-approved 2026-09-21)**

The coremark build (post patch 3) failed because **`-mtune=c906` is
rejected by standard gcc `cc1` for `.c` files**:

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
  it.
- No T-Head toolchain exists on the machine to accept it (`/opt` inventory:
  /opt/riscv 15.1.0, xpack 15.2.0, and an RV32-only c2rtl 9.2.0).

Applied change (same file, 2 lines, functionally neutral — `-mtune` is a
tuning hint with no effect on generated-code semantics; identical-both-sides:
T3's `test/m8/Makefile` coremark line simply omits `-mtune=c906`):

```diff
-FLAG_MARCH = -mtune=c906 
+FLAG_MARCH =  # -mtune=c906 (T6: vendor tune rejected by standard gcc cc1, see M8-T6 note)
```
(line 33; the trailing `#`-comment form follows T1's crt0 `#mxstatus`
traceability convention; make evaluates the value as empty) and on line 50
delete the `-mtune=c906 ` token (post-patch-3 state).

md5: `86c235ed3f8b0c7895751520dde19bbc` → `199be82a5e22921d843aeea1f38fa920`

Controller rationale (recorded): "tuning hint with no semantic effect, both
sides omit it (T3's test/m8 coremark line simply never includes it), same
class as T1's approved test-flow delta."

### Patch 5 - `tests/lib/Makefile` line 50 (four GCC-15 acceptance tokens) - **APPLIED (controller-approved 2026-09-21)**

After patch 4, the build advanced past `-mtune` and hit a **second,
distinct** wall: the coremark `.c` sources (and two clib files) were written
for the older T-Head GCC, where the constructs below were warnings; GCC ≥14
makes them **hard errors by default**:

1. **Implicit function declarations** (error by default in C90+ modes, GCC
   14+): `printf` called at `core_main.c:79` before any stdio include
   (the `#include "stdio.h"` in core_main.c is at `:126`, after the call);
   `sim_end()` at `core_main.c:85` (its declaration in `clib/vtimer.h:22`
   is present but `#include "vtimer.h"` is commented out at `core_main.c:28`);
   `ck_intc_init()` at `core_main.c:140` (no header declares it at all).
2. **int→pointer conversion without cast** (`-Wint-conversion`, error by
   default, GCC 14+): `clib/intc.c:26` `int *picr = TCIP_BASE;`
   (`TCIP_BASE 0xE0000000`), `clib/vtimer.c:35` `END_ADDR = 0x6000FFF8;`.
3. **`uint32_t`/`uint8_t` unknown in `clib/uart.h`** (it includes only
   quoted `"stdio.h"`, which does not pull in stdint): errors at
   `uart.h:63,72,73,74,89,96,104,112,119`.

**Measured fix — CFLAGS only, no case-source change.** Probed on
`/opt/riscv` 15.1.0 (the donor toolchain): adding exactly

```
-Wno-implicit-function-declaration -Wno-implicit-int -Wno-int-conversion -include stdint.h
```

to the coremark CFLAGS (line 50) makes **every** TU compile (verified:
all 7 coremark `.c` + `fputc.c` `intc.c` `uart.c` `vtimer.c` + newlib_wrap
`printf.c` `__thead_printf.c` `vasprintf.c` — 14 TUs, zero errors; the
residual `printf`/`align_mem`/`ITERATIONS-redefined` messages are
warnings only, and objects emit). Notes on each token:
- `-Wno-implicit-function-declaration -Wno-implicit-int`: restores the
  older-GCC behavior (declaration missing → warning). Empirically verified
  to downgrade the errors (probe: exit 0 with the flags, error without).
  The called functions exist and are linked from the case's own clib
  objects; on RV64 the implicit `int`-return assumption is harmless for
  the void-returning `sim_end()` (return value unused).
- `-Wno-int-conversion`: downgrades the two int→pointer initializations
  (verified: `intc.c`, `vtimer.c` compile clean). Both target unmapped
  donor addresses (0xE0000000 CLINT-ish, 0x6000FFF8 console) whose writes
  go to axi_err — invisible, and the donor flow has always compiled this
  way with the vendor toolchain.
- `-include stdint.h`: force-includes `<stdint.h>` at the top of every TU,
  supplying `uint32_t`/`uint8_t` to `uart.h`. Safe: `clib/datatype.h`
  typedefs `uint8_t`/`uint16_t` as the identical types (legal C11 re-types,
  and the probe confirmed no conflict).

**Pre-validated end-to-end to the ELF** (in /tmp scratch, zero
donor-tree writes): all 14 TUs compile, and the flow's **exact** link line
(`-Tlinker.lcf -nostartfiles -march=rv64imafdc_zicsr_zifencei
-mabi=lp64d -lc -lgcc <objs> -lm`, run with bare relative object names as
the flow's `make -C work` does) links clean → **`core_main.elf`,
91,888 bytes** (fits the linker.lcf MEM1 256 KB / MEM2 768 KB windows —
ld would have rejected an overflow). Only the `Srec2vmem` conversion and
the vvp run itself remain unverified — i.e. exactly the steps the §6
resume procedure covers. (Side finding from the link probe, for the
record: the flow's link works because the linker script's `crt0.o (.text)`
input reference matches the flow's `crt0.o` only when objects are passed
by bare relative name from `work/`; with absolute-path object names, ld
falls back to the `-L` search and pulls the toolchain's own
`/opt/riscv/riscv64-unknown-elf/lib/crt0.o` (with `_start` →
`__libc_init_array`/`exit` → the link fails). No action needed — the flow
always builds in `work/`.)

Applied change (1 line, same file, line 50, post-patch-4 state):

```diff
-  CFLAGS +=  -c -O3 -static -funroll-all-loops -finline-limit=500 -fgcse-sm -fno-schedule-insns --param max-rtl-if-conversion-unpredictable-cost=100 -fno-code-hoisting -DITERATIONS=10000
+  CFLAGS +=  -c -O3 -static -funroll-all-loops -finline-limit=500 -fgcse-sm -fno-schedule-insns --param max-rtl-if-conversion-unpredictable-cost=100 -fno-code-hoisting -DITERATIONS=10000 -Wno-implicit-function-declaration -Wno-implicit-int -Wno-int-conversion -include stdint.h
```

md5: `199be82a5e22921d843aeea1f38fa920` -> `7cd369b87bb96cd5e57c2304ff24aa3b` (applied 2026-09-21; line 50 now ends with the four tokens as proposed).

Controller ruling (recorded): same class as patches 3/4 (compiler-acceptance flags, no semantic effect, md5-ledgered); CFLAGS-only keeps the D-M8-1 `.c`-source-identity claim intact; identical-both-sides - T3 told to include the same four tokens in the rv906 coremark CFLAGS line. Executed the section-6 resume procedure from step 1; coremark then built cleanly (91,888-byte ELF, matching the pre-validation) and ran to the verdict in section 6.

### Patch 6 - `tests/cases/coremark/core_main.c:178` (identical-both-sides iteration trim) - **APPLIED**

Protocol (task recipe + section 6): the 2-iteration run was killed at the ~2 h wall guard with no `$finish`, so the fallback trim was applied - the one place the two sides' coremark is allowed to deviate, only identically on both sides (T4 applies the same trim on rv906):

```diff
-        results[0].iterations= 2;
+        results[0].iterations= 1;
```
(`:178`; the `#if CORE_DEBUG` line's `=1;` untouched - diff-verified: exactly one line changed.)

md5: `5cc9d76be5086852d941165b049485df` -> `bb0aae42c4104f5e9da1cbf723fba40c`. Before-image kept at `/tmp/m8_t6_core_main_before_trim.c`.

The trimmed run (iterations=1) PASSed - **this is the setting that produced the final coremark verdict** (section 6).

## 5. Deviations from the design doc

- **D1 — §2.3/§4.5 premise error (measured, this session):** "the remaining
  CFLAGS, incl. … are accepted by /opt/riscv 15.1.0" was measured against
  `.s` builds only; `-mtune=c906` (present in `FLAG_MARCH` at
  `Makefile:33` *and* at `:50`) is rejected by `cc1` on `.c` files on both
  toolchains (verbatim errors above). Consequence: coremark needed a 4th
  ledgered donor patch (applied, controller-approved 2026-09-21, §4) and
  T3's rv906 coremark CFLAGS line omits `-mtune=c906` (identical-both-sides,
  recorded). **Status: resolved** — the `-mtune` wall is cleared (rebuild
  advances past `core_list_join.o` into `core_main.o`).
- **D2 — data-driven, not a deviation: exception drops out.** Donor verdict
  is TEST FAIL ⇒ per D-M8-2 consequence clause the case drops from the
  parity set and T4 skips it (no port to a failing oracle).
- **D3 — second §4.5 premise error (measured after patch 4, this session):**
  "`.c` files byte-identical … CFLAGS (both sides): drop the 4 vendor
  flags" implies the coremark `.c` sources build once the vendor flags are
  gone. False: the sources target the older T-Head GCC; GCC ≥14 rejects
  their implicit function declarations (`core_main.c:79,85,140`) and
  int→pointer initializations (`clib/intc.c:26`, `clib/vtimer.c:35`) as
  hard errors, and `clib/uart.h` lacks a stdint include. Unlike D1, this is
  fixable **without touching any case source** (DONE: patch 5 applied 2026-09-21, controller-approved; coremark then built and PASSed, section 6) — four CFLAGS acceptance
  tokens (patch 5, section 4; pre-validated to a linked 91,888-byte ELF before application).
  (D-M8-1's `.c`-source-identity claim was in fact unaffected - the fix is CFLAGS-only; T3's rv906 coremark line carries the same four tokens, T3 told).

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

**Final verdict: TEST PASS - produced by the trimmed run (iterations=1),
435,894.5 cycles, 13,441 s wall.** The 2-iteration run never completed
(2 h guard trip), so iterations=1 is the setting both sides use (T4 must
apply the identical trim on the rv906 side).

Run history (all on the reused xuantie_core.vvp; no donor RTL changed):

| # | Setting | Wall | Outcome |
|---|---|---|---|
| 1 (invalidated) | iterations=2 | 122 s | **DISCARDED.** A concurrent session's `buildcase CASE=csr` (T3 donor-reproduce gate, 07:46:31 JST) overwrote `work/case.pat` between this run's build (07:45 JST) and its vvp start (07:49:19 JST), so vvp simulated the **csr** program: `$finish` at 273650 (100ps) - bit-identical to the csr run - and the report read "TEST PASSTEST PASS" (18 bytes = stale + new). Run 1's artifacts are not used. |
| 2 (isolated) | iterations=2 | 7,200 s (guard) | **2 h wall guard tripped** (`timeout 7200` kill, exit 124; no `$finish`). Sim healthy: 100% CPU across all 12 ten-minute samples (CPU time == elapsed). Pacing 23.4-25.3 cyc/s (small cases) => 2 iterations need well over the budget; the trim path was taken. |
| 3 (isolated, trimmed) | **iterations=1** | **13,441 s (3 h 44 min 1 s)** | **TEST PASS.** `$finish` at **43589450 (100ps)** = **435,894.5 cycles** (tb.v:269 PASS branch, 04:58:43Z). Effective pace 32.4 cyc/s. Report: fresh 9-byte `TEST PASS` file (no trailing newline, no concatenation - isolation verified). |

Isolation (runs 2-3): vvp executed from a private dir
(`/tmp/m8_t6_cm_isolated{,3}/`) with inst.pat/data.pat/case.pat copied there
after each build (freshness verified <300 s old); the tb's CWD-relative
`$readmemh` reads and its CWD-relative `run_case.report` write then stay
out of the shared `work/` dir, making a multi-hour sim immune to
concurrent `buildcase` runs (run 1's invalidation was exactly that hazard).
Post-hoc coremark analysis uses the isolated .pat copies, not work/.

Measurement notes:

- 1 iteration = 435,894.5 cycles => the design doc's 100K-500K-cycle
  (2-iteration) estimate was ~4x low; the 2-iteration run would have
  needed ~7-8 h (beyond even the 6 h hard cap) - the guard + trim was
  decisive, and the trimmed run finished in ~1/3 of the cap. The 6 h cap
  was never reached.
- Controller's independent donor MMU re-run (10:22-10:25 JST,
  `test/m8/run_donor.sh MMU`) confirmed TEST PASS, matching this note's
  4550.5-cycle record; that run left `work/` holding MMU artifacts - no
  effect on run 3, which loaded its image from the isolated dir.
- Trim md5 (patch 6): `5cc9d76be5086852d941165b049485df` ->
  `bb0aae42c4104f5e9da1cbf723fba40c`.

## 7. FINAL PARITY SET declaration (T6 output)

Donor-PASS cases measured in T6:

| Case | Donor verdict | In final parity set? |
|---|---|---|
| `csr` | TEST PASS (2736.5 cyc) | **yes** |
| `interrupt` | TEST PASS (3716.5 cyc) | **yes** |
| `MMU` | TEST PASS (4550.5 cyc) | **yes** |
| `exception` | TEST FAIL (R1, source-inherent) | **no — dropped** (recorded; T4 skips the port) |
| `coremark` | TEST PASS (trimmed run, 435,894.5 cyc) | **yes** |

**FINAL PARITY SET (T6 output, complete): {csr, interrupt, MMU, coremark}**
- all four TEST PASS on the donor (table above). T4 ports csr + MMU
(+ coremark, with the identical iterations=1 trim); interrupt is T5; the
exception port stays cancelled. T7's table covers exactly this four-case
set, both sides.

## 8. Artifacts left on disk

- Donor tree: the five patched files (md5s in section 4: 3 case-source files + the
  Makefile, through patch 6); `work/` holds the MMU re-run's artifacts
  (controller's 10:22 JST independent check - section 6) + the reused
  `xuantie_core.vvp` (untouched) and `work/coremark_build.case.log` (the
  last, successful, build log).
- /tmp (ephemeral): `m8_t6_{mmu,exception}_before.s`,
  `m8_t6_makefile_{before,after_p3}.mk`, `m8_t6_md5_{before,after}.txt`,
  `m8_t6_run_{csr,interrupt,MMU,exception}.log` (full vvp transcripts),
  `m8_t6_report_{csr,interrupt,MMU,exception}.txt` (byte-exact reports),
  `m8_t6_build_*.log` (coremark build attempts 1-5: 1/2 the two pre-patch-4/5 failures, 3 after patch 4, 4 after patch 5, 5 after the trim),
  `m8_t6_run_case.sh` (per-case runner),
  `m8_t6_cm_probe/` (14-TU probe objects + probe scripts
  `m8_t6_cm_probe.sh`, `m8_t6_cm_probe2.sh`),
  `m8_t6_linktest_dir/` (flow-exact link pre-validation: linked
  `core_main.elf`, 91,888 bytes), `tune_probe.c` / `tune_probe*.o`
  (the -mtune probes), `impdecl_probe.c` (the -Wno downgrading probe),
  `m8_t6_cm_run{1,2,3}.sh` + `m8_t6_cm_sample{2,3,4}.sh` (coremark
  run/sampler scripts), `m8_t6_cm_progress{,2,3}.log` (10-min ps samples;
  progress3 = the trimmed run, 12 samples all 100% CPU),
  `m8_t6_cm_isolated/` + `m8_t6_cm_isolated3/` (isolated run dirs: fresh
  .pat copies; isolated3 holds the byte-exact `run_case.report`
  `TEST PASS` + the 122s/2h/3h44m run evidence),
  `m8_t6_report_coremark.txt` + `m8_t6_run_coremark_final.log` (run 3
  report + console), `m8_t6_core_main_before_trim.c` (patch-6 before
  image), `m8_t6_makefile_after_p4.mk` (post-patch-4 Makefile image).
