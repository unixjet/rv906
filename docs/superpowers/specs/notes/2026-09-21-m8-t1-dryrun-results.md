# M8-T1 Dry-Run Results — Donor smart_run under iverilog (csr + 10-case sweep)

Date: 2026-09-21 (session started 2026-09-11; machine local time JST/UTC+9).
Supersedes the "trust but verify" items of
`2026-09-11-m8-crosscheck-exploration.md` §5/§6-R2 with measured results.

## Verdict

**DONOR FLOW FEASIBLE UNDER IVERILOG-12.** The openc906 `smart_run` flow
compiles the full C906 RTL + SoC testbench with iverilog 12.0 and executes
cases under `vvp`; the tb's magic-value exit sniffing works. Of the 11
`CASE_LIST` entries, the **2 cases buildable by any standard RISC-V
toolchain (`csr`, `interrupt`) both report `TEST PASS`**. The other 9 fail at
the *test-build* stage (never reach the simulator) because their sources use
T-Head CSRs/instructions or vendor-only gcc flags that no standard
gcc/binutils can encode — a toolchain limitation, not a flow or simulator
failure.

## 1. Environment

| Item | Value |
|---|---|
| Donor tree | `/home/vlsilab/.../rv906/refs/openc906/` (standalone clone of `https://github.com/XUANTIE-RV/openc906`, branch `main` @ `b0c06eb1f8b3bae663bd8b87eac89ff48e68a57f`) |
| iverilog | `Icarus Verilog version 12.0 (stable)`, `/usr/bin/iverilog` |
| Toolchain | `riscv64-unknown-elf-gcc (g1b306039ac4) 15.1.0`, `TOOL_EXTENSION=/opt/riscv/bin` |
| `CODE_BASE_PATH` | `/home/vlsilab/.../refs/openc906/C906_RTL_FACTORY` |
| Machine | 32-core Linux; flow is sequential |
| Compiled design | 293 files / ~154,555 lines: gen_rtl filelists (260 files, 141,483 lines, incl. `.h`) + `logical/` SoC+tb (33 files, ~13,072 lines) |

## 2. Donor tree baseline and changes

Baseline (2026-09-11, session start): clone at `b0c06eb` on `main`,
tracking `origin/main`. Pre-patch checksums recorded for every file touched:
`tests/lib/Makefile` `60f1bd5fa4b63b3bf8a09e243bcb581d`,
`tests/lib/crt0.s` (via `/tmp/crt0.s.before`), `tests/bin/Srec2vmem`
`9a12b1e2b8e40259afa918f14c38c557` (unchanged content, mode only).

**No RTL was modified. No git commits made.** Exactly 3 source-side changes
(all under `smart_run/`, i.e. test-flow tooling, not RTL):

### Patch 1 — `smart_run/tests/lib/Makefile` line 40 (march override)

```diff
   ifeq (${CPU_ARCH_FLAG_0}, c906fd)
-    FLAG_MARCH += -march=rv64imafdcxtheadc
+    FLAG_MARCH += -march=rv64imafdc_zicsr_zifencei
     FLAG_ABI   = -mabi=lp64d
```

Why: gcc 15.1 rejects `xtheadc` ("unsupported non-standard extension");
`xtheadcsr` also rejected. The `c906fd` branch is the one every case uses
(`smart_cfg.mk` pins `CPU_ARCH_FLAG_0=c906fd` on all build recipes, and the
value is hard-coded per-recipe — no variable passthrough, so a file edit is
the only route). `_zicsr` is required for the CSR instructions in this
binutils generation; `_zifencei` for `fence.i` (non-canonical lowercase
march strings do not imply them; the `cache` case's first error proved the
need). `-mtune=c906` (line 33) is accepted by gcc 15.1 and left alone.
Final md5 `9e245e1542f6f23104533468ef8480d4`.

### Patch 2 — `smart_run/tests/lib/crt0.s` line 31 (named CSR → numeric)

```diff
 # enable extension
   li   x3, 0x400000
-  csrs mxstatus,x3
+  csrs 0x7c0,x3  #mxstatus
```

Why: upstream binutils doesn't know the T-Head CSR *name* `mxstatus`; its
numeric address is 0x7c0, so this encodes the identical CSR write. This is
**not** a semantic "fix" of the crt0 — the poke is unchanged (donor RTL
implements mxstatus 0x7C0). All other crt0 CSRs (`mcor` 0x7c2, `mhcr` 0x7c1,
`mhint` 0x7c5) are already written numerically by the donor and needed no
touch. Final md5 `002084ed36e9b16c78cb04354fd24859`.

### Patch 3 — `smart_run/tests/bin/Srec2vmem` (mode only)

`chmod +x` — the vendored static x86-64 binary (valid ELF, 1.35 MB) lost its
exec bit in checkout; `buildcase` died with `Permission denied` (exit 126)
before this. Content checksum unchanged.

### Runtime-only requirements (no file edits)

1. `SHELL=/bin/bash` on every make invocation — `/bin/sh` is dash here and
   the `smart_cfg.mk` recipes use `>& logfile` redirection, which dash
   rejects ("Bad fd number").
2. `mkdir -p smart_run/work` — a fresh clone has no `work/` dir; the
   `cleancase`/`compile` targets `cd` into it.
3. Env: `CODE_BASE_PATH` + `TOOL_EXTENSION` exported (flow warns without them).

## 3. Commands (exact)

```sh
cd refs/openc906/smart_run
export CODE_BASE_PATH=.../refs/openc906/C906_RTL_FACTORY
export TOOL_EXTENSION=/opt/riscv/bin
mkdir -p work
make compile SIM=iverilog SHELL=/bin/bash          # iverilog -> work/xuantie_core.vvp
make buildcase CASE=csr SIM=iverilog SHELL=/bin/bash
cd work && vvp xuantie_core.vvp                     # -> work/run_case.report
```

(The Makefile's `runcase` = compile + buildcase + `vvp -l run.iverilog.log`;
`-l` produced no file under iverilog 12, so stdout was captured instead.
`runcase` re-runs `compile` per case; the sweep reused the single
`xuantie_core.vvp` and did buildcase+vvp per case, which is the same
sequence minus the redundant recompile.)

## 4. Results per case (CASE_LIST, 11 entries)

| Case | Built? | Simulated? | Verdict | Blocker (first assembler/gcc error, verbatim) |
|---|---|---|---|---|
| `csr` | yes | yes | **TEST PASS** | — |
| `interrupt` | yes | yes | **TEST PASS** | — |
| `coremark` | **no** | — | not run | `error: unrecognized command-line option '-msignedness-cmpiv'` (+ `-mno-thread-jumps1`, `-mno-iv-adjust-addr-cost`, `-mno-expand-split-imm` — T-Head-vendor gcc flags at `tests/lib/Makefile:50`) |
| `ISA_THEAD` | no | — | not run | `unrecognized opcode 'fsurw f10,x3,x4,3'` (XThead FP indexed stores; also `addsl`, `mxstatus`) |
| `ISA_INT` | no | — | not run | `unrecognized opcode 'surw x8,x4,x5,3'` (XThead indexed stores; also `srb/srw/…`) |
| `ISA_LS` | no | — | not run | `unrecognized opcode 'dcache.cva x22'` (XTheadCmo; also named `mhcr`/`mxstatus`) |
| `ISA_FP` | no | — | not run | `unknown CSR 'fxcr'` (T-Head FPU control CSR; also `.h` Zfh ops) |
| `MMU` | no | — | not run | `unknown CSR 'mxstatus'` (line 246; only blocker — rest is standard) |
| `exception` | no | — | not run | `unrecognized opcode 'dcache.ciall'` (line 161); `unknown CSR 'mhcr'` (line 164) |
| `cache` | no | — | not run | `unrecognized opcode 'dcache.cva x10'` (also `civa/ipa/cpa/cipa`; named `mhcr`/`mcor` CSRs) |
| `debug` | no | — | not run | `unknown CSR 'mhcr'` (lines 22-23); additionally needs a separate iverilog compile with `JTAG_DRV.vh`+`C906_DEBUG_PATTERN.v` (JTAG force-driver tb) — not attempted, moot while the program can't build |

Notes:
- The 9 build failures are all *source-vs-toolchain* mismatches (XThead
  CSRs/instructions, vendor compiler flags). They match the exploration
  note's R1 prediction; none indicates a problem with the iverilog flow.
- `MMU` and `exception` are otherwise fully standard and would become
  buildable with a 1-3 line name→address patch (like crt0's) — left
  unpatched here by the "donor case sources are the oracle" rule; flagged
  as the cheapest expansion of the donor-side set if M8 wants it.
- `coremark` would build if the 4 vendor `-m*` tuning flags were dropped
  from the `CASENAME=coremark` CFLAGS line (semantically neutral for
  instruction selection under a standard compiler, per exploration note
  §3) — a M8 decision, not applied here.
- `debug` is M7 territory per the exploration note; excluded from M8
  functional parity.

## 5. Timings and evidence

| Step | Wall time | Evidence |
|---|---|---|
| `make compile SIM=iverilog` (293 files / ~154.5K lines → `work/xuantie_core.vvp`, 57,808,961 bytes) | **2 s** (1789933020→1789933022) | clean log (iverilog silent on success); `COMPILE_START/END` stamps |
| `buildcase csr` | ~10 s | `work/csr_build.case.log` empty on success |
| `vvp` csr | **116 s** | `$finish` at sim time 273650×100 ps = 27.365 µs = **2736.5 cycles** @10 ns; report `TEST PASS` |
| `buildcase interrupt` | ~10 s | `work/interrupt_build.case.log` empty |
| `vvp` interrupt | **147 s** | `$finish` at 371650×100 ps = **3716.5 cycles**; report `TEST PASS` |

Report excerpts (byte-exact, `work/run_case.report`, no trailing newline):

```
TEST PASS
```

tb console for both runs:

```
 ********* Init Program *********
 ********* Wipe memory to 0 *********
 ********* Read program *********
 ********* Load program to memory *********
 **********************************************
 *    simulation finished successfully        *
 **********************************************
 ../logical/tb/tb.v:269: $finish called at 273650 (100ps)     # csr
 ../logical/tb/tb.v:269: $finish called at 371650 (100ps)     # interrupt
```

`interrupt` is the stronger result: it exercises the in-core PLIC at
0x4000000000 (threshold/prio/enable/pending registers), a real M-mode
interrupt delivery through `wfi`, claim/complete via the PLIC claim
register, and `mret` — all on donor RTL under iverilog.

## 6. Artifacts left on disk

- Donor tree (expected, per flow): `smart_run/work/` — `xuantie_core.vvp`
  (57.8 MB), `interrupt` case artifacts (`.s/.o/.elf/.hex/.obj/.pat`),
  `run_case.report` (`TEST PASS`, from the last run = interrupt), the two
  patched files, `Srec2vmem` with +x. The 3 patched/modified files are the
  complete source-side delta (see §2; before-images saved at
  `/tmp/Makefile_lib.before`, `/tmp/crt0.s.before`).
- Logs (in /tmp, ephemeral): `m8_compile.log` (iverilog compile),
  `m8_csr_final_run.log`, `m8_interrupt_final_run.log` (full per-run
  transcripts with UTC stamps), `m8_builddown.log` (the 9-case build
  sweep), `m8_csr_report.txt` / `m8_interrupt_report.txt` (saved reports).

## 7. Implications for the M8 design

1. **R2 (donor-side sim bring-up) is de-risked**: iverilog compiles gen_rtl
   in seconds and runs cases in ~2 min wall each for these small tests.
   Donor-side cycle capture needs only the 2-line `$display` patch at the
   PASS branch (rv12 P4 pattern); the tb already prints `$finish` sim time,
   which at 10 ns/clock is the cycle count (2736.5 / 3716.5 here).
2. **Donor-side case set = {csr, interrupt} as-is.** Expanding requires
   name→address CSR patches in case sources (MMU, exception: 1-3 lines
   each) — changes to the oracle sources that M8-T0 should accept/reject.
   The XThead-heavy cases (ISA_THEAD, ISA_INT, ISA_LS, ISA_FP, cache) and
   coremark (vendor flags) remain unbuildable with any standard toolchain
   present; that is the same wall the rv906 side hits (R1), so it does not
   shrink the *parity* set, only the *donor-observable* set.
3. No RTL patches, no RTL edits, no commits: donor RTL ran byte-as-shipped.
