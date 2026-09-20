# M8: Cross-Check vs Original openc906 RTL — Design (Task 0)

Date: 2026-09-21
Status: FINAL — pins every decision the M8 implementation tasks (T2–T7) need.
Parent: `docs/superpowers/specs/2026-08-20-rv906-c906-clone-design.md:318` —
verbatim umbrella row:

> | M8 | cross-check vs original openc906 RTL: same smart_run (non-vector) binaries on both simulators | functional parity; cycle-count comparison noted, not gated |

Non-goals carried from the umbrella doc (:367–369, :375):
"Cycle-level fidelity is not a goal: parity is functional/structural";
"Cycle-exact equivalence with the original RTL" (non-goal). Vector cases
excluded (:366).

Inputs (all read, all claims below re-verified against the tree on 2026-09-21):

1. `docs/superpowers/specs/notes/2026-09-11-m8-crosscheck-exploration.md` —
   T0 exploration (flow census, donor SoC map, risks R1–R8, task shape T0–T8).
2. `docs/superpowers/specs/notes/2026-09-21-m8-t1-dryrun-results.md` — **T1
   (DONE, committed 81351a5)**: measured donor-side bring-up under iverilog-12.
3. `/home/vlsilab/zhouz/workspace/C2RTL/rvproc/RVProc6/vla/riscv/rv12/docs/superpowers/specs/2026-09-11-m8-crosscheck-design.md` —
   rv12's M8 spec (process precedent; its P1–P6 / D-M8-1..5 shape).
4. `docs/superpowers/specs/2026-09-11-m7-debug-design.md` — structure
   template, and the blocker: M7 is open in this worktree (Tasks 1, 4 landed;
   `rtl/CSR.v`, `rtl/DTU.v`, `rtl/IFU.v`, `rtl/rvproc_pkg.sv` carry in-flight
   M7 edits; `rtl/TDT_DM.v`, `test/m7/dm_tb.cpp` untracked). **T2 is blocked on
   M7 Task 10 (acceptance + close-out)** because T2 edits `rtl/CSR.v`.

Line-number note: all rv906 line numbers are as of 2026-09-21 with the
in-flight M7 edits applied. `rtl/CSR.v` will move again before T2 runs; T2
must re-locate the `CSR_TIME` arm by symbol, not by line.

---

## 1. Context — what M8 is, and what T1 proved

M8 runs the same smart_run non-vector case programs on **both** the donor
openc906 RTL (iverilog, `smart_run` flow) and rv906 (Verilator, `testbench`)
and gates on **functional parity**: each case in the final set reports the
same verdict (PASS) on both sides. Cycle counts are captured both sides and
reported in the close-out table; they are never a gate.

**T1 measured results** (dry-run note, authoritative):

- Donor flow is feasible under iverilog-12: full compile 2 s
  (293 files / ~154.5K lines → `smart_run/work/xuantie_core.vvp`,
  57,808,961 bytes — **still on disk, T6 reuses it, no recompile**);
  per-case `vvp` wall time ~2 min for the small tests.
- Donor-side verdicts: `csr` **TEST PASS** ($finish at 273650×100 ps →
  2736.5 cycles @10 ns), `interrupt` **TEST PASS** (3716.5 cycles)
  (`smart_run/logical/tb/tb.v:266,269`; Icarus prints `$finish called at
  273650 (100ps)` itself).
- Of the 11 `CASE_LIST` entries (`smart_run/setup/smart_cfg.mk:18-30`), 9 fail
  at **test-build** (never reach the simulator): XThead CSRs/instructions
  (`dcache.ciall`, `mxstatus`, `mhcr`, `fxcr`, `fsurw`, `surw`, …) or
  vendor-only gcc flags (`-msignedness-cmpiv` etc.,
  `smart_run/tests/lib/Makefile:50`). Toolchain limitation, not a flow
  failure — the same wall rv906 hits (no T-Head toolchain in either tree).
- Donor-side delta to get there: exactly 3 files, all under `smart_run/`
  (tooling, not RTL), recorded with before/after md5 in the T1 note
  (§2): `tests/lib/Makefile:40` (march →
  `rv64imafdc_zicsr_zifencei`), `tests/lib/crt0.s:31` (`mxstatus` → `0x7c0`),
  `tests/bin/Srec2vmem` (+x only). **The donor tree `refs/openc906/` is
  read-only from here on; T6 may add further name→numeric patches to DONOR
  case sources only if D-M8-2(a) is invoked — see §4.2–§4.3; each is md5-ledgered.**

What M8 actually changes in rv906:

- **RTL: one read arm.** `rtl/CSR.v:1651` `CSR_TIME: csr_read_mux = mtime;`
  → `csr_read_mux = mcycle_reg;` (D-M8-5). No other RTL changes.
- **Test scaffolding: new `test/m8/`** (Makefile, ported crt0, ported link
  script, ported clib, 5 case ports, `run_all.sh`) — mirrors `test/m6/`
  structure.
- **Tooling: `test/m8/run_donor.sh`** (wraps the donor flow, reuses
  `xuantie_core.vvp`) and **`test/m8/compare.py`** (verdict/cycle table).

---

## 2. Verified current state (anchors both sides)

### 2.1 rv906 side

| Fact | Anchor |
|---|---|
| Sysmap: MEM@0x80000000 (2 GB), CLINT@0x02000000, PLIC@0x0C000000, UART@0x10000000; `SI_MEM=0/SI_CLINT=1/SI_PLIC=2/SI_UART=3` | `rtl/RVProcAXI.v:152-159` |
| **Unmapped addresses route to MEM** (`AXICrossbar .DEFAULT_SLAVE (SI_MEM)`, 64-bit addressing) — an "out-of-window" access on rv906 is a silent MEM hit, never a bus error | `rtl/RVProcAXI.v:749-760` |
| CLINT `mtime` = CPU clk ÷ 100 (tick at `rtc_div==7'd99`); feeds the `time` CSR only | `rtl/RVProcAXI.v:374-393` |
| UART IRQ = PLIC source 7 (`.int_src({G_io_pins_uart_irq, 7'b0})`); 16550 IER bit1 (THRE) drives the level | `rtl/RVProcAXI.v:680`, `dut.cpp:80-85`, `device/uart16550.cpp:101-104` |
| 16550 THR (offset 0) → console out | `device/uart16550.cpp:66-71` |
| `time` (0xC01) read arm = `mtime`; **no `mtime` (0xB01) CSR arm exists** — mtime is MMIO-only, `time` is the mirror | `rtl/CSR.v:1651` (arm), `rtl/rvproc_pkg.sv:315` |
| `mcycle_reg`: 64-bit, free-runs per cycle, writable via mcycle CSR (donor-compatible) | `rtl/CSR.v:1255-1268` |
| Counter read arms: MCYCLE/MINSTRET/CYCLE/TIME/INSTRET | `rtl/CSR.v:1648-1652` |
| **Unknown CSR: read → 0 (`default: csr_read_mux = 64'd0`); write → dropped (no local_en); NO trap** — `csr_access_illegal = csr_priv_bad \|\| csr_ro_write \|\| satp_tvm_illegal \|\| csr_fp_illegal` has no unknown-CSR term; IDU CSR decode has no address whitelist | `rtl/CSR.v:1683`, `rtl/CSR.v:1795-1796`, `rtl/IDU.v:839-901` |
| T-Head CSRs present: MXSTATUS 0x7C0 (only `mm` bit15 modeled, reset 1), MHCR 0x7C1, MHINT 0x7C5; **no MCOR 0x7C2, no TIMEH, no MCINDEX/MCINS/MCDATA** | `rtl/rvproc_pkg.sv:280-281,284`, `rtl/CSR.v:1453-1463,1411-1443,1465-1520` |
| M-mode MMU bypass (VA=PA): `*_mmu_en = sv39_en && (priv != PRIV_M)` | `rtl/MMU.v:226-227` (donor analog: `aq_mmu_ptw.v` `PTW_MACH_PMP` state) |
| Sv39 PTE: only bits 0–7 + PPN[10:49] extracted; **PTE[62:59] THEADFLAG ignored** | `rtl/MMU.v:547-550` |
| No MAEE/PTE-encoded PMA — "rv906 PMA always comes from its own sysmap table" | `rtl/MMU.v:679-681` |
| Misaligned data access: trap at issue (cause 4/6), no mm-gated bypass; documented ma_data fail 86/87 | `rtl/LSU.v:17`, `rtl/LSU.v:1076-1084`, `docs/08-verification.md:29` |
| PMP: 8 entries, 4 KB grain; **M-mode bypasses PMP unless the matching entry is L-locked** (data and fetch channels both); no-hit → M allow, S/U deny | `rtl/PMP.v:277-278,314-333` (donor: `aq_pmp_acc.v:289`) |
| **PMP NAPOT quirk (donor-inherited)**: an invalid NAPOT encoding (no trailing ones) falls to `default: napot_mask_fn = 29'h00000000` → mask 0 → the entry **matches every address** | `rtl/PMP.v:176-214,248` (donor: `gen_rtl/pmp/rtl/aq_pmp_comp_hit.v:108-143,111`) |
| PLIC: 8 sources, 3-bit prio (prio[id]=0x4·id), 0x1000 pending (RO), 0x2000 enable, 0x200000 threshold, 0x200004 claim/complete, 1 M-mode context | `rtl/PLIC.v:9-17` |
| tohost protocol: ELF exports `tohost`/`fromhost`; harness polls by symbol; `out & 1` → stop; `result==1` → PASS else FAIL, testno = result>>1; `0x01010000` char protocol; cycle counter printed every 1 M cycles | `testbench/TestBench.cpp:188-207,239-245,256-285,343-356` |
| Exit encoding: `tohost = (code<<1)|1` (PASS code 0 → 1) | `test/entry.S:183-190,204-212` |
| `.tohost` at 0x7FFFF000 (uncached aperture below the image); image at 0x80000000 | `test/m2/common.ld:38-39` |
| M6 toolchain: xpack gcc 15.2.0, `-march=rv64imac_zicsr -mabi=lp64 -O2`, `run_all.sh` = `timeout 300 bin/verisim/testbench --print-result <elf> | grep -q PASS` | `test/m6/Makefile:31,35`, `test/m6/run_all.sh:21-28` |
| M6 interrupt pattern (the template for the M8 interrupt port): PLIC prio[7]@0x0C00001C, enable bit7@0x0C002000, threshold@0x0C200000, claim/complete@0x0C200004; UART IER@0x10000004=2 asserts, deassert before complete | `test/m6/plic_uart.S` |
| `mtip.S` arms `mtimecmp = time+1` and waits MTIP — **sensitive to D-M8-5** | `test/m6/mtip.S:35-38,84-86` |
| No ISS-diff machinery exists in rv906 (no m5_iss/m6_iss; `m1_iss.h` has no CSR model) → D-M8-5 needs no ISS adoption | (verified absent) |
| RV906_BOOT_MHCR preamble pokes mhcr=0x3 (IE+DE) in the directed suites; reset MHCR=0 | `test/m2/Makefile:38-44`, `test/m2/env/riscv_test.h:196-222` |

### 2.2 Donor side (read-only, `refs/openc906/`)

| Fact | Anchor |
|---|---|
| `CASE_LIST` 11 entries; `CPU_ARCH_FLAG_0=c906fd` pinned per recipe | `smart_run/setup/smart_cfg.mk:17-30` |
| Donor case build flags (T1-patched): `-march=rv64imafdc_zicsr_zifencei -mabi=lp64d`; others `-O2`; **coremark**: `-O3 … -msignedness-cmpiv -fno-code-hoisting -mno-thread-jumps1 -mno-iv-adjust-addr-cost -mno-expand-split-imm -DITERATIONS=10000` | `smart_run/tests/lib/Makefile:40,50-52` |
| Donor link: MEM1 0x0/0x40000 (text), MEM2 0x40000/0xC0000 (data), `__kernel_stack`=0xee000 | `smart_run/tests/lib/linker.lcf` |
| Donor crt0 (post-T1): mxstatus 0x7c0=0x400000 (`:30-31`), mstatus 0x802000 (`:34-35`), MIE (`:119-120`), mcor 0x7c2=0x30013 (`:124-125`), mhcr 0x7c1=0x7f (`:128-129`), mhint 0x7c5=0x610c (`:135-136`), `jal main` (`:139`), `__exit` x3=0x444333222 (`:144-150`), `__fail` x3=0x2382348720 (`:153-157`), trap handler + 128-entry vector_table→`__fail` (`:160-230`) | `smart_run/tests/lib/crt0.s` |
| Donor tb: 1 ns/100 ps, CLK_PERIOD=10 (`:20-21`); MAX_RUN_TIME=700000000 (100 ps) = 7 M-cycle watchdog (`:24`); `cycle_count` free-runs but is never printed (`:202-211`); PASS→`TEST PASS`+`$finish` (`:266,269`); FAIL→`TEST FAIL` (`:278`); console watch at 0x10015000 (`:285`) | `smart_run/logical/tb/tb.v` |
| Donor verdict mechanism: magic 0x444333222 / 0x2382348720 in x3 sniffed on the writeback bus | `smart_run/logical/tb/tb.v:266-285` |
| Donor SoC: SRAM 0x0/0x100000, console 0x10015000, **in-core PLIC 0x4000000000**, everything else unmapped → axi_err | T1 note §1 (re-verified in flow); SoC map table in the exploration note |
| Donor PLIC register map used by the interrupt case: base 0x4000000000, prios 0x0+, INTPEND 0x1000 (**writable test backdoor** — the case `sw 0x2` to set source-1 pending; the case PASSES on the donor (T1), so the write is live), INTIE 0x2000, INTIE_HART 0x80, INTTH 0x200000, INTCLAIM 0x200004 (rv906's analog pending reg is RO, `rtl/PLIC.v:9-17`) | `smart_run/tests/cases/interrupt/C906_plic_int_smoke.s:24-33,120-126` |
| Donor PMP: `pmp_default_flg = cp0_mach_mode ? 4'b0111 : 4'b0` (M-mode default-allow) | `gen_rtl/pmp/rtl/aq_pmp_acc.v:289` |
| Donor console for coremark: `fputc` → `li x13,0x6000fff8; sw` → unmapped → axi_err → **invisible** | `smart_run/tests/lib/clib/fputc.c:19` |
| Donor `get_vtimer` = `csrr time` (32-bit) — coremark's self-measurement | `smart_run/tests/lib/clib/vtimer.c:15-30` |
| coremark: **`results[0].iterations = 2` is hard-coded**; the Makefile's `-DITERATIONS=10000` is vestigial (only file that uses the macro, `core_portme.c:25-26`, redefines it to 1 locally); console = `printf`→`fputc` (above); `UART0_BASE_ADDR 0x40015000` driver in clib is compiled but not called by coremark | `smart_run/tests/cases/coremark/core_main.c:178,78-84`, `core_portme.c:25-26`, `smart_run/tests/lib/clib/uart.h:23` |

### 2.3 Toolchain measurements (this session, /tmp probes)

- `/opt/riscv/bin/riscv64-unknown-elf-gcc` 15.1.0: `-fno-code-hoisting`
  **accepted** (exit 0) → for coremark only the 4 `-m*` vendor flags
  (`-msignedness-cmpiv -mno-thread-jumps1 -mno-iv-adjust-addr-cost
  -mno-expand-split-imm`) must be dropped; `-fno-code-hoisting`,
  `--param max-rtl-if-conversion-unpredictable-cost=100` are standard and
  stay (source identity of the CFLAGS line otherwise).
- xpack 15.2.0 `riscv-none-elf-as`: rejects the CSR **names** `mxstatus` and
  `mhcr` (exit 1) → name→numeric patches are needed on **both** sides
  (donor's /opt/riscv 15.1.0 is the same, T1).

### 2.4 Corrections to the exploration note (R1–R8)

- **R4 correction (verified wrong):** the note claimed an unknown CSR write
  (e.g. `csrs 0x7c2` mcor) would trap illegal-CSR on rv906 and that the
  ported crt0 must drop the mcor poke. **False**: rv906 reads unknown CSRs as
  0 (`rtl/CSR.v:1683`), drops unknown writes (no local_en), and has no
  unknown-CSR trap term (`rtl/CSR.v:1795-1796`; `rtl/IDU.v:839-901` no
  whitelist). The ported crt0 **keeps the mcor poke** for source identity
  (it is a no-op) — matching rv12's D-M8-3 "issue the same csrs, unmodeled
  bits are no-ops".
- **R2 de-risked:** measured feasible (T1).
- **Coremark sizing premise corrected:** the brief/exploration assumed
  `-DITERATIONS=10000` scales the run. It does not — the loop count is the
  hard-coded 2 (`core_main.c:178`). The donor wall-time question is
  answered by the first T6 run, not by a flag (§4.5, R2).

---

## 3. Decisions

### D-M8-1 — "same binaries" = source-identical programs + per-platform glue

**Settled.** Byte-identical ELFs are impossible (different memory maps,
exit mechanisms, and no T-Head toolchain in either tree; a standard toolchain
emits no XThead regardless of flags). The contract is:

1. **Same case sources**: each case's `.s`/`.c` body is byte-identical
   between `refs/openc906/smart_run/tests/cases/<case>/` and
   `test/m8/cases/<case>/` **except** for the per-case deltas listed in §4,
   each recorded in the deviation ledger (§9) with file, line, donor value,
   rv906 value, and reason.
2. **Same march**: both sides compile with
   `-march=rv64imafdc_zicsr_zifencei -mabi=lp64d` — literally the T1-patched
   donor string (`smart_run/tests/lib/Makefile:40`), so instruction
   selection policy is identical. rv906 uses the xpack 15.2.0 toolchain
   (`test/m6/Makefile:31` pattern); the donor uses /opt/riscv 15.1.0. GCC
   minor-version skew is accepted (both standard binutils; no vendor
   extensions in the code after the §4 deltas).
3. **Per-platform glue** (never part of the case body): crt0 (ported, §4.6),
   link script (ported), clib (ported for coremark), harness/tb.
4. **Per-case source-identity level** (final, after §4 deltas):

| Case | Body identity |
|---|---|
| csr | **byte-identical** (standard CSR ops only) |
| exception | identical except: 1 dropped line, 1 name→numeric (both sides), satp-PPN remap + ACCERR remap (rv906 only, §4.2) |
| MMU | identical except: 6 name→numeric (both sides), 2 PPN constants (rv906 only, §4.3) |
| interrupt | **restructured init** (rv906 only, §4.4) — claim/complete/wfi/`mret`/vector_table structure kept |
| coremark | `.c` files byte-identical; glue-only deltas (fputc target, CFLAGS 4 flags, both sides §4.5) |

### D-M8-2 — Donor-observable set

**Settled.** Baseline donor-observable set = **{csr, interrupt}** (measured
PASS, T1). Expansions:

- **(a) ACCEPT: MMU + exception via name→numeric patches to donor case
  sources.** Exactly 7 instruction lines across 2 files, all address
  encoding only (no semantic change; donor RTL implements the same CSR
  numbers):
  - `tests/cases/MMU/C906_mmu_basic.s`: `mxstatus` → `0x7c0` at 6 sites
    (macro bodies `:152-161` and `:242-251`).
  - `tests/cases/exception/C906_Exception.s`: `csrci mhcr,0x2` → `csrci
    0x7c1,0x2` (`:164`). (`dcache.ciall` at `:161` is **not** patchable —
    it has no numeric form in standard binutils; see D-M8-3: the donor-side
    exception build keeps that line **commented out**, recorded as a
    donor-side deviation; on the donor, cache flush in M-mode with caches
    enabled is a performance hint, not a functional dependency — the
    subtests that follow re-read through the cache hierarchy and the case's
    checks are on trap CSRs, not data; T6's run confirms the donor verdict
    is unaffected, and if it is not, exception drops out per D-M8-7.)
  - Each patched file: before/after md5 recorded in the ledger (§9), same
    discipline as T1's 3-file delta.
- **(b) ACCEPT: coremark by dropping the 4 vendor `-m*` flags** from
  `smart_run/tests/lib/Makefile:50` (measured: the remaining CFLAGS, incl.
  `-fno-code-hoisting` and the `--param`, are accepted by /opt/riscv 15.1.0).
  `-DITERATIONS=10000` stays (harmless, keeps the CFLAGS line otherwise
  byte-identical). Donor console is invisible (fputc→0x6000fff8→axi_err,
  `clib/fputc.c:19`), so **coremark parity = verdict + cycles only** —
  recorded, not a gap.
- **REJECT: nothing else.** The 4 remaining XThead-heavy cases (ISA_THEAD,
  ISA_INT, ISA_LS, ISA_FP, cache) stay unbuildable on both sides (R1); debug
  is M7 territory (JTAG force-driver tb, separate compile).

**Consequence for D-M8-7:** the *final* parity case set is defined as the
cases that PASS on the **donor** side in T6 — {csr, interrupt} ∪ {MMU,
exception, coremark} ∩ (T6 verdicts). The rv906 side ports exactly that set.
If T6 shows donor-side FAIL for MMU or exception, those cases drop out of the
parity set (recorded) rather than being ported to a failing oracle.

### D-M8-3 — rv906-side case set + per-case port deltas

**Settled.** Case set (pending T6 donor verdicts per D-M8-2):
{csr, exception, MMU, interrupt, coremark}. Deltas in §4; the mechanisms
below are pinned, the exact surviving subtest set for exception is
data-driven from T6 (§4.2).

### D-M8-4 — Excluded set; T6 (filtered standard-subset smokes) DROPPED

**Settled.** Excluded from M8 entirely:
- `ISA_THEAD, ISA_INT, ISA_LS, ISA_FP, cache` — XThead surface rv906
  deliberately does not implement (umbrella scope; unbuildable on both
  sides, R1).
- `debug` — M7 territory; JTAG-driven force-driver tb, separate iverilog
  compile; M7 has its own `dm_tb` gate.
- Vector cases (umbrella :366).

**T6 (the exploration note's "filtered standard-subset smokes") is DROPPED.**
Rationale: the standard-ISA surface is already gated by the M2 (rv64ui/um/uc),
M4 (priv/MMU/PMP directed set, `docs/08-verification.md:108-123`), M5 and
M6 suites, which must stay green after M8 (D-M8-7 floor). Filtered XThead
smokes would be **weaker duplicates** of those suites (they test the subset
of the XThead cases that compiles under a standard toolchain — i.e. less
than the M2/M4 coverage already provides). The 5 real cases in D-M8-3 give
the cross-check its value (they exercise the *donor's* test programs,
including donor crt0/crt0-pokes, donor PLIC structure, and Sv39 in S-mode).

### D-M8-5 — `time` CSR re-point (task T2)

**Settled.** Change: `rtl/CSR.v:1651` —
`CSR_TIME: csr_read_mux = mtime;` → `csr_read_mux = mcycle_reg;` (one line;
re-locate by symbol at T2 time — the file moves with M7 close-out).

Rationale:
- Donor C906 `time` increments **per cycle**; rv906's current `time` is the
  CLINT `mtime` mirror at clk/100 (`rtl/RVProcAXI.v:374-393`). Re-pointing to
  `mcycle_reg` (free-running per cycle, writable via mcycle —
  `rtl/CSR.v:1255-1268`) makes the two sides' `time` semantically
  comparable, which is what CoreMark's `vcycles` self-measurement
  (`clib/vtimer.c:18`, printed as `VCUNT_SIM`, `core_main.c:78-84`) needs
  for the cycle column of the T7 table.
- `mcycle_reg` is the correct arm, not a new counter: the donor's `time` is
  the mcycle alias; using the same register preserves write semantics.
- Scope: **read arm only.** No `timeh` (0xC81) — the case sources use
  32-bit `csrr` (vtimer.c), rv906 is the M2–M8 functional surface, and
  adding timeh is datapath growth M8 does not need. No mtime change. No ISS
  adoption (there is no ISS to update — verified absent).
- **Sequencing: T2 runs AFTER M7 Task 10** (M7 is editing `rtl/CSR.v` now;
  two writers on one file is how milestones die). T2's gate includes the
  known-sensitive test below.
- **Known interaction (measured by analysis, gate-verified at T2):**
  `test/m6/mtip.S:35-38` arms `mtimecmp = time + 1`. After the re-point,
  `time` = mcycle (≈100× larger than mtime at the same instant), so the
  armed mtimecmp value jumps to a mcycle-scale number that the clk/100
  `mtime` mirror needs ~100× as long to reach. The test still PASSES (it
  only checks `time >= armed` after MTIP) but waits ~`0.9·C` extra mtime
  ticks ≈ `90·C` extra cycles where C is the mcycle at arm time. C is small
  (mtip runs early in the m6 suite) → seconds, not minutes. If the m6 suite
  wall time regresses beyond the 300 s/elf `timeout`
  (`test/m6/run_all.sh:21-28`), the documented fallback is to re-arm
  mtip.S from the CLINT mtime MMIO (0x0200BFF8) instead of the `time` CSR —
  a test-side change, not an RTL one; the RTL re-point stands.
- Floor: full M2/M4/M5/M6 suites green after the change (D-M8-7).

### D-M8-6 — Cycle-count protocol (noted, not gated)

**Settled.**
- **Donor side:** cycles = `$finish` sim-time ÷ 10 ns. The tb already prints
  it: Icarus emits `$finish called at 273650 (100ps)` at `tb.v:269`
  (measured: csr 2736.5, interrupt 3716.5). `run_donor.sh` scrapes that
  line from `vvp` stdout. No tb patch required; the optional rv12-P4 2-line
  `$display(cycle_count)` at the PASS branch is **not** done (it would touch
  donor tb — keep the donor delta at the 3-file T1 baseline + the D-M8-2
  case-source patches only).
- **rv906 side:** cycles = the harness's `dut.step()` counter. Two options,
  both fine, T3 picks: (a) read the "cycle" value TestBench.cpp:239-245
  already prints every 1 M cycles (no code change, coarse for short cases —
  a 3K-cycle case prints nothing); (b) **preferred:** one added print of the
  final cycle count at the tohost-stop point in
  `testbench/TestBench.cpp:256-285` (1 line, harness-only, recorded in the
  ledger). `compare.py` records both; the T7 table shows the ratio. **Never
  gated.**
- Clock basis: both sides nominally "1 cycle = 10 ns / one `clk` tick";
  rv906's Verilator `clk` has no period, so the rv906 column is pure cycle
  count and the donor column is sim-time-derived cycles — comparable by
  construction (both = CPU cycles executed), which is the point of D-M8-5.

### D-M8-7 — Acceptance gate

**Settled.**
1. **Parity gate (the M8 gate):** for every case in the final set
   (donor-PASS set per D-M8-2), the rv906 side reports the **same verdict**
   (PASS) with the same tohost/magic mechanism semantics. A case that PASSes
   on the donor and FAILs on rv906 is a milestone failure (a real bug,
   class B per clone discipline) — M8 does not close until all final-set
   cases pass both sides.
2. **Floor (regression gate):** after T2's CSR change, the standing suites
   stay green: M2 sweep (86/87, documented ma_data
   `docs/08-verification.md:29`), M4 directed set (6/6 + rv64mi-p 17/17,
   `docs/08-verification.md:108-109`), M5 suites, **M6 8/8**
   (`test/m6/run_all.sh`). M7's own gate is M7's (Task 10).
3. **Cycle column:** present in the T7 table for all final-set cases;
   annotated, never a pass/fail input.
4. **Negative check:** T7 includes one deliberately-mismatched record
   (synthesized) that `compare.py` must flag, proving the comparator
   actually compares (rv12 precedent).

---

## 4. Per-case port deltas (the pinned deltas of D-M8-3)

All deltas below are the **complete** per-case delta set; anything else the
port needs at T4/T5 time is a deviation from this design and must be
escalated, not silently absorbed. Ledger entries are pre-seeded in §11.

### 4.1 csr — trivial

- Donor: `tests/cases/csr/C906_CSR_OPERATION.s` — standard
  `csrrw/csrrs/csrrc/csrrc.i` on `mstatus` only, exits `__exit`. **No
  non-standard CSRs** (verified: the T1 build succeeded unmodified).
- rv906: case body **byte-identical**; only the glue differs (ported crt0 +
  link script, §4.6). `mstatus` is fully functional on rv906.
- Expectation: PASS both sides (donor measured PASS, T1).

### 4.2 exception — conditional, T6-arbitrated

Donor-side (D-M8-2a): `mxstatus`-free case; patches = `mhcr`→`0x7c1`
(`:164`) + `dcache.ciall` (`:161`) **commented out** (no numeric form;
donor-side-only deviation, md5-ledgered). T6 runs it; **the donor verdict
decides whether exception is in the final set.**

Source-level observation (why T6 decides, not this doc): the case's chain
runs ILL(2)→EBREAK(3)→MISALIGN(4)→LOAD_ACCERR(5)→STORE_MISALIGN(6)→
STORE_ACCERR(7)→MECALL(11)→INST_SCCERR(1)→SECALL(9)→INS/LOAD/STORE_PAGEFAULT
(12/13/15)→UECALL(8). The pagefault subtest **faulting instructions are
commented out** (`:475-495`: `jalr x0,x1,0x0`, `lw x3,0x0(x1)`, `sw
x3,0x0(x1)` all `#`-commented) while their **checks remain live**
(`INS_PAGEFAULT_BEG:313-323` etc. still `bne` mepc==0xfff00000 /
mcause==12/13/15 → `TEST_FAIL`). Read literally, the chain cannot pass; read
charitably, the mode state (MPP through the mret chain) may keep the S/U
subtests in a regime where the checks are skipped or the flow short-circuits.
Either way: **T6's donor run is the arbiter.** If donor exception PASSES,
T4 ports the exact subtest chain the donor executed (the `vector_table`
handler indirection makes this mechanical); if it FAILs, exception drops out
of the final set (recorded) and T4 skips it.

rv906-side deltas (for the port, mechanism pinned):
1. **Drop `dcache.ciall`** (`:161`) — XThead CMO, not implemented (rv906
   has no CMO decode); on the donor the same line is commented out (above),
   so the bodies still match.
2. **`csrci mhcr,0x2` → `csrci 0x7c1,0x2`** (`:164`) — same text both sides.
3. **`MMU_CFG` block (`:150-160`): add satp-PPN remap before
   `MMU_PTW_4K 0x0,0x0,0xff,0xf`.** As written, the 4K PTE write lands at
   root-table-base + 0, and satp PPN resets to 0 → on rv906 the write
   targets PA 0 → crossbar DEFAULT_SLAVE → **0x80000000, the running
   program image** (`rtl/RVProcAXI.v:749-760`) — self-clobber. The port
   inserts a PPN write (satp PPN ← 0x81000, root table at PA 0x81000000,
   same safe hole as the MMU case, §4.3) before the PTE write. The PTE
   itself (VA[0,4K)→PA[0,4K)) is inert on rv906: every live subtest is
   M-mode (MMU bypass, `rtl/MMU.v:226`), and the pagefault subtests are
   defunct on both sides. (On the donor the same write clobbers PA 0 =
   `__start`, which is never re-executed — works by luck; the remap makes
   the rv906 port safe rather than lucky. Donor text unchanged → recorded
   as an rv906-side-only delta.)
4. **ACCERR subtests (LOAD/STORE/INST at 0x600000010, `:411-461`): remap
   the fault mechanism.** Donor mechanism: 0x600000010 is unmapped → axi_err
   bus error → cause 5/7/1, mtval = address. (The preceding 6-entry PMP
   reconfig, `:413-424`, is verified a **no-op for this address** under the
   shared NAPOT decode — the entries' windows are near 0x2FFF_FFF0 /
   [0x1800FFF000, 0x1807FFF000) / 6–256 GB TORs; none covers 0x600000010
   (`aq_pmp_comp_hit.v:108-143`; the M-mode default-allow
   `aq_pmp_acc.v:289` then lets the request reach the bus).) On rv906 the
   same address is a **silent MEM hit** (DEFAULT_SLAVE, offset 0x600000010)
   → no fault → subtest would never trap. The port re-targets the three
   subtests to **PMP L-bit deny at a MEM address**: pick one MEM scratch
   address (e.g. PA 0x800200000, 2 MB into MEM, clear of the image and the
   0x81000000 page-table hole), and in the ACCERR setup replace the
   6-entry reconfig with a **single locked deny entry** — `pmpcfg0 = 0x80`
   (entry 0: L=1, A=11, R=W=X=0) + `pmpaddr0` = the NAPOT encoding of that
   4 KB page. M-mode is checked because the entry is locked
   (`rtl/PMP.v:314-320` data, `:325-331` fetch — the fetch channel has the
   identical M-mode/locked semantics, so INST_ACCERR's `jalr` faults too).
   Expected: cause 5 (lw) / 7 (sw) / 1 (jalr), **mtval = the remapped MEM
   address** (RTU tval allowlist includes causes 1/5/7,
   `rtl/RTU.v:1014-1023`; T4 confirms the access-fault tval carries the
   faulting address — the M4 `mmu_pmp`/`mmu_pmpstore` directed tests cover
   the PMP-deny path but check mcause only, `test/m4/mmu_pmp.S:173-175`).
   The subtest bodies keep their exact mepc/mcause check structure; only
   the address constant and the PMP-setup block change → **mtval check
   values change** (0x600000010 → the MEM address) — ledgered.
5. **MISALIGN subtest (`:404-410`): no delta.** rv906 traps misaligned data
   accesses at issue (cause 4/6, `rtl/LSU.v:1076-1084`), matching the
   donor's expectation (mtval 0x6fff).
6. Everything else (ILL/EBREAK/MECALL/SECALL/UECALL structure, the
   `SETMEXP`/`vector_table` machinery, the `0x800` mstatus MPP pokes)
   ports verbatim.

### 4.3 MMU — pinned constants

Donor: `tests/cases/MMU/C906_mmu_basic.s`. Donor-side patch per D-M8-2a:
6× `mxstatus`→`0x7c0`. T6 runs it; donor-PASS ⇒ final set.

rv906-side deltas (exactly these, all in the case `.s`):

1. **`MMU_SATP_PPN 0x40` → `MMU_SATP_PPN 0x81000`** — root (L1) table at PA
   0x81000000 (16 MB into the 2 GB MEM window at 0x80000000; clear of the
   image, which grows from 0x80000000 and is < 1 MB for this case —
   assumption recorded; T4 measures the image size). Donor PA 0x40000 was
   its data SRAM (linker.lcf MEM2).
2. **`MMU_PTW_1G 0x0,0x0,0xcf,0xf` → `MMU_PTW_1G 0x0,0x80000,0xcf,0xf`** —
   the macro builds the PTE as `(\PPN<<10)|(\THEADFLAG<<59)|\FLAG`
   (`:24-72`), i.e. `\PPN` is the Sv39 PPN field = PA[51:12] with the
   large-page low bits zero. A 1G identity page over the MEM region:
   PA base 0x80000000 ⇒ PPN = 0x80000000>>12 = **0x80000** (1G-aligned:
   0x80000000 mod 2^30 = 0; PPN[17:0]=0 ✓). VA 0 → PA 0x80000000; the
   S-mode `sd/ld` at 0x30000 (`:290-292`) → PA 0x80030000 (192 K into MEM).
3. **`mxstatus` → `0x7c0`** at the same 6 sites — identical text both
   sides. The macro bodies (`MXSTATUS_THEADISAEE :152-161`,
   `MXSTATUS_MAEE :242-251`) do csrr/or/csrw with **no readback checks**, so
   rv906's MXSTATUS modeling only `mm` bit15 (`rtl/CSR.v:1453-1463`) makes
   the theisaee(22)/maee(21) pokes no-ops without changing the flow.
4. **PMP block (`:265-271`: `pmpcfg0=0x0f`, `pmpaddr0=0xc0000000`) —
   NO DELTA.** Verified: under the shared NAPOT decode, this encoding
   (stored value = wdata[37:9] = 0x600000, no trailing ones) falls to
   `default` → mask 0 → **matches every address** on **both** sides
   (donor `aq_pmp_comp_hit.v:108-143` default→0; rv906
   `rtl/PMP.v:176-214,248` identical). The case comment's "0x0~0x2_FFFF_FFFF"
   is this catch-all, approximated. So the S-mode access at 0x80030000 is
   PMP-allowed on rv906 exactly as 0x30000 is on the donor — same behavior,
   same (quirky) mechanism, zero delta. (This also covers the PTW's own
   PMP check of the root table at 0x81000000.)
5. **THEADFLAG=0xf kept** — rv906's Sv39 walker reads PTE bits 0–7 + PPN
   only (`rtl/MMU.v:547-550`); PTE[62:59] is ignored, source identity kept.
6. `MMU_EN` (MODE=8), `MMU_SATP_ASID 0x1`, `MMODE_SMODE`, `EXIT` → all
   verbatim. M-mode writes the PTE directly (MMU bypass, `rtl/MMU.v:226`),
   then mret into S-mode where the MMU is live — the donor's exact pattern
   (donor analog: `PTW_MACH_PMP` in `aq_mmu_ptw.v`).

### 4.4 interrupt — restructured init (rv906 side)

Donor: `tests/cases/interrupt/C906_plic_int_smoke.s` — measured PASS (T1,
3716.5 cycles). Structure: `set_mthreshold_mask` (threshold 0x1f) →
`init_ip` (`sw 0x2` to INTPEND, source-1 pending) → `init_prio` (source 1
prio 0xa) → `init_ie` (enable source 1 at INTIE+0 and +0x80) →
`set_mthreshold_off` → `wfi` → handler (push, claim, 4 nops, complete,
pop, `mret`) → `__exit`. `SETINT 11` installs the handler at
vector_table+128+88 (the MEIP slot). The handler does **no mcause check** —
it is a smoke test: PASS iff `wfi` returns and the claim/complete/`mret`
round-trip finishes (a hang hits the tb watchdog → FAIL).

Donor PLIC is in-core at 0x4000000000 with a two-level (hart×context)
enable space (INTIE 0x2000 + INTIE_HART 0x80) and the **writable INTPEND
0x1000** backdoor (`:120-126`; rv906's pending reg is RO,
`rtl/PLIC.v:9-17`).

rv906 port (keep the structure — init/claim/complete/wfi/`mret`/
vector_table/SETINT-11 — replace the init's source):
1. `PLICBASE_M` 0x4000000000 → **0x0C000000**; register offsets map 1:1 for
   the subset rv906 implements (prio[id] = 0x4·id; enable 0x2000;
   threshold 0x200000; claim/complete 0x200004 — `rtl/PLIC.v:9-17`). The
   INTIE_HART 0x80 poke is dropped (rv906 is single-context; 0x2000 is the
   context-0 enable) — delta recorded.
2. **Interrupt source: real UART IRQ (PLIC source 7)**, replacing the
   INTPEND backdoor poke. Sequence (exactly the proven M6 pattern,
   `test/m6/plic_uart.S`): prio[7]@0x0C00001C = 4, enable bit7
   @0x0C00002000, threshold @0x0C200000 = 0, then UART IER @0x10000004 = 2
   (THRE → `G_io_pins_uart_irq`, `device/uart16550.cpp:101-104`,
   `rtl/RVProcAXI.v:680`), `wfi`, on the m-mode interrupt claim
   0x0C200004 (returns 7), **deassert IER = 0 before completing**
   (complete = write 0x0C200004 = 0), `mret`. The IER deassert is
   **mandatory** on rv906 (the THRE level stays high while IER is set —
   without the deassert the interrupt re-fires after `mret` and the case
   loops in claim forever; the donor's one-shot INTPEND pending bit has no
   such issue).
3. Handler body (4 nops) and `SETINT 11` port verbatim; the pass condition
   is unchanged (wfi returns, round-trip completes, `__exit` reached).

### 4.5 coremark — glue-only deltas (both sides)

`.c` files byte-identical (9 files in `tests/cases/coremark/`,
`core_list_join.c … cvt.c`). Deltas:
1. **CFLAGS (both sides):** drop the 4 vendor `-m*` flags from
   `smart_run/tests/lib/Makefile:50` (donor, D-M8-2b) and use the
   same-minus-those-flags line in `test/m8/Makefile` (rv906). Everything
   else in the line (`-O3 -mtune=c906 -static -funroll-all-loops
   -finline-limit=500 -fgcse-sm -fno-schedule-insns --param
   max-rtl-if-conversion-unpredictable-cost=100 -fno-code-hoisting
   -DITERATIONS=10000`) is accepted by both toolchains (measured for
   /opt/riscv 15.1.0; xpack 15.2.0 is the rv906 standard, `test/m6/Makefile:31`).
2. **`-DITERATIONS` / loop count: NO DELTA.** `results[0].iterations = 2`
   is hard-coded (`core_main.c:178`); the flag is vestigial (§2.4). The run
   is 2 benchmark iterations on **both** sides. **Wall-time is the open
   variable:** at the measured donor pace (~25 cycles/s iverilog, T1) 2
   coremark iterations (est. ~100K–500K CPU cycles) is **~1–6 h**. T6 runs
   it once and measures; if the first run exceeds ~12 h (or hits the
   7 M-cycle watchdog, `tb.v:24`), the fallback is an **identical** trim on
   both sides — `results[0].iterations = 1` and/or the size macros in
   `core_portme.h` (COREMATRIX_ORDER etc.) — recorded in the ledger. This
   is the only case whose iteration/size may deviate, and it may deviate
   only identically on both sides.
3. **Console (rv906 glue only):** ported clib `fputc.c` writes THR
   **0x10000000** (16550 → verisim console, `device/uart16550.cpp:66-71`)
   instead of 0x6000fff8 (donor: axi_err, invisible). The `VCUNT_SIM` score
   line is thus **visible on rv906, absent on donor** — recorded; the
   `time`-based score is only meaningful post-T2 (D-M8-5) and only feeds
   the cycle column.
4. **Exit (rv906 glue only):** ported crt0 `__exit` → tohost=1 (§4.6).
   Donor: magic register (unchanged).
5. clib `vtimer.c` (`csrr time`) — no delta on either side; on rv906 it
   reads mcycle post-T2 (donor-compatible).

### 4.6 Ported crt0 + link script + scaffolding (test/m8, task T3)

**`test/m8/crt0_m8.s`** = copy of the donor `crt0.s` (post-T1) with:
1. **Keep every poke, byte-identical poke stream** (source identity):
   `csrs 0x7c0,0x400000` (mxstatus theisaee — no-op on rv906), mstatus
   0x802000, MIE, `csrs 0x7c2,0x30013` (mcor — **no-op, R4 correction**,
   unknown-CSR write dropped, `rtl/CSR.v:1795-1796`), `csrs 0x7c1,0x7f`
   (mhcr — rv906 models exactly these bits: ie/de/wa/rse/bpe/btbe, reset 0,
   `rtl/CSR.v:1411-1443`; **both sides end at MHCR=0x7f** — the
   rv12-P3-style "full config, value-equal" pin), `csrs 0x7c5,0x610c`
   (mhint — rv906 models a subset, `rtl/CSR.v:1465-1520`, rest
   storage/no-op).
2. **`__exit` → tohost=1; `__fail` → tohost=(1<<1)|1=3** (fixed testno 1 —
   the donor `__fail` takes no argument; recorded). Uses the `tohost`
   symbol (harness polls by symbol, the tohost-gotcha memory: the ELF must
   export it, `test/entry.S:204-212` pattern).
3. Keep the trap handler + 128-entry `vector_table`→`__fail` scaffold
   (`crt0.s:160-230`) — works on rv906 (mtvec/cause-based dispatch, the
   M2–M6 trap path is standard).
4. Stack: `__kernel_stack` provided by the link script (below, not the
   donor's 0xee000).
5. Drop nothing else — no mcor drop (R4), no mhcr/mhint drop.

**`test/m8/link_m8.ld`:** image at 0x80000000 (MEM, `rtl/RVProcAXI.v:152`);
`.tohost`/`.fromhost` at **0x7FFFF000** (uncached aperture,
`test/m2/common.ld:38-39` convention); stack grows down from the top of a
1 MB window at 0x80100000; page-table scratch **reserved at 0x81000000**
(the MMU/exception root-table hole, §4.2/§4.3) — the linker must not place
any section there (it won't: the image is < 1 MB and the hole is 16 MB
in; the reservation is documented, not enforced).

**`test/m8/Makefile`:** mirrors `test/m6/Makefile` — xpack gcc
(`:31`), `-march=rv64imafdc_zicsr_zifencei -mabi=lp64d` (D-M8-1: the
donor's T1-patched string, `smart_run/tests/lib/Makefile:40`), `-O2` for
the `.s` cases, coremark CFLAGS per §4.5, `-T link_m8.ld`, `TESTS` globbed
from `cases/<name>/*.s|*.c`.

**`test/m8/run_all.sh`:** mirrors `test/m6/run_all.sh` —
`timeout 300 bin/verisim/testbench --print-result <elf> | grep -q PASS`
per case; coremark gets its own (longer) timeout per §4.5.

**`test/m8/run_donor.sh` (new):** donor-side runner — `export
CODE_BASE_PATH=…/refs/openc906/C906_RTL_FACTORY; export TOOL_EXTENSION=
/opt/riscv/bin; cd smart_run; SHELL=/bin/bash`; **reuse the existing
`work/xuantie_core.vvp` (no recompile — T1's 57.8 MB artifact is current:
no RTL changed since)**; per case: `make buildcase CASE=<c> SIM=iverilog
SHELL=/bin/bash` + `cd work && vvp xuantie_core.vvp`; capture
`run_case.report` (TEST PASS/FAIL) + the `$finish` sim-time line from vvp
stdout → cycles = simtime/100ps/10ns (D-M8-6). Runtime requirements (T1):
`SHELL=/bin/bash` on every make invocation (dash rejects the
`smart_cfg.mk` `>&` redirections), `work/` exists (it does), env vars
exported.

**`test/m8/compare.py` (new):** JSON records
`{case, side, verdict, cycles, console_excerpt, toolchain, march, deltas[]}`;
gate = every final-set case verdict-equal (D-M8-7.1); cycle column printed
(not gated); negative check (D-M8-7.4).

---

## 5. Task table

T1 (donor dry-run) is DONE — committed 81351a5 with the results note.
T6 (filtered smokes) is DROPPED (D-M8-4); numbering below is final.

| # | Task | Content | Gate (must be green before next task) |
|---|---|---|---|
| **T2** | `time`→`mcycle` re-point | 1 line, `rtl/CSR.v:1651` (`CSR_TIME` arm → `mcycle_reg`); re-locate by symbol post-M7. **BLOCKED on M7 Task 10** (CSR.v ownership). | Directed: a new `test/m8/time_cyc.S` — `rdtime` twice across a known loop, assert delta == loop length ± 2, and assert `time` advances while `mtime` (MMIO 0x0200BFF8) is ~100× slower (locks the semantic, not just the value). Floor: **m6 8/8** (mtip.S slower-but-passing, D-M8-5), M2 sweep 86/87, M4 17+6/17+6, M5 suites — all re-run and green. |
| **T3** | `test/m8/` scaffolding | Makefile, `crt0_m8.s`, `link_m8.ld`, ported clib, `run_all.sh`, `run_donor.sh`, `compare.py` skeleton (§4.6). | **csr case PASSes on rv906** (first end-to-end through the new glue); `run_donor.sh` reproduces donor csr PASS + cycles from the reused vvp (no recompile); compare.py negative check works. |
| **T4** | Core set on rv906: **csr + exception + MMU** (+ coremark build & run once its donor verdict is known) | Port deltas §4.1–§4.3, §4.5; ledger entries finalized as the port is written. | csr PASS; MMU PASS (S-mode sd/ld through the 1G identity page at 0x80030000, no PMP/MMU fault); exception PASS **iff** donor exception passed in T6 (else: skipped, recorded). Floor still green. |
| **T5** | **interrupt** port | §4.4: PLIC 0x0C000000 + real UART IRQ source 7, structure kept. | PASS on rv906; claim returns 7; IER deassert-before-complete holds (no spurious re-fire — the M6 pattern's known gotcha). |
| **T6** | **Donor-side runs** (was T7) | `run_donor.sh` over {csr, interrupt, MMU, exception, coremark} with the D-M8-2 donor patches (7 lines + 1 commented line, md5-ledgered); **reuse `xuantie_core.vvp`**; capture verdicts + `$finish` cycles; coremark first-run wall-time measurement (the §4.5 trigger point for the identical-both-sides trim). | All five runs complete with a recorded verdict each; {csr, interrupt} reproduce T1's PASS + cycle counts within run-to-run noise (they must — same vvp, same case); the **final parity set is declared here** (donor-PASS cases). |
| **T7** | **Compare + close-out** (was T8) | `compare.py` over all final-set cases both sides; cycle table; floor re-run (post all RTL-affecting work — only T2 touched RTL, but re-run anyway); deviation ledger finalized; close-out note `notes/2026-09-2x-m8-results.md` (M8 analog of the T1 note). | **D-M8-7: every final-set case PASSes both sides; floor green; cycle table present for all; negative check flagged.** Milestone closes. |

Sequencing: T2 ∥ {T3→T4→T5} are independent chains (T2 only touches
CSR.v; T3–T5 only touch test/m8) — but T4's coremark run wants T2 landed
for a meaningful `VCUNT_SIM`. T2's directed `time_cyc.S` lives in
`test/m8/`: if T2 lands before T3's scaffolding, T2 builds it ad-hoc
(one `.S`, direct testbench invocation — no Makefile needed) and T3 folds
it into the case set. **T6 is independent of T4/T5** (donor-side; needs
only this design + the D-M8-2 patches) — run it early (its verdicts define
T4's exception scope and T7's table). T7 is last.

---

## 6. Verification plan (D-M8-7, operational)

1. **Parity:** final-set (T6-declared) × {donor, rv906} verdict matrix
   from `compare.py`; all PASS.
2. **Floor, re-run at T7 close-out:** M2 sweep (86/87 + documented
   ma_data), M4 (rv64mi-p 17/17 + directed 6/6), M5 suites, M6 8/8 —
   unchanged from their standing gates; the only RTL delta since M6 close
   is T2's one line.
3. **Cycle table:** donor ($finish simtime ÷ 10 ns) vs rv906 (harness step
   count, final-cycle print per D-M8-6); annotated (pipelining/caching
   differences are expected; no threshold).
4. **Ledger audit:** every §4 delta has a ledger row (file, line, donor
   text, rv906 text, reason, mechanism reference); the donor-tree md5
   audit (baseline + T1 + T6 patches) is printed in the close-out note.
5. **Negative check:** synthesized mismatch record rejected by compare.py.

## 7. Risk table (ranked)

| # | Risk | Likelihood | Impact | Mitigation / decision already made |
|---|---|---|---|---|
| R1 | **Donor exception case fails on the donor** (its pagefault subtest faulting instructions are commented out while the checks are live, §4.2) | Medium | Low — the case drops out of the final set, parity set = {csr, interrupt, MMU, coremark} | D-M8-2 consequence clause: donor verdict decides (T6); no port-to-a-failing-oracle. The M4 exception directed coverage (rv64mi) already gates the underlying trap paths, so losing the case costs little. |
| R2 | **coremark donor wall-time** exceeds practical bounds (est. 1–6 h; worst case the 7 M-cycle watchdog, `tb.v:24`) | Medium | Medium — the only case with C-level workload | First T6 run measures; fallback = identical-both-sides trim (iterations 2→1 and/or `core_portme.h` sizes), ledgered (§4.5). Both sides stay source-identical, so the parity meaning is preserved. |
| R3 | **T2 slows m6-mtip past the 300 s/elf timeout** (`test/m6/run_all.sh:21-28`) | Low (mtip runs early, C small) | Low — floor gate | Measured-by-analysis: +≈90·C cycles (D-M8-5); if it trips, the documented fallback re-arms mtip from the CLINT mtime MMIO (test-side only). |
| R4 | **ACCERR remap mtval semantics**: the PMP access-fault tval on rv906 might not carry the faulting address (spec allows 0) | Low | Low — exception subtest | M4 `mmu_pmp`/`mmu_pmpstore` already pass through this exact deny path (cause-gated); T4 confirms the tval value and, if it is 0, adjusts the remapped subtest's mtval check to expect 0 — ledgered, mechanism unchanged. |
| R5 | **CSR.v line drift** between this design and T2 execution (M7 is mid-edit) | Certain | Low | T2 locates the arm by symbol (`CSR_TIME:` in the read mux), not by line; T2 re-runs the floor before closing. |
| R6 | **rv906 image growth** collides with the 0x81000000 page-table hole or the 0x80030000 S-mode data spot (MMU case) | Very low (case images < 1 MB; holes 16 MB in) | Low — a silent clobber would be a confusing hang | T4 measures the linked image size per case (objdump) and asserts it stays < 128 K before the 0x80030000 data spot and < 16 MB before the table hole; the linker script documents both reservations. |
| R7 | **run-to-run cycle noise** in the donor column (vvp is deterministic, but wall-time is not) | Certain (wall), none (sim time) | None | The cycle column uses **sim time** ($finish line), not wall time — deterministic for a given vvp+case. |
| R8 | **M7 close-out slips**, blocking T2 | Medium | Schedule only | T2 is the only RTL task; T3–T6 proceed in parallel (test/m8 + donor side are independent). M8 close-out waits on T2+T7, nothing else. |

## 8. Files expected to change (T2–T7, all of it)

- `rtl/CSR.v` — 1 line (T2). **Nothing else in `rtl/`.**
- `test/m8/` (new): `Makefile`, `crt0_m8.s`, `link_m8.ld`, `clib/` (fputc.c
  ported; rest copied), `cases/{csr,exception,MMU,interrupt,coremark}/`
  (ported case sources), `run_all.sh`, `run_donor.sh`, `compare.py`,
  `records/` (JSON), `README.md`.
- `testbench/TestBench.cpp` — 1 line (final-cycle print, D-M8-6, T3).
- **Donor tree (`refs/openc906/`) — the ONLY files T6 may touch**, each
  md5-ledgered: `smart_run/tests/cases/MMU/C906_mmu_basic.s` (6 lines),
  `smart_run/tests/cases/exception/C906_Exception.s` (1 line changed +
  1 line commented), `smart_run/tests/lib/Makefile` (4 flags dropped from
  the coremark CFLAGS line). Plus T1's existing 3-file delta (untouched).
  **No donor RTL, no donor tb.**
- `docs/superpowers/specs/notes/2026-09-2x-m8-results.md` (T7 close-out
  note, mirrors the T1 note).
- This design doc + the §11 ledger as the design record.

## 9. Deviation ledger (template + pre-seeded rows)

Format: `| id | case | side | where | original text | final text (rv906, or patched donor for donor-side rows) | reason (mechanism ref) |`.
Seeded (T4/T6 finalize the exact line numbers as the ports are written):

| id | case | side | where | original | final | reason |
|---|---|---|---|---|---|---|
| L1 | exception | both | `C906_Exception.s:164` | `csrci mhcr,0x2` | `csrci 0x7c1,0x2` | name→numeric (both toolchains reject the name; same CSR, `rtl/rvproc_pkg.sv:281`) |
| L2 | exception | donor | `C906_Exception.s:161` | `dcache.ciall` | `#dcache.ciall` | XThead CMO has no standard encoding; donor-side build necessity (D-M8-2a) |
| L3 | exception | rv906 | `C906_Exception.s:161` | (present, active) | dropped | same line, no CMO decode in rv906 (body parity with the patched donor) |
| L4 | exception | rv906 | `MMU_CFG` `:150-160` | PPN=0 (implicit) | +satp PPN←0x81000 before `MMU_PTW_4K` | PTE write must not land at PA 0 = the image on rv906 (DEFAULT_SLAVE, `rtl/RVProcAXI.v:749-760`) |
| L5 | exception | rv906 | `LOAD/STORE/INST_ACCERR` `:411-461` | 0x600000010 (bus error) + 6-entry PMP no-op | PMP-locked-deny at MEM scratch (e.g. 0x800200000), `pmpcfg0=0x80`+NAPOT pmpaddr0; mtval check → same new address | 0x600000010 is a silent MEM hit on rv906 (DEFAULT_SLAVE); PMP L-bit deny is the rv906-native access-fault mechanism (`rtl/PMP.v:314-333`) |
| L6 | MMU | both | `C906_mmu_basic.s` 6 sites | `mxstatus` | `0x7c0` | name→numeric (same CSR, `rtl/rvproc_pkg.sv:280`) |
| L7 | MMU | rv906 | `:263` | `MMU_SATP_PPN 0x40` | `MMU_SATP_PPN 0x81000` | root table in MEM (donor PA 0x40000 = its data SRAM) |
| L8 | MMU | rv906 | `:281` | `MMU_PTW_1G 0x0,0x0,0xcf,0xf` | `MMU_PTW_1G 0x0,0x80000,0xcf,0xf` | 1G identity over MEM base 0x80000000 (Sv39 PPN field = PA[51:12] = 0x80000000>>12 = 0x80000) |
| L9 | MMU | rv906 | theisaee/maee pokes | active bits | no-ops (MXSTATUS models `mm` only, `rtl/CSR.v:1453-1463`) | source identity kept; no readback checks in the macros |
| L10 | interrupt | rv906 | init block | in-core PLIC 0x4000000000 + writable-INTPEND poke + INTIE_HART 0x80 | PLIC 0x0C000000 + UART IRQ source 7 (IER@0x10000004, deasserted before complete) + single-context enable | platform PLIC difference; real level source replaces the donor's one-shot pending backdoor (pattern: `test/m6/plic_uart.S`) |
| L11 | coremark | both | CFLAGS | `… -msignedness-cmpiv -mno-thread-jumps1 -mno-iv-adjust-addr-cost -mno-expand-split-imm …` | same minus the 4 flags | vendor-only gcc flags, rejected by standard toolchains (measured); `-fno-code-hoisting`/`--param` kept (accepted) |
| L12 | coremark | rv906 | `clib/fputc.c:19` | `li x13,0x6000fff8` | THR 0x10000000 (16550 → console) | platform console; donor console is axi_err (invisible) |
| L13 | coremark | rv906 | crt0 `__exit/__fail` | magic x3 0x444333222/0x2382348720 | tohost=1 / tohost=3 | platform exit protocol (harness polls the `tohost` symbol) |
| L14 | all | rv906 | crt0 | — | keeps mcor 0x7c2 / mhint / theisaee pokes (no-ops) | **R4 correction**: unknown-CSR writes are dropped, not trapped (`rtl/CSR.v:1795-1796`) — poke stream stays byte-identical |
| L15 | all | rv906 | crt0 `__fail` | no testnum arg | fixed testno 1 | donor `__fail` takes no argument |
