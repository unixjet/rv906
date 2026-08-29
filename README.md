# rv906

rv906 is a textbook-grade, hand-written Verilog clone of T-Head's XuanTie
C906 (RV64IMAFDC, 5-stage single-issue in-order pipeline), built in the
coding style of the RVProc6 project — few files, clear pipeline stages,
explicit stall/flush feedback — so the microarchitecture is actually
readable as teaching material. It is the sibling of `../rv12`, which clones
C910 (an out-of-order machine) the same way. See
`docs/superpowers/specs/2026-08-20-rv906-c906-clone-design.md` for the full
design document.

## Status

**M2: integer machine complete, pending review.** The integer decode +
execute pipeline (IDU, IU, LSU, RTU/CSR) is built and green: the riscv-tests
`rv64ui-p-*` + `rv64um-p-*` self-checking suites plus a local compressed-
instruction test all pass with caches on (67/68 — `rv64ui-p-ma_data` is the
one documented exception, hardware misaligned access deferred to M4; see
`docs/08-verification.md` §8.7.1). M1's front end (fetch + predictor) sits in
front of it unchanged.

Quick start (M2):

    make verisim                                  # build the simulator
    make -C test/m2                               # build the riscv-tests ELFs
    bin/verisim/testbench --print-result test/m2/build/rv64ui-p-add.elf
    make -C test/m2 unit                          # (see below) unit benches
    make -C test/m2/unit run                      # per-unit benches (all 7)

(`rv64ui-p-ma_data` needs HW misaligned access, deferred to M4; run the
caches-off sanity cross-check with `make -C test/m2 RV906_BOOT_MHCR=0`.)

**M1: front end complete.** The instruction fetch unit, the
32 KB L1 instruction cache and the full three-structure branch predictor
(BHT + BTB + RAS) are built and green across the whole acceptance matrix.

What exists and works:

- **The front end** (`docs/02-ifu.md` and `docs/03-bpu.md` are its two
  chapters): `rtl/IFU.v`, the flat single-issue fetch pipeline (PCGEN →
  ICache access → IPACK → IBUF) with its 7-level redirect ladder and
  3-destination cancel network; `rtl/ICache.v`, 32 KB / 2-way / 64 B-line
  with live RVC-boundary detection and the fence.i invalidate walk;
  `rtl/BPU.v`, all three C906 predictors (BHT: 1024×16 SRAM, pure-GHR index;
  BTB: 16-entry flop CAM; RAS: 4-entry flop stack, pointer-only resync);
  `rtl/SRAM.v`, the one behavioral SRAM model for the project.
- **The M1 oracle pair**: `rtl/FetchSink.v` stands in for IDU/IU/RTU/CP0
  (fake BJU + fake RTU + the harness config bank) and reports over
  `tohost`; a C++ golden fetch-ISS (`m1_iss.h`, driven from `RVProcTest.cpp`)
  derives the expected committed stream independently from the same spec
  text, and the testbench compares the two online, every cycle.
- **The acceptance matrix**: 11 directed tests × 4 rungs of the predictor
  chicken-bit ladder × `--sink-stall` off/on, plus a mid-run invalidate
  sweep pass at the top rung — 110 runs, all green
  (`test/m1/run_all.sh --full-matrix`). Every test's committed stream is
  identical at every rung, which is the property the whole ladder rests on:
  a predictor changes *when* instructions are fetched, never *which* ones
  commit.
- **A full AXI SoC simulation stack** (from M0): Verilator-simulated
  `RVProcAXI` fabric (crossbar, memory controller, CLINT, PLIC) wired to a
  C++ testbench (ELF loading, tohost/fromhost protocol, FDT generation, an
  NS16550A UART model). The M0 `TestMaster.v` scaffold and its `test/smoke`
  firmware are retired: their D-side write/tohost path is a strict subset of
  what every M1 test exercises through FetchSink.
- A bare-metal firmware kit (`test/`) that builds with the xpack RISC-V
  toolchain but does not run yet (needs a back end, M2).

See `docs/02-ifu.md` (fetch pipeline + ICache: principle, implementation,
C906 file cross-reference, design discussion) and `docs/03-bpu.md` (branch
predictor: same structure) for the front-end chapters, and
`docs/08-verification.md` for the simulation stack, the tohost protocol and
the M1 harness. `docs/superpowers/plans/2026-08-20-m1-ifu.md` has the
task-by-task history, including every deviation and finding this milestone
made while reading the real C906 RTL.

## Directory map

| Path | Contents |
|------|----------|
| `rtl/` | Verilog/SystemVerilog RTL: AXI fabric (`AXICrossbar.v`, `AXIAddrDecode.v`, `AXIWidthAdapter.v`, `AXI4LSlave.v`), memory controller (`MEMCTL_AXI4L_step.RTL.v`), CLINT/PLIC, the SoC wrapper (`RVProcAXI.v`), the shared parameter package (`rvproc_pkg.sv`), the behavioral SRAM model (`SRAM.v`), and the core shell (`RVProc.v`: `IFU.v` + `ICache.v` + `BPU.v` + `FetchSink.v`) |
| `testbench/` | C++ simulation harness: ELF loader (`load_elf.cpp`), FDT builder (`fdt.cpp`), the tohost/fromhost `TestBench` base class |
| `io/` | AXI4-Lite C++ models: external memory (`ExtMem.h`), the generic AXI4L slave/converter templates (`RVProc_io.h`) |
| `device/` | Peripheral device models (NS16550A UART, tty server) |
| `test/` | Bare-metal firmware kit (build-only until M2); `test/m1/`, the M1 directed test suite (11 `.S` tests) plus `test/m1/run_all.sh` (the full matrix runner) and `test/m1/unit/`, the M1 standalone RTL/ISS unit benches (`icache_tb.cpp`, `fetchsink_tb.cpp`, `iss_tb.cpp`) |
| `docs/` | Subsystem chapters (`02-ifu.md`, `03-bpu.md`), verification notes (`08-verification.md`), and the specs/plans under `docs/superpowers/` |
| `refs/` | Reference RTL pulled in for porting/comparison (gitignored, not part of this repository's source) |

## Prerequisites

- [Verilator](https://verilator.org/)
- `clang`/`clang++` (used as the Verilator C++ compiler and linker)
- `libelf` and `libfdt` development headers/libraries (`-lelf -lfdt`)
- The xpack RISC-V toolchain, expected at
  `/opt/xpack-riscv-none-elf-gcc-15.2.0-1/bin/riscv-none-elf-` (used to build
  the `test/` firmware kit and the M1 directed test suite as it lands)

## Quick start

**M0's smoke test is retired** (design doc S4.3) along with `TestMaster.v` —
`FetchSink.v` reports through the same `tohost` protocol now, exercised by
every M1 test.

```bash
make verisim                    # build the simulator (bin/verisim/testbench)
make -C test/m1                 # build the M1 directed tests (.S -> .out ELFs)

# one test: call/return nests at and just past the 4-entry RAS limit, at
# rung 2 (RAS on), checked instruction by instruction against the golden ISS
timeout 60 bin/verisim/testbench --print-result --m1-rung=2 test/m1/callret.out
echo "exit=$?"                  # expect: "test/m1/callret.out: PASS." and exit=1
```

`--m1-rung=<1..4>` selects a rung of the predictor chicken-bit ladder — 1 =
every predictor off, then +RAS, +BTB, +BHT. The same test must pass, with
the same committed stream, at every rung.

The full M1 acceptance matrix (110 runs: 11 tests × 4 rungs × `--sink-stall`
off/on, plus 22 more at rung 4 with `--inv-test`):

```bash
test/m1/run_all.sh --full-matrix   # prints a per-rung table, a cycle-count
                                    # summary and the slot-count invariant
                                    # check; exits 0 on ALL PASS
```

(`test/m1/run_all.sh` with no arguments, or `--m1-rung=N`, runs the older
single-rung mode used during Tasks 6-9's own bring-up.)

The standalone unit benches, which drive `ICache.v`/`FetchSink.v` directly
without the SoC around them, plus the golden ISS's own zero-RTL self-test:

```bash
make -C test/m1/unit run    # icache_tb + fetchsink_tb + iss_tb, UNIT-SUITE-PASS
bin/verisim/testbench --iss-selftest    # the golden ISS's own gate, no RTL
```

Each unit bench prints one line per check and ends with `UNIT-PASS` or
`UNIT-FAIL`; see `docs/superpowers/plans/2026-08-20-m1-ifu.md` for what each
task's gate actually is, and `docs/08-verification.md` for the full CLI flag
table and the two-oracle architecture.

To build the bare-metal firmware kit (build-only until M2 brings up a real
core):

```bash
make -C test
```

To trace waveforms (FST, written to `run/dump.fst`):

```bash
make verisim VERISIM_TRACE=1
```
