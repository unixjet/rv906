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

**M1: front end bring-up in progress.** `RVProcAXI` now instantiates the
real core shell (`rtl/RVProc.v`): IFU + ICache + BPU (predictor structures
still land in later M1 tasks) + `FetchSink.v` (an M1-only scaffold that
plays fake BJU/RTU/CP0 until the real IDU/IU/RTU exist in M2). What exists
and works:

- **A full AXI SoC simulation stack**: Verilator-simulated `RVProcAXI`
  fabric (crossbar, memory controller, CLINT, PLIC — all real RTL) wired to
  a C++ testbench (ELF loading, the tohost/fromhost protocol, FDT
  generation, an NS16550A UART model).
- **`rtl/ICache.v`** (complete + unit-tested) and **`rtl/IFU.v`** (complete,
  predictor-less pipeline) — see `docs/superpowers/plans/2026-08-20-m1-ifu.md`
  for the task-by-task history.
- **`rtl/FetchSink.v`**, the M1 scaffolding core-shell tenant: consumes the
  IFU's single-instruction-per-cycle delivery, resolves every control
  transfer against its own predictor-independent oracle (direction rule,
  decoded immediates, a 16-entry shadow call stack, the JR_TARGET formula),
  and reports through `tohost` exactly as M0's `TestMaster.v` did.
- **M0's `rtl/TestMaster.v` and `test/smoke/` are RETIRED** (design doc
  S4.3): TestMaster's D-side AXI write FSM was lifted verbatim into
  FetchSink.v, and every M1 test exercises a strict superset of what the
  smoke test covered.
- **A bare-metal firmware kit** (`test/`) that builds with the xpack
  RISC-V toolchain but does not run yet — running it needs a real core
  (M2).

The M1 directed test suite, the C++ fetch-ISS/online checker, and the
chicken-bit predictor ladder (Tasks 5-10 of the M1 plan) are still being
built — there is no runnable M1 test target yet. A full M1 test-matrix
quick-start lands with Task 10; until then, `make verisim` (below) is the
build-correctness gate. See `docs/08-verification.md` for the simulation
stack and the tohost protocol, and
`docs/superpowers/plans/2026-08-20-m1-ifu.md` for the M1 task plan.

## Directory map

| Path | Contents |
|------|----------|
| `rtl/` | Verilog/SystemVerilog RTL: AXI fabric (`AXICrossbar.v`, `AXIAddrDecode.v`, `AXIWidthAdapter.v`, `AXI4LSlave.v`), memory controller (`MEMCTL_AXI4L_step.RTL.v`), CLINT/PLIC, the SoC wrapper (`RVProcAXI.v`), the shared parameter package (`rvproc_pkg.sv`), the behavioral SRAM model (`SRAM.v`), and the core shell (`RVProc.v`: `IFU.v` + `ICache.v` + `BPU.v` + `FetchSink.v`) |
| `testbench/` | C++ simulation harness: ELF loader (`load_elf.cpp`), FDT builder (`fdt.cpp`), the tohost/fromhost `TestBench` base class |
| `io/` | AXI4-Lite C++ models: external memory (`ExtMem.h`), the generic AXI4L slave/converter templates (`RVProc_io.h`) |
| `device/` | Peripheral device models (NS16550A UART, tty server) |
| `test/` | Bare-metal firmware kit (build-only until M2) and `test/m1/unit/`, the M1 standalone RTL unit benches (`icache_tb.cpp`, `fetchsink_tb.cpp`) |
| `docs/` | Verification notes (`08-verification.md`) and the specs/plans under `docs/superpowers/` |
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
there is no end-to-end runnable test target yet. M1 bring-up is in progress;
a full M1 test-matrix quick-start (`test/m1/run_all.sh` across the
chicken-bit predictor ladder) lands with plan Task 10. For now:

```bash
make verisim                    # build the simulator (bin/verisim/testbench)

make -C test/m1/unit icache && bin/unit/icache_tb   # ICache standalone unit bench
make -C test/m1/unit fetchsink && bin/unit/fetchsink_tb  # FetchSink standalone unit bench
```

Each unit bench prints one line per check and ends with `UNIT-PASS` or
`UNIT-FAIL`; see `docs/superpowers/plans/2026-08-20-m1-ifu.md` for what each
task's gate actually is.

To build the bare-metal firmware kit (build-only until M2 brings up a real
core):

```bash
make -C test
```

To trace waveforms (FST, written to `run/dump.fst`):

```bash
make verisim VERISIM_TRACE=1
```
