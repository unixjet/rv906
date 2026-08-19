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

**M0: simulation scaffold complete.** There is no CPU pipeline yet — no
IFU, no decode, no execute. What exists and works:

- **A full AXI SoC simulation stack**: Verilator-simulated `RVProcAXI`
  fabric (crossbar, memory controller, CLINT, PLIC — all real RTL) wired to
  a C++ testbench (ELF loading, the tohost/fromhost protocol, FDT
  generation, an NS16550A UART model).
- **`rtl/TestMaster.v`**, a placeholder core: not a CPU, just a small FSM
  that exercises the AXI write/read/CLINT-read path and reports the result
  through `tohost`. It stands in for the real core until M2.
- **A passing end-to-end AXI smoke test** (`test/smoke/`): `PASS.`,
  `exit=1`. The fail-detection path was also verified real — a deliberate
  mismatch was injected, correctly produced `FAIL. test no. = 1` / `exit=3`,
  then reverted.
- **A bare-metal firmware kit** (`test/`) that builds with the xpack
  RISC-V toolchain but does not run yet — running it needs a real core
  (M2).

M1 (front end: IFU + ICache + branch predictor) is the next milestone. See
`docs/08-verification.md` for the simulation stack, the tohost protocol, and
the M2 restore checklist (what has to change once a real core replaces
`TestMaster.v`).

## Directory map

| Path | Contents |
|------|----------|
| `rtl/` | Verilog/SystemVerilog RTL: AXI fabric (`AXICrossbar.v`, `AXIAddrDecode.v`, `AXIWidthAdapter.v`, `AXI4LSlave.v`), memory controller (`MEMCTL_AXI4L_step.RTL.v`), CLINT/PLIC, the SoC wrapper (`RVProcAXI.v`), the shared parameter package (`rvproc_pkg.sv`), and the M0 placeholder core (`TestMaster.v`) |
| `testbench/` | C++ simulation harness: ELF loader (`load_elf.cpp`), FDT builder (`fdt.cpp`), the tohost/fromhost `TestBench` base class |
| `io/` | AXI4-Lite C++ models: external memory (`ExtMem.h`), the generic AXI4L slave/converter templates (`RVProc_io.h`) |
| `device/` | Peripheral device models (NS16550A UART, tty server) |
| `test/` | Bare-metal firmware kit (build-only until M2) and `test/smoke/`, the M0 AXI smoke test firmware |
| `docs/` | Verification notes (`08-verification.md`) and the specs/plans under `docs/superpowers/` |
| `refs/` | Reference RTL pulled in for porting/comparison (gitignored, not part of this repository's source) |

## Prerequisites

- [Verilator](https://verilator.org/)
- `clang`/`clang++` (used as the Verilator C++ compiler and linker)
- `libelf` and `libfdt` development headers/libraries (`-lelf -lfdt`)
- The xpack RISC-V toolchain, expected at
  `/opt/xpack-riscv-none-elf-gcc-15.2.0-1/bin/riscv-none-elf-` (used to build
  `test/smoke` and the `test/` firmware kit)

## Quick start

```bash
make verisim                    # build the simulator (bin/verisim/testbench)

make -C test/smoke              # build the M0 smoke firmware (test/smoke/smoke.out)
timeout 30 bin/verisim/testbench --print-result test/smoke/smoke.out
echo "exit=$?"                  # expect: "test/smoke/smoke.out: PASS." and exit=1
```

`--print-result` is required to see the `PASS.`/`FAIL. test no. = N`
message; the process exit code is the **raw tohost value** (1 = PASS), not a
conventional 0/1 success code — see `docs/08-verification.md` for why.

To build the bare-metal firmware kit (build-only until M2 brings up a real
core):

```bash
make -C test
```

To trace waveforms (FST, written to `run/dump.fst`):

```bash
make verisim VERISIM_TRACE=1
```
