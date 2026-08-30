# 8. Verification

Sections 8.1-8.8 describe the M0 simulation scaffold as originally built: at
that point there was no CPU pipeline (`rtl/TestMaster.v` stood in for one),
and every claim in those sections should be read as a description of that
M0-era state, kept for the historical record and because the fabric/protocol
facts it documents (the tohost encoding, the AXI stack, the M2 restore
checklist) are still exactly true. `rtl/TestMaster.v` and `test/smoke/` are
now retired (design doc S4.3, M1 plan Task 4) — `rtl/RVProc.v` (IFU + ICache
+ BPU + FetchSink) is the real core shell in `RVProcAXI.v` today, and every
M1 test exercises the same tohost path TestMaster's smoke test did, plus a
great deal more. Section 8.9 is the M1 harness this milestone added: the
two-oracle architecture, the CLI flags, the directed test suite, the full
regression matrix and its slot-count invariant, and the unit benches. The
AXI fabric, memory controller, and interrupt controllers are still real RTL
(ported from rv12/rocketM). Section 8.6 remains the checklist for whoever
starts M2 and needs to retire the FetchSink-as-core-shell arrangement for a
real IDU/IU/RTU.

## 8.1 Simulation stack

```
bin/verisim/testbench (RVProcTest.cpp: TB::step, main)
    |
    |  RVProcAXI_Verilator() -- one call per simulated cycle
    v
dut.cpp (DUT::step)
    |  drives clk 0->1->0 across the Verilated model; moves the MEMCTLPin
    |  bus and the UART AXI4-Lite channel across the C++/RTL boundary
    v
Verilated RVProcAXI (rtl/RVProcAXI.v, obj/verisim/VRVProcAXI*)
    |
    +-- u_core = TestMaster.v (M0 scaffold: an AXI-exerciser FSM, not a CPU;
    |            axi_i_* (ICache-style master) is tied off/inactive,
    |            axi_d_* (DCache-style master) drives the whole smoke test)
    |
    +-- AXICrossbar (2 masters x 4 slaves, rtl/AXICrossbar.v)
             |
             +-- slave 0 (SI_MEM,   0x8000_0000) -> MEMCTL_AXI4L_step -> mpin
             |            (rtl/MEMCTL_AXI4L_step.RTL.v, C2RTL-generated)
             +-- slave 1 (SI_CLINT, 0x0200_0000) -> AXIWidthAdapter -> CLINT.v
             +-- slave 2 (SI_PLIC,  0x0C00_0000) -> AXIWidthAdapter -> PLIC.v
             +-- slave 3 (SI_UART,  0x1000_0000, ch_2) -> external AXI4L channel
                          (UART register file decoded at 0x1000_1000, §8.7)
```

Below the Verilated boundary, `TB::step()` in `RVProcTest.cpp` services the
two external ports every simulated cycle:

- `xmem.update(&io_pins.mpin)` -- `io/ExtMem.h`, backs `SI_MEM` (the `mpin`
  pins MEMCTL exposes). A byte-addressable, page-based host memory model.
- `uart_cvt.fsm(&axi_bus.s_ch[DI_UART], ...)` (`io/RVProc_io.h`'s
  `AXI4L::Converter`) plus `axi_uart.update(...)` (`device/uart16550.cpp`'s
  NS16550A model), backing `SI_UART`.

`DUT::sync(cpu)` (`dut.cpp`) mirrors architectural state (currently just
`gpr[]`) from the Verilated model into the global `CoreState cpu` after
every step. In M0 this is entirely compiled out: `rtl/verisim.h` defines
`VERISIM_NO_CPU_STATE`, which guards both the `DUT::init()` preload and the
`DUT::sync()` copy with `#ifndef` — there is no `CPU_PC`/`CPU_GPR` Verilator
internal-signal path to read yet, because `TestMaster.v` has no architectural
register file (it is not a CPU; see §8.5).

## 8.2 The tohost protocol

M0 reuses the riscv-tests/HTIF-style `tohost`/`fromhost` handshake, unchanged
from rv12's own scaffold:

1. **ELF symbol lookup.** `TestBench::parse_arg()` (`testbench/TestBench.cpp`)
   loads the target ELF via `load_elf()` (`testbench/load_elf.cpp`, using
   `libelf`) and resolves the `tohost`, `fromhost`, `begin_signature`,
   `end_signature`, and `_stack_top` symbols. The address itself has no
   special meaning beyond agreement between the linker script and the RTL:
   in M0 that address is `0x9000_1000`, fixed by `ADDR_TOHOST` in
   `rtl/rvproc_pkg.sv`, by the `TOHOST_ADDR` localparam in
   `rtl/TestMaster.v`, and by `test/smoke/smoke.ld`'s `.tohost` section
   placement.
2. **Host-side polling.** `TestBench::run()`'s main loop (`testbench/TestBench.cpp`)
   calls `step()` once per cycle and, whenever `tohost != (uint64_t)-1` (the
   symbol resolved), reads `read_mem(tohost)` every cycle. A value of `0`
   means "not done yet."
3. **Encoding.** Once nonzero, bit 0 of the tohost word distinguishes a
   completion code from a syscall pointer:
   - `tohost & 1`: completion, encoded as `(testnum << 1) | 1`. `testnum ==
     0` (i.e. `tohost == 1`) is PASS; any other `testnum` is a failing test
     number, printed as `FAIL. test no. = <testnum>`.
   - `tohost & 1 == 0` (nonzero): a pointer to a syscall argument block
     (`which`/`arg0`/`arg1`/`arg2`); the framework implements `which == 64`
     (write). Not exercised by `TestMaster.v` (it only ever performs the
     completion write) — this path exists for the firmware kit's
     `syscalls.c`, unused until a real core runs it (M2+).
   - `tohost >> 32 == 0x0101_0000`: single-character console output. Also
     unused by the M0 smoke test.
4. **10-cycle settle.** After the first nonzero read, `run()` waits 10 more
   cycles (`tohost_wait`, decremented once per cycle) before treating the
   value as final — a debounce inherited from the donor framework.
5. Once settled, `main()` (`RVProcTest.cpp`) returns whatever `run()`
   returns, and `run()` returns `out`, the raw tohost word — see §8.3 for
   why the process exit code is that raw word, not a conventional 0/1 code.

## 8.3 Build and run

```bash
make verisim
timeout 30 bin/verisim/testbench --print-result test/smoke/smoke.out
echo "exit=$?"
```

- **`--print-result` is required** to see `PASS.`/`FAIL. test no. = N` on
  stdout at all — `TestBench::run()` only prints those lines when the flag
  is set. Without it, a successful (or failing) run produces no PASS/FAIL
  text; you would have to infer the result from the exit code alone.
- **The process exit code is the raw tohost value, not a conventional
  0-means-success code.** `main()` returns `TestBench::inst->run(...)`,
  which returns `out` — the raw completion word read from `tohost`. So
  `exit=1` means PASS (`testnum==0`, encoding `(0<<1)|1`); `exit=3` means the
  first failing test is test number 1 (`(1<<1)|1`); and so on. This is by
  design (the riscv-tests convention) — do not "fix" it to return 0 on
  success.
- `timeout N` is a safety net, not part of the protocol: `TestMaster.v`'s
  own on-chip watchdog (13-bit counter, `rtl/TestMaster.v`) converts a
  stalled fabric into a reported `FAIL. test no. = 1` after 8192 cycles, so
  in practice the run terminates on its own well before any reasonable
  timeout.

## 8.4 Waveform tracing (FST)

```bash
make verisim VERISIM_TRACE=1
timeout 30 bin/verisim/testbench --print-result test/smoke/smoke.out
```

`VERISIM_TRACE=1` (`verilator.mk`) recompiles with `--trace-fst` and
`-DVERISIM_TRACE`, which makes `DUT::init()`/`DUT::step()` (`dut.cpp`) open
`run/dump.fst` and dump full-depth (`vdut->trace(tfp, 99)`) waves on both
clock phases of every step. Switching `VERISIM_TRACE` on or off invalidates
the previous build via the `TRACE_STAMP` file in `obj/verisim/`, so the
rebuild is automatic. Tracing is off by default (it costs simulation speed);
`run/` is gitignored, so the trace file never needs manual cleanup.

## 8.5 The M0 TestMaster smoke test

`rtl/TestMaster.v` stands in for the CPU inside `RVProcAXI.v`. It drives only
the DCache-style AXI master port (`axi_d_*`; the ICache port `axi_i_*` is
tied off/inactive) through a seven-state FSM:

1. `S_WRITE`: write a 64-byte line at `0x9000_2000` (`PATTERN_ADDR`) with
   sentinel values in AXI data lanes 0 and 7 (`PATTERN = 0xC906_5AFE_A5A5_0000`,
   `PATTERN7 = 0xDEAD_BEEF_0000_0007`) — exercising the full write path:
   crossbar arbitration, MEMCTL, `ExtMem`.
2. `S_R_ADDR`/`S_R_DATA`: read the same line back and compare both lanes
   against the sentinels, proving the write landed and the AXI data path
   routes both lanes correctly.
3. `S_C_ADDR`/`S_C_DATA`: read `CLINT_MTIME` (`0x0200_BFF8`) through the
   AXIWidthAdapter -> CLINT.v path.
4. `S_TOHOST`: write the completion word (`1` if every check above passed,
   `3` — i.e. `(1<<1)|1`, "test no. = 1" — otherwise) to `TOHOST_ADDR =
   0x9000_1000`.
5. `S_DONE`: park forever; `quitted` is tied to `1'b0` (completion is
   reported purely via `tohost`).

**Result:** the smoke test passes. `bin/verisim/testbench --print-result
test/smoke/smoke.out` prints `test/smoke/smoke.out: PASS.` and exits with
`exit=1`.

**Fail-detection path, verified real.** To confirm the harness can actually
detect a failure (and is not just always printing PASS), the lane-7 compare
on the `S_R_DATA` state was temporarily inverted — from
`axi_d_rdata[DATA_WIDTH-1 -: 64] != PATTERN7` to the complementary sense, so
a check that should pass now deliberately fails — and the simulator was
rebuilt and rerun. This reproduced `test/smoke/smoke.out: FAIL. test no. =
1` with `exit=3`, confirming the tohost/testnum encoding and the
`--print-result` path both work in the failure direction, not just the
success direction. The change was then reverted, rebuilt, and `PASS.`/
`exit=1` reconfirmed before proceeding.

## 8.6 Open items

- **CLINT read path.** The M0 plan flagged a risk that the
  `AXIWidthAdapter` -> `CLINT.v` read (`S_C_ADDR`/`S_C_DATA` in
  `TestMaster.v`) might stall and require dropping those states from the
  FSM. In this rv906 run, the CLINT read completed cleanly on the first
  attempt and no fallback was needed — the risk did not materialize, and the
  states remain in the FSM as written.
- Everything else considered open for M0 is folded into the M2 restore
  checklist below, plus the PLIC `int_src` item, which is called out
  separately since M6 (not M2) owns it.

## 8.7 M2 restore checklist

The M0 scaffold intentionally elides architectural-state plumbing that only
matters once a real CPU core exists. This list is for whoever starts M2 and
needs to retire `TestMaster.v`:

1. **Define `CPU_PC`/`CPU_GPR` and drop the no-state guard.** `rtl/verisim.h`
   currently only does `typedef VRVProcAXI VDUT;` plus `#define
   VERISIM_NO_CPU_STATE 1` — there is no `CPU_PC`/`CPU_GPR` Verilator
   internal-signal path defined because `TestMaster.v` has no architectural
   register file to point at. M2 must add those paths (once the real core's
   register file exists) and remove `VERISIM_NO_CPU_STATE`, so the
   `#ifndef VERISIM_NO_CPU_STATE` blocks in `DUT::init()` and `DUT::sync()`
   (`dut.cpp`) compile in and actually preload/mirror `pc`/`gpr[]`.
2. **Restore a `read_mem` D-cache-mirror fast path once the real DCache
   exists.** `TB::read_mem()` (`RVProcTest.cpp`) currently always reads
   straight through `ExtMem` (there is no D-cache in the RTL to mirror or
   go stale relative to). Once M2/M3 add a real DCache, restore a fast path
   that reads through a mirror of the cache array instead of always hitting
   `ExtMem`, and account for the write-hazard this reintroduces (a
   `write_mem()`/`write_byte()` from the host side can leave a stale mirror
   entry if the RTL's own D-cache still holds the old line).
3. **Extend `CoreState`/`sync()` to carry the cache mirror.** `DUT::sync()`
   only copies `gpr[]` today. When item 2 lands, extend `CoreState` and
   `sync()` to carry the D-cache mirror too — consider syncing it lazily
   (only when `TestBench` actually needs it, e.g. around a `read_mem`
   fast-path lookup) rather than copying a full cache mirror every cycle.
4. **Prefer a `RESET_VECTOR` parameter over a post-reset `CPU_PC` poke.**
   `rtl/RVProcAXI.v` already instantiates `u_core` with a `RESET_VECTOR
   (64'h8000_0000)` parameter (and `TestMaster.v` declares a matching
   `RESET_VECTOR` module parameter), but `TestMaster.v` has no PC/fetch logic
   to consume it — the parameter is currently structural only. `dut.cpp`'s
   `DUT::init()` still pokes `rootp->CPU_PC = pc` directly (compiled out
   under `VERISIM_NO_CPU_STATE` in M0). When the real core lands, wire
   `RESET_VECTOR` through to wherever the core actually initializes its PC
   at reset, and prefer that over relying on the post-reset poke as the
   primary reset mechanism (the poke can remain as a convenience for
   redirecting a test's start PC, but boot-time reset should not depend on
   it).
5. **MEMCTL's AXI slave port caps bursts at 16 beats.** Confirmed in
   `rtl/RVProcAXI.v`: `mem_raddr_m_len` and `mem_waddr_m_len` are declared
   `[3:0]` and assigned from `s_arlen[SI_MEM][3:0]` / `s_awlen[SI_MEM][3:0]`
   (around lines 434/439/477/483) before being handed to
   `MEMCTL_AXI4L_step`. AXI `len` is otherwise 8 bits end to end (see the
   `axi_i_awlen`/`axi_d_awlen` port widths in `TestMaster.v` and the
   crossbar's `m_awlen`/`s_awlen` buses), but MEMCTL only ever sees the low
   4 bits — i.e. a cap of 16 beats per burst. The real core's ICache/DCache
   AXI masters must never issue a burst longer than 16 beats against this
   memory port, or the high bits of `len` are silently truncated and the
   extra beats will be misinterpreted by MEMCTL.
6. **UART/Converter C++ path compiles but is unexercised.** `uart_cvt.fsm()`
   (`AXI4L::Converter`, `io/RVProc_io.h`) and `axi_uart` (an NS16550A model,
   `device/uart16550.cpp`) are wired into `TB::step()` (`RVProcTest.cpp`)
   and build cleanly, but `TestMaster.v` never issues a UART transaction, so
   this path has never actually moved a byte. Treat it as unverified until
   M2 firmware (`test/hello.c` et al., built but not yet run — see §8.8)
   exercises it for real.
7. **ISA staging (RVC).** The firmware kit (`test/Makefile`) builds with
   `-march=rv64imac_zicsr` for every test except `test/smoke`, which
   deliberately builds `-march=rv64i` only (see `test/smoke/Makefile`). A
   real RV64IMAFDC core (C906's baseline, per the design doc) that does not
   yet decode RVC cannot run the firmware kit's normal build at all. The M2
   sub-spec must decide between (a) landing RVC decode as part of the M2
   core itself, or (b) adding a no-C fallback build target
   (`-march=rv64im_zicsr` or similar, no `A`, no `C`) to the firmware kit so
   early core bring-up has something it can actually run before RVC decode
   exists.
8. **PLIC's `int_src` has no UART-IRQ wire yet.** `rtl/RVProcAXI.v` ties the
   PLIC's external interrupt-source bus to zero: `.int_src (8'b0), // No
   external interrupts for now`. The UART model
   (`device/uart16550.cpp`) is not yet wired to raise an interrupt through
   PLIC to the core. This is an explicit M6 to-do (interrupt path
   integration), not M2 — M2's scope is the integer execute/retire path,
   not interrupts.

### 8.7.1 M2 close-out (per-item disposition)

Contract 17 asked M2 to resolve items 1/4/5/7 and record the disposition of
2/3/6/8. Final disposition:

1. **RESOLVED.** `rtl/verisim.h` now defines `CPU_PC`/`CPU_GPR` Verilator
   internal-signal paths against the real IDU register file
   (`IDU.gpr_r[*]`) and retire PC (`RTU.ex2_cur_pc`), and
   `VERISIM_NO_CPU_STATE` is removed so `dut.cpp`'s preload/mirror compiles
   in.
2. **NOT RESTORED (deliberate).** `TB::read_mem()` still reads straight
   through `ExtMem`. With a real write-back DCache + STB, a host-side
   D-cache mirror would reintroduce exactly the stale-mirror write-hazard
   the item warns about, and the M2 tests never need host reads of cached
   memory (they self-report over `tohost`). Deferred to whoever first needs
   it (most likely a later debug milestone).
3. **NOT DONE (deliberate, subsumed by item 2).** `CoreState`/`sync()` carry
   only `gpr[]` (plus PC). No cache mirror is carried, consistent with item 2.
4. **RESOLVED.** `RESET_VECTOR` is wired through to the core's reset PC via
   `cp0_xx_mrvbr` (CSR.v) rather than relying on a post-reset poke; the poke
   remains only as a convenience.
5. **RESOLVED.** The ICache/DCache AXI masters issue single-beat 64-byte
   bursts (`awlen`/`arlen` = 0, `awsize`/`arsize` = 6), well under MEMCTL's
   16-beat cap, so the low-4-bit `len` truncation is harmless.
6. **STILL UNEXERCISED (as predicted).** The UART/Converter C++ path still
   builds but never moves a byte in M2 (no firmware runs it). Unchanged from
   M0; first real exercise belongs to the firmware/debug milestones.
7. **RESOLVED via option (a): RVC decode landed in the M2 core.** `IDU.v`
   decodes RVC directly (`is32` select + 16-bit `casez`), and `rv64uc-p-rvc`
   (a local compressed-instruction test) passes with caches on. The firmware
   kit keeps its `-march=rv64imac` build now that the core decodes C.
8. **DEFERRED TO M6 (as predicted).** PLIC `int_src` still tied to zero;
   interrupt-path integration is M6, not M2.

**Additional M2 acceptance notes (recorded here per contract 17's spirit):**

- **`rv64ui-p-ma_data` is the one riscv-tests M2 exception.** It exercises
  hardware misaligned load/store, which the design doc §2.3.3 (contract 3)
  deliberately defers to M4 (M2 is trap-only for misalignment). The design
  doc's premise that "rv64ui exercises aligned accesses only" is wrong for
  this one test; it is documented here rather than silently skipped. All
  other 53 `rv64ui-p-*` + all 13 `rv64um-p-*` + `rv64uc-p-rvc` pass with
  caches on.
- **Caches-off sanity cross-check** (Task 10.1's non-gate run): 66/68.
  Besides `ma_data`, `rv64uc-p-rvc` fails with caches off (it passes caches
  on). This is a caches-off-specific fetch-path discrepancy flagged for
  follow-up, not part of the caches-on acceptance gate.

## 8.8 What "build-only" means for the firmware kit

`test/` (hello/clint_test/plic_test/intr_test, plus the shared `entry.S`,
`link.ld`, `syscalls.c`, `uart.c`/`uart.h`, `printf.c`) builds cleanly with
the xpack toolchain in M0 (`make -C test`), but none of these `.out` ELFs
are ever loaded into the simulator or run — there is no CPU to run them.
They exist to prove the toolchain and link script are wired correctly ahead
of M2, which is the first milestone with a core capable of executing them
(subject to the RVC decision in §8.7 item 7).

## 8.9 The M1 harness

M1 has a fetch pipeline (`rtl/IFU.v`), an instruction cache (`rtl/ICache.v`)
and a branch predictor (`rtl/BPU.v`) but no decode or execute stage — there
is no register file and no ALU to check results against. What M1 verifies
is narrower and different in kind from a normal CPU test: not "did the
program compute the right answer" but "did the front end fetch and commit
the right *stream of instructions, in the right order*, regardless of which
predictor configuration was steering it." `docs/02-ifu.md` §6 and
`docs/03-bpu.md` §5 point back here for exactly this reason.

### 8.9.1 Two-oracle architecture

Two independent implementations of the same behavioral contract (design doc
S4.1) are compared online, every simulated cycle:

- **`rtl/FetchSink.v`** (RTL) stands in for IDU + IU + RTU + CP0. It
  consumes the IFU's single delivered instruction per cycle, plays a fake
  branch unit (a fixed direction rule `taken = ^pc[7:4]`, decoded B/CB/J/CJ
  immediates, a 16-entry shadow call stack, the `JR_TARGET(pc)` formula for
  every other indirect jump) and a fake retire unit (retire pulses that
  drive BPU's architectural GHR shift and RAS/BTB updates), hosts the
  harness's config register bank (§8.9.2), and reports over `tohost` exactly
  as M0's `TestMaster.v` did.
- **`m1_iss.h`**, a C++ golden fetch-ISS driven from `RVProcTest.cpp`, walks
  the *same* contract over the ELF image loaded in `ExtMem` — its own
  independent shadow call stack, its own `JR_TARGET` formula, its own
  sentinel-stop logic.

**Neither was ported from the other.** They were implemented independently
from the spec text specifically so that a shared bug between them validates
nothing (the same rule rv12's own M1 harness used). `RVProcTest.cpp`'s
`M1Checker::slot()` samples the committed instruction via `verisim.h`'s
`m1sink` accessors every cycle, compares `(pc, opcode)` against the ISS's
next expected entry in order, and on the first divergence prints both
streams' last 16 entries plus the cycle number before exiting
(`M1_FAIL_STATUS = 2`).

The one place FetchSink has to *observe* the front end rather than purely
model it is the jalr-family mispredict rule: since FetchSink cannot see BPU
internals, it compares the actual target (from its own shadow stack or
`JR_TARGET`) against **the PC delivered behind the jump** — whatever the
front end actually fetched next, be that a RAS pop at rung 2, a BTB-fed
sequential continuation at rung 1, or anything else. One comparison rule
therefore covers every rung without a rung-specific special case.

### 8.9.2 CLI flags

Parsed out of `argv` in `RVProcTest.cpp` before `TestBench::parse_arg` ever
sees it (the M0 harness's own `parse_arg`, not Verilator plusargs — same
mechanism rv12 used), then poked into FetchSink's config register bank via
`verisim.h`'s `m1sink` accessors after `dut.init()`:

| Flag | Meaning |
|---|---|
| `--m1-rung=<1..4>` | predictor chicken-bit ladder, cumulative: 1 = all predictors off, 2 = +RAS, 3 = +BTB, 4 = +BHT |
| `--max-insts=<N>` | FetchSink's commit budget (default 200000); exhaustion is `perr_code=1`, not a tohost value. Tested at commit-group granularity, so a run can end slightly past N — a documented floor, not an exact count |
| `--inv-test` | pulse `cfg_bht_inv`/`cfg_btb_clr` mid-run, one at a time, spaced far enough apart that the 1024-cycle BHT sweep always completes before the next pulse |
| `--sink-stall` | FetchSink's pseudo-random `id_stall` mode, exercising the stall/backpressure path independently of the redirect path |
| `--fencei-patch=<addr>:<word32>[:<commit>]` | the fence.i mechanism (`test/m1/fencei.S`): at commit boundary `<commit>` (default 1000, a count of instructions the checker has already compared, not a cycle), the host writes `<word32>` into the ELF image in `ExtMem` at `<addr>` and pulses `cfg_icache_inv` for one cycle. Both oracles switch images at exactly that boundary — the RTL because the invalidate forces a refill from patched memory, the ISS because it reads the image lazily at the moment each instruction commits. The run FAILS if the patch never fires, so `fencei.S` cannot pass vacuously against an unpatched image |
| `--iss-selftest` | run the golden ISS's own gate (27 hand-derived entries from `test/m1/iss_selftest.S`'s real disassembly) and exit — no RTL involved |
| `--no-checker` | sample and compare nothing — a debug escape hatch for telling a raw RTL hang/crash apart from a bug in the checker itself |

These four (`--m1-rung`, `--max-insts`, `--inv-test`, `--sink-stall`) were
the ones the plan pinned in advance; the other three
(`--fencei-patch`, `--iss-selftest`, `--no-checker`) were added as the test
suite needed them during Tasks 5-6, the same precedent rv12's own harness
set.

### 8.9.3 The RAS-faithful grading path

The real `aq_ifu_ras.v` is 4 flop entries with pointer-only misprediction
resync — correctness is only guaranteed for ≤4 in-flight unresolved
call/return predictions (design doc S2.1, `docs/03-bpu.md` §4.3).
FetchSink's general-purpose correctness oracle is a 16-entry shadow stack,
deliberately much deeper than the real RAS, because the *checker's* job is
to know the ground-truth committed stream at every rung, not to reproduce
the real RAS's shallow-depth behavior.

A **second, diagnostic-only** model, `FetchSink.v`'s SECTION RAS-FAITHFUL
GRADING MODEL, mirrors `aq_ifu_ras.v`'s algorithm exactly (one one-hot
4-bit pointer, one physical 4-entry array, no content resync, no
empty-stack special case) so that a trace can *confirm* `test/m1/callret.S`'s
past-depth-4 nesting is actually exercising the real limitation, rather than
merely trusting that it does. It does not gate pass/fail — the online
checker's committed-stream comparison is already invariant to RAS
prediction accuracy by construction.

**Documented limitation**: this grading model is driven by FetchSink's own
*committed* push/pop events, so it structurally cannot see genuine
speculative/wrong-path RAS activity inside `BPU.v` — a real RAS
misprediction that gets corrected before anything commits is invisible to
it. `BPU.v`'s own trace is the only ground truth for that; the grading
model is a best-effort cross-check on top of the checker, not a replacement
for reading the real RAS's behavior directly.

### 8.9.4 Unit benches (`test/m1/unit/`, `make -C test/m1/unit run`)

Three standalone benches, each proving one oracle or one RTL module against
an independently-written model, without the SoC around them:

- **`icache_tb.cpp`** drives `ICache.v`'s fetch port directly against a
  golden memory array behind a behavioral AXI slave: hit after refill,
  both-ways fill and FIFO replacement, alias-free indexing, live
  RVC-boundary detection against a C++ reimplementation, INV_ALL then
  re-miss, the uncached path, and AXI-error → access-fault reporting.
- **`fetchsink_tb.cpp`** drives `FetchSink.v`'s single-instruction intake
  port with hand-built delivery sequences and asserts the contract
  directly: the squash rule on a mispredict, correct-prediction commits,
  shadow-stack push/pop including pop-on-empty, the `JR_TARGET` formula, the
  resolve-signal protocol, and sentinel/tohost reporting. This is also
  where the `--sink-stall` port-wiring bug (`docs/02-ifu.md` §5.5) was
  caught, by its own `T14` (`test_sink_stall_mode`).
- **`iss_tb.cpp`** is the golden ISS's own gate (plan Task 5.3): a plain
  C++ program with no RTL dependency at all, run *before* `m1_iss.h` is
  ever trusted to grade FetchSink/IFU/ICache. It is the same check
  `--iss-selftest` runs inside the full testbench binary, built standalone
  here so the ISS can be gated in isolation.

Each bench prints one line per check and ends `UNIT-PASS`/`UNIT-FAIL`;
`make -C test/m1/unit run` builds and runs all three regardless of an
individual failure (so one broken bench never hides the other two) and
prints `UNIT-SUITE-PASS`/`UNIT-SUITE-FAIL` at the end.

### 8.9.5 The full regression matrix and its slot-count invariant

`test/m1/run_all.sh` has two modes. The original single-rung mode
(`test/m1/run_all.sh` or `--m1-rung=N`) is the bring-up loop Tasks 6-9 used
while landing each rung. `test/m1/run_all.sh --full-matrix` (plan Task 10.1)
is the milestone's acceptance gate:

```
11 tests x 4 rungs x --sink-stall {off,on}                        =  88 runs
11 tests x rung 4 x --sink-stall {off,on} x --inv-test             =  22 runs
                                                                    -----------
                                                                      110 runs
```

The 11 tests are `seq`, `rvc_mix`, `jal_chain`, `callret`, `ind_jr`,
`dense_br`, `thrash`, `uncached`, `fencei`, `mixed`, `iss_selftest` — the M1
spec's directed suite (S4.2). (C906's live, non-precomputed RVC-boundary
detection means the `missigned`-style replay-livelock test rv12 needed for
C910's precomputed bry0/bry1 phases has no analogous failure mode to
exercise here — confirmed and *not* built, per the plan's own instruction
to drop a vacuous test rather than author one for coverage's sake.)

**The invariant the script asserts, not just reports**: design doc S4.1
states that a predictor changes *when* an instruction is fetched, never
*which* instructions commit. `run_all.sh --full-matrix` parses each run's
`[checker] <N> instructions compared ...` line (`RVProcTest.cpp`'s own
`term()` printf) and requires that count to be **identical across every one
of a test's ten runs** (four rungs × two stall modes, sharing the rung-4
`--inv-test` pair's expected count). A differing slot count is a loud,
nonzero-exit failure even when every individual run reports PASS — this is
the online checker's own predictor-agnostic design promise, made into an
automated, per-test assertion rather than left as something a human has to
notice by eye.

**Result, current tree: 110/110 PASS, slot-count invariant holds for all 11
tests.** Cycle counts, `--sink-stall` off, by rung (columns `r1`-`r4` are
the four rungs; `r4i` is rung 4 with `--inv-test`; `slots` is the invariant
count):

| test | r1 | r2 | r3 | r4 | r4i | slots |
|---|---|---|---|---|---|---|
| seq | 452 | 452 | 452 | 452 | 452 | 320 |
| rvc_mix | 1566 | 1566 | 1566 | 1566 | 1566 | 1208 |
| jal_chain | 3051 | 3051 | 3051 | 3051 | 3051 | 526 |
| callret | 275 | 266 | 266 | 266 | 266 | 60 |
| ind_jr | 300 | 300 | 300 | 300 | 300 | 28 |
| dense_br | 2801 | 2805 | 2803 | 2837 | 2813 | 809 |
| thrash | 24374 | 24374 | 24374 | 24374 | 24374 | 15885 |
| uncached | 185 | 185 | 185 | 185 | 185 | 34 |
| fencei | 345 | 345 | 345 | 345 | 345 | 30 |
| mixed | 909 | 909 | 909 | 909 | 909 | 445 |
| iss_selftest | 175 | 175 | 175 | 175 | 175 | 39 |

Reading it: `callret` is the RAS test and rung 2 is where it pays (RAS
absorbs mispredicted returns that rung 1 has to resolve through a full
flush/refetch); `dense_br` moves at every rung as BHT/BTB engage, including
*upward* at rung 4 — consistent with `docs/03-bpu.md` §4.1's GHR
read/write-window finding, which predicts a real but harmless
accuracy/cycle-count cost, never a correctness one; `thrash` and `ind_jr`
are flat because their control flow's predictable component (capacity
misses, or an indirect target that is a pure function of PC) does not
change with the predictor set; every test's `slots` column is identical
across the whole row, which is the invariant itself, made visible.

## 8.10 What M1 does not cover

M1 has no execute stage, so nothing here validates instruction *semantics*
— only which instructions are fetched, in what order. There is no register
file and no ALU, so a delivered instruction's opcode is checked, never its
operands or its result. Privilege switching does not exist (no CSR file, no
trap path before M2), and the only self-modifying code exercised is the
host-driven `fence.i` patch (§8.9.2) — nothing here proves the RTL's own
fence.i is sufficient once a store unit can trigger it for real. Those
arrive with M2 and M4; the harness's `term()` checks in `RVProcTest.cpp` are
written to fail loudly rather than silently pass once those assumptions
change.

## 8.11 The M2 harness

M2 is the first milestone with a full execute/retire path, so its oracle
changes kind: from "did the front end fetch the right stream" (M1) to "did
the program compute the right architectural result." M2 uses two layers:

1. **riscv-tests self-checking suites** (`test/m2/`, `make -C test/m2`): the
   upstream `rv64ui-p-*` (53 tests) and `rv64um-p-*` (13) plus a local
   `rv64uc-p-rvc` compressed-instruction test (68 ELFs total), built
   `-march=rv64imc_zicsr_zifencei` against the vendored p-env
   (`test/m2/env/`). Each test self-reports over `tohost` (`PASS`/`FAIL test
   no. = N`). The build compiles the upstream `.S` in place from the
   workspace riscv-tests checkout (recorded in `test/m2/Makefile`); only the
   env (linker script + p-env header) and `rvc.S` are vendored. A boot
   preamble (`RV906_BOOT_MHCR`, on by default) enables MHCR.ie/de so the
   acceptance sweep genuinely exercises the DCache (design doc §7.3);
   `make -C test/m2 RV906_BOOT_MHCR=0` builds it out for the caches-off
   sanity cross-check.
2. **Per-unit benches** (`test/m2/unit/`, `make -C test/m2/unit run`):
   `csr_tb`, `iu_tb`, `rtu_tb`, `idu_tb`, `dcache_tb`, `mmu_tb`, `lsu_tb` —
   hand-scripted single-instruction stimulus against each module in
   isolation (the same tick()-based clocking as M1's benches), each asserting
   the module's contract (decode values, hazard stalls, cache hit/miss/
   invalidate timing, misalign trap, STB forward, etc.). This is the layer
   that localizes a failure to a module when a riscv-test trips.

**Bring-up ladder** (Task 9): before the full sweep, directed per-feature
tests (alu, bju, muldiv, ld_st, csr_trap) plus incremental rv64ui/um subsets
were used to localize the bring-up bugs (recorded in the module chapters'
"bugs found" sections: WBT cancel gate, GPR read-during-write, `c_lw_imm`
transpose, DIV dest latch, MULT issue-stall, sub-word store-miss sizing,
unconditional-jump redirect).

**Result, current tree:** caches-on acceptance sweep 67/68 (`rv64ui-p-ma_data`
is the documented M2 exception, §8.7.1); unit suite `UNIT-SUITE-PASS` (all 7
benches); caches-off sanity cross-check 66/68 (ma_data + the caches-off rvc
follow-up, §8.7.1).

## 8.12 What M2 does not cover

M2 validates the integer execute/retire path and the aligned-access memory
system. Not covered, by design-doc carve-out: hardware misaligned access
(`rv64ui-p-ma_data`, deferred to M4, §8.7.1), S/U-mode trap delegation and
the interrupt path (M6), debug (M7), FP/vector, and the caches-off RVC
fetch-path discrepancy (§8.7.1, follow-up). MMU translation is an
identity-map stub (M4). Atomics (LR/SC/AMO) are M3. Those arrive with their
milestones; the M2 harness's self-checking suites are written so each of them
fails loudly rather than silently passes once its feature lands.

## 8.13 M3 acceptance: atomics (rv64ua)

M3 adds the A extension (LR/SC + AMO, W and D widths) on top of the M2
integer machine. The oracle layers are the M2 ones, extended:

1. **riscv-tests `rv64ua-p-*`** (19 ELFs: 18 AMO + `lrsc`), built
   `-march=rv64ima_zicsr_zifencei` and appended to the same caches-on sweep
   (`test/m2/Makefile`'s `RV64UA_TESTS`).
2. **`lr_sc_tb`** in `test/m2/unit/` (18 tests): LR/SC basic paths, all
   W/D AMO ops, store→AMO tight-forward timing, the M3 stress set —
   warm-cache LR/SC retry loop, intervening-store reservation loss,
   reservation-consumption semantics (SC after successful AND after failed
   SC), byte_off=4 positioned AMO/SC writeback, LR.D/SC.D — plus the four
   audit regressions (T15-T18): STB-full store admission, STB-full AMO
   commit, misaligned-AMO recovery, and LR;LR;SC re-key. `idu_tb` T10 also
   checks that a reserved AMO funct5 decodes illegal.

**Result, current tree:** all 19 `rv64ua-p-*` PASS, including `lrsc`
(the 1024-iteration LR/SC accumulation loop, the barrier AMO, and the
sc-after-sc cases). Full caches-on sweep is 86/87 — the one failure remains
the documented `rv64ui-p-ma_data` carve-out (§8.7.1); misaligned AMO/LR/SC
trap under the same contract-3 rule (a misaligned SC reports the
store-misalign vector, since SC is a store per the A spec). Unit suite
`UNIT-SUITE-PASS`.

**What lrsc exposed (four bugs, all fixed in M3 close-out):**

- *Stale reservation exclusion.* The LR-buffer exclusion sampled `dc_hit_c`
  / `dc_is_store_r` unqualified, but in `ST_IDLE` the DCache response bus
  holds the previous transaction's hit-way — any idle cycle after a hit
  cleared a fresh reservation, so the loop's SC always failed and the test
  hung in its retry loop. Exclusion is now qualified to a real in-DCS
  response.
- *Successful SC never committed.* SC rides the load-like path
  (`func[0]=0`), so no STB entry was ever created for it; the accumulation
  loop spun on memory that never changed. A successful SC now commits
  through the ordinary STB create-or-merge path (with full-STB
  backpressure), `was_hit=dc_ca_r` since a cacheable SC-miss refills.
- *Stale SC forward at ST_DCS.* On the SC's first DCS cycle the ex2 forward
  carried `da_final` (memory) instead of the SC result, and the RAW-exempt
  forward let the consumer branch dispatch with the stale value (test 2).
  The SC result is now resolved combinationally at ST_DCS and forwarded.
- *Unpositioned AMO writeback.* The AMO STB entry stored its computed value
  in the extracted (value) domain with a dw_off-based mask; the STB holds
  positioned data. A W-AMO at byte_off≠0 (lrsc's barrier `amoadd.w` lives at
  offset 4 of its dword) drained zeros into the correct lanes. AMO data and
  byte mask are now positioned by the latched byte offset.

**What the post-close-out audit exposed (all confirmed, all fixed; each has a
regression test):**

- *Full-STB REPLY deadlock (HIGH; the store form predates M3, SC inherited
  it).* A completing store/SC/AMO whose completion needs a NEW STB slot
  stalled in `ST_REPLY` when all 4 slots were occupied — but drains start
  only from `ST_IDLE`, which the FSM never reaches while held: deadlock (a
  5th consecutive distinct-dword store hung the machine). Fix: STB-full
  *admission control* — `lsu_idu_full` now includes `stb_full &&
  ag_needs_slot_c`, holding a slot-needing op in EX1 (the IDU keeps it the
  same way it honors any `lsu_idu_full`) while drains free a slot; the
  REPLY-hold remains as defense-in-depth (T15).
- *AMO writeback dropped on a full STB (HIGH).* The AMO's STB entry was
  created the cycle AFTER read completion via a pending flag that was
  cleared regardless of whether a slot existed — a saturated STB silently
  lost the write. Fix: the AMO entry is created in the REPLY cycle itself
  (same create-or-merge path as store/SC, with the same backpressure and
  `was_hit=dc_ca_r`), so it can neither be dropped nor miss the fence/
  STB-empty quiescence window (T16). The donor creates its AMO entry at DC
  (`aq_lsu_stb.v:654`); same-cycle creation here provides the same
  guarantee.
- *Stuck `amo_active` after a trapped AMO (HIGH).* A misaligned AMO traps,
  but `amo_active` was only cleared inside the read-completion capture
  (which the trap skips) — every later completing LSU op was then mistaken
  for the AMO's read completion: its writeback ran through the W-width AMO
  sign-extend mux and a bogus ALU store was issued to its address. Fix:
  `amo_active` clears at the AMO's own REPLY, trap or not (T17).
- *AMO STB-create never merged (HIGH).* The AMO create only allocated,
  breaking the at-most-one-entry-per-dword invariant: an older same-dword
  store at a higher index drained after the AMO entry (drain is
  lowest-index-first) and overwrote the AMO result; the DA forward also
  merged only the lowest entry's mask while two entries shared a dword.
  Fix: the AMO create shares the store/SC merge branch.
- *Reservation gaps (MEDIUM, donor cross-check).* Exclusion now also fires
  for an AMO (the donor's lock monitor clears on SC *and AMO*,
  `aq_lsu_dc.v:1620`) including a missed or uncached one, and for any
  completing load (its refill may evict the reserved line — conservative,
  spec-legal). SC match now also requires the donor's size match
  (`lm_size == lm_req_size`, `aq_lsu_lm.v:159-161`). A trapped (misaligned)
  LR installs no reservation, and `rtu_lsu_expt_ack/exit` kill the
  reservation (donor `aq_lsu_lm.v:129,135` — matters once M6 interrupts
  land). LR-over-LR re-keys instead of destroying the reservation (donor
  `lm_set` overwrites in EXCL state) — previously LR#2's own DCS exclusion
  cleared `lr_addr_set` before the set-term fired (T18).
- *Reserved AMO funct5 executed as "store zero" (MEDIUM).* The IDU AMO
  catch-all accepted all funct5s and `amo_alu_compute` defaulted to 0. The
  donor's decode lists exactly the nine defined funct5s
  (`aq_idu_id_decd.v:2028-2052`); reserved values now decode illegal
  (`idu_tb` T10). Misaligned AMOs also report vector 6 (store/AMO address
  misaligned), same as SC.

**Documented deviations / model limits (M3):**

- **SC is load-like in this clone** (`LSU_FUNC_SC_W/D` keep `func[0]=0`);
  the donor's `FUNC_SC_W/D` are store-like (`aq_idu_cfig.h:520-523`; the
  donor LR encodings match this clone's in the functionally-active low-4
  func bits — load/sign/size — while the prefix bits differ). The deviation
  is contained: SC commits via an explicit STB-create at reply gated on the
  reservation match, and takes the store-misalign vector.
- **Reservation model:** one entry, exact-address match on PA[55:0] plus
  access-size match (donor-faithful), with conservative exclusion (any
  store, AMO, or completing load while a reservation is held clears it;
  every completed SC consumes it, success or failure; exception ack/exit
  clears it). The spec permits spurious SC failure, and single-issue
  in-order execution with no other-hart traffic makes forward progress
  structural (no timeout counter needed).
- **aq/rl bits are ignored.** Decode treats `inst[26:25]` as don't-care;
  the donor's split unit inserts `fence iorw,iorw` before `.rl` and after
  `.aq` atomics (`aq_idu_id_split.v:367-428`). Benign for a single-hart
  in-order core (and rv64ua uses plain atomics), but unbuilt and flagged
  here for any future SMP work.
- **LR.W sign-extends** exactly like LW (`func[1]=1`, matching the donor's
  `FUNC_LR_W` low-4 pattern).
- The M2 STB residual risk (a later miss's victim-pick overwriting an
  undrained entry's way, documented in `LSU.v` §STB) applies equally to
  AMO/SC STB entries.
