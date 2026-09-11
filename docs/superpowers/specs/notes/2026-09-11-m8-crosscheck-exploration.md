# M8 Exploration Note — Cross-Check vs. Original openc906 RTL

Source: M8 exploration subagent (2026-09-11), 128 tool uses. All paths
verified by that agent: `refs/openc906/...` relative to the rv906 repo
root, `WT/` = the m2-integer worktree, `RV12/` = the sibling rv12 root
(/home/vlsilab/zhouz/workspace/C2RTL/rvproc/RVProc6/vla/riscv/rv12/).

## 1. THE SMART_RUN FLOW

### 1.1 Entry points and simulator

- `smart_run/Makefile:16-17` includes `setup/smart_cfg.mk` + `setup/env_check.mk`. `SIM` defaults to **`iverilog`** (`Makefile:22`), with `vcs` and `nc` (irun) as alternates (`Makefile:35-54`). Icarus flags: `-g2012 -Diverilog=1` (`Makefile:49`). The donor flow ships with an open-source simulator as its default target — VCS is not required.
- Build/run: `make runcase CASE=<x>` → `compile` (compile RTL) + `buildcase` (compile test) + sim run (`Makefile:116-139`). `make regress` loops all cases and runs `tests/regress/report_gen.pl`, which greps each `run_case.report` for `TEST PASS`/`TEST FAIL` (`report_gen.pl:47-59`).
- RTL filelists: iverilog path uses `-f gen_rtl/filelists/C906_asic_rtl.fl -f gen_rtl/filelists/tdt_dmi_top_rtl.fl -c logical/filelists/smart.fl -c logical/filelists/tb.fl` (`Makefile:57`); VCS/nc use `sim.fl` which is just ip.fl + smart.fl + tb.fl (`logical/filelists/sim.fl:15-17`). `smart.fl` is library dirs (`-y`) for the SoC fabric; `tb.fl` compiles `logical/tb/tb.v`.
- RTL uses `${CODE_BASE_PATH}` env var in filelists (e.g. `C906_asic_rtl.fl:1`), so `CODE_BASE_PATH=refs/openc906/C906_RTL_FACTORY` must be exported. Test compile uses `${TOOL_EXTENSION}/riscv64-unknown-elf-gcc` (`tests/lib/Makefile:17`), checked by `env_check.mk:18-31` (warning only if unset).

### 1.2 Test program build

- `tests/lib/Makefile`: compiles all `*.s/*.S/*.c` in work dir with `CC = riscv64-unknown-elf-gcc`, `-mtune=c906`, and per `CPU_ARCH_FLAG_0` (`smart_cfg.mk:17` pins `c906fd` for every case): `-march=rv64imafdcxtheadc -mabi=lp64d` (`tests/lib/Makefile:35-45`). coremark gets `-O3` + `-DITERATIONS=10000` + T-Head-vendor flags (`-fno-schedule-insns`, `-mno-thread-jumps1`, ...) at `tests/lib/Makefile:49-53`; other cases get `-O2` (`:52`). Link: `-Tlinker.lcf -nostartfiles ... -lc -lgcc -lm` (`:55,77`).
- ELF → hex: `objcopy -O srec` three times (`.text`/`.rodata`, `.data`/`.bss`, whole file) then a vendored x86-64 static binary `tests/bin/Srec2vmem` converts to `inst.pat`/`data.pat`/`case.pat` (`tests/lib/Makefile:82-99`). The donor tb loads **hex patterns**, not the ELF directly.
- Linker script `tests/lib/linker.lcf:15-20`: `MEM1(RWX) ORIGIN=0x00000000 LEN=0x40000` (text+rodata), `MEM2(RWX) ORIGIN=0x00040000 LEN=0xc0000` (data+bss); stack `__kernel_stack = 0xee000` (`:20`); entry `__start`. **The entire program image lives below 0x100000 (1 MB)** and the reset vector is 0x0.
- crt0 `tests/lib/crt0.s:27-139`: sets `mxstatus[22]` (theisaee, `li x3,0x400000; csrs mxstatus` `:30-31`), `mstatus` FS bits (`:34-35`), zeroes GPRs, sets `sp=__kernel_stack`, `mtvec=__trap_handler` (`:114-115`), enables MIE (`:119-120`), pokes **mcor 0x7c2=0x30013** (cache invalidate) and **mhcr 0x7c1=0x7f** (I$/D$/BHT/BTB/RAS/WA enable) and **mhint 0x7c5=0x610c** (`:124-136`), then `jal main`.
- Exit protocol: `__exit` loads **x3 = 0x444333222** (`crt0.s:145-150`); `__fail` loads **x3 = 0x2382348720** (`:153-157`). No tohost, no memory-mapped exit — completion is a magic value written into an integer register.

### 1.3 PASS/FAIL detection in the tb

`logical/tb/tb.v`:

- Load: `$readmemh("inst.pat"/"data.pat")` into temp arrays, poked byte-wise into hierarchical RAM (`tb.x_soc.x_axi_slave128.x_f_spsram_524288x128_{L,H}`, `:32-33`) — inst at RAM offset 0, data at offset 0x4000×16B = 0x40000 (`:128-179`).
- **PASS**: sniff CPU writeback buses `x_aq_rtu_wb.wb_wb0_data/wb_wb1_data`; if either == `64'h444333222` → `TEST PASS` (`:257-270`). **FAIL**: == `64'h2382348720` → `TEST FAIL` (`:271-282`).
- **Hang detection 1**: `#MAX_RUN_TIME = 700000000` (700 ms sim time = 70M cycles at 10 ns clock) → FAIL (`:29,190-199`).
- **Hang detection 2 (watchdog)**: every `LAST_CYCLE=50000` cycles, if `retire_inst_in_period == 0` (no instruction retired in 50K cycles, retire = `tb_retire0` = `core0_pad_retire`) → FAIL (`:201-235`).
- **Console**: stores to AXI address **0x10015000** with `awlen==0` get `$write("%c", ...)` from the wdata lane selected by wstrb (`:284-309`) — also appended to `run_case.report`.
- **Cycle counter**: `cycle_count[31:0]` free-runs on clk, used only as the watchdog modulus; **never printed** (`:204-211`). No instruction popcount either.

### 1.4 Donor SoC memory map

From `logical/axi/axi_interconnect128.v:358-368` and `logical/apb/apb_bridge.v:34-59` and `logical/ahb/ahb.v:37-41`:

| Region | Address range | Slave |
|---|---|---|
| SRAM | 0x00000000 – 0x00FFFFFF (16 MB window) | `axi_slave128` (two `f_spsram_524288x128` banks; tb loads only first 1 MB) |
| ERR1 | 0x01000000 – 0x0FFFFFFF | axi_err (slv-error responder) |
| APB | 0x10000000 – 0x1FFFFFFF | axi2ahb → ahb → apb peripherals |
| ERR2 | 0x20000000 – 0xFF_FFFFFFFF | axi_err |

APB sub-decode (`apb_bridge.v:34-59`, wired in `apb.v:180-238`): **UART 0x10015000** (PS1, psel_s1), timer (DW-APB-timer style, 4× 32-bit timers) 0x10011000 (PS2), intc 0x10010000 (PS4 — note `prdata_s4` is tied 0 at `apb.v:227`, i.e. the intc peripheral is a **stub**), GPIO 0x10019000 (PS5), clkgen 0x10016000 (PS6), second timer 0x10017000 (PS7), bus_delay 0x1001A000 (PS8). AHB S1 (0x1F000000-0x1F01FFFF) → `mem_ctrl` (a second small RAM; `ahb.v:37-38,241`).

Interrupt lines: `xx_intc_vld[39:0] = {21'b0, stim[3:0], gpio[7:0], 1'b0, tim[3:0], 1'b0, uart}` (`apb.v:335`) → wired to `pad_plic_int_vld[39:0]` inside the CPU wrapper (`tr_axi_interconnect.v:960`) — i.e. the PLIC is **inside the openC906 core itself**, addressed via its own sys-APB at `pad_cpu_apb_base = 40'h4000000000` (`tr_axi_interconnect.v:963`). Reset vector base `pad_cpu_rvba = 0x0` (`:964`). Free-running `pad_cpu_sys_cnt` per-CPU-cycle counter feeds the `time` CSR (`:951-957`).

**rv906 comparison** (`WT/rtl/RVProcAXI.v:152-159`): MEM 0x80000000 (+2 GB), CLINT 0x02000000, PLIC 0x0C000000, UART 0x10000000; tohost 0x7FFFF000 (`WT/rtl/rvproc_pkg.sv:35`). Incompatibilities: (a) donor RAM at 0x0 vs rv906 RAM at 0x80000000 — every case must be **re-linked**; (b) donor UART 0x10015000 (custom 32-bit-reg UART, `uart.v`) vs rv906 16550 at 0x10000000 (32-bit stride regs, `WT/device/uart16550.cpp:8-61`) — different device model AND address; (c) donor in-core PLIC at 0x4000000000 vs rv906's standard PLIC at 0x0C000000 with SiFive-style register layout (`WT/rtl/PLIC.v:9`) — the `interrupt` case's PLIC addresses are incompatible; (d) donor `time` = per-cycle, rv906 `time` (0xC01) = CLINT mtime mirror ticking at **clk/100** (`WT/rtl/RVProcAXI.v:378-390` + `WT/rtl/CSR.v:1576`) — core self-measurement via `csrr time` is 100× coarser on rv906 today.

## 2. THE NON-VECTOR CASE SET

`smart_cfg.mk:18-30` `CASE_LIST` has **10 cases** (the `ISA_VECTOR` source dir `tests/cases/ISA/ISA_VECTOR/C906_VECTOR_FP_SMOKE.s` exists on disk but is **not in CASE_LIST** — the shipped config is c906fd, no vector). Every case is built `CPU_ARCH_FLAG_0=c906fd` (`smart_cfg.mk:32-99`). All are **bare-metal** (crt0 → main → magic-register exit; no OpenSBI, no OS). `debug` is special: its tb is a Verilog force-driver (`smart_cfg.mk:103-105` swaps SIM_FILELIST to `JTAG_DRV.vh` + `C906_DEBUG_PATTERN.v`), not a program run.

Mnemonic census (each case's full instruction set extracted from its `.s` source):

| Case | What it tests | Vector? | Non-standard insns used | Viable on rv906? |
|---|---|---|---|---|
| `ISA_INT` (`ISA/ISA_INT/C906_INT_SMOKE.s`, 2082 ln) | RV64IMAC smoke: ALU, mul/div, **AMOs** (`amoadd.d/w` etc.), LR/SC, compressed (`c.*`) | No | `srb/srd/srh/srw/surb/surd/surh/surw` — XThead **indexed store** mnemonics (XTheadMemIdx) | **No as-is** — uses T-Head indexed stores intermixed with standard ops; rv906 decodes none of them |
| `ISA_LS` (`ISA/ISA_LS/C906_LSU_SMOKE.s`, 2324 ln) | Load/store sizes + **`dcache.cva`/`dcache.civa`** + `sfence.vma` | No | `dcache.civa`, `dcache.cva` (XTheadCmo) | **No as-is** — CMO instructions unimplemented in rv906 (grep of `WT/rtl/*.v` finds no `dcache.` decode) |
| `ISA_FP` (`ISA/ISA_FP/C906_FPU_SMOKE.s`, 3886 ln) | F/D/**H** arithmetic: `fadd.h`..`fcvt.*.h`, `fmv.x.h`, `flh/fsh`, plus c.fld/c.fsd | No | All `*.h` half-precision ops = **Zfh** | **No as-is** — rv906 is F/D only (M5 row: "F/D, half-precision transfers"); `fadd.h` et al. need Zfh which rv906 does not implement |
| `ISA_THEAD` (`ISA/ISA_THEAD/C906_THEAD_ISA_EXTENSION.s`, 1548 ln) | The XTheadc extension itself: `addsl`, `rev/revw`, `ff0/ff1`, `srri(w)`, `ext/extu`, `tst/tstnbz`, `mveqz/mvnez`, `mula/mulah/mulaw/muls/mulsh/mulsw`, indexed load/store (`l*ia/ib`, `lr*`, `lur*`, `s*ia/ib`, `sr*`, `sur*`), FP indexed (`flr*/fsr*`) | No | ~60 XThead mnemonics | **No** — the case's entire purpose is T-Head extensions; rv906 decodes none |
| `MMU` (`MMU/C906_mmu_basic.s`, 302 ln) | sv39 1G-page map, satp mode/PPN/ASID, S-mode switch, PMP setup, `sfence.vma` | No | None (standard csrr/csrw/sfence.vma only) — but PTEs use **THEADFLAG** (T-Head extended PTE bits 59-62) via `MXSTATUS_MAEE 1` (`:266-267`, `MMU_PTW_1G ... 0xf` THEADFLAG `:281`) | **Borderline** — rv906 MMU ignores MAEE (`WT/rtl/MMU.v:679`); PTE reserved bits 59-62 set on rv906 → depends on rv906's PTE reserved-bit policy. Test itself is trivially weak (one sd/ld pair) |
| `interrupt` (`interrupt/C906_plic_int_smoke.s`, 171 ln) | PLIC claim/complete via in-core PLIC at **0x4000000000** (`:26`), wfi wake | No | None | **No as-is** — hard-codes the C906-internal PLIC base `PLICBASE_M=0x4000000000` and forces pending bit by **writing the read-only INTPEND register** (`:120-126`), which works on C906's sys-APB view but not on rv906's SiFive PLIC (0x0C000000, pending RO). Needs porting: drive a real PLIC source (rv906 UART IRQ at 0x10000000 is wired to PLIC, `WT/dut.cpp:80-85`) |
| `exception` (`exception/C906_Exception.s`, 515 ln) | Illegal-insn, ebreak, misaligned ld/st, PMP access faults, M/S/U ecall, mepc/mcause/mtval checks | No | `dcache.ciall` once (`:160`) + `csrci mhcr` (cache CSR); rest standard | **Mostly** — needs the single `dcache.ciall` line dropped/ported; checks exact mepc/mcause/mtval values → sensitive but standard-architected |
| `csr` (`csr/C906_CSR_OPERATION.s`, 75 ln) | csrrw/csrrs/csrrc + imm forms on `mstatus` only | No | None | **Yes, trivially** (weakest case — no value checks, just exercises encodings) |
| `cache` (`cache/C906_IDCACHE_OPER.s`, 158 ln) | `fence.i`; **mhcr/mcor/mcindex/mcins/mcdata0/1** CSRs; `icache.iall/ialls/iva/ipa`, `dcache.iall/call/ciall/isw/csw/cisw/iva/cva/civa/ipa/cpa/cipa` | No | All XTheadCmo + cache CSRs | **No** — rv906 implements `mhcr` (0x7C1) and `mxstatus` (0x7C0) but **not** `mcor`/`mcindex`/`mcins`/`mcdata0/1` (no hits in `WT/rtl/rvproc_pkg.sv` + `WT/rtl/CSR.v`) and none of the CMO instructions |
| `coremark` (`coremark/`, 7 C files) | CoreMark 1.0, `ITERATIONS=10000` (build flag; `core_portme.c:25` shows 1 if flag absent), prints `VCUNT_SIM: CoreMark ... one times cost %d cycles` (`core_main.c:79`) via `get_vtimer()` = `csrr time` (`clib/vtimer.c:18`) | No | None (C source; standard gcc emits no XThead regardless of march flag) | **Yes, after port** — needs: UART/console redirect (clib `fputc.c:19-20` hard-codes `0x6000fff8`; `uart.h:23` hard-codes `0x40015000` — **neither matches the actual tb UART at 0x10015000**, so on the donor side only the tb's 0x10015000 AXI-watch produces console output; coremark's own uart_init writes go nowhere visible); exit via magic reg; `time` CSR at clk/100 resolution issue on rv906 |

Notes: `debug` (`debug/C906_DEBUG_PATTERN.s` + `.v` + `JTAG_DRV.vh`) is a JTAG DTM/DM testbench — the "case" is a SystemVerilog `class ext_debug` (`JTAG_DRV.vh:936-1027`) driving TCK/TMS through the tb. It requires rv906's M7 debug unit and a JTAG-driver harness; **exclude from M8 functional parity** (it's M7's own territory, and it doesn't run "on" the CPU).

## 3. RUNNABLE ORACLE

**No smart_run ELF can run unmodified on rv906**, for three independent reasons:

1. **Link address**: donor images link at 0x0 (`linker.lcf:17`); rv906 RAM base is 0x80000000 (`WT/rtl/RVProcAXI.v:152`) and its ELF loader honors the ELF's program headers (`WT/testbench/TestBench.cpp:199` → `load_elf`), so a 0x0-linked ELF would load below MEM_BASE where rv906 decodes nothing. **Re-link is mandatory** — but since smart_run ships **sources**, not prebuilt ELFs, M8 rebuilds from source anyway.
2. **Exit protocol**: donor = magic value 0x444333222 in x3 at writeback (`tb.v:259`); rv906 = `tohost` symbol write, polled by the C++ harness (`WT/testbench/TestBench.cpp:188-199,256-285`), PASS iff tohost==1 (`:352`). rv906's RTL has no writeback-value sniffer. Options: (a) port each case's `__exit`/`__fail` (they live in shared `crt0.s`) to store 1/testnum to `tohost` — the crt0 is copied into every case build, so one ported crt0 fixes all cases; (b) add a Verilator sniff. **(a) is strictly better and is exactly the rv12 precedent** (RV12 M8 spec §2.3 P1, D-M8-4).
3. **Memory map / devices**: UART 0x10015000-custom vs 0x10000000-16550, PLIC 0x4000000000 vs 0x0C000000 (§1.4).

**Toolchain** (measured on this machine): the donor build wants `-march=rv64imafdcxtheadc` (`tests/lib/Makefile:40`). Measured results:

- `/opt/riscv/bin/riscv64-unknown-elf-gcc` 15.1.0: **rejects** `xtheadc` ("unsupported non-standard extension"); `-mtune=c906` rejected but **`-mtune=thead-c906` accepted** (it knows the c906 tune).
- `/opt/xpack-riscv-none-elf-gcc-15.2.0-1/bin/riscv-none-elf-gcc` 15.2.0 (the one rv906's test/m2+ Makefiles use, `WT/test/m2/Makefile:50`, `WT/test/m6/Makefile:31`): **rejects** `xtheadc` identically. But its binutils **does** know the decomposed `xthead*` family: `-march=rv64imafdc_xtheadba` assembles `th.addsl`, `_xtheadbb` assembles `th.rev/th.ff0/th.tstnbz`, `_xtheadcmo` assembles `th.dcache.*`/`th.icache.*`, etc. — **but only with the `th.` mnemonic prefix** (bare `addsl` fails; `th.addsl` passes). The donor's sources use the **bare** mnemonics (`addsl`, `ff0`, `dcache.civa`), so even the extension-aware assembler cannot assemble the donor's XThead cases unmodified.
- Consequence (identical to RV12's measurement 1): **no toolchain in this tree can produce the byte-identical `xtheadc` binary**. For the standard-RISC-V cases (csr, exception, coremark, MMU), `-march=rv64imafdc` (or `rv64imafdc_zicsr` as gcc-15 requires) builds them fine, and a standard gcc emits no XThead instructions from C sources regardless of the flag — so coremark compiled `rv64imafdc` is instruction-identical to coremark compiled `rv64imafdcxtheadc` with a standard compiler.

**Loader compatibility**: rv906's `load_elf` needs `tohost`/`fromhost` **symbols** (polled by symbol lookup, `TestBench.cpp:190-197`) — so the ported crt0/link script must emit them, matching the existing convention (`WT/test/m2/common.ld:38-39` puts `.tohost` at 0x7FFFF000; `WT/test/entry.S:204-210` defines the symbols). With a ported crt0 + link.ld at 0x80000000, the stock `bin/verisim/testbench --print-result <elf>` consumes the binaries directly (`WT/test/m2/run_all.sh:27` is the run pattern).

## 4. CYCLE COUNTS

**Donor side**: `cycle_count[31:0]` free-runs per clk in tb.v (`:204-211`) but is **never printed**; there is no retire popcount (only a 1-bit OR watchdog, `:233-234`). The program-visible cycle source is `pad_cpu_sys_cnt` (per-CPU-cycle, `tr_axi_interconnect.v:951-957`) which the C906 `time` CSR mirrors — CoreMark's `vcycles = vtimer_end - vtimer_start` (`coremark/core_main.c:77`, via `clib/vtimer.c:18` `csrr time`) is therefore a per-cycle region measurement, printed as the `VCUNT_SIM:` score line. For M8 the donor-side tb would need a 2-line `$display` patch at the PASS branch (rv12 pinned exactly this as a tracked `.diff`, RV12 spec P4) — but see §5: this is only relevant if we run the donor RTL at all.

**rv906 side**: `mcycle`/`minstret` are real per-cycle/per-retire counters (`WT/rtl/CSR.v:1230-1234`, `:1573-1577`); the C++ harness counts `dut.step()` calls (the `cycle` variable in `WT/testbench/TestBench.cpp:239-245`, printed every 1M cycles). **`time` (0xC01) currently reads CLINT mtime at clk/100** (`WT/rtl/CSR.v:1576` + `WT/rtl/RVProcAXI.v:385`) — so CoreMark's self-measured `vcycles` would be 100× too coarse on rv906 unless M8 either (a) re-points `time` to `mcycle` (rv12's P2/D-M8-2 decision, verbatim applicable) or (b) measures region cycles harness-side via `minstret`/`mcycle` sampled around the benchmark markers. rv12 chose (a): one CSR read-arm change in CSR.v.

## 5. RV12 PRECEDENT

**Yes — rv12 has a complete, unexecuted M8 design spec**: `RV12/docs/superpowers/specs/2026-09-11-m8-crosscheck-design.md` (762 lines, status "Draft for review", blocked on M5/M6/M7 closing; `git status` shows it and the sibling M6/M7/M9 specs uncommitted; no `test/m8/` exists yet). It is the direct process precedent and most of its analysis transfers to rv906 with renamed constants. Key decisions it pinned:

- **P1 (comparison architecture)**: Design-1 *port* (re-link same sources per platform) over Design-2 *emulate* (make the clone masquerade as the C910 memory map). Rationale: the clone's harness is proven; binary count is small; the clone boundary is the core, not the SoC. Applies to rv906 verbatim.
- **P2 / D-M8-2**: decode `time`/`timeh` as per-cycle `mcycle` (split from CLINT's clk/100 `mtime`), because CoreMark's `get_vtimer()` reads `time`. rv906 has the identical gap and the identical fix applies.
- **P3**: fixed benchmark config = same cache-enable CSRs poked on both sides; unmodeled donor CSRs are no-ops. For rv906 the analogous pin is `mhcr=0x7f` + `mcor=0x30013` from the donor crt0 (`tests/lib/crt0.s:124-136`); rv906 implements `mhcr` (`WT/rtl/CSR.v:1343`) but **not `mcor`** — a `csrs 0x7c2` on rv906 would trap illegal-CSR; **the ported crt0 must drop the mcor poke**.
- **P4**: donor-side tb needs a tracked instrument patch (popcount + `$display` cycle/retire counts at PASS). Needed for rv906's M8 only if we run the donor RTL.
- **P5**: case classification into first-pass core / extension / second pass, with explicit exclusions (debug, sleep) — the table in §2 mirrors this.
- **P6**: measurement region delimited program-side by the binary's own `time` reads; boot excluded.

**Can the donor C906 RTL itself be simulated here?** Yes, in principle: smart_run's **default SIM is iverilog** (`Makefile:22`) and iverilog 12.0 + Verilator 5.020 are installed (`/usr/bin/verilator`, `/usr/bin/iverilog`). The iverilog path needs no license. Unknowns: (a) whether the full C906 gen_rtl compiles under iverilog-12 — the donor shipped this path, so presumably yes at their tested version; (b) Verilator on C906 gen_rtl is NOT shipped by the donor (no `tb_verilator.v` exists in openc906's smart_run, unlike openc910) — would be new work. The toolchain side (`TOOL_EXTENSION`) is satisfiable with `/opt/riscv` or xpack gcc using `rv64imafdc` + a march override (the donor Makefile hard-codes `xtheadc` at `tests/lib/Makefile:40`, so a one-line override is needed).

**Re-scoping decision this forces**: the donor side CAN likely run under iverilog on this machine, so the full rv12-style two-simulator cross-check is feasible. But it is a heavier lift than rv12's (openc906 ships no Verilator tb; iverilog is slow for 70M-cycle runs like CoreMark-10000). The M8 gate per the umbrella is *functional parity* (`WT/docs/.../2026-08-20-rv906-c906-clone-design.md:318`: "cycle-count comparison noted, not gated"), so a defensible ladder is: (a) first pass = donor case sources run on rv906 sim with **expected results taken from the case sources themselves** (each case is self-checking: `bne ... TEST_FAIL` chains ending at the magic `__exit`/`__fail` registers — the expected result is "reaches __exit", which rv906 observes as tohost==1 after the crt0 port); (b) second pass = donor RTL under iverilog for the same cases, PASS verdicts + cycle counts compared. smart_run carries no separate expected-result files — the case sources ARE the expected results (self-checking), plus `run_case.report` TEST PASS strings and console output for coremark (`VCUNT_SIM:` lines).

## 6. RISKS + PROPOSED M8 TASK LIST

### Risks, ranked

1. **R1 — XThead cases are unrunnable, and that's most of the interesting surface.** 4 of 10 cases (ISA_THEAD, cache, ISA_LS via `dcache.c*` ops, ISA_INT via indexed stores) use XThead instructions rv906 deliberately doesn't implement (rv906 = RV64GC+F/D per design doc §7.4 rows M2-M5; XThead is not in rv906's milestone set at all, unlike rv12 which has an XThead milestone). **The umbrella M8 row says "smart_run (non-vector) binaries" — but 4 of the 10 non-vector cases are non-standard-ISA.** M8's design doc must explicitly re-scope the case set, or rv906 grows XThead decode (a whole milestone rv12 spent a milestone on). Recommendation: re-scope to the 5 standard cases (csr, exception, MMU, interrupt, coremark) + optionally a **filtered ISA_INT/ISA_FP** (the XThead sub-tests could be `#if 0`'d out at the source level; FP needs Zfh or the `.h` tests dropped).
2. **R2 — Donor-side sim bring-up cost.** openc906 ships iverilog flow but no Verilator tb; iverilog on 139K-line RTL + up to 70M-cycle cases is slow (CoreMark 10000 iters on an interpreted sim could take hours-days). Mitigation: gate donor-side runs to the smaller cases first; try Verilator on gen_rtl as a stretch task (C906 is Verilog-2001-ish but has SystemVerilog `class` usage only in the debug case's JTAG_DRV, not in the core).
3. **R3 — `time` CSR semantics.** rv906's `time` = mtime at clk/100; CoreMark self-measures with it. Fix = one CSR read-arm re-point (rv12 P2), sequenced after M6/M7 land to avoid CSR.v conflicts.
4. **R4 — mcor/cache-CSR pokes in shared crt0.** Donor crt0 pokes `mcor` (0x7C2) which rv906 doesn't decode → illegal-CSR trap on rv906 for EVERY case (all share crt0). The ported crt0 must drop mcor and trim mhint to rv906's implemented bits (`WT/rtl/rvproc_pkg.sv:283` has CSR_MHINT=0x7C5 — check bit compatibility at port time).
5. **R5 — interrupt case needs a real port, not a re-link.** It pokes the C906-internal PLIC's pending register. On rv906 it must instead raise a real PLIC source (e.g. the UART device IRQ, which `WT/dut.cpp:80-85` already routes to the PLIC, or CLINT software interrupt). This rewrites the case's init; the claim/complete handler logic (which is what actually tests rv906's M6 path) survives.
6. **R6 — FP case needs Zfh or surgery.** ISA_FP is ~80% half-precision ops; rv906 is F/D only. Options: drop `.h` mnemonics from the source (keep the F/D subset) or skip ISA_FP entirely (rv64uf/ud already passed in M5).
7. **R7 — Exit-protocol port must be exact.** Magic-reg → tohost port in crt0 must also handle `vector_table`-based traps (the exception case patches vector_table at runtime, `crt0.s:225-230`); keeping crt0's trap scaffold intact while changing only `__exit`/`__fail` is the safe edit.
8. **R8 — MMU case's THEADFLAG PTE bits.** `MMU_PTW_1G ... 0xf` sets PTE[62:59]=0xF (T-Head extended attrs) and enables `mxstatus.maee`. rv906's MMU ignores MAEE (`WT/rtl/MMU.v:679`); whether rv906 faults on reserved PTE bits needs a directed check before claiming MMU-case parity. Mitigation: port the case with THEADFLAG=0 (functionally identical mapping on rv906) and note the deviation.

### Proposed M8 task table

| # | Task | Gate |
|---|---|---|
| M8-T0 | **Design doc + this exploration note** (`docs/superpowers/specs/2026-09-XX-m8-crosscheck-design.md`); re-scope the case set per R1/R6: core = {csr, exception, MMU(≠), interrupt(≠), coremark} + filtered {ISA_INT, ISA_LS, ISA_FP} as stretch; record all deviations D-M8-n | doc review |
| M8-T1 | **Toolchain + donor-flow dry run**: export TOOL_EXTENSION=/opt/riscv/bin (or xpack), CODE_BASE_PATH; patch march override; `make runcase CASE=csr SIM=iverilog` end-to-end on the donor RTL. Proves R2's feasibility and produces the first donor-side `run_case.report`. | donor `csr` reports TEST PASS under iverilog |
| M8-T2 | **rv906 `time` CSR re-point** (rtl/CSR.v: `CSR_TIME`/`CSR_TIMEH` arms → mcycle source), sequenced after M7 lands; directed rdtime test | `csrr time` deltas == cycle deltas; M2/M4/M5/M6 suites stay green |
| M8-T3 | **test/m8/ scaffolding** mirroring test/m6/ shapes: `Makefile` (xpack gcc, `-march=rv64imafdc_zicsr`, link at 0x80000000, `.tohost` at 0x7FFFF000), ported `crt0.s` (drop mcor poke, keep mhcr/mhint subset, `__exit`→tohost=1 / `__fail`→tohost=fail-code), ported clib `fputc` → 16550 THR at 0x10000000, `run_all.sh` → `bin/verisim/testbench --print-result` + grep PASS | `csr` case PASSes on rv906 |
| M8-T4 | **Core case set on rv906**: csr, exception (minus `dcache.ciall`), MMU (THEADFLAG=0), coremark (ITERATIONS pinned, e.g. 100 to keep sim time sane, recorded) | all PASS on rv906; coremark prints VCUNT_SIM line on rv906 UART |
| M8-T5 | **interrupt case port**: rewrite init to use rv906 PLIC at 0x0C000000 + a real source (CLINT msip or UART IRQ); keep claim/complete/wfi structure | interrupt case PASSes on rv906 |
| M8-T6 | **Filtered XThead-free ISA smokes** (stretch): strip indexed-store lines from ISA_INT, `dcache.*` from ISA_LS, `.h` ops from ISA_FP — build the standard subsets | filtered cases PASS on rv906 |
| M8-T7 | **Donor-side runs** (iverilog): same case set (with donor's own crt0), capture TEST PASS + `cycle_count` via a 2-line tb.v `$display` patch applied as a tracked `.diff` (rv12 P4 pattern); cycle counts recorded | donor reports match rv906 verdicts |
| M8-T8 | **compare + close-out**: `test/m8/compare.py` (JSON records: case/verdict/cycles both sides); cycle-count table noted-not-gated; full M2-M6 suite re-run as the floor | per-case functional parity table green; docs updated |

Sequencing note: T2 touches CSR.v which M7 is actively editing — land T2 after M7's go-live commit, exactly the rv12 P2 merge-avoidance pattern. T3-T6 are independent of T7/T8 (rv906-side parity is meaningful before the donor side runs). T1 is cheap and de-risks R2 early; if T1 fails (iverilog chokes on gen_rtl), fall back to re-scoped M8 = "donor sources (built with standard march) run on rv906; expected results = the cases' own self-checks" and drop T7/T8 to a stretch Verilator-on-gen_rtl investigation.
