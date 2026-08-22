# rv906 — Handoff (updated 2026-08-23, after Task 6 completion)

## Standing goal (verbatim, do not paraphrase)

完整、可读的 T-Head 玄铁 C906 手写 Verilog 克隆(RV64IMAFDC, 5级单发顺序流水线)——在仿真里对 riscv-tests / Spike / 原厂 openc906 RTL 三方验证通过,最终跑通单核 Linux 并上真实 FPGA 板测试(M9)。

Long-running autonomous build. Per-milestone process: design doc → implementation plan → subagent-driven-development, task by task, with **independent verification of every subagent's work** (never trust a self-report — rebuild, relint, rerun regressions, spot-check diffs yourself). M9 (physical FPGA) needs the user's hardware; drive M0–M8 autonomously otherwise.

Repo root: `/home/vlsilab/zhouz/workspace/C2RTL/rvproc/RVProc6/vla/riscv/rv906`
Sibling reference (C910 clone, same process): `../rv12`. C906 reference RTL (gitignored, always re-derive facts from here with file:line citations, never assume C910 numbers transfer): `refs/openc906/C906_RTL_FACTORY/gen_rtl/`.

## Where things stand

- **M0 (scaffold)**, **M1 (front end: IFU+ICache+BPU)**: DONE, verified, merged to `master` (M1 merge `bb6f04d`). M1 full-matrix regression: 110/110 (re-verified again today from this worktree).
- **M2 (integer pipeline)**: in worktree **`m2-integer`** (branch `worktree-m2-integer`). Design: `docs/superpowers/specs/2026-08-20-m2-integer-design.md`. Plan: `docs/superpowers/plans/2026-08-20-m2-integer-machine.md` (11 tasks; Global Contracts at lines 84–283).
- **Tasks 1–6 DONE and committed** on this branch:
  - `fa74d0b` Task 1 fixup (missing `idu_lsu_ex1_sel` port)
  - `4880f3a` Task 2: CSR.v real body + unit bench
  - `359e009` Task 3: IU.v real body (ALU+BJU+MULT+DIV) + unit bench
  - `06164bd` Task 4: RTU.v real body + unit bench
  - `564b694` Task 5: IDU.v real body (decode+WBT+GPR+dispatch) + unit bench
  - `1acdf58` **Task 6: DCache.v + MMU.v + LSU.v real bodies + 3 unit benches** (THIS SESSION — see "Task 6 debug findings" below)

## Task 6 debug findings (this session — read before touching MMU/LSU/lsu_tb again)

The crashed-agent Task 6 attempt had left real, mostly-correct work on disk (uncommitted). Two `lsu_tb` failures (T4 STB-create-vs-flush, T5 victim-writeback ordering) were investigated and turned out to be THREE distinct bugs, all fixed and committed in `1acdf58`:

1. **Testbench ordering bug** (`lsu_tb.cpp` `tick()`): `drive_mmu()` ran BEFORE the first `dut->eval()`, so the MMU stub answered the PREVIOUS cycle's `lsu_mmu_va` (usually 0). Every op's AG latched `ag_pa = {0,12'b0} = 0` and `mmu_lsu_ca = 0` — all addresses silently squashed to 0, every "cacheable" access degraded to an uncached direct AXI hit on PA 0. T1–T3 passed by coincidence (the squashed sequence was self-consistent); T4/T5's cross-address checks exposed it. Fix: eval first, then `drive_mmu()`. **General lesson: any bench stub that responds from the DUT's combinational outputs must eval before it reads them.**
2. **Real RTL bug** (`MMU.v` D-side): the DTLB identity map emitted `lsu_mmu_va[27:0]` (low 28 bits of the byte VA) instead of the **page number** `va[39:12]`. `ag_pa = {mmu_lsu_pa, ag_addr[11:0]}` needs the page number; with `va[27:0]`, every address ≥ 0x1000_0000 (i.e. all of DRAM at 0x8000_0000) reconstructed to the wrong PA. The I-side is correct as-is because `ifu_mmu_va` is the VPN (ICache drives `icache_rd_addr[63:12]`) — the two ports' input conventions genuinely differ, now documented in `MMU.v` (header + D-side comment) and `mmu_tb.cpp`. (Also caught an off-by-one in my own fix: `[11 +: 28]` is va[38:11], not va[39:12]; correct is `[12 +: MMU_PA_WIDTH]`.)
3. **Bench convention bug** (`mmu_tb.cpp`): it fed the VPN into the DTLB port while the sole real client (LSU.v) feeds the byte VA. Aligned to the byte-VA convention per contract 2.

After the fixes: all 7 unit benches green (`csr`/`iu`/`rtu`/`idu`/`dcache`/`mmu`/`lsu`), `make verisim` clean rebuild OK, `verilator --lint-only -Wno-fatal --top-module RVProcAXI rtl/rvproc_pkg.sv rtl/*.v` clean, `bash test/m1/run_all.sh --full-matrix` 110/110.

**Known latent M1/M4 note (NOT a Task 6 defect, do not "fix" now)**: `RVProc.v`'s M1 inline I-side stub decides cacheability from `ifu_mmu_va[19]` (VA bit 31), while `MMU.v`'s I-side uses the PMA range check `{vpn,12'b0} ∈ [0x8000_0000, 0xFFFF_FFFF]`. They agree below 2GB (all of M1's test space); they diverge above it. When Task 7 swaps MMU.v in, the PMA version takes over — the PMA range check is the more correct one per contract 5. No action needed until Task 7.

## Next steps (in order)

1. **Task 7** (plan lines 664–700): retire FetchSink.v's tohost-only write FSM, rewire `RVProc.v` to the real core (IFU→IDU→IU→LSU→RTU + CSR.v + MMU.v, DCache stays inside LSU), SoC integration. This is M2's "core swap," analogous to M1's Task 4/7 pattern. Note MMU.v's I-side must now feed ICache's MMU ports (replacing RVProc.v's inline stub at ~lines 214–229).
2. **Task 8** (701–780): verification infra — from-scratch `m2_iss.h` reference model (**no Spike** — none exists in this environment; rv12's own M2 built the ISS from scratch, same precedent) + per-retire trace diff (contract 13 tuple `{pc, insn, rd, wdata_valid, wdata, is_store, store_addr, store_bytes, store_data, trap_taken, cause, epc, tval}` exported from RTU's EX2) + riscv-tests build env with the uncached-tohost linker override.
3. **Task 9** (780–829): bring-up ladder / directed tests (ALU/BJU/MULT-DIV/loads-stores cached+uncached/CSR-traps) — M2's "hard integration gate."
4. **Task 10** (829–end): full rv64ui/rv64um sweep + regression + docs `04-idu.md`/`05-iu.md`/`06-rtu-csr.md`/`07-lsu.md` + plan bookkeeping + clean rebuild + commit. Do NOT merge to master from inside the worktree — merge from the main checkout only.
5. Subagent dispatches: keep the **single-writer constraint** (top-level agent alone writes files; spawned research sub-agents are READ-ONLY) — established after a real concurrent-agent file-thrashing incident in Task 4; held clean since.
6. After M2 done+verified: merge to `master` from the main checkout, then M3 (full LSU: non-blocking D$, miss buffer, HW prefetch, atomics) onward through M9, each milestone = own design doc + plan + subagent-driven-development.

## Load-bearing decisions already made (do not re-litigate without reason)

- **tohost relocated to `0x7FFF_F000`** (from `0x9000_1000`) to stay inside the `<0x8000_0000` uncached aperture — a dirty write-back D$ line could otherwise swallow the tohost store and hang the harness (`testbench/TestBench.cpp`'s `read_mem`/`write_mem` peek the backing store directly). `MHCR.wa` also defaults 0 as defense in depth. `test/m1/common.ld` edited to match (committed as `d815112`, M2 Task 1).
- **No Spike.** M2's oracle is a from-scratch `m2_iss.h` (rv12 precedent; no spike in this environment).
- **DCache**: 32KB 4-way, 64B line, 128 sets; tag=PA[39:13] (27b), index=PA[12:6] (7b) — the 28b-tag Task-1 values were incoherent (28+7+6=41 ≠ 40-bit PA) and fixed in Task 6. Minimal single-line victim-writeback IN scope; full `aq_lsu_vb.v` generality not.
- **Misalignment = trap-only for M2** (`MXSTATUS.mm` flop real, HW-split load deferred). **Atomics 100% out of scope** (`misa.A`=0, M3).
- **RTU's `retire_mmu_trap` mtval allowlist is `{1,2,4,5,6,7,12,13,15}`** — the donor's actual set, NOT "fixed" to the naive `{12,13,15}`; carried forward as-is (design doc §8, an M4 question).
- **DIV/narrow-MUL `cmplt` is a LEVEL signal** held high across the whole busy span (`aq_iu_div.v:755`) — any minstret/oracle-trace logic must edge-qualify or gate on wb0/wb1, never raw `retire_vld`.
- **IU→RTU has FOUR separate writeback buses** (ALU/BJU/MULT/DIV) — deliberate documented exception to the "one signal per boundary" rule (that rule governs stall/ready control signals, not data payloads terminating at one neighbor).
- Extraction notes (file:line-cited into the donor): `docs/superpowers/specs/notes/2026-08-20-c906-{idu,iu,rtu,lsu-base-cp0}-extraction.md`. Read the relevant one before touching its module.

## Verification battery (run after EVERY task, all must pass)

```
make -C test/m2/unit clean && make -C test/m2/unit run        # all benches UNIT-PASS
rm -rf obj_dir bin/verisim obj/verisim && make verisim        # full sim clean rebuild
verilator --lint-only -Wno-fatal --top-module RVProcAXI rtl/rvproc_pkg.sv rtl/*.v
bash test/m1/run_all.sh --full-matrix                          # must stay "ALL PASS (110 runs)"
```
Plus: `git log`/`git status` sanity, and spot-check any subagent's specific claims against the real files / donor RTL yourself. This discipline has caught 8 real bugs across M2 (missing port, CSRRS/CSRRC x0-skip, DIV memo self-reference, CP0 completion too narrow, missing CSR decode arm, TB MMU-stale-VA, MMU D-side page-number, TB DTLB-VPN convention).
