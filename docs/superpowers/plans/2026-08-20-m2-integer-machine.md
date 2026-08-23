# M2: Integer Machine End-to-End Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Bring C906's integer machine end to end — IDU decode+dispatch, a
single IU execute pipe (ALU/BJU/MULT/DIV), a minimal M-mode CSR file, a
blocking DCache load/store base path in LSU, and in-order un-buffered retire
in RTU — replacing `FetchSink.v` with the real units on the exact
single-instruction interface M1 froze. Pass bar (parent design §7.4 row M2):
**`rv64ui`/`rv64um` riscv-tests green.**

**Architecture:** C906's integer machine is *materially simpler* than C910's
(no rename, no PRF, no ROB — extraction notes' own §0 sections say this
explicitly for every unit), so this plan does **not** mirror rv12's C910 M2
plan's task shape (that plan is read only for tone/granularity precedent,
per the assignment — its ISU/IRF/PST/ROB tasks have no C906 analog). Instead
it follows M1's own shape in this repo: Task 1 freezes every new module's
port list and the package constants those ports depend on; **RVProc.v and
`FetchSink.v` are deliberately left untouched through Tasks 2-6** — the M1
front end keeps running exactly as merged, gated by
`test/m1/run_all.sh --full-matrix`, while each new unit (CSR, IU, RTU, IDU,
LSU+DCache+MMU) is built and unit-bench-verified standalone against its
frozen ports, mirroring how M1 built ICache (Task 2) and IFU predictor-less
(Task 3) before FetchSink+the SoC swap (Task 4). Task 7 is M2's own "FetchSink
+ core shell + SoC swap": `FetchSink.v` is deleted, `RVProc.v` is rewired to
`IFU -> IDU -> IU -> LSU -> RTU` plus `CSR.v`/`MMU.v`, and only then does the
M1 regression stop being the relevant gate. Tasks 8-10 mirror M1's
ISS-and-harness / hard-integration-gate / final-regression-and-docs shape,
adapted to M2's own oracle (design doc §7.3: a commit-trace diff against an
architectural reference, not a predictor-agnostic fetch-stream check).

**Unit build order (Tasks 2-6), and why:** CSR before IU before RTU before
IDU before LSU+DCache+MMU. CSR is smallest/most self-contained (single-cycle
combinational RMW, no multi-cycle protocol, no dependency on any other new
M2 unit's real body) and goes first for the same reason ICache went first in
M1. IU goes next because RTU's `rbus` writeback arbiter and one-hot
completion OR need IU's *real, bench-confirmed* completion-latency classes
(ALU/BJU fixed-EX1 vs. MULT's EX1-EX3 vs. DIV's data-dependent EX-stage)
settled before RTU's priority chain (`EX1-group > DIV > MUL-EX3`, RTU note
§3) and writeback-grant protocol (`rtu_iu_mul_wb_grant`/`_div_wb_grant`) can
be written with confidence rather than from the extraction note's prose
alone. RTU goes next because IDU's WBT except-clauses and forward mux need
RTU's *real* `fwd0/1/2` mutual-exclusivity guarantee and `wb0/1` semantics
confirmed — this is exactly where design doc §8's flagged risk ("`fwd0/1/2`
mutual-exclusivity is asserted-by-construction, not proven") gets resolved,
before IDU is written assuming it. IDU goes last of the four "core" units
because it is the one node in the dependency graph that needs ALL of
CSR/IU/RTU's real completion-timing facts (IDU note §5.2's "except" clauses
are keyed on each producer's completion-latency class). LSU+DCache+MMU is
last overall because it depends on IDU's real EX1 dispatch-bus shape
(`idu_lsu_ex1_dp_sel`) and RTU's real `rtu_lsu_expt_ack`/`_expt_exit` timing,
and is independently the most complex remaining piece (a real DCache with
tag/data/dirty SRAM arrays, a store buffer, and a single-outstanding-miss
refill FSM).

**Tech stack:** M1 scaffold (Verilator flow, `make verisim`, tohost
protocol), xpack GCC toolchain (`/opt/xpack-riscv-none-elf-gcc-15.2.0-1`),
restricted-SV recipe (umbrella spec §6). New for M2: an architectural
reference model for the commit-trace diff (design doc §7.3) and vendored
upstream `riscv-tests` sources for the acceptance sweep.

**Normative documents (implementers MUST read the cited sections before
coding):**
- Spec: `docs/superpowers/specs/2026-08-20-m2-integer-design.md` (THE
  contract — §2.3 for every resolved design decision, §4 for the unit graph
  and boundary structs, §5 for the pipeline-stage table, §6 for the
  writeback-bus decision, §7 for the verification design, §8 for open risks).
- Notes: `notes/2026-08-20-c906-idu-extraction.md`, `-iu-`, `-rtu-`,
  `-lsu-base-cp0-extraction.md`. COPY behavior from the file:line spans
  cited there, not from memory. If a task needs a detail neither the design
  doc nor a note resolved, read the cited `refs/openc906/...` RTL directly.
- Parent: `docs/superpowers/specs/2026-08-20-rv906-c906-clone-design.md` §6
  (coding conventions — the struct/section/stall-signal rules every task
  must follow) and §7.4 (M2's row in the milestone table).
- M1 precedent: `docs/superpowers/plans/2026-08-20-m1-ifu.md` (task
  granularity, "Global contracts" header pattern, unit-bench pattern,
  closing-convention phrasing — all reused here) and
  `docs/08-verification.md` §8.7 ("M2 restore checklist", written by M0/M1
  and addressed to whoever starts M2 — items 1/4/5/7 are this plan's
  responsibility, see the Global Contracts section).

---

## Global contracts fixed by this plan (every task agrees on these)

**The eight items the design doc itself resolved, restated here so no task
re-derives or contradicts them:**

1. **IU -> RTU: four separate writeback buses (ALU/BJU/MULT/DIV), no merge
   inside `IU.v`.** RTU's own one-hot completion OR + `rbus` arbiter is where
   the merge happens (design doc §6). This is **not** an exception to the
   umbrella's §6.2 rule 5 ("one combined stall/ready signal per boundary") —
   rule 5 governs stall/ready *control* signals read by a non-adjacent
   stage; these four buses are *data payloads* that all terminate at the one
   immediately-adjacent unit (RTU), and IDU never sees any of them directly
   (only `rtu_idu_fwd0/1/2`/`wb0/1`, sourced from RTU). See item 8 below for
   where rule 5 *does* apply on this same boundary.
2. **MMU/DTLB stub interface (design doc §2.3.2), shared by IFU's ITLB port
   and LSU's DTLB port:** request `{va[51:0], va_vld, priv_mode, st_inst}`
   per port; response `{pa[27:0], pa_vld, ca, so, buf, sec, sh, page_fault,
   access_fault}`. **`va` is the PAGE NUMBER (VPN), not the byte VA, on
   BOTH ports** — exactly the donor's own split: the I-side's `ifu_mmu_va`
   is `icache_rd_addr[63:12]` and the D-side's `lsu_mmu_va` is
   `ag_pipe_addr[63:12]` (donor `aq_lsu_ag.v:1566`; `lsu_mmu_va` is a
   52-bit *output* of the LSU, `aq_lsu_ag.v:271`). Likewise `pa[27:0]` is
   the PHYSICAL PAGE NUMBER — the donor's `mmu_lsu_pa` is `input [27:0]`
   (`aq_lsu_ag.v:201`) and the requester reassembles the full PA itself as
   `{mmu_pa, addr[11:0]}` (`aq_lsu_ag.v:1446`). Stub behavior: `pa_vld=1`
   always; `pa[27:0]=va[27:0]` (identity map on the page number);
   `page_fault=access_fault=0` always; `ca`/`so`/`buf`/`sec`/`sh` come
   from the PMA/sysmap lookup (item 5 below), independent of the
   identity-map logic.
3. **Misalignment: trap-only for M2, HW-split deferred.** AG detects
   misalignment combinationally exactly as C906 does and LSU always raises
   the misaligned-address exception (cause 4 load / 6 store) — the value of
   `MXSTATUS.mm` is **never consulted** by this trap decision. `mm`
   (MXSTATUS bit 15) is nonetheless a real, plain R/W flop, **resetting to
   1** (matching real C906's reset default), so CSR read-after-write
   software sees correct state; only the HW-split *feature* it would gate is
   absent.
4. **Store buffer: clone as-is, no RTU-commit gate.** LSU's 4-entry STB
   drains unconditionally once created; there is no queued/commit-gated
   write mechanism to build. The one precondition (design doc §2.3.1):
   **STB-create must be gated by the same local flush/cancel input every
   other EX1-resident structure listens to** — the ordinary
   `if (flush) ... else if (!stall) ...` pipeline-register discipline
   (umbrella §6.2 rule 3), not a new precision mechanism. Design doc §8's
   own last risk item flags this interlock as "reasoned, not yet verified
   cycle-by-cycle" — Task 6 must verify it directly against RTU's real flush
   fan-out timing (built in Task 4) and LSU's actual STB-create trigger,
   not just assume it.
5. **PMA table (design doc §2.3.5), keyed on the physical/identity-mapped
   address, two attribute bits only (`cacheable`, `strongly-ordered`):**

   | Region | PA range | Attributes |
   |---|---|---|
   | DRAM | `0x8000_0000`-`0xFFFF_FFFF` | cacheable, bufferable |
   | CLINT | `0x0200_0000`-`0x0200_FFFF` | uncached, strongly ordered |
   | PLIC | `0x0C00_0000`-`0x0CFF_FFFF` | uncached, strongly ordered |
   | UART | `0x1000_0000`-`0x1000_FFFF` | uncached, strongly ordered |
   | everything else in `0x0000_0000`-`0x7FFF_FFFF` | uncached / reserved (deliberate divergence from `AXICrossbar`'s `DEFAULT_SLAVE=SI_MEM` convenience) |

   The relocated `tohost`/`fromhost` aperture (item 6 below) sits inside the
   last row.
6. **`tohost` relocation + `MHCR.wa=0`, the resolved write-back-DCache
   hazard fix (design doc §8, mirroring RV12's proven C910 fix):**
   - `ADDR_TOHOST` moves from `0x9000_1000` to **`0x7FFF_F000`** (inside the
     uncached aperture, `< 0x8000_0000`, the same "bit 31 clear = uncached"
     convention `test/m1/uncached.S`/`common.ld` already prove — chosen
     60KB past `M1_UNCACHED_BASE` (`0x7FFF_0000`) to sit clear of
     `test/m1/uncached.S`'s own content, which peaks at offset `0xBE`).
     **This value change, and the matching `test/m1/common.ld` edit, land in
     Task 1 — one task, not left ambiguous.** M2's own test infra
     (`test/m2/common.ld`, Task 8) and the riscv-tests env/linker override
     (also Task 8) use the same relocated value from the start. The fix is
     *proven* (not merely asserted) in Task 9's bring-up step 5 ("loads/
     stores with caches ON"), the first point a real write-back DCache
     exists to make the hazard real rather than moot.
   - `MHCR.wa` (write-allocate, bit 2) defaults to **0** — real C906's own
     reset default — as defense in depth (Task 2, CSR.v).
7. **Minimal CSR set (design doc §2.3.6), exactly:** `mstatus` (only
   `MIE`/`MPIE` real, standard trap-entry swap; `MPP` tied `2'b11`
   read-only; everything else tied 0/RO), `mtvec` (real flop, direct mode
   only — mode bit tied 0), `mepc` (real flop, LSB forced 0 on write),
   `mcause` (real flop: interrupt bit + 5-bit cause), `mscratch` (real flop,
   plain R/W), `mtval` (real flop, populated only for vec allowlist
   `{1,2,4,5,6,7,12,13,15}`, else 0), `mie`/`mip` (`mip` bits are read-only
   wires from `mtip`/`msip`/`meip`; `mie` real R/W flop — not required for
   M2's pass bar, cheap given the M1 ports already exist), `misa`
   (read-only: `MXL=64`, extensions `I|M|C`; writes ignored),
   `mvendorid`/`marchid`/`mimpid`/`mhartid` (hardwired constants, values
   cosmetic), `mcycle`/`minstret` (two local free-running 64-bit counters in
   `CSR.v`, not a PMU stub). Everything else (S-mode CSRs, `satp`, PMP,
   `fcsr`, vector CSRs, HPM beyond cycle/instret) is **absent**, not
   stubbed-and-tied.
8. **Point-to-point stall/ready convention (umbrella §6.2 rule 5): one
   signal per boundary, confined to the two adjacent units.** Applies to
   `idu_ifu_id_stall` (the only IFU<->IDU signal, already frozen since M1),
   `iu_idu_mult_issue_stall`/`_mult_full`/`_div_full` and
   `iu_idu_bju_full`/`_bju_global_full` (each a single named point-to-point
   signal from exactly one IU sub-unit to IDU), and whatever single stall
   signal LSU exposes to IDU for its EX1 issue-gate. **Item 1's four
   writeback buses are the deliberate, justified exception described
   there** — they are data payloads terminating at the one adjacent unit
   (RTU), not hazard-detection bits read by a non-adjacent stage, so rule 5
   does not govern them at all.

**Additional contracts this plan pins (not restated from the design doc,
but needed for every task to agree):**

9. **DCache geometry for M2 omits the VIPT alias-detection second bank
   entirely (not merely disables it).** Real C906 DCache uses two 4-way tag
   banks (bank0=ways0-3, bank1=ways4-7) selected by `VA[12]`, with the
   *other* bank checked as a synonym detector (LSU note A3). Design doc
   §2.2 defers this whole mechanism to M4 as safe under an identity-map MMU
   (`VA[12]==PA[12]` always in M2). **`DCache.v` therefore implements a
   single 128-set x 4-way group**, `PA[39:12]` tag (28 bits), no alias bank
   — but the tag-row bit layout must still budget room for a future
   valid/tag pair per way so M4 can add the second bank without a full
   re-derivation (LSU note cross-cutting #3's own recommendation). Pinned in
   Task 1's package constants, implemented in Task 6.
10. **LR/SC/AMO* are entirely out of `LSU.v`'s scope for M2** (design doc
    §2.3.4) — not stubbed, not decoded. `aq_lsu_lm.v` (reservation monitor)
    and `aq_lsu_amo_alu.v` have no M2 counterpart; IDU decodes LR/SC/AMO*
    opcodes as illegal instructions (Task 5). `misa.A` stays 0 until M3.
11. **A dirty-line victim writeback is in scope for M2**, despite not being
    named as its own task in the design doc. `MHCR.wa=0` (item 6) only
    suppresses *allocation on a store miss*; it says nothing about ordinary
    dirty-line eviction caused by a *load* miss needing to refill a way that
    already holds modified data. Without some writeback-on-evict mechanism,
    M2's DCache would not really be the write-back cache the design doc's
    own tohost-hazard reasoning (§8) depends on being genuinely exercised
    (§7.3's explicit "the acceptance sweep must run at least once with
    caches on to be a meaningful test of what M2 built"). Task 6 builds a
    minimal single-line victim-writeback path (not the full `aq_lsu_vb.v`
    module's generality) as part of the DCache/LSU base path, not as a
    deferred item.
12. **Oracle: resolved to a from-scratch in-repo ISS, not Spike** (umbrella
    §7.2's "Spike diff" plan was the aspiration; design doc §7.3 explicitly
    permits "an in-repo ISS of equivalent fidelity" as a fallback, and the
    controlling session has now exercised that fallback rather than leaving
    it open — see task 8.1). No `spike` binary and no `riscv-isa-sim`
    source exist anywhere in this environment; internet access is
    available but building Spike from source is a real side-quest (its own
    toolchain/dtc/boost dependencies) for a tool that isn't load-bearing —
    rv906's own oracle-pair architecture already proved itself through all
    of M1 without Spike, and **RV12's own M2 never used Spike either**
    (zero references in its plan; it built `m2_iss.h` from scratch, the
    exact precedent this task follows). M2 does **not** grow M1's
    `m1_iss.h` (a ~200-line, deliberately fetch-stream-only oracle) into
    this full architectural simulator — task 8.1 explains why a full
    RV64IMC semantics+trap+memory model deserves its own independent
    implementation rather than an ad-hoc extension of a file scoped to a
    narrower problem.
13. **Per-retire trace tuple (design doc §7.3), the exact fields every
    oracle-diff task must agree on:** `{pc, insn, rd, wdata_valid, wdata,
    is_store, store_addr, store_bytes, store_data, trap_taken, cause, epc,
    tval}`, exported once per retiring instruction from RTU's EX2 stage.
    Register results join at this same retire event (RTU's EX2 register
    *is* where `rtu_idu_wb0/1` fire — no separate writeback-event join is
    needed, design doc §7.3). Stores are compared at STB-drain/DCache-write
    time (address, bytes, data), not via `rd`. `rd=x0` creates no join.
    `mcycle`/`minstret` reads adopt the RTL's value into the reference model
    at the read (a non-determinism exception, not a general policy).
14. **RVC decode resolution (`docs/08-verification.md` §8.7 item 7,
    written by M0/M1 for whoever starts M2):** choice (a) — M2's IDU decodes
    RVC natively as part of the core itself (IDU note §3.5: RVC is a peer
    decode path into the same `EU`/`FUNC`/`*_vld` fields, not a separate
    expand-then-decode pass), consistent with `misa` advertising `C`. No
    no-C fallback build target is needed for the firmware kit.
15. **Test infrastructure location: `test/m2/`, parallel to `test/m1/`, not
    inside it.** `test/m2/unit/Makefile` follows `test/m1/unit/Makefile`'s
    exact pattern (one `--top-module` per bench, `obj/unit/<name>`,
    `bin/unit/<name>_tb`, `UNIT-SUITE-PASS`/`FAIL`). `test/m2/common.ld` and
    `test/m2/macros.h` are **new files**, not shared with `test/m1/`'s: M1's
    macros encode a *synthetic* fetch-only oracle's rules (`JR_TARGET`,
    the 16-entry shadow stack, `taken=^pc[7:4]`) that have no meaning once a
    real BJU computes real branch/jump targets from real operands. The one
    file M1 owns that M2 *does* edit is `test/m1/common.ld` (item 6's
    `ADDR_TOHOST` value, Task 1 only) — M1's own directed tests must keep
    passing after that edit, confirmed by `run_all.sh --full-matrix`.
16. **Documentation chapter numbering.** `docs/02-ifu.md` and
    `docs/03-bpu.md` already exist (M1) and `docs/08-verification.md`/
    `docs/09-fpga.md` are reserved by the parent doc's plan (§8), leaving
    exactly four free slots (04-07) for M2's four new units — one fewer
    slot than the five new files (IDU/IU/CSR/RTU/LSU+DCache+MMU). **This
    plan folds CP0/CSR into the RTU chapter** (RTU and CP0 are the two units
    most tightly bound by the trap-entry/exception-priority contract, RTU
    note §7) rather than into IU's, and lands: `docs/04-idu.md`,
    `docs/05-iu.md`, `docs/06-rtu-csr.md`, `docs/07-lsu.md`. This is a
    recorded, reasoned deviation from the design doc's own suggested
    filenames (`03-idu.md`/`04-iu-rtu.md`/`05-lsu.md`), which were written
    before `03-bpu.md` existed as a same-numbered M1 chapter.
17. **M0's restore checklist (`docs/08-verification.md` §8.7), disposed of
    explicitly, not silently:** items 1 (`CPU_PC`/`CPU_GPR`, drop
    `VERISIM_NO_CPU_STATE`) and 4 (prefer `RESET_VECTOR` wiring over a
    post-reset poke) are Task 8/Task 3 work respectively; item 5 (MEMCTL's
    16-beat AXI burst cap) constrains Task 6's DCache refill FSM (a
    single 512-bit-beat refill per 64B line trivially satisfies it, same as
    ICache's own deviation 1); item 7 is resolved by contract 14 above;
    items 2/3 (a D-cache host-side read-mirror for `TB::read_mem()`) are
    **explicitly deferred, not built** — `read_mem` keeps reading `ExtMem`
    directly, which is not merely acceptable but is exactly the behavior
    contract 6's tohost relocation is designed around; item 6 (UART
    C++ path) and item 8 (PLIC `int_src`) stay out of scope (no riscv-tests
    in M2's pass bar touches either).
18. **Every task ends with:** full-stack lint clean (`verilator --lint-only
    -Wno-fatal --top-module RVProcAXI rtl/rvproc_pkg.sv rtl/*.v`), build
    clean, commit with the Claude trailer. English only.

---

## Task 0: Branch

- [x] Already done: this plan is executed inside the `worktree-m2-integer`
  git worktree (branch `worktree-m2-integer`), started from the M1 merge.
  This plan starts execution from Task 1. No separate branch-creation step
  needed.

## Task 1: Frozen module skeletons + package constants + `tohost` relocation

**Files:** Create `rtl/CSR.v`, `rtl/IU.v`, `rtl/RTU.v`, `rtl/IDU.v`,
`rtl/LSU.v`, `rtl/DCache.v`, `rtl/MMU.v` (skeletons, empty/tied bodies);
Modify `rtl/rvproc_pkg.sv`, `test/m1/common.ld`. **`rtl/RVProc.v` and
`rtl/FetchSink.v` are NOT touched in this task** — unlike M1's Task 1 (which
created `RVProc.v` fresh), M2's `RVProc.v` already exists and stays wired
exactly as M1 shipped it until Task 7's swap; freezing the seven new
modules' port lists does not require instantiating them anywhere yet.

- [ ] **1.1** `rvproc_pkg.sv`: add M2 constants, all cited with a donor
  source per the umbrella's traceability rule.
  - DCache geometry per contract 9: `DCACHE_SIZE=32768`, `DCACHE_WAYS=4`,
    `DCACHE_LINE_BYTES=64`, `DCACHE_SETS=128`, `DCACHE_TAG_WIDTH=28`
    (`PA[39:12]`), `DCACHE_INDEX_W=7`. No alias-bank constants.
  - Standard RISC-V CSR addresses (not donor-specific, safe to pin from the
    ISA spec directly): `CSR_MSTATUS=12'h300`, `CSR_MISA=12'h301`,
    `CSR_MIE=12'h304`, `CSR_MTVEC=12'h305`, `CSR_MSCRATCH=12'h340`,
    `CSR_MEPC=12'h341`, `CSR_MCAUSE=12'h342`, `CSR_MTVAL=12'h343`,
    `CSR_MIP=12'h344`, `CSR_MCYCLE=12'hB00`, `CSR_MINSTRET=12'hB02`,
    `CSR_MVENDORID=12'hF11`, `CSR_MARCHID=12'hF12`, `CSR_MIMPID=12'hF13`,
    `CSR_MHARTID=12'hF14`.
  - Custom T-Head CSR addresses (`MHCR`, `MXSTATUS`): **do not assume
    rv12's C910 numbers (e.g. `0x7C1`) transfer** — read
    `refs/openc906/.../cp0/rtl/aq_cp0_ext_csr.v` directly to confirm C906's
    real addresses before pinning them, per the project's "never quoted
    from memory" rule.
  - MHCR bit positions (confirmed, LSU/CP0 note B2): `ie`=bit0 (icache-en),
    `de`=bit1 (dcache-en), `wa`=bit2 (write-allocate), `rse`=bit4, `bpe`=
    bit5, `btbe`=bit6, all reset 0; `wb`/`wbr` hardwired 1.
  - `MXSTATUS_MM = 15` (bit position, confirmed design doc §2.3.3).
  - `ADDR_TOHOST` **relocated from `64'h9000_1000` to `64'h7FFF_F000`**
    (contract 6) — edit the existing parameter in place, do not add a
    second constant.
  - EU one-hot bit assignments (`EU_WIDTH=10`, IDU note §7/cfig.h:100):
    pin `EU_ALU`/`EU_BJU`/`EU_MULT`/`EU_DIV`/`EU_CP0`/`EU_LSU` bit
    positions from `aq_idu_cfig.h:105-153` directly; the remaining 4 bits
    (`EU_FP`/`EU_VEC`/etc.) are declared but permanently unreachable in
    M2's decode (contract 10, and FP/VEC are out of scope per the parent
    design's M5/never rows) — document that at the declaration site, don't
    just omit them.
  - `WB_INT_TYPE` producer-tag encoding (IDU note §5.1): `OTHER=0`,
    `ALU=1`, `BJU=2`, `MULT=3`, `LSU=4`. **Whether DIV reuses `MULT`'s tag
    is an open item (design doc §8) — Task 3 (IU) resolves it by reading
    the RTL directly; pin the 4 confirmed values now and leave a `//
    TODO(Task 3)` marker rather than guessing a 5th value.**
  - `id_ex1_t` payload field set (design doc §4.2): pin the FIELDS the
    design doc names (func, EU one-hot, src0/1/2 data+ready, dst0/1 reg,
    imm, illegal, PC) as flat-vector bit-range `` `define ``s per umbrella
    §6.2 rule 8 (no packed struct on a port). The exact 311-bit layout and
    offsets are confirmed against `aq_idu_id_dp.v:934-1064` and
    `aq_idu_id_ctrl.v:598-616` in this task — read the RTL now, don't defer
    the widths to Task 5.
- [ ] **1.2** Skeleton port lists, from the design doc's §4.1 unit graph +
  §4.2 boundary structs + each extraction note's interface-describing
  sections, donor signal names verbatim (project traceability convention):
  - `rtl/CSR.v`: `idu_cp0_ex1_*` (func/opcode/src0/src1/dst0_reg) in;
    `cp0_rtu_ex1_*` (wb_data/preg/vld) + `cp0_rtu_ex1_expt_*`/`_chgflw*`
    out; `rtu_yy_xx_expt_vld/_int/_vec`, `rtu_yy_xx_flush_fe/_flush`,
    `rtu_cp0_epc/_tval` in (trap-entry capture, RTU note §7);
    `cp0_ifu_icache_en/_iwpe/_icache_pref_en/_bht_en/_btb_en/_ras_en` +
    inv/clr request-done pairs out (MHCR fan-out, replacing FetchSink's
    config bank per the "open integration item," design doc §2.3.6);
    `cp0_lsu_dcache_en`, `cp0_lsu_mm`, `cp0_lsu_wa` out; `cp0_xx_mrvbr` out
    (already exists as an M1 port on `RVProc.v`, now sourced for real);
    `mtip`/`msip`/`meip` in; `rtu_hpcp_*`-shaped retire-count in for
    `minstret` (or a direct `rtu_idu_wb0/1_vld`-style commit pulse — pick
    whichever the RTU note's retire-vld signal actually is, confirmed in
    Task 4, not guessed here).
  - `rtl/IU.v`: `idu_iu_ex1_*` (func/src0/1/2_data+_ready/dst0_reg) in;
    `iu_idu_mult_issue_stall/_mult_full/_div_full`,
    `iu_idu_bju_full/_bju_global_full` out (contract 8); `iu_rtu_ex1_alu_*`,
    `iu_rtu_ex1_bju_*` (+ `_cur_pc/_next_pc`, `_ras_mispred`,
    `_depd_lsu_chgflow_*`), `iu_rtu_ex1_mul_cmplt(_dp)` +
    `iu_rtu_ex3_mul_*`, `iu_rtu_ex1_div_cmplt(_dp)` + `iu_rtu_div_*` out
    (contract 1's four buses, none carrying exception flags); every
    already-frozen-since-M1 IFU-facing BJU port
    (`iu_ifu_tar_pc_vld/_tar_pc`, `iu_ifu_pc_mispred`, `iu_ifu_bht_*`,
    `iu_ifu_link_vld`, `iu_ifu_ret_vld`, `ifu_iu_chgflw_vld/_pc`) reused
    **unchanged** (re-verify byte-for-byte against `RVProc.v`'s current
    wire declarations, do not rename); `da_xx_fwd_*`/`lsu_iu_ex2_*` in
    (BJU's private LSU-dependent-branch forward, IU note §4.4);
    `rtu_iu_mul_wb_grant`/`_div_wb_grant` in; `iu_cp0_ex1_cur_pc` out.
  - `rtl/RTU.v`: the completion-source inputs from every producer
    (`{alu,bju,mult,div}_cmplt_dp` from IU, `lsu_rtu_ex1_cmplt` family from
    LSU, `cp0_rtu_ex1_*` from CSR); `rtu_idu_fwd0/1/2_*`, `rtu_idu_wb0/1_*`
    out (contract 8's exception-side); `rtu_iu_mul_wb_grant`/
    `_div_wb_grant` out; `rtu_ifu_chgflw_vld/_pc`, `rtu_ifu_flush_fe`,
    `rtu_idu_flush_fe/_flush_stall/_flush_wbt`, `rtu_idu_commit(_for_bju)`,
    `rtu_idu_pipeline_empty`, `rtu_yy_xx_*` broadcast (flush_fe/flush/
    expt_vld/_int/_vec/dbgon), `rtu_lsu_expt_ack/_expt_exit` out (RTU note
    §6, directly relevant to Task 6's LSU); `rtu_cp0_epc/_tval` out.
  - `rtl/IDU.v`: `ifu_idu_id_inst/_inst_vld/_bht_pred` in,
    `idu_ifu_id_stall` out (unchanged since M1); `idu_iu_ex1_*`,
    `idu_lsu_ex1_dp_sel`(+payload), `idu_cp0_ex1_*` out (the shared EX1
    payload, field-sliced per consumer, design doc §4.2); `rtu_idu_fwd0/
    1/2_*`, `rtu_idu_wb0/1_*` in; `iu_idu_mult_issue_stall/_mult_full/
    _div_full`, `iu_idu_bju_full/_global_full` in; whatever single stall
    signal Task 6 will need from LSU (name it now from the LSU note's A2
    section, confirm in Task 6); `rtu_idu_flush_fe/_flush_stall/_flush_wbt`,
    `rtu_idu_commit(_for_bju)` in.
  - `rtl/LSU.v`: `idu_lsu_ex1_dp_sel` family in; `lsu_idu_*` stall out;
    `lsu_rtu_*` (ex1_cmplt, wb_vld/data/preg, expt_vld/vec/tval, async
    error) out; `lsu_mmu_va/_va_vld/_priv_mode/_st_inst` out,
    `mmu_lsu_pa/_pa_vld/_ca/_so/_buf/_sec/_sh/_page_fault/_access_fault`
    in (contract 2); `rtu_lsu_expt_ack/_expt_exit` in; `cp0_lsu_dcache_en/
    _mm/_wa` in; the AXI D-channel port group (moved here from
    `FetchSink.v`'s tohost-only write FSM, design doc §2.1).
  - `rtl/DCache.v`: tag/data/dirty SRAM-array-facing ports (via `SRAM.v`),
    a plain read/write/invalidate/victim-writeback interface to `LSU.v` —
    no direct RTU/IDU/CSR ports (mirrors `ICache.v`'s M1 precedent of being
    LSU-internal, per umbrella §6.2 rule 7).
  - `rtl/MMU.v`: two independent port groups (contract 2), one for IFU's
    ITLB request (replacing `RVProc.v`'s current inline stub) and one for
    LSU's DTLB request; the PMA/sysmap lookup (contract 5) is internal.
- [ ] **1.3** `test/m1/common.ld`: move the `.tohost` output section from
  `0x9000_1000` to `0x7FFF_F000` to match the new `ADDR_TOHOST` (contract
  6); keep the 64-byte pad. No other M1 file changes.
- [ ] **1.4** Lint every new skeleton standalone (each against
  `rvproc_pkg.sv` alone) and the full stack (`verilator --lint-only
  -Wno-fatal --top-module RVProcAXI rtl/rvproc_pkg.sv rtl/*.v`); `make
  verisim` still builds (`RVProc.v`/`FetchSink.v` untouched, so this is
  unaffected by the seven new files); **`test/m1/run_all.sh --full-matrix`
  must still be 110/110** — the only way this task can regress M1 is the
  `ADDR_TOHOST` edit, so this run is the actual gate on that change, not a
  formality. Commit ("M2: frozen skeletons, package constants, `tohost`
  relocation").

## Task 2: `CSR.v` + unit bench

**Files:** `rtl/CSR.v` (real body); Create `test/m2/unit/csr_tb.cpp`;
Modify `test/m2/unit/Makefile` (create it here, per contract 15, following
`test/m1/unit/Makefile`'s exact pattern).

- [ ] **2.1** Implement per design doc §2.3.6 (contract 7) + CP0/LSU note
  §B: the exact CSR set, MHCR (all bits reset 0 except `wb`/`wbr`=1),
  `MXSTATUS.mm` (real flop, reset 1, otherwise unconsumed by CSR.v itself —
  Task 6 owns the trap decision that ignores it), CSR RMW off the
  address-decoded bus (`csrrw`/`csrrs`/`csrrc` + `*I` forms, same
  three-op RMW shape as the donor, CP0 note B1), old-CSR-value writeback
  riding `cp0_rtu_ex1_wb_*` exactly like any other EX1 producer (no FSM —
  CP0 note B1 found none in the donor either), trap-entry capture on
  `rtu_yy_xx_expt_vld` (`mepc`/`mcause`/`mtval`/MIE-MPIE swap), `mret`
  sequencing (`cp0_rtu_ex1_chgflw`/`_chgflw_pc`), `mie`/`mip` wiring from
  `mtip`/`msip`/`meip`, local `mcycle`/`minstret` counters. No privilege
  mode besides M exists — `mstatus.MPP` is tied `2'b11`, there is no `sret`
  arm (IDU never emits `CP0_FUNC_SRET` per Task 5's decode scope).
- [ ] **2.2** Unit bench: scripted `idu_cp0_ex1_*` driver + fake
  `rtu_yy_xx_expt_vld/_int/_vec`/flush pulses. Cover: all 6 CSR RMW forms;
  trap-entry state changes (MIE/MPIE swap, mepc/mcause/mtval capture,
  vec-allowlist vs. non-allowlist `mtval` cases); `mret` pop; `mie`/`mip`
  masking; MHCR fan-out readback; `MXSTATUS.mm` R/W (value stored, never
  consumed here); counter increment (`mcycle` every cycle, `minstret` on
  the retire-commit pulse). Mutation-check at least one bit (e.g. swap the
  MIE/MPIE trap-entry order) and confirm the bench catches it before
  reverting. `UNIT-PASS`.
- [ ] **2.3** Lint, `make verisim` (unaffected — `CSR.v` not yet
  instantiated in `RVProc.v`), commit.

## Task 3: `IU.v` (ALU + BJU + MULT + DIV) + unit bench

**Files:** `rtl/IU.v` (real body); Create `test/m2/unit/iu_tb.cpp`.

- [ ] **3.1** ALU per IU note §2: one shared 65-bit adder covering
  ADD/SUB/ADDW/SUBW/SLT/SLTU via the operand-prepare one-hot mux (not a
  separate 32-bit path); one 128-bit barrel shifter for SLL/SRL/SRA/`*W`;
  AND/OR/XOR; XThead REV/TST/FF0/FF1/MVEQZ/MVNEZ. **No MAX/MIN/ADDSL** —
  confirmed dead in this C906 build (IU note §2/§11.3), do not port them.
  Purely combinational, EX1-only, no pipeline register.
- [ ] **3.2** Address generator (IU note §3): one shared 64-bit adder for
  branch/JAL/JALR target and AUIPC's `pc+imm` — **AUIPC's result is NOT
  computed by the ALU block**, it rides BJU's writeback bus.
- [ ] **3.3** BJU per IU note §4: private comparator (not the ALU's
  adder); mispredict/RAS/BHT-feedback signals to IFU — **re-verify these
  against `RVProc.v`'s current wire declarations byte-for-byte** (IU note
  §4.3 confirms every name already matches M1's frozen placeholder guess,
  so this should be confirmation, not renaming); the 1-entry LSU-dependent
  conditional-branch buffer (`bju_entry_vld`, released by `da_xx_fwd_*` or
  `lsu_iu_ex2_*`, IU note §4.4) with its two-signal backpressure
  (`iu_idu_bju_full`/`_bju_global_full`); BJU's own PC copy
  (`bju_pcgen_pc`), reset from `cp0_xx_mrvbr` (restore-checklist item 4).
- [ ] **3.4** MULT per IU note §5: one 33x33 Booth-radix-4 array reused
  iteratively (3 cycles for operands fitting in 33 bits, up to ~6 cycles
  for a full 64x64 via up to 4 passes); `iu_idu_mult_issue_stall`/
  `_mult_full` to IDU; EX1 early-accept + EX3 data/writeback split, gated
  on `rtu_iu_mul_wb_grant`.
- [ ] **3.5** DIV per IU note §6: radix-4, 2-bits/cycle non-restoring
  divider with leading-1 alignment early-out and a 1-entry memo/hit
  buffer; `~4` to `~36` cycles, data-dependent; `iu_idu_div_full` to IDU;
  DIV/DIVU/REM/REMU share one core (`div_res_sel_quotient` just picks
  which result feeds the bus); gated on `rtu_iu_div_wb_grant`.
- [ ] **3.6** **Resolve the two design-doc §8 open items this unit owns:**
  (a) the exact `idu_iu_ex1_func` bit-per-opcode table for ALU/BJU/MULT/
  DIV — cross-reference `aq_idu_id_decd.v`'s casez tables against IU's own
  consumer-side op-group tests (IU note §12/§13), and record the finalized
  table in `rvproc_pkg.sv` (fills in Task 1's deferred item); (b) whether
  `WB_INT_TYPE` tags DIV results as `MULT`-type or needs its own tag — read
  `aq_idu_id_wbt.v`/`aq_iu_top.v` far enough to settle it, record the
  finding at the `rvproc_pkg.sv` `// TODO(Task 3)` marker Task 1 left.
- [ ] **3.7** Unit bench: per-op ALU vectors (incl. `*W` forms, shifts,
  SLT/SLTU, every XThead op); MULT all 5 RV64M ops incl. narrow/wide
  timing; DIV all 4 ops + divide-by-zero/signed-overflow/memo-hit fast
  paths + handshake timing; BJU branch matrix (taken/not-taken x
  predicted-right/wrong) plus the LSU-dependent-branch buffer's
  release-on-forward and release-on-`lsu_iu_ex2` paths; confirm the
  IFU-facing redirect signal names/timing match M1's frozen consumer
  exactly (a re-verification, not new derivation). Mutation-check at least
  2 (e.g. break the MULT iteration-count early-out, break DIV's memo
  buffer). `UNIT-PASS`.
- [ ] **3.8** Lint, `make verisim` (unaffected), commit.

## Task 4: `RTU.v` + unit bench

**Files:** `rtl/RTU.v` (real body); Create `test/m2/unit/rtu_tb.cpp`.

- [ ] **4.1** Per RTU note: one-hot completion bus (`{alu,mul,bju,div,
  lsu,cp0,vec}_cmplt_dp`, `vec` permanently 0 in M2) OR'd into
  `dp_ex1_cmplt`; the single un-skidded EX1->EX2 retire register (RTU note
  §2 — retiring **at most 1/cycle, 0/cycle on any stall, no queue**); the
  exception/interrupt priority chain exactly as coded (RTU note §4:
  pending-breakpoint > interrupt > LSU async bus error > ebreak/debug
  breakpoint > the synchronous EX1 exception CP0/LSU already decided) —
  M2 has no debug unit and no directed interrupt test, so those two legs
  are wired but structurally never fire; `mtval` populated only for the
  vec allowlist `{1,2,4,5,6,7,12,13,15}` (note the donor's `retire_mmu_trap`
  uses `{1,13,15}` not `{12,13,15}` — design doc §8 flags this as an M4
  question, carry it forward unchanged, do not "fix" it); the 5-state
  flush FSM (`IDLE->FE->[WAIT]->BE->IDLE`, RTU note §6 — the async-debug
  `IDLE->FE_BE->IDLE` shortcut is cloned as dead-but-present per the design
  doc's "clone the FSM shape as-is" note); `rbus` writeback arbitration
  (EX1 group > DIV > MUL-EX3, RTU note §3); 2 architectural GPR write
  ports (`rtu_idu_wb0/1`) + 3 forward ports (`rtu_idu_fwd0`=EX1 group,
  `fwd1`=MUL-EX3, `fwd2`=LSU-EX2); the full RTU-originated signal
  inventory relevant to M2 (RTU note §6): `rtu_ifu_chgflw_vld/_pc`,
  `rtu_ifu_flush_fe`, `rtu_idu_flush_fe/_flush_stall/_flush_wbt`,
  `rtu_idu_commit(_for_bju)`, `rtu_idu_pipeline_empty`, `rtu_yy_xx_*`
  broadcast, `rtu_lsu_expt_ack/_expt_exit` (directly needed by Task 6),
  `rtu_cp0_epc/_tval`.
- [ ] **4.2** **Resolve the design doc §8 risk this unit owns**: add a
  real simulation assertion that the one-hot completion bus
  (`dp_cmplt_source`) is actually one-hot every cycle, and that
  `fwd0`/`fwd1`/`fwd2` never target the same destination register in the
  same cycle — the donor's own RTL never checked either (RTU note §2's
  `// TODO add assertion here` and IDU note §6's flagged reliance on this
  exact invariant). This is the resolution IDU's Task 5 forward mux is
  allowed to assume without re-deriving it.
- [ ] **4.3** Unit bench: scripted create/complete/commit stimuli from
  each of the 5 (ALU/BJU/MULT/DIV/CP0) fake producer ports; retire-blocking
  cases (nothing completes -> EX2 holds); flush paths (BJU mispredict, CSR/
  fence-serializing flush, taken exception) with the redirect-target mux
  checked against each source; exception priority-chain matrix (force two
  candidate causes simultaneously, confirm the fixed order); `rbus`
  arbitration matrix (force EX1-group + DIV + MUL-EX3 to all want the bus
  the same cycle, confirm the priority holds); the one-hot/fwd-collision
  assertion from 4.2 actually fires when deliberately violated (a
  mutation-style check, then revert). `UNIT-PASS`.
- [ ] **4.4** Lint, `make verisim` (unaffected), commit.

## Task 5: `IDU.v` (decode + WBT scoreboard + GPR + dispatch) + unit bench

**Files:** `rtl/IDU.v` (real body); Create `test/m2/unit/idu_tb.cpp`.

- [ ] **5.1** Decode per IDU note §3: the 6-way coarse classifier, then
  per-class parallel casez tables for the 32-bit integer/LSU/BJU/CP0 class
  and the 16-bit RVC class (decoded natively into the same `EU`/`FUNC`/
  `*_vld` fields, contract 14 — not a separate expand-then-decode pass).
  **Every other class decodes to illegal in M2**: FP (`opcode 1010011`/
  FP-load-store) — reachable in the donor but no FALU/FMAU/FDSU exists yet
  (parent design M5); vector — structurally dead in the donor itself
  (decd_sel[5]=1'b0), naturally unreachable; C-SKY custom (`cp0_idu_cskyee`
  tied 0) — naturally unreachable once that tie-off is asserted; AMO/LR/SC
  and the `lsd`/`che`/`fnc`(non-fence) split-instruction classes (contract
  10, design doc §2.3.4) — plain `FENCE`/`FENCE.I` stay in scope as
  ordinary single-beat `EU_CP0` ops, `sfence.vma` does not (needs the real
  MMU, M4). Immediate generation per IDU note §4 (`src1_imm`/`src2_imm`
  one-hot selectors, both 32-bit and RVC variants).
- [ ] **5.2** WBT scoreboard per IDU note §5.1/§5.2: the 32-entry
  busy-bit + producer-type + outstanding-count table (31 flop entries + x0
  hardwired always-ready); RAW/WAW stall computation with the
  producer-type-aware "except" clauses exactly as enumerated (ALU/BJU
  single-cycle producers never stall a consumer; LSU-to-conditional-branch
  allowed through; an RTU forward-bus hit this cycle satisfies readiness;
  same-latency-class WAW producers don't serialize dispatch) — **by this
  task, Task 3's `WB_INT_TYPE`-for-DIV finding and Task 4's fwd0/1/2
  mutual-exclusivity assertion are both already resolved facts, cite them,
  don't re-derive**.
- [ ] **5.3** GPR per IDU note §5.4: 31-entry gated register file (+
  hardwired x0), 3 read ports (src0/1/2), 2 write ports (`rtu_idu_wb0/1`) —
  same structural pattern as the WBT. A same-cycle `wb0`==`wb1` collision
  on one register silently drops the write (matches the donor's
  `gated_reg.v` shape exactly) — verified in the bench to never actually
  occur, given RTU's one-hot completion guarantee (Task 4.2's assertion).
- [ ] **5.4** Forward mux + EU dispatch per IDU note §6/§7: the ID/DIS
  stage forward mux (3-way one-hot compare against `rtu_idu_fwd0/1/2`,
  falls to `default: {64{1'bx}}` on a non-hit exactly like the donor — the
  invariant that prevents a real multi-hit is Task 4.2's assertion, not
  logic living here); the EX1-stage late forward (re-check against
  `wb0`/`wb1` only, for an instruction already parked in EX1); the 10-bit
  one-hot EU select feeding the single EX1 pipeline register (311b
  payload, Task 1's pinned layout); the EX1 issue-gate
  (`!ctrl_ex1_internal_stall && rtu_idu_commit && !<EU>_idu_full` per EU) —
  **an instruction can sit valid-but-not-issuing in EX1**, this is not the
  same condition as the register being valid (IDU note §7's "Critically"
  paragraph); `idu_ifu_id_stall` driven for real (unchanged signal, now a
  real reason instead of FetchSink's fake one).
- [ ] **5.5** Unit bench: decode vectors covering every `rv64imc` class
  plus every illegal case from 5.1's closed list (FP/vector/custom/AMO/
  lsd/che/sfence.vma all correctly trap); RVC pairs decoding to the exact
  same `EU`/`FUNC`/`*_vld` shape as their 32-bit twin where one exists;
  WBT RAW/WAW except-clause matrix (each of the 5 exceptions in §5.2,
  individually and defeated); GPR read/write incl. the x0-hardwire and
  `wb0`==`wb1` collision case; EU one-hot dispatch; EX1 issue-gate
  hold-and-drain (force `rtu_idu_commit`=0 and each `<EU>_idu_full` in
  turn, confirm the instruction sits valid-but-not-issuing and that a
  stuck EX1 backpressures `idu_ifu_id_stall`). Mutation-check at least 2.
  `UNIT-PASS`.
- [ ] **5.6** Lint, `make verisim` (unaffected), commit.

## Task 6: `LSU.v` + `DCache.v` + `MMU.v` + unit benches

**Files:** `rtl/DCache.v`, `rtl/MMU.v`, `rtl/LSU.v` (real bodies); Create
`test/m2/unit/dcache_tb.cpp`, `test/m2/unit/mmu_tb.cpp`,
`test/m2/unit/lsu_tb.cpp`.

- [ ] **6.1** `DCache.v` standalone (mirrors M1 Task 2's ICache-alone
  pattern): tag/data/dirty SRAM arrays via `SRAM.v` (contract 9's single
  128-set x 4-way group, `PA[39:12]` 28-bit tag, no alias bank); the
  4-state DC FSM shape (`IDLE->DCS->{FRZ|REPLY}`, LSU note A2 — `FRZ` is a
  pure write-port-arbitration stall, not nominal hit latency); byte
  rotate + sign/zero-extend for DA (LSU note A5, the inline `case
  ({sign_ext,size})` shape, not a separate helper module); write-back
  policy with a minimal single-line victim-writeback path on eviction
  (contract 11 — the mechanism, not the donor's full `aq_lsu_vb.v`
  generality); a single-outstanding-miss refill FSM issuing one 512-bit
  single-beat AXI read per 64B line (same bus-width pattern as ICache's
  own deviation 1 — satisfies the 16-beat MEMCTL cap trivially, contract
  17). `cp0_lsu_dcache_en` gates hit reporting exactly like `MHCR.de`
  gates `ICache.v`'s (LSU note B2's confirmed "every load/store forced to
  miss until boot code sets `de=1`" fact).
- [ ] **6.2** `MMU.v` standalone (contract 2): the identity-map + PMA/
  sysmap stub, two independent port groups (IFU's ITLB request, LSU's
  DTLB request), combinational, same-cycle response; the PMA table from
  contract 5's exact regions.
- [ ] **6.3** `LSU.v` real body, wired against the now-real `DCache.v`/
  `MMU.v`: `AG` (one 64-bit adder, combinational, misalign detection —
  contract 3, always traps regardless of `MXSTATUS.mm`'s value — plus the
  MMU-stub request and DCache tag/data read issued the same cycle, LSU
  note A2); `DC`/`DA` per 6.1; the 4-entry STB with byte-granular
  store-to-load forwarding (LSU note A4); **the STB-create-vs-flush
  interlock this task must actually verify** (contract 4's design doc §8
  risk item) — confirm against RTU's real flush-fan-out timing (Task 4)
  and LSU's own STB-create trigger, cycle by cycle, not by assumption;
  point-to-point stall to IDU (contract 8); `lsu_rtu_*` writeback/
  completion family; the D-side AXI master moved here from `FetchSink.v`'s
  old tohost-only write channel (design doc §2.1 — a real load-bearing
  bus path now, not a fixed-address write FSM).
- [ ] **6.4** Unit benches: `dcache_tb.cpp` (hit/miss/refill/dirty-evict-
  writeback/invalidate, mirroring `icache_tb.cpp`'s scripted-driver-against-
  a-golden-memory-array pattern); `mmu_tb.cpp` (identity-map correctness +
  every PMA region's `ca`/`so` classification, incl. the relocated `tohost`
  aperture reading uncached); `lsu_tb.cpp` (load/store byte-width matrix
  with sign/zero extension; misaligned access traps unconditionally
  regardless of a scripted `MXSTATUS.mm` value; STB byte-granular forward
  hit/no-hit/partial-overlap; a scripted flush arriving the same cycle as
  an STB-create attempt, confirming the create is suppressed; a
  single-outstanding-miss refill end to end, incl. a dirty victim being
  written back first). Mutation-check at least 2 (e.g. break the
  STB-create/flush interlock, break the victim-writeback ordering).
  `UNIT-PASS`.
- [ ] **6.5** Lint, `make verisim` (unaffected — none of these three files
  are yet instantiated in `RVProc.v`), commit.

## Task 7: FetchSink retirement + `RVProc.v` rewire + SoC integration

**Files:** Delete `rtl/FetchSink.v`; Modify `rtl/RVProc.v` (real rewire);
Modify `rtl/verisim.h`, `rtl/RVProcTest.cpp` (M1-era wire-path updates only
— the fuller verisim.h/harness work is Task 8). `rtl/RVProcAXI.v` is
**unchanged** — the core instance is already `RVProc` since M1's Task 4.2;
M2 only rewrites `RVProc.v`'s internals.

- [ ] **7.1** Delete `rtl/FetchSink.v`. Rewire `RVProc.v`: `IFU -> IDU ->
  IU -> LSU -> RTU`, plus `CSR.v` and `MMU.v` (replacing the current inline
  `assign mmu_ifu_pa = ...` ITLB stub with a real `MMU.v` instance serving
  both IFU's ITLB port and LSU's DTLB port — same interface contract 2
  already pins, so no `ICache.v`/`IFU.v` port changes are needed here).
  Re-point `ICache.v`'s `cp0_ifu_icache_en/_iwpe/_icache_pref_en` and
  `BPU.v`'s `cp0_ifu_bht_en/_btb_en/_ras_en/_bht_inv/_btb_clr` from
  FetchSink's old config bank to `CSR.v`'s real MHCR-derived wires (the
  "open integration item" design doc §2.3.6 and §8 both flag). Move the
  D-side AXI master (`axi_d_*`) from its old FetchSink-only tohost path to
  `LSU.v`'s real bus path. Wire `mtip`/`msip`/`meip` into `CSR.v`'s `mip`.
  Confirm `ADDR_TOHOST`'s Task-1 relocation reaches `LSU.v`'s real store
  path unchanged (a confirmation, not a new decision — contract 6 already
  landed the value in Task 1).
- [ ] **7.2** `rtl/verisim.h`: define `CPU_PC`/`CPU_GPR` Verilator internal
  paths against IDU's real GPR array and RTU's real retire-PC, drop
  `VERISIM_NO_CPU_STATE` (restore-checklist item 1) so `dut.cpp`'s
  `#ifndef VERISIM_NO_CPU_STATE` blocks compile in for real. Remove the
  now-dead `FSINK()`/M1 config-bank macros; the fuller retire-trace export
  (contract 13) is Task 8's addition on top of this.
- [ ] **7.3** Full-stack lint (`--top-module RVProcAXI`), `make verisim`
  green. **`test/m1/run_all.sh --full-matrix` is expected to fail/be
  meaningless from this point on** (FetchSink's fetch-stream oracle has no
  real consumer anymore) — this is the intended, documented end of that
  regression's relevance, not a regression to chase; record this
  explicitly in the commit message rather than leaving it looking like an
  unexplained break. Commit ("M2: FetchSink retired, real integer pipeline
  wired into RVProc.v").

## Task 8: Verification infrastructure — reference model + riscv-tests build

**Files:** Create `test/m2/` (common.ld, macros.h, unit/Makefile already
exist from earlier tasks); Create `m2_iss.h` (the from-scratch
architectural reference model, contract 12 — resolved, not Spike); Modify
`rtl/RVProcTest.cpp` (retire-trace export + online/offline diff driver).

- [ ] **8.1** **Resolved (controlling-session decision, not left for
  execution time): build the from-scratch architectural reference model,
  do not attempt Spike.** No `spike` binary and no `riscv-isa-sim` source
  exist anywhere in this environment (confirmed by search during M2
  planning); internet access IS available, but building Spike from source
  (autoconf, dtc, boost, device-tree/target toolchain dependencies) is a
  real side-quest with its own failure modes, for a tool that is not load
  bearing here — rv906's own oracle-pair verification architecture (two
  independently-derived checkers cross-validated against each other) is
  already proven end-to-end through the whole of M1's predictor ladder
  without Spike, catching every real bug found there. Decisively: **RV12
  itself never used Spike for its own M2 either** — its M2 plan has zero
  Spike references and it built `m2_iss.h` from scratch, the exact
  precedent this task follows. Build a from-scratch architectural
  reference model in C++, sized exactly to contract 13's tuple and RV64IMC
  + the minimal CSR set (contract 7) + the trap-priority/vec-allowlist
  rules (RTU note §4). **Do not attempt this by mechanically growing
  `m1_iss.h`**; that file's entire design point is being a narrow,
  predictor-agnostic fetch-stream checker (M1 plan's own framing), and a
  full architectural model has a different job (real register/memory/CSR
  semantics) that deserves an independent implementation, not an ad-hoc
  extension of a file scoped to a different problem. Record this decision
  (with this reasoning) at the top of `m2_iss.h`.
- [ ] **8.2** RTL-side retire-trace export: add `verisim.h` paths (or a
  `verilator public` export block in `RTU.v` itself, whichever is more
  direct) for contract 13's tuple, sourced from RTU's real EX2 retire
  register (design doc §7.3 — register results join here for free, no
  separate writeback-event join needed; stores are compared at
  STB-drain/DCache-write time instead of via `rd`).
- [ ] **8.3** Harness: extend `RVProcTest.cpp` with an online or offline
  commit-trace diff against 8.1's chosen reference (umbrella §7.2's
  "Spike diff: ... scripts diff the commit sequences offline. First
  divergent instruction -> cycle -> waveform" is the target shape if Spike
  was chosen; an online per-cycle diff is the natural shape if the
  fallback ISS was chosen, mirroring M1's own `M1Checker` pattern but
  against the fuller tuple). Implement the `mcycle`/`minstret`
  nondeterminism policy (contract 13's last sentence). New CLI flags as
  needed (following M1's own precedent of adding flags as the harness
  needs them and recording it) — at minimum something to select which
  ELF(s) to run and whether the reference-model diff is on or off (an
  `--iss-diff`-style default-on flag, for raw-speed regression runs later
  if ever needed).
- [ ] **8.4** riscv-tests build infrastructure: vendor upstream
  `riscv-tests` sources for the `rv64ui-p-*`/`rv64um-p-*` binaries.
  **Do not fetch fresh from the network** — an already-checked-out copy
  exists in this same workspace at
  `/home/vlsilab/zhouz/workspace/C2RTL/rvproc/test/rv-test/riscv-tests`
  (confirmed present during planning; a second candidate exists at
  `/home/vlsilab/zhouz/workspace/C2RTL/vla/kuiper/c2rtl_riscv/riscv-tests`)
  — copy or reference whichever is more current, recording which one at
  the vendoring site. Build the base-ISA test binaries against rv906's own
  toolchain (`/opt/xpack-riscv-none-elf-gcc-15.2.0-1`), with **a new env/
  linker override** (design doc §8, mirroring RV12's own C910 fix) so
  `.tohost`/`.fromhost` land at the relocated `ADDR_TOHOST` (contract 6)
  when building against rv906 specifically — riscv-tests' own upstream
  `env/p/link.ld` does not know about this address and must not be used
  unmodified. Add a boot preamble (design doc §7.3's explicit requirement)
  that writes `MHCR` (`ie`/`de`=1 at minimum) before jumping to each test's
  entry point — without it, `MHCR.de=0` at reset means the DCache is never
  genuinely exercised and the acceptance sweep would not test what M2
  built.
- [ ] **8.5** `test/m2/common.ld`/`test/m2/macros.h` (contract 15): a
  fresh linker script + macro header for M2's own hand-written directed
  ELFs (Task 9), using the same `ADDR_TOHOST`/reset-vector conventions as
  the riscv-tests override in 8.4, but **not** carrying over any of
  `test/m1/macros.h`'s synthetic-oracle macros (`JR_TARGET`, the shadow
  call stack, the `^pc[7:4]` direction rule) — those encoded M1's fake
  BJU's rules, which have no meaning against a real BJU and a real
  reference model.
- [ ] **8.6** Lint, `make verisim`, commit ("M2: reference-model diff
  infrastructure + riscv-tests build environment").

## Task 9: Bring-up ladder (the hard integration gate)

**Files:** Create `test/m2/{alu_seq,bju_seq,muldiv_seq,ld_st_uncached,
ld_st_cached,csr_trap}.S`, `test/m2/run_all.sh`.

Follows the design doc's own §7.4 bring-up ladder exactly, in order.
**Budget the majority of M2's remaining debug time here** — this is where
real pipeline bugs get found via directed tests before the full riscv-tests
suite is attempted, mirroring M1 Task 6's role exactly.

- [ ] **9.1** Single-instruction streams (no memory): decode -> dispatch ->
  ALU/BJU -> retire, diffed against the Task 8 reference model on
  hand-written ELFs, including an RVC-mix stream (contract 14 — RVC decode
  is native, so this should be free if 5.1/5.5 are actually correct).
  `test/m2/alu_seq.S`.
- [ ] **9.2** Branches + BJU resolve + the mispredict flush path — this is
  where M1's already-verified `iu_ifu_tar_pc_vld`/BHT-feedback consumer
  side gets exercised by a real producer for the first time (every prior
  exercise of that consumer side was FetchSink's fake resolve logic).
  `test/m2/bju_seq.S`.
- [ ] **9.3** MULT/DIV variable-latency issue/stall/writeback-grant
  protocol: narrow and wide multiply paths, abnormal-result and
  memo-buffer divide paths, back-to-back dependent MULT/DIV chains
  exercising the WBT's outstanding-count field for real. `test/m2/
  muldiv_seq.S`.
- [ ] **9.4** Loads/stores with caches OFF (`MHCR.de=0`, the reset
  default) — the uncached/always-miss path, the simplest correct base
  case. `test/m2/ld_st_uncached.S`.
- [ ] **9.5** Loads/stores with caches ON — DCache hit/miss, STB
  byte-granular forwarding, the single-outstanding-miss stand-in's refill
  path, dirty-line victim writeback, **and the `tohost`-visibility
  question from contract 6 resolved concretely here**: confirm a `tohost`
  store lands in `ExtMem` promptly enough for the harness's poll loop to
  observe it, with the DCache genuinely caching everything else in the
  test. `test/m2/ld_st_cached.S`.
- [ ] **9.6** CSR ops + traps (`ecall`/`ebreak`/illegal-instruction) +
  `mret`, directed — confirm `mtvec`/`mepc`/`mcause`/`mtval`/MIE-MPIE swap
  all land correctly, since this is exactly the mechanism riscv-tests'
  `RVTEST_PASS`/`RVTEST_FAIL` depend on (design doc §7.2). `test/m2/
  csr_trap.S`.
- [ ] **9.7** `rv64ui` riscv-tests, one test at a time (caches on, per
  9.5's resolved boot preamble), fixing whatever breaks before moving to
  the next; then `rv64um`, same discipline. This is deliberately NOT the
  full clean-room sweep (that's Task 10) — it is the hands-on debugging
  pass where individual-test failures get root-caused one at a time.
  `test/m2/run_all.sh` grows to drive this incrementally.
- [ ] **9.8** Lint, full build, commit ("M2: bring-up ladder green through
  individual rv64ui/um tests").

## Task 10: `rv64ui`/`um` full sweep, regression, docs, close-out

- [ ] **10.1** Full `rv64ui-p-*` + `rv64um-p-*` suite green, `MHCR`
  enabled (caches on) per design doc §7.3's explicit requirement that the
  acceptance sweep genuinely exercise the DCache. Also run the suite once
  with caches off as a sanity cross-check (not the acceptance gate, but a
  useful signal if the two runs disagree). Extend `test/m2/run_all.sh` to
  drive the full matrix and record pass/fail + a cycle-count table, same
  spirit as M1's `run_all.sh --full-matrix` summary.
- [ ] **10.2** Full regression: `test/m2/unit/` `run` target (all of
  csr/iu/rtu/idu/dcache/mmu/lsu unit benches, `UNIT-SUITE-PASS`); the Task
  9 bring-up ladder tests (9.1-9.6) still green; the Task 10.1 riscv-tests
  sweep. `test/m1/run_all.sh` is **not** re-run as a gate (Task 7 already
  recorded why it stopped being meaningful) — note this explicitly in the
  regression summary rather than silently omitting it.
- [ ] **10.3** Write `docs/04-idu.md`, `docs/05-iu.md`,
  `docs/06-rtu-csr.md`, `docs/07-lsu.md` (contract 16's numbering
  resolution): tutorial chapters, principle -> this implementation ->
  C906 file cross-reference table -> design discussion, covering every
  deviation/finding Tasks 1-9 actually made (the DIV/`WB_INT_TYPE` tag
  resolution, the `idu_iu_ex1_func` table, the DCache single-group
  simplification, the victim-writeback addition, the `tohost` relocation
  proof from 9.5, whatever the Spike-vs-fallback-ISS decision from 8.1
  turned out to be). Update `docs/08-verification.md` with M2's harness
  section (mirroring §8.9's structure for M1) and close out §8.7's
  restore-checklist items per contract 17 (mark 1/4/5/7 resolved, note
  2/3/6/8's disposition explicitly). Update `README.md` status to "M2:
  integer machine complete, pending review," with a quick-start pointing
  at `test/m2/run_all.sh` and `test/m2/unit`.
- [ ] **10.4** Plan bookkeeping: tick every checkbox in this plan with a
  completion note describing what was actually found/decided (mirroring
  M1 Task 10.3's "DONE:"-annotation style — this plan's checkboxes start
  as unchecked placeholders, not a record of completed work). Full clean
  rebuild (`make clean && make verisim && make -C test/m2/unit && bash
  test/m2/run_all.sh`) green from scratch; commit. **Do NOT merge** —
  merging into the mainline is the controlling session's job, after a
  final review, same as M1's own Task 10.3 handoff.

---

## Out of scope (fenced)

Non-blocking D$ (8-entry LFB-equivalent), hit-under-miss beyond the
single-outstanding stand-in, HW stream prefetch (M3); atomics — LR/SC
reservation, AMO ALU, the IDU `amo` split FSM (M3, contract 10); VIPT
alias-detection / the DCache's second tag bank (contract 9, M4); a real
Sv39 MMU — uTLB/JTLB/hardware PTW (M4); any privilege level besides M,
delegation, PMP (M4); the scalar FPU and `f`/`d` decode (M5); full
interrupt delivery/servicing, WFI power semantics, vectored `mtvec` (M6);
RISC-V Debug (M7); a D-cache host-side read-mirror for `TB::read_mem()`
(contract 17, explicitly deferred, not merely unscheduled); performance
tuning of any kind (M8's job); configurability across openc906's own
config options (deferred past Phase 1 per the parent design doc).
