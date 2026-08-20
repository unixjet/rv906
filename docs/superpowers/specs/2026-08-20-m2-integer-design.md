# M2: Integer Machine End-to-End — Design

Date: 2026-08-20
Status: Draft for review
Parent: `2026-08-20-rv906-c906-clone-design.md` (§7.4 row M2)
Extraction notes (normative source of C906 facts, keep open while implementing):
`notes/2026-08-20-c906-idu-extraction.md` (IDU),
`notes/2026-08-20-c906-iu-extraction.md` (IU),
`notes/2026-08-20-c906-rtu-extraction.md` (RTU),
`notes/2026-08-20-c906-lsu-base-cp0-extraction.md` (LSU base path + DCache + CP0/CSR).
Style/structure precedent only, not content (C910 is out-of-order with a real ROB — its
control flow does not transfer): `rv12/docs/superpowers/specs/2026-08-19-m2-integer-machine-design.md`.
Same-repo precedent: `2026-08-20-m1-ifu-design.md` (M1, complete, merged, 110/110 regression).

## 1. Goal

Bring C906's integer machine end to end: single-instruction decode+dispatch → single execute
pipe (ALU/BJU/MULT/DIV) → a blocking DCache load/store base path → in-order, un-buffered
retire → a minimal M-mode-only CSR file. This is the parent design's M2 row (§7.4): **IDU + IU
+ RTU + LSU base path + DCache + minimal CSR**, pass bar **rv64ui/um riscv-tests green**.
`FetchSink.v` (M1's fake-BJU/fake-RTU/CP0 stand-in) is deleted; the real units take over the
single-instruction interface M1 froze, unchanged.

Unlike C910, there is **no** rename, no physical register file, no reorder buffer, no
out-of-order issue anywhere in this milestone — every one of IDU/IU/RTU/LSU is, per the
extraction notes, materially simpler than its C910 counterpart (IDU note §0: "no rename and no
physical register file"; IU note §0: "genuinely single execute pipe"; RTU note §0: "not a
reorder buffer... a thin, un-buffered continuation of the single EX stage"; RTU note §9:
"4,010 lines total vs. C910 RTU's 35,643 — an ~9x reduction"). M2's job is to clone that
simplicity faithfully, not to import OoO machinery RV906 doesn't need.

## 2. Scope

### 2.1 In scope (geometries/latencies cited from the extraction notes)

- **IDU** — one combinational ID/DIS stage (decode + WBT/GPR read + RAW/WAW hazard check +
  RTU-sourced forward mux + EU one-hot select, IDU note §2) feeding a single EX1 pipeline
  register (311-bit integer payload + 10-bit one-hot `EU_WIDTH` select, IDU note §2/§7,
  `aq_idu_id_ctrl.v:598-616`, `aq_idu_id_dp.v:934-1064`). Hazard tracking is a **32-entry
  busy-bit scoreboard** (`aq_idu_id_wbt.v`: 31 flop entries + hardwired-always-ready x0),
  each entry carrying a not-busy flag, a 2-bit outstanding-producer count, and a 3-bit
  producer-type tag (`WB_INT_TYPE`: OTHER/ALU/BJU/MULT/LSU — no DIV tag, IDU note §5.1, an
  open item carried to §8 below). RAW/WAW stalls have producer-type-aware "except" clauses
  (IDU note §5.2) so single-cycle ALU/BJU producers never force a scoreboard stall. Forwarding
  into IDU's operand read is sourced **exclusively from RTU** (`rtu_idu_fwd0/1/2` — 3 generic
  broadcast slots, `rtu_idu_wb0/1` — 2 commit slots; IDU note §6) — there is no direct
  IU→IDU or LSU→IDU bypass wire. EX1 issue is gated a second time in place (commit +
  target-EU-full check, IDU note §7) — an instruction can sit valid-but-not-issuing in EX1.
  GPR is a plain 31-entry gated-register file (+ hardwired x0) with 3 read ports and 2 write
  ports (`rtu_idu_wb0/1`), architecturally identical in structure to the WBT (IDU note §5.4).
  CSR ops (CSRRW/S/C + `*I` forms) decode into a dedicated `EU_CP0` dispatch target exactly
  like ALU/LSU targets — CP0 is a peer EU for dispatch purposes, not something IDU owns
  (IDU note §8).
- **IU** — one shared ALU datapath (a single 65-bit adder covers ADD/SUB/ADDW/SUBW/SLT/SLTU
  via an operand-prepare one-hot mux, not a separate 32-bit path for `*w` ops; one 128-bit
  barrel shifter covers SLL/SRL/SRA/`*W`; AND/OR/XOR; XThead REV/TST/FF0/FF1/MVEQZ/MVNEZ — no
  MAX/MIN/ADDSL, dead in this C906 build, IU note §2/§11.3), fully combinational, EX1-only.
  BJU: its own private comparator (not the ALU's adder), mispredict/RAS/BHT-feedback signals
  to IFU (already consumed on the IFU side per M1), and the **only** IU-internal
  operand-forward/stall structure — a 1-entry buffer holding a conditional branch whose
  operand depends on an outstanding load, released by an early forward bus or LSU's EX2
  completion (IU note §4.4). A separate shared 64-bit adder (`aq_iu_addr_gen`-equivalent)
  computes branch/JAL/JALR targets and AUIPC's `pc+imm` (IU note §3). MULT: one 33×33
  Booth-radix-4 multiplier array reused iteratively; **3 cycles** for operands that fit in 33
  bits (the common case, including all `*W` multiplies), up to **~6 cycles** for a full 64×64
  multiply via up to 4 iteration passes through the same array (IU note §5) — a genuinely
  variable-latency multiplier, not fixed-cycle. DIV: a radix-4, 2-bits/cycle non-restoring
  divider with a leading-1 alignment early-out and a 1-entry memo/hit buffer; **~4 to ~36
  cycles**, data-dependent (IU note §6). Both MULT and DIV expose real point-to-point
  stall/full signals to IDU (`iu_idu_mult_issue_stall`/`_mult_full`, `iu_idu_div_full`) and
  consume single-bit writeback grants from RTU (`rtu_iu_mul_wb_grant`, `rtu_iu_div_wb_grant`)
  — no IU-internal arbiter exists; RTU is where completion-vs-writeback races are resolved
  (IU note §0/§5/§6). IU exposes **four separate writeback buses to RTU** (ALU/BJU/MULT/DIV),
  none carrying exception flags (IU note §10) — see §6 below for the design decision on this.
- **RTU** — a single un-buffered EX1→EX2 retire register fed by a 7-source one-hot completion
  OR (`{alu,mul,bju,div,lsu,cp0,vec}_cmplt_dp`, RTU note §2), retiring **at most 1
  instruction/cycle, 0/cycle on any stall** (no queue, no "skip" — a stalled EX1 simply holds
  the retire register at its previous value). Writeback arbitration (`rbus`) prioritizes the
  EX1 group (ALU/BJU/CP0/LSU-ex1, mutually exclusive by the single-issue one-hot guarantee)
  over DIV over MUL-at-EX3 (RTU note §3); 2 architectural GPR write ports to IDU
  (`rtu_idu_wb0/1`) plus 3 forward ports (`rtu_idu_fwd0`=EX1 group, `fwd1`=MUL-EX3,
  `fwd2`=LSU-EX2). Exception/interrupt priority is a fixed chain: pending-breakpoint >
  interrupt > LSU async bus error > ebreak/debug-trigger breakpoint > the synchronous EX1
  exception CP0/LSU already decided (RTU note §4). A 5-state flush FSM
  (`IDLE→FE→[WAIT]→BE→IDLE`, with an unconditional `IDLE→FE_BE→IDLE` shortcut on an async
  debug-halt request — not needed for M2's scope but the FSM shape is cloned as-is) sequences
  every redirect: trap taken, CSR-serializing/fence/xret flush, or a BJU
  mispredict/RAS-fail/dependent-LSU-changeflow forcing the front-end flush directly (RTU note
  §6). CSR read-modify-write and trap-entry CSR capture both happen at CP0's own EX1 with
  **no RTU-side buffering or commit gate** (RTU note §7) — C906's single-issue in-order
  property makes "precise" free here.
- **LSU base path + DCache** — address-gen `AG` (one 64-bit adder, combinational, issues the
  MMU-stub request and the DCache tag/data SRAM read the same cycle, LSU note A2) → `DC` (a
  4-state FSM, `IDLE→DCS→{FRZ|REPLY}`; tag compare resolves hit/miss in the `DCS` cycle; `FRZ`
  is a pure write-port-arbitration stall, not part of nominal hit latency) → `DA` (byte
  rotate + sign/zero-extend, asserts the writeback). **Net: 3 cycles AG→DC→DA for a cache
  hit**, +1 (`FRZ`) only on a writeback-port collision (LSU note A2). DCache geometry: **32KB,
  4-way, 64B line, 128 sets** — confirmed materially different from the ICache's 2-way (LSU
  note A3) — PIPT tag (`PA[39:12]`, 28 bits), VIPT-with-alias-detection index (`{VA[12],
  PA[11:6]}`). Store buffer: **4-entry** (`aq_lsu_stb.v`, `DEPTH=4`), byte-granular
  store-to-load forwarding (LSU note A4). **No RTU-commit gate exists anywhere in the scalar
  LSU** — stores drain unconditionally once created (LSU note A4, cross-cutting #1) — see §6
  for why M2 clones this as-is. LR/SC (single reservation register, address+size exact match)
  and AMO (a standalone AMO ALU reading the old value like a load and writing the result back
  through the STB like a store) are 100% LSU-resident in C906 (LSU note A6/A8) but are
  **deferred to M3** for M2 — see §2.3.4. Misaligned access is detected combinationally in AG
  (`|addr[...]` vs. size) and either HW-split (2-pass, 8-byte-rounded re-issue, gated by
  `MXSTATUS.mm`) or trapped (LSU note A6) — see §2.3.3 for M2's decision. A real
  request/response protocol to a separate MMU unit is unconditionally exercised by every
  load/store (`lsu_mmu_va[51:0]/_va_vld/_priv_mode/_st_inst` → `mmu_lsu_pa[27:0]/_pa_vld/_ca/
  _so/_buf/_sec/_sh/_page_fault/_access_fault`, LSU note A7) — see §2.3.2 for M2's stub. A PMA
  ("sysmap") table gives cacheability/MMIO attributes purely from the physical address,
  independent of translation (LSU note B3) — see §2.3.5 for rv906's own table. Some
  **single-outstanding-miss stand-in is unavoidable even for M2** (a load miss must eventually
  get its line back) — full non-blocking (8-entry LFB-equivalent, HW prefetch) is M3's job;
  M2's base path may simply stall the whole pipe on a miss while a single AXI refill completes.
- **CP0 minimal (CSR)** — see §2.3.6 for the exact enumerated set.
- **Integration** — `RVProc.v` rewired: `IFU → IDU → IU → LSU → RTU`, plus `CSR.v` and
  `MMU.v` (new; graduates M1's inline ITLB stub, see §4.3). `FetchSink.v` is deleted. The
  D-side AXI master (`axi_d_*`, currently FetchSink's tohost-only write channel) moves into
  LSU's real bus path; `tohost` becomes an ordinary store to a fixed physical address, not a
  special mechanism (§7.2). `mtip`/`msip`/`meip` — already piped into `RVProc.v`'s port list
  in M1 but unconnected — terminate in CSR.v's `mip` wiring.

### 2.2 Out of scope / deferred

| Item | Where it actually lives in M2's cut | Restored |
|---|---|---|
| Non-blocking D$ (8-entry LFB-equivalent), hit-under-miss, HW stream prefetch | LSU base path is blocking; single-outstanding miss stand-in only | M3 |
| Atomics: LR/SC reservation, AMO ALU, the IDU `amo` multi-beat split FSM | decode does not dispatch LR/SC/AMO*; traps as illegal instruction | M3 (per parent milestone table: "M3 ... atomics \| rv64ua pass") |
| VIPT alias-detection (`dc_virt_idx[0]`/synonym check) | omitted; safe under an identity-map MMU stub where VA[12]==PA[12] always | M4 (real Sv39 can produce synonyms) |
| Real Sv39 MMU: uTLB/JTLB/hardware PTW | `MMU.v` is a combinational identity-map + PMA-lookup stub (§2.3.2) | M4 |
| Any other privilege level (S/U-mode), delegation, PMP | M-mode-only throughout; CSRs for other modes don't exist | M4 |
| Scalar FPU (FALU/FMAU/FDSU), `f`/`d` decode, `fcsr` | not decoded; F/D-class instructions trap illegal | M5 |
| Full interrupt delivery/servicing, WFI power semantics, vectored `mtvec` | `mie`/`mip` exist as real wires (cheap, already piped) but no directed interrupt test is part of M2's acceptance; `mtvec` direct mode only | M6 (interrupts), partially M7 (debug-adjacent halt interaction) |
| RISC-V Debug (DTM/DM/SBA, `dtu` triggers) | absent; RTU's flush FSM states/paths that exist only for debug are not exercised | M7 |
| The IDU multi-beat split FSM's other three sub-FSMs: `lsd` (custom T-Head load/store-double), `che` (cache-maintenance custom opcodes), `fnc` (`sfence.vma`) | decode does not dispatch these classes; plain `FENCE`/`FENCE.I` are ordinary single-beat `EU_CP0` ops and stay in scope | never (`lsd`/`che`, C-SKY custom, out of RV906's ISA target) / M4 (`fnc`, needs the real MMU) |
| RVV (vector), XThead MAX/MIN/ADDSL long-ALU ops | dead in the donor RTL itself for this config; never decoded | never |
| `mcycle`/`minstret` backed by a real PMU unit | implemented as two simple local free-running counters directly in CSR.v (nearly free, not gating M2 acceptance) | true PMU semantics, if ever needed |

### 2.3 Documented deviations / key design decisions

These are the six calls the extraction notes explicitly flagged as needing a design decision
rather than being pure RTL fact. Each is decided here, not left open.

#### 2.3.1 Store buffer commit-gating: clone as-is, no RTU-commit gate

**Decision: clone C906 exactly — LSU's STB drains unconditionally, with no RTU-commit-gated
write mechanism.** This is safe for a strictly single-issue in-order machine: by the time a
store reaches AG/DC/STB it is, by construction, the oldest and only in-flight instruction on
that path — there is no younger speculative state for it to be wrong relative to. The one
real precondition (flagged by the LSU note itself, A4) is that a store behind an
already-mispredicted branch must never reach STB-create in the first place. RV906 already
has the mechanism for this: RTU's flush signals fan out globally the same cycle a mispredict
or exception is discovered (RTU note §6, "Broadcast `rtu_yy_xx_flush_fe`/`_flush`... reaches
every unit"), and BJU's own redirect (`iu_ifu_tar_pc_vld`/cancel) is even faster than the
retire-stage flush for a resolved branch. **M2's STB-create logic must be gated by the same
local flush/cancel input every other EX1-resident structure listens to** — this is the
existing §6.2-rule-3 pipeline-register discipline ("if (flush) ... else if (!stall) ..."),
not a new precision mechanism. No RTU-commit gate is invented to replace what C906 doesn't
have. **Carried-forward risk**: this reasoning depends on flush fan-out timing being
respected everywhere a store could be created; M6 (interrupts) and M7 (debug halt) introduce
new, asynchronous sources of "late-arriving cancel" and must re-audit this exact interlock —
recorded in §8.

#### 2.3.2 DTLB/MMU stub for M2: identity map, shared with IFU's existing stub

M1 already built exactly this shape inline for the ITLB (`RVProc.v:181-226`:
`mmu_ifu_pa = ifu_mmu_va[MMU_PA_WIDTH-1:0]`, `MMU_PA_WIDTH = 28` in `rvproc_pkg.sv`). **Decision:
factor this into a real `MMU.v` file (per the parent design's architecture diagram, `MMU.v <->
mmu/`) serving both IFU's ITLB request and LSU's DTLB request, replacing M1's inline stub.**
The module implements, combinationally, in the same cycle a request arrives (matching AG's
stall-logic timing expectation, LSU note A7):

- **Request** (per-port: one for IFU, one for LSU): `{va[51:0], va_vld, priv_mode, st_inst}` —
  same field names/widths as the real `lsu_mmu_va`/etc. ports for donor traceability (project
  convention §6.3: signal glossary maps RV906 names to C906 names).
- **Response**: `{pa[27:0], pa_vld, ca, so, buf, sec, sh, page_fault, access_fault}`.
- **Stub behavior, exactly enumerated**: `pa_vld = 1` always (never a miss, never blocks);
  `pa[27:0] = va[27:0]` (identity map — PA = VA truncated to the 28-bit PPN, i.e. VA==PA
  everywhere in the 40-bit PA/VA space, consistent with M1's existing convention and with
  `RESET_VECTOR = 64'h8000_0000` living unmapped-but-identical in both spaces); `page_fault =
  access_fault = 0` always (M2 is M-mode-only bare-metal — no permission checks of any kind,
  U/S/X/W/R bits are irrelevant); `ca`/`so`/`buf`/`sec`/`sh` come from the PMA/sysmap lookup
  (§2.3.5) keyed purely on the physical (== virtual) address, independent of this stub's
  identity-map logic — matching C906's own structure, where the sysmap fires "whether or not
  a real Sv39 walk happened" (LSU note B3).
- **Why this is the right M2 swap-in target for M4**: the interface is real and unconditionally
  exercised today (both IFU and LSU AG issue it every cycle), so M4's real uTLB/JTLB/PTW only
  ever needs to replace `MMU.v`'s internals — no AG, IFU, or CSR port changes.

#### 2.3.3 Misalignment: trap-only for M2, HW-split deferred

**Decision: M2 implements only the trap path for misaligned load/store.** AG detects
misalignment combinationally exactly as C906 does (`|addr[...]` vs. transfer size); LSU raises
the standard misaligned-address exception (cause 4 load / 6 store) through the same
synchronous-exception port CP0/LSU already need for illegal-instruction/ecall. The 2-pass
HW-split re-issue FSM (`AG`'s `UNALIGN_IDLE`+ states, LSU note A6) is **not** built in M2.

This is a legitimate, cited scope reduction rather than a correctness gap for M2's acceptance
bar specifically: riscv-tests' base-ISA suites (`rv64ui-p-*`, `rv64um-p-*`) exercise aligned
accesses only — misaligned-access behavior is a `rv64mi` (machine-mode/privileged) concern,
outside M2's pass criterion. Real C906 resets `MXSTATUS.mm = 1` (HW-split is the out-of-reset
default, LSU note B2/A6) — a fully faithful clone that silently always-traps would diverge
from real hardware's reset behavior in a way a later milestone could trip over. So: **`mm`
(MXSTATUS bit 15) is implemented as a real, plain R/W flop, resetting to 1**, so CSR
read-after-write software sees correct state — but for M2, **the AG/LSU misalign path ignores
its value and always takes the trap** regardless. The bit exists; the HW-split feature it's
supposed to gate does not yet. This is cheaper and more honest than a fake read-only stub
(which would silently swallow writes) and costs nothing extra. Deferred, timing not yet
pinned — likely paired with whichever milestone first needs a directed misaligned-access test
(most plausibly M4, since `rv64mi` is privilege-complete territory).

#### 2.3.4 Atomics (LR/SC/AMO*): deferred whole to M3

The LSU note's own framing (A1) lists LM (reservation monitor) and the AMO ALU as small enough
to fit a "blocking base path." **Decision: defer them to M3 anyway**, for two reasons that
outrank the individual-module simplicity argument: (1) the parent design's own milestone table
(§7.4) explicitly pairs "atomics" with M3's non-blocking-D$/miss-buffer/prefetch work and gates
M2's acceptance on `rv64ui/um` only, not `rv64ua` — M2 should not silently expand its own pass
bar; (2) atomics are **not** a single-shot LSU transaction from IDU's side — they ride IDU's
genuine multi-beat split FSM (`aq_idu_id_split.v`'s `amo` sub-FSM, states
`AMO_LR/AMO_SC/AMO_AMO/AMO_AQ`, IDU note §5.3), a chunk of IDU control logic M2 does not
otherwise need (plain `FENCE`/`FENCE.I` dispatch as ordinary single-beat `EU_CP0` ops and do
**not** need the split FSM; only `sfence.vma`, which needs the real MMU, does). Building the
`amo` split path for M2 would mean building dispatch-side machinery whose only consumer is a
feature M2 doesn't test. `misa.A` stays 0 until M3.

#### 2.3.5 PMA/uncached address table: rv906's own map, not T-Head's

C906's real sysmap (LSU note B3) hardwires cacheability by physical-address range at
RTL-compile time — a mechanism, not a set of numbers, that rv906 must reproduce against **its
own** SoC memory map, established in M0/M1 and confirmed by reading `rtl/RVProcAXI.v`'s
`AXICrossbar` instantiation directly:

| Region | PA range | Attributes | Source |
|---|---|---|---|
| DRAM (cacheable) | `0x8000_0000`–`0xFFFF_FFFF` (2GB) | cacheable, bufferable | `MEM_BASE=0x8000_0000`/`MEM_MASK=0x8000_0000`, `RVProcAXI.v:145-146`; matches `RESET_VECTOR=0x8000_0000` |
| CLINT | `0x0200_0000`–`0x0200_FFFF` (64KB) | uncached, strongly ordered | `CLINT_BASE`/`CLINT_MASK`, `RVProcAXI.v:147-148` |
| PLIC | `0x0C00_0000`–`0x0CFF_FFFF` (16MB) | uncached, strongly ordered | `PLIC_BASE`/`PLIC_MASK`, `RVProcAXI.v:149-150` |
| UART | `0x1000_0000`–`0x1000_FFFF` (64KB) | uncached, strongly ordered | `UART_BASE`/`UART_MASK`, `RVProcAXI.v:151-152` |
| everything else in `0x0000_0000`–`0x7FFF_FFFF` | not otherwise covered | **uncached / reserved** in the CPU-side PMA table | see note below |

The last row is a deliberate divergence from the bus-decode convenience in
`AXICrossbar`'s `DEFAULT_SLAVE = SI_MEM`: the crossbar will happily route a stray access
below `0x8000_0000` to the memory controller (a testbench simplification), but the CPU's own
architectural PMA view should **not** extend cacheability there — only the documented 2GB
DRAM aperture is cacheable. This avoids masking address-decode bugs where code accidentally
touches low, unmapped addresses and gets a silently "working" cache hit/miss instead of
uncached behavior that would expose the bug.

`rv906`'s PMA table needs only two attribute bits for M2 (`cacheable`, `strongly-ordered`) —
C906's real sysmap flag is 5 bits (`StrongOrder/Cacheable/Bufferable/Shareable/Security`,
LSU note B3); `Shareable` is irrelevant (single-core, forever, per the parent design's
non-goals) and `Security` is PMP/M4 territory. Dropping them for M2 is a legitimate
simplification, not a fidelity gap the way VIPT-alias omission (§2.2) is — those bits govern
features RV906 will never need to model at all.

**Flagged, needs a human decision (carried to §8):** `ADDR_TOHOST = 64'h9000_1000`
(`rvproc_pkg.sv:25`, pinned in M0) falls **inside** the cacheable DRAM window above. With a
real write-back DCache, a `tohost` store that hits (or even one that misses under
write-allocate) may only dirty a cache line without ever reaching the AXI bus — the same
hazard RV12's C910 clone hit and solved by keeping `tohost` in an explicitly uncached
aperture. This is not something M2 can silently paper over by simply not testing the DCache;
see §7.3 for why M2's own verification plan needs the DCache genuinely exercised, which makes
this hazard real rather than moot.

#### 2.3.6 Minimal CSR set for M2

Real hardware in M2, matching exactly what RTU note §8 identifies as load-bearing for
`rv64ui/um`'s own trap-redirect mechanics (riscv-tests' harness sets `mtvec` and expects
`ecall`/illegal-instruction/misaligned traps to land there, per `env/p/riscv_test.h`):

- **`mstatus`** — only `MIE`/`MPIE` are real flops, with the standard trap-entry swap
  (`MPIE⟵MIE; MIE⟵0` on trap, `MIE⟵MPIE; MPIE⟵1` on `mret`). `MPP` is tied to the constant
  `2'b11` (M) — read-only, since M2 implements no other privilege level to return to or from
  (a legal WARL choice for an M-mode-only core). Every other bit (`SXL`/`UXL`/`TVM`/`TW`/
  `TSR`/`MXR`/`SUM`/`MPRV`/`SPP`/`SIE`/`SPIE`/`*BE`/`SD`) is tied to 0/read-only.
- **`mtvec`** — real flop, **direct mode only**; the mode bit is tied to 0 and writes
  attempting vectored mode are accepted-but-ignored on the mode bit. riscv-tests' own trap
  handler never relies on vectored dispatch, so this is a legitimate, cited scope reduction
  vs. real C906 (which supports vectored mode).
- **`mepc`** — real flop; LSB forced to 0 on write (matches C906's own
  `regs_iui_mepc={mepc_reg[38:0],1'b0}`, enforcing 2-byte alignment, consistent with M1's
  confirmed byte-address PC convention).
- **`mcause`** — real flop: interrupt bit + 5-bit cause code, populated exactly per RTU's
  `rtu_yy_xx_expt_int`/`_expt_vec` (RTU note §4/§7).
- **`mscratch`** — real flop, plain R/W (riscv-tests' standard trap-handler macro uses it to
  save/restore a register).
- **`mtval`** — real flop, populated for the vec-allowlist causes `{1,2,4,5,6,7,12,13,15}`
  exactly as RTU note §4 describes; 0 for every other cause.
- **`mie`/`mip`** — real state: `mip`'s three bits are read-only wires sourced from
  `mtip`/`msip`/`meip` (already piped into `RVProc.v`'s port list since M1, currently
  unconnected); `mie` is a real R/W flop. Not required for M2's `rv64ui/um` pass bar (no
  directed interrupt test is part of M2's acceptance — that's M6), but it is nearly free given
  the M1 ports already exist, and it avoids a fake stub that M6 would otherwise have to tear
  out and rebuild.
- **`misa`** — read-only hardwired constant: `MXL=64`, extensions = `I|M|C` for M2 (`A` joins
  at M3, `F|D` at M5, matching the parent milestone table exactly). Writes are ignored (a
  legal implementation choice).
- **`mvendorid`/`marchid`/`mimpid`/`mhartid`** — hardwired constants; exact values are
  cosmetic (not exercised by riscv-tests pass/fail) and don't need to match T-Head's real IDs.
- **`mcycle`/`minstret`** — implemented as two simple local free-running 64-bit counters
  directly in `CSR.v` (increment every cycle / every retired instruction respectively) rather
  than a `pmu`-shaped stub reading garbage. RTU note §8 flags these as "very unlikely to be
  required for `rv64ui/um` pass/fail specifically" — true, but implementing them for real is
  nearly free and removes a footgun for later debugging.
- **Everything else** (all S-mode CSRs, `satp`, PMP CSRs, `fcsr`, vector CSRs, HPM counters
  beyond cycle/instret) — **absent**, not stubbed-and-tied — matching riscv-tests' own
  bare-metal M-mode-only assumption and the parent design's M4 = "privilege complete + MMU"
  boundary.

**Open integration item, needs the same care M1's own risk section called out for its own
chicken bits**: M1's chicken-bit ladder (`--m1-rung`) is currently poked into `FetchSink`'s
harness config bank, not real CSR state — M1's design doc did not carry forward an explicit
"re-home this in M2" obligation the way RV12's M1 doc did. M2's `CSR.v` must implement C906's
real `MHCR` (`ie`/`de`/`wa`/`rse`/`bpe`/`btbe`, **all reset 0** — boot code must explicitly
enable I$/D$, LSU note B2 — plus `wb`/`wbr` hardwired 1) and re-point IFU's `icache_en`
consumer from `FetchSink`'s bank to the real `cp0_ifu_icache_en`. This is real integration
work M1 left implicit; flagged again in §8.

## 3. References

| Material | Role |
|---|---|
| `docs/superpowers/specs/notes/2026-08-20-c906-idu-extraction.md` | decode, WBT scoreboard, forward network, dispatch (normative) |
| `docs/superpowers/specs/notes/2026-08-20-c906-iu-extraction.md` | ALU/BJU/MULT/DIV structure, writeback buses (normative) |
| `docs/superpowers/specs/notes/2026-08-20-c906-rtu-extraction.md` | retire register, rbus/wb arbitration, flush FSM, exception priority (normative) |
| `docs/superpowers/specs/notes/2026-08-20-c906-lsu-base-cp0-extraction.md` | LSU pipeline, DCache geometry, STB, MMU protocol, sysmap, CSR set (normative) |
| `refs/openc906/C906_RTL_FACTORY/gen_rtl/idu/rtl/` | `aq_idu_top.v`, `aq_idu_id_decd.v`, `aq_idu_id_wbt.v`(+`_entry.v`), `aq_idu_id_dp.v`, `aq_idu_id_ctrl.v`, `aq_idu_id_gpr.v`(+`_gated_reg.v`), `aq_idu_id_split.v` |
| `refs/openc906/C906_RTL_FACTORY/gen_rtl/iu/rtl/` | `aq_iu_top.v`, `aq_iu_alu.v`, `aq_iu_bju.v`, `aq_iu_mul.v`, `aq_iu_div.v`(+`_shift2_kernel.v`), `aq_iu_addr_gen.v` |
| `refs/openc906/C906_RTL_FACTORY/gen_rtl/rtu/rtl/` | `aq_rtu_top.v`, `aq_rtu_ctrl.v`, `aq_rtu_dp.v`, `aq_rtu_rbus.v`, `aq_rtu_retire.v`, `aq_rtu_wb.v`, `aq_rtu_int.v` |
| `refs/openc906/C906_RTL_FACTORY/gen_rtl/lsu/rtl/` | `aq_lsu_top.v`, `aq_lsu_ag.v`, `aq_lsu_dc.v`, `aq_lsu_stb.v`(+`_entry.v`), `aq_lsu_lm.v`, `aq_lsu_amo_alu.v`, `aq_lsu_rdl.v`, `aq_dcache_*` arrays |
| `refs/openc906/C906_RTL_FACTORY/gen_rtl/mmu/rtl/` | `aq_mmu_top.v`, `aq_mmu_utlb/jtlb/ptw.v` (M4 targets), `aq_mmu_sysmap*.v`/`sysmap.h` (PMA reference shape) |
| `refs/openc906/C906_RTL_FACTORY/gen_rtl/cp0/rtl/` | `aq_cp0_top.v`, `aq_cp0_regs.v`, `aq_cp0_trap_csr.v`, `aq_cp0_ext_csr.v`, `aq_cp0_info_csr.v`, `aq_cp0_hpcp_csr.v` |
| `rtl/RVProcAXI.v` | rv906's own SoC address map (§2.3.5) — read directly, not assumed |
| `rtl/rvproc_pkg.sv`, `rtl/RVProc.v` | `MMU_PA_WIDTH=28`, `ADDR_TOHOST=0x9000_1000`, M1's inline ITLB stub, `mtip`/`msip`/`meip` ports already present |
| `docs/superpowers/specs/2026-08-20-rv906-c906-clone-design.md` | umbrella conventions: §6 coding rules, §7.4 milestone table |
| `docs/superpowers/specs/2026-08-20-m1-ifu-design.md` | same-repo precedent for structure/tone; M1 interfaces frozen for M2 |

## 4. Architecture

### 4.1 Unit graph

```
 IFU (M1, frozen)
   │ ifu_idu_id_inst[31:0] / _vld         ▲ idu_ifu_id_stall (the ONLY signal this way)
   ▼
 ┌───────────────────────────────────────────────────────────────────────────┐
 │ IDU  ID/DIS (comb: decode, WBT+GPR read, RAW/WAW+except, fwd mux, EU sel) │
 │      ──▶ EX1 latch (311b payload + 10b one-hot EU select)                 │
 │      ──▶ EX1 issue-gate (commit && !target-EU-full)                       │
 └───┬────────────────────┬───────────────────────┬───────────────────────┬──┘
     │ idu_iu_ex1_*        │ idu_lsu_ex1_dp_sel     │ idu_cp0_ex1_*         │ rtu_idu_wb0/1
     ▼                     ▼                        ▼                      │ rtu_idu_fwd0/1/2
 ┌────────────┐      ┌────────────┐            ┌────────────┐              │
 │ IU         │      │ LSU        │            │ CSR (CP0)  │              │
 │ ALU (comb) │      │ AG→DC→DA   │            │ mstatus/   │              │
 │ BJU (+addr │      │ STB(4)     │            │ mie/mip/   │              │
 │  _gen, RAS)│      │ 1-outst.   │◀──lsu_mmu──▶│ mtvec/mepc/│              │
 │ MULT (3-6c)│      │ miss stand-│  MMU.v      │ mcause/    │              │
 │ DIV (~4-36)│      │ in         │  (identity- │ mscratch/  │              │
 └─────┬──────┘      └─────┬──────┘  map+PMA)   │ mtval      │              │
       │ iu_rtu_ex1/ex3_*  │ lsu_rtu_*          └─────┬──────┘              │
       │ (4 buses:         │                          │ cp0_rtu_ex1_*       │
       │ ALU/BJU/MULT/DIV) │                          │ rtu_yy_xx_expt_*    │
       ▼                   ▼                          ▼                     │
 ┌───────────────────────────────────────────────────────────────────────────┤
 │ RTU: one-hot cmplt OR (7 sources) → EX1→EX2 retire latch → exception/int  │
 │ priority chain → 5-state flush FSM → rbus arbiter (EX1-grp>DIV>MUL-EX3)  │
 │ → wb.v (2 GPR ports) + 3 forward ports ──────────────────────────────────┘
 └── rtu_ifu_chgflw_vld/pc, flush_fe/idu_flush_*, rtu_lsu_expt_ack/exit,
     rtu_mmu_expt_vld (plumbing only, MMU stub ignores it in M2)
```

DCache (32KB/4-way/64B/128-set, tag+data+dirty SRAM arrays) sits behind LSU's DC stage; it is
its own file (`DCache.v`) per the project's "a submodule earns its own file" rule (parent
§6.2 rule 7), matching M1's `ICache.v` precedent.

### 4.2 Boundary structs

Per the umbrella's §6.2 rule 2 ("one struct per pipeline boundary, named after the two stages
it joins"):

- `id_ex1_t` — IDU's EX1 payload (func, EU one-hot, src0/1/2 data+ready, dst0/1 reg, imm,
  illegal, PC). This is the single struct IU/LSU/CSR each field-slice their own view out of,
  matching C906's own "one shared EX1 register, sliced per consumer" structure (IDU note §7) —
  **not** four separate structs routed to four separate destination registers.
- `iu_rtu_t` (×4: `alu`, `bju`, `mult`, `div`) — see §6 for why these stay separate rather than
  collapsing to one struct.
- `lsu_rtu_t` — LSU's EX1/DA-stage completion + writeback (data, preg, cmplt, expt_vld/vec/tval,
  async-error flag).
- `cp0_rtu_t` — CP0's EX1 completion (old-CSR-value writeback) + the trap-declaration bus
  (`expt_vld`/`_int`/`_vec`, `chgflw`/`chgflw_pc` for `mret`).
- `rtu_ex2_t` — RTU's own internal EX1→EX2 retire-packet register (pc, next-pc, flags, expt
  vec/tval, one-hot source select) — internal to RTU.v, not crossing a module boundary.
- `mmu_req_t` / `mmu_rsp_t` — the identity-map stub's request/response shape (§2.3.2), shared
  by IFU's ITLB port and LSU's DTLB port.

### 4.3 File organization

```
rtl/IDU.v      ID/DIS decode, WBT scoreboard, GPR, RAW/WAW+except stall,
               RTU-fwd mux, EU one-hot dispatch, EX1 register + issue-gate
rtl/IU.v       ALU, BJU (+addr-gen, RAS/BHT feedback already IFU-consumed),
               MULT, DIV
rtl/LSU.v      AG, DC, DA, STB(4), single-outstanding miss stand-in
rtl/DCache.v   tag/data/dirty SRAM arrays (32KB/4-way/64B/128-set) via SRAM.v
rtl/MMU.v      identity-map + PMA/sysmap stub (§2.3.2/§2.3.5); replaces M1's
               inline ITLB stub in RVProc.v; also serves LSU's DTLB request
rtl/CSR.v      minimal M-mode CP0: mstatus/mie/mip/mtvec/mepc/mcause/
               mscratch/mtval, MHCR, MXSTATUS.mm (stub), misa, local
               mcycle/minstret, trap-entry/mret sequencing, int priority
rtl/RTU.v      one-hot cmplt OR, EX1->EX2 retire latch, exception/int
               priority, 5-state flush FSM, rbus arbiter, wb (2 GPR ports)
rtl/RVProc.v   rewired: IFU -> IDU -> IU -> LSU -> RTU, + CSR.v + MMU.v;
               FetchSink.v DELETED
```

Style per the umbrella's §6: stage-sectioned files, feedback section up top, packed structs
internal-only, flat wire ports, entry arrays (WBT/GPR/STB entries) not per-entry modules.

## 5. Pipeline details per stage

| Stage | Unit | Cycles | What happens |
|---|---|---|---|
| ID/DIS | IDU | 1, combinational | decode (6-way coarse classify then per-class casez tables) + WBT/GPR read (5 address-decoded 32-way muxes) + RAW/WAW hazard check with producer-type excepts + RTU-forward mux (fwd0/1/2, wb0/1) + EU one-hot select |
| EX1 latch | IDU/EU boundary | held ≥1 cycle | the *only* register inside IDU proper; 311b int payload + 10b EU select, field-sliced per consumer |
| EX1 issue-gate | IDU (EX1-resident) | 0+ extra cycles | `!ctrl_ex1_internal_stall && rtu_idu_commit && !<EU>_idu_full` — a valid-but-not-issuing instruction can sit here |
| EX1 | IU: ALU | 1 | one shared 65-bit adder + 128-bit barrel shifter + logic/misc blocks, purely combinational |
| EX1 (or held) | IU: BJU | 1 (+ hold in the 1-entry LSU-dependent buffer) | private comparator, mispredict/RAS resolve, redirect to IFU |
| EX1–EX3 | IU: MULT | 3 (narrow) to ~6 (64×64, iterative) | Booth-radix-4 33×33 array reused up to 4 passes; EX3 = 4:2-compress + writeback, gated on RTU's `mul_wb_grant` |
| EX1..CMPLT/WFWB | IU: DIV | ~4–36, data-dependent | radix-4 2-bit/cycle non-restoring iteration + memo-buffer/abnormal early-out; `WFWB` waits for RTU's `div_wb_grant` |
| AG | LSU | 1 | address-gen adder + misalign check + MMU-stub request + DCache tag/data SRAM read issued, all combinational |
| DC | LSU | 1 (+1 `FRZ` on write-port conflict) | 4-way tag compare against `PA[39:12]`; hit/miss resolves this cycle; STB hit-check happens here too |
| DA | LSU | 1 | byte rotate + sign/zero extend + `lsu_rtu_wb_vld` assert |
| EX1 | CSR (CP0) | 1, combinational | RMW off the CSR-address-decoded bus; old-value writeback rides the same one-hot rbus source as ALU/BJU/LSU |
| EX1→EX2 | RTU | 1 | un-buffered retire-register latch; exception/interrupt priority decode; flush-FSM transition; rbus/wb arbitration |

Two structural points worth restating because they're easy to collapse by accident when
implementing: (1) ID and Dispatch are **one** stage, not two — there is no register between
decode and hazard-check/EU-select (IDU note §2); (2) an instruction that has issued into IU
can still be waiting in EX1 for RTU's commit/EU-full gate — the EX1 register's validity and
its *issue* are two different conditions (IDU note §7's "Critically" paragraph).

## 6. Coding conventions reminder — and the IU→RTU writeback-bus decision

Per the umbrella's §6.2: restricted Verilog-2001 + the small SV whitelist, one struct per
pipeline boundary, one section per stage (combinational first, sequential last), a feedback
section up top listing every backward-flowing signal, entry arrays never per-entry modules,
and — the rule under direct examination here — **rule 5: "each pipeline boundary exposes
exactly one combined stall/ready signal to its immediate upstream neighbor."**

**Decision: IU exposes four separate writeback buses to RTU (ALU/BJU/MULT/DIV), mirroring
C906 exactly. This is not an exception to rule 5 — rule 5 does not govern this bus at all.**

Reasoning:

1. **Rule 5 is scoped to stall/ready (control) signals, not data payloads.** Its stated
   purpose (parent design §5.2/§6.2) is to bound the *fanout of hazard-detection logic* so a
   distant stage never reads a raw bit from a non-adjacent producer. A writeback bus carrying
   `{data, preg, valid}` is not a hazard-detection signal being read by a non-adjacent
   consumer — it's the payload of the *single* IU→RTU boundary, and every bus terminates at
   RTU, the immediately adjacent unit. Nothing skips past RTU to reach IDU directly: IDU only
   ever sees `rtu_idu_fwd0/1/2` and `rtu_idu_wb0/1` (IDU note §6, verified — no `iu_idu_ex1_fwd`
   or `lsu_idu_fwd` port exists anywhere). The fanout-bounding goal the rule exists to serve is
   fully satisfied regardless of how many named buses cross this one boundary.
2. **Merging them would relocate, not eliminate, an arbiter — and relocate it to the wrong
   side.** RTU already needs per-source completion detail to do its own job: the one-hot
   `cmplt` OR (7 sources) that drives the retire register, and the `rbus` priority arbiter
   (`EX1-group > DIV > MUL-EX3`, RTU note §3) that picks the writeback winner. If IU merged its
   four buses into one canonical struct first, IU would need to build an internal arbiter
   duplicating exactly what RTU's `rbus` already does — and would have to pad every source's
   struct to the union of all four shapes (BJU's redirect/PC-bookkeeping fields, MULT/DIV's
   split accept-at-EX1/data-at-variable-EX timing) even though most fields are meaningless for
   most sources (IU note §10). Keeping the merge on RTU's side (in `RTU.v`'s planned `rbus`
   section, §4.3) is where C906 puts it and where the natural consumer of "what completed this
   cycle" already lives.
3. **The per-unit *point-to-point* discipline the rule actually cares about is preserved
   at a finer grain than the bus count.** Each of ALU/BJU/MULT/DIV's completion is its own
   single valid bit, consumed by exactly one arbiter (RTU's rbus) and nothing else; MULT/DIV's
   backpressure to IDU (`iu_idu_mult_full`, `iu_idu_div_full`) and their writeback grants from
   RTU (`rtu_iu_mul_wb_grant`, `rtu_iu_div_wb_grant`) are each already single, named,
   point-to-point signals between exactly two units — precisely the pattern rule 5 wants,
   just instantiated four times because there are genuinely four independent completion
   sources with different latencies, not because anyone skipped folding them.

So: **four buses, no merge inside IU.v; the merge (rbus arbitration) lives in RTU.v, matching
C906's own module boundary exactly.** Exception/fault flags are *not* part of any of the four
buses (IU note §10 confirms — illegal instruction, misaligned, etc. arrive at RTU via
`idu_cp0_ex1_expt_*`/CP0's own path); RTU's retire packet gets exception context from CP0/LSU,
not from IU, regardless of how IU's writeback buses are shaped.

## 7. Verification design

### 7.1 Pass bar

`rv64ui-p-*` and `rv64um-p-*` from riscv-tests, built with the xpack toolchain already
established in M0/M1, run to completion via the existing `tohost` testbench convention.

### 7.2 What actually makes a trap redirect correctly land on `tohost`

RTU note §8 is decisive here and worth restating as the verification design's foundation:
**`tohost` is confirmed, by grep of the entire reference RTL, to be purely an RV906 testbench
convention — real C906 silicon has no tohost CSR or mechanism.** But riscv-tests' own harness
(`env/p/riscv_test.h`) is not CSR-free: it sets `mtvec` at boot, and expects every unexpected
trap (illegal instruction, misaligned access) *or* the deliberate `ecall` that
`RVTEST_PASS`/`RVTEST_FAIL` issue to redirect to that handler, which then performs the actual
`tohost` store. So even though the pass/fail *signal* is a plain memory write (an LSU/DCache
concern), the trap *redirect mechanics* — `mtvec` read, `mepc`/`mcause`/`mtval` write, the
`MIE`/`MPIE` swap — are load-bearing for M2's acceptance criterion. This is exactly why §2.3.6
enumerates those CSRs as real hardware and nothing else.

### 7.3 Oracle: commit-trace diff against an architectural model, `tohost` as the acceptance gate

Unlike M1 (a predictor-agnostic committed-instruction-stream check against a fake BJU/RTU),
M2 has a real, in-order, single-issue pipeline with genuine architectural state — so the
oracle graduates to what the parent design's §7.2 always intended: a **commit-trace diff
against an architectural reference (Spike, or an in-repo ISS of equivalent fidelity)**, with
`tohost` PASS/FAIL as the acceptance gate riscv-tests already provides for free.

- **Per-retire export** (from RTU's EX2 stage, since that's the one real "this instruction is
  now architectural" point in this machine): `{pc, insn, rd, wdata_valid, wdata, is_store,
  store_addr, store_bytes, store_data, trap_taken, cause, epc, tval}`. Because C906 retires
  exactly one instruction per cycle with **no** split-instruction classes in M2's scope (§2.2
  — `lsd`/`amo`/`che` excluded; `fnc`'s only live member, plain `FENCE`/`FENCE.I`, is
  single-beat), there is no multi-slot collapse logic to build here — a real simplification
  vs. what a `jal`-with-link or AMO sequence would require, deferred to whichever milestone
  reintroduces multi-beat dispatch.
- **Register results are joined at the retire export directly**, not at a separate writeback
  event: because RTU's EX2 retire register *is* the point where the GPR write ports
  (`rtu_idu_wb0/1`) are driven (RTU note §3), retire and architectural writeback are the same
  cycle for every source in this design (no DIV-completes-after-retire skew the way C910's
  ROB-decoupled writeback needed — RTU note §2/§9 confirms retire simply doesn't fire until
  the producing unit's own completion pulse is asserted).
- **Stores are compared at STB-drain / DCache-write time** (address, bytes, data) — precise
  and deterministic per §2.3.1's no-commit-gate reasoning, not via `rd`.
- **Nondeterminism policy**: counter CSR reads (`mcycle`/`minstret`) adopt the RTL's value into
  the reference model at the read (matching RV12's M2 precedent) — M2 has no interrupt test in
  its acceptance path, so this policy is simple and uncontested for M2's scope.
- **rd=x0 creates no join** (matches every producer's own `dst0_vld` gating).

**M2's test recipe must include an explicit boot preamble that writes `MHCR` to enable I$ and
D$ before jumping into the riscv-tests entry point** (mirroring RV12's own M2 precedent for
C910: "at reset everything is off, faithfully to the donor"). Without this, `MHCR.de` stays 0
for the entire run (real C906's own out-of-reset default, LSU note B2) and every load/store
takes the always-miss/uncached-passthrough path — meaning `rv64ui/um` could pass while the
DCache's actual hit/tag-compare/STB-forwarding logic is never genuinely exercised. Since
M2's own stated scope is "DCache," the acceptance sweep must run at least once with caches on
to be a meaningful test of what M2 built — this is also exactly what makes the `tohost`
placement hazard flagged in §2.3.5 real rather than moot, and is why it's carried to §8 as a
decision needing sign-off before M2's plan locks in a boot-preamble recipe.

### 7.4 Bring-up ladder (directed, before the riscv-tests sweep)

1. Single-instruction streams (no memory): decode→dispatch→ALU/BJU→retire, diffed against the
   reference model on hand-written ELFs, including an RVC-mix stream (RVC decode is free —
   M1's IFU/decode path already handles it, and IDU decodes RVC natively per its own note §3.5).
2. Branches + BJU resolve + the mispredict flush path (BJU vs. the M1 front end — this is where
   M1's already-verified `iu_ifu_tar_pc_vld`/BHT-feedback consumer side gets exercised by a
   real producer for the first time).
3. MULT/DIV variable-latency issue/stall/writeback-grant protocol (narrow and wide multiply
   paths; abnormal-result and memo-buffer divide paths).
4. Loads/stores with caches OFF (uncached/always-miss path) — the simplest correct base case.
5. Loads/stores with caches ON — DCache hit/miss, STB byte-granular forwarding, the
   single-outstanding-miss stand-in's refill path, and the `tohost`-visibility question from
   §2.3.5 resolved concretely here, before the full sweep.
6. CSR ops + traps (`ecall`/`ebreak`/illegal-instruction) + `mret`, directed.
7. `rv64ui` one test at a time, then `rv64um`, then the full sweep with caches on.

M2 is done when: `rv64ui`+`rv64um` full suites pass with `MHCR` enabled per §7.3, `docs/03-idu.md`
+ `04-iu-rtu.md` + `05-lsu.md` chapter drafts exist per the parent's documentation plan (§8),
and the integration items in §2.3.6's "open integration item" and §8 are either resolved or
explicitly deferred with a recorded reason.

## 8. Risks / Open Points

Carried forward from the extraction notes, plus what surfaced while writing this doc:

- **`fwd0`/`fwd1`/`fwd2` mutual-exclusivity is asserted-by-construction, not proven.** IDU's
  forward mux has no priority encoding for a simultaneous multi-hit (falls to `default:
  {64{1'bx}}`, IDU note §6); the RTL's own comment on the completion OR
  (`aq_rtu_dp.v:326: // TODO add assertion here: cmplt_dp is onehot`) shows C906 itself never
  formally checked this either. **RV906 should add a real simulation assertion** on the
  one-hot completion bus and on `fwd0/1/2` non-collision when IDU/RTU are implemented — cheap
  insurance the donor RTL itself lacks.
- **Exact `idu_iu_ex1_func` opcode-to-func-bit table.** Confirmed *mechanism* (one-hot op-group
  select per unit) but not the full bit-pattern-per-opcode mapping for ALU/BJU/MULT/DIV — must
  be derived from `aq_idu_id_decd.v`'s casez tables during implementation, cross-checked
  against IU's consumer-side op-group tests (IU note §12/§13).
- **`WB_INT_TYPE` has no DIV tag** (only OTHER/ALU/BJU/MULT/LSU, IDU note §5.1/§10) — unresolved
  whether DIV reuses MULT's producer-type tag in the scoreboard's outstanding-count scheme.
  Must be confirmed from RTL directly (plausible: DIV shares MULT's tag since both are
  variable-latency units tracked the same way) before the WBT is implemented.
- **`idu_iu_ex1_src0/1_ready` semantics** — inferred as "producer is an outstanding load" from
  BJU's own usage (IU note §13), not confirmed as the *only* reason a source can be not-ready
  (could a still-in-flight MULT/DIV result also clear this bit?). Needs confirmation before
  IDU's scoreboard-to-IU `_ready` wiring is finalized.
- **`lsu_rtu_ex1_cmplt` vs. cache-miss-refill timing** — the LSU note's own AG→DC→DA staging
  (§5) provisionally resolves this (a miss simply doesn't reach `DCS`'s resolved-hit state
  until the single-outstanding refill completes, so `cmplt` just doesn't fire until then), but
  RTU note §2 flags this as not independently confirmed from RTU-side RTL alone — cross-check
  both notes' claims against each other during implementation, not just trust one.
- **`retire_mmu_trap` checks vec `{1,13,15}`, not the standard `{12,13,15}`** (RTU note §4) —
  an M4 problem (M2's MMU stub never raises a page fault), but M2's CP0/RTU trap-vector
  plumbing is exactly what M4 will build on, so get the vec-allowlist wiring right now even
  though M2 never exercises the MMU-trap branch of it.
- **`mstatus` `MIE`/`MPIE` swap and `mcause_local_en` exact gating** — glimpsed in the CP0 note
  (B1) but not traced symbol-by-symbol; read `aq_cp0_regs.v`/`aq_cp0_trap_csr.v` directly
  before implementing, not from this doc's summary.
- **VIPT-alias omission (§2.2) is safe only as long as the MMU stub stays identity-mapped.**
  If any M2-era test or later debugging trick ever creates two different VAs for one PA before
  M4's real MMU lands, the omitted alias-detection logic becomes a real correctness gap, not a
  deferred feature. Budget the DCache tag-array bit layout so re-adding it at M4 doesn't
  require re-deriving the SRAM organization (LSU note cross-cutting #3's own recommendation).
- **`tohost` placement vs. the write-back DCache — needs a human decision, not an M2-internal
  one.** `ADDR_TOHOST = 0x9000_1000` (pinned in `rvproc_pkg.sv` during M0) sits inside the
  cacheable DRAM window (§2.3.5). Once M2's boot preamble turns caches on (§7.3, required for
  the DCache to be genuinely tested), a `tohost` store could dirty a line without ever
  reaching the AXI bus the testbench watches. Options, none exercised yet: (a) carve a small
  uncached PMA window around `ADDR_TOHOST` in the M2 sysmap table; (b) move `ADDR_TOHOST` to
  an uncached region (touches M0-pinned test infrastructure, out of this milestone's own
  authority to change unilaterally); (c) accept write-through-only DCache behavior for M2
  (simpler DCache, less faithful to C906's real write-back policy) so every store — cached or
  not — always reaches the bus. This needs sign-off before the M2 implementation plan commits
  to a specific DCache write policy and boot-preamble recipe.
- **M1's chicken-bit re-home was never written down as an explicit M1→M2 handoff obligation**
  (unlike RV12's M1 doc, which had a numbered §10.7.1/§10.7.2 obligations list) — §2.3.6 states
  what M2 must do (build real `MHCR`, re-point IFU's `icache_en` consumer), but whether this
  should also be retroactively noted in the M1 doc is a process question for the controlling
  session, not something this doc can resolve unilaterally.
- **Squash-vs-STB-create interlock timing (§2.3.1)** is reasoned here from RTU's documented
  flush fan-out, not verified cycle-by-cycle against LSU's actual STB-create trigger condition
  — a first implementation task, not a settled fact.
