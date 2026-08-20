# C906 RTU Extraction Notes (M2 working material)

Source: `refs/openc906/C906_RTL_FACTORY/gen_rtl/rtu/rtl/` (7 files, 4,010 lines total;
file:line refs relative to it unless another dir is named). Also read the ~10 CP0-boundary
lines needed to answer the CSR-writeback question, out of `refs/openc906/C906_RTL_FACTORY/
gen_rtl/cp0/rtl/` (a large, separate unit — `aq_cp0_top.v` alone is 48KB across 15 files —
**not otherwise extracted here**; that is a distinct M2/CSR task). Cross-checked against the
sibling `2026-08-20-c906-idu-extraction.md` and `2026-08-20-c906-iu-extraction.md` notes
already in this directory, which independently confirm several boundary claims below.

For style calibration only: `../rv12/docs/superpowers/specs/notes/2026-08-19-c910-rtu-extraction.md`
(C910's real 64-entry ROB + PST-preg rename recovery — **materially different**, see §9).

## 0. Architecture shape vs. C910 (read this first)

C906's "RTU" is **not a reorder buffer**. There is no entry array, no IID, no age-tournament
comparator, no rename/free-list. The whole unit is a thin, un-buffered continuation of the
single EX stage: a one-hot "something completed" pulse from whichever execution unit finished
this cycle, captured into **one** register (`aq_rtu_dp.v`'s `dp_ex2_*` flops), decoded
combinationally for exceptions/interrupts/flush next cycle (`aq_rtu_retire.v`), plus two small
sibling modules that route the writeback value (`aq_rtu_rbus.v`, `aq_rtu_wb.v`) and prioritize
pending interrupt causes (`aq_rtu_int.v`). 4,010 lines total vs. C910 RTU's 35,643 — an ~9x
reduction, consistent with dropping the ROB/PST-preg machinery entirely rather than shrinking it.

## 1. Module graph (`aq_rtu_top.v`, 1013 lines, instantiates each once)

- `aq_rtu_ctrl` (top.v:676-703, `aq_rtu_ctrl.v` 248 lines) — ORs the 7 EX1-completion sources
  into one `cmplt` pulse, registers it one cycle to make `retire_vld`, and drives the RTU's own
  clock gate.
- `aq_rtu_dp` (top.v:707-780, `aq_rtu_dp.v` 547 lines) — the EX1->EX2 "retire packet" pipe
  register: PC, next-PC, branch/mret/sret/flush/ebreak/dret flags, expt vec/tval, halt-info,
  vstart, fs/vs-dirty, one-hot source select.
- `aq_rtu_rbus.v` (530 lines) — writeback-value arbiter across {EX1 ALU/BJU/CP0/LSU, DIV,
  MUL(ex3)} plus the 3 forward (bypass) ports to IDU.
- `aq_rtu_retire.v` (1296 lines, the actual "retire unit") — exception/interrupt priority,
  debug/breakpoint request timing, the flush FSM, changeflow-PC mux, and every RTU->{IFU, IDU,
  LSU, CP0, DTU, MMU, HPCP} output.
- `aq_rtu_wb.v` (283 lines) — final 2-port GPR/FPR writeback packaging (rbus/VPU winner on
  port0, LSU's own late data on port1) sent to IDU.
- `aq_rtu_int.v` (93 lines) — 15-source interrupt-cause priority encoder.

## 2. Retire structure: un-buffered, exactly the EX/WB pipeline boundary — confirmed NOT a queue

- **One-hot completion bus.** `dp_cmplt_source[6:0] = {alu,mul,bju,div,lsu,cp0,vec}_cmplt_dp`
  (aq_rtu_dp.v:318-325); `dp_ex1_cmplt_dp = |dp_cmplt_source` with an explicit RTL TODO
  admitting the one-hot assumption is unchecked (aq_rtu_dp.v:326: `// TODO add assertion here:
  cmplt_dp is onehot.`). This is the hardware's own documentation that at most one instruction
  can be completing per cycle — a structural guarantee from the single-issue in-order EU, not
  an arbitration RTU performs.
- **The "retire register" is a single un-skidded D-flop set.** `aq_rtu_ctrl.v:174-182`:
  `ctrl_ex2_cmplt <= ctrl_ex1_cmplt` on `cmplt_clk`; `ctrl_ex2_retire_vld = ctrl_ex2_cmplt`
  becomes `retire_ex2_retire_vld` (aq_rtu_retire.v:432). `aq_rtu_dp.v:434-454` latches the full
  payload into `dp_ex2_*` under the identical condition (`dp_ex1_cmplt || ifu_rtu_warm_up`).
  There is no second entry, no valid-count, no "full" signal anywhere in the RTU — confirming
  the task's "whatever completes this cycle commits next cycle, in order, no buffering"
  hypothesis exactly.
- **Retire is therefore ≤1/cycle, and 0/cycle whenever nothing completes.** A multi-cycle
  MUL/DIV or a DCache-miss load simply does not assert its `*_cmplt` this cycle, so
  `ctrl_ex1_cmplt` is 0, `dp_ex1_cmplt` is 0, and the EX2 register **holds its previous value**
  (no clock edge updates it) — i.e. a stall-induced bubble, not a "skip" or "buffer it for
  later." Cross-checked against `2026-08-20-c906-idu-extraction.md` §5.2: IDU's `ctrl_ex1_stall`
  holds a stuck EX1 instruction and backpressures new dispatch (`ctrl_dis_stall`,
  `idu_ifu_id_stall`) — i.e. the classic in-order full-pipeline stall, not an OoO
  completion/buffering scheme. RAW hazards on the result of an in-flight multi-cycle op are
  handled by IDU's 32-entry busy-bit scoreboard (`aq_idu_id_wbt.v`, cleared busy on dispatch,
  cleared to ready on `rtu_idu_wb{0,1}_vld` — ibid. §5.1), which is orthogonal to RTU's
  retire-valid pulse.
- **Two separate EX2 latch domains for timing.** `dp_ex2_vstart/tval2_vld/expt_vec/tval` are
  captured on a *second*, separately-clock-gated slice (`dp_trap_clk`, aq_rtu_dp.v:460-470,
  475-486) rather than the main `dp_ex2_*` set, with an explicit RTL comment explaining why:
  "inst_expt timing is bad. use lsu_cmplt_dp instead." (aq_rtu_dp.v:456-457). Purely a timing
  split of one logical retire packet — not a second pipeline stage or a second in-flight slot.
- **Open item — LSU miss vs. retire-vld exact timing not fully resolved from RTU's own RTL.**
  `lsu_rtu_ex1_cmplt` (retire heartbeat) is a different signal from `lsu_rtu_ex1_data`/
  `lsu_rtu_ex1_wb_vld` (rbus fast path), `lsu_rtu_ex2_data`/`_data_vld` (one-cycle-later
  forward, aq_rtu_rbus.v:333-335), and `lsu_rtu_wb_data`/`_vld`/`_dest_reg` (a *third*, fully
  separate write port consumed only in `aq_rtu_wb.v:180-197`, port1). The naming is consistent
  with a plain MEM->WB pipeline-register split for load-data timing (cmplt fires when the
  cache access resolves — including waiting out a miss — and the value is written into the GPR
  1-2 cycles later purely for timing, with `ex2_data` as the intervening bypass entry for a
  back-to-back dependent instruction) rather than an asynchronous/decoupled-retire scheme. I
  could not confirm this from RTU-side RTL alone; **the LSU/DCache researcher should confirm
  the miss-refill-to-`lsu_rtu_ex1_cmplt` timing directly from `lsu/rtl`.**

## 3. Register writeback: RTU does not contain the architectural regfile

- **The x0-x31 array physically lives in IDU, not RTU**, confirmed independently in
  `2026-08-20-c906-idu-extraction.md` §5.4: `aq_idu_id_gpr.v` + 31x `aq_idu_id_gpr_gated_reg.v`
  entries, write-enabled by `rtu_idu_wb0_vld`/`wb1_vld` exactly as this note's RTU-side reading
  predicts. RTU's job is purely to **compute and arbitrate** the final `{data, preg, vld}`
  triple for two write ports and hand them to IDU.
- **Port 0 (`aq_rtu_wb.v:186-190`)**: `rbus` winner OR VPU's direct GPR write
  (`wb_vpu_wb_grant = !wb_rbus_wb_vld`, line 175) — i.e. rbus (ALU/BJU/CP0/LSU-ex1/DIV/MUL)
  always wins over a same-cycle VPU (M5) write, mirroring the fact only one EX1 source can be
  valid by construction. Port-0 data is captured one extra cycle behind `rbus_wb_rbus_wb_*`
  (`aq_rtu_wb.v:160-166`, the "Rbus Datapath" register) before fan-out to IDU — a second
  timing-only pipeline stage, same pattern as §2's `dp_trap_clk` split.
- **Port 1 (`aq_rtu_wb.v:195-197`)** is the LSU's *own* late writeback
  (`lsu_rtu_wb_data`/`_dest_reg`/`_vld`), entirely independent of the rbus arbiter — this is
  the load-data-timing path flagged as an open item in §2.
- **rbus arbitration itself** (`aq_rtu_rbus.v:422-462`): EX1 group (ALU/BJU/CP0/LSU-ex1,
  pre-merged at `rbus_ex1_wb_dp`) > DIV > MUL(ex3), by `if/else if` priority — again safe only
  because the EX1 group is one-hot and DIV/MUL report independently on their own multi-cycle
  completion. Three separate forward (bypass, not architectural-write) ports to IDU exist in
  parallel (`rtu_idu_fwd0/1/2`, lines 479-489): fwd0 = same EX1 group, fwd1 = MUL-ex3, fwd2 =
  LSU-ex2 — matching IDU's 3 independent forward-mux inputs
  (`2026-08-20-c906-idu-extraction.md` line 326).
- **Preg width is uniformly 6 bits** across every source — `cp0_rtu_ex1_wb_preg[5:0]`,
  `iu_rtu_ex1_alu_preg[5:0]`, `lsu_rtu_ex1_dest_reg[5:0]`, and (M5) `vpu_rtu_gpr_wb_index[5:0]`
  — one bit wider than a bare 5-bit GPR index. This is strong (but not directly-verified-in-IDU)
  evidence that **the write-port datapath is already sized generically for a GPR/FPR class-select
  bit**, i.e. RTU's own plumbing does not need to change for M5's FP writeback; only the actual
  decode of bit `[5]` inside IDU's regfile (which I did not re-read) would need confirming.
  `2026-08-20-c906-iu-extraction.md` independently reports the same `preg[5:0]` width on every
  IU completion port (line 421,423,429,434), corroborating this.
- **FP/vector status side-channel, not GPR data**: `aq_rtu_wb.v:268-272` forwards
  `vpu_rtu_fflag_vld`/`vpu_rtu_fflag[5:0]` straight to `rtu_cp0_fflags`/`rtu_cp0_vxsat` (fflags
  + vxsat), and `aq_rtu_rbus.v:514-524` similarly ORs vs/fs-dirty bits from VPU, CP0, and
  RTU's own retire-stage LSU-dirty flags into `rtu_cp0_{fs,vs}_dirty_updt`. All M5/out-of-scope
  for M2's integer path, but confirms these status-update ports are likewise already
  multi-source-generic rather than needing rework later.

## 4. Exception detection and priority (`aq_rtu_retire.v`)

- **Sources arbitrated at retire**: pending-breakpoint (DTU debug trigger, timing-1, held from
  a prior split instruction), interrupt, LSU async bus error, normal breakpoint (ebreak / t0
  debug trigger), and the synchronous EX1 exception forwarded from CP0 or LSU
  (`dp_retire_ex2_inst_expt`, itself `cp0_rtu_ex1_expt_vld || lsu_rtu_ex1_expt_vld &&
  lsu_rtu_ex1_cmplt_dp`, aq_rtu_dp.v:409-410 — i.e. illegal instruction / ecall / misaligned /
  page-fault-vector-carrying causes are **CP0's and LSU's own EX1-stage job to detect**; RTU
  only consumes the already-decided `expt_vld` + 5-bit `vec` + `tval`).
- **Priority, exactly as coded** (`aq_rtu_retire.v:481-501`, one `if/else if` chain):
  1. `retire_pending_bkpt_expt` (buffered debug trigger from an earlier split sub-op) — cause 3
  2. `retire_int_inst` (interrupt, §5)
  3. `retire_async_expt` (LSU async bus error) — vec 5 (load) or 7 (store),
     `lsu_rtu_async_ld_inst ? 5 : 7` (line 451-452)
  4. `retire_bkpt_expt` (ebreak or t0 debug trigger) — cause 3
  5. else the synchronous EX1 exception's own vec (`retire_inst_expt_vec`, from CP0/LSU)
- **tval selection** (`aq_rtu_retire.v:530-550`): pending-bkpt uses DTU's buffered tval;
  interrupt uses 0; async uses the LSU bus-error address; a fixed vec allowlist
  `{1,2,4,5,6,7,12,13,15}` (line 514-522, matching inst-access-fault/illegal-inst/
  misaligned/access-fault/page-fault causes) uses the pipeline's own `dp_retire_ex2_tval`
  (sign-extended if MMU enabled, line 526-528); everything else is 0.
- **epc selection** (`aq_rtu_retire.v:564-577`): current PC for a synchronous exception (or an
  async exception landing on a split sub-op); otherwise next PC (covers interrupts and
  non-split async exceptions) — standard precise-exception PC bookkeeping, trivial here because
  only one instruction is ever "at" the exception point.
- **Trap actually taken** only when `retire_ex2_retire_vld && !halt_req && !dbg_mode_on &&
  (retire_expt_inst || retire_int_inst)` (line 585-587) — **debug-mode halt requests take
  priority over taking a trap at all**, deferring the exception to `rtu_dtu_retire_debug_expt_vld`
  instead (line 1229-1231) when already halted.
- **Page-fault plumbing already exists, MMU itself is M4-deferred**: `rtu_mmu_expt_vld`/
  `rtu_mmu_bad_vpn[26:0]` (aq_rtu_retire.v:1286-1288) fire whenever the taken trap's vec is
  1, 13, or 15 (`retire_mmu_trap`, line 503-505) and forward `tval[38:12]` as the faulting VPN
  — i.e. RTU's exception-vector routing needs **no new work** to carry a page-fault cause once
  MMU produces one; only the MMU/TLB itself is M4 scope.
  - **Open item, flag for M4**: standard RISC-V numbers instruction-page-fault as cause 12, but
    `retire_mmu_trap` checks `{1, 13, 15}`, not `{12, 13, 15}` — cause 1 is normally
    "instruction access fault." This may reflect C906's IMMU-stub reporting fetch failures
    uniformly as vec 1 rather than 12 (plausible per the M1 IFU note's "immu/high_hw expt"
    stub), but I did not chase this into `ifu`/`cp0` far enough to confirm; do not assume
    standard cause-12 routing works here without checking `cp0/rtl` and the IFU's IMMU stub
    when M4 lands.

## 5. Interrupt-taken sequencing

- **Cause priority is a flat casez over a 15-bit already-qualified vector**
  (`aq_rtu_int.v:52-74`): `cp0_rtu_int_vld[14:0]` arrives from CP0 pre-masked by that CSR's own
  mie/mip/mstatus.MIE logic (RTU does **no** enable/pending computation of its own — confirmed
  by the total absence of mstatus/mie-shaped signals anywhere in `rtu/rtl`). Standard causes
  present: MSI=3, MTI=7, MEI=11, SSI=1, STI=5, SEI=9. Three non-standard causes also appear,
  each **encoded twice** at two different priority tiers (bits [14:9] and [8:0]): mcip
  (16, bits 14 & 5), mhip (18, bits 13 & 4), moip (17, bits 6 & 0) — likely two upstream
  sources for the same cause (e.g. local vs. PLIC/CLINT-routed pending), but the module that
  builds `cp0_rtu_int_vld` lives in `cp0/rtl` and was not read; not resolved here.
- **RTU-side masking**: `int_vld = |int_vld_raw && !dtu_rtu_int_mask && !dp_int_ex2_inst_split`
  (aq_rtu_int.v:79-81) — blocked only by (a) DTU debug-mode interrupt mask and (b) being
  mid-way through a split (multi-beat) instruction, i.e. an interrupt cannot land between the
  two halves of one architectural instruction.
- **Taken exactly like any other trap**: once `retire_int_inst` wins §4's priority mux, the
  same `retire_trap_vld`/`retire_chgflw_vld`/flush-FSM path fires (§6) — there is no separate
  "interrupt injection" mechanism distinct from the exception path; `rtu_yy_xx_expt_int`
  (line 1154) is the only extra bit distinguishing an interrupt from a synchronous exception
  for CP0's `mcause` assembly (confirmed in `cp0/rtl/aq_cp0_trap_csr.v:1107`:
  `mcause_value = {m_intr, 58'b0, m_vector[4:0]}`, fed by `rtu_yy_xx_expt_int`/`_expt_vec`).
- **Redirect target = CP0's mtvec-derived value**, read combinationally by RTU
  (`cp0_rtu_trap_pc[39:0]`, consumed at `aq_rtu_retire.v:1035`; produced in
  `cp0/rtl/aq_cp0_trap_csr.v:1396`: `cp0_rtu_trap_pc = regs_trap_pc`, itself
  `mtvec_value` unless a vectored-mode interrupt selects `vec_int_pc` per-cause, lines
  1346-1358 — vectored mtvec mode exists in the reference but is CP0-internal, not re-verified
  here). RTU just latches this into `retire_chgflw_pc` (line 1034-1035) whenever
  `retire_trap_chgflw_vld` is set, one cycle after `retire_trap_vld`, and asserts
  `rtu_ifu_chgflw_vld/pc` when the flush FSM reaches `FLUSH_FE` (line 1003-1008, 1240-1242).

## 6. Flush/redirect signals RTU sends to earlier stages

**Flush FSM** (`aq_rtu_retire.v:929-978`), 5 states — materially simpler than C910's 7:
`IDLE -(flush_fe_set)-> FE -(drained)-> BE -> IDLE`, with `WAIT` inserted between FE and BE if
the pipeline isn't drained yet, and a direct `IDLE -> FE_BE -> IDLE` shortcut taken
**unconditionally** on an async debug halt request (`halt_req_dm_async`, line 939-940,
bypassing the normal next-state logic entirely — the only case that skips the drain wait).
`retire_flush_fe_set` (line 905-916) fires on: trap taken (expt or int), a CP0-signalled
flush instruction (fence/CSR-serializing/xret — `dp_retire_ex2_inst_flush`), a vstart update,
a timing-1 breakpoint request, a debug sync-flush request, or any halt request (t0 or t1).
Separately, `retire_bju_flush_req` (line 893-894: BJU RAS-mispredict or a dependent-LSU
changeflow) also forces the FE flush **without going through retire** (a branch/RAS
misprediction is resolved and flushed without ever being "the currently retiring instruction").
Drain gate: `retire_cpu_no_op` = no retire-vld this cycle AND `wb_retire_wb_no_op` (aq_rtu_wb's
"nothing to write back") AND `lsu/iu/vpu/vidu_rtu_no_op` all quiescent (line 980-985);
`retire_pipeline_empty` additionally requires `!lsu_rtu_ex1_buffer_vld` (line 986-987, an LSU
store-buffer-not-empty check) and is exported as `rtu_idu_pipeline_empty` for IDU's own
full-drain waits (e.g. fence.i-adjacent serialization).

Full RTU-originated signal inventory, by destination:

- **IFU**: `rtu_ifu_chgflw_vld`/`_pc[39:0]` (redirect target, line 1240,1242 — this is the
  signal M1 already consumed), `rtu_ifu_flush_fe` (= `retire_flush_fe`, the 1-cycle `FLUSH_FE`
  pulse, line 1241 — kills in-flight fetch/ICache-request/IBUF contents), `rtu_ifu_dbg_mask`
  (= `dbg_mode_on_after_req`, line 1244 — halts fetch while entering debug mode).
- **IDU**: `rtu_idu_flush_fe` (same FE pulse, line 1262 — kills the decode-stage instruction),
  `rtu_idu_flush_stall` (= `FLUSH_WAIT || FLUSH_BE`, line 1260 — holds new dispatch while
  waiting for drain and during the BE-flush cycle itself), `rtu_idu_flush_wbt` (=
  `retire_flush_be`, line 1263 — the 1-cycle `FLUSH_BE` pulse; named "wbt" = write-back-table,
  IDU's scoreboard, matching `aq_idu_id_wbt.v` in the IDU note — this is almost certainly what
  clears/resets IDU's busy-bit scoreboard on a flush), `rtu_idu_pipeline_empty` (drain status,
  not a flush pulse), plus the steady-state `rtu_idu_commit`/`_commit_for_bju` from
  `aq_rtu_ctrl.v:234-235` (`= !retire_ctrl_commit_clear[_for_bju]`, i.e. "this cycle's EX1
  completion is real, not about to be discarded by a same-cycle flush decision" — gates
  whatever per-cycle bookkeeping IDU does on a completion, e.g. its scoreboard-clear path;
  cross-check against IDU's own read of this signal, not independently re-verified here).
- **VIDU** (M5): `rtu_vidu_flush_wbt` (same `retire_flush_be` pulse, line 1264).
- **Broadcast (`rtu_yy_xx_*`, fans out to CP0/IU/LSU/VPU/MMU/HPCP/etc., exact consumer list not
  enumerated — only CP0's use was checked)**: `rtu_yy_xx_flush_fe` (= FE pulse, line 1159),
  `rtu_yy_xx_flush` (= BE pulse, line 1160 — this is the pulse CP0 uses to latch
  `mepc`/`mcause` on a trap, see §7), `rtu_yy_xx_expt_vld`/`_int`/`_vec[4:0]` (the trap
  declaration itself, line 1153-1155), `rtu_yy_xx_dbgon` (level, = `dbg_mode_on`, line 1157),
  `rtu_yy_xx_async_flush` (= registered `halt_req_dm_async`, line 1162 — a second, faster
  flush strobe specifically for the async-debug-halt shortcut path).
- **LSU**: `rtu_lsu_expt_ack` (= `retire_trap_chgflw_vld && retire_flush_be`, line 1269 — tells
  LSU a trap redirect is being committed this cycle), `rtu_lsu_expt_exit` (=
  `retire_xret_vld && retire_flush_be`, line 1270 — an mret/sret is committing), and
  `rtu_lsu_async_expt_ack` (line 1271-1273, debug-mode-gated ack of an async bus error) —
  these three are the "point of no return" signals LSU needs to release/drop any buffered
  store state; **directly relevant to M2's LSU base path**.
- **MMU** (M4): `rtu_mmu_expt_vld`/`rtu_mmu_bad_vpn` (§4).
- **HPCP** (perf counters): `rtu_hpcp_retire_inst_vld` (= retire-vld and not mid-split, line
  1279), `rtu_hpcp_retire_pc`, `rtu_hpcp_int_vld` (a registered "trap taken was an interrupt"
  pulse, line 1281,1089-1095).
- **DTU** (debug unit): a large block (`rtu_dtu_*`, lines 1176-1235) of halt-cause/pending-ack/
  retire-vld/mret/sret/branch-taken signals — debug-infrastructure plumbing, out of M2 scope,
  not detailed further here.

## 7. CSR writeback timing: not deferred to a commit stage — happens at CP0's own EX1

- **CSRRW/CSRRS/CSRRC's GPR side-effect (old CSR value into `rd`) goes through the *same*
  single-cycle EX1 completion + rbus arbiter as an ALU result** — `cp0_rtu_ex1_wb_data/_preg/
  _wb_vld/_wb_dp` are just another one-hot source alongside ALU/BJU/LSU in
  `aq_rtu_rbus.v:362-365` (declaration) and lines 389-407 (arbiter case, CP0 wins whenever its
  bit is set — mutually exclusive with the ALU/BJU/LSU group by the single-issue one-hot
  guarantee, not by an explicit priority decision). There is **no extra RTU-side buffering or
  commit gate** on this path beyond the generic `dp_ex2_*` register every EX1 result passes
  through (§2).
- **The CSR register array's own write likewise happens directly inside CP0 at its own EX1**,
  independent of RTU's retire bookkeeping: `cp0/rtl/aq_cp0_trap_csr.v:1339`,
  `mepc_local_en = regs_csr_wen && regs_csr_imm[11:0] == MEPC` — an ordinary decoded
  software-CSR-write strobe, presumably gated inside CP0 by its own privilege/legality check
  before `regs_csr_wen` is even asserted (not independently re-verified — CP0-internal). This
  confirms IU's finding (`2026-08-20-c906-iu-extraction.md` §9: "CSR read-modify-write is not
  an IU concern at all; it is entirely a CP0 concern") from the RTU/CP0 boundary side: **RTU
  does not stage, buffer, or defer ordinary CSR writes for precise-exception reasons.** This
  matches the task's expectation — C906 is in-order single-issue, so the CSR instruction *is*
  the oldest (only) in-flight instruction when it executes; there is nothing to roll back.
- **Trap-entry CSR capture is the one CSR write RTU *does* directly drive**, and it is a
  single un-buffered strobe, not a queued commit event: `cp0/rtl/aq_cp0_trap_csr.v:1042-1046`
  — `mepc_reg <= rtu_cp0_epc[63:1]` **on `rtu_yy_xx_expt_vld`**, else `<= iui_regs_wdata[63:1]`
  on ordinary `mepc_local_en` — two independent write sources arbitrated inside CP0's own
  always-block, both single-cycle. `rtu_cp0_epc`/`rtu_cp0_tval` are computed combinationally
  by RTU's §4 exception-priority logic and asserted for exactly the one cycle `rtu_yy_xx_expt_vld`
  is high (aq_rtu_retire.v:1153, 1249-1250). `mcause_value` similarly latches directly off
  `rtu_yy_xx_expt_int`/`_expt_vec` (`aq_cp0_trap_csr.v:1107`, gated by the same
  `mcause_local_en`, not traced further). mstatus's MIE/MPIE trap-entry swap was seen to also
  reference `rtu_yy_xx_expt_vld` in `aq_cp0_regs.v` (line ~2204) but the exact swap logic was
  not read in detail — out of this note's RTU-centric scope.
- **MRET/SRET redirect is uniform with every other changeflow**, not a special RTU case: CP0
  computes the return PC itself and asserts `cp0_rtu_ex1_chgflw`/`_chgflw_pc` during its own
  EX1 cycle (aq_rtu_dp.v:393-394); this flows through the identical `dp_ex2_inst_chgflw` ->
  `retire_chgflw_vld` -> `rtu_ifu_chgflw_vld/pc` path as a fence/CSR-serializing flush (§5, §6).

## 8. Minimal CSR set for M2

Grepped the entire `gen_rtl` tree: **zero occurrences of "tohost" anywhere in the reference
RTL.** This confirms tohost/HTIF is purely an rv906 testbench convention (the M0/M1 FetchSink
mechanism watching a fixed physical-address store) with no CP0/CSR involvement in real
silicon — C906 has no tohost CSR or register.

However, real riscv-tests (`rv64ui-p-*`, `rv64um-p-*`) are **not** CSR-free: their standard
harness (`riscv-tests/env/p/riscv_test.h`) sets `mtvec` to a trap handler at boot and expects
any unexpected trap (illegal instruction, misaligned access, page fault) *or* the deliberate
`ecall` used by `RVTEST_PASS`/`RVTEST_FAIL` to redirect correctly to that handler, which then
performs the tohost store itself. So even though the pass/fail **signal** is tohost/memory-based
(LSU/DCache concern, not RTU), the trap **redirect mechanics** this note documents are
load-bearing for M2 test pass/fail: `mtvec` (read via `cp0_rtu_trap_pc`, §5), `mepc` (write via
`rtu_cp0_epc`, §7), `mcause`-equivalent (`rtu_yy_xx_expt_vld/_int/_vec`, §4/§7), and a minimal
`mstatus` (MIE/MPIE swap on trap-entry/mret so the handler and the eventual `mret`-or-just-halt
sequence don't themselves fault) are **all required** for M2's rv64ui/um scope to pass, purely
so that ecall/illegal-instruction/misaligned traps land at the right handler PC with the right
epc/cause. `mcycle`/`minstret` (implemented in `cp0/rtl/aq_cp0_hpcp_csr.v`, not read in detail)
are **not** exercised by RTU at all and are very unlikely to be required for rv64ui/um pass/fail
specifically (they matter more for rv64mi/counter-focused tests) — recommend deferring them
unless M2's concrete test list is later found to check counter values. This assessment is
partly domain knowledge about riscv-tests' structure rather than something read out of the RTU
RTL itself, and should be cross-checked against whatever CSR/CP0 extraction note covers
`cp0/rtl` directly.

## 9. C906-specific simplifications vs. C910 (verified, not assumed)

- **No ROB, no IID, no age-tournament comparator, no rename/free-list/PST-preg lifecycle.**
  The entire "retire" concept collapses to one un-buffered EX1->EX2 pipeline register (§2) —
  there is no structure in `rtu/rtl` analogous to C910's `rob`/`rob_entry`/`rob_expt`/
  `pst_preg`/`compare_iid`.
  Retirement really is "at most 1/cycle, 0/cycle on any stall, no queue" — the simplest
  possible reading of the task's hypothesis, and directly confirmed by the RTL rather than
  assumed.
- **Flush FSM: 5 states, not 7** (`aq_rtu_retire.v:929-933`) — no separate SSF (split
  speculative-fail) FSM, no multi-state async-exception FSM. LSU async bus errors
  (`lsu_rtu_async_expt_vld`) are folded into the *same* single priority mux and the *same*
  flush FSM as every other trap (§4) rather than getting their own AE_IDLE/WFC/WFI machine —
  a direct consequence of there being only ever one instruction to blame.
- **No fold/no multi-commit.** C910 folds up to 3 AIQ/VIQ instructions into one ROB entry and
  can commit 3/cycle; C906 has no equivalent concept anywhere in `rtu/rtl` — the front end is
  genuinely single-issue end to end (matches the M1 IFU note's confirmed 1-inst/cycle
  `ifu_idu_id_inst` handoff).
- **CSR writeback needs no precision machinery** (§7) — C910's retire-alone-plus-full-flush
  serialization exists because CSR/fence ops must drain a real multi-instruction pipeline
  before executing; C906's CSR op simply *is* the only in-flight instruction when it runs, so
  "precise" is free. The flush FSM still exists here (§6) but only to sequence the
  front-end-kill / drain-wait / back-end-state-reload handshake for redirect targets, not to
  protect CSR ordering.
- **Write-port count**: 2 architectural-write ports (`rtu_idu_wb0/1`) vs. C910's ROB retiring
  up to 3 instructions/cycle each needing its own preg-release/architectural-map update — a
  direct consequence of ≤1 instruction completing per cycle plus one extra port for the LSU's
  late-data path (§3).

## 10. Read-directly spans

`aq_rtu_retire.v:606-887` (debug halt-request / breakpoint-trigger timing-0/timing-1
interaction with split instructions, including the buffered-split-trigger-merge logic) — dense
DTU-debug-specific logic sharing the *same* `flush_fe_set`/FSM path as ordinary exceptions
(§6), so a correct-enough M2 stub must not break this path even though DTU/debug triggers
themselves are out of scope. `aq_rtu_retire.v:929-987` (flush FSM + drain gating) — needs
faithful reproduction for M2's redirect sequencing. `aq_rtu_dp.v:338-383` (one-hot
inst_len/inst_split case-mux) — trivial but depends on the one-hot assumption from §2 holding.
`cp0/rtl/aq_cp0_trap_csr.v` (1439 lines) and the rest of `cp0/rtl` (15 files, `aq_cp0_top.v`
alone 48KB) — only the ~10 lines cited in §7 were read; treat as a wholly separate CSR
extraction task, not covered here.

## 11. Open items / not independently verified

1. Exact `lsu_rtu_ex1_cmplt` vs. cache-miss-refill timing (§2) — needs `lsu/rtl` confirmation.
2. `preg[5:0]` bit `[5]` GPR/FPR class-select semantics (§3) — inferred from uniform width
   across every source port, not confirmed by re-reading IDU's regfile decode logic.
3. `retire_mmu_trap`'s use of vec 1 (not 12) for an MMU-flavored trap (§4) — flagged for M4,
   not resolved; may be correct C906 behavior (IMMU stub reporting instruction-fetch faults as
   vec 1) or may need to also include vec 12 once a real IMMU exists.
4. The duplicate-tier interrupt-cause encoding in `cp0_rtu_int_vld[14:0]` (§5) — the module
   that produces this 15-bit vector lives in `cp0/rtl` (or possibly `clint`/`plic`) and was not
   read; RTU only consumes it as given.
5. `rtu_idu_commit`/`_commit_for_bju` (§6) — I read what *produces* these signals but did not
   re-verify what IDU actually *does* with them beyond the general "gates per-cycle
   bookkeeping" characterization.
6. mstatus MIE/MPIE trap-entry/xret swap sequencing and `mcause_local_en`'s exact gating (§7)
   — glimpsed in `cp0/rtl/aq_cp0_regs.v` but not read in detail; CP0-extraction scope.
