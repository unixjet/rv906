# C906 IU Execute Extraction Notes (M2 working material)

Source: `refs/openc906/C906_RTL_FACTORY/gen_rtl/iu/rtl/` (9 files, 4,954 lines;
`aq_iu_top.v`, `aq_iu_alu.v`, `aq_iu_bju.v`, `aq_iu_mul.v`, `aq_iu_div.v`,
`aq_iu_div_shift2_kernel.v`, `aq_iu_addr_gen.v`, `multiplier_33x33_partial.v`,
`booth_code_33_bit.v`; refs relative to this dir unless another dir is
named). Also opened `idu/rtl/aq_idu_id_gpr.v`/`aq_idu_id_gpr_gated_reg.v`
(register file lives there, not in IU) and grepped `idu/rtl/aq_idu_id_decd.v`,
`idu/rtl/aq_idu_id_ctrl.v`, `cp0/rtl/`, `lsu/rtl/` far enough to pin down
where CSR execution and AMO/LR/SC actually live (both outside IU — see §7/§8).

## 0. Architecture shape vs. C910 (read this first)

**Confirmed from RTL, not assumed**: C906's IU is a genuinely single execute
pipe. `aq_iu_top.v` instantiates exactly one `aq_iu_alu`, one `aq_iu_bju`
(+its `aq_iu_addr_gen` helper), one `aq_iu_mul`, one `aq_iu_div`
(top.v:399-585) — no `x2`/pipe0-pipe1 duplication anywhere, unlike C910's
`ct_iu_top` which instantiates `alu` **twice** (pipe0/pipe1, identical RTL)
for its dual-issue OoO scheme. There is no `cbus`/`rbus` arbitration module
inside `iu/rtl` either: each of ALU/BJU/MULT/DIV drives its own fixed,
independently-named output bus straight to RTU (§9), and the only
arbitration IU participates in is consuming two RTU-supplied one-bit grants
(`rtu_iu_mul_wb_grant`, `rtu_iu_div_wb_grant`, top.v:125-134) for the two
multi-cycle units — the arbiter itself is RTU-side, not IU-side. This is
consistent with C906 being single-issue in-order: there is only ever one
instruction in EX1 competing for these units, so no intra-IU issue
arbitration is needed, only a completion-vs-writeback-bus race between the
multi-cycle units and whatever the RTU is retiring that cycle.

## 1. Module graph (`aq_iu_top.v`, 603 lines of pure glue)

- `aq_iu_alu` (top.v:399) — 868 lines. Adder/shifter/logic/misc ALU, pure
  combinational, EX1 only (§2).
- `aq_iu_mul` (top.v:422) — 733 lines. 33x33 Booth-radix-4 partial multiplier
  reused iteratively for 64-bit operands; EX1-EX3 pipe (§5).
- `aq_iu_div` (top.v:453) — 831 lines. Radix-4 (2 bits/cycle) non-restoring
  divider with a leading-1 alignment stage and a 1-entry memo/hit buffer
  (§6). Instantiates `aq_iu_div_shift2_kernel` (top.v:711, the actual
  compare/subtract/shift kernel, 194 lines).
- `aq_iu_bju` (top.v:482) — 907 lines. Branch/jump resolution, RAS
  call/return detection, IFU redirect generation, and the *only* IU-internal
  operand-forwarding/stall logic (a private 1-entry "entry" buffer for
  LSU-dependent conditional branches, §4).
- `aq_iu_addr_gen` (top.v:575) — 94 lines. One shared 64-bit adder used by
  BJU for branch/jump target and AUIPC's `pc+imm` (§3) — **not** part of the
  main ALU adder.
- `iu_xx_no_op` (top.v:589) = `!idu_iu_ex1_inst_vld && div_ctrl_no_op &&
  mul_ctrl_no_op && bju_entry_no_vld && bju_ras_not_vld` — the single
  "IU is completely idle" signal (used for power/perf counting elsewhere).

No register file, no CSR/CP0 logic, and no AMO/atomics logic exists anywhere
under `iu/rtl` — all three are structurally elsewhere (§7/§8/§10).

## 2. ALU (`aq_iu_alu.v`) — single shared datapath, purely combinational, EX1-only

The whole module is `assign`/`always @(*)` with **no clocked always-block and
no pipeline register** — `iu_rtu_ex1_alu_data` is asserted the same cycle
`idu_iu_ex1_*` operands arrive (alu.v:844-860). One cycle, full stop; there
is nothing to stall internally.

**Op-group select is one-hot on `idu_iu_ex1_func[3:0]`** (func[19:0] is a
20-bit bus shared across ALU/BJU/MULT/DIV, gated per-unit by that unit's own
`*_dp_sel`, alu.v:179): `func[0]`=adder, `func[1]`=shifter, `func[2]`=logic,
`func[3]`=misc (alu.v:187,287,572,594). Final result is a 4-way `{64{sel}}&`
OR-mux of the four sub-blocks (alu.v:844-847) — genuinely one shared
datapath, not four parallel pipes.

- **Adder block** (alu.v:182-280): does ADD/SUB/ADDW/SUBW/SLT/SLTU/LUI/LI,
  all through **one 65-bit adder** (`alu_adder_add_rst[64:0] = rs0+rs1+sub`,
  alu.v:239). Width handling for RV64's `*w` ops is a **sign/zero-extend
  select on the adder's own 65-bit input, not a separate 32-bit ALU path**:
  a one-hot `alu_adder_rs0/rs1_sel_onehot` (alu.v:198-199) picks one of five
  operand-prepare cases per the `always` blocks at alu.v:203-236 — "sign 32
  op" (sign-extend bits[31:0]) and "unsign 32 op" (zero-extend bits[31:0])
  are the ADDW/SUBW-class cases; "sign 64 op"/"unsign 64 op" are the plain
  64-bit cases. The 65-bit sum is then truncated+re-sign-extended for word
  ops by `alu_adder_rst_word` (alu.v:250,274-275: `{{32{add_rst[31]}},
  add_rst[31:0]}`). **SLT/SLTU share this same adder**: `alu_adder_rst_lt =
  cmp && lt` selects `{63'b0, alu_adder_cin}` (alu.v:248,261,276) — the carry
  out of a two's-complement subtract, the standard trick — with signed vs.
  unsigned selected by which of the "sign 64"/"unsign 64" operand-prepare
  cases is chosen for that instruction (same onehot mux as ADD/SUB use).
  I did not reverse the exact IDU func-bit encoding that picks each of the
  5 onehot cases per opcode (that's decode-table work, IDU's scope) — the
  mechanism (one adder, width/sign selected by operand-prepare mux) is what
  matters for M2 and is fully confirmed in RTL.
- **Shift block** (alu.v:282-565): SLL/SRL/SRA and their `*W` forms all go
  through **one 128-bit-wide barrel shifter** (`alu_shift_shifter_rst[127:0]
  = {input_127_64,input_63_0} >> count`, alu.v:372), with left-shift done by
  bit-reversing the input, shifting right, then bit-reversing the result
  back (alu.v:304-319,374-392) — again one shared piece of hardware, not
  independent left/right shifters. `*W` variants zero/sign-extend
  bits[31:0] into the 64-bit shift input before the same shift
  (alu.v:359-364). XThead custom ops share this block: `srri`/`srriw`
  (rotate, `alu_shift_op_circle`, alu.v:294) and `ext`/`extu` (bitfield
  extract via a shift + one of 64 precomputed masks, alu.v:397-467).
- **Logic block** (alu.v:567-587): AND/OR/XOR, plain 64-bit, no width games
  (RISC-V has no `*w` logic ops).
- **Misc block** (alu.v:589-837): XThead custom ops only —
  `rev`/`revw` (byte-reverse), `tst`/`tstnbz` (bit-test / per-byte
  nonzero-test), `ff0`/`ff1` (find-first-0/1, a 64-way priority mux,
  alu.v:611-683), `mveqz`/`mvnez` (conditional move). **No MAX/MIN/ADDSL**:
  alu.v:245-265 has an explicit commented-out `alu_adder_rst_max`/
  `alu_adder_sel_rst` block ("4. max/min : src0 or src1") that is dead code
  in this file — C906's ALU does **not** implement the XThead
  MAX/MAXU/MIN/MINU/ADDSL long-ALU ops that C910's `alu.v` wires up live.
  Flagged as a genuine C906-simpler-than-C910 point (§11), not an oversight
  on my part.
- **AUIPC is NOT in this ALU.** `LUI` is (rs1-prepare case "lui",
  alu.v:228: `{src1[51],src1[51:0],12'b0}`, rs0 forced to 0), but AUIPC's
  `pc+imm` is computed by the separate `aq_iu_addr_gen` adder and delivered
  to RTU through BJU's writeback bus (§3/§9), not through
  `iu_rtu_ex1_alu_data`. Worth remembering when wiring M2's decode-to-IU
  op routing.

## 3. Address generator (`aq_iu_addr_gen.v`, 94 lines) — shared BJU-target/AUIPC adder

One 64-bit adder (`ag_adder_res = ag_adder_rs2 + ag_adder_rs1`,
addr_gen.v:84), instantiated once and used for two purposes selected by
`bju_ag_use_pc`/`bju_ag_offset_sel`:
- **Branch/JAL/JALR target**: rs1 = current PC (sign-extended per
  `mmu_xx_mmu_en`, addr_gen.v:77) or `src0` (register, for JALR), rs2 = a
  BJU-precomputed sign-extended offset (`bju_ag_offset`) or `src2` (the raw
  immediate).
- **AUIPC**: rs1 = current PC, rs2 = `src2` (the `imm[31:12]<<12` value) —
  same adder, `bju_ag_offset_sel=0` path (addr_gen.v:81).

Its output `ag_bju_pc[63:0]` feeds back into `aq_iu_bju.v` as `bju_ag_tar_pc`
(bju.v:597) for both the branch-target-vs-RAS-prediction compare and the
AUIPC result value (bju.v:809).

## 4. BJU (`aq_iu_bju.v`, 907 lines) — comparator, mispredict, RAS, and the only IU-side LSU interlock

### 4.1 Condition-code comparator (bju.v:604-616)
```
bju_beq_taken           = src0 == src1
bju_src0_lt_src1        = src0 <  src1                      // unsigned <
bju_src0_lt_src1_signed = (src0[63] & src1[63] & unsigned_lt)
                        |  (src0[63] & !src1[63])
                        | (!src0[63] & !src1[63] & unsigned_lt)
bju_blt_taken  = op_func[4] ? signed_lt : unsigned_lt        // [4]: BLT/BGE=1, BLTU/BGEU=0
cond_br_taken_raw = (beq_taken ^ op_func[3]) & op_func[2]     // BEQ/BNE group
                   | (blt_taken ^ op_func[3]) & op_func[1]    // BLT*/BGE* group
```
`op_func[3]` is the "invert" bit shared by BNE (inverts BEQ) and BGE*
(inverts BLT*); `op_func[2]`/`[1]` select which comparator group applies.
The signed-less-than is built the classic way (compare sign bits first,
fall back to the unsigned `<` only when both operands share a sign) rather
than via a wider sign-extended subtract — a different implementation
strategy from the ALU's SLT (which reuses the adder's carry-out, §2), i.e.
**BJU has its own private comparator, it does not call into the ALU
block**.

### 4.2 Prediction / resolution interaction (bju.v:614-641)
`bju_cond_br_taken` (the value actually used to pick next-PC) is
`cond_br_taken_raw` **only if operands are ready this cycle**
(`bju_ex1_inst_no_depd || bju_entry_pop`, bju.v:615); otherwise it
provisionally reuses the BHT prediction bit `bju_bht_pred[1]` while the
branch sits in BJU's private stall entry (§4.4) waiting on a load. Mispredict
is `bht_mispred_no_entry`/`_entry` = `cond_sel && (cond_br_taken ^
bht_pred[1])` evaluated either in EX1 (no dependency) or later when the
entry resolves (bju.v:637-638,651).

### 4.3 IFU redirect signals, full IU-side context (M1 already found these from the IFU side)
- `iu_ifu_tar_pc_vld`/`_gate`, `iu_ifu_tar_pc[63:0]` (bju.v:777-781) — the
  corrected-target redirect. Source is one of three: a mispredicted
  conditional branch held in the entry (`bju_not_pred_pc_flop`), a
  RAS/JALR-target mismatch (`bju_ras_mispred_vld`, using the *computed*
  target `bju_src0_flop`, not the RAS-predicted one), or the normal EX1
  `bju_next_pc` for a non-entry mispredict.
- **`iu_yy_xx_cancel = iu_ifu_tar_pc_vld`** (bju.v:783) — confirmed: this
  global "kill everything younger" signal is generated directly off BJU's
  own redirect-valid, nothing more.
- BHT feedback: `iu_ifu_br_vld(_gate)`, `iu_ifu_bht_cur_pc`,
  `iu_ifu_bht_taken`, `iu_ifu_bht_pred[1:0]`, `iu_ifu_bht_mispred(_gate)`
  (bju.v:785-791).
- RAS feedback: `iu_ifu_ret_vld`/`_gate` (JALR with rs1=x1, rd≠src, i.e. a
  real return — `bju_ret_vld_raw`, bju.v:657), `iu_ifu_link_vld`/`_gate`
  (JAL/JALR with rd=x1, a call — `bju_link_vld_raw`, bju.v:659),
  `iu_ifu_pc_mispred`/`_gate` (JALR with rs1≠x1: RAS shouldn't have
  predicted it at all — `bju_pc_reg_mispred`, bju.v:649).
- **JALR-vs-RAS-prediction check** (bju.v:646-654): `bju_pc_cmp_fail`
  compares the *computed* JALR target (`ag_bju_pc`, via addr_gen, §3)
  against `ifu_iu_ex1_pc_pred` (the RAS-predicted target IFU already
  speculated on) for the `rs1=x1` return case; a mismatch sets
  `bju_ras_mispred_vld` **one cycle later** (registered, bju.v:669-677) and
  forces the delayed redirect via `bju_not_ex1_chgflw` (bju.v:705). This is
  a separate, later-arriving mispredict path from the BHT-conditional-branch
  one — two distinct timing points for "IU says redirect," both funneling
  into the same `iu_ifu_tar_pc_vld` signal.
- `ifu_iu_chgflw_pc`/`_vld` (input, RTU/exception-driven redirect from IFU,
  highest priority — bju.v:692-693) and `ifu_iu_ex1_pc_pred` (input, the
  RAS-predicted target for the compare above) are the only two chgflw-shaped
  inputs IU consumes from IFU.

### 4.4 The only IU-internal operand-forward/stall logic: BJU's 1-entry LSU-dependent-branch buffer
Conditional branches (`bju_func[6]` only — unconditional jumps/AUIPC never
stall here) whose source registers aren't ready yet
(`idu_iu_ex1_src0_ready`/`src1_ready` = 0, i.e. the producer is an
outstanding load, bju.v:424-427) are held in a **single** flopped entry
(`bju_entry_vld`, `bju_entry_src0_vld`, `bju_entry_src1_vld`,
bju.v:200-217,490-556) rather than blocking the whole EX1 stage. The entry
is released by either of two forward sources:
- `da_xx_fwd_data`/`_dst_reg`/`_vld` (bju.v:430-431) — an early/fast forward
  bus, consumed **only by BJU** in this directory (not by ALU/MULT/DIV).
- `lsu_iu_ex2_data`/`_data_vld`/`_dest_reg` (bju.v:453-456) — the LSU EX2
  completion forward, also BJU-only here.

Backpressure to IDU is exactly the point-to-point pattern the project
convention wants: **`iu_idu_bju_full`** (entry occupied, both operands
already valid — bju.v:477) and **`iu_idu_bju_global_full`** (entry occupied,
still waiting on at least one operand — bju.v:478) are two separate signals,
letting IDU distinguish "BJU can drain this cycle" from "BJU is genuinely
stuck," both driven straight off BJU's own entry-valid bits with no
intermediate hub.

**This interlock is BJU-specific.** Nothing in `alu.v`/`mul.v`/`div.v`
consumes `da_xx_fwd_*` or `lsu_iu_ex2_*` — ordinary ALU/MULT/DIV ops must
therefore have their load-dependency stall resolved entirely upstream in
IDU's dispatch/scoreboard logic before `idu_iu_ex1_*` ever asserts, which is
IDU's extraction scope, not IU's; flagging it here so the IDU note's
bypass-path section knows BJU is the one exception it needs to cross-check
against.

### 4.5 IU's own private PC copy
BJU maintains its own `bju_pcgen_pc_39_1` register (bju.v:688-698), updated
on reset/`ifu_iu_chgflw_vld`/entry-resolved-redirect/normal EX1 completion —
i.e. **IU tracks "current PC" redundantly rather than reading it from IFU
every cycle**, feeding `iu_cp0_ex1_cur_pc`, `iu_lsu_ex1_cur_pc`,
`iu_rtu_ex1_cur_pc` (bju.v:811,828,833). Likely a timing decision (avoids a
cross-unit PC bus fanning into every consumer each cycle); worth keeping in
mind if M2 wants a single canonical PC source instead.

## 5. Multiply (`aq_iu_mul.v`, 733 lines) — one 33x33 Booth multiplier, reused iteratively; 3-stage pipe, extended for wide 64-bit operands

**Hardware**: a single `multiplier_33x33_partial` (33x33 Booth-radix-4
array, built from `booth_code_33_bit` 3-bit-Booth recoders, purely
combinational, outputs two 69-bit **redundant** (carry-save) partial
products `result_0`/`result_1`, mul.v:447-464) — **one physical multiplier
array shared by every RV64M op** (MUL/MULH/MULHSU/MULHU/MULW), not separate
per-instruction hardware. Sign/unsigned mode is selected per-operand by how
each 33-bit input is extended (`mul_ex1_src0_sign64 = !usign`,
`mul_ex1_src1_sign64 = sign && !su`, mul.v:266-267) — this is exactly how
MULHSU shares the array with MULH/MULHU (one operand sign-extended, the
other zero-extended into its 33rd bit).

**Pipeline**: EX1 (operand/func prepare) -> EX2 (33x33 multiply, purely
combinational within the array) -> EX3 (4:2-compressor final add +
result-width mux + writeback), each stage a real flop boundary
(mul.v:420-434,479-493). **3 cycles latency for any operand that fits in 33
bits** — i.e. RV32-style 32-bit multiplies (`MULW`) and RV64 multiplies
whose actual operand magnitudes fit in 33 bits (`mul_ex1_inst64_nosplit`,
checked by testing if the upper 32 bits are all-0 or (for signed operands)
all-1, mul.v:269-276).

**Wide 64-bit operands are handled by re-running the same 33x33 array up to
4 times**, not by a wider physical multiplier: a small FSM
(`IDLE->SPLIT0->SPLIT1->CMPLT`, mul.v:208-395) steps `mul_iter_count`
through 0..3, each pass feeding a different 32-bit half of each operand
into the 33-bit multiplier input (mul.v:311-341) and accumulating the
shifted partial results into a running 69-bit accumulator
(`mul_ex2_accumulate_rst`, `mul_ex3_acc_rst`, mul.v:470,483) plus a
carried low-64-bit register (`mul_ex3_low_64`, mul.v:563-580). **This is a
genuine variable-latency multiplier**: 3 cycles for the common/narrow case,
up to roughly 3 + 3 extra iteration passes (~6 cycles) for a full 64x64
multiply whose operands don't fit in 33 bits — a magnitude-dependent
early-out exactly analogous in spirit to C910's SRT divider early-out, but
here it's the **multiplier** that's variable-latency, not the divider (the
divider has its own, different early-out, §6).

**Stall/backpressure — point-to-point, not broadcast**, matching the
project convention:
- `iu_idu_mult_issue_stall` (mul.v:597) — tells IDU "don't issue a new mult
  op this cycle," asserted while starting or mid-iteration
  (`mul_cur_state==IDLE && iter_start || mul_iter`).
- `iu_idu_mult_full` (mul.v:598) — tells IDU "EX2 is backed up," asserted
  when EX2 holds a non-iterating instruction whose EX3 wants to write back
  but hasn't been granted the RTU write port yet
  (`mul_ex3_wb_vld && !rtu_iu_mul_wb_grant_for_full`).
- `mul_ex3_stall = mul_ex3_wb_vld && !rtu_iu_mul_wb_grant` (mul.v:561) — the
  actual EX3-stage stall, driven by an RTU-supplied one-bit grant, not an
  IU-internal arbiter (§0/§9).

## 6. Divide (`aq_iu_div.v` + `aq_iu_div_shift2_kernel.v`) — radix-4 non-restoring divider, data-dependent latency, 1-entry memo buffer

**Not an SRT divider and not shared with an FP unit** — this is a much
simpler design than C910's SRT-radix-16-shared-with-vfdsu divider. The
kernel (`aq_iu_div_shift2_kernel.v`) computes **2 quotient bits per cycle**
via three parallel subtractions of the divisor at 1x/2x/3x against the
running remainder (`div_suber_res1_01/10/11`, kernel.v:137-146) and a 4-way
select (kernel.v:151-185) — a textbook radix-4 restoring-compare divider,
not a quotient-digit-selection SRT array.

**FSM** (div.v:242-313): `IDLE -> WFI2 -> ALIGN -> ITER -> CMPLT -> WFWB`.
- `IDLE`: operand prep + **early-out abnormal-result detection**, all
  combinational and resolved same-cycle: divide-by-zero (`q=all-1s,
  rem=dividend`), dividend==0, and signed overflow (`MIN_INT / -1`)
  (div.v:330-349) — these skip straight to `WFWB` in 1 cycle (`div_ex1_res_vld`).
  Also checks the **1-entry hit/memo buffer** (`div_hit_buffer`,
  div.v:779-787): if this op's dividend/divisor/sign/word fields exactly
  match the immediately-preceding divide, the cached result is reused
  (`div_hit_buffer_res_vld`) instead of re-iterating — same fast-exit path
  as the abnormal cases.
- `WFI2`/`ALIGN`: compute each operand's absolute value and leading-one
  position (`div_ff1_res`, a 64-way priority encoder, div.v:426-636) for
  both dividend and divisor, then **align the divisor's leading one to the
  dividend's** by left-shifting it (`div_divisor_update_data`,
  kernel.v:130) and computing the iteration count as the *difference* in
  leading-one positions (`div_iter_count`, kernel.v:99-123) — this is the
  early-out mechanism: **iteration count is data-dependent** (small when
  the divisor is close in magnitude to the dividend or larger; up to ~32
  passes at 2 bits/pass for a 64-bit dividend divided by a very small
  divisor).
- `ITER`: 2 bits/cycle via the kernel until `div_iter_count` reaches 0/1
  (kernel.v:125-126).
- `CMPLT`: sign-correct quotient/remainder (div.v:730-738) and, if the
  instruction is a `*W` op, sign-extend the 32-bit result.
- `WFWB`: pure wait-for-RTU-grant state if `CMPLT` couldn't write back
  immediately.

**DIV/DIVU/REM/REMU (and their `*W` forms) share one divider core.** Func
bits are just `word`(0)/`quotient-select`(1)/`signed`(2) (div.v:191-193);
the core always computes **both** quotient and remainder simultaneously
into `div_quotient_reg`/`div_remainder_reg` (div.v:688-707), and
`div_res_sel_quotient_flop` (func[1], latched) just picks which one feeds
`iu_rtu_div_data` at the end (div.v:748) — there is no separate remainder
unit.

**Latency**: abnormal/hit-buffer fast path ~2 cycles; otherwise
`1(IDLE)+1(WFI2)+1(ALIGN)+N(ITER, data-dependent, up to ~32)+1(CMPLT)` before
`WFWB`, i.e. roughly 4 to ~36 cycles depending on operand magnitudes —
genuinely variable/early-out, not fixed-cycle.

**Stall/backpressure**: `iu_idu_div_full` (div.v:764) — asserted whenever
DIV is busy (`prepare_src1 || align || iterating`) or has a result ready but
not yet granted (`(cmplt||wfwb) && !rtu_iu_div_wb_grant_for_full`) — same
point-to-point shape as MULT's (§5), consumed only by IDU, with the actual
per-cycle writeback grant (`rtu_iu_div_wb_grant`) again an RTU-supplied
single bit, not an IU-internal arbiter.

## 7. Atomics (LR/SC/AMO*) — confirmed in LSU, not IU

Grepped the whole `gen_rtl` tree: `lsu/rtl/aq_lsu_amo_alu.v` exists and
`iu/rtl` has nothing AMO-shaped at all (no port, no func bit, no module).
IDU's decoder/split logic (`idu/rtl/aq_idu_id_split.v`,
`idu/rtl/aq_idu_id_ctrl.v`) shows AMO/fence-adjacent instructions dispatched
under an `EU_CP0` or LSU execution-unit tag, never `EU_IU`. **This confirms
the task's hypothesis**: LR.W/LR.D/SC.W/SC.D and every AMO* op execute
entirely inside LSU as a read-modify-write against the DCache/memory port,
with a dedicated small ALU (`aq_lsu_amo_alu.v`) doing the AMO's
add/and/or/xor/max/min/swap arithmetic on the loaded value before the
store-back. I did not open `aq_lsu_amo_alu.v` itself — that belongs to the
LSU extraction task, not this one; noting only that IU is completely
uninvolved.

## 8. Register file — lives in IDU, not IU; IU never sees a regfile port

No regfile/GPR file exists under `iu/rtl`. `idu/rtl/aq_idu_id_gpr.v` (824
lines) instantiates `aq_idu_id_gpr_gated_reg.v` (114 lines) **31 times**
(one flop-based 64-bit register per architectural register x1-x31,
gpr.v:117-160+); x0 is hardwired: `read_data_0[63:0] = 64'b0` (gpr.v:110),
no register instance backs it.

- **Read**: 3 ports (`gpr_dp_src0_data`/`src1`/`src2`, gpr.v:46-48),
  combinationally muxed from the 32 `read_data_N` wires by
  `dp_gpr_src0_reg`/`src1_reg`/`src2_reg` — the module has no read clock
  input at all, so read is purely combinational off whatever IDU's dispatch
  stage currently holds as the source register numbers. (I did not read the
  32-way read-mux case statement itself, gpr.v:~161-824 — the port-level
  conclusion "3 read ports, combinational" is solid; the exact mux
  implementation is IDU-extraction-scope detail.)
- **Write**: exactly **2 write ports** per register cell
  (`rtu_idu_wb0_data`/`wb1_data` + one-hot-decoded per-register
  `wb0_vld[N]`/`wb1_vld[N]`, gated_reg.v:85-99). **Priority note worth
  flagging**: the write-mux is `case({wb1_vld_x,wb0_vld_x})` with only
  `2'b01`->wb0 and `2'b10`->wb1 spelled out explicitly; both `2'b00` (no
  write) and `2'b11` (**both ports target this register the same cycle**)
  fall into the same `default: write_data = reg_dout` (hold-old-value,
  gated_reg.v:93-97). If wb0 and wb1 are ever allowed to collide on the same
  destination register, **the write is silently dropped**, not merged or
  prioritized. This is presumably guaranteed never to happen by RTU's
  commit/retire arbitration logic upstream (a single register can't retire
  twice in one cycle) — but it's a real correctness contract living outside
  this file, worth a callout for whoever writes RTU's extraction note.
- **What IU actually sees**: `idu_iu_ex1_src0/1/2_data[63:0]` arriving at
  `aq_iu_top.v` are **already-read, already-forwarded** operand values
  (top.v:593-595 is a pure rename, no mux) plus two 1-bit "ready" flags
  (`idu_iu_ex1_src0/1_ready`, only consumed by BJU, §4.4). IU itself never
  touches a register-file port; all GPR read/bypass-mux work is IDU's, and
  all GPR write-port arbitration is RTU's (IU only produces
  {data,preg,valid} tuples toward RTU, §9). This is the clean boundary the
  IDU extraction task's bypass-path section should build on.

## 9. CSR execution — confirmed in CP0, IU is fully bypassed

Grepped `idu/rtl/aq_idu_id_decd.v:2126-2161`: CSRRW/CSRRS/CSRRC/CSRRWI/
CSRRSI/CSRRCI are decoded there into `FUNC_CSRRW` etc., and
`idu/rtl/aq_idu_id_ctrl.v:574,632,649,660` show these (along with
fence/sfence/AMO-fence/csync) are dispatched under a **separate execution-
unit tag, `EU_CP0`**, driving a wholly separate `idu_cp0_ex1_*` port bundle
out of `aq_idu_top.v` (grepped: `idu_cp0_ex1_dp_sel`, `_dst0_reg`,
`_expt_*`, `_func`, `_opcode`, `_sel`, `_src0_data`, `_src1_data`, etc.,
`aq_idu_top.v:40-54`) that never touches `aq_iu_top.v` at all. `cp0/rtl/`
has its own `aq_cp0_regs.v`/`aq_cp0_trap_csr.v`/`aq_cp0_prtc_csr.v`/etc.
doing the actual CSR-address decode and read-modify-write. **`iu/rtl` has
zero CSR-shaped signals anywhere in its port lists** (checked every file) —
the only IU<->CP0 connection at all is `iu_cp0_ex1_cur_pc` (bju.v:828, a
plain PC passthrough CP0 needs for `mepc`/trap-context bookkeeping) and
`cp0_iu_icg_en`/`cp0_xx_mrvbr`/`mmu_xx_mmu_en` (clock-gate enable, reset
vector, MMU-enable — configuration inputs, not CSR RMW traffic). For M2:
CSR read-modify-write is **not** an IU concern at all; it is entirely a
CP0-unit extraction task (out of scope here, flagging the exact dispatch
seam — `EU_CP0` — for whoever picks that up).

## 10. Writeback path to RTU — four independent per-unit buses, not one combined bus

Unlike what a single clean "IU→RTU pipeline register" might suggest, C906's
IU exposes **four separate, differently-shaped output buses** to RTU, one
per execution unit, all live simultaneously (top.v ports):
- ALU (EX1, always-valid-if-selected): `iu_rtu_ex1_alu_{cmplt,cmplt_dp,
  data[63:0],inst_len,inst_split,preg[5:0],wb_dp,wb_vld}` (alu.v:853-860).
- BJU (EX1, or later if the LSU-dependent entry resolves it):
  `iu_rtu_ex1_bju_{cmplt,cmplt_dp,data[63:0],inst_len,preg[5:0],wb_dp,
  wb_vld}` plus `iu_rtu_ex1_branch_inst`, `_cur_pc`, `_next_pc`,
  `iu_rtu_ex2_bju_ras_mispred`, `iu_rtu_depd_lsu_chgflow_vld/_next_pc`
  (bju.v:804-816) — richer than the others because BJU also has to report
  PC-increment bookkeeping and the delayed-entry mispredict case.
- MULT (EX3): `iu_rtu_ex1_mul_cmplt(_dp)` at EX1 (early "I've accepted
  this op" signal) + `iu_rtu_ex3_mul_{data[63:0],preg[5:0],wb_vld}` at EX3
  (mul.v:588-592) — note the **completion notification (EX1) and the
  data/writeback (EX3) are reported at different pipeline points**, gated
  by the RTU wb-grant (§5).
- DIV (variable EX-stage): `iu_rtu_ex1_div_cmplt(_dp)` at EX1 (accept) +
  `iu_rtu_div_{data[63:0],preg[5:0],wb_dp,wb_vld}` whenever `CMPLT`/`WFWB`
  finally fires (div.v:755-762) — same early-accept/late-data split as MULT.

**None of these carry exception/fault flags** — no `expt_vld`/`expt_vec`
field anywhere in IU's RTU-facing ports (contrast C910's `cbus` payload,
which does carry `abnormal`/`expt_vec`/`mtval` etc.). The only
fault-adjacent thing IU produces is BJU's own mispredict/RAS-fail bits,
which are branch-resolution info, not architectural exceptions — illegal-
instruction/page-fault/etc. must be carried by IDU/CP0's own path to RTU
(`idu_cp0_ex1_expt_*`, seen in §9's grep) rather than through IU. **For M2's
single combined-signal-per-boundary convention**: this file confirms RTU is
where these four buses would need to be merged if you want one canonical
"IU result" signal — that merge does not exist inside `iu/rtl` itself; it's
RTU-side work (or a deliberate M2 design decision to keep them separate,
mirroring C906's own choice).

## 11. C906-specific simplifications vs. C910 (verified in RTL, not assumed)

1. **Single execute pipe**, no pipe0/pipe1 ALU duplication (§0) — expected
   given single-issue, but confirmed via `aq_iu_top.v`'s instance list.
2. **No `cbus`/`rbus` arbiter module in IU** — C910 has dedicated `cbus.v`/
   `rbus.v` files doing multi-source completion/writeback-bus arbitration
   inside IU; C906 has none. Arbitration for the two units that need it
   (MULT/DIV) is pushed entirely into RTU via single-bit grant inputs
   (§0/§5/§6).
3. **No XThead MAX/MAXU/MIN/MINU/ADDSL "long ALU" ops** — dead/commented-out
   in `alu.v` (§2), whereas C910's `alu.v` has these live per the sibling
   note.
4. **Multiply is a single reused 33x33 array**, not a wider one-shot
   multiplier — 64x64 operands cost extra iteration passes through the same
   hardware (§5), a genuinely different (cheaper-area, sometimes-slower)
   design point than a single-pass 65x65 array.
5. **Divide has no shared-FP-divider dependency at all** — C910's divider
   borrows an external SRT-radix-16 core shared with the FP divide unit
   (`vfdsu`); C906's `aq_iu_div.v` is a fully self-contained radix-4
   compare/subtract divider with its own tiny memo buffer (§6), no
   cross-unit register-borrow ports, no FP entanglement. Much simpler to
   clean-room.
6. **GPR write ports: 2, not more** — `aq_idu_id_gpr.v` only wires
   `wb0`/`wb1` (§8); no evidence of a 3rd port, consistent with single-issue
   (at most: one normal EX1 completion + one delayed MULT/DIV/BJU-entry
   completion needing to retire in the same cycle).
7. **BJU's own 1-entry stall buffer plus direct LSU forward taps** (§4.4) is
   IU-internal machinery C910's note doesn't call out at this granularity —
   worth double-checking against the eventual IDU/RTU notes whether C910 has
   an analogous per-unit private entry or does all of this centrally.

## 12. Read-directly spans (too intricate to fully resolve at this pass)

- `alu.v:198-236` (adder rs0/rs1 onehot operand-prepare — I traced the
  *mechanism* but not which exact `idu_iu_ex1_func` bit pattern IDU will
  emit per opcode; that's decode-table work).
- `mul.v:280-345` (split-vs-nosplit source-operand mux across the 4 FSM
  states — the iter_count-to-operand-half mapping is dense).
- `div.v:378-406` (special-result-quotient/remainder onehot merge) and
  `div_shift2_kernel.v:130-185` (the 2-bit-per-cycle compare/select/shift
  core) — both correct as traced, but worth a fresh read immediately before
  reimplementing given how compressed the encodings are.
- `bju.v:636-677` (RAS/JALR mispredict timing: `bju_pc_cmp_fail` combinational
  this cycle vs. `bju_ras_mispred_vld` registered next cycle, feeding
  `bju_not_ex1_chgflw` — the one-cycle-later redirect path interacting with
  the entry logic is subtle).
- `multiplier_33x33_partial.v`/`booth_code_33_bit.v` in full — confirmed
  structurally (Booth-radix-4 recoding, two redundant 69-bit outputs) but
  not traced gate-by-gate; irrelevant to M2's architectural-level clean-room
  scope, relevant only if synthesis-level multiplier design is ever copied
  verbatim (it shouldn't be).

## 13. Open items / not independently verified

- The precise `idu_iu_ex1_func` bit-to-opcode table for ALU/BJU/MULT/DIV is
  IDU decode-table work I did not extract here (out of this task's scope,
  per the assignment) — I only confirmed the *shape* of each unit's op-group
  selection, not the full RV64IMA opcode-to-func mapping.
  Cross-reference with the IDU extraction note before wiring M2's decode
  logic.
- `idu_iu_ex1_src0/1_ready` semantics: I inferred "producer is an
  outstanding load" from context (only consumed by BJU's LSU-dependency
  logic, bju.v:426-427), but did not verify against IDU's scoreboard RTL
  that this is the *only* reason a source can be not-ready (e.g., could a
  still-in-flight MULT/DIV result also clear this bit? plausible but
  unconfirmed — would matter if M2's IDU scoreboard needs to set it for
  more than just loads).
- Whether `rtu_iu_ex1_inst_split`/`idu_iu_ex1_split` (consumed at
  bju.v:581-582,663,806) represents the same "instruction split into two
  RTU-visible pieces" mechanism C910 uses for `jal`/`jalr`-with-link (per
  the sibling note's pcfifo dual-slot rule) — plausible given the naming and
  usage pattern, but I did not cross-check against IDU's actual split logic
  (`idu/rtl/aq_idu_id_split.v`) to confirm it's the same concept rather than
  something narrower.
- I did not open `aq_lsu_amo_alu.v` (§7) or `aq_cp0_regs.v`/
  `aq_cp0_trap_csr.v` (§9) beyond confirming their existence and the
  dispatch-seam signal names — both are explicitly other tasks' scope per
  the assignment, flagged here only so those tasks know exactly where to
  start.
