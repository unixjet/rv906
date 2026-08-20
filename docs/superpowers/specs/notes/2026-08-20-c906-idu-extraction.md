# C906 IDU Extraction Notes (M2 working material)

Source: `refs/openc906/C906_RTL_FACTORY/gen_rtl/idu/rtl/` (all file:line refs relative to it
unless another dir is named). Files: `aq_idu_top.v` (810 lines, pure glue), `aq_idu_id_decd.v`
(6662 lines, decoder), `aq_idu_id_split.v` (914 lines, multi-beat instruction splitter),
`aq_idu_id_gpr.v` (824 lines) + `aq_idu_id_gpr_gated_reg.v` (114 lines, x1 entry each),
`aq_idu_id_wbt.v` (1212 lines, scoreboard) + `aq_idu_id_wbt_entry.v` (167 lines, x1 entry
each), `aq_idu_id_dp.v` (1147 lines, datapath/forward mux/EX1 pipeline register),
`aq_idu_id_ctrl.v` (757 lines, stall + EU-select control), `aq_idu_expand_32.v` (72 lines,
one-hot 5b->32b expander utility), `aq_idu_cfig.h` (opcode/field-width config, already
partially explored in M1).

Style/content calibration only (not a content source — C910 is OoO with rename/a scoreboard
of a different shape): `rv12/docs/superpowers/specs/notes/2026-08-19-c910-idu-extraction.md`.

## 0. Architecture shape vs. C910 (read this first)

C906's IDU has **no rename and no physical register file**. The architectural GPR is read
directly at decode/dispatch time (`aq_idu_id_gpr.v`), hazards are tracked by a **fixed
32-entry busy-bit scoreboard indexed directly by architectural register number** (the
"write-back table", `aq_idu_id_wbt.v` — 31 flop-based entries + a hardwired x0), and there is
exactly **one dispatch slot per cycle** (a single `ex1_inst_vld`/`ex1_eu_sel` pipeline
register in `aq_idu_id_ctrl.v:598-616`). This is structurally much simpler than C910's
scoreboard/ROB. It is **not** simpler in every respect, though: the RAW/WAW hazard logic in
`aq_idu_id_ctrl.v` carries a dense set of per-producer-type "except" clauses (§3) so that
in-order dispatch doesn't stall more than necessary, and a genuine multi-beat
decode-and-redispatch FSM (`aq_idu_id_split.v`, §5.3) exists for a handful of
instruction classes. Also verified, not assumed: **all inter-instruction forwarding into
IDU's operand-read logic is routed through RTU** (`rtu_idu_fwd0/1/2_*`, `rtu_idu_wb0/1_*`),
not via direct wires from EU1/LSU — see §4.

## 1. Module graph (`aq_idu_top.v:483-792`, one instance each, no unrolling)

- `aq_idu_id_decd` (`x_aq_idu_id_decd`, top.v:484) — the decoder. Pure combinational; takes
  the raw 32-bit `ifu_idu_id_inst`/`dp_decd_inst` and produces one 265-bit bus
  `decd_dp_inst_data[`IDU_WIDTH-1:0]` (`IDU_WIDTH`=265, cfig.h:194-198).
- `aq_idu_id_split` (`x_aq_idu_id_split`, top.v:502) — multi-beat splitter for
  load-store-double/AMO/cache-maintenance/fence instruction classes (§5.3). Produces an
  alternate 265-bit bus (`split_dp_inst_data`) of the *same* IDU_* layout, muxed against the
  decoder's output.
  Also takes `cp0_yy_priv_mode` — mret/sret/dret and privilege-mode-adjacent fence checks
  interact with this module too (cf. `sfence.vma` "deal in fence / split", decd.v:2119-2120).
- `aq_idu_id_gpr` (`x_aq_idu_id_gpr`, top.v:525) — the architectural GPR, read this cycle.
- `aq_idu_id_wbt` (`x_aq_idu_id_wbt`, top.v:543) — the 32-entry busy-bit scoreboard.
- `aq_idu_id_dp` (`x_aq_idu_id_dp`, top.v:574) — datapath top: decoder/splitter output mux,
  WBT/GPR read-address generation, the 3-source forward mux, the ID/EX1 operand mux, and the
  EX1 pipeline register itself (both integer 311b and vector 180b copies).
- `aq_idu_id_ctrl` (`x_aq_idu_id_ctrl`, top.v:697) — RAW/WAW stall computation, EU one-hot
  select, EX1 issue-enable gating (commit/full checks), and the front-end stall output
  `idu_ifu_id_stall`.

No separate CSR/CP0 register file lives in `idu/rtl`. IDU decodes CSR ops and hands operands
to CP0 via `idu_cp0_ex1_*` ports exactly like it hands ALU ops to IU and loads/stores to LSU
(§6) — CP0 is a distinct top-level unit (consistent with M1's finding of `cp0_ifu_*` signals
on the front end).

## 2. Pipeline shape: ID+DIS is ONE combinational stage, EX1 is the next register

The RTL's own naming ("id" vs "dis") suggests two stages, but there is **no pipeline
register between them**. `ctrl_dis_inst_vld` is computed directly off the same-cycle
`ifu_idu_id_inst_vld` (`aq_idu_id_ctrl.v:350-351`), and *all* of decode, split-mux, GPR/WBT
read, forward mux, RAW/WAW stall computation, and EU one-hot select happen combinationally
within that one cycle. The **only** registered boundary inside IDU proper is the EX1 latch:
- `ex1_inst_vld`/`ex1_eu_sel[9:0]` in `aq_idu_id_ctrl.v:598-616` (the "which EU" tag), and
- `ex1_int_inst_data[310:0]` / `ex1_vec_inst_data[179:0]` in `aq_idu_id_dp.v:934-1064` (the
  operand/func/dst payload, split into per-field clock-gated sub-registers so an unready
  operand doesn't force-toggle the whole 311-bit register every cycle — see the four
  separate `ex1_int_src{0,1,2}_clk`/`ex1_int_inst_clk` gates, dp.v:805-891).

So for M2's design doc: **ID and Dispatch are the same physical stage** (decode + hazard
check + operand read + forward + EU select, all combinational), and **EX1 is the first real
pipeline register**, shared by every functional-unit destination (IU/LSU/CP0/VIDU each get
their own view of the *same* EX1 register via field-slicing, `aq_idu_id_dp.v:1066-1132`).
There is a second stall point *after* this register too (§5.2) — EX1 itself can hold
(`ctrl_ex1_stall`), and that holds back new dispatch (`ctrl_dis_stall` includes
`ctrl_ex1_stall`, ctrl.v:390-394).

## 3. Decode structure (`aq_idu_id_decd.v`)

### 3.1 Coarse classifier, then parallel per-class sub-decoders

A 6-bit one-hot classifier `decd_sel[5:0]` (`decd.v:989-1011`) picks exactly one of six
per-class decode results to become `x_decd_*` (`decd.v:1090-1242`, `case(decd_sel[5:0])`):

| bit | class | condition (decd.v) | producer signals |
|---|---|---|---|
| `[0]` | 32-bit int/LSU/BJU/CP0 | `decd_length && !fp && !cache && !perf` (993-997) | `decd_32_*` |
| `[1]` | 16-bit (RVC) | `!decd_length` (999) | `decd_16_*` |
| `[2]` | scalar FP | opcode `1010011` or the FP-load/store `10011` pattern (989-1001) | `decd_fp_*` |
| `[3]` | cache-maintenance (custom) | fixed 21-bit pattern, gated by `cp0_idu_cskyee` (1003-1005) | `decd_cache_*` |
| `[4]` | "perf"/custom-0 | opcode `0001011`, funct3≠0, gated by `cp0_idu_cskyee` (1007-1009) | `decd_perf_*` |
| `[5]` | vector | **hardwired `1'b0`** (1011) | `decd_vec_*` (dead) |

**Important, verified fact:** vector decode is **structurally present but unreachable** in
this RTL drop. `decd_sel[5] = 1'b0` (decd.v:1011) and `decd_vec_inst = 1'b0`
(decd.v:3740, feeding `decd_inst_vec` at 3754). Despite this, the file still carries ~2400
lines of vector opcode decode (`casez({x_inst[31:26],x_inst[14:12]})`, decd.v:4172-6566) and
a full complement of `cp0_idu_vlmul`/`cp0_idu_vsew`/`cp0_idu_vstart`/`EU_VEC_SEL` plumbing
that a `grep` alone would suggest is live. **Do not assume V-extension support carries into
M2** from the presence of these signals — in this configuration no instruction can ever
classify as vector. (FP class `[2]` is likewise out of M2's minimal-CSR/base-integer scope
but *is* reachable — flagging only vector as dead, not FP.)

Classes `[3]`/`[4]` are C-SKY legacy custom-opcode-space (`0001011`) instructions gated by
`cp0_idu_cskyee` — not core RV, irrelevant to a minimal RV64GC M2 scope, but they explain why
opcode `0001011` shows up repeatedly in the immediate-selection logic (§3.3 below, the
"lsi"/"lsr" cases) even though it isn't a standard RV major opcode.

### 3.2 The 32-bit integer sub-decoder is itself several parallel bit-keyed case tables

Rather than one hierarchical opcode→funct3→funct7 tree, `decd_32_*` is assembled from
multiple independent `casez` blocks over different raw bit-field groupings, each covering a
disjoint slice of the encoding space, OR'd/muxed together:

- `casez({x_inst[31:25], x_inst[14:12], x_inst[6:2]})` — decd.v:1541-2169. The main table:
  R-type ALU/M-extension ops, I-type ALU-immediate ops, LUI/AUIPC, branches, loads, JAL/JALR,
  FENCE/FENCE.I, ECALL/EBREAK/MRET/SRET/DRET/WFI, and CSRRW/S/C(+I) (§6). Example entries
  (decd.v:1568-1650): `beq`→`EU_BJU`/`FUNC_BEQ`, src0_vld+src1_vld+src2_imm_vld;
  `lb`→`EU_LSU`/`FUNC_LB`, src0_vld+src1_imm_vld+dst0_vld.
- `casez({x_inst[31:20], x_inst[14:12]})` — decd.v:2194-2772 (a second I-type-keyed table;
  not traced in full — FP-load/misc I-type overlap, out of M2 scope to fully unwind).
- `casez({x_inst[26:25], x_inst[4:2]})` — decd.v:2797-2910 (AMO-adjacent, not fully traced).
- `casez({x_inst[31:25],x_inst[14:12]})` — decd.v:3187-3732 (large, ~545 lines; FP-format
  heavy, out of M2 scope).
- The vector table, decd.v:4172-6566, is dead per §3.1.

Each `case` arm sets: `decd_32_eu` (one-hot EU target, `EU_ALU`/`EU_BJU`/`EU_LSU`/`EU_CP0`/
etc., cfig.h:105-153), `decd_32_func` (20-bit `FUNC_WIDTH` code, cfig.h:302), and the
per-operand `_vld` bits (`decd_32_src0_vld`, `_src1_vld`, `_src1_imm_vld`, `_src2_vld`,
`_src2_imm_vld`, `_dst0_vld`) — i.e. decode directly produces which of {rs1 register, rs2
register, generated immediate} feeds each of three operand slots, and whether rd is written.
No `default` clears these between arms of the *same* case (decd.v:1554-1559: values init'd to
0 once at top of the `always` block, decd.v:1580-1591), so unhit fields stay 0/invalid.

### 3.3 Instruction "func" codes are flag-bearing, not opaque enums

`FUNC_WIDTH` = 20 bits (cfig.h:302). Individual bit *positions* within the 20-bit func code
are separately named and tested directly by control logic without a full func decode, e.g.
`FUNC_STORE_SEL`=0, `FUNC_NO_FENCE_SEL`=3, `FUNC_CONDBR_SEL`=6, `FUNC_AUIPC_SEL`=7
(cfig.h:310-313). `dp_ctrl_dis_inst_store` in `aq_idu_id_dp.v:522-523` is literally
`dp_id_inst_data[...+FUNC_STORE_SEL]` (bit 0 of the LSU-class func code), and
`dp_ctrl_inst_amo` in `dp.v:1132` reads bit 6 of the CP0-class... no, LSU-class func code
(shared bit position with `FUNC_CONDBR_SEL`=6, safe because ALU/BJU/LSU func codes are
mutually exclusive per instruction). `FUNC_ADD` itself is defined as a 19-bit literal
pattern (cfig.h:322), not a small sequential enum — confirms func codes are purpose-built
bit vectors, not opcode tags for the EU to re-decode.

### 3.4 Decoded-output bus shape

`decd_dp_inst_data` (265b, `IDU_WIDTH`=265, cfig.h:194) packs: func (20b), eu (10b one-hot,
§5.1), opcode (32b, raw or RVC-derived — see §3.5), vec-split-type (4b, always
`VEC_SPLIT_NON` since vector is dead), illegal (1b), length (1b, =1 for 32-bit), src1/src2
generated immediates (64b each) + their valid bits, vector dst/src register fields (dead),
dst0/dst1 register indices (6b each — 6 bits because vector register numbers need a bank bit;
integer register numbers occupy the low 5), src0/1/2 register indices (6b each), and a long
list of dst/src *_vld bits for scalar, FP, and vector operand classes. This is a genuinely
flat "control-signal bus" (no nested struct-of-structs in the RTL — everything is bit-range
`\`define`s into one flat vector), matching the C910 note's description of the general style.
Key M2-relevant fields: `IDU_EU` (EU one-hot target), `IDU_FUNC` (func code), `IDU_DST0_VLD`/
`IDU_SRC0_VLD`/`IDU_SRC1_VLD`/`IDU_SRC2_VLD` (operand-presence flags), `IDU_SRC1_IMM`/
`IDU_SRC2_IMM` (generated immediates), `IDU_ILLEGAL`.

### 3.5 RVC handled inline, not as a pre-expansion pass

`decd_length = (x_inst[1:0] == 2'b11)` (decd.v:448) — IDU **independently re-derives**
32-bit-vs-compressed from the raw instruction bits it receives (consistent with the M1 IFU
note: the front end detects RVC boundaries live too, rather than storing a predecode bit).
`x_decd_opcode = decd_length ? x_inst[31:0] : {16'b0, x_inst[15:0]}` (decd.v:450) — for a
16-bit instruction, the "opcode" field IDU forwards downstream is just the zero-extended
16-bit RVC encoding, not an expanded-to-32-bit equivalent. There *is* a small helper,
`aq_idu_expand_32.v` (72 lines), but it is a generic 5-bit-index→32-bit-one-hot expander used
by the GPR/WBT write-enable logic (§4.1), unrelated to RVC expansion. The compressed-format
`casez({x_inst[15:10], x_inst[6:5], x_inst[1:0]})` table (decd.v:1269-1505, "16 bits Full
Decoder") decodes RVC mnemonics directly into the same `EU`/`FUNC`/`*_vld` fields as the
32-bit table — RVC is a peer decode path into the same output bus, not a separate
expand-then-decode pass.

## 4. Immediate generation (`aq_idu_id_decd.v:461-640`)

Two independent generators, both decode-time-computed directly from the raw instruction bits
(no shared 5-format generic immediate module):

- **`src1_imm`**: a 14-way priority-free one-hot selector `decd_src1_imm_sel[13:0]`
  (decd.v:467-519) feeding a `case` (decd.v:543-561) that produces a sign/zero-extended
  64-bit value per selected format: U-type imm20 (LUI/AUIPC-adjacent, `sel[0]`), I-type
  imm12 (`sel[1]`), RVC imm6 (c.addi/c.li/c.lui-style, `sel[2]`, masked off for RVC branches
  via `decd_src1_imm_c_branch_mask`), RVC `c.addi16sp` (`sel[3]`), RVC `c.addi4spn`
  (`sel[4]`), S-type store imm12 (`sel[5]`), RVC `c.lwsp` (`sel[6]`), RVC `c.lw`/`c.sw`
  (`sel[7]`), RVC `c.swsp` (`sel[8]`), RVC `c.fld`/`c.fsd`/`c.ld`/`c.sd` (`sel[9]`), RVC
  `c.fldsp`/`c.ldsp` (`sel[10]`), RVC `c.fsdsp`/`c.sdsp` (`sel[11]`), a custom "lsi" shift-imm
  (`sel[12]`, opcode `0001011`), and vector `opivi` (`sel[13]`, dead per §3.1).
- **`src2_imm`**: a 7-way selector `decd_src2_imm_sel[6:0]` (decd.v:565-593) feeding a
  second `case` (decd.v:608-628): U-type imm20 again (AUIPC-position, `sel[0]`), I-type
  imm12 again (`sel[1]`, for the operand-2 position — e.g. JALR's target-offset consumer),
  RVC imm6 again (`sel[2]`), B-type branch imm (`sel[3]`), J-type JAL imm (`sel[4]`), RVC
  `c.branch` imm8 (`sel[5]`), RVC `c.j`/`c.jal` imm (`sel[6]`).
- A third, tiny **`src3` immediate** (2 bits, decd.v:635-639) exists only for the custom
  "lsr" opcode (`0001011`, funct3-adjacent bits) — a T-Head custom extension, not core RV;
  flagged as unclear/out-of-scope (§7).

Both selectors are computed from raw `x_inst` bit slices per format (e.g. B-type:
`{x_inst[31],x_inst[7],x_inst[30:25],x_inst[11:8],1'b0}` sign-extended, decd.v:598-599; J-type:
`{x_inst[19:12],x_inst[20],x_inst[30:21],1'b0}` sign-extended, decd.v:600-601) — i.e. the
canonical RV32/64 I/S/B/U/J bit-shuffle-and-sign-extend patterns, plus the RVC-specific
shuffles for each compressed immediate-bearing form. **The same imm12 format (U/I) appears
in both selectors** because which GPR-operand-position (src1 vs src2) receives the generated
immediate depends on the instruction (e.g. LUI's imm rides in the src1 slot, AUIPC's
imm20 rides in the src2 slot alongside PC in src1 — consistent with an ALU op shaped
`dst = src0/src1 + src2`-style operand plumbing, not verified in exact ALU-input wiring here
since that's IU's file, not IDU's).

## 5. Hazard/interlock scheme (`aq_idu_id_wbt.v` + `aq_idu_id_ctrl.v`) — the critical section

### 5.1 The scoreboard: `aq_idu_id_wbt.v`, a 32-entry busy-bit + producer-type + outstanding-count table

Not a CAM, not comparator-based tag matching — a **fixed 32-entry register file indexed
directly by 5-bit architectural register number**, read via five independent address-decoded
32-way muxes (src0/src1/src2/dst0/dst1, `aq_idu_id_wbt.v:809-1189`, one `case
(dp_wbt_srcN_reg[4:0]) ... 5'd0: ... 5'd31: ...` per port) — structurally identical in style
to the GPR read ports (§5.4). Entry 0 (x0) is hardwired always-valid
(`aq_idu_id_wbt.v:208: read_data_0 = {...,1'b1}` — x0 is never busy). Entries 1-31 are
instances of `aq_idu_id_wbt_entry.v` (167 lines each).

Each entry (`aq_idu_id_wbt_entry.v`) holds:
- `wb` (1b) — **not-busy** flag. Cleared (busy=1) on `create_en` (a new dispatched
  instruction targets this register, `entry.v:75,89-90`); set (busy=0, i.e. "ready") when the
  *last* outstanding producer writes back (`wb_en_x && cnt_is_1`, `entry.v:91-92`).
- `cnt[1:0]` (2b) — **outstanding-producer counter**: 0/1/2 encode "1/2/3 producers still
  pending" (comment, `entry.v:120-122`). Increments on `create_en` without simultaneous
  writeback, decrements on writeback without simultaneous create (`entry.v:139-151`). This is
  the **WAW-handling mechanism**: C906 allows *multiple* in-flight producers for the same
  architectural register rather than blocking dispatch outright, and tracks how many are
  still outstanding so downstream logic knows whether the "ready" transition is imminent.
- `inst_type[2:0]` — the **producer's type tag**, captured at `create_en` time from
  `dp_wbt_dst0_type`/`dst1_type` (`entry.v:104-114`). Values (cfig.h:168-172):
  `WB_INT_TYPE_OTHER=0`, `_ALU=1`, `_BJU=2`, `_MULT=3`, `_LSU=4`.

7-bit read-port layout (`WB_INT_WIDTH`=7, cfig.h:161-166): bit `[0]`=VLD (ready),
`[3:1]`=TYPE (producer type), `[5:4]`=CNT (outstanding count), `[6]`=WB_CNT2 (a "2nd producer
about to complete" flag, asserted when `wb_en_x && cnt_is_2`, entry.v:98).

Write (create) side: `aq_idu_id_wbt.v:775-801` — `dp_wbt_dst0_reg`/`dst1_reg` (5b each) are
expanded to 32-bit one-hot via `aq_idu_expand_32` instances (wbt.v:781-791), ANDed with
`dp_wbt_inst_dst{0,1}_vld && ctrl_wbt_dis_inst_vld && !ctrl_xx_dis_stall` (wbt.v:794-801) —
**creation only happens on a non-stalled dispatch**, matching the ID+DIS-is-one-stage finding
of §2. Write (writeback/clear) side: `wb_en[31:0] = dp_wbt_wb_vld[31:0]` (wbt.v:1195), a
32-bit one-hot vector built in `aq_idu_id_dp.v:593-617` from `rtu_idu_wb0_reg`/`wb1_reg`
(again via `aq_idu_expand_32`) ANDed with `rtu_idu_wb{0,1}_vld` — **so the WBT's clear/busy
mechanism is driven directly by RTU's 2 commit ports**, the same 2 ports that feed the GPR
write side (§5.4) and the EX1-stalled-instruction late-bypass (§4/§5.5).

### 5.2 RAW/WAW stall computation with producer-type-aware exceptions (`aq_idu_id_ctrl.v:398-536`)

Base rule, straightforward: `ctrl_dis_srcN_raw = srcN_vld && !wbt_srcN_info[WB_INT_VLD] &&
!except` (ctrl.v:411-419); `ctrl_dis_dstN_waw` symmetric (ctrl.v:494-499). What makes this
section dense is the **"except" clause per source/dest**, which is where C906 avoids
stalling even though the scoreboard says "busy":

- **src RAW exceptions** (`ctrl_dis_srcN_raw_except`, ctrl.v:431-484), any of:
  1. Producer type is ALU or BJU (ctrl.v:433-434) — these complete in exactly one EX1 cycle,
     so the consumer can dispatch speculatively and rely on a **downstream, same/adjacent
     -stage bypass inside IU itself** (see §4) rather than stalling at ID/DIS.
  2. Producer is LSU and consumer is a BJU conditional branch, with ≤1 producer left
     outstanding (ctrl.v:436-440) — load-to-branch is allowed through, presumably resolved
     by IU's own load-data bypass.
  3. `dp_ctrl_srcN_fwd_vld` is set (an RTU forward-bus hit this cycle, §4) **and** it's not
     the specific "2-outstanding LSU/MULT producer" corner case (ctrl.v:441-445,457-461,
     473-477) — i.e. if RTU is broadcasting the value *this cycle* via `rtu_idu_fwdN_*`, no
     need to also stall.
  4. (src2 only) producer is LSU and consumer is a store using src2 as store-data, gated on
     outstanding-count/WB_CNT2 (ctrl.v:478-484) — store-data-from-load forwarding case.
- **dst WAW exceptions** (`ctrl_dis_dstN_waw_except`, ctrl.v:508-536), any of:
  1. Old and new producer are **both** LSU-type or **both** MULT-type, with ≤1 outstanding
     or the older one mid-writeback (ctrl.v:512-518,527-533) — same-latency-class producers
     to the same register don't need to serialize dispatch, only their relative *completion*
     order (guaranteed by in-order LSU/MULT queues elsewhere) matters.
  2. Old producer is ALU or BJU (ctrl.v:520-521,535-536) — single-cycle producers can't
     create a WAW hazard against a not-yet-dispatched instruction in the first place.

**Net effect for M2**: this is not a simple "stall until scoreboard bit set" interlock. It's
"stall unless (a) an in-flight forward bus already carries the value this cycle, or (b) the
producer's completion latency class structurally guarantees correct same/adjacent-stage
ordering without a stall." Reproducing this in M2 requires knowing each functional unit's
completion latency class (ALU/BJU = 1 cycle fixed, LSU/MULT = variable, tracked via the
outstanding-count field) — a fact that must come from IU/LSU/RTU's own extraction, not IDU's.

### 5.3 Multi-beat instruction splitting (`aq_idu_id_split.v`) — dispatch is not always 1-shot

Four small FSMs, each re-decoding the *same* static fetched instruction across 2+ dispatch
beats while asserting `split_ctrl_id_stall` (which becomes `idu_ifu_id_stall` via
`ctrl_split_stall`, ctrl.v:367-369) to hold the front end:
- **`lsd`** ("load store double", decd.v §148 heading / split.v:148-219) — 2-state FSM
  (`LSD_IDLE`→`LSD_SPLIT`→idle). Exact instruction class not fully confirmed from files in
  scope — plausibly the custom "lsi"/"lsr" (opcode `0001011`) T-Head extension instructions
  seen in the immediate-select logic (§4), which appear to produce two destinations (dst0 +
  dst1, with `dp_wbt_dst1_type` hardwired to `WB_INT_TYPE_ALU`, "dst1 can only be alu from
  indexed load", `aq_idu_id_dp.v:571-572`) — **flagged as genuinely unclear**; not needed for
  a minimal RV64GC M2 scope but worth resolving before touching any T-Head custom opcode.
- **`amo`** (split.v:306-410+) — a longer FSM with states named `AMO_LR`/`AMO_SC`/`AMO_AMO`/
  `AMO_AQ` (split.v:395-407) — atomics are dispatched as a multi-beat sequence from IDU's
  perspective, not a single-shot LSU transaction.
- **`che`** (split.v:603+) — cache-maintenance instruction split (custom, gated by
  `cp0_idu_cskyee` per §3.1 class `[3]`).
- **`fnc`** (split.v:716+) — fence-class split (e.g. `sfence.vma`, flagged "deal in fence /
  split" at decd.v:2119-2120).

For M2: if the design doc's minimal scope excludes AMO/custom-extension instructions, this
whole module can likely be skipped; if AMO (`lr.w`/`sc.w`/`amoadd.w` etc.) is in scope, note
that C906 does **not** treat it as a single atomic LSU op from IDU's perspective — it's a
multi-cycle IDU-driven sequence.

### 5.4 GPR read/write (`aq_idu_id_gpr.v` + `aq_idu_id_gpr_gated_reg.v`)

Same structural pattern as the WBT: 31 flop-based entries (`aq_idu_id_gpr_gated_reg.v`, one
per non-x0 register, instantiated x31 in `aq_idu_id_gpr.v:117-387+`), each independently
clock-gated (`write_en = wb0_vld_x || wb1_vld_x`, gpr_gated_reg.v:79,85). Two write ports
(`rtu_idu_wb0_data`/`wb1_data`, muxed by `{wb1_vld_x,wb0_vld_x}` per entry,
gpr_gated_reg.v:88-97), matching RTU's 2 commit ports exactly. Reads are three independent
32-way address-decoded muxes (src0/src1/src2), architecturally identical to the WBT reads —
this is a plain small register file, not a CAM.

## 6. Forwarding/bypass network — sourced entirely from RTU, not EU/LSU directly

**Verified, not assumed:** every bypass path that feeds IDU's operand-read/forward mux comes
from `rtu_idu_fwd0/1/2_{data,reg,vld}` (3 generic 64-bit broadcast slots) and
`rtu_idu_wb0/1_{data,reg,vld}` (2 commit slots) — there is no `iu_idu_ex1_fwd_*` or
`lsu_idu_fwd_*` port anywhere in `aq_idu_top.v`'s port list. RTU is the single hub that both
broadcasts speculative-forward values and commits architectural state; IDU never sees EU1 or
LSU results directly.

- **ID/DIS-stage forward mux** (`aq_idu_id_dp.v:636-797`, one block per src0/src1/src2):
  `dp_fwd_srcN_sel[2:0]` is a 3-bit one-hot compare of the operand's register number against
  `rtu_idu_fwd{0,1,2}_reg` (gated by each `_vld`), and a `case` picks the matching data bus
  (dp.v:648-661 etc.). **No explicit priority is encoded** — the `case` only matches the
  three single-bit patterns `3'b001`/`3'b010`/`3'b100`; a simultaneous multi-hit falls to
  `default: {64{1'bx}}`. This means **the RTL relies on an upstream invariant** (enforced by
  RTU's own arbitration and/or the WAW-except rules of §5.2, which permit same-cycle
  in-flight duplicate producers only for latency classes that can't complete simultaneously)
  that no two of fwd0/1/2 ever target the same register in the same cycle. **Flagged for
  M2/RTU extraction to corroborate** — this note cannot confirm the guarantee from IDU's
  files alone.
  - src0 never takes an immediate (`dp_dis_int_inst_data[SRC0_DATA] = fwd_vld ? fwd_data :
    gpr_data`, dp.v:737-738); src1/src2 do (`!SRC1_VLD ? imm : (fwd_vld ? fwd : gpr)`,
    dp.v:754-760, and symmetric for src2, dp.v:783-789) — i.e. rs1 always occupies the src0
    slot; rs2-or-immediate occupies src1/src2.
  - Each `dp_dis_int_inst_data[SRCN_RDY]` bit is set on `wbt_vld || fwd_vld ||
    !srcN_vld` (dp.v:740-742,766-768,795-797) — **this "ready" bit is not merely "stall or
    don't"**: it rides downstream into EX1 (`idu_iu_ex1_src{0,1}_ready`, dp.v:1082-1083) so
    that IU's own execute-stage bypass network knows whether the src0/1 data IDU handed it is
    already final or still needs a same/adjacent-stage forward (this is exactly how the
    RAW-except case 5.2.1 above gets resolved — not inside IDU, but flagged downstream for IU
    to resolve).

- **EX1-stage late forward** (`aq_idu_id_dp.v:893-997`) — a *second*, separate bypass check
  for an instruction sitting stalled in the EX1 register waiting on an operand: if that
  operand's `RDY` bit was 0 when latched, and `rtu_idu_wb0/wb1` now matches that operand's
  register, the EX1 register's data field is **overwritten in place** with the wb data and
  its `RDY` bit flips to 1 (dp.v:917-996) — this only checks the 2 commit ports (`wb0`/`wb1`),
  not the 3 generic forward ports, since by the time an instruction is parked in EX1 waiting,
  only an actual writeback (not a same-cycle speculative forward) can resolve it.

## 7. Dispatch to functional units — real "dispatch" concept, single mutually-exclusive target/cycle

Yes, there is a genuine dispatch concept, not just a decode→execute handoff: `aq_idu_id_ctrl.v`
computes a single one-hot `ctrl_dis_inst_eu[9:0]` per cycle (`EU_WIDTH`=10, cfig.h:100) that
selects **exactly one** of ALU/BJU/MULT/DIV/CP0/LSU/FP/VEC as the instruction's destination
(ctrl.v:567-584 — with exceptions to force `EU_CP0` on exception/cancel, ctrl.v:573-574).
This one-hot tag rides into the single EX1 pipeline register (§2) and fans out combinationally
to per-EU "select"/"dp_sel"/"gateclk_sel" output ports (`idu_iu_ex1_alu_sel`,
`idu_lsu_ex1_dp_sel`, `idu_cp0_ex1_sel`, `idu_vidu_ex1_fp_sel`, etc., ctrl.v:627-663) — so
architecturally there is one shared "ID/EX1" boundary, but the boundary's *content* is
interpreted differently per consuming unit via field-slicing of the same latched bus
(`aq_idu_id_dp.v:1066-1132`), not a routed crossbar to N separate destination registers.

Critically, EX1 issue is **not automatic once latched** — each `*_sel` signal is additionally
gated by `!ctrl_ex1_internal_stall && rtu_idu_commit && !<EU>_idu_full` (ctrl.v:627-653). So
an instruction can sit **valid-but-not-issuing** in the EX1 register for one or more cycles
if RTU withholds commit or the target EU reports full — this is the "structural hazard"
resolution point, and it's downstream of the ID/DIS decode+dispatch decision, not upstream of
it. `ctrl_ex1_stall` (eu-full OR issue-stall OR internal-stall, ctrl.v:692) feeds back into
`ctrl_dis_stall` (ctrl.v:390-394), so a stuck EX1 instruction backpressures new dispatch too.

For M2's design-doc question of whether to model an explicit "dispatch" stage/struct: **yes**
— C906 genuinely has (a) a decode+hazard-check+EU-select combinational stage (ID/DIS, §2),
(b) a single EX1 pipeline register holding exactly one in-flight instruction's full operand
payload plus its EU tag, and (c) a *second*, EX1-resident issue-gate (commit + EU-full check)
that can hold that same instruction for extra cycles before it actually fires into the target
unit. A model that collapses (a)+(b)+(c) into one instantaneous decode→execute handoff would
miss the EX1-held/backpressure behavior in §"Critically" above.

## 8. CSR read/decode (`aq_idu_id_decd.v:2126-2163`, `aq_idu_id_dp.v:1125-1132`)

CSRRW/CSRRS/CSRRC and their `*I` immediate-rs1 variants decode inside the **main 32-bit
integer casez table** (§3.2, keyed on `{x_inst[31:25],x_inst[14:12],x_inst[6:2]}` — SYSTEM
major opcode `1110011`, funct3 selects the CSR sub-op) into `EU_CP0` with func codes
`FUNC_CSRRW`/`FUNC_CSRRS`/`FUNC_CSRRC`/`FUNC_CSRRWI`/`FUNC_CSRRSI`/`FUNC_CSRRCI`
(decd.v:2126-2163). Register-source variants (CSRRW/S/C) set `src0_vld=1` (rs1); all six set
`src1_imm_vld=1` and `dst0_vld=1`. **The CSR address rides in the shared I-type-imm12 slot**
(`decd_src1_imm_sel[1]`, §4) — the same immediate generator ADDI etc. use — sign-extended
into `x_decd_src1_imm`, then delivered as `idu_cp0_ex1_src1_data` at the EX1 boundary. `rd`'s
old-CSR-value destination is `dst0`, same as any other CP0-class producer (WBT entry marked
busy the same way as an ALU/LSU dest, §5.1). `dp_ctrl_inst_csr` is derived downstream as "is
this a CP0-class func code with its bit-1 flag set" (`aq_idu_id_dp.v:1125`,
`ex1_int_inst_data[...FUNC-FUNC_WIDTH+1]`).

**IDU does not own the CSR register file.** It only decodes the instruction and forwards
`{func, opcode, dst0_reg, src0_data(rs1 or none), src1_data(csr addr imm)}` to CP0 via
`idu_cp0_ex1_*` ports (`aq_idu_id_dp.v:1088-1100`) — architecturally identical treatment to
how IDU hands off to IU/LSU (§7). The actual CSR state and read-modify-write logic live in a
separate CP0 unit outside `idu/rtl` (consistent with M1's finding that the front end talks to
`cp0_ifu_*` — CP0 is a distinct top-level unit that IDU, like IFU, treats as a peer
"execute unit" target for dispatch purposes, not something it owns internally).

Also decoded in the same table into `EU_CP0` (not CSR, but CP0-class privileged/sync ops):
FENCE (`FUNC_FENCE`... — not shown verbatim but same table region), `fence.i`
(`FUNC_FENCEI`, decd.v:2101-2103), `ecall`/`ebreak` (`FUNC_ECALL`/`FUNC_EBREAK` selected by
`x_inst[20]`, decd.v:2106-2108), `dret`/`mret`/`sret`/`wfi` (decd.v:2111-2126). None of these
carry register operands (`decd_32_src0_vld`/etc. left at their 0 default, §3.2).

## 9. Read-directly spans (too intricate to fully resolve at this pass)

- `aq_idu_id_decd.v:2194-2772` — second I-type-keyed casez table (funct12+funct3); likely
  covers FP loads and/or misc SYSTEM sub-encodings not covered by the main table. Not needed
  for M2's base-integer/minimal-CSR scope, but if a decode gap turns up during M2
  implementation, check here.
- `aq_idu_id_decd.v:2797-2910` and `:3187-3732` — AMO-adjacent and FP-heavy decode tables;
  out of scope for M2, not traced.
- `aq_idu_id_split.v` `lsd` FSM target instruction class (§5.3) — plausibly the custom
  "lsi"/"lsr" T-Head opcode-`0001011` extension, not confirmed.

## 10. Open items / not independently verified

- Whether `rtu_idu_fwd0`/`fwd1`/`fwd2` ever *can* collide on the same destination register in
  the same cycle is asserted-by-construction from IDU's side (no priority encoding exists to
  handle it) but the actual guarantee must come from RTU's arbitration logic — needs
  corroboration from the RTU extraction note before M2 relies on "fwd0/1/2 are mutually
  exclusive" as a hard invariant.
- The exact mapping of `EU_ALU`/`EU_BJU`/`EU_MULT`/`EU_DIV`/`EU_LSU` completion latency (1
  cycle fixed for ALU/BJU per the RAW-except logic; variable, counted via WBT's 2-bit
  outstanding-count for LSU/MULT) is inferred from the *hazard-avoidance* logic in
  `aq_idu_id_ctrl.v`, not from IU/LSU's own pipeline structure — should be cross-checked
  against whatever IU/LSU extraction notes exist or get written for M2.
  <br>
- `WB_INT_TYPE` only enumerates OTHER/ALU/BJU/MULT/LSU (cfig.h:168-172, values 0-4 of a
  3-bit field) — DIV isn't a separate producer type here; not confirmed whether DIV results
  are tagged as MULT-type (shared completion-queue with MULT, plausible given IU/DIV are
  often fused) or something else. Flagged for IU/RTU extraction to confirm before modeling
  DIV completion/WAW behavior in M2.
