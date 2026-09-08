# M6 exploration note 2/3: donor C906 interrupt machinery

Source: Explore agent ab6487fefc4c445c6 (read-only, donor + rv12 cross-check),
2026-09-08. Root abbreviations (all absolute):
- **$F** = `.../rv906/refs/openc906/C906_RTL_FACTORY/gen_rtl`
- **$S** = `.../rv906/refs/openc906/smart_run`
- **$R12** = `.../rv12/rtl`

Factory layout: `C906_RTL_FACTORY/` (gen_rtl: biu, **clint**, clk, common,
cp0, cpu, dtu, filelists, fpga, idu, ifu, iu, lsu, mmu, **plic**, pmp, pmu,
rst, rtu, tdt, v*), `doc/` (datasheet + C906 user manual + integration
manual PDFs), `smart_run/` (logical SoC: ahb/apb/axi/bus/clk/common/gpio/
mem/tb/uart; tests), README, LICENSE. **The CLINT and PLIC are inside the
core wrapper `$F/cpu/rtl/openC906.v`**, not in smart_run.

## 1. Interrupt entry machinery (cp0): aq_cp0_trap_csr.v

**Pending-and-enabled set.** Each source computes `*_en = mie_bit_source &
mip_source` then a privilege/delegation split. `$F/cp0/rtl/
aq_cp0_trap_csr.v:1269-1304`:

```verilog
assign mhip_en = mhie & mhip;      // mhip/mhie are constant 0
assign moip_en = moie & moip;
assign mcip_en = mcie & mcip;      // mcie constant 0
assign meip_en = meie & meip;
assign mtip_en = mtie & mtip;
assign msip_en = msie & msip;
assign seip_en = seie & seip;
assign stip_en = stie & stip;
assign ssip_en = ssie & ssip;

// For MEI, MTI, MSI (NOT delegable):
assign meip_vld = (pm[1:0] != 2'b11 || mie_bit) && meip_en;
assign mtip_vld = (pm[1:0] != 2'b11 || mie_bit) && mtip_en;
assign msip_vld = (pm[1:0] != 2'b11 || mie_bit) && msip_en;
```

For the S trio (delegable), `:1310-1330` — the nodeleg/deleg pair (note
the S-Mode arm uses `sie_bit`, U-Mode is "global always on"):

```verilog
assign seip_nodeleg_vld = (pm[1:0] == 2'b11 && mie_bit
                          || pm[1:0] == 2'b01
                          || pm[1:0] == 2'b00)
                        && seip_en && !mideleg[9];
assign seip_deleg_vld   = (pm[1:0] == 2'b01 && sie_bit
                        || pm[1:0] == 2'b00)
                      && seip_en && mideleg[9];
// same shape for stip (mideleg[5]), ssip (mideleg[1]),
// and for mhip/moip/mcip with mideleg[18]/[17]/[16]
```

The result is a 15-bit vector, `:1332-1338`:

```verilog
assign int_sel[14:0] = {mcip_nodeleg_vld, mhip_nodeleg_vld,
                        meip_vld, msip_vld, mtip_vld,
                        seip_nodeleg_vld, ssip_nodeleg_vld, stip_nodeleg_vld,
                        moip_nodeleg_vld,
                        mcip_deleg_vld, mhip_deleg_vld,
                        seip_deleg_vld, ssip_deleg_vld, stip_deleg_vld,
                        moip_deleg_vld};
```

**Priority/ordering** lives in the RTU, not cp0:
`$F/rtu/rtl/aq_rtu_int.v:53-74` is a `casez` over
`int_vld_raw[14:0]` — highest bit first, so the winning order is:

**MCIP(16) > MHIP(18) > MEI(11) > MSI(3) > MTI(7) > SEI(9) > SSI(1) >
STI(5) > MOI(17) > (then the six deleg variants: MCIP, MHIP, SEI, SSI,
STI, MOI)**

Note the subtlety: every non-deleg (M-target) source outranks every
deleg (S-target) source, and MOI (HPM overflow, cause 17) is sandwiched
between the S nodeleg group and the deleg group.

**Target priv / trap-FSM branch.** There is no explicit "trap FSM" —
delegation is combinational at CSR-write/trap time.
`$F/cp0/rtl/aq_cp0_trap_csr.v:797-851`:

```verilog
// medeleg valid when cpu in s-mode and vector hit
assign medeleg_vld_dp = (pm[1] == 1'b0) && !rtu_yy_xx_expt_int
                 && |(vec_num[15:0] & edeleg[15:0]);
// mideleg valid when cpu in s-mode/u-mode and vector hit
assign mideleg_vld_dp = (pm[1] == 1'b0) && rtu_yy_xx_expt_int
                 && |(vec_num[18:0] & mideleg[18:0]);
assign mdeleg_vld_dp  = medeleg_vld_dp || mideleg_vld_dp;
```

(`vec_num` at `:772-794` is a one-hot decode of `rtu_yy_xx_expt_vec`;
note interrupt causes 1/3/5/7/9/11/16/17/18 map through the same
decoder.) The next priv is set by the pm FSM at `:605-616`:

```verilog
if(rtu_cp0_exit_debug)      pm_wdata[1:0] = dtu_cp0_dcsr_prv[1:0];
else if(iui_regs_inst_mret) pm_wdata[1:0] = mpp[1:0];
else if(iui_regs_inst_sret) pm_wdata[1:0] = {1'b0, sstatus_spp};
else if(!mdeleg_vld_dp)     pm_wdata[1:0] = 2'b11;   // -> M
else if(mdeleg_vld_dp)      pm_wdata[1:0] = 2'b01;   // -> S
```

**mcause bit 63 / mepc / MPP / MPIE / MIE** (`:1083-1107`,
`:635-727`) — all on `regs_flush_clk`, all branched on
`rtu_yy_xx_expt_vld && !mdeleg_vld_dp` (M bank) vs `&& mdeleg_vld_dp`
(S bank):

```verilog
// mcause: m_intr <= rtu_yy_xx_expt_int;  m_vector <= rtu_yy_xx_expt_vec[4:0];
assign mcause_value[63:0] = {m_intr, 58'b0, m_vector[4:0]};       // :1107
// mepc:  mepc_reg[62:0] <= rtu_cp0_epc[63:1];                     // :1044
// MPP:   mpp <= pm[1:0] on !mdeleg;  spp <= pm[0] on mdeleg;      // :639, :656
// MPIE:  mpie <= mie_bit;   SPIE: spie <= sie_bit;                // :674, :688
// MIE:   mie_bit <= 1'b0;   SIE:  sie_bit <= 1'b0;                // :704, :718
```

mret/sret restore (`:641-642`, `:706`, `:720`): `mpp<=2'b00` after
mret use, `mie_bit <= mpie`, `sie_bit <= spie`, and `mpie/spie <= 1'b1`
after use.

**Interrupt tval is forced 0** at the RTU:
`$F/rtu/rtl/aq_rtu_retire.v:541-542`
`else if (retire_int_inst) retire_trap_tval[63:0] = 64'b0;`

## 2. mip/mie sources (cp0 top)

mip assembly, `$F/cp0/rtl/aq_cp0_trap_csr.v:1230-1245` (verbatim):

```verilog
assign mhip = 1'b0;
assign moip = hpcp_cp0_int_vld;
// assign mcip = ecc_int_vld;
assign mcip = 1'b0;
assign meip = biu_cp0_me_int;
assign mtip = biu_cp0_mt_int;
assign msip = biu_cp0_ms_int;

assign seip = biu_cp0_se_int || seip_reg;
assign stip = biu_cp0_st_int && regs_clintee || stip_reg;
assign ssip = biu_cp0_ss_int && regs_clintee || ssip_reg;

assign mip_value[63:0] =  {45'b0, mhip, moip, mcip, 4'b0,
                                  meip, 1'b0, seip, 1'b0,
                                  mtip, 1'b0, stip, 1'b0,
                                  msip, 1'b0, ssip, 1'b0};
```

- **Pinned from outside**: `meip` (PLIC `plic_core0_me_int`),
  `mtip`/`msip` (CLINT `clint_core0_mt_int`/`ms_int`), plus the S pins
  `biu_cp0_{se,st,ss}_int` (CLINT `st_int`/`ss_int`, PLIC `se_int`).
  These are 2-flop re-synchronized from the APB clock domain in
  `$F/cpu/rtl/aq_sysio_kid.v:156-183` (`kid_int_clk`, enabled by
  `apb_clk_en`), then passed straight through
  `$F/cpu/rtl/aq_cpuio_top.v:89-96`.
- **Internal**: `moip` = PMU counter-overflow int,
  `$F/pmu/rtl/aq_hpcp_top.v:2674`
  `assign hpcp_cp0_int_vld = |(cntinten_value[31:0] & cntof_value[31:0]);`
  (mcntinten 0x7CA / mcntof 0x7CB). **T-Head platform interrupts exist:
  bit 16 mcip (ECC, tied 0), bit 17 moip (HPM overflow, live), bit 18
  mhip (PC-trace halt, tied 0)** — comments at `:807-809`.
- **S-writable flops** `seip_reg/stip_reg/ssip_reg` (`:1202-1228`):
  written by `mip` CSR writes (all three) or by `sip` writes (only
  `ssip_reg`, gated `ssip_acc_en = mideleg[1]`). `regs_clintee` is
  T-Head `mxstatus[17]`, **reset to 1**
  (`$F/cp0/rtl/aq_cp0_ext_csr.v:600`, `:611`) — it gates the CLINT
  S-mode pins (st/ss) but not the seip pin.
- `mie_value`/`sie_value`/`sip_value` at `:909-926`, `:1260-1264`;
  `sie` shows an S-level enable only where mideleg delegates it.

## 3. Interrupt latency/granularity: retire-time claim

Interrupts are taken **only at instruction-retire boundaries** (no
mid-pipe kill for interrupts):

- cp0 publishes `cp0_rtu_int_vld[14:0] = int_sel` and
  `cp0_rtu_trap_pc[39:0]` (`:1395-1396`).
- `$F/rtu/rtl/aq_rtu_int.v:79-81`:

```verilog
assign int_vld = |int_vld_raw[14:0]
                 && !dtu_rtu_int_mask        // debug single-step: dcsr_step && !dcsr_stepie ($F/dtu/rtl/aq_dtu_ctrl.v:376)
                 && !dp_int_ex2_inst_split;  // not on the first half of a split inst
```

- **Claim insertion** in `$F/rtu/rtl/aq_rtu_retire.v:470-471, 585-589`:

```verilog
assign retire_int_inst        = int_retire_int_vld;
assign retire_trap_vld = retire_ex2_retire_vld
                         && !halt_req && !dbg_mode_on
                         && (retire_expt_inst || retire_int_inst);
assign retire_trap_int = retire_int_inst && !retire_pending_bkpt_expt;
```

i.e. the "pending & enabled → redirect at next commit" term is exactly
`retire_ex2_retire_vld && retire_int_inst`. Trap priority at retire
(`:490-499`): pending-bkpt > **int** > async expt > sync expt.
- **Redirect target**: the trap FSM flushes FE
  (`:905-916`, `retire_inst_flush_fe_set` includes `retire_int_inst`)
  and the changeflow PC is flopped: `:993-1001, 1026-1038`:

```verilog
else if(retire_trap_chgflw_vld)
    retire_chgflw_pc[39:0] = cp0_rtu_trap_pc[39:0];
...
assign rtu_ifu_chgflw_pc[39:0] = retire_chgflw_pc[39:0];
```

- **Vectored target computation in cp0**,
  `$F/cp0/rtl/aq_cp0_trap_csr.v:1346-1359`:

```verilog
assign regs_tvec[39:0]  = pm[1:0] == 2'b11 ? mtvec_value[39:0] : stvec_value[39:0];
assign regs_vector[4:0] = pm[1:0] == 2'b11 ? m_vector[4:0]    : s_vector[4:0];
assign regs_intr        = pm[1:0] == 2'b11 ? m_intr            : s_intr;
assign vec_int_pc[39:0] = {regs_tvec[39:2], 2'b0}
                        + {33'b0, regs_vector[4:0], 2'b0};
assign regs_trap_pc[39:0] = regs_intr && regs_tvec[0] ? vec_int_pc[39:0]
                                                      : {regs_tvec[39:2], 2'b0};
```

(The tvec select uses the *current* pm; m_vector/s_vector are the
causes captured for THIS trap.) EPC for an int-only trap is the **next**
pc (`:571-575`: sync-expt/split-async use `cur_pc`, else
`dp_retire_ex2_next_pc`).

## 4. WFI

WFI is a real hardware halt with a **3-state low-power FSM**,
`$F/cp0/rtl/aq_cp0_lpmd.v:107-155` (`IDLE → WAIT → LPMD`). Decode:
`$F/idu/rtl/aq_idu_id_decd.v:2114-2116` (SYSTEM major-op); cp0 EX1:
`$F/cp0/rtl/aq_cp0_iui.v:530-531, 785` (`iui_inst_wfi`,
`iui_special_wfi`). Illegal in S-mode under `mstatus.TW` and always in
U-mode (`:574-584`).

Mechanics: `lpmd_stall` (`:174-176`) stalls the cp0 pipe via
`special_iui_stall` (`$F/cp0/rtl/aq_cp0_special.v:270-271`) →
`iui_inst_issue_stall` → `cp0_idu_issue_stall` (so the wfi instruction
itself does not complete); the FSM requests BIU/IFU/MMU quiesce
(`lpmd_ack = ifu_no_op && lsu_sync_ack && mmu_no_op`, `:169-172`),
then in LPMD **gates the whole core clock**:
`cp0_yy_clk_en = lpmd_b[1] & lpmd_b[0]` (`:217`) and asserts
`cp0_biu_lpmd_b = 2'b00` to the outside.

**Wake** (`:186-202`):
`(dtu_cp0_wake_up || regs_lpmd_int_vld) && cpu_in_lpmd` (or entering
debug). Two wake sources:
- `regs_lpmd_int_vld = lpmd_ack_vld` where
  `lpmd_ack_vld = meip_en || mtip_en || msip_en || moip_en || mcip_en
  || seip_en || stip_en || ssip_en`
  (`aq_cp0_trap_csr.v:1340-1341`) — **mip & mie ONLY, no MIE/SIE/
  privilege gate** (spec-conformant: WFI resumes on pending-and-enabled
  even when globally masked). This term is combinational and lives off
  the forever-clock domain, so it stays live while the core clock is
  gated.
- `dtu_cp0_wake_up` = `$F/dtu/rtl/aq_dtu_ctrl.v:623`
  `dtu_rtu_sync_halt_req || async_halt_req_wakeup || dcsr_step ||
  pending_halt`.

After wake the wfi retires, the LPMD FSM returns to IDLE on
`rtu_yy_xx_flush` (`:127-128`), and the already-asserted interrupt is
taken at that retire boundary. The shipped smoke test ends with a bare
`wfi` (`$S/tests/cases/interrupt/C906_plic_int_smoke.s:171`) waiting
for a PLIC MEI.

## 5. mtime/mtimecmp — CLINT ships ($F/clint/rtl/clint_func.v)

The factory **does ship a CLINT**, instantiated inside the core wrapper
`$F/cpu/rtl/openC906.v:887-905` (`clint_top x_clint_top`). Register
file (`clint_func.v:118-144`, decode `:217-237`, APB pprot-based priv
check `:236-237`):

| Offset (within CLINT) | Register | Access |
|---|---|---|
| 0x0000 | `msip0` (bit 0) | M-mode write only (`mreg_wen`) |
| 0x4000/0x4004 | `mtimecmp0` lo/hi (32-bit halves) | M-mode |
| 0xC000 | `ssip0` (bit 0) | S/M-mode (`sreg_wen`) |
| 0xD000/0xD004 | `stimecmp0` lo/hi | S/M-mode |

(MSIP1-3/MTIMECMP1-3/SSIP1-3/STIMECMP1-3 constants exist for 4 harts;
only core0 implemented.)

- **mtime is NOT inside the CLINT.** The comparator input
  `sysio_clint_mtime[63:0]` is a *sampled external system counter*:
  `$F/cpu/rtl/aq_sysio_top.v:143-151` — `ccvr` flops
  `pad_cpu_sys_cnt[63:0]` every CPU clk;
  `assign sysio_clint_mtime[63:0] = ccvr[63:0];`. In the smart_run SoC
  the tick source is a plain CPU-clock incrementor:
  `$S/logical/common/tr_axi_interconnect.v:952-957`
  (`pad_cpu_sys_cnt <= pad_cpu_sys_cnt + 1`).
- **Comparator → mtip** (`clint_func.v:369-385`):

```verilog
always@(posedge mtime_clk ...) clint_mtime_reg[63:0] <= sysio_clint_mtime[63:0];  // re-sample into apb clk
assign clint_core0_ms_int = msip0_reg;
assign clint_core0_ss_int = ssip0_reg;
assign clint_core0_mt_int = !({mtimecmph0_reg[31:0], mtimecmp0_reg[31:0]}
                          > clint_mtime_reg[63:0]);      // i.e. mtime >= mtimecmp
assign clint_core0_st_int = !({stimecmph0_reg[31:0], stimecmp0_reg[31:0]}
                          > clint_mtime_reg[63:0]);
assign clint_core0_time[63:0] = sysio_clint_mtime[63:0];
```

- CLINT is reached through the core's internal APB:
  `$F/biu/rtl/aq_biu_apbif.v:285-292` —
  `clint_hit = (apbif_addr[26:16] == 11'h400)`,
  `plic_hit = (apbif_addr[26] == 0)`; APB base =
  `pad_cpu_apb_base[39:27]` (`aq_sysio_top.v:209-218`), in smart_run
  `pad_cpu_apb_base = 40'h4000000000`
  (`tr_axi_interconnect.v:966`). So **CLINT phys = 0x4000400000, PLIC
  phys = 0x4000000000** (the smoke test's `PLICBASE_M,0x4000000000`
  confirms).

## 6. PLIC — ships ($F/plic/)

`plic_top` instantiated at `$F/cpu/rtl/openC906.v:916-939` with
`INT_NUM = PLIC_INT_NUM+16 = 256`, `PRIO_BIT=5`, `ID_NUM=10`,
`HART_NUM=1` for PROCESSOR_0 (`$F/cpu/rtl/cpu_cfig.h:195-199, 425-429`).
`openC906.v:951-953`:

```verilog
assign plic_int_vld[`PLIC_INT_NUM+15:0] = {pad_plic_int_vld[`PLIC_INT_NUM-1:0],14'b0,l2c_plic_ecc_int_vld,1'b0};
```

so external pad source *i* is PLIC ID *i+16*; ID 0 dead, ID 1 = L2 ECC
(tied 0 here), IDs 2-15 reserved.

- **Register map** (`plic_top.v:418-429` base addresses; per-context
  decode `plic_hreg_busif.v:472-533`): priority array at +0x0 (4
  B/source, 5 bits), pending/setip at +0x1000, enable at +0x2000 (M
  bank; **S bank at +0x2080**, from the smoke test writing both
  `0x0` and `0x80` offsets), and per-hart context at +0x200000 +
  hart*0x2000, split by `paddr[12]`: 0 = M-mode {threshold @0x0,
  claim/complete @0x4}, 1 = S-mode {sthreshold @0x0, sclaim @0x4}.
- **Per-source state** (`$F/plic/rtl/plic_int_kid.v:103-159`): 2-flop
  input sync (`plic_kid_busif.v:175-181`), pending set on edge
  (`int_cfg=1` → pulse) or level (with complete semantics), cleared by
  claim; a separate **active** bit masks a claimed-but-not-completed
  source:
  `kid_arb_int_req_x = int_pending && !int_active && (int_priority != 0)`.
- **Claim/complete** (`plic_hreg_busif.v:627-655, 796-812`): a
  background arbiter (started on new int, priority/ie/ict writes, or
  claim/complete — `arb_start_en` at `:770-776`) continuously maintains
  `hart_mclaim_flop`/`hart_sclaim_flop` = current winner. **Reading
  claim returns that ID and clears the pending** of the ID if it is
  enabled in the matching bank (`hreg_kid_claim_vld`, `:813-815`);
  **writing claim = completion** (`hreg_kid_cmplt_vld` with written ID,
  clears `int_active`).
- **meip/seip generation** (`$F/plic/rtl/plic_arb_ctrl.v:239-308`):
  4-state arb FSM (IDLE/ARBTRATE/ARB_DELAY/WRITE_CLAIM) over a
  1024-entry priority tree (`plic_32to1_arb`/`plic_granu_arb`); the
  MSB of the per-source prio word is the M/S bank flag:

```verilog
assign arbx_core_mint_req_en = ... && arb_ctrl_int_prio[PRIO_BIT]      // M bank
        && (arb_ctrl_int_prio[PRIO_BIT-1:0] > hreg_arbx_prio_mth);    // > M threshold
assign arbx_core_sint_req_en = ... && !arb_ctrl_int_prio[PRIO_BIT]     // S bank
        && (arb_ctrl_int_prio[PRIO_BIT-1:0] > hreg_arbx_prio_sth);    // > S threshold
assign arbx_hartx_mint_req = mint_out_req;   // -> plic_core0_me_int -> meip
assign arbx_hartx_sint_req = sint_out_req;   // -> plic_core0_se_int -> seip
```

`mint_req` clears on M-claim or when priority drops to/below the
(possibly just rewritten) threshold.
- Outputs: `openC906.v:954-955`
  `plic_core0_me_int = plic_hartx_mint_req[0]; plic_core0_se_int =
  plic_hartx_sint_req[0];` → `aq_sysio_kid` 2-flop sync →
  `biu_cp0_me_int`/`biu_cp0_se_int`.

## 7. Vectored vs direct tvec — YES, vectored is supported

`mtvec_mode[1:0]` is stored (`aq_cp0_trap_csr.v:936-944`) but **only
bit 0 is architecturally visible**:
`assign mtvec_value[63:0] = {mtvec_base[61:0], 1'b0, mtvec_mode[0]};`
(`:956`; stvec identical at `:987`). The redirect equation
(`:1358-1359`, quoted in §3) uses `regs_intr && regs_tvec[0] ? base +
4*cause : base` — so MODE=1 gives `base + 4*vector` for **interrupts
only**; exceptions always go to plain base.

## 8. mideleg — delegable bits

`aq_cp0_trap_csr.v:815-845`: writable bits are `ssie_deleg = wdata[1]`,
`stie_deleg = wdata[5]`, `seie_deleg = wdata[9]`, and the T-Head extra
`moie_deleg = wdata[17]` (HPM-overflow delegation). `mhie_deleg`/
`mcie_deleg` are constants 0 (sources don't exist):

```verilog
assign mideleg[63:0] = {45'b0, mhie_deleg, moie_deleg, mcie_deleg,
                        6'b0, seie_deleg, 1'b0,
                        2'b0, stie_deleg, 1'b0,
                        2'b0, ssie_deleg, 1'b0};
```

(The donor also hardwires medeleg bits 14, 11, 10 to 0:
`edeleg_upd_val` at `:755-757`.)

## 9. time CSR (0xC01)

`TIME` reads the **same live system counter that feeds the CLINT
comparator** — a read-only mirror, not a separate register. Path:
`pad_cpu_sys_cnt` → `sysio_clint_mtime` (aq_sysio_top.v:151) →
`clint_core0_time` (clint_func.v:387) → `clint_cpuio_time` →
`biu_hpcp_time` (`aq_cpuio_top.v:96`
`assign biu_hpcp_time[63:0] = clint_cpuio_time[63:0];`) → read mux
`$F/pmu/rtl/aq_hpcp_top.v:2655`:

```verilog
TIME       : data_out[63:0] = biu_hpcp_time[63:0];
```

Address map: `TIME = 12'hC01` (`aq_cp0_regs.v:783`); **there is NO
`mtime` CSR alias at 0xB04 — 0xB04 is MHPMCNT4**
(`aq_cp0_regs.v:680`). `time` access is U-counter-gated
(`regs_imm_inv = regs_ucnt_inv`, `:1139`, i.e. mcounteren/scounteren
logic) and has no write path.

## 10. Linux boot support in the factory — NONE

The public factory is **core + verification SoC only**; it does not
ship Linux boot content:

- Top level of `refs/openc906/`: only `C906_RTL_FACTORY/`, `doc/`
  (openc906 datasheet.pdf, 玄铁C906用户手册, 玄铁C906集成手册 —
  hardware manuals), `smart_run/`, `README.md`, `LICENSE`. No board/,
  no opensbi/, no dts/.
- Searches for `linux|Linux|opensbi|OpenSBI|u-boot|bootrom|firmware|
  dtb|fdt` over `README.md`, `smart_run/` (Makefile, setup, tests)
  return **zero hits** (only `tests/` "include the test suit, linker
  file, boot code" in README — meaning `tests/lib/crt0.s` bare-metal
  crt0).
- `smart_run` is a **simulation SoC**: `$S/logical/tb/tb.v` loads
  `inst.pat`/`data.pat` via `$readmemh` into a 128-bit AXI SRAM (64
  MB), the console is **tb-side snooping of AXI writes to
  0x10015000** (`tb.v:284-309`, printing `biu_pad_wdata` bytes) — i.e.
  the APB UART at 0x10015000 (`$S/logical/apb/apb_bridge.v:34-35` PS1)
  with an actual `uart.v` in `$S/logical/uart/`, but the report path is
  the tb sniff.
- **Interrupt sources wired to the PLIC** (40 bits):
  `$S/logical/apb/apb.v:335`
  `assign xx_intc_vld[39:0] = {21'b0,stim_intc_int[3:0],gpio_intc_int[7:0],1'b0,tim_intc_int[3:0],1'b0,uart0_intc_int};`
  → `$S/logical/common/tr_axi_interconnect.v:960`
  `assign pad_plic_int_vld = {{240-40{1'b0}}, xx_intc_int[39:0]};` with
  `pad_cpu_sys_cnt` incremented per CPU clock (`:952-957`). Mapping:
  pad bit 0 = uart0 → PLIC ID 16; timers 18-21; gpio 23-30; stimer
  31-34.
- The only interrupt test:
  `$S/tests/cases/interrupt/C906_plic_int_smoke.s` — software-sets PLIC
  source-1 pending (write 0x2 to +0x1000), sets its priority (+0x4),
  enables it (M bank +0x2000 and +0x2080), clears M-threshold
  (+0x200000), enables `mstatus.MIE`+`mie.MEIE`, `wfi`, then in the
  MEI handler claims (+0x200004), completes (write-back), `mret`.
- No boot ROM: reset vector comes from `pad_cpu_rvba` latched at reset
  (`aq_sysio_top.v:181-185`), tied 0 in smart_run.

## 11. rv12 cross-check (brief)

rv12 (C910-clone, M4-done) cloned the donor's cp0 interrupt network
**byte-for-byte** but its pipeline/SoC side diverges in what is live:

- **Live**: `$R12/CSR.v:2212-2290` has the full 15-term `int_sel`
  (same order as donor `:1332`), the same `casez` priority (causes
  named in `$R12/rvproc_pkg.sv:1526-1541`, comment: "the donor's cause
  numbers at ct_cp0_iui.v:1594-1611"), a **registered, ACTIVE-LOW**
  `cp0_rtu_xx_int_b` + registered `cp0_rtu_xx_vec` (`:2300-2321`), the
  full mideleg storage + `mdeleg_vld` recompute at delivery. CLINT.v
  (`msip@0x0, mtimecmp@0x4000, mtime@0xBFF8`, `mtip = mtime >=
  mtimecmp`, mtime incremented by `rtc_tick`) and PLIC.v
  (`threshold@0x200000, claim@0x200004, meip = max_prio > threshold &&
  max_id != 0`) are wired at `$R12/RVProcAXI.v:924-926` through
  `$R12/RVProc.v:1939-1941` (`biu_cp0_me_int(meip)` etc.). RTU side:
  `$R12/RTU.v:2028` `rob_read0_int_vld = !cp0_rtu_xx_int_b &&
  !had_rtu_xx_tme && !rob_commit2;` (blocked when 3 instructions
  commit, with the donor's own un-committed-store rationale quoted at
  `:2019-2027`), and `rtu_cp0_int_ack` at `:3810`.
- **Deferred/diverged**: (a) no S-mode CLINT banks and no
  `biu_cp0_{se,st,ss}_int` pins — S pending bits exist only as CSR
  flops (`CSR.v:1752-1799` documents: "rv12's frozen port list has no
  ... pin, so where the donor ORs pin and flop rv12 has the flop
  alone"); (b) WFI is a flushing **no-op** — the lpmd FSM was
  deliberately dropped (`CSR.v:148-152`), TW-trap kept; (c) rv12's
  CLINT keeps mtime **inside** the CLINT and writable, vs donor's
  external read-only system counter; (d) rv12's PLIC has a single
  context (no S threshold/claim, no M/S enable banks); (e) rv12 notes
  `rtu_cp0_int_ack` is declared-but-never-read in the donor
  (`CSR.v:2320-2325`).

# DONOR SCHEME SUMMARY (what rv906 must build for M6)

1. **mip assembly (CSR.v)**: keep the M4 shape and add pin sources:
   `msip/mtip` from CLINT, `meip` from PLIC, all read-only wires;
   `seip = seip_pin || seip_reg`, `stip/ssip = (pin & clintee) || reg`
   (rv906 may drop the T-Head `clintee` gate or hardwire it 1);
   S-writable flops via mip/sip writes (only SSIP via sip). Bits
   16/17/18 (mcip/moip/mhip): tie 0 or implement moip if a PMU
   overflow int is wanted. Compute `int_sel[14:0]` exactly as donor
   `:1332` with the nodeleg/deleg pairs; export
   `cp0_rtu_int_vld[14:0]` plus `cp0_rtu_trap_pc` (already have tvec/
   deleg machinery from M4).
2. **Int-claim insertion (RTU)**: fill the INTERRUPT placeholder at
   the retire priority chain with donor semantics: `int_vld =
   |int_sel && !dbg_step_mask && !split_first_half`; trap priority
   pending-bkpt > int > async > sync; `trap_vld = retire_vld && !halt
   && !dbg && (expt || int)`; EPC = next PC; tval = 0; cause =
   priority-casez winner (order MCIP>MHIP>MEI>MSI>MTI>SEI>SSI>STI>MOI,
   nodeleg before deleg); redirect PC = `pm==M ? mtvec : stvec`,
   vectored when `tvec[0]`: `base + 4*cause` (interrupts only);
   MPP/MPIE/MIE and SPP/SPIE/SIE updates keyed on `mdeleg_vld` as M4
   already does.
3. **mtime (CLINT)**: build an APB/Bus CLINT — `msip@0x0` (W1, bit0),
   `mtimecmp lo/hi@0x4000/0x4004` (M-only), optionally S-mode
   `ssip@0xC000`, `stimecmp@0xD000/0xD004`; keep a **64-bit
   free-running mtime** (donor keeps the counter outside the CLINT and
   only samples it — rv906 can follow rv12 and put a plain 64-bit
   incrementer inside, which is simpler and equivalent for single-
   core); `mtip = mtime >= mtimecmp`; route `clint_mtime` out to the
   `time` CSR read mux (`0xC01` returns the live counter, read-only;
   no 0xB04 alias).
4. **PLIC interface**: instantiate or wrap a PLIC on the SoC bus
   (donor's is inside `openC906`, at APB offset 0 with CLINT at
   +0x400000; rv12 uses standard 0x0C000000). Minimum viable clone:
   per-source pending+priority(5b)+enable, M/S enable banks, per-hart
   M and S context {threshold, claim/complete at +0x4},
   `meip = (max_prio > mth)`, `seip = (max_prio > sth)`; claim = read
   (returns winner ID, clears pending), complete = write ID (clears
   active). Wire `meip→mip[11]`, `seip→mip[9]` pins into the core,
   2-flop synchronized if crossing clock domains.
5. **WFI**: implement as a pipeline stall + wake, not necessarily full
   clock gating: decode wfi in IDU/cp0; trap Illegal under S+TW /
   U-mode; stall the wfi at cp0 until `wake = (mip & mie) != 0 ||
   debug_wake` — note the donor's wake term deliberately omits the
   global MIE/SIE/privilege gates; if rv906 gates the clock, the wake
   comparator must live in the always-on domain.

# DONOR ABSENCE LIST (rv906 must source elsewhere)

- **Linux/boot content: absent.** No OpenSBI, no DTB, no boot ROM, no
  kernel, no Linux mention anywhere in the factory (only bare-metal
  crt0.s + linker script in `smart_run/tests/lib/`). For a Linux
  bring-up rv906 needs OpenSBI + DTB from the OpenSBI/riscv-opensbi
  tree, plus a memory map the C906 integration manual (doc/玄铁C906
  集成手册) can inform.
- **SoC outside the core: minimal.** The smart_run SoC is
  simulation-only: an AXI SRAM, AHB/APB fabric, APB UART (console via
  tb snooping 0x10015000), APB timers/GPIO as PLIC sources, JTAG DTM.
  No DMA, no interrupts beyond uart/timer/gpio, no clock/power
  management, no board files. rv906's M6 PLIC source set must come
  from our own SoC plan (rv12's PLIC with N sources is a good
  template).
- **mtime tick source: external by design.** The donor core takes
  `pad_cpu_sys_cnt` as a pad input; the actual counter is SoC
  property (in smart_run just a CPU-clock incrementor). rv906 must
  decide its own tick (CPU clock or a slower RTC domain — rv12 used
  `rtc_tick`).
- **No Sstc/`stimecmp` CSR** — the S-timer exists only as CLINT MMIO
  (`ssip/stimecmp` banks), with T-Head `mxstatus.CLINTEE` gating, not
  the standard `stimecmp` CSR (Sstc extension).
- **No vectored-mode tvec[1] semantics** (only mode bit 0), **no
  `mhpmevent`-style delegation of bit 16/18** (mcip/mhip tied 0), **no
  CLINT S-mode pprot equivalent in rv12** — these are donor facts, not
  gaps, but rv906 should mirror them rather than invent.
- **ECC (mcip) and trace-halt (mhip) interrupt sources**: tied off in
  the donor; do not build producers.
- **PLIC security extension (`PLIC_SEC`)**: compiled out by default
  (`cpu_cfig.h` has no define); ignore.
