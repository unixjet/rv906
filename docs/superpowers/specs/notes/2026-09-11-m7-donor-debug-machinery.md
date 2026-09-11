# M7 exploration 2 — donor C906 debug architecture (M7 spec extraction)

Agent report 2026-09-11. Donor root: `refs/openc906/C906_RTL_FACTORY/gen_rtl/`
(`$G` below). All line numbers into the named file.

**Top-level architecture (critical structural fact):** the "TDT" debug
subsystem is split into two chip-level modules:
- `tdt_dmi_top` = JTAG DTM + DMI-to-APB bridge. **Not instantiated inside the
  CPU** — chip-level, next to `openC906` (reference TB
  `smart_run/logical/common/tr_axi_interconnect.v:919`), takes the raw JTAG
  pads and produces an APB master bus.
- `tdt_top` = Debug Module (DM) + SBA AXI master, instantiated inside
  `openC906` (`$G/cpu/rtl/openC906.v:799`) as an APB **slave** on the
  8-signal `tdt_dmi_p*` bus driven by `tdt_dmi_top`.
- `aq_dtu_top` = core-side debug unit, instantiated inside `aq_top`
  (`$G/cpu/rtl/aq_top.v:887`), wired to `tdt_top` at `openC906.v:511-516,
  577-586`.

Chain: JTAG pads → DTM (TAP + TDRs, `tck`) → per-op APB transaction
(`sys_apb_clk`) → DM APB slave (`sys_apb_clk`, gated by `dmactive`) → hart
interface (CDC to `forever_cpuclk` via `aq_dtu_cdc`) → DTU → RTU/pipeline.

C906 config macros (`$G/cpu/rtl/cpu_cfig.h` unless noted):
- `TDT_DM_SBA` + `TDT_DM_SBA_AXI`; `TDT_DM_SBA_DW_128` → DW=128, BW=16
  (cpu_cfig.h:463, tdt_define.h:142-153); `TDT_DM_SBAW = PA_WIDTH = 40`.
- `TDT_DM_PB_EN` + `TDT_DM_IEBREAK` + `TDT_DM_PB_SIZE=4` (cpu_cfig.h:228-236).
- `TDT_DM_CORE_RV64` → CORE_MAX_XLEN=64, CORE_ISA=4'h0 (cpu_cfig.h:453).
- `TDT_DM_CORE_NSCRATCH=4'h2` (cpu_cfig.h:455).
- Single core: `TDT_DM_CORE_NUM=1`, `TDT_DM_SINGLE_CORE` (tdt_define.h:16-99).
- `TDT_ACC_CSR` defined (tdt_define.h:140) — abstract CSR access enabled.
- `TDT_TM_MCONTROL_TRI_NUM=8`, `TDT_TM_OTHER_TRI_NUM=2` → 10 triggers
  (cpu_cfig.h:261-265, 476-496). `TDT_DEBUG_PCFIFO` + 16-entry (:270-276).
- `TDT_DMI_SLAVE_0` only → SLAVE_NUM=1, SINGLE_SLAVE, HIGH_ADDR_W=0;
  `TDT_DMI_SYSAPB_EN` NOT defined; `TDT_DMI_IDLE_CYCLE=3'h7`.

## 1. JTAG/DTM (tdt_dtm_top.v + children)

- Ports: JTAG `pad_dtm_tclk, pad_dtm_trst_b, pad_dtm_jtag2_sel,
  pad_dtm_tap_en, pad_dtm_tdi, pad_dtm_tms_i, dtm_pad_tdo, dtm_pad_tdo_en,
  dtm_pad_tms_o, dtm_pad_tms_oe`; DMI request `dtm_apbm_wr_vld,
  dtm_apbm_wr_addr[DTM_ABITS-1:0], dtm_apbm_wr_flg[1:0],
  dtm_apbm_wdata[31:0], dmihardreset`; response `apbm_dtm_rdata[31:0],
  apbm_dtm_wr_ready`. DTM_ABITS default 16, overridden to
  `TDT_DMI_HIGH_ADDR_W+10 = 10` by tdt_dmi.v:140.
- `tdt_dtm_ctrl.v`: TWO TAPs selected by `pad_dtm_jtag2_sel`:
  - **TAP5** standard 16-state IEEE 1149.1 (lines 93-208; TAP5_RESET=0000,
    IDLE=0001, ... PAUSE_IR=1101, EXIT2_DR=1110). TDO enable registered on
    negedge tclk during SHIFT_DR/IR (:223-234).
  - **TAP2** T-Head custom 2-wire (bidirectional TMS) protocol FSM
    (:239-343), RESET after 80 tck of TMS=1 (:264-279).
- TDRs (`tdt_dtm_chain.v`): IR opcodes `IDCODE=5'h01, DMI_ACC=5'h02,
  DTMCS=5'h10, DMI=5'h11`; 5-bit IR. DR widths: IDCODE/DTMCS=32b;
  DMI=`DTM_ABITS+2+32`=44 bits (C906: 10+34); in `idr_dmi_mode` the DMI DR
  shifts only 34 bits (data+op, address frozen); DMI_ACC and all other IRs =
  1-bit bypass. Shift LSB-first posedge tck; TDO negedge tclk, idle 1.
- **JTAG ID code** (`tdt_dtm_idr.v:45`): `{4'h1, 16'h0, 12'b1011_011_0111_1}`
  = **32'h1000_0B6F** (version 1, partnum 0, T-Head JEP106 0xB6F).
- **DTMCS** (`tdt_dtm_idr.v:46,81-84,204`): `version[3:0]=4'h1` (spec 0.13),
  `abits[9:4]=10`, `dmistat[11:10]`, `idle[14:12]=3'h7`; bit17 dmihardreset,
  bit16 dmireset.
- **DMI DR** = `{address[DTM_ABITS-1:0], data[31:0], op[1:0]}`
  (`tdt_dtm_idr.v:115-143`); request fires on UPDATE_DR when op==01(read)||
  op==10(write) and op_stat==0 and not in-flight; `op_stat` = 2'b11 busy if a
  new op is scanned while a request is running.
- **DTM→DMI protocol**: `dtm_apbm_wr_vld` (1-tck pulse), `dtm_apbm_wr_addr`
  = WORD OFFSET (spec DMI address), `dtm_apbm_wr_flg[1:0]` op, `wdata[31:0]`;
  response `apbm_dtm_wr_ready` (1-tck pulse) + `rdata` held; in-flight
  stability contract documented by assertions (:218-243).

## 2. DMI layer — DMI→APB bridge (confirmed)

`tdt_dmi.v` (:139-415):
1. `tdt_dtm_top #(.DTM_ABITS(10))` (:139-160).
2. `tdt_apb_master` (:166-194): each DTM request pulse → APB transaction.
   FSM IDLE/APB_SETUP/APB_ACCESS (:103-162); `paddr = {addr, 2'b0}` (byte
   address, :208); `pwrite = wr_flg[1]`; `trans_req = cmd_vld_sync_dly &
   (wr_flg[0]^wr_flg[1]) & addr_is_legal` (:128); completion `pready&penable`
   (:216) delayed one cycle, pulse-synced back to tck; rdata = prdata at
   completion (:230-251). APB master clock `sys_apb_clk` gated by activity.
3. `tdt_apb_decoder` — single slave degenerates to direct assigns (:285-293).

**Address mapping**: DMI address = spec DM word offset. DMI 0x10 (dmcontrol)
→ APB paddr 0x40 → DM decodes `dm_paddr[11:2]==10'h10` (tdt_dm.v:183, 663).
Single 4KB APB space, slave 0 at base 0.

**Clock domains**: `tck` (JTAG, external) vs `sys_apb_clk` (DM). Crossings:
only the pulse synchronizers in `tdt_apb_master` (:55-71): cmd_vld and
dmihardreset tck→pclk; apb_wr_ready pclk→tck — each `tdt_dmi_pulse_sync`.
Multi-bit payload sampled on the synchronized pulse, held stable in flight.
`tdt_dmi_pulse_sync.v`: toggle 4-F handshake; `tdt_dmi_sync_dff.v`: 2-FF
chain. Freq constraint (tdt_dmi_define.h:301-309): with SYSAPB off,
`Freq.pclk/Freq.tclk > 8/(IDLE_CYCLE-4) = 8/3`.

## 3. Debug Module (tdt_dm_top.v + tdt_dm.v)

`tdt_dm_top.v` (:280-414): gates `sys_apb_clk` into `dm_pclk`
(`local_en = dm_pclk_en`), instantiates `tdt_dm` with APB_AW=12, CORE_NUM=1,
PB_SIZE=4, IMP_EBREAK=1, CORE_MAX_XLEN=64, NEXTDM_BASEADDR=0, SBAW=40,
ALLCORE_NSCRATCH=4'h2, ALLCORE_ISA=4'h0.

**APB register map** (word offsets `dm_paddr[11:2]`, lines 171-227; read mux
:3413-3470):
| word off | byte | register |
|---|---|---|
| 0x04-0x0f | 0x10-0x3c | data0..data11 (only 0/1 for RV64; datacount=2 :2055-2059) |
| 0x10 | 0x40 | dmcontrol |
| 0x11 | 0x44 | dmstatus |
| 0x12 | 0x48 | hartinfo |
| 0x15 | 0x54 | hawindow |
| 0x16 | 0x58 | abstractcs |
| 0x17 | 0x5c | command |
| 0x18 | 0x60 | abstractauto |
| 0x1d | 0x74 | nextdm (=0) |
| 0x1f | 0x7c | **itr** (custom direct instruction) |
| 0x20-0x2f | 0x80-0xbc | progbuf0..15 (4 implemented + implied ebreak at index 4, :627-632; reads of 5-15 = 0) |
| 0x32 | 0xc8 | dmcs2 (reads 0) |
| 0x38-0x3a | 0xe0-0xe8 | sbcs, sbaddress0, sbaddress1 |
| 0x3c-0x3f | 0xf0-0xfc | sbdata0..3 |
| 0x40 | 0x100 | haltsum0 (hartsum0 = {31'b0, core_dm_halted_i} :2530-2534) |
| 0x70-0x79 | 0x1c0-0x1e4 | cuscs, cuscmd, cusbuf0-7 (custom; BUF_MOVE off) |
| 0x7f | 0x1fc | compid = {vendorid 12'b1011_011_0111_1, comptype 0, compversion 1} (:229-232, 2735-2739) |

**Spec version = 0.13** (TDT_DM_VERSION=4'h2 :230; DTM version 4'h1;
dcsr.xdebugver=4'b0100; field sets incl. 3-bit dcsr.cause, haltsum at 0x100).

**dmcontrol** (:663-836): bit31 haltreq (write-only edge), bit30 resumereq
(one-shot), bit29 hartreset (drives dm_core_rstn = !hartreset :1229-1251),
bit28 ackhavereset (one-shot pulse), bit26 hasel, [25:16] hartsel (single
core: hartsello forced 0), bit3 setresethaltreq → dm_core_halt_on_reset_o,
bit2 clrresethaltreq, bit1 ndmreset → ndmresetn (:808-815), bit0 dmactive
(on ungated pclk :817-829). dmactive 1→0 generates sync_rst (resets every
dm_pclk register); `dm_pclk_en = dmactive | dmactive_d1` gates the DM clock.

**dmstatus** (:893-969): bit22 impebreak=1, [19:18] all/anyhavereset,
[17:16] all/anyresumeack, [15:14] all/anyunavail, [13:12] all/anyrunning,
[9:8] all/anyhalted, [7] authenticated=1, [6] authbusy=0, [5]
haresethaltreq=1, [3:0] version=2.

**Halt/resume flow**:
- haltreq=1 → `dm_core_halt_req_o` (level, held until halted or haltreq
  falls; :1487-1514). `dm_core_halt_req_cause_o=2'b00`.
- Core halts → `core_dm_halted_i` (from dtu_tdt_dm_halted = synced
  rtu_yy_xx_dbgon) → allhalted/anyhalted, hartsum0[0]=1 (gates abstract/itr
  activity, `legal_cmd_sel` :1694-1700).
- Resume: resumereq with haltreq=0 → `dm_core_resume_req_o` 1-cycle pulse
  only when hartsum0[0] (:1533-1562). Core exits debug (dbgon drops) → DM
  edge-detects `core_dm_resume_req = halted_d && !halted` (:676-677) →
  resumeack.
- `dm_core_async_halt_req_o` is NOT the spec haltreq — it is the custom
  CUSCMD-0 async halt (:2696-2722). The spec haltreq
  (`dm_core_halt_req_o` → `dtu_rtu_sync_halt_req`) is **timing-1**
  (retire-boundary); the custom async one is timing-0 (immediate flush).
- `dm_core_halt_on_reset_o` (setresethaltreq) → dtu_ifu_halt_on_reset → IFU
  raises `ifu_rtu_reset_halt_req` on next reset (rtu cause 5).

**Abstract command engine** (command write gated `!busy && cmderr==0`,
:1691-1709):
- Fields: cmdtype=dm_pwdata[31:24] (only 0x00), aarsize=[22:20],
  aarpostincrement=[19], aarpostexec=[18], transfer=[17], write=[16],
  regno=[15:0].
- Decode (:2185-2197): access_gpr = regno[15:5]==11'h080 (0x1000-0x101f);
  access_csr = regno[15:12]==0 (any 12-bit CSR, TDT_ACC_CSR); access_dsc0 =
  regno==0x7b2; else cmderr=2. aarsize_err (RV64, :2017): aarsize!=3 &&
  (write || aarsize!=2).
- **Mechanism: the DM never touches the GPR file directly** — it uses the
  `itr` channel to execute synthesized system instructions in the core:
  - GPR read: itr = `csrrw x0, dscratch0, x{regno}` (:2280-2285), then read
    dscratch0 back.
  - GPR write: `dm_core_wr_vld` + `dm_core_wr_flg=2'b01` writes dscratch0
    (dtu side aq_dtu_ctrl.v:318, 398-408), then itr = `csrrc x{regno},
    dscratch0, x0` (:2286-2290).
  - CSR read FSM REGACC_* (:243-253, 2079-2181): X6_2_DSC1 (save), C_2_X6,
    X6_2_DSC0, DSC0/1_2_X6 restore; encoded itrs :2293-2324.
  - `dm_core_wr_flg` readback source (dtu side aq_dtu_cdc.v:366-379):
    00=dscratch0 (abstract read result), 01=write dscratch0, 10=latest_pc,
    11=satp (custom CUSCMD ops 2/3).
- Data: data0/data1 written by APB when !busy, or latched from
  core_rdata_mux on core_rdata_vld_r (:2477-2525); `dm_core_wdata =
  {data1,data0}` for RV64 (:1770).
- `abstractcs` (:2061): [28:24] progbufsize=4, [12] busy, [10:8] cmderr,
  [3:0] datacount=2. `busy = pb_work | cmd_work | itr_work` (:2005). cmderr:
  1=busy-write, 2=unsupported, 3=abstract inst raised exception
  (`core_dm_exce_retire && busy` :2041), 4=hart not halted. Cleared by
  writing 111 to [10:8].
- `abstractauto` (:1786-1804): autoexecdata[11:0], autoexecprogbuf[15:0].
- progbuf execution (`pb_work`, :1952-1988): itr sends progbuf[pb_idx],
  advances on core_dm_itr_done_mux, stops at EBREAK/CEBREAK (:240-241) or
  exception; implied ebreak at progbuf[4]. cmderr=3 on exception retire.

**SBA registers** (:2744-3130):
- `sbcs` (:2963-2964): [31:29] sbversion=1, [22] sbbusyerror, [21] sbbusy,
  [20] sbreadonaddr, [19:17] sbaccess (legal 2/3/4 for DW=128 :2862-2883),
  [16] sbautoincrement, [15] sbreadondata, [14:12] sberror, [11:5]
  sbasize=40, [4:0]=5'b11100 (0.13 layout).
- sbaddress0 (32b) + sbaddress1[7:0] (bits 39:32); autoincrement {4,8,16}
  for sbaccess {2,3,4}.
- sbdata0-3 (128-bit total).
- Issue: write sbdata0 (aligned) = write; write sbaddress0 with sbreadonaddr
  OR read sbdata0 with sbreadondata = read. `sba_wr_vld` pulse; alignment
  violation → sberror 3; unsupported sbaccess → 4; bus error (bresp[1]/
  rresp[1]) → 7. sbbusy from sba_wr_vld to sba_wr_ready.

## 4. SBA — tdt_sba_axi.v (AXI4-lite-style master)

Ports (:22-67): cmd `wr_data[127:0], wr_flg, wr_addr[39:0], wr_vld,
wr_size[2:0]`; resp `rd_data[127:0], axi_wr_ready (pulse), sba_error`; full
AXI channels (id[3:0], addr[39:0], len[3:0], size[2:0], burst[1:0], cache,
lock, prot; wdata[127:0], wstrb[15:0], wlast; rdata[127:0], rlast, bresp/
rresp[1:0]). Single-beat: awlen=arlen=0, wlast=1, bready=rready=1, id=0
(:232-242). Fixed attrs (tdt_dm.v:3266-3273): burst INCR, cache=0, lock=0,
prot=3'b010. awsize/arsize = sbaccess; wstrb lane-shifted by addr[3:0]
(:158-230); read data lane-extracted (:286-308); completion pulse after B or
R (:276-318); sba_error = bresp[1] | rresp[1] (:327-342).

Clocking: runs on `mclk = forever_cpuclk` gated by `axim_clk_en =
sys_bus_clk_en` (tdt_dm.v:3215-3265). DM↔SBA CDC (tdt_dm.v:3141-3212):
sba_wr_vld pclk→cpuclk pulse sync; response 2FF-synced back to pclk.

At chip level the SBA AXI master is an `openC906` port set (openC906.v:837-862)
for the SoC interconnect; the reference smart_run TB leaves it unconnected
(tr_axi_interconnect.v:861-895). **rv906 note: rv906's fabric is AXI — SBA
attaches as a crossbar master; no APB bridge needed here.**

## 5. dtu (core-side debug unit)

### aq_dtu_top.v (:15-210)
Submodules: `aq_dtu_ctrl` (cpuclk; :401-481), `aq_dtu_cdc` (DM crossing;
:485-524).

Core-provided inputs: CSR port `cp0_dtu_addr[11:0], cp0_dtu_wdata[63:0],
cp0_dtu_wreg/rreg, cp0_yy_priv_mode[1:0], cp0_dtu_satp[63:0],
cp0_dtu_mexpt_vld, cp0_dtu_pcfifo_frz, cp0_dtu_debug_info[5:0]`; RTU:
`rtu_dtu_dpc[63:0], rtu_dtu_halt_ack, rtu_dtu_pending_ack, rtu_dtu_retire_vld,
rtu_dtu_retire_next_pc[39:0], rtu_dtu_retire_chgflw, rtu_dtu_retire_halt_info[21:0],
rtu_dtu_retire_mret/sret, rtu_dtu_retire_debug_expt_vld, rtu_dtu_tval[63:0],
rtu_yy_xx_dbgon, rtu_yy_xx_expt_vld/int/vec[4:0]`; IFU (2 fetch slots):
`ifu_dtu_addr_vld0/1, ifu_dtu_data_vld0/1, ifu_dtu_exe_addr0/1[39:0],
ifu_dtu_exe_data0/1[31:0]`; LSU: `lsu_dtu_ldst_addr[39:0],
lsu_dtu_ldst_addr_vld, lsu_dtu_ldst_data[63:0], lsu_dtu_ldst_data_vld,
lsu_dtu_ldst_type[1:0] (2'b10 store / 2'b01 load), lsu_dtu_ldst_bytes_vld[15:0],
lsu_dtu_mem_access_size[2:0], lsu_dtu_halt_info[21:0], lsu_dtu_last_check`;
debug-info buses from idu/ifu/iu/lsu/mmu/rtu.

Outputs: to CP0 `dtu_cp0_rdata[63:0], dtu_cp0_dcsr_prv[1:0],
dtu_cp0_dcsr_mprven, dtu_cp0_wake_up`; to RTU `dtu_rtu_sync_halt_req,
dtu_rtu_async_halt_req, dtu_rtu_resume_req, dtu_rtu_step_en (=dcsr.step),
dtu_rtu_sync_flush (=icount_enable), dtu_rtu_int_mask (=step && !stepie),
dtu_rtu_ebreak_action, dtu_rtu_dpc[63:0], dtu_rtu_pending_tval[63:0]`; to IFU
`dtu_ifu_halt_info0/1[21:0], dtu_ifu_halt_info_vld, dtu_ifu_debug_inst[31:0]
+ vld (the itr), dtu_ifu_halt_on_reset`; to LSU `dtu_lsu_halt_info[21:0],
dtu_lsu_halt_info_vld, dtu_lsu_addr_trig_en, dtu_lsu_data_trig_en`; to HPCP
`dtu_hpcp_dcsr_stopcount`.

### aq_dtu_ctrl.v
Debug CSRs (read mux :581-618): tselect 0x7a0, tdata1 0x7a1, tdata2 0x7a2,
tdata3 0x7a3, tinfo 0x7a4, tcontrol 0x7a5, mcontext 0x7a8, scontext 0x7aa,
dcsr 0x7b0, dpc 0x7b1, dscratch0 0x7b2, dscratch1 0x7b3, haltcause 0xfe0
(custom 4-bit), dbgfifo 0xfe1 (custom), pcfifo 0xfe2 (custom).

**dcsr** (:296-379): `{xdebugver=4'b0100[31:28], 12'b0, ebreakm[15], 0,
ebreaks[13], ebreaku[12], stepie[11], stopcount[10], 0, cause[2:0][8:6], 0,
mprven[4], nmip=0[3], step[2], prv[1:0]}` — exact 0.13 layout. Writable only
in debug mode (`cp0_write_dcsr` requires rtu_yy_xx_dbgon :314); dcsr_prv
latches cp0_yy_priv_mode and dcsr_cause latches dtu_cause at
rtu_dtu_halt_ack (:353-370). ebreak_action = priv==U&&ebreaku || S&&ebreaks ||
M&&ebreakm (:377-379); int_mask = step && !stepie (:376). GPR file access is
NOT a DTU port — abstract GPR access goes through dscratch0 CSRRW sequences.

**dpc** latches rtu_dtu_dpc at halt_ack (:384-392). **dscratch0** write
sources: cp0 (debug mode), `tdt_dm_write_dscratch0` (= tdt_dm_wr_vld &&
wr_flg==2'b01 && dbgon), `updata_tval` (trigger tval); `dtu_rtu_pending_tval`
outputs dscratch0 (:398-410). **haltcause** 4-bit capture of dtu_cause at
halt_ack (:425-431).

**Pipeline on halt**: DTU does not flush — RTU does. Donor RTU
(aq_rtu_retire.v:609-793) distinguishes timing-0 halts (`halt_req =
reset_halt(ifu) || ebreak(action) || trigger_t0 || pending || dm_async` —
immediate flush) from timing-1 halts (`halt_req_t1 = dm_sync || trigger_t1 ||
step`, honored at a non-split retire). Entering debug sets dbg_mode_on after
frontend flush (:814-822) = rtu_yy_xx_dbgon, reported to DM as
dtu_tdt_dm_halted. Exit: dtu_rtu_resume_req or a retiring dret (:757-760).
Cause priority (:778-793): async=8, pending=halt_info cause, trigger=2,
ebreak=1, reset=5, dm_sync=3, step=4.
**Step**: dcsr.step → dtu_rtu_step_en → one instruction then re-halt (cause 4).
**icount**: icount_enable → dtu_rtu_sync_flush → retire_debug_flush
(:747-750). Wakeup: `low_power_wakeup = sync_halt_req || async_halt_req_wakeup
|| dcsr_step || pending_halt` (:623-626). **Reset-in-debug**: dmactive never
resets the core; hartreset/ndmreset are separate pad outputs. havereset FSM
in aq_dtu_cdc (below).

### aq_dtu_cdc.v — DM↔DTU CDC
`dtu_cdc_clk` = forever_cpuclk gated by cp0_yy_clk_en (:564-573).
- DM→DTU: halt_req 3FF level; async_halt_req 3FF + 2-edge pulse + wakeup;
  halt_on_reset 3FF; resume_req 4FF toggle pulse; ack_havereset 4FF; itr
  pulse+32b registered at pulse → dtu_ifu_debug_inst(+vld); wr_vld pulse +
  wr_flg[1:0] + wdata[63:0] registered at pulse.
- DTU→DM: dbgon 1FF+3FF → dtu_tdt_dm_halted; `rtu_dtu_retire_vld && dbgon`
  4FF → itr_done pulse; retire_debug_expt_vld 4FF; wr response: wr_vld
  delayed 1c → 4FF → wr_ready pulse; rx_data latched from dm_rdata mux
  (selected by wr_flg: 00=dscratch0, 01=0, 10=latest_pc, 11=satp).
- havereset FSM (:486-525): IDLE→PULSE→HAVE_RESET after cpurst_b release;
  PENDING after DM ack.
- Primitives: aq_dtu_cdc_lvl.v (3FF level), aq_dtu_cdc_pulse.v (toggle 4FF).

### Trigger architecture
`aq_dtu_trigger_module.v` (:223-344): core context (m/s/u_mode from priv +
mret/sret :227-229; write decodes :232-238; ldst addr/data vld :247-250;
access sizes :255-264), instantiates `aq_dtu_m_iie_all`.

`aq_dtu_m_iie_all.v`: **10 triggers** — 8 mcontrol (:618-707) + 2 iie
(:712-799). `tselect[3:0]` 0-9, writes >9 clamp to 9 (:376-386). Trigger CSR
read mux by tselect (:413-611). Shared `tcontrol` (MTE bit3, MPTE bit7,
mret/mexpt auto-update :829-857), mcontext[12:0] (0x7a8), scontext[33:0]
(0x7aa).

**mcontrol (type 2)** (`aq_dtu_mcontrol.v`):
- tdata1 encoding (:620-623): `{type=2[63:60], dmode[59], maskmax=6'd8[35:30],
  sizehi[23:22], hit[20], select[19], timing[18], sizelo[17:16], 3'b0[15:13],
  action[12], chain[11], match[3:0][10:7], M[6], 0[5], S[4], U[3], execute[2],
  store[1], load[0]}` — 0.13 layout.
- Write legalization: size — exe {0,2,3}, ldst {0,1,2,3,5,9(addr only)}
  (:504-512); action — 0 (bkpt exception) or 1 (enter debug), action=1
  requires dmode=1 (:515-517); match 0..5 only (:520-522); timing forced 0
  for execute (:525-527); chain blocked if prev/next chains (:534-536); dmode
  writable in debug mode only (:485-498); tdata_writable = `!dmode &&
  (m_mode||dbgon) || dmode && dbgon` (:553-554). hit set by set_trigger_hit
  (:610-618).
- **Match types** (:1125-1130): 0 equal; 1 NAPOT; 2 >=; 3 <; 4 lower-half
  32b value; 5 upper-half 32b value. Values: `ifu_dtu_exe_addr*` (execute
  addr) or extended `ifu_dtu_exe_data*` (execute data, select=1);
  `lsu_dtu_ldst_addr` / `lsu_dtu_ldst_data` with byte-lane masks
  (`lsu_dtu_ldst_bytes_vld`) + `lsu_dtu_mem_access_size` (:1104-1160+).
- tdata2 = full 64-bit. tdata3 = {mvalue[12:0], mselect, 14'b0, svalue[33:0],
  sselect[1:0]} (:690-691); sselect: 1=scontext, 2=satp asid, 3 illegal→0
  (:715-733).
- `tinfo` = 3'b100 (type 2 only).
- **Trigger→halt**: each mcontrol produces exe0/exe1/ldst_match + timing +
  action + chain. `aq_dtu_mcontrol_output_select.v` builds 22-bit halt_info
  bundles (mcontrol_halt_info0/1 for the two IFU fetch slots,
  dtu_lsu_halt_info) — they flow down the pipeline (IFU/LSU merge in their
  own halt_info registers, qualified by halt_info_vld) and reach RTU at
  retire as `rtu_dtu_retire_halt_info`; RTU fires halt_req_trigger_t0/t1
  (action=1) or breakpoint exception (action=0) — cause 2, dcsr.cause=2.
  Chaining: trigger N's chain requires N-1 match; pairs must share timing.

**IIE triggers** (`aq_dtu_iie_trigger.v`, triggers 8-9): type 3 = icount
(count hardwired 1; icount_enable = type3 && enabled → dtu_rtu_sync_flush;
icount_match = icount_enable && retire_vld); type 4 = itrigger (fires on
interrupt: exception_codes_onehot & tdata2[17:0] with int_vld :441-442);
type 5 = etrigger (fires on exception :448-449). tinfo = 6'b111000. Action
0/1, same dmode rule. Match sets pending_halt (:987-1001) → RTU
halt_req_pending; dtu_cause=4'd2 for iie action-1 (:969-975).

**dcsr.cause**: from rtu_retire_halt_info[CAUSE:CAUSE-3] at halt_ack
(:962-976).

**PC FIFO** (`aq_dtu_pcfifo.v`): 16x40b, pushes rtu_dtu_retire_next_pc on
retire_vld && retire_chgflw (:85), read via CSR 0xfe2, rptr reset to wptr at
halt_ack (:171-172); latest_pc feeds the DM rx path for CUSCMD-2.
**dbginfo** (`aq_dtu_dbginfo.v`): 4-entry 60-bit FIFO, CSR 0xfe1, recorded on
async halt.

## 6. Factory wiring

Hierarchy (openC906.v): aq_top :489 (contains aq_dtu_top aq_top.v:887-984) +
tdt_top :799. aq_core wires aq_rtu_top with dtu_rtu_*/rtu_dtu_* (aq_core.v:
2145-2352).
- tdt↔dtu: 11 signal groups route openC906.v:511-516, 577-586 → aq_top
  ports → aq_dtu_top (aq_top.v:924-929, 972-981), identical names both sides.
- DMI APB: `tdt_dmi_p*` are openC906 ports (openC906.v:134-141), produced by
  tdt_dmi_top at chip level (tr_axi_interconnect.v:919-943, fed by pad_dtm_*).
- Clocks: forever_cpuclk (core+RTU+DTU+DM cpuclk-side CDC+SBA), sys_apb_clk
  (DM + DTM APB master), tck (external JTAG). sys_bus_clk_en = axim_clk_en_f.
- Resets: cpurst_b (core), ciu_rst_b (tdt cpuclk side), sys_apb_rst_b,
  trst_b (external). tdt_dm_pad_ndmreset_n / hartreset_n are chip outputs
  (TB leaves open). pad_tdt_dm_core_unavail tied 0.

## WIRING MAP

### tdt (DM) <-> dtu — sys_apb_clk <-> forever_cpuclk (via aq_dtu_cdc.v)

| Signal | W | Source -> Sink | CDC | Notes |
|---|---|---|---|---|
| tdt_dm_dtu_halt_req | 1 | tdt_dm dm_core_halt_req_o (:1505) -> cdc:174 -> dtu_rtu_sync_halt_req -> rtu halt_req_dm_sync | 3FF lvl | spec haltreq, timing-1 |
| tdt_dm_dtu_async_halt_req | 1 | dm_core_async_halt_req_o (:2699, CUSCMD-0) -> cdc:188 -> dtu_rtu_async_halt_req pulse + wakeup | 3FF+2edg | timing-0 |
| tdt_dm_dtu_halt_req_cause | 2 | :1516 -> unused single-core (00) | lvl | |
| tdt_dm_dtu_resume_req | 1 | :1553 -> cdc:232 -> dtu_rtu_resume_req | 4FF pulse | |
| tdt_dm_dtu_halt_on_reset | 1 | :781 -> cdc:218 -> dtu_ifu_halt_on_reset -> rtu halt_req_reset | 3FF lvl | setresethaltreq |
| tdt_dm_dtu_ack_havereset | 1 | :1254 -> cdc:250 | 4FF pulse | |
| tdt_dm_dtu_itr | 32 | dm_core_itr_o = itr reg (:2013) -> cdc:294 -> dtu_ifu_debug_inst | lvl @ pulse | abstract/progbuf/itr inst |
| tdt_dm_dtu_itr_vld | 1 | :2343 -> cdc:269 -> dtu_ifu_debug_inst_vld | 4FF pulse | |
| tdt_dm_dtu_wr_vld | 1 | :2373 -> cdc:307 | 4FF pulse | abstract/custom data req |
| tdt_dm_dtu_wr_flg | 2 | :1756 -> cdc:341 | @ pulse | 00 rd dsc0 / 01 wr dsc0 / 10 rd latest_pc / 11 rd satp |
| tdt_dm_dtu_wdata | 64 | {data1,data0} :1770 -> cdc:350 -> dscratch0 write | @ pulse | |
| dtu_tdt_dm_halted | 1 | rtu_yy_xx_dbgon -> cdc:392 -> core_dm_halted_i | 3FF lvl | = dmstatus/hartsum0 |
| dtu_tdt_dm_havereset | 1 | cdc FSM :511 -> :520 -> core_dm_havereset_i | 3FF lvl | until ack |
| dtu_tdt_dm_itr_done | 1 | rtu_dtu_retire_vld && dbgon -> cdc:415 -> core_dm_itr_done_i | 4FF pulse | one per itr retire |
| dtu_tdt_dm_retire_debug_expt_vld | 1 | rtu_dtu_retire_debug_expt_vld -> cdc:433 | 4FF pulse | abstract cmderr=3 |
| dtu_tdt_dm_wr_ready | 1 | cdc:452 -> core_dm_wr_ready | 4FF pulse | |
| dtu_tdt_dm_rx_data | 64 | cdc rx_data (:366-379) -> core_dm_rx_data -> data0/1 | lvl @ pulse | source per wr_flg |
| tdt_dm_pad_ndmreset_n / hartreset_n | 1 | -> chip pads | out | |

### dtu <-> core (all forever_cpuclk, cpurst_b)

rtu->dtu: rtu_yy_xx_dbgon (halted), rtu_dtu_halt_ack (latches dpc/dcsr),
rtu_dtu_pending_ack, rtu_dtu_dpc[63:0], rtu_dtu_retire_vld,
rtu_dtu_retire_next_pc[39:0], rtu_dtu_retire_chgflw,
rtu_dtu_retire_halt_info[21:0], rtu_dtu_retire_mret/sret,
rtu_dtu_retire_debug_expt_vld, rtu_dtu_tval[63:0],
rtu_yy_xx_expt_vld/int/vec[4:0], rtu_dtu_debug_info[14:0].
dtu->rtu: dtu_rtu_sync_halt_req, dtu_rtu_async_halt_req, dtu_rtu_resume_req,
dtu_rtu_step_en, dtu_rtu_sync_flush, dtu_rtu_int_mask, dtu_rtu_ebreak_action,
dtu_rtu_dpc[63:0], dtu_rtu_pending_tval[63:0].
cp0<->dtu: cp0_dtu_addr/wdata/wreg/rreg, cp0_yy_priv_mode, cp0_dtu_satp,
cp0_dtu_mexpt_vld, cp0_dtu_pcfifo_frz / dtu_cp0_rdata, dtu_cp0_dcsr_prv,
dtu_cp0_dcsr_mprven, dtu_cp0_wake_up.
ifu<->dtu: ifu_dtu_addr_vld0/1, data_vld0/1, exe_addr0/1[39:0],
exe_data0/1[31:0] / dtu_ifu_halt_info0/1[21:0], halt_info_vld,
dtu_ifu_debug_inst[31:0]+vld, dtu_ifu_halt_on_reset.
lsu<->dtu: lsu_dtu_ldst_addr[39:0]+vld, ldst_data[63:0]+vld,
ldst_type[1:0], ldst_bytes_vld[15:0], mem_access_size[2:0],
lsu_dtu_halt_info[21:0]+last_check / dtu_lsu_halt_info[21:0]+vld,
dtu_lsu_addr_trig_en, dtu_lsu_data_trig_en.
dtu->hpcp: dtu_hpcp_dcsr_stopcount.

## Clone-driving observations
1. DMI is an APB bridge: every DMI op = one APB transaction, 7-idle-cycle
   budget (IDLE_CYCLE=3'h7).
2. Abstract commands execute via itr CSRRW/CSRRC sequences on
   dscratch0/1 + x6 — the DM never touches the GPR file directly.
3. Standard haltreq is timing-1 (retire boundary); only the custom CUSCMD-0
   async halt is timing-0.
4. dtu_tdt_dm_wr_ready / rx_data form a bidirectional req/resp pair; wr_flg
   selects the readback source (dscratch0 / latest_pc / satp).
5. Config-pinned constants to replicate: datacount=2, progbufsize=4 + implied
   ebreak, sbasize=40, SBA 128-bit, tinfo values, IDCODE 0x10000B6F, 10
   triggers with tselect clamp at 9.
