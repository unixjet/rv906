# 10. The Debug Subsystem: JTAG DTM + Debug Module + SBA + Core-Side Triggers

RISC-V Debug Spec 0.13 subsystem (`rtl/TDT_DTM.v` + `rtl/TDT_DM.v` +
`rtl/SBA_AxiUp.v` + `rtl/DTU.v`, plus the halt/resume/step seams in
`rtl/{RTU,CSR,IFU,LSU,IDU,RVProc,RVProcAXI}.v`), per M7 design doc
`docs/superpowers/specs/2026-09-11-m7-debug-design.md`.

How to read it: §1 is the normative model (donor TDT_DTM/TDT_DM/DTU +
RISC-V Debug Spec 0.13). §2 is this implementation, per block, with
file:line for every claim. §3 is the rv906→C906 file cross-reference. §4
is the design discussion: the M7 deviation ledger, findings, and what
stayed unbuilt.

Normative documents:

- `docs/superpowers/specs/2026-09-11-m7-debug-design.md` — M7 design doc
  (architecture §3, D-M7-1..10, task table, "Files expected to change").
- RISC-V Debug Spec 0.13 — DMI register layout, DM register map, SBA,
  abstract commands, cmderr semantics.
- Donor RTL `refs/openc906/C906_RTL_FACTORY` — `tdt_dtm_*.v`,
  `tdt_dm.v`, `tdt_sba_axi.v`, `aq_dtu_*.v`, `aq_rtu_retire.v`,
  `aq_cp0_*.v`. Donor line cites in `rtl/` headers and inline comments
  are the spec per the clone-discipline rule (CLAUDE.md).

## 1. Principle

The C906 debug subsystem splits three ways, and rv906 keeps the split
(D-M7-1: single clock domain — all three blocks are plain
posedge(clk) clk_ff; the only asynchronous thing is the JTAG clock):

- **TDT_DTM** (tck domain, `rtl/TDT_DTM.v`): the IEEE 1149.1 TAP-5
  (16-state) FSM, the IR/DR shift chain, and the DMI→APB bridge. It
  converts TCK-phase shift operations into APB transactions on clk,
  through 4-FF toggle-pulse synchronizers with a held-stable payload
  contract (donor `tdt_dmi_define.h:301-309` timing contract:
  `Freq.pclk/Freq.tck > 8/(IDLE_CYCLE-4)`; with `IDLE_CYCLE=7` that is
  8/3, so any clk:tck ≥ 8:1 with 7 idle TCKs per DMI is safe — the C++
  driver runs exactly clk:tck = 8:1, §2.6).
- **TDT_DM** (clk domain, `rtl/TDT_DM.v`): the Debug Module — the 0.13
  APB register file (dmcontrol/dmstatus/abstract*/progbuf/hartinfo/...),
  the abstract command engine (GPR/CSR access + ITR), and the SBA
  registers plus its 128-bit AXI master. The DM talks to the core only
  through the abstract engine: every core access is a debug-mode
  instruction (ITR) or a dscratch-mediated CSR move — there is no
  direct DM↔core register wire.
- **DTU** (clk domain, `rtl/DTU.v`): the core-side Debug Trigger
  Unit — dcsr/dpc/dscratch0/1, 10 trigger slots (8 mcontrol + 2 iie),
  execute/load/store comparators, the halt_info verdict, and dret
  wake. The DTU has no register file of its own; it is a verdict
  machine that rides the normal fetch/issue/retire pipeline.

The **halt flow** (donor `aq_rtu_retire.v:606-682, 755-795, 905-913`):
a halt request is formed at RETIRE in RTU from five sources —
debug-exception (ebreak, cause=1), trigger match at issue (timing-0,
cause=2), DM sync halt (cause=3), step (cause=4), reset halt-on-reset
(cause=5); the request flushes the pipeline front-end, latches
`dpc = current retired PC`, sets `rtu_yy_xx_dbgon` (the one global
"debug mode is on" bit every block gates on), and freezes commit:
`retire_trap_vld` is gated by `!halt_req && !dbg_mode_on` (RTU.v:1175-1176),
interrupt delivery is gated by the DTU's `dtu_rtu_int_mask`
(RTU.v:940, DTU.v:259 = `dcsr.step && !dcsr.stepie`). **Resume** is a
retire-time event: `retire_exit_debug = dbg_mode_on_after_req &&
(resume_req || dret)` (RTU.v:1143-1144) redirects the fetch to dpc and
drops dbgon; privilege at resume comes from the exit-debug priority arm
of the CSR privilege latch (`pm_wdata = dtu_cp0_dcsr_prv`,
CSR.v:622-624). **Step** is the same machinery with
`int_mask` armed so nothing interrupts the one instruction
(dcsr.stepie=0 → mask=1).

The **trigger model** (donor `aq_dtu_mcontrol.v`,
`aq_dtu_trigger_module.v:247-250`,
`aq_dtu_mcontrol_output_select.v`): each enabled slot compares its
address against the executing instruction's PC (execute trigger, live
at the IFU ibuf head) or against the physical address (or data, for
data-match triggers) of an issued load/store (ldst trigger, live at the
LSU AG stage). A match assembles a 22-bit `halt_info` bundle
(rvproc_pkg.sv:334-356, donor `aq_dtu_cfig.h` `TDT_HINFO_*`) that rides
the instruction: execute path IFU→IDU(ex1)→RTU(ex2), ldst path
LSU→RTU, ORed at RTU like donor `aq_rtu_dp.v:399-401`. Two timing
modes: **timing-0** fires on the matched instruction (execute: halt it
at the retire boundary; ldst: trap at issue, the access never commits);
**timing-1** fires at the retire boundary after the instruction's side
effects happen. Two actions: **0 = breakpoint trap** (mcause=3,
mepc = the trigger PC, handler skips via mepc+=4) and **1 = enter
debug** (dcsr.cause=2).

The **debug host** is the Verilator/C++ testbench itself — no OpenOCD.
`dut.cpp` owns the four JTAG pads and a JTAG clock discipline
(§2.6); `RVProcTest.cpp` implements the DMI client (TLR, IDCODE,
DTMCS, DM register r/w, abstract commands, SBA) as plain C++ over
`JTAG::cycle()`.

## 2. This implementation

### 2.1 JTAG DTM — `rtl/TDT_DTM.v` (743 lines)

Clone of `tdt_dtm_ctrl.v` + `tdt_dtm_chain.v` + `tdt_dtm_idr.v` +
`tdt_apb_master.v` + `tdt_dmi_pulse_sync.v` + `tdt_dmi_sync_dff.v`
(header 1-49 lists the donor files and the dropped T-Head bits).

**TAP-5 FSM** (83-179, verbatim `tdt_dtm_ctrl.v:93-208`): 16 states
(83-98), `dmihardreset` forces TAP5_RESET (110-115), `tck`-edge update,
`trst` tied 1 — no async reset pad (D-M7-2). The transition table is a
case over state + TMS (118-170); the fsm status decodes (shift/update/
capture ir/dr) are at 174-179.

**TDR chain** (181-265, `tdt_dtm_chain.v:42-86`): IR opcodes
(192-195) `IDCODE=5'h01 / DMI_ACC=5'h02 / DTMCS=5'h10 / DMI=5'h11`;
`CHAIN_DW=44` (197-199) so the DMI DR shift register
`{addr[9:0], data[31:0], op[1:0]}` is LSB-first with `op` in the last
two positions. The capture/shift mux (213-239) is verbatim donor
`tdt_dtm_chain.v:56-86`; the `mode` bit (a DMI_ACC-DR-written flop,
338-345) selects the 34-bit vs 44-bit DMI shift in the DR shift path
(224-236). **TDO is negedge-TCK-registered** (256-263, donor
`tdt_dtm_chain.v:105-122`) — it updates on the TCK falling edge and
holds until the next one (the C++ driver samples it after the
falling-edge eval, §2.6), with an `else tdo<=1` fallthrough (261-262)
so the TAP reads 1 whenever it is not shifting — the first TLR bit is
`1`, and the no-`trst_n` power-up is covered (250-255).

**TDR + DMI request engine** (267-440, `tdt_dtm_idr.v:45-208`):
`IDCODE_REG_DEFINE = 32'h1000_0B6F` (280), `DTM_VERSION=4'h1`,
`IDLE_CYCLE=3'h7` (281-282). IR latch loads the TDR on TLR (304-311 —
the no-`trst_n` compensation for "IDCODE always selectable").
`dmihardreset` (316-325, 1-tck pulse) forces TAP5_RESET
(110-115) and, through the pulse sync (469-476), resets the pclk
APB bridge; `dmireset` (327-336) clears the DMI request engine
(`op_stat` arm, 406-410). The DMI request engine (353-414): the
write request fires on an `IR_DMI` update carrying a read (01) or
write (10) op (`^chain_idr_data[1:0]`, 384-385) while idle (376-387);
the in-flight flag (391-398); the DMI DR read mux (426-436) returns
`dmi_total` where **op==2'b11 = busy** (a mid-flight capture, 418-420;
`op_stat` 406-414).

**DMI→APB bridge** (442-644, `tdt_apb_master.v:55-265`): three
`tdt_dmi_pulse_sync` instances (454-485) convert the held-stable
payload `{op, addr[9:0], data[31:0]}` into clk-domain pulses; the APB
FSM (509-555) runs APB_IDLE/SETUP/ACCESS, back-to-back (donor
`:140-162`), `paddr = {addr, 2'b0}` byte-aligned (597); **read data
is held stable across the whole DMI cycle** — the `prdata_smp` latch
is only updated at transaction completion (619-633), so the
7-idle-TCK readback window never sees glitches (donor
`tdt_apb_master.v:164-251`). The 4-FF handshake module is inlined as
`tdt_dmi_pulse_sync` (648-718) + `tdt_dmi_sync_dff` (720-743); the
header cites the donor timing contract (`tdt_dmi_define.h:301-309`)
and notes the C++ driver honors it with clk:tck = 8:1 (§2.6).

### 2.2 Debug Module — `rtl/TDT_DM.v` (1323 lines)

Clone of `tdt_dm.v` (header 1-40 lists the config cites: 1 hart,
`SINGLE_CORE=1`, `PREG_LEN=4`, `DCC_LEN=32`, `PREG_NUM=4`, `DSCR_NUM=2`,
10 triggers, SBA 128-bit single-beat, `SBASIZE=40`).

**Register map** — DMI word offsets (116-142, donor
`tdt_dm.v:171-227`; the DMI addr field IS the word offset, TDT_DTM
shifts <<2 to paddr): DATA0/1 = 0x04/0x05; DMCONTROL = 0x10,
DMSTATUS = 0x11, HARTINFO = 0x12, HAWINDOW = 0x15, ABSTRACTCS = 0x16,
COMMAND = 0x17, ABSTRACTAUTO = 0x18, NEXTDM = 0x1d, ITR = 0x1f; PB0-3
= 0x20-0x23; DMCS2 = 0x32; SBCS = 0x38, SBADDR0/1 = 0x39/0x3a,
SBDATA0-3 = 0x3c-0x3f; HARTSUM0 = 0x40; CUSCS/CUSCMD = 0x70/0x71
(D-M7-5: read 0 / writes ignored); COMPID = 0x7f. APB mini-decode
(279-285, donor :598-609). `JEP106_ID = 12'hB6F`, `DM_VERSION = 4'h2`
(144-145); `DSCR0/1_ADDR = 0x7B2/0x7B3` (148-149); `EBREAK_INST =
0x00100073` (154). `hartinfo` = nscratch=2 (1273-1275); `compid`
(895).

**progbuf** (290-315, donor :614-657): 4 usable entries + the implied
ebreak at index 4 (305-307), rest zero (309-315) — the 0.13
`progbufsize=4` layout. Writes only when `~busy` (299).

**dmcontrol** (320-402, donor :660-836): `haltreq` rising edge
(321-322), `resumereq` gated on `hartsum0[0]` (335-340, donor :1486
"resume requires halted"), `hartreset` (342-346), `ackhavereset`
(348-353), `hasel` (355-359), `setresethaltreq`/`clrresethaltreq`
(361-373), `halt_on_reset` (375-380), `ndmreset` (382-386), `dmactive`
(388-391) with **`sync_rst = dmactive_d1 & ~dmactive`** (398, donor
:1257-1259 — dropping dmactive resets the DM into its reset state:
this is D-M7-9's "no clock gating; held in reset" mechanism). Readback
(400-402) returns the stored bits (WARL read-your-write).

**dmstatus** (417-431, donor :890-969): `version=2`,
`anyhalted/anyrunning` from `core_dm_halted_i` (427-429),
`impebreak=1`, `authenticated=1` (430-431), `haltave`/`hasresethaltreq`
(428). `resumeack = !sba_busy & !anyrunning` (433-438).

**Reset outputs** (445-462, donor :1226-1288): `dm_core_rstn_o` =
ndmreset|hartreset with ackhavereset/hasel semantics; `ack_havereset`
(451-456); `ndmreset_n` (458-462). (Dangling as chip outputs, §4.)

**halt/resume** (467-482, donor :1486-1562): `dtu_dm_halt_req` pulses
on `haltreq` rising edge (472-474); `dtu_dm_resume_req` on `resumereq`
(476-482).

**Abstract command engine** (487-666, donor :1688-2061): the COMMAND
word (0x17) starts a command when `!busy` (574-582) and `regno <
0x1020` (582). The command is decoded into a REGACC FSM sequence
(668-756, donor :2064-2181): `access_gpr` (regno[15:5]==5'h080, 761),
`access_csr` (regno[15:12]==0, 762), `access_dsc0` (regno==0x7B2,
763), else **cmderr=2 not supported** (764; 652-655). **cmderr**
(646-663, 0.13 §2.6.2): 1 = busy conflict (661-662), 2 = not
supported (652-655), 3 = debug-mode exception (656-657), 4 = not
halted (658-660); W1C clear (649-651). `abstractcs` readback
(665-666): `progbufsize=4`, `datacount=2`.

**The core access path is pure debug-mode instructions** (796-834,
donor :2182-2446): GPR read = ITR `csrrw x0, dscratch0, x{regno}`
(796-799); GPR write = store regno in dscratch0, ITR
`csrrc x{regno}, dscratch0, x0` (800-802); CSR r/w routes through the
dscratch1/x6 REGACC FSM (804-818). `itr_vld` (821-834) drives
`dtu_dm_itr_vld` + `dtu_dm_itr_inst`; the DTU executes it as a normal
instruction in debug mode (§2.4). `data0`/`data1` (875-891, donor
:2448-2525) mirror the core's dscratch0/dscratch1. `haltsum0 =
{31'b0, halted}` / `hartsum0` (893). `sba busy`/`abstractcmd busy`
share the single `busy` (638).

**SBA registers** (900-1106, donor :2741-3130): `sbaddress0/1`
(915-931, 40-bit; auto-increment 920-921/929-930, `sbaddrplus` 902-909),
`sbversion = 3'h1` (933), `sbbusy` (935-940), `sbbusyerror` (942-954),
`sbaccess_unalian` (956-958: 32-bit needs a[1:0]==0, 64-bit a[2:0]==0,
128-bit a[3:0]==0). **`sberror`** (982-993): **4 = unsupported
access** (987-988, `sberror_will_be_4` set when sbaccess ∉ {2,3,4},
965-966), **3 = unaligned** (989-990, 976-977), **7 = sba_error
(bresp/rresp)** (991-992). `sbasize = 7'd40` (995),
`sbaccess_info = 5'b11100` (996: 128/64/32 supported, no 8/4).
`sbaccess` reset value 3'h2 (32-bit) (1004-1008); on an sbcs write the
register **latches the raw requested width** (1007 — donor-faithful;
spec 0.13 would clear sbaccess on a failed access, see §4). `sbcs`
readback (1022-1024). `sba_write` fires on the SBDATA0 write (1026-1027);
`sba_read` fires on the SBADDR0 write with sbreadonaddr, or the SBDATA0
read with sbreadondata (1028-1031). A transaction is launched only
`sb_noerr && !sberror_will_be_3 && !sberror_will_be_4` (1063) — an
unsupported/unaligned request is ignored. `sbdata0-3` (1067-1101);
`sba_w_data = {sbdata3..0}` (1103).

**SBA AXI master** (1113-1259, clone of `tdt_sba_axi.v:72-342`):
single-beat, `awsize = sbaccess` (1148-1151), `awlen = 4'h0` (1188),
`burst = 2'b01` (1193, 1197), `prot = 3'b010` (1196, 1200),
`bready/rready = 1` (1186, 1190); `wstrb_pre = 16'hffff/00ff/000f` by
size (1153-1154) with the 128-bit `wdata`/`wstrb` lane-shifted by
`addr[3:2]` (1158-1174); the read side re-shifts `rdata_smp` back down
by `addr[3:2]` (1230-1237). Donor-unchanged surface (D-M7-4); the
128→512 up-conversion to the rv906 crossbar is §2.3.

**APB read mux** (1282-1313, donor :3364-3470): the full 0.13 map;
CUSCS/CUSCMD read 0 (D-M7-5).

### 2.3 SBA→crossbar up-converter — `rtl/SBA_AxiUp.v` (220 lines)

rv906-specific (no donor — the C906 SBA master runs 128-bit AXI against
the C906 fabric; rv906's crossbar is 512-bit, contract 17). Pure
combinational: the 16-byte window is placed at **beat byte offset
`16*a[5:4]`** (155-188) — `s_awaddr[5:0]` selects the 16B slot within
the 64B line; the write side is four 512-bit placement wires
`wdata_p0..p3` (175-178) and four 64-bit `wstrb_p*` lane-shifts
(179-182), selected by 2:1 trees on `w_win = m_awaddr[5:4]` (183-188).
Read side (207-213): four 128-bit part-selects
`rdata_p0..p3 = s_rdata[127:0 + 128*i]` (207-210), same 2:1 tree on
`r_win = m_araddr[5:4]` (211-213). Address 40→64-bit zero-extension
(158, 198); `awlen=4'h0`, `awsize = 2/3/4` (the actual access size),
`burst=2'b01`, `prot=3'b010` pass through (159-166). The byte-lane
proof is in the header (30-72).

### 2.4 Core-side DTU — `rtl/DTU.v` (932 lines)

Clone of `aq_dtu_ctrl.v` (dcsr/dpc/dscratch, :296-420) +
`aq_dtu_m_iie_all.v` (trigger storage, :375-611) +
`aq_dtu_mcontrol.v` (comparators, :480-1330) +
`aq_dtu_iie_trigger.v` (:174-184) +
`aq_dtu_mcontrol_output_select.v` (:2968-3395) + `aq_dtu_top.v`
(port list :15-210).

**dcsr** (176-256): 0.13 layout (176-179); software-writable fields
step/mprven/stopcount/stepie/ebreaku/ebreaks/ebreakm (205-224);
`dcsr.prv` latched from `cp0_yy_priv_mode` at halt_ack, also writable
in debug mode (halt_ack wins ties — donor order, 226-239);
**`dcsr.cause` latch-only at halt_ack** from `rtu_dtu_halt_cause[2:0]`
(241-248); read value `xdebugver = 4'b0100`, `nmip` tied 0
(250-256). **`dtu_rtu_int_mask = dcsr_step_r && !dcsr_stepie_r`**
(259). **`dtu_rtu_ebreak_action`** (260-262): the current privilege's
ebreak field (ebreakm/s/u) selects enter-debug over trap.

**dpc** (264-279): three arms only — reset 0, cp0 write (debug-mode
redirect), and **`halt_ack` = the retired PC that halted**
(277-278). No +4 arm — stepping advances dpc one halt_ack at a time
(donor does not auto-increment either; RTU.v:1409).

**dscratch0/1** (281-301): dscratch0 write priority cp0 > DM
(287-294); dscratch1 is cp0-write-only (296-301).

**Wake / status** (309-324): `dtu_cp0_wake_up = tdt_dm_dtu_halt_req ||
(dcsr_step_r && !rtu_yy_xx_dbgon)` (318); `dtu_rtu_dpc` (323) /
`dtu_rtu_pending_tval` tied 0 (324, single-issue, D-M7-8);
`dtu_tdt_dm_halted = rtu_yy_xx_dbgon` (352); `dtu_tdt_dm_itr_done`
(360-367, one pulse per debug-mode-retired instruction).

**ITR injection** (337-346): `dtu_ifu_debug_inst_vld/inst` = the ITR
payload, same-cycle echo of `tdt_dm_dtu_itr_vld` (the DM's abstract
engine's instruction is re-injected at the IFU ibuf head, §2.5).

**havereset FSM** (417-443): the donor havereset/`dm_core_rstn`
handshake; `dtu_dm_ack_havereset` pulses.

**Triggers** (451-927): storage = 10 slots × (ttype/tdata1/tdata2/
tdata3) with the **standard 0.13 `mcontrol` tdata1 layout** (455-470,
D-M7-5 — the T-Head custom bit positions are dropped): type[63:60],
dmode[59], maskmax[58:52], data[51:20], select[19], timing[18],
action[17:12], chain[11], match[10:7], m[6], s[4], u[3], execute[2],
store[1], load[0] (473-506 localparams; the as-built bit list is the
`m_tdata1_legal` concat at 574-590). `tselect` WARL
(512-528, donor `aq_dtu_m_iie_all.v:375-386` — clamp at 9; `tsel_oh`
one-hot 531-546). `mcontrol tdata1` WARL legalization (561-590):
`type` must be 2 (else stored as type=0, 564-565), `dmode` sticky 0
outside debug (566-567), `match ≤ 5` (568-569), `action ≤ 1`
(570-571), **`timing` forced 0 for execute triggers** (572-573 — an
execute trigger that halts on the next instruction is meaningless in
single-issue; the donor allows it, rv906 legalizes it). `iie tdata1`
WARL (592-609): type must be 3 (593-594), `icount` forced 1 (601 —
the multi-instruction counter is not built, §4). `tcontrol` (639-649): **MTE gates** action-0 M-mode
triggers (641-642, per-slot `s_tctl` at 754); MPTE stored but not
gated (634-638 — the mret save/restore path is dropped, §4).
`mcontext`/`scontext` storage (651-660) — read back, but no chain
match (single hart).

`tinfo` (666-669, 901-903): mcontrol `0x10` (reduced from donor's
0x30 — the donor's extra bits advertised `mask`/`snapshot`/`data`
extensions rv906 doesn't implement; D-M7-5), iie `0x30`.

**Comparators** (676-787, the T8b core): **the address inputs are
sign-extended** (707-710, T8b fix — see §4):
`exe_addr64 = {exe_sig_ext, ifu_dtu_exe_addr}`,
`ldst_addr64 = {ldst_sig_ext, lsu_dtu_ldst_addr[39:0]}`. The match
function (716-731): 0=eq, 1=NAPOT (`lowbit_index` 791-804), 2=ge,
3=lt, 4=low32, 5=up32. Per-slot generate (743-787): `s_en` (755-756) =
type==2 (749) & priv-match (750-752) & MTE (754) &
**`!rtu_yy_xx_dbgon`** & !dmode (756) — **triggers are inactive in
debug mode**. Execute match (758-760): the PC at the ibuf head
(`ifu_dtu_exe_addr_vld`). Ldst match (762-768): gated by the slot's
load/store direction bits (762-763) and, for data-match triggers
(`s_sel`, 765), compares `lsu_dtu_ldst_data` instead of the address
(766). **timing-0 ldst cancel** (779): `s_ldst_match && !TIMING` —
either access type, either action: the LSU traps it at issue (the
access never commits). **Store-suppression enables** (784-785): a
timing-0 matched STORE trigger, address- (`!s_sel`) or data- (`s_sel`)
matching — the store is suppressed at commit.

**Halt info assembly** (806-850, donor
`aq_dtu_mcontrol_output_select.v:2968-3395`): lowest-index matching
slot wins (`lowbit_index`, 806-809). **Execute path** (811-830):
`{TRIGGER[21:12]=slot, CAUSE[11:8]=4'd2 (donor const, 814),
TIMING=0 forced, ACTION01=0 (single-match conflict flag),
ACTION=action01 (820), CHAIN=0, LDST=0, MATCH=1, CANCEL=1 const
(827 — donor `exe0_cancel`: every non-chain execute match cancels the
instruction's side effects)}`. RTU then interprets the bundle:
action-1 → timing-0 halt (dcsr.cause=2), action-0 → the leg-4
breakpoint trap (mcause=3, §2.5). **Ldst path** (832-846): same shape
with `TIMING = the trigger's timing` (837), `LDST=1` (844),
**`CANCEL = timing-0 ldst match`** (846 — donor `ldst_cancel`).
**Store-suppression enables** (849-850):
`dtu_lsu_addr_trig_en` / `dtu_lsu_data_trig_en` = any timing-0 matched
store trigger.

**iie/icount** (852-868): slots 8-9, `type==3`; count is hardwired 1,
so an enabled icount trigger matches on every retire (donor
`icount_match = icount_enable && rtu_dtu_retire_vld`,
`aq_dtu_iie_trigger.v:432-433`) — structurally live, but no M7 gate
arms a type-3 slot, so it never fires in the gate set (§4).

**Pending-halt record** (870-888, donor
`aq_dtu_mcontrol_output_select.v:2968-2984`): a timing-1 action-1
match (or iie icount) arms `dtu_rtu_pending_halt`; RTU honors it at the
next t1 retire boundary (RTU.v:1113).

**Trigger CSR read mux** (890-927): 0x7A0-0x7AA (tdata1/2/3 ×
tselect, tinfo, tselect, tcontrol, mcontext, scontext) — the CSR.v
router sends exactly these plus 0x7B0-0x7B3 (CSR.v:1370-1379,
1656-1670).

### 2.5 Core seams — the halt/trigger/step plumbing

The DTU verdicts and the DM's `rtu_yy_xx_dbgon` bit touch every
pipeline block. Cited seams:

**RTU** (`rtl/RTU.v`): halt source (309-351). **Timing-0** halt
(1066-1083): reset (1067), ebreak (1068), trigger-t0 (1076-1082:
`ex2_retire_vld && MATCH && !CHAIN && !TIMING && ACTION &&
!PENDING_HALT && !dbg_mode_on_after_req` — an action-1 trigger match
halts the core with the triggered instruction's side effects already
removed). **Timing-1** (1091-1115), all under `halt_req_t1_retire_vld
= ex2_retire_vld && !dbg_mode_on_after_req && !halt_req_t0`
(1091-1092): dm_sync (1093), step (1094), trigger-t1 (1101-1106:
`MATCH && !CHAIN && TIMING && ACTION && !PENDING_HALT` — the
instruction completes, then halts at the retire boundary),
pending-halt (1113). `halt_req` (1118). **Halt cause** (1125-1137):
trigger=2, ebreak=1, reset=5, dm_sync=3, step=4 (priority order).
**The action-0 trigger trap** (975-979, leg 4 of the exception chain):
`retire_bkpt_expt = ex2_retire_vld && MATCH && !CHAIN && !ACTION &&
!PENDING_HALT` → `retire_trap_vec = 5'd3` (989-990), counted as a sync
exception so `mepc = ex2_cur_pc` (1010-1011, 1038) and
`mtval = 0` (1031); a timing-1 action-0 match traps after the side
effects (the leg is deliberately not TIMING-qualified, 966-972). Leg 1
(pending-breakpoint, donor `bkpt_req_pending`) stays 0 in rv906 — the
pending path is the DTU level, honored as a t1 halt (955-961).
`retire_exit_debug = dbg_mode_on_after_req && (dtu_rtu_resume_req ||
(ex2_retire_vld && ex2_inst_dret))` (1143-1144).
`dbg_mode_on_after_req` sets on `halt_req`, clears on exit (1146-1153);
**`dbg_mode_on` (= `rtu_yy_xx_dbgon`, 1392-1393) sets only at the
frontend flush** (`retire_flush_be && dbg_mode_on_after_req`, 1155-1162)
— the two-stage: `_after_req` is "a halt is requested this cycle",
`dbg_mode_on` is "the pipeline has actually flushed and we're in
debug" (the bit every block gates on). **`retire_trap_vld` is gated by
`!halt_req && !dbg_mode_on`** (1175-1176) — a halted hart does not
commit traps; `retire_expt_debug` (1167-1169) excludes the trigger
breakpoint (it is the trigger's own action, not a debug exception).
Flush (1217) on `halt_req`. `retire_chgflw_pc` (1280-1291) = dpc on
`retire_exit_debug` (resume/dret redirect). `ex1_wb_cancel`
(1314-1322) = `ex1_halt_info[TDT_HINFO_CANCEL]` — cancels the WBT
entry for a trigger-cancelled instruction. `rtu_dtu_dpc = ex2_cur_pc`
(1409), `halt_ack` (1414), `retire_halt_info` (1426), `pending_ack`
(1430), `exit_debug` (1437). `halt_req_dm_async_tied0` (1194) — no
async halt (D-M7-7); the dead-but-present async shortcut (1245) is kept
for donor-diffability.

**CSR** (`rtl/CSR.v`): cp0↔dtu port (297-320). **ecall/mret/sret/wfi
fire are gated `&& !rtu_yy_xx_dbgon`** (367-372); **ebreak in dbgon** is
converted to a timing-0 halt instead: `cp0_rtu_ebreak_halt = is_ebreak
&& (dtu_rtu_ebreak_action || dbgon)` (1810) and the vec-3 `ebreak_expt`
is withheld (1811) — a halted hart cannot self-except except via dret;
**`is_dret = ex1_inst_dret && rtu_yy_xx_dbgon`** (377) — dret is
illegal outside debug (IDU.v:872-873).
**`wfi_wake = (mip_value & mie_reg) != 0 || dtu_cp0_wake_up`** (442) —
the DM's haltreq or a pending step wakes a WFI-halted core. **Exit-debug
privilege arm** (616-624): `pm_wen` (622) top priority =
`rtu_cp0_exit_debug`, `pm_wdata = dtu_cp0_dcsr_prv` (624) — resume
privilege comes from dcsr.prv. **`pm_eff = pm_r | {2{dbgon}}`** (640) —
debug mode is at least M. **MPRV in debug** (1341-1347):
`cp0_lsu_mprv = dbgon ? (dcsr.mprven && mprv_f) : mprv_f` (1347) — the
0.13 `mprven` rule. cp0→dtu routing (1370-1379): `csr_dtu_addr`
covers 0x7B0-0x7B3 + 0x7A0-0x7AA; read mux (1656-1670) returns
`dtu_cp0_rdata`. `cp0_rtu_ex1_inst_dret` (1828).
**IFU** (`rtl/IFU.v`): DTU debug ports (198-232). **Reset halt**
(258-270): `reset_halt_req` at the first fetch after reset when
`dtu_ifu_halt_on_reset` (265-270). **Debug-mode fetch mask** (521-525):
`ctrl_inst_fetch = ibuf_ctrl_inst_fetch && !rtu_yy_xx_dbgon` — in
debug the normal fetch is masked; only the ITR debug instruction
fetches. **IBUF output** (1073-1097): the ITR instruction is injected
at the ibuf head (1074-1077); **the T8b BUG-1 gate (1093-1095)** —
`ifu_idu_id_inst_vld = (pop_entry_vld && !rtu_yy_xx_dbgon) ||
dtu_dbg_inst_deliver` (the `!rtu_yy_xx_dbgon` is the load-bearing
donor-deviation term, §4). **Live execute verdict** (1114-1124):
`ifu_idu_id_halt_info` = the DTU's execute-trigger match on the PC at
the ibuf head (1124-1125); `ifu_dtu_exe_addr = ibuf_pc[ibuf_head]`
(1145-1146) — the head PC, with `pop_entry_vld` as the vld.

**LSU** (`rtl/LSU.v`): DTU ldst ports (220-246). **ldst feed**
(679-701): `lsu_dtu_ldst_addr = ag_pa` (685), `vld = issue_real`
(686), `type` bit0=store/bit1=load (694). **`issue_real =
ag_issue_ready && mmu_lsu_pa_vld && !rtu_yy_xx_dbgon`** (1190 — the
T8b BUG-1 defense-in-depth term, §4). **Trigger fault** (1210-1248):
`ag_halt_info_buf = dtu_lsu_halt_info_vld ? dtu_lsu_halt_info :
idu_lsu_ex1_halt_info` (1240-1242); **`trig_fault_issue = issue_real
&& !ag_misalign && !mmu_fault_issue && ag_halt_info_buf[TDT_HINFO_CANCEL]`**
(1247-1248 — donor `aq_lsu_ag.v:1432`, `ag_pipe_dt_cancel` is the
CANCEL bit alone). `dc_halt_info_r` latch (1355, for the reply path);
`dc_store_cancel_r = dtu_lsu_addr_trig_en || dtu_lsu_data_trig_en`
(1356). **Store commit suppression** (3049-3057): the ST_REPLY store
commit is gated by `!dc_store_cancel_r`. **ldst verdict to RTU**
(3164-3181): `lsu_rtu_ex1_halt_info` live leg =
`trig_fault_issue ? ag_halt_info_buf : (misalign || mmu_fault) ? ... :
dc_halt_info_r` (3177-3181 — donor `aq_lsu_ag.v:1698`). The trigger
fault also raises `lsu_rtu_expt_vld` (3245-3246) with
`lsu_rtu_expt_vec = 5'd3` (breakpoint, 3269-3270) and
`lsu_rtu_tval = 64'd0` (3278 — a trigger breakpoint carries no tval).

**IDU** (`rtl/IDU.v`): `ifu_idu_id_halt_info` input (155), latched to
EX1 (2079-2083, 2190), output `idu_iu_ex1_halt_info` (2316) — the
execute-verdict pipe. **dret decode** (865-873): `15'b011110100011100`
→ `CP0_FUNC_DRET` (20'h00202, rvproc_pkg.sv:545), **illegal unless
`rtu_yy_xx_dbgon`** (872-873). `rtu_yy_xx_dbgon` input (378).

**RVProc** (`rtl/RVProc.v`): DM-side DTU ports (137-157), debug net
declarations (579-657), `DTU u_dtu` instance (1688, connected
1722-1752), and the **T8b BUG-2 wiring** `.idu_lsu_ex1_halt_info
(idu_iu_ex1_halt_info)` (1278).

**RVProcAXI** (`rtl/RVProcAXI.v`): 4 JTAG pads (113-119 inputs
`jtag_tck/tms/tdi/tdt_rst_n`, 164-170 outputs
`jtag_tdo`/`ndmreset_n`/`hartreset_n`). **`N_MASTERS = 3`** (261-264): crossbar masters m[0]=ICache,
m[1]=DCache, **m[2]=SBA** (D-M7-3). `TDT_DTM u_tdt_dtm` (432),
`TDT_DM u_tdt_dm` (452), `SBA_AxiUp u_sba_axiup` (523).

### 2.6 The C++ debug host

`dut.cpp` (66-107) is the JTAG clock driver: `JTAG::cycle(tms, tdi,
core_tick)` — TMS/TDI settled (71-73), **4 clk edges low phase**
(79-80), TCK rise (83-84), **4 clk edges high phase** (88-89), TCK
fall (92-93), **TDO sampled after the falling-edge eval** (96,
negedge-registered per TDT_DTM.v:256-263). `clk` and `tck` are never
toggled in the same eval; the `core_tick` callback advances the core
during the scan so a halted core's clock (and the DM's) keeps running.
This is exactly the donor `tdt_dmi_define.h:301-309` contract at
clk:tck = 8:1. `run_to_idle()` (99-107) TMS=1 until IDLE; reset pads
(117-129) set `tdt_rst_n=1` (JTAG runs after reset release).

`RVProcTest.cpp` is the DMI client (port of the donor `JTAG_DRV.vh`
ext_debug class, header 131-158 documents the mapping: IR codes
142-145, DMI ops 144-145, DR layout 146-150, poll/timeout 151-155, DM
offsets 157-158). `M7JTAG` (199-615): `tlr()` (209), `write_ir` (220),
`shift_dr` (235), `dmi_rw_check` with the 30-round `M7_MAX_POLL` busy
poll (264-279, constant at 171 — the "abstractcmd busy timeout" fail
marker), `abstract_cmd` (318), `abstract_reg_read/write` (346/365),
`execute_itr` (405), `halt_req`/`resume_req`/`wait_halted`/
`wait_running` (466/485/498/512), `sbcs_wr`/`sba_wr`/`sba_rd`
(532/579/600).

**Halt/resume/step flow** — `m7_debug_smoke` (1313-1402) runs the base
6 steps then the extended e2e: (1) TLR, TDO=1 (1320-1324); (2) IDCODE
0x1000_0B6F (1326-1332); (3) DTMCS version=1 abits=10 (1334-1346); (4)
dmactive=1 + readback (1348-1359); (5) dmstatus version=2 anyhalted=0
(1361-1374); (6) dmactive=0 (1376-1385); then
`m7_debug_extended` (666-971): (0) dmactive re-enable after the base
smoke's clear (700-708); (c1) **cmderr=4** — abstract GPR read while
RUNNING is refused (710-733); (a) **halt** — haltreq →
`dcsr.cause=3` (dm_sync — the DM-initiated halt cause per the donor's
cause table, clone-faithful; NOT 1, see the note at 661-664), dpc in
the loop range, x8 intact (735-759); (b) **abstract GPR r/w** — x8
read 0x5A5A0000, 64-bit x9 write via DATA0+DATA1 (761-778); (c2)
**cmderr=2** — unsupported cmdtype (780-799); (d) **ITR** —
`addi x13,x13,1`, dpc unchanged (801-820); (e1) **progbuf raw r/w**
byte-exact (822-840); (e2) **progbuf execution NOT exercised**
(842-861 — Verilator "Active region did not converge" on the real
core, reported not fixed, §4); (f) **step** — exactly one
instruction, `dcsr.cause=4`, x10 +1 (863-893); (g) **resume** via
`resumereq` (895-915); (h) **dret** via ITR `0x7B200073` (917-946);
then the trigger e2e (952) and the SBA e2e (967-968). Exit prints the
TCK total (1393-1396) and **M7-DEBUG-PASS** (1397-1400).

**Trigger e2e** — `m7_debug_triggers` (1017-1302; doc block 973-1016)
against the spin ELF (`test/m7/directed/debug_spin.S`: RVC-free
`.option norvc`, LOOP=0x80000040 `addi x10,x10,1`, LOAD_PC=0x80000044
`lw x14,0(x12)`, STORE_PC=0x80000048 `sw x15,0(x12)`, j LOOP at
0x8000004C, handler=0x80000050 `csrr x17,mepc; addi x17,x17,4;
csrw mepc,x17; mret` — the handler skips the triggered instruction
and does not disarm the trigger, so it keeps firing each lap;
B=0x80001000, C=0x11111111): (i) **execute trigger action-1**
(tdata1=0x2000000000001044, tdata2=LOOP) → halt at LOOP,
`dcsr.cause=2`, bit-exact tdata1 readback (1050-1096); (ii)
**execute trigger action-0** (0x2000000000000044) → **breakpoint
trap**, `mcause=3`, `mepc` in {LOOP, LOOP+4} (the trap is at the
trigger PC; the handler's skip is why the running value oscillates —
doc note 1010-1015) (1098-1124); (iii) **load trigger action-0**
(0x2000000000000041, tdata2=0xFFFFFFFF80001000 — the sign-extended PA,
bit 39 set, 1023-1028) → trap, x14 stays at its pre-load value W
(0x22222222): the load is cancelled (1126-1173); (iv) **store
trigger action-0** (0x2000000000000042) → trap, B stays C: the store
is suppressed (the ST_REPLY gate, LSU.v:3049-3057) (1175-1224); (v)
**second trigger** via `tselect=1` (1226-1302). Every action-0 step
sets `tcontrol.MTE=1` first (DTU.v:754).

**SBA e2e** — `m7_debug_sba` (1498-1802; doc block 1404-1497): (o0)
halt precondition (1523-1536); (o) **sbcs readback** — version=1,
access=2, size=40, info=5'b11100, all-else-0 (1538-1567); (p1)
**32-bit** r/w @0x80002000 P32=0xDEADBEEF (1569-1583); (p2) **64-bit**
@0x80002010, 8 distinct bytes 0xA0..0xA7 (1585-1600); (p3) **128-bit**
@0x80002020, 16 distinct bytes 0x00..0x0F (1602-1617); (p4)
**byte-lane proof** — 32-bit clobber of the low word, high 96 bits
intact (1619-1640); (q) **tohost uncached readback** 0x77777777
@0x7FFFF000 (1642-1659 — the SBA bypasses the DCache, so the prologue's
store is bit-exact); (q2) word B readback (1661-1691 — phase-dependent
dirty-line value, §4); (r1) **sberror=4 unsupported access** —
`sbaccess=1` → the attempted 32-bit write is ignored, sberror=4, and
the sbaccess field latches the raw value (1693-1741, TDT_DM.v:1007);
(r2) **sberror=3 unaligned** — 128-bit @A128+8 → sberror=3, the 128-bit
window unchanged (1743-1797). **Core left HALTED** at exit
(1799-1800).

### 2.7 Verification

- **Unit** — `test/m7/unit/` (`dm_tb.cpp`: TDT_DM alone, C++ APB
  master + fake core + fake AXI slave; `dtm_tb.cpp`: TDT_DTM alone,
  two clocks, TAP-5 walk T1-T10); `make -C test/m7/unit run` →
  UNIT-SUITE-PASS. `test/m2/unit/dtu_tb.cpp` (DTU.v standalone
  white-box) is in the 10-bench suite; `make -C test/m2/unit run` →
  UNIT-SUITE-PASS.
- **e2e** — `bash test/m7/run_debug.sh` (300 s cap, `--m7-debug` on
  the spin ELF) → **M7-DEBUG-PASS**: steps (i)-(v) all ok, SBA o0-r
  all ok (12 ok lines), core left HALTED, 58540 TCK, zero
  "abstractcmd busy timeout", sim exit=0 (controller-verified on
  HEAD=ceb6834).
- **Directed** — `test/m7/directed/run_directed.sh`: m7-break_no_skip
  PASS (tohost=1); m7-debug_spin FAIL **by design** (the spin ELF
  never writes tohost=1 — it is the JTAG host's target, not a tohost
  test; annotated in the runner, not gated).
- **Re-baseline** — rv64mi-p-breakpoint (M4 directed) PASS at the
  **1026-cyc** baseline, restored by the T8b word-align omission (§4).

## 3. C906 file cross-reference

| rv906 file / section | Donor C906 (refs/openc906/C906_RTL_FACTORY) |
|---|---|
| `rtl/TDT_DTM.v` TAP-5 FSM (83-179) | `cpu/rtl/tdt/tdt_dtm_ctrl.v:93-208` (FSM), `:213-218` (decode); `tdt_tap_defines.v:17-27` (state encodings) |
| `rtl/TDT_DTM.v` TDR chain (181-265) | `tdt/tdt_dtm_chain.v:42-48` (opcodes), `:56-86` (shift mux), `:90-92` (IDLE), `:105-122` (TDO negedge) |
| `rtl/TDT_DTM.v` TDR + DMI engine (267-440) | `tdt/tdt_dtm_idr.v:45-47` (IDCODE/DTMCS/IDLE const), `:69-208` (DMI req engine) |
| `rtl/TDT_DTM.v` DMI→APB bridge (442-644) | `tdt/tdt_apb_master.v:55-71` (ports), `:72-100` (pulse sync), `:102-162` (APB FSM), `:164-251` (paddr/prdata) |
| `rtl/TDT_DTM.v` pulse-sync / sync-dff (648-743) | `tdt/tdt_dmi_pulse_sync.v`, `tdt/tdt_dmi_sync_dff.v` (inlined) |
| `rtl/TDT_DTM.v` timing contract (header 1-49) | `tdt/tdt_dmi_define.h:301-309` (freq ratio), `:282-285` (IR), `:287-294` (DTMCS/IDCODE) |
| `rtl/TDT_DM.v` register map (116-142) | `cpu/rtl/tdt/tdt_dm.v:171-227` (offsets) |
| `rtl/TDT_DM.v` APB decode (279-285) | `tdt_dm.v:598-609` |
| `rtl/TDT_DM.v` progbuf (290-315) | `tdt_dm.v:614-657` (storage), `:1952-1988` (engine) |
| `rtl/TDT_DM.v` dmcontrol (320-402) | `tdt_dm.v:660-836` |
| `rtl/TDT_DM.v` dmstatus (417-438) | `tdt_dm.v:890-969` |
| `rtl/TDT_DM.v` reset outputs (445-462) | `tdt_dm.v:1226-1288` |
| `rtl/TDT_DM.v` halt/resume (467-482) | `tdt_dm.v:1486-1562` |
| `rtl/TDT_DM.v` abstract engine (487-666) | `tdt_dm.v:1688-2061` |
| `rtl/TDT_DM.v` REGACC FSM (668-756) | `tdt_dm.v:2064-2181` |
| `rtl/TDT_DM.v` ITR/DCC (796-844) | `tdt_dm.v:2182-2446` |
| `rtl/TDT_DM.v` data/hartsum (858-895) | `tdt_dm.v:2448-2525` |
| `rtl/TDT_DM.v` SBA registers (900-1106) | `tdt_dm.v:2741-3130` |
| `rtl/TDT_DM.v` SBA AXI master (1113-1259) | `tdt/tdt_sba_axi.v:72-342` |
| `rtl/TDT_DM.v` APB read mux (1282-1313) | `tdt_dm.v:3364-3470` |
| `rtl/SBA_AxiUp.v` (whole) | **no donor** — rv906-specific 128→512 up-converter (the donor SBA runs 128-bit AXI against the C906 fabric) |
| `rtl/DTU.v` dcsr/dpc/dscratch (176-301) | `cpu/rtl/dtu/aq_dtu_ctrl.v:296-420` |
| `rtl/DTU.v` havereset FSM (417-443) | `aq_dtu_ctrl.v` (havereset/dm_core_rstn) |
| `rtl/DTU.v` trigger storage (451-632) | `aq_dtu_m_iie_all.v:375-611` (tselect/storage); `aq_dtu_mcontrol.v:480-536` (tdata1 WARL) |
| `rtl/DTU.v` comparators (676-787) | `aq_dtu_mcontrol.v:711-1330` |
| `rtl/DTU.v` sign-extension (707-710) | `aq_dtu_mcontrol.v:1104-1105` (exe), `:1236-1237` (ldst) — the active lines |
| `rtl/DTU.v` iie/icount (852-868) | `aq_dtu_iie_trigger.v:174-184`, `:432-433` |
| `rtl/DTU.v` halt_info assembly (806-850) | `aq_dtu_mcontrol_output_select.v:3380-3387` (exec), `:2968-2984` (ldst/store-cancel) |
| `rtl/RTU.v` halt entry (1057-1176) | `cpu/rtl/rtu/aq_rtu_retire.v:606-682` (dbg mode), `:755-760` (after_req), `:768-795` (cause), `:905-913` (req) |
| `rtl/RTU.v` trigger trap legs (955-979) | `aq_rtu_retire.v:692-700` (bkpt legs), `:447,:455,:564-577` (epc/tval) |
| `rtl/RTU.v` exit/debug (1139-1162, 1280-1291) | `aq_rtu_retire.v:1157` (trap gate), `:1194` (async tied 0), `:1200-1223` (exit), `:1255` (dbgon) |
| `rtl/CSR.v` cp0↔dtu (297-320, 1354-1379) | `aq_dtu_top.v:15-210` (port list); `aq_cp0_iui.v:523-533` |
| `rtl/CSR.v` ecall/mret/sret/wfi/dret gates + ebreak-halt (367-377, 442, 1810-1811) | `aq_cp0_iui.v:816` (wfi_wake), `aq_cp0_trap_csr.v:585-631` |
| `rtl/CSR.v` exit-debug privilege (616-624) | `aq_cp0_trap_csr.v:1422` (dtu write) |
| `rtl/IFU.v` debug inject + fetch mask (521-525, 1073-1124) | `aq_ifu_ibuf.v:949-953` (debug instr), `:1085` (halt_on_reset), `:1098` (exec verdict), `:1354-1357` (pop) |
| `rtl/LSU.v` ldst feed + cancel (679-701, 1182-1248) | `aq_lsu_ag.v:899-901` (halt_info buf), `:1372-1374`, `:1432` (dt_cancel) |
| `rtl/LSU.v` ldst verdict + trigger expt (3164-3278) | `aq_lsu_ag.v:1698` |
| `rtl/IDU.v` dret decode (865-873) | `aq_idu_id_decd.v:857` (legality), `:2110-2114` (decode) |
| `rtl/IDU.v` ex1 halt_info pipe (2079-2316) | `aq_idu_id_dp.v:433-454`, `:1099-1100`, `:1117-1118` |
| `rtl/RVProcAXI.v` JTAG pads + DTM/DM/SBA (46-71, 432-523) | `tdt/tdt_top.v:74-75`, `:179-186` (JTAG/DM top wiring) |
| `dut.cpp` JTAG clocking (66-107) | `smart_run/JTAG_DRV.vh:559-729` (clocking + TAP seq), `:458-469` (IR/op) |
| `RVProcTest.cpp` M7JTAG + e2e (199-615, 666-1802) | `JTAG_DRV.vh:502-503` (offsets), `:591-729` (TAP), `:936-1143` (ext_debug: halt/GPR/ITR/progbuf/step/resume/dret), `:1040-1143` (poll constants) |

## 4. Design discussion: deviations, findings, and what stayed unbuilt

### 4.1 The M7 deviation ledger (complete)

**1. T8b BUG-1 fixes = two DONOR DEVIATIONS.** The donor C906 has
**no** `dbgon` gate in the fetch/issue/commit path — verified across
`aq_lsu_ag.v` / `aq_lsu_dc.v` / `aq_rtu_wb.v` / `aq_idu_id_ctrl.v` (the
donor simply never dispatches user-mode memory ops once halted, by
construction of its out-of-order pipeline; a faithful in-order clone
does not inherit that for free). A halted rv906 hart would otherwise
keep committing user-mode loads/stores — a debug-spec violation the
donor also exhibits but rv906 closes:
- **(a) `rtl/IFU.v:1094`** — `ifu_idu_id_inst_vld = (pop_entry_vld &&
  !rtu_yy_xx_dbgon) || dtu_dbg_inst_deliver`. **Load-bearing**: without
  the `!rtu_yy_xx_dbgon` term, the IDU dispatches the refetched burst
  during the halt window, creating a WBT entry the halted LSU never
  writes back, and every subsequent ITR times out (the "abstractcmd
  busy timeout" that was the M7 Task 8 symptom).
- **(b) `rtl/LSU.v:1190`** — `issue_real` gains `&& !rtu_yy_xx_dbgon`.
  Defense-in-depth: even if a load/store is already in the AG when the
  halt lands, it does not issue (no memory side-effects from a halted
  hart). The rationale comment is at LSU.v:261-277.

**2. T8b BUG-2 = DONOR-FAITHFUL fanout restoration (not a deviation).**
The clone's LSU was missing the donor's `idu_lsu_ex1_halt_info` fanout
— the donor fans the execute-verdict to **both** cp0 and LSU
(`aq_idu_id_dp.v:1099-1100` and `:1117-1118`); rv906's LSU only got the
DTU's live ldst verdict, so an execute-trigger match on a load/store
PC never reached the LSU to cancel the load. Restored:
- `rtl/LSU.v:138` — `idu_lsu_ex1_halt_info` port;
- `rtl/RVProc.v:1278` — wiring `.idu_lsu_ex1_halt_info(idu_iu_ex1_halt_info)`;
- `rtl/LSU.v:1240` — `ag_halt_info_buf` (DTU-live if valid, else the
  IDU latched verdict — mirror of donor `aq_lsu_ag.v:899-901`);
- `rtl/LSU.v:1247` — `trig_fault_issue` = CANCEL bit alone (donor
  `ag_pipe_dt_cancel = ag_pipe_halt_info[TDT_HINFO_CANCEL]`,
  `aq_lsu_ag.v:1432`);
- `rtl/LSU.v:1355` — `dc_halt_info_r` latch (reply path);
- `rtl/LSU.v:3177` — `lsu_rtu_ex1_halt_info` live leg (donor
  `aq_lsu_ag.v:1698`; RTU ORs the two legs, donor `aq_rtu_dp.v:399-401`).

**3. DTU trigger-address SIGN-extension restored.**
`rtl/DTU.v:707-710` — the clone had zero-extended
`{24'b0, addr}`; the donor sign-extends
(`aq_dtu_mcontrol.v:1105` exec, `:1237` ldst — the **active** lines;
the zero-extend variants at `:1104`/`:1236` are commented out
upstream). With a 40-bit physical address, sign-extension is what
makes a trigger whose address has PA bit 39 set (a high-VA trigger,
e.g. the negative-address form `0xFFFFFFFF80001000` used by the M7
load/store trigger e2e) matchable.

**4. NEW deviation: the ldst-path low-4-bit WORD-ALIGN is deliberately
omitted.** `rtl/DTU.v:681-706` documents it. The donor's ldst
comparator masks the address to word alignment (`addr[39:4], 4'b0` at
`aq_dtu_mcontrol.v:1237`; the `tdata2[3:0] & {4{!match_ldst_addr}}`
data-match term at `:837`) — i.e. a word-aligned trigger matches any
byte within the word. With that mask, rv64mi-p-breakpoint regressed
1026→1049 cyc FAIL at test no.11 (bisection: sign-extension alone
PASSES; sign-extension + word-align FAILS — the mask made a
low-halfword-offset access in that test match a word-aligned trigger it
should not). M7's ldst targets are already word-aligned (RV64 loads/
stores), so the mask costs nothing on the M7 gate set; **sign-extension
alone is the minimal donor-faithful restoration** that keeps both
rv64mi-p-breakpoint (1026 cyc) and the M7 trigger e2e green.

**5. D-M7 design-time decisions that remained as-is** (from the M7
design doc §5, lines 166-228), with the ones amended by implementation
findings noted:
- **D-M7-1** single clock domain (all blocks posedge(clk); only the JTAG
  clock is async) — as designed; the pulse-sync + held-stable-payload
  contract is what makes the tck↔clk bridge legal.
- **D-M7-2** no `trst_n`, no TAP2, no `jtag2_sel`, no `tap_en` — as
  designed (TDT_DTM.v:1-49); the no-`trst_n` IDCODE-always-selectable
  behavior is compensated by the TLR arm (TDT_DTM.v:304-311).
- **D-M7-3** SBA = crossbar master #3 (N_MASTERS=3, RVProcAXI.v:261-264)
  — as designed; the 128→512 up-conversion is rv906-specific
  (SBA_AxiUp.v, no donor).
- **D-M7-4** SBA donor surface unchanged (tdt_sba_axi.v clone) — as
  designed; gated against spec 0.13 + the rv906 crossbar contract 17.
  The donor NEVER exercised SBA memory access in sim (the donor SoC
  ties the SBA port off, `tr_axi_interconnect.v:861-895`), so the SBA
  e2e is gated against spec 0.13 + the clone RTL (RVProcTest.cpp:1408-1415).
- **D-M7-5** standard 0.13 `mcontrol` tdata1 layout (T-Head custom bit
  positions dropped); T-Head custom types 4/5 (itrigger/etrigger) read
  back as type=0; **amended**: `tinfo` reduced to 0x10 for mcontrol
  (donor 0x30 advertised `mask`/`snapshot`/`data` extensions rv906
  doesn't implement — DTU.v:666-669, 901-903).
- **D-M7-6** T-Head CSRs dropped: `cuscs`/`cuscmd`/`cusbuf` (0x70-0x79)
  read 0 / writes ignored; DTU 0xFE0-0xFE2 (haltcause/dbgfifo/pcfifo)
  absent; `wr_flg` 10/11 (latest_pc/satp) dropped (DTU.v:383-384,
  TDT_DM.v header) — as designed.
- **D-M7-7** no custom async halt (the donor's `halt_req_dm_async` is
  tied 0, RTU.v:1194; the dead-but-present shortcut at RTU.v:1245 is
  kept for donor-diffability) — as designed.
- **D-M7-8** single-issue DTU: IFU fetch slot 1 tied, `pending_tval`
  tied 0 (DTU.v:324) — as designed; **amended**: the iie/icount
  comparator hardwires count=1 (DTU.v:853-856) and the M7 gate set
  never arms a type-3 slot, so the icount path is structurally live
  but unexercised (a finding, not a deviation — the donor's icount>1
  multi-instruction counter is the thing that stayed unbuilt, §4.3).
- **D-M7-9** no DM clock gating (`dmactive=0` → `sync_rst`,
  TDT_DM.v:398) — as designed; the DM is held in reset, not
  clock-gated.
- **D-M7-10** constants kept (JEP106_ID 12'hB6F, DM_VERSION=2,
  xdebugver=4'b0100, IDCODE 0x1000_0B6F, abits=10, IDLE=7) — as
  designed.
- **Task-1 cause-plumbing note** (DTU.v:21-31): Task 1 used a direct
  4-bit `rtu_dtu_halt_cause` export; the final form keeps that direct
  cause (RTU.v:1415) **and** the real 22-bit `halt_info` bundle for
  the trigger/pending path (RTU.v:1426) — both are donor-faithful
  (`aq_dtu_top.v` carries both `dtu_rtu_halt_cause` and
  `dtu_rtu_retire_halt_info`).

### 4.2 Findings (implemented and gated)

- **`(e2)` progbuf execution against the real core is unbuilt**
  (RVProcTest.cpp:842-861): driving a progbuf via a postexec GPR write
  aborts Verilator with "%Error: Active region did not converge" (a
  combinational loop) on the REAL core — for both a memory pb and a
  pure-ALU pb. Classification in the comment: driver-side NO (the
  single-ITR path passes, the pb address is valid), RTL-side YES (the
  DM pb engine is unit-verified in isolation; the loop is in the
  real-core interaction with `pb_work` mode). Reported, not fixed —
  no RTL edits in M7 Task 7. Progbuf **raw r/w** (e1) is gated.
- **`(q2)` SBA word-B read is phase-dependent** (RVProcTest.cpp:1661-1691):
  reading the loop's load/store word B via the SBA while the core is
  halted returns 0x0 for this ELF — the loop's B stores live DIRTY in
  the write-back DCache and have not reached the system bus by (o0);
  the SBA (crossbar master #3) bypasses the DCache, so it reads the
  ExtMem value. With the committed Task 8 spin ELF prologue the same
  read returns C (the 9-lap loop-phase shift flips the DCache
  victim-writeback timing) — flagged as a Class-B candidate;
  EXPECT_B pins this ELF's observed value. The uncached tohost word
  ((q)) is the phase-independent proof that SBA sees the same memory
  the core wrote.
- **`sberror` latches the raw `sbaccess` on error** (TDT_DM.v:1007):
  donor-faithful — spec 0.13 codes 4 as "Halt"/2 as "NotSupported" and
  would clear sbaccess on a failed access, but the donor latches the
  requested width. The e2e (r1) asserts the donor behavior (register
  latches the raw value; RVProcTest.cpp:1693-1741, citing
  tdt_dm.v:2854,2907-2908).
- **`(c1)` DM-initiated halt cause is 3 (dm_sync), not 1**
  (RVProcTest.cpp:661-664): the donor's cause table
  (aq_rtu_retire.v:768-795, cloned at RTU.v:1125-1137) encodes a
  dmcontrol.haltreq halt as cause=3; 1 is reserved for ebreak.
  Clone-faithful.

### 4.3 What stayed unbuilt (deliberate)

- **iie/icount > 1** — the donor's multi-instruction icount counter
  (type 3, count up to 2^10) is not built; the comparator matches on
  count=1 (DTU.v:853-856) and the gate set never arms a type-3 slot.
  `tinfo` for iie still reads 0x30 (the slot exists, the count doesn't
  vary).
- **`mask`/`snapshot`/`data` mcontrol extensions** — the reduced
  `tinfo`=0x10 advertises they are absent (D-M7-5 amendment).
- **MPTE / mret privilege save-restore** — `tcontrol.MPTE` is stored
  but not gated (DTU.v:634-638); the mret save/restore path is dropped.
- **`mcontext`/`scontext` matching** — storage only, no chain match
  (single hart).
- **Donor leg-1 pending-breakpoint form** — `retire_pending_bkpt_expt`
  stays 0 in rv906 (RTU.v:961); the pending path is the DTU
  `dtu_rtu_pending_halt` level honored as a t1 halt (RTU.v:1113).
- **Timing-1 execute path** — the `dc_halt_info_r` latch (LSU.v:1355)
  exists for a future timing-1 ldst verdict at retire; the current
  timing-1 path is the pending-halt record (DTU.v:870-888) only.
- **LFB-deferred load timing-1 matches** — a timing-1 trigger match on
  a load that is still in the LFB at the match is not carried to the
  LFB entry (LSU.v:3173-3175 documents the basic gap).
- **T-Head custom surface** — types 4/5, cuscs/cuscmd/cusbuf,
  haltcause/dbgfifo/pcfifo, `wr_flg` 10/11 (D-M7-5/6).
- **`trst_n` / TAP2 / `jtag2_sel` / `tap_en`** (D-M7-2).
- **Async halt** (D-M7-7) — tied 0.
- **DM clock gating** (D-M7-9) — held in reset instead.
- **`ndmreset_n` / `hartreset_n`** — dangling as chip outputs (the
  rv906 SoC has no reset fabric to wire them into yet).
- **progbuf execution e2e** (the (e2) Verilator combinational loop,
  §4.2).
