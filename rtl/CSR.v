//=============================================================================
// CSR.v - minimal M-mode CP0/CSR file  (M2 SKELETON: ports frozen)
//=============================================================================
// C906 files covered (real body arrives in plan Task 2; this file freezes
// the port list only):
//   gen_rtl/cp0/rtl/aq_cp0_top.v       (aq_cp0_iui + aq_cp0_regs + aq_cp0_
//                                        special glue)
//   gen_rtl/cp0/rtl/aq_cp0_iui.v       (CSR RMW: csrrw/csrrs/csrrc(+I))
//   gen_rtl/cp0/rtl/aq_cp0_regs.v      (CSR address decode/wiring hub)
//   gen_rtl/cp0/rtl/aq_cp0_trap_csr.v  (mstatus/mie/mip/mtvec/mscratch/
//                                        mepc/mcause/mtval)
//   gen_rtl/cp0/rtl/aq_cp0_info_csr.v  (mvendorid/marchid/mimpid/mhartid/
//                                        misa)
//   gen_rtl/cp0/rtl/aq_cp0_hpcp_csr.v  (mcycle/minstret -- interface shape
//                                        only; M2 implements two local free
//                                        -running counters directly here)
//   gen_rtl/cp0/rtl/aq_cp0_ext_csr.v   (MHCR/MXSTATUS -- CSR addresses and
//                                        bit layouts CONFIRMED by reading
//                                        this file + aq_cp0_regs.v:846-847
//                                        directly, see rvproc_pkg.sv)
// References: design doc S2.3.6 (contract 7, the exact minimal CSR set),
// S4.2 (cp0_rtu_t boundary struct), RTU note S6/S7 (trap-entry capture,
// mret sequencing, CSR-writeback timing), LSU/CP0 note B (the CSR set,
// what's real HW vs. derived).
//
// SCOPE NOTE (contract 7): mstatus (only MIE/MPIE real, MPP tied 2'b11 RO),
// mtvec (direct mode only), mepc (LSB forced 0), mcause, mscratch, mtval
// (vec allowlist {1,2,4,5,6,7,12,13,15} only), mie/mip (mip read-only wires
// from mtip/msip/meip), misa (RO: MXL=64, I|M|C|F|D), mvendorid/marchid/mimpid/
// mhartid (hardwired), mcycle/minstret (two local free-running counters).
// Everything else is ABSENT, not stubbed -- no satp/PMP/fcsr/vector-CSR/S-
// mode ports exist on this module at all.
//
// KNOWN, DELIBERATE GAP (not a freeze violation -- mirrors M1's BPU.v
// Task 7 amendment precedent): minstret's retire-commit pulse input is NOT
// yet on this port list. The plan's own Task 1.1 CSR.v bullet says to pick
// "rtu_hpcp_*-shaped retire-count" OR "a direct rtu_idu_wb0/1_vld-style
// commit pulse... confirmed in Task 4, not guessed here." RTU.v's real body
// (Task 4) resolves which signal actually exists and this port list gains
// exactly one new input then -- an anticipated, documented amendment, not a
// silent one. Task 2's own body (below) therefore implements minstret's
// storage/R/W path for real but does NOT auto-increment it yet -- see the
// "MCYCLE / MINSTRET" section for the exact, documented interim behavior.
//
// TASK 2 DISCOVERED GAP, FIXED HERE (same "documented amendment, not silent"
// discipline as the minstret note above, and the same precedent as commit
// fa74d0b's missed `idu_lsu_ex1_sel` port): Task 1's port list had NO way
// for CSR.v to expose `mtvec`'s value to RTU at all. Real C906 needs this
// every cycle a trap is taken (RTU note S5/S6: `cp0_rtu_trap_pc[39:0]`,
// read combinationally by RTU's retire stage independent of CP0's own EX1
// completion pulse -- confirmed directly, aq_cp0_trap_csr.v:1396
// `cp0_rtu_trap_pc[39:0] = regs_trap_pc[39:0]`) -- without it, NO trap could
// ever redirect anywhere once RTU.v (Task 4) is built. Unlike the minstret
// gap, this one was never flagged as deliberately deferred, so Task 2 adds
// the output port now (`cp0_rtu_trap_pc` below) rather than leaving a
// bring-up-ladder-breaking hole for Task 4 to discover the hard way. RTU.v
// itself does not exist yet (Task 4), so nothing consumes this port today --
// it is wired for the first time when RTU.v's real body is built. Flagged
// prominently in the Task 2 completion report; not a silent scope creep.
//=============================================================================

import rvproc_pkg::*;

module CSR #(
    // Boot PC, sourced "for real" from mrvbr (this header's own pre-
    // existing note above `cp0_xx_mrvbr`) -- mirrors FetchSink.v's own
    // RESET_VECTOR parameter name/type/default exactly (FetchSink.v:129) so
    // RVProc.v's eventual rewire (replacing FetchSink with CSR+real units)
    // can pass the same top-level parameter straight through unchanged.
    parameter [63:0] RESET_VECTOR = 64'h0000_0000_8000_0000
)(
    input  wire                     clk,
    input  wire                     rst_n,

    //=========================================================================
    // IDU -> CSR : EX1 dispatch, CSR's slice of the shared id_ex1_t payload
    // (design doc S4.2; IDU note S8 -- CSR is a peer EU dispatch target,
    // not something IDU owns). CSRRW/S/C(+I) carry the CSR address in
    // src1_data (I-type imm12 slot, IDU note S8); ECALL/EBREAK/MRET/FENCE/
    // FENCE.I carry no register operands, only `func`.
    //=========================================================================
    input  wire                     idu_cp0_ex1_sel,       // ctrl.v:632 idu_cp0_ex1_sel
    input  wire [FUNC_WIDTH-1:0]    idu_cp0_ex1_func,
    input  wire [31:0]              idu_cp0_ex1_opcode,
    input  wire                     idu_cp0_ex1_illegal,   // IDU_ILLEGAL slice
    // M4 Task 6 (S13, D2): fetch-fault siblings, IDU.v's own new ports --
    // win priority over ex1_illegal below (donor aq_cp0_iui.v:645-666:
    // pgflt(12) > accflt(1) > illegal(2)).
    input  wire                     idu_cp0_ex1_fetch_pgflt,
    input  wire                     idu_cp0_ex1_fetch_accflt,
    input  wire [63:0]              idu_cp0_ex1_src0_data, // rs1 (CSRRW/S/C only)
    input  wire [63:0]              idu_cp0_ex1_src1_data, // CSR address
    input  wire [GPR_IDX_WIDTH-1:0] idu_cp0_ex1_dst0_reg,  // rd (old CSR value)
    // Task 7.3: the EX1 instruction's length (1=32b,0=16b RVC; c.ebreak is
    // 16-bit) for the CSR slice -- feeds the RTU pcgen inst_len mux
    // (aq_rtu_dp.v:371 cp0 arm).
    input  wire                     idu_cp0_ex1_inst_len,
    // CSR -> IDU : FENCE/FENCE.I EX1-hold backpressure (Task 10.1,
    // rv64ui-p-fence_i): high while the fence waits for LSU quiescence;
    // IDU folds it into ctrl_ex1_eu_full so EX1 keeps the fence until
    // completion is allowed (SECTION DECODE note).
    output wire                     cp0_idu_fencei_full,

    //=========================================================================
    // IU -> CSR : the only IU<->CP0 connection besides config (IU note
    // S4.5/S9) -- a plain current-PC passthrough CP0 needs for trap-context
    // bookkeeping. Not re-listed in the plan's own CSR.v bullet text, but
    // required for IU.v's `iu_cp0_ex1_cur_pc` output (Task 1.2's IU.v
    // bullet) to have a consumer -- added here with this note rather than
    // silently matched.
    //=========================================================================
    input  wire [PC_WIDTH-1:0]      iu_cp0_ex1_cur_pc,

    //=========================================================================
    // CSR -> RTU : cp0_rtu_t (design doc S4.2) -- EX1 completion (old-CSR-
    // value writeback, one-hot completion source) + the trap-declaration
    // bus (RTU note S4/S7).
    //=========================================================================
    output wire                     cp0_rtu_ex1_cmplt_dp,  // one-hot cmplt source
    // Task 7.3: the completing CSR instruction's length, for the RTU pcgen
    // inst_len mux (donor aq_rtu_dp.v:371 cp0 arm).
    output wire                     cp0_rtu_ex1_inst_len,
    output wire [63:0]              cp0_rtu_ex1_wb_data,
    output wire [GPR_IDX_WIDTH-1:0] cp0_rtu_ex1_wb_preg,
    output wire                     cp0_rtu_ex1_wb_vld,
    output wire                     cp0_rtu_ex1_expt_vld,
    output wire                     cp0_rtu_ex1_expt_int,
    output wire [4:0]               cp0_rtu_ex1_expt_vec,
    output wire                     cp0_rtu_ex1_chgflw,     // mret
    output wire [PC_WIDTH-1:0]      cp0_rtu_ex1_chgflw_pc,

    //=========================================================================
    // CSR -> RTU : mtvec's direct-mode redirect target, read combinationally
    // by RTU's retire stage whenever it decides to take a trap -- NOT part
    // of the EX1 completion pulse above (RTU note S5/S6, confirmed directly
    // aq_cp0_trap_csr.v:1396). See this file's header "TASK 2 DISCOVERED
    // GAP" note: added here because Task 1's frozen port list omitted it.
    //=========================================================================
    output wire [PC_WIDTH-1:0]      cp0_rtu_trap_pc,

    //=========================================================================
    // CSR -> RTU : M6 Task 1 interrupt claim export. The donor's own CSR->RTU
    // interrupt port is COMBINATIONAL (aq_cp0_trap_csr.v:1395 `assign
    // cp0_rtu_int_vld[14:0] = int_sel[14:0]`, consumed raw by aq_rtu_int.v:50
    // `int_vld_raw = cp0_rtu_int_vld`). rv906 instead follows the REGISTERED,
    // ACTIVE-LOW export of sibling rv12 (rtl/CSR.v:2300-2321, registered
    // `cp0_rtu_xx_int_b` + registered vec) -- the proven clone of this exact
    // donor network: every source here is level-based (mip pins / CSR flops
    // persist until serviced), so a one-cycle registration costs latency
    // without losing a claim, and it keeps the CSR->RTU boundary glitch-
    // clean. RECORDED DEVIATION from the donor's combinational shape (M6
    // design doc Task-1 row). DARK UNTIL M6 TASK 2: no consumer exists yet --
    // RTU.v's int_vld_raw stays 15'd0 and RVProc.v sinks these into
    // _unused_ok until Task 2 threads them into the live priority encoder.
    //=========================================================================
    output wire [14:0]              cp0_rtu_int_sel,   // donor int_sel[14:0], registered
    output wire                     cp0_rtu_int_b,     // registered active-low == !|int_sel

    //=========================================================================
    // RTU -> CSR : trap-entry capture (RTU note S7 -- mepc/mcause/mtval
    // written directly off RTU's exception-priority decision, no RTU-side
    // buffering) + the broadcast flush pulses CP0 also listens on.
    //=========================================================================
    input  wire                     rtu_yy_xx_expt_vld,
    input  wire                     rtu_yy_xx_expt_int,
    input  wire [4:0]               rtu_yy_xx_expt_vec,
    input  wire                     rtu_yy_xx_flush_fe,
    input  wire                     rtu_yy_xx_flush,
    input  wire [PC_WIDTH-1:0]      rtu_cp0_epc,
    input  wire [63:0]              rtu_cp0_tval,
    // M4 Task 1: one pulse per architecturally-retiring instruction, feeding
    // minstret's auto-increment (this file's header "KNOWN, DELIBERATE GAP"
    // note is discharged here; RTU.v exposes ex2_retire_vld, wired by RVProc).
    input  wire                     rtu_cp0_inst_retire,
    // M5 Task 8 (D7): the retiring FP op's accrued flags + the
    // FP-instruction-retire pulse, both EX2-registered in RTU.v (donor-
    // named: aq_rtu_wb.v:268-269 feeds `rtu_cp0_fflags[_updt]`,
    // aq_rtu_rbus.v:514-516 feeds `rtu_cp0_fs_dirty_updt`; consumed here
    // exactly as the donor consumes them in aq_cp0_float_csr.v:234-238 and
    // aq_cp0_trap_csr.v:562-569).
    input  wire [4:0]               rtu_cp0_fflags,
    input  wire                     rtu_cp0_fs_dirty_updt,

    //=========================================================================
    // CSR -> FPU (M5 Task 11 BUG 2): frm CSR read-out for dynamic rounding.
    // RISC-V: an FP instruction's rm=111 (DYN) resolves to frm; FPU.v does
    // the resolution at its own single rm_eff point (rv12/rtl/FPU.v:297's
    // own `cp0_fpu_frm` consumer pattern), this port just exposes frm_reg.
    //=========================================================================
    output wire [2:0]               cp0_fpu_frm,

    //=========================================================================
    // CSR -> IFU/ICache/BPU : MHCR fan-out, replacing FetchSink's harness
    // config bank (design doc S2.3.6's "open integration item," S8). Port
    // names/widths match ICache.v's/BPU.v's ALREADY-FROZEN-SINCE-M1 input
    // ports exactly (icache.v/bpu.v headers) -- these are the same wires
    // RVProc.v currently drives from FetchSink; Task 7 re-points them here.
    //=========================================================================
    output wire                     cp0_ifu_icache_en,
    output wire                     cp0_ifu_iwpe,
    output wire                     cp0_ifu_icache_pref_en,
    output wire [63:0]              cp0_ifu_icache_inv_addr,
    output wire                     cp0_ifu_icache_inv_req,
    output wire [1:0]               cp0_ifu_icache_inv_type,
    input  wire                     ifu_cp0_icache_inv_done,
    output wire                     cp0_ifu_bht_en,
    output wire                     cp0_ifu_btb_en,
    output wire                     cp0_ifu_ras_en,
    output wire                     cp0_ifu_bht_inv,
    output wire                     cp0_ifu_btb_clr,
    input  wire                     bht_cp0_inv_done,

    //=========================================================================
    // CSR -> LSU/DCache : MHCR.de/wa + MXSTATUS.mm (design doc S2.3.3/
    // S2.3.6; LSU note B2 -- "every load/store forced to miss until boot
    // code sets de=1", the same pattern ICache.v's M1 `icache_en` gate has).
    //=========================================================================
    output wire                     cp0_lsu_dcache_en,
    output wire                     cp0_lsu_mm,
    output wire                     cp0_lsu_wa,
    // M3b Task D: MHINT D-cache prefetch controls (donor aq_cp0_ext_csr.v
    // :861,:868) routed to the LSU's PFB.
    output wire                     cp0_lsu_dcache_pref_en,
    output wire [1:0]               cp0_lsu_dcache_pref_dist,
    // M3b Task E: MHINT.amr to the LSU's AMR (ext_csr.v:881).
    output wire [1:0]               cp0_lsu_amr,

    //=========================================================================
    // M4 Task 2: PMP register interface. The pmpcfg/pmpaddr STORAGE lives in
    // rtl/PMP.v (mirroring donor aq_pmp_regs inside aq_pmp_top); CSR.v decodes
    // the addresses, strokes the writes, and reads back via pmp_*_value.
    //=========================================================================
    output wire                     pmp_cfg0_wen,
    output wire [63:0]              pmp_cfg0_wdata,
    output wire [7:0]               pmp_addr_wen,
    output wire [63:0]              pmp_addr_wdata,
    output wire [2:0]               pmp_addr_rsel,
    input  wire [63:0]              pmp_cfg0_value,
    input  wire [63:0]              pmp_addr_value,
    // current privilege mode fed to PMP (for the M-mode bypass)
    output wire [1:0]               cp0_pmp_priv_mode,

    //=========================================================================
    // M4 Task 1: privilege + MMU controls. cp0_yy_priv_mode is the current
    // privilege mode broadcast (donor aq_cp0_trap_csr.v:1390). satp routing +
    // MXR/SUM/MPRV feed the MMU/LSU (wired through RVProc; the MMU gives them
    // meaning at Tasks 3-5). cp0_rtu_trap_pc's M/S mux lives below.
    //=========================================================================
    output wire [1:0]               cp0_yy_priv_mode,
    output wire [63:0]              cp0_mmu_satp_data,
    output wire                     cp0_mmu_satp_wen,
    output wire                     cp0_mmu_mxr,
    output wire                     cp0_mmu_sum,
    // CSR -> MMU / MMU -> CSR : sfence.vma whole-TLB invalidate handshake
    // (Task 7). cp0_mmu_sfence_vld is the single-cycle launch pulse (held
    // off IDU dispatch until the STB is quiescent, see SF_IDLE/SF_WAIT
    // above); mmu_cp0_sfence_done pulses back once the invalidate has
    // actually applied (immediately if the PTW is idle, else once any
    // in-flight walk drains to PTW_IDLE -- MMU.v's tlb_inv_all site).
    output wire                     cp0_mmu_sfence_vld,
    input  wire                     mmu_cp0_sfence_done,
    output wire                     cp0_lsu_mprv,
    output wire [1:0]               cp0_lsu_mpp,
    // LSU -> CSR : store-buffer/pipe quiescence (Task 10.1): FENCE/FENCE.I
    // hold in EX1 while this is low -- stores must reach their completion
    // point before the fence's I-side invalidate (or any later observer) may
    // proceed. (RVProc.v wires LSU's quiescent output here.)
    input  wire                     lsu_cp0_stb_empty,
    // CSR -> LSU / LSU -> CSR : FENCE.I D-cache clean-walk handshake
    // (Task 10.1; donor aq_cp0_fence_inst.v FNC_CDCA stage): CSR holds
    // cp0_lsu_dcache_clean while fencei_state==FI_CLEAN; LSU walks all
    // dirty lines back to memory and pulses lsu_cp0_clean_done when done.
    output wire                     cp0_lsu_dcache_clean,
    input  wire                     lsu_cp0_clean_done,

    //=========================================================================
    // CSR -> XX : reset vector (already exists as an M1 port on RVProc.v,
    // sourced from a fixed RESET_VECTOR parameter there -- now sourced for
    // real from mrvbr).
    //=========================================================================
    output wire [PC_WIDTH-1:0]      cp0_xx_mrvbr,

    //=========================================================================
    // Interrupt pins (already piped into RVProc.v's port list since M1,
    // unconnected) -- terminate in CSR.v's mip wiring (design doc S2.1).
    // Not required for M2's rv64ui/um pass bar; wired because it is nearly
    // free given the ports already exist (contract 7).
    //=========================================================================
    input  wire                     mtip,
    input  wire                     msip,
    input  wire                     meip
);

    //=========================================================================
    // SECTION DECODE -- dispatch acceptance + the exact CP0 sub-op the
    // donor's own aq_cp0_iui.v decodes (confirmed directly,
    // aq_cp0_iui.v:462-533 + aq_idu_cfig.h:453-474 for the FUNC values
    // themselves, pinned as CP0_FUNC_* in rvproc_pkg.sv -- see that file's
    // header comment for why Task 2 pins them instead of deferring to
    // Task 5).
    //
    // `ex1_flush` mirrors the donor's own `!iui_cancel` qualifier
    // (aq_cp0_iui.v:434 `iui_inst_sel = idu_cp0_ex1_gateclk_sel &&
    // !iui_idu_expt_vld && !iui_cancel`): CP0 is the one M2 unit whose EX1
    // completion can itself declare a NEW exception/changeflow
    // (`cp0_rtu_ex1_expt_vld`/`_chgflw`), which feeds straight back into
    // RTU's own retire-priority mux and flush FSM -- unlike IU/LSU, whose
    // only EX1 output is a plain GPR writeback RTU's commit-clear logic
    // already discards on a flush. So CSR.v (unlike IU.v/LSU.v, which have
    // no `rtu_yy_xx_flush_fe`/`_flush` ports at all) locally gates its own
    // dispatch acceptance on the same two broadcast flush pulses, to avoid
    // ever declaring a second trap/changeflow the same cycle an OLDER one
    // is already being flushed.
    //=========================================================================
    wire ex1_flush  = rtu_yy_xx_flush_fe || rtu_yy_xx_flush;
    wire ex1_active = idu_cp0_ex1_sel && !ex1_flush;
    // M4 Task 6: a fetch-fault marker is excluded from ex1_ok exactly like
    // an illegal decode is -- its (legal-looking) opcode bits must never be
    // interpreted as a real ecall/ebreak/CSR access. Priority pgflt > accflt
    // (donor aq_cp0_iui.v:645-666) -- moot in practice since MMU.v only
    // ever raises one of mmu_lsu_page_fault/_access_fault per access
    // (LSU.v's own SECTION-5 comment makes the same observation), stated
    // rather than left to be discovered.
    wire ex1_fetch_pgflt  = ex1_active && idu_cp0_ex1_fetch_pgflt;
    wire ex1_fetch_accflt = ex1_active && idu_cp0_ex1_fetch_accflt && !idu_cp0_ex1_fetch_pgflt;
    wire ex1_fetch_fault  = ex1_fetch_pgflt || ex1_fetch_accflt;
    wire ex1_ok      = ex1_active && !idu_cp0_ex1_illegal && !ex1_fetch_fault;   // legal, actionable this cycle
    wire ex1_illegal = ex1_active &&  idu_cp0_ex1_illegal;

    wire is_ecall  = ex1_ok && (idu_cp0_ex1_func == CP0_FUNC_ECALL);
    wire is_ebreak = ex1_ok && (idu_cp0_ex1_func == CP0_FUNC_EBREAK);
    wire is_mret   = ex1_ok && (idu_cp0_ex1_func == CP0_FUNC_MRET);
    // M4: SRET / WFI / SFENCE.VMA (IDU decodes them as CP0 ops; privilege-
    // based legality -- TSR/TW/TVM/U-mode -- is checked below in CSR.v).
    wire is_sret   = ex1_ok && (idu_cp0_ex1_func == CP0_FUNC_SRET);
    wire is_wfi    = ex1_ok && (idu_cp0_ex1_func == CP0_FUNC_WFI);
    wire is_sfence = ex1_ok && (idu_cp0_ex1_func == CP0_FUNC_SFENCE);
    // FENCE/FENCE.I serialization sequence (Task 10.1, rv64ui-p-fence_i;
    // donor aq_cp0_fence_inst.v's FNC_FENC->FNC_CDCA->FNC_IICA ordering):
    // (1) hold in EX1 until the LSU is quiescent (`lsu_cp0_stb_empty` --
    // store buffer drained, pipe idle), i.e. every prior store has reached
    // its completion point; (2) FENCE.I only: the LSU walks the D-cache
    // writing back every dirty line (SECTION CLEAN in LSU.v) so store-hit
    // bytes reach the backing memory the ICache refills from; (3) FENCE.I
    // only: ICache INV_ALL walk (`inv_block` stalls all fetch meanwhile);
    // (4) complete with changeflow (mret-style: RTU's ex1_inst_chgflw ->
    // flush_fe + refetch at PC+4), so the first post-fence fetch reads
    // freshly invalidated state. `cp0_idu_fencei_full` stalls IDU dispatch
    // for the whole sequence so EX1 keeps the fence. This supersedes the
    // earlier M2 "no real invalidate handshake" scope note: fence_i IS in
    // rv64ui's acceptance set, and the donor C906 executes fence.i with a
    // genuine D-clean + I-invalidate.
    wire is_fence  = ex1_ok && (idu_cp0_ex1_func == CP0_FUNC_FENCE);
    wire is_fencei = ex1_ok && (idu_cp0_ex1_func == CP0_FUNC_FENCEI);
    wire fence_quiesce_wait = (is_fence || is_fencei) && !lsu_cp0_stb_empty;

    // FENCE.I clean/invalidate sequencer: FI_CLEAN runs LSU.v's D-cache
    // clean walk (cp0_lsu_dcache_clean held until lsu_cp0_clean_done
    // pulses), FI_INV runs the ICache INV_ALL walk (cp0_ifu_icache_inv_req
    // held until ifu_cp0_icache_inv_done pulses), FI_CMPLT is the single
    // completion cycle (cmplt_dp + chgflw both fire that cycle only).
    localparam [1:0] FI_IDLE  = 2'b00, FI_CLEAN = 2'b01,
                     FI_INV   = 2'b10, FI_CMPLT = 2'b11;
    reg [1:0] fencei_state;
    wire fencei_launch = is_fencei && ex1_active && !fence_quiesce_wait
                       && (fencei_state == FI_IDLE);
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) fencei_state <= FI_IDLE;
        else case (fencei_state)
            FI_IDLE:  if (fencei_launch)           fencei_state <= FI_CLEAN;
            FI_CLEAN: if (lsu_cp0_clean_done)      fencei_state <= FI_INV;
            FI_INV:   if (ifu_cp0_icache_inv_done) fencei_state <= FI_CMPLT;
            FI_CMPLT:                              fencei_state <= FI_IDLE;
        endcase
    end
    // EX1 hold: from the launch cycle through the INV walk; released at
    // FI_CMPLT so completion fires exactly that one cycle.
    wire fence_hold = fence_quiesce_wait
                    || (is_fencei && (fencei_launch || fencei_state == FI_CLEAN
                                      || fencei_state == FI_INV));

    wire is_csrrw  = ex1_ok && (idu_cp0_ex1_func == CP0_FUNC_CSRRW);
    wire is_csrrs  = ex1_ok && (idu_cp0_ex1_func == CP0_FUNC_CSRRS);
    wire is_csrrc  = ex1_ok && (idu_cp0_ex1_func == CP0_FUNC_CSRRC);
    wire is_csrrwi = ex1_ok && (idu_cp0_ex1_func == CP0_FUNC_CSRRWI);
    wire is_csrrsi = ex1_ok && (idu_cp0_ex1_func == CP0_FUNC_CSRRSI);
    wire is_csrrci = ex1_ok && (idu_cp0_ex1_func == CP0_FUNC_CSRRCI);
    wire is_csr_op = is_csrrw || is_csrrs || is_csrrc
                  || is_csrrwi || is_csrrsi || is_csrrci;

    wire csr_write_form = is_csrrw || is_csrrwi;
    wire csr_set_form   = is_csrrs || is_csrrsi;
    wire csr_clear_form = is_csrrc || is_csrrci;
    wire csr_imm_form   = is_csrrwi || is_csrrsi || is_csrrci;

    // Same three-op RMW shape as the donor (CP0 note B1, aq_cp0_iui.v:
    // 491-500): the *I forms' operand is the zero-extended 5-bit rs1/uimm
    // FIELD taken straight off the raw opcode (`opcode[19:15]`, matching
    // `iui_csr_rs1 = iui_inst_func[0] && iui_inst_func[9] ? {59'b0,
    // iui_inst_opcode[19:15]} : iui_inst_rs1` exactly); the register forms'
    // operand is the rs1 VALUE IDU already read out of the GPR file into
    // `idu_cp0_ex1_src0_data` (this file's own header comment on that port:
    // "rs1 (CSRRW/S/C only)").
    wire [63:0] csr_rs1_operand = csr_imm_form ? {59'b0, idu_cp0_ex1_opcode[19:15]}
                                                : idu_cp0_ex1_src0_data;
    wire [11:0] csr_addr = idu_cp0_ex1_src1_data[11:0];

    // RISC-V priv spec: CSRRS/CSRRC (and their *I forms) with rs1==x0 (or,
    // for the *I forms, a literal zero uimm -- same bit field either way)
    // "will not write to the CSR at all, and so shall not cause any of the
    // side effects" -- CSRRW always writes regardless. Confirmed the donor
    // implements exactly this, aq_cp0_iui.v:453,463-465: `iui_inst_rs1_x0 =
    // iui_inst_opcode[19:15]==5'b0`, gating `iui_csr_wen` for csr_func[1]/
    // [2] only. This matters beyond spec purity: without it, a pure status
    // read of a self-modifying counter (`csrrs t0, mcycle, x0`) would hit
    // this file's `mcycle_local_en` write path (wdata==rdata|0==rdata) and
    // SUPPRESS that cycle's natural +1 increment -- caught by this file's
    // own unit bench (csr_tb.cpp T20) before it ever reached Task 4.
    wire rs1_is_x0   = (idu_cp0_ex1_opcode[19:15] == 5'b0);
    wire csr_wen_raw = is_csr_op && (csr_write_form || !rs1_is_x0);
    // M4: an illegal CSR access (privilege/RO/TVM) raises an exception and
    // must NOT write (spec). csr_wen is csr_access_illegal-gated; the gate is
    // defined in the exception section below (forward ref), and csr_ro_write
    // there uses csr_wen_raw (not csr_wen) to keep the graph acyclic.
    wire csr_wen     = csr_wen_raw && !csr_access_illegal;

    // MRET/SRET/trap sequencing shares the same priority the donor's
    // mstatus/mepc/mcause/mtval always-blocks use (trap capture > xret pop >
    // software CSR write > hold) -- one flop per bit, no FSM (CP0 note B1).
    // xret_fire is gated by the privilege checks below (an illegal mret/sret
    // must NOT change pm); the illegal condition itself is raised as vec 2 in
    // the exception section. pm_r/tsr_f/tw_f/tvm_f are forward references to
    // the flops defined in the PRIVILEGE/MSTATUS sections below.
    wire mret_priv_illegal  = is_mret && (pm_r != PRIV_M);
    wire sret_priv_illegal  = is_sret && ((pm_r == PRIV_U)
                                          || (pm_r == PRIV_S && tsr_f));
    wire wfi_priv_illegal   = is_wfi  && (pm_r != PRIV_M) && tw_f;
    wire sfence_priv_illegal= is_sfence && ((pm_r == PRIV_U)
                                          || (pm_r == PRIV_S && tvm_f));
    wire xret_illegal = mret_priv_illegal || sret_priv_illegal
                      || wfi_priv_illegal || sfence_priv_illegal;
    wire mret_fire = is_mret && !mret_priv_illegal;
    wire sret_fire = is_sret && !sret_priv_illegal;

    // sfence.vma sequencer (M4 Task 7; donor aq_cp0_fence_inst.v FNC_IDLE
    // ->FNC_CMMU->FNC_IICA->FNC_CMPLT, :146-194). Donor's FNC_CMMU asserts
    // special_fence_mmu_req and waits for special_op_done (the jTLB's
    // multi-cycle counter-walk invalidate); rv906's D11 single flop-array
    // TLB clears all 128 entries in one cycle (tlb_inv_all, MMU.v:258), so
    // the wait-for-done half collapses to a 1-cycle round trip UNLESS a PTW
    // walk is in flight (see below). The donor's follow-on FNC_IICA
    // (I-cache invalidate) is dropped per D-M4-6: rv906's ICache is
    // physically tagged, so a VA->PA remap can never leave a stale I$ line
    // mis-associated with the wrong PA -- no invalidate is needed. No
    // chgflw either: aq_cp0_fence_inst.v has no PC-redirect output at all
    // for this path (FNC_CMPLT->FNC_IDLE is unconditional, :188-189).
    //
    // Quiescence: unlike the donor -- whose FNC_CMMU is entered directly
    // from FNC_IDLE with no STB-drain wait -- rv906 holds on
    // `lsu_cp0_stb_empty` first, same as FENCE/FENCE.I. This is a
    // documented, deliberate DEVIATION (not a donor mirror): rv906's PTW
    // servant probes the D-cache array directly with no STB-forwarding
    // path (D3), so a pending store not yet visible to that probe could
    // let the walker read a stale PTE; draining the STB first (as rv12
    // does ahead of its own three-micro-op sfence.vma) is what keeps the
    // probe-only channel coherent. The donor doesn't need this because its
    // PTW walker IS routed through the full D-cache pipe with STB
    // forwarding.
    //
    // Mid-walk hazard (MMU.v SECTION 6's own flagged gap, closed here):
    // the donor avoids a stale in-flight walk racing the invalidate by
    // ABORTING it outright (PTW_ABT/ABT_DATA drain when tlboper preempts,
    // aq_mmu_ptw.v). rv906 doesn't build an abort-drain path; instead the
    // MMU-side handshake (mmu_cp0_sfence_done) only fires once the PTW has
    // returned to PTW_IDLE, so any walk that read pre-invalidate PTE data
    // is forced to finish (and write its -- now stale but harmless --
    // result) strictly BEFORE the invalidate pulse fires and wipes it.
    // Wait-then-wipe is equivalent to abort-then-drain for correctness,
    // without the extra FSM states.
    wire sfence_fire = is_sfence && !sfence_priv_illegal;
    localparam [1:0] SF_IDLE = 2'b00, SF_WAIT = 2'b01, SF_CMPLT = 2'b10;
    reg [1:0] sfence_state;
    wire sfence_quiesce_wait = sfence_fire && !lsu_cp0_stb_empty;
    wire sfence_launch = sfence_fire && !sfence_quiesce_wait
                        && (sfence_state == SF_IDLE);
    // NB: mmu_cp0_sfence_done is combinational off cp0_mmu_sfence_vld
    // (MMU.v's sfence_apply) whenever the PTW is already idle -- the
    // common case -- so it can pulse on the VERY SAME cycle sfence_launch
    // fires, before this FSM has registered the SF_IDLE->SF_WAIT move.
    // Catch that same-cycle completion here (go straight to SF_CMPLT),
    // else the done pulse is missed entirely (SF_WAIT never sees a
    // done -- the MMU already serviced it and won't pulse again) and
    // sfence_hold parks the pipe forever.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) sfence_state <= SF_IDLE;
        else case (sfence_state)
            SF_IDLE:  if (sfence_launch)
                          sfence_state <= mmu_cp0_sfence_done ? SF_CMPLT : SF_WAIT;
            SF_WAIT:  if (mmu_cp0_sfence_done)   sfence_state <= SF_CMPLT;
            SF_CMPLT:                            sfence_state <= SF_IDLE;
            default:                             sfence_state <= SF_IDLE;
        endcase
    end
    // EX1 hold: from the quiesce wait through the WAIT state; released at
    // SF_CMPLT so completion fires exactly that one cycle (fencei_state's
    // FI_CMPLT pattern).
    wire sfence_hold = sfence_quiesce_wait || sfence_launch
                      || (sfence_state == SF_WAIT);
    assign cp0_mmu_sfence_vld = sfence_launch;

    //=========================================================================
    // SECTION PRIVILEGE MODE + DELEGATION (M4 Task 1). pm register + the
    // trap-to-M-vs-S routing, donor aq_cp0_trap_csr.v:590-631 (pm FSM) and
    // :755-849 (medeleg/mideleg + mdeleg_vld). VERBATIM spans cited in the
    // extraction notes §B.1/§B.2.
    //=========================================================================
    // Delegation registers. medeleg: 16-bit, write mask hardwires bits
    // 14/11/10 to 0 (donor :755-757; delegable 0-9,12,13,15). mideleg: S
    // interrupt bits SSIP(1)/STIP(5)/SEIP(9) writable (donor :815-838).
    reg [15:0] medeleg_reg;
    reg [63:0] mideleg_reg;
    wire medeleg_local_en = csr_wen && (csr_addr == CSR_MEDELEG);
    wire mideleg_local_en = csr_wen && (csr_addr == CSR_MIDELEG);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            medeleg_reg <= 16'd0;
        else if (medeleg_local_en)
            medeleg_reg <= {csr_wdata[15], 1'b0, csr_wdata[13:12], 2'b0,
                            csr_wdata[9:0]};
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            mideleg_reg <= 64'd0;
        else if (mideleg_local_en)
            mideleg_reg <= csr_wdata & 64'h0000_0000_0000_022A;  // bits 1,5,9
    end

    // mdeleg_vld (donor :796-798 exceptions, :848-849 interrupts): a trap is
    // delegated to S only when taken from below M and the cause's bit is set.
    wire trap_vld      = rtu_yy_xx_expt_vld;
    wire trap_int      = rtu_yy_xx_expt_int;
    wire [4:0] trap_vec = rtu_yy_xx_expt_vec;
    wire trap_deleg    = trap_vld && (pm_r != PRIV_M) &&
                         (trap_int ? mideleg_reg[{1'b0, trap_vec}]
                                   : (trap_vec <= 5'd15) && medeleg_reg[trap_vec[3:0]]);

    // pm register (donor :590-631). Priority mret > sret > trap (the data-mux
    // order below is the donor's; mret/sret are gated by rtu_idu_commit so a
    // same-cycle older trap always wins in practice, see the DECODE note).
    reg [1:0] pm_r;
    reg [1:0] pm_wdata;
    wire      pm_wen = trap_vld || mret_fire || sret_fire;
    always @* begin
        if (mret_fire)                 pm_wdata = mpp_field;
        else if (sret_fire)            pm_wdata = {1'b0, spp_field};
        else if (trap_vld && !trap_deleg) pm_wdata = PRIV_M;
        else                           pm_wdata = PRIV_S;   // trap && trap_deleg
    end
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) pm_r <= PRIV_M;
        else if (pm_wen) pm_r <= pm_wdata;
    end
    assign cp0_yy_priv_mode = pm_r;

    //=========================================================================
    // SECTION MSTATUS -- full arm set (M4 Task 1; donor aq_cp0_trap_csr.v
    // :639-727). Fields: MIE/MPIE/SIE/SPIE (trap/xret swap), MPP/SPP
    // (trap/xret), SUM/MXR/MPRV/TSR/TW/TVM/FS (software-write only), SD
    // computed (FS==dirty). Reset: MPP=11, SPP=1, all else 0 (donor,
    // extraction notes §B.1). MPP write is WARL: 2'b10 -> 2'b00.
    //=========================================================================
    reg mie_f, mpie_f, sie_f, spie_f, spp_f, sum_f, mxr_f, mprv_f, tsr_f, tw_f, tvm_f;
    reg [1:0] mpp_field;
    wire [1:0] spp_field_w = {1'b0, spp_f};   // SPP is 1-bit; alias for pm mux
    wire spp_field = spp_f;
    wire mstatus_local_en  = csr_wen && (csr_addr == CSR_MSTATUS);
    wire sstatus_local_en  = csr_wen && (csr_addr == CSR_SSTATUS);
    wire mstatus_wr        = mstatus_local_en || sstatus_local_en;
    wire [1:0] mpp_write   = (csr_wdata[12:11] == 2'b10) ? 2'b00 : csr_wdata[12:11];

    wire trap_to_m = trap_vld && !trap_deleg;
    wire trap_to_s = trap_vld &&  trap_deleg;

    // MIE / MPIE (M path: trap_to_m clears MIE, mret pops MPIE).
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mie_f  <= 1'b0;
            mpie_f <= 1'b0;
        end else if (trap_to_m) begin
            mpie_f <= mie_f;
            mie_f  <= 1'b0;
        end else if (mret_fire) begin
            mie_f  <= mpie_f;
            mpie_f <= 1'b1;
        end else if (mstatus_wr) begin
            mpie_f <= csr_wdata[MSTATUS_MPIE_BIT];
            mie_f  <= csr_wdata[MSTATUS_MIE_BIT];
        end
    end

    // SIE / SPIE (S path: trap_to_s clears SIE, sret pops SPIE).
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sie_f  <= 1'b0;
            spie_f <= 1'b0;
        end else if (trap_to_s) begin
            spie_f <= sie_f;
            sie_f  <= 1'b0;
        end else if (sret_fire) begin
            sie_f  <= spie_f;
            spie_f <= 1'b1;
        end else if (mstatus_wr) begin
            spie_f <= csr_wdata[MSTATUS_SPIE_BIT];
            sie_f  <= csr_wdata[MSTATUS_SIE_BIT];
        end
    end

    // MPP (trap_to_m captures pm; mret sets U) and SPP (trap_to_s captures
    // pm[0]; sret sets 0).
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            mpp_field <= PRIV_M;
        else if (trap_to_m)
            mpp_field <= pm_r;
        else if (mret_fire)
            mpp_field <= PRIV_U;
        else if (mstatus_wr)
            mpp_field <= mpp_write;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            spp_f <= 1'b1;                 // donor reset SPP=1
        else if (trap_to_s)
            spp_f <= pm_r[0];
        else if (sret_fire)
            spp_f <= 1'b0;
        else if (mstatus_wr)
            spp_f <= csr_wdata[MSTATUS_SPP_BIT];
    end

    // Software-write-only arms (SUM/MXR/MPRV/TSR/TW/TVM/FS). Writable from
    // both mstatus and sstatus views (donor :535-557); only the S-visible
    // subset (SUM/MXR/FS/SPP/SPIE/SIE) takes an sstatus write.
    wire sstatus_subset = sstatus_local_en;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sum_f  <= 1'b0;
            mxr_f  <= 1'b0;
            mprv_f <= 1'b0;
            tsr_f  <= 1'b0;
            tw_f   <= 1'b0;
            tvm_f  <= 1'b0;
        end else if (mstatus_local_en) begin
            sum_f  <= csr_wdata[MSTATUS_SUM_BIT];
            mxr_f  <= csr_wdata[MSTATUS_MXR_BIT];
            mprv_f <= csr_wdata[MSTATUS_MPRV_BIT];
            tsr_f  <= csr_wdata[MSTATUS_TSR_BIT];
            tw_f   <= csr_wdata[MSTATUS_TW_BIT];
            tvm_f  <= csr_wdata[MSTATUS_TVM_BIT];
        end else if (sstatus_subset) begin
            sum_f  <= csr_wdata[MSTATUS_SUM_BIT];
            mxr_f  <= csr_wdata[MSTATUS_MXR_BIT];
        end
    end

    // FS: M4 kept it storage-only; M5 Task 1 adds the real Clean/Initial->
    // Dirty auto-transition (donor aq_cp0_trap_csr.v:571-583) on top of the
    // pre-existing software-write path. An explicit mstatus/sstatus write
    // takes priority over the dirty update (same order as the donor's own
    // if-elsif chain at :575-580). `fs_dirty_upd` (SECTION FP CSR STATE
    // below) is a forward reference -- this file already forward-
    // references wires declared later (e.g. csr_wdata itself, defined at
    // the RMW mux far below but consumed here), so this is not a new
    // pattern.
    reg [1:0] fs_field;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            fs_field <= 2'b00;
        else if (mstatus_wr)
            fs_field <= csr_wdata[MSTATUS_FS_HI:MSTATUS_FS_LO];
        else if (fs_dirty_upd)
            fs_field <= 2'b11;
    end
    wire sd_bit = (fs_field == 2'b11);

    // 64-bit mstatus layout (donor trap_csr.v:486-494). SXL/UXL = 2'b10 RO.
    // Bit map: [63]SD [62:38]rsvd [37:36]SBE/MBE=0 [35:34]SXL [33:32]UXL
    // [31:23]rsvd [22]TSR [21]TW [20]TVM [19]MXR [18]SUM [17]MPRV [16:15]XS
    // [14:13]FS [12:11]MPP [10:9]VS [8]SPP [7]MPIE [6]UBE [5]SPIE [4]UPIE
    // [3]MIE [2]rsvd [1]SIE [0]UIE.
    wire [63:0] mstatus_value =
          {sd_bit, 25'b0, 2'b0, 2'b10, 2'b10, 9'b0,     // [63:23]
           tsr_f, tw_f, tvm_f, mxr_f, sum_f, mprv_f,    // [22:17]
           2'b0, fs_field, mpp_field, 2'b0,             // [16:15]XS [14:13]FS [12:11]MPP [10:9]VS
           spp_f, mpie_f, 1'b0, spie_f, 1'b0, mie_f, 1'b0, sie_f, 1'b0};
    // sstatus is the S-view (donor :742-745): SD, UXL, MXR, SUM, FS, SPP,
    // SPIE, SIE visible; M-only fields (TSR/TW/TVM/MPRV/MPP/MPIE/MIE) read 0.
    wire [63:0] sstatus_value =
          {sd_bit, 29'b0, 2'b10, 12'b0,                 // [63:34] [33:32]UXL [31:20]rsvd
           mxr_f, sum_f, 3'b0, fs_field, 4'b0,          // [19:18] [17:15]MPRV/XS=0 [14:13]FS [12:9]MPP/VS=0
           spp_f, 1'b0, 1'b0, spie_f, 1'b0, 1'b0, 1'b0, sie_f, 1'b0};


    //=========================================================================
    // SECTION MTVEC / STVEC -- real flops. M2 kept tvec direct-mode only
    // (mode bit tied 0, accepted-but-ignored). M4 Task 1 added stvec; the
    // trap redirect target is muxed on the POST-trap pm (donor
    // aq_cp0_trap_csr.v:1346-1359): M trap -> mtvec, delegated S trap ->
    // stvec. pm updates on the expt_vld cycle, so the mux is settled by the
    // time RTU samples cp0_rtu_trap_pc (one cycle later; the donor timing
    // subtlety, extraction notes §B.4). M6 Task 1 adds the VECTORED mode
    // arm: mode bit 0 is stored for BOTH mtvec and stvec (donor
    // aq_cp0_trap_csr.v:936-944, :956/:987 -- `mtvec_value =
    // {mtvec_base, 1'b0, mtvec_mode[0]}`, only bit 0 is architecturally
    // visible; bit 1 is stored on write then masked off at read, donor
    // shape) and the redirect adds the donor's `intr && tvec[0] ? base +
    // 4*cause : base` term (donor :1355-1359, verbatim below).
    //=========================================================================
    reg [PC_WIDTH-3:0] mtvec_base;   // bits [PC_WIDTH-1:2]
    reg [PC_WIDTH-3:0] stvec_base;
    reg               mtvec_mode0;   // mode bit [0]: 0=direct, 1=vectored
    reg               stvec_mode0;
    wire mtvec_local_en = csr_wen && (csr_addr == CSR_MTVEC);
    wire stvec_local_en = csr_wen && (csr_addr == CSR_STVEC);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mtvec_base  <= {(PC_WIDTH-2){1'b0}};
            mtvec_mode0 <= 1'b0;
        end else if (mtvec_local_en) begin
            mtvec_base  <= csr_wdata[PC_WIDTH-1:2];
            mtvec_mode0 <= csr_wdata[0];
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            stvec_base  <= {(PC_WIDTH-2){1'b0}};
            stvec_mode0 <= 1'b0;
        end else if (stvec_local_en) begin
            stvec_base  <= csr_wdata[PC_WIDTH-1:2];
            stvec_mode0 <= csr_wdata[0];
        end
    end

    wire [PC_WIDTH-1:0] mtvec_pc    = {mtvec_base, 1'b0, mtvec_mode0};
    wire [PC_WIDTH-1:0] stvec_pc    = {stvec_base, 1'b0, stvec_mode0};
    // Sign-extend for CSR reads (csrr mtvec/stvec): a kernel-space vector
    // base must read back as a canonical VA, same defect class as
    // IU.v's iu_ifu_tar_pc/ag_rs1_live/bju_wb_data.
    wire [63:0]         mtvec_value = {{(64-PC_WIDTH){mtvec_pc[PC_WIDTH-1]}}, mtvec_pc};
    wire [63:0]         stvec_value = {{(64-PC_WIDTH){stvec_pc[PC_WIDTH-1]}}, stvec_pc};

    // The trap redirect target RTU reads every cycle it takes a trap (see
    // this file's header "TASK 2 DISCOVERED GAP" note) -- muxed on the
    // current pm, which already holds the post-trap mode when RTU samples.
    //
    // M6 Task 1 VECTORED arm (donor aq_cp0_trap_csr.v:1346-1359, verbatim
    // shape): the donor selects tvec/vector/intr on the CURRENT pm and adds
    // `4*vector` when the trap being taken is an interrupt and the selected
    // tvec's mode bit is 1:
    //   regs_tvec  = pm==M ? mtvec_value : stvec_value
    //   regs_vector= pm==M ? m_vector    : s_vector
    //   regs_intr  = pm==M ? m_intr      : s_intr
    //   vec_int_pc = {regs_tvec[39:2], 2'b0} + {33'b0, regs_vector[4:0], 2'b0}
    //   regs_trap_pc = regs_intr && regs_tvec[0] ? vec_int_pc
    //                                            : {regs_tvec[39:2], 2'b0}
    // TIMING (why this is safe here): m_intr/m_vector/s_intr/s_vector are
    // the mcause/scause CAPTURE flops (SECTION MCAUSE), latched on the
    // rtu_yy_xx_expt_vld cycle; pm_r flips the same edge. RTU reads
    // cp0_rtu_trap_pc through its REGISTERED retire_trap_chgflw_vld +
    // retire_chgflw_pc path (RTU.v:933-951) one+ cycles AFTER the trap
    // cycle, so the cause and the post-trap pm are both settled flops when
    // the mux is evaluated -- the donor's own "m_vector is the cause
    // captured for THIS trap" argument (extraction notes §3). The
    // cause-capture logic itself is untouched (mcause bit-63 arm already
    // exists).
    wire [PC_WIDTH-1:0] regs_tvec   = (pm_r == PRIV_M) ? mtvec_pc : stvec_pc;
    wire [4:0]          regs_vector = (pm_r == PRIV_M) ? m_vector : s_vector;
    wire                regs_intr   = (pm_r == PRIV_M) ? m_intr   : s_intr;
    wire [PC_WIDTH-1:0] vec_int_pc  = {regs_tvec[PC_WIDTH-1:2], 2'b00}
                                    + {{(PC_WIDTH-7){1'b0}}, regs_vector, 2'b00};
    wire [PC_WIDTH-1:0] regs_trap_pc =
          (regs_intr && regs_tvec[0]) ? vec_int_pc
                                      : {regs_tvec[PC_WIDTH-1:2], 2'b00};
    assign cp0_rtu_trap_pc = regs_trap_pc;

    //=========================================================================
    // SECTION MEPC / SEPC -- real flops, LSB forced 0 on write. Trap-entry
    // capture routed by delegation (donor trap_csr.v:1039-1049 mepc,
    // :1061-1073 sepc): non-delegated traps capture mepc, delegated traps
    // capture sepc.
    //=========================================================================
    reg [PC_WIDTH-2:0] mepc_reg;   // stores epc[PC_WIDTH-1:1]; LSB re-added on read
    reg [PC_WIDTH-2:0] sepc_reg;
    wire mepc_local_en = csr_wen && (csr_addr == CSR_MEPC);
    wire sepc_local_en = csr_wen && (csr_addr == CSR_SEPC);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            mepc_reg <= {(PC_WIDTH-1){1'b0}};
        else if (trap_vld && !trap_deleg)
            mepc_reg <= rtu_cp0_epc[PC_WIDTH-1:1];
        else if (mepc_local_en)
            mepc_reg <= csr_wdata[PC_WIDTH-1:1];
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            sepc_reg <= {(PC_WIDTH-1){1'b0}};
        else if (trap_vld && trap_deleg)
            sepc_reg <= rtu_cp0_epc[PC_WIDTH-1:1];
        else if (sepc_local_en)
            sepc_reg <= csr_wdata[PC_WIDTH-1:1];
    end

    wire [PC_WIDTH-1:0] mepc_pc    = {mepc_reg, 1'b0};
    wire [PC_WIDTH-1:0] sepc_pc    = {sepc_reg, 1'b0};
    // Sign-extend for CSR reads (csrr mepc/sepc): same defect class as
    // mtvec_value/stvec_value above.
    wire [63:0]         mepc_value = {{(64-PC_WIDTH){mepc_pc[PC_WIDTH-1]}}, mepc_pc};
    wire [63:0]         sepc_value = {{(64-PC_WIDTH){sepc_pc[PC_WIDTH-1]}}, sepc_pc};

    // mret/sret redirect targets -- CP0 computes the return PC itself and
    // asserts chgflw/chgflw_pc during the xret's own EX1 cycle, uniform with
    // every other CP0-declared changeflow (RTU note S7).
    assign cp0_rtu_ex1_chgflw    = mret_fire || sret_fire
                                 || (is_fencei && (fencei_state == FI_CMPLT));
    assign cp0_rtu_ex1_chgflw_pc = mret_fire ? mepc_pc
                                 : sret_fire ? sepc_pc
                                             : iu_cp0_ex1_cur_pc + {{(PC_WIDTH-3){1'b0}}, 3'd4};

    // FENCE.I walk requests (Task 10.1): FI_CLEAN holds the LSU's D-cache
    // clean walk, FI_INV holds the ICache INV_ALL request (ICache.v treats a
    // held request as INV_ALL; `inv_block` stalls fetch until done).
    assign cp0_lsu_dcache_clean   = (fencei_state == FI_CLEAN);

    //=========================================================================
    // SECTION MCAUSE -- real flop: interrupt bit + 5-bit cause (contract 7).
    // Trap-entry capture confirmed bit-exact, trap_csr.v:1083-1105
    // (`m_intr <= rtu_yy_xx_expt_int; m_vector <= rtu_yy_xx_expt_vec` on
    // `rtu_yy_xx_expt_vld`, else on `mcause_local_en`).
    //=========================================================================
    reg        m_intr;
    reg  [4:0] m_vector;
    wire mcause_local_en = csr_wen && (csr_addr == CSR_MCAUSE);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            m_intr   <= 1'b0;
            m_vector <= 5'd0;
        end else if (trap_vld && !trap_deleg) begin
            m_intr   <= rtu_yy_xx_expt_int;
            m_vector <= rtu_yy_xx_expt_vec;
        end else if (mcause_local_en) begin
            m_intr   <= csr_wdata[63];
            m_vector <= csr_wdata[4:0];
        end
    end

    wire [63:0] mcause_value = {m_intr, 58'b0, m_vector};

    // SCAUSE (donor trap_csr.v:1117-1141): delegated traps capture here.
    reg        s_intr;
    reg  [4:0] s_vector;
    wire scause_local_en = csr_wen && (csr_addr == CSR_SCAUSE);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s_intr   <= 1'b0;
            s_vector <= 5'd0;
        end else if (trap_vld && trap_deleg) begin
            s_intr   <= rtu_yy_xx_expt_int;
            s_vector <= rtu_yy_xx_expt_vec;
        end else if (scause_local_en) begin
            s_intr   <= csr_wdata[63];
            s_vector <= csr_wdata[4:0];
        end
    end

    wire [63:0] scause_value = {s_intr, 58'b0, s_vector};

    //=========================================================================
    // SECTION MSCRATCH / SSCRATCH -- plain flopped R/W registers (contract 7;
    // trap_csr.v :998-1008 mscratch, :1018-1028 sscratch, trivial).
    //=========================================================================
    reg [63:0] mscratch_reg;
    reg [63:0] sscratch_reg;
    wire mscratch_local_en = csr_wen && (csr_addr == CSR_MSCRATCH);
    wire sscratch_local_en = csr_wen && (csr_addr == CSR_SSCRATCH);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            mscratch_reg <= 64'd0;
        else if (mscratch_local_en)
            mscratch_reg <= csr_wdata;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            sscratch_reg <= 64'd0;
        else if (sscratch_local_en)
            sscratch_reg <= csr_wdata;
    end

    //=========================================================================
    // SECTION MTVAL -- real flop, populated ONLY for the vec allowlist
    // {1,2,4,5,6,7,12,13,15} (contract 7 / RTU note S4), else 0 on a trap.
    // The donor's own trap_csr.v (:1151-1161) just latches `rtu_cp0_tval`
    // unconditionally on `rtu_yy_xx_expt_vld` -- the allowlist filter lives
    // upstream, in RTU's OWN retire logic (aq_rtu_retire.v:514-522), which
    // decides what `rtu_cp0_tval` even IS before CP0 ever sees it. But
    // CSR.v's own port list gives it BOTH `rtu_yy_xx_expt_vec` and
    // `rtu_cp0_tval` directly, and plan Task 2.2 explicitly requires this
    // bench to exercise "vec-allowlist vs. non-allowlist mtval cases" on
    // CSR.v ALONE (no RTU.v in this bench at all) -- so CSR.v re-implements
    // the same filter itself as defense-in-depth: correct whether or not
    // RTU.v (Task 4, built later) also filters upstream, and independently
    // testable right now.
    //=========================================================================
    wire vec_in_tval_allowlist = (rtu_yy_xx_expt_vec == 5'd1)
                              || (rtu_yy_xx_expt_vec == 5'd2)
                              || (rtu_yy_xx_expt_vec == 5'd4)
                              || (rtu_yy_xx_expt_vec == 5'd5)
                              || (rtu_yy_xx_expt_vec == 5'd6)
                              || (rtu_yy_xx_expt_vec == 5'd7)
                              || (rtu_yy_xx_expt_vec == 5'd12)
                              || (rtu_yy_xx_expt_vec == 5'd13)
                              || (rtu_yy_xx_expt_vec == 5'd15);

    reg [63:0] mtval_reg;
    reg [63:0] stval_reg;
    wire mtval_local_en = csr_wen && (csr_addr == CSR_MTVAL);
    wire stval_local_en = csr_wen && (csr_addr == CSR_STVAL);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            mtval_reg <= 64'd0;
        else if (trap_vld && !trap_deleg)
            mtval_reg <= vec_in_tval_allowlist ? rtu_cp0_tval : 64'd0;
        else if (mtval_local_en)
            mtval_reg <= csr_wdata;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            stval_reg <= 64'd0;
        else if (trap_vld && trap_deleg)
            stval_reg <= vec_in_tval_allowlist ? rtu_cp0_tval : 64'd0;
        else if (stval_local_en)
            stval_reg <= csr_wdata;
    end

    //=========================================================================
    // SECTION MIE / MIP (+ SIE / SIP views, M4 Task 1). mie is a real R/W
    // flop. mip has writable SSIP(1)/STIP(5)/SEIP(9) flops plus RO
    // MEIP(11)/MTIP(7)/MSIP(3) from the pins (donor trap_csr.v:1202-1240).
    // sie/sip are mideleg-masked views of mie/mip (donor :922-926,:1260-1264);
    // writes through sie/sip touch only the delegated bits (donor :1216-1221).
    // Interrupt delivery is M6; M4 provides the storage/views the si/mi tests
    // exercise.
    //=========================================================================
    reg [63:0] mie_reg;
    reg        ssip_f, stip_f, seip_f;     // writable S-interrupt pending bits
    wire mie_local_en  = csr_wen && (csr_addr == CSR_MIE);
    wire mip_local_en  = csr_wen && (csr_addr == CSR_MIP);
    wire sie_local_en  = csr_wen && (csr_addr == CSR_SIE);
    wire sip_local_en  = csr_wen && (csr_addr == CSR_SIP);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            mie_reg <= 64'd0;
        else if (mie_local_en)
            mie_reg <= csr_wdata;
        else if (sie_local_en)                 // sie write touches delegated bits only
            mie_reg <= (mie_reg & ~mideleg_reg) | (csr_wdata & mideleg_reg);
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ssip_f <= 1'b0;
            stip_f <= 1'b0;
            seip_f <= 1'b0;
        end else if (mip_local_en) begin
            ssip_f <= csr_wdata[1];
            stip_f <= csr_wdata[5];
            seip_f <= csr_wdata[9];
        end else if (sip_local_en) begin
            ssip_f <= csr_wdata[1];            // only SSIP is S-writable
        end
    end

    wire mip_meip = meip, mip_mtip = mtip, mip_msip = msip;
    wire [63:0] mip_value = {52'b0, mip_meip, 3'b0, mip_mtip, 3'b0, mip_msip, 3'b0}
                          | (64'd1 << 9) * seip_f | (64'd1 << 5) * stip_f | (64'd1 << 1) * ssip_f;
    // sie/sip views masked by mideleg.
    wire [63:0] sie_value = mie_reg & mideleg_reg;
    wire [63:0] sip_value = mip_value & mideleg_reg;

    //=========================================================================
    // SECTION INTERRUPT CLAIM -- M6 Task 1: the donor's 15-term int_sel
    // (aq_cp0_trap_csr.v:1269-1338), the pending-and-enabled set the RTU's
    // priority encoder (RTU.v's casez, donor aq_rtu_int.v:53-74) reads.
    //
    // Per-source enable `*_en = mie_bit & mip_bit` (donor :1269-1278; our
    // mie_reg / mip flops-or-pins are the same storage the donor feeds
    // from). T-Head customs MCIP(16)/MHIP(18) have NO source in rv906 --
    // the donor itself ties them 0 (:1230 `mhip = 1'b0`, :1233 `mcip =
    // 1'b0`; mhie/mcie are constant 0 in the donor's mie block) -- and
    // MOIP(17)'s producer is the donor's PMU overflow (hpcp_cp0_int_vld,
    // aq_hpcp_top.v:2674), which rv906 does not model (no mhpmevent/mcntof
    // CSRs exist here, so mie bit 17 could never be enabled anyway). All
    // three *_en terms are tied 0 below; the 15-bit bus WIDTH is kept so
    // RTU.v's casez matches bit-for-bit. mideleg also cannot hold bits
    // 16/17/18 (write mask is bits {1,5,9}, CSR.v's mideleg_reg block),
    // matching the donor's constant-0 mhie/mcie deleg terms.
    //
    // Privilege gating, VERBATIM donor structure (:1281-1330):
    //  - M trio MEI/MTI/MSI (NOT delegable): `pm != M || mie_bit` (:1302-1304)
    //  - S trio + customs, nodeleg arm: `(pm==M && mie_bit || pm==S || pm==U)
    //    && *_en && !mideleg[s]` -- NOTE the donor's pm==S term carries NO
    //    SIE qualifier in the nodeleg arm (:1310-1321); the SIE gate lives
    //    ONLY in the deleg arm (:1322-1330). A pending non-delegated source
    //    in S-mode targets M, and M's global enable (mie_bit) is the gate
    //    that matters -- exactly the donor's equation.
    //  - deleg arm: `(pm==S && sie_bit || pm==U) && *_en && mideleg[s]` --
    //    U-mode is "global always on" (donor comment :1306-1309).
    // pm_r/mie_f/sie_f are the live flops (SECTION PRIVILEGE / MSTATUS).
    //=========================================================================
    wire meip_en = mie_reg[11] & mip_meip;
    wire mtip_en = mie_reg[7]  & mip_mtip;
    wire msip_en = mie_reg[3]  & mip_msip;
    wire seip_en = mie_reg[9]  & seip_f;
    wire stip_en = mie_reg[5]  & stip_f;
    wire ssip_en = mie_reg[1]  & ssip_f;
    // T-Head customs: sources absent, tied 0 (section header above).
    wire mhip_en = 1'b0;   // donor :1230 mhip=1'b0; mhie constant 0
    wire moip_en = 1'b0;   // donor :1231 moip=PMU overflow; no rv906 producer
    wire mcip_en = 1'b0;   // donor :1233 mcip=1'b0 (ECC); mcie constant 0

    // M trio (donor :1302-1304).
    wire meip_vld = (pm_r != PRIV_M || mie_f) && meip_en;
    wire mtip_vld = (pm_r != PRIV_M || mie_f) && mtip_en;
    wire msip_vld = (pm_r != PRIV_M || mie_f) && msip_en;

    // Delegable sources: nodeleg/deleg pair (donor :1281-1301 customs,
    // :1310-1330 S trio, verbatim shapes).
    wire seip_nodeleg_vld = ((pm_r == PRIV_M && mie_f)
                          || (pm_r == PRIV_S) || (pm_r == PRIV_U))
                         && seip_en && !mideleg_reg[9];
    wire stip_nodeleg_vld = ((pm_r == PRIV_M && mie_f)
                          || (pm_r == PRIV_S) || (pm_r == PRIV_U))
                         && stip_en && !mideleg_reg[5];
    wire ssip_nodeleg_vld = ((pm_r == PRIV_M && mie_f)
                          || (pm_r == PRIV_S) || (pm_r == PRIV_U))
                         && ssip_en && !mideleg_reg[1];
    wire seip_deleg_vld = ((pm_r == PRIV_S && sie_f) || (pm_r == PRIV_U))
                        && seip_en && mideleg_reg[9];
    wire stip_deleg_vld = ((pm_r == PRIV_S && sie_f) || (pm_r == PRIV_U))
                        && stip_en && mideleg_reg[5];
    wire ssip_deleg_vld = ((pm_r == PRIV_S && sie_f) || (pm_r == PRIV_U))
                        && ssip_en && mideleg_reg[1];
    // Customs' pair terms kept structurally (donor :1281-1301); their *_en
    // is 0 so they can never assert, and mideleg_reg bits 16/17/18 are
    // hardwired 0 by the write mask anyway.
    wire mhip_nodeleg_vld = ((pm_r == PRIV_M && mie_f)
                          || (pm_r == PRIV_S) || (pm_r == PRIV_U))
                         && mhip_en && !mideleg_reg[18];
    wire moip_nodeleg_vld = ((pm_r == PRIV_M && mie_f)
                          || (pm_r == PRIV_S) || (pm_r == PRIV_U))
                         && moip_en && !mideleg_reg[17];
    wire mcip_nodeleg_vld = ((pm_r == PRIV_M && mie_f)
                          || (pm_r == PRIV_S) || (pm_r == PRIV_U))
                         && mcip_en && !mideleg_reg[16];
    wire mhip_deleg_vld = ((pm_r == PRIV_S && sie_f) || (pm_r == PRIV_U))
                        && mhip_en && mideleg_reg[18];
    wire moip_deleg_vld = ((pm_r == PRIV_S && sie_f) || (pm_r == PRIV_U))
                        && moip_en && mideleg_reg[17];
    wire mcip_deleg_vld = ((pm_r == PRIV_S && sie_f) || (pm_r == PRIV_U))
                        && mcip_en && mideleg_reg[16];

    // The select vector (donor :1332-1338, VERBATIM 15-term order). Bit ->
    // RTU.v casez arm -> cause, cross-checked one line per bit against
    // RTU.v:748-763 (which is itself aq_rtu_int.v:53-74 verbatim):
    //   [14] mcip nodeleg -> 15'b1??????????????  -> cause 16
    //   [13] mhip nodeleg -> 15'b01?????????????  -> cause 18
    //   [12] meip         -> 15'b001????????????  -> cause 11
    //   [11] msip         -> 15'b0001???????????  -> cause 3
    //   [10] mtip         -> 15'b00001??????????  -> cause 7
    //   [ 9] seip nodeleg -> 15'b000001?????????  -> cause 9
    //   [ 8] ssip nodeleg -> 15'b0000001????????  -> cause 1
    //   [ 7] stip nodeleg -> 15'b00000001???????  -> cause 5
    //   [ 6] moip nodeleg -> 15'b000000001??????  -> cause 17
    //   [ 5] mcip deleg   -> 15'b0000000001?????  -> cause 16
    //   [ 4] mhip deleg   -> 15'b00000000001????  -> cause 18
    //   [ 3] seip deleg   -> 15'b000000000001???  -> cause 9
    //   [ 2] ssip deleg   -> 15'b0000000000001??  -> cause 1
    //   [ 1] stip deleg   -> 15'b00000000000001?  -> cause 5
    //   [ 0] moip deleg   -> 15'b000000000000001  -> cause 17
    // Every nodeleg (M-target) term outranks every deleg (S-target) term;
    // MOIP(17) sits between the S nodeleg group and the deleg group --
    // the donor's own ordering, carried unchanged.
    wire [14:0] int_sel = {mcip_nodeleg_vld,   // [14]
                           mhip_nodeleg_vld,   // [13]
                           meip_vld,           // [12]
                           msip_vld,           // [11]
                           mtip_vld,           // [10]
                           seip_nodeleg_vld,   // [ 9]
                           ssip_nodeleg_vld,   // [ 8]
                           stip_nodeleg_vld,   // [ 7]
                           moip_nodeleg_vld,   // [ 6]
                           mcip_deleg_vld,     // [ 5]
                           mhip_deleg_vld,     // [ 4]
                           seip_deleg_vld,     // [ 3]
                           ssip_deleg_vld,     // [ 2]
                           stip_deleg_vld,     // [ 1]
                           moip_deleg_vld};    // [ 0]

    //=========================================================================
    // SECTION INTERRUPT CLAIM EXPORT -- registered, ACTIVE-LOW (rv12
    // template rtl/CSR.v:2300-2321; the donor's own export is combinational,
    // aq_cp0_trap_csr.v:1395 -- the registered choice is recorded at the
    // cp0_rtu_int_sel port comment above). The registered active-low resets
    // to 1 (idle) and int_sel computes 0 out of reset (mie_reg and all mip
    // flops reset 0; the mip pins are driven 0 by CLINT/PLIC/testbench at
    // reset), so the export is provably dark at reset -- the M6 OFF-path
    // invariant that the entire pre-M6 battery stays bit-identical holds by
    // construction.
    //=========================================================================
    reg        int_sel_b_r;
    reg [14:0] int_sel_r;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            int_sel_b_r <= 1'b1;
            int_sel_r   <= 15'd0;
        end else begin
            int_sel_b_r <= !(|int_sel);
            int_sel_r   <= int_sel;
        end
    end
    assign cp0_rtu_int_sel = int_sel_r;
    assign cp0_rtu_int_b   = int_sel_b_r;


    //=========================================================================
    // SECTION MISA / MVENDORID / MARCHID / MIMPID / MHARTID -- hardwired RO
    // constants (contract 7). MXL=64 ("10"), extensions I(bit8)|M(bit12)|
    // C(bit2)|F(bit5)|D(bit3). F/D are flipped 0->1 here at THE SWAP (M5
    // Task 9, D11) -- the ONE commit where FP-opcode acceptance goes live,
    // mirroring the decode's own hardwired illegal-gate flip in IDU.v (no
    // live CSR plumbing: misa is a permanently-RO hardware capability, so
    // the two constants are flipped together and must stay in lockstep).
    // mvendorid mirrors the real donor's own JEDEC-ish constant
    // (info_csr.v, LSU/CP0 note B2) purely for traceability -- values are
    // cosmetic per contract 7, not exercised by riscv-tests pass/fail.
    //=========================================================================
    wire [63:0] misa_value      = 64'h8000_0000_0000_112C;
    wire [63:0] mvendorid_value = 64'h0000_0000_0000_05B7;
    wire [63:0] marchid_value   = 64'd0;
    wire [63:0] mimpid_value    = 64'd0;
    wire [63:0] mhartid_value   = 64'd0;

    //=========================================================================
    // SECTION MCYCLE / MINSTRET -- two 64-bit counters. mcycle increments
    // every cycle unconditionally. minstret auto-increments on each retired
    // architectural instruction (rtu_cp0_inst_retire), with WRITE-TAKES-
    // PRECEDENCE so an explicit minstret write suppresses the writing
    // instruction's own retire increment (rv64mi-p-instret_overflow; rv12's
    // M-1 term, a one-shot flag armed by the commit-gated write that eats
    // exactly the next retire pulse). M4 Task 1 discharges the header's
    // "KNOWN, DELIBERATE GAP" minstret note.
    //=========================================================================
    reg [63:0] mcycle_reg;
    reg [63:0] minstret_reg;
    wire mcycle_local_en   = csr_wen && (csr_addr == CSR_MCYCLE);
    wire minstret_local_en = csr_wen && (csr_addr == CSR_MINSTRET);
    wire inst_retire       = rtu_cp0_inst_retire;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            mcycle_reg <= 64'd0;
        else if (mcycle_local_en)
            mcycle_reg <= csr_wdata;
        else
            mcycle_reg <= mcycle_reg + 64'd1;
    end

    // minstret write-precedence one-shot (rv12 CSR.v M-1 term). The writer's
    // own retirement arrives the cycle after the commit-gated EX1 write; the
    // flag armed by the write eats exactly that next retire pulse.
    reg minstret_wr_pend;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            minstret_wr_pend <= 1'b0;
        else if (minstret_local_en)
            minstret_wr_pend <= 1'b1;
        else if (inst_retire)
            minstret_wr_pend <= 1'b0;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            minstret_reg <= 64'd0;
        else if (minstret_local_en)
            minstret_reg <= csr_wdata;
        else if (inst_retire && !minstret_wr_pend)
            minstret_reg <= minstret_reg + 64'd1;
    end

    //=========================================================================
    // SECTION MCOUNTEREN / SCOUNTEREN + user counter aliases (M4 Task 1;
    // donor aq_cp0_hpcp_csr.v). mcounteren/scounteren are 32-bit R/W; the
    // user RO aliases cycle(0xC00)/instret(0xC02) are gated by the counteren
    // chain (M always allowed; S gated by mcounteren; U by mcounteren &
    // scounteren -- donor :266-276). time(0xC01) is deferred to M6 (D-M4-9).
    //=========================================================================
    reg [31:0] mcounteren_reg;
    reg [31:0] scounteren_reg;
    wire mcounteren_local_en  = csr_wen && (csr_addr == CSR_MCOUNTEREN);
    wire scounteren_local_en  = csr_wen && (csr_addr == CSR_SCOUNTEREN);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            mcounteren_reg <= 32'd0;
        else if (mcounteren_local_en)
            mcounteren_reg <= csr_wdata[31:0];
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            scounteren_reg <= 32'd0;
        else if (scounteren_local_en)
            scounteren_reg <= csr_wdata[31:0];
    end

    //=========================================================================
    // SECTION SATP (M4 Task 1 storage; the MMU consumes it at Tasks 3-5).
    // WARL per donor aq_cp0_prtc_csr.v:139-140: only mode bit 63 is writable
    // when wdata[62:60]==0 (Mode in {0=Bare, 8=Sv39}); a write with an
    // unsupported mode leaves the WHOLE register unmodified (spec-conformant
    // WARL). ASID[59:44] and PPN[27:0] stored. satp writes also pulse
    // cp0_mmu_satp_wen (flushes the TLB, wired at Task 3).
    //=========================================================================
    reg [63:0] satp_reg;
    wire satp_local_en = csr_wen && (csr_addr == CSR_SATP);
    wire satp_mode_ok  = (csr_wdata[62:60] == 3'b0);   // Mode bit63 only, Bare/Sv39

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            satp_reg <= 64'd0;
        else if (satp_local_en && satp_mode_ok)
            satp_reg <= {csr_wdata[63], 3'b0, csr_wdata[59:44], 16'b0, csr_wdata[27:0]};
    end

    assign cp0_mmu_satp_data = satp_reg;
    assign cp0_mmu_satp_wen  = satp_local_en && satp_mode_ok;
    assign cp0_mmu_mxr       = mxr_f;
    assign cp0_mmu_sum       = sum_f;
    assign cp0_lsu_mprv      = mprv_f;
    assign cp0_lsu_mpp       = mpp_field;

    //=========================================================================
    // SECTION DEBUG TRIGGERS (M4 Task 1: zero-trigger escape hatch, D-M4-5;
    // real triggers arrive at M7). tselect/tdata1/tdata2/tdata3/tcontrol are
    // the ZERO-TRIGGER configuration of the debug spec: tselect hardwired 0
    // and tdata* read back 0, so rv64mi-p-breakpoint's "unsupported type"
    // skip fires (csrr tdata1 returns 0 != the written mcontrol value). The
    // addresses exist (writes are accepted-but-ignored, no illegal trap, so
    // the test's "csrs tcontrol may trap" branch is free to fall through);
    // the real trigger registers are built at M7. Donor aq_cp0_regs.v:826-831
    // (addresses 0x7A0-0x7A5).
    //=========================================================================
    wire [63:0] tselect_value  = 64'd0;
    wire [63:0] tdata1_value   = 64'd0;
    wire [63:0] tdata2_value   = 64'd0;
    wire [63:0] tdata3_value   = 64'd0;
    wire [63:0] tcontrol_value = 64'd0;

    //=========================================================================
    // SECTION PMP (M4 Task 2). Decode pmpcfg0/pmpcfg2/pmpaddr0-7; the storage
    // is in rtl/PMP.v. Writes: pmpcfg0_wen / pmp_addr_wen[7:0] strobes. Reads:
    // pmpcfg0/pmpcfg2 -> pmp_cfg0_value (pmpcfg2 reads 0 inside PMP), and
    // pmpaddr0-7 -> pmp_addr_value selected by pmp_addr_rsel. The current
    // privilege mode is exported for PMP's M-mode bypass.
    //=========================================================================
    assign pmp_cfg0_wen   = csr_wen && (csr_addr == CSR_PMPCFG0);
    assign pmp_cfg0_wdata = csr_wdata;
    // A pmpcfg2 write is a valid CSR access but has no effect in the 8-entry
    // config (entries 8-15 absent); it is simply not strobed into PMP.

    // pmpaddr0-7 = 0x3B0..0x3B7; wen one-hot by (csr_addr - CSR_PMPADDR0).
    wire [2:0] pmpaddr_off = csr_addr[2:0];   // 0x3B0..0x3B7 -> off 0..7
    assign pmp_addr_rsel  = pmpaddr_off;
    assign pmp_addr_wdata = csr_wdata;
    genvar gp;
    generate
        for (gp = 0; gp < 8; gp = gp + 1) begin : g_pmpwen
            assign pmp_addr_wen[gp] = csr_wen && (csr_addr == (CSR_PMPADDR0 + gp[11:0]));
        end
    endgenerate

    // PMP read values come from PMP.v; pmpcfg2 reads 0.
    wire [63:0] pmpcfg2_value = 64'd0;

    // current privilege mode for PMP's M-mode bypass
    assign cp0_pmp_priv_mode = pm_r;


    //=========================================================================
    // SECTION MHCR -- all bits reset 0 except wb/wbr (hardwired 1, RO);
    // contract 6/7. Bit-exact to ext_csr.v:674-725: ie/de/wa/rse/bpe/btbe
    // are the only writable bits, all reset 0 together; wb(bit3)/wbr(bit8)
    // are `assign`s in the donor, not flops at all.
    //=========================================================================
    reg mhcr_ie, mhcr_de, mhcr_wa, mhcr_rse, mhcr_bpe, mhcr_btbe;
    wire mhcr_local_en = csr_wen && (csr_addr == CSR_MHCR);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mhcr_ie   <= 1'b0;
            mhcr_de   <= 1'b0;
            mhcr_wa   <= 1'b0;
            mhcr_rse  <= 1'b0;
            mhcr_bpe  <= 1'b0;
            mhcr_btbe <= 1'b0;
        end else if (mhcr_local_en) begin
            mhcr_ie   <= csr_wdata[MHCR_IE_BIT];
            mhcr_de   <= csr_wdata[MHCR_DE_BIT];
            mhcr_wa   <= csr_wdata[MHCR_WA_BIT];
            mhcr_rse  <= csr_wdata[MHCR_RSE_BIT];
            mhcr_bpe  <= csr_wdata[MHCR_BPE_BIT];
            mhcr_btbe <= csr_wdata[MHCR_BTBE_BIT];
        end
    end

    // {..., wbr(8)=1, ibpe(7, not modeled)=0, btbe(6), bpe(5), rse(4),
    //  wb(3)=1, wa(2), de(1), ie(0)} -- matches ext_csr.v:724 exactly modulo
    // the not-modeled `ibpe`/`sck`/`l0btbe` fields, all tied 0.
    wire [63:0] mhcr_value = {55'b0, 1'b1, 1'b0,
                              mhcr_btbe, mhcr_bpe, mhcr_rse,
                              1'b1, mhcr_wa, mhcr_de, mhcr_ie};

    //=========================================================================
    // SECTION MXSTATUS -- ONLY `mm` (bit 15) is part of M2's minimal set
    // (contract 3/7): a real, plain R/W flop, resetting to 1 (ext_csr.v:
    // 550-556's cited reset value), otherwise unconsumed by CSR.v itself --
    // Task 6 (LSU/AG) owns the trap decision that ignores it for M2. Every
    // other MXSTATUS bit (cskyisaee/maee/insde/mhrd/clintee/ucme/pmdm/pmds/
    // pmdu/pm/v/ve/...) is NOT part of M2's minimal CSR set and reads 0.
    //=========================================================================
    reg mm;
    wire mxstatus_local_en = csr_wen && (csr_addr == CSR_MXSTATUS);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            mm <= 1'b1;
        else if (mxstatus_local_en)
            mm <= csr_wdata[MXSTATUS_MM];
    end

    wire [63:0] mxstatus_value = {48'b0, mm, 15'b0};

    //=========================================================================
    // SECTION MHINT -- M hint register (donor aq_cp0_ext_csr.v:840-914, CSR
    // address aq_cp0_regs.v:851). M3b Task D: the D-cache stride prefetcher
    // (PFB) is controlled from here. Bit-exact donor layout of the WRITABLE
    // fields:
    //   bit 2     dcache_pref_en    (reset 0)  -- enables the PFB; clearing
    //                                             it flushes all PFB entries
    //                                             (pfb_top.v:363-367)
    //   bits 4:3  amr               (reset 0)  -- write-allocate disabler,
    //                                             storage only until M3b Task E
    //   bit 8     icache_pref_en    (reset 0)  -- storage only: rv906's IFU
    //                                             has no prefetcher structure
    //   bit 10    iwpe              (reset 0)  -- storage only (branch-pred
    //                                             weight enhancement, no rv906
    //                                             consumer)
    //   bits 14:13 dcache_pref_dist (reset 2'b10) -- PFB lookahead distance
    //                                             = stride << dist (pfb.v:494)
    //   bit 24    pcfifo_freeze     (reset 0)  -- storage only (no rv906
    //                                             consumer)
    // Every other MHINT bit reads 0 (donor ties them off at :905-910).
    //=========================================================================
    reg        mhint_dcache_pref_en;
    reg [1:0]  mhint_amr;
    reg        mhint_icache_pref_en;
    reg        mhint_iwpe;
    reg [1:0]  mhint_dcache_pref_dist;
    reg        mhint_pcfifo_freeze;
    wire mhint_local_en = csr_wen && (csr_addr == CSR_MHINT);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mhint_dcache_pref_en   <= 1'b0;
            mhint_amr              <= 2'b0;
            mhint_icache_pref_en   <= 1'b0;
            mhint_iwpe             <= 1'b0;
            mhint_dcache_pref_dist <= 2'b10;   // donor reset, ext_csr.v:868
            mhint_pcfifo_freeze    <= 1'b0;
        end else if (mhint_local_en) begin
            mhint_dcache_pref_en   <= csr_wdata[2];
            mhint_amr              <= csr_wdata[4:3];
            mhint_icache_pref_en   <= csr_wdata[8];
            mhint_iwpe             <= csr_wdata[10];
            mhint_dcache_pref_dist <= csr_wdata[14:13];
            mhint_pcfifo_freeze    <= csr_wdata[24];
        end
    end

    // Donor read-value layout (ext_csr.v:909-912): {32'b0, 7'b0,
    // pcfifo_freeze, 2'b0, tlb_broad_dis, l2stpld, ecc_en, nsfe,
    // l2_pref_dist[1:0], l2pld, dcache_pref_dist[1:0], 1'b0, sre, iwpe,
    // lpe, icache_pref_en, 2'b0, amr2, amr[1:0], dcache_pref_en, 2'b0}
    // -- all unmodeled fields tied 0 (ext_csr.v:905-910).
    wire [63:0] mhint_value = {32'b0, 7'b0, mhint_pcfifo_freeze, 2'b0,
                               1'b0, 1'b0, 1'b0,
                               1'b0, 2'b0, 1'b0,
                               mhint_dcache_pref_dist, 1'b0, 1'b0,
                               mhint_iwpe, 1'b0, mhint_icache_pref_en, 2'b0, 1'b0,
                               mhint_amr, mhint_dcache_pref_en, 2'b0};

    //=========================================================================
    // SECTION FP CSR STATE (M5 Task 1). fflags(0x001)/frm(0x002)/fcsr(0x003)
    // storage, donor aq_cp0_float_csr.v:220-268: an fcsr write updates BOTH
    // frm[7:5] and fflags[4:0] at once (its own `fcsr_local_en` arm in each
    // register's always block); a direct fflags/frm write touches only its
    // own field. rv906 drops the donor's T-Head vxrm/vxsat/fxcr vector-
    // extension bits (no vector extension) -- fcsr's bits [10:8] stay 0
    // unlike the donor's non-zero vxrm/vxsat, otherwise the standard
    // (non-T-Head) fcsr encoding {frm[7:5], fflags[4:0]}. FS-off illegal
    // gating (SECTION EX1 COMPLETION's csr_access_illegal, donor
    // aq_cp0_regs.v:1101-1104) and the dirty-on-FP-CSR-write transition
    // (`fs_dirty_upd` below, consumed by SECTION MSTATUS's fs_field always
    // block; donor aq_cp0_trap_csr.v:562-569) are both wired here. The
    // FP-instruction-retire terms -- `rtu_cp0_fflags` sticky-OR'd into
    // fflags on retire (D7, donor aq_cp0_float_csr.v:234-238) and
    // `rtu_cp0_fs_dirty_updt` OR'ed into `fs_dirty_upd` (the donor's own
    // term in the same expression, aq_cp0_trap_csr.v:562) -- are wired
    // since M5 Task 8.
    //=========================================================================
    reg [4:0] fflags_reg;
    reg [2:0] frm_reg;
    wire fflags_local_en = csr_wen && (csr_addr == CSR_FFLAGS);
    wire frm_local_en    = csr_wen && (csr_addr == CSR_FRM);
    wire fcsr_local_en   = csr_wen && (csr_addr == CSR_FCSR);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            fflags_reg <= 5'b0;
        else if (fcsr_local_en)
            fflags_reg <= csr_wdata[4:0];
        else if (fflags_local_en)
            fflags_reg <= csr_wdata[4:0];
        // M5 Task 8 (D7): sticky OR-in of the retiring FP op's flags
        // (donor aq_cp0_float_csr.v:234-238, `rtu_cp0_fflags_updt` arm).
        // PRIORITY, per D7's explicit-instruction: this arm is LAST in the
        // if-elsif chain, so an explicit csrw/csrs/csrc to fflags/fcsr in
        // the SAME cycle as an FP-op retire wins over the accrual (the
        // write is observed on top of, not clobbered by, it). In rv906's
        // in-order single-issue pipe the two events almost never share a
        // cycle anyway (the retire lands one cycle after the write's EX1),
        // but the priority is documented in the RTL regardless.
        else if (rtu_cp0_fs_dirty_updt)
            fflags_reg <= fflags_reg | rtu_cp0_fflags;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            frm_reg <= 3'b0;
        else if (fcsr_local_en)
            frm_reg <= csr_wdata[7:5];
        else if (frm_local_en)
            frm_reg <= csr_wdata[2:0];
    end

    // M5 Task 11 BUG 3 (rv64uf/ud-p-fcmp test 10, Category A -- rv906-own
    // pipeline race; the public C906 factory has no scalar FPU, so there is
    // no donor read-path precedent): a csrrw/fsflags following an FP op
    // back-to-back executes its EX1 on the FP op's EX2-retire cycle, and
    // reads fflags_reg BEFORE the sticky-OR accrual arm (last arm of the
    // always block above) lands at the END of that cycle -- the read misses
    // the just-accrued bits (fcmp test 10: feq.s(sNaN,0) accrues NV=0x10,
    // the csrrw reads 0). Program order requires the read to observe the
    // prior FP op's flags, so the READ VIEW forwards the in-flight retire
    // packet (rtu_cp0_fflags qualified by rtu_cp0_fs_dirty_updt). Storage
    // and the write-vs-accrual priority above are unchanged: an explicit
    // csrrw still replaces (the forwarded value is already captured by the
    // read), and the csrrs/csrrc RMW forms pick the accrued bits up through
    // this same csr_rdata. Same fix class as rv12's live-bus fflags mux
    // (../rv12/rtl/RTU.v:2810-2840, its own stale-registered-fflags bug).
    wire [4:0] fflags_rd = fflags_reg
                         | (rtu_cp0_fs_dirty_updt ? rtu_cp0_fflags : 5'b0);
    wire [63:0] fflags_value = {59'b0, fflags_rd};
    wire [63:0] frm_value    = {61'b0, frm_reg};
    wire [63:0] fcsr_value   = {56'b0, frm_reg, fflags_rd};

    // M5 Task 11 BUG 2: frm read-out to FPU.v's dynamic-rounding resolution
    // (rm=111 -> frm; see the cp0_fpu_frm port comment above). Live register
    // value, not a snapshot -- an in-order csrw frm followed by a DYN FP op
    // must observe the new mode, and EX1 dispatch is always behind the CSR
    // write's own completion in this pipe.
    assign cp0_fpu_frm = frm_reg;

    // Clean/Initial -> Dirty on ANY FP state change: an explicit FP-CSR
    // write OR an FP-instruction retire (M5 Task 8 -- donor
    // aq_cp0_trap_csr.v:562-569's fs_dirty_upd OR's
    // rtu_cp0_fs_dirty_updt together with the local_en terms exactly this
    // way). Off(00) and already-Dirty(11) are excluded exactly as the
    // donor excludes them.
    wire fs_dirty_upd = (fflags_local_en || frm_local_en || fcsr_local_en
                       || rtu_cp0_fs_dirty_updt)
                     && (fs_field == 2'b01 || fs_field == 2'b10);

    //=========================================================================
    // SECTION READ MUX -- the generic address-decoded read bus every RMW
    // (and every plain CSR read) goes through (CP0 note B1's "hybrid" bus).
    // Unimplemented addresses read 0 -- IDU (Task 5) is the one that must
    // flag an access to a CSR outside M2's minimal set as illegal before
    // dispatch ever reaches here (contract 7's "everything else is absent,
    // not stubbed"); this mux is simply never consulted for those.
    //=========================================================================
    function automatic [63:0] csr_read_mux(input [11:0] addr);
        case (addr)
            CSR_MSTATUS:    csr_read_mux = mstatus_value;
            CSR_MISA:       csr_read_mux = misa_value;
            CSR_MEDELEG:    csr_read_mux = {48'b0, medeleg_reg};
            CSR_MIDELEG:    csr_read_mux = mideleg_reg;
            CSR_MIE:        csr_read_mux = mie_reg;
            CSR_MTVEC:      csr_read_mux = mtvec_value;
            CSR_MCOUNTEREN: csr_read_mux = {32'b0, mcounteren_reg};
            CSR_MSCRATCH:   csr_read_mux = mscratch_reg;
            CSR_MEPC:       csr_read_mux = mepc_value;
            CSR_MCAUSE:     csr_read_mux = mcause_value;
            CSR_MTVAL:      csr_read_mux = mtval_reg;
            CSR_MIP:        csr_read_mux = mip_value;
            CSR_SSTATUS:    csr_read_mux = sstatus_value;
            CSR_SIE:        csr_read_mux = sie_value;
            CSR_STVEC:      csr_read_mux = stvec_value;
            CSR_SCOUNTEREN: csr_read_mux = {32'b0, scounteren_reg};
            CSR_SSCRATCH:   csr_read_mux = sscratch_reg;
            CSR_SEPC:       csr_read_mux = sepc_value;
            CSR_SCAUSE:     csr_read_mux = scause_value;
            CSR_STVAL:      csr_read_mux = stval_reg;
            CSR_SIP:        csr_read_mux = sip_value;
            CSR_SATP:       csr_read_mux = satp_reg;
            CSR_MCYCLE:     csr_read_mux = mcycle_reg;
            CSR_MINSTRET:   csr_read_mux = minstret_reg;
            CSR_CYCLE:      csr_read_mux = mcycle_reg;
            CSR_INSTRET:    csr_read_mux = minstret_reg;
            CSR_TSELECT:    csr_read_mux = tselect_value;
            CSR_TDATA1:     csr_read_mux = tdata1_value;
            CSR_TDATA2:     csr_read_mux = tdata2_value;
            CSR_TDATA3:     csr_read_mux = tdata3_value;
            CSR_TCONTROL:   csr_read_mux = tcontrol_value;
            CSR_PMPCFG0:    csr_read_mux = pmp_cfg0_value;
            CSR_PMPCFG2:    csr_read_mux = pmpcfg2_value;
            CSR_MVENDORID:  csr_read_mux = mvendorid_value;
            CSR_MARCHID:    csr_read_mux = marchid_value;
            CSR_MIMPID:     csr_read_mux = mimpid_value;
            CSR_MHARTID:    csr_read_mux = mhartid_value;
            CSR_MXSTATUS:   csr_read_mux = mxstatus_value;
            CSR_MHCR:       csr_read_mux = mhcr_value;
            CSR_MHINT:      csr_read_mux = mhint_value;
            CSR_FFLAGS:     csr_read_mux = fflags_value;
            CSR_FRM:        csr_read_mux = frm_value;
            CSR_FCSR:       csr_read_mux = fcsr_value;
            default:        csr_read_mux = 64'd0;
        endcase
    endfunction

    // pmpaddr0-7 (0x3B0..0x3B7) read from PMP.v (selected by pmp_addr_rsel);
    // the case-mux default returns 0 for them, so mux it in here.
    wire pmpaddr_addr_sel = (csr_addr >= CSR_PMPADDR0)
                         && (csr_addr <= (CSR_PMPADDR0 + 12'd7));
    wire [63:0] csr_rdata = pmpaddr_addr_sel ? pmp_addr_value
                                             : csr_read_mux(csr_addr);

    // Same three-op RMW mux as the donor (CP0 note B1, aq_cp0_iui.v:
    // 494-500: csrrw_rs1=rs1; csrrs_rs1=rdata|rs1; csrrc_rs1=rdata&~rs1).
    wire [63:0] csr_wdata = csr_write_form ? csr_rs1_operand
                          : csr_set_form   ? (csr_rdata | csr_rs1_operand)
                          : csr_clear_form ? (csr_rdata & ~csr_rs1_operand)
                          : 64'd0;

    //=========================================================================
    // SECTION EX1 COMPLETION -- cp0_rtu_t (design doc S4.2): old-CSR-value
    // writeback riding the same single-cycle EX1 completion path as any
    // other producer (no FSM, CP0 note B1), plus the synchronous-exception
    // declaration bus.
    //
    // TASK 4 DISCOVERED BUG, FIXED HERE (same "documented amendment, not
    // silent" discipline as this file's own "TASK 2 DISCOVERED GAP" note
    // above): `cp0_rtu_ex1_cmplt_dp` is RTU's one-hot RETIRE-heartbeat leg
    // (RTU note S2: `dp_cmplt_source[6:0] = {alu,mul,bju,div,lsu,cp0,
    // vec}_cmplt_dp`, OR'd into `dp_ex1_cmplt_dp`, which gates whether
    // RTU's EX1->EX2 retire register latches AT ALL this cycle) -- this is
    // a DIFFERENT signal from `cp0_rtu_ex1_wb_vld`/`_wb_dp` (RTU's rbus
    // GPR-writeback-source-select bit, RTU note S3). Confirmed directly
    // against the donor: `cp0/rtl/aq_cp0_iui.v:752,804-809` drives these
    // two families from DIFFERENT expressions --
    // `cp0_rtu_ex1_wb_dp = iui_inst_dst_vld_dp = iui_inst_csr` (CSR-op-only,
    // matches `wb_vld` below) vs. `cp0_rtu_ex1_cmplt_dp = idu_cp0_ex1_dp_sel`
    // (ANY CP0-dispatched, non-internally-stalled instruction -- ecall/
    // ebreak/mret/fence/fence.i included, no GPR-result filter at all) --
    // and `rtu/rtl/aq_rtu_ctrl.v:190,197-199`'s `cmplt_clk_en = ctrl_ex1_
    // cmplt_dp || ...` is the literal retire-latch enable this feeds.
    // Tying `cp0_rtu_ex1_cmplt_dp` 1:1 with `is_csr_op` (the previous body
    // of this file) meant ecall/ebreak/mret/fence/fence.i -- all of which
    // set `idu_cp0_ex1_sel` but not `is_csr_op` (this file's own DECODE
    // section note: "FENCE/FENCE.I ... produce none of the three
    // completion signals below at all") -- would NEVER assert RTU's
    // retire-latch-enable and could therefore never retire once RTU.v
    // (Task 4) exists: a real, pipeline-wedging bug, not a style nit.
    // Fix: `cp0_rtu_ex1_cmplt_dp` is wired to `ex1_active` (this file's own
    // `idu_cp0_ex1_sel && !ex1_flush`, already computed in the DECODE
    // section above) -- the direct rv906 analogue of the donor's dispatch-
    // select-minus-internal-stall-and-flush -- while `cp0_rtu_ex1_wb_vld`
    // correctly stays CSR-op-only (it already matched the donor's narrower
    // `wb_dp`/`wb_vld` family and needed no change). RTU.v itself does not
    // exist yet at the time of this fix (Task 4 lands it in the same
    // commit) -- nothing was silently broken; this is the first cycle
    // anything reads this port for real.
    //=========================================================================
    assign cp0_rtu_ex1_wb_vld   = is_csr_op;
    assign cp0_rtu_ex1_wb_data  = csr_rdata;
    assign cp0_rtu_ex1_wb_preg  = idu_cp0_ex1_dst0_reg;
    // fence_hold (above) holds a FENCE/FENCE.I in EX1 from LSU-quiescence
    // through the clean/invalidate walks -- cmplt_dp must NOT heartbeat
    // while held, otherwise RTU would retire the fence before its ordering
    // guarantee is established.
    assign cp0_rtu_ex1_cmplt_dp = ex1_active && !fence_hold && !sfence_hold;
    // Port name predates M4 (FENCE.I-only originally); it now stalls IDU
    // dispatch for the sfence.vma sequencer too, same mechanism.
    assign cp0_idu_fencei_full  = fence_hold || sfence_hold;
    // Task 7.3: the completing CSR instruction's length. CP0 completes in
    // EX1 (single cycle), so the completing instruction IS the live EX1
    // instruction -- no latching needed.
    assign cp0_rtu_ex1_inst_len = idu_cp0_ex1_inst_len;

    // Synchronous exceptions CP0 itself detects: illegal instruction (any
    // CP0-dispatched op IDU already flagged illegal), ecall (per-priv cause),
    // ebreak (cause 3). Standard RISC-V cause encoding; none is an interrupt.
    //-------------------------------------------------------------------------
    // M4 Task 1: CSR access qualification (donor aq_cp0_iui.v:602-619).
    // addr[9:8] encodes the minimum privilege (00=U,01=S,11=M; 10 reserved).
    // Access below the minimum priv is illegal; a write to a read-only CSR
    // (addr[11:10]==11) is illegal; S-mode satp access with TVM=1 is illegal.
    //-------------------------------------------------------------------------
    wire [1:0] csr_min_priv   = csr_addr[9:8];
    wire csr_priv_bad = is_csr_op &&
                        ((csr_min_priv == 2'b10)                              // reserved
                      || (pm_r == PRIV_U && csr_min_priv != 2'b00)            // U: U-only
                      || (pm_r == PRIV_S && csr_min_priv == 2'b11));          // S: not M
    wire csr_ro_write = is_csr_op && (csr_addr[11:10] == 2'b11) && csr_wen_raw;
    wire satp_tvm_illegal = is_csr_op && (csr_addr == CSR_SATP)
                          && (pm_r == PRIV_S) && tvm_f;
    // M5 Task 1: fflags/frm/fcsr access with mstatus.FS==Off is illegal
    // (donor aq_cp0_regs.v:1101-1104, `regs_imm_inv = regs_fs_off` grouped
    // identically for all three addresses).
    wire csr_fp_addr = (csr_addr == CSR_FFLAGS) || (csr_addr == CSR_FRM)
                     || (csr_addr == CSR_FCSR);
    wire csr_fp_illegal = is_csr_op && csr_fp_addr && (fs_field == 2'b00);
    wire csr_access_illegal = csr_priv_bad || csr_ro_write || satp_tvm_illegal
                            || csr_fp_illegal;

    // Per-privilege ecall cause (U=8, S=9, M=11).
    wire [4:0] ecall_vec = (pm_r == PRIV_M) ? CAUSE_MACHINE_ECALL
                         : (pm_r == PRIV_S) ? CAUSE_SUPERVISOR_ECALL
                                            : CAUSE_USER_ECALL;

    // M4 Task 6: ex1_fetch_fault (pgflt || accflt) OR'd in, and given
    // priority OVER everything else in the vec mux below (donor
    // aq_cp0_iui.v:645-666: pgflt(12) > accflt(1) > illegal(2) > ecall).
    assign cp0_rtu_ex1_expt_vld = ex1_fetch_fault || ex1_illegal || is_ecall || is_ebreak
                                || csr_access_illegal || xret_illegal;
    assign cp0_rtu_ex1_expt_int = 1'b0;
    assign cp0_rtu_ex1_expt_vec = ex1_fetch_pgflt  ? CAUSE_FETCH_PAGE_FAULT :
                                  ex1_fetch_accflt ? CAUSE_FETCH_ACCESS :
                                  (ex1_illegal || csr_access_illegal || xret_illegal) ? CAUSE_ILLEGAL :
                                  is_ecall     ? ecall_vec :
                                  is_ebreak    ? CAUSE_BREAKPOINT : 5'd0;

    //=========================================================================
    // SECTION MHCR / MXSTATUS FAN-OUT -- replaces FetchSink's harness config
    // bank (design doc S2.3.6's "open integration item", S8). 1:1 with the
    // donor's own 6 writable MHCR bits: ie->icache_en, de->dcache_en,
    // wa->wa, rse->ras_en, bpe->bht_en, btbe->btb_en. The remaining ICache/
    // BPU config ports (iwpe, icache_pref_en, the icache/bht invalidate-
    // request triples) have no MHCR bit to source from in M2's minimal set
    // -- cache-maintenance custom ops (icache.iva/dcache.iall/etc, the
    // donor's FUNC_ICACHE_*/FUNC_DCACHE_*) are out of M2's decode scope
    // (design doc S2.2's "che" sub-FSM note), so these stay tied to their
    // quiescent defaults. The one exception is FENCE.I, which drives the
    // INV_ALL request line from the fencei_state FI_INV stage (SECTION
    // DECODE note) -- required by rv64ui-p-fence_i, faithful to the donor.
    //=========================================================================
    assign cp0_ifu_icache_en       = mhcr_ie;
    assign cp0_ifu_iwpe            = ICACHE_IWPE_DEFAULT;
    assign cp0_ifu_icache_pref_en  = 1'b0;
    assign cp0_ifu_icache_inv_addr = 64'd0;
    assign cp0_ifu_icache_inv_req  = (fencei_state == FI_INV);
    assign cp0_ifu_icache_inv_type = 2'd0;
    assign cp0_ifu_bht_en          = mhcr_bpe;
    assign cp0_ifu_btb_en          = mhcr_btbe;
    assign cp0_ifu_ras_en          = mhcr_rse;
    assign cp0_ifu_bht_inv         = 1'b0;
    assign cp0_ifu_btb_clr         = 1'b0;

    assign cp0_lsu_dcache_en = mhcr_de;
    assign cp0_lsu_mm        = mm;
    assign cp0_lsu_wa        = mhcr_wa;
    // M3b Task D: MHINT prefetch controls to the LSU's PFB (ext_csr.v:861/868)
    assign cp0_lsu_dcache_pref_en   = mhint_dcache_pref_en;
    assign cp0_lsu_dcache_pref_dist = mhint_dcache_pref_dist;
    // M3b Task E: MHINT.amr to the LSU's AMR (ext_csr.v:881)
    assign cp0_lsu_amr              = mhint_amr;

    assign cp0_xx_mrvbr = RESET_VECTOR[PC_WIDTH-1:0];

    // `bht_cp0_inv_done` (handshake-done input) has no consumer in M2's
    // minimal CSR set -- no BHT invalidate request is ever issued (tied 0 in
    // the MHCR fan-out above). `ifu_cp0_icache_inv_done` IS consumed: it
    // advances the FENCE.I fencei_state sequencer (SECTION DECODE note).
    // `iu_cp0_ex1_cur_pc` feeds the FENCE.I changeflow PC (PC+4) in addition
    // to its original trap-context role.

endmodule
