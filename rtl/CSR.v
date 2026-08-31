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
// from mtip/msip/meip), misa (RO: MXL=64, I|M|C), mvendorid/marchid/mimpid/
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
    wire ex1_ok      = ex1_active && !idu_cp0_ex1_illegal;   // legal, actionable this cycle
    wire ex1_illegal = ex1_active &&  idu_cp0_ex1_illegal;

    wire is_ecall  = ex1_ok && (idu_cp0_ex1_func == CP0_FUNC_ECALL);
    wire is_ebreak = ex1_ok && (idu_cp0_ex1_func == CP0_FUNC_EBREAK);
    wire is_mret   = ex1_ok && (idu_cp0_ex1_func == CP0_FUNC_MRET);
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
    wire rs1_is_x0 = (idu_cp0_ex1_opcode[19:15] == 5'b0);
    wire csr_wen   = is_csr_op && (csr_write_form || !rs1_is_x0);

    // MRET/trap sequencing shares the exact same three-way priority the
    // donor's mstatus/mepc/mcause/mtval always-blocks use (trap capture >
    // mret pop > software CSR write > hold) -- one flop per bit, no FSM
    // (CP0 note B1).
    wire mret_fire = is_mret;

    //=========================================================================
    // SECTION MSTATUS -- only MIE/MPIE are real flops; MPP tied 2'b11 RO;
    // everything else tied 0/RO (contract 7, design doc S2.3.6). Trap-entry
    // swap / mret pop confirmed bit-exact, aq_cp0_trap_csr.v:669-711:
    //   trap : mpie <= mie_bit; mie_bit <= 1'b0;
    //   mret : mie_bit <= mpie; mpie <= 1'b1;
    //   sw wr: mpie <= wdata[7]; mie_bit <= wdata[3];
    // (the donor's `!mdeleg_vld_dp` qualifier is dropped -- M2 has no S-mode
    // to delegate to, so it is unconditionally true here.)
    //=========================================================================
    reg mie_bit, mpie_bit;
    wire mstatus_local_en = csr_wen && (csr_addr == CSR_MSTATUS);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mie_bit  <= 1'b0;
            mpie_bit <= 1'b0;
        end else if (rtu_yy_xx_expt_vld) begin
            mpie_bit <= mie_bit;
            mie_bit  <= 1'b0;
        end else if (mret_fire) begin
            mie_bit  <= mpie_bit;
            mpie_bit <= 1'b1;
        end else if (mstatus_local_en) begin
            mpie_bit <= csr_wdata[7];
            mie_bit  <= csr_wdata[3];
        end
    end

    // Layout matches the real 64-bit mstatus bit positions (trap_csr.v:
    // 486-494) with every non-implemented field tied 0: [12:11]=MPP(fixed
    // 2'b11, contract 7 -- no other privilege level exists to hold), [7]=
    // MPIE, [3]=MIE, everything else (SD/MPV/SXL/UXL/TSR/TM/TVM/MXR/SUM/
    // MPRV/FS/SPP/SPIE/SIE/...) reads 0.
    wire [63:0] mstatus_value = {51'b0, 2'b11, 3'b0,
                                 mpie_bit, 1'b0, 1'b0, 1'b0,
                                 mie_bit,  1'b0, 1'b0, 1'b0};

    //=========================================================================
    // SECTION MTVEC -- real flop, direct mode only (contract 7): mode bit
    // (and the reserved bit above it) tied 0; writes attempting vectored
    // mode are accepted-but-ignored on the mode bit (i.e. the mode bit is
    // simply never stored). Base layout matches trap_csr.v:956
    // (`mtvec_value = {mtvec_base[61:0], 1'b0, mtvec_mode[0]}`).
    //=========================================================================
    reg [PC_WIDTH-3:0] mtvec_base;   // bits [PC_WIDTH-1:2]
    wire mtvec_local_en = csr_wen && (csr_addr == CSR_MTVEC);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            mtvec_base <= {(PC_WIDTH-2){1'b0}};
        else if (mtvec_local_en)
            mtvec_base <= csr_wdata[PC_WIDTH-1:2];
    end

    wire [PC_WIDTH-1:0] mtvec_pc    = {mtvec_base, 2'b00};   // mode forced 0 (direct)
    wire [63:0]         mtvec_value = {{(64-PC_WIDTH){1'b0}}, mtvec_pc};

    // The one direct-mode redirect target RTU needs every cycle it takes a
    // trap (see this file's header "TASK 2 DISCOVERED GAP" note) -- exposed
    // unconditionally, not gated on any dispatch/valid signal, exactly like
    // the donor's own `cp0_rtu_trap_pc` (aq_cp0_trap_csr.v:1396).
    assign cp0_rtu_trap_pc = mtvec_pc;

    //=========================================================================
    // SECTION MEPC -- real flop, LSB forced 0 on write (contract 7, matches
    // `regs_iui_mepc={mepc_reg[38:0],1'b0}`, trap_csr.v:1382). Trap-entry
    // capture confirmed bit-exact, trap_csr.v:1039-1049 (`mepc_reg[62:0] <=
    // rtu_cp0_epc[63:1]` on `rtu_yy_xx_expt_vld`, else on `mepc_local_en`).
    //=========================================================================
    reg [PC_WIDTH-2:0] mepc_reg;   // stores epc[PC_WIDTH-1:1]; LSB re-added on read
    wire mepc_local_en = csr_wen && (csr_addr == CSR_MEPC);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            mepc_reg <= {(PC_WIDTH-1){1'b0}};
        else if (rtu_yy_xx_expt_vld)
            mepc_reg <= rtu_cp0_epc[PC_WIDTH-1:1];
        else if (mepc_local_en)
            mepc_reg <= csr_wdata[PC_WIDTH-1:1];
    end

    wire [PC_WIDTH-1:0] mepc_pc    = {mepc_reg, 1'b0};
    wire [63:0]         mepc_value = {{(64-PC_WIDTH){1'b0}}, mepc_pc};

    // mret's redirect target -- CP0 computes it itself and asserts
    // chgflw/chgflw_pc during its own EX1 cycle, uniform with every other
    // CP0-declared changeflow (RTU note S7: "MRET/SRET redirect is uniform
    // with every other changeflow... CP0 computes the return PC itself").
    assign cp0_rtu_ex1_chgflw    = mret_fire || (is_fencei && (fencei_state == FI_CMPLT));
    assign cp0_rtu_ex1_chgflw_pc = mret_fire ? mepc_pc
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
        end else if (rtu_yy_xx_expt_vld) begin
            m_intr   <= rtu_yy_xx_expt_int;
            m_vector <= rtu_yy_xx_expt_vec;
        end else if (mcause_local_en) begin
            m_intr   <= csr_wdata[63];
            m_vector <= csr_wdata[4:0];
        end
    end

    wire [63:0] mcause_value = {m_intr, 58'b0, m_vector};

    //=========================================================================
    // SECTION MSCRATCH -- plain flopped R/W register (contract 7; trap_csr.v
    // :998-1008, trivial).
    //=========================================================================
    reg [63:0] mscratch_reg;
    wire mscratch_local_en = csr_wen && (csr_addr == CSR_MSCRATCH);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            mscratch_reg <= 64'd0;
        else if (mscratch_local_en)
            mscratch_reg <= csr_wdata;
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
    wire mtval_local_en = csr_wen && (csr_addr == CSR_MTVAL);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            mtval_reg <= 64'd0;
        else if (rtu_yy_xx_expt_vld)
            mtval_reg <= vec_in_tval_allowlist ? rtu_cp0_tval : 64'd0;
        else if (mtval_local_en)
            mtval_reg <= csr_wdata;
    end

    //=========================================================================
    // SECTION MIE / MIP -- mie is a real R/W flop; mip's bits are pure
    // read-only wires from mtip/msip/meip (contract 7; trap_csr.v:1234-1245
    // `assign meip=biu_cp0_me_int; assign mtip=biu_cp0_mt_int; assign msip=
    // biu_cp0_ms_int` -- NOT flops at all in the donor either). Standard
    // bit positions MEIP=11/MTIP=7/MSIP=3; mie mirrors the same positions
    // for the matching enables, though M2 does not gate anything on them
    // (contract 7: "not required for M2's rv64ui/um pass bar... it is
    // nearly free given the M1 ports already exist").
    //=========================================================================
    reg [63:0] mie_reg;
    wire mie_local_en = csr_wen && (csr_addr == CSR_MIE);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            mie_reg <= 64'd0;
        else if (mie_local_en)
            mie_reg <= csr_wdata;
    end

    wire [63:0] mip_value = {52'b0, meip, 3'b0, mtip, 3'b0, msip, 3'b0};

    //=========================================================================
    // SECTION MISA / MVENDORID / MARCHID / MIMPID / MHARTID -- hardwired RO
    // constants (contract 7). MXL=64 ("10"), extensions I(bit8)|M(bit12)|
    // C(bit2) -- A joins at M3, F/D at M5 (design doc S2.3.6). Writes are
    // ignored (no *_local_en decode exists for these addresses at all).
    // mvendorid mirrors the real donor's own JEDEC-ish constant
    // (info_csr.v, LSU/CP0 note B2) purely for traceability -- values are
    // cosmetic per contract 7, not exercised by riscv-tests pass/fail.
    //=========================================================================
    wire [63:0] misa_value      = 64'h8000_0000_0000_1104;
    wire [63:0] mvendorid_value = 64'h0000_0000_0000_05B7;
    wire [63:0] marchid_value   = 64'd0;
    wire [63:0] mimpid_value    = 64'd0;
    wire [63:0] mhartid_value   = 64'd0;

    //=========================================================================
    // SECTION MCYCLE / MINSTRET -- two local free-running 64-bit counters
    // directly in CSR.v (contract 7), not a PMU stub. `mcycle` increments
    // every cycle, unconditionally (real RISC-V semantics: counts core
    // clocks, not retired work) with a plain CSR-write override.
    //
    // `minstret` is implemented R/W for real (software can read/write it
    // today), but does NOT yet auto-increment on retirement: this file's
    // header "KNOWN, DELIBERATE GAP" note (carried over from Task 1, plan
    // Task 1.1's own CSR.v bullet: "confirmed in Task 4, not guessed here")
    // explains why CSR.v's port list has no general "an instruction
    // retired" pulse yet -- CSR.v only ever sees idu_cp0_ex1_sel fire for
    // CSR/CP0-dispatched instructions specifically, which would undercount
    // minstret drastically if used as a stand-in. Task 4 (RTU.v) adds the
    // real retire-commit pulse to this port list and this always-block
    // gains its `else if (<pulse>) minstret_reg <= minstret_reg + 1;` arm
    // then -- an anticipated, documented amendment, not a silent gap.
    //=========================================================================
    reg [63:0] mcycle_reg;
    reg [63:0] minstret_reg;
    wire mcycle_local_en   = csr_wen && (csr_addr == CSR_MCYCLE);
    wire minstret_local_en = csr_wen && (csr_addr == CSR_MINSTRET);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            mcycle_reg <= 64'd0;
        else if (mcycle_local_en)
            mcycle_reg <= csr_wdata;
        else
            mcycle_reg <= mcycle_reg + 64'd1;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            minstret_reg <= 64'd0;
        else if (minstret_local_en)
            minstret_reg <= csr_wdata;
        // else: holds -- see the section header; Task 4 adds the increment arm.
    end

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
    // SECTION READ MUX -- the generic address-decoded read bus every RMW
    // (and every plain CSR read) goes through (CP0 note B1's "hybrid" bus).
    // Unimplemented addresses read 0 -- IDU (Task 5) is the one that must
    // flag an access to a CSR outside M2's minimal set as illegal before
    // dispatch ever reaches here (contract 7's "everything else is absent,
    // not stubbed"); this mux is simply never consulted for those.
    //=========================================================================
    function automatic [63:0] csr_read_mux(input [11:0] addr);
        case (addr)
            CSR_MSTATUS:   csr_read_mux = mstatus_value;
            CSR_MISA:      csr_read_mux = misa_value;
            CSR_MIE:       csr_read_mux = mie_reg;
            CSR_MTVEC:     csr_read_mux = mtvec_value;
            CSR_MSCRATCH:  csr_read_mux = mscratch_reg;
            CSR_MEPC:      csr_read_mux = mepc_value;
            CSR_MCAUSE:    csr_read_mux = mcause_value;
            CSR_MTVAL:     csr_read_mux = mtval_reg;
            CSR_MIP:       csr_read_mux = mip_value;
            CSR_MCYCLE:    csr_read_mux = mcycle_reg;
            CSR_MINSTRET:  csr_read_mux = minstret_reg;
            CSR_MVENDORID: csr_read_mux = mvendorid_value;
            CSR_MARCHID:   csr_read_mux = marchid_value;
            CSR_MIMPID:    csr_read_mux = mimpid_value;
            CSR_MHARTID:   csr_read_mux = mhartid_value;
            CSR_MXSTATUS:  csr_read_mux = mxstatus_value;
            CSR_MHCR:      csr_read_mux = mhcr_value;
            CSR_MHINT:     csr_read_mux = mhint_value;
            default:       csr_read_mux = 64'd0;
        endcase
    endfunction

    wire [63:0] csr_rdata = csr_read_mux(csr_addr);

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
    assign cp0_rtu_ex1_cmplt_dp = ex1_active && !fence_hold;
    assign cp0_idu_fencei_full  = fence_hold;
    // Task 7.3: the completing CSR instruction's length. CP0 completes in
    // EX1 (single cycle), so the completing instruction IS the live EX1
    // instruction -- no latching needed.
    assign cp0_rtu_ex1_inst_len = idu_cp0_ex1_inst_len;

    // Synchronous exceptions CP0 itself detects: illegal instruction (any
    // CP0-dispatched op IDU already flagged illegal), ecall (M-mode only --
    // cause 11, no other priv level exists to ecall from), ebreak (cause 3).
    // Standard RISC-V cause encoding; neither is an interrupt.
    assign cp0_rtu_ex1_expt_vld = ex1_illegal || is_ecall || is_ebreak;
    assign cp0_rtu_ex1_expt_int = 1'b0;
    assign cp0_rtu_ex1_expt_vec = ex1_illegal ? 5'd2  :
                                  is_ecall     ? 5'd11 :
                                  is_ebreak    ? 5'd3  : 5'd0;

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

    assign cp0_xx_mrvbr = RESET_VECTOR[PC_WIDTH-1:0];

    // `bht_cp0_inv_done` (handshake-done input) has no consumer in M2's
    // minimal CSR set -- no BHT invalidate request is ever issued (tied 0 in
    // the MHCR fan-out above). `ifu_cp0_icache_inv_done` IS consumed: it
    // advances the FENCE.I fencei_state sequencer (SECTION DECODE note).
    // `iu_cp0_ex1_cur_pc` feeds the FENCE.I changeflow PC (PC+4) in addition
    // to its original trap-context role.

endmodule
