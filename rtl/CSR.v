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
// silent one.
//=============================================================================

import rvproc_pkg::*;

module CSR (
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
    output wire [63:0]              cp0_rtu_ex1_wb_data,
    output wire [GPR_IDX_WIDTH-1:0] cp0_rtu_ex1_wb_preg,
    output wire                     cp0_rtu_ex1_wb_vld,
    output wire                     cp0_rtu_ex1_expt_vld,
    output wire                     cp0_rtu_ex1_expt_int,
    output wire [4:0]               cp0_rtu_ex1_expt_vec,
    output wire                     cp0_rtu_ex1_chgflw,     // mret
    output wire [PC_WIDTH-1:0]      cp0_rtu_ex1_chgflw_pc,

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
    // SKELETON BODY (plan Task 2 replaces it): every output inactive/0.
    //=========================================================================
    assign cp0_rtu_ex1_cmplt_dp  = 1'b0;
    assign cp0_rtu_ex1_wb_data   = 64'd0;
    assign cp0_rtu_ex1_wb_preg   = {GPR_IDX_WIDTH{1'b0}};
    assign cp0_rtu_ex1_wb_vld    = 1'b0;
    assign cp0_rtu_ex1_expt_vld  = 1'b0;
    assign cp0_rtu_ex1_expt_int  = 1'b0;
    assign cp0_rtu_ex1_expt_vec  = 5'd0;
    assign cp0_rtu_ex1_chgflw    = 1'b0;
    assign cp0_rtu_ex1_chgflw_pc = {PC_WIDTH{1'b0}};

    assign cp0_ifu_icache_en       = 1'b0;   // MHCR.ie resets 0 (LSU/CP0 note B2)
    assign cp0_ifu_iwpe            = 1'b0;
    assign cp0_ifu_icache_pref_en  = 1'b0;
    assign cp0_ifu_icache_inv_addr = 64'd0;
    assign cp0_ifu_icache_inv_req  = 1'b0;
    assign cp0_ifu_icache_inv_type = 2'd0;
    assign cp0_ifu_bht_en          = 1'b0;
    assign cp0_ifu_btb_en          = 1'b0;
    assign cp0_ifu_ras_en          = 1'b0;
    assign cp0_ifu_bht_inv         = 1'b0;
    assign cp0_ifu_btb_clr         = 1'b0;

    assign cp0_lsu_dcache_en = 1'b0;         // MHCR.de resets 0 (LSU/CP0 note B2)
    assign cp0_lsu_mm        = 1'b1;         // MXSTATUS.mm resets 1 (contract 3)
    assign cp0_lsu_wa        = 1'b0;         // MHCR.wa resets 0 (contract 6)

    assign cp0_xx_mrvbr = {PC_WIDTH{1'b0}};

endmodule
