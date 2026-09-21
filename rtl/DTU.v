//=============================================================================
// DTU.v - core-side debug unit  (M7 Task 1)
//=============================================================================
// C906 files covered:
//   gen_rtl/dtu/rtl/aq_dtu_top.v    (port split: cp0/rtu/ifu/tdt_dm sides)
//   gen_rtl/dtu/rtl/aq_dtu_ctrl.v   (dcsr/dpc/dscratch0/1 storage, ebreak_
//                                    action, int_mask, low_power_wakeup)
//   gen_rtl/dtu/rtl/aq_dtu_cdc.v    (DM<->DTU handshake -- SAME-CLOCK in
//                                    rv906, D-M7-1: no synchronizers, direct
//                                    wires + 1-cycle response flops)
// References: M7 design doc (docs/superpowers/specs/2026-09-11-m7-debug-
// design.md, decisions D-M7-1/D-M7-6/D-M7-7/D-M7-10), donor machinery note
// (WIRING MAP).
//
// TASK-1 SUBSET: dcsr/dpc/dscratch0/dscratch1 ONLY. Trigger CSRs
// (tselect/tdata/tinfo/tcontrol/mcontext/scontext, donor aq_dtu_trigger_
// module.v + aq_dtu_m_iie_all.v), halt_info bundle generation, the pending-
// halt record machinery, pcfifo/dbgfifo (T-Head custom 0xFE0-0xFE2, dropped
// per D-M7-6) all arrive at Task 2.
//
// TASK-1 CAUSE PLUMBING DEVIATION (flagged, not silent): in the donor the
// dcsr.cause source is the trigger module's `dtu_cause` (aq_dtu_m_iie_all.v
// :962-976), itself a copy of the RTU's halt cause bits inside
// rtu_dtu_retire_halt_info[CAUSE:CAUSE-3] (aq_rtu_retire.v:1194) -- i.e. the
// RTU's halt_cause rides a 22-bit retire-side bundle through the trigger
// module and back into the ctrl. Task 2 replaces this with the real
// halt_info flow. For Task 1 (no trigger module exists) the RTU exports the
// halt cause directly as `rtu_dtu_halt_cause[3:0]` (see RTU.v, the exact
// same 4-bit value the donor's bundle CAUSE field carries), and this module
// latches it at rtu_dtu_halt_ack exactly the way aq_dtu_ctrl.v:368-369
// latches dtu_cause. Same information, one hop shorter; recorded here.
//=============================================================================

import rvproc_pkg::*;

module DTU (
    input  wire        clk,
    input  wire        rst_n,

    //=========================================================================
    // CP0 <-> DTU : the donor's cp0 side (aq_dtu_top.v:15-210). CSR.v decodes
    // 0x7B0-0x7B3 and pulses wreg/rreg; this module does the debug-mode
    // (dbgon) write gating itself (aq_dtu_ctrl.v:314-317).
    //=========================================================================
    input  wire [11:0] cp0_dtu_addr,
    input  wire [63:0] cp0_dtu_wdata,
    input  wire        cp0_dtu_wreg,
    input  wire        cp0_dtu_rreg,
    input  wire [1:0]  cp0_yy_priv_mode,
    output wire [63:0] dtu_cp0_rdata,
    output wire [1:0]  dtu_cp0_dcsr_prv,
    output wire        dtu_cp0_dcsr_mprven,
    output wire        dtu_cp0_wake_up,

    //=========================================================================
    // TDT_DM <-> DTU : the DM side (the DM module itself is Task 3; these
    // ports are exercised by the unit benches and wired at Task 5). D-M7-1:
    // single clock domain, so the donor's aq_dtu_cdc synchronizers
    // degenerate to direct wires; the 1-cycle response registers
    // (itr/wr_vld payload staging, wr_ready, rx_data) are KEPT, same-clock
    // (aq_dtu_cdc.v:285-356,468-483).
    //=========================================================================
    input  wire        tdt_dm_dtu_halt_req,      // level, spec haltreq (timing-1)
    input  wire        tdt_dm_dtu_resume_req,    // pulse
    input  wire        tdt_dm_dtu_halt_on_reset, // level
    input  wire        tdt_dm_dtu_ack_havereset, // pulse
    input  wire [31:0] tdt_dm_dtu_itr,
    input  wire        tdt_dm_dtu_itr_vld,       // pulse
    input  wire        tdt_dm_dtu_wr_vld,        // pulse
    input  wire [1:0]  tdt_dm_dtu_wr_flg,        // 00=rd dscratch0 / 01=wr dscratch0
    input  wire [63:0] tdt_dm_dtu_wdata,
    output wire        dtu_tdt_dm_halted,        // = dbgon (aq_dtu_cdc.v:392-410)
    output wire        dtu_tdt_dm_havereset,     // level until ack (cdc :486-525)
    output wire        dtu_tdt_dm_itr_done,      // pulse per retired debug inst
    output wire        dtu_tdt_dm_retire_debug_expt_vld, // pulse through from RTU
    output wire        dtu_tdt_dm_wr_ready,      // pulse, wr/rd accepted
    output wire [63:0] dtu_tdt_dm_rx_data,       // readback: dscratch0

    //=========================================================================
    // RTU <-> DTU (aq_dtu_top.v). Halt-ack/dpc in; sync-halt/resume/step/
    // int-mask/ebreak-action/dpc out.
    //=========================================================================
    input  wire [63:0] rtu_dtu_dpc,        // would-be-next PC at halt_ack
    input  wire        rtu_dtu_halt_ack,   // pulse: halt taken this cycle
    input  wire [3:0]  rtu_dtu_halt_cause, // TASK-1 direct cause (header note)
    input  wire        rtu_dtu_retire_vld, // one pulse per retiring inst
    input  wire        rtu_dtu_retire_debug_expt_vld, // exception while dbgon
    input  wire        rtu_yy_xx_dbgon,    // dbg_mode_on
    output wire        dtu_rtu_sync_halt_req, // D-M7-1: = tdt_dm_dtu_halt_req
    output wire        dtu_rtu_resume_req,    // D-M7-1: = tdt_dm_dtu_resume_req
    output wire        dtu_rtu_step_en,       // = dcsr.step (aq_dtu_top.v:390)
    output wire        dtu_rtu_int_mask,      // = step && !stepie (ctrl :376)
    output wire        dtu_rtu_ebreak_action, // per-priv ebreak* (ctrl :377-379)
    output wire [63:0] dtu_rtu_dpc,           // = dpc reg (aq_dtu_top.v:392)
    output wire [63:0] dtu_rtu_pending_tval,  // tied 0 in Task 1 (trigger tval
                                              // arrives with Task 2; donor ties
                                              // it to dscratch0, ctrl :410)

    //=========================================================================
    // DTU -> IFU : debug-instruction (itr) injection + halt-on-reset.
    //=========================================================================
    output wire [31:0] dtu_ifu_debug_inst,     // = itr reg (aq_dtu_cdc.v:301)
    output wire        dtu_ifu_debug_inst_vld, // 1-cycle pulse per itr (cdc :292)
    output wire        dtu_ifu_halt_on_reset,  // D-M7-1: = tdt_dm_dtu_halt_on_reset

    //=========================================================================
    // DTU -> HPCP : dcsr.stopcount. Dangling in the SoC (rv906 has no HPCP);
    // documented dead output, kept for the donor's port shape (D-M7-10).
    //=========================================================================
    output wire        dtu_hpcp_dcsr_stopcount,

    //=========================================================================
    // DTU <-> IFU : execute-trigger match on the ibuf-head PC (M7 Task 2).
    // The IFU presents the PC of the instruction currently at its ibuf head
    // (about to be delivered to IDU this cycle -- the donor's aq_ifu_pred.v
    // PRED-stage position); the DTU compares it against the 8 mcontrol
    // execute triggers combinationally and returns the 22-bit halt_info
    // bundle the IFU passes live to IDU (rider: IFU->IDU->RTU).
    // D-M7-8: single-issue -- one head slot per cycle (the donor's 2
    // PRED-stage slots dtu_ifu_halt_info0/1 collapse to one; the donor's
    // slot-1 instruction is checked here the cycle IT reaches the head).
    //=========================================================================
    input  wire [39:0] ifu_dtu_exe_addr,
    input  wire        ifu_dtu_exe_addr_vld,
    output wire [TDT_HINFO_WIDTH-1:0] dtu_ifu_halt_info,
    output wire        dtu_ifu_halt_info_vld,

    //=========================================================================
    // DTU <-> LSU : ldst-trigger match on the store/load address (+data).
    // The LSU presents the AG-stage access; the DTU compares against the
    // mcontrol load/store triggers and returns the halt_info bundle (the
    // LSU latches it and returns it to RTU at the op's cmplt) plus the two
    // store-suppression enables (a trigger-hit store must not commit).
    // Type encoding (donor aq_dtu_trigger_module.v:247-250): bit0=store,
    // bit1=load, so 2'b10=store, 2'b01=load.
    //=========================================================================
    input  wire [39:0] lsu_dtu_ldst_addr,
    input  wire        lsu_dtu_ldst_addr_vld,
    input  wire [63:0] lsu_dtu_ldst_data,
    input  wire        lsu_dtu_ldst_data_vld,
    input  wire [1:0]  lsu_dtu_ldst_type,
    input  wire [15:0] lsu_dtu_ldst_bytes_vld,
    input  wire [2:0]  lsu_dtu_mem_access_size,
    output wire [TDT_HINFO_WIDTH-1:0] dtu_lsu_halt_info,
    output wire        dtu_lsu_halt_info_vld,
    output wire        dtu_lsu_addr_trig_en,   // store suppressed (addr match)
    output wire        dtu_lsu_data_trig_en,   // store suppressed (data match)

    //=========================================================================
    // RTU <-> DTU (M7 Task 2 trigger side): the retire-side halt_info the
    // RTU latched onto the retiring instruction (drives the DTU's
    // dcsr.cause / pending-halt record, donor aq_dtu_m_iie_all.v:884-1015),
    // and the pending-halt record handshake.
    //=========================================================================
    input  wire [TDT_HINFO_WIDTH-1:0] rtu_dtu_retire_halt_info,
    input  wire        rtu_dtu_pending_ack,
    output wire        dtu_rtu_pending_halt     // level: timing-1 halt armed
);

    //=========================================================================
    // SECTION WRITE DECODE (aq_dtu_ctrl.v:314-318). cp0 writes are gated on
    // rtu_yy_xx_dbgon -- outside debug mode dcsr/dpc/dscratch writes are
    // dropped (the CSR.v side has already passed its privilege check; the
    // 0.13 "these CSRs are only accessible in debug mode" rule is enforced
    // HERE, not as an illegal-instruction trap, mirroring the donor).
    //=========================================================================
    wire cp0_write_dcsr      = cp0_dtu_wreg && (cp0_dtu_addr == CSR_DCSR)      && rtu_yy_xx_dbgon;
    wire cp0_write_dpc       = cp0_dtu_wreg && (cp0_dtu_addr == CSR_DPC)       && rtu_yy_xx_dbgon;
    wire cp0_write_dscratch0 = cp0_dtu_wreg && (cp0_dtu_addr == CSR_DSCRATCH0) && rtu_yy_xx_dbgon;
    wire cp0_write_dscratch1 = cp0_dtu_wreg && (cp0_dtu_addr == CSR_DSCRATCH1) && rtu_yy_xx_dbgon;
    // DM-side dscratch0 write (aq_dtu_ctrl.v:318): wr_flg==2'b01, dbgon-gated.
    wire tdt_dm_write_dscratch0 = tdt_dm_dtu_wr_vld && (tdt_dm_dtu_wr_flg == 2'b01)
                                && rtu_yy_xx_dbgon;

    //=========================================================================
    // SECTION DCSR (aq_dtu_ctrl.v:296-379) -- 0.13 layout:
    // {xdebugver=4'b0100[31:28], 12'b0[27:16], ebreakm[15], 1'b0[14],
    //  ebreaks[13], ebreaku[12], stepie[11], stopcount[10], 1'b0[9],
    //  cause[8:6], 1'b0[5], mprven[4], nmip=0[3], step[2], prv[1:0]}.
    //=========================================================================
    localparam DCSR_PRV_HI    = 1;
    localparam DCSR_STEP      = 2;
    localparam DCSR_NMIP      = 3;    // tied 0
    localparam DCSR_MPRVEN    = 4;
    localparam DCSR_CAUSE_HI  = 8;    // [8:6]
    localparam DCSR_STOPCOUNT = 10;
    localparam DCSR_STEPIE    = 11;
    localparam DCSR_EBREAKU   = 12;
    localparam DCSR_EBREAKS   = 13;
    localparam DCSR_EBREAKM   = 15;

    reg        dcsr_step_r;
    reg        dcsr_mprven_r;
    reg        dcsr_stopcount_r;
    reg        dcsr_stepie_r;
    reg        dcsr_ebreaku_r;
    reg        dcsr_ebreaks_r;
    reg        dcsr_ebreakm_r;
    reg [1:0]  dcsr_prv_r;
    reg [2:0]  dcsr_cause_r;
    reg [63:0] dpc_r;
    reg [63:0] dscratch0_r;
    reg [63:0] dscratch1_r;

    // Software-writable field set (aq_dtu_ctrl.v:326-348).
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dcsr_step_r      <= 1'b0;
            dcsr_mprven_r    <= 1'b0;
            dcsr_stopcount_r <= 1'b0;
            dcsr_stepie_r    <= 1'b0;
            dcsr_ebreaku_r   <= 1'b0;
            dcsr_ebreaks_r   <= 1'b0;
            dcsr_ebreakm_r   <= 1'b0;
        end else if (cp0_write_dcsr) begin
            dcsr_step_r      <= cp0_dtu_wdata[DCSR_STEP];
            dcsr_mprven_r    <= cp0_dtu_wdata[DCSR_MPRVEN];
            dcsr_stopcount_r <= cp0_dtu_wdata[DCSR_STOPCOUNT];
            dcsr_stepie_r    <= cp0_dtu_wdata[DCSR_STEPIE];
            dcsr_ebreaku_r   <= cp0_dtu_wdata[DCSR_EBREAKU];
            dcsr_ebreaks_r   <= cp0_dtu_wdata[DCSR_EBREAKS];
            dcsr_ebreakm_r   <= cp0_dtu_wdata[DCSR_EBREAKM];
        end
    end

    // dcsr.prv (aq_dtu_ctrl.v:353-361): latched from cp0_yy_priv_mode at
    // halt_ack (the mode the core was in when it halted); ALSO writable by a
    // debug-mode dcsr write -- the donor has a cp0_write_dcsr arm (halt_ack
    // wins ties), so the debugger can set the resume privilege explicitly.
    // (The M7 Task-1 prompt's "prv/cause are latch-only" conflicts with the
    // donor here; clone discipline follows the donor.)
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            dcsr_prv_r <= 2'b0;
        else if (rtu_dtu_halt_ack)
            dcsr_prv_r <= cp0_yy_priv_mode;
        else if (cp0_write_dcsr)
            dcsr_prv_r <= cp0_dtu_wdata[DCSR_PRV_HI:0];
    end

    // dcsr.cause (aq_dtu_ctrl.v:364-370): latch-only at halt_ack. The cause
    // value itself is the RTU's (Task-1 direct export, header note).
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            dcsr_cause_r <= 3'b0;
        else if (rtu_dtu_halt_ack)
            dcsr_cause_r <= rtu_dtu_halt_cause[2:0];
    end

    wire [31:0] dcsr_value = {4'b0100, 12'b0,                       // xdebugver=0.13, reserved
                              dcsr_ebreakm_r, 1'b0,
                              dcsr_ebreaks_r, dcsr_ebreaku_r,
                              dcsr_stepie_r, dcsr_stopcount_r, 1'b0,
                              dcsr_cause_r, 1'b0,
                              dcsr_mprven_r, 1'b0,                  // nmip=0
                              dcsr_step_r, dcsr_prv_r};

    // int_mask + ebreak_action (aq_dtu_ctrl.v:376-379).
    assign dtu_rtu_int_mask = dcsr_step_r && !dcsr_stepie_r;
    assign dtu_rtu_ebreak_action = (cp0_yy_priv_mode == PRIV_U) && dcsr_ebreaku_r
                                 || (cp0_yy_priv_mode == PRIV_S) && dcsr_ebreaks_r
                                 || (cp0_yy_priv_mode == PRIV_M) && dcsr_ebreakm_r;

    //=========================================================================
    // SECTION DPC (aq_dtu_ctrl.v:384-392) -- three arms ONLY, clone-faithful:
    // reset, cp0 write (debug-mode redirect), and halt_ack latch from the
    // RTU's rtu_dtu_dpc. The donor does NOT auto-increment dpc per retired
    // debug instruction -- dpc advances via the RTU's rtu_dtu_dpc on each
    // halt_ack (which fires on every single-step retire, aq_rtu_retire.v:
    // 1223 halt_ack=halt_req incl. the step leg). Do not add a +4 arm.
    //=========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            dpc_r <= 64'd0;
        else if (cp0_write_dpc)
            dpc_r <= cp0_dtu_wdata;
        else if (rtu_dtu_halt_ack)
            dpc_r <= rtu_dtu_dpc;
    end

    //=========================================================================
    // SECTION DSCRATCH0/1 (aq_dtu_ctrl.v:398-420). dscratch0's write
    // priority: trigger-tval update > cp0 write > DM write (donor order
    // :402-407; the updata_tval term is Task-2 scope, dropped here).
    // dscratch1 is cp0-write-only.
    //=========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            dscratch0_r <= 64'd0;
        else if (cp0_write_dscratch0)
            dscratch0_r <= cp0_dtu_wdata;
        else if (tdt_dm_write_dscratch0)
            dscratch0_r <= tdt_dm_dtu_wdata;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            dscratch1_r <= 64'd0;
        else if (cp0_write_dscratch1)
            dscratch1_r <= cp0_dtu_wdata;
    end

    //=========================================================================
    // SECTION READ MUX -- moved to the TRIGGER section below (the trigger
    // CSR readback terms reference the trigger storage declared there; kept
    // as the single driver of dtu_cp0_rdata at the end of the module).
    //=========================================================================

    //=========================================================================
    // SECTION CP0/RTU/HPCP OUTPUTS (aq_dtu_top.v:354-398).
    //=========================================================================
    assign dtu_cp0_dcsr_prv    = dcsr_prv_r;
    assign dtu_cp0_dcsr_mprven = dcsr_mprven_r;

    // low-power wakeup (aq_dtu_ctrl.v:623-626, minus the dropped async-halt
    // and pending-halt terms -- D-M7-7 drops the async halt; pending_halt is
    // Task-2 trigger machinery).
    assign dtu_cp0_wake_up = tdt_dm_dtu_halt_req || (dcsr_step_r && !rtu_yy_xx_dbgon);

    assign dtu_rtu_sync_halt_req = tdt_dm_dtu_halt_req;   // D-M7-1 direct
    assign dtu_rtu_resume_req    = tdt_dm_dtu_resume_req; // D-M7-1 direct
    assign dtu_rtu_step_en       = dcsr_step_r;
    assign dtu_rtu_dpc           = dpc_r;
    assign dtu_rtu_pending_tval  = 64'd0;                 // Task 2

    assign dtu_hpcp_dcsr_stopcount = dcsr_stopcount_r;    // dangling in SoC

    //=========================================================================
    // SECTION DM->DTU SAME-CLOCK HANDSHAKES (D-M7-1; aq_dtu_cdc.v degenerated
    // to direct wires + the donor's 1-cycle staging flops kept same-clock).
    //
    // itr (aq_dtu_cdc.v:285-301): the itr payload registers on the vld pulse;
    // dtu_ifu_debug_inst_vld is the SAME-cycle echo of tdt_dm_dtu_itr_vld --
    // rv906's IFU consumes the injection the cycle it is presented (the
    // donor's extra register stage was CDC latency, not pipeline latency).
    //=========================================================================
    reg [31:0] itr_reg;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            itr_reg <= 32'd0;
        else if (tdt_dm_dtu_itr_vld)
            itr_reg <= tdt_dm_dtu_itr;
    end
    assign dtu_ifu_debug_inst     = tdt_dm_dtu_itr_vld ? tdt_dm_dtu_itr : itr_reg;
    assign dtu_ifu_debug_inst_vld = tdt_dm_dtu_itr_vld;
    assign dtu_ifu_halt_on_reset  = tdt_dm_dtu_halt_on_reset;

    //=========================================================================
    // SECTION DTU->DM RESPONSES (aq_dtu_cdc.v:392-483, same-clock).
    //=========================================================================
    // halted = dbgon (aq_dtu_cdc.v:392-410, direct).
    assign dtu_tdt_dm_halted = rtu_yy_xx_dbgon;

    // itr_done: one pulse per retiring debug-mode instruction. Donor source
    // is `rtu_dtu_retire_vld && dbgon` (aq_dtu_cdc.v:413) pulse-synced to the
    // DM clock; same-clock in rv906, so the pulse is the term itself
    // (registered one cycle to keep the donor's "DM sees it after the
    // retire" ordering -- cdc:415's pulse-sync delivers one dst cycle after
    // the source pulse). The unit bench confirms the exact N.
    reg itr_done_r;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            itr_done_r <= 1'b0;
        else
            itr_done_r <= rtu_dtu_retire_vld && rtu_yy_xx_dbgon;
    end
    assign dtu_tdt_dm_itr_done = itr_done_r;

    // retire_debug_expt_vld: the RTU's debug-mode exception pulse, passed
    // through with the same 1-cycle registration as itr_done (cdc:433).
    reg debug_expt_r;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            debug_expt_r <= 1'b0;
        else
            debug_expt_r <= rtu_dtu_retire_debug_expt_vld;
    end
    assign dtu_tdt_dm_retire_debug_expt_vld = debug_expt_r;

    //=========================================================================
    // SECTION WR/RX DATA PATH (aq_dtu_cdc.v:323-386,449-483, same-clock).
    // wr_flg/wdata stage on the wr_vld pulse; wr_ready pulses one cycle
    // later; rx_data latches the readback at wr_ready. wr_flg 2'b10/2'b11
    // (latest_pc/satp) are DROPPED per D-M7-6 -- rx_data = 0 for them.
    //=========================================================================
    reg [1:0] wr_flg_r;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            wr_flg_r <= 2'b00;
        else if (tdt_dm_dtu_wr_vld)
            wr_flg_r <= tdt_dm_dtu_wr_flg;
    end

    reg wr_ready_r;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            wr_ready_r <= 1'b0;
        else
            wr_ready_r <= tdt_dm_dtu_wr_vld;
    end
    assign dtu_tdt_dm_wr_ready = wr_ready_r;

    reg [63:0] rx_data_r;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            rx_data_r <= 64'd0;
        else if (tdt_dm_dtu_wr_vld)
            rx_data_r <= (tdt_dm_dtu_wr_flg == 2'b00) ? dscratch0_r : 64'd0;
    end
    assign dtu_tdt_dm_rx_data = rx_data_r;

    //=========================================================================
    // SECTION HAVERESET FSM (aq_dtu_cdc.v:486-525, same-clock port, D-M7-1:
    // no sys_apb_clk synchronizer). After rst_n release, havereset asserts
    // until the DM's ack_havereset pulse.
    //=========================================================================
    localparam [1:0] HR_IDLE       = 2'b00;
    localparam [1:0] HR_PULSE      = 2'b01;
    localparam [1:0] HR_HAVE_RESET = 2'b10;
    localparam [1:0] HR_PENDING    = 2'b11;

    reg [1:0] hr_state;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            hr_state <= HR_IDLE;
        else case (hr_state)
            HR_IDLE:       hr_state <= HR_PULSE;
            HR_PULSE:      hr_state <= HR_HAVE_RESET;
            HR_HAVE_RESET: hr_state <= tdt_dm_dtu_ack_havereset ? HR_PENDING
                                                              : HR_HAVE_RESET;
            HR_PENDING:    hr_state <= HR_PENDING;
            default:       hr_state <= HR_IDLE;
        endcase
    end

    reg havereset_r;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            havereset_r <= 1'b0;
        else
            havereset_r <= (hr_state == HR_HAVE_RESET);
    end
    assign dtu_tdt_dm_havereset = havereset_r;

    // tdt_dm_dtu_wr_flg's readback-select register (wr_flg_r) is only read
    // by the rx_data path's Task-2 extension; the current body selects on
    // the live wr_flg at the wr_vld pulse itself. Keep the staged register
    // for the donor's port shape (cdc:341-347).

    //=========================================================================
    // SECTION TRIGGERS (M7 Task 2; donor aq_dtu_m_iie_all.v +
    // aq_dtu_mcontrol.v + aq_dtu_iie_trigger.v). 10 triggers: slots 0-7 =
    // mcontrol (type 2), slots 8-9 = iie (type 3 icount, count hardwired 1).
    //
    // D-M7-5 (deviation, flagged): tdata1 uses the STANDARD RISC-V Debug
    // 0.13 mcontrol layout, NOT the donor's T-Head custom layout (which
    // splits a 4-bit "size" field across [SIZEHI:SIZELO] and an action
    // one-hot). Standard layout (bits of the 64-bit tdata1):
    //   [63:60] type      [59] dmode     [58:52] maskmax
    //   [51:20] data1     [19] select    [18] timing
    //   [17:12] action    [11] chain     [10:7]  match
    //   [6] mmode         [5] hmode      [4] smode   [3] umode
    //   [2] execute       [1] store      [0] load
    // match modes 0-5 = eq/NAPOT/ge/lt/low32/up32 (donor aq_dtu_mcontrol.v
    // :1125-1130,1294-1299). action 0 = breakpoint exception (cause 3),
    // action 1 = enter debug (halt, cause 2). The donor's per-trigger
    // enable is `tcontrol_enable = !(!MTE && action==0 && m_mode)`
    // (aq_dtu_mcontrol.v:711) -- MTE (tcontrol[3]) is the master enable;
    // MPTE (tcontrol[7]) is only the mret save/restore copy, NOT a match
    // gate. Timing is forced 0 for execute triggers (donor :526).
    //=========================================================================

    // ---- standard mcontrol tdata1 bit positions (D-M7-5 layout) ----
    localparam MD_TYPE_HI    = 63;
    localparam MD_TYPE_LO    = 60;
    localparam MD_DMODE      = 59;
    localparam MD_MASK_HI    = 58;
    localparam MD_MASK_LO    = 52;
    localparam MD_DATA1_HI   = 51;
    localparam MD_DATA1_LO   = 20;
    localparam MD_SELECT     = 19;
    localparam MD_TIMING     = 18;
    localparam MD_ACTION_HI  = 17;
    localparam MD_ACTION_LO  = 12;
    localparam MD_CHAIN      = 11;
    localparam MD_MATCH_HI   = 10;
    localparam MD_MATCH_LO   = 7;
    localparam MD_MMODE      = 6;
    localparam MD_HMODE      = 5;
    localparam MD_SMODE      = 4;
    localparam MD_UMODE      = 3;
    localparam MD_EXECUTE    = 2;
    localparam MD_STORE      = 1;
    localparam MD_LOAD       = 0;
    // iie (type 3 icount) tdata1 (donor aq_dtu_iie_trigger.v:174-184): the
    // standard icount layout puts type [63:60]=3, count [23:10]=1 (hard-
    // wired), m [6] (donor uses M=9 but that is its custom layout; the
    // standard icount uses the same privilege bits as mcontrol, so reuse
    // the mcontrol positions for privilege/action to keep one decoder).
    localparam IE_TYPE_HI    = 63;
    localparam IE_TYPE_LO    = 60;
    localparam IE_COUNT      = 10;     // count, hardwired 1
    localparam IE_MMODE      = 6;
    localparam IE_SMODE      = 4;
    localparam IE_UMODE      = 3;
    localparam IE_ACTION     = 2;      // 0=breakpoint,1=debug (1 bit, std)

    // ---- trigger storage --------------------------------------------------
    // tselect: 4 bits, WARL-clamped to 0..TDT_TM_TRI_NUM-1 (0..9). Donor
    // aq_dtu_m_iie_all.v:376-386 (wdata>=TRIGGER_NUM or any high bit ->
    // clamp to TRIGGER_NUM=9).
    reg [3:0] tselect_lowbits;
    wire tselect_wdata_big = (cp0_dtu_wdata[3:0] >= 4'd9)
                                  || (|cp0_dtu_wdata[63:4]);
    wire cp0_write_tselect = cp0_dtu_wreg && (cp0_dtu_addr == CSR_TSELECT);
    wire cp0_write_tdata1  = cp0_dtu_wreg && (cp0_dtu_addr == CSR_TDATA1);
    wire cp0_write_tdata2  = cp0_dtu_wreg && (cp0_dtu_addr == CSR_TDATA2);
    wire cp0_write_tdata3  = cp0_dtu_wreg && (cp0_dtu_addr == CSR_TDATA3);
    wire cp0_write_tcontrol= cp0_dtu_wreg && (cp0_dtu_addr == CSR_TCONTROL);
    wire cp0_write_mcontext= cp0_dtu_wreg && (cp0_dtu_addr == CSR_MCONTEXT);
    wire cp0_write_scontext= cp0_dtu_wreg && (cp0_dtu_addr == CSR_SCONTEXT);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            tselect_lowbits <= 4'd0;
        else if (cp0_write_tselect)
            tselect_lowbits <= tselect_wdata_big ? 4'd9 : cp0_dtu_wdata[3:0];
    end

    // one-hot select from tselect (donor :391-407).
    reg [9:0] tsel_oh;
    always @* begin
        case (tselect_lowbits)
            4'd0: tsel_oh = 10'b0000000001;
            4'd1: tsel_oh = 10'b0000000010;
            4'd2: tsel_oh = 10'b0000000100;
            4'd3: tsel_oh = 10'b0000001000;
            4'd4: tsel_oh = 10'b0000010000;
            4'd5: tsel_oh = 10'b0000100000;
            4'd6: tsel_oh = 10'b0001000000;
            4'd7: tsel_oh = 10'b0010000000;
            4'd8: tsel_oh = 10'b0100000000;
            4'd9: tsel_oh = 10'b1000000000;
            default: tsel_oh = 10'b0;
        endcase
    end

    // mcontrol slots 0-7: WARL-legalized tdata1 + tdata2 (match value) +
    // tdata3 (scratch). tdata2/tdata3 are plain storage (donor: tdata2 is
    // the match address/data, tdata3 is the mcontext/scontext context the
    // trigger compares -- plain storage here, the mcontext/scontext match
    // is not exercised by the M7 gate set).
    reg [63:0] m_tdata1 [0:7];
    reg [63:0] m_tdata2 [0:7];
    reg [63:0] m_tdata3 [0:7];
    // iie slots 8-9: type 3 icount (count hardwired 1).
    reg [63:0] iie_tdata1 [0:1];
    reg [63:0] iie_tdata2 [0:1];
    reg [63:0] iie_tdata3 [0:1];

    // mcontrol tdata1 WARL legalization (donor aq_dtu_mcontrol.v:480-536,
    // adapted to the standard layout). type must be 2 (mcontrol) or the slot
    // reads back type=0 (disabled); unsupported T-Head types 4/5 also read 0.
    wire [3:0] md_type_legal = (cp0_dtu_wdata[MD_TYPE_HI:MD_TYPE_LO] == 4'd2)
                             ? 4'd2 : 4'd0;
    wire       md_dmode_legal = rtu_yy_xx_dbgon ? cp0_dtu_wdata[MD_DMODE]
                                               : 1'b0;   // dmode sticky 0 outside debug
    wire [3:0] md_match_legal = (cp0_dtu_wdata[MD_MATCH_HI:MD_MATCH_LO] <= 4'd5)
                               ? cp0_dtu_wdata[MD_MATCH_HI:MD_MATCH_LO] : 4'd0;
    wire [5:0] md_action_legal = (cp0_dtu_wdata[MD_ACTION_HI:MD_ACTION_LO] <= 6'd1)
                                ? cp0_dtu_wdata[MD_ACTION_HI:MD_ACTION_LO] : 6'd0;
    wire       md_timing_legal = cp0_dtu_wdata[MD_EXECUTE] ? 1'b0
                                                          : cp0_dtu_wdata[MD_TIMING];
    wire [63:0] m_tdata1_legal = {
        md_type_legal,                                   // [63:60]
        md_dmode_legal,                                  // [59]
        cp0_dtu_wdata[MD_MASK_HI:MD_MASK_LO],            // [58:52] maskmax
        cp0_dtu_wdata[MD_DATA1_HI:MD_DATA1_LO],          // [51:20] data1
        cp0_dtu_wdata[MD_SELECT],                        // [19]
        md_timing_legal,                                 // [18]
        md_action_legal,                                 // [17:12]
        cp0_dtu_wdata[MD_CHAIN],                          // [11]
        md_match_legal,                                  // [10:7]
        cp0_dtu_wdata[MD_MMODE],                          // [6]
        1'b0,                                             // [5] hmode (no H)
        cp0_dtu_wdata[MD_SMODE],                          // [4]
        cp0_dtu_wdata[MD_UMODE],                          // [3]
        cp0_dtu_wdata[MD_EXECUTE],                        // [2]
        cp0_dtu_wdata[MD_STORE],                          // [1]
        cp0_dtu_wdata[MD_LOAD]};                          // [0]

    // iie tdata1 WARL: type must be 3 (icount) else 0; count hardwired 1.
    wire [3:0] ie_type_legal = (cp0_dtu_wdata[IE_TYPE_HI:IE_TYPE_LO] == 4'd3)
                             ? 4'd3 : 4'd0;
    wire [63:0] iie_tdata1_legal = {
        ie_type_legal,                                   // [63:60]
        1'b0,                                             // [59]
        34'b0,                                            // [58:25]
        1'b1,                                             // [24] icount_hit placeholder (0)
        13'b0,                                            // [23:11]
        1'b1,                                             // [10] count=1 (hardwired)
        3'b0,                                             // [9:7]
        cp0_dtu_wdata[IE_MMODE],                          // [6]
        1'b0,                                             // [5]
        cp0_dtu_wdata[IE_SMODE],                          // [4]
        cp0_dtu_wdata[IE_UMODE],                          // [3]
        cp0_dtu_wdata[IE_ACTION],               // [2]
        1'b0,                                             // [1]
        1'b0};                                            // [0]

    integer ti;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (ti = 0; ti < 8; ti = ti + 1) begin
                m_tdata1[ti] <= 64'd0; m_tdata2[ti] <= 64'd0; m_tdata3[ti] <= 64'd0;
            end
            for (ti = 0; ti < 2; ti = ti + 1) begin
                iie_tdata1[ti] <= 64'd0; iie_tdata2[ti] <= 64'd0; iie_tdata3[ti] <= 64'd0;
            end
        end else begin
            for (ti = 0; ti < 8; ti = ti + 1) begin
                if (cp0_write_tdata1 && tsel_oh[ti]) m_tdata1[ti] <= m_tdata1_legal;
                if (cp0_write_tdata2 && tsel_oh[ti]) m_tdata2[ti] <= cp0_dtu_wdata;
                if (cp0_write_tdata3 && tsel_oh[ti]) m_tdata3[ti] <= cp0_dtu_wdata;
            end
            for (ti = 0; ti < 2; ti = ti + 1) begin
                if (cp0_write_tdata1 && tsel_oh[ti+8]) iie_tdata1[ti] <= iie_tdata1_legal;
                if (cp0_write_tdata2 && tsel_oh[ti+8]) iie_tdata2[ti] <= cp0_dtu_wdata;
                if (cp0_write_tdata3 && tsel_oh[ti+8]) iie_tdata3[ti] <= cp0_dtu_wdata;
            end
        end
    end

    // ---- tcontrol / mcontext / scontext storage ---------------------------
    // MTE (bit 3) + MPTE (bit 7). Only MTE gates matches (donor :711); MPTE
    // is stored+read back for the donor's readback fidelity but the mret
    // save/restore mechanism is DROPPED (D-M7-9: not exercised by the M7
    // gate set, would need cp0_dtu_mexpt_vld/retire_mret ports).
    reg tcontrol_mte_r, tcontrol_mpte_r;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tcontrol_mte_r   <= 1'b0;
            tcontrol_mpte_r  <= 1'b0;
        end else if (cp0_write_tcontrol) begin
            tcontrol_mte_r   <= cp0_dtu_wdata[3];
            tcontrol_mpte_r  <= cp0_dtu_wdata[7];
        end
    end
    wire tcontrol_mte = tcontrol_mte_r;

    reg [12:0] mcontext_lowbits;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) mcontext_lowbits <= 13'b0;
        else if (cp0_write_mcontext) mcontext_lowbits <= cp0_dtu_wdata[12:0];
    end
    reg [33:0] scontext_lowbits;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) scontext_lowbits <= 34'b0;
        else if (cp0_write_scontext) scontext_lowbits <= cp0_dtu_wdata[33:0];
    end

    // ---- trigger CSR read mux (donor aq_dtu_m_iie_all.v:413-611) ---------
    // Extends the existing dtu_cp0_rdata_r case: the trigger CSRs return the
    // selected slot's value (tdata1/2/3/tinfo) or the global tcontrol/
    // mcontext/scontext/tselect.
    wire [63:0] trig_tinfo = (tselect_lowbits >= 4'd8) ? 64'h0000_0000_0000_0030
                                                       : 64'h0000_0000_0000_0010;
    // mcontrol tinfo[9:4]=6'b001000 (D-M7-5, reduced from donor 6'b111000);
    // iie tinfo[9:4]=6'b111000 (donor aq_dtu_iie_trigger.v:373).

    // ---- privilege decode (donor aq_dtu_trigger_module.v:227-229) --------
    wire m_mode = (cp0_yy_priv_mode == PRIV_M);
    wire s_mode = (cp0_yy_priv_mode == PRIV_S);
    wire u_mode = (cp0_yy_priv_mode == PRIV_U);

    // ---- per-slot comparators (donor aq_dtu_mcontrol.v:711-1330) ---------
    // Two accesses are checked against every mcontrol slot: the IFU fetch
    // (execute) and the LSU ldst. A slot matches an access only if it is
    // type-2, privilege-ok, tcontrol-enabled, not in debug mode, its access
    // class (exe/ld/st) is selected, and the match-mode compare passes.
    wire [63:0] exe_addr64  = {24'b0, ifu_dtu_exe_addr};
    wire [63:0] ldst_addr64 = {24'b0, lsu_dtu_ldst_addr};
    wire        ldst_is_ld  = lsu_dtu_ldst_type[1];
    wire        ldst_is_st  = lsu_dtu_ldst_type[0];

    // Pure match-mode comparator (donor :1125-1130 / :1294-1299):
    //   0=eq  1=NAPOT  2=ge  3=lt  4=low32  5=up32
    function automatic mcontrol_addr_match;
        input [63:0] t1;   // tdata1 (provides the match mode)
        input [63:0] t2;   // tdata2 (match value)
        input [63:0] val;  // value to compare (address or data)
        begin
            case (t1[MD_MATCH_HI:MD_MATCH_LO])
                4'd0:    mcontrol_addr_match = (val == t2);                        // eq
                4'd1:    mcontrol_addr_match = ((val & ~t2) == (t2 & ~t2));        // NAPOT
                4'd2:    mcontrol_addr_match = (val >= t2);                        // ge
                4'd3:    mcontrol_addr_match = (val <  t2);                        // lt
                4'd4:    mcontrol_addr_match = (val[31:0] == t2[31:0]);            // low32
                4'd5:    mcontrol_addr_match = (val[63:32] == t2[63:32]);          // up32
                default: mcontrol_addr_match = 1'b0;
            endcase
        end
    endfunction

    // Per-slot match results collected into vectors (one bit per slot).
    wire [7:0] m_exe_match_vec;
    wire [7:0] m_exe_action01_vec;
    wire [7:0] m_ld_match_vec;
    wire [7:0] m_ld_action01_vec;
    wire [7:0] m_ld_timing_vec;
    wire [7:0] m_ld_t0_cancel_vec;
    wire [7:0] m_ld_addr_trig_vec;
    wire [7:0] m_ld_data_trig_vec;

    genvar gk;
    generate
        for (gk = 0; gk < 8; gk = gk + 1) begin : mslot
            wire [63:0] s_t1 = m_tdata1[gk];
            wire [63:0] s_t2 = m_tdata2[gk];
            // slot base enable (donor :711,745-756)
            wire s_active  = (s_t1[MD_TYPE_HI:MD_TYPE_LO] == 4'd2);
            wire s_priv    = (s_t1[MD_MMODE] && m_mode)
                           || (s_t1[MD_SMODE] && s_mode)
                           || (s_t1[MD_UMODE] && u_mode);
            wire s_action0 = (s_t1[MD_ACTION_HI:MD_ACTION_LO] == 6'd0);
            wire s_tctl    = !(!tcontrol_mte && s_action0 && m_mode);
            wire s_en      = s_active && s_priv && s_tctl && !rtu_yy_xx_dbgon
                           && (s_t1[MD_DMODE] == 1'b0);
            // execute access (donor exe0/exe1 collapse to one, D-M7-8)
            wire s_exe        = s_en && s_t1[MD_EXECUTE];
            wire s_exe_match  = s_exe && ifu_dtu_exe_addr_vld
                              && mcontrol_addr_match(s_t1, s_t2, exe_addr64);
            // ldst access
            wire s_ld         = s_en && s_t1[MD_LOAD]  && ldst_is_ld;
            wire s_st         = s_en && s_t1[MD_STORE] && ldst_is_st;
            wire s_ldst       = s_ld || s_st;
            wire s_sel        = s_t1[MD_SELECT];
            wire [63:0] s_cmp_val = s_sel ? lsu_dtu_ldst_data : ldst_addr64;
            wire s_ldst_match = s_ldst && lsu_dtu_ldst_addr_vld
                              && mcontrol_addr_match(s_t1, s_t2, s_cmp_val);
            assign m_exe_match_vec[gk]    = s_exe_match;
            assign m_exe_action01_vec[gk] = s_en ? ~s_action0 : 1'b0;
            assign m_ld_match_vec[gk]     = s_ldst_match;
            assign m_ld_action01_vec[gk]  = s_en ? ~s_action0 : 1'b0;
            assign m_ld_timing_vec[gk]    = s_en ? s_t1[MD_TIMING] : 1'b0;
            // CANCEL source (donor ldst_cancel, aq_dtu_mcontrol_output_
            // select.v:3380-3387): a timing-0 ldst match, EITHER access
            // type and EITHER action -- the LSU traps it at AG (the access
            // never commits); a timing-1 match does NOT cancel (the spec's
            // "after completion" semantics: side effects happen first).
            assign m_ld_t0_cancel_vec[gk] = s_ldst_match && !s_t1[MD_TIMING];
            // store-suppression enables (spec's dtu_lsu_{addr,data}_trig_
            // en): a timing-0 matched STORE trigger, address- or data-
            // matching. Timing-1 store triggers do NOT suppress (same
            // after-completion semantics as CANCEL above).
            assign m_ld_addr_trig_vec[gk] = s_st && !s_sel && s_ldst_match && !s_t1[MD_TIMING];
            assign m_ld_data_trig_vec[gk] = s_st &&  s_sel && s_ldst_match && !s_t1[MD_TIMING];
        end
    endgenerate

    // ---- halt_info assembly (lowest-index matching slot wins) ------------
    // Priority encoder: lowest set bit index.
    function automatic [3:0] lowbit_index;
        input [7:0] v;
        begin
            if      (v[0]) lowbit_index = 4'd0;
            else if (v[1]) lowbit_index = 4'd1;
            else if (v[2]) lowbit_index = 4'd2;
            else if (v[3]) lowbit_index = 4'd3;
            else if (v[4]) lowbit_index = 4'd4;
            else if (v[5]) lowbit_index = 4'd5;
            else if (v[6]) lowbit_index = 4'd6;
            else if (v[7]) lowbit_index = 4'd7;
            else           lowbit_index = 4'd0;
        end
    endfunction

    wire [3:0] exe_sel = lowbit_index(m_exe_match_vec);
    wire [3:0] ld_sel  = lowbit_index(m_ld_match_vec);
    wire        exe_any = |m_exe_match_vec;
    wire        ld_any  = |m_ld_match_vec;

    assign dtu_ifu_halt_info_vld = exe_any;
    assign dtu_ifu_halt_info = {
        {6'b0, exe_sel},                       // TRIGGER [21:12]
        4'd2,                                  // CAUSE [11:8] (donor const, :2982)
        1'b0,                                  // PENDING_HALT [7]
        1'b0,                                  // TIMING [6] (execute: forced 0)
        1'b0,                                  // ACTION01 [5] (donor :2974:
                                               // action0 && action1 = conflict
                                               // flag; 0 for a single match)
        m_exe_action01_vec[exe_sel[2:0]],      // ACTION [4] (donor :2968/:2983:
                                               // exe0_action = !action0 &&
                                               // action1 = "matched trigger's
                                               // action is exactly 1")
        1'b0,                                  // CHAIN [3]
        1'b0,                                  // LDST [2] (execute, not ldst)
        1'b1,                                  // MATCH [1]
        1'b1};                                 // CANCEL [0] (donor exe0_cancel,
                                               // :2977-2980: every non-chain
                                               // execute match cancels the
                                               // instruction's side effects)

    assign dtu_lsu_halt_info_vld = ld_any;
    assign dtu_lsu_halt_info = {
        {6'b0, ld_sel},                        // TRIGGER [21:12]
        4'd2,                                  // CAUSE [11:8] (donor const, :3395)
        1'b0,                                  // PENDING_HALT [7]
        m_ld_timing_vec[ld_sel[2:0]],           // TIMING [6]
        1'b0,                                  // ACTION01 [5] (conflict flag, 0
                                               // for a single match)
        m_ld_action01_vec[ld_sel[2:0]],         // ACTION [4] (donor ldst_action:
                                               // "matched trigger's action is
                                               // exactly 1")
        1'b0,                                  // CHAIN [3]
        1'b1,                                  // LDST [2] (ldst, donor const 1)
        1'b1,                                  // MATCH [1]
        m_ld_t0_cancel_vec[ld_sel[2:0]]};      // CANCEL [0] (donor ldst_cancel)

    // store suppression enables (a trigger-hit store must not commit)
    assign dtu_lsu_addr_trig_en = |m_ld_addr_trig_vec;
    assign dtu_lsu_data_trig_en = |m_ld_data_trig_vec;

    // ---- iie icount (type 3) comparator (donor aq_dtu_iie_trigger.v) -----
    // count is hardwired 1, so an enabled icount trigger matches on every
    // retire (donor icount_match = icount_enable && rtu_dtu_retire_vld,
    // :432-433). Structurally live; never fires in the M7 gate set (no gate
    // configures an iie trigger, and at reset type!=3 / priv-bits=0).
    wire iie0_icount_en = (iie_tdata1[0][IE_TYPE_HI:IE_TYPE_LO] == 4'd3)
                        && ((iie_tdata1[0][IE_MMODE] && m_mode)
                          || (iie_tdata1[0][IE_SMODE] && s_mode)
                          || (iie_tdata1[0][IE_UMODE] && u_mode))
                        && tcontrol_mte && !rtu_yy_xx_dbgon;
    wire iie1_icount_en = (iie_tdata1[1][IE_TYPE_HI:IE_TYPE_LO] == 4'd3)
                        && ((iie_tdata1[1][IE_MMODE] && m_mode)
                          || (iie_tdata1[1][IE_SMODE] && s_mode)
                          || (iie_tdata1[1][IE_UMODE] && u_mode))
                        && tcontrol_mte && !rtu_yy_xx_dbgon;
    wire iie_icount_match = (iie0_icount_en || iie1_icount_en)
                          && rtu_dtu_retire_vld;

    // ---- pending-halt (timing-1) record (donor :979-1015, basic) ---------
    // A timing-1 (at-retire) action-1 trigger (or an iie icount match) arms a
    // pending halt that the RTU honors at the next non-split retire boundary.
    // Not exercised by the M7 gate set (all triggers are timing-0) but
    // structurally present.
    reg pending_halt_r;
    wire rtu_retire_hinfo_match  = rtu_dtu_retire_halt_info[TDT_HINFO_MATCH]
                                 && !rtu_dtu_retire_halt_info[TDT_HINFO_CHAIN];
    wire gen_pending_halt = ((rtu_dtu_retire_vld || rtu_dtu_halt_ack)
                           && rtu_retire_hinfo_match
                           && rtu_dtu_retire_halt_info[TDT_HINFO_TIMING]
                           && rtu_dtu_retire_halt_info[TDT_HINFO_ACTION])
                          || iie_icount_match;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)         pending_halt_r <= 1'b0;
        else if (gen_pending_halt) pending_halt_r <= 1'b1;
        else if (rtu_dtu_pending_ack) pending_halt_r <= 1'b0;
    end
    assign dtu_rtu_pending_halt = pending_halt_r;

    // ---- trigger CSR read mux (donor aq_dtu_m_iie_all.v:413-611) ---------
    // Extends the Task-1 dtu_cp0_rdata_r case (0x7B0-0x7B3) with the trigger
    // CSRs 0x7A0-0x7AA. Placed here so every referenced storage signal is
    // already declared. cp0_dtu_rreg is not needed combinationally -- CSR.v
    // only samples dtu_cp0_rdata on a read cycle it already qualified.
    wire [63:0] trig_tdata1_sel = (tselect_lowbits < 4'd8) ? m_tdata1[tselect_lowbits[2:0]]
                                                           : iie_tdata1[tselect_lowbits[0]];
    wire [63:0] trig_tdata2_sel = (tselect_lowbits < 4'd8) ? m_tdata2[tselect_lowbits[2:0]]
                                                           : iie_tdata2[tselect_lowbits[0]];
    wire [63:0] trig_tdata3_sel = (tselect_lowbits < 4'd8) ? m_tdata3[tselect_lowbits[2:0]]
                                                           : iie_tdata3[tselect_lowbits[0]];
    // tinfo: mcontrol tinfo[9:4]=6'b001000 -> 0x10 (D-M7-5, reduced from the
    // donor's 0x30); iie tinfo[9:4]=6'b111000 -> 0x30 (donor :373).
    wire [63:0] trig_tinfo_sel  = (tselect_lowbits < 4'd8) ? 64'h10 : 64'h30;
    wire [63:0] tselect_read    = {60'b0, tselect_lowbits};
    wire [63:0] tcontrol_read   = {56'b0, tcontrol_mpte_r, 3'b0, tcontrol_mte_r, 3'b0};
    wire [63:0] mcontext_read   = {51'b0, mcontext_lowbits};
    wire [63:0] scontext_read   = {30'b0, scontext_lowbits};

    reg [63:0] dtu_cp0_rdata_r;
    always @* begin
        case (cp0_dtu_addr)
            CSR_DCSR:      dtu_cp0_rdata_r = {32'b0, dcsr_value};
            CSR_DPC:       dtu_cp0_rdata_r = dpc_r;
            CSR_DSCRATCH0: dtu_cp0_rdata_r = dscratch0_r;
            CSR_DSCRATCH1: dtu_cp0_rdata_r = dscratch1_r;
            CSR_TSELECT:   dtu_cp0_rdata_r = tselect_read;
            CSR_TDATA1:    dtu_cp0_rdata_r = trig_tdata1_sel;
            CSR_TDATA2:    dtu_cp0_rdata_r = trig_tdata2_sel;
            CSR_TDATA3:    dtu_cp0_rdata_r = trig_tdata3_sel;
            CSR_TINFO:     dtu_cp0_rdata_r = trig_tinfo_sel;
            CSR_TCONTROL:  dtu_cp0_rdata_r = tcontrol_read;
            CSR_MCONTEXT:  dtu_cp0_rdata_r = mcontext_read;
            CSR_SCONTEXT:  dtu_cp0_rdata_r = scontext_read;
            default:       dtu_cp0_rdata_r = 64'd0;
        endcase
    end
    assign dtu_cp0_rdata = dtu_cp0_rdata_r;
    wire _unused_rreg = cp0_dtu_rreg;

    wire _unused_ok = &{1'b0, wr_flg_r, dcsr_value[31:16]};

endmodule
