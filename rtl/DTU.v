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
    output wire        dtu_hpcp_dcsr_stopcount
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
    // SECTION READ MUX (aq_dtu_ctrl.v:581-606, Task-1 subset: 0x7B0-0x7B3;
    // trigger 0x7Ax + custom 0xFEx arms arrive at Task 2 / are dropped per
    // D-M7-6). cp0_dtu_rreg is not needed combinationally here -- CSR.v only
    // samples dtu_cp0_rdata on a read cycle it already qualified.
    //=========================================================================
    reg [63:0] dtu_cp0_rdata_r;
    always @* begin
        case (cp0_dtu_addr)
            CSR_DCSR:      dtu_cp0_rdata_r = {32'b0, dcsr_value};
            CSR_DPC:       dtu_cp0_rdata_r = dpc_r;
            CSR_DSCRATCH0: dtu_cp0_rdata_r = dscratch0_r;
            CSR_DSCRATCH1: dtu_cp0_rdata_r = dscratch1_r;
            default:       dtu_cp0_rdata_r = 64'd0;
        endcase
    end
    assign dtu_cp0_rdata = dtu_cp0_rdata_r;
    wire _unused_rreg = cp0_dtu_rreg;

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
    wire _unused_ok = &{1'b0, wr_flg_r, dcsr_value[31:16]};

endmodule
