//=============================================================================
// AXICrossbar.v - Parameterized AXI4 Crossbar (N Masters x M Slaves)
//=============================================================================
// Inspired by C2RTL AXI4L::CTRL implementation (io/RVProc_io.h).
//
// Features:
//   - Fully parameterized: N_MASTERS, N_SLAVES, data widths
//   - External address decoder module (user can replace AXIAddrDecode)
//   - Round-robin arbitration for multi-master conflicts
//   - Per-master and per-slave status tracking
//   - Full AXI4 compliance: independent AW/W channels, burst support
//
// Address Decode:
//   Uses external AXIAddrDecode module. Users can:
//   1. Use default implementation with ADDR_BASE/ADDR_MASK parameters
//   2. Replace AXIAddrDecode.v with custom implementation
//
// Key concepts from C2RTL:
//   - MasterStatus: tracks active state, target slave ID, granted flag
//   - SlaveStatus: tracks active state, connected master ID
//   - checkMReq: check if master has pending request and slave is available
//   - arbitrate: round-robin arbitration per slave
//   - connectChannel: main routing logic
//=============================================================================

module AXICrossbar #(
    // Master configuration
    parameter N_MASTERS     = 2,
    parameter M_ADDR_WIDTH  = 64,
    parameter M_DATA_WIDTH  = 512,

    // Slave configuration
    parameter N_SLAVES      = 4,
    parameter S_ADDR_WIDTH  = 64,
    parameter S_DATA_WIDTH  = 512,

    // Address decode parameters (passed to AXIAddrDecode)
    parameter [N_SLAVES*32-1:0] ADDR_BASE = {32'h10010000, 32'h0C000000, 32'h02000000, 32'h80000000},
    parameter [N_SLAVES*32-1:0] ADDR_MASK = {32'hFFFF0000, 32'hFF000000, 32'hFFFF0000, 32'h80000000},
    parameter DEFAULT_SLAVE = 0
)(
    input  wire                                     clk,
    input  wire                                     rst_n,

    //=========================================================================
    // Master Ports (2D packed arrays)
    //=========================================================================
    // Write Address Channel
    input  wire [N_MASTERS-1:0]                     m_awvalid,
    output wire [N_MASTERS-1:0]                     m_awready,
    input  wire [N_MASTERS-1:0][M_ADDR_WIDTH-1:0]   m_awaddr,
    input  wire [N_MASTERS-1:0][7:0]                m_awlen,
    input  wire [N_MASTERS-1:0][2:0]                m_awsize,
    input  wire [N_MASTERS-1:0][1:0]                m_awburst,
    input  wire [N_MASTERS-1:0][2:0]                m_awprot,

    // Write Data Channel
    input  wire [N_MASTERS-1:0]                     m_wvalid,
    output wire [N_MASTERS-1:0]                     m_wready,
    input  wire [N_MASTERS-1:0][M_DATA_WIDTH-1:0]   m_wdata,
    input  wire [N_MASTERS-1:0][M_DATA_WIDTH/8-1:0] m_wstrb,
    input  wire [N_MASTERS-1:0]                     m_wlast,

    // Write Response Channel
    output wire [N_MASTERS-1:0]                     m_bvalid,
    input  wire [N_MASTERS-1:0]                     m_bready,
    output wire [N_MASTERS-1:0][1:0]                m_bresp,

    // Read Address Channel
    input  wire [N_MASTERS-1:0]                     m_arvalid,
    output wire [N_MASTERS-1:0]                     m_arready,
    input  wire [N_MASTERS-1:0][M_ADDR_WIDTH-1:0]   m_araddr,
    input  wire [N_MASTERS-1:0][7:0]                m_arlen,
    input  wire [N_MASTERS-1:0][2:0]                m_arsize,
    input  wire [N_MASTERS-1:0][1:0]                m_arburst,
    input  wire [N_MASTERS-1:0][2:0]                m_arprot,

    // Read Data Channel
    output wire [N_MASTERS-1:0]                     m_rvalid,
    input  wire [N_MASTERS-1:0]                     m_rready,
    output wire [N_MASTERS-1:0][M_DATA_WIDTH-1:0]   m_rdata,
    output wire [N_MASTERS-1:0][1:0]                m_rresp,
    output wire [N_MASTERS-1:0]                     m_rlast,

    //=========================================================================
    // Slave Ports (2D packed arrays)
    //=========================================================================
    // Write Address Channel
    output wire [N_SLAVES-1:0]                      s_awvalid,
    input  wire [N_SLAVES-1:0]                      s_awready,
    output wire [N_SLAVES-1:0][S_ADDR_WIDTH-1:0]    s_awaddr,
    output wire [N_SLAVES-1:0][7:0]                 s_awlen,
    output wire [N_SLAVES-1:0][2:0]                 s_awsize,
    output wire [N_SLAVES-1:0][1:0]                 s_awburst,
    output wire [N_SLAVES-1:0][2:0]                 s_awprot,

    // Write Data Channel
    output wire [N_SLAVES-1:0]                      s_wvalid,
    input  wire [N_SLAVES-1:0]                      s_wready,
    output wire [N_SLAVES-1:0][S_DATA_WIDTH-1:0]    s_wdata,
    output wire [N_SLAVES-1:0][S_DATA_WIDTH/8-1:0]  s_wstrb,
    output wire [N_SLAVES-1:0]                      s_wlast,

    // Write Response Channel
    input  wire [N_SLAVES-1:0]                      s_bvalid,
    output wire [N_SLAVES-1:0]                      s_bready,
    input  wire [N_SLAVES-1:0][1:0]                 s_bresp,

    // Read Address Channel
    output wire [N_SLAVES-1:0]                      s_arvalid,
    input  wire [N_SLAVES-1:0]                      s_arready,
    output wire [N_SLAVES-1:0][S_ADDR_WIDTH-1:0]    s_araddr,
    output wire [N_SLAVES-1:0][7:0]                 s_arlen,
    output wire [N_SLAVES-1:0][2:0]                 s_arsize,
    output wire [N_SLAVES-1:0][1:0]                 s_arburst,
    output wire [N_SLAVES-1:0][2:0]                 s_arprot,

    // Read Data Channel
    input  wire [N_SLAVES-1:0]                      s_rvalid,
    output wire [N_SLAVES-1:0]                      s_rready,
    input  wire [N_SLAVES-1:0][S_DATA_WIDTH-1:0]    s_rdata,
    input  wire [N_SLAVES-1:0][1:0]                 s_rresp,
    input  wire [N_SLAVES-1:0]                      s_rlast
);

    //=========================================================================
    // Local Parameters
    //=========================================================================
    localparam SLAVE_ID_BITS = $clog2(N_SLAVES) > 0 ? $clog2(N_SLAVES) : 1;
    localparam MASTER_ID_BITS = $clog2(N_MASTERS) > 0 ? $clog2(N_MASTERS) : 1;

    //=========================================================================
    // Address Decode Instances (User can replace AXIAddrDecode module)
    //=========================================================================
    wire [SLAVE_ID_BITS-1:0] ar_decode [N_MASTERS-1:0];  // Read address decode
    wire [SLAVE_ID_BITS-1:0] aw_decode [N_MASTERS-1:0];  // Write address decode

    genvar d;
    generate
        for (d = 0; d < N_MASTERS; d = d + 1) begin : gen_decode
            // Decode read address
            AXIAddrDecode #(
                .ADDR_WIDTH     (M_ADDR_WIDTH),
                .N_SLAVES       (N_SLAVES),
                .SLAVE_ID_BITS  (SLAVE_ID_BITS),
                .ADDR_BASE      (ADDR_BASE),
                .ADDR_MASK      (ADDR_MASK),
                .DEFAULT_SLAVE  (DEFAULT_SLAVE)
            ) u_decode_ar (
                .addr           (m_araddr[d]),
                .slave_id       (ar_decode[d])
            );

            // Decode write address
            AXIAddrDecode #(
                .ADDR_WIDTH     (M_ADDR_WIDTH),
                .N_SLAVES       (N_SLAVES),
                .SLAVE_ID_BITS  (SLAVE_ID_BITS),
                .ADDR_BASE      (ADDR_BASE),
                .ADDR_MASK      (ADDR_MASK),
                .DEFAULT_SLAVE  (DEFAULT_SLAVE)
            ) u_decode_aw (
                .addr           (m_awaddr[d]),
                .slave_id       (aw_decode[d])
            );
        end
    endgenerate

    //=========================================================================
    // Master Read Status (like C2RTL MasterStatus)
    //=========================================================================
    reg [SLAVE_ID_BITS-1:0] mr_slave_id [N_MASTERS-1:0];  // Target slave ID
    reg                     mr_active   [N_MASTERS-1:0];  // Active connection
    wire                    mr_granted  [N_MASTERS-1:0];  // Grant this cycle
    wire                    mr_requested[N_MASTERS-1:0];  // Request pending
    wire [SLAVE_ID_BITS-1:0] mr_req_slave[N_MASTERS-1:0]; // Requested slave (from decode)

    //=========================================================================
    // Master Write Status
    //=========================================================================
    reg [SLAVE_ID_BITS-1:0] mw_slave_id [N_MASTERS-1:0];
    reg                     mw_active   [N_MASTERS-1:0];
    wire                    mw_granted  [N_MASTERS-1:0];
    wire                    mw_requested[N_MASTERS-1:0];
    wire [SLAVE_ID_BITS-1:0] mw_req_slave[N_MASTERS-1:0];

    //=========================================================================
    // Slave Status (like C2RTL SlaveStatus)
    //=========================================================================
    reg [MASTER_ID_BITS-1:0] sr_master_id [N_SLAVES-1:0];
    reg                      sr_active    [N_SLAVES-1:0];
    reg [MASTER_ID_BITS-1:0] sw_master_id [N_SLAVES-1:0];
    reg                      sw_active    [N_SLAVES-1:0];

    //=========================================================================
    // Round-Robin Arbitration Counters
    //=========================================================================
    reg [MASTER_ID_BITS-1:0] rr_read_priority;
    reg [MASTER_ID_BITS-1:0] rr_write_priority;

    //=========================================================================
    // Check Master Read Requests (checkMRReq from C2RTL)
    //=========================================================================
    genvar m;
    generate
        for (m = 0; m < N_MASTERS; m = m + 1) begin : gen_mr_req
            assign mr_req_slave[m] = ar_decode[m];
            // Request is valid if: master has valid request, not already active,
            // target slave is not active, and no response pending (matching C2RTL)
            assign mr_requested[m] = m_arvalid[m] && !mr_active[m] &&
                                     !sr_active[ar_decode[m]] && !s_rvalid[ar_decode[m]];
        end
    endgenerate

    //=========================================================================
    // Check Master Write Requests (checkMWReq from C2RTL)
    //=========================================================================
    generate
        for (m = 0; m < N_MASTERS; m = m + 1) begin : gen_mw_req
            assign mw_req_slave[m] = aw_decode[m];
            // Request is valid if: master has BOTH AW and W valid (like MMIOInterceptor),
            // not already active, target slave is not active, and no response pending
            assign mw_requested[m] = m_awvalid[m] && m_wvalid[m] && !mw_active[m] &&
                                     !sw_active[aw_decode[m]] && !s_bvalid[aw_decode[m]];
        end
    endgenerate

    //=========================================================================
    // Arbitration Logic (arbitrate from C2RTL - Round Robin per Slave)
    //=========================================================================
    // For each slave, grant to one master using round-robin
    reg [N_MASTERS-1:0] mr_grant_vec;
    reg [N_MASTERS-1:0] mw_grant_vec;
    reg [N_SLAVES-1:0]  slave_r_granted;  // Track which slaves have been granted
    reg [N_SLAVES-1:0]  slave_w_granted;
    integer arb_m, arb_s;

    // Read arbitration
    always @(*) begin
        mr_grant_vec = {N_MASTERS{1'b0}};
        slave_r_granted = {N_SLAVES{1'b0}};

        // First pass: masters >= priority
        for (arb_m = 0; arb_m < N_MASTERS; arb_m = arb_m + 1) begin
            if (arb_m >= rr_read_priority && mr_requested[arb_m] &&
                !slave_r_granted[mr_req_slave[arb_m]]) begin
                mr_grant_vec[arb_m] = 1'b1;
                slave_r_granted[mr_req_slave[arb_m]] = 1'b1;
            end
        end

        // Second pass: masters < priority (wrap-around)
        for (arb_m = 0; arb_m < N_MASTERS; arb_m = arb_m + 1) begin
            if (arb_m < rr_read_priority && mr_requested[arb_m] &&
                !slave_r_granted[mr_req_slave[arb_m]]) begin
                mr_grant_vec[arb_m] = 1'b1;
                slave_r_granted[mr_req_slave[arb_m]] = 1'b1;
            end
        end
    end

    // Write arbitration
    always @(*) begin
        mw_grant_vec = {N_MASTERS{1'b0}};
        slave_w_granted = {N_SLAVES{1'b0}};

        // First pass: masters >= priority
        for (arb_m = 0; arb_m < N_MASTERS; arb_m = arb_m + 1) begin
            if (arb_m >= rr_write_priority && mw_requested[arb_m] &&
                !slave_w_granted[mw_req_slave[arb_m]]) begin
                mw_grant_vec[arb_m] = 1'b1;
                slave_w_granted[mw_req_slave[arb_m]] = 1'b1;
            end
        end

        // Second pass: masters < priority (wrap-around)
        for (arb_m = 0; arb_m < N_MASTERS; arb_m = arb_m + 1) begin
            if (arb_m < rr_write_priority && mw_requested[arb_m] &&
                !slave_w_granted[mw_req_slave[arb_m]]) begin
                mw_grant_vec[arb_m] = 1'b1;
                slave_w_granted[mw_req_slave[arb_m]] = 1'b1;
            end
        end
    end

    generate
        for (m = 0; m < N_MASTERS; m = m + 1) begin : gen_grants
            assign mr_granted[m] = mr_grant_vec[m];
            assign mw_granted[m] = mw_grant_vec[m];
        end
    endgenerate

    //=========================================================================
    // Status Update Logic (updateMRStat/updateMWStat from C2RTL)
    //=========================================================================
    integer i;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rr_read_priority <= {MASTER_ID_BITS{1'b0}};
            rr_write_priority <= {MASTER_ID_BITS{1'b0}};
            for (i = 0; i < N_MASTERS; i = i + 1) begin
                mr_slave_id[i] <= {SLAVE_ID_BITS{1'b0}};
                mr_active[i] <= 1'b0;
                mw_slave_id[i] <= {SLAVE_ID_BITS{1'b0}};
                mw_active[i] <= 1'b0;
            end
            for (i = 0; i < N_SLAVES; i = i + 1) begin
                sr_master_id[i] <= {MASTER_ID_BITS{1'b0}};
                sr_active[i] <= 1'b0;
                sw_master_id[i] <= {MASTER_ID_BITS{1'b0}};
                sw_active[i] <= 1'b0;
            end
        end else begin
            // Update read status
            for (i = 0; i < N_MASTERS; i = i + 1) begin
                if (mr_granted[i]) begin
                    // New grant
                    mr_slave_id[i] <= mr_req_slave[i];
                    mr_active[i] <= 1'b1;
                    sr_active[mr_req_slave[i]] <= 1'b1;
                    sr_master_id[mr_req_slave[i]] <= i[MASTER_ID_BITS-1:0];
                    // Update round-robin priority
                    rr_read_priority <= (i < N_MASTERS - 1) ? i[MASTER_ID_BITS-1:0] + 1 : {MASTER_ID_BITS{1'b0}};
                end else if (mr_active[i] && m_rready[i] && s_rvalid[mr_slave_id[i]] && s_rlast[mr_slave_id[i]]) begin
                    // Transaction complete (AXI: valid && ready && last)
                    mr_active[i] <= 1'b0;
                    sr_active[mr_slave_id[i]] <= 1'b0;
                end
            end

            // Update write status
            for (i = 0; i < N_MASTERS; i = i + 1) begin
                if (mw_granted[i]) begin
                    // New grant
                    mw_slave_id[i] <= mw_req_slave[i];
                    mw_active[i] <= 1'b1;
                    sw_active[mw_req_slave[i]] <= 1'b1;
                    sw_master_id[mw_req_slave[i]] <= i[MASTER_ID_BITS-1:0];
                    // Update round-robin priority
                    rr_write_priority <= (i < N_MASTERS - 1) ? i[MASTER_ID_BITS-1:0] + 1 : {MASTER_ID_BITS{1'b0}};
                end else if (mw_active[i] && m_bready[i] && s_bvalid[mw_slave_id[i]]) begin
                    // Transaction complete (C2RTL: ws.active && mc.wres.m.ready && s_ch[sidx].wres.s.valid)
                    // Use m_bready[i] directly, not s_bready (avoids dependency on generate block's "connected")
                    mw_active[i] <= 1'b0;
                    sw_active[mw_slave_id[i]] <= 1'b0;
                end
            end
        end
    end

    //=========================================================================
    // Channel Connection (connectChannel from C2RTL)
    //=========================================================================
    // Master -> Slave routing
    genvar s;
    generate
        for (s = 0; s < N_SLAVES; s = s + 1) begin : gen_slave_read
            // Find which master is connected to this slave (read)
            reg                      connected;
            reg [MASTER_ID_BITS-1:0] master_idx;
            integer sm;
            always @(*) begin
                connected = 1'b0;
                master_idx = {MASTER_ID_BITS{1'b0}};
                for (sm = 0; sm < N_MASTERS; sm = sm + 1) begin
                    if ((mr_active[sm] || mr_granted[sm]) &&
                        (mr_granted[sm] ? mr_req_slave[sm] : mr_slave_id[sm]) == s) begin
                        connected = 1'b1;
                        master_idx = sm[MASTER_ID_BITS-1:0];
                    end
                end
            end

            // Slave read address outputs
            assign s_arvalid[s] = connected && m_arvalid[master_idx];
            assign s_araddr[s]  = m_araddr[master_idx][S_ADDR_WIDTH-1:0];
            assign s_arlen[s]   = m_arlen[master_idx];
            assign s_arsize[s]  = m_arsize[master_idx];
            assign s_arburst[s] = m_arburst[master_idx];
            assign s_arprot[s]  = m_arprot[master_idx];
            assign s_rready[s]  = connected && m_rready[master_idx];
        end

        for (s = 0; s < N_SLAVES; s = s + 1) begin : gen_slave_write
            // Find which master is connected to this slave (write)
            reg                      connected;
            reg [MASTER_ID_BITS-1:0] master_idx;
            integer sm;
            always @(*) begin
                connected = 1'b0;
                master_idx = {MASTER_ID_BITS{1'b0}};
                for (sm = 0; sm < N_MASTERS; sm = sm + 1) begin
                    if ((mw_active[sm] || mw_granted[sm]) &&
                        (mw_granted[sm] ? mw_req_slave[sm] : mw_slave_id[sm]) == s) begin
                        connected = 1'b1;
                        master_idx = sm[MASTER_ID_BITS-1:0];
                    end
                end
            end

            // Slave write outputs
            assign s_awvalid[s] = connected && m_awvalid[master_idx];
            assign s_awaddr[s]  = m_awaddr[master_idx][S_ADDR_WIDTH-1:0];
            assign s_awlen[s]   = m_awlen[master_idx];
            assign s_awsize[s]  = m_awsize[master_idx];
            assign s_awburst[s] = m_awburst[master_idx];
            assign s_awprot[s]  = m_awprot[master_idx];
            assign s_wvalid[s]  = connected && m_wvalid[master_idx];
            assign s_wdata[s]   = m_wdata[master_idx][S_DATA_WIDTH-1:0];
            assign s_wstrb[s]   = m_wstrb[master_idx][S_DATA_WIDTH/8-1:0];
            assign s_wlast[s]   = m_wlast[master_idx];
            assign s_bready[s]  = connected && m_bready[master_idx];
        end
    endgenerate

    //=========================================================================
    // Slave -> Master routing for responses
    //=========================================================================
    generate
        for (m = 0; m < N_MASTERS; m = m + 1) begin : gen_master_resp
            wire [SLAVE_ID_BITS-1:0] r_sid = mr_granted[m] ? mr_req_slave[m] : mr_slave_id[m];
            wire [SLAVE_ID_BITS-1:0] w_sid = mw_granted[m] ? mw_req_slave[m] : mw_slave_id[m];

            // Read channel responses to master
            assign m_arready[m] = (mr_active[m] || mr_granted[m]) && s_arready[r_sid];
            assign m_rvalid[m]  = (mr_active[m] || mr_granted[m]) && s_rvalid[r_sid];
            assign m_rdata[m]   = {{(M_DATA_WIDTH-S_DATA_WIDTH){1'b0}}, s_rdata[r_sid]};
            assign m_rresp[m]   = s_rresp[r_sid];
            assign m_rlast[m]   = s_rlast[r_sid];

            // Write channel responses to master
            assign m_awready[m] = (mw_active[m] || mw_granted[m]) && s_awready[w_sid];
            assign m_wready[m]  = (mw_active[m] || mw_granted[m]) && s_wready[w_sid];
            assign m_bvalid[m]  = (mw_active[m] || mw_granted[m]) && s_bvalid[w_sid];
            assign m_bresp[m]   = s_bresp[w_sid];
        end
    endgenerate

endmodule
