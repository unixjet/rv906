#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sstream>
#include <vector>

#include "RVProc.h"
#include "io/RVProc_io.h"
#include "io/ExtMem.h"
#include "device/uart16550.h"
#include "testbench/TestBench.h"
#include "m1_iss.h"

enum {
    DI_NODEV = 0,
    DI_EXT_MEM,
    DI_UART,
    DI_NUM,
};

#define EXT_MEM_ADDR    0x0000000080000000ULL
#define EXT_MEM_MASK    0xFFFFFFFF80000000ULL

#define UART_ADDR       0x0000000010001000ULL
#define UART_MASK       0xFFFFFFFFFFFFFFE0ULL


unsigned AXI4L::decodeAddr(AXI_AType addr) {
	if ((addr & UART_MASK) == UART_ADDR) return DI_UART;
	return DI_EXT_MEM;
}

#define NUM_MASTERS 2
#define NUM_SLAVES  DI_NUM

struct IO_PINS {
	MEMCTLPin mpin;
	AXI4L::TCH<UINT8> uart_ch;
};


IO_PINS io_pins;
AXI4L::BUS<NUM_MASTERS, NUM_SLAVES> axi_bus;
ExtMem xmem;
AXI4L::Converter<UINT8> uart_cvt;
UART16550_AXI4L axi_uart;

#include "dut.h"
DUT dut;
CoreState cpu;
constexpr int DUT_DI_UART = DI_UART;
int RVProcAXI_Verilator(AXI4L::BUS<NUM_MASTERS, NUM_SLAVES> *axi_bus, IO_PINS *io_pins)
{
    dut.write<DUT_DI_UART>(&axi_bus->s_ch[DI_UART]);
    // M6 Task 6: sample the UART model's interrupt level pre-clock so the
    // DUT sees it this step (device model state from the prior post-clock
    // update -- level signal, 1-step skew is intentional).
    UINT32 uart_irq = axi_uart.irq();
    bool quitted = dut.step(&io_pins->mpin, uart_irq);
    dut.read<DUT_DI_UART>(&axi_bus->s_ch[DI_UART]);
    dut.sync(cpu);
    return quitted;
}
#define INIT_REG(cpu, initial_pc, initial_sp, dtb_addr) \
    do{dut.init(initial_pc, initial_sp, dtb_addr); dut.sync(cpu);}while(0)

//=============================================================================
// M7 Task 6: JTAG DMI driver + --m7-debug smoke
//=============================================================================
// C++ port of the donor's ext_debug class (refs/openc906/smart_run/tests/
// cases/debug/JTAG_DRV.vh:936-1143): jtag_tlr, write_ir, shift_dr and
// dmi_rw with busy-poll + timeout. The donor's TAP sequences, IR/DR scan
// formats, poll/timeout constants and bit positions are the de-facto
// contract (design doc M7 "SoC layer", risk #1) and are ported verbatim;
// the low-level clocking (TCK/clk interleave, TDO sampling) lives in the
// JTAG driver in dut.cpp.
//
// IR codes (JTAG_DRV.vh:458-465): IDCODE=5'h01, DMI_ACC=5'h02,
// DTMCS=5'h10, DMI=5'h11; abits=10, dtmcs.version=1 (TDT_DTM.v).
// DMI ops (JTAG_DRV.vh:467-469): NOP=00, READ=01, WRITE=10 (the TDT_DTM
// request engine fires on ^op, i.e. 01 or 10 -- TDT_DTM.v wr_vld).
// DMI DR = {addr[9:0], data[31:0], op[1:0]}, 44 bits, LSB-first scan
// (donor :1103, TDT_DTM.v address/data/op capture). The DR readout is
// {addr[43:34], data[33:2], res_op[1:0]} (TDT_DTM.v dmi_total):
// res_op 00=done, 11=busy (request in flight), 10=failed -> error
// (donor rwMemorybyDMI error check, :1109-1112).
// Polling (rwMemorybyDMIwithCheck, :1121-1143): after an op, re-scan the
// DMI DR with op=NOP until res_op==00; 30-round timeout (max_poll_round_num,
// :1047). The 7-idle-TCK spacing after every DR update (dtmcs.idle, learned
// in initDTM, :1061-1079) is what guarantees the request has completed
// before the next scan (TDT_DTM.v TIMING CONTRACT).
//
// DM word offsets used by the smoke (JTAG_DRV.vh:502-503): dmcontrol=0x10,
// dmstatus=0x11.
struct M7Opts {
    bool debug = false;   // --m7-debug
};
static M7Opts m7_opts;

#define M7_IR_IDCODE   0x01
#define M7_IR_DMI      0x11
#define M7_IR_DTMCS    0x10
#define M7_DMI_NOP     0x0
#define M7_DMI_READ    0x1
#define M7_DMI_WRITE   0x2
#define M7_DMI_DR_LEN  44          // abits(10) + data(32) + op(2)
#define M7_MAX_POLL    30          // donor max_poll_round_num (:1047)
#define M7_IDCODE      0x10000B6FU // TDT_DTM.v IDCODE_REG_DEFINE
#define M7_DM_DMCONTROL 0x10
#define M7_DM_DMSTATUS  0x11
// M7 Task 7: remaining DM word offsets (rtl/TDT_DM.v:116-138; donor
// tdt_dm.v:171-227 -- the DMI addr field IS the word offset, TDT_DTM
// shifts <<2 to paddr).
#define M7_DM_DATA0     0x04
#define M7_DM_DATA1     0x05
#define M7_DM_ABSTRACTCS 0x16
#define M7_DM_COMMAND   0x17
#define M7_DM_ITR       0x1F   // custom instruction channel (TDT_DM.v:139)
#define M7_DM_PB0       0x20   // progbuf0-3 = 0x20-0x23 (TDT_DM.v:126-129)
#define M7_DM_HALTSUM0  0x40   // = {31'b0, core_dm_halted_i} (TDT_DM.v:893)

struct M7JTAG {
    int idle_cycle_num = 7;        // from dtmcs.idle (initDTM :1077)
    // The harness's "one core clock cycle" service callback (TB::
    // core_tick) -- passed into every JTAG::cycle so the core KEEPS
    // RUNNING (its fetches/AXI/UART serviced) while the scan runs.
    std::function<void()> tick;

    // jtag_tlr: the standard 5x TMS=1 (D-M7-2; the clone has no trst_n --
    // the donor's jtag_rst used trst_b, JTAG_DRV.vh:559-588), then run to
    // Idle. Returns the last TDO (1 while the TAP is not shifting).
    uint32_t tlr() {
        uint32_t tdo = 0;
        for (int i = 0; i < 5; i++) tdo = dut.jtag.cycle(1, 0, tick);
        tdo = dut.jtag.cycle(0, 0, tick);   // TLR -> Run-Test/Idle
        return tdo;
    }

    // write_ir (JTAG_DRV.vh:591-654, JTAG_5 path; rwDTMReg :1086 calls it
    // with idle_cycles=0): IDLE -> SELECT-DR -> SELECT-IR -> CAPTURE-IR ->
    // SHIFT-IR (5 bits LSB-first) -> EXIT1-IR, leaving the TAP in
    // EXIT1-IR; shift_dr() completes the IR update and the DR scan.
    void write_ir(uint32_t ir) {
        dut.jtag.cycle(1, 0, tick);   // IDLE -> SELECT_DR_SCAN
        dut.jtag.cycle(1, 0, tick);   // -> SELECT_IR_SCAN
        dut.jtag.cycle(0, 0, tick);   // -> CAPTURE_IR
        dut.jtag.cycle(0, 0, tick);   // -> SHIFT_IR (IR captured)
        dut.jtag.cycle(0, 0, tick);   // donor's wash cycle (:636; TDI don't-care)
        for (int i = 0; i < 4; i++) { dut.jtag.cycle(0, ir & 1, tick); ir >>= 1; }
        dut.jtag.cycle(1, ir & 1, tick);   // 5th bit (LSB-first) -> EXIT1_IR
    }

    // shift_dr (JTAG_DRV.vh:658-729, JTAG_5 path): completes the IR update
    // (TMS=1, TMS=1), captures the DR, shifts `len` bits LSB-first while
    // shifting `din` in LSB-first, exits to IDLE and idles idle_cycle_num
    // TCK cycles (the DMI spacing contract). Returns the shifted-out
    // value (bit i of the DUT's DR at bit i of the return).
    uint64_t shift_dr(int len, uint64_t din) {
        uint64_t dout = 0;
        dut.jtag.cycle(1, 0, tick);   // EXIT1_IR -> UPDATE_IR
        dut.jtag.cycle(1, 0, tick);   // -> SELECT_DR_SCAN (IR updated on this edge)
        dut.jtag.cycle(0, 0, tick);   // -> CAPTURE_DR
        dout |= dut.jtag.cycle(0, 0, tick);   // -> SHIFT_DR; TDO = DR bit 0
        for (int i = 1; i < len; i++) {
            uint32_t tdi = (din >> (i - 1)) & 1;
            dout |= ((uint64_t)dut.jtag.cycle(0, tdi, tick) & 1) << i;
        }
        dut.jtag.cycle(1, 0, tick);   // -> EXIT1_DR
        dut.jtag.cycle(1, 0, tick);   // -> UPDATE_DR
        dut.jtag.cycle(0, 0, tick);   // -> IDLE (DR updated; DMI request fires)
        for (int i = 0; i < idle_cycle_num; i++) dut.jtag.cycle(0, 0, tick);
        return dout;
    }

    // rwDTMReg (:1084-1091): write_ir + shift_dr for the DMI register.
    uint64_t dmi_scan(uint32_t op, uint32_t addr, uint32_t data) {
        uint64_t wr = ((uint64_t)addr << 34) | ((uint64_t)data << 2) | op;
        write_ir(M7_IR_DMI);
        return shift_dr(M7_DMI_DR_LEN, wr);
    }

    // rwMemorybyDMIwithCheck (:1121-1143): issue one DMI op, then poll the
    // DMI DR with op=NOP until res_op==00 (or the 30-round timeout /
    // res_op==10 failed). `data` is updated with the readout data field
    // ([33:2] -- the read value once the request completes).
    // Returns true when res_op==00 with no failed op.
    bool dmi_rw_check(uint32_t op, uint32_t addr, uint32_t &data,
                      const char *what) {
        uint64_t rd = dmi_scan(op, addr, data);
        uint32_t res_op = rd & 3;
        data = (uint32_t)((rd >> 2) & 0xFFFFFFFFULL);
        bool error = (res_op == 2);
        for (int i = 0; i < M7_MAX_POLL; i++) {
            rd = dmi_scan(M7_DMI_NOP, addr, 0);
            res_op = rd & 3;
            data = (uint32_t)((rd >> 2) & 0xFFFFFFFFULL);
            if (res_op == 2) error = true;
            if (res_op == 0 || res_op == 2) break;
        }
        if (res_op == 3) {
            printf("M7-DEBUG-FAIL: %s: DMI busy poll timeout "
                   "(%d rounds, last res_op=busy)\n", what, M7_MAX_POLL);
            return false;
        }
        if (error) {
            printf("M7-DEBUG-FAIL: %s: DMI res_op=10 (op failed)\n", what);
            return false;
        }
        return true;
    }

    //=============================================================
    // M7 Task 7: DM-level method ports (donor ext_debug, JTAG_DRV.vh)
    //=============================================================
    // Thin DMI wrappers over dmi_rw_check (which already busy-polls the
    // individual scan to completion).
    bool dmi_read(uint32_t addr, uint32_t &data, const char *what)
    { return dmi_rw_check(M7_DMI_READ, addr, data, what); }
    bool dmi_write(uint32_t addr, uint32_t data, const char *what)
    { return dmi_rw_check(M7_DMI_WRITE, addr, data, what); }

    // abstract command word, donor access_register_by_abscmd
    // (JTAG_DRV.vh:1471-1481): {cmdtype[31:24], aarsize[22:20],
    // aarpostincrement[19], postexec[18], transfer[17], write[16],
    // regno[15:0]}. aarsize=3 (XLEN=64) for the GPR/CSR access.
    static uint32_t abs_cmd(uint32_t cmdtype, uint32_t aarsize,
                            bool aarpi, bool postexec, bool transfer,
                            bool wr, uint32_t regno)
    {
        return (cmdtype << 24) | (aarsize << 20) | (aarpi ? (1u << 19) : 0) |
               (postexec ? (1u << 18) : 0) | (transfer ? (1u << 17) : 0) |
               (wr ? (1u << 16) : 0) | regno;
    }

    // Issue an abstract command (write COMMAND 0x17, TDT_DM.v:134) and
    // poll ABSTRACTCS 0x16 (busy bit 12) like the donor
    // (JTAG_DRV.vh:1484-1494: a real DMI_READ of DM_ABSTRACTCS per round;
    // the clone latches abstractcs.busy in TDT_DM.v:665-667).
    // Returns the final ABSTRACTCS word; false on DMI error or busy
    // timeout (the donor's "abstractcmd timeout" check).
    bool abstract_cmd(uint32_t cmd, uint32_t &abstractcs, const char *what)
    {
        if (!dmi_write(M7_DM_COMMAND, cmd, what)) return false;
        for (int i = 0; i < M7_MAX_POLL; i++) {
            if (!dmi_read(M7_DM_ABSTRACTCS, abstractcs, what)) return false;
            if ((abstractcs >> 12) & 1u) continue;   // busy
            return true;                             // busy deasserted
        }
        printf("M7-DEBUG-FAIL: %s: abstractcmd busy timeout\n", what);
        return false;
    }

    // DATA0/DATA1 (0x04/0x05): the abstract command data registers
    // (TDT_DM.v:119-120; donor DM_DATA0/1). 64-bit value = {data1,data0}.
    bool abscmd_data_wr(uint32_t lo, uint32_t hi, const char *what)
    {
        if (!dmi_write(M7_DM_DATA0, lo, what)) return false;
        return dmi_write(M7_DM_DATA1, hi, what);
    }
    bool abscmd_data_rd(uint32_t &lo, uint32_t &hi, const char *what)
    {
        if (!dmi_read(M7_DM_DATA0, lo, what)) return false;
        return dmi_read(M7_DM_DATA1, hi, what);
    }

    // Register read: command {aarsize=3, transfer=1, write=0, regno};
    // result via DATA0/DATA1 (donor JTAG_DRV.vh:1496-1515: data_id==1 ->
    // DATA0, ==2 -> DATA1). Fails on any nonzero cmderr.
    bool abstract_reg_read(uint32_t regno, uint64_t &val, const char *what)
    {
        uint32_t acs = 0;
        if (!abstract_cmd(abs_cmd(0, 3, false, false, true, false, regno),
                          acs, what)) return false;
        if (((acs >> 8) & 7) != 0) {
            printf("M7-DEBUG-FAIL: %s: cmderr=%u after regno=0x%04x read\n",
                   what, (acs >> 8) & 7, regno);
            return false;
        }
        uint32_t lo, hi;
        if (!abscmd_data_rd(lo, hi, what)) return false;
        val = ((uint64_t)hi << 32) | lo;
        return true;
    }

    // Register write: DATA0/DATA1 first, then command {aarsize=3,
    // transfer=1, write=1, regno} (donor JTAG_DRV.vh:1516-1530). Fails on
    // any nonzero cmderr.
    bool abstract_reg_write(uint32_t regno, uint64_t val, const char *what)
    {
        if (!abscmd_data_wr((uint32_t)(val & 0xFFFFFFFFu),
                            (uint32_t)(val >> 32), what)) return false;
        uint32_t acs = 0;
        if (!abstract_cmd(abs_cmd(0, 3, false, false, true, true, regno),
                          acs, what)) return false;
        if (((acs >> 8) & 7) != 0) {
            printf("M7-DEBUG-FAIL: %s: cmderr=%u after regno=0x%04x write\n",
                   what, (acs >> 8) & 7, regno);
            return false;
        }
        return true;
    }

    // Same as abstract_reg_write but with an explicit postexec bit. The
    // donor launches the program buffer with postexec=1 on the GPR write
    // that sets the pb's last scratch register (JTAG_DRV.vh:2108-2120
    // write_word_by_pb / :1996-2000 read_word_by_pb); pb_work_start then
    // fires on (transfer && cmd_done && aarpostexec, TDT_DM.v:624-627).
    bool abstract_reg_write_pe(uint32_t regno, uint64_t val, bool postexec,
                               const char *what)
    {
        if (!abscmd_data_wr((uint32_t)(val & 0xFFFFFFFFu),
                            (uint32_t)(val >> 32), what)) return false;
        uint32_t acs = 0;
        if (!abstract_cmd(abs_cmd(0, 3, false, postexec, true, true, regno),
                          acs, what)) return false;
        if (((acs >> 8) & 7) != 0) {
            printf("M7-DEBUG-FAIL: %s: cmderr=%u after regno=0x%04x "
                   "pe-write\n", what, (acs >> 8) & 7, regno);
            return false;
        }
        return true;
    }

    // Custom instruction channel: write ITR (0x1F, TDT_DM.v:139), poll
    // ABSTRACTCS busy (donor execute_itr, JTAG_DRV.vh:1404-1426: same
    // write-then-poll shape; the clone clears busy on core_dm_itr_done_i,
    // TDT_DM.v:302/789-813).
    bool execute_itr(uint32_t inst, const char *what)
    {
        if (!dmi_write(M7_DM_ITR, inst, what)) return false;
        for (int i = 0; i < M7_MAX_POLL; i++) {
            uint32_t acs;
            if (!dmi_read(M7_DM_ABSTRACTCS, acs, what)) return false;
            if ((acs >> 12) & 1u) continue;   // busy
            return true;
        }
        printf("M7-DEBUG-FAIL: %s: ITR busy timeout\n", what);
        return false;
    }

    // Program buffer DMI access: word id 0-3 -> DM offset 0x20+id
    // (TDT_DM.v:126-129; donor access_progbuf, JTAG_DRV.vh:1341-1354).
    bool progbuf_wr(uint32_t id, uint32_t val, const char *what)
    { return dmi_write(M7_DM_PB0 + id, val, what); }
    bool progbuf_rd(uint32_t id, uint32_t &val, const char *what)
    { return dmi_read(M7_DM_PB0 + id, val, what); }

    // Donor write_word_by_pb (JTAG_DRV.vh:2071-2130): progbuf =
    // {sw x7,0(x6); ebreak}. The address is written to x6 (0x1006) by a
    // plain GPR write; the value is written to x7 (0x1007) by a GPR write
    // with postexec=1, which launches the pb on (transfer && cmd_done &&
    // aarpostexec) (TDT_DM.v:624-627). The core runs sw, then ebreak
    // re-halts it (cause=1). SW_X7_0_X6=0x00732023, EBREAK=0x00100073.
    bool write_word_by_pb(uint64_t addr, uint32_t val, const char *what)
    {
        if (!progbuf_wr(0, 0x00732023, what)) return false;  // sw x7,0(x6)
        if (!progbuf_wr(1, 0x00100073, what)) return false;  // ebreak
        if (!abstract_reg_write_pe(0x1006, addr, false, what))
            return false;                              // x6 = addr
        if (!abstract_reg_write_pe(0x1007, val, true, what))
            return false;                              // x7 = val; launch pb
        uint32_t hs = 0;
        if (!dmi_read(M7_DM_HALTSUM0, hs, what)) return false;
        if (hs != 1) {
            printf("M7-DEBUG-FAIL: %s: pb ebreak did not re-halt\n", what);
            return false;
        }
        return true;
    }
    // Donor read_word_by_pb (JTAG_DRV.vh:1959-2013): progbuf =
    // {lw x7,0(x6); ebreak}; launched by the GPR write x6=addr with
    // postexec=1; the loaded x7 is read back with a plain GPR read.
    // LW_X7_0_X6=0x00032383, EBREAK=0x00100073.
    bool read_word_by_pb(uint64_t addr, uint32_t &val, const char *what)
    {
        if (!progbuf_wr(0, 0x00032383, what)) return false;  // lw x7,0(x6)
        if (!progbuf_wr(1, 0x00100073, what)) return false;  // ebreak
        if (!abstract_reg_write_pe(0x1006, addr, true, what))
            return false;                              // x6 = addr; launch pb
        uint64_t v = 0;
        if (!abstract_reg_read(0x1007, v, what)) return false;  // x7 = loaded
        val = (uint32_t)(v & 0xFFFFFFFFu);
        return true;
    }

    // halt_req: dmcontrol = haltreq(31)|dmactive(0) = 0x80000001, then
    // poll HALTSUM0 (0x40) == 1 and check dmstatus.anyhalted (bit 8) --
    // donor hart0_sync_halt_req (JTAG_DRV.vh:1281-1309).
    bool halt_req(uint32_t &dmstatus, const char *what)
    {
        if (!dmi_write(M7_DM_DMCONTROL, 0x80000001u, what)) return false;
        uint32_t hs = 0;
        for (int i = 0; i < M7_MAX_POLL; i++) {
            if (!dmi_read(M7_DM_HALTSUM0, hs, what)) return false;
            if (hs == 1) break;
        }
        if (hs != 1) {
            printf("M7-DEBUG-FAIL: %s: haltsum0 timeout\n", what);
            return false;
        }
        if (!dmi_read(M7_DM_DMSTATUS, dmstatus, what)) return false;
        return ((dmstatus >> 8) & 1u) == 1;   // anyhalted
    }

    // resume: dmcontrol = resumereq(30)|dmactive(0) = 0x40000001, poll
    // dmstatus anyresumeack(16)/allresumeack(17) -- donor hart0_resume
    // (JTAG_DRV.vh:1311-1338).
    bool resume_req(uint32_t &dmstatus, const char *what)
    {
        if (!dmi_write(M7_DM_DMCONTROL, 0x40000001u, what)) return false;
        for (int i = 0; i < M7_MAX_POLL; i++) {
            if (!dmi_read(M7_DM_DMSTATUS, dmstatus, what)) return false;
            if (((dmstatus >> 16) & 3u) != 0) break;
        }
        return ((dmstatus >> 16) & 3u) != 0;
    }

    // wait_halted: poll dmstatus until anyhalted(8)==1 (no dmcontrol
    // write). Used to catch the step-halt / pb-ebreak re-halt that happens
    // AFTER a resume was already issued.
    bool wait_halted(uint32_t &dmstatus, const char *what)
    {
        for (int i = 0; i < M7_MAX_POLL; i++) {
            if (!dmi_read(M7_DM_DMSTATUS, dmstatus, what)) return false;
            if ((dmstatus >> 8) & 1u) return true;
        }
        printf("M7-DEBUG-FAIL: %s: wait_halted timeout (anyhalted stuck 0)\n",
               what);
        return false;
    }

    // wait_running: poll dmstatus until anyrunning(10)==1 && anyhalted(8)==0
    // (no dmcontrol write). Used after a resume/dret to confirm the core is
    // genuinely running.
    bool wait_running(uint32_t &dmstatus, const char *what)
    {
        for (int i = 0; i < M7_MAX_POLL; i++) {
            if (!dmi_read(M7_DM_DMSTATUS, dmstatus, what)) return false;
            if (((dmstatus >> 10) & 1u) && !((dmstatus >> 8) & 1u))
                return true;
        }
        printf("M7-DEBUG-FAIL: %s: wait_running timeout (anyrunning stuck 0)\n",
               what);
        return false;
    }
};

//=============================================================================
// M7 Task 7: the extended debug e2e (deliverable 3, steps a-h) against the
// spin ELF. Runs against a RUNNING core (called from m7_debug_smoke after
// the base 6 steps re-enabled the DM). The C++ testbench IS the debug host
// (no OpenOCD): every operation is a DMI scan through the JTAG driver.
//
// Donor semantics are ported verbatim from JTAG_DRV.vh (ext_debug):
//   halt        <- hart0_sync_halt_req (:1281-1309)  dmcontrol=0x80000001,
//                  poll HALTSUM0==1 then dmstatus.anyhalted
//   resume      <- hart0_resume        (:1311-1338)  dmcontrol=0x40000001,
//                  poll dmstatus anyresumeack/allresumeack
//   abstract    <- access_register_by_abscmd (:1455-1535)  COMMAND 0x17,
//                  poll ABSTRACTCS 0x16 busy(12), cmderr=[10:8], data via
//                  DATA0/DATA1 0x04/0x05
//   ITR         <- execute_itr         (:1404-1426)  ITR 0x1F, poll ABSTRACTCS
//   progbuf     <- access_progbuf      (:1341-1354)  + write/read_word_by_pb
//                  (:2245-2300)
//
// Register map (rtl/TDT_DM.v): DMCONTROL 0x10 (haltreq=31, resumereq=30,
// dmactive=0; :321-391), DMSTATUS 0x11 (version[3:0]=2, anyhalted=8,
// anyrunning=10, anyresumeack=16, allresumeack=17; :427-431), ABSTRACTCS
// 0x16 (busy=12, cmderr=[10:8]; :665-666), COMMAND 0x17 (:589), ITR 0x1F
// (:789-844), PB0-3 0x20-0x23 (:126-129), HALTSUM0 0x40 (=halted; :893),
// DATA0/1 0x04/0x05 (:116-117, :875-891).
//
// dcsr/dpc live in the DTU (rtl/DTU.v): dcsr 0x7B0 (xdebugver[31:28]=4,
// cause[8:6] latch-only at halt_ack, step=2, prv[1:0]; :175-256), dpc 0x7B1
// (cp0_write_dpc + halt_ack latch of rtu_dtu_dpc; :264-271). They are
// reached through the abstract CSR path (aarsize=3, regno<0x1000), which
// the DM's REGACC FSM implements by saving x6 through dscratch1 and
// routing the access through dscratch0 (TDT_DM.v:682-756, :796-818).
//
// Halt causes (rtl/RTU.v:1126-1137): trigger=2, ebreak=1, reset=5,
// dm_sync=3 (a dmcontrol.haltreq halt), step=4. dpc at a halt = the
// retiring instruction's PC (rtu_dtu_dpc = ex2_cur_pc, :1409).
//
// NOTE vs the brief: the brief's step (a) says "dcsr.cause==1 haltreq" but
// the RTL (donor aq_rtu_retire.v:768-795, cloned at RTU.v:1126-1137)
// encodes a dmcontrol.haltreq halt as cause=3 (dm_sync), NOT 1 (ebreak).
// We assert cause==3 here, per clone discipline (donor is the spec).
//=============================================================================
static bool m7_debug_extended(M7JTAG &j)
{
    // Spin-ELF layout (test/m7/directed/debug_spin.S, objdump-verified):
    //   x8  = 0x5A5A0000  (loop never touches x8 -> abstract read oracle)
    //   x9  = 0xDEAD0000  (abstract write target)
    //   x10 = loop counter, +1 per iteration (step oracle)
    //   x13 = 0           (ITR target: addi x13,x13,1)
    //   x6/x7 (0x1006/0x1007) are the pb scratch registers (donor pb path);
    //   the spin loop never touches them.
    //   LOOP = 0x80000020: addi x10,x10,1 ; j LOOP  (RVC-free, +4 stride).
    // A dpc-range / step check therefore assumes every advance is +4.
    const uint64_t LOOP   = 0x80000020ULL;
    const uint64_t LOOPEN = 0x80000028ULL;
    const uint32_t X8_VAL = 0x5A5A0000U;
    const uint32_t R_X8   = 0x1008, R_X9   = 0x1009;
    const uint32_t R_X10  = 0x100A, R_X13  = 0x100D;
    const uint32_t CSR_DCSR = 0x7B0, CSR_DPC = 0x7B1;
    const uint32_t DCSR_STEP = 1u << 2;
    const uint32_t CAUSE_HALTEQ = 3, CAUSE_STEP = 4, CAUSE_EBREAK = 1;
    const uint32_t DRET = 0x7B200073;
    const uint32_t ADDI_X13 = 0x00130693;   // addi x13, x13, 1  (rd=x13, rs1=x13, imm=1)

    bool ok = true;
    uint32_t dmstatus = 0, dcsr = 0, acs = 0;

    printf("[m7] === M7 Task 7 extended e2e (spin ELF; "
           "halt/abstract/ITR/progbuf/step/resume/dret) ===\n");

    // (0) Re-enable the DM: the base smoke's step (6) cleared dmactive, and
    // while dmactive=0 the DM sits in sync_rst with the command engine
    // held at reset (TDT_DM.v). Re-assert dmactive=1 before any command.
    {
        bool o = j.dmi_write(M7_DM_DMCONTROL, 1, "dmactive re-enable");
        printf("[m7] (0) dmactive=1 (re-enable after base-smoke clear): %s\n",
               o ? "ok" : "FAIL");
        ok = ok && o;
    }

    // (c1) cmderr=4: issue an abstract GPR READ while the core is RUNNING.
    // TDT_DM.v:658-660: (cmd_start && !hartsum0[0] && ~busy) -> cmderr=4
    // ("not halted"). The command is refused; the engine stays idle.
    {
        uint32_t cmd = M7JTAG::abs_cmd(0, 3, false, false, true, false, R_X8);
        bool o = j.abstract_cmd(cmd, acs, "running-abstract probe");
        uint32_t ce = (acs >> 8) & 7, busy = (acs >> 12) & 1;
        printf("[m7] (c1) abstract GPR read x8 while RUNNING: cmd=0x%08x "
               "abstractcs=0x%08x (busy=%u cmderr=%u; expect cmderr=4): %s\n",
               cmd, acs, busy, ce, (o && ce == 4) ? "ok" : "FAIL");
        ok = ok && o && (ce == 4) && (busy == 0);
        // Clear cmderr (TDT_DM.v:649-651 / donor tdt_dm.v:2030):
        // abstractcs.cmderr[10:8] is write-1-to-clear, so all three bits
        // must be set = 0x700 (bits 8,9,10). The passing dm_tb uses the
        // same value (test/m7/unit/dm_tb.cpp:405 clear_cmderr ->
        // apb_write(0x16, 0x700), verified cmderr==0 at :640).
        bool oc = j.dmi_write(M7_DM_ABSTRACTCS, 0x700, "cmderr clear");
        uint32_t acs_after = 0;
        j.dmi_read(M7_DM_ABSTRACTCS, acs_after, "cmderr clear readback");
        printf("[m7] (c1) clear cmderr (abstractcs=0x700): %s; readback "
               "abstractcs=0x%08x cmderr=%u (expect 0)\n",
               oc ? "ok" : "FAIL", acs_after, (acs_after >> 8) & 7);
        ok = ok && oc && ((acs_after >> 8) & 7) == 0;
    }

    // (a) halt: dmcontrol.haltreq -> poll anyhalted; dcsr.cause==3 (dm_sync),
    // dpc in the loop region, x8 still intact (the loop never writes x8).
    {
        bool o = j.halt_req(dmstatus, "halt_req");
        uint64_t dpc = 0, x8 = 0, dcsr64 = 0;
        bool od  = j.abstract_reg_read(CSR_DPC,  dpc,    "dpc read");
        bool ox8 = j.abstract_reg_read(R_X8,     x8,     "x8 read");
        bool ods = j.abstract_reg_read(CSR_DCSR, dcsr64, "dcsr read");
        dcsr = (uint32_t)dcsr64;
        uint32_t cause = (dcsr >> 6) & 7;
        bool inloop = (dpc >= LOOP && dpc < LOOPEN);
        printf("[m7] (a) halted: dmstatus=0x%08x dpc=0x%llx "
               "dcsr=0x%08x (cause=%u; expect 3=dm_sync) x8=0x%llx\n",
               dmstatus, (unsigned long long)dpc, dcsr, cause,
               (unsigned long long)x8);
        bool oka = o && od && ox8 && ods && inloop &&
                   (cause == CAUSE_HALTEQ) && (x8 == X8_VAL);
        printf("[m7] (a) check: dpc in [0x%llx,0x%llx)=%s cause=3:%s "
               "x8 intact:%s\n",
               (unsigned long long)LOOP, (unsigned long long)LOOPEN,
               inloop ? "ok" : "FAIL",
               (cause == CAUSE_HALTEQ) ? "ok" : "FAIL",
               (x8 == X8_VAL) ? "ok" : "FAIL");
        ok = ok && oka;
    }

    // (b) abstract GPR read (x8, matches the loop invariant) + write (x9)
    // + readback. The x9 write uses a full 64-bit value to exercise the
    // DATA0+DATA1 path (donor reads data0 and data1 separately,
    // JTAG_DRV.vh:402-411). Donor access_register_by_abscmd r/w path.
    {
        const uint64_t X9_WR = 0xDEADBEEF11223344ULL;
        uint64_t x8r = 0, x9r = 0;
        bool or1 = j.abstract_reg_read(R_X8, x8r, "abs read x8");
        bool ow1 = j.abstract_reg_write(R_X9, X9_WR, "abs write x9");
        bool or2 = j.abstract_reg_read(R_X9, x9r, "abs read x9 readback");
        printf("[m7] (b) abstract GPR: x8 read=0x%016llx (expect "
               "0x%016llx); x9 write=0x%016llx readback=0x%016llx\n",
               (unsigned long long)x8r, (unsigned long long)X8_VAL,
               (unsigned long long)X9_WR, (unsigned long long)x9r);
        bool okb = or1 && ow1 && or2 && (x8r == X8_VAL) && (x9r == X9_WR);
        printf("[m7] (b) check: %s\n", okb ? "ok" : "FAIL");
        ok = ok && okb;
    }

    // (c2) unsupported abstract command while HALTED: cmdtype=1 (illegal)
    // -> cmderr=2 (TDT_DM.v:652-655: apbw_abscmd && cmdtype!=0 && ~busy).
    {
        uint32_t cmd = M7JTAG::abs_cmd(1, 3, false, false, false, false, 0);
        bool o = j.abstract_cmd(cmd, acs, "unsupported-cmd probe");
        uint32_t ce = (acs >> 8) & 7;
        printf("[m7] (c2) unsupported abstract cmd (cmdtype=1) while halted: "
               "cmd=0x%08x abstractcs=0x%08x (cmderr=%u; expect 2): %s\n",
               cmd, acs, ce, (o && ce == 2) ? "ok" : "FAIL");
        ok = ok && o && (ce == 2);
        // Clear cmderr (write-1-to-clear on [10:8] = 0x700; TDT_DM.v:649-651,
        // dm_tb.cpp:405). Verify the clear took before the ITR step, which
        // needs cmderr==0 for the pre/post register reads.
        bool oc = j.dmi_write(M7_DM_ABSTRACTCS, 0x700, "cmderr clear (c2)");
        uint32_t acs2 = 0;
        j.dmi_read(M7_DM_ABSTRACTCS, acs2, "cmderr clear (c2) readback");
        printf("[m7] (c2) clear cmderr (abstractcs=0x700): %s; readback cmderr=%u (expect 0)\n",
               oc ? "ok" : "FAIL", (acs2 >> 8) & 7);
        ok = ok && oc && ((acs2 >> 8) & 7) == 0;
    }

    // (d) ITR: inject `addi x13,x13,1` via the ITR register. x13 0 -> 1.
    // dpc is NOT auto-advanced by an ITR retire (DTU.dpc has no +4 arm;
    // it latches only on halt_ack or cp0_write_dpc) -- the observable
    // effect is the register increment, which is what we assert.
    {
        uint64_t x13a = 0, x13b = 0, dpca = 0, dpcl = 0;
        bool o1  = j.abstract_reg_read(R_X13, x13a, "x13 pre-ITR");
        bool o2  = j.abstract_reg_read(CSR_DPC, dpca, "dpc pre-ITR");
        bool oi  = j.execute_itr(ADDI_X13, "ITR addi x13");
        bool o3  = j.abstract_reg_read(R_X13, x13b, "x13 post-ITR");
        bool o4  = j.abstract_reg_read(CSR_DPC, dpcl, "dpc post-ITR");
        printf("[m7] (d) ITR addi x13,x13,1: x13 0x%llx -> 0x%llx "
               "(expect +1); dpc 0x%llx -> 0x%llx (expect unchanged)\n",
               (unsigned long long)x13a, (unsigned long long)x13b,
               (unsigned long long)dpca, (unsigned long long)dpcl);
        bool okd = o1 && o2 && oi && o3 && o4 &&
                   (x13b == x13a + 1) && (dpcl == dpca);
        printf("[m7] (d) check: %s\n", okd ? "ok" : "FAIL");
        ok = ok && okd;
    }

    // (e1) progbuf raw DMI r/w: write a 4-word pattern to progbuf[0..3]
    // (DM offset 0x20-0x23, TDT_DM.v:126-129), read back byte-exact.
    {
        uint32_t pat[4] = {0xDEADBEEF, 0xCAFEBABE, 0x0BADF00D, 0x12345678};
        uint32_t rb[4]  = {0, 0, 0, 0};
        bool okw = true, okr = true;
        for (int i = 0; i < 4; i++)
            okw = okw && j.progbuf_wr(i, pat[i], "pb wr");
        for (int i = 0; i < 4; i++)
            okr = okr && j.progbuf_rd(i, rb[i], "pb rd");
        bool match = okw && okr &&
                     rb[0]==pat[0] && rb[1]==pat[1] &&
                     rb[2]==pat[2] && rb[3]==pat[3];
        printf("[m7] (e1) progbuf raw r/w: wrote {0x%08x,0x%08x,0x%08x,"
               "0x%08x} readback {0x%08x,0x%08x,0x%08x,0x%08x}: %s\n",
               pat[0],pat[1],pat[2],pat[3], rb[0],rb[1],rb[2],rb[3],
               match ? "ok" : "FAIL");
        ok = ok && match;
    }

    // (e2) progbuf EXECUTION on the real core is NOT exercised here. Launching
    // the pb via a postexec GPR write (TDT_DM.v:624-627 pb_work_start ->
    // pb_work -> itr_send_pb -> the core executes the pb word) aborts Verilator
    // with "%Error: Active region did not converge" (a combinational loop) on
    // the REAL core -- for BOTH a memory pb (donor write_word_by_pb `sw`) and
    // a pure-ALU pb ({addi x13,x13,1; ebreak}). Classification:
    //   * Driver-side: NO. The postexec GPR write is issued exactly as the
    //     donor does (JTAG_DRV.vh write_word_by_pb :2071-2130 /
    //     read_word_by_pb :1959-2013); the (d) single-ITR path (same
    //     dm_core_itr channel, itr_work mode) passes; PB_ADDR 0x80001000 is a
    //     valid SRAM address (AXIAddrDecode slave 0 base 0x80000000).
    //   * RTL-side: YES. The DM's pb engine is unit-verified in isolation
    //     (test/m7/unit/dm_tb.cpp t12_progbuf, with a fake core that pulses
    //     core_dm_itr_done_i), and the single-ITR (itr_work) path passes
    //     e2e -- so the loop is in the real-core interaction with pb_work
    //     mode (back-to-back pb-injected instructions), not the DM FSM or the
    //     driver. Reported, not fixed (no RTL edits in M7 Task 7).
    // The required brief step (e) "progbuf DMI-write pattern + byte-exact
    // readback" is (e1) above and passes. The write_word_by_pb /
    // read_word_by_pb donor methods remain available (deliverable 2).

    // (f) step: dpc=LOOP head, dcsr.step=1, resume -> the core executes
    // EXACTLY one instruction (addi x10,x10,1) then re-halts (cause=4).
    // x10 (the loop counter) must advance by exactly 1.
    {
        uint64_t x10a = 0, x10b = 0, dpcr = 0, dcsr64 = 0;
        bool o1  = j.abstract_reg_read(R_X10, x10a, "x10 pre-step");
        bool o2  = j.abstract_reg_write(CSR_DPC, LOOP, "dpc write (step)");
        bool o3  = j.abstract_reg_read(CSR_DCSR, dcsr64, "dcsr read (step)");
        bool o4  = j.abstract_reg_write(CSR_DCSR,
                                        (uint64_t)((uint32_t)dcsr64 |
                                                   DCSR_STEP),
                                        "dcsr.step=1");
        // resume (step armed): runs one inst from dpc, then step-halts.
        bool ores = j.resume_req(dmstatus, "step resume");
        bool oh   = j.wait_halted(dmstatus, "step halt");
        bool o5  = j.abstract_reg_read(R_X10, x10b, "x10 post-step");
        bool o6  = j.abstract_reg_read(CSR_DPC, dpcr, "dpc post-step");
        bool o7  = j.abstract_reg_read(CSR_DCSR, dcsr64, "dcsr post-step");
        uint32_t cause = ((uint32_t)dcsr64 >> 6) & 7;
        printf("[m7] (f) step: x10 0x%llx -> 0x%llx (expect +1); "
               "dpc=0x%llx (expect 0x%llx); dcsr=0x%08x (cause=%u; "
               "expect 4=step)\n",
               (unsigned long long)x10a, (unsigned long long)x10b,
               (unsigned long long)dpcr, (unsigned long long)LOOP,
               (uint32_t)dcsr64, cause);
        bool okf = o1 && o2 && o3 && o4 && ores && oh && o5 && o6 && o7 &&
                   (x10b == x10a + 1) && (dpcr == LOOP) &&
                   (cause == CAUSE_STEP);
        printf("[m7] (f) check: %s\n", okf ? "ok" : "FAIL");
        ok = ok && okf;
    }

    // (g) resume (no step): clear dcsr.step=0 (still set from (f)), resume
    // -> core runs freely (anyrunning=1, anyhalted=0).
    {
        uint64_t dcsr64 = 0;
        bool o1  = j.abstract_reg_read(CSR_DCSR, dcsr64, "dcsr read (resume)");
        bool o2  = j.abstract_reg_write(CSR_DCSR,
                                        (uint64_t)((uint32_t)dcsr64 &
                                                   ~DCSR_STEP),
                                        "dcsr.step=0");
        bool ores = j.resume_req(dmstatus, "free resume");
        bool orun = j.wait_running(dmstatus, "post-resume running");
        uint32_t anyhalted  = (dmstatus >> 8) & 1;
        uint32_t anyrunning = (dmstatus >> 10) & 1;
        printf("[m7] (g) resume (step cleared): dmstatus=0x%08x "
               "(anyhalted=%u anyrunning=%u; expect 0/1)\n",
               dmstatus, anyhalted, anyrunning);
        bool okg = o1 && o2 && ores && orun &&
                   (anyhalted == 0) && (anyrunning == 1);
        printf("[m7] (g) check: %s\n", okg ? "ok" : "FAIL");
        ok = ok && okg;
    }

    // (h) dret: halt the running core (from (g)), position dpc at the loop
    // head, then resume by executing DRET (0x7b200073) via ITR. The core
    // exits debug mode (RTU.v:1143 retire_exit_debug = ... ex2_inst_dret)
    // and runs from dpc. Confirm by re-halting and checking dpc is back in
    // the loop and x10 advanced (genuinely running, not stuck).
    {
        uint64_t x10pre = 0, x10post = 0, dpcpost = 0;
        // the core is running (from (g)); halt it before any register r/w.
        bool oh1 = j.halt_req(dmstatus, "dret halt");
        bool o1  = j.abstract_reg_read(R_X10, x10pre, "x10 pre-dret");
        bool o2  = j.abstract_reg_write(CSR_DPC, LOOP, "dpc write (dret)");
        bool od  = j.execute_itr(DRET, "ITR dret");
        bool orun = j.wait_running(dmstatus, "post-dret running");
        // let it run a few loop iterations, then re-halt and verify.
        bool oh2 = j.halt_req(dmstatus, "post-dret re-halt");
        bool o3  = j.abstract_reg_read(CSR_DPC, dpcpost, "dpc post-dret");
        bool o4  = j.abstract_reg_read(R_X10, x10post, "x10 post-dret");
        bool inloop = (dpcpost >= LOOP && dpcpost < LOOPEN);
        printf("[m7] (h) dret: dmstatus(after)=0x%08x; re-halt dpc=0x%llx "
               "(expect in loop); x10 0x%llx -> 0x%llx (expect advanced)\n",
               dmstatus, (unsigned long long)dpcpost,
               (unsigned long long)x10pre, (unsigned long long)x10post);
        bool okh = oh1 && o1 && o2 && od && orun && oh2 && o3 && o4 &&
                   inloop && (x10post > x10pre);
        printf("[m7] (h) check: dret ran=%s dpc in loop=%s x10 advanced:%s\n",
               (od && orun) ? "ok" : "FAIL",
               inloop ? "ok" : "FAIL",
               (x10post > x10pre) ? "ok" : "FAIL");
        ok = ok && okh;
    }

    return ok;
}

// The Task-6 smoke (deliverable 3): run against a loaded ELF at reset
// release -- the core is powered, out of reset and running, so dmstatus
// reports version=2 with anyhalted=0 (no halt requested). `core_tick`
// is the harness's one-core-clock-cycle service callback (TB::
// core_tick); it runs between the TCK phases so the core KEEPS RUNNING
// (fetches/AXI/UART serviced) while the scan runs -- an un-serviced
// core would sample the pre-initialised memory response (0) on its
// first fetch and trap-loop on mtvec=0.
static bool m7_debug_smoke(const std::function<void()> &core_tick)
{
    printf("[m7] M7 debug smoke: JTAG DTM/DMI against the running core\n");
    M7JTAG j;
    j.tick = core_tick;
    bool ok = true;

    // (1) jtag_tlr: 5x TMS=1 + run to Idle; TDO must read 1 (TDT_DTM.v
    //     holds tdo=1 whenever the TAP is not shifting).
    uint32_t tdo = j.tlr();
    printf("[m7] (1) jtag_tlr: TDO=%u (expect 1)\n", tdo);
    if (tdo != 1) ok = false;

    // (2) IDCODE read: IR=5'h01, 32-bit DR (JTAG_DRV.vh:1040).
    uint64_t rd = 0;
    j.write_ir(M7_IR_IDCODE);
    rd = j.shift_dr(32, 0);
    printf("[m7] (2) IDCODE = 0x%08llx (expect 0x%08x)\n",
           (unsigned long long)rd, M7_IDCODE);
    if ((uint32_t)rd != M7_IDCODE) ok = false;

    // (3) DTMCS read: IR=5'h10, 32-bit DR; the initDTM checks
    // (JTAG_DRV.vh:1061-1079): version==1, abits==10; learn idle[14:12].
    j.write_ir(M7_IR_DTMCS);
    rd = j.shift_dr(32, 0);
    uint32_t dtmcs = (uint32_t)rd;
    uint32_t dtm_version = dtmcs & 0xF;
    uint32_t dtm_abits   = (dtmcs >> 4) & 0x3F;
    uint32_t dtm_idle    = (dtmcs >> 12) & 0x7;
    printf("[m7] (3) DTMCS = 0x%08x (version=%u abits=%u idle=%u; "
           "expect version=1 abits=10)\n", dtmcs, dtm_version, dtm_abits,
           dtm_idle);
    if (dtm_version != 1 || dtm_abits != 10) ok = false;
    j.idle_cycle_num = (dtm_idle ? (int)dtm_idle : 7);

    // (4) DMI write dmcontrol.dmactive=1 (dmcontrol=0x10; data=0x1), then
    // read dmcontrol back to prove the bit landed (the DM APB slave is
    // live regardless of dmactive, so the readback is the gate).
    uint32_t w = 1, rdata = 0;
    bool ok4w = j.dmi_rw_check(M7_DMI_WRITE, M7_DM_DMCONTROL, w,
                               "dmcontrol dmactive=1 write");
    bool ok4r = j.dmi_rw_check(M7_DMI_READ, M7_DM_DMCONTROL, rdata,
                               "dmcontrol readback");
    printf("[m7] (4) DMI write dmcontrol=0x%08x (dmactive=1): %s; "
           "readback dmcontrol=0x%08x (expect dmactive=1)\n", w,
           ok4w ? "ok" : "FAIL", rdata);
    ok = ok && ok4w && ok4r && ((rdata & 1) == 1);

    // (5) DMI read dmstatus (0x11): version==2 (TDT_DM.v DM_VERSION,
    //     spec 0.13), anyhalted==0 (bit 8 -- core running, no halt
    //     requested).
    uint32_t dmstatus = 0;
    bool ok5 = j.dmi_rw_check(M7_DMI_READ, M7_DM_DMSTATUS, dmstatus,
                              "dmstatus read");
    uint32_t dm_version  = dmstatus & 0xF;
    uint32_t anyhalted   = (dmstatus >> 8) & 1;
    uint32_t anyrunning  = (dmstatus >> 10) & 1;
    printf("[m7] (5) dmstatus = 0x%08x (version=%u anyhalted=%u "
           "anyrunning=%u; expect version=2 anyhalted=0): %s\n",
           dmstatus, dm_version, anyhalted, anyrunning,
           ok5 ? "ok" : "FAIL");
    ok = ok && ok5 && (dm_version == 2) && (anyhalted == 0);

    // (6) DMI write dmcontrol.dmactive=0 + readback (dmactive set/clear).
    uint32_t w0 = 0;
    bool ok6w = j.dmi_rw_check(M7_DMI_WRITE, M7_DM_DMCONTROL, w0,
                               "dmcontrol dmactive=0 write");
    bool ok6r = j.dmi_rw_check(M7_DMI_READ, M7_DM_DMCONTROL, rdata,
                               "dmcontrol readback after clear");
    printf("[m7] (6) DMI write dmcontrol=0x%08x (dmactive=0): %s; "
           "readback dmcontrol=0x%08x (expect dmactive=0)\n", w0,
           ok6w ? "ok" : "FAIL", rdata);
    ok = ok && ok6w && ok6r && ((rdata & 1) == 0);

    // M7 Task 7: the extended e2e (halt/abstract/ITR/progbuf/step/resume/
    // dret) against the spin ELF. The spin ELF NEVER terminates, so the
    // harness exits right after this smoke (TB::init, see below) -- the
    // marker below gates the whole sequence (base 6 steps + extended).
    ok = ok && m7_debug_extended(j);

    dut.jtag.run_to_idle();
    printf("[m7] JTAG scan totals: %llu TCK cycles "
           "(clk/tck=8 interleave, TDO negedge-sampled)\n",
           (unsigned long long)dut.jtag.tck_cycles);
    if (ok)
        printf("M7-DEBUG-PASS\n");
    else
        printf("M7-DEBUG-FAIL\n");
    return ok;
}

//=============================================================================
// M1 FetchSink accessors -- RETIRED in Task 7 (FetchSink.v deleted)
//=============================================================================
// These functions used to poke/sample FetchSink's harness config bank and
// committed-stream registers through the (now removed) rtl/verisim.h
// M1_* macros. The real core has no such registers: the config bank is
// CSR.v's MHCR (written by the test program itself via CSRS, not poked by
// the harness), and the committed stream is RTU's per-retire trace, which
// Task 8 re-exports (contract 13). Per Task 7.2's bar ("compiles and
// links; dead M1 paths neutralized with a pointer at Task 8"), the
// functions below are NEUTRALIZED -- no-ops and constant returns -- and
// the M1 test bodies that consumed them (the online committed-stream
// checker, see TB::term()/TB::sample_commit()) are disabled below. Task
// 8 replaces this whole namespace with the real retire-trace accessors.
namespace m1sink {

// Config bank: no RTL target exists anymore (it was FetchSink's; the real
// bank is CSR.v's MHCR). No-op; Task 8 owns any harness-side config.
static void poke_cfg(int rung, bool sink_stall, uint32_t max_insts)
{
    (void) rung; (void) sink_stall; (void) max_insts;
}

// --inv-test pulse: no RTL target (it was FetchSink's cfg_bht_inv /
// cfg_btb_clr passthroughs; the real BPU config is CSR.v's MHCR). No-op.
static void set_inv(int which)
{
    (void) which;
}

// --fencei-patch: no RTL target (it was FetchSink's cfg_icache_inv; the
// real ICache invalidate request is CSR.v's cp0_ifu_icache_inv_req, which
// M2's minimal CSR set does not expose to the harness). No-op.
static void set_icache_inv(bool on)
{
    (void) on;
}

// Committed stream + resolve event/kind: the M1 fetch-stream oracle has
// no consumer (Task 7). Neutral constants until Task 8 exports RTU's
// per-retire trace (contract 13).
static bool     cmt_valid()     { return false; }
static uint64_t cmt_pc()        { return 0; }
static uint32_t cmt_opcode()    { return 0; }
static uint64_t cmt_count()     { return 0; }
static bool     resolve_event() { return false; }
static unsigned resolve_kind()  { return 0; }
static unsigned perr_code()     { return 0; }

} // namespace m1sink

//=============================================================================
// M1 harness: CLI options, config-bank poke, online committed-stream checker
//=============================================================================
// The M0 TestBench uses its own parse_arg (NOT Verilator plusargs), and it
// treats every unrecognised argument as an ELF file name, so the M1 flags
// are stripped from argv in main() below before TestBench::parse_arg ever
// sees them. That keeps testbench/TestBench.cpp's protocol untouched: the
// extra M1 checks are layered on the TB side (init/step/term) instead.
//
// TASK 7 NOTE: the M1 flags below are all parsed but INERT now that
// FetchSink is gone (their poke/sample targets are the neutralized
// m1sink:: no-ops above); --iss-selftest is the only one with a live
// effect. Task 8 replaces this machinery with the per-retire trace
// harness (contract 13).
//
//   --m1-rung=<1..4>   predictor chicken-bit ladder (spec S4.3, cumulative):
//                      1 = all predictors off, 2 = +RAS, 3 = +BTB, 4 = +BHT.
//   --max-insts=<N>    FetchSink's commit budget (default 200000); running
//                      out is reported as perr_code 1 (rtl/FetchSink.v), not
//                      as a tohost value.
//   --inv-test         pulse cfg_bht_inv / cfg_btb_clr mid-run, one at a
//                      time (pacing: see TB::inv_pulse_update() below).
//   --sink-stall       FetchSink's pseudo-random id_stall mode.
//   --fencei-patch=<addr>:<word32>[:<commit>]
//                      the fence.i mechanism (Task 6.1; see test/m1/
//                      fencei.S). M1 has no store unit and cannot execute
//                      fence.i itself, so BOTH halves of a self-modifying-
//                      code test are the host's job: at the COMMIT BOUNDARY
//                      <commit> (default 1000, a count of instructions
//                      already compared by the checker, not a cycle) the
//                      host (a) writes <word32> into the ELF image in ExtMem
//                      at <addr> and (b) pulses cfg_icache_inv for one
//                      cycle. Both oracles switch images at EXACTLY that
//                      boundary: the RTL because the invalidate throws out
//                      the stale line and forces a refill from the patched
//                      memory, the golden ISS because it reads the image
//                      lazily, at the moment each instruction is committed
//                      (m1_iss.h "LAZINESS"). If the flag is given the run
//                      FAILS unless the patch actually fired -- otherwise
//                      fencei.S would pass vacuously against an unpatched
//                      image.
//   --iss-selftest     run the golden ISS's own gate and exit (no RTL) --
//                      plan Task 5.3.
//   --no-checker       sample nothing, compare nothing (debug escape
//                      hatch, same precedent as rv12's own harness: useful
//                      during Task 6 bring-up to tell a raw RTL hang/crash
//                      apart from a bug in the checker itself).
//=============================================================================

struct M1Opts {
    int      rung = 1;
    uint32_t max_insts = 200000;
    bool     inv_test = false;
    bool     sink_stall = false;
    bool     checker_on = true;
    // --fencei-patch
    bool     fencei_armed = false;
    uint64_t fencei_addr = 0;
    uint32_t fencei_word = 0;
    uint64_t fencei_commit = 1000;
};
static M1Opts m1_opts;

// Exit status for any M1 checker failure. FetchSink's own tohost protocol
// (rtl/FetchSink.v, mirrored from M0's TestMaster/rv12's FetchSink) uses
// 1=PASS (sentinel reached) and 3=FAIL (perr_code != 0, e.g. --max-insts
// exhausted); a checker-level verdict (stream divergence, or a run that
// never reported anything through tohost at all) is a THIRD, harness-owned
// outcome and gets its own status rather than overloading either.
#define M1_FAIL_STATUS 2

struct M1Checker {
    static const int HIST = 16;

    m1::Iss  iss;
    uint64_t compared = 0;
    m1::Entry iss_hist[HIST];
    struct RtlEnt { uint64_t pc; uint32_t opcode; uint64_t cycle; };
    RtlEnt   rtl_hist[HIST] = {};
    int      hist_n = 0;

    void arm(m1::Iss::ReadHalf reader, uint64_t start_pc) {
        iss.reset(reader, start_pc);
    }

    void dump_streams() const {
        const int n = hist_n < HIST ? hist_n : HIST;
        const int first = hist_n - n;
        printf("[checker] last %d ISS (expected) entries:\n", n);
        for (int i = 0; i < n; i++) {
            const m1::Entry &e = iss_hist[(first + i) % HIST];
            printf("[checker]   #%llu pc=%016llx opcode=%08x %s\n",
                   (unsigned long long)(first + i), (unsigned long long)e.pc,
                   e.opcode, e.rvc ? "rvc" : "32b");
        }
        printf("[checker] last %d committed (RTL) entries:\n", n);
        for (int i = 0; i < n; i++) {
            const RtlEnt &r = rtl_hist[(first + i) % HIST];
            printf("[checker]   #%llu pc=%016llx opcode=%08x  (cycle %llu)\n",
                   (unsigned long long)(first + i), (unsigned long long)r.pc,
                   r.opcode, (unsigned long long)r.cycle);
        }
    }

    // One committed instruction. Diverging is fatal: the streams are only
    // meaningful in order, so there is nothing useful to check afterward.
    void slot(uint64_t cycle, uint64_t pc, uint32_t opcode) {
        m1::Entry e = iss.next();
        const bool op_ok = e.rvc ? ((opcode & 0xFFFFU) == (e.opcode & 0xFFFFU))
                                  : (opcode == e.opcode);
        iss_hist[hist_n % HIST] = e;
        rtl_hist[hist_n % HIST] = RtlEnt{ pc, opcode, cycle };
        hist_n++;
        compared++;
        if (pc != e.pc || !op_ok) {
            printf("\n[checker] M1-CHECKER-FAIL: committed stream diverged "
                   "from the golden ISS at entry #%llu (cycle %llu)\n",
                   (unsigned long long)(compared - 1), (unsigned long long)cycle);
            printf("[checker]   expected  pc=%016llx opcode=%08x (%s)\n",
                   (unsigned long long)e.pc, e.opcode, e.rvc ? "rvc" : "32b");
            printf("[checker]   committed pc=%016llx opcode=%08x\n",
                   (unsigned long long)pc, opcode);
            dump_streams();
            fflush(stdout);
            exit(M1_FAIL_STATUS);
        }
    }
};
static M1Checker m1_chk;

#define	RVProcAXI RVProcAXI_Verilator

struct TB : public TestBench {
    // AXI memory liveness counters, reported periodically from step().
    uint64_t mem_reads = 0, mem_writes = 0;
    uint64_t mem_last_addr = 0;
    uint64_t tb_cycle = 0;

    // --inv-test pulse bookkeeping. BHT_INV_CYCLES (rvproc_pkg.sv) is 1024
    // cycles for a full sweep; there is no equivalent BTB constant
    // (BTB_ENTRIES=16, so its clear is expected to be far shorter than
    // BHT's 1024-row scan), so both kinds share the same 2048-cycle spacing
    // -- comfortably longer than the documented BHT sweep with margin left
    // over for the shorter, undocumented BTB one. First pulse at 2048
    // cycles gives a short directed test time to actually get running
    // first (documented choice, plan Task 5.2).
    static const uint64_t INV_FIRST   = 2048;
    static const uint64_t INV_SPACING = 2048;
    static const int      INV_PULSES  = 6;   // 3 bht + 3 btb, alternating
    int  inv_issued = 0;
    bool inv_active = false;

    // --fencei-patch bookkeeping (see the flag documentation above and
    // test/m1/fencei.S for the program-side mechanism).
    bool     fencei_fired = false;
    bool     fencei_inv_active = false;
    uint32_t fencei_before = 0;

    TB() {
        // M6 Task 8: fw_jump's fw_next_arg1 returns the FIXED FDT address
        // 0x82200000 (opensbi-1.3 firmware/fw_jump.S:46-52, verified by
        // disassembly of the prebuilt fw_jump.elf) -- the TB must place the
        // generated DTB there. 0x82200000 sits between the kernel image
        // (0x80200000, ~14 MB) and the initrd (0x84000000).
        dtb_addr = 0x82200000;
        initrd_addr = 0x84000000;
    }


    void build_fdt(FDT::Node *root) override {
        uint64_t *prop_data64;
        uint32_t *prop_data32;

        // cpus
        FDT::Node cpus = root->create("cpus");
        cpus.setprop("#address-cells", (uint32_t)1);
        cpus.setprop("#size-cells", (uint32_t)0);
        cpus.setprop("timebase-frequency", (uint32_t)1000000);

        FDT::Node cpu = cpus.create("cpu@0");
        cpu.setprop("device_type", "cpu");
        cpu.setprop("reg", (uint32_t)0);
        cpu.setprop("status", "okay");
        cpu.setprop("compatible", "riscv");
#if CONFIG_RV64I
        // M6 Task 5: advertise the full implemented ISA (M3 A, M5 F/D, M4
        // S/U) + the two implemented Z* extensions. Must stay in lockstep
        // with CSR.v's misa_value (IMACFDSU) -- the kernel cross-checks the
        // two. No zbb/zicbom: SVPBMT/ZBB/ZICBOM are hwcap-gated, safe to omit.
        cpu.setprop("riscv,isa", "rv64imafdc_zicsr_zifencei");
        cpu.setprop("mmu-type", "riscv,sv39");
#else
        cpu.setprop("riscv,isa", "rv32imac");
        cpu.setprop("mmu-type", "riscv,sv32");
#endif
        cpu.setprop("clock-frequency", (uint32_t)20000000);

        FDT::Node intc = cpu.create("interrupt-controller");
        intc.setprop("#interrupt-cells", (uint32_t)1);
        intc.setprop("compatible", "riscv,cpu-intc");
        intc.setprop("interrupt-controller");
        uint32_t intc_phandle = intc.get_phandle();

        // memory
        FDT::Node memory = root->create("memory@80000000");
        memory.setprop("device_type", "memory");
        memory.setprop("reg", 2, &prop_data64);
        prop_data64[0] = cpu_to_fdt64(0x80000000LL);
        prop_data64[1] = cpu_to_fdt64(mem_size);

        // clint@2000000
        FDT::Node clint = root->create("clint@2000000");
        clint.setprop("compatible", "riscv,clint0");
        clint.setprop("reg", 2, &prop_data64);
        prop_data64[0] = cpu_to_fdt64(0x02000000LL);
        prop_data64[1] = cpu_to_fdt64(0x10000LL);
        clint.setprop("interrupts-extended", 4, &prop_data32);
        prop_data32[0] = cpu_to_fdt32(intc_phandle);
        prop_data32[1] = cpu_to_fdt32(3);   // MSIP
        prop_data32[2] = cpu_to_fdt32(intc_phandle);
        prop_data32[3] = cpu_to_fdt32(7);   // MTIP

        // plic@c000000
        FDT::Node plic = root->create("plic@c000000");
        plic.setprop("compatible", "riscv,plic0");
        plic.setprop("#interrupt-cells", (uint32_t)1);
        plic.setprop("interrupt-controller");
        plic.setprop("riscv,ndev", (uint32_t)7);
        plic.setprop("reg", 2, &prop_data64);
        prop_data64[0] = cpu_to_fdt64(0x0C000000LL);
        prop_data64[1] = cpu_to_fdt64(0x1000000LL);
        plic.setprop("interrupts-extended", 4, &prop_data32);
        prop_data32[0] = cpu_to_fdt32(intc_phandle);
        prop_data32[1] = cpu_to_fdt32(11);  // MEIP
        prop_data32[2] = cpu_to_fdt32(intc_phandle);
        prop_data32[3] = cpu_to_fdt32(9);   // SEIP
        uint32_t plic_phandle = plic.get_phandle();

        // uart@10001000
        FDT::Node uart = root->create("uart@10001000");
        uart.setprop("compatible", "ns16550a");
        uart.setprop("reg", 2, &prop_data64);
        prop_data64[0] = cpu_to_fdt64(UART_ADDR);
        prop_data64[1] = cpu_to_fdt64(0x20LL);
        uart.setprop("reg-shift", (uint32_t)2);
        uart.setprop("reg-io-width", (uint32_t)4);
        uart.setprop("clock-frequency", (uint32_t)2000000);
        uart.setprop("interrupts", (uint32_t)1);
        uart.setprop("interrupt-parent", plic_phandle);
    }

    void init() override {
        INIT_REG(cpu, initial_pc, initial_sp, dtb_addr);
        printf("[init] initial_pc=%lx initial_sp=%lx dtb_addr=%lx\n",
            (unsigned long)initial_pc, (unsigned long)initial_sp, (unsigned long)dtb_addr);

        // M7 Task 7: --m7-debug -- run the JTAG DMI smoke (m7_debug_smoke,
        // base 6 steps) + the extended e2e (m7_debug_extended: halt /
        // abstract GPR+CSR r/w / ITR / progbuf / step / resume / dret)
        // against the spin ELF, now that the core is out of reset and
        // running, BEFORE normal stepping begins. The JTAG driver
        // (dut.cpp) owns tck and interleaves it with the core clk; between
        // the TCK phases it advances the core through core_tick() -- the
        // full step() protocol (memory/AXI/UART service) -- so the core
        // keeps executing for the cycles the scan takes. The verdict
        // (M7-DEBUG-PASS/FAIL) gates the WHOLE sequence. The spin ELF
        // NEVER terminates (no tohost write), so the harness EXITS here
        // (0 on PASS, 1 on FAIL) instead of entering the normal run loop.
        if (m7_opts.debug) {
            exit(m7_debug_smoke([this] { core_tick(); }) ? 0 : 1);
        }

        // Config bank: poked after dut.init() and before the first step,
        // per the plan's pinned harness config mechanism (owner: Task 5.2).
        // These registers have no RTL driver, so what is written here
        // stays written for the rest of the run.
        const int r = m1_opts.rung;
        m1sink::poke_cfg(r, m1_opts.sink_stall, m1_opts.max_insts);
        printf("[m1] rung=%d (ras=%d btb=%d bht=%d) sink_stall=%d "
               "inv_test=%d max_insts=%u checker=%d\n",
               r, (int)(r >= 2), (int)(r >= 3), (int)(r >= 4),
               (int)m1_opts.sink_stall, (int)m1_opts.inv_test,
               m1_opts.max_insts, (int)m1_opts.checker_on);

        // Arm the golden ISS on the image already loaded into ExtMem. The
        // walk starts at the RESET VECTOR the core actually boots from
        // (FetchSink.v's RESET_VECTOR parameter / cp0_xx_mrvbr, m1::
        // RESET_VECTOR), which is not necessarily the ELF entry point.
        // Reuses TestBench::get_page (base-class infra, testbench/
        // TestBench.cpp) rather than re-deriving ExtMem's page-table walk;
        // the two independent get_page() calls (not one call plus a +1
        // offset within the same page) are what correctly handle a
        // halfword straddling a 4KB page boundary.
        if (m1_opts.checker_on) {
            if (initial_pc != m1::RESET_VECTOR)
                printf("[m1] NOTE: ELF entry %llx differs from the reset "
                       "vector %llx; the ISS follows the reset vector\n",
                       (unsigned long long)initial_pc,
                       (unsigned long long)m1::RESET_VECTOR);
            m1_chk.arm([](uint64_t addr) -> uint16_t {
                           uint8_t *lo_pg = (uint8_t *)TestBench::inst->get_page(addr);
                           uint8_t *hi_pg = (uint8_t *)TestBench::inst->get_page(addr + 1);
                           uint8_t lo = lo_pg[addr & 0xfff];
                           uint8_t hi = hi_pg[(addr + 1) & 0xfff];
                           return (uint16_t)lo | ((uint16_t)hi << 8);
                       },
                       m1::RESET_VECTOR);
        }

        axi_uart.start(alloc_terminal);
        if (expect)
            axi_uart.expect(expect);
    }

    // Sample the committed instruction FetchSink exported on this cycle's
    // edge (verisim.h: valid/pc/opcode, ONE slot -- C906 delivers a single
    // instruction/cycle to IDU).
    void sample_commit() {
        if (!m1_opts.checker_on)
            return;
        if (m1sink::cmt_valid()) {
            m1_chk.slot(tb_cycle, m1sink::cmt_pc(), m1sink::cmt_opcode());
            // Strictly BETWEEN two compared commits: everything up to here
            // was decoded from the old image (by both oracles), everything
            // after it from the new one (Task 6 fencei mechanism).
            if (m1_opts.fencei_armed && !fencei_fired &&
                m1_chk.compared >= m1_opts.fencei_commit)
                fencei_apply();
        }
    }

    // --fencei-patch pacing/pulse-clear (Task 6.1; test/m1/fencei.S documents
    // the program side). Same one-cycle "raise, then drop before the next
    // sample" discipline as inv_pulse_update() below: cfg_icache_inv is
    // rising-edge detected on ICache.v's own consumer side (SECTION
    // INVALIDATE: `inv_req_rise = cp0_ifu_icache_inv_req && !inv_req_r`), so
    // it must be dropped BEFORE sample_commit() could raise it again on a
    // later commit -- dropping it later in the same step would clear the
    // request before any clock edge ever sampled it.
    void fencei_pulse_clear() {
        if (fencei_inv_active) {
            m1sink::set_icache_inv(false);
            fencei_inv_active = false;
        }
    }

    // The fence.i mechanism (plan Task 6.1). M1 has no store unit and no
    // fence.i execution, so BOTH halves of a self-modifying-code test are the
    // host's job:
    //   * the WRITE: the host patches the ELF image in ExtMem, which is what
    //     the RTL's next refill will read AND what the golden ISS's lazy
    //     rd_half() will read from that point on;
    //   * the INVALIDATE: the host pulses cfg_icache_inv, the wire
    //     ICache.v's fence.i path is driven from (FetchSink.v's CP0
    //     stand-in), so the ICache runs its real 256-set INV_ALL walk and the
    //     patched line has to be refetched rather than served stale.
    // Both must land at the SAME point in the committed stream in both
    // worlds: the patch is applied after exactly `fencei_commit` instructions
    // have been COMPARED (a stream position, not a cycle), so it is identical
    // regardless of stalls/refills/--sink-stall -- the golden ISS sees the
    // old bytes for every earlier instruction and the new bytes for every
    // later one, by construction (m1_iss.h's laziness note). On the RTL side
    // the guarantee is the test program's: fencei.S puts far more committed
    // filler between the patch boundary and the next fetch of the patched
    // line than the IBUF (6 halfwords) plus the pipeline's few stages can
    // hold in flight, so nothing stale can still be buffered when the
    // invalidate fires.
    void fencei_apply() {
        const uint64_t base = m1_opts.fencei_addr & ~7ULL;
        const unsigned sh   = (m1_opts.fencei_addr & 4) ? 32 : 0;
        uint64_t w = read_mem(base);
        fencei_before = (uint32_t)(w >> sh);
        w = (w & ~(0xFFFFFFFFULL << sh)) |
            ((uint64_t)m1_opts.fencei_word << sh);
        TestBench::inst->write_mem(base, w);

        m1sink::set_icache_inv(true);      // one-cycle pulse, cleared above
        fencei_inv_active = true;
        fencei_fired = true;
        printf("[m1] fence.i patch at commit %llu (cycle %llu): [%016llx] "
               "%08x -> %08x, cfg_icache_inv pulsed\n",
               (unsigned long long)m1_chk.compared, (unsigned long long)tb_cycle,
               (unsigned long long)m1_opts.fencei_addr, fencei_before,
               m1_opts.fencei_word);
    }

    // Mid-run invalidate sweeps for --inv-test: one bit at a time, held for
    // a single cycle, INV_SPACING cycles apart so a 1024-cycle BHT walk
    // always finishes before the next pulse (starving the front end
    // otherwise) -- spec S4.3: "invalidate sweeps (BHT/BTB) run mid-test at
    // the top rung without corrupting the stream." The checker's
    // predictor-agnostic contract means this is safe to pulse at any rung
    // (a disabled predictor has nothing to invalidate), but it is only
    // MEANINGFUL at --m1-rung=4.
    void inv_pulse_update() {
        if (!m1_opts.inv_test)
            return;
        if (inv_active) {
            m1sink::set_inv(-1);
            inv_active = false;
        }
        if (inv_issued >= INV_PULSES || tb_cycle < INV_FIRST)
            return;
        if (((tb_cycle - INV_FIRST) % INV_SPACING) != 0)
            return;
        static const char *const kind[2] = { "bht", "btb" };
        const int which = inv_issued % 2;
        m1sink::set_inv(which);
        inv_active = true;
        inv_issued++;
        printf("[m1] inv pulse %d/%d: %s at cycle %llu\n", inv_issued,
               INV_PULSES, kind[which], (unsigned long long)tb_cycle);
    }

    // M7 Task 6: one full core clock cycle with full peripheral
    // servicing -- the step() protocol factored out so the --m7-debug
    // JTAG smoke (TB::init) can advance the core cycle-by-cycle between
    // TCK phases (JTAG::cycle's core_tick callback). The core keeps
    // running (fetches/AXI/UART serviced) while the scan runs; without
    // this its first fetch would sample the pre-initialised memory
    // response (0) and trap-loop on mtvec=0.
    bool core_tick() {
        bool quitted = RVProcAXI(&axi_bus, &io_pins);

        // Post-clock: the committed-stream registers hold what this edge
        // retired, so sample before anything else disturbs the model.
        tb_cycle++;

        fencei_pulse_clear();
        sample_commit();
        inv_pulse_update();

        // Post-clock device servicing: results driven here are seen by the
        // core at the NEXT step's pre-clock. The memory path (xmem.update)
        // is 1-cycle; the UART AXI4L converter path (uart_cvt.fsm) is
        // 2-cycle. This order is load-bearing -- do not reorder.
        xmem.update(&io_pins.mpin);

        if (io_pins.mpin.cs & 1) {
            if (io_pins.mpin.we) mem_writes++; else mem_reads++;
            mem_last_addr = io_pins.mpin.addr;
        }
        if ((tb_cycle % 1000000) == 0)
            printf("[axi] mem rd=%llu wr=%llu last_addr=%llx\n",
                   (unsigned long long)mem_reads, (unsigned long long)mem_writes,
                   (unsigned long long)mem_last_addr);

        uart_cvt.fsm(&axi_bus.s_ch[DI_UART], &io_pins.uart_ch);
        axi_uart.update(&io_pins.uart_ch);

        return quitted;
    }

    bool step() override {
        return core_tick();
    }

    // End-of-run verdict. TestBench::run() calls term() BEFORE its
    // --print-result block, so failing here (hard exit) means the
    // harness's own PASS/FAIL line is never printed: the extra M1
    // conditions are layered on top of the tohost protocol, not a
    // replacement for it.
    void term() override {
        axi_uart.stop(alloc_terminal);

        if (!m1_opts.checker_on) {
            printf("[checker] disabled (--no-checker); nothing was compared\n");
            return;
        }

        // Task 7: the M1 fetch-stream oracle is RETIRED -- FetchSink (and
        // its committed-stream registers) no longer exist, so there is
        // nothing for this checker to sample or compare. The tohost
        // protocol verdict still applies via TestBench's own result block
        // (run() calls term() before printing it); the per-retire trace
        // harness that replaces the deleted M1 decision block arrives in
        // Task 8 (contract 13). test/m1/run_all.sh --full-matrix is
        // therefore expected to fail / be meaningless from the Task 7
        // commit on -- a documented end of that regression, not a bug to
        // chase.
        printf("[checker] AXI memory traffic: %llu reads, %llu writes\n",
               (unsigned long long)mem_reads, (unsigned long long)mem_writes);
        printf("[m1] fetch-stream checker RETIRED in Task 7 (FetchSink "
               "removed; no committed-stream registers on the real core) "
               "-- Task 8 installs the per-retire trace harness "
               "(contract 13); no stream comparison this run\n");
    }


    // M2 restore point for the D-cache-mirror fast path; xmem-only is correct for M0 (no cache in the RTL).
    uint64_t read_mem(uint64_t addr) override {
        ExtMem::page *page = xmem.get_page(addr);
        return page->m[(addr & 0xfff) / sizeof(page->m[0])];
    }
};

static TB tb;

// Pull the M1 flags out of argv (TestBench::parse_arg would otherwise take
// them for ELF file names) and leave everything else untouched for the M0
// harness -- same mechanism rv12's own harness used.
static bool m1_take_uint(const char *arg, const char *flag, uint64_t *out)
{
    const size_t n = strlen(flag);
    if (strncmp(arg, flag, n) != 0 || arg[n] != '=')
        return false;
    char *end = NULL;
    const unsigned long long v = strtoull(arg + n + 1, &end, 0);
    if (end == arg + n + 1 || (end && *end)) {
        fprintf(stderr, "%s: expected an integer\n", arg);
        exit(M1_FAIL_STATUS);
    }
    *out = (uint64_t)v;
    return true;
}

// --fencei-patch=<addr>:<word32>[:<commit>] (Task 6.1) -- see the flag
// documentation above and TB::fencei_apply()/TB::sample_commit() above.
static bool m1_take_fencei(const char *arg)
{
    static const char flag[] = "--fencei-patch";
    const size_t n = sizeof(flag) - 1;
    if (strncmp(arg, flag, n) != 0 || arg[n] != '=')
        return false;

    const char *p = arg + n + 1;
    char *end = NULL;
    bool ok = true;

    const unsigned long long addr = strtoull(p, &end, 0);
    if (end == p || *end != ':')
        ok = false;

    unsigned long long word = 0;
    if (ok) {
        p = end + 1;
        word = strtoull(p, &end, 0);
        if (end == p || (*end != '\0' && *end != ':') || word > 0xFFFFFFFFULL)
            ok = false;
    }
    if (ok && *end == ':') {
        p = end + 1;
        const unsigned long long at = strtoull(p, &end, 0);
        if (end == p || *end != '\0' || at == 0)
            ok = false;
        else
            m1_opts.fencei_commit = (uint64_t)at;
    }
    if (!ok) {
        fprintf(stderr, "--fencei-patch=<addr>:<word32>[:<commit>] expected\n");
        exit(M1_FAIL_STATUS);
    }
    if (addr & 3ULL) {
        fprintf(stderr, "--fencei-patch: address must be 4-byte aligned\n");
        exit(M1_FAIL_STATUS);
    }

    m1_opts.fencei_armed = true;
    m1_opts.fencei_addr  = (uint64_t)addr;
    m1_opts.fencei_word  = (uint32_t)word;
    return true;
}

int main(int argc, char** argv)
{
    std::vector<char *> kept;
    kept.push_back(argv[0]);

    for (int i = 1; i < argc; i++) {
        char *cp = argv[i];
        uint64_t v = 0;

        if (strcmp(cp, "--iss-selftest") == 0)
            return m1::selftest() ? 0 : M1_FAIL_STATUS;
        if (strcmp(cp, "--inv-test") == 0)   { m1_opts.inv_test = true; continue; }
        if (strcmp(cp, "--sink-stall") == 0) { m1_opts.sink_stall = true; continue; }
        if (strcmp(cp, "--no-checker") == 0) { m1_opts.checker_on = false; continue; }
        // M7 Task 6: opt-in JTAG DMI smoke mode (default off; the OFF-path
        // identity is untouched -- without the flag the JTAG pads stay
        // idle and no smoke runs).
        if (strcmp(cp, "--m7-debug") == 0)  { m7_opts.debug = true; continue; }
        if (m1_take_fencei(cp))              continue;
        if (m1_take_uint(cp, "--m1-rung", &v)) {
            if (v < 1 || v > 4) {
                fprintf(stderr, "--m1-rung must be 1..4\n");
                return M1_FAIL_STATUS;
            }
            m1_opts.rung = (int)v;
            continue;
        }
        if (m1_take_uint(cp, "--max-insts", &v)) {
            if (v == 0 || v > 0xFFFFFFFFULL) {
                fprintf(stderr, "--max-insts must be 1..4294967295\n");
                return M1_FAIL_STATUS;
            }
            m1_opts.max_insts = (uint32_t)v;
            continue;
        }
        kept.push_back(cp);
    }

    return TestBench::inst->run((int)kept.size(), &kept[0]);
}
