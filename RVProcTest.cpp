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

    bool step() override {
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
