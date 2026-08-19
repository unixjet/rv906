#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <sstream>

#include "RVProc.h"
#include "io/RVProc_io.h"
#include "io/ExtMem.h"
#include "device/uart16550.h"
#include "testbench/TestBench.h"

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
    bool quitted = dut.step(&io_pins->mpin);
    dut.read<DUT_DI_UART>(&axi_bus->s_ch[DI_UART]);
    dut.sync(cpu);
    return quitted;
}
#define INIT_REG(cpu, initial_pc, initial_sp, dtb_addr) \
    do{dut.init(initial_pc, initial_sp, dtb_addr); dut.sync(cpu);}while(0)

#define	RVProcAXI RVProcAXI_Verilator

struct TB : public TestBench {
    // AXI memory liveness counters, reported periodically from step().
    uint64_t mem_reads = 0, mem_writes = 0;
    uint64_t mem_last_addr = 0;
    uint64_t tb_cycle = 0;

    TB() {
        dtb_addr = 0x87000000;
        initrd_addr = 0x84000000;
    }


    void build_fdt(FDT::Node *root) override {
        uint64_t *prop_data64;
        uint32_t *prop_data32;

        // cpus
        FDT::Node cpus = root->create("cpus");
        cpus.setprop("#address-cells", (uint32_t)1);
        cpus.setprop("#size-cells", (uint32_t)0);
        cpus.setprop("timebase-frequency", (uint32_t)1250000);

        FDT::Node cpu = cpus.create("cpu@0");
        cpu.setprop("device_type", "cpu");
        cpu.setprop("reg", (uint32_t)0);
        cpu.setprop("status", "okay");
        cpu.setprop("compatible", "riscv");
#if CONFIG_RV64I
        cpu.setprop("riscv,isa", "rv64imac");
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
        axi_uart.start(alloc_terminal);
        if (expect)
            axi_uart.expect(expect);
    }

    bool step() override {
        bool quitted = RVProcAXI(&axi_bus, &io_pins);

        // Post-clock device servicing: results driven here are seen by the
        // core at the NEXT step's pre-clock. The memory path (xmem.update)
        // is 1-cycle; the UART AXI4L converter path (uart_cvt.fsm) is
        // 2-cycle. This order is load-bearing -- do not reorder.
        xmem.update(&io_pins.mpin);

        if (io_pins.mpin.cs & 1) {
            if (io_pins.mpin.we) mem_writes++; else mem_reads++;
            mem_last_addr = io_pins.mpin.addr;
        }
        if ((++tb_cycle % 1000000) == 0)
            printf("[axi] mem rd=%llu wr=%llu last_addr=%llx\n",
                   (unsigned long long)mem_reads, (unsigned long long)mem_writes,
                   (unsigned long long)mem_last_addr);

        uart_cvt.fsm(&axi_bus.s_ch[DI_UART], &io_pins.uart_ch);
        axi_uart.update(&io_pins.uart_ch);

        return quitted;
    }

    void term() override {
        axi_uart.stop(alloc_terminal);
    }

    // M2 restore point for the D-cache-mirror fast path; xmem-only is correct for M0 (no cache in the RTL).
    uint64_t read_mem(uint64_t addr) override {
        ExtMem::page *page = xmem.get_page(addr);
        return page->m[(addr & 0xfff) / sizeof(page->m[0])];
    }
};

static TB tb;

int main(int argc, char** argv)
{
    return TestBench::inst->run(argc, argv);
}
