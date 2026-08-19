//=============================================================================
// int_test.c - Interrupt Test
//=============================================================================
// Tests machine timer interrupt (MTIP) and software interrupt (MSIP).
//=============================================================================

#include <stdint.h>
#include <stdio.h>

// CLINT base address
#define CLINT_BASE      0x02000000UL
#define CLINT_MSIP      0x0000
#define CLINT_MTIMECMP  0x4000
#define CLINT_MTIME     0xBFF8

#define CLINT_REG32(offset) (*(volatile uint32_t *)(CLINT_BASE + (offset)))
#define CLINT_REG64(offset) (*(volatile uint64_t *)(CLINT_BASE + (offset)))

// CSR access
#define read_csr(reg) ({ unsigned long __tmp; \
    asm volatile ("csrr %0, " #reg : "=r"(__tmp)); __tmp; })
#define write_csr(reg, val) ({ \
    asm volatile ("csrw " #reg ", %0" :: "rK"(val)); })
#define set_csr(reg, bit) ({ \
    asm volatile ("csrs " #reg ", %0" :: "rK"(bit)); })
#define clear_csr(reg, bit) ({ \
    asm volatile ("csrc " #reg ", %0" :: "rK"(bit)); })

// mstatus bits
#define MSTATUS_MIE     (1UL << 3)

// mie/mip bits
#define MIP_MSIP        (1UL << 3)   // Machine Software Interrupt
#define MIP_MTIP        (1UL << 7)   // Machine Timer Interrupt

// Interrupt counters
volatile int msip_count = 0;
volatile int mtip_count = 0;

// Trap handler (called from entry.S)
void trap_handler(void) {
    uint64_t mcause = read_csr(mcause);
    uint64_t mepc = read_csr(mepc);
    uint64_t mstatus = read_csr(mstatus);
    uint64_t mip = read_csr(mip);
    uint64_t mtval = read_csr(mtval);

    printf("[TRAP] mcause=0x%016lx mepc=0x%016lx\n", mcause, mepc);
    printf("       mstatus=0x%016lx mip=0x%016lx mtval=0x%016lx\n", mstatus, mip, mtval);

    if (mcause == (0x8000000000000003ULL)) {  // Machine software interrupt
        msip_count++;
        printf("  -> MSIP handled\n");
        // Clear MSIP
        CLINT_REG32(CLINT_MSIP) = 0;
    } else if (mcause == (0x8000000000000007ULL)) {  // Machine timer interrupt
        mtip_count++;
        printf("  -> MTIP handled\n");
        // Clear MTIP by setting mtimecmp to max
        CLINT_REG64(CLINT_MTIMECMP) = 0xFFFFFFFFFFFFFFFFULL;
    } else {
        printf("  -> Unknown interrupt! mcause=0x%lx\n", mcause);
    }
}

int test_passed = 0;
int test_failed = 0;

void check(const char *name, int condition) {
    if (condition) {
        printf("%s: PASS\n", name);
        test_passed++;
    } else {
        printf("%s: FAIL\n", name);
        test_failed++;
    }
}

int main(int argc, char **argv) {
    printf("\n=== Interrupt Test ===\n\n");

    // Show initial CSR state
    printf("Initial state:\n");
    printf("  mstatus = 0x%016lx\n", read_csr(mstatus));
    printf("  mie     = 0x%016lx\n", read_csr(mie));
    printf("  mip     = 0x%016lx\n", read_csr(mip));
    printf("  mtvec   = 0x%016lx\n\n", read_csr(mtvec));

    // Disable mtimecmp interrupt first
    CLINT_REG64(CLINT_MTIMECMP) = 0xFFFFFFFFFFFFFFFFULL;
    CLINT_REG32(CLINT_MSIP) = 0;

    //=========================================================================
    // Test 1: Software Interrupt (MSIP)
    //=========================================================================
    printf("Test 1: Software Interrupt (MSIP)\n");

    msip_count = 0;

    // Enable MSIP in mie
    set_csr(mie, MIP_MSIP);
    printf("  mie after enable MSIP = 0x%016lx\n", read_csr(mie));

    // Enable global interrupts
    set_csr(mstatus, MSTATUS_MIE);
    printf("  mstatus after enable MIE = 0x%016lx\n", read_csr(mstatus));

    // Trigger MSIP
    printf("  Triggering MSIP...\n");
    CLINT_REG32(CLINT_MSIP) = 1;

    // Wait for interrupt to be handled
    for (volatile int i = 0; i < 100; i++);

    printf("  msip_count = %d\n", msip_count);

    check("  MSIP interrupt", msip_count == 1);

    // Disable MSIP
    clear_csr(mie, MIP_MSIP);

    //=========================================================================
    // Test 2: Timer Interrupt (MTIP)
    //=========================================================================
    printf("\nTest 2: Timer Interrupt (MTIP)\n");

    mtip_count = 0;

    // Enable MTIP in mie
    set_csr(mie, MIP_MTIP);
    printf("  mie after enable MTIP = 0x%016lx\n", read_csr(mie));

    // Read current mtime
    uint64_t mtime_now = CLINT_REG64(CLINT_MTIME);
    printf("  mtime now = 0x%016lx\n", mtime_now);

    // Set mtimecmp to trigger soon
    uint64_t mtimecmp_val = mtime_now + 10;  // Trigger after 10 ticks
    printf("  Setting mtimecmp = 0x%016lx\n", mtimecmp_val);
    CLINT_REG64(CLINT_MTIMECMP) = mtimecmp_val;

    // Wait for interrupt
    printf("  Waiting for MTIP...\n");
    for (volatile int i = 0; i < 1000; i++);

    printf("  mtip_count = %d\n", mtip_count);

    check("  MTIP interrupt", mtip_count >= 1);

    // Disable interrupts
    clear_csr(mstatus, MSTATUS_MIE);
    clear_csr(mie, MIP_MTIP);

    //=========================================================================
    // Summary
    //=========================================================================
    printf("\n=== Summary ===\n");
    printf("Passed: %02d\n", test_passed);
    printf("Failed: %02d\n", test_failed);

    if (test_failed == 0) {
        printf("\nInterrupt Test: ALL PASSED\n");
        return 0;
    } else {
        printf("\nInterrupt Test: FAILED\n");
        return 1;
    }
}
