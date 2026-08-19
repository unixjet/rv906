//=============================================================================
// clint_test.c - CLINT Register Access Test
//=============================================================================
// Tests CLINT (Core Local Interruptor) register read/write functionality.
// CLINT base address: 0x02000000
//
// Register Map:
//   0x0000 - msip[0]     : Machine Software Interrupt Pending (hart 0)
//   0x4000 - mtimecmp[0] : Machine Timer Compare (hart 0, 64-bit)
//   0xBFF8 - mtime       : Machine Time (64-bit, read-only counter)
//=============================================================================

#include <stdint.h>
#include <stdio.h>

// CLINT base address
#define CLINT_BASE      0x02000000UL

// CLINT register offsets
#define CLINT_MSIP      0x0000  // msip[hart] - 4 bytes per hart
#define CLINT_MTIMECMP  0x4000  // mtimecmp[hart] - 8 bytes per hart
#define CLINT_MTIME     0xBFF8  // mtime - 64-bit

// CLINT register access macros
#define CLINT_REG32(offset) (*(volatile uint32_t *)(CLINT_BASE + (offset)))
#define CLINT_REG64(offset) (*(volatile uint64_t *)(CLINT_BASE + (offset)))

// CSR access
#define read_csr(reg) ({ unsigned long __tmp; \
    asm volatile ("csrr %0, " #reg : "=r"(__tmp)); __tmp; })

// tohost for signaling test result
extern volatile uint64_t tohost;

// Trap handler for unexpected exceptions
void trap_handler(void) {
    tohost = 3;  // Signal failure
    while(1);
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
    printf("\n=== CLINT Test ===\n\n");

    //=========================================================================
    // Test 1: Read mtime (should be incrementing)
    //=========================================================================
    printf("Test 1: mtime read\n");
    uint64_t mtime1 = CLINT_REG64(CLINT_MTIME);
    printf("  mtime[0] = 0x%016lx\n", mtime1);

    // Small delay
    for (volatile int i = 0; i < 100; i++);

    uint64_t mtime2 = CLINT_REG64(CLINT_MTIME);
    printf("  mtime[1] = 0x%016lx\n", mtime2);

    check("  mtime incrementing", mtime2 > mtime1);

    //=========================================================================
    // Test 2: Read/Write mtimecmp
    //=========================================================================
    printf("\nTest 2: mtimecmp read/write\n");

    // Read initial value
    uint64_t mtimecmp_init = CLINT_REG64(CLINT_MTIMECMP);
    printf("  mtimecmp initial = 0x%016lx\n", mtimecmp_init);

    // Write test pattern
    uint64_t test_val = 0xDEADBEEF12345678ULL;
    CLINT_REG64(CLINT_MTIMECMP) = test_val;

    // Read back
    uint64_t mtimecmp_read = CLINT_REG64(CLINT_MTIMECMP);
    printf("  mtimecmp written = 0x%016lx\n", test_val);
    printf("  mtimecmp readback = 0x%016lx\n", mtimecmp_read);

    check("  mtimecmp write/read", mtimecmp_read == test_val);

    // Restore to max value (disable timer interrupt)
    CLINT_REG64(CLINT_MTIMECMP) = 0xFFFFFFFFFFFFFFFFULL;

    //=========================================================================
    // Test 3: Read/Write msip
    //=========================================================================
    printf("\nTest 3: msip read/write\n");

    // Read initial value
    uint32_t msip_init = CLINT_REG32(CLINT_MSIP);
    printf("  msip initial = 0x%08x\n", msip_init);

    // Write 1 to set MSIP
    CLINT_REG32(CLINT_MSIP) = 1;
    uint32_t msip_set = CLINT_REG32(CLINT_MSIP);
    printf("  msip after set = 0x%08x\n", msip_set);

    check("  msip set to 1", (msip_set & 1) == 1);

    // Write 0 to clear MSIP
    CLINT_REG32(CLINT_MSIP) = 0;
    uint32_t msip_clear = CLINT_REG32(CLINT_MSIP);
    printf("  msip after clear = 0x%08x\n", msip_clear);

    check("  msip cleared to 0", (msip_clear & 1) == 0);

    //=========================================================================
    // Summary
    //=========================================================================
    printf("\n=== Summary ===\n");
    printf("Passed: %02d\n", test_passed);
    printf("Failed: %02d\n", test_failed);

    if (test_failed == 0) {
        printf("\nCLINT Test: ALL PASSED\n");
        return 0;
    } else {
        printf("\nCLINT Test: FAILED\n");
        return 1;
    }
}
