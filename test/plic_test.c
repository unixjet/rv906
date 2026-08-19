//=============================================================================
// plic_test.c - PLIC Register Access Test
//=============================================================================
// Tests PLIC (Platform-Level Interrupt Controller) register read/write.
// PLIC base address: 0x0C000000
//
// Note: Since int_src is tied to 0, we can only test register access.
// External interrupt testing would require testbench modifications.
//=============================================================================

#include <stdint.h>
#include <stdio.h>

// PLIC base address
#define PLIC_BASE       0x0C000000UL

// PLIC register offsets
#define PLIC_PRIO(n)    (0x000000 + (n) * 4)     // Priority for source n (n=1-7)
#define PLIC_PENDING    0x001000                  // Pending bits
#define PLIC_ENABLE     0x002000                  // Enable bits context 0
#define PLIC_THRESHOLD  0x200000                  // Threshold context 0
#define PLIC_CLAIM      0x200004                  // Claim/complete context 0

#define PLIC_REG32(offset) (*(volatile uint32_t *)(PLIC_BASE + (offset)))
#define PLIC_REG64(offset) (*(volatile uint64_t *)(PLIC_BASE + (offset)))

// CSR access
#define read_csr(reg) ({ unsigned long __tmp; \
    asm volatile ("csrr %0, " #reg : "=r"(__tmp)); __tmp; })

// mie/mip bits
#define MIP_MEIP        (1UL << 11)  // Machine External Interrupt

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
    printf("\n=== PLIC Test ===\n\n");

    //=========================================================================
    // Test 1: Priority Registers
    //=========================================================================
    printf("Test 1: Priority registers\n");

    // Read initial values
    printf("  Initial priorities:\n");
    for (int i = 1; i <= 7; i++) {
        printf("    prio[%d] = 0x%08x\n", i, PLIC_REG32(PLIC_PRIO(i)));
    }

    // Write test values
    for (int i = 1; i <= 7; i++) {
        PLIC_REG32(PLIC_PRIO(i)) = i;  // Set priority to source number
    }

    // Read back
    printf("  After write:\n");
    int prio_ok = 1;
    for (int i = 1; i <= 7; i++) {
        uint32_t val = PLIC_REG32(PLIC_PRIO(i));
        printf("    prio[%d] = 0x%08x\n", i, val);
        if ((val & 0x7) != (i & 0x7)) prio_ok = 0;  // Only 3 bits used
    }
    check("  Priority read/write", prio_ok);

    //=========================================================================
    // Test 2: Enable Register
    //=========================================================================
    printf("\nTest 2: Enable register\n");

    // Read initial
    uint32_t enable_init = PLIC_REG32(PLIC_ENABLE);
    printf("  Enable initial = 0x%08x\n", enable_init);

    // Write test pattern (enable sources 1,3,5,7)
    PLIC_REG32(PLIC_ENABLE) = 0xAA;  // 10101010
    uint32_t enable_read = PLIC_REG32(PLIC_ENABLE);
    printf("  Enable written = 0xAA\n");
    printf("  Enable readback = 0x%08x\n", enable_read);
    // Bit 0 is always 0 (reserved), so expect 0xAA & ~1 = 0xAA
    check("  Enable read/write", (enable_read & 0xFE) == 0xAA);

    // Disable all
    PLIC_REG32(PLIC_ENABLE) = 0;

    //=========================================================================
    // Test 3: Threshold Register
    //=========================================================================
    printf("\nTest 3: Threshold register\n");

    // Read initial
    uint32_t thresh_init = PLIC_REG32(PLIC_THRESHOLD);
    printf("  Threshold initial = 0x%08x\n", thresh_init);

    // Write test value
    PLIC_REG32(PLIC_THRESHOLD) = 5;
    uint32_t thresh_read = PLIC_REG32(PLIC_THRESHOLD);
    printf("  Threshold written = 5\n");
    printf("  Threshold readback = 0x%08x\n", thresh_read);
    check("  Threshold read/write", (thresh_read & 0x7) == 5);

    // Reset threshold
    PLIC_REG32(PLIC_THRESHOLD) = 0;

    //=========================================================================
    // Test 4: Pending Register (read-only)
    //=========================================================================
    printf("\nTest 4: Pending register\n");

    uint32_t pending = PLIC_REG32(PLIC_PENDING);
    printf("  Pending = 0x%08x\n", pending);
    // Since int_src is tied to 0, pending should be 0
    check("  Pending is zero (no ext int)", pending == 0);

    //=========================================================================
    // Test 5: Claim/Complete Register
    //=========================================================================
    printf("\nTest 5: Claim/Complete register\n");

    // With no pending interrupts, claim should return 0
    uint32_t claim = PLIC_REG32(PLIC_CLAIM);
    printf("  Claim = 0x%08x\n", claim);
    check("  Claim returns 0 (no pending)", claim == 0);

    // Write complete (should be safe even with ID=0)
    PLIC_REG32(PLIC_CLAIM) = 0;
    printf("  Complete write OK\n");

    //=========================================================================
    // Test 6: Check MIP.MEIP
    //=========================================================================
    printf("\nTest 6: MIP.MEIP status\n");

    uint64_t mip = read_csr(mip);
    printf("  mip = 0x%016lx\n", mip);
    check("  MEIP is clear (no ext int)", (mip & MIP_MEIP) == 0);

    //=========================================================================
    // Summary
    //=========================================================================
    printf("\n=== Summary ===\n");
    printf("Passed: %02d\n", test_passed);
    printf("Failed: %02d\n", test_failed);

    if (test_failed == 0) {
        printf("\nPLIC Test: ALL PASSED\n");
        return 0;
    } else {
        printf("\nPLIC Test: FAILED\n");
        return 1;
    }
}
