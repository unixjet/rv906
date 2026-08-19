#=============================================================================
# entry.S - Unified entry point for rocketM tests
#=============================================================================
# Usage:
#   Default:    Simple mode (no libc, no trap handler)
#   -DWITH_TRAP: Add trap handler support
#   -DWITH_LIBC: Use libc initialization (atexit, __libc_init_array)
#
# TestBench pre-initializes:
#   - PC  = ELF entry point
#   - SP  = _stack_top
#   - A1  = DTB address
# So we skip clearing x2(SP), x10(A0), x11(A1)
#=============================================================================

#if __riscv_xlen == 64
# define STORE    sd
# define LOAD     ld
# define REGBYTES 8
#else
# define STORE    sw
# define LOAD     lw
# define REGBYTES 4
#endif

#define STACK_TOP _stack_top

    .section ".text.init","ax",@progbits
    .globl _start
    .align 2

_start:
    # Set up trap vector FIRST (before anything else)
#ifdef WITH_TRAP
    la t0, trap_vector
#else
    la t0, default_trap
#endif
    csrw mtvec, t0

    #=========================================================================
    # Clear registers (x2/x10/x11 skipped - set by testbench)
    #=========================================================================
    li x1, 0
    # x2 (SP) - set by testbench
    li x3, 0
    li x4, 0
    li x5, 0
    li x6, 0
    li x7, 0
    li x8, 0
    li x9, 0
    # x10 (A0) - used for argc
    # x11 (A1) - set by testbench (DTB)
    li x12, 0
    li x13, 0
    li x14, 0
    li x15, 0
    li x16, 0
    li x17, 0
    li x18, 0
    li x19, 0
    li x20, 0
    li x21, 0
    li x22, 0
    li x23, 0
    li x24, 0
    li x25, 0
    li x26, 0
    li x27, 0
    li x28, 0
    li x29, 0
    li x30, 0
    li x31, 0

    #=========================================================================
    # Set up stack (only if not already set by testbench)
    #=========================================================================
    bnez sp, 1f
    la sp, STACK_TOP
1:

    #=========================================================================
    # Clear BSS section
    #=========================================================================
    la t0, __bss_start
    la t1, __bss_end
2:
    bge t0, t1, 3f
    STORE zero, 0(t0)
    addi t0, t0, REGBYTES
    j 2b
3:

#ifdef WITH_LIBC
    #=========================================================================
    # LIBC mode: With argc/argv and printf support
    #=========================================================================
    # Load argc/argv from stack (set by testbench)
    LOAD s0, 0(sp)      # argc
    LOAD s1, 8(sp)      # argv

    # If argc is 0, set up default
    bnez s0, 4f
    addi sp, sp, -16
    STORE zero, 0(sp)
    li s0, 0
    mv s1, sp
4:
    mv a0, s0
    mv a1, s1
    call main
    j _exit

#else
    #=========================================================================
    # Simple mode: Direct call to main
    #=========================================================================
    li a0, 0
    li a1, 0
    call main
    j _exit
#endif

#ifdef WITH_TRAP
    #=========================================================================
    # Trap Vector (only with WITH_TRAP)
    #=========================================================================
    .align 2
trap_vector:
    # Save caller-saved registers
    addi sp, sp, -128
    STORE ra,  0*REGBYTES(sp)
    STORE t0,  1*REGBYTES(sp)
    STORE t1,  2*REGBYTES(sp)
    STORE t2,  3*REGBYTES(sp)
    STORE t3,  4*REGBYTES(sp)
    STORE t4,  5*REGBYTES(sp)
    STORE t5,  6*REGBYTES(sp)
    STORE t6,  7*REGBYTES(sp)
    STORE a0,  8*REGBYTES(sp)
    STORE a1,  9*REGBYTES(sp)
    STORE a2, 10*REGBYTES(sp)
    STORE a3, 11*REGBYTES(sp)
    STORE a4, 12*REGBYTES(sp)
    STORE a5, 13*REGBYTES(sp)
    STORE a6, 14*REGBYTES(sp)
    STORE a7, 15*REGBYTES(sp)

    # Call C trap handler
    call trap_handler

    # Restore caller-saved registers
    LOAD ra,  0*REGBYTES(sp)
    LOAD t0,  1*REGBYTES(sp)
    LOAD t1,  2*REGBYTES(sp)
    LOAD t2,  3*REGBYTES(sp)
    LOAD t3,  4*REGBYTES(sp)
    LOAD t4,  5*REGBYTES(sp)
    LOAD t5,  6*REGBYTES(sp)
    LOAD t6,  7*REGBYTES(sp)
    LOAD a0,  8*REGBYTES(sp)
    LOAD a1,  9*REGBYTES(sp)
    LOAD a2, 10*REGBYTES(sp)
    LOAD a3, 11*REGBYTES(sp)
    LOAD a4, 12*REGBYTES(sp)
    LOAD a5, 13*REGBYTES(sp)
    LOAD a6, 14*REGBYTES(sp)
    LOAD a7, 15*REGBYTES(sp)
    addi sp, sp, 128

    mret
#endif

    #=========================================================================
    # Exit handler
    #=========================================================================
    .globl _exit
    .align 2
_exit:
    # Disable interrupts first
    csrci mstatus, 0x8
    # a0 = exit status
    # Write to tohost: (exit_code << 1) | 1
    slli a0, a0, 1
    ori a0, a0, 1
    la t0, tohost
    STORE a0, 0(t0)
1:  j 1b

#ifndef WITH_TRAP
    #=========================================================================
    # Default trap handler (spin forever)
    #=========================================================================
    .align 2
default_trap:
    j default_trap
#endif

    #=========================================================================
    # Data section
    #=========================================================================
    .globl dtb
    .globl tohost
    .globl fromhost
    .section .data,"aw"
    .align 3
dtb:
    .skip REGBYTES
tohost:
    .skip 8
fromhost:
    .skip 8

    # BSS markers (defined by linker)
    .globl __bss_start
    .globl __bss_end
