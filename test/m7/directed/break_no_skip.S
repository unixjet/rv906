#=============================================================================
# break_no_skip.S - M7 directed test: execute trigger fires exactly at the
# armed PC, exactly as many times as the PC is reached, and never executes
# the triggered instruction (breakpoint semantics, mcause 3).
#=============================================================================
# Arms one execute trigger (mcontrol, eq match, M-mode, action=breakpoint)
# on the label `trig`, then walks over `trig` 3 times in a loop. Each pass
# must take exactly one breakpoint trap with mcause=3 and mepc == &trig
# (the handler enforces both per trap); the triggered instruction itself
# must NEVER execute (timing-0 execute trigger: trap before execution),
# and after the loop the trap count must equal 3.
#
# FAIL encoding (distinct test numbers for diagnosis):
#   tohost = (N<<1)|1  -> "FAIL. test no. = N"
#     2+x13 = trap count != 3  (2 -> 0 traps, 3 -> 1, 4 -> 2, 6 -> 4, ...)
#     3+x16 = trigger instruction executed (3 -> x16==0 impossible, 4 -> 1, ...)
#     4     = handler: mcause != 3
#     5     = handler: mepc != &trig
#=============================================================================

    .section ".text.init","ax",@progbits
    .globl _start
    .align 3

_start:
    # mtvec first: any early trap must land in the handler
    la   x12, handler
    csrw mtvec, x12

    #-------------------------------------------------------------------------
    # Enable M-mode triggers and arm the execute trigger
    #-------------------------------------------------------------------------
    li   x12, 0x8               # tcontrol.MTE
    csrs tcontrol, x12          # must not trap; enables M-mode triggers

    li   x12, 0x0
    csrw tselect, x12
    li   x12, 0x2000000000000044  # type=2(mcontrol), mmode, execute,
                                  # match=eq, action=0(breakpoint)
    csrw tdata1, x12
    la   x12, trig
    csrw tdata2, x12

    #-------------------------------------------------------------------------
    # Walk over `trig` 3 times
    #-------------------------------------------------------------------------
    li   x13, 0                  # trap count (handler increments)
    li   x14, 3                  # expected trap count
    li   x15, 3                  # loop iteration count
    li   x16, 0                  # executed-by-trigger counter (must stay 0)

loop:
    beq  x0, x0, loopbody        # 32-bit always-taken filler (PC gap)
loopbody:
    nop
    .align 2
trig:
    .word 0x00180813          # addi x16,x16,1 -- FORCED 32-bit (the
                              # assembler compresses addi x16 to C.ADDI,
                              # which would break the handler's mepc+4
                              # skip). Trap-before-execution: it never runs.
    addi x15, x15, -1         # runs after mret (mepc+4 lands here)
    bne  x15, x0, loop

    #-------------------------------------------------------------------------
    # Checks: exactly 3 traps, triggered instruction never executed
    #-------------------------------------------------------------------------
    bne  x13, x14, fail_tc         # trap count != 3
    bne  x16, x0,   fail_exec      # trigger instruction executed

    #-------------------------------------------------------------------------
    # Disarm + PASS
    #-------------------------------------------------------------------------
    li   x12, 0x0
    csrw tcontrol, x12
    csrw tdata1, x12

    li   x10, 0x7FFFF000
    li   x11, 1
    sd   x11, 0(x10)
    j    .

#=============================================================================
# handler - execute trigger (breakpoint, mcause 3). mtvec is BASE-aligned,
# so the label itself must be 4-byte aligned: .align before the label.
#=============================================================================
    .align 2
handler:
    # scratch: x12, x17 (main flow keeps x13=count x14=expected x15=iters x16)
    csrr x12, mcause
    li   x17, 3
    bne  x12, x17, fail_cause      # not a breakpoint (testno 4)

    csrr x12, mepc
    la   x17, trig
    bne  x12, x17, fail_epc        # not at the armed PC (testno 5)

    addi x13, x13, 1             # trap count++

    # skip the (never-executed) 32-bit triggered instruction
    csrr x12, mepc
    addi x12, x12, 4
    csrw mepc, x12
    mret

fail_cause:
    li x10, 0x7FFFF000
    li x11, (4<<1) | 1
    sd x11, 0(x10)
    j fail_cause

fail_epc:
    li x10, 0x7FFFF000
    li x11, (5<<1) | 1
    sd x11, 0(x10)
    j fail_epc

fail_tc:
    # encode the ACTUAL trap count into the test number: testno = 2 + x13
    # (2 -> 0 traps, 3 -> 1, 4 -> 2, 6 -> 4, ...)
    li   x17, 2
    add  x17, x17, x13
    slli x17, x17, 1
    ori  x17, x17, 1
    li   x10, 0x7FFFF000
    sd   x17, 0(x10)
    j    fail_tc

fail_exec:
    # encode the ACTUAL executed-count: testno = 3 + x16
    li   x17, 3
    add  x17, x17, x16
    slli x17, x17, 1
    ori  x17, x17, 1
    li   x10, 0x7FFFF000
    sd   x17, 0(x10)
    j    fail_exec
