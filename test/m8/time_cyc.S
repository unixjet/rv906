#=============================================================================
# time_cyc.S - M8 T2 directed test: `time` CSR = per-cycle (mcycle alias)
#
# Design: docs/superpowers/specs/2026-09-21-m8-crosscheck-design.md D-M8-5.
# Gates the re-point  CSR_TIME: mtime -> mcycle_reg  (rtl/CSR.v
# csr_read_mux) by locking the SEMANTICS, not just the value:
#
#   check 1: `time` advances per cycle -- the delta across a known
#            2000-instruction loop is >= 2000 (a clk/100 counter, i.e.
#            the pre-T2 mtime mirror, reads ~40 here and fails).
#   check 2: `time` is not faster than ~3 cycles/instruction (catches a
#            multiplied/aliased counter; the measured steady state is
#            2 cycles/instr = 4 cyc/iter on this core, band holds to 3x).
#   check 3: `mtime` (MMIO 0x0200BFF8) is NOT per-cycle -- dm*50 <= dt
#            (it ticks at clk/100, so dm ~= dt/100; a per-cycle mtime
#            would read dm ~= dt and fail).
#   check 4: `mtime` is alive -- dm*200 >= dt - 400 (equivalently
#            dm >= dt/200 - 2; a stopped mtime fails; the 2x margin on
#            both ratio bounds absorbs rtc_tick phase and MMIO latency).
#
# Environment: a standalone ELF has no crt0, and out of reset EVERY
# MHCR enable is 0 (caches off + predictor off, where this bne
# self-loop costs ~12 cyc/instr and checks 1/2 cannot discriminate).
# The test therefore self-boots the M8 donor mhcr poke (crt0_m8.s:156-157,
# `csrs 0x7c1, 0x7f` = ie/de/wa/rse/bpe/btbe) at _start, so the
# back-edge converges after a couple of mispredicts and the loop runs
# a steady ~4 cyc/iter, as checks 1/2 assume.
#
# The exact steady-state delta is deterministic for a given binary: the
# measured values are printed in HEX to the UART (THR 0x10000000 ->
# testbench stdout) so the bands can be pinned tight (delta == measured
# +/- 2) on a later revision and re-verified.
#
# FAIL encoding: tohost = (check#<<1)|1  (PASS = tohost 1).  NB the
# testbench prints "test no." = tohost>>1, and the trap handler uses the
# SAME (cause<<1)|1 scheme -- a trap with mcause 1 reads as "test no. 1",
# so any unexpected trap here is a test bug by construction.
#
# Build (ad-hoc per D-M8-5; T3 folds this into the case set):
#   /opt/xpack-riscv-none-elf-gcc-15.2.0-1/bin/riscv-none-elf-gcc \
#       -march=rv64imac_zicsr -mabi=lp64 -O2 -nostdlib \
#       -T test/m6/common.ld -o test/m8/build/time_cyc.elf test/m8/time_cyc.S
#   timeout 300 bin/verisim/testbench --print-result \
#       test/m8/build/time_cyc.elf
#=============================================================================

    .section ".text.init","ax",@progbits
    .globl _start
    .align 3

    .equ TOHOST, 0x7FFFF000
    .equ MTIME,  0x0200BFF8
    .equ THR,    0x10000000
    .equ N_ITER, 1000

_start:
    #------------------------------------------------------------------
    # M8 env (donor boot): enable caches + predictors exactly as
    # crt0_m8.s:155-157 (`csrs 0x7c1, 0x7f` = ie/de/wa/rse/bpe/btbe).
    # See header: out of reset every MHCR enable is 0, which makes the
    # loop ~12 cyc/instr and the bands below meaningless.
    #------------------------------------------------------------------
    li   x3, 0x7f
    csrs 0x7c1, x3                     # mhcr: donor M8 boot (crt0_m8.s:156)

    la   x12, handler
    csrw mtvec, x12

    #------------------------------------------------------------------
    # mtime0 (MMIO), then the measured window: time0, loop, time1, mtime1
    #------------------------------------------------------------------
    la   x16, MTIME
    ld   x18, 0(x16)                  # x18 = mtime0

    csrr x12, 0xC01                   # x12 = time0

    li   x14, 0
    li   x15, N_ITER
loop:
    addi x14, x14, 1
    bne  x14, x15, loop

    csrr x13, 0xC01                   # x13 = time1
    ld   x19, 0(x16)                  # x19 = mtime1

    sub  x12, x13, x12                # x12 = dt = time1 - time0
    sub  x17, x19, x18                # x17 = dm = mtime1 - mtime0

    #------------------------------------------------------------------
    # check 1: dt >= 2000  (per-cycle; a clk/100 counter reads ~40)
    #------------------------------------------------------------------
    li   x15, N_ITER * 2
    blt  x12, x15, fail1
    #------------------------------------------------------------------
    # check 2: dt <= 6000  (not faster than 3 cyc/instr)
    #------------------------------------------------------------------
    li   x15, N_ITER * 6
    bgt  x12, x15, fail2
    #------------------------------------------------------------------
    # check 3: dm*50 <= dt   (mtime is clk/100, not per-cycle)
    # (dm*50 vs dt instead of dm vs dt/50: keeps the constants 12-bit
    #  and avoids the mulh-64-bit-constant trap -- mulh shifts by 64,
    #  not 32, so a "dt/50" mulh constant computes ~0, not dt/50.)
    #------------------------------------------------------------------
    li   x15, 50
    mul  x16, x17, x15                # x16 = dm*50
    bgt  x16, x12, fail3
    #------------------------------------------------------------------
    # check 4: dm*200 >= dt - 400   (mtime is alive, ~= clk/100)
    #------------------------------------------------------------------
    li   x15, 200
    mul  x16, x17, x15                # x16 = dm*200
    addi x15, x12, -400               # x15 = dt - 400
    blt  x16, x15, fail4

    #------------------------------------------------------------------
    # PASS: print "dt=0x.. dm=0x..\n" to the UART (testbench stdout),
    # then tohost=1
    #------------------------------------------------------------------
    mv   x21, x17                     # x21 = dm (x17 gets clobbered below)
    li   x15, THR
    li   x16, 'd'
    sb   x16, 0(x15)
    li   x16, 't'
    sb   x16, 0(x15)
    li   x16, '='
    sb   x16, 0(x15)
    mv   x16, x12                     # dt
    j    emit_hex
    .align 3
emit_hex:                             # x16 = value (low 32 bits), x15 = THR
    li   x17, 8
eh:
    srli x18, x16, 28
    andi x18, x18, 0xF
    li   x19, 0xA
    blt  x18, x19, ehd
    addi x18, x18, 39                 # 'a' - 0x30 - 0xA = 0x61-0x30-0xA = 39
ehd:
    addi x18, x18, '0'
    sb   x18, 0(x15)
    slli x16, x16, 4
    addi x17, x17, -1
    bnez x17, eh
    li   x16, ' '
    sb   x16, 0(x15)
    li   x16, 'd'
    sb   x16, 0(x15)
    li   x16, 'm'
    sb   x16, 0(x15)
    li   x16, '='
    sb   x16, 0(x15)
    mv   x16, x21                     # dm
    j    emit_hex2
    .align 3
emit_hex2:                            # x16 = value, x15 = THR
    li   x18, 8
eh2:
    srli x19, x16, 28
    andi x19, x19, 0xF
    li   x20, 0xA
    blt  x19, x20, eh2d
    addi x19, x19, 39
eh2d:
    addi x19, x19, '0'
    sb   x19, 0(x15)
    slli x16, x16, 4
    addi x18, x18, -1
    bnez x18, eh2
    li   x16, '\n'
    sb   x16, 0(x15)

    li   x10, 1                       # tohost = 1 (PASS)
    la   x11, TOHOST
    sd   x10, 0(x11)
1:   j 1b

fail1:
    li   x10, (1<<1)|1
    j    finish
fail2:
    li   x10, (2<<1)|1
    j    finish
fail3:
    li   x10, (3<<1)|1
    j    finish
fail4:
    li   x10, (4<<1)|1
    j    finish
finish:
    li   x11, TOHOST
    sd   x10, 0(x11)
    j    .

    .section ".tohost","aw",@progbits
    .quad 0
    .align 3

#=============================================================================
# handler - any trap here is a test bug: tohost = (mcause<<1)|1
#=============================================================================
    .section ".text.init","ax",@progbits
handler:
    csrr x10, 0x342                   # mcause
    csrr x11, 0x341                   # mepc (unread: no tval to report)
    slli x10, x10, 1
    ori  x10, x10, 1
    li   x11, TOHOST
    sd   x10, 0(x11)
    j    .
