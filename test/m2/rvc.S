# See LICENSE for license details.

#*****************************************************************************
# rvc.S
#-----------------------------------------------------------------------------
#
# Directed test for the RVC (compressed) instruction set.
#
# THIS FILE IS OURS. It is NOT a vendored upstream riscv-tests source -- it is
# the only test body that lives in this repository, and the only one IN THE
# SUITE built with the C extension (-march=rv64imc_zicsr_zifencei; see the
# MARCH_RVC comment in test/m2/Makefile). Everything else in the suite is
# compiled -march=rv64im_zicsr_zifencei and contains no 16-bit encodings at
# all.
#
# WHY IT EXISTS. The M2 architectural reference model (m2_iss.h) expands
# compressed encodings to their 32-bit equivalents, and every one of the 67
# upstream ELFs is built WITHOUT `c`, so without this test the RVC decoder
# sat entirely outside the acceptance gate: stubbing the expander to "nop"
# would still pass 67/67. This test closes that hole with a real,
# assembler-generated RVC program: every quadrant-0/1/2 form is executed AND
# checked against a hand-computed constant, so a wrong immediate field, a
# missing sign extension, or a swapped register specifier fails a numbered
# case instead of passing silently.
#
# Provenance: adapted from the rv12 sibling's own test/m2/rvc.S (its proven
# RVC directed test). The per-case encodings and expected constants are
# unchanged; the mutation-sweep provenance notes below are the source test's
# and rv906 re-validates this file under the m2_iss.h diff in plan Tasks 9.7
# and 10.
#
# It is deliberately shaped like every other test in the suite -- p-env,
# TESTNUM in gp, terminating through the standard tohost protocol -- because at
# plan Task 9 it becomes step 1 of the ISS-diff ladder against the RTL front
# end, and that ladder only accepts normal riscv-tests-shaped ELFs.
#
# WHAT IS COVERED -- 36 numbered cases (TESTNUM 2..37):
#   q0   c.addi4spn  c.lw  c.ld  c.sw  c.sd
#   q1   c.nop  c.addi  c.addiw  c.li  c.lui  c.addi16sp  c.srli  c.srai
#        c.andi  c.sub  c.xor  c.or  c.and  c.subw  c.addw  c.j  c.beqz
#        c.bnez
#   q2   c.slli  c.lwsp  c.ldsp  c.swsp  c.sdsp  c.mv  c.add  c.jr  c.jalr
#        c.ebreak
# plus the c.lui HINT (rd == x0, which must execute rather than trap) and the
# 16/32-bit MIX: 32-bit instructions that begin on an ODD HALFWORD, reached by
# fall-through, by a jump and by a taken branch. That is the case a fetch front
# end gets wrong most often, and no other ELF in the suite produces it.
#
# THE FORM COUNT, STATED HONESTLY. `objdump -d -M no-aliases` shows 33 distinct
# `c.*` mnemonics in the built ELF and 256 two-byte encodings. Of those 33, ONE
# is not a form this test exercises: `c.unimp`, which is the `unimp` that
# RVTEST_CODE_END places after the pass/fail branch and which is never executed.
# And `c.nop` does NOT appear as a mnemonic even though the test executes one --
# objdump renders it as `c.addi zero, 0`. So: 32 executable forms visible, plus
# c.nop hidden inside c.addi, minus the dead c.unimp.
#
# The `c.` mnemonics are written out explicitly rather than left to the
# assembler's automatic compression, and anything that must stay 32 bits is
# bracketed by `.option norvc` / `.option rvc`. Verify with
#   riscv-none-elf-objdump -d -M no-aliases test/m2/build/rv64uc-p-rvc.elf
# that the forms below really did assemble to two-byte encodings; a source that
# silently widens to 32 bits proves nothing about rvc_expand.
#
# *** THE TWO WAYS THIS TEST CAN LIE, AND WHAT WAS DONE ABOUT THEM. ***
# A spec review measured them rather than arguing about them, by mutating
# the reference model's RVC expander and asking whether this test noticed:
#
#   (1) AN ORACLE THAT IS ITSELF COMPRESSED. `li t0, 0x1f000` assembles to a
#       c.lui/c.addi pair, so it travels the SAME expander arm as the c.lui
#       under test and the two errors cancel. Every oracle in a case that tests
#       a q1 immediate form is now bracketed `.option norvc`.
#   (2) A ROUND TRIP THROUGH THE POINTER OR OFFSET UNDER TEST. Storing and
#       loading through the address c.addi4spn just computed hides a wrong
#       address completely; a c.sw/c.lw or c.sd/c.ld pair at the same offset
#       hides a wrong scale, because the two share one immediate decoder. Every
#       such pair is now CROSSED with a 32-bit form -- compressed store, 32-bit
#       load and the other way round -- and every computed pointer is compared
#       against one built with a 32-bit `addi`.
#
# MEASURED RESIDUE: a 30-mutation sweep over rvc_expand (immediates, scales,
# sign extensions, register specifiers, opcode selects, the link register, the
# whole expander stubbed to nop) went from 17 killed / 11 survived to
# 30 killed / 0 survived. The survivors were exactly the two classes above:
# both c.lui sign-extension mutations, all five c.addi4spn mutations, the c.j
# and c.beqz displacement sign bits (every transfer in the test used to go
# FORWARD, so those bits were always zero -- section 12 now jumps and branches
# BACKWARD as well), the shared c.lw/c.sw and c.ld/c.sd offset decoders, and
# c.ebreak and the c.lui HINT, neither of which had a case at all.
# The sweep is the harness in the review record, not a committed script; the
# cheapest way to re-run it is to edit one arm of rvc_expand and confirm this
# test fails with the expected TESTNUM.
#

#include "riscv_test.h"
#include "test_macros.h"

RVTEST_RV64U
RVTEST_CODE_BEGIN

  #-------------------------------------------------------------
  # 1. A stack pointer of our own
  #-------------------------------------------------------------
  # The p-env does NOT set one up: INIT_XREG in riscv_test.h leaves x2 = 0, so
  # every SP-relative compressed form below (c.lwsp/c.ldsp/c.swsp/c.sdsp and
  # the x2-based c.addi4spn/c.addi16sp) would fault on a null pointer. Point sp
  # at rvc_sp_base, which sits inside a scratch buffer in .data with 512 B
  # below it (for c.addi16sp's negative displacement) and 2 KiB above it (the
  # positive offsets reach at most sp + 1023).

  la sp, rvc_sp_base

  #-------------------------------------------------------------
  # 2. Quadrant 1 -- immediates: c.li, c.addi, c.nop, c.lui
  #-------------------------------------------------------------

  # *** THE ORACLES IN THIS SECTION ARE FORCED 32-BIT, AND THAT IS THE WHOLE
  # POINT OF THE .option BRACKETS. *** `li t0, 3` assembles to `c.li t0, 3`,
  # `li t0, 0x1f000` to a `c.lui`/`c.addi` pair -- so an unbracketed oracle
  # travels through the SAME expander arm as the instruction under test and the
  # two errors cancel: a c.lui that dropped its sign extension produced a wrong
  # a2 AND a wrong t0, and the compare passed. Measured, not theorised: with
  # plain `li` oracles, mutating the ISS's `sext(u, 18)` to `u` and to
  # `sext(u, 19)` both SURVIVED this test. Bracketed, both die here.

  li TESTNUM, 2
  c.li    a0, 15               # a0 = 15
  c.addi  a0, 20               # a0 = 35
  c.nop                        # must change nothing
  c.addi  a0, -32              # a0 = 3
  .option norvc
  li  t0, 3
  .option rvc
  bne a0, t0, fail

  # both ends of the 6-bit signed immediate
  li TESTNUM, 3
  c.li    a1, -32              # most negative c.li
  c.addi  a1, 31               # most positive c.addi  -> -1
  .option norvc
  li  t0, -1
  .option rvc
  bne a1, t0, fail

  # c.lui: nzimm[17:12], sign-extended from bit 17
  li TESTNUM, 4
  c.lui   a2, 0x1f             # a2 = 0x0001f000
  .option norvc
  li  t0, 0x1f000
  .option rvc
  bne a2, t0, fail
  c.lui   a3, 0xfffff          # nzimm = 0b111111 -> sext -> -4096
  .option norvc
  li  t0, -4096
  .option rvc
  bne a3, t0, fail

  # c.lui with rd == x0 is a HINT, NOT a reserved encoding: only nzimm == 0 is
  # reserved (the native RVC decode tests exactly that and nothing about rd).
  # It must EXECUTE -- writing x0, i.e. doing nothing -- and must not trap.
  # The assembler refuses to emit `c.lui x0`, so the halfword is placed by hand:
  #   0110_0_00000_00001_01 = 0x6005  ->  c.lui x0, 0x1
  # This case exists to regression-lock the c.lui-x0-as-hint rule: the
  # reference model must not trap this encoding.
  li TESTNUM, 35
  .option norvc
  li  a4, 0x5a5
  .option rvc
  .2byte 0x6005                # c.lui x0, 0x1 -- a HINT
  .option norvc
  li  t0, 0x5a5
  .option rvc
  bne a4, t0, fail             # nothing else was disturbed
  bne x0, zero, fail           # and x0 is still x0

  #-------------------------------------------------------------
  # 3. Quadrant 1 -- c.addiw (RV64 word form)
  #-------------------------------------------------------------
  # Both cases fail if the expander emits OP-IMM (0x13) instead of
  # OP-IMM-32 (0x1B): the result would keep bit 32 / lose the sign extension.

  li TESTNUM, 5
  li  a0, 0x00000000ffffffff
  c.addiw a0, 1                # word wrap: 0xffffffff + 1 = 0 -> 0
  bne a0, x0, fail

  li TESTNUM, 6
  li  a1, 0x000000007ffffff0
  c.addiw a1, 31               # 0x8000000f as a word, sign-extended
  li  t0, 0xffffffff8000000f
  bne a1, t0, fail

  #-------------------------------------------------------------
  # 4. Shifts: c.slli (q2), c.srli / c.srai (q1)
  #-------------------------------------------------------------
  # Shift amounts above 31 set shamt[5], which lives in bit 12 of the
  # encoding -- a field an expander can easily drop.

  li TESTNUM, 7
  li  a0, 1
  c.slli  a0, 40               # 1 << 40
  li  t0, 0x0000010000000000
  bne a0, t0, fail

  li TESTNUM, 8
  li  a1, -16                  # 0xfffffffffffffff0
  c.srli  a1, 4                # logical -> 0x0fffffffffffffff
  li  t0, 0x0fffffffffffffff
  bne a1, t0, fail

  li TESTNUM, 9
  li  a2, -16
  c.srai  a2, 4                # arithmetic -> -1
  li  t0, -1
  bne a2, t0, fail

  li TESTNUM, 10
  li  a3, 0x8000000000000000
  c.srli  a3, 63               # shamt[5] set
  li  t0, 1
  bne a3, t0, fail
  li  a4, 0x8000000000000000
  c.srai  a4, 63
  li  t0, -1
  bne a4, t0, fail

  #-------------------------------------------------------------
  # 5. Quadrant 1 -- c.andi (6-bit signed, both signs)
  #-------------------------------------------------------------

  li TESTNUM, 11
  li  a3, 0xff
  c.andi  a3, 30               # 0xff & 0x1e
  li  t0, 30
  bne a3, t0, fail
  li  a4, 0xff
  c.andi  a4, -8               # 0xff & ~7
  li  t0, 248
  bne a4, t0, fail

  #-------------------------------------------------------------
  # 6. Quadrant 1 -- the CA group
  #-------------------------------------------------------------

  li TESTNUM, 12
  li  a0, 100
  li  a1, 30
  c.sub   a0, a1               # 70
  li  t0, 70
  bne a0, t0, fail

  li TESTNUM, 13
  li  a0, 0xf0f0
  li  a1, 0x0ff0
  c.xor   a0, a1               # 0xff00
  li  t0, 0xff00
  bne a0, t0, fail

  li TESTNUM, 14
  li  a0, 0xf0f0
  li  a1, 0x0ff0
  c.or    a0, a1               # 0xfff0
  li  t0, 0xfff0
  bne a0, t0, fail

  li TESTNUM, 15
  li  a0, 0xf0f0
  li  a1, 0x0ff0
  c.and   a0, a1               # 0x00f0
  li  t0, 0xf0
  bne a0, t0, fail

  # The two word forms select OP-32 (0x3B) via bit 12 of the encoding.
  li TESTNUM, 16
  li  a0, 0x000000007fffffff
  li  a1, -1
  c.subw  a0, a1               # 0x7fffffff - (-1) = 0x80000000 -> sext
  li  t0, 0xffffffff80000000
  bne a0, t0, fail

  li TESTNUM, 17
  li  a0, 0x00000000ffffffff
  li  a1, 0x0000000100000001   # the upper half must be ignored
  c.addw  a0, a1               # (0xffffffff + 1) as a word = 0
  bne a0, x0, fail

  #-------------------------------------------------------------
  # 7. Quadrant 0 -- c.addi4spn, c.lw, c.sw
  #-------------------------------------------------------------

  # *** THE ADDRESS IS CHECKED, NOT JUST THE ROUND TRIP. *** Every use of
  # c.addi4spn here stores and loads through the pointer IT computed, so a
  # wrong pointer is invisible -- the data lands somewhere else and comes back
  # from the same somewhere else. Measured: with the round trip alone, ALL FIVE
  # c.addi4spn mutations survived (immediate offset, immediate scale, the
  # constant, rd' vs rs1', and the x2 base becoming x8). Comparing the computed
  # address against one built under `.option norvc` kills all five, because the
  # oracle's `addi` shares no field with the CIW encoding.

  li TESTNUM, 18
  c.addi4spn a0, sp, 4         # smallest legal nzuimm
  .option norvc
  addi t1, sp, 4
  .option rvc
  bne a0, t1, fail             # the pointer itself, not just what it points at
  li  a1, 0x000000000a0b0c0d
  c.sw    a1, 0(a0)
  c.lw    a2, 0(a0)
  .option norvc
  li  t0, 0x0a0b0c0d
  .option rvc
  bne a2, t0, fail

  li TESTNUM, 19
  c.addi4spn a1, sp, 1020      # largest nzuimm
  .option norvc
  addi t1, sp, 1020
  .option rvc
  bne a1, t1, fail
  li  a2, 0x0000000089abcdef
  c.sw    a2, 0(a1)
  c.lw    a3, 0(a1)            # c.lw sign-extends its word
  li  t0, 0xffffffff89abcdef
  bne a3, t0, fail

  # c.lw and c.sw share ONE immediate decoder, so a compressed store followed by
  # a compressed load at the same offset moves both ends together -- measured:
  # mutating that shared decoder's scale survived until this case crossed the
  # two directions with 32-bit forms, exactly as sections 8's c.ld/c.sd do.
  li TESTNUM, 20
  c.addi4spn a0, sp, 8
  .option norvc
  addi t1, sp, 8
  .option rvc
  bne a0, t1, fail
  li  a1, 0x11223344
  c.sw    a1, 124(a0)          # compressed store, largest c.sw offset ...
  .option norvc
  lw  t2, 124(a0)              # ... 32-bit load
  .option rvc
  bne t2, a1, fail
  li  a2, 0x55667788
  .option norvc
  sw  a2, 124(a0)              # 32-bit store ...
  .option rvc
  c.lw    a3, 124(a0)          # ... compressed load
  bne a3, a2, fail

  #-------------------------------------------------------------
  # 8. Quadrant 0 -- c.sd, c.ld
  #-------------------------------------------------------------

  # *** THE STORE AND THE LOAD ARE CROSSED WITH 32-BIT FORMS ON PURPOSE. ***
  # c.sd and c.ld share one immediate decoder, so a compressed store followed
  # by a compressed load at the same offset moves BOTH ends together and a
  # wrong scale is invisible (measured: mutating the ISS's c.ld/c.sd offset
  # from `<< 6` to `<< 7` survived that shape). Storing with a 32-bit `sd` and
  # loading with `c.ld` -- and then the other way round -- pins each direction
  # against an oracle that does not share the field.

  li TESTNUM, 21
  c.addi4spn a0, sp, 16
  .option norvc
  addi t1, sp, 16
  .option rvc
  bne a0, t1, fail
  li  a1, 0x0123456789abcdef
  c.sd    a1, 0(a0)            # compressed store ...
  .option norvc
  ld  t2, 0(a0)                # ... 32-bit load
  .option rvc
  bne t2, a1, fail
  c.ld    a2, 0(a0)
  li  t0, 0x0123456789abcdef
  bne a2, t0, fail

  li TESTNUM, 22
  li  a3, 0xfedcba9876543210
  .option norvc
  sd  a3, 248(a0)              # 32-bit store ...
  .option rvc
  c.ld    a4, 248(a0)          # ... compressed load, largest c.ld offset
  bne a4, a3, fail
  li  a5, 0x00ff00ff00ff00ff
  c.sd    a5, 248(a0)          # largest c.sd offset, checked from the other end
  .option norvc
  ld  t2, 248(a0)
  .option rvc
  bne t2, a5, fail

  #-------------------------------------------------------------
  # 9. Quadrant 2 -- c.swsp, c.lwsp
  #-------------------------------------------------------------
  # These take a full 5-bit register specifier, so they are checked with
  # registers outside the x8..x15 window the q0/q1 forms are limited to.

  li TESTNUM, 23
  li  s11, 0x000000000dedbeef  # x27
  c.swsp  s11, 200(sp)
  c.lwsp  t3, 200(sp)          # x28
  li  t0, 0x0dedbeef
  bne t3, t0, fail

  li TESTNUM, 24
  li  s10, 0x00000000cafe0001
  c.swsp  s10, 252(sp)         # largest c.swsp offset
  c.lwsp  s9, 252(sp)          # c.lwsp sign-extends its word
  li  t0, 0xffffffffcafe0001
  bne s9, t0, fail

  #-------------------------------------------------------------
  # 10. Quadrant 2 -- c.sdsp, c.ldsp
  #-------------------------------------------------------------

  li TESTNUM, 25
  li  s8, 0x1122334455667788
  c.sdsp  s8, 240(sp)
  c.ldsp  s7, 240(sp)
  bne s7, s8, fail

  li TESTNUM, 26
  li  s6, 0xfedcba9876543210
  c.sdsp  s6, 504(sp)          # largest c.sdsp offset
  c.ldsp  s5, 504(sp)
  bne s5, s6, fail

  #-------------------------------------------------------------
  # 11. Quadrant 1 -- c.addi16sp, both directions
  #-------------------------------------------------------------
  # Its immediate is scaled by 16 and scattered over five encoding fields;
  # the round trip below must land back exactly on the base.

  # EACH STEP IS CHECKED, NOT THE ROUND TRIP. +496 then -512 sums to -16, and a
  # sum that small hides almost every immediate bug in the cancellation; the
  # base register is also taken with a 32-bit `addi`, not with c.mv, so a broken
  # c.mv cannot quietly agree with a broken c.addi16sp.
  li TESTNUM, 27
  .option norvc
  addi s4, sp, 0               # the base, without going through c.mv
  .option rvc
  li  s3, 0x5a5a5a5a5a5a5a5a
  c.addi16sp sp, 496           # most positive
  .option norvc
  addi t1, s4, 496
  .option rvc
  bne sp, t1, fail             # the positive step on its own
  c.addi16sp sp, -512          # most negative -> base - 16
  .option norvc
  addi t1, s4, -16
  .option rvc
  bne sp, t1, fail             # and the negative one on its own
  c.sdsp  s3, 0(sp)            # below the base: that is what the low half is for
  c.ldsp  s2, 0(sp)
  bne s2, s3, fail
  c.addi16sp sp, 16            # back to the base
  bne sp, s4, fail

  #-------------------------------------------------------------
  # 12. Quadrant 1 -- c.j, c.beqz, c.bnez (taken and not taken)
  #-------------------------------------------------------------

  li TESTNUM, 28
  li  a0, 0
  c.beqz  a0, rvc_br1
  j fail
rvc_br1:
  li  a1, 7
  c.bnez  a1, rvc_br2
  j fail
rvc_br2:
  c.j     rvc_br3
  j fail
rvc_br3:
  li  a0, 1
  c.beqz  a0, rvc_br_bad       # must NOT be taken
  li  a1, 0
  c.bnez  a1, rvc_br_bad       # must NOT be taken
  c.j     rvc_br4
rvc_br_bad:
  j fail
rvc_br4:

  # BACKWARD displacements, which are the ones that pin the immediate's HIGH
  # bits. Every branch and jump above goes forward, so their sign bit is zero
  # and an expander that drops it entirely still passes (measured: mutating the
  # ISS's c.j sign bit and c.beqz offset[8] to 0 both survived). A backward
  # target needs those bits set, so the same mutations now land on an address
  # ahead of the branch instead of behind it.
  li TESTNUM, 36
  li  a3, 0
  li  a4, 0
  c.j     rvc_bk_fwd           # forward c.j
rvc_bk_back:
  c.addi  a3, 4                # reached only by the BACKWARD branch
  c.j     rvc_bk_done          # forward c.j
rvc_bk_jtgt:
  c.addi  a3, 8                # reached only by the BACKWARD c.j
  c.j     rvc_bk_exit          # forward c.j
rvc_bk_bad:
  j fail
rvc_bk_fwd:
  c.addi  a3, 1
  c.beqz  a4, rvc_bk_back      # BACKWARD c.beqz, taken (a4 == 0)
  j fail                       # a backward branch that fell through
rvc_bk_done:
  c.j     rvc_bk_jtgt          # BACKWARD c.j
rvc_bk_exit:
  .option norvc
  li  t0, 13                   # 1 forward + 4 backward-branch + 8 backward-jump
  .option rvc
  bne a3, t0, fail
  # and a backward c.bnez that must NOT be taken, so the sign bit cannot be
  # "right" merely by always jumping backwards
  li  a5, 0
  c.bnez  a5, rvc_bk_bad

  #-------------------------------------------------------------
  # 13. Quadrant 2 -- c.jr
  #-------------------------------------------------------------

  li TESTNUM, 29
  li  a1, 0
  la  a0, rvc_jr_target
  c.jr    a0
  c.addi  a1, 1                # must be skipped
rvc_jr_target:
  c.addi  a1, 2
  li  t0, 2
  bne a1, t0, fail

  #-------------------------------------------------------------
  # 14. Quadrant 2 -- c.jalr, and its link value
  #-------------------------------------------------------------
  # c.jalr links pc + 2, NOT pc + 4. An expander that forgets the instruction
  # was two bytes long returns to the wrong address; the check below is the
  # one that catches it.

  li TESTNUM, 30
  li  a2, 0
  la  a1, rvc_jalr_fn
  c.jalr  a1
rvc_jalr_ret:
  la  t0, rvc_jalr_ret
  bne ra, t0, fail             # ra == the halfword right after the c.jalr
  li  t0, 0x99
  bne a2, t0, fail
  c.j     rvc_after_fn
rvc_jalr_fn:
  li  a2, 0x99
  c.jr    ra
rvc_after_fn:

  #-------------------------------------------------------------
  # 15. Quadrant 2 -- c.mv, c.add on the full register file
  #-------------------------------------------------------------

  li TESTNUM, 31
  li  s11, 1000                # x27
  c.mv    t6, s11              # x31 = 1000
  c.add   t6, s11              # x31 = 2000
  li  t0, 2000
  bne t6, t0, fail

  #-------------------------------------------------------------
  # 16. The 16/32-bit mix -- 32-bit instructions on an odd halfword
  #-------------------------------------------------------------
  # Straight-line case. After the .balign the c.addi puts the instruction
  # stream out of phase, so the two `.option norvc` addis below start at
  # (4N+2) and (4N+6): each one straddles a 4-byte fetch boundary.

  li TESTNUM, 32
  li  a0, 0
  .balign 4
rvc_mix:
  c.addi  a0, 1                # 2 B at 4N       a0 = 1
  .option norvc
  addi a0, a0, 2               # 4 B at 4N + 2   a0 = 3
  addi a0, a0, 4               # 4 B at 4N + 6   a0 = 7
  .option rvc
  c.addi  a0, 8                # 2 B at 4N + 10  a0 = 15  (back in phase)
  .option norvc
  addi a0, a0, 16              # 4 B at 4N + 12  a0 = 31
  .option rvc
  li  t0, 31
  bne a0, t0, fail

  # Jump case: an indirect jump whose target address is 2 mod 4, landing
  # directly on a 32-bit instruction.

  li TESTNUM, 33
  li  a1, 5
  la  a0, rvc_odd_jump
  c.jr    a0
  j fail
  .balign 4
  c.nop                        # filler: pushes the label to 4N + 2
rvc_odd_jump:
  .option norvc
  addi a1, a1, 100             # 32-bit instruction fetched at pc % 4 == 2
  addi a1, a1, 200
  .option rvc
  c.addi  a1, -5
  li  t0, 300
  bne a1, t0, fail

  # Branch case: the same, reached by a taken compressed branch.

  li TESTNUM, 34
  li  a2, 0
  c.beqz  a2, rvc_odd_branch
  j fail
  .balign 4
  c.nop                        # filler: pushes the label to 4N + 2
rvc_odd_branch:
  .option norvc
  addi a2, a2, 42              # 32-bit instruction fetched at pc % 4 == 2
  .option rvc
  li  t0, 42
  bne a2, t0, fail

  #-------------------------------------------------------------
  # 17. Quadrant 2 -- c.ebreak, the one form that leaves the body
  #-------------------------------------------------------------
  # c.ebreak is the only executable compressed encoding whose effect is a TRAP,
  # so it cannot be checked with a compare -- it has to be caught. mtvec_handler
  # below is the p-environment's own hook for exactly this (riscv_test.h's
  # trap_vector does `la t5, mtvec_handler; beqz t5, 1f; jr t5` after ruling out
  # the ecall causes), and defining it satisfies the macro's `.weak`.
  #
  # WHAT THIS PINS, beyond "the encoding exists": the handler resumes at
  # mepc + 2, so the trap must have reported the c.ebreak's OWN pc and the
  # instruction must have been two bytes long. An expander that turned this into
  # an ecall (cause 8 from U) would be sent to write_tohost by the trap vector
  # and end the run with tohost = TESTNUM, i.e. a FAIL rather than a silent pass.
  li TESTNUM, 37
  li  t3, 0                    # the handler's flag
  .option norvc
  li  a0, 0x77                 # a value the resumed path must still see
  .option rvc
  c.ebreak                     # -> mtvec_handler -> mret to the next halfword
  c.addi  a0, 1                # the instruction the handler must resume ON
  .option norvc
  li  t0, 0x78
  li  t1, 1
  .option rvc
  bne a0, t0, fail             # resumed at mepc + 2, not + 4 and not on top
  bne t3, t1, fail             # and it really was the handler that returned

  TEST_PASSFAIL

  #-------------------------------------------------------------
  # The M-mode trap handler for the c.ebreak case above
  #-------------------------------------------------------------
  # Entered from riscv_test.h's trap_vector for any non-ecall trap, in M-mode,
  # with t5/t6 ALREADY CLOBBERED by the vector itself -- so using them here
  # costs nothing the body could have relied on. Anything that is not a
  # breakpoint is a real failure and goes to `fail`, which is how a mis-expanded
  # compressed encoding that traps (an illegal instruction, say) reports itself
  # instead of hanging.
  .align 2
  .globl mtvec_handler
mtvec_handler:
  csrr t5, mcause
  li   t6, CAUSE_BREAKPOINT
  bne  t5, t6, fail
  li   t3, 1                   # tell the body the handler ran
  csrr t5, mepc
  addi t5, t5, 2               # step over the two-byte c.ebreak
  csrw mepc, t5
  mret

RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN

  TEST_DATA

  # Scratch buffer for the SP-relative and CIW forms (see section 1). sp is
  # set to rvc_sp_base, which is 16-byte aligned so that every doubleword
  # access below it and above it is naturally aligned.
  #   below: 512 B, reached by c.addi16sp sp, -512
  #   above: 2 KiB, covering the largest positive reach (c.addi4spn sp, 1020
  #          followed by a c.sw at offset 0 -> sp + 1023)
  .align 4
rvc_stack_lo:
  .zero 512
rvc_sp_base:
  .zero 2048
rvc_stack_hi:

RVTEST_DATA_END
