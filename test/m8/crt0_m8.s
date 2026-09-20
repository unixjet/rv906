/*Copyright 2020-2021 T-Head Semiconductor Co., Ltd.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/
# M8 port of the donor smart_run/tests/lib/crt0.s (post-T1 state) for the
# rv906 harness. Source-identity policy (design doc
# docs/superpowers/specs/2026-09-21-m8-crosscheck-design.md, section 4.6):
#
#   - Every boot poke is BYTE-IDENTICAL to the donor, including the
#     unknown-CSR pokes that are no-ops on rv906. R4 correction: unknown
#     CSR writes are dropped (no local_en), NOT trapped
#     (rtl/CSR.v:1795-1796), so the poke stream is kept verbatim:
#         csrs 0x7c0, 0x400000  (mxstatus theisaee -- no-op on rv906)
#         csrs mstatus, 0x802000 (FS=dirty, MPP=M)
#         csrs mstatus, 0x8       (MIE)
#         csrs 0x7c2, 0x30013     (mcor -- no-op; rv906 has no MCOR)
#         csrs 0x7c1, 0x7f        (mhcr -- ie/de/wa/rse/bpe/btbe, reset 0)
#         csrs 0x7c5, 0x610c      (mhint -- subset modeled)
#   - sp <- __kernel_stack, provided by link_m8.ld (not the donor's 0xee000).
#   - The trap handler + 128-entry vector_table -> __fail scaffold is kept
#     verbatim (donor crt0.s:160-230); rv906's mtvec/cause dispatch is
#     standard (the M2-M6 trap path).
#   - EXIT PROTOCOL DELTA (ledger L13 + L15): the donor's magic-x3 exit
#     (0x444333222 / 0x2382348720 sniffed by the donor tb on the writeback
#     bus, tb.v:259-282) is replaced by the rv906 tohost protocol:
#         __exit -> tohost = 1  (PASS: (0<<1)|1)
#         __fail -> tohost = 3  (FAIL: fixed testno 1 = (1<<1)|1; the
#                                donor __fail takes no argument)
#     The `tohost` symbol is exported (link_m8.ld, at 0x7FFFF000 in the
#     uncached aperture); the C++ harness polls it by SYMBOL (the
#     test/entry.S:204-212 .globl pattern -- an ELF that does not export
#     it hangs the run).
#   - Ends with `jal main` exactly as the donor (crt0.s:139). Nothing
#     else is dropped.
#   - Section note: the startup code (.text.startup) is placed first in
#     the image by link_m8.ld so __start sits at the 0x80000000 reset
#     vector (the donor's `crt0.o (.text)`-first link order, by wildcard).
#     The trap handler below sits in .crt_text (own output section, so
#     its `.align 10` cannot push the image start off 0x80000000); the
#     scaffold bytes are unchanged.

	.section .text.startup

	.global	__start
__start:

# enable extension
  li   x3, 0x400000
  csrs 0x7c0,x3  #mxstatus

# enable fpu
  li   x3, 0x802000
  csrs mstatus,x3


# PART 1: initialize all registers
  li  x1, 0
  li  x2, 0
  li  x3, 0
  li  x4, 0
  li  x5, 0
  li  x6, 0
  li  x7, 0
  li  x8, 0
  li  x9, 0
  li  x10,0
  li  x11,0
  li  x12,0
  li  x13,0
  li  x14,0
  li  x15,0
  li  x16,0
  li  x17,0
  li  x18,0
  li  x19,0
  li  x20,0
  li  x21,0
  li  x22,0
  li  x23,0
  li  x24,0
  li  x25,0
  li  x26,0
  li  x27,0
  li  x28,0
  li  x29,0
  li  x30,0
  li  x31,0

.ifdef C906FDV
  li		x31, 0x1
  vsetvli	x31, x0, e8, m1
  vmv.v.x	v0, x0 
  vmv.v.x	v1, x0 
  vmv.v.x	v2, x0 
  vmv.v.x	v3, x0 
  vmv.v.x	v4, x0 
  vmv.v.x	v5, x0 
  vmv.v.x	v6, x0 
  vmv.v.x	v7, x0 
  vmv.v.x	v8, x0 
  vmv.v.x	v9, x0 
  vmv.v.x	v10, x0 
  vmv.v.x	v11, x0 
  vmv.v.x	v12, x0 
  vmv.v.x	v13, x0 
  vmv.v.x	v14, x0 
  vmv.v.x	v15, x0 
  vmv.v.x	v16, x0 
  vmv.v.x	v17, x0 
  vmv.v.x	v18, x0 
  vmv.v.x	v19, x0 
  vmv.v.x	v20, x0 
  vmv.v.x	v21, x0 
  vmv.v.x	v22, x0 
  vmv.v.x	v23, x0 
  vmv.v.x	v24, x0 
  vmv.v.x	v25, x0 
  vmv.v.x	v26, x0 
  vmv.v.x	v27, x0 
  vmv.v.x	v28, x0 
  vmv.v.x	v29, x0 
  vmv.v.x	v30, x0 
  vmv.v.x	v31, x0
##restore csr to initial state 
  vsetvli	x0, x0, e128
.endif
  .global cpu_0_sp
cpu_0_sp:
  la x2, __kernel_stack

# PART 3:initialize mtvec value
  la    x3,__trap_handler
  csrw  mtvec,x3


  # enable mie
  li   x3, 0x8
  csrs mstatus,x3


  # invalid all memory for BTB,BHT,DCACHE,ICACHE
  li x3, 0x30013
  csrs 0x7c2,x3  #mcor

  # enable ICACHE,DCACHE,BHT,BTB,RAS,WA
  li x3, 0x7f
  csrs 0x7c1,x3  #mhcr

  


  # enable data_cache_prefetch, amr
  li x3, 0x610c
  csrs 0x7c5,x3   #mhint


  jal	main



  .global __exit
__exit:
  addi x10,x0,0x0
  addi x1,x0,0x5a
  addi x2,x0,0x6b
  addi x3,x0,0x7c
  li   x3, 1                 # rv906 (L13): tohost = (0<<1)|1 = PASS
  la   x4, tohost
  sd   x3, 0(x4)
1:  j 1b
#
  .global __fail
__fail:
  addi x10,x0,0x0
  addi x1,x0,0x2c
  addi x2,x0,0x3b
  li x3,3                    # rv906 (L13/L15): tohost = (1<<1)|1 = FAIL, testno 1
  la   x4, tohost
  sd   x3, 0(x4)
1:  j 1b

.section .crt_text,"ax"
__trap_handler:
  j __synchronous_exception
  .align 2
  j __asychronous_int
  .align 2
  nop #reserved
  .align 2
  j __asychronous_int
  .align 2
  j __asychronous_int
  .align 2
  j __asychronous_int
  .align 2
  nop #reserved
  .align 2
  j __asychronous_int
  .align 2
  j __asychronous_int
  .align 2
  j __asychronous_int
  .align 2
  nop #reserved
  .align 2
  j __asychronous_int
  j __fail

__synchronous_exception:
  #push
#sw   x13,-4(sp)
  sd   x14,-16(sp)
  sd   x15,-24(sp)
  csrr x14,mcause
  andi x15,x14,0x1f    #cause
  srli x14,x14,0x3b   #int
  andi x14,x14,0x10   #mask bit
  add  x14,x14,x15    #{int,cause}

  slli x14,x14,0x3 #offset
  la   x15,vector_table
  add  x15,x14,x15  #target pc
  ld   x14, 0(x15)  #get exception addr
#  lw   x13, -4(sp)  #recover x13
  ld   x15, -24(sp) #recover x15
  addi x14,x14,-4
  jr   x14

__asychronous_int:
  sw   x13,-4(sp)
  sw   x14,-8(sp)
  sw   x15,-12(sp)
  csrr x14,mcause
  andi x15,x14,0x1f    #cause
  srli x14,x14,0x3b   #int
  andi x14,x14,0x10   #mask bit
  add  x14,x14,x15    #{int,cause}

  slli x14,x14,0x3   #offset
  la   x15,vector_table
  add  x15,x14,x15   #target pc
  lw   x14, 0(x15)   #get exception addr
  lw   x13, -4(sp)   #recover x13
  lw   x15, -12(sp)  #recover x15
  addi x14,x14,-4
  jr   x14

.global vector_table
 .align  10
  vector_table:   #totally 256 entries
  .rept   128
  .long   __fail
  .endr

  .global __dummy
__dummy:

  .data
  nop
  nop
  nop
