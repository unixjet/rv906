# rv906 M2 unit benches -- standalone Verilator models, one obj dir per
# bench, parallel to test/m1/unit/ (contract 15: NEW directory, same exact
# structural pattern -- one --top-module per bench target, obj/unit/<name>,
# bin/unit/<name>_tb, a run/UNIT-SUITE-PASS aggregate target).
#
# Pattern copied verbatim from test/m1/unit/Makefile: each RTL bench
# verilates ONE rtl module (plus what it instantiates) with its own --Mdir
# under obj/unit/, and links a plain C++ main. Nothing here touches the
# full-stack build in verilator.mk.
#
# CSR.v/IU.v/RTU.v are the benches so far (plan Tasks 2.2/3.7/4.3); IDU.v
# joins here (plan Task 5.6). Task 6 (LSU+DCache+MMU) adds its own targets
# here the same way, following this exact template -- do not restructure
# when it lands.
#
#   make -C test/m2/unit csr        # build the CSR bench
#   make -C test/m2/unit iu         # build the IU bench (plan Task 3.7)
#   make -C test/m2/unit rtu        # build the RTU bench (plan Task 4.3)
#   make -C test/m2/unit idu        # build the IDU bench (plan Task 5.6)
#   make -C test/m2/unit dcache     # build the DCache bench (plan Task 6.4)
#   make -C test/m2/unit mmu        # build the MMU bench (plan Task 6.4)
#   make -C test/m2/unit lsu        # build the LSU bench (plan Task 6.4)
#   make -C test/m2/unit lr_sc      # build the LR/SC bench (M3 Task 1)
#   make -C test/m2/unit run        # build and run every bench (UNIT-SUITE-PASS/FAIL)
#   make -C test/m2/unit clean

ROOT      := $(abspath $(CURDIR)/../../..)
VERILATOR ?= verilator
CXX_TOOL  ?= clang++

VFLAGS = --cc --exe -O2 -Wno-fatal --compiler clang

BIN_DIR = $(ROOT)/bin/unit

.PHONY: all run clean
all: csr iu rtu idu dcache mmu lsu lr_sc fpu dtu

#-----------------------------------------------------------------------------
# CSR (plan task 2.2)
#-----------------------------------------------------------------------------
CSR_OBJ = $(ROOT)/obj/unit/csr
CSR_BIN = $(BIN_DIR)/csr_tb
# M7 Task 1: DTU.v joins the CSR bench's RTL list (CSR.v's 0x7B0-0x7B3
# decode arms route to the DTU; DTU.v is compiled alongside even though the
# CSR --top-module only elaborates CSR.v, keeping the two debug-state files
# lint-checked together in one build).
CSR_RTL = $(ROOT)/rtl/rvproc_pkg.sv $(ROOT)/rtl/CSR.v $(ROOT)/rtl/DTU.v

.PHONY: csr
csr: $(CSR_BIN)

$(CSR_BIN): $(CSR_RTL) $(CURDIR)/csr_tb.cpp
	@mkdir -p $(CSR_OBJ) $(BIN_DIR)
	$(VERILATOR) $(VFLAGS) --top-module CSR --Mdir $(CSR_OBJ) \
		-o $@ $(CSR_RTL) $(CURDIR)/csr_tb.cpp
	$(MAKE) -C $(CSR_OBJ) -f VCSR.mk CXX=$(CXX_TOOL) LINK=$(CXX_TOOL)

#-----------------------------------------------------------------------------
# IU (plan task 3.7)
#-----------------------------------------------------------------------------
IU_OBJ = $(ROOT)/obj/unit/iu
IU_BIN = $(BIN_DIR)/iu_tb
IU_RTL = $(ROOT)/rtl/rvproc_pkg.sv $(ROOT)/rtl/IU.v

.PHONY: iu
iu: $(IU_BIN)

$(IU_BIN): $(IU_RTL) $(CURDIR)/iu_tb.cpp
	@mkdir -p $(IU_OBJ) $(BIN_DIR)
	$(VERILATOR) $(VFLAGS) --top-module IU --Mdir $(IU_OBJ) \
		-o $@ $(IU_RTL) $(CURDIR)/iu_tb.cpp
	$(MAKE) -C $(IU_OBJ) -f VIU.mk CXX=$(CXX_TOOL) LINK=$(CXX_TOOL)

#-----------------------------------------------------------------------------
# RTU (plan task 4.3)
#-----------------------------------------------------------------------------
RTU_OBJ = $(ROOT)/obj/unit/rtu
RTU_BIN = $(BIN_DIR)/rtu_tb
RTU_RTL = $(ROOT)/rtl/rvproc_pkg.sv $(ROOT)/rtl/RTU.v

.PHONY: rtu
rtu: $(RTU_BIN)

$(RTU_BIN): $(RTU_RTL) $(CURDIR)/rtu_tb.cpp
	@mkdir -p $(RTU_OBJ) $(BIN_DIR)
	$(VERILATOR) $(VFLAGS) --top-module RTU --Mdir $(RTU_OBJ) \
		-o $@ $(RTU_RTL) $(CURDIR)/rtu_tb.cpp
	$(MAKE) -C $(RTU_OBJ) -f VRTU.mk CXX=$(CXX_TOOL) LINK=$(CXX_TOOL)

#-----------------------------------------------------------------------------
# IDU (plan task 5.6)
#-----------------------------------------------------------------------------
IDU_OBJ = $(ROOT)/obj/unit/idu
IDU_BIN = $(BIN_DIR)/idu_tb
IDU_RTL = $(ROOT)/rtl/rvproc_pkg.sv $(ROOT)/rtl/IDU.v

.PHONY: idu
idu: $(IDU_BIN)

$(IDU_BIN): $(IDU_RTL) $(CURDIR)/idu_tb.cpp
	@mkdir -p $(IDU_OBJ) $(BIN_DIR)
	$(VERILATOR) $(VFLAGS) --top-module IDU --Mdir $(IDU_OBJ) \
		-o $@ $(IDU_RTL) $(CURDIR)/idu_tb.cpp
	$(MAKE) -C $(IDU_OBJ) -f VIDU.mk CXX=$(CXX_TOOL) LINK=$(CXX_TOOL)

#-----------------------------------------------------------------------------
# DCache (plan task 6.4)
#-----------------------------------------------------------------------------
DCACHE_OBJ = $(ROOT)/obj/unit/dcache
DCACHE_BIN = $(BIN_DIR)/dcache_tb
DCACHE_RTL = $(ROOT)/rtl/rvproc_pkg.sv $(ROOT)/rtl/SRAM.v $(ROOT)/rtl/DCache.v

.PHONY: dcache
dcache: $(DCACHE_BIN)

$(DCACHE_BIN): $(DCACHE_RTL) $(CURDIR)/dcache_tb.cpp
	@mkdir -p $(DCACHE_OBJ) $(BIN_DIR)
	$(VERILATOR) $(VFLAGS) --top-module DCache --Mdir $(DCACHE_OBJ) \
		-o $@ $(DCACHE_RTL) $(CURDIR)/dcache_tb.cpp
	$(MAKE) -C $(DCACHE_OBJ) -f VDCache.mk CXX=$(CXX_TOOL) LINK=$(CXX_TOOL)

#-----------------------------------------------------------------------------
# MMU (plan task 6.4)
#-----------------------------------------------------------------------------
MMU_OBJ = $(ROOT)/obj/unit/mmu
MMU_BIN = $(BIN_DIR)/mmu_tb
MMU_RTL = $(ROOT)/rtl/rvproc_pkg.sv $(ROOT)/rtl/MMU.v

.PHONY: mmu
mmu: $(MMU_BIN)

$(MMU_BIN): $(MMU_RTL) $(CURDIR)/mmu_tb.cpp
	@mkdir -p $(MMU_OBJ) $(BIN_DIR)
	$(VERILATOR) $(VFLAGS) --top-module MMU --Mdir $(MMU_OBJ) \
		-o $@ $(MMU_RTL) $(CURDIR)/mmu_tb.cpp
	$(MAKE) -C $(MMU_OBJ) -f VMMU.mk CXX=$(CXX_TOOL) LINK=$(CXX_TOOL)

#-----------------------------------------------------------------------------
# LSU (plan task 6.4) -- verilates LSU.v + the DCache.v it instantiates
# internally + SRAM.v underneath that, exactly mirroring how RVProc.v will
# eventually build it (DCache.v is instantiated INSIDE LSU.v, not at
# RVProc.v's top level -- see DCache.v's own header).
#-----------------------------------------------------------------------------
LSU_OBJ = $(ROOT)/obj/unit/lsu
LSU_BIN = $(BIN_DIR)/lsu_tb
LSU_RTL = $(ROOT)/rtl/rvproc_pkg.sv $(ROOT)/rtl/SRAM.v $(ROOT)/rtl/DCache.v $(ROOT)/rtl/PFB.v $(ROOT)/rtl/AMR.v $(ROOT)/rtl/LSU.v

.PHONY: lsu
lsu: $(LSU_BIN)

$(LSU_BIN): $(LSU_RTL) $(CURDIR)/lsu_tb.cpp
	@mkdir -p $(LSU_OBJ) $(BIN_DIR)
	$(VERILATOR) $(VFLAGS) --top-module LSU --Mdir $(LSU_OBJ) \
		-o $@ $(LSU_RTL) $(CURDIR)/lsu_tb.cpp
	$(MAKE) -C $(LSU_OBJ) -f VLSU.mk CXX=$(CXX_TOOL) LINK=$(CXX_TOOL)

#-----------------------------------------------------------------------------
# LR/SC (M3 Task 1) -- standalone bench for LSU.v LR.W / SC.W foundation
#-----------------------------------------------------------------------------
LR_SC_OBJ = $(ROOT)/obj/unit/lr_sc
LR_SC_BIN = $(BIN_DIR)/lr_sc_tb
LR_SC_RTL = $(ROOT)/rtl/rvproc_pkg.sv $(ROOT)/rtl/SRAM.v $(ROOT)/rtl/DCache.v $(ROOT)/rtl/PFB.v $(ROOT)/rtl/AMR.v $(ROOT)/rtl/LSU.v

.PHONY: lr_sc
lr_sc: $(LR_SC_BIN)

$(LR_SC_BIN): $(LR_SC_RTL) $(CURDIR)/lr_sc_tb.cpp
	@mkdir -p $(LR_SC_OBJ) $(BIN_DIR)
	$(VERILATOR) $(VFLAGS) --top-module LSU --Mdir $(LR_SC_OBJ) \
		-o $@ $(LR_SC_RTL) $(CURDIR)/lr_sc_tb.cpp
	$(MAKE) -C $(LR_SC_OBJ) -f VLSU.mk CXX=$(CXX_TOOL) LINK=$(CXX_TOOL)

#-----------------------------------------------------------------------------
# FPU (M5 task 3) -- FALU sub-block only (add/sub/cmp/min-max, sign-inject
# + fclass, float-to-float convert); not yet reachable from IDU decode.
#-----------------------------------------------------------------------------
FPU_OBJ = $(ROOT)/obj/unit/fpu
FPU_BIN = $(BIN_DIR)/fpu_tb
FPU_RTL = $(ROOT)/rtl/rvproc_pkg.sv $(ROOT)/rtl/FPU.v

.PHONY: fpu
fpu: $(FPU_BIN)

$(FPU_BIN): $(FPU_RTL) $(CURDIR)/fpu_tb.cpp
	@mkdir -p $(FPU_OBJ) $(BIN_DIR)
	$(VERILATOR) $(VFLAGS) --top-module FPU --Mdir $(FPU_OBJ) \
		-o $@ $(FPU_RTL) $(CURDIR)/fpu_tb.cpp
	$(MAKE) -C $(FPU_OBJ) -f VFPU.mk CXX=$(CXX_TOOL) LINK=$(CXX_TOOL)

#-----------------------------------------------------------------------------
# DTU (M7 Task 1) -- core-side debug unit, standalone white-box bench.
#-----------------------------------------------------------------------------
DTU_OBJ = $(ROOT)/obj/unit/dtu
DTU_BIN = $(BIN_DIR)/dtu_tb
DTU_RTL = $(ROOT)/rtl/rvproc_pkg.sv $(ROOT)/rtl/DTU.v

.PHONY: dtu
dtu: $(DTU_BIN)

$(DTU_BIN): $(DTU_RTL) $(CURDIR)/dtu_tb.cpp
	@mkdir -p $(DTU_OBJ) $(BIN_DIR)
	$(VERILATOR) $(VFLAGS) --top-module DTU --Mdir $(DTU_OBJ) \
		-o $@ $(DTU_RTL) $(CURDIR)/dtu_tb.cpp
	$(MAKE) -C $(DTU_OBJ) -f VDTU.mk CXX=$(CXX_TOOL) LINK=$(CXX_TOOL)

# Run EVERY bench even when one fails, then report -- a recipe line that
# exits non-zero would otherwise abort the target and hide the benches
# behind it, exactly backwards for a regression run (test/m1/unit/Makefile
# precedent).
run: all
	@rc=0; \
	$(CSR_BIN) || rc=1; \
	$(IU_BIN) || rc=1; \
	$(RTU_BIN) || rc=1; \
	$(IDU_BIN) || rc=1; \
	$(DCACHE_BIN) || rc=1; \
	$(MMU_BIN) || rc=1; \
	$(LSU_BIN) || rc=1; \
	$(LR_SC_BIN) || rc=1; \
	$(FPU_BIN) || rc=1; \
	$(DTU_BIN) || rc=1; \
	if [ $$rc -ne 0 ]; then echo "UNIT-SUITE-FAIL"; else echo "UNIT-SUITE-PASS"; fi; \
	exit $$rc

clean:
	rm -rf $(ROOT)/obj/unit/csr $(CSR_BIN) $(ROOT)/obj/unit/iu $(IU_BIN) $(ROOT)/obj/unit/rtu $(RTU_BIN) $(ROOT)/obj/unit/idu $(IDU_BIN) \
	       $(ROOT)/obj/unit/dcache $(DCACHE_BIN) $(ROOT)/obj/unit/mmu $(MMU_BIN) $(ROOT)/obj/unit/lsu $(LSU_BIN) \
	       $(ROOT)/obj/unit/lr_sc $(LR_SC_BIN) $(ROOT)/obj/unit/fpu $(FPU_BIN) $(ROOT)/obj/unit/dtu $(DTU_BIN)
