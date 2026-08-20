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
# CSR.v/IU.v are the benches so far (plan Tasks 2.2/3.7); RTU.v joins here
# (plan Task 4.3). Tasks 5-6 (IDU/LSU+DCache+MMU) each add their own target
# here the same way, following this exact template -- do not restructure
# when they land.
#
#   make -C test/m2/unit csr        # build the CSR bench
#   make -C test/m2/unit iu         # build the IU bench (plan Task 3.7)
#   make -C test/m2/unit rtu        # build the RTU bench (plan Task 4.3)
#   make -C test/m2/unit run        # build and run every bench (UNIT-SUITE-PASS/FAIL)
#   make -C test/m2/unit clean

ROOT      := $(abspath $(CURDIR)/../../..)
VERILATOR ?= verilator
CXX_TOOL  ?= clang++

VFLAGS = --cc --exe -O2 -Wno-fatal --compiler clang

BIN_DIR = $(ROOT)/bin/unit

.PHONY: all run clean
all: csr iu rtu

#-----------------------------------------------------------------------------
# CSR (plan task 2.2)
#-----------------------------------------------------------------------------
CSR_OBJ = $(ROOT)/obj/unit/csr
CSR_BIN = $(BIN_DIR)/csr_tb
CSR_RTL = $(ROOT)/rtl/rvproc_pkg.sv $(ROOT)/rtl/CSR.v

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

# Run EVERY bench even when one fails, then report -- a recipe line that
# exits non-zero would otherwise abort the target and hide the benches
# behind it, exactly backwards for a regression run (test/m1/unit/Makefile
# precedent).
run: all
	@rc=0; \
	$(CSR_BIN) || rc=1; \
	$(IU_BIN) || rc=1; \
	$(RTU_BIN) || rc=1; \
	if [ $$rc -ne 0 ]; then echo "UNIT-SUITE-FAIL"; else echo "UNIT-SUITE-PASS"; fi; \
	exit $$rc

clean:
	rm -rf $(ROOT)/obj/unit/csr $(CSR_BIN) $(ROOT)/obj/unit/iu $(IU_BIN) $(ROOT)/obj/unit/rtu $(RTU_BIN)
