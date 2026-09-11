# rv906 M7 unit benches -- standalone Verilator models, one obj dir per
# bench, same structural pattern as test/m2/unit/Makefile (contract 15):
# one --top-module per bench target, obj/m7-unit/<name>, bin/unit/<name>_tb,
# a run/UNIT-SUITE-PASS aggregate target.
#
#   make -C test/m7/unit dm        # build the DM bench (M7 Task 3)
#   make -C test/m7/unit dtm       # build the DTM bench (M7 Task 4)
#   make -C test/m7/unit run       # build and run every bench
#   make -C test/m7/unit clean
#
# NOTE: these benches verilate modules that are NOT yet part of the SoC
# (TDT_DM.v / TDT_DTM.v join RVProcAXI.v at M7 Task 5), so NO gate here
# runs the full-stack build -- the SoC is untouched by this directory.

ROOT      := $(abspath $(CURDIR)/../../..)
VERILATOR ?= verilator
CXX_TOOL  ?= clang++

VFLAGS = --cc --exe -O2 -Wno-fatal --compiler clang

BIN_DIR = $(ROOT)/bin/unit

.PHONY: all run clean
all: dm dtm

#-----------------------------------------------------------------------------
# DM (M7 Task 3) -- TDT_DM.v alone (APB slave + abstract engine + SBA)
#-----------------------------------------------------------------------------
DM_OBJ = $(ROOT)/obj/m7-unit/dm
DM_BIN = $(BIN_DIR)/dm_tb
DM_RTL = $(ROOT)/rtl/TDT_DM.v

.PHONY: dm
dm: $(DM_BIN)

$(DM_BIN): $(DM_RTL) $(CURDIR)/dm_tb.cpp
	@mkdir -p $(DM_OBJ) $(BIN_DIR)
	$(VERILATOR) $(VFLAGS) --top-module TDT_DM --Mdir $(DM_OBJ) \
		-o $@ $(DM_RTL) $(CURDIR)/dm_tb.cpp
	$(MAKE) -C $(DM_OBJ) -f VTDT_DM.mk CXX=$(CXX_TOOL) LINK=$(CXX_TOOL)

#-----------------------------------------------------------------------------
# DTM (M7 Task 4) -- TDT_DTM.v alone (TAP-5 + TDRs + DMI-to-APB bridge)
#-----------------------------------------------------------------------------
DTM_OBJ = $(ROOT)/obj/m7-unit/dtm
DTM_BIN = $(BIN_DIR)/dtm_tb
DTM_RTL = $(ROOT)/rtl/TDT_DTM.v

.PHONY: dtm
dtm: $(DTM_BIN)

$(DTM_BIN): $(DTM_RTL) $(CURDIR)/dtm_tb.cpp
	@mkdir -p $(DTM_OBJ) $(BIN_DIR)
	$(VERILATOR) $(VFLAGS) --top-module TDT_DTM --Mdir $(DTM_OBJ) \
		-o $@ $(DTM_RTL) $(CURDIR)/dtm_tb.cpp
	$(MAKE) -C $(DTM_OBJ) -f VTDT_DTM.mk CXX=$(CXX_TOOL) LINK=$(CXX_TOOL)

# Run EVERY bench even when one fails, then report (test/m2/unit/Makefile
# precedent).
run: all
	@rc=0; \
	$(DM_BIN) || rc=1; \
	$(DTM_BIN) || rc=1; \
	if [ $$rc -ne 0 ]; then echo "UNIT-SUITE-FAIL"; else echo "UNIT-SUITE-PASS"; fi; \
	exit $$rc

clean:
	rm -rf $(ROOT)/obj/m7-unit/dm $(DM_BIN) $(ROOT)/obj/m7-unit/dtm $(DTM_BIN)
