# rv906 unit benches -- standalone Verilator models, one obj dir per bench.
#
# Pattern copied from rv12's test/m1/unit/Makefile: each bench verilates ONE
# rtl module (plus what it instantiates) with its own --Mdir under obj/unit/,
# and links a plain C++ main. Nothing here touches the full-stack build in
# verilator.mk. This is M1's first bench (ICache, plan task 2.2); later
# milestones add their own targets here the same way rv12 did.
#
#   make -C test/m1/unit icache        # build the ICache bench
#   make -C test/m1/unit run           # build and run every bench (UNIT-SUITE-PASS/FAIL)
#   make -C test/m1/unit clean

ROOT      := $(abspath $(CURDIR)/../../..)
VERILATOR ?= verilator
CXX_TOOL  ?= clang++

VFLAGS = --cc --exe -O2 -Wno-fatal --compiler clang

BIN_DIR = $(ROOT)/bin/unit

.PHONY: all run clean
all: icache

#-----------------------------------------------------------------------------
# ICache (plan task 2.2)
#-----------------------------------------------------------------------------
ICACHE_OBJ = $(ROOT)/obj/unit/icache
ICACHE_BIN = $(BIN_DIR)/icache_tb
ICACHE_RTL = $(ROOT)/rtl/rvproc_pkg.sv $(ROOT)/rtl/ICache.v $(ROOT)/rtl/SRAM.v

.PHONY: icache
icache: $(ICACHE_BIN)

$(ICACHE_BIN): $(ICACHE_RTL) $(CURDIR)/icache_tb.cpp
	@mkdir -p $(ICACHE_OBJ) $(BIN_DIR)
	$(VERILATOR) $(VFLAGS) --top-module ICache --Mdir $(ICACHE_OBJ) \
		-o $@ $(ICACHE_RTL) $(CURDIR)/icache_tb.cpp
	$(MAKE) -C $(ICACHE_OBJ) -f VICache.mk CXX=$(CXX_TOOL) LINK=$(CXX_TOOL)

# Run EVERY bench even when one fails, then report -- a recipe line that
# exits non-zero would otherwise abort the target and hide the benches
# behind it, exactly backwards for a regression run (rv12 precedent).
run: all
	@rc=0; \
	$(ICACHE_BIN) || rc=1; \
	if [ $$rc -ne 0 ]; then echo "UNIT-SUITE-FAIL"; else echo "UNIT-SUITE-PASS"; fi; \
	exit $$rc

clean:
	rm -rf $(ROOT)/obj/unit $(ICACHE_BIN)
