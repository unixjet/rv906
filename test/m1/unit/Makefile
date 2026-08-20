# rv906 unit benches -- standalone Verilator models, one obj dir per bench,
# PLUS the ISS's own zero-RTL unit test (plan Task 5.3).
#
# Pattern copied from rv12's test/m1/unit/Makefile: each RTL bench verilates
# ONE rtl module (plus what it instantiates) with its own --Mdir under
# obj/unit/, and links a plain C++ main. Nothing here touches the full-stack
# build in verilator.mk. This is M1's unit-bench Makefile (ICache: plan task
# 2.2; FetchSink: plan task 4.3; ISS: plan task 5.3); later milestones add
# their own targets here the same way rv12 did.
#
# `iss` is deliberately NOT a verilator target: m1_iss.h is plain C++ with no
# RTL dependency at all, and the whole point of Task 5.3 is to gate it
# BEFORE it is trusted to grade FetchSink/IFU/ICache -- so it just compiles
# and links directly.
#
#   make -C test/m1/unit icache        # build the ICache bench
#   make -C test/m1/unit fetchsink     # build the FetchSink bench
#   make -C test/m1/unit iss           # build the fetch-ISS's own unit test
#   make -C test/m1/unit run           # build and run every bench (UNIT-SUITE-PASS/FAIL)
#   make -C test/m1/unit clean

ROOT      := $(abspath $(CURDIR)/../../..)
VERILATOR ?= verilator
CXX_TOOL  ?= clang++

VFLAGS = --cc --exe -O2 -Wno-fatal --compiler clang

BIN_DIR = $(ROOT)/bin/unit

.PHONY: all run clean
all: icache fetchsink iss

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

#-----------------------------------------------------------------------------
# FetchSink (plan task 4.3)
#-----------------------------------------------------------------------------
FETCHSINK_OBJ = $(ROOT)/obj/unit/fetchsink
FETCHSINK_BIN = $(BIN_DIR)/fetchsink_tb
FETCHSINK_RTL = $(ROOT)/rtl/rvproc_pkg.sv $(ROOT)/rtl/FetchSink.v

.PHONY: fetchsink
fetchsink: $(FETCHSINK_BIN)

$(FETCHSINK_BIN): $(FETCHSINK_RTL) $(CURDIR)/fetchsink_tb.cpp
	@mkdir -p $(FETCHSINK_OBJ) $(BIN_DIR)
	$(VERILATOR) $(VFLAGS) --top-module FetchSink --Mdir $(FETCHSINK_OBJ) \
		-o $@ $(FETCHSINK_RTL) $(CURDIR)/fetchsink_tb.cpp
	$(MAKE) -C $(FETCHSINK_OBJ) -f VFetchSink.mk CXX=$(CXX_TOOL) LINK=$(CXX_TOOL)

#-----------------------------------------------------------------------------
# fetch-ISS self-test (plan task 5.3) -- plain C++, no verilator, no RTL.
#-----------------------------------------------------------------------------
ISS_BIN = $(BIN_DIR)/iss_tb
ISS_HDR = $(ROOT)/m1_iss.h

.PHONY: iss
iss: $(ISS_BIN)

$(ISS_BIN): $(ISS_HDR) $(CURDIR)/iss_tb.cpp
	@mkdir -p $(BIN_DIR)
	$(CXX_TOOL) -std=c++14 -O2 -I$(ROOT) -o $@ $(CURDIR)/iss_tb.cpp

# Run EVERY bench even when one fails, then report -- a recipe line that
# exits non-zero would otherwise abort the target and hide the benches
# behind it, exactly backwards for a regression run (rv12 precedent).
run: all
	@rc=0; \
	$(ICACHE_BIN) || rc=1; \
	$(FETCHSINK_BIN) || rc=1; \
	$(ISS_BIN) || rc=1; \
	if [ $$rc -ne 0 ]; then echo "UNIT-SUITE-FAIL"; else echo "UNIT-SUITE-PASS"; fi; \
	exit $$rc

clean:
	rm -rf $(ROOT)/obj/unit $(ICACHE_BIN) $(FETCHSINK_BIN) $(ISS_BIN)


