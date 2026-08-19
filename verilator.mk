# Verilator RTL simulation for rv906
VERISIM_TOP     = RVProcAXI
VERISIM_RTL     = rtl/rvproc_pkg.sv $(filter-out rtl/rvproc_pkg.sv, \
                    $(wildcard rtl/*.sv) $(wildcard rtl/*.v))
VERISIM_OBJ_DIR = obj/verisim
VERISIM_TARGET  = bin/verisim/testbench
VERISIM_CFLAGS  = -I$(CURDIR) -I$(CURDIR)/$(VERISIM_OBJ_DIR) -I$(CURDIR)/rtl \
                  -Wno-unknown-warning-option -DVERILATOR_SIM
VERISIM_SRCS    = RVProcTest.cpp dut.cpp \
                  testbench/TestBench.cpp testbench/load_elf.cpp testbench/fdt.cpp \
                  io/RVProc_io.cpp device/ttysrv.cpp device/uart16550.cpp

VERISIM_FLAGS   = --cc --exe -O3 -Wno-fatal --top-module $(VERISIM_TOP) \
                  --compiler clang -LDFLAGS "-lelf -lfdt"

# Optional FST tracing: `make verisim VERISIM_TRACE=1`; waves in run/dump.fst
ifdef VERISIM_TRACE
VERISIM_FLAGS  += --trace-fst
VERISIM_CFLAGS += -DVERISIM_TRACE
endif

VERISIM_HDRS = config.h RVProc.h RVProcArch.h C2Rdef.h dut.h \
               $(wildcard rtl/*.h) $(wildcard testbench/*.h) \
               $(wildcard io/*.h) $(wildcard device/*.h)

# Trace-mode stamp: switching VERISIM_TRACE on/off invalidates the build
TRACE_STAMP = $(VERISIM_OBJ_DIR)/.trace-$(if $(VERISIM_TRACE),1,0)
$(TRACE_STAMP):
	@mkdir -p $(VERISIM_OBJ_DIR)
	@rm -f $(VERISIM_OBJ_DIR)/.trace-*
	@touch $@

.PHONY: verisim
verisim: $(VERISIM_TARGET)

$(VERISIM_TARGET): $(VERISIM_RTL) $(VERISIM_SRCS) $(VERISIM_HDRS) $(TRACE_STAMP)
	@mkdir -p $(dir $@) $(VERISIM_OBJ_DIR)
	cp rtl/verisim.h $(VERISIM_OBJ_DIR)/verisim.h
	verilator $(VERISIM_FLAGS) -CFLAGS "$(VERISIM_CFLAGS)" \
		--Mdir $(VERISIM_OBJ_DIR) -o $(CURDIR)/$@ \
		$(VERISIM_RTL) $(addprefix $(CURDIR)/,$(VERISIM_SRCS))
	$(MAKE) -C $(VERISIM_OBJ_DIR) -f V$(VERISIM_TOP).mk CXX=clang++ LINK=clang++
