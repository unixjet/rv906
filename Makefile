# rv906 top-level Makefile
.DEFAULT_GOAL := all
include config

# Generate config.h from config (same rule as rv12/rocket-chip)
config.h: config
	@echo "// Generated from 'config' -- do not edit" > $@
	@sed -n 's/^\(CONFIG_[A-Za-z0-9_]*\)=\(.*\)$$/#define \1 \2/p' config >> $@

all: verisim

# -include (not include): verilator.mk does not exist until Task 6, and
# "make config.h" must already work in Task 2.
-include verilator.mk

clean::
	rm -rf bin obj config.h
