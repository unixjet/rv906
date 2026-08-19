#ifndef	_LOAD_ELF_H_
#define	_LOAD_ELF_H_

#include <stdio.h>
#include <stdint.h>
#include <functional>

struct symbol_table {
	const char *name;
	uint64_t *value;
};

extern bool load_elf(const char *filename, std::function<void*(uint64_t)> get_page, uint64_t *initial_pc, struct symbol_table *syms);
extern bool load_elf(const char *filename, std::function<void(uint64_t, uint8_t)> write_byte, uint64_t *initial_pc, struct symbol_table *syms);
extern bool load_bin(const char *filename, std::function<void*(uint64_t)> get_page, uint64_t addr);
extern bool load_bin(const char *filename, std::function<void(uint64_t, uint8_t)> write_byte, uint64_t addr);

#endif	// _LOAD_ELF_H_
