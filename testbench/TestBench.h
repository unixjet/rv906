#ifndef	_TESTBENCH_H_
#define	_TESTBENCH_H_

#include <vector>
#include <string>
#include "fdt.h"

struct TestBench {
	static TestBench *inst;
	static bool stop;
	uint64_t initial_pc;
	uint64_t initial_sp = 0;
	uint64_t stack_top = 0;
	uint64_t tohost = -1;
	uint64_t fromhost = -1;
	uint64_t dtb_addr;
	uint64_t kernel_addr;
	uint64_t initrd_addr;
	uint64_t mem_size = 0x40000000;  // 1GB default
	std::vector<const char *> elf;
	bool alloc_terminal = false;
	bool sxs = false;
	bool print_result = false;
	const char *expect = NULL;
	const char *disk_image[2] = {};
	// Signature: Some RV tests need memory dump for result verification
	const char *signature_file = NULL;  // Output file for signature dump
	uint64_t sig_begin = 0;  // Start address of signature region (from ELF symbol)
	uint64_t sig_end = 0;    // End address of signature region (from ELF symbol)
	std::vector<std::string> elf_args; // Elf Arguments, e.g.: testbench xxx.elf --args "--msg hello"
	bool no_testvec = false;

	TestBench();
	virtual ~TestBench() {}
	virtual void build_fdt(FDT::Node *root) {}
	virtual void init() {}
	virtual bool step() = 0;
	virtual void term() {}
	virtual void interrupted() {}
	virtual uint64_t read_mem(uint64_t addr);
	virtual void write_mem(uint64_t addr, uint64_t data);
	virtual void write_byte(uint64_t addr, uint8_t data);
	virtual void *get_page(uint64_t addr);
	void check();

	void parse_arg(int argc, char **argv);
	int run(int argc, char **argv);

	static uint64_t parse_mem_size(const char *str);
	void pass_elf_args(const std::vector<std::string>& args, uint64_t addr);
	void dump_signature();      // Dump memory [sig_begin, sig_end) to signature_file
};

#endif	// _TESTBENCH_H_
