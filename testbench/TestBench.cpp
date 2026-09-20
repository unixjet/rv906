#include <fcntl.h>
#include <signal.h>
#include <unistd.h>
#include <string.h>
#include <sstream>
#include <cstdlib>
#include <cctype>
#include <fstream>
#include <iomanip>
#include "config.h"
#include "load_elf.h"
#include "TestBench.h"
#include "RVProc.h"
#include "io/ExtMem.h"

#define GP  3
#define A0  10

extern ExtMem xmem;

void *TestBench::get_page(uint64_t paddr) { return xmem.get_page(paddr)->m; }

extern CoreState cpu;

#define	PAGE_SIZE	0x1000

TestBench *TestBench::inst;

TestBench::TestBench() {
#if	CONFIG_RV64I
	kernel_addr = 0x80200000;
#else
	kernel_addr = 0x80400000;
#endif
	inst = this;
}

uint64_t TestBench::read_mem(uint64_t addr) {
	ExtMem::page *page = xmem.get_page(addr);
	return page->m[(addr & 0xfff) / sizeof(page->m[0])];
}
void TestBench::write_mem(uint64_t addr, uint64_t data) {
	ExtMem::page *page = xmem.get_page(addr);
	page->m[(addr & 0xfff) / sizeof(page->m[0])] = data;
}
void TestBench::write_byte(uint64_t addr, uint8_t data) {
	uint8_t *page = (uint8_t *)get_page(addr);
	page[addr & 0xfff] = data;
}

void TestBench::parse_arg(int argc, char **argv)
{
	char *kernel = NULL;
	char *initrd = NULL;
	std::string bootargs = "console=ttyS0 earlycon";
	std::vector<std::string> elf_args;  // Store arguments for the ELF program

	while (--argc) {
		char *cp = *++argv;

		if (strcmp(cp, "--kernel") == 0) {
			if (argc > 1) {
				argc--;
				kernel = *(++argv);
			}
			continue;
		} else if (strcmp(cp, "--initrd") == 0) {
			if (argc > 1) {
				argc--;
				initrd = *(++argv);
			}
			continue;
		} else if (strcmp(cp, "--bootargs") == 0) {
			if (argc > 1) {
				argc--;
				bootargs = *(++argv);
			}
			continue;
		} else if (strcmp(cp, "--expect") == 0) {
			if (argc > 1) {
				argc--;
				expect = *(++argv);
			}
			continue;
		} else if (strcmp(cp, "--alloc-terminal") == 0) {
			alloc_terminal = true;
			continue;
		} else if (strcmp(cp, "--sxs") == 0) {
			sxs = true;
			continue;
		} else if (strcmp(cp, "--print-result") == 0) {
			print_result = true;
			continue;
		} else if (strcmp(cp, "--disk-image") == 0) {
			if (argc > 1) {
				argc--;
				if (disk_image[0] == NULL)
					disk_image[0] = *(++argv);
				else
					disk_image[1] = *(++argv);
			}
			continue;
		} else if (strcmp(cp, "--mem_size") == 0) {
			if (argc > 1) {
				argc--;
				const uint64_t size = parse_mem_size(*(++argv));
				if (size > 0) mem_size = size;
			}
			continue;
		} else if (strcmp(cp, "--signature") == 0) {
			if (argc > 1) {
				argc--;
				signature_file = *(++argv);
			}
			continue;
		} else if (strcmp(cp, "--") == 0) {
			// Collect all remaining arguments as ELF program arguments
			while (argc > 1) {
				argc--;
				char *args_str = *(++argv);
				elf_args.push_back(args_str);
			}
			continue;
		} else if (strcmp(cp, "--no-testvec") == 0) {
			no_testvec = true;
			continue;
		}

		elf.push_back(cp);
	}

	//// build fdt
	FDT fdt;

        fdt.create(0x10000);

        FDT::Node root = fdt.open("/");

        root.setprop("#address-cells", (uint32_t)2);
        root.setprop("#size-cells", (uint32_t)2);

	build_fdt(&root);

        FDT::Node chosen = root.create("chosen");
        chosen.setprop("stdout-path", "/uart@10001000");
        if (initrd) {
            struct stat sb;

            stat(initrd, &sb);
            chosen.setprop("linux,initrd-start", initrd_addr);
            chosen.setprop("linux,initrd-end", initrd_addr + sb.st_size);

#if	0
	    bootargs += " initrd=";
	    std::stringstream ss;
	    ss << "0x" << std::hex << initrd_addr << ",0x" << std::hex << sb.st_size;
	    bootargs += ss.str();
#endif
        }
	chosen.setprop("bootargs", bootargs.c_str());

        fdt.pack();

        mkdir("run", 0755);
        int fd = creat("run/c2rtl.dtb", 0666);
        write(fd, fdt.blob, fdt.totalsize());
        close(fd);

	uint32_t size = fdt.totalsize();
	uint64_t paddr = dtb_addr;
	char *blob = (char *)fdt.blob;

	while (size) {
		void *page = get_page(paddr);
		ssize_t count = size;

		if (count > PAGE_SIZE)
			count = PAGE_SIZE;
		memcpy(page, blob, count);
		size -= count;
		paddr += count;
		blob += count;
	}

	//// load files
	std::function<void(uint64_t, uint8_t)> f_write_byte = [this](uint64_t addr, uint8_t data) { write_byte(addr, data); };

	tohost = -1;
	if (elf.size()) {
		struct symbol_table syms[] = {
			{ "tohost", &tohost },
			{ "fromhost", &fromhost },
			{ "begin_signature", &sig_begin },
			{ "end_signature", &sig_end },
			{ "_stack_top", &stack_top },
			{ NULL, NULL },
		};

		load_elf(elf[0], f_write_byte, &initial_pc, syms);

		// Use _stack_top from ELF if available, otherwise fallback to dtb_addr - 64KB
		if (stack_top)
			initial_sp = stack_top;
		else
			initial_sp = dtb_addr - 0x10000;
		pass_elf_args(elf_args, initial_sp);
	}
	for (size_t i = 1; i < elf.size(); i++) {
		uint64_t tmp;

		load_elf(elf[i], f_write_byte, &tmp, NULL);
	}
	if (kernel)
		load_bin(kernel, f_write_byte, kernel_addr);
	if (initrd)
		load_bin(initrd, f_write_byte, initrd_addr);
}

bool TestBench::stop = false;

void sigint_handler(int signum)
{
	TestBench::stop = true;

	signal(SIGINT, SIG_DFL);
}

int TestBench::run(int argc, char **argv)
{
	parse_arg(argc, argv);

	signal(SIGINT, sigint_handler);
	init();

	bool quitted = false;
	uint64_t out = 0;

	int tohost_wait = 10;
	uint64_t cycle = 0;

	while (!quitted) {
		if (++cycle % 1000000 == 0) {
			printf("cycle %lu gp=%llx tohost=%llx\n", (unsigned long)cycle, (long long)cpu.gpr[3], (long long)(tohost != -1 ? read_mem(tohost) : 0));
			fflush(stdout);
		}
		if (sxs)
			check();
		quitted = step();

		if (stop) {
			interrupted();
			fprintf(stderr, "%s: Interrupted\n", elf[0]);
			kill(getpid(), SIGINT);
		}

		if (tohost != -1) {
			/// p: fail/pass -> 0 : testnum ====> auipc, sw(0), auipc, sw(+4)
			/// v: while (tohost)
			///        fromhost = 0;
			///    tohost = 0 : testnum / 0x01010000: char
			/// TB: exit -> 0 : testnum
			/// TB: syscall -> tohost -> pointer
			///                while (fromhost == 0)
			///                    ;
			///                fromhost = 0;
			out = read_mem(tohost);

			if (!out)
				continue;
			if (tohost_wait--)
				continue;

			tohost_wait = 10;

			uint32_t out2 = out >> 32;

			if (out2 == 0x01010000) {
				printf("%c", (char)out);
			} else if (out & 1) {
				if (out2)
					printf("out2 = 0x%08x\n", out2);
				if (signature_file && sig_begin && sig_end) {
					dump_signature();
				}
				break;
			} else {
				// syscall
				uint64_t which = read_mem(out);
				uint64_t arg0 = read_mem(out + 0x8);
				uint64_t arg1 = read_mem(out + 0x10);
				uint64_t arg2 = read_mem(out + 0x18);
				uint64_t ret = -1;

				if (which == 64) {
					fflush(stdout);
					ret = arg2;

					while (arg2) {
						uint64_t data = read_mem(arg1);
						int offset = arg1 & 7;
						int count = 8 - offset;

						if (count > arg2)
							count = arg2;
						write(1, (char *)&data + offset, count);
						arg1 += count;
						arg2 -= count;
					}
				} else {
					printf("out = %lx: which = %lx, args = %lx, %lx, %lx\n", out, which, arg0, arg1, arg2);
					// Exit on unknown syscall or error condition
					if (which == (uint64_t)-1) {
						printf("Error: Invalid syscall, exiting...\n");
						if (signature_file && sig_begin && sig_end) {
							dump_signature();
						}
						break;
					}
				}
				write_mem(out, ret);
			}
			write_mem(tohost, 0);
			write_mem(fromhost, 1);
		}
	}

	printf("final_cycles %lu\n", (unsigned long)cycle);

	term();

	uint64_t result;
	bool aborted = false;
	bool check_quitted;

	if (tohost != -1)
		result = cpu.gpr[GP];
	else
		result = cpu.gpr[A0];
#if CONFIG_Zicsr
	check_quitted = true;
#else
	check_quitted = false;
#endif

	if (print_result) {
		if (tohost != -1) {
			if (check_quitted) {
				aborted = quitted;
				result = out;
			}

			if (aborted) {
				printf("%s: Execution Aborted\n", elf[0]);
			} else if (result == 1) {
				printf("%s: PASS.\n", elf[0]);
			} else {
				printf("%s: FAIL. test no. = %d\n", elf[0], (uint32_t)result >> 1);
			}
		} else {
			printf("result = %d\n", (uint32_t)result);
		}
	}

	return out;
}

void TestBench::check() {
	// Not used for RV32I
}

uint64_t TestBench::parse_mem_size(const char *str) {
	uint64_t size = 0;
	char *endptr;

	size = strtoull(str, &endptr, 10);

	if (*endptr) {
		char unit = toupper(*endptr);
		switch (unit) {
			case 'G':
				size *= 1024ULL * 1024ULL * 1024ULL;
				break;
			case 'M':
				size *= 1024ULL * 1024ULL;
				break;
			case 'K':
				size *= 1024ULL;
				break;
			default:
				fprintf(stderr, "Unknown memory size unit: %c\n", unit);
				exit(1);
		}
	}

	return size;
}

void TestBench::pass_elf_args(const std::vector<std::string>& elf_args, uint64_t addr) {
	uint64_t elf_argc = 0;
	uint64_t elf_argv = 0;

	// Set up argc/argv for ELF program
	if (!elf_args.empty() || !elf.empty()) {
		// Add program name as argv[0]
		// Always set up argv even if no args, as argv[0] should be program name
		std::vector<std::string> args;
		if (!elf.empty()) {
			// Extract just the filename from the path
			const char* elf_name = strrchr(elf[0], '/');
			if (elf_name) {
				elf_name++;  // Skip the '/'
			} else {
				elf_name = elf[0];
			}
			args.push_back(elf_name);
		}
		// Add user-provided arguments
		args.insert(args.end(), elf_args.begin(), elf_args.end());

		uint64_t argv_base = addr + 2 * 8;	// argc, argv
		uint64_t string_addr = argv_base + (args.size() + 1) * 8;  // After argv array

		elf_argc = args.size();
		elf_argv = argv_base;

		// Write argv array pointers
		for (size_t i = 0; i < args.size(); i++) {
			write_mem(argv_base + i * 8, string_addr);

			// Write string data
			const char* str = args[i].c_str();
			size_t len = args[i].length() + 1;  // Include null terminator

			while (len > 0) {
				uint64_t data = 0;
				size_t bytes = (len > 8) ? 8 : len;
				memcpy(&data, str, bytes);
				write_mem(string_addr, data);
				string_addr += 8;
				str += bytes;
				len -= bytes;
			}

			// Align to 8 bytes for next string
			string_addr = (string_addr + 7) & ~7ULL;
		}

		// NULL terminate argv array
		write_mem(argv_base + args.size() * 8, 0);

	}
	write_mem(addr, elf_argc);
	write_mem(addr + 8, elf_argv);
}

void TestBench::dump_signature() {
	if (!signature_file || !sig_begin || !sig_end) return;
	if (sig_end <= sig_begin) return;

	std::ofstream out(signature_file);
	if (!out) {
		fprintf(stderr, "Failed to open signature file: %s\n", signature_file);
		return;
	}

	fprintf(stderr, "Dumping signature to %s (0x%lx - 0x%lx)\n",
		signature_file, sig_begin, sig_end);

	// Dump memory in 8-byte chunks
	for (uint64_t addr = sig_begin; addr < sig_end; addr += 8) {
		uint64_t data = read_mem(addr);
		// Output in little-endian format, 4 bytes per line
		for (int i = 0; i < 8; i += 4) {
			uint32_t word = (data >> (i * 8)) & 0xFFFFFFFF;
			out << std::hex << std::setw(8) << std::setfill('0') << word << std::endl;
		}
	}

	out.close();
	fprintf(stderr, "Signature dumped successfully\n");
}
