#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <gelf.h>

#include "load_elf.h"

#define	PAGE_SIZE	0x1000

static void load(int fd, std::function<void*(uint64_t)> get_page, uint64_t paddr, ssize_t size)
{
	while (size) {
		void *page = get_page(paddr);
		ssize_t count = size;

		if (count > PAGE_SIZE)
			count = PAGE_SIZE;
		count = read(fd, page, count);
		if (count < 0)
			break;
		size -= count;
		paddr += count;
	}
}

static void load(int fd, std::function<void(uint64_t, uint8_t)> write_byte, uint64_t paddr, ssize_t size)
{
	uint8_t buf[PAGE_SIZE];
	while (size) {
		ssize_t count = size > PAGE_SIZE ? PAGE_SIZE : size;
		count = read(fd, buf, count);
		if (count < 0)
			break;
		for (ssize_t i = 0; i < count; i++)
			write_byte(paddr + i, buf[i]);
		size -= count;
		paddr += count;
	}
}

static void find_symbols(Elf *elf, struct symbol_table *syms)
{
	Elf_Scn *scn = NULL;
	GElf_Shdr shdr;
	Elf_Data *data;
	int count = 0;

	while ((scn = elf_nextscn(elf, scn)) != NULL) {
		if (gelf_getshdr(scn, &shdr) != &shdr) {
			continue;
		}
		if (shdr.sh_type == SHT_SYMTAB) {
			data = elf_getdata(scn, NULL);
			count = shdr.sh_size / shdr.sh_entsize;
			break;
		}
	}
	for (; syms->name; syms++) {
		for (int i = 0; i < count; i++) {
			GElf_Sym sym;

			gelf_getsym(data, i, &sym);
			if (strcmp(syms->name, elf_strptr(elf, shdr.sh_link, sym.st_name)) == 0) {
				*syms->value = sym.st_value;
			}
		}
	}
}

bool load_elf(const char *filename, std::function<void*(uint64_t)> get_page, uint64_t *initial_pc, struct symbol_table *syms)
{
	int fd = open(filename, O_RDONLY, 0);
	Elf *elf;
	GElf_Ehdr ehdr;
	bool ret = false;
	size_t n;
	int i;

	if (elf_version(EV_CURRENT) == EV_NONE) {
		fprintf(stderr, "%s: ELF library initialization failed.\n", __func__);
		return ret;
	}

	if (fd < 0) {
		fprintf(stderr, "%s: can't open %s\n", __func__, filename);
		return ret;
	}

	elf = elf_begin(fd, ELF_C_READ, NULL);
	if (elf == NULL) {
		fprintf(stderr, "%s: elf_begin() failed\n", __func__);
		goto close_fd;
	}

	if (elf_kind(elf) != ELF_K_ELF) {
		fprintf(stderr, "%s: %s is not ELF file\n", __func__, filename);
		goto close_elf;
	}

	if (gelf_getehdr(elf, &ehdr) != &ehdr) {
		fprintf(stderr, "%s: %s has no Ehdr\n", __func__, filename);
		goto close_elf;
	}
	*initial_pc = ehdr.e_entry;

	if (elf_getphdrnum(elf, &n) != 0) {
		fprintf(stderr, "%s: elf_getphdrnum() failed: %s.\n", __func__, elf_errmsg(-1));
		goto close_elf;
	}
	for (i = 0; i < n; i++) {
		GElf_Phdr phdr;

		if (gelf_getphdr(elf, i, &phdr) != &phdr) {
			fprintf(stderr, "%s: getphdr() failed: %s.\n", __func__, elf_errmsg(-1));
			continue;
		}
		if (phdr.p_type == PT_LOAD) {
			lseek(fd, phdr.p_offset, SEEK_SET);
			//load(fd, xmem, phdr.p_paddr, phdr.p_filesz);
			load(fd, get_page, phdr.p_paddr, phdr.p_filesz);
		}
	}

	if (syms)
		find_symbols(elf, syms);

	ret = true;
close_elf:
	elf_end(elf);
close_fd:
	close(fd);
	return ret;
}

bool load_bin(const char *filename, std::function<void*(uint64_t)> get_page, uint64_t addr)
{
	bool ret = false;
	struct stat sb;
	int fd = open(filename, O_RDONLY, 0);

	if (fd < 0) {
		fprintf(stderr, "%s: can't open %s\n", __func__, filename);
		return false;
	}

	if (fstat(fd, &sb) < 0) {
		fprintf(stderr, "%s: can't stat %s\n", __func__, filename);
		goto close_fd;
	}

	load(fd, get_page, addr, sb.st_size);

	ret = true;
close_fd:
	close(fd);
	return ret;
}

bool load_elf(const char *filename, std::function<void(uint64_t, uint8_t)> write_byte, uint64_t *initial_pc, struct symbol_table *syms)
{
	int fd = open(filename, O_RDONLY, 0);
	Elf *elf;
	GElf_Ehdr ehdr;
	bool ret = false;
	size_t n;
	int i;

	if (elf_version(EV_CURRENT) == EV_NONE) {
		fprintf(stderr, "%s: ELF library initialization failed.\n", __func__);
		return ret;
	}

	if (fd < 0) {
		fprintf(stderr, "%s: can't open %s\n", __func__, filename);
		return ret;
	}

	elf = elf_begin(fd, ELF_C_READ, NULL);
	if (elf == NULL) {
		fprintf(stderr, "%s: elf_begin() failed\n", __func__);
		goto close_fd;
	}

	if (elf_kind(elf) != ELF_K_ELF) {
		fprintf(stderr, "%s: %s is not ELF file\n", __func__, filename);
		goto close_elf;
	}

	if (gelf_getehdr(elf, &ehdr) != &ehdr) {
		fprintf(stderr, "%s: %s has no Ehdr\n", __func__, filename);
		goto close_elf;
	}
	*initial_pc = ehdr.e_entry;

	if (elf_getphdrnum(elf, &n) != 0) {
		fprintf(stderr, "%s: elf_getphdrnum() failed: %s.\n", __func__, elf_errmsg(-1));
		goto close_elf;
	}
	for (i = 0; i < (int)n; i++) {
		GElf_Phdr phdr;

		if (gelf_getphdr(elf, i, &phdr) != &phdr) {
			fprintf(stderr, "%s: getphdr() failed: %s.\n", __func__, elf_errmsg(-1));
			continue;
		}
		if (phdr.p_type == PT_LOAD) {
			lseek(fd, phdr.p_offset, SEEK_SET);
			load(fd, write_byte, phdr.p_paddr, phdr.p_filesz);
		}
	}

	if (syms)
		find_symbols(elf, syms);

	ret = true;
close_elf:
	elf_end(elf);
close_fd:
	close(fd);
	return ret;
}

bool load_bin(const char *filename, std::function<void(uint64_t, uint8_t)> write_byte, uint64_t addr)
{
	bool ret = false;
	struct stat sb;
	int fd = open(filename, O_RDONLY, 0);

	if (fd < 0) {
		fprintf(stderr, "%s: can't open %s\n", __func__, filename);
		return false;
	}

	if (fstat(fd, &sb) < 0) {
		fprintf(stderr, "%s: can't stat %s\n", __func__, filename);
		goto close_fd;
	}

	load(fd, write_byte, addr, sb.st_size);

	ret = true;
close_fd:
	close(fd);
	return ret;
}
