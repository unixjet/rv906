#if !defined(EXTMEM_H)
#define EXTMEM_H

#include <string.h>
#include <stdio.h>
#include "RVProc_io.h"

#define	DONT_CLEAR_BIT_31

////typedef ELF_Parser::Elf32_Shdr	Elf32_Shdr;
////typedef ELF_Parser::Elf64_Shdr	Elf64_Shdr;
typedef struct _MEMCTLPin {	///	memory controller pins!!!
	AXI4L::AXI_AType    addr;
	AXI4L::AXI_DType    din, dout;
	UINT3	    size;
#if 1   /// 12/7/21
    unsigned    cs _BW((1 << RV_BYTE_POS_BW));	///	4/8-bit chip select
#else
    UINT4	    cs;	///	4-bit chip select
#endif
	BIT		    we, ras, cas;
	void set_cs(UINT4 idx) {
		switch (idx) {
		case 0:
		case 2:		cs = 1; break;
		case 1:
		case 3:		cs = 2; break;
		default:	cs = 0; break;
		}
	}
	void set_outpin(struct _MEMCTLPin *xmem_pin) {
		xmem_pin->din = dout;
		xmem_pin->addr = addr;
		xmem_pin->size = size;
		xmem_pin->cs = cs;
		xmem_pin->we = we;
		xmem_pin->cas = cas;
		xmem_pin->ras = ras;
	}
} MEMCTLPin;

typedef MEMCTLPin D_MEMCTLPin _T(direct_signal);

//#if defined(FIX_READ_SETUP_BUG)
struct MEMCTL_AXI4L : AXI4L::SlaveFSM {	///	memory controller!!!!
    MEMCTLPin	mpin _T(state);
    ST_UINT8	readCount;
    AXI4L::AXI_AType   mraddr, mwaddr;
    AXI4L::AXI_DType   din;
    UINT3		mrsize, mwsize;
    //#if defined(ENABLE_AXI_BURST)
    ST_UINT5	burst_count_mr;
    ST_UINT4	burst_count_mw;
    ST_UINT4    read_blen;      /// 4/18/22 : added
    BIT         read_pending;   /// 4/18/22 : added
    BIT devReadSetup(AXI4L::AXI_AType addr, unsigned blen) {
        read_blen = blen;   /// 4/18/22 : added
        read_pending = 1;   /// 4/18/22 : added
        if (w_state != W_Init)
            return 0;
        mraddr = addr;
        burst_count_mr = 0;
        return 1;
    }
    BIT devRead(AXI4L::AXI_DType *data, AXI4L::AXI_AType addr, UINT3 size) {
#define READ_WAIT	2//3//4//4//4
        bool readReady = (readCount >= READ_WAIT);
        *data = din;
        read_pending = (burst_count_mr <= read_blen);   /// 4/18/22 : added
        mraddr = addr + (burst_count_mr << size);
        {
            burst_count_mr++;
            burst_count_mr &= 0x1f;
        }
        mrsize = size;
        readFlag = 1;
        return readReady;
    }
    BIT devWriteSetup(AXI4L::AXI_AType addr) {
        if ((readFlag != 0) || (read_pending != 0))
            return 0;

        mwaddr = addr;
        burst_count_mw = 0;
        return 1;
    }
    BIT devWrite(AXI4L::AXI_DType data, AXI4L::AXI_AType addr, UINT3 size) {
        mpin.dout = data;
        writeFlag = 1;
        mwaddr = addr + (burst_count_mw << size);
        burst_count_mw++;
        burst_count_mw &= 0xf;
        mwsize = size;
        return 1;
    }
    void fsmUser() {
        if (readFlag) { readCount++; }
        else { readCount = 0; }
        /// 4/18/22
        mpin.set_cs((readFlag && read_pending) ? 0 : (writeFlag) ? 0 : 4);
        mpin.we = !readFlag;
        if (readFlag || writeFlag) {	///	these are bogus....
            mpin.cas = 0;
            mpin.ras = 0;
        }
        else {
            mpin.cas = 1;
            mpin.ras = 1;
        }
#if defined(DONT_CLEAR_BIT_31)
        mpin.addr = ((readFlag) ? mraddr : mwaddr);
#else
        mpin.addr = ((readFlag) ? mraddr : mwaddr) & (~MEM_DEV_ADDR_MASK);
#endif
        mpin.size = (readFlag) ? mrsize : mwsize;
    }
    _C2R_FUNC(1)
    void step(AXI4L::CH *axi, D_MEMCTLPin *mem_pin) {
        din = mem_pin->dout;
        mpin.set_outpin(mem_pin);
        mraddr = 0;
        fsm(axi);
    }
};

#define PG_SIZE_BW      (12)
#define PG_SIZE_BYTES   (1 << PG_SIZE_BW)   /// 4096
#define PG_SIZE_WORDS   (1 << (PG_SIZE_BW - RV_BYTE_POS_BW))

#define PN_MASK(PPN_BW) ((1 << (PPN_BW)) - 1)

#define PG_BKT_SIZE_BW  (10)
#define PG_BKT_SIZE     (1 << PG_BKT_SIZE_BW)

#define PPN0_POS    12
#if defined(PROC_64BIT)    /// Sv39
#define PPN0_BW     9
#define PPN1_POS    (PPN0_POS + PPN0_BW)
#define PPN1_BW     9
#define PPN2_POS    (PPN1_POS + PPN1_BW)
#define PPN2_BW     20
#else   /// Sv32
#define PPN0_BW     10
#define PPN1_POS    (PPN0_POS + PPN0_BW)
#define PPN1_BW     12
#endif

#if defined(DONT_CLEAR_BIT_31)   /// 12/7/21 : std::map
#include <map>
//#define TEST_ELF_LOADER /// 4/1/22 : for testing elfLoader
struct ExtMem {
    int cycle;
#if defined(DBG_AXI_SLAVE)  /// 4/18/22
    ST_UINT32   dbg_rcount, dbg_wcount;
    void resetDBGCount() { dbg_rcount = 0; dbg_wcount = 0; }
#endif
    void update(MEMCTLPin *mem_pin) {
        if (mem_pin->cs & 1) {
            mem_access(mem_pin);
#if defined(DBG_AXI_SLAVE)  /// 4/18/22
            if (mem_pin->we) { dbg_wcount++; }
            else             { dbg_rcount++; }
#endif
        }
    }
    // elf loader
    FILE *fp;
#if defined(TEST_ELF_LOADER)   /// 4/1/22 : test elf loader
    ELF_Parser::CharArray elfCharArray;
    ELF_Parser::Loader elfLoader;
#endif
#if	0
    Elf64_Half phnum;
    /// 7/1/20
    std::vector<ELF_Parser::Elf_Phdr> phdr;
#endif

    struct page {
	    union {
	    	RV_UType  m[4096 / sizeof(RV_UType)];
#if (RV_MEM_DATA_WIDTH < AXI_DATA_WIDTH)	// wide data bus
		RV_MType a[4096 / sizeof(RV_MType)];
#else
		AXI4L::AXI_DType  a[4096 / sizeof(AXI4L::AXI_DType)];
#endif
	    };
    };
    std::map<RV_UType, page> pageMap[PG_BKT_SIZE];

    struct page *get_page(RV_UType addr) {
        auto addr0 = addr;
        RV_UType ppn1 = (addr >> PPN1_POS);// &0x3ff;
        RV_UType ppn2 = (addr >> PG_SIZE_BW) & PN_MASK(PG_BKT_SIZE_BW);

        auto &pm = pageMap[ppn2];
        auto pmi = pm.find(ppn1);
        if (pmi != pm.end()) {  /// page @ addr exists...
            return &pm[ppn1];
        }
        auto &pg = pm[ppn1];

#if	0
        addr &= (((RV_UType)(-1)) << PPN0_POS);

        unsigned PT_LOAD_val = ELF_Parser::ptypeGroup.GetValue("LOAD")->value;
        int i;
        for (i = 0; i < phnum; i++) {
            if (phdr[i].p_type == PT_LOAD_val) {
                RV_UType p_start = (RV_UType)phdr[i].p_paddr;
                RV_UType p_end = (p_start + (RV_UType)phdr[i].p_filesz);
                RV_UType p_addr = addr;
                if ((p_addr + 0x1000 > p_start) && (p_addr < p_end)) {
                    RV_UType offset, p_offset;
                    if (p_addr < p_start) {
                        /// 7/1/20
                        offset = p_start - p_addr;
                        p_offset = 0;
                    }
                    else {
                        offset = 0;
                        p_offset = p_addr - p_start;
                    }
                    RV_UType sz = 0x1000 - offset;

                    if (p_start + p_offset + sz > p_end) {
                        /// 7/1/20
                        sz = p_end - (p_start + p_offset);
                    }
                    /// 7/1/20
#if defined(TEST_ELF_LOADER)   /// 4/1/22 : test elf loader
                    elfLoader.elfArray.Seek((long)(phdr[i].p_offset + p_offset));
                    elfLoader.elfArray.Read((char *)pg.m + offset, 1, sz);
                    printf("elfLoader::Read(elfAddr(%08llx), size(%08llx) --> addr(%08llx))\n",
                        (long long)(phdr[i].p_offset + p_offset), (long long)sz, (long long)addr0);
#else
                    fseek(fp, (long)(phdr[i].p_offset + p_offset), SEEK_SET);
                    fread((char *)pg.m + offset, 1, sz, fp);
#endif
                }
            }
        }
#endif
        return &pg;
    }

    void mem_access(MEMCTLPin *mem_pin) {
        struct page *pg = get_page(mem_pin->addr);
        unsigned int pg_addr = (mem_pin->addr) & 0xfff;
#if (RV_MEM_DATA_WIDTH < AXI_DATA_WIDTH)	// wide data bus
        unsigned char *mem = (unsigned char *)pg->a + pg_addr;
        unsigned addrMask = sizeof(mem_pin->din.data) - 1;
        if (mem_pin->we) {
            unsigned char *din = (unsigned char *)&mem_pin->din.data + (mem_pin->addr & addrMask);
            memcpy(mem, din, 1 << mem_pin->size);
        } else {
            unsigned char *dout = (unsigned char *)&mem_pin->dout.data + (mem_pin->addr & addrMask);
            memcpy(dout, mem, 1 << mem_pin->size);
        }
#else
        /// 12/7/21 : for both RV32/RV64
        if (mem_pin->we) {
            unsigned wmask = AXI_BYTE_POS_MASK & (AXI_BYTE_POS_MASK << mem_pin->size);
            unsigned sft = (pg_addr & wmask) * 8;
            switch (mem_pin->size) {
            case 0: /// 1-byte
                ((unsigned char *)pg->a)[pg_addr] = ((unsigned char)(mem_pin->din >> sft));
                break;
            case 1: /// 2-bytes
                ((unsigned short *)pg->a)[pg_addr >> 1] = ((unsigned short)(mem_pin->din >> sft));
                break;
            case 2: /// 4-bytes
                ((unsigned int *)pg->a)[pg_addr >> 2] = ((unsigned int)(mem_pin->din >> sft));
                break;
            case 3: /// 8-bytes (RV64 only)
                pg->a[pg_addr >> 3] = mem_pin->din;
            }
        }
        else {
            mem_pin->dout = pg->a[pg_addr >> AXI_BYTE_POS_BW];
        }
#endif
    }
    void cleanup() {    /// 3/31/22
        if (fp) {
            fclose(fp);
        }
        for (int h = 0; h < PG_BKT_SIZE; h++) {
            pageMap[h].clear();
        }
    }
#if defined(TEST_ELF_LOADER)   /// 4/1/22 : test elf loader
    void load_program_with_elf_loader(const char *filename, RV_UType *pc, RV_UType *sp, int skip) { /// 3/31/22
        cleanup();
        fp = fopen(filename, "rb");
        if (fp == NULL) {
            fprintf(stderr, "Can't open program: %s\n", filename);
            return;
        }
#if 1   /// 3/31/22
#if 1
        elfCharArray.Create(fp);
        elfLoader.Create(elfCharArray.data, elfCharArray.size);
#else
        ELF_Parser::CharArray ca(fp);
        ELF_Parser::Loader elfLoader(ca.data, ca.size);
#endif
        auto &hdr = elfLoader.elfHeader;
        /// 3/31/22
        phnum = elfLoader.elfHeader.ehdr.e_phnum;
        phdr.resize(phnum);
        for (int j = 0; j < phnum; ++j) {
            phdr[j] = elfLoader.segmentTable.phdr[j];
        }
        // get end address.
#if 1
        RV_UType end_addr = (RV_UType)elfLoader.maxAllocAddr;
#else
        RV_UType end_addr = 0;

        unsigned SHF_ALLOC_val = ELF_Parser::shflagGroup.GetValue("A")->value;
        /// 7/1/20
        fseek(fp, (long)hdr.ehdr.e_shoff, SEEK_SET);
        bool isElf64 = hdr.IsElf64();

        ELF_Parser::Elf_Shdr shdr;
        for (int i = 0; i < hdr.ehdr.e_shnum; i++) {
            if (isElf64) {
                ELF_Parser::Elf64_Shdr shdr64;
                fread(&shdr64, sizeof(ELF_Parser::Elf64_Shdr), 1, fp);
                shdr.Copy(&shdr64);
            }
            else {
                ELF_Parser::Elf32_Shdr shdr32;
                fread(&shdr32, sizeof(ELF_Parser::Elf32_Shdr), 1, fp);
                shdr.Copy(&shdr32);
            }
            if (shdr.sh_flags & SHF_ALLOC_val) {
                RV_UType addr = (RV_UType)(shdr.sh_addr + shdr.sh_size);
                if (addr > end_addr)
                    end_addr = addr;
            }
        }
#endif
#if 1   /// 4/1/22
        * pc = (RV_UType)(hdr.ehdr.e_entry + skip);
#define STACK_ALIGN     16//8
        * sp = (RV_UType)(end_addr + 0x1000) & ~(STACK_ALIGN - 1);
#else
        /// 7/1/20
        *pc = (unsigned)(hdr.ehdr.e_entry + skip);
        /// 12/6/21
#define STACK_ALIGN     16//8
        * sp = (unsigned)(end_addr + 0x1000) & ~(STACK_ALIGN - 1);
#endif
#endif
#undef STACK_ALIGN
    }
#endif
    void load_program(const char *filename, RV_UType *pc, RV_UType *sp, int skip) {
#if 1   /// 3/31/22
        cleanup();
#else
        // cleanup
        if (fp) {
            fclose(fp);
        }
        for (int h = 0; h < PG_BKT_SIZE; h++) {
            pageMap[h].clear();
        }
#endif
        //
        fp = fopen(filename, "rb");
        if (fp == NULL) {
            fprintf(stderr, "Can't open program: %s\n", filename);
            return;
        }

#if	0
        /// 7/1/20
        ELF_Parser::Info elfInfo;
        auto &hdr = elfInfo.elfHeader;
        /// 7/1/20
        if (!elfInfo.elfHeader.Parse(fp)) {
            fprintf(stderr, "ELF header error\n");
            fclose(fp);
            fp = NULL;
            return;
        }
        /// 7/1/20
        if (!elfInfo.segmentTable.Parse(&elfInfo, fp)) {
            fprintf(stderr, "ELF pheader error\n");
            fclose(fp);
            fp = NULL;
            return;
        }
        /// 7/1/20
        phnum = elfInfo.elfHeader.ehdr.e_phnum;
        phdr.resize(phnum);
        for (int j = 0; j < phnum; ++j) {
            phdr[j] = elfInfo.segmentTable.phdr[j];
        }
        // get end address.
        RV_UType end_addr = 0;

        unsigned SHF_ALLOC_val = ELF_Parser::shflagGroup.GetValue("A")->value;
        /// 7/1/20
        fseek(fp, (long)hdr.ehdr.e_shoff, SEEK_SET);
        bool isElf64 = hdr.IsElf64();

        ELF_Parser::Elf_Shdr shdr;
        for (int i = 0; i < hdr.ehdr.e_shnum; i++) {
            if (isElf64) {
                ELF_Parser::Elf64_Shdr shdr64;
                fread(&shdr64, sizeof(ELF_Parser::Elf64_Shdr), 1, fp);
                shdr.Copy(&shdr64);
            }
            else {
                ELF_Parser::Elf32_Shdr shdr32;
                fread(&shdr32, sizeof(ELF_Parser::Elf32_Shdr), 1, fp);
                shdr.Copy(&shdr32);
            }
            if (shdr.sh_flags & SHF_ALLOC_val) {
                RV_UType addr = (RV_UType)(shdr.sh_addr + shdr.sh_size);
                if (addr > end_addr)
                    end_addr = addr;
            }
        }
#if 1   /// 4/1/22
        * pc = (RV_UType)(hdr.ehdr.e_entry + skip);
#define STACK_ALIGN     16//8
        * sp = (RV_UType)(end_addr + 0x1000) & ~(STACK_ALIGN - 1);
#else
        /// 7/1/20
        *pc = (unsigned)(hdr.ehdr.e_entry + skip);
        /// 12/6/21
#define STACK_ALIGN     16//8
        * sp = (unsigned)(end_addr + 0x1000) & ~(STACK_ALIGN - 1);
#endif
#endif
    }

	void load_binary(const char *filename, RV_UType *pc, int skip) {
#if 1   /// 3/31/22
        cleanup();
#else
        // cleanup
        if (fp) {
            fclose(fp);
        }
        for (int h = 0; h < PG_BKT_SIZE; h++) {
            pageMap[h].clear();
        }
#endif

#if	0
        phnum = 0;
        phdr.resize(phnum);

		fp = fopen(filename, "rb");
        if (fp == NULL) {
            fprintf(stderr, "Can't open dtb: %s\n", filename);
            return;
        }

		RV_UType load_addr = 0x80000000;
		*pc = load_addr + skip;
		
		while (!feof(fp)) {
			struct page *pg = get_page(load_addr);
			fread(pg->m, 1, 0x1000, fp);
			load_addr += 0x1000;
		}

		fclose(fp);
		fp = NULL;
#endif
	}

#if	0
    void load_file(const char *filename, RV_UType addr, const char *type) {
        FILE *fp;

	if (filename == NULL) return;

        fp = fopen(filename, "rb");
        if (fp == NULL) {
            fprintf(stderr, "Can't open %s: %s\n", type, filename);
            return;
        }

	while (!feof(fp)) {
            struct page *pg = get_page(addr);
            fread(pg->m, 1, 0x1000, fp);
            addr += 0x1000;
	}

        fclose(fp);
    }
    void load_dtb(void *fdt, uint32_t size, RV_UType *dtb) {
	RV_UType addr = 0xbfc00000;
	char *blob = (char *)fdt;

        *dtb = addr;

	while (size) {
	    uint32_t count = size;
	    if (count > 4096)
		count = 4096;
            struct page *pg = get_page(addr);
	    memcpy(pg->m, blob, count);
	    addr += count;
	    blob += count;
	    size -= count;
	}
    }
    void load_dtb(const char *filename, RV_UType *dtb) {
	load_file(filename, 0xbfc00000, "dtb");
        *dtb = 0xbfc00000;
    }
    void load_payload(const char *filename) {
#if defined(PROC_64BIT)
	load_file(filename, 0x80200000, "payload");
#else
	load_file(filename, 0x80400000, "payload");
#endif
    }
    void load_initrd(const char *filename) {
	load_file(filename, 0xb0000000, "initrd");
    }
#endif
#if	0
    void copy_memory(C2R::ElfSymExtractor::Symbol& si, int dir = 1) {
        C2R_ASSERT(dir == 1);   /// read-only for now...
        RV_UType addr = (RV_UType)si.memAddrAligned;
        RV_UType end_addr = addr + (RV_UType)si.memSize;
        while (addr < end_addr) {
            auto* pg = get_page(addr);
            RV_UType pa = addr & (PG_SIZE_BYTES - 1);    /// page internal address
            RV_UType psz = PG_SIZE_BYTES - pa;    /// page size from pa
            if (addr + psz > end_addr) {
                psz = end_addr - addr;
            }
            RV_UType ofs = addr - (RV_UType)si.memAddrAligned;
            RV_UType* v = &si.GetRawValue<RV_UType>() + (ofs >> BPW_BITS);
            RV_UType* dst = (dir == 0) ? &pg->m[pa >> BPW_BITS] : v;
            RV_UType* src = (dir == 1) ? &pg->m[pa >> BPW_BITS] : v;
            for (RV_UType i = 0, ii = 0; i < psz; i += (1 << BPW_BITS), ++ii) {
                if (!si.IsCached(i + ofs)) {
                    dst[ii] = src[ii];
                }
            }
            addr += psz;
        }
    }
#endif
};
#else
struct ExtMem {
	int cycle;
	void update(MEMCTLPin *mem_pin) {
		if (mem_pin->cs & 1) {
			mem_access(/*mem, */mem_pin);
		}
	}
	// elf loader
	FILE *fp;
    Elf32_Half phnum;
    /// 7/1/20
    std::vector<ELF_Parser::Elf_Phdr> phdr;

	struct page {
		struct page *next;
		unsigned int  tag;
		unsigned int  m[4096/sizeof(unsigned int)];
	};
	struct page *bucket[1024];

	struct page *get_page(RV_UType addr) {
#if defined(DONT_CLEAR_BIT_31)
        //unsigned int ppn1 = (addr >> 22) & 0x3ff;
        unsigned int ppn1 = (addr >> PPN1_POS) & 0x3ff;
#else
		unsigned int ppn1 = (addr >> 22) & 0x1ff;
#endif
		//unsigned int ppn2 = (addr >> 12) & 0x3ff;
        unsigned int ppn2 = (addr >> PG_SIZE_BW) & PN_MASK(PG_BKT_SIZE_BW);

		struct page *pg = bucket[ppn2];

		for (pg = bucket[ppn2]; pg; pg = pg->next) {
			if (pg->tag == ppn1) {
				return pg;
			}
		}

		pg = (struct page *)calloc(1, sizeof(struct page));
		pg->tag = ppn1;
		pg->next = bucket[ppn2];
		bucket[ppn2] = pg;

		addr &= 0xfffff000;

#if !defined(DONT_CLEAR_BIT_31)   /// 11/17/21
        C2R_ASSERT((addr & 0x80000000) == 0 || addr == 0xbfc00000); /// msb of addr should be masked out here...
#endif
        unsigned PT_LOAD_val = ELF_Parser::ptypeGroup.GetValue("LOAD")->value;
        int i;
		for (i = 0; i < phnum; i++) {
			if (phdr[i].p_type == PT_LOAD_val) {
#if 1   /// 11/17/21 : program headers may start below 0x80000000 (for elf header??)
                unsigned p_start = (unsigned)phdr[i].p_paddr;
                unsigned p_end = (p_start + (unsigned)phdr[i].p_filesz);
                unsigned p_addr = addr;
#if !defined(DONT_CLEAR_BIT_31)
                if (p_end & 0x80000000) {  /// First program header begins from 0x7ffff000 when start of .text is specified as 0x8000000
                    C2R_ASSERT(i == 0); /// this only happens on the first program header
                    p_addr |= 0x80000000; /// recover msb
                }
#endif
                if ((p_addr + 0x1000 > p_start) && (p_addr < p_end)) {
                    unsigned offset, p_offset;

                    if (p_addr < p_start) {
                        /// 7/1/20
                        offset = p_start - p_addr;
                        p_offset = 0;
                    }
                    else {
                        offset = 0;
                        p_offset = p_addr - p_start;
                    }
                    unsigned sz = 0x1000 - offset;

                    if (p_start + p_offset + sz > p_end) {
                        /// 7/1/20
                        sz = p_end - (p_start + p_offset);
                    }
                    /// 7/1/20
                    fseek(fp, (long)(phdr[i].p_offset + p_offset), SEEK_SET);
                    fread((char *)pg->m + offset, 1, sz, fp);
                }
#else
				if ((addr < phdr[i].p_paddr + phdr[i].p_filesz) && 
				    (addr + 0x1000 > phdr[i].p_paddr)) {
					int offset;

					if (addr < phdr[i].p_paddr) {
                        /// 7/1/20
                        offset = (int)(phdr[i].p_paddr - addr);
						addr = 0;
					} else {
						offset = 0;
                        /// 7/1/20
                        addr -= (int)phdr[i].p_paddr;
					}
					int sz = 0x1000 - offset;

                    if (addr + sz > phdr[i].p_filesz) {
                        /// 7/1/20
                        sz = (int)(phdr[i].p_filesz - addr);
                    }
                    /// 7/1/20
                    fseek(fp, (long)(phdr[i].p_offset + addr), SEEK_SET);
					fread((char *)pg->m + offset, 1, sz, fp);
				}
#endif
            }
		}

		return pg;
	}

	void mem_access(MEMCTLPin *mem_pin) {
		struct page *pg = get_page(mem_pin->addr);
		unsigned int addr;

		addr = (mem_pin->addr) & 0xfff;
		if (mem_pin->we) {
			switch (mem_pin->size) {
			case 0:
				((unsigned char *)pg->m)[addr] = mem_pin->din >> ((addr & 3) * 8);
				break;
			case 1:
				((unsigned short *)pg->m)[addr >> 1] = mem_pin->din >> ((addr & 2) * 8);
				break;
			case 2:
				pg->m[addr >> 2] = mem_pin->din;
				break;
			}
		} else {
			mem_pin->dout = pg->m[addr >> 2];
		}
	}

    void load_program(const char *filename, unsigned int *pc, unsigned int *sp, int skip) {
        // cleanup
        if (fp)
            fclose(fp);
        int h;
        for (h = 0; h < 1024; h++) {
            while (bucket[h]) {
                struct page *pg = bucket[h];

                bucket[h] = pg->next;
                free(pg);
            }
        }

        //
        fp = fopen(filename, "rb");
        if (fp == NULL) {
            fprintf(stderr, "Can't open program: %s\n", filename);
            return;
        }

        /// 7/1/20
        ELF_Parser::Info elfInfo;
        auto &hdr = elfInfo.elfHeader;
        /// 7/1/20
        if (!elfInfo.elfHeader.Parse(fp)) {
            fprintf(stderr, "ELF header error\n");
            fclose(fp);
            fp = NULL;
            return;
        }
        /// 7/1/20
        if (!elfInfo.segmentTable.Parse(&elfInfo, fp)) {
            fprintf(stderr, "ELF pheader error\n");
            fclose(fp);
            fp = NULL;
            return;
        }
        /// 7/1/20
        phnum = elfInfo.elfHeader.ehdr.e_phnum;
        phdr.resize(phnum);
        for (int j = 0; j < phnum; ++j) {
            phdr[j] = elfInfo.segmentTable.phdr[j];
        }
#if !defined(DONT_CLEAR_BIT_31)
        for (int i = 0; i < phnum; i++) {
            phdr[i].p_paddr &= 0x7fffffff;
        }
#endif
        // get end address.
        unsigned int end_addr = 0;

        unsigned SHF_ALLOC_val = ELF_Parser::shflagGroup.GetValue("A")->value;
        /// 7/1/20
        fseek(fp, (long)hdr.ehdr.e_shoff, SEEK_SET);
        for (int i = 0; i < hdr.ehdr.e_shnum; i++) {
            Elf32_Shdr shdr;

            fread(&shdr, sizeof(Elf32_Shdr), 1, fp);
            if (shdr.sh_flags & SHF_ALLOC_val) {
                unsigned int addr;

                addr = shdr.sh_addr + shdr.sh_size;
                if (addr > end_addr)
                    end_addr = addr;
            }
        }
        /// 7/1/20
        *pc = (unsigned)(hdr.ehdr.e_entry + skip);
#if 1   /// 12/6/21
#define STACK_ALIGN     16//8
        * sp = (unsigned)(end_addr + 0x1000) & ~(STACK_ALIGN - 1);
#else
        *sp = (unsigned)(end_addr + 0x1000);
#endif

    }

	void load_dtb(const char *filename, unsigned int *dtb) {
		FILE *fp;

		fp = fopen(filename, "rb");
		if (fp == NULL) {
			fprintf(stderr, "Can't open dtb: %s\n", filename);
			return;
		}

		struct page *pg = get_page(0xbfc00000);
		fread(pg->m, 1, 0x1000, fp);

		*dtb = 0xbfc00000;

		fclose(fp);
	}
};
#endif

#if	0
struct ExtMemNV {
    int cycle;
#if defined(DBG_AXI_SLAVE)  /// 4/18/22
    ST_UINT32   dbg_rcount, dbg_wcount;
    void resetDBGCount() { dbg_rcount = 0; dbg_wcount = 0; }
#endif
    RV_UType addrOffset;
    size_t size;
    char * mem;
    ExtMemNV(RV_UType addrofs) : cycle(0), addrOffset(addrofs), mem(nullptr) {}
    virtual ~ExtMemNV() { cleanup(); }
    void cleanup() { if (mem) { delete[] mem; } mem = nullptr; }
    void load_file(const char *filename) {
        cleanup();
        if (FILE *fp = fopen(filename, "rb")) {
            fseek(fp, 0L, SEEK_END);
            size = ftell(fp);
            mem = new char[size];
            rewind(fp);
            fread(mem, 1, size, fp);
            printf("ExtMemNV::load_file(%s) : size = %08" PF_INT "x\n", filename, (RV_UType)size);
        }
    }
    void update(MEMCTLPin *mem_pin) {
        if (mem_pin->cs & 1) {
            mem_access(mem_pin);
#if defined(DBG_AXI_SLAVE)  /// 4/18/22
            if (mem_pin->we) { dbg_wcount++; }
            else             { dbg_rcount++; }
#endif
        }
    }
    void mem_access(MEMCTLPin *mem_pin) {
        auto addr = mem_pin->addr - addrOffset;
#if 1   /// 4/18/22 : debugging...
        if (addr >= (RV_UType)size) {
            printf("ERROR in ExtMemNV::mem_access : addr = %08" PF_INT "x, size = %08" PF_INT "x\n", addr, (RV_UType)size);
        }
#endif
        C2R_ASSERT(addr < (RV_UType)size);
        if (mem_pin->we) {
            unsigned wmask = RV_BYTE_POS_MASK & (RV_BYTE_POS_MASK << mem_pin->size);
            unsigned sft = (addr & wmask) * 8;
            switch (mem_pin->size) {
            case 0: /// 1-byte
                ((unsigned char *)mem)[addr] = ((unsigned char)(mem_pin->din >> sft));
                break;
            case 1: /// 2-bytes
                ((unsigned short *)mem)[addr >> 1] = ((unsigned short)(mem_pin->din >> sft));
                break;
            case 2: /// 4-bytes
                ((unsigned int *)mem)[addr >> 2] = ((unsigned int)(mem_pin->din >> sft));
                break;
            case 3: /// 8-bytes (RV64 only)
                ((unsigned long long *)mem)[addr >> 3] = (unsigned long long)mem_pin->din;
                break;
            }
        }
        else {
            mem_pin->dout = ((RV_UType*)mem)[addr >> RV_BYTE_POS_BW];
        }
    }
};
#endif

#endif  /// !defined(EXTMEM_H)
