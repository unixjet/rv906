#ifndef	__FDT_H__
#define	__FDT_H__

#include <sys/stat.h>
#include <libfdt.h>

struct FDT {
    struct Node {
        int offset;
	void *fdt;

	Node() {}
	Node(int o, void *blob):offset(o), fdt(blob) {}

	Node create(const char *name) {
            return Node(fdt_add_subnode(fdt, offset, name), fdt);
        }
	int setprop(const char *name, uint64_t val) {
            return fdt_setprop_u64(fdt, offset, name, val);
        }
	int setprop(const char *name, uint32_t val) {
            return fdt_setprop_u32(fdt, offset, name, val);
        }
	int setprop(const char *name, const char *val) {
            return fdt_setprop_string(fdt, offset, name, val);
	}
	int setprop(const char *name) {
            return fdt_setprop_empty(fdt, offset, name);
	}
        int setprop(const char *name, int n, uint32_t **prop_data) {
            return fdt_setprop_placeholder(fdt, offset, name,
		        n * sizeof(**prop_data), (void **)prop_data);
	}
        int setprop(const char *name, int n, uint64_t **prop_data) {
            return fdt_setprop_placeholder(fdt, offset, name,
		        n * sizeof(**prop_data), (void **)prop_data);
	}

        uint32_t get_phandle() {
	    uint32_t phandle = fdt_get_phandle(fdt, offset);

	    if (phandle == 0) {
                fdt_generate_phandle(fdt, &phandle);
                fdt_setprop_cell(fdt, offset, "phandle", phandle);
	    }
            return phandle;
	}
    };

    void *blob;

    ~FDT();

    int create(size_t size);
    Node open(const char *name);
    int pack() {
	return fdt_pack(blob);
    }
    uint32_t totalsize() {
	return fdt_totalsize(blob);
    }
};

#endif	// __FDT_H__
