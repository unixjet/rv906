#include <sys/stat.h>
#include <fcntl.h>
#if !defined(_MSC_VER)
#include <unistd.h>
#endif
#include <stdio.h>
#include "fdt.h"

FDT::~FDT()
{
    free(blob);
}

int FDT::create(size_t size)
{
    blob = malloc(size);
    int ret = fdt_create_empty_tree(blob, size);
    if (ret < 0) {
        free(blob);
        blob = NULL;
    }
    return ret;
}

FDT::Node FDT::open(const char *name)
{
    return Node(fdt_path_offset(blob, name), blob);
}
