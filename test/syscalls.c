// syscalls.c - Syscall stubs for newlib
#include <sys/stat.h>
#include <stdint.h>
#include "uart.h"

// errno
int errno;

// Heap pointers from linker script
extern char _heap_start;
extern char _heap_end;
static char *heap_ptr = 0;

void *_sbrk(ptrdiff_t incr) {
    if (heap_ptr == 0) {
        heap_ptr = &_heap_start;
    }
    char *prev = heap_ptr;
    if (heap_ptr + incr > &_heap_end) {
        errno = 12;  // ENOMEM
        return (void *)-1;
    }
    heap_ptr += incr;
    return prev;
}

int _close(int fd) {
    return -1;
}

int _read(int fd, char *buf, int len) {
    return 0;
}

int _write(int fd, char *buf, int len) {
    for (int i = 0; i < len; i++) {
        uart_putc(buf[i]);
    }
    return len;
}

int _lseek(int fd, int offset, int whence) {
    return 0;
}

int _fstat(int fd, struct stat *st) {
    st->st_mode = S_IFCHR;
    return 0;
}

int _isatty(int fd) {
    return 1;
}

int _kill(int pid, int sig) {
    errno = 22;  // EINVAL
    return -1;
}

int _getpid(void) {
    return 1;
}
