/*Copyright 2020-2021 T-Head Semiconductor Co., Ltd.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/
/*
 * syscalls.c -- M8 T4b (rv906 glue, NEW file; the donor tree's clib has
 * no equivalent): newlib bare-metal syscall stubs for the coremark case.
 *
 * Why: the rv906 toolchain's (xpack) newlib pulls its reent syscall layer
 * (_write_r, _sbrk_r, ...) whenever stdio objects (printf/puts/memset)
 * are linked from -lc. The donor's T-Head newlib clearly did not require
 * any of them (T6's donor link line "-- ... -lc -lgcc <objs> -lm" linked
 * clean with no stub file). The stubs are per-platform glue (design doc
 * section 4.6: glue is per-platform; case bodies stay byte-identical).
 *
 * Console behavior (design ledger L12): stdout must reach the verisim
 * console. Two newlib paths can carry stdout, and both are covered:
 *   - the fputc path (fp->_file == -1 -> newlib sfwritr calls fputc):
 *     the linked fputc is clib/printf.c's, which stores THR 0x10000000
 *     (device/uart16550.cpp -> ttysrv::out -> verisim stdout);
 *   - the _write path (fp->_file != -1): _write below stores the SAME
 *     THR 0x10000000, so the console works either way.
 * _isatty returns 1 so stdout init picks LINE buffering: each '\n'
 * flushes, so the VCUNT_SIM lines appear as printed instead of sitting
 * in a full-mode buffer that may never fill.
 * _exit maps to crt0 __fail (tohost = 3): if newlib's abort() is ever
 * reached the run reports FAIL via the tohost protocol instead of
 * hanging to the run_all wall cap.
 * Everything else is the standard no-op stub.
 */
#include <stdint.h>
#include <stddef.h>
#include <sys/types.h>
#include <sys/stat.h>

/* 16550 THR, rv906 console (rtl/RVProcAXI.v:183 UART_BASE 0x10000000). */
#define THR_ADDR ((volatile uint32_t *)0x10000000u)

int _close(int file)
{
  (void)file;
  return -1;
}

int _fstat(int file, struct stat *st)
{
  (void)file;
  st->st_mode = S_IFCHR;	/* console: character device */
  return 0;
}

int _getpid(void)
{
  return 1;
}

int _isatty(int file)
{
  (void)file;
  return 1;			/* line-buffered stdout (see header) */
}

int _kill(int pid, int sig)
{
  (void)pid;
  (void)sig;
  return -1;
}

off_t _lseek(int file, off_t offset, int whence)
{
  (void)file;
  (void)offset;
  (void)whence;
  return 0;
}

ssize_t _read(int file, void *ptr, size_t len)
{
  (void)file;
  (void)ptr;
  (void)len;
  return 0;			/* stdin: immediate EOF */
}

/* newlib's abort() calls _exit; report it through the tohost protocol. */
extern void __fail(void);	/* crt0_m8.s: tohost = 3 (FAIL), never returns */
void _exit(int status)
{
  (void)status;
  __fail();
}

static char heap[8192];
static char *heap_end;

void *_sbrk(ptrdiff_t incr)
{
  char *prev = heap_end;
  if (heap_end == NULL)
    heap_end = heap;
  if (heap_end + incr > heap + sizeof(heap))
    return (void *)-1;
  heap_end += incr;
  return prev;
}

ssize_t _write(int file, const void *ptr, size_t len)
{
  const uint8_t *p = ptr;
  size_t i;
  (void)file;
  for (i = 0; i < len; i++)
    *THR_ADDR = p[i];
  return (ssize_t)len;
}
