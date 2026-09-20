# test/m8 -- M8 cross-check vs the original openc906 RTL

Design: `docs/superpowers/specs/2026-09-21-m8-crosscheck-design.md`
(run notes: `docs/superpowers/specs/notes/2026-09-21-m8-t1-dryrun-results.md`,
`2026-09-21-m8-t6-donor-runs.md`).

Runs the same smart_run non-vector case programs on BOTH sides:

- **donor** openc906 RTL under iverilog (`smart_run` flow, reusing
  `refs/openc906/smart_run/work/xuantie_core.vvp` -- never recompiled)
- **rv906** under the Verilator testbench (tohost protocol)

and gates on functional parity (same verdict both sides). Cycle counts
are captured and reported, never gated (D-M8-6).

## Files

| File | Role |
|---|---|
| `crt0_m8.s` | ported donor `tests/lib/crt0.s` -- poke stream byte-identical (incl. the no-op mcor/mhint pokes, R4 correction); `__exit`->tohost=1, `__fail`->tohost=3 (L13/L15) |
| `link_m8.ld` | ported donor `linker.lcf` -- image @ 0x80000000, .tohost @ 0x7FFFF000, `__kernel_stack` @ top of the 1 MB window at 0x80100000, 0x800100000 page-table hole documented |
| `clib/` | ported donor `tests/lib/clib/` -- ONLY deltas: `fputc.c` console 0x6000fff8 -> 0x10000000, `printf.c:39` `lrw` -> standard `li`/`sw` (L12) |
| `cases/csr/` | byte-identical donor `C906_CSR_OPERATION.s` (md5-verified at T3) |
| `Makefile` | xpack cross-compile; march = donor's T1-patched string; coremark CFLAGS per design 4.5 (both sides, minus the vendor flags) |
| `run_all.sh` | rv906-side runner; writes `records/<case>.rv906.json` |
| `run_donor.sh` | donor-side runner (buildcase + reused vvp); writes `records/<case>.donor.json` |
| `compare.py` | parity gate + cycle table; `--selftest` = D-M8-7.4 negative check; `--strict` = T7 mode |
| `records/` | per-run JSON records + raw logs (compare.py input) |

## Usage

```sh
make -C test/m8                 # build case ELFs
bash test/m8/run_all.sh         # rv906 side (writes records)
bash test/m8/run_donor.sh csr   # donor side (one case; writes records)
python3 test/m8/compare.py      # parity table (missing = PENDING)
python3 test/m8/compare.py --strict   # T7: all final-set pairs must exist
python3 test/m8/compare.py --selftest # negative check
```

Final parity set (T6 declaration): `{csr, interrupt, MMU}` + coremark
pending; exception dropped (donor TEST FAIL, R1 source-inherent).
