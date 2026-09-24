#!/usr/bin/env python3
"""M8 parity comparator (design 2026-09-21-m8-crosscheck-design.md
section 4.6 / D-M8-7).

Reads the per-case JSON records written by run_all.sh (side "rv906") and
run_donor.sh (side "donor") from test/m8/records/ and gates on verdict
equality for every final-set case. Cycles are printed as a column
(D-M8-6: noted, NEVER gated).

Final parity set -- T6 declaration
(docs/superpowers/specs/notes/2026-09-21-m8-t6-donor-runs.md section 7):
    {csr, interrupt, MMU, coremark}  (donor TEST PASS; coremark entered
    the set when its donor build unblocked and it PASSed at 435894.5 cyc)
    exception              (donor TEST FAIL, R1 source-inherent: dropped)

Usage:
    python3 test/m8/compare.py            # compare what is recorded
                                          # (missing = PENDING, not fatal)
    python3 test/m8/compare.py --strict   # T7 mode: every final-set case
                                          # must have both records present
    python3 test/m8/compare.py --selftest # D-M8-7.4 negative check: a
                                          # synthesized mismatch record
                                          # must be flagged

Exit codes:
    0  all present final-set pairs verdict-equal (strict: all present)
    1  a verdict mismatch / non-PASS verdict was found
    2  a final-set case has missing records (strict mode only)
"""

import json
import os
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
RECORDS = os.path.join(HERE, "records")

FINAL_SET = ["csr", "interrupt", "MMU", "coremark"]
PENDING = []   # coremark entered the set once the donor run PASSed (T6)


def load_record(recdir, case, side):
    path = os.path.join(recdir, "%s.%s.json" % (case, side))
    with open(path) as f:
        rec = json.load(f)
    assert rec.get("side") == side, "side field mismatch in %s" % path
    return rec


def compare_pair(recdir, case):
    """Return (status, row) where status is one of:
       'ok' (both PASS), 'fail' (verdicts differ or either FAIL),
       'missing' (one/both records absent)."""
    rv906 = donor = None
    try:
        rv906 = load_record(recdir, case, "rv906")
    except (OSError, ValueError, AssertionError):
        pass
    try:
        donor = load_record(recdir, case, "donor")
    except (OSError, ValueError, AssertionError):
        pass
    if rv906 is None or donor is None:
        present = []
        if rv906 is None:
            present.append("rv906")
        if donor is None:
            present.append("donor")
        return "missing", (case, rv906, donor, "missing: " + "/".join(present))
    r_v, d_v = rv906.get("verdict"), donor.get("verdict")
    if r_v == "PASS" and d_v == "PASS":
        status = "ok"
        note = "PASS both sides"
    elif r_v == d_v:
        status = "fail"
        note = "identical non-PASS verdict: %s" % r_v
    else:
        status = "fail"
        note = "verdict mismatch: rv906=%s donor=%s" % (r_v, d_v)
    return status, (case, rv906, donor, note)


def render_table(rows):
    out = []
    out.append("%-12s %-28s %-28s %s" %
               ("case", "rv906 (verdict/cycles)", "donor (verdict/cycles)", "status"))
    out.append("-" * 92)

    def fmt(rec):
        if rec is None:
            return "-"
        cyc = rec.get("cycles")
        return "%s / %s" % (rec.get("verdict"), cyc if cyc is not None else "-")

    for case, rv906, donor, note in rows:
        out.append("%-12s %-28s %-28s %s" % (case, fmt(rv906), fmt(donor), note))
    return "\n".join(out)


def compare(recdir=RECORDS, cases=None, strict=True, verbose=True):
    """Compare; return process exit code (0 ok / 1 mismatch / 2 missing-strict)."""
    cases = list(cases) if cases is not None else FINAL_SET + PENDING
    rows = []
    n_fail = 0
    n_missing = 0
    for case in cases:
        status, row = compare_pair(recdir, case)
        rows.append(row)
        if status == "fail":
            n_fail += 1
        elif status == "missing":
            n_missing += 1
            if not strict:
                rows[-1] = (case, row[1], row[2], "PENDING " + row[3])
    if verbose:
        print(render_table(rows))
        print()
        print("cycle column is annotated, never gated (D-M8-6)")
        if n_fail:
            print("FAIL: %d case(s) with mismatched/non-PASS verdicts" % n_fail)
        if n_missing:
            if strict:
                print("FAIL: %d final-set case(s) missing records" % n_missing)
            else:
                print("NOTE: %d case(s) not yet run on one/both sides" % n_missing)
    if n_fail:
        return 1
    if n_missing and strict:
        return 2
    return 0


def write_pair(recdir, case, rv906_verdict, rv906_cyc, donor_verdict, donor_cyc):
    for side, verdict, cyc in (("rv906", rv906_verdict, rv906_cyc),
                               ("donor", donor_verdict, donor_cyc)):
        rec = {"case": case, "side": side, "verdict": verdict, "cycles": cyc,
               "console_excerpt": "", "toolchain": "selftest", "march": "selftest",
               "deltas": []}
        with open(os.path.join(recdir, "%s.%s.json" % (case, side)), "w") as f:
            json.dump(rec, f)


def selftest():
    """D-M8-7.4 negative check: the comparator must FLAG a synthesized
    verdict mismatch and ACCEPT a matching pair (with deliberately
    different cycles -- cycles are never gated)."""
    ok = True
    with tempfile.TemporaryDirectory(prefix="m8_compare_selftest_") as td:
        # control: matching pair, different cycles -> must pass
        write_pair(td, "good", "PASS", 345, "PASS", 2736.5)
        rc_good = compare(td, ["good"], strict=True, verbose=False)
        # negative: synthesized mismatch -> must be flagged (exit 1)
        write_pair(td, "bad", "PASS", 345, "FAIL", 13293.5)
        rc_bad = compare(td, ["bad"], strict=True, verbose=False)
        rc_bad_verbose = compare(td, ["bad"], strict=True, verbose=True)
        # mixed dir -> overall failure must surface
        rc_mixed = compare(td, ["good", "bad"], strict=True, verbose=False)

        print("selftest: control  pair  rc=%d (want 0)  %s" %
              (rc_good, "OK" if rc_good == 0 else "BROKEN"))
        ok &= (rc_good == 0)
        print("selftest: mismatch pair  rc=%d (want 1)  %s" %
              (rc_bad, "OK" if rc_bad == 1 else "BROKEN"))
        ok &= (rc_bad == 1)
        print("selftest: mixed dir    rc=%d (want 1)  %s" %
              (rc_mixed, "OK" if rc_mixed == 1 else "BROKEN"))
        ok &= (rc_mixed == 1)
    print("selftest: %s" % ("PASS -- negative check works" if ok
                            else "FAIL -- comparator is broken"))
    return 0 if ok else 1


def main(argv):
    recdir = RECORDS
    strict = False   # default: report missing as PENDING; --strict = T7 mode
    selftest_mode = False
    for arg in argv:
        if arg == "--selftest":
            selftest_mode = True
        elif arg == "--strict":
            strict = True
        elif arg == "--non-strict":
            strict = False
        elif arg.startswith("--records="):
            recdir = arg.split("=", 1)[1]
        else:
            print(__doc__)
            return 2
    if selftest_mode:
        return selftest()
    rc = compare(recdir, strict=strict, verbose=True)
    return rc


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
