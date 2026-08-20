// iss_tb.cpp - standalone driver for the M1 fetch-ISS's own unit test
// (plan Task 5.3). Deliberately NOT a verilator bench: the whole point of
// this gate is to prove m1_iss.h correct with ZERO RTL involved, before it
// is ever trusted to grade FetchSink/IFU/ICache in Task 6. Plain g++/clang++
// build, no verilator, no DUT.
//
//   make -C test/m1/unit iss
//   ./bin/unit/iss_tb
//
// The real work (the golden image, the hand-derived expected commit
// sequence, and the direct JR_TARGET/direction-rule checks) lives in
// m1_iss.h's selftest() -- this file is just the entry point.
#include "../../../m1_iss.h"

int main()
{
    return m1::selftest() ? 0 : 1;
}
