#ifndef _RVPROC_H_
#define _RVPROC_H_

// rv906: pure-RTL simulation -- no C model. This shim provides the type
// definitions the ported test framework expects. CoreState stands in for
// a C-model core object: TestBench only ever reads gpr[].
#include "RVProcArch.h"

struct CoreState {
	RV_UType gpr[32];
};

#endif // _RVPROC_H_
