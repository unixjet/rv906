#ifndef	_RVPROC_ARCH_H_
#define	_RVPROC_ARCH_H_

#include "C2Rdef.h"
#include "config.h"

enum {
	ZERO = 0,
	RA = 1,
	SP = 2,
	GP = 3,
	TP = 4,
	T0 = 5,
	T1 = 6,
	T2 = 7,
	S0 = 8,
	FP = 8,
	S1 = 9,
	A0 = 10,
	A1 = 11,
	A2 = 12,
	A3 = 13,
	A4 = 14,
	A5 = 15,
	A6 = 16,
	A7 = 17,
	S2 = 18,
	S3 = 19,
	S4 = 20,
	S5 = 21,
	S6 = 22,
	S7 = 23,
	S8 = 24,
	S9 = 25,
	S10 = 26,
	S11 = 27,
	T3 = 28,
	T4 = 29,
	T5 = 30,
	T6 = 31,
};

enum OpCode {
	OPC_load      = 0x03,
	OPC_load_fp   = 0x07,
	OPC_custom_0  = 0x0b,
	OPC_misc_mem  = 0x0f,
	OPC_op_imm    = 0x13,
	OPC_auipc     = 0x17,
	OPC_op_imm_32 = 0x1b,
	// 48b
	OPC_store     = 0x23,
	OPC_store_fp  = 0x27,
	OPC_custom_1  = 0x2b,
	OPC_amo       = 0x2f,
	OPC_op        = 0x33,
	OPC_lui       = 0x37,
	OPC_op_32     = 0x3b,
	// 64b
	OPC_madd      = 0x43,
	OPC_msub      = 0x47,
	OPC_nmsub     = 0x4b,
	OPC_nmadd     = 0x4f,
	OPC_op_fp     = 0x53,
	OPC_op_v      = 0x57,
	OPC_custom_2  = 0x5b,
	// 48b
	OPC_branch    = 0x63,
	OPC_jalr      = 0x67,
	// reserved
	OPC_jal       = 0x6f,
	OPC_system    = 0x73,
	OPC_op_ve     = 0x77,
	OPC_custom_3  = 0x7b,
	// >=80b
};

enum {
	BEQ = 0,
	BNE = 1,
	BF  = 2,
	BT  = 3,
	BLT = 4,
	BGE = 5,
	BLTU = 6,
	BGEU = 7
};

enum {
	ALU_add    = 0x0,  /// with : subFlag = 0(add), 1(sub)
	ALU_shl    = 0x1,
	ALU_slt    = 0x2,
	ALU_sltu   = 0x3,
	ALU_xor    = 0x4,
	ALU_shr    = 0x5,  /// with : sextFlag = 0(srl), 1(sra)
	ALU_or     = 0x6,
	ALU_and    = 0x7,
};

enum {
	MEM_lb     = 0x0,
	MEM_lh     = 0x1,
	MEM_lw     = 0x2,
	MEM_ld     = 0x3,
	MEM_lbu    = 0x4,
	MEM_lhu    = 0x5,
	MEM_lwu    = 0x6,
	MEM_sb     = 0x0,
	MEM_sh     = 0x1,
	MEM_sw     = 0x2,
	MEM_sd     = 0x3,
};

enum {
	AMO_lr   = 0x02,
	AMO_sc   = 0x03,
	AMO_swap = 0x01,
	AMO_add  = 0x00,
	AMO_xor  = 0x04,
	AMO_and  = 0x0c,
	AMO_or   = 0x08,
	AMO_min  = 0x10,
	AMO_max  = 0x14,
	AMO_minu = 0x18,
	AMO_maxu = 0x1c,
};

enum TrapCause {
	Trap_InstAddrMisalign = 0,
	Trap_InstAccessFault = 1,
	Trap_IllegalInst = 2,
	Trap_BreakPoint = 3,
	Trap_LoadAddrMisalign = 4,
	Trap_LoadAccessFault = 5,
	Trap_StoreAddrMisalign = 6,
	Trap_StoreAccessFault = 7,
	Trap_ECallFromU = 8,
	Trap_ECallFromS = 9,
	Trap_ECallFromVS = 10,
	Trap_ECallFromM = 11,
	Trap_InstPageFault = 12,
	Trap_LoadPageFault = 13,
	// Reserved
	Trap_StorePageFault = 15,
	// Reserved
	Trap_SoftwareCheck = 18,
	Trap_HardwareError = 19,
	Trap_InstGuestPageFault = 20,
	Trap_LoadGuestPageFault = 21,
	Trap_VirtualInst = 22,
	Trap_StoreGuestPageFault = 23,
	// Reserved
	// quit simulation
	Trap_Quit = 0x3f,
};

#if defined(CONFIG_RV64I) || defined(CONFIG_D)
#define RV_MEM_DATA_WIDTH  64
typedef UINT64 RV_MType;
#define RV_MEM_BYTE_POS_MASK   0x7
#define RV_MEM_BYTE_POS_BW     0x3
#define PF_MEM  "ll"
#else	// defined(CONFIG_RV64I) || defined(CONFIG_D)
#define RV_MEM_DATA_WIDTH  32	// 64 also works
typedef UINT32 RV_MType;
#define RV_MEM_BYTE_POS_MASK   0x3
#define RV_MEM_BYTE_POS_BW     0x2
#define PF_MEM  ""
#endif	// defined(CONFIG_RV64I) || defined(CONFIG_D)


#if	CONFIG_RV64I
typedef	UINT64	RV_UType;
typedef	SINT64	RV_SType;
typedef	UINT64	RV_AType;	// Address type

#define	RV_SHF_MAX	6
#define	RV_BYTE_POS_BW	3
#define	PF_RVType	"ll"
#else
typedef	UINT32	RV_UType;
typedef	SINT32	RV_SType;
#if	CONFIG_S
typedef	UINT34	RV_AType;	// Address type
#else
typedef	UINT32	RV_AType;	// Address type
#endif

#define	RV_SHF_MAX	5
#define	RV_BYTE_POS_BW	2
#define	PF_RVType
#endif

typedef	RV_UType RV_DUType _T(direct_signal);

#define	RV_BW	(1 << RV_SHF_MAX)
#define	RV_MSB	(RV_BW - 1)

//// Processor state

enum {
	U_MODE = 0,
	S_MODE = 1,
	M_MODE = 3,
};

struct ProcessorState {
	UINT2 mode;
	RV_UType mstatus;
#if	CONFIG_RV64I
	UINT4	address_translation_mode;
	UINT44	root_page_table;
#else
	BIT	address_translation_mode;
	UINT22	root_page_table;
#endif
#if CONFIG_F
	RV_UType fcsr;
#endif
	BIT flush_icache;  // fence.i
};

#endif	// _RVPROC_ARCH_H_
