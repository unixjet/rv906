#ifndef	_C2R_DEF_H_
#define	_C2R_DEF_H_

#include <stdint.h>

#ifdef	__LLVM_C2RTL__
#define	_BW(n)		__attribute__((C2RTL_bit_width(n)))
#define	_BWT(n)		__attribute__((C2RTL_bit_width(#n)))
#define	_T(t)		__attribute__((C2RTL_type(t)))
#define	_C2R_FUNC(n)	__attribute__((C2RTL_function(n)))
#define	_C2R_MODULE_	__attribute__((C2RTL_module))
#else
#define	_BW(n)
#define	_BWT(n)
#define	_T(t)
#define	_C2R_FUNC(n)
#define	_C2R_MODULE_
#endif

typedef	uint8_t BIT _BW(1);
typedef	uint8_t UINT2 _BW(2);
typedef	uint8_t UINT3 _BW(3);
typedef	uint8_t UINT4 _BW(4);
typedef	uint8_t UINT5 _BW(5);
typedef	uint8_t UINT6 _BW(6);
typedef	uint8_t UINT7 _BW(7);
typedef	uint8_t UINT8 _BW(8);
typedef	uint16_t UINT9 _BW(9);
typedef	uint16_t UINT10 _BW(10);
typedef	uint16_t UINT11 _BW(11);

typedef	int8_t SINT8 _BW(8);

typedef	uint16_t UINT12 _BW(12);
typedef	uint16_t UINT13 _BW(13);
typedef	uint16_t UINT15 _BW(15);
typedef	uint16_t UINT16 _BW(16);

typedef	int16_t SINT12 _BW(12);
typedef	int16_t SINT16 _BW(16);

typedef	uint32_t UINT18 _BW(18);
typedef	uint32_t UINT19 _BW(19);
typedef	uint32_t UINT20 _BW(20);
typedef	uint32_t UINT21 _BW(21);
typedef	uint32_t UINT22 _BW(22);
typedef	uint32_t UINT23 _BW(23);
typedef	uint32_t UINT24 _BW(24);
typedef	uint32_t UINT25 _BW(25);
typedef	uint32_t UINT26 _BW(26);
typedef	uint32_t UINT27 _BW(27);
typedef	uint32_t UINT28 _BW(28);
typedef	uint32_t UINT30 _BW(30);
typedef	uint32_t UINT32 _BW(32);

typedef	int32_t SINT32 _BW(32);

typedef	unsigned long long UINT33 _BW(33);
typedef	unsigned long long UINT34 _BW(34);
typedef	unsigned long long UINT35 _BW(35);
typedef	unsigned long long UINT44 _BW(44);
typedef	unsigned long long UINT46 _BW(46);
typedef	unsigned long long UINT48 _BW(48);
typedef	unsigned long long UINT49 _BW(49);
typedef	unsigned long long UINT50 _BW(50);
typedef	unsigned long long UINT52 _BW(52);
typedef	unsigned long long UINT53 _BW(53);
typedef	unsigned long long UINT54 _BW(54);
typedef	unsigned long long UINT55 _BW(55);
typedef	unsigned long long UINT56 _BW(56);
typedef	unsigned long long UINT57 _BW(57);
typedef	unsigned long long UINT58 _BW(58);
typedef	unsigned long long UINT64 _BW(64);

typedef	long long SINT64 _BW(64);

typedef	BIT FB_BIT;
typedef	BIT DI_BIT _T(direct_signal);
typedef	UINT32 D_UINT32 _T(direct_signal);
typedef	UINT64 D_UINT64 _T(direct_signal);

typedef	BIT ST_BIT _T(state);
typedef	UINT2 ST_UINT2 _T(state);
typedef	UINT3 ST_UINT3 _T(state);
typedef	UINT4 ST_UINT4 _T(state);
typedef	UINT5 ST_UINT5 _T(state);
typedef	UINT7 ST_UINT7 _T(state);
typedef	UINT8 ST_UINT8 _T(state);
typedef	UINT16 ST_UINT16 _T(state);
typedef	UINT32 ST_UINT32 _T(state);
typedef	UINT64 ST_UINT64 _T(state);

#endif	// _C2R_DEF_H_
