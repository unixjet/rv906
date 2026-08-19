#if !defined(RV_PROC_IO_H)
#define RV_PROC_IO_H

#include <stdio.h>

#define	TEST_VIRTUAL_METHOD_C2R_FUNC

#define	C2R_ASSERT(s)

enum IO_OP_TYPE
{
    IO_Idle         = 0x0,	///	000
    ///	2/23/15 : changed bit patterns
    IO_ReadData     = 0x1,	///	001
    IO_WriteData    = 0x2,	///	010
    IO_WriteCommand = 0x4,	///	100
};

enum IO_Parity
{
	IOP_None = 0,
	IOP_Odd = 1,
	IOP_Even = 3,
	IOP_High = 5,
	IOP_Low = 7,
};

enum UART_BIT
{
	UART_BIT5 = 0,
	UART_BIT6 = 1,
	UART_BIT7 = 2,
	UART_BIT8 = 3,
};

enum UART_BAUD
{
	UART_BAUD_50 = 0,
	UART_BAUD_300 = 1,
	UART_BAUD_1200 = 2,
	UART_BAUD_2400 = 3,
	UART_BAUD_4800 = 4,
	UART_BAUD_9600 = 5,
	UART_BAUD_19200 = 6,
	UART_BAUD_38400 = 7,
	UART_BAUD_57600 = 8,
	UART_BAUD_115200 = 9,
	///	4/27/15 : added UART_BAUD_ULTRA_FAST
	UART_BAUD_ULTRA_FAST = 15,
};

template <class T, int SIZE>
struct FIFO {
	int rptr, wptr, size;
	T buf[SIZE];

	FIFO() { rptr = 0; wptr = 0; size = 0; }
	void push(T data) {
		if (size < SIZE) {
			buf[wptr++] = data;
			if (wptr == SIZE)
				wptr = 0;
			size++;
		}
	}
	T pop() {
		T data = buf[rptr];
		if (size) {
			rptr++;
			if (rptr == SIZE)
				rptr = 0;
			size--;
		}

		return data;
	}
	bool empty() { return size == 0; }
	bool full() { return size == SIZE; }
	void reset() { rptr = 0; wptr = 0; size = 0; }
};

#define MAX_IO_BUF_SIZE	2

//int nxt_pnt(int cur_pnt);

/// 11/30/21
template<typename T, int bw, int IO_BUF_SIZE = MAX_IO_BUF_SIZE>
struct IOBuf
{
    ST_BIT		not_empty, full;
    ST_UINT8	rp, wp;
    T	buf[IO_BUF_SIZE] _BWT(bw) _T(state);	///	reg-file
    void reset() { rp = 0; wp = 0; full = 0; not_empty = 0; }
    void update(int rflag, int wflag) {
        if (rflag || wflag) {
            int nrp = (rflag) ? nxt_pnt(rp) : rp;
            int nwp = (wflag) ? nxt_pnt(wp) : wp;
            not_empty = (wflag) ? 1 : (nrp != nwp);
            full = (rflag) ? 0 : (nrp == nwp);
            rp = nrp;
            wp = nwp;
        }
    }
    int nxt_pnt(int cur_pnt){ return (cur_pnt == IO_BUF_SIZE - 1) ? 0 : cur_pnt + 1;}
};

struct AXI4L
{
    enum TransferSize { // in bytes
        TRANSFER_SIZE_1   = 0,
        TRANSFER_SIZE_2   = 1,
        TRANSFER_SIZE_4   = 2,
        TRANSFER_SIZE_8   = 3,
        TRANSFER_SIZE_16  = 4,
        TRANSFER_SIZE_32  = 5,
        TRANSFER_SIZE_64  = 6,
        TRANSFER_SIZE_128 = 7,
    };
    enum RespEnum { RSP_OK = 0, RSP_EXOK = 1, RSP_SLV_ERR = 2, RSP_DEC_ERR = 3, };

#if	CONFIG_RV64I
#define	AXI_ADDR_WIDTH	33	// MAX 54
    typedef UINT54 AXI_AType;
#else
//#define	AXI_ADDR_WIDTH	34
//    typedef UINT34 AXI_AType;
#define	AXI_ADDR_WIDTH	32
    typedef UINT32 AXI_AType;
#endif

#if !defined(AXI_DATA_WIDTH)
#define AXI_DATA_WIDTH  512
#endif

#if (RV_MEM_DATA_WIDTH < AXI_DATA_WIDTH)	// wide data bus
    struct AXI_DType {
        static constexpr int AR_SIZE = AXI_DATA_WIDTH / RV_MEM_DATA_WIDTH;
#if (AXI_DATA_WIDTH == 512)
        static constexpr int TRANSFER_SIZE = TRANSFER_SIZE_64;
        typedef UINT64 STROBE;
#elif (AXI_DATA_WIDTH == 256)
        static constexpr int TRANSFER_SIZE = TRANSFER_SIZE_32;
        typedef UINT32 STROBE;
#elif (AXI_DATA_WIDTH == 128)
        static constexpr int TRANSFER_SIZE = TRANSFER_SIZE_16;
        typedef UINT16 STROBE;
#elif (AXI_DATA_WIDTH == 64)
        static constexpr int TRANSFER_SIZE = TRANSFER_SIZE_8;
        typedef UINT8 STROBE;
#else
#error "Invalid AXI DATA WIDTH"
#endif
        RV_MType data[AR_SIZE];

	AXI_DType(RV_MType val = 0) {
            for (int i = 0; i < AR_SIZE; i++)
                data[i] = val;
        }
#if	0
	AXI_DType(const AXI_DType &o) {
            for (int i = 0; i < AR_SIZE; i++)
                data[i] = o.data[i];
        }
#endif

	AXI_DType &operator |=(const AXI_DType &o) {
            for (int i = 0; i < AR_SIZE; i++)
                data[i] |= o.data[i];
            return *this;
	}
	RV_MType &operator[](unsigned index) __attribute__((always_inline)) {
            return data[index];
        }

	RV_MType &getByAddr(AXI_AType addr) __attribute__((always_inline)) {
            unsigned index = addr >> RV_MEM_BYTE_POS_BW;
            index &= AR_SIZE - 1;
            return data[index];
        }
    };
    typedef AXI_DType::STROBE AXI_STROBE;
#define AXI_BYTE_POS_MASK   RV_MEM_BYTE_POS_MASK
#define AXI_BYTE_POS_BW     RV_MEM_BYTE_POS_BW
#elif (AXI_DATA_WIDTH == 64)
    typedef UINT64 AXI_DType;
    typedef UINT8  AXI_STROBE;
#define AXI_BYTE_POS_MASK   0x7
#define AXI_BYTE_POS_BW     0x3
#else
    typedef UINT32 AXI_DType;
    typedef UINT4  AXI_STROBE;
#define AXI_BYTE_POS_MASK   0x3
#define AXI_BYTE_POS_BW     0x2
#endif

    static void getBitPosMask64b(unsigned &bitPos, UINT64 &bitMask, AXI_AType addr, UINT3 size) {
        bitPos = (addr & 7) << 3;
        switch (size) {
        case 0: bitMask = 0xff; break;  /// TRANSFER_SIZE_1 : 1 byte
        case 1: bitMask = 0xffff; break; /// TRANSFER_SIZE_2 : 2 bytes
        case 2: bitMask = 0xffffffff; break; /// TRANSFER_SIZE_4 : 4 bytes;
        default: bitMask = 0xffffffffffffffff; break; /// TRANSFER_SIZE_4 : 8 bytes;
        }
    }
    static void getBitPosMask32b(unsigned &bitPos, unsigned &bitMask, AXI_AType addr, UINT3 size) {
#if !defined(__LLVM_C2RTL__)
#if (RV_MEM_DATA_WIDTH < AXI_DATA_WIDTH)	// wide data bus
        C2R_ASSERT(sizeof(RV_MType) == 8); /// 64 bits
#else
        C2R_ASSERT(sizeof(AXI_DType) == 8); /// 64 bits
#endif
#endif
        bitPos = (addr & AXI_BYTE_POS_MASK) << 3;
        switch (size) {
        case 0: bitMask = 0xff; break;  /// TRANSFER_SIZE_1 : 1 byte
        case 1: bitMask = 0xffff; break; /// TRANSFER_SIZE_2 : 2 bytes
        default: bitMask = 0xffffffff; break; /// TRANSFER_SIZE_4 : 4 bytes;
        }
    }
#if (RV_MEM_DATA_WIDTH < AXI_DATA_WIDTH)	// wide data bus
    static void AXI_DType_to_UINT32(AXI_DType * axi_data, UINT32 * ud32, AXI_AType addr, UINT3 size) {
        RV_MType data = axi_data->getByAddr(addr);

        if (sizeof(RV_MType) == sizeof(UINT32)) {
            *ud32 = data;
        }
        else {
            unsigned bitPos, bitMask;
            getBitPosMask32b(bitPos, bitMask, addr, size);
            *ud32 = (UINT32)((data >> bitPos) & bitMask);
        }
    }
    static void UINT32_to_AXI_DType(const UINT32 * ud32, AXI_DType * axi_data, AXI_AType addr, UINT3 size) {
        RV_MType *data = &axi_data->getByAddr(addr);

        if (sizeof(RV_MType) == sizeof(UINT32)) {
            *data = *ud32;
        }
        else {
            unsigned bitPos, bitMask;
            getBitPosMask32b(bitPos, bitMask, addr, size);
            *data = (((RV_MType)*ud32) & bitMask) << bitPos;   //#if defined(BUG_FIX_AXI_SLAVE_ALIGNMENT_READ)
        }
    }
#else
    static void AXI_DType_to_UINT32(AXI_DType * axi_data, UINT32 * ud32, AXI_AType addr, UINT3 size) {
        if (sizeof(AXI_DType) == sizeof(UINT32)) {
            *ud32 = (UINT32)(*axi_data);
        }
        else {
            unsigned bitPos, bitMask;
            getBitPosMask32b(bitPos, bitMask, addr, size);
            *ud32 = (UINT32)(((*axi_data) >> bitPos) & bitMask);
        }
    }
    static void UINT32_to_AXI_DType(UINT32 * ud32, AXI_DType * axi_data, AXI_AType addr, UINT3 size) {
        if (sizeof(AXI_DType) == sizeof(UINT32)) {
            *axi_data = (AXI_DType)(*ud32);
        }
        else {
            unsigned bitPos, bitMask;
            getBitPosMask32b(bitPos, bitMask, addr, size);
            *axi_data = ((((AXI_DType)(*ud32)) & bitMask) << bitPos);   //#if defined(BUG_FIX_AXI_SLAVE_ALIGNMENT_READ)
        }
    }
#endif

    static void From_AXI_DType(AXI_DType axi_data, UINT32 * ud32, AXI_AType addr, UINT3 size) {
#if (RV_MEM_DATA_WIDTH < AXI_DATA_WIDTH)	// wide data bus
        RV_MType data = axi_data.getByAddr(addr);
#else
        AXI_DType   data = axi_data;
#endif

#if	0
        unsigned bitPos;
        UINT64 bitMask;

        getBitPosMask64b(bitPos, bitMask, addr, size);
	bitMask <<= bitPos;
	if (addr & 0x4) {
		bitMask >>= 32;
		data >>= 32;
	}
	*ud32 = (*ud32 & ~bitMask) | (data & bitMask);
#else
        if (sizeof(data) == sizeof(UINT32)) {
            *ud32 = data;
        } else {
            if (addr & 0x4) {
                *ud32 = ((UINT64)data >> 32) & 0xffffffff;
            } else {
                *ud32 = data & 0xffffffff;
            }
        }
#endif
    }
    static void To_AXI_DType(UINT32 ud32, AXI_DType * axi_data, AXI_AType addr, UINT3 size) {
#if !defined(__LLVM_C2RTL__)
        C2R_ASSERT((addr & ((1 << size) - 1)) == 0);
#endif

#if (RV_MEM_DATA_WIDTH < AXI_DATA_WIDTH)	// wide data bus
        RV_MType *data = &axi_data->getByAddr(addr);
#else
        AXI_DType   *data = axi_data;
#endif

        if (sizeof(*data) == sizeof(UINT32)) {
            *axi_data = ud32;
	} else {
            if (addr & 0x4)
                *axi_data = (UINT64)ud32 << 32;
            else
                *axi_data = ud32;
	}
    }
    static void To_AXI_DType(const UINT32 *ud32, AXI_DType * axi_data, AXI_AType addr, UINT3 size) {
#if !defined(__LLVM_C2RTL__)
	C2R_ASSERT((addr & ((1 << size) - 1)) == 0);
#endif

#if (RV_MEM_DATA_WIDTH < AXI_DATA_WIDTH)	// wide data bus
        RV_MType *data = &axi_data->getByAddr(addr);
#else
        AXI_DType   *data = axi_data;
#endif

        if (sizeof(*data) == sizeof(UINT32)) {
	    if (size <= TRANSFER_SIZE_4) {
                *data = *ud32;
#if (RV_MEM_DATA_WIDTH < AXI_DATA_WIDTH)	// wide data bus
	    } else if (size == AXI_DType::TRANSFER_SIZE) {
		for (int i = 0; i < AXI_DType::AR_SIZE; i++)
		    axi_data->data[i] = *ud32++;
#endif
	    }
	} else {
	    if (size <= TRANSFER_SIZE_4) {
                if (addr & 0x4)
                    *data = (UINT64)*ud32 << 32;
                else
                    *data = *ud32;
	    } else if (size == TRANSFER_SIZE_8) {
		auto tmp = *ud32++;
		*data++ = ((UINT64)*ud32++ << 32) | tmp;
#if (RV_MEM_DATA_WIDTH < AXI_DATA_WIDTH)	// wide data bus
	    } else if (size == AXI_DType::TRANSFER_SIZE) {
		for (int i = 0; i < AXI_DType::AR_SIZE; i++) {
		    auto tmp = *ud32++;
		    axi_data->data[i] = ((UINT64)*ud32++ << 32) | tmp;
		}
#endif
	    }
	}
    }

    static void From_AXI_DType(AXI_DType axi_data, UINT64 * ud64, AXI_AType addr, UINT3 size) {
#if (RV_MEM_DATA_WIDTH < AXI_DATA_WIDTH)	// wide data bus
        RV_MType data = axi_data.getByAddr(addr);
#else
        AXI_DType   data = axi_data;
#endif
        unsigned bitPos;
        UINT64 bitMask;

        getBitPosMask64b(bitPos, bitMask, addr, size);
	bitMask <<= bitPos;
        *ud64 = (*ud64 & ~bitMask) | (data & bitMask);
    }
    static void To_AXI_DType(UINT64 ud64, AXI_DType * axi_data, AXI_AType addr, UINT3 size) {
#if (RV_MEM_DATA_WIDTH < AXI_DATA_WIDTH)	// wide data bus
        RV_MType *data = &axi_data->getByAddr(addr);
#else
        AXI_DType   *data = axi_data;
#endif

        if (sizeof(*data) == sizeof(UINT64)) {
            *axi_data = ud64;
	} else {
            if (addr & 0x4)
                *axi_data = ud64 >> 32;
            else
                *axi_data = ud64 & 0xffffffff;
	}
    }

    /// 11/30/21 : RV_UType
    template <class DType>
    struct TCH { // channel
        struct ADDR { /// addr-read, addr-write
            struct MA {
                AXI_AType addr; UINT3 size; BIT valid; UINT4 len; UINT3 prot; UINT2 burst;
                void set(AXI_AType a, UINT3 s, BIT v, UINT4 l, UINT3 p, UINT2 b) { addr = a; size = s; valid = v; len = l; prot = p; burst = b;}
                void set(MA &m) { set(m.addr, m.size, m.valid, m.len, m.prot, m.burst); }
                void reset() { set(0, TRANSFER_SIZE_4, 0, 0, 7, 1); }
            } m;
            struct SL {
                BIT ready;
                void set(BIT r) { ready = r; }
                void set(SL &s) { set(s.ready); }
                void reset() { set(0); }
            } s;
        } raddr, waddr;
        struct RDAT {  /// read-data
            struct MA {
                BIT ready;
                void set(BIT r) { ready = r; }
                void set(MA &m) { set(m.ready); }
                void reset() { set(0); }
            } m;
            struct SL {
                DType data; UINT2 resp; BIT valid, last;
                void set(DType d, UINT2 r, BIT v, BIT l) { data = d; valid = v; resp = r; last = l; }
                void set(SL &s) { set(s.data, s.resp, s.valid, s.last); }
                void reset() { set(0, 0, 0, 0); }
            } s;
        } rdat;
        struct WDAT {  /// write-data
            struct MA {
                DType data; AXI_STROBE strobe; BIT valid, last;
                void set(DType d, AXI_STROBE s, BIT v, BIT l) { data = d; strobe = s; valid = v; last = l; }
                void set(MA &m) { set(m.data, m.strobe, m.valid, m.last); }
                void reset() { set(0, 0, 0, 0); }
            } m;
            struct SL {
                BIT ready;
                void set(BIT r) { ready = r; }
                void set(SL &s) { set(s.ready); }
                void reset() { set(0); }
            } s;
        } wdat;
        struct WRES {  /// write-resp
            struct MA {
                BIT ready;
                void set(BIT r) { ready = r; }
                void set(MA &m) { set(m.ready); }
                void reset() { set(0); }
            } m;
            struct SL {
                UINT2 resp; BIT valid;
                void set(UINT2 r, BIT v) { resp = r; valid = v; }
                void set(SL &s) { set(s.resp, s.valid); }
                void reset() { set(0, 0); }
            } s;
        } wres;
        UINT32 intr;
        void setMARead(typename ADDR::MA &ra, typename RDAT::MA &rd) { raddr.m.set(ra); rdat.m.set(rd); }
        void setMAWrite(typename ADDR::MA &wa, typename WDAT::MA &wd, typename WRES::MA &wr) { waddr.m.set(wa); wdat.m.set(wd); wres.m.set(wr); }
        void setMARead(TCH &c) { raddr.m.set(c.raddr.m); rdat.m.set(c.rdat.m); }
        void setMAWrite(TCH &c) { waddr.m.set(c.waddr.m); wdat.m.set(c.wdat.m); wres.m.set(c.wres.m); }
        void resetMA() { resetMARead(); resetMAWrite(); }
        void resetMARead() { raddr.m.reset(); rdat.m.reset(); }
        void resetMAWrite() { waddr.m.reset(); wdat.m.reset(); wres.m.reset(); }
        void resetSL() { resetSLRead(); resetSLWrite(); }
        void setSLRead(typename ADDR::SL &ra, typename RDAT::SL &rd) { raddr.s.set(ra); rdat.s.set(rd); }
        void setSLWrite(typename ADDR::SL &wa, typename WDAT::SL &wd, typename WRES::SL &wr) { waddr.s.set(wa); wdat.s.set(wd); wres.s.set(wr); }
        void setSLRead(TCH &c) { raddr.s.set(c.raddr.s); rdat.s.set(c.rdat.s); }
        void setSLWrite(TCH &c) { waddr.s.set(c.waddr.s); wdat.s.set(c.wdat.s); wres.s.set(c.wres.s); }
        void resetSLRead() { raddr.s.reset(); rdat.s.reset(); }
        void resetSLWrite() { waddr.s.reset(); wdat.s.reset(); wres.s.reset(); }
        static void connectRCh(TCH &mc, TCH &sc) { sc.setMARead(mc);  mc.setSLRead(sc); }
        static void connectWCh(TCH &mc, TCH &sc) { sc.setMAWrite(mc); mc.setSLWrite(sc); }
    } _T(direct_signal);    /// struct TCH
    typedef TCH<AXI_DType> CH;

    template <class DType>
    struct PORT {   /// for AXI-FSM IOs : split each MA/SL channels and merge as MA/SL PORTs
        struct MA { /// for AXI-Master-FSM outPorts
            typename TCH<DType>::ADDR::MA raddr, waddr; typename TCH<DType>::RDAT::MA rdat; typename TCH<DType>::WDAT::MA wdat; typename TCH<DType>::WRES::MA wres;
            void setRead(AXI_AType ra_addr, UINT3 size, BIT ra_valid, BIT rd_ready, UINT4 len, UINT3 prot)
            { raddr.set(ra_addr, size, ra_valid, len, prot, 1); rdat.set(rd_ready); }
            void setWrite(AXI_AType wa_addr, UINT2 size, BIT wa_valid, DType wd_data, AXI_STROBE wd_strobe,
                BIT wd_valid, BIT wd_last, BIT wr_ready, UINT4 len, UINT3 prot)
            { waddr.set(wa_addr, size, wa_valid, len, prot, 1); wdat.set(wd_data, wd_strobe, wd_valid, wd_last); wres.set(wr_ready); }
        };
        struct SL { /// for AXI-Slave-FSM outPorts, AXI-Master-FSM inPorts
            typename TCH<DType>::ADDR::SL raddr, waddr; typename TCH<DType>::RDAT::SL rdat; typename TCH<DType>::WDAT::SL wdat; typename TCH<DType>::WRES::SL wres;
            void resetRead() { raddr.reset(); rdat.reset(); }
            void resetWrite() { waddr.reset(); wdat.reset(); wres.reset(); }
        };
    };

    static unsigned decodeAddr(AXI_AType addr);

    struct MasterFSM;
    template <class DType>
    struct TSlaveFSM;
    typedef TSlaveFSM<AXI_DType> SlaveFSM;
    struct MasterFSM {
        PORT<AXI_DType>::MA out _T(state); /// for driving MA-outputs
        PORT<AXI_DType>::SL in _T(state);  /// for latching MA-inputs
        enum RWState { RW_Init, RW_RAddr, RW_WAddr, };
        enum AXIOp { AXI_Read, AXI_Write, AXI_Idle, };
        ST_UINT3    rw_state;
        ST_UINT4    burst_count;
        UINT4       burst_length;
        UINT3       mem_prot;
        bool fsm(UINT2 op, CH *axi, AXI_AType addr, UINT3 size, AXI_DType dout, AXI_DType *din, BIT *data_latched);
        unsigned getReadChannelStat() { return ((in.rdat.valid << 1) | in.raddr.ready); }
        unsigned getWriteChannelStat() { return ((in.wres.valid << 2) | (in.wdat.ready << 1) | in.waddr.ready); }
    };
    #define ENABLE_MUTEX_MSG
    /// 6/22/17 : change to counting semaphore!!!
    struct Mutex {
        ST_UINT4    lockCount;
        ST_UINT4    ownerID;
        ST_UINT32   lockErrorCount;
        BIT getLock(UINT4 ID, const char *name, UINT32 cycle) {
            if (lockCount && ownerID != ID) {
                if ((lockErrorCount & 0x7ff) == 0)
                    printf("[%8d] [getLock:%s:%d] ERROR (already locked by owner(%d))!! (total lockErrors = %d)\n",
                        (unsigned)cycle, name, ID, ownerID, lockErrorCount);
                lockErrorCount++;
                return 0;
            }   /// already locked!!!
#if defined(ENABLE_MUTEX_MSG)
            if (cycle < 2000)
                printf("[%8d] [getLock:%s] locked by owner(%d) : lockCount(%d)!!\n", (unsigned)cycle, name, ID, lockCount + 1);
#endif
            ownerID = ID; lockCount++; return 1;
        }
        BIT releaseLock(UINT4 ID, const char *name, UINT32 cycle) {
            if (!lockCount || ID != ownerID) {
                if (!lockCount) { printf("[%8d] [releaseLock:%s:%d] ERROR (not locked)!!\n", (unsigned)cycle, name, ID); }
                else { printf("[%8d] [releaseLock:%s:%d] ERROR (ownerID(%d) != ID(%d))!!\n", (unsigned)cycle, name, ID, ownerID, ID); }
                return 0; /// not locked or not owner
            }
#if defined(ENABLE_MUTEX_MSG)
            if (cycle < 2000)
                printf("[%8d] [releaseLock:%s] released by owner(%d) : lockCount(%d)!!\n", (unsigned)cycle, name, ID, lockCount - 1);
#endif
            lockCount--; return 1;
        }
    };

#define FIX_READ_SETUP_BUG  /// 4/18/22
//#define DBG_AXI_SLAVE

    template <class DType>
    struct TSlaveFSM {
        typename PORT<DType>::SL out _T(state);
        enum ReadState { R_Init, R_Addr, R_End, };
        enum WriteState { W_Init, W_AddrData, W_AddrDataBurst, W_End, };
        /// regs (states)
        ST_UINT3 r_state, w_state;
        AXI_AType raddr, waddr _T(state);
        ST_UINT3 rsize, wsize;
        ST_BIT raddr_end, waddr_end, wres_end; /// indicate that raddr/waddr are released by master
        ST_BIT writePending;   /// need to indicate difference of write-operation and mutex
        ST_UINT32 intrFlag;
        Mutex mutex;
        /// wires (no-states)
        UINT32 nxt_intrFlag;
        int readFlag, writeFlag;
        UINT32 writeData;
        UINT32 cycle _T(state);
        BIT mutexError;
        ST_UINT4    burst_count_r, burst_count_w;
        ST_BIT      burst_end_r;
#if defined(DBG_AXI_SLAVE)  /// 4/18/22
        ST_UINT32   dbg_rcount, dbg_wcount;
#endif
        /// methods
        void fsmRead(TCH<DType> *axi)
	{
	//    axi->setSLRead(out.raddr, out.rdat);
	    switch (r_state) {
	    case R_Init:
	//#if defined(FIX_READ_SETUP_BUG) /// 4/18/22
		if (axi->raddr.m.valid && devReadSetup(axi->raddr.m.addr, axi->raddr.m.len)) {
		    raddr = axi->raddr.m.addr;
		    rsize = axi->raddr.m.size;
		    burst_count_r = axi->raddr.m.len;
		    burst_end_r = 0;
		    out.raddr.set(1);
		    raddr_end = 0;
		    r_state = R_Addr;
		}
		break;
	    case R_Addr: {
		out.raddr.reset();
		if (!axi->raddr.m.valid) { raddr_end = 1; }
		auto d = out.rdat.data;
		if ((out.rdat.valid && burst_end_r) || devRead(&d, raddr, rsize)) {
		    out.rdat.set(d, RSP_OK, 1, (burst_count_r == 0));
#if defined(DBG_AXI_SLAVE)  /// 4/18/22
		    dbg_rcount++;
#endif
		    if (burst_count_r == 0) {
			if (axi->rdat.m.ready) { r_state = R_End; }
			burst_end_r = 1;
		    } else {
			burst_count_r--;
		    }
		}
		break;
	    }
	    case R_End: /// make sure raddr.valid is deasserted BEFORE going back to R_Init
		out.rdat.reset();
		if (raddr_end || !axi->raddr.m.valid) { r_state = R_Init; nxt_intrFlag |= 1; }
		break;
	    }
	    axi->setSLRead(out.raddr, out.rdat);
	}
        void fsmWrite(TCH<DType> *axi)
	{
		switch (w_state) {
		case W_Init:
			if (axi->waddr.m.valid && devWriteSetup(axi->waddr.m.addr)) {
				waddr = axi->waddr.m.addr;
				wsize = axi->waddr.m.size;
				burst_count_w = axi->waddr.m.len;
				out.waddr.set(1);
				waddr_end = 0;
				if (axi->waddr.m.len) {
					out.wdat.set(1);
					w_state = W_AddrDataBurst;
				} else {
					w_state = W_AddrData;
				}
			}
			break;
		case W_AddrData:
			out.waddr.reset();
			if (!axi->waddr.m.valid) { waddr_end = 1; }
			if (axi->wdat.m.valid && devWrite(axi->wdat.m.data, waddr, wsize)) {
				wres_end = 0;
				out.wdat.set(1);
				w_state = W_End;
				out.wres.set((mutexError) ? RSP_SLV_ERR : RSP_OK, 1);   /// 6/23/17
				writePending = (writeFlag == 1);
#if defined(DBG_AXI_SLAVE)  /// 4/18/22
		    dbg_wcount++;
#endif
		}
			break;
		case W_AddrDataBurst:
			out.waddr.reset();
			if (!axi->waddr.m.valid) { waddr_end = 1; }
			if (axi->wdat.m.valid) {
				if (devWrite(axi->wdat.m.data, waddr, wsize)) {
					wres_end = 0;
					if (burst_count_w == 0) {
						out.wdat.reset();
						w_state = W_End;
						out.wres.set((mutexError) ? RSP_SLV_ERR : RSP_OK, 1);   /// 6/23/17
					} else {
						burst_count_w--;
					}
					writePending = (writeFlag == 1);
#if defined(DBG_AXI_SLAVE)  /// 4/18/22
			dbg_wcount++;
#endif
		    }
			}
			break;
		case W_End:
			out.wdat.reset();
		/// fixed version
			{
				BIT nxt_wres_end = wres_end;
				if (axi->wres.m.ready) { wres_end = 1; nxt_wres_end = 1; }
				if ((nxt_wres_end || axi->wres.m.ready) && (waddr_end || !axi->waddr.m.valid)) {
					out.wres.reset();
					w_state = W_Init;
					if (writePending) { nxt_intrFlag |= 2; }
				}
			}
			break;
		}
		axi->setSLWrite(out.waddr, out.wdat, out.wres);
	}
        void fsm(TCH<DType> *axi) {
            axi->intr = intrFlag;
            nxt_intrFlag = 0;
            preFsmUser();
            fsmRead(axi);
            fsmWrite(axi);
            fsmUser();
#if defined(TEST_MPSOC)
            intrFlag = (nxt_intrFlag) << (TOTAL_INTERRUPTS * mutex.ownerID);
#else
            intrFlag = nxt_intrFlag;
#endif
            cycle++;
        }
        virtual BIT devRead(DType *data, AXI_AType addr, UINT3 size) = 0;
        virtual BIT devReadSetup(AXI_AType addr, unsigned blen) { return 1; }
        virtual BIT devWrite(DType data, AXI_AType addr, UINT3 size) = 0;
        virtual BIT devWriteSetup(AXI_AType addr) { return 1; }
        virtual void preFsmUser() { readFlag = 0; writeFlag = 0; writeData = 0; mutexError = 0; };
        virtual void fsmUser() {};
    };
    struct NoDev {
        PORT<AXI_DType>::SL out _T(state);

        _C2R_FUNC(1)
        void step(AXI4L::CH *axi) {
            if (axi->raddr.m.valid) {
	        out.raddr.set(1);
            } else {
	        out.raddr.set(0);
            }
            if (axi->rdat.m.ready) {
	        out.rdat.set(0, RSP_DEC_ERR, 1, 1);
            } else {
	        out.rdat.set(0, RSP_OK, 0, 0);
            }
            axi->setSLRead(out.raddr, out.rdat);

            if (axi->waddr.m.valid) {
	        out.waddr.set(1);
            } else {
	        out.waddr.set(0);
            }
            if (axi->wdat.m.valid) {
                out.wdat.set(1);
            } else {
                out.wdat.set(0);
            }
            if (axi->wres.m.ready) {
                out.wres.set(RSP_DEC_ERR, 1);
            } else {
                out.wres.set(RSP_OK, 0);
            }
	    axi->setSLWrite(out.waddr, out.wdat, out.wres);
        }
    };
    /// 9/25/22 : remove VIRTUAL_BUS codes...
    template <int MC, int SC> struct BUS {
        CH m_ch[MC], s_ch[SC];
        void resetAllChannelSinks() {
            for (unsigned i = 0; i < SC; ++i) { s_ch[i].resetMA(); }
            for (unsigned i = 0; i < MC; ++i) { m_ch[i].resetSL(); }
        }
        void connectInterrupts() { // hard-coded for now... : no intr for spi
            m_ch[0].intr = 0;
        }
    };
    struct MasterStatus {   /// ok, another problem... C2R crashes if MasterStatus is declared under BUS
                            /// put _T(state) inside MasterStatus/SlaveStatus
        ST_UINT8   slaveID; ////    0xffff : invalid device
        ST_BIT     active;
        BIT        granted;     /// wire
        BIT        requested;   /// wire
        UINT8      reqSlaveID;  /// wire
        void set(UINT8 sID, BIT a) { slaveID = sID; active = a; }
        void set(MasterStatus &ms) { set(ms.slaveID, ms.active); }
        void reset() { slaveID = 0; active = 0; }
        void resetFlags() { granted = 0; requested = 0; reqSlaveID = 0; }
    };
    template <int MC> struct SlaveStatus {
        ST_UINT8    masterID;
        ST_BIT      active;
    };
    template <int MC, int SC> struct CTRL {
        /// first try... assume MA_COUNT == 1; (no arbitration)
        /// put _T(state) inside MasterStatus/SlaveStatus
        MasterStatus MRStat[MC], MWStat[MC];
        SlaveStatus<MC> SRStat[SC], SWStat[SC];
        ST_UINT4     roundRobinRVal, roundRobinWVal;
        void checkMRReq(BUS<MC, SC> *bus, int midx) {
            CH &mc = bus->m_ch[midx];
            MasterStatus &rs = MRStat[midx];
            rs.resetFlags();
            if (!rs.active && mc.raddr.m.valid) {
                UINT32 sidx = decodeAddr(mc.raddr.m.addr);
                if (!SRStat[sidx].active && !bus->s_ch[sidx].rdat.s.valid) {
                    rs.requested = 1;
                    rs.reqSlaveID = sidx;
                }
            }
        }
        void checkMWReq(BUS<MC, SC> *bus, int midx) {
            CH &mc = bus->m_ch[midx];
            MasterStatus &ws = MWStat[midx];
            ws.resetFlags();
            if (!ws.active && mc.waddr.m.valid) {
                unsigned sidx = decodeAddr(mc.waddr.m.addr);
                if (!SWStat[sidx].active && !bus->s_ch[sidx].wres.s.valid) {
                    ws.requested = 1;
                    ws.reqSlaveID = sidx;
                }
            }
        }
        BIT arbitrate(UINT4 sidx, MasterStatus *ms, UINT4 rrVal) {
            BIT flag = 0;
            for (int i = 0; i < MC; ++i) {  /// 1st round
                if (i >= rrVal && !flag && ms[i].requested && ms[i].reqSlaveID == sidx) {
                    ms[i].granted = 1; flag = 1;
                }
            }
            for (int i = 0; i < MC; ++i) {  /// 2nd round
                if (!flag && ms[i].requested && ms[i].reqSlaveID == sidx) {
                    ms[i].granted = 1; flag = 1;
                }
            }
            return flag;
        }
        void arbitrateMR() {
            int granted = 0;
            for (int i = 0; i < SC; ++i) { granted |= arbitrate(i, MRStat, roundRobinRVal); }
            if (granted) { roundRobinRVal = (roundRobinRVal < MC - 1) ? roundRobinRVal + 1 : 0; }
        }
        void arbitrateMW() {
            int granted = 0;
            for (int i = 0; i < SC; ++i) { granted |= arbitrate(i, MWStat, roundRobinWVal); }
            if (granted) { roundRobinWVal = (roundRobinWVal < MC - 1) ? roundRobinWVal + 1 : 0; }
        }
        //#define DBG_MW
#if defined(DBG_MW)
        ST_UINT32 dbg_cycle;
#endif
#if defined(TEST_VIRTUAL_METHOD_C2R_FUNC)
        _C2R_FUNC(1)
#endif
        void connectChannel(BUS<MC, SC> *bus) {
            bus->resetAllChannelSinks();
            bus->connectInterrupts();
#if defined(DBG_MW)
            if (dbg_cycle < 2000) {
                if (bus->m_ch[0].intr) { printf("[%8d] m[0].intr = %08x\n", dbg_cycle, bus->m_ch[0].intr); }
                if (bus->m_ch[1].intr) { printf("[%8d] m[1].intr = %08x\n", dbg_cycle, bus->m_ch[1].intr); }
            }
#endif
            int i, j;
            for (i = 0; i < MC; ++i) { checkMRReq(bus, i); checkMWReq(bus, i); }
            arbitrateMR(); arbitrateMW();
            for (i = 0; i < MC; ++i) {  /// i : const (due to loop-unrolling), j : variable
                if (updateMRStat(bus, i, &j)) { CH::connectRCh(bus->m_ch[i], bus->s_ch[j]); }
            }
            for (i = 0; i < MC; ++i) {
                if (updateMWStat(bus, i, &j)) { CH::connectWCh(bus->m_ch[i], bus->s_ch[j]); }
            }
#if defined(DBG_MW)
            ++dbg_cycle;
#endif
        }
        BIT updateMRStat(BUS<MC, SC> *bus, int midx, int *sidx) {
            CH &mc = bus->m_ch[midx];
            MasterStatus &rs = MRStat[midx];
            *sidx = rs.slaveID;
            if (rs.granted) {
                *sidx = rs.reqSlaveID;
                rs.set(*sidx, 1);
                SRStat[*sidx].active = 1;
                SRStat[*sidx].masterID = midx;
                return 1;
            }
            if (rs.active && mc.rdat.m.ready && bus->s_ch[*sidx].rdat.s.last) {
                rs.reset();  SRStat[*sidx].active = 0; return 1;
            }
            else { return rs.active; }
        }
        BIT updateMWStat(BUS<MC, SC> *bus, int midx, int *sidx) {
            CH &mc = bus->m_ch[midx];
            MasterStatus &ws = MWStat[midx];
            *sidx = ws.slaveID;
            if (ws.granted) {
                *sidx = ws.reqSlaveID;
                ws.set(*sidx, 1);
                SWStat[*sidx].active = 1;
                SWStat[*sidx].masterID = midx;
#if defined(DBG_MW)
                if (dbg_cycle < 2000)
                    printf("[%8d] W-granted : M[%d]->S[%d]\n", dbg_cycle, midx, *sidx);
#endif
                return 1;
            }
            if (ws.active && mc.wres.m.ready && bus->s_ch[*sidx].wres.s.valid) {
                ws.reset(); SWStat[*sidx].active = 0;
#if defined(DBG_MW)
                if (dbg_cycle < 2000)
                    printf("[%8d] W-end     : M[%d]->S[%d]\n", dbg_cycle, midx, *sidx);
#endif
                return 1;
            }
            else { return ws.active; }
        }
    };

    static void From_AXI_DType(AXI_DType &axi_data, UINT8 *u8, AXI_AType addr, UINT3 size) {
        RV_MType data = axi_data.getByAddr(addr);

	data >>= (addr & RV_MEM_BYTE_POS_MASK) * 8;
	*u8 = data & 0xff;
    }
    static void To_AXI_DType(UINT8 u8, AXI_DType *axi_data, AXI_AType addr, UINT3 size) {
        RV_MType data = (RV_MType)u8 << (addr & RV_MEM_BYTE_POS_MASK) * 8;

	*axi_data = data;
    }

    template<class DType>
    struct Converter {
	BIT st_waddr _T(state);
	BIT st_wdat _T(state);
	BIT st_wres _T(state);
	BIT st_raddr _T(state);
	BIT st_rdat _T(state);
	typename PORT<AXI_DType>::SL s_port _T(state);
	typename PORT<DType>::MA m_port _T(state);
        UINT32 intr _T(state);

        _C2R_FUNC(1)
        void fsm(CH *s_axi, TCH<DType> *m_axi) {
            // set output
            // waddr
            m_axi->waddr.m.set(m_port.waddr.addr,
			       m_port.waddr.size,
			       m_port.waddr.valid,
			       m_port.waddr.len,
			       m_port.waddr.prot,
			       m_port.waddr.burst);
	    s_axi->waddr.s.set(st_waddr == 0);
	    // wdat
            m_axi->wdat.m.set(m_port.wdat.data,
			      m_port.wdat.strobe,
			      m_port.wdat.valid,
			      m_port.wdat.last);
	    s_axi->wdat.s.set(st_wdat == 0);
	    // wres
            m_axi->wres.m.set(st_wres == 0);
	    s_axi->wres.s.set(s_port.wres.resp,
			      s_port.wres.valid);
	    // raddr
            m_axi->raddr.m.set(m_port.raddr.addr,
			       m_port.raddr.size,
			       m_port.raddr.valid,
			       m_port.raddr.len,
			       m_port.raddr.prot,
			       m_port.raddr.burst);
	    s_axi->raddr.s.set(st_raddr == 0);
	    // rdat
            m_axi->rdat.m.set(st_rdat == 0);
	    s_axi->rdat.s.set(s_port.rdat.data,
			      s_port.rdat.resp,
			      s_port.rdat.valid,
			      s_port.rdat.last);
	    //
	    s_axi->intr = intr;

	    // convert
	    AXI_AType waddr = m_port.waddr.addr;
	    AXI_AType raddr = m_port.raddr.addr;
	    UINT3 wsize = m_port.waddr.size;
	    UINT3 rsize = m_port.raddr.size;

	    if (s_axi->waddr.m.valid) {
	    	waddr = s_axi->waddr.m.addr;
	    	wsize = s_axi->waddr.m.size;
	    }
	    if (s_axi->raddr.m.valid) {
	    	raddr = s_axi->raddr.m.addr;
	    	rsize = s_axi->raddr.m.size;
	    }

	    DType wdata = 0;
	    AXI_DType rdata;
	    UINT32 strobe = ~0;

	    AXI4L::From_AXI_DType(s_axi->wdat.m.data, &wdata, waddr, wsize);
	    AXI4L::To_AXI_DType(m_axi->rdat.s.data, &rdata, raddr, rsize);

	    // latch data & update state
	    if (st_waddr == 0) {
	        if (s_axi->waddr.m.valid) {
		    m_port.waddr.set(s_axi->waddr.m.addr,
				     s_axi->waddr.m.size,
				     s_axi->waddr.m.valid,
				     s_axi->waddr.m.len,
				     s_axi->waddr.m.prot,
				     s_axi->waddr.m.burst);
		    st_waddr = 1;
		}
	    } else {
	        if (m_axi->waddr.s.ready) {
		    m_port.waddr.valid = 0;
		    st_waddr = 0;
		}
	    }

	    if (st_wdat == 0) {
		if (s_axi->wdat.m.valid) {
		    m_port.wdat.set(wdata,
				    strobe,
				    s_axi->wdat.m.valid,
				    s_axi->wdat.m.last);
		    st_wdat = 1;
		}
	    } else {
	        if (m_axi->wdat.s.ready) {
		    m_port.wdat.reset();
		    st_wdat = 0;
		}
	    }

	    if (st_wres == 0) {
		if (m_axi->wres.s.valid) {
		    s_port.wres.set(m_axi->wres.s.resp,
				    m_axi->wres.s.valid);
		    st_wres = 1;
		}
	    } else {
	        if (s_axi->wres.m.ready) {
		    s_port.wres.reset();
		    st_wres = 0;
		}
	    }

	    if (st_raddr == 0) {
	        if (s_axi->raddr.m.valid) {
		    m_port.raddr.set(s_axi->raddr.m.addr,
				     s_axi->raddr.m.size,
				     s_axi->raddr.m.valid,
				     s_axi->raddr.m.len,
				     s_axi->raddr.m.prot,
				     s_axi->raddr.m.burst);
		    st_raddr = 1;
		}
	    } else {
	        if (m_axi->raddr.s.ready) {
		    m_port.raddr.valid = 0;
		    st_raddr = 0;
		}
	    }

	    // rdat
	    if (st_rdat == 0) {
		if (m_axi->rdat.s.valid) {
		    s_port.rdat.set(rdata,
			            m_axi->rdat.s.resp,
				    m_axi->rdat.s.valid,
				    m_axi->rdat.s.last);
		    st_rdat = 1;
		}
	    } else {
	        if (s_axi->rdat.m.ready) {
		    s_port.rdat.reset();
		    st_rdat = 0;
		}
	    }

	    //
	    intr = m_axi->intr;
	}
    };
};

struct RV_AXI4L : AXI4L::MasterFSM {
//#if (RV_MEM_DATA_WIDTH == 32) && (AXI_DATA_WIDTH == 64)
#if RV_MEM_DATA_WIDTH < AXI_DATA_WIDTH
    typedef AXI4L::AXI_DType DType;
#else
    typedef RV_MType      DType;
#endif
    AXI4L::AXI_AType    addr_out;   /// from master-core
    DType       data_in;    /// from slave
    UINT3       transfer_size;
    ST_UINT3    pending_op;
    BIT         active, stalled, data_latched;
#if (RV_MEM_DATA_WIDTH == 64) && (AXI_DATA_WIDTH == 32)
    BIT         state _T(state);
    AXI4L::AXI_DType    l_data_in _T(state);
#endif
    BIT         error;
    void fsm(BIT cancel_io_insn, UINT3 io_op_in, DType src0, AXI4L::CH *axi);
};

#endif
