
#include "../RVProc.h"
#include "RVProc_io.h"

#if (RV_MEM_DATA_WIDTH <= AXI_DATA_WIDTH)	// wide data bus
void RV_AXI4L::fsm(BIT cancel_io_insn, UINT3 io_op_in, DType src0, AXI4L::CH *axi) {
    UINT3 io_op = (cancel_io_insn) ? IO_Idle : (io_op_in | pending_op);
    error = 0;
#if	0
    if (io_op != IO_Idle) {
        error = pmp.check_addr(addr_out);
        if (error)
            io_op = IO_Idle;
    }
#endif
    int isRead = (io_op & IO_ReadData);
    int isWrite = (io_op & IO_WriteData);
    AXIOp axiOp = (isRead) ? AXI_Read : (isWrite) ? AXI_Write : AXI_Idle;
    data_latched = 0;
    data_in = 0;
    stalled = !MasterFSM::fsm(axiOp, axi, addr_out, transfer_size, src0, &data_in, &data_latched);
    pending_op = (stalled) ? io_op : IO_Idle;
    active = (io_op != IO_Idle);
}
#elif (RV_MEM_DATA_WIDTH == 64) && (AXI_DATA_WIDTH == 32)
void RV_AXI4L::fsm(BIT cancel_io_insn, UINT3 io_op_in, RV_MEM_TYPE src0, AXI4L::CH *axi) {
    UINT3 io_op = (cancel_io_insn) ? IO_Idle : (io_op_in | pending_op);
    error = 0;
#if	0
    if (io_op != IO_Idle) {
        error = pmp.check_addr(addr_out);
        if (error)
            io_op = IO_Idle;
    }
#endif
    int isRead = (io_op & IO_ReadData);
    int isWrite = (io_op & IO_WriteData);
    AXIOp axiOp = (isRead) ? AXI_Read : (isWrite) ? AXI_Write : AXI_Idle;
    data_latched = 0;
    data_in = 0;

    AXI4L::AXI_DType    w_data_in = 0;
    auto w_transfer_size = transfer_size;
    auto w_burst_length = burst_length;
    AXI4L::AXI_DType w_src0;
    bool isOp64 = (io_op != IO_Idle) && (transfer_size == AXI4L::TRANSFER_SIZE_8);

    if (isOp64) {
        burst_length = w_burst_length * 2 + 1;
        w_transfer_size = AXI4L::TRANSFER_SIZE_4;
        if (state == 0) {
            w_src0 = (AXI4L::AXI_DType)(src0 & 0xffffffff);
        } else {
            w_src0 = (AXI4L::AXI_DType)(src0 >> 32);
        }
    } else {
        if ((addr_out & 0x4) == 0) {
            w_src0 = (AXI4L::AXI_DType)(src0 & 0xffffffff);
        } else {
            w_src0 = (AXI4L::AXI_DType)(src0 >> 32);
        }
    }

    stalled = !MasterFSM::fsm(axiOp, axi, addr_out, w_transfer_size, w_src0, &w_data_in, &data_latched);
    
    if (isOp64) {
        burst_length = w_burst_length;
        if (state == 0) {
            if (data_latched) {
                l_data_in = w_data_in;
                state = 1;
            }
            data_latched = 0;
        } else {
            if (data_latched) {
                data_in = ((RV_MEM_TYPE)w_data_in << 32) | l_data_in;
                state = 0;
            }
        }
    } else {
        if ((addr_out & 0x4) == 0) {
            data_in = (RV_MEM_TYPE)w_data_in;
        } else {
            data_in = (RV_MEM_TYPE)w_data_in << 32;
        }
    }
    pending_op = (stalled) ? io_op : IO_Idle;
    active = (io_op != IO_Idle);
}
#else
#error "Please check RV_MEM_DATA_WIDTH & AXI_DATA_WIDTH setting"
#endif

//#define DBG_MFSMW
#if defined(DBG_MFSMW)
extern CPU cpu1, cpu2;
#endif
#define SUPRESS_LOCK_ERROR_MSG
#define UNIFY_AXI_SIG_ASSIGN_CODE   /// 9/27/22
bool AXI4L::MasterFSM::fsm(UINT2 op, CH *axi, AXI_AType addr, UINT3 size, AXI_DType dout, AXI_DType *din, BIT *data_latched) {
    BIT stalled = 0;
#if 0 // #ifdef	__LLVM_C2RTL__ /// 9/27/22 : remove C2RTL-dependent source code
    axi->setMARead(out.raddr, out.rdat);
    axi->setMAWrite(out.waddr, out.wdat, out.wres);
#endif
    switch (rw_state) {
    case RW_Init:
        if (op == AXI_Read) {
            out.setRead(addr, size, 1, 1, burst_length, mem_prot);    /// ra_addr, ra_size, ra_valid, rd_ready
            burst_count = burst_length;
            in.resetRead();
            stalled = 1;
            rw_state = RW_RAddr;
        }
        else if (op == AXI_Write) {  /// isWrite
            AXI_STROBE strobe;
#if (AXI_DATA_WIDTH == 512)
            switch (size) {
            case TRANSFER_SIZE_1: strobe = 0x01 << (addr & 63); break;		// 0x3f
            case TRANSFER_SIZE_2: strobe = 0x03 << (addr & 62); break;		// 0x3e
            case TRANSFER_SIZE_4: strobe = 0x0f << (addr & 60); break;		// 0x3c
            case TRANSFER_SIZE_8: strobe = 0xff << (addr & 56); break;		// 0x38
            case TRANSFER_SIZE_16: strobe = 0xffff << (addr & 48); break;	// 0x30
            case TRANSFER_SIZE_32: strobe = 0xffffffff << (addr & 32); break;	// 0x20
            default:              strobe = 0xffffffffffffffffLL; break;
            }
#elif (AXI_DATA_WIDTH == 64)
            switch (size) {
            case TRANSFER_SIZE_1: strobe = 0x01 << (addr & 7); break;
            case TRANSFER_SIZE_2: strobe = 0x03 << (addr & 6); break;
            case TRANSFER_SIZE_4: strobe = 0x0f << (addr & 4); break;
            default:              strobe = 0xff; break;
            }
#elif (AXI_DATA_WIDTH == 32)
            switch (size) {
            case TRANSFER_SIZE_1: strobe = 0x1 << (addr & 3); break;
            case TRANSFER_SIZE_2: strobe = 0x3 << (addr & 2); break;
            default:              strobe = 0xf; break;
            }
#else
#error "Unsupported AXI_DATA_WIDTH"
#endif
            out.setWrite(addr, size, 1, dout, strobe, 1, (burst_length == 0), 1, burst_length, mem_prot); /// wa_addr, wa_size, wa_valid,wd_data,wd_strobe,wd_valid,wr_ready
            burst_count = burst_length;
            in.resetWrite();
            stalled = 1;
            *data_latched = 1;
            rw_state = RW_WAddr;
#if defined(DBG_MFSMW)
            int myID = (this == &cpu1.io.axim) ? 0 : 1;
            UINT32 cycle = (this == &cpu1.io.axim) ? cpu1.cycle : cpu2.cycle;
            printf("[%8d] AXIM-Write(cpu.%d) : addr(%08x), data(%08x)\n", cycle, myID, addr, dout);
#endif
        }
        break;
    case RW_RAddr: {
        int n_rflag = (in.rdat.valid << 1) | in.raddr.ready;
        if (axi->raddr.s.ready) {
            n_rflag |= 1;
            in.raddr.set(axi->raddr.s);
            out.raddr.reset();
        }
        if (axi->rdat.s.valid) {
            if (burst_count == 0) {
                n_rflag |= 2; in.rdat.set(axi->rdat.s);   out.rdat.reset();
            } else {
                burst_count--;
            }
            *din = axi->rdat.s.data;
            *data_latched = 1;
            if (axi->rdat.s.resp != RSP_OK) {
                printf("[AXI::Master] bad rdat.resp from slave!!!\n");
            }
        }
        if (n_rflag == 3) { rw_state = RW_Init; }
        else { stalled = 1; }
        break;
    }
    case RW_WAddr: {
        int n_wflag = (in.wres.valid << 2) | (in.wdat.ready << 1) | in.waddr.ready;
        int bad_resp = (in.wres.resp != RSP_OK);
        if (axi->waddr.s.ready) {
            n_wflag |= 1;
            in.waddr.set(axi->waddr.s);
            out.waddr.reset();
        }
        if (axi->wdat.s.ready) {
            if (burst_count == 0) {
                n_wflag |= 2;
                out.wdat.reset();
            } else {
                out.wdat.last = (burst_count == 1);
                out.wdat.data = dout;
                burst_count--;
                *data_latched = 1;
            }
            in.wdat.set(axi->wdat.s);
        }
        if (axi->wres.s.valid) {
            n_wflag |= 4; in.wres.set(axi->wres.s);   out.wres.reset();
            if (axi->wres.s.resp != RSP_OK) {
#if !defined(SUPRESS_LOCK_ERROR_MSG)
                printf("[AXI::Master] bad wres.resp from slave!!! (lock/unlock error?)\n");
#endif
                bad_resp = 1;
            }
        }
        if (n_wflag == 7) {  /// 6/23/17 : if (bad response) { try again; }
            rw_state = RW_Init;
            if (bad_resp) { stalled = 1; }  /// try again
        }
        else { stalled = 1; }
        break;
    }
    }
//#ifndef    __LLVM_C2RTL__
#if 1 // #ifndef    __LLVM_C2RTL__ /// 9/27/22 : remove C2RTL-dependent source code
    axi->setMARead(out.raddr, out.rdat);
    axi->setMAWrite(out.waddr, out.wdat, out.wres);
#endif
    return !stalled;// != 0;
}
