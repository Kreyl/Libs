/*
 * kl_buf.h
 *
 *  Created on: 07.04.2013
 *      Author: kreyl
 */

#pragma once

#include "ch.h"
#include "string.h" // for memcpy
#include <kl_lib.h>

// Lib version
#define KL_BUF_VERSION      "20151102_1339"

enum AddRslt_t {addrOk, addrFail, addrSwitch};

// Simple buffer
struct Buf_t {
    uint8_t *Ptr;
    uint32_t Length;
};

#if 1 // ========================= Circular buffer =============================
template <typename T, uint32_t Sz>
class CircBuf_t {
protected:
    uint32_t IFullSlotsCount=0;
    T IBuf[Sz], *PRead=IBuf, *PWrite=IBuf;
public:
    uint8_t Get(T *p) {
        if(IFullSlotsCount == 0) return retvEmpty;
        memcpy(p, PRead, sizeof(T));
        if(++PRead > (IBuf + Sz - 1)) PRead = IBuf;     // Circulate buffer
        IFullSlotsCount--;
        return retvOk;
    }
    uint8_t GetPAndMove(T **pp) {
    	if(IFullSlotsCount == 0) return retvEmpty;
    	*pp = PRead;
        if(++PRead > (IBuf + Sz - 1)) PRead = IBuf;     // Circulate buffer
        IFullSlotsCount--;
        return retvOk;
    }
    uint8_t GetLastP(T **pp) {
    	if(IFullSlotsCount == 0) return retvEmpty;
		*pp = PRead;
		return retvOk;
    }

    uint8_t PutAnyway(T *p) {
		memcpy(PWrite, p, sizeof(T));
		if(++PWrite > (IBuf + Sz - 1)) PWrite = IBuf;   // Circulate buffer
		if(IFullSlotsCount < Sz) IFullSlotsCount++;
		return retvOk;
	}
    uint8_t Put(T *p) {
        if(IFullSlotsCount >= Sz) return retvOverflow;
        return PutAnyway(p);
    }

    inline bool IsEmpty() { return (IFullSlotsCount == 0); }
    inline uint32_t GetEmptyCount() { return Sz-IFullSlotsCount; }
    inline uint32_t GetFullCount()  { return IFullSlotsCount; }
    void Flush(uint32_t ALength) {
        TRIM_VALUE(ALength, IFullSlotsCount);
        IFullSlotsCount -= ALength;
        uint32_t PartSz = (IBuf + Sz) - PRead;
        if(ALength >= PartSz) {
            ALength -= PartSz;
            PRead = IBuf + ALength;
        }
        else PRead += ALength;
    }
    void Flush() {
        IFullSlotsCount = 0;
        PRead = PWrite;
    }
};
#endif

/*
template <typename T>
class CircBufSemaphored_t : public CircBuf_t<T> {
private:
    BinarySemaphore SemAddItem;
public:
    uint8_t PutWithTimeout(T *p, uint32_t Timeout_ms) {
        if(Timeout_ms != TIME_INFINITE) Timeout_ms = MS2ST(Timeout_ms);
        if(chBSemWaitTimeout(&SemAddItem, Timeout_ms) != RDY_OK) return FAILURE;
        this->Put(p);
        // Check if buf is not full
        if(this->IFullSlotsCount != this->IBufSize) chBSemSignal(&SemAddItem);
        return OK;
    }

    uint8_t Get(T *p) {
        if(this->IFullSlotsCount == 0) return FAILURE;
        memcpy(p, this->PRead, sizeof(T));
        if(++this->PRead > (this->IPBuf + this->IBufSize - 1)) this->PRead = this->IPBuf;     // Circulate buffer
        this->IFullSlotsCount--;
        chBSemSignal(&SemAddItem);
        return OK;
    }

    void Reset() {
        this->PRead = this->IPBuf;
        this->PWrite = this->IPBuf;
        this->IFullSlotsCount = 0;
        chBSemReset(&SemAddItem, NOT_TAKEN);
    }

    void Init(T *PBuf, uint32_t Sz) {
        chBSemInit(&SemAddItem, NOT_TAKEN);
        this->IPBuf = PBuf;
        this->IBufSize = Sz;
        Reset();
    }
};
*/

#if 1 // =========== Buffer for simple types, like uint8_t etc. ================
template <typename T, uint32_t Sz>
class CircBufNumber_t : public CircBuf_t<T, Sz> {
public:
    uint32_t Get(T *p, uint32_t ALength) {
        uint32_t Cnt = 0;
        while(this->IFullSlotsCount > 0 and Cnt < ALength) {
            *p++ = *this->PRead++;
            if(this->PRead >= (this->IBuf + Sz)) this->PRead = this->IBuf;
            this->IFullSlotsCount--;
            Cnt++;
        }
        return Cnt; // return how many items were written
    }

    uint8_t Put(T *p, uint32_t Length) {
        uint8_t Rslt = retvFail;
        if(this->GetEmptyCount() >= Length) {    // check if Buffer overflow
            this->IFullSlotsCount += Length;                      // 'Length' slots will be occupied
            uint32_t PartSz = (this->IBuf + Sz) - this->PWrite;  // Data from PWrite to right bound
            if(Length > PartSz) {
                memcpy(this->PWrite, p, PartSz);
                this->PWrite = this->IBuf;     // Start from beginning
                p += PartSz;
                Length -= PartSz;
            }
            memcpy(this->PWrite, p, Length);
            this->PWrite += Length;
            if(this->PWrite >= (this->IBuf + Sz)) this->PWrite = this->IBuf; // Circulate pointer
            Rslt = retvOk;
        }
        return Rslt;
    }

    uint8_t Get(T *p) {
        if(this->IFullSlotsCount == 0) return retvEmpty;
        *p = *this->PRead;
        if(++this->PRead > (this->IBuf + Sz - 1)) this->PRead = this->IBuf;     // Circulate buffer
        this->IFullSlotsCount--;
        return retvOk;
    }

    uint8_t GetAndDoNotRemove(T *p) {
        if(this->IFullSlotsCount == 0) return retvEmpty;
        *p = *this->PRead;
        return retvOk;
    }

    uint8_t Put(T Value) {
        *this->PWrite = Value;
        if(++this->PWrite > (this->IBuf + Sz - 1)) this->PWrite = this->IBuf;   // Circulate buffer
        if(this->IFullSlotsCount >= Sz) return retvOverflow;
        else {
            this->IFullSlotsCount++;
            return retvOk;
        }
    }

    uint8_t PutIfNotOverflow(T *p) {
        if(this->IFullSlotsCount >= Sz) return retvOverflow;
        else return Put(p);
    }
};
#endif

#if 1 // ========================== Double buffer ==============================
// Append to buf 1, until filled up. Switch buffers and return addrSwitch.
template <typename T, uint32_t Sz>
class DoubleBuf_t {
private:
    T Buf1[Sz], Buf2[Sz];
    T *pWrite = Buf1;
    T *pRead = Buf2;
public:
    AddRslt_t Append(T Value) {
        *pWrite++ = Value;
        if(pRead == Buf1) {             // Check if overflow
            if(pWrite >= &Buf2[Sz]) {    // Switch buffers
                pRead = Buf2;
                pWrite = Buf1;
                return addrSwitch;
            }
            else return addrOk;
        }
        else {  // pRead == Buf2
            if(pWrite >= &Buf1[Sz]) {    // Switch buffers
                pRead = Buf1;
                pWrite = Buf2;
                return addrSwitch;
            }
            else return addrOk;
        }
    }
    T* GetBufToRead() { return pRead; }
};

#endif

#if 1 // ========================= Counting buffer =============================
template <typename T, uint32_t Sz>
class CountingBuf_t {
private:
    T IBuf[Sz];
    uint32_t Cnt;
public:
    void Add(T Value) {
        for(uint32_t i=0; i<Cnt; i++) {
            if(IBuf[i] == Value) return;   // do not add what exists
        }
        IBuf[Cnt] = Value;
        Cnt++;
    }
    uint32_t GetCount() { return Cnt; }
    void Clear() { Cnt = 0; }
};
#endif

#if 1 // ============================ LIFO =====================================
template <typename T, uint32_t Sz>
class LifoNumber_t {
protected:
    uint32_t Cnt=0;
    T IBuf[Sz];
public:
    uint8_t Put(T Value) {
        if(Cnt == Sz) return retvOverflow;
        IBuf[Cnt] = Value;
        Cnt++;
        return retvOk;
    }

    uint8_t Get(T *p) {
        if(Cnt == 0) return retvEmpty;
        Cnt--;
        *p = IBuf[Cnt];
        return retvOk;
    }

    uint8_t GetAndDoNotRemove(T *p) {
        if(Cnt == 0) return retvEmpty;
        *p = IBuf[Cnt-1];
        return retvOk;
    }

    inline uint32_t GetFullCount()  { return Cnt; }
};

#endif
