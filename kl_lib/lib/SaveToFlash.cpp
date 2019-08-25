/*
 * SaveToFlash.cpp
 *
 *  Created on: 5 февр. 2017 г.
 *      Author: Kreyl
 */

#include "SaveToFlash.h"
#include <cstring>  // For memcpy
#include "kl_lib.h"
#include "uart.h"

// Data inside the Flash. Init value is dummy.
// For some reason "aligned" attribute does not work, therefore array used here.
#ifdef STM32L4XX
#define WORD64_CNT     (FLASH_PAGE_SIZE/8)
__attribute__ ((section("MyFlash1")))
volatile const uint64_t IData[WORD64_CNT] = { 0xCA115EA1 };
#else
__attribute__ ((section("MyFlash1")))
volatile const uint32_t IData[(FLASH_PAGE_SIZE/4)] = { 0xCA115EA1 };

#if FLASH_2_BANKS
__attribute__ ((section("MyFlash2")))
volatile const uint32_t IData2[(FLASH_PAGE_SIZE/4)] = { 0xCA115EA1 };
#endif
#endif

namespace Flash {

void* GetFlashPointer() {
    return (void*)IData;
}

void LoadI(void *ptr, uint32_t ByteSz) {
    memcpy(ptr, (const void*)IData, ByteSz);
}
void Load(void *ptr, uint32_t ByteSz) {
    chSysLock();
    LoadI(ptr, ByteSz);
    chSysUnlock();
}

#if FLASH_2_BANKS
void Load2I(uint32_t *ptr, uint32_t ByteSz) {
    memcpy(ptr, IData2, ByteSz);
}
void Load2(uint32_t *ptr, uint32_t ByteSz) {
    chSysLock();
    Load2I(ptr, ByteSz);
    chSysUnlock();
}
#endif

// Data must be aligned to uint64_t
uint8_t Save(void *ptr, uint32_t ByteSz) {
#ifdef STM32L4XX
    uint32_t Addr = (uint32_t)&IData;
    uint8_t Rslt = retvOk;
    uint64_t *Buf = (uint64_t*)ptr;
    uint32_t DWordCnt = (ByteSz + 7) / 8;
    chSysLock();
    Flash::LockFlash(); // Otherwise HardFault occurs after flashing and without reset
    Flash::UnlockFlash();
    chSysUnlock();

    if(Flash::ErasePage(Addr / FLASH_PAGE_SIZE) != retvOk) {
        Printf("\rPage Erase fail\r");
        chThdSleepMilliseconds(45);
        Rslt = retvFail;
        goto End;
    }

    for(uint32_t i=0; i<DWordCnt; i++) {
        if(Flash::ProgramDWord(Addr, Buf[i]) != retvOk) {
            Printf("Write Fail\r");
            chThdSleepMilliseconds(45);
            Rslt = retvFail;
            goto End;
        }
        Addr += 8;
    }
    End:
    chSysLock();
    Flash::LockFlash();
    chSysUnlock();
    return Rslt;
#else
    return Flash::ProgramBuf(ptr, ByteSz, (uint32_t)&IData[0]);
#endif
}

#if FLASH_2_BANKS
uint8_t Save2I(uint32_t *ptr, uint32_t ByteSz) {
    return SaveCommon(ptr, ByteSz, (uint32_t)&IData2[0]);
}
uint8_t Save2(uint32_t *ptr, uint32_t ByteSz) {
    chSysLock();
    uint8_t Rslt = Save2I(ptr, ByteSz);
    chSysUnlock();
    return Rslt;
}
#endif
} // namespace
