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
__attribute__ ((section("MyFlash1")))
const uint32_t IData[(FLASH_PAGE_SIZE/4)] = { 0xCA115EA1 };

#if FLASH_2_BANKS
__attribute__ ((section("MyFlash2")))
const uint32_t IData2[(FLASH_PAGE_SIZE/4)] = { 0xCA115EA1 };
#endif

namespace Flash {

void* GetFlashPointer() {
    return (void*)IData;
}

void LoadI(uint32_t *ptr, uint32_t ByteSz) {
    memcpy(ptr, IData, ByteSz);
}
void Load(uint32_t *ptr, uint32_t ByteSz) {
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

uint8_t Save(uint32_t *ptr, uint32_t ByteSz) {
    return Flash::ProgramBuf(ptr, ByteSz, (uint32_t)&IData[0]);
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
