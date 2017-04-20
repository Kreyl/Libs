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
__attribute__ ((section("MyFlash")))
const uint32_t IData[(FLASH_PAGE_SIZE/4)] = { 0xCA115EA1 };

#if FLASH_2_BANKS
__attribute__ ((section("MyFlash2")))
const uint32_t IData2[(FLASH_PAGE_SIZE/4)] = { 0xCA115EA1 };
#endif


static uint8_t ErasePage(uint32_t PageAddress) {
    uint8_t status = Flash::WaitForLastOperation(FLASH_EraseTimeout);
    if(status == retvOk) {
        FLASH->CR |= FLASH_CR_PER_Set;
        FLASH->AR = PageAddress;
        FLASH->CR |= FLASH_CR_STRT_Set;
        // Wait for last operation to be completed
        status = Flash::WaitForLastOperation(FLASH_EraseTimeout);
        // Disable the PER Bit
        FLASH->CR &= FLASH_CR_PER_Reset;
    }
    return status;
}

static uint8_t ProgramWord(uint32_t Address, uint32_t Data) {
    uint8_t status = Flash::WaitForLastOperation(FLASH_ProgramTimeout);
    if(status == retvOk) {
        FLASH->CR |= FLASH_CR_PG_Set;
        // Program the new first half word
        *(volatile uint16_t*)Address = (uint16_t)Data;
        status = Flash::WaitForLastOperation(FLASH_ProgramTimeout);
        if(status == retvOk) {
            // Program the new second half word
            uint32_t tmp = Address + 2;
            *(volatile uint16_t*)tmp = Data >> 16;
            status = Flash::WaitForLastOperation(FLASH_ProgramTimeout);
        }
        FLASH->CR &= FLASH_CR_PG_Reset;  // Disable the PG Bit
    }
    return status;
}

static uint8_t SaveCommon(uint32_t *ptr, uint32_t ByteSz, uint32_t Addr) {
//    Uart.Printf("SaveCommon: Sz %u; Addr %08X\r", ByteSz, Addr);
    uint8_t status = retvOk;
    uint32_t DataWordCount = (ByteSz + 3) / 4;
    chSysLock();
    Flash::Unlock();
    // Erase flash
    Flash::ClearFlag(FLASH_SR_EOP | FLASH_SR_PGERR | FLASH_SR_WRPRTERR);   // Clear all pending flags
    status = ErasePage(Addr);
//    Uart.PrintfI("  Flash erase %u: %u\r", status);
    if(status != retvOk) {
        Uart.PrintfI("Flash erase error\r");
        goto end;
    }
    // Program flash
    for(uint32_t i=0; i<DataWordCount; i++) {
        status = ProgramWord(Addr, *ptr);
        if(status != retvOk) {
            Uart.PrintfI("Flash write error\r");
            goto end;
        }
        Addr += 4;
        ptr++;
    }
    end:
    Flash::Lock();
    chSysUnlock();
    return status;
}

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
    return SaveCommon(ptr, ByteSz, (uint32_t)&IData[0]);
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
