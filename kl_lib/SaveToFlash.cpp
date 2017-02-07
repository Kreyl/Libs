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

#define FLASH_KEY1          ((uint32_t)0x45670123)
#define FLASH_KEY2          ((uint32_t)0xCDEF89AB)
#define CR_LOCK_Set         ((uint32_t)0x00000080)
#define CR_PER_Set          ((uint32_t)0x00000002)
#define CR_PER_Reset        ((uint32_t)0x00001FFD)
#define CR_STRT_Set         ((uint32_t)0x00000040)
#define CR_PG_Set           ((uint32_t)0x00000001)
#define CR_PG_Reset         ((uint32_t)0x00001FFE)
#define EraseTimeout        ((uint32_t)0x000B0000)
#define ProgramTimeout      ((uint32_t)0x00002000)

// Data inside the Flash. Init value is dummy.
// For some reason "aligned" attribute does not work, therefore array used here.
__attribute__ ((section("MyFlash")))
const uint32_t IData[(FLASH_PAGE_SIZE/4)] = { 0xCA115EA1 };

static inline void Unlock() {
    FLASH->KEYR = FLASH_KEY1;
    FLASH->KEYR = FLASH_KEY2;
}
static inline void Lock(void) { FLASH->CR |= CR_LOCK_Set; }
static inline void ClearFlag(uint32_t FLASH_FLAG) { FLASH->SR = FLASH_FLAG; }

static uint8_t GetBank1Status(void) {
    if(FLASH->SR & FLASH_SR_BSY) return BUSY;
    else if(FLASH->SR & FLASH_SR_PGERR) return FAILURE;
    else if(FLASH->SR & FLASH_SR_WRPRTERR) return FAILURE;
    else return OK;
}

static uint8_t WaitForLastOperation(uint32_t Timeout) {
    uint8_t status = OK;
    // Wait for a Flash operation to complete or a TIMEOUT to occur
    do {
        status = GetBank1Status();
        Timeout--;
    } while((status == BUSY) and (Timeout != 0x00));
    if(Timeout == 0x00) status = TIMEOUT;
    return status;
}

static uint8_t ErasePage(uint32_t PageAddress) {
    uint8_t status = WaitForLastOperation(EraseTimeout);
    if(status == OK) {
        FLASH->CR |= CR_PER_Set;
        FLASH->AR = PageAddress;
        FLASH->CR |= CR_STRT_Set;
        // Wait for last operation to be completed
        status = WaitForLastOperation(EraseTimeout);
        // Disable the PER Bit
        FLASH->CR &= CR_PER_Reset;
    }
    return status;
}

static uint8_t ProgramWord(uint32_t Address, uint32_t Data) {
    uint8_t status = WaitForLastOperation(ProgramTimeout);
    if(status == OK) {
        FLASH->CR |= CR_PG_Set; // program the new first half word
        *(__IO uint16_t*)Address = (uint16_t)Data;
        /* Wait for last operation to be completed */
        status = WaitForLastOperation(ProgramTimeout);
        if(status == OK) {
            // program the new second half word
            uint32_t tmp = Address + 2;
            *(__IO uint16_t*)tmp = Data >> 16;
            /* Wait for last operation to be completed */
            status = WaitForLastOperation(ProgramTimeout);
            /* Disable the PG Bit */
            FLASH->CR &= CR_PG_Reset;
        }
        else FLASH->CR &= CR_PG_Reset;  // Disable the PG Bit
    }
    return status;
}

namespace Flash {

void Load(uint32_t *ptr, uint32_t ByteSz) {
    memcpy(ptr, IData, ByteSz);
}

uint8_t Save(uint32_t *ptr, uint32_t ByteSz) {
    uint8_t status = OK;
    uint32_t FAddr = (uint32_t)&IData[0];
//    Uart.PrintfI("F addr: %08X\r", FAddr);
    uint32_t DataWordCount = (ByteSz + 3) / 4;
    chSysLock();
    Unlock();
    // Erase flash
    ClearFlag(FLASH_SR_EOP | FLASH_SR_PGERR | FLASH_SR_WRPRTERR);   // Clear all pending flags
    status = ErasePage(FAddr);
    //Uart.Printf("  Flash erase %u: %u\r", i, FLASHStatus);
    if(status != OK) {
        Uart.PrintfI("Flash erase error\r");
        goto end;
    }
    // Program flash
    for(uint32_t i=0; i<DataWordCount; i++) {
        status = ProgramWord(FAddr, *ptr);
        if(status != OK) {
            Uart.PrintfI("Flash write error\r");
            goto end;
        }
        FAddr += 4;
        ptr++;
    }
    Lock();
    end:
    chSysUnlock();
    return status;
}

} // namespace
