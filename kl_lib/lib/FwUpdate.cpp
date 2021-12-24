/*
 * UpdateFw.cpp
 *
 *  Created on: May 5, 2020
 *      Author: layst
 */

#include "kl_crc.h"
#include "FwUpdate.h"
#include "kl_lib.h"
#include "shell.h"
#ifdef UpdateFromDisk
#include "kl_fs_utils.h"
#endif

FwUpdater_t FwUpdater;

void FwUpdater_t::Restart() {
    SzTotal = 0;
    CurrAddr = FlashStartAddr;
    EraseBank2();
}

uint8_t FwUpdater_t::WriteBuf(int32_t Sz, uint16_t *CrcIn) {
    // Restart if not yet
    if(CurrAddr == 0xFFFFFFFF) Restart();
    uint16_t crc = Crc::CalculateCRC16HW(Buf, Sz);
    if(crc != *CrcIn) {
        *CrcIn = crc;
        return retvCRCError;
    }
//    Printf("%A; 0x%04X, 0x%04X\r", Buf, Sz, ' ', CrcIn, crc);
    if(Write(CurrAddr, Sz) == retvOk) {
        CurrAddr += Sz;
        SzTotal += Sz;
        return retvOk;
    }
    else return retvFail;
}

uint8_t FwUpdater_t::CheckItAll(uint16_t *CrcIn) {
    if(SzTotal) {
        uint16_t crc = Crc::CalculateCRC16HW((uint8_t*)FlashStartAddr, SzTotal);
        if(crc != *CrcIn) {
            *CrcIn = crc;
            return retvCRCError;
        }
        else return retvOk;
    }
    else return retvFail;
}

uint8_t FwUpdater_t::EraseBank2() {
    // Unlock flash
    chSysLock();
    Flash::LockFlash(); // Otherwise HardFault occurs after flashing and without reset
    Flash::UnlockFlash();
    chSysUnlock();
    // Erase other bank
    while(FLASH->SR & FLASH_SR_BSY); // Wait for flash to become idle
    FLASH->SR |= 0xC3FA; // Clean err flags
    // Select bank to clear
    if(FLASH->OPTR & FLASH_OPTR_BFB2) FLASH->CR |= FLASH_CR_MER1; // if current bank is B, clean A
    else FLASH->CR |= FLASH_CR_MER2; // else clean B
    // Clean it
    FLASH->CR |= FLASH_CR_STRT;
    while(FLASH->SR & FLASH_SR_BSY); // Wait for flash to become idle
    FLASH->CR &= ~(FLASH_CR_MER1 | FLASH_CR_MER2); // Clear MassErase flags
    Flash::LockFlash();
    return retvOk;
}

void FwUpdater_t::SwitchAndRun() {
    Flash::ToggleBootBankAndReset();
}

uint8_t FwUpdater_t::Write(uint32_t Addr, uint32_t Sz) {
    // Disable flash and instruction cache
    uint32_t OldFlashReg = FLASH->ACR;
    FLASH->ACR &= ~(FLASH_ACR_ICEN | FLASH_ACR_PRFTEN);

    // Unlock flash
    chSysLock();
    Flash::LockFlash(); // Otherwise HardFault occurs after flashing and without reset
    Flash::UnlockFlash();
    chSysUnlock();

    // Prepare
    uint32_t Address = Addr;
    uint32_t *p32 = (uint32_t*)Buf;
    uint32_t *PEnd = (uint32_t*)(Buf + Sz);
    while(FLASH->SR & FLASH_SR_BSY); // Wait for flash to become idle
    FLASH->SR |= 0xC3FB; // Clean err flags
    uint8_t Rslt = retvOk;
    // ==== Do it ====
    chSysLock();
    FLASH->CR |= FLASH_CR_PG;
    while(p32 < PEnd and Rslt == retvOk) {
        *(volatile uint32_t*)Address = *p32++;
        Address += 4;
        *(volatile uint32_t*)Address = *p32++;
        Address += 4;
        // Wait completion
        while(FLASH->SR & FLASH_SR_BSY);
        if(FLASH->SR & FLASH_SR_EOP) FLASH->SR |= FLASH_SR_EOP;
        // Check for errors
        if(FLASH->SR & 0xC3FB) {
            Rslt = retvFail;
            PrintfI("SR: %X\r", FLASH->SR);
            break;
        }
    }
    FLASH->CR &= ~FLASH_CR_PG;
    chSysUnlock();

    Flash::LockFlash();
    // Reset instruction cache and Restore Flash settings
    FLASH->ACR |= FLASH_ACR_ICRST;
    FLASH->ACR &= ~FLASH_ACR_ICRST;
    FLASH->ACR = OldFlashReg;
    return Rslt;
}

#ifdef UpdateFromDisk
uint8_t FwUpdater_t::CheckFileOnDisk(const char *AFileName) {
    // Try open file, jump to main app if not found
    if(f_findfirst(&Dir, &FileInfo, "", AFileName) != FR_OK) {
        Printf("File search fail\r");
        chThdSleepMilliseconds(99);
        return retvFail;   // OnError();
    }
    if(FileInfo.fname[0] == 0) {
        Printf("%S not found\r", AFileName);
        chThdSleepMilliseconds(99);
        return retvFail;   // OnError();
    }
    Printf("Found: %S\r", FileInfo.fname);
    if(TryOpenFileRead(FileInfo.fname, &CommonFile) != retvOk) return retvFail;    // OnError();
    TotalLen = f_size(&CommonFile);
    return retvOk;
}

uint8_t FwUpdater_t::ReadFileOnDisk() {
    // ==== Read file block by block, do not write first page ====
    uint32_t BytesCnt;
    while(TotalLen != 0) {
        // How many bytes to read?
        BytesCnt = MIN_(TotalLen, FLASH_PAGE_SZ);
        TotalLen -= BytesCnt;
        // Read block
        if(f_read(&CommonFile, Buf, BytesCnt, &BytesCnt) != FR_OK) {
            Printf("Read error\r");
            return retvFail;
        }

        if(Write(CurrAddr, BytesCnt) == retvOk) {
            CurrAddr += BytesCnt;
            SzTotal += BytesCnt;
        }
        else return retvFail;
    } // while
    return DeliteFileOnDisk();
}
uint8_t FwUpdater_t::ReadFileOnDiskToBuff(uint32_t &BytesCnt, uint16_t &CrcOut) {
    // ==== Read file block by block, do not write first page ====
    if (TotalLen != 0) {
        // How many bytes to read?
        BytesCnt = MIN_(TotalLen, FLASH_PAGE_SZ);
        TotalLen -= BytesCnt;
        // Read block
        if(f_read(&CommonFile, Buf, BytesCnt, &BytesCnt) != FR_OK) {
            Printf("Read error\r");
            return retvFail;
        }
        CrcOut = Crc::CalculateCRC16HWDMA(Buf, BytesCnt);
        return retvOk;
    } // if
    else return retvEmpty;
}
uint8_t FwUpdater_t::CalcCrcOfFullyFile(uint16_t &CrcOut) {
    uint32_t BytesCnt;
    uint16_t Crc = CRC_INITVALUE;
    f_close(&CommonFile);
    if(TryOpenFileRead(FileInfo.fname, &CommonFile) != retvOk) return retvFail;
    TotalLen = f_size(&CommonFile);
    while(TotalLen != 0) {
        // How many bytes to read?
        BytesCnt = MIN_(TotalLen, FLASH_PAGE_SZ);
        TotalLen -= BytesCnt;
        // Read block
        if(f_read(&CommonFile, Buf, BytesCnt, &BytesCnt) != FR_OK) {
            Printf("Read error\r");
            return retvFail;
        }
        Crc = Crc::CalculateCRC16HW(Buf, BytesCnt, Crc);
    } // while
    CrcOut = Crc;
    return retvOk;
}
uint8_t FwUpdater_t::DeliteFileOnDisk() {
    Printf("\rWriting done\r");
    chThdSleepMilliseconds(99);
    f_close(&CommonFile);
    // Remove firmware file
    f_unlink(FileInfo.fname);
    chThdSleepMilliseconds(99);
    return retvOk;
}

#endif
