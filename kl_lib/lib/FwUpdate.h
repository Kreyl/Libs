#pragma once

#include <inttypes.h>

//#define UpdateFromDisk

// Constants, see datasheet
#define FLASH_START_ADDR        0x08000000UL
#define FLASH_PAGE_SZ           2048L       // bytes in page, see datasheet
#define PAGE_SZ_WORD64          (FLASH_PAGE_SIZE/8)

#define FW_UPD_BUF_SZ           FLASH_PAGE_SZ

class FwUpdater_t {
private:
    uint32_t SzTotal, FlashStartAddr, CurrAddr, CurrPageN;
    uint8_t EraseBank2();
    uint8_t Write(uint32_t Addr, uint32_t Sz);
    uint32_t TotalLen = 0;
public:
    FwUpdater_t() {
        SzTotal = 0;
        CurrAddr = 0xFFFFFFFF;
        CurrPageN = 256; // Always 256, see refman
        // Get address of Bank2
        uint32_t FlashSz = *(volatile uint16_t*)0x1FFF75E0;
        if(FlashSz == 256) FlashStartAddr = 0x08020000;
        else if(FlashSz == 512) FlashStartAddr = 0x08040000;
        else FlashStartAddr = 0x08080000;
    }
    union {
        uint64_t __DDWord64;
        uint8_t Buf[FW_UPD_BUF_SZ];
    };
    void Restart();
    uint8_t WriteBuf(int32_t Sz, uint16_t *CrcIn);
    uint8_t CheckItAll(uint16_t *CrcIn);
#ifdef UpdateFromDisk
    uint8_t CheckFileOnDisk(const char *AFileName);
    uint8_t ReadFileOnDisk();
#endif
    void SwitchAndRun();
};

extern FwUpdater_t FwUpdater;


// ========================== Custom functions =================================

uint32_t CheckFWfile (const char *AFileName);
uint8_t ReadFWfile (uint32_t TotalLen);
