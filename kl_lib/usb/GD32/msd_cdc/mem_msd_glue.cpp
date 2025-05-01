/*
 * mem_msd_glue.cpp
 *
 *  Created on: 2016
 *      Author: Kreyl
 */

#include "mem_msd_glue.h"
#include "shell.h"
#include "diskio.h"
#include "gd_sd.h"

namespace MsdMem {


uint32_t GetBlockCnt() { return sd.capacity_blk; }

uint32_t GetBlockSz() { return SD_BLOCK_SIZE; }

retv Read(uint32_t block_address, uint8_t *ptr, uint32_t blocks_cnt) {
//    Printf("R %u %u\r", BlockAddress, BlocksCnt);
    // return SpiFlash.ReadQ(BlockAddress * BlockSz, Ptr, BlocksCnt * BlockSz);
//    Mem.Read(BlockAddress * MSD_BLOCK_SZ, Ptr, BlocksCnt * MSD_BLOCK_SZ);
    if(disk_read(0, ptr, block_address, blocks_cnt) == RES_OK) return retv::Ok;
    else return retv::Fail;
//    memcpy(Ptr, &buf[BlockAddress*MsdBlockSz], BlocksCnt*MsdBlockSz);
//    return retv::Ok;
}

retv Write(uint32_t block_address, uint8_t *ptr, uint32_t blocks_cnt) {
//    Printf("W %u %u\r", BlockAddress, BlocksCnt);
#if 0 // SPI FLASH
    uint32_t PageCnt = (BlocksCnt * BlockSz) / SPIFLASH_PAGE_SZ;
    // Erase sectors
    uint32_t Addr = BlockAddress * BlockSz;
    while(BlocksCnt) {
        if(SpiFlash.EraseSector4k(Addr) != retv::Ok) return retv::Fail;
        Addr += BlockSz;
        BlocksCnt--;
    }
    // Write data page by page
    Addr = BlockAddress * BlockSz;
    while(PageCnt) {
//        if(SpiFlash.WritePage(Addr, Ptr, SPIFLASH_PAGE_SZ) != retv::Ok) {
        if(SpiFlash.WritePageQ(Addr, Ptr, SPIFLASH_PAGE_SZ) != retv::Ok) return retv::Fail;
        Addr += SPIFLASH_PAGE_SZ;
        Ptr += SPIFLASH_PAGE_SZ;
        PageCnt--;
    }
    return retv::Ok;
#endif
//    memcpy(&buf[BlockAddress*MsdBlockSz], Ptr, BlocksCnt*MsdBlockSz);
#if 1 // SD
   if(disk_write(0, ptr, block_address, blocks_cnt) == RES_OK) return retv::Ok;
   else return retv::Fail;
#endif
#if 0 // Some memory
   while(BlocksCnt != 0) {
        // Calculate Mem Sector addr
       uint32_t SectorStartAddr = block_address * MEM_SECTOR_SZ;
        // Write renewed sector
       if(Mem.EraseAndWriteSector4k(SectorStartAddr, Ptr) != OK) return FAILURE;
        // Process variables
       block_address += MSD_BLOCK_SZ;
       blocks_cnt--;
   }
   return retv::Ok;
#endif
}

} // namespace
