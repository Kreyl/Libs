/*
 * mem_msd_glue.cpp
 *
 *  Created on: 30 ���. 2016 �.
 *      Author: Kreyl
 */

#include "mem_msd_glue.h"
#include "uart.h"

uint8_t MSDRead(uint32_t BlockAddress, uint8_t *Ptr, uint32_t BlocksCnt) {
    //Uart.Printf("R Addr: %u; Cnt: %u\r", BlockAddress, BlocksCnt);
    Mem.Read(BlockAddress * MSD_BLOCK_SZ, Ptr, BlocksCnt * MSD_BLOCK_SZ);
    return OK;
}

uint8_t MSDWrite(uint32_t BlockAddress, uint8_t *Ptr, uint32_t BlocksCnt) {
//    Uart.Printf("WRT Addr: %u; Cnt: %u\r", BlockAddress, BlocksCnt);
    while(BlocksCnt != 0) {
        // Calculate Mem Sector addr
        uint32_t SectorStartAddr = BlockAddress * MEM_SECTOR_SZ;
        // Write renewed sector
        if(Mem.EraseAndWriteSector4k(SectorStartAddr, Ptr) != OK) return FAILURE;
        // Process variables
        BlockAddress += MSD_BLOCK_SZ;
        BlocksCnt--;
    }
    return OK;
}
