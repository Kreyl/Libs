/*
 * mem_msd_glue.h
 *
 *  Created on: 30 ���. 2016 �.
 *      Author: Kreyl
 */

#ifndef MEM_MSD_GLUE_H__
#define MEM_MSD_GLUE_H__

#include "types.h"

namespace MsdMem {

uint32_t GetBlockCnt();
uint32_t GetBlockSz();

retv Read(uint32_t block_address, uint8_t *ptr, uint32_t blocks_cnt);
retv Write(uint32_t block_address, uint8_t *ptr, uint32_t blocks_cnt);

} // namespace

#endif // MEM_MSD_GLUE_H__
