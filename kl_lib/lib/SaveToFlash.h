/*
 * SaveToFlash.h
 *
 *  Created on: 5 февр. 2017 г.
 *      Author: Kreyl
 */

#pragma once

#include "inttypes.h"
#include "chtypes.h"
#include "board.h"

/* Time of saving: 32ms @ 8MHz
 *
 * Linker script (i.e. rules.ld) must contain section aligned to some page address.
 ***************************************
 *  Here is place to read and write to *
 ***************************************
SECTIONS {
    MyFlash1 0x0800E400 :
    {
        *(MyFlash)
    } > flash0
}
*/

#if defined STM32L476xx || defined STM32L433xx
#define FLASH_PAGE_SIZE     2048UL
#else
#define FLASH_PAGE_SIZE     1024UL
#endif

#define FLASH_2_BANKS       FALSE

namespace Flash {

void Load(void *ptr, uint32_t ByteSz);
void LoadI(void *ptr, uint32_t ByteSz);
uint8_t Save(void *ptr, uint32_t ByteSz);

void* GetFlashPointer();

#if FLASH_2_BANKS
void Load2(uint32_t *ptr, uint32_t ByteSz);
void Load2I(uint32_t *ptr, uint32_t ByteSz);
uint8_t Save2(uint32_t *ptr, uint32_t ByteSz);
uint8_t Save2I(uint32_t *ptr, uint32_t ByteSz);
#endif

}
