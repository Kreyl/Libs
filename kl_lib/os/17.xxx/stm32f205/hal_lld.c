/*
    ChibiOS - Copyright (C) 2006..2016 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/**
 * @file    STM32F4xx/hal_lld.c
 * @brief   STM32F4xx/STM32F2xx HAL subsystem low level driver source.
 *
 * @addtogroup HAL
 * @{
 */

#include "hal.h"


/**
 * @brief   Low level HAL driver initialization.
 *
 * @notapi
 */
void hal_lld_init(void) {

  /* Reset of all peripherals. AHB3 is not reseted because it could have
     been initialized in the board initialization file (board.c) and AHB2 is not
     present in STM32F410. */
  rccResetAHB1(~0);
#if !defined(STM32F410xx)
  rccResetAHB2(~0);
#endif
  rccResetAPB1(~RCC_APB1RSTR_PWRRST);
  rccResetAPB2(~0);

  /* PWR clock enabled.*/
  rccEnablePWRInterface(FALSE);

#if defined(STM32_DMA_REQUIRED)
  dmaInit();
#endif

  /* Programmable voltage detector enable.*/
#if STM32_PVD_ENABLE
  PWR->CR |= PWR_CR_PVDE | (STM32_PLS & STM32_PLS_MASK);
#endif /* STM32_PVD_ENABLE */
}
