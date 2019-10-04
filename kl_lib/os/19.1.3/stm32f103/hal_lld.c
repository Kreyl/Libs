/*
    ChibiOS - Copyright (C) 2006..2018 Giovanni Di Sirio

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
 * @file    STM32F1xx/hal_lld.c
 * @brief   STM32F1xx HAL subsystem low level driver source.
 *
 * @addtogroup HAL
 * @{
 */

#include "hal.h"

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if defined(STM32_DMA_REQUIRED) || defined(__DOXYGEN__)
#if defined(STM32_DMA2_CH45_HANDLER) || defined(__DOXYGEN__)
/**
 * @brief   DMA2 streams 4 and 5 shared ISR.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_DMA2_CH45_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  /* Check on channel 4 of DMA2.*/
  dmaServeInterrupt(STM32_DMA2_STREAM4);

  /* Check on channel 5 of DMA2.*/
  dmaServeInterrupt(STM32_DMA2_STREAM5);

  OSAL_IRQ_EPILOGUE();
}
#endif /* defined(STM32_DMA2_CH45_HANDLER) */
#endif /* defined(STM32_DMA_REQUIRED) */

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level HAL driver initialization.
 *
 * @notapi
 */
void hal_lld_init(void) {

  /* Reset of all peripherals.*/
  rccResetAPB1(0xFFFFFFFF);
  rccResetAPB2(0xFFFFFFFF);

  /* PWR and BD clocks enabled.*/
  rccEnablePWRInterface(true);
  rccEnableBKPInterface(true);

  /* DMA subsystems initialization.*/
#if defined(STM32_DMA_REQUIRED)
  dmaInit();
#endif

  /* IRQ subsystem initialization.*/
  irqInit();

  /* Programmable voltage detector enable.*/
#if STM32_PVD_ENABLE
  PWR->CR |= PWR_CR_PVDE | (STM32_PLS & STM32_PLS_MASK);
#endif /* STM32_PVD_ENABLE */
}

/** @} */
