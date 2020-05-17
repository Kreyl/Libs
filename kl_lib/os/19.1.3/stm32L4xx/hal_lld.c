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
 * @file    STM32L4xx/hal_lld.c
 * @brief   STM32L4xx HAL subsystem low level driver source.
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

  /* Reset of all peripherals.
     Note, GPIOs are not reset because initialized before this point in
     board files.*/
  rccResetAHB1(~0);
  rccResetAHB2(~STM32_GPIO_EN_MASK);
  rccResetAHB3(~0);
  rccResetAPB1R1(~RCC_APB1RSTR1_PWRRST);
  rccResetAPB1R2(~0);
  rccResetAPB2(~0);

  /* PWR clock enabled.*/
  rccEnablePWRInterface(true);

  /* DMA subsystems initialization.*/
#if defined(STM32_DMA_REQUIRED)
  dmaInit();
#endif

  /* IRQ subsystem initialization.*/
  irqInit();

  /* Programmable voltage detector enable.*/
#if STM32_PVD_ENABLE
  PWR->CR2 = PWR_CR2_PVDE | (STM32_PLS & STM32_PLS_MASK);
#else
  PWR->CR2 = 0;
#endif /* STM32_PVD_ENABLE */

  /* Enabling independent VDDUSB.*/
#if HAL_USE_USB
  PWR->CR2 |= PWR_CR2_USV;
#endif /* HAL_USE_USB */

  /* Enabling independent VDDIO2 required by GPIOG.*/
#if STM32_HAS_GPIOG
  PWR->CR2 |= PWR_CR2_IOSV;
#endif /* STM32_HAS_GPIOG */
}

/** @} */
