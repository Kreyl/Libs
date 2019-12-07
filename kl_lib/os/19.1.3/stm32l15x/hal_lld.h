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
 * @file    STM32L1xx/hal_lld.h
 * @brief   STM32L1xx HAL subsystem low level driver header.
 * @pre     This module requires the following macros to be defined in the
 *          @p board.h file:
 *          - STM32_LSECLK.
 *          - STM32_HSECLK.
 *          - STM32_HSE_BYPASS (optionally).
 *          .
 *          One of the following macros must also be defined:
 *          - STM32L100xB, STM32L100xBA, STM32L100xC.
 *          - STM32L151xB, STM32L151xBA, STM32L151xC, STM32L151xCA,
 *            STM32L151xD, STM32L151xDX, STM32L151xE.
 *          - STM32L152xB, STM32L152xBA, STM32L152xC, STM32L152xCA,
 *            STM32L152xD, STM32L152xDX, STM32L152xE.
 *          - STM32L162xC, STM32L162xCA, STM32L162xD, STM32L162xDX,
 *            STM32L162xE.
 *          .
 *
 * @addtogroup HAL
 * @{
 */

#ifndef HAL_LLD_H
#define HAL_LLD_H

#include "stm32_registry.h"
#include "mcuconf.h"

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @name    Platform identification
 * @{
 */
#if defined(STM32L100xB) || defined(STM32L151xB) ||                         \
    defined(STM32L152xB) || defined(__DOXYGEN__)
#define PLATFORM_NAME           "STM32L1xx Ultra Low Power Medium Density"

#elif defined(STM32L100xBA) || defined(STM32L100xC)  ||                     \
      defined(STM32L151xBA) || defined(STM32L151xC)  ||                     \
      defined(STM32L151xCA) || defined(STM32L152xBA) ||                     \
      defined(STM32L152xC)  || defined(STM32L152xCA) ||                     \
      defined(STM32L162xC)  || defined(STM32L162xCA)
#define PLATFORM_NAME           "STM32L1xx Ultra Low Power Medium Density Plus"

#elif defined(STM32L151xD)  || defined(STM32L151xDX) ||                     \
      defined(STM32L151xE)  || defined(STM32L152xD)  ||                     \
      defined(STM32L152xDX) || defined(STM32L152xE)  ||                     \
      defined(STM32L162xD)  || defined(STM32L162xDX) ||                     \
      defined(STM32L162xE)
#define PLATFORM_NAME           "STM32L1xx Ultra Low Power High Density"

#else
#error "STM32L1xx device not specified"
#endif

/**
 * @brief   Sub-family identifier.
 */
#if !defined(STM32L1XX) || defined(__DOXYGEN__)
#define STM32L1XX
#endif
/** @} */


/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    Configuration options
 * @{
 */
/**
 * @brief   Disables the PWR/RCC initialization in the HAL.
 */
#if !defined(STM32_NO_INIT) || defined(__DOXYGEN__)
#define STM32_NO_INIT               FALSE
#endif

/*
 * Configuration-related checks.
 */
#if !defined(STM32L1xx_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32L1xx_MCUCONF not defined"
#endif

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

/* Various helpers.*/
#include "nvic.h"
#include "cache.h"
#include "mpu_v7m.h"
#include "stm32_isr.h"
#include "stm32_dma.h"
#include "stm32_exti.h"
#include "stm32_rcc.h"

#ifdef __cplusplus
extern "C" {
#endif
  void hal_lld_init(void);
#ifdef __cplusplus
}
#endif

#endif /* HAL_LLD_H */

/** @} */
