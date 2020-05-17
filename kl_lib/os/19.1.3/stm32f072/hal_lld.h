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
 * @file    STM32F0xx/hal_lld.h
 * @brief   STM32F0xx HAL subsystem low level driver header.
 * @pre     This module requires the following macros to be defined in the
 *          @p board.h file:
 *          - STM32_LSECLK.
 *          - STM32_LSEDRV.
 *          - STM32_LSE_BYPASS (optionally).
 *          - STM32_HSECLK.
 *          - STM32_HSE_BYPASS (optionally).
 *          .
 *          One of the following macros must also be defined:
 *          - STM32F030x4, STM32F030x6, STM32F030x8, STM32F030xC,
 *            STM32F070x6, STM32F070xB for Value Line devices.
 *          - STM32F031x6, STM32F051x8, STM32F071xB, STM32F091xC
 *            for Access Line devices.
 *          - STM32F042x6, STM32F072xB for USB Line devices.
 *          - STM32F038xx, STM32F048xx, STM32F058xx, STM32F078xx,
 *            STM32F098xx for Low Voltage Line devices.
 *          .
 *
 * @addtogroup HAL
 * @{
 */

#ifndef HAL_LLD_H
#define HAL_LLD_H

#include "board.h"

/*
 * Registry definitions.
 */
#include "stm32_registry.h"

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @name    Platform identification macros
 * @{
 */
#if defined(STM32F030x4) || defined(__DOXYGEN__)
#define PLATFORM_NAME           "STM32F030x4 Entry Level Value Line devices"

#elif defined(STM32F030x6)
#define PLATFORM_NAME           "STM32F030x6 Entry Level Value Line devices"

#elif defined(STM32F030x8)
#define PLATFORM_NAME           "STM32F030x8 Entry Level Value Line devices"

#elif defined(STM32F030xC)
#define PLATFORM_NAME           "STM32F030xC Entry Level Value Line devices"

#elif defined(STM32F070x6)
#define PLATFORM_NAME           "STM32F070x6 Entry Level Value Line devices"

#elif defined(STM32F070xB)
#define PLATFORM_NAME           "STM32F070xB Entry Level Value Line devices"

#elif defined(STM32F031x6)
#define PLATFORM_NAME           "STM32F031x6 Entry Level Access Line devices"

#elif defined(STM32F051x8)
#define PLATFORM_NAME           "STM32F051x8 Entry Level Access Line devices"

#elif defined(STM32F071xB)
#define PLATFORM_NAME           "STM32F071xB Entry Level Access Line devices"

#elif defined(STM32F091xC)
#define PLATFORM_NAME           "STM32F091xC Entry Level Access Line devices"

#elif defined(STM32F042x6)
#define PLATFORM_NAME           "STM32F042x6 Entry Level USB Line devices"

#elif defined(STM32F072xB)
#define PLATFORM_NAME           "STM32F072xB Entry Level USB Line devices"

#elif defined(STM32F038xx)
#define PLATFORM_NAME           "STM32F038xx Entry Level Low Voltage Line devices"

#elif defined(STM32F048xx)
#define PLATFORM_NAME           "STM32F048xx Entry Level Low Voltage Line devices"

#elif defined(STM32F058xx)
#define PLATFORM_NAME           "STM32F058xx Entry Level Low Voltage Line devices"

#elif defined(STM32F078xx)
#define PLATFORM_NAME           "STM32F078xx Entry Level Low Voltage Line devices"

#elif defined(STM32F098xx)
#define PLATFORM_NAME           "STM32F098xx Entry Level Low Voltage Line devices"

#else
#error "STM32F0xx device unsupported or not specified"
#endif
/** @} */

/**
 * @name    Absolute Maximum Ratings
 * @{
 */
/**
 * @brief   Maximum system clock frequency.
 */
#define STM32_SYSCLK_MAX        48000000

/**
 * @brief   Maximum HSE clock frequency.
 */
#define STM32_HSECLK_MAX        32000000

/**
 * @brief   Minimum HSE clock frequency.
 */
#define STM32_HSECLK_MIN        1000000

/**
 * @brief   Maximum LSE clock frequency.
 */
#define STM32_LSECLK_MAX        1000000

/**
 * @brief   Minimum LSE clock frequency.
 */
#define STM32_LSECLK_MIN        32768

/**
 * @brief   Maximum PLLs input clock frequency.
 */
#define STM32_PLLIN_MAX         25000000

/**
 * @brief   Minimum PLLs input clock frequency.
 */
#define STM32_PLLIN_MIN         1000000

/**
 * @brief   Maximum PLL output clock frequency.
 */
#define STM32_PLLOUT_MAX        48000000

/**
 * @brief   Minimum PLL output clock frequency.
 */
#define STM32_PLLOUT_MIN        16000000

/**
 * @brief   Maximum APB clock frequency.
 */
#define STM32_PCLK_MAX          48000000
/** @} */

/**
 * @name    Internal clock sources
 * @{
 */
#define STM32_HSICLK            8000000     /**< High speed internal clock. */
#define STM32_HSI14CLK          14000000    /**< 14MHz speed internal clock.*/
#define STM32_HSI48CLK          48000000    /**< 48MHz speed internal clock.*/
#define STM32_LSICLK            40000       /**< Low speed internal clock.  */
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
#define STM32_NO_INIT                       FALSE
#endif


/* Various helpers.*/
#include "nvic.h"
#include "cache.h"
#include "stm32_isr.h"
#include "stm32_dma.h"
#include "stm32_exti.h"
#include "stm32_rcc.h"

#ifdef __cplusplus
extern "C" {
#endif
  void hal_lld_init(void);
  void stm32_clock_init(void);
#ifdef __cplusplus
}
#endif

#endif /* HAL_LLD_H */

/** @} */
