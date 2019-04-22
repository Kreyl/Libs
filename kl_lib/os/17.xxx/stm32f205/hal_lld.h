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
 * @file    STM32F4xx/hal_lld.h
 * @brief   STM32F4xx/STM32F2xx HAL subsystem low level driver header.
 * @pre     This module requires the following macros to be defined in the
 *          @p board.h file:
 *          - STM32_LSECLK.
 *          - STM32_LSE_BYPASS (optionally).
 *          - STM32_HSECLK.
 *          - STM32_HSE_BYPASS (optionally).
 *          - STM32_VDD (as hundredths of Volt).
 *          .
 *          One of the following macros must also be defined:
 *          - STM32F2XX for High-performance STM32F2 devices.
 *          - STM32F405xx, STM32F415xx, STM32F407xx, STM32F417xx,
 *            STM32F446xx for High-performance STM32F4 devices of
 *            Foundation line.
 *          - STM32F401xC, STM32F401xE, STM32F410Cx, STM32F410Rx, STM32F411xE
 *            for High-performance STM32F4 devices of Access line.
 *          - STM32F427xx, STM32F437xx, STM32F429xx, STM32F439xx, STM32F469xx,
 *            STM32F479xx for High-performance STM32F4 devices of Advanced line.
 *          .
 *
 * @addtogroup HAL
 * @{
 */

#ifndef HAL_LLD_H
#define HAL_LLD_H

#include "stm32_registry.h"

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @brief   Defines the support for realtime counters in the HAL.
 */
#define HAL_IMPLEMENTS_COUNTERS              TRUE

/**
 * @name    Platform identification macros
 * @{
 */
#if defined(STM32F205xx) || defined(__DOXYGEN__)
#define PLATFORM_NAME           "STM32F205 High Performance"

#elif defined(STM32F207xx)
#define PLATFORM_NAME           "STM32F207 High Performance"

#elif defined(STM32F215xx)
#define PLATFORM_NAME           "STM32F215 High Performance"

#elif defined(STM32F217xx)
#define PLATFORM_NAME           "STM32F217 High Performance"

#elif defined(STM32F401xx)
#define PLATFORM_NAME           "STM32F401 High Performance with DSP and FPU"

#elif defined(STM32F405xx)
#define PLATFORM_NAME           "STM32F405 High Performance with DSP and FPU"

#elif defined(STM32F407xx)
#define PLATFORM_NAME           "STM32F407 High Performance with DSP and FPU"

#elif defined(STM32F410xx)
#define PLATFORM_NAME           "STM32F410 High Performance with DSP and FPU"

#elif defined(STM32F411xx)
#define PLATFORM_NAME           "STM32F411 High Performance with DSP and FPU"

#elif defined(STM32F412xx)
#define PLATFORM_NAME           "STM32F412 High Performance with DSP and FPU"

#elif defined(STM32F415xx)
#define PLATFORM_NAME           "STM32F415 High Performance with DSP and FPU"

#elif defined(STM32F417xx)
#define PLATFORM_NAME           "STM32F417 High Performance with DSP and FPU"

#elif defined(STM32F427xx)
#define PLATFORM_NAME           "STM32F427 High Performance with DSP and FPU"

#elif defined(STM32F429xx)
#define PLATFORM_NAME           "STM32F429 High Performance with DSP and FPU"

#elif defined(STM32F437xx)
#define PLATFORM_NAME           "STM32F437 High Performance with DSP and FPU"

#elif defined(STM32F439xx)
#define PLATFORM_NAME           "STM32F439 High Performance with DSP and FPU"

#elif defined(STM32F446xx)
#define PLATFORM_NAME           "STM32F446 High Performance with DSP and FPU"

#elif defined(STM32F469xx)
#define PLATFORM_NAME           "STM32F469 High Performance with DSP and FPU"

#elif defined(STM32F479xx)
#define PLATFORM_NAME           "STM32F479 High Performance with DSP and FPU"

#else
#error "STM32F2xx/F4xx device not specified"
#endif
/** @} */

/**
 * @name    Absolute Maximum Ratings
 * @{
 */
#if defined(STM32F427xx) || defined(STM32F437xx) ||                         \
    defined(STM32F429xx) || defined(STM32F439xx) ||                         \
    defined(STM32F469xx) || defined(STM32F479xx) || defined(__DOXYGEN__)
/**
 * @brief   Absolute maximum system clock.
 */
#define STM32_SYSCLK_MAX        180000000

/**
 * @brief   Maximum HSE clock frequency.
 */
#define STM32_HSECLK_MAX        26000000

/**
 * @brief   Maximum HSE clock frequency using an external source.
 */
#define STM32_HSECLK_BYP_MAX    50000000

/**
 * @brief   Minimum HSE clock frequency.
 */
#define STM32_HSECLK_MIN        4000000

/**
 * @brief   Minimum HSE clock frequency.
 */
#define STM32_HSECLK_BYP_MIN    1000000

/**
 * @brief   Maximum LSE clock frequency.
 */
#define STM32_LSECLK_MAX        32768

/**
 * @brief   Maximum LSE clock frequency.
 */
#define STM32_LSECLK_BYP_MAX    1000000

/**
 * @brief   Minimum LSE clock frequency.
 */
#define STM32_LSECLK_MIN        32768

/**
 * @brief   Maximum PLLs input clock frequency.
 */
#define STM32_PLLIN_MAX         2100000

/**
 * @brief   Minimum PLLs input clock frequency.
 */
#define STM32_PLLIN_MIN         950000

/**
 * @brief   Maximum PLLs VCO clock frequency.
 */
#define STM32_PLLVCO_MAX        432000000

/**
 * @brief   Minimum PLLs VCO clock frequency.
 */
#define STM32_PLLVCO_MIN        192000000

/**
 * @brief   Maximum PLL output clock frequency.
 */
#define STM32_PLLOUT_MAX        180000000

/**
 * @brief   Minimum PLL output clock frequency.
 */
#define STM32_PLLOUT_MIN        24000000

/**
 * @brief   Maximum PLLI2S output clock frequency.
 */
#define STM32_PLLI2SOUT_MAX     216000000

/**
 * @brief   Maximum PLLSAI output clock frequency.
 */
#define STM32_PLLSAIOUT_MAX     216000000

/**
 * @brief   Maximum APB1 clock frequency.
 */
#define STM32_PCLK1_MAX         (STM32_PLLOUT_MAX / 4)

/**
 * @brief   Maximum APB2 clock frequency.
 */
#define STM32_PCLK2_MAX         (STM32_PLLOUT_MAX / 2)

/**
 * @brief   Maximum SPI/I2S clock frequency.
 */
#define STM32_SPII2S_MAX        45000000
#endif

#if defined(STM32F40_41xxx)
#define STM32_SYSCLK_MAX        168000000
#define STM32_HSECLK_MAX        26000000
#define STM32_HSECLK_BYP_MAX    50000000
#define STM32_HSECLK_MIN        4000000
#define STM32_HSECLK_BYP_MIN    1000000
#define STM32_LSECLK_MAX        32768
#define STM32_LSECLK_BYP_MAX    1000000
#define STM32_LSECLK_MIN        32768
#define STM32_PLLIN_MAX         2100000
#define STM32_PLLIN_MIN         950000
#define STM32_PLLVCO_MAX        432000000
#define STM32_PLLVCO_MIN        192000000
#define STM32_PLLOUT_MAX        168000000
#define STM32_PLLOUT_MIN        24000000
#define STM32_PCLK1_MAX         42000000
#define STM32_PCLK2_MAX         84000000
#define STM32_SPII2S_MAX        42000000
#endif

#if defined(STM32F401xx)
#define STM32_SYSCLK_MAX        84000000
#define STM32_HSECLK_MAX        26000000
#define STM32_HSECLK_BYP_MAX    50000000
#define STM32_HSECLK_MIN        4000000
#define STM32_HSECLK_BYP_MIN    1000000
#define STM32_LSECLK_MAX        32768
#define STM32_LSECLK_BYP_MAX    1000000
#define STM32_LSECLK_MIN        32768
#define STM32_PLLIN_MAX         2100000
#define STM32_PLLIN_MIN         950000
#define STM32_PLLVCO_MAX        432000000
#define STM32_PLLVCO_MIN        192000000
#define STM32_PLLOUT_MAX        84000000
#define STM32_PLLOUT_MIN        24000000
#define STM32_PCLK1_MAX         42000000
#define STM32_PCLK2_MAX         84000000
#define STM32_SPII2S_MAX        42000000
#endif

#if defined(STM32F410xx) || defined(STM32F411xx) ||                         \
    defined(STM32F412xx)
#define STM32_SYSCLK_MAX        100000000
#define STM32_HSECLK_MAX        26000000
#define STM32_HSECLK_BYP_MAX    50000000
#define STM32_HSECLK_MIN        4000000
#define STM32_HSECLK_BYP_MIN    1000000
#define STM32_LSECLK_MAX        32768
#define STM32_LSECLK_BYP_MAX    1000000
#define STM32_LSECLK_MIN        32768
#define STM32_PLLIN_MAX         2100000
#define STM32_PLLIN_MIN         950000
#define STM32_PLLVCO_MAX        432000000
#define STM32_PLLVCO_MIN        100000000
#define STM32_PLLOUT_MAX        100000000
#define STM32_PLLOUT_MIN        24000000
#define STM32_PCLK1_MAX         50000000
#define STM32_PCLK2_MAX         100000000
#define STM32_SPII2S_MAX        50000000
#endif

#if defined(STM32F446xx)
#define STM32_SYSCLK_MAX        180000000
#define STM32_HSECLK_MAX        26000000
#define STM32_HSECLK_BYP_MAX    50000000
#define STM32_HSECLK_MIN        4000000
#define STM32_HSECLK_BYP_MIN    1000000
#define STM32_LSECLK_MAX        32768
#define STM32_LSECLK_BYP_MAX    1000000
#define STM32_LSECLK_MIN        32768
#define STM32_PLLIN_MAX         2100000
#define STM32_PLLIN_MIN         950000
#define STM32_PLLVCO_MAX        432000000
#define STM32_PLLVCO_MIN        100000000
#define STM32_PLLOUT_MAX        180000000
#define STM32_PLLOUT_MIN        12500000
#define STM32_PLLI2SOUT_MAX     216000000
#define STM32_PLLSAIOUT_MAX     216000000
#define STM32_PCLK1_MAX         (STM32_PLLOUT_MAX / 4)
#define STM32_PCLK2_MAX         (STM32_PLLOUT_MAX / 2)
#define STM32_SPII2S_MAX        45000000
#endif

#if defined(STM32F2XX)
#define STM32_SYSCLK_MAX        120000000
#define STM32_HSECLK_MAX        26000000
#define STM32_HSECLK_BYP_MAX    26000000
#define STM32_HSECLK_MIN        1000000
#define STM32_HSECLK_BYP_MIN    1000000
#define STM32_LSECLK_MAX        32768
#define STM32_LSECLK_BYP_MAX    1000000
#define STM32_LSECLK_MIN        32768
#define STM32_PLLIN_MAX         2000000
#define STM32_PLLIN_MIN         950000
#define STM32_PLLVCO_MAX        432000000
#define STM32_PLLVCO_MIN        192000000
#define STM32_PLLOUT_MAX        120000000
#define STM32_PLLOUT_MIN        24000000
#define STM32_PCLK1_MAX         30000000
#define STM32_PCLK2_MAX         60000000
#define STM32_SPII2S_MAX        30000000
#endif
/** @} */

/**
 * @name    Internal clock sources
 * @{
 */
#define STM32_HSICLK            16000000    /**< High speed internal clock. */
#define STM32_LSICLK            32000       /**< Low speed internal clock.  */
/** @} */

/**
 * @name    PWR_CR register bits definitions
 * @{
 */
#define STM32_VOS_SCALE3        0x00004000
#define STM32_VOS_SCALE2        0x00008000
#define STM32_VOS_SCALE1        0x0000C000
#define STM32_PLS_MASK          (7 << 5)    /**< PLS bits mask.             */
#define STM32_PLS_LEV0          (0 << 5)    /**< PVD level 0.               */
#define STM32_PLS_LEV1          (1 << 5)    /**< PVD level 1.               */
#define STM32_PLS_LEV2          (2 << 5)    /**< PVD level 2.               */
#define STM32_PLS_LEV3          (3 << 5)    /**< PVD level 3.               */
#define STM32_PLS_LEV4          (4 << 5)    /**< PVD level 4.               */
#define STM32_PLS_LEV5          (5 << 5)    /**< PVD level 5.               */
#define STM32_PLS_LEV6          (6 << 5)    /**< PVD level 6.               */
#define STM32_PLS_LEV7          (7 << 5)    /**< PVD level 7.               */
/** @} */

/**
 * @name    RCC_PLLCFGR register bits definitions
 * @{
 */
#define STM32_PLLP_MASK         (3 << 16)   /**< PLLP mask.                 */
#define STM32_PLLP_DIV2         (0 << 16)   /**< PLL clock divided by 2.    */
#define STM32_PLLP_DIV4         (1 << 16)   /**< PLL clock divided by 4.    */
#define STM32_PLLP_DIV6         (2 << 16)   /**< PLL clock divided by 6.    */
#define STM32_PLLP_DIV8         (3 << 16)   /**< PLL clock divided by 8.    */

#define STM32_PLLSRC_HSI        (0 << 22)   /**< PLL clock source is HSI.   */
#define STM32_PLLSRC_HSE        (1 << 22)   /**< PLL clock source is HSE.   */
/** @} */

/**
 * @name    RCC_CFGR register bits definitions
 * @{
 */
#define STM32_SW_MASK           (3 << 0)    /**< SW mask.                   */
#define STM32_SW_HSI            (0 << 0)    /**< SYSCLK source is HSI.      */
#define STM32_SW_HSE            (1 << 0)    /**< SYSCLK source is HSE.      */
#define STM32_SW_PLL            (2 << 0)    /**< SYSCLK source is PLL.      */

#define STM32_HPRE_MASK         (15 << 4)   /**< HPRE mask.                 */
#define STM32_HPRE_DIV1         (0 << 4)    /**< SYSCLK divided by 1.       */
#define STM32_HPRE_DIV2         (8 << 4)    /**< SYSCLK divided by 2.       */
#define STM32_HPRE_DIV4         (9 << 4)    /**< SYSCLK divided by 4.       */
#define STM32_HPRE_DIV8         (10 << 4)   /**< SYSCLK divided by 8.       */
#define STM32_HPRE_DIV16        (11 << 4)   /**< SYSCLK divided by 16.      */
#define STM32_HPRE_DIV64        (12 << 4)   /**< SYSCLK divided by 64.      */
#define STM32_HPRE_DIV128       (13 << 4)   /**< SYSCLK divided by 128.     */
#define STM32_HPRE_DIV256       (14 << 4)   /**< SYSCLK divided by 256.     */
#define STM32_HPRE_DIV512       (15 << 4)   /**< SYSCLK divided by 512.     */

#define STM32_PPRE1_MASK        (7 << 10)   /**< PPRE1 mask.                */
#define STM32_PPRE1_DIV1        (0 << 10)   /**< HCLK divided by 1.         */
#define STM32_PPRE1_DIV2        (4 << 10)   /**< HCLK divided by 2.         */
#define STM32_PPRE1_DIV4        (5 << 10)   /**< HCLK divided by 4.         */
#define STM32_PPRE1_DIV8        (6 << 10)   /**< HCLK divided by 8.         */
#define STM32_PPRE1_DIV16       (7 << 10)   /**< HCLK divided by 16.        */

#define STM32_PPRE2_MASK        (7 << 13)   /**< PPRE2 mask.                */
#define STM32_PPRE2_DIV1        (0 << 13)   /**< HCLK divided by 1.         */
#define STM32_PPRE2_DIV2        (4 << 13)   /**< HCLK divided by 2.         */
#define STM32_PPRE2_DIV4        (5 << 13)   /**< HCLK divided by 4.         */
#define STM32_PPRE2_DIV8        (6 << 13)   /**< HCLK divided by 8.         */
#define STM32_PPRE2_DIV16       (7 << 13)   /**< HCLK divided by 16.        */

#define STM32_RTCPRE_MASK       (31 << 16)  /**< RTCPRE mask.               */

#define STM32_MCO1SEL_MASK      (3 << 21)   /**< MCO1 mask.                 */
#define STM32_MCO1SEL_HSI       (0 << 21)   /**< HSI clock on MCO1 pin.     */
#define STM32_MCO1SEL_LSE       (1 << 21)   /**< LSE clock on MCO1 pin.     */
#define STM32_MCO1SEL_HSE       (2 << 21)   /**< HSE clock on MCO1 pin.     */
#define STM32_MCO1SEL_PLL       (3 << 21)   /**< PLL clock on MCO1 pin.     */

#define STM32_I2SSRC_MASK       (1 << 23)   /**< I2CSRC mask.               */
#define STM32_I2SSRC_PLLI2S     (0 << 23)   /**< I2SSRC is PLLI2S.          */
#define STM32_I2SSRC_CKIN       (1 << 23)   /**< I2S_CKIN is PLLI2S.        */

#define STM32_MCO1PRE_MASK      (7 << 24)   /**< MCO1PRE mask.              */
#define STM32_MCO1PRE_DIV1      (0 << 24)   /**< MCO1 divided by 1.         */
#define STM32_MCO1PRE_DIV2      (4 << 24)   /**< MCO1 divided by 2.         */
#define STM32_MCO1PRE_DIV3      (5 << 24)   /**< MCO1 divided by 3.         */
#define STM32_MCO1PRE_DIV4      (6 << 24)   /**< MCO1 divided by 4.         */
#define STM32_MCO1PRE_DIV5      (7 << 24)   /**< MCO1 divided by 5.         */

#define STM32_MCO2PRE_MASK      (7 << 27)   /**< MCO2PRE mask.              */
#define STM32_MCO2PRE_DIV1      (0 << 27)   /**< MCO2 divided by 1.         */
#define STM32_MCO2PRE_DIV2      (4 << 27)   /**< MCO2 divided by 2.         */
#define STM32_MCO2PRE_DIV3      (5 << 27)   /**< MCO2 divided by 3.         */
#define STM32_MCO2PRE_DIV4      (6 << 27)   /**< MCO2 divided by 4.         */
#define STM32_MCO2PRE_DIV5      (7 << 27)   /**< MCO2 divided by 5.         */

#define STM32_MCO2SEL_MASK      (3 << 30)   /**< MCO2 mask.                 */
#define STM32_MCO2SEL_SYSCLK    (0 << 30)   /**< SYSCLK clock on MCO2 pin.  */
#define STM32_MCO2SEL_PLLI2S    (1 << 30)   /**< PLLI2S clock on MCO2 pin.  */
#define STM32_MCO2SEL_HSE       (2 << 30)   /**< HSE clock on MCO2 pin.     */
#define STM32_MCO2SEL_PLL       (3 << 30)   /**< PLL clock on MCO2 pin.     */

/**
 * @name    RCC_PLLI2SCFGR register bits definitions
 * @{
 */
#define STM32_PLLI2SM_MASK      (31 << 0)   /**< PLLI2SM mask.              */
#define STM32_PLLI2SN_MASK      (511 << 6)  /**< PLLI2SN mask.              */
#define STM32_PLLI2SP_MASK      (3 << 16)   /**< PLLI2SP mask.              */
#define STM32_PLLI2SP_DIV2      (0 << 16)   /**< PLLI2S clock divided by 2. */
#define STM32_PLLI2SP_DIV4      (1 << 16)   /**< PLLI2S clock divided by 4. */
#define STM32_PLLI2SP_DIV6      (2 << 16)   /**< PLLI2S clock divided by 6. */
#define STM32_PLLI2SP_DIV8      (3 << 16)   /**< PLLI2S clock divided by 8. */
#define STM32_PLLI2SQ_MASK      (15 << 24)  /**< PLLI2SQ mask.              */
#define STM32_PLLI2SR_MASK      (7 << 28)   /**< PLLI2SR mask.              */
/** @} */

/**
 * @name    RCC_PLLSAICFGR register bits definitions
 * @{
 */
#define STM32_PLLSAIM_MASK      (31 << 0)   /**< PLLSAIM mask.              */
#define STM32_PLLSAIN_MASK      (511 << 6)  /**< PLLSAIN mask.              */
#define STM32_PLLSAIP_MASK      (3 << 16)   /**< PLLSAIP mask.              */
#define STM32_PLLSAIP_DIV2      (0 << 16)   /**< PLLSAI clock divided by 2. */
#define STM32_PLLSAIP_DIV4      (1 << 16)   /**< PLLSAI clock divided by 4. */
#define STM32_PLLSAIP_DIV6      (2 << 16)   /**< PLLSAI clock divided by 6. */
#define STM32_PLLSAIP_DIV8      (3 << 16)   /**< PLLSAI clock divided by 8. */
#define STM32_PLLSAIQ_MASK      (15 << 24)  /**< PLLSAIQ mask.              */
#define STM32_PLLSAIR_MASK      (7 << 28)   /**< PLLSAIR mask.              */
/** @} */

/**
 * @name    RCC_BDCR register bits definitions
 * @{
 */
#define STM32_RTCSEL_MASK       (3 << 8)    /**< RTC source mask.           */
#define STM32_RTCSEL_NOCLOCK    (0 << 8)    /**< No RTC source.             */
#define STM32_RTCSEL_LSE        (1 << 8)    /**< RTC source is LSE.         */
#define STM32_RTCSEL_LSI        (2 << 8)    /**< RTC source is LSI.         */
#define STM32_RTCSEL_HSEDIV     (3 << 8)    /**< RTC source is HSE divided. */
/** @} */

/**
 * @name    RCC_DCKCFGR register bits definitions
 * @{
 */
#define STM32_PLLI2SDIVQ_MASK   (31 << 0)   /**< PLLI2SDIVQ mask.           */

#define STM32_PLLSAIDIVQ_MASK   (31 << 8)   /**< PLLSAIDIVQ mask.           */

#define STM32_PLLSAIDIVR_MASK   (3 << 16)   /**< PLLSAIDIVR mask.           */
#define STM32_PLLSAIDIVR_DIV2   (0 << 16)   /**< LCD_CLK is R divided by 2. */
#define STM32_PLLSAIDIVR_DIV4   (1 << 16)   /**< LCD_CLK is R divided by 4. */
#define STM32_PLLSAIDIVR_DIV8   (2 << 16)   /**< LCD_CLK is R divided by 8. */
#define STM32_PLLSAIDIVR_DIV16  (3 << 16)   /**< LCD_CLK is R divided by 16.*/
#define STM32_PLLSAIDIVR_OFF    0xFFFFFFFFU /**< LCD CLK is not required.   */

#define STM32_SAI1SEL_MASK      (3 << 20)   /**< SAI1SEL mask.              */
#define STM32_SAI1SEL_PLLSAI    (0 << 20)   /**< SAI1 source is PLLSAI.     */
#define STM32_SAI1SEL_PLLI2S    (1 << 20)   /**< SAI1 source is PLLI2S.     */
#define STM32_SAI1SEL_PLLR      (2 << 20)   /**< SAI1 source is PLLR.       */
#define STM32_SAI1SEL_OFF       0xFFFFFFFFU /**< SAI1 clock is not required.*/

#define STM32_SAI2SEL_MASK      (3 << 22)   /**< SAI2SEL mask.              */
#define STM32_SAI2SEL_PLLSAI    (0 << 22)   /**< SAI2 source is PLLSAI.     */
#define STM32_SAI2SEL_PLLI2S    (1 << 22)   /**< SAI2 source is PLLI2S.     */
#define STM32_SAI2SEL_PLLR      (2 << 22)   /**< SAI2 source is PLLR.       */
#define STM32_SAI2SEL_OFF       0xFFFFFFFFU /**< SAI2 clock is not required.*/

#define STM32_TIMPRE_MASK       (1 << 24)   /**< TIMPRE mask.               */
#define STM32_TIMPRE_PCLK       (0 << 24)   /**< TIM clocks from PCLKx.     */
#define STM32_TIMPRE_HCLK       (1 << 24)   /**< TIM clocks from HCLK.      */

#define STM32_I2S1SEL_MASK      (3 << 25)   /**< I2S1SEL mask.              */
#define STM32_I2S1SEL_PLLR      (0 << 25)   /**< I2S1 source is PLLR.       */
#define STM32_I2S1SEL_AFIN      (1 << 25)   /**< I2S1 source is AF Input.   */
#define STM32_I2S1SEL_MCO1      (2 << 25)   /**< I2S1 source is MCO1.       */
#define STM32_I2S1SEL_OFF       0xFFFFFFFFU /**< I2S1 clock is not required.*/

#define STM32_I2S2SEL_MASK      (3 << 27)   /**< I2S2SEL mask.              */
#define STM32_I2S2SEL_PLLR      (0 << 27)   /**< I2S2 source is PLLR.       */
#define STM32_I2S2SEL_AFIN      (1 << 27)   /**< I2S2 source is AF Input.   */
#define STM32_I2S2SEL_MCO1      (2 << 27)   /**< I2S2 source is MCO1.       */
#define STM32_I2S2SEL_OFF       0xFFFFFFFFU /**< I2S2 clock is not required.*/

#define STM32_DSISEL_MASK       (1 << 28)   /**< DSISEL mask.               */
#define STM32_DSISEL_PHY        (0 << 28)   /**< DSI source is DSI-PSY.     */
#define STM32_DSISEL_PLLR       (1 << 28)   /**< DSI source is PLLR.        */
/** @} */

/**
 * @name    RCC_DCKCFGR2 register bits definitions
 * @{
 */
#define STM32_I2C1SEL_MASK      (3 << 22)   /**< I2C1SEL mask.              */
#define STM32_I2C1SEL_PCLK1     (0 << 22)   /**< I2C1 source is PCLK1.      */
#define STM32_I2C1SEL_SYSCLK    (1 << 22)   /**< I2C1 source is SYSCLK.     */
#define STM32_I2C1SEL_HSI       (2 << 22)   /**< I2C1 source is HSI.        */

#define STM32_CECSEL_MASK       (1 << 26)   /**< CECSEL mask.               */
#define STM32_CECSEL_LSE        (0 << 26)   /**< CEC source is LSE.         */
#define STM32_CECSEL_HSIDIV488  (1 << 26)   /**< CEC source is HSI/488.     */

#define STM32_CK48MSEL_MASK     (1 << 27)   /**< CK48MSEL mask.             */
#define STM32_CK48MSEL_PLL      (0 << 27)   /**< PLL48CLK source is PLL.    */
#define STM32_CK48MSEL_PLLSAI   (1 << 27)   /**< PLL48CLK source is PLLSAI. */

#define STM32_SDMMCSEL_MASK     (1 << 28)   /**< SDMMCSEL mask.             */
#define STM32_SDMMCSEL_PLL48CLK (0 << 28)   /**< SDMMC source is PLL48CLK.  */
#define STM32_SDMMCSEL_SYSCLK   (1 << 28)   /**< SDMMC source is SYSCLK.    */

#define STM32_SPDIFSEL_MASK     (1 << 29)   /**< SPDIFSEL mask.             */
#define STM32_SPDIFSEL_PLLI2S   (0 << 29)   /**< SPDIF source is PLLI2S.    */
#define STM32_SPDIFSEL_PLL      (1 << 29)   /**< SPDIF source is PLL.       */
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

/**
 * @brief   Enables or disables the programmable voltage detector.
 */
#if !defined(STM32_PVD_ENABLE) || defined(__DOXYGEN__)
#define STM32_PVD_ENABLE            FALSE
#endif

/**
 * @brief   Sets voltage level for programmable voltage detector.
 */
#if !defined(STM32_PLS) || defined(__DOXYGEN__)
#define STM32_PLS                   STM32_PLS_LEV0
#endif

/**
 * @brief   Enables the backup RAM regulator.
 */
#if !defined(STM32_BKPRAM_ENABLE) || defined(__DOXYGEN__)
#define STM32_BKPRAM_ENABLE         FALSE
#endif


/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if defined(STM32F4XX) || defined(__DOXYGEN__)
/*
 * Configuration-related checks.
 */
#if !defined(STM32F4xx_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32F4xx_MCUCONF not defined"
#endif

#else /* !defined(STM32F4XX) */
/*
 * Configuration-related checks.
 */
#if !defined(STM32F2xx_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32F2xx_MCUCONF not defined"
#endif
#endif /* !defined(STM32F4XX) */


/* There are differences in vector names in the various sub-families,
   normalizing.*/
#if 0
#define TIM1_BRK_IRQn       TIM1_BRK_TIM9_IRQn
#define TIM1_UP_IRQn        TIM1_UP_TIM10_IRQn
#define TIM1_TRG_COM_IRQn   TIM1_TRG_COM_TIM11_IRQn
#define TIM8_BRK_IRQn       TIM8_BRK_TIM12_IRQn
#define TIM8_UP_IRQn        TIM8_UP_TIM13_IRQn
#define TIM8_TRG_COM_IRQn   TIM8_TRG_COM_TIM14_IRQn
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

/* Various helpers.*/
#include "nvic.h"
#include "stm32_isr.h"
#include "stm32_dma.h"
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
