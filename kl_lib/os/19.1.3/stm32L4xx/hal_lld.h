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
 * @file    STM32L4xx/hal_lld.h
 * @brief   STM32L4xx HAL subsystem low level driver header.
 * @pre     This module requires the following macros to be defined in the
 *          @p board.h file:
 *          - STM32_LSECLK.
 *          - STM32_LSEDRV.
 *          - STM32_LSE_BYPASS (optionally).
 *          - STM32_HSECLK.
 *          - STM32_HSE_BYPASS (optionally).
 *          .
 *          One of the following macros must also be defined:
 *          - STM32L432xx, STM32L433xx, STM32L443xx.
 *          - STM32L471xx, STM32L475xx, STM32L476xx, STM32L496xx.
 *          - STM32L485xx, STM32L486xx, STM32L4A6xx.
 *          .
 *
 * @addtogroup STM32L4xx_ISR
 * @{
 */

#ifndef HAL_LLD_H
#define HAL_LLD_H

#include "stm32_registry.h"

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @name    Platform identification
 * @{
 */
#if defined(STM32L432xx) || defined(STM32L433xx) || defined(STM32L443xx) || \
    defined(STM32L471xx) || defined(STM32L475xx) ||                         \
    defined(STM32L476xx) || defined(STM32L496xx) || defined(__DOXYGEN__)
#define PLATFORM_NAME           "STM32L4xx Ultra Low Power"

#elif defined(STM32L485xx) || defined(STM32L486xx) || defined(STM32L4A6xx)
#define PLATFORM_NAME           "STM32L4xx Ultra Low Power with Crypto"

#else
#error "STM32L4xx device not specified"
#endif

/**
 * @brief   Sub-family identifier.
 */
#if !defined(STM32L4XX) || defined(__DOXYGEN__)
#define STM32L4XX
#endif
/** @} */

/**
 * @name    Internal clock sources
 * @{
 */
#define STM32_HSI16CLK          16000000    /**< 16MHz internal clock.  */
#define STM32_HSI48CLK          48000000    /**< 48MHz internal clock.      */
#define STM32_LSICLK            32000       /**< Low speed internal clock.  */
/** @} */

/**
 * @name    PWR_CR1 register bits definitions
 * @{
 */
#define STM32_VOS_MASK          (3 << 9)    /**< Core voltage mask.         */
#define STM32_VOS_RANGE1        (1 << 9)    /**< Core voltage 1.2 Volts.    */
#define STM32_VOS_RANGE2        (2 << 9)    /**< Core voltage 1.0 Volts.    */
/** @} */

/**
 * @name    PWR_CR2 register bits definitions
 * @{
 */
#define STM32_PLS_MASK          (7 << 1)    /**< PLS bits mask.             */
#define STM32_PLS_LEV0          (0 << 1)    /**< PVD level 0.               */
#define STM32_PLS_LEV1          (1 << 1)    /**< PVD level 1.               */
#define STM32_PLS_LEV2          (2 << 1)    /**< PVD level 2.               */
#define STM32_PLS_LEV3          (3 << 1)    /**< PVD level 3.               */
#define STM32_PLS_LEV4          (4 << 1)    /**< PVD level 4.               */
#define STM32_PLS_LEV5          (5 << 1)    /**< PVD level 5.               */
#define STM32_PLS_LEV6          (6 << 1)    /**< PVD level 6.               */
#define STM32_PLS_EXT           (7 << 1)    /**< PVD level 7.               */
/** @} */

/**
 * @name    RCC_CR register bits definitions
 * @{
 */
#define STM32_MSIRANGE_MASK     (15 << 4)   /**< MSIRANGE field mask.       */
#define STM32_MSIRANGE_100K     (0 << 4)    /**< 100kHz nominal.            */
#define STM32_MSIRANGE_200K     (1 << 4)    /**< 200kHz nominal.            */
#define STM32_MSIRANGE_400K     (2 << 4)    /**< 400kHz nominal.            */
#define STM32_MSIRANGE_800K     (3 << 4)    /**< 800kHz nominal.            */
#define STM32_MSIRANGE_1M       (4 << 4)    /**< 1MHz nominal.              */
#define STM32_MSIRANGE_2M       (5 << 4)    /**< 2MHz nominal.              */
#define STM32_MSIRANGE_4M       (6 << 4)    /**< 4MHz nominal.              */
#define STM32_MSIRANGE_8M       (7 << 4)    /**< 8MHz nominal.              */
#define STM32_MSIRANGE_16M      (8 << 4)    /**< 16MHz nominal.             */
#define STM32_MSIRANGE_24M      (9 << 4)    /**< 24MHz nominal.             */
#define STM32_MSIRANGE_32M      (10 << 4)   /**< 32MHz nominal.             */
#define STM32_MSIRANGE_48M      (11 << 4)   /**< 48MHz nominal.             */
/** @} */

/**
 * @name    RCC_CFGR register bits definitions
 * @{
 */
#define STM32_SW_MASK           (3 << 0)    /**< SW field mask.             */
#define STM32_SW_MSI            (0 << 0)    /**< SYSCLK source is MSI.      */
#define STM32_SW_HSI16          (1 << 0)    /**< SYSCLK source is HSI.      */
#define STM32_SW_HSE            (2 << 0)    /**< SYSCLK source is HSE.      */
#define STM32_SW_PLL            (3 << 0)    /**< SYSCLK source is PLL.      */

#define STM32_HPRE_MASK         (15 << 4)   /**< HPRE field mask.           */
#define STM32_HPRE_DIV1         (0 << 4)    /**< SYSCLK divided by 1.       */
#define STM32_HPRE_DIV2         (8 << 4)    /**< SYSCLK divided by 2.       */
#define STM32_HPRE_DIV4         (9 << 4)    /**< SYSCLK divided by 4.       */
#define STM32_HPRE_DIV8         (10 << 4)   /**< SYSCLK divided by 8.       */
#define STM32_HPRE_DIV16        (11 << 4)   /**< SYSCLK divided by 16.      */
#define STM32_HPRE_DIV64        (12 << 4)   /**< SYSCLK divided by 64.      */
#define STM32_HPRE_DIV128       (13 << 4)   /**< SYSCLK divided by 128.     */
#define STM32_HPRE_DIV256       (14 << 4)   /**< SYSCLK divided by 256.     */
#define STM32_HPRE_DIV512       (15 << 4)   /**< SYSCLK divided by 512.     */

#define STM32_PPRE1_MASK        (7 << 8)    /**< PPRE1 field mask.          */
#define STM32_PPRE1_DIV1        (0 << 8)    /**< HCLK divided by 1.         */
#define STM32_PPRE1_DIV2        (4 << 8)    /**< HCLK divided by 2.         */
#define STM32_PPRE1_DIV4        (5 << 8)    /**< HCLK divided by 4.         */
#define STM32_PPRE1_DIV8        (6 << 8)    /**< HCLK divided by 8.         */
#define STM32_PPRE1_DIV16       (7 << 8)    /**< HCLK divided by 16.        */

#define STM32_PPRE2_MASK        (7 << 11)   /**< PPRE2 field mask.          */
#define STM32_PPRE2_DIV1        (0 << 11)   /**< HCLK divided by 1.         */
#define STM32_PPRE2_DIV2        (4 << 11)   /**< HCLK divided by 2.         */
#define STM32_PPRE2_DIV4        (5 << 11)   /**< HCLK divided by 4.         */
#define STM32_PPRE2_DIV8        (6 << 11)   /**< HCLK divided by 8.         */
#define STM32_PPRE2_DIV16       (7 << 11)   /**< HCLK divided by 16.        */

#define STM32_STOPWUCK_MASK     (1 << 15)   /**< STOPWUCK field mask.       */
#define STM32_STOPWUCK_MSI      (0 << 15)   /**< Wakeup clock is MSI.       */
#define STM32_STOPWUCK_HSI16    (1 << 15)   /**< Wakeup clock is HSI16.     */

#define STM32_MCOSEL_MASK       (15 << 24)  /**< MCOSEL field mask.         */
#define STM32_MCOSEL_NOCLOCK    (0 << 24)   /**< No clock on MCO pin.       */
#define STM32_MCOSEL_SYSCLK     (1 << 24)   /**< SYSCLK on MCO pin.         */
#define STM32_MCOSEL_MSI        (2 << 24)   /**< MSI clock on MCO pin.      */
#define STM32_MCOSEL_HSI16      (3 << 24)   /**< HSI16 clock on MCO pin.    */
#define STM32_MCOSEL_HSE        (4 << 24)   /**< HSE clock on MCO pin.      */
#define STM32_MCOSEL_PLL        (5 << 24)   /**< PLL clock on MCO pin.      */
#define STM32_MCOSEL_LSI        (6 << 24)   /**< LSI clock on MCO pin.      */
#define STM32_MCOSEL_LSE        (7 << 24)   /**< LSE clock on MCO pin.      */
#define STM32_MCOSEL_HSI48      (8 << 24)   /**< HSI48 clock on MCO pin.    */

#define STM32_MCOPRE_MASK       (7 << 28)   /**< MCOPRE field mask.         */
#define STM32_MCOPRE_DIV1       (0 << 28)   /**< MCO divided by 1.          */
#define STM32_MCOPRE_DIV2       (1 << 28)   /**< MCO divided by 2.          */
#define STM32_MCOPRE_DIV4       (2 << 28)   /**< MCO divided by 4.          */
#define STM32_MCOPRE_DIV8       (3 << 28)   /**< MCO divided by 8.          */
#define STM32_MCOPRE_DIV16      (4 << 28)   /**< MCO divided by 16.         */
/** @} */

/**
 * @name    RCC_PLLCFGR register bits definitions
 * @{
 */
#define STM32_PLLSRC_MASK       (3 << 0)    /**< PLL clock source mask.     */
#define STM32_PLLSRC_NOCLOCK    (0 << 0)    /**< PLL clock source disabled. */
#define STM32_PLLSRC_MSI        (1 << 0)    /**< PLL clock source is MSI.   */
#define STM32_PLLSRC_HSI16      (2 << 0)    /**< PLL clock source is HSI16. */
#define STM32_PLLSRC_HSE        (3 << 0)    /**< PLL clock source is HSE.   */
/** @} */

/**
 * @name    RCC_CCIPR register bits definitions
 * @{
 */
#define STM32_USART1SEL_MASK    (3 << 0)    /**< USART1SEL mask.            */
#define STM32_USART1SEL_PCLK2   (0 << 0)    /**< USART1 source is PCLK2.    */
#define STM32_USART1SEL_SYSCLK  (1 << 0)    /**< USART1 source is SYSCLK.   */
#define STM32_USART1SEL_HSI16   (2 << 0)    /**< USART1 source is HSI16.    */
#define STM32_USART1SEL_LSE     (3 << 0)    /**< USART1 source is LSE.      */

#define STM32_USART2SEL_MASK    (3 << 2)    /**< USART2 mask.               */
#define STM32_USART2SEL_PCLK1   (0 << 2)    /**< USART2 source is PCLK1.    */
#define STM32_USART2SEL_SYSCLK  (1 << 2)    /**< USART2 source is SYSCLK.   */
#define STM32_USART2SEL_HSI16   (2 << 2)    /**< USART2 source is HSI16.    */
#define STM32_USART2SEL_LSE     (3 << 2)    /**< USART2 source is LSE.      */

#define STM32_USART3SEL_MASK    (3 << 4)    /**< USART3 mask.               */
#define STM32_USART3SEL_PCLK1   (0 << 4)    /**< USART3 source is PCLK1.    */
#define STM32_USART3SEL_SYSCLK  (1 << 4)    /**< USART3 source is SYSCLK.   */
#define STM32_USART3SEL_HSI16   (2 << 4)    /**< USART3 source is HSI16.    */
#define STM32_USART3SEL_LSE     (3 << 4)    /**< USART3 source is LSE.      */

#define STM32_UART4SEL_MASK     (3 << 6)    /**< UART4 mask.                */
#define STM32_UART4SEL_PCLK1    (0 << 6)    /**< UART4 source is PCLK1.     */
#define STM32_UART4SEL_SYSCLK   (1 << 6)    /**< UART4 source is SYSCLK.    */
#define STM32_UART4SEL_HSI16    (2 << 6)    /**< UART4 source is HSI16.     */
#define STM32_UART4SEL_LSE      (3 << 6)    /**< UART4 source is LSE.       */

#define STM32_UART5SEL_MASK     (3 << 8)    /**< UART5 mask.                */
#define STM32_UART5SEL_PCLK1    (0 << 8)    /**< UART5 source is PCLK1.     */
#define STM32_UART5SEL_SYSCLK   (1 << 8)    /**< UART5 source is SYSCLK.    */
#define STM32_UART5SEL_HSI16    (2 << 8)    /**< UART5 source is HSI16.     */
#define STM32_UART5SEL_LSE      (3 << 8)    /**< UART5 source is LSE.       */

#define STM32_LPUART1SEL_MASK   (3 << 10)   /**< LPUART1 mask.              */
#define STM32_LPUART1SEL_PCLK1  (0 << 10)   /**< LPUART1 source is PCLK1.   */
#define STM32_LPUART1SEL_SYSCLK (1 << 10)   /**< LPUART1 source is SYSCLK.  */
#define STM32_LPUART1SEL_HSI16  (2 << 10)   /**< LPUART1 source is HSI16.   */
#define STM32_LPUART1SEL_LSE    (3 << 10)   /**< LPUART1 source is LSE.     */

#define STM32_I2C1SEL_MASK      (3 << 12)   /**< I2C1SEL mask.              */
#define STM32_I2C1SEL_PCLK1     (0 << 12)   /**< I2C1 source is PCLK1.      */
#define STM32_I2C1SEL_SYSCLK    (1 << 12)   /**< I2C1 source is SYSCLK.     */
#define STM32_I2C1SEL_HSI16     (2 << 12)   /**< I2C1 source is HSI16.      */

#define STM32_I2C2SEL_MASK      (3 << 14)   /**< I2C2SEL mask.              */
#define STM32_I2C2SEL_PCLK1     (0 << 14)   /**< I2C2 source is PCLK1.      */
#define STM32_I2C2SEL_SYSCLK    (1 << 14)   /**< I2C2 source is SYSCLK.     */
#define STM32_I2C2SEL_HSI16     (2 << 14)   /**< I2C2 source is HSI16.      */

#define STM32_I2C3SEL_MASK      (3 << 16)   /**< I2C3SEL mask.              */
#define STM32_I2C3SEL_PCLK1     (0 << 16)   /**< I2C3 source is PCLK1.      */
#define STM32_I2C3SEL_SYSCLK    (1 << 16)   /**< I2C3 source is SYSCLK.     */
#define STM32_I2C3SEL_HSI16     (2 << 16)   /**< I2C3 source is HSI16.      */

#define STM32_LPTIM1SEL_MASK    (3 << 18)   /**< LPTIM1SEL mask.            */
#define STM32_LPTIM1SEL_PCLK1   (0 << 18)   /**< LPTIM1 source is PCLK1.    */
#define STM32_LPTIM1SEL_LSI     (1 << 18)   /**< LPTIM1 source is LSI.      */
#define STM32_LPTIM1SEL_HSI16   (2 << 18)   /**< LPTIM1 source is HSI16.    */
#define STM32_LPTIM1SEL_LSE     (3 << 18)   /**< LPTIM1 source is LSE.      */

#define STM32_LPTIM2SEL_MASK    (3 << 20)   /**< LPTIM2SEL mask.            */
#define STM32_LPTIM2SEL_PCLK1   (0 << 20)   /**< LPTIM2 source is PCLK1.    */
#define STM32_LPTIM2SEL_LSI     (1 << 20)   /**< LPTIM2 source is LSI.      */
#define STM32_LPTIM2SEL_HSI16   (2 << 20)   /**< LPTIM2 source is HSI16.    */
#define STM32_LPTIM2SEL_LSE     (3 << 20)   /**< LPTIM2 source is LSE.      */

#define STM32_SAI1SEL_MASK      (3 << 22)   /**< SAI1SEL mask.              */
#define STM32_SAI1SEL_PLLSAI1   (0 << 22)   /**< SAI1 source is PLLSAI1-P.  */
#define STM32_SAI1SEL_PLLSAI2   (1 << 22)   /**< SAI1 source is PLLSAI2-P.  */
#define STM32_SAI1SEL_PLL       (2 << 22)   /**< SAI1 source is PLL-P.      */
#define STM32_SAI1SEL_EXTCLK    (3 << 22)   /**< SAI1 source is external.   */
#define STM32_SAI1SEL_OFF       0xFFFFFFFFU /**< SAI1 clock is not required.*/

#define STM32_SAI2SEL_MASK      (3 << 24)   /**< SAI2SEL mask.              */
#define STM32_SAI2SEL_PLLSAI1   (0 << 24)   /**< SAI2 source is PLLSAI1-P.  */
#define STM32_SAI2SEL_PLLSAI2   (1 << 24)   /**< SAI2 source is PLLSAI2-P.  */
#define STM32_SAI2SEL_PLL       (2 << 24)   /**< SAI2 source is PLL-P.      */
#define STM32_SAI2SEL_EXTCLK    (3 << 24)   /**< SAI2 source is external.   */
#define STM32_SAI2SEL_OFF       0xFFFFFFFFU /**< SAI2 clock is not required.*/

#define STM32_CLK48SEL_MASK     (3 << 26)   /**< CLK48SEL mask.             */
#if !STM32_CLOCK_HAS_HSI48
#define STM32_CLK48SEL_NOCLK    (0 << 26)   /**< CLK48 disabled.            */
#else
#define STM32_CLK48SEL_HSI48    (0 << 26)   /**< CLK48 source is HSI48.     */
#endif
#define STM32_CLK48SEL_PLLSAI1  (1 << 26)   /**< CLK48 source is PLLSAI1-Q. */
#define STM32_CLK48SEL_PLL      (2 << 26)   /**< CLK48 source is PLL-Q.     */
#define STM32_CLK48SEL_MSI      (3 << 26)   /**< CLK48 source is MSI.       */

#define STM32_ADCSEL_MASK       (3 << 28)   /**< ADCSEL mask.               */
#define STM32_ADCSEL_NOCLK      (0 << 28)   /**< ADC clock disabled.        */
#define STM32_ADCSEL_PLLSAI1    (1 << 28)   /**< ADC source is PLLSAI1-R.   */
#define STM32_ADCSEL_PLLSAI2    (2 << 28)   /**< ADC source is PLLSAI2-R.   */
#define STM32_ADCSEL_SYSCLK     (3 << 28)   /**< ADC source is SYSCLK.      */

#define STM32_SWPMI1SEL_MASK    (1 << 30)   /**< SWPMI1SEL mask.            */
#define STM32_SWPMI1SEL_PCLK1   (0 << 30)   /**< SWPMI1 source is PCLK1.    */
#define STM32_SWPMI1SEL_HSI16   (1 << 30)   /**< SWPMI1 source is HSI16.    */

#define STM32_DFSDMSEL_MASK     (1 << 31)   /**< DFSDMSEL mask.             */
#define STM32_DFSDMSEL_PCLK2    (0 << 31)   /**< DFSDM source is PCLK2.     */
#define STM32_DFSDMSEL_SYSCLK   (1 << 31)   /**< DFSDM source is SYSCLK.    */
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

#define STM32_LSCOSEL_MASK      (3 << 24)   /**< LSCO pin clock source.     */
#define STM32_LSCOSEL_NOCLOCK   (0 << 24)   /**< No clock on LSCO pin.      */
#define STM32_LSCOSEL_LSI       (1 << 24)   /**< LSI on LSCO pin.           */
#define STM32_LSCOSEL_LSE       (3 << 24)   /**< LSE on LSCO pin.           */
/** @} */

/**
 * @name    RCC_CSR register bits definitions
 * @{
 */
#define STM32_MSISRANGE_MASK    (15 << 8)   /**< MSISRANGE field mask.      */
#define STM32_MSISRANGE_1M      (4 << 8)    /**< 1MHz nominal.              */
#define STM32_MSISRANGE_2M      (5 << 8)    /**< 2MHz nominal.              */
#define STM32_MSISRANGE_4M      (6 << 8)    /**< 4MHz nominal.              */
#define STM32_MSISRANGE_8M      (7 << 8)    /**< 8MHz nominal.              */
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

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*
 * Configuration-related checks.
 */
#if !defined(STM32L4xx_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32L4xx_MCUCONF not defined"
#endif

/* Only some devices have strongly checked mcuconf.h files. Others will be
   added gradually.*/
#if defined(STM32L432xx) && !defined(STM32L432_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32L432_MCUCONF not defined"
#endif

#if defined(STM32L433xx) && !defined(STM32L433_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32L433_MCUCONF not defined"
#endif

#if defined(STM32L476xx) && !defined(STM32L476_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32L476_MCUCONF not defined"
#endif

#if defined(STM32L486xx) && !defined(STM32L486_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32L486_MCUCONF not defined"
#endif

#if defined(STM32L496xx) && !defined(STM32L496_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32L496_MCUCONF not defined"
#endif

#if defined(STM32L4A6xx) && !defined(STM32L4A6_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32L4A6_MCUCONF not defined"
#endif

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
