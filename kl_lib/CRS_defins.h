/*
 * CRS_defins.h
 *
 *  Created on: 26 дек. 2015 г.
 *      Author: Kreyl
 */

#pragma once

#define RCC_CRS_SYNC_DIV1        ((uint32_t)0x00)                          /*!< Synchro Signal not divided (default) */
#define RCC_CRS_SYNC_DIV2        CRS_CFGR_SYNCDIV_0                        /*!< Synchro Signal divided by 2 */
#define RCC_CRS_SYNC_DIV4        CRS_CFGR_SYNCDIV_1                        /*!< Synchro Signal divided by 4 */
#define RCC_CRS_SYNC_DIV8        (CRS_CFGR_SYNCDIV_1 | CRS_CFGR_SYNCDIV_0) /*!< Synchro Signal divided by 8 */
#define RCC_CRS_SYNC_DIV16       CRS_CFGR_SYNCDIV_2                        /*!< Synchro Signal divided by 16 */
#define RCC_CRS_SYNC_DIV32       (CRS_CFGR_SYNCDIV_2 | CRS_CFGR_SYNCDIV_0) /*!< Synchro Signal divided by 32 */
#define RCC_CRS_SYNC_DIV64       (CRS_CFGR_SYNCDIV_2 | CRS_CFGR_SYNCDIV_1) /*!< Synchro Signal divided by 64 */
#define RCC_CRS_SYNC_DIV128      CRS_CFGR_SYNCDIV                          /*!< Synchro Signal divided by 128 */

// RCCEx_CRS_SynchroSource RCCEx CRS SynchroSource
#define RCC_CRS_SYNC_SOURCE_GPIO       ((uint32_t)0x00)        /*!< Synchro Signal soucre GPIO */
#define RCC_CRS_SYNC_SOURCE_LSE        CRS_CFGR_SYNCSRC_0      /*!< Synchro Signal source LSE */
#define RCC_CRS_SYNC_SOURCE_USB        CRS_CFGR_SYNCSRC_1      /*!< Synchro Signal source USB SOF (default)*/

// RCCEx_CRS_SynchroPolarity RCCEx CRS SynchroPolarity
#define RCC_CRS_SYNC_POLARITY_RISING        ((uint32_t)0x00)      /*!< Synchro Active on rising edge (default) */
#define RCC_CRS_SYNC_POLARITY_FALLING       CRS_CFGR_SYNCPOL      /*!< Synchro Active on falling edge */

// RCCEx_CRS_ReloadValueDefault RCCEx CRS ReloadValueDefault
#define RCC_CRS_RELOADVALUE_DEFAULT         ((uint32_t)0xBB7F)      /*!< The reset value of the RELOAD field corresponds
                                                                         to a target frequency of 48 MHz and a synchronization signal frequency of 1 kHz (SOF signal from USB). */
// RCCEx_CRS_ErrorLimitDefault RCCEx CRS ErrorLimitDefault
#define RCC_CRS_ERRORLIMIT_DEFAULT          ((uint32_t)0x22)      /*!< Default Frequency error limit */

// RCCEx_CRS_HSI48CalibrationDefault RCCEx CRS HSI48CalibrationDefault
#define RCC_CRS_HSI48CALIBRATION_DEFAULT    ((uint32_t)0x20)   /*!< The default value is 32, which corresponds to the middle of the trimming interval.
                                                                    The trimming step is around 67 kHz between two consecutive TRIM steps. A higher TRIM value
                                                                    corresponds to a higher output frequency */

