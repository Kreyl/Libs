/*
    ChibiOS - Copyright (C) 2006..2020 Giovanni Di Sirio

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

/*
 * STM32L4xx drivers configuration.
 * The following settings override the default settings present in
 * the various device driver implementation headers.
 * Note that the settings for each driver only have effect if the whole
 * driver is enabled in halconf.h.
 *
 * IRQ priorities:
 * 15...0       Lowest...Highest.
 *
 * DMA priorities:
 * 0...3        Lowest...Highest.
 */

#ifndef MCUCONF_H
#define MCUCONF_H

#define STM32L4xx_MCUCONF
#define STM32L475_MCUCONF
#define STM32L476_MCUCONF
#define STM32L486_MCUCONF

/*
 * SDC driver system settings.
 */
#define STM32_SDC_USE_SDMMC1                TRUE
#define STM32_SDC_SDMMC_UNALIGNED_SUPPORT   TRUE
#define STM32_SDC_SDMMC_WRITE_TIMEOUT       1000
#define STM32_SDC_SDMMC_READ_TIMEOUT        1000
#define STM32_SDC_SDMMC_CLOCK_DELAY         10
#define STM32_SDC_SDMMC1_DMA_PRIORITY       3
#define STM32_SDC_SDMMC1_IRQ_PRIORITY       9
//#define STM32_SDC_SDMMC1_DMA_STREAM         STM32_DMA_STREAM_ID(2, 4)

/*
 * ST driver system settings.
 */
//#define STM32_ST_IRQ_PRIORITY               8
//#define STM32_ST_USE_TIMER                  2

/*
 * USB driver system settings.
 */
#define STM32_USB_USE_OTG1                  FALSE
#define STM32_USB_OTG1_IRQ_PRIORITY         14
#define STM32_USB_OTG1_RX_FIFO_SIZE         512

#endif /* MCUCONF_H */
