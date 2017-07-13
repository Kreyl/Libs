/*
 * board.h
 *
 *  Created on: 12 сент. 2015 г.
 *      Author: Kreyl
 */

#pragma once

#include <inttypes.h>

// ==== General ====
#define BOARD_NAME          "GSLeaf01"
#define APP_NAME            "GSLeaf"

// MCU type as defined in the ST header.
#define STM32L476xx

// Freq of external crystal if any. Leave it here even if not used.
#define CRYSTAL_FREQ_HZ         12000000

// OS timer settings
#define STM32_ST_IRQ_PRIORITY   2
#define STM32_ST_USE_TIMER      5
#define SYS_TIM_CLK             (Clk.APB1FreqHz)    // Timer 5 is clocked by APB1

//  Periphery
#define I2C1_ENABLED            TRUE
#define I2C2_ENABLED            FALSE
#define I2C3_ENABLED            FALSE
#define SIMPLESENSORS_ENABLED   TRUE
#define BUTTONS_ENABLED         TRUE

#define ADC_REQUIRED            FALSE
#define STM32_DMA_REQUIRED      TRUE    // Leave this macro name for OS

#if 1 // ========================== GPIO =======================================
// EXTI
#define INDIVIDUAL_EXTI_IRQ_REQUIRED    FALSE

// Buttons
#define BTN1_PIN        GPIOA, 0, pudPullDown
#define BTN2_PIN        GPIOA, 1, pudPullDown

// UART
#define UART_GPIO       GPIOA
#define UART_TX_PIN     9
#define UART_RX_PIN     10
#define UART_AF         AF7 // for all USARTs

// RGB LED
#define LED_RED_CH      { GPIOB, 0, TIM3, 3, invNotInverted, omPushPull, 255 }
#define LED_GREEN_CH    { GPIOB, 1, TIM3, 4, invNotInverted, omPushPull, 255 }
#define LED_BLUE_CH     { GPIOB, 4, TIM3, 1, invNotInverted, omPushPull, 255 }

// PWR_EN
#define PWR_EN_PIN      GPIOA, 15, omPushPull
// Charge
#define CHARGE_PIN      GPIOB, 11, pudPullUp

// CS42L52
#define AU_RESET        GPIOB, 3, omPushPull
#define AU_SDIN         GPIOC, 3, omPushPull, pudNone, AF13 // MOSI; SAI1_A
#define AU_SDOUT        GPIOB, 5, omPushPull, pudNone, AF13 // MISO; SAI1_B
#define AU_MCLK         GPIOB, 8, omPushPull, pudNone, AF13
#define AU_MCLK_TIM     { GPIOB, 8, TIM4, 3, invNotInverted, omPushPull, 1 }
#define AU_LRCK         GPIOB, 9, omPushPull, pudNone, AF13
#define AU_SCLK         GPIOB, 10, omPushPull, pudNone, AF13
#define AU_i2c          i2c1
#define AU_SAI          SAI1
#define AU_SAI_A        SAI1_Block_A
#define AU_SAI_B        SAI1_Block_B
#define AU_SAI_RccEn()  RCC->APB2ENR |= RCC_APB2ENR_SAI1EN

// Acc
#define Acc_i2c         i2c1
#define ACC_IRQ_GPIO    GPIOB
#define ACC_IRQ_PIN     2

// I2C
#define I2C1_GPIO       GPIOB
#define I2C1_SCL        6
#define I2C1_SDA        7
#define I2C2_GPIO       GPIOB
#define I2C2_SCL        10
#define I2C2_SDA        11
#define I2C3_GPIO       GPIOC
#define I2C3_SCL        0
#define I2C3_SDA        1
// I2C Alternate Function
#define I2C_AF          AF4

// SD
#define SD_PWR_PIN      GPIOC, 7
#define SD_AF           AF12
#define SD_DAT0         GPIOC,  8, omPushPull, pudPullUp, SD_AF
#define SD_DAT1         GPIOC,  9, omPushPull, pudPullUp, SD_AF
#define SD_DAT2         GPIOC, 10, omPushPull, pudPullUp, SD_AF
#define SD_DAT3         GPIOC, 11, omPushPull, pudPullUp, SD_AF
#define SD_CLK          GPIOC, 12, omPushPull, pudNone,   SD_AF
#define SD_CMD          GPIOD,  2, omPushPull, pudPullUp, SD_AF

// Radio: SPI, PGpio, Sck, Miso, Mosi, Cs, Gdo0
#define CC_Setup0       SPI1, GPIOA, 5,6,7, 4, 3

#endif // GPIO

#if 1 // =========================== SPI =======================================
#define CC_SPI          SPI1
#define CC_SPI_AF       AF5
#endif

#if 1 // ========================== USART ======================================
#define PRINTF_FLOAT_EN FALSE
#define CMD_UART        USART1
#define UART_USE_INDEPENDENT_CLK    TRUE
#define UART_TXBUF_SZ   1024
#endif

#if 1 // =========================== I2C =======================================
// i2cclkPCLK1, i2cclkSYSCLK, i2cclkHSI
#define I2C_CLK_SRC     i2cclkHSI
#define I2C_BAUDRATE_HZ 400000
#endif

#if ADC_REQUIRED // ======================= Inner ADC ==========================
// Clock divider: clock is generated from the APB2
#define ADC_CLK_DIVIDER		adcDiv2

// ADC channels
#define ADC_BATTERY_CHNL 	14
// ADC_VREFINT_CHNL
#define ADC_CHANNELS        { ADC_BATTERY_CHNL, ADC_VREFINT_CHNL }
#define ADC_CHANNEL_CNT     2   // Do not use countof(AdcChannels) as preprocessor does not know what is countof => cannot check
#define ADC_SAMPLE_TIME     ast24d5Cycles
#define ADC_OVERSAMPLING_RATIO  64   // 1 (no oversampling), 2, 4, 8, 16, 32, 64, 128, 256
#endif

#if 1 // =========================== DMA =======================================
// ==== Uart ====
// Remap is made automatically if required
#define UART_DMA_TX     STM32_DMA1_STREAM4
#define UART_DMA_RX     STM32_DMA1_STREAM5
#define UART_DMA_CHNL   2

// ==== I2C ====
#define I2C1_DMA_TX     STM32_DMA2_STREAM7
#define I2C1_DMA_RX     STM32_DMA2_STREAM6
#define I2C1_DMA_CHNL   5
#define I2C2_DMA_TX     STM32_DMA1_STREAM4
#define I2C2_DMA_RX     STM32_DMA1_STREAM5
#define I2C2_DMA_CHNL   3
#define I2C3_DMA_TX     STM32_DMA1_STREAM2
#define I2C3_DMA_RX     STM32_DMA1_STREAM3
#define I2C3_DMA_CHNL   3

// ==== SAI ====
#define SAI_DMA_A       STM32_DMA2_STREAM1
#define SAI_DMA_B       STM32_DMA2_STREAM2
#define SAI_DMA_CHNL    1

// ==== SDMMC ====
#define STM32_SDC_SDMMC1_DMA_STREAM   STM32_DMA_STREAM_ID(2, 5)

#if ADC_REQUIRED
#define ADC_DMA         STM32_DMA1_STREAM1
#define ADC_DMA_MODE    STM32_DMA_CR_CHSEL(0) |   /* DMA1 Stream1 Channel0 */ \
                        DMA_PRIORITY_LOW | \
                        STM32_DMA_CR_MSIZE_HWORD | \
                        STM32_DMA_CR_PSIZE_HWORD | \
                        STM32_DMA_CR_MINC |       /* Memory pointer increase */ \
                        STM32_DMA_CR_DIR_P2M |    /* Direction is peripheral to memory */ \
                        STM32_DMA_CR_TCIE         /* Enable Transmission Complete IRQ */
#endif // ADC

#endif // DMA
