/*
 * board.h
 *
 *  Created on: 05 08 2018
 *      Author: Kreyl
 */

#pragma once

// ==== General ====
#define APP_NAME            "MusicBox3"

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

#define ADC_REQUIRED            TRUE
#define STM32_DMA_REQUIRED      TRUE    // Leave this macro name for OS

#if 1 // ========================== GPIO =======================================
// EXTI
#define INDIVIDUAL_EXTI_IRQ_REQUIRED    FALSE

// Buttons
#define BtnR_VolUp_PIN  GPIOB, 0
#define Btn_VolDown_PIN GPIOC, 2
#define BtnL_PIN        GPIOC, 5
#define BtnCenter_PIN   GPIOA, 2

// Peripheral power enable
#define PeriphyBattEN_Pin   GPIOC, 14, omOpenDrain
#define Periphy3V3EN_Pin    GPIOC, 15, omOpenDrain

// Battery Management Switch
#define BattMeasEN_Pin      GPIOC, 6, omOpenDrain

// Sensors


// CS42L52
#define AU_RESET        GPIOB, 3, omPushPull
#define AU_MCLK         GPIOA, 8, omPushPull, pudNone, AF0  // MCO (12MHz_Clk)
//#define AU_MCLK_TIM     GPIOA, 8, TIM1, 1, invNotInverted, omPushPull, 1
#define AU_SDIN         GPIOC, 3, omPushPull, pudNone, AF13 // MOSI; SAI1_A
#define AU_SDOUT        GPIOB, 5, omPushPull, pudNone, AF13 // MISO; SAI1_B
#define AU_LRCK         GPIOB, 9, omPushPull, pudNone, AF13 // FSYNC
#define AU_SCLK         GPIOB, 10, omPushPull, pudNone, AF13
#define AU_i2c          i2c1
#define AU_SAI          SAI1
#define AU_SAI_A        SAI1_Block_A
#define AU_SAI_B        SAI1_Block_B
#define AU_SAI_RccEn()  RCC->APB2ENR |= RCC_APB2ENR_SAI1EN

// Acc
#define Acc_i2c         i2c2
#define ACC_IRQ_GPIO    GPIOC
#define ACC_IRQ_PIN     13

// DBG UART
#define UART_GPIO       GPIOA
#define UART_TX_PIN     9
#define UART_RX_PIN     10
#define CMD_UART        USART1

// I2C
#define I2C1_GPIO       GPIOB
#define I2C1_SCL        6
#define I2C1_SDA        7
#define I2C2_GPIO       GPIOB
#define I2C2_SCL        13
#define I2C2_SDA        14
// I2C Alternate Function
#define I2C_AF          AF4

// USB
#define USB_DM          GPIOA, 11
#define USB_DP          GPIOA, 12
#define USB_AF          AF10
// USB detect
#define USB_DETECT_PIN  GPIOA, 0

// SD
//#define SD_PWR_PIN      GPIOD, 3
#define SD_AF           AF12
#define SD_DAT0         GPIOC,  8, omPushPull, pudPullUp, SD_AF
#define SD_DAT1         GPIOC,  9, omPushPull, pudPullUp, SD_AF
#define SD_DAT2         GPIOC, 10, omPushPull, pudPullUp, SD_AF
#define SD_DAT3         GPIOC, 11, omPushPull, pudPullUp, SD_AF
#define SD_CLK          GPIOC, 12, omPushPull, pudNone,   SD_AF
#define SD_CMD          GPIOD,  2, omPushPull, pudPullUp, SD_AF

#endif // GPIO

#if 1 // =========================== I2C =======================================
// i2cclkPCLK1, i2cclkSYSCLK, i2cclkHSI
#define I2C_CLK_SRC     i2cclkHSI
#define I2C_BAUDRATE_HZ 400000

#define BRD_EE_I2C_ADDR     0x56 // Depends on hardware connection
#endif

#if 1 // =========================== SPI =======================================
#define CC_SPI          SPI1
#define CC_SPI_AF       AF5
#endif

#if ADC_REQUIRED // ======================= Inner ADC ==========================
// Clock divider: clock is generated from the APB2
#define ADC_CLK_DIVIDER		adcDiv2

// ADC channels pins (GPIO, PIN, ADC channel)
#define ADC_Measure5V_PIN       GPIOC, 4, 13
#define ADC_VrefVirtual_PIN     GPIOC, 4, ADC_VREFINT_CHNL

#define ADC_SAMPLE_TIME         ast47d5Cycles   // ast24d5Cycles
#define ADC_OVERSAMPLING_RATIO  64   // 1 (no oversampling), 2, 4, 8, 16, 32, 64, 128, 256
#define MEAS_SampleFrequency_Hz 40

#endif

#if 1 // =========================== DMA =======================================
// ==== Uart ====
// Remap is made automatically if required
#define UART_DMA_TX     STM32_DMA_STREAM_ID(2, 6)
#define UART_DMA_RX     STM32_DMA_STREAM_ID(2, 7)
#define UART_DMA_CHNL   2
#define UART_DMA_TX_MODE(Chnl) (STM32_DMA_CR_CHSEL(Chnl) | DMA_PRIORITY_LOW | STM32_DMA_CR_MSIZE_BYTE | STM32_DMA_CR_PSIZE_BYTE | STM32_DMA_CR_MINC | STM32_DMA_CR_DIR_M2P | STM32_DMA_CR_TCIE)
#define UART_DMA_RX_MODE(Chnl) (STM32_DMA_CR_CHSEL(Chnl) | DMA_PRIORITY_MEDIUM | STM32_DMA_CR_MSIZE_BYTE | STM32_DMA_CR_PSIZE_BYTE | STM32_DMA_CR_MINC | STM32_DMA_CR_DIR_P2M | STM32_DMA_CR_CIRC)

// ==== SDMMC ====
#define STM32_SDC_SDMMC1_DMA_STREAM   STM32_DMA_STREAM_ID(2, 4)

// ==== I2C ====
#define I2C_USE_DMA     TRUE
#define I2C1_DMA_TX     STM32_DMA_STREAM_ID(1, 6)
#define I2C1_DMA_RX     STM32_DMA_STREAM_ID(1, 7)
#define I2C1_DMA_CHNL   3
//#define I2C2_DMA_TX     STM32_DMA_STREAM_ID(1, 4)
//#define I2C2_DMA_RX     STM32_DMA_STREAM_ID(1, 5)
//#define I2C2_DMA_CHNL   3
//#define I2C3_DMA_TX     STM32_DMA_STREAM_ID(1, 2)
//#define I2C3_DMA_RX     STM32_DMA_STREAM_ID(1, 3)
//#define I2C3_DMA_CHNL   3

// ==== SAI ====
#define SAI_DMA_A       STM32_DMA_STREAM_ID(2, 1)
//#define SAI_DMA_B       STM32_DMA2_STREAM2//STM32_DMA_STREAM_ID(2, 2)
#define SAI_DMA_CHNL    1

// ==== DAC ====
//#define DAC_DMA         STM32_DMA_STREAM_ID(2, 4)
//#define DAC_DMA_CHNL    3

#if ADC_REQUIRED
#define ADC_DMA         STM32_DMA_STREAM_ID(1, 1)
#define ADC_DMA_MODE    STM32_DMA_CR_CHSEL(0) |   /* DMA1 Stream1 Channel0 */ \
                        DMA_PRIORITY_LOW | \
                        STM32_DMA_CR_MSIZE_HWORD | \
                        STM32_DMA_CR_PSIZE_HWORD | \
                        STM32_DMA_CR_MINC |       /* Memory pointer increase */ \
                        STM32_DMA_CR_DIR_P2M |    /* Direction is peripheral to memory */ \
                        STM32_DMA_CR_TCIE         /* Enable Transmission Complete IRQ */
#endif // ADC

#endif // DMA

#if 1 // ========================== USART ======================================
#define PRINTF_FLOAT_EN FALSE
#define UART_TXBUF_SZ   2048
#define UART_RXBUF_SZ   1024
#define CMD_BUF_SZ      1024

#define CMD_UART_PARAMS \
    CMD_UART, UART_GPIO, UART_TX_PIN, UART_GPIO, UART_RX_PIN, \
    UART_DMA_TX, UART_DMA_RX, UART_DMA_TX_MODE(UART_DMA_CHNL), UART_DMA_RX_MODE(UART_DMA_CHNL), \
    uartclkHSI // Use independent clock

#endif
