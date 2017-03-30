/*
 * board.h
 *
 *  Created on: 01.02.2017
 *      Author: Kreyl
 */

#pragma once

#include <inttypes.h>

// ==== General ====
#define BOARD_NAME          "DrumStick 01"
#define APP_NAME            "DrumStick"

// MCU type as defined in the ST header.
#define STM32F030x8

// Freq of external crystal if any. Leave it here even if not used.
#define CRYSTAL_FREQ_HZ     12000000

#define SYS_TIM_CLK         (Clk.APBFreqHz)

#define SIMPLESENSORS_ENABLED   TRUE
#define ADC_REQUIRED            TRUE
#define I2C1_ENABLED            TRUE
#define I2C_USE_SEMAPHORE       FALSE
#define INDIVIDUAL_EXTI_IRQ_REQUIRED    FALSE

#if 1 // ========================== GPIO =======================================
// PortMinTim_t: GPIO, Pin, Tim, TimChnl, invInverted, omPushPull, TopValue

// Buttons
#define BTN1_PIN        GPIOB, 11, pudPullDown
#define BTN2_PIN        GPIOA, 8, pudPullDown
#define BTN3_PIN        GPIOA, 0, pudPullDown
// Charge input
#define CHARGE_PIN      GPIOB, 8, pudPullUp

// Peripheral power enable
#define PWR_EN_PIN      GPIOB, 9, omPushPull
// Usb-based kbrd power
#define PWR_KBRD_PIN    GPIOA, 15, omPushPull
// Measurement enable pin
#define MEASURE_EN_PIN  GPIOB, 10, omPushPull

// Battery measurement
#define ADC_BATT_PIN    GPIOA, 1
#define ADC_KBRD_PIN    GPIOA, 2

// UART
#define UART_GPIO       GPIOA
#define UART_TX_PIN     9
#define UART_RX_PIN     10
#define UART_AF         AF1 // for USART1 @ GPIOA

// LEDs
#define LED_RED_CH      { GPIOB, 1, TIM3, 4, invNotInverted, omPushPull, 255 }
#define LED_GREEN_CH    { GPIOB, 5, TIM3, 2, invNotInverted, omPushPull, 255 }
#define LED_BLUE_CH     { GPIOB, 0, TIM3, 3, invNotInverted, omPushPull, 255 }

#define LED1_SETUP      GPIOB, 13, omPushPull
#define LED2_SETUP      GPIOB, 14, omPushPull
#define LED3_SETUP      GPIOB, 15, omPushPull

// Accelerometer
#define ACC1_INT_PIN    GPIOB, 2, pudPullDown
#define ACC2_INT_PIN    GPIOB, 4, pudPullDown

// I2C
#define I2C1_GPIO       GPIOB
#define I2C1_SCL        6
#define I2C1_SDA        7
#define I2C_AF          AF1 // I2C @ GPIOB

// Radio: SPI, PGpio, Sck, Miso, Mosi, Cs, Gdo0
#define CC_Setup0       SPI1, GPIOA, 5,6,7, 4, 3
#endif // GPIO

#if 1 // ========================== USART ======================================
#define UART            USART1
#define UART_TX_REG     UART->TDR
#define UART_RX_REG     UART->RDR
#endif

#if ADC_REQUIRED // ======================= Inner ADC ==========================
// Clock divider: clock is generated from the APB2
#define ADC_CLK_DIVIDER     adcDiv2

// ADC channels
#define ADC_BATT_CHNL       1
#define ADC_KBRD_CHNL       2

#define ADC_VREFINT_CHNL    17  // All 4xx, F030, F072 and L151 devices. Do not change.
#define ADC_CHANNELS        { ADC_BATT_CHNL, ADC_KBRD_CHNL, ADC_VREFINT_CHNL }
#define ADC_CHANNEL_CNT     2   // Do not use countof(AdcChannels) as preprocessor does not know what is countof => cannot check
#define ADC_SAMPLE_TIME     ast96Cycles
#define ADC_SAMPLE_CNT      4   // How many times to measure every channel

#define ADC_SEQ_LEN         (ADC_SAMPLE_CNT * ADC_CHANNEL_CNT)

#endif

#if 1 // =========================== DMA =======================================
#define STM32_DMA_REQUIRED  TRUE
// ==== Uart ====
#define UART_DMA_TX     STM32_DMA1_STREAM4
#define UART_DMA_RX     STM32_DMA1_STREAM5
#define UART_DMA_CHNL   0   // Dummy

// ==== I2C1 ====
#define I2C1_DMA_TX     STM32_DMA1_STREAM2
#define I2C1_DMA_RX     STM32_DMA1_STREAM3
#define I2C1_DMA_CHNL   0   // Dummy

#if ADC_REQUIRED
#define ADC_DMA         STM32_DMA1_STREAM1
#define ADC_DMA_MODE    STM32_DMA_CR_CHSEL(0) |   /* dummy */ \
                        DMA_PRIORITY_LOW | \
                        STM32_DMA_CR_MSIZE_HWORD | \
                        STM32_DMA_CR_PSIZE_HWORD | \
                        STM32_DMA_CR_MINC |       /* Memory pointer increase */ \
                        STM32_DMA_CR_DIR_P2M |    /* Direction is peripheral to memory */ \
                        STM32_DMA_CR_TCIE         /* Enable Transmission Complete IRQ */
#endif // ADC

#endif // DMA
