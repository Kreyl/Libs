/*
 * ws2812b.h
 *
 *  Created on: 05 апр. 2014 г.
 *      Author: Kreyl
 */

#pragma once

/*
 * ========== WS2812 control module ==========
 * Only basic command "SetCurrentColors" is implemented, all other is up to
 * higher level software.
 * SPI input frequency should be 8 MHz (which results in 4MHz bitrate)
 */


#include "ch.h"
#include "hal.h"
#include "kl_lib.h"
#include "color.h"
#include "uart.h"

#define LED_CNT             3   // Number of WS2812 LEDs

#define NPX_POWER_PIN_EN    TRUE

// Do not touch
#define SEQ_LEN             4
#define RST_W_CNT           4 // zero words before and after data to produce reset

// SPI16 Buffer (no tuning required)
#define DATA_BIT_CNT        (LED_CNT * 3 * 8 * SEQ_LEN)   // Each led has 3 channels 8 bit each
#define DATA_W_CNT          ((DATA_BIT_CNT + 15) / 16)
#define TOTAL_W_CNT         (DATA_W_CNT + RST_W_CNT)

#define NPX_DMA_MODE(Chnl) \
                        (STM32_DMA_CR_CHSEL(Chnl) \
                        | DMA_PRIORITY_HIGH \
                        | STM32_DMA_CR_MSIZE_HWORD \
                        | STM32_DMA_CR_PSIZE_HWORD \
                        | STM32_DMA_CR_MINC     /* Memory pointer increase */ \
                        | STM32_DMA_CR_DIR_M2P)  /* Direction is memory to peripheral */ \
                        | STM32_DMA_CR_TCIE

struct NeopixelParams_t {
    Spi_t ISpi;
    GPIO_TypeDef *PGpio;
    uint16_t Pin;
    AlterFunc_t Af;
    // DMA
    const stm32_dma_stream_t *PDma;
    uint32_t DmaMode;
#if NPX_POWER_PIN_EN
    GPIO_TypeDef *IPwrPGPIO;
    uint16_t IPwrPin;
    PinOutMode_t IPwrOutputType;
#endif
    NeopixelParams_t(SPI_TypeDef *ASpi,
            GPIO_TypeDef *APGpio, uint16_t APin, AlterFunc_t AAf,
            const stm32_dma_stream_t *APDma, uint32_t ADmaMode
#if NPX_POWER_PIN_EN
    , GPIO_TypeDef *APwrPGPIO, uint16_t APwrPin, PinOutMode_t APwrOutputType
#endif
    ) :
                ISpi(ASpi), PGpio(APGpio), Pin(APin), Af(AAf),
                PDma(APDma), DmaMode(ADmaMode)
#if NPX_POWER_PIN_EN
                , IPwrPGPIO(APwrPGPIO), IPwrPin(APwrPin), IPwrOutputType(APwrOutputType)
#endif
    {}
};


class Neopixels_t {
private:
    const NeopixelParams_t *Params;
    uint16_t IBuf[TOTAL_W_CNT];
    uint16_t *PBuf;
    void AppendBitsMadeOfByte(uint8_t Byte);
#if NPX_POWER_PIN_EN
    PinOutput_t PwrPin;
    bool IsAllBlack = true;
#endif
public:
    Neopixels_t(const NeopixelParams_t *APParams) : Params(APParams), PBuf(IBuf)
#if NPX_POWER_PIN_EN
        , PwrPin(Params->IPwrPGPIO, Params->IPwrPin, Params->IPwrOutputType)
#endif
    {}

    void Init();
    bool AreOff() {
        for(uint8_t i=0; i<LED_CNT; i++) {
            if(ICurrentClr[i] != clBlack) return false;
        }
        return true;
    }
    // Inner use
    Color_t ICurrentClr[LED_CNT];
    void ISetCurrentColors();
    void OnDmaDone();
};
