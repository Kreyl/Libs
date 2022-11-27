#pragma once

/*
 * ========== WS2812 control module ==========
 * Only basic command "SetCurrentColors" is implemented, all other is up to
 * higher level software.
 * There are different timings for V2 and V5. Wir verachten sie.
 */
//#define WS2812B_V2    TRUE
#define WS2812B_V5    TRUE  // (and SK6812SIDE)

/*
 * WS2812 V2 requires timings, ns: (400 + 850) +- 150 each.
 *
 * SPI bitrate must be 2500kHz => 400nS per bit.
 * Then, LED's 0 is 100 (400 + 800), LED's 1 is 110 (800 + 400).
 * Reset must be:
 *    WS2812: 50uS => 125bit => ~16 bytes
 *    WS2813: 300uS => 750bit => 94 bytes
 *
 * WS2812 V5 requires timings, ns:
 * 0 is [220;380]+[580;1000]
 * 1 is [580;1000]+[580;1000]
 * SPI bitrate must be 3000 kHz => 333nS per bit.
 * Then, LED's 0 is 100 (333+666), LED's 1 is 1100 (666+666).
 * To simplify BitBuffer construction, using 0 as 1000 (333+999) and 1 as 1100,
 * thus 1 LED byte is 4 real bytes.
 *
 * SK6812-SIDE (4020) requires timings, ns:
 * 0 is [200;400(typ 320)]+[800;...]
 * 1 is [620;1000(typ 640)]+[200;...]
 * SPI bitrate must be 3000 kHz => 333nS per bit.
 * Then, LED's 0 is 1000 (333+999), LED's 1 is 110 (666+333).
 * To simplify BitBuffer construction, using 0 as 1000 and 1 as 1100 (666+666),
 * thus 1 LED byte is 4 real bytes.
 * Reset must be: 80uS => ~248bit => ~31 bytes
 *
== EXAMPLE ==
#define NPX_LED_CNT     (14*3)
#define NPX_SPI         SPI2
#define NPX_DATA_PIN    GPIOC, 3, AF1
#define NPX_PWR_EN      GPIOC, 0

static const NeopixelParams_t NpxParams{NPX_SPI, NPX_DATA_PIN, LEDWS_DMA, NPX_DMA_MODE(0), NPX_LED_CNT, npxRGBW};
Neopixels_t Leds{&NpxParams};
 */

#include "ch.h"
#include "hal.h"
#include "kl_lib.h"
#include "color.h"
#include "uart.h"
#include "board.h"
#include <vector>

typedef std::vector<Color_t> ColorBuf_t;

// SPI Buffer (no tuning required)
#if WS2812B_V2
#define NPX_SPI_BITRATE         2500000
#define NPX_SPI_BITNUMBER       bitn8
#define NPX_BYTES_PER_BYTE      3 // 3 bits of SPI to produce 1 bit of LED data
#define NPX_RST_BYTE_CNT        100

#define NPX_DMA_MODE(Chnl) \
                        (STM32_DMA_CR_CHSEL(Chnl) \
                        | DMA_PRIORITY_HIGH \
                        | STM32_DMA_CR_MSIZE_BYTE \
                        | STM32_DMA_CR_PSIZE_BYTE \
                        | STM32_DMA_CR_MINC     /* Memory pointer increase */ \
                        | STM32_DMA_CR_DIR_M2P)  /* Direction is memory to peripheral */ \
                        | STM32_DMA_CR_TCIE

#else // WS2812B_V5 and SK6812SIDE
#define NPX_SPI_BITRATE         3000000
#define NPX_SPI_BITNUMBER       bitn16
#define NPX_BYTES_PER_BYTE      4 // 2 bits are 1 byte, 8 bits are 4 bytes
#define NPX_RST_BYTE_CNT        108

#define NPX_DMA_MODE(Chnl) \
                        (STM32_DMA_CR_CHSEL(Chnl) \
                        | DMA_PRIORITY_HIGH \
                        | STM32_DMA_CR_MSIZE_HWORD \
                        | STM32_DMA_CR_PSIZE_HWORD \
                        | STM32_DMA_CR_MINC     /* Memory pointer increase */ \
                        | STM32_DMA_CR_DIR_M2P)  /* Direction is memory to peripheral */ \
                        | STM32_DMA_CR_TCIE
#endif


void NpxPrintTable();

enum NpxType_t {npxRGB, npxRGBW};

struct NeopixelParams_t {
    // SPI
    Spi_t ISpi;
    GPIO_TypeDef *PGpio;
    uint16_t Pin;
    AlterFunc_t Af;
    // DMA
    uint32_t DmaID;
    uint32_t DmaMode;
    // Count
    uint32_t NpxCnt;
    NpxType_t Type;
    NeopixelParams_t(SPI_TypeDef *ASpi,
            GPIO_TypeDef *APGpio, uint16_t APin, AlterFunc_t AAf,
            uint32_t ADmaID, uint32_t ADmaMode,
            uint32_t NpxCnt, NpxType_t Type) :
                ISpi(ASpi), PGpio(APGpio), Pin(APin), Af(AAf),
                DmaID(ADmaID), DmaMode(ADmaMode), NpxCnt(NpxCnt), Type(Type) {}
};


class Neopixels_t {
private:
    uint32_t IBitBufSz = 0;
    uint8_t *IBitBuf = nullptr;
    const NeopixelParams_t *Params;
    const stm32_dma_stream_t *PDma = nullptr;
public:
    bool TransmitDone = false;
    ftVoidVoid OnTransmitEnd = nullptr;
    // Methods
    Neopixels_t(const NeopixelParams_t *APParams) : Params(APParams) { }

    void SetCurrentColors();
    void OnDmaDone();
    ColorBuf_t ClrBuf;
    void Init();
    void SetAll(Color_t Clr) { for(auto &IClr : ClrBuf) IClr = Clr; }
    void MixAllwWeight(Color_t Clr, uint32_t Weight) {
        for(auto &IClr : ClrBuf) IClr.MixwWeight(Clr, Weight);
    }
    // Brt = [0; 255]
    void SetBrightness(uint32_t Brt) {
        ColorHSV_t ClrH;
        for(auto &IClr : ClrBuf) {
            ClrH.FromRGB(IClr);
            ClrH.V = (ClrH.V * Brt) / 255;
            IClr.FromHSV(ClrH.H, ClrH.S, ClrH.V);
        }
    }
    bool AreOff() {
        for(auto &IClr : ClrBuf) if(IClr != clBlack) return false;
        return true;
    }
};
