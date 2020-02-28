#pragma once

/*
 * ========== WS2812 control module ==========
 * Only basic command "SetCurrentColors" is implemented, all other is up to
 * higher level software.
 * WS2812 requires following timings, ns: (400 + 850) +- 150 each.
 *
 * SPI bitrate must be 2500kHz => 400nS per bit.
 * Then, LED's 0 is 100 (400 + 800), LED's 1 is 110 (800 + 400).
 * Reset must be:
 *    WS2812: 50uS => 125bit => ~16 bytes
 *    WS2813: 300uS => 750bit => 94 bytes
 */

#include "ch.h"
#include "hal.h"
#include "kl_lib.h"
#include "color.h"
#include "uart.h"
#include "board.h"
#include <vector>

typedef std::vector<Color_t> ColorBuf_t;

// SPI8 Buffer (no tuning required)
#define NPX_SEQ_LEN_BITS        3 // 3 bits of SPI to produce 1 bit of LED data
#define NPX_RST_BYTE_CNT        100
#define DATA_BIT_CNT(LedCnt)    (LedCnt * 3 * 8 * NPX_SEQ_LEN_BITS) // Each led has 3 channels 8 bit each
#define DATA_BYTE_CNT(LedCnt)   ((DATA_BIT_CNT(LedCnt) + 7) / 8)
#define TOTAL_BYTE_CNT(LedCnt)  (DATA_BYTE_CNT(LedCnt) + NPX_RST_BYTE_CNT)

#define NPX_DMA_MODE(Chnl) \
                        (STM32_DMA_CR_CHSEL(Chnl) \
                        | DMA_PRIORITY_HIGH \
                        | STM32_DMA_CR_MSIZE_BYTE \
                        | STM32_DMA_CR_PSIZE_BYTE \
                        | STM32_DMA_CR_MINC     /* Memory pointer increase */ \
                        | STM32_DMA_CR_DIR_M2P)  /* Direction is memory to peripheral */ \
                        | STM32_DMA_CR_TCIE

// Band setup
enum BandDirection_t {dirForward, dirBackward};
struct BandSetup_t {
    int32_t Length;
    BandDirection_t Dir;
};

struct NeopixelParams_t {
    // SPI
    Spi_t ISpi;
    GPIO_TypeDef *PGpio;
    uint16_t Pin;
    AlterFunc_t Af;
    // DMA
    uint32_t DmaID;
    uint32_t DmaMode;
    NeopixelParams_t(SPI_TypeDef *ASpi,
            GPIO_TypeDef *APGpio, uint16_t APin, AlterFunc_t AAf,
            uint32_t ADmaID, uint32_t ADmaMode) :
                ISpi(ASpi), PGpio(APGpio), Pin(APin), Af(AAf),
                DmaID(ADmaID), DmaMode(ADmaMode) {}
};


class Neopixels_t {
private:
    uint32_t IBitBufSz = 0;
    uint8_t *IBitBuf = nullptr;
    const NeopixelParams_t *Params;
    const stm32_dma_stream_t *PDma;
public:
    // Band setup
    const int32_t BandCnt;
    const BandSetup_t *BandSetup;
    bool TransmitDone = false;
    ftVoidVoid OnTransmitEnd = nullptr;
    // Methods
    Neopixels_t(const NeopixelParams_t *APParams,
            const uint32_t ABandCnt, const BandSetup_t *PBandSetup) : Params(APParams),
                    BandCnt(ABandCnt), BandSetup(PBandSetup) { }
    void SetCurrentColors();
    void OnDmaDone();
    int32_t LedCntTotal = 0;
    ColorBuf_t ClrBuf;
    void Init();
    void SetAll(Color_t Clr) {
        for(int32_t i=0; i<LedCntTotal; i++) ClrBuf[i] = Clr;
    }
    bool AreOff() {
        for(uint8_t i=0; i<LedCntTotal; i++) { if(ClrBuf[i] != clBlack) return false; }
        return true;
    }
};
