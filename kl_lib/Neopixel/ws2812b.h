#pragma once



/*
 * ========== WS2812 control module ==========
 * Only basic command "SetCurrentColors" is implemented, all other is up to
 * higher level software.
 * There are different timings for V2 and V5. Wir verachten sie.
 */
//#define WS2812B_V2    TRUE
#define WS2812B_V5      TRUE

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
#define NPX_SEQ_LEN_BITS        3 // 3 bits of SPI to produce 1 bit of LED data
#define NPX_RST_BYTE_CNT        100
#define DATA_BIT_CNT(LedCnt)    (LedCnt * 3 * 8 * NPX_SEQ_LEN_BITS) // Each led has 3 channels 8 bit each
#define DATA_BYTE_CNT(LedCnt)   ((DATA_BIT_CNT(LedCnt) + 7) / 8)
#define TOTAL_BYTE_CNT(LedCnt)  (DATA_BYTE_CNT(LedCnt) + NPX_RST_BYTE_CNT)

#else // WS2812B_V5
#define NPX_SPI_BITRATE         3000000
#define NPX_BYTES_PER_BYTE      4 // 2 bits are 1 byte, 8 bits are 4 bytes
#define NPX_RST_BYTE_CNT        108
#define NPX_DATA_BYTE_CNT(LedCnt)   ((LedCnt) * 3 * NPX_BYTES_PER_BYTE)
#define NPX_TOTAL_BYTE_CNT(LedCnt)  (NPX_DATA_BYTE_CNT(LedCnt) + NPX_RST_BYTE_CNT)
#define NPX_WORD_CNT(LedCnt)        ((NPX_TOTAL_BYTE_CNT(LedCnt) + 1) / 2)
#endif

#define NPX_DMA_MODE(Chnl) \
                        (STM32_DMA_CR_CHSEL(Chnl) \
                        | DMA_PRIORITY_HIGH \
                        | STM32_DMA_CR_MSIZE_HWORD \
                        | STM32_DMA_CR_PSIZE_HWORD \
                        | STM32_DMA_CR_MINC     /* Memory pointer increase */ \
                        | STM32_DMA_CR_DIR_M2P)  /* Direction is memory to peripheral */ \
                        | STM32_DMA_CR_TCIE

void NpxPrintTable();

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
    uint32_t IBitBufWordCnt = 0;
    uint32_t *IBitBuf = nullptr;
    const NeopixelParams_t *Params;
    const stm32_dma_stream_t *PDma = nullptr;
public:
    // Band setup
    const int32_t BandCnt;
    const BandSetup_t *BandSetup;
    bool TransmitDone = false;
    ftVoidVoid OnTransmitEnd = nullptr;
    // Methods
    Neopixels_t(const NeopixelParams_t *APParams,
            const uint32_t ABandCnt, const BandSetup_t *PBandSetup) :
                Params(APParams), BandCnt(ABandCnt), BandSetup(PBandSetup) { }
    void SetCurrentColors();
    void OnDmaDone();
    ColorBuf_t ClrBuf;
    void Init();
    void SetAll(Color_t Clr) {
        for(auto &IClr : ClrBuf) IClr = Clr;
    }
    bool AreOff() {
        for(auto &IClr : ClrBuf) {
            if(IClr != clBlack) return false;
        }
        return true;
    }
};
