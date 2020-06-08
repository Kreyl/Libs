/*
 * kl_DAC.cpp
 *
 *  Created on: 21 мая 2020 г.
 *      Author: layst
 */

#include "kl_DAC.h"
#include "kl_lib.h"
#include "kl_buf.h"
#include "math.h"
#include "shell.h"

static const stm32_dma_stream_t *PDma;
static Timer_t SamplingTmr{TMR_DAC_SMPL};

#define DAC_DMA_MODE(Chnl) \
            (STM32_DMA_CR_CHSEL(Chnl) | \
            DMA_PRIORITY_MEDIUM | \
            STM32_DMA_CR_MSIZE_HWORD | \
            STM32_DMA_CR_PSIZE_HWORD | \
            STM32_DMA_CR_MINC | STM32_DMA_CR_DIR_M2P | STM32_DMA_CR_CIRC | STM32_DMA_CR_TCIE)

#define DAC_BUF_SZ  2000
typedef BufTypeSz_t<uint16_t, DAC_BUF_SZ>     DacBuf_t;
static DacBuf_t IBuf1, IBuf2;
static DacBuf_t *BufToWriteTo = &IBuf1;
static volatile bool NewBufIsReady = false;

void Dac_t::Init() {
    rccEnableDAC1(FALSE);
    DAC->CR = 0; // Reset value
    DAC->MCR = 0; // Connected to ext pin w buf en
    // Calibrate ch 1
    DAC->CR = DAC_CR_CEN1;
    for(uint32_t i=0; i<0b11111UL; i++) {
        DAC->CCR = (DAC->CCR & ~0b11111UL) | i;
        chThdSleepMilliseconds(1);
        if(DAC->SR & DAC_SR_CAL_FLAG1) break;
    }
    DAC->CR = 0;
    // Calibrate ch 2
    DAC->CR = DAC_CR_CEN2;
    for(uint32_t i=0; i<0b11111UL; i++) {
        DAC->CCR = (DAC->CCR & ~(0b11111UL << 16)) | (i << 16);
        chThdSleepMilliseconds(1);
        if(DAC->SR & DAC_SR_CAL_FLAG2) break;
    }
    DAC->CR = 0;
}

void Dac_t::EnableCh1() { DAC->CR |= DAC_CR_EN1; }
void Dac_t::EnableCh2() { DAC->CR |= DAC_CR_EN2; }
void Dac_t::DisableCh1() { DAC->CR &= ~DAC_CR_EN1; }
void Dac_t::DisableCh2() { DAC->CR &= ~DAC_CR_EN2; }

void Dac_t::SetCh1(uint16_t AValue) { DAC->DHR12R1 = AValue; }
void Dac_t::SetCh2(uint16_t AValue) { DAC->DHR12R2 = AValue; }

// ============================= DMA and timer =================================
void DmaUartTxIrq(void *p, uint32_t flags) {
    if(NewBufIsReady) {
        dmaStreamDisable(PDma);
        dmaStreamSetMode(PDma, DAC_DMA_MODE(DAC_DMA_CHNL));
        dmaStreamSetMemory0(PDma, BufToWriteTo->Buf);
        dmaStreamSetTransactionSize(PDma, BufToWriteTo->Length);
        dmaStreamEnable(PDma);
        BufToWriteTo = (BufToWriteTo == &IBuf1)? &IBuf2 : &IBuf1;
        NewBufIsReady = false;
    }
}

void Dac_t::InitDMAAndTmr() {
    DisableCh1();
    // Enable DAC, enable DMA, TIM7 TRGO evt as trigger, trigger enable
    DAC->CR |= DAC_CR_EN1 | DAC_CR_DMAEN1 | (0b010 << 3) | DAC_CR_TEN1;
    // ==== DMA ====
    PDma = dmaStreamAlloc(DAC_DMA, IRQ_PRIO_HIGH, DmaUartTxIrq, nullptr);
    dmaStreamSetPeripheral(PDma, &DAC->DHR12R1);
    // ==== Sampling timer ====
    SamplingTmr.Init();
    SamplingTmr.SetUpdateFrequencyChangingTopValue(SAMPLING_FREQ_HZ);
    SamplingTmr.SelectMasterMode(mmUpdate);
}

void Dac_t::DeInitDMAAndTmr() {
    dmaStreamDisable(PDma);
    SamplingTmr.Disable();
    DAC->CR &= ~(DAC_CR_EN1 | DAC_CR_DMAEN1 | (0b010 << 3) | DAC_CR_TEN1);
}

void Dac_t::ConstructSinAndStart(uint32_t FreqHz, uint32_t Amplitude) {
    uint32_t Len =  SAMPLING_FREQ_HZ / FreqHz;
    if(Len > DAC_BUF_SZ) Len = DAC_BUF_SZ;
    BufToWriteTo->Length = Len;
    uint16_t *p = BufToWriteTo->Buf;
    float Multi = 2.0 * M_PI * (float)FreqHz / (float)SAMPLING_FREQ_HZ; // normalize freq
    for(uint32_t i=0; i<Len; i++) {
        *p++ = (uint16_t)(2048.0 + Amplitude * sinf(Multi * (float)i));
    }

    // Start if not yet
    if(SamplingTmr.IsEnabled()) NewBufIsReady = true;
    else {
        NewBufIsReady = false;
        dmaStreamDisable(PDma);
        dmaStreamSetMode(PDma, DAC_DMA_MODE(DAC_DMA_CHNL));
        dmaStreamSetMemory0(PDma, BufToWriteTo->Buf);
        dmaStreamSetTransactionSize(PDma, BufToWriteTo->Length);
        dmaStreamEnable(PDma);
        SamplingTmr.Enable();
        BufToWriteTo = (BufToWriteTo == &IBuf1)? &IBuf2 : &IBuf1;
    }
}
