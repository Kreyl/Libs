/*
 * tsdp.h
 *
 *  Created on: Dec 10, 2019
 *      Author: laurelindo
 */

#pragma once

#include "board.h"

enum Fs_t {fs16000 = 16000, fs22050 = 22050, fs32000 = 32000, fs44100 = 44100, fs48000 = 48000};

class TSDP_t {
private:
    const stm32_dma_stream_t *PDmaRx;
    void EnableSAI()  { TSDP_SAI_A->CR1 |=  SAI_xCR1_SAIEN; }
    void DisableSAI() { TSDP_SAI_A->CR1 &= ~SAI_xCR1_SAIEN; }
public:
    void Init();
    void StartRx(Fs_t Fs);
    void OnNewSampleI();
    void OnDmaIrq();
};

extern TSDP_t Tsdp;
