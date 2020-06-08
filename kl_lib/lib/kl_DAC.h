/*
 * kl_DAC.h
 *
 *  Created on: 21 мая 2020 г.
 *      Author: layst
 */

#pragma once

#include <inttypes.h>

#define SAMPLING_FREQ_HZ    100000
#define TMR_DAC_SMPL        TIM7

class Dac_t {
private:
public:
    void Init();

    void EnableCh1();
    void EnableCh2();
    void DisableCh1();
    void DisableCh2();

    void SetCh1(uint16_t AValue);
    void SetCh2(uint16_t AValue);

    void InitDMAAndTmr();
    void DeInitDMAAndTmr();
    void ConstructSinAndStart(uint32_t FreqHz, uint32_t Amplitude);
};
