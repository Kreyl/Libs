/*
 * kl_adc.h
 *
 *  Created on: 12.04.2013
 *      Author: kreyl
 */

#ifndef ADCF072_H_
#define ADCF072_H_

#include "kl_lib.h"
#include "board.h"
#include <vector>

#if ADC_REQUIRED

#define ADC_MAX_VALUE           4095    // const: 2^12

// Do not change, see ref man p.211
#define ADC_TEMPERATURE_CHNL    16
#define ADC_VREFINT_CHNL        17
#define ADC_VBAT_CHNL           18
#define ADC_MAX_SEQ_LEN         16
// See datasheet, search VREFINT_CAL
#define ADC_VREFINT_CAL_mV      3300UL
#define ADC_VREFINT_CAL     (*(volatile uint16_t*)0x1FFFF7BA)

// ADC sampling_times
enum AdcSampleTime_t {
    ast1d5Cycles    = 0,
    ast7d5Cycles    = 1,
    ast13d5Cycles   = 2,
    ast28d5Cycles   = 3,
    ast41d5Cycles   = 4,
    ast55d5Cycles   = 5,
    ast71d5Cycles   = 6,
    ast239d5Cycles  = 7
};

struct AdcChannel_t {
    GPIO_TypeDef *GPIO;
    uint32_t Pin;
    uint32_t ChannelN;
};

struct AdcSetup_t {
    uint32_t SampleTime;
    enum Oversampling_t : uint32_t {
            oversmpDis=1, oversmp2=2, oversmp4=4, oversmp8=8, oversmp16=16,
            oversmp32=32, oversmp64=64, oversmp128=128, oversmp256=256
        } Oversampling;
    ftVoidVoid DoneCallback;
    std::vector<AdcChannel_t> Channels;
};

typedef std::vector<uint16_t> AdcBuf_t;

class Adc_t {
private:
    const stm32_dma_stream_t *PDma;
    AdcBuf_t IBuf1, IBuf2, *PBufW = &IBuf1, *PBufR = &IBuf2;
    ftVoidVoid ICallback = nullptr;
    void StartConversion() { ADC1->CR |= ADC_CR_ADSTART; }
    void DisableCalibrateEnableSetDMA(); // Service routine
public:
    bool Periodic;
    AdcBuf_t& GetBuf() { return *PBufR; }
    void Init();
    void Deinit();
    void EnableVref();
    void DisableVref();
    void StartSingleMeasurement();
    void StartPeriodicMeasurement(uint32_t FSmpHz);
    void StartContinuosMeasurement();
    void Stop();
    uint32_t Adc2mV(uint32_t AdcChValue, uint32_t VrefValue);
//    uint32_t GetResult(uint8_t AChannel);
    // Inner use
    void IOnDmaIrq();
};

extern Adc_t Adc;
extern const AdcSetup_t AdcSetup;

#endif // ADC_REQUIRED

#endif // ADCF072_H_
