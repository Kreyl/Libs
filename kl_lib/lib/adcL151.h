/*
 * kl_adc.h
 *
 *  Created on: 12.04.2013
 *      Author: kreyl
 */

#pragma once

#include "kl_lib.h"
#include "board.h"

#if ADC_REQUIRED

#define ADC_MAX_VALUE           4095    // const: 2^12
extern const uint8_t AdcChannels[ADC_CHANNEL_CNT];

/*
 * Don't forget to enable HSI. It runs on HSI.
 */

#define ADC_MAX_SEQ_LEN     27  // 1...27; Const, see ref man p.301

#if (ADC_SEQ_LEN > ADC_MAX_SEQ_LEN) || (ADC_SEQ_LEN == 0)
#error "Wrong ADC channel count and sample count"
#endif

// See datasheet, search VREFINT_CAL
#define ADC_VREFINT_CAL     (*(volatile uint16_t*)0x1FF80078)

enum AdcSampleTime_t {
        ast4Cycles = 0b000,
        ast9Cycles = 0b001,
        ast16Cycles = 0b010,
        ast24Cycles = 0b011,
        ast48Cycles = 0b100,
        ast96Cycles = 0b101,
        ast192Cycles = 0b110,
        ast384Cycles = 0b111
};

#define ADC_SAMPLE_TIME_DEFAULT     ast96Cycles

enum ADCDiv_t {
    adcDiv1 = (uint32_t)(0b00 << 16),
    adcDiv2 = (uint32_t)(0b01 << 16),
    adcDiv4 = (uint32_t)(0b10 << 16),
};
#endif

#if defined STM32F2XX || defined STM32F4XX ||defined STM32L1XX
class Adc_t {
private:
    uint16_t IBuf[ADC_SEQ_LEN];
    void SetupClk(ADCDiv_t Div) { ADC->CCR = (ADC->CCR & ~ADC_CCR_ADCPRE) | (uint32_t)Div; }
    void SetSequenceLength(uint32_t ALen);
    void SetChannelSampleTime(uint32_t AChnl, AdcSampleTime_t ASampleTime);
    void SetSequenceItem(uint8_t SeqIndx, uint32_t AChnl);
    void StartConversion() { ADC1->CR2 |= ADC_CR2_SWSTART; }
public:
    void EnableVRef()  { ADC->CCR |= (uint32_t)ADC_CCR_TSVREFE; }
    void DisableVRef() { ADC->CCR &= (uint32_t)(~ADC_CCR_TSVREFE); }
    uint32_t GetVDAmV(uint32_t VrefADC) { return ((ADC_VREFINT_CAL * 3000UL) / VrefADC); }
    void Init();
    void StartMeasurement();
    void Disable() { ADC1->CR2 = 0; }
    void ClockOff() { rccDisableADC1(); }
    uint32_t GetResult(uint8_t AChannel);
    uint32_t Adc2mV(uint32_t AdcChValue, uint32_t VrefValue) {
        return ((3300UL * ADC_VREFINT_CAL / ADC_MAX_VALUE) * AdcChValue) / VrefValue;
    }
};

extern Adc_t Adc;
#endif // ADC_REQUIRED
