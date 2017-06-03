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

#if defined STM32F2XX || defined STM32F4XX || defined STM32F0XX
// =========================== Constants and Types =============================
// All 2xx, 4xx and F072 devices. Do not change.
#define ADC_CHNL_TEMPERATURE    16
#define ADC_CHNL_VREFINT        17
#define ADC_MAX_SEQ_LEN         16  // 1...16; Const, see ref man
// See datasheet, search VREFINT_CAL
#if defined STM32F072xB || defined STM32F030
#define ADC_VREFINT_CAL     (*(volatile uint16_t*)0x1FFFF7BA)
#else
#define ADC_VREFINT_CAL     (*(volatile uint16_t*)0x1FFF7A2A)   // for 4xx, ds p139
#endif

// ADC sampling_times
enum AdcSampleTime_t {
#if defined STM32F0XX
	ast1d5Cycles 	= 0,
	ast7d5Cycles	= 1,
	ast13d5Cycles 	= 2,
	ast28d5Cycles	= 3,
	ast41d5Cycles	= 4,
	ast55d5Cycles	= 5,
	ast71d5Cycles	= 6,
	ast239d5Cycles	= 7
#else
	ast3Cycles      = 0,
	ast15Cycles     = 1,
	ast28Cycles     = 2,
	ast56Cycles     = 3,
	ast84Cycles     = 4,
	ast112Cycles    = 5,
	ast144Cycles    = 6,
	ast480Cycles    = 7
#endif
};

enum ADCDiv_t {
    adcDiv2 = (uint32_t)(0b00 << 16),
    adcDiv4 = (uint32_t)(0b01 << 16),
    adcDiv6 = (uint32_t)(0b10 << 16),
    adcDiv8 = (uint32_t)(0b11 << 16),
};
#endif

#if defined STM32F0XX
class Adc_t {
private:
    uint16_t IBuf[ADC_SEQ_LEN];
    void StartConversion() { ADC1->CR |= ADC_CR_ADSTART; }
    void IDisableNow() { ADC1->CR = 0; }  // Clear all bits
public:
    void Init();
    void StartMeasurement();
    void EnableVRef()  { ADC->CCR |=  ADC_CCR_VREFEN; }
    void DisableVRef() { ADC->CCR &= ~ADC_CCR_VREFEN; }
    uint32_t GetResult(uint8_t AChannel);
    uint32_t Adc2mV(uint32_t AdcChValue, uint32_t VrefValue) {
        return ((3300UL * ADC_VREFINT_CAL / ADC_MAX_VALUE) * AdcChValue) / VrefValue;
    }
    uint32_t Vref2mV(uint32_t VrefValue) {
        return (3300UL * ADC_VREFINT_CAL) / VrefValue;
    }
    void Stop() { ADC1->CR |= ADC_CR_ADSTP; }
    void Disable();
};
#endif

#if defined STM32L1XX
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
    void ClockOff() { rccDisableADC1(FALSE); }
    uint32_t GetResult(uint8_t AChannel);
    uint32_t Adc2mV(uint32_t AdcChValue, uint32_t VrefValue) {
        return ((3300UL * ADC_VREFINT_CAL / ADC_MAX_VALUE) * AdcChValue) / VrefValue;
    }
};
#endif // f4xx & L151

#if defined STM32L4XX
#define ADC_VREFINT_CHNL        0   // Do not change, see ref man
#define ADC_TEMPERATURE_CHNL    17
#define ADC_VBAT_CHNL           18
#define ADC_MAX_SEQ_LEN         16  // 1...16; Const, see ref man
#define ADC_VREFINT_CAL         (*(volatile uint16_t*)0x1FFF75AA)

#define ADC_SEQ_LEN             (ADC_CHANNEL_CNT)

#if (ADC_SEQ_LEN > ADC_MAX_SEQ_LEN) || (ADC_SEQ_LEN == 0)
#error "Wrong ADC channel count and sample count"
#endif

// ADC sampling_times
enum AdcSampleTime_t {
    ast2d5Cycles    = 0,
    ast6d5Cycles    = 1,
    ast12d5Cycles   = 2,
    ast24d5Cycles   = 3,
    ast47d5Cycles   = 4,
    ast92d5Cycles   = 5,
    ast247d5Cycles  = 6,
    ast640d5Cycles  = 7
};

class Adc_t {
private:
    uint16_t IBuf[ADC_SEQ_LEN];
    void Calibrate();
    bool IsEnabled() { return READ_BIT(ADC1->CR, ADC_CR_ADEN); }
    void SetSequenceLength(uint32_t ALen);
    void SetChannelSampleTime(uint32_t AChnl, AdcSampleTime_t ASampleTime);
    void SetSequenceItem(uint8_t SeqIndx, uint32_t AChnl);
    void StartConversion() { ADC1->CR |= ADC_CR_ADSTART; }
public:
    void Init();
    void EnableVref()  { ADC123_COMMON->CCR |= ADC_CCR_VREFEN; }
    void DisableVref() { ADC123_COMMON->CCR &= ADC_CCR_VREFEN; }
    void StartMeasurement();
    uint32_t Adc2mV(uint32_t AdcChValue, uint32_t VrefValue);
    void Disable() { SET_BIT(ADC1->CR, ADC_CR_ADDIS); }
    void ClockOff() { rccDisableADC123(FALSE); }
    uint32_t GetResult(uint8_t AChannel);
};
#endif

extern Adc_t Adc;
#endif // ADC_REQUIRED
