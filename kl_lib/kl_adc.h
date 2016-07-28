/*
 * kl_adc.h
 *
 *  Created on: 12.04.2013
 *      Author: kreyl
 */

#ifndef KL_ADC_H_
#define KL_ADC_H_

#include "kl_lib.h"
#include "board.h"

#if ADC_REQUIRED

#if defined STM32F4XX || defined STM32F0XX
// =========================== Constants and Types =============================
// ADC sampling_times
enum AdcSampleTime_t {
	ast1d5Cycles 	= 0,
	ast7d5Cycles	= 1,
	ast13d5Cycles 	= 2,
	ast28d5Cycles	= 3,
	ast41d5Cycles	= 4,
	ast55d5Cycles	= 5,
	ast71d5Cycles	= 6,
	ast239d5Cycles	= 7
};

#define ADC_MAX_VALUE   4095

// See datasheet, search VREFINT_CAL
#ifdef STM32F072xB
#define ADC_VREFINT_CAL     (*(volatile uint16_t*)0x1FFFF7BA)
#else
#define ADC_VREFINT_CAL     (*(volatile uint16_t*)0x1FFF7A2A)	// for 4xx
#endif

enum ADCDiv_t {
    adcDiv2 = (uint32_t)(0b00 << 16),
    adcDiv4 = (uint32_t)(0b01 << 16),
    adcDiv6 = (uint32_t)(0b10 << 16),
    adcDiv8 = (uint32_t)(0b11 << 16),
};
#endif

#if defined STM32F4XX
class Adc_t {
private:
    uint16_t IBuf[ADC_SEQ_LEN];
    void SetupClk(ADCDiv_t Div) { ADC->CCR |= (uint32_t)Div; }
    void SetSequenceLength(uint32_t ALen);
    void SetChannelSampleTime(uint32_t AChnl, AdcSampleTime_t ASampleTime);
    void SetSequenceItem(uint8_t SeqIndx, uint32_t AChnl);
    void StartConversion() { ADC1->CR2 |= ADC_CR2_SWSTART; }
public:
    void EnableVref()  { ADC->CCR |= (uint32_t)ADC_CCR_TSVREFE; }
    void DisableVref() { ADC->CCR &= (uint32_t)(~ADC_CCR_TSVREFE); }
    void Init();
    void StartMeasurement();
    void Disable() { ADC1->CR2 = 0; }
    void ClockOff() { rccDisableADC1(FALSE); }
    uint32_t GetResult(uint8_t AChannel);
};
#endif // f4xx

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
    uint32_t Adc2mV(uint32_t AdcChValue, uint32_t VrefValue);
    void Stop() { ADC1->CR |= ADC_CR_ADSTP; }
    void Disable();
};
#endif

extern Adc_t Adc;
#endif // ADC_REQUIRED

#endif /* KL_ADC_H_ */
