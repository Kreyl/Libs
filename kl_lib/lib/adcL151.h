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

// ==== Config ====
// Don't forget to enable HSI. It runs on HSI.
// If defined, HSI will be enabled and disabled when required by ADC
// #define ADC_EN_AND_DIS_HSI

// Select one of the next modes
// #define ADC_MODE_PERIODIC_MEASUREMENT
// #define ADC_MODE_SYNC_MEASUREMENT  // Sleep until measurement done
#define ADC_MODE_MEASURE_BY_REQUEST

namespace Adc {

extern const uint8_t kAdcChannels[ADC_CHANNEL_CNT];

enum SampleTime {
        ast4Cycles = 0b000,
        ast9Cycles = 0b001,
        ast16Cycles = 0b010,
        ast24Cycles = 0b011,
        ast48Cycles = 0b100,
        ast96Cycles = 0b101,
        ast192Cycles = 0b110,
        ast384Cycles = 0b111
};


void Init();
void StartMeasurement();
void StartMeasurementAndWaitCompletion();

uint32_t GetVDAmV(uint32_t Vref_ADC);
uint32_t GetResultAverage(uint8_t channel);
uint32_t GetResultMedian(uint8_t channel);
uint32_t Adc2mV(uint32_t adc_ch_value, uint32_t Vref_value);
uint32_t GetVdda_mv(uint32_t vref_value);

void ClockOff();

} // namespace Adc

#endif // ADC_REQUIRED