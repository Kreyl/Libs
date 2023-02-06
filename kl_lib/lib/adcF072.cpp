/*
 * adc_f2.cpp
 *
 *  Created on: 09.04.2022
 *      Author: kreyl
 */

#include <adcF072.h>
#include "board.h"
#include "MsgQ.h"
#include "shell.h"

#if ADC_REQUIRED

Adc_t Adc;
#ifdef ADC_TIM
static const Timer_t ITmr(ADC_TIM);
#endif

// Wrapper for IRQ
extern "C"
void AdcRdyIrq(void *p, uint32_t flags) {
    chSysLockFromISR();
    Adc.IOnDmaIrq();
    chSysUnlockFromISR();
}

void Adc_t::IOnDmaIrq() {
    dmaStreamDisable(PDma);
    // Switch buffers
    if(PBufW == &IBuf1) { PBufW = &IBuf2; PBufR = &IBuf1; }
    else                { PBufW = &IBuf1; PBufR = &IBuf2; }
    // Restart DMA in case of Periodic Conversion Mode
    if(Periodic) {
        dmaStreamSetMemory0(PDma, PBufW->data());
        dmaStreamSetTransactionSize(PDma, PBufW->size());
        dmaStreamSetMode(PDma, ADC_DMA_MODE);
        dmaStreamEnable(PDma);
    }
    // Signal event
    if(ICallback != nullptr) ICallback();
}

void Adc_t::Init() {
    rccResetADC1();
    rccEnableADC1(FALSE); // Enable digital clock
    // Configure clock
//    ADC1->CFGR2 = (0b01 << 30); // Clock: PCLK/2
    ADC1->CFGR2 = (0b00 << 30); // Clock: ADCCLK, will be turned on by ADC itself
    // ==== Setup channels ====
    EnableVref();
    uint32_t ChnlCnt = AdcSetup.Channels.size();
    uint32_t BufSz = ChnlCnt * (uint32_t)AdcSetup.Oversampling;
    IBuf1.resize(BufSz);
    IBuf2.resize(BufSz);
    ADC1->CHSELR = 0; // Reset it
    for(uint32_t i=0; i<ChnlCnt; i++) {
        const AdcChannel_t& Chnl = AdcSetup.Channels[i];
        if(Chnl.GPIO != nullptr) PinSetupAnalog(Chnl.GPIO, Chnl.Pin);
        ADC1->CHSELR |= (1 << Chnl.ChannelN);
    }
    ADC1->SMPR = (uint32_t)AdcSetup.SampleTime; // Setup sampling time
    ICallback = AdcSetup.DoneCallback;
    // ==== DMA ====
    PDma = dmaStreamAlloc(ADC_DMA, IRQ_PRIO_MEDIUM, AdcRdyIrq, nullptr);
    dmaStreamSetPeripheral(PDma, &ADC1->DR);
    dmaStreamSetMode      (PDma, ADC_DMA_MODE);
    ADC1->CFGR1 = ADC_CFGR1_DMACFG | ADC_CFGR1_DMAEN;
}

static bool IsEnabled() { return (ADC1->CR & ADC_CR_ADEN); }

// Not used directly, only by StartMeasurement etc.
static void Calibrate() {
    ADC1->CR |= ADC_CR_ADCAL;        // Start calibration
    while((ADC1->CR & ADC_CR_ADCAL) != 0);   // Let it to complete
    // ADEN bit cannot be set during ADCAL=1 and 4 ADC clock cycle after the ADCAL bit is cleared by hardware(end of the calibration).
    for(volatile uint32_t i=0; i<99; i++) __NOP();
}

void Adc_t:: Stop() {
    dmaStreamDisable(PDma);
    if(IsEnabled()) {
        SET_BIT(ADC1->CR, ADC_CR_ADSTP);    // Stop any ongoing conversion
        while(READ_BIT(ADC1->CR, ADC_CR_ADSTP) != 0);   // Let it to complete
        ADC1->CR |= ADC_CR_ADDIS; // Disable
        while(IsEnabled());   // Let it to complete
    }
}

void Adc_t::Deinit() {
    Stop();
#ifdef ADC_TIM
    ITmr.Deinit();
#endif
    DisableVref();
    rccDisableADC1(); // Disable clock
}

void Adc_t::EnableVref()  { ADC1_COMMON->CCR |= ADC_CCR_VREFEN; }
void Adc_t::DisableVref() { ADC1_COMMON->CCR &= ADC_CCR_VREFEN; }

// Service routine
void Adc_t::DisableCalibrateEnableSetDMA() {
    Stop();
    Calibrate();
    ADC1->ISR |= ADC_ISR_ADRDY; // Clear flags
    SET_BIT(ADC1->CR, ADC_CR_ADEN);     // Enable ADC
    while(READ_BIT(ADC1->ISR, ADC_ISR_ADRDY) == 0);   // Let it to complete
    // Disable continuous mode
    ADC1->CFGR1 &= ~ADC_CFGR1_CONT;
    // Disable trigger
    ADC1->CFGR1 &= ~ADC_CFGR1_EXTEN;
    // DMA
    dmaStreamSetMemory0(PDma, PBufW->data());
    dmaStreamSetTransactionSize(PDma, PBufW->size());
    dmaStreamSetMode(PDma, ADC_DMA_MODE);
    dmaStreamEnable(PDma);
}

// Start sequence conversion and run callback when done
void Adc_t::StartSingleMeasurement() {
    DisableCalibrateEnableSetDMA();
    Periodic = false;
    StartConversion();
}

// Start periodic conversions, run callback every time when done
#ifdef ADC_TIM
void Adc_t::StartPeriodicMeasurement(uint32_t FSmpHz) {
    DisableCalibrateEnableSetDMA();
    // Enable trigger
    ADC1->CFGR1 &= ~(0b111UL << ADC_CFGR1_EXTSEL_Pos); // Clear it
    ADC1->CFGR1 |=  (0b000UL << ADC_CFGR1_EXTSEL_Pos); // 000 = TRG0 = TIM6_TRGO
    ADC1->CFGR1 |=  (0b01UL  << ADC_CFGR1_EXTEN_Pos);  // trigger detection on the rising edge
    Periodic = true;
    StartConversion();
    // Setup timer
    ITmr.Init();
    ITmr.SetUpdateFrequencyChangingBoth(FSmpHz * (uint32_t)AdcSetup.Oversampling);
    ITmr.SelectMasterMode(mmUpdate);
    ITmr.Enable();
}
#endif

void Adc_t::StartContinuosMeasurement() {
    DisableCalibrateEnableSetDMA();
    Periodic = true;
    // Enable continuous mode
    ADC1->CFGR1 |= ADC_CFGR1_CONT;
    StartConversion();
}

uint32_t Adc_t::Adc2mV(uint32_t AdcChValue, uint32_t VrefValue) {
//    Printf("%u %u; %u\r", AdcChValue, VrefValue, ADC_VREFINT_CAL);
    return ((ADC_VREFINT_CAL_mV * (uint32_t)ADC_VREFINT_CAL / ADC_MAX_VALUE) * AdcChValue) / VrefValue;
}
#endif  // ADC_REQUIRED
