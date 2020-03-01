/*
 * adc_f2.cpp
 *
 *  Created on: 25 рту. 2013 у.
 *      Author: kreyl
 */

#include "board.h"
#include "MsgQ.h"
#include "shell.h"
#include "adcL476.h"

#if ADC_REQUIRED

Adc_t Adc;

static const Timer_t ITmr(ADC_TIM);

// Wrapper for IRQ
extern "C"
void AdcTxIrq(void *p, uint32_t flags) {
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
    if(ITmr.IsEnabled()) {
        dmaStreamSetMemory0(PDma, PBufW->data());
        dmaStreamSetTransactionSize(PDma, PBufW->size());
        dmaStreamSetMode(PDma, ADC_DMA_MODE);
        dmaStreamEnable(PDma);
    }
    // Signal event
    if(ICallback != nullptr) ICallback();
}

void Adc_t::Init(const AdcSetup_t& Setup) {
    rccEnableADC123(FALSE);      // Enable AHB clock
    // Power-on ADC, exit deep power-down mode
    ADC1->CR &= ~(ADC_CR_DEEPPWD | ADC_CR_ADCAL | ADC_CR_JADSTP | ADC_CR_ADSTP | ADC_CR_JADSTART | ADC_CR_ADSTART | ADC_CR_ADDIS | ADC_CR_ADEN);
    ADC1->CR |= ADC_CR_ADVREGEN; // Enable ADC internal voltage regulator
    chThdSleepMicroseconds(20);
    // Setup clock
    ADC123_COMMON->CCR = 0b0000UL << 18; // Prescaler = 1
    EnableVref();
    // Enable: allowed to write them only if the ADC is enabled
    SET_BIT(ADC1->ISR, ADC_ISR_ADRDY);  // Clear ADRDY bit by writing 1 to it
    SET_BIT(ADC1->CR, ADC_CR_ADEN);     // Enable ADC
    while(READ_BIT(ADC1->ISR, ADC_ISR_ADRDY) == 0);   // Let it to complete

    ADC1->CFGR = ADC_CFGR_JQDIS; // Injected Queue disable
    // Setup oversampler
    ADC1->CFGR2 = (uint32_t)Setup.Oversampling;

    // ==== Setup channels ====
    //    ADC123_COMMON->CCR |= ADC_CCR_VBATEN;   // Enable VBat channel
    uint32_t ChnlCnt = Setup.Channels.size();
    IBuf1.resize(ChnlCnt);
    IBuf2.resize(ChnlCnt);
    SetSequenceLength(ChnlCnt);
    for(uint32_t i=0; i<ChnlCnt; i++) {
        const AdcChannel_t& Chnl = Setup.Channels[i];
        PinSetupAnalog(Chnl.GPIO, Chnl.Pin);
        PinConnectAdc(Chnl.GPIO, Chnl.Pin);
        SetChannelSampleTime(Chnl.ChannelN, Setup.SampleTime);
        SetSequenceItem(i+1, Chnl.ChannelN);   // First sequence item is 1, not 0
    }
    ICallback = Setup.DoneCallback;

    // ==== DMA ====
    PDma = dmaStreamAlloc(ADC_DMA, IRQ_PRIO_MEDIUM, AdcTxIrq, nullptr);
    dmaStreamSetPeripheral(PDma, &ADC1->DR);
    dmaStreamSetMode      (PDma, ADC_DMA_MODE);
    ADC1->CFGR |= ADC_CFGR_DMACFG | ADC_CFGR_DMAEN; // DMA is in circ mode, DMA en
}

static bool IsEnabled() { return (ADC1->CR & ADC_CR_ADEN); }

static void Calibrate() {
    ADC1->CR &= ~ADC_CR_ADCALDIF;   // Calibration for single-ended inputs
    ADC1->CR |= ADC_CR_ADCAL;        // Start calibration
    while((ADC1->CR & ADC_CR_ADCAL) != 0);   // Let it to complete
    // ADEN bit cannot be set during ADCAL=1 and 4 ADC clock cycle after the ADCAL bit is cleared by hardware(end of the calibration).
    for(volatile uint32_t i=0; i<99; i++) __NOP();
}

static void StopAndDisable() {
    if(IsEnabled()) {
        SET_BIT(ADC1->CR, ADC_CR_ADSTP);    // Stop any ongoing conversion
        while(READ_BIT(ADC1->CR, ADC_CR_ADSTP) != 0);   // Let it to complete
        ADC1->CR |= ADC_CR_ADDIS; // Disable
        while(IsEnabled());   // Let it to complete
    }
}

static inline void StartConversion() {
    ADC1->CR |= ADC_CR_ADSTART;
}

void Adc_t::Deinit() {
    StopAndDisable();
    ITmr.Deinit();
    DisableVref();
    rccDisableADC123(); // Disable clock
}

void Adc_t::EnableVref()  { ADC123_COMMON->CCR |= ADC_CCR_VREFEN; }
void Adc_t::DisableVref() { ADC123_COMMON->CCR &= ADC_CCR_VREFEN; }

void Adc_t::SetSequenceLength(uint32_t ALen) {
    ADC1->SQR1 &= ~ADC_SQR1_L;  // Clear count
    ADC1->SQR1 |= (ALen - 1);   // 0000: 1 conversion; 0001: 2 conversions...
}

void Adc_t::SetChannelSampleTime(uint32_t AChnl, uint32_t ASampleTime) {
    uint32_t Offset;
    if(AChnl <= 9) { // [0; 9]
        Offset = AChnl * 3;
        uint32_t tmp = ADC1->SMPR1 & ~(0b111UL << Offset);  // Clear bits
        tmp |= ASampleTime << Offset; // Set new bits
        ADC1->SMPR1 = tmp;
    }
    else { // [10; 18]
        Offset = (AChnl - 10) * 3;
        uint32_t tmp = ADC1->SMPR2 & ~(0b111UL << Offset);  // Clear bits
        tmp |= ASampleTime << Offset; // Set new bits
        ADC1->SMPR2 = tmp;
    }
}

void Adc_t::SetSequenceItem(uint8_t SeqIndx, uint32_t AChnl) {
    uint32_t Offset;
    if(SeqIndx <= 4) {          // SQR1: [1; 4]
        Offset = SeqIndx * 6;   // 1,2,3,4 => 6,12,18,24
        ADC1->SQR1 &= ~(0b11111UL << Offset);
        ADC1->SQR1 |= (AChnl << Offset);
    }
    else if(SeqIndx <= 9) {     // SQR2: [5; 9]
        Offset = (SeqIndx - 5) * 6; // 5,6,7,8,9 => 0,6,12,18,24
        ADC1->SQR2 &= ~(0b11111UL << Offset);
        ADC1->SQR2 |= (AChnl << Offset);
    }
    else if(SeqIndx <= 14) {    // SQR3: [10; 14]
        Offset = (SeqIndx - 10) * 6;
        ADC1->SQR3 &= ~(0b11111UL << Offset);
        ADC1->SQR3 |= (AChnl << Offset);
    }
    else if(SeqIndx <= 16) {    // SQR4: 15, 16
        Offset = (SeqIndx - 15) * 6;
        ADC1->SQR4 &= ~(0b11111UL << Offset);
        ADC1->SQR4 |= (AChnl << Offset);
    }
}

// Service routine
void Adc_t::DisableCalibrateEnableSetDMA() {
    StopAndDisable();
    Calibrate();
    // Enable
    SET_BIT(ADC1->ISR, ADC_ISR_ADRDY);  // Clear ADRDY bit by writing 1 to it
    SET_BIT(ADC1->CR, ADC_CR_ADEN);     // Enable ADC
    while(READ_BIT(ADC1->ISR, ADC_ISR_ADRDY) == 0);   // Let it to complete
    // Disable continuous mode
    ADC1->CFGR &= ~ADC_CFGR_CONT;
    // Disable trigger
    ADC1->CFGR &= ~ADC_CFGR_EXTEN;
    // DMA
    dmaStreamSetMemory0(PDma, PBufW->data());
    dmaStreamSetTransactionSize(PDma, PBufW->size());
    dmaStreamSetMode(PDma, ADC_DMA_MODE);
    dmaStreamEnable(PDma);
}

// Start sequence conversion and run callback when done
void Adc_t::StartSingleMeasurement() {
    DisableCalibrateEnableSetDMA();
    StartConversion();
}

// Start periodic conversions, run callback every time when done
void Adc_t::StartPeriodicMeasurement(uint32_t FSmpHz) {
    DisableCalibrateEnableSetDMA();
    // Enable trigger
    ADC1->CFGR &= ~(0b1111UL << ADC_CFGR_EXTSEL_Pos); // Clear it
    ADC1->CFGR |=  (0b1101UL << ADC_CFGR_EXTSEL_Pos); // EXT13 = TIM6_TRGO
    ADC1->CFGR |=  (0b01UL   << ADC_CFGR_EXTEN_Pos);  // trigger detection on the rising edge
    StartConversion();
    // Setup timer
    ITmr.Init();
    ITmr.SetUpdateFrequencyChangingBoth(FSmpHz);
    ITmr.SelectMasterMode(mmUpdate);
    ITmr.Enable();
}


//uint32_t Adc_t::GetResult(uint8_t AChannel) {
//    Uart.Printf("SQR1: %X; SQR2: %X; ISR: %X\r", ADC1->SQR1, ADC1->SQR2, ADC1->ISR);
//    for(int i=0; i<ADC_SEQ_LEN; i++) Uart.Printf("%u ", IBuf[i]);
//    Uart.Printf("\r");
#if (ADC_CHANNEL_CNT > 1)
    // Find Channel indx
    for(uint32_t i=0; i < ADC_CHANNEL_CNT; i++) {
//        if(AdcChannels[i] == AChannel) return IBuf[i];
    }
#endif
//    return IBuf[0];
//}



//uint32_t Adc_t::Adc2mV(uint32_t AdcChValue, uint32_t VrefValue) {
//    return ((3000UL * ADC_VREFINT_CAL / ADC_MAX_VALUE) * AdcChValue) / VrefValue;
//}

#endif  // ADC_REQUIRED
