/*
 * adc_f2.cpp
 *
 *  Created on: 25 рту. 2013 у.
 *      Author: kreyl
 */

#include "kl_adc.h"
#include "main.h"

#if ADC_REQUIRED

Adc_t Adc;

static const uint8_t AdcChannels[] = ADC_CHANNELS;

#if defined STM32F0XX

// Wrapper for IRQ
extern "C" {
void AdcTxIrq(void *p, uint32_t flags) {
    dmaStreamDisable(ADC_DMA);
    Adc.Stop();
    // Signal event
    chSysLockFromISR();
    App.SignalEvtI(EVTMSK_ADC_DONE);
    chSysUnlockFromISR();
}
} // extern C

void Adc_t::Init() {
    rccResetADC1();
    rccEnableADC1(FALSE);           // Enable digital clock
    // Configure
    ADC1->CFGR1 = (ADC_CFGR1_CONT | ADC_CFGR1_DMAEN); // Enable Continuous mode and DMA request
    ADC1->CFGR2 = (0b01 << 30);     // Clock: PCLK/2
    // Setup channels
    ADC1->CHSELR = 0;
    for(uint8_t i=0; i < ADC_CHANNEL_CNT; i++) {
        ADC1->CHSELR |= (1 << AdcChannels[i]);
    }
    ADC1->SMPR = (uint32_t)ast55d5Cycles;       // Setup sampling time
    // Calibrate
    uint32_t cnt=0;
    ADC1->CR |= ADC_CR_ADCAL;   // Start calibration
    while(BitIsSet(ADC1->CR, ADC_CR_ADCAL)) {
        if(cnt++ >= 63000) {
            Uart.Printf("ADC calib fail\r");
            return;
        }
    }
    // Enable ADC
    ADC1->CR |= ADC_CR_ADEN;   // Enable ADC
    while(!BitIsSet(ADC1->ISR, ADC_ISR_ADRDY)); // Wait until ADC is ready
    // ==== DMA ====
    dmaStreamAllocate     (ADC_DMA, IRQ_PRIO_LOW, AdcTxIrq, NULL);
    dmaStreamSetPeripheral(ADC_DMA, &ADC1->DR);
    dmaStreamSetMode      (ADC_DMA, ADC_DMA_MODE);
//    Uart.Printf("ADC is set\r");
}

void Adc_t::StartMeasurement() {
    while(BitIsSet(ADC1->CR, ADC_CR_ADSTP));        // Wait until stop is completed
    // DMA
    dmaStreamSetMemory0(ADC_DMA, IBuf);
    dmaStreamSetTransactionSize(ADC_DMA, ADC_SEQ_LEN);
    dmaStreamSetMode(ADC_DMA, ADC_DMA_MODE);
    dmaStreamEnable(ADC_DMA);
    // ADC
    StartConversion();
}

void Adc_t ::Disable() {
    if(!BitIsSet(ADC1->CR, ADC_CR_ADSTP)) Stop();   // Stop if not stopped
    while(BitIsSet(ADC1->CR, ADC_CR_ADSTP));        // Wait until stop is completed
    ADC1->CR |= ADC_CR_ADDIS;                       // Start disabling ADC
    while(BitIsSet(ADC1->CR, ADC_CR_ADDIS));        // Wait until disabled
}

uint32_t Adc_t::GetResult(uint8_t AChannel) {
    uint32_t Indx = 0;
    uint32_t Rslt = 0;
#if (ADC_CHANNEL_CNT > 1)
    // Find Channel indx
    for(uint32_t i=0; i < ADC_CHANNEL_CNT; i++) {
        if(AdcChannels[i] == AChannel) {
            Indx = i;
            break;
        }
    }
#endif
    // Average values
    for(uint32_t i = Indx; i < ADC_SEQ_LEN; i += ADC_CHANNEL_CNT) {
        Rslt += IBuf[i];
//        Uart.Printf("%u; ", IBuf[i]);
    }
//    Uart.Printf("\r");
    return Rslt / ADC_SAMPLE_CNT;
}

uint32_t Adc_t::Adc2mV(uint32_t AdcChValue, uint32_t VrefValue) {
    return ((3300UL * ADC_VREFINT_CAL / ADC_MAX_VALUE) * AdcChValue) / VrefValue;
}

#endif // stm32f0

#if defined STM32F4XX
// Wrapper for IRQ
extern "C" {
void AdcTxIrq(void *p, uint32_t flags) {
    dmaStreamDisable(ADC_DMA);
    Adc.Disable();
    // Signal event
    chSysLockFromISR();
    App.SignalEvtI(EVTMSK_ADC_DONE);
    chSysUnlockFromISR();
}
} // extern C

void Adc_t::Init() {
    rccEnableADC1(FALSE);   	// Enable digital clock
    SetupClk(ADC_CLK_DIVIDER);  // Setup ADCCLK
    // Setup channels
    SetSequenceLength(ADC_SEQ_LEN);
    uint8_t SeqIndx = 1;    // First sequence item is 1, not 0
    for(uint8_t i=0; i < ADC_CHANNEL_CNT; i++) {
		SetChannelSampleTime(AdcChannels[i], ADC_SAMPLE_TIME);
		for(uint8_t j=0; j<ADC_SAMPLE_CNT; j++) SetSequenceItem(SeqIndx++, AdcChannels[i]);
	}
    // ==== DMA ====
    dmaStreamAllocate     (ADC_DMA, IRQ_PRIO_LOW, AdcTxIrq, NULL);
    dmaStreamSetPeripheral(ADC_DMA, &ADC1->DR);
    dmaStreamSetMode      (ADC_DMA, ADC_DMA_MODE);
}

void Adc_t::SetSequenceLength(uint32_t ALen) {
    ADC1->SQR1 &= ~ADC_SQR1_L;  // Clear count
    ADC1->SQR1 |= (ALen - 1) << 20;
}
void Adc_t::SetChannelSampleTime(uint32_t AChnl, AdcSampleTime_t ASampleTime) {
    uint32_t Offset;
    if(AChnl <= 9) {
        Offset = AChnl * 3;
        ADC1->SMPR2 &= ~((uint32_t)0b111 << Offset);    // Clear bits
        ADC1->SMPR2 |= (uint32_t)ASampleTime << Offset; // Set new bits
    }
    else {
        Offset = (AChnl - 10) * 3;
        ADC1->SMPR1 &= ~((uint32_t)0b111 << Offset);    // Clear bits
        ADC1->SMPR1 |= (uint32_t)ASampleTime << Offset; // Set new bits
    }
}
void Adc_t::SetSequenceItem(uint8_t SeqIndx, uint32_t AChnl) {
    uint32_t Offset;
    if(SeqIndx <= 6) {
        Offset = (SeqIndx - 1) * 5;
        ADC1->SQR3 &= ~(uint32_t)(0b11111 << Offset);
        ADC1->SQR3 |= (uint32_t)(AChnl << Offset);
    }
    else if(SeqIndx <= 12) {
        Offset = (SeqIndx - 7) * 5;
        ADC1->SQR2 &= ~(uint32_t)(0b11111 << Offset);
        ADC1->SQR2 |= (uint32_t)(AChnl << Offset);
    }
    else if(SeqIndx <= 16) {
        Offset = (SeqIndx - 13) * 5;
        ADC1->SQR1 &= ~(uint32_t)(0b11111 << Offset);
        ADC1->SQR1 |= (uint32_t)(AChnl << Offset);
    }
}

void Adc_t::StartMeasurement() {
    // DMA
    dmaStreamSetMemory0(ADC_DMA, IBuf);
    dmaStreamSetTransactionSize(ADC_DMA, ADC_SEQ_LEN);
    dmaStreamSetMode(ADC_DMA, ADC_DMA_MODE);
    dmaStreamEnable(ADC_DMA);
    // ADC
    ADC1->CR1 = ADC_CR1_SCAN;
    ADC1->CR2 = ADC_CR2_DMA | ADC_CR2_ADON;
    StartConversion();
}

uint32_t Adc_t::GetResult(uint8_t AChannel) {
    uint32_t Indx = 0;
#if (ADC_CHANNEL_CNT > 1)
    // Find Channel indx
    for(uint32_t i=0; i < ADC_CHANNEL_CNT; i++) {
        if(AdcChannels[i] == AChannel) {
            Indx = i;
            break;
        }
    }
#endif
    // Find bounds
    uint32_t Start = Indx * ADC_SAMPLE_CNT;
    uint32_t Stop  = Start + ADC_SAMPLE_CNT;
    // Average values
    uint32_t Rslt = 0;
    for(uint32_t i = Start; i < Stop; i++) Rslt += IBuf[i];
    return Rslt / ADC_SAMPLE_CNT;
}
#endif // f4xx

#endif  // ADC_REQUIRED
