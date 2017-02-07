/*
 * adc_f2.cpp
 *
 *  Created on: 25 рту. 2013 у.
 *      Author: kreyl
 */

#include "kl_adc.h"
#include "main.h"
#include "board.h"

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
    App.SignalEvtI(EVT_ADC_DONE);
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
#endif // stm32f0

#if defined STM32F4XX || defined STM32L1XX
// Wrapper for IRQ
extern "C" {
void AdcTxIrq(void *p, uint32_t flags) {
    dmaStreamDisable(ADC_DMA);
    Adc.Disable();
    // Signal event
    chSysLockFromISR();
    App.SignalEvtI(EVT_ADC_DONE);
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
#if defined STM32F4XX
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
#elif defined STM32L1XX
    if(AChnl <= 9) {
        Offset = AChnl * 3;
        ADC1->SMPR3 &= ~((uint32_t)0b111 << Offset);    // Clear bits
        ADC1->SMPR3 |= (uint32_t)ASampleTime << Offset; // Set new bits
    }
    else if(AChnl <= 19) {
        Offset = (AChnl - 10) * 3;
        ADC1->SMPR2 &= ~((uint32_t)0b111 << Offset);    // Clear bits
        ADC1->SMPR2 |= (uint32_t)ASampleTime << Offset; // Set new bits
    }
    else {
        Offset = (AChnl - 20) * 3;
        ADC1->SMPR1 &= ~((uint32_t)0b111 << Offset);    // Clear bits
        ADC1->SMPR1 |= (uint32_t)ASampleTime << Offset; // Set new bits
    }
#endif
}
void Adc_t::SetSequenceItem(uint8_t SeqIndx, uint32_t AChnl) {
    uint32_t Offset;
#if defined STM32F4XX
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
#elif defined STM32L1XX
    if(SeqIndx <= 6) {
        Offset = (SeqIndx - 1) * 5;
        ADC1->SQR5 &= ~(uint32_t)(0b11111 << Offset);
        ADC1->SQR5 |= (uint32_t)(AChnl << Offset);
    }
    else if(SeqIndx <= 12) {
        Offset = (SeqIndx - 7) * 5;
        ADC1->SQR4 &= ~(uint32_t)(0b11111 << Offset);
        ADC1->SQR4 |= (uint32_t)(AChnl << Offset);
    }
    else if(SeqIndx <= 18) {
        Offset = (SeqIndx - 13) * 5;
        ADC1->SQR3 &= ~(uint32_t)(0b11111 << Offset);
        ADC1->SQR3 |= (uint32_t)(AChnl << Offset);
    }
    else if(SeqIndx <= 24) {
        Offset = (SeqIndx - 19) * 5;
        ADC1->SQR2 &= ~(uint32_t)(0b11111 << Offset);
        ADC1->SQR2 |= (uint32_t)(AChnl << Offset);
    }
    else if(SeqIndx <= 28) {    // 28 in high and medium density, 27 in others
        Offset = (SeqIndx - 25) * 5;
        ADC1->SQR1 &= ~(uint32_t)(0b11111 << Offset);
        ADC1->SQR1 |= (uint32_t)(AChnl << Offset);
    }
#endif
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
#endif // f4xx & L151

#if defined STM32L4XX
// Wrapper for IRQ
extern "C" {
void AdcTxIrq(void *p, uint32_t flags) {
    dmaStreamDisable(ADC_DMA);
    Adc.Disable();
    // Signal event
    chSysLockFromISR();
    App.SignalEvtI(EVT_ADC_DONE);
    chSysUnlockFromISR();
}
} // extern C

void Adc_t::Init() {
    rccEnableADC123(FALSE);       // Enable AHB clock
    // Setup ADC clock: PLLSAI1 "R" selected as ADC clk
    MODIFY_REG(RCC->CCIPR, RCC_CCIPR_ADCSEL, RCC_CCIPR_ADCSEL_0);
    // Setup PLL
    if(Clk.SetupPllSai1(16, 8) != OK) return;
    Clk.EnableSai1ROut();
    // Power-on ADC
    CLEAR_BIT(ADC1->CR, ADC_CR_DEEPPWD);    // Exit deep power-down mode
    SET_BIT(ADC1->CR, ADC_CR_ADVREGEN);     // Enable ADC internal voltage regulator
    chThdSleepMicroseconds(20);
    // Setup channels
    SetSequenceLength(ADC_SEQ_LEN);
    for(uint8_t i=0; i < ADC_CHANNEL_CNT; i++) {
        SetChannelSampleTime(AdcChannels[i], ADC_SAMPLE_TIME);
        SetSequenceItem(i+1, AdcChannels[i]);   // First sequence item is 1, not 0
    }
    // ==== DMA ====
    dmaStreamAllocate     (ADC_DMA, IRQ_PRIO_LOW, AdcTxIrq, NULL);
    dmaStreamSetPeripheral(ADC_DMA, &ADC1->DR);
    dmaStreamSetMode      (ADC_DMA, ADC_DMA_MODE);
}

void Adc_t::Calibrate() {
    CLEAR_BIT(ADC1->CR, ADC_CR_ADCALDIF);   // Calibration for single-ended inputs
    SET_BIT(ADC1->CR, ADC_CR_ADCAL);        // Start calibration
    while(READ_BIT(ADC1->CR, ADC_CR_ADCAL) != 0);   // Let it to complete
}

void Adc_t::SetSequenceLength(uint32_t ALen) {
    ADC1->SQR1 &= ~ADC_SQR1_L;  // Clear count
    ADC1->SQR1 |= (ALen - 1);   // 0000: 1 conversion; 0001: 2 conversions...
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
    if(SeqIndx <= 4) {          // SQR1: 1...4
        Offset = (SeqIndx - 1) * 6 + 6;
        ADC1->SQR1 &= ~(uint32_t)(0b11111 << Offset);
        ADC1->SQR1 |= (uint32_t)(AChnl << Offset);
    }
    else if(SeqIndx <= 9) {     // SQR2: 5...9
        Offset = (SeqIndx - 5) * 6;
        ADC1->SQR2 &= ~(uint32_t)(0b11111 << Offset);
        ADC1->SQR2 |= (uint32_t)(AChnl << Offset);
    }
    else if(SeqIndx <= 14) {    // SQR3: 10...14
        Offset = (SeqIndx - 10) * 6;
        ADC1->SQR3 &= ~(uint32_t)(0b11111 << Offset);
        ADC1->SQR3 |= (uint32_t)(AChnl << Offset);
    }
    else if(SeqIndx <= 16) {    // SQR4: 15, 16
        Offset = (SeqIndx - 15) * 6;
        ADC1->SQR4 &= ~(uint32_t)(0b11111 << Offset);
        ADC1->SQR4 |= (uint32_t)(AChnl << Offset);
    }
}

void Adc_t::StartMeasurement() {
    // Stop ADC
    if(IsEnabled()) {
        SET_BIT(ADC1->CR, ADC_CR_ADSTP);    // Stop any ongoing conversion
        while(READ_BIT(ADC1->CR, ADC_CR_ADSTP) != 0);   // Let it to complete
        Disable();
        while(IsEnabled());   // Let it to complete
    }
    Calibrate();
    __NOP(); __NOP(); __NOP(); __NOP(); // ADEN bit cannot be set during ADCAL=1 and 4 ADC clock cycle after the ADCAL bit is cleared by hardware(end of the calibration).
    // Start ADC
    SET_BIT(ADC1->ISR, ADC_ISR_ADRDY);  // Clear ADRDY bit by writing 1 to it
    SET_BIT(ADC1->CR, ADC_CR_ADEN);     // Enable ADC
    while(READ_BIT(ADC1->ISR, ADC_ISR_ADRDY) == 0);   // Let it to complete
    // Setup oversampler
#if ADC_OVERSAMPLING_RATIO == 1
    ADC1->CFGR2 = 0;    // Oversampler disabled
#elif ADC_OVERSAMPLING_RATIO == 2
    ADC1->CFGR2 = (0b0001 << 5) | (0b000 << 2) | ADC_CFGR2_ROVSE;
#elif ADC_OVERSAMPLING_RATIO == 4
    ADC1->CFGR2 = (0b0010 << 5) | (0b001 << 2) | ADC_CFGR2_ROVSE;
#elif ADC_OVERSAMPLING_RATIO == 8
    ADC1->CFGR2 = (0b0011 << 5) | (0b010 << 2) | ADC_CFGR2_ROVSE;
#elif ADC_OVERSAMPLING_RATIO == 16
    ADC1->CFGR2 = (0b0100 << 5) | (0b011 << 2) | ADC_CFGR2_ROVSE;
#elif ADC_OVERSAMPLING_RATIO == 32
    ADC1->CFGR2 = (0b0101 << 5) | (0b100 << 2) | ADC_CFGR2_ROVSE;
#elif ADC_OVERSAMPLING_RATIO == 64
    ADC1->CFGR2 = (0b0110 << 5) | (0b101 << 2) | ADC_CFGR2_ROVSE;
#elif ADC_OVERSAMPLING_RATIO == 128
    ADC1->CFGR2 = (0b0111 << 5) | (0b110 << 2) | ADC_CFGR2_ROVSE;
#elif ADC_OVERSAMPLING_RATIO == 256
    ADC1->CFGR2 = (0b1000 << 5) | (0b111 << 2) | ADC_CFGR2_ROVSE;
#endif
    // Setup ADC. Do not set OVRMOD bit as it breaks sequence in case of DMA
    SET_BIT(ADC1->CFGR, ADC_CFGR_DMAEN);    // Enable DMA
    // DMA
    dmaStreamSetMemory0(ADC_DMA, IBuf);
    dmaStreamSetTransactionSize(ADC_DMA, ADC_SEQ_LEN);
    dmaStreamSetMode(ADC_DMA, ADC_DMA_MODE);
    dmaStreamEnable(ADC_DMA);
    // ADC
    StartConversion();
}

uint32_t Adc_t::GetResult(uint8_t AChannel) {
//    Uart.Printf("SQR1: %X; SQR2: %X; ISR: %X\r", ADC1->SQR1, ADC1->SQR2, ADC1->ISR);
//    for(int i=0; i<ADC_SEQ_LEN; i++) Uart.Printf("%u ", IBuf[i]);
//    Uart.Printf("\r");
#if (ADC_CHANNEL_CNT > 1)
    // Find Channel indx
    for(uint32_t i=0; i < ADC_CHANNEL_CNT; i++) {
        if(AdcChannels[i] == AChannel) return IBuf[i];
    }
#endif
    return IBuf[0];
}

uint32_t Adc_t::Adc2mV(uint32_t AdcChValue, uint32_t VrefValue) {
    return ((3000UL * ADC_VREFINT_CAL / ADC_MAX_VALUE) * AdcChValue) / VrefValue;
}
#endif // L476

#endif  // ADC_REQUIRED
