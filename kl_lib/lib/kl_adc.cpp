/*
 * adc_f2.cpp
 *
 *  Created on: 25 рту. 2013 у.
 *      Author: kreyl
 */

#include "kl_adc.h"
#include "board.h"
#include "MsgQ.h"
#include "shell.h"

#if ADC_REQUIRED

Adc_t Adc;
//const uint8_t AdcChannels[ADC_CHANNEL_CNT] = ADC_CHANNELS;
//static thread_reference_t ThdRef;
//static bool FirstConversion;

//static THD_WORKING_AREA(waAdcThread, 128);
//__noreturn
//static void AdcThread(void *arg) {
//    chRegSetThreadName("Adc");
//    while(true) {
////        chThdSleepMilliseconds(ADC_MEAS_PERIOD_MS);
//        chSysLock();
//        Adc.StartMeasurement();
//        chThdSuspendS(&ThdRef);
//        chSysUnlock();
//        // Will be here after measurements done
////        Printf("AdcDone\r");
//        if(FirstConversion) FirstConversion = false;
//        else {
////            uint32_t VRef_adc = Adc.GetResult(ADC_VREFINT_CHNL);
////            Printf("VRef_adc=%u\r", VRef_adc);
//            // Iterate all channels
//            for(int i=0; i<ADC_CHANNEL_CNT; i++) {
//                if(AdcChannels[i] == ADC_VREFINT_CHNL) continue; // Ignore VrefInt channel
////                uint32_t Vadc = Adc.GetResult(AdcChannels[i]);
////                uint32_t Vmv = Adc.Adc2mV(Vadc, VRef_adc);   // Resistor divider
////                Printf("N=%u; Vadc=%u; Vmv=%u\r", i, Vadc, Vmv);
////                EvtMsg_t Msg(evtIdAdcRslt, AdcChannels[i], Vmv);
////                EvtQMain.SendNowOrExit(Msg);
//            } // for
//        } // not first conv
//    } // while true
//}

// Wrapper for IRQ
//extern "C"
//void AdcTxIrq(void *p, uint32_t flags) {
//    dmaStreamDisable(ADC_DMA);
//    Adc.Disable();
//    // Wake thread
//    chSysLockFromISR();
//    chThdResumeI(&ThdRef, MSG_OK);
//    chSysUnlockFromISR();
//}

#if defined STM32F0XX
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

#if defined STM32F2XX || defined STM32F4XX || defined STM32L1XX
void Adc_t::Init() {
    FirstConversion = true;
    rccEnableADC1(FALSE);   	// Enable digital clock
    SetupClk(ADC_CLK_DIVIDER);  // Setup ADCCLK
    // Setup channels
    SetSequenceLength(ADC_SEQ_LEN);
    uint8_t SeqIndx = 1;    // First sequence item is 1, not 0
    for(uint8_t i=0; i < ADC_CHANNEL_CNT; i++) {
		SetChannelSampleTime(AdcChannels[i], ADC_SAMPLE_TIME_DEFAULT);
		for(uint8_t j=0; j<ADC_SAMPLE_CNT; j++) SetSequenceItem(SeqIndx++, AdcChannels[i]);
	}
    EnableVRef();
    // ==== DMA ====
    dmaStreamAllocate     (ADC_DMA, IRQ_PRIO_LOW, AdcTxIrq, NULL);
    dmaStreamSetPeripheral(ADC_DMA, &ADC1->DR);
    dmaStreamSetMode      (ADC_DMA, ADC_DMA_MODE);
    // ==== Thread ====
    chThdCreateStatic(waAdcThread, sizeof(waAdcThread), NORMALPRIO, (tfunc_t)AdcThread, NULL);
}

void Adc_t::SetSequenceLength(uint32_t ALen) {
    ADC1->SQR1 &= ~ADC_SQR1_L;  // Clear count
    ADC1->SQR1 |= (ALen - 1) << 20;
}

void Adc_t::SetChannelSampleTime(uint32_t AChnl, AdcSampleTime_t ASampleTime) {
    uint32_t Offset;
#if defined STM32F2XX || defined STM32F4XX
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
#if defined STM32F2XX || defined STM32F4XX
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
    ADC1->CR1 = ADC_CR1_SCAN;               // Mode = scan
    ADC1->CR2 = ADC_CR2_DMA | ADC_CR2_ADON; // Enable DMA, enable ADC
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
extern "C"
void AdcTxIrq(void *p, uint32_t flags) { Adc.OnDmaIrq(); }

void Adc_t::OnDmaIrq() {
    dmaStreamDisable(PDma);
    Disable();
    // Signal event
    chSysLockFromISR();
    chThdResumeI(&ThdRef, MSG_OK);
    chSysUnlockFromISR();
}

void Adc_t::Init() {
    rccEnableADC123(FALSE);      // Enable AHB clock
    // Power-on ADC, exit deep power-down mode
    ADC1->CR &= ~(ADC_CR_DEEPPWD | ADC_CR_ADCAL | ADC_CR_JADSTP | ADC_CR_ADSTP | ADC_CR_JADSTART | ADC_CR_ADSTART | ADC_CR_ADDIS | ADC_CR_ADEN);
    ADC1->CR |= ADC_CR_ADVREGEN; // Enable ADC internal voltage regulator
    chThdSleepMicroseconds(20);
    // Setup clock
    ADC123_COMMON->CCR = 0b0000UL << 18; // Prescaler = 1
    EnableVref();
    ADC1->CFGR = ADC_CFGR_JQDIS | ADC_CFGR_OVRMOD; // Overwrite DR even if old value was not read
    ADC123_COMMON->CCR |= ADC_CCR_VBATEN;
    // Setup channels
//    SetSequenceLength(ADC_SEQ_LEN);
//    for(uint8_t i=0; i < ADC_CHANNEL_CNT; i++) {
//        SetChannelSampleTime(AdcChannels[i], ADC_SAMPLE_TIME);
//        SetSequenceItem(i+1, AdcChannels[i]);   // First sequence item is 1, not 0
//    }
    // Setup oversampler: measure 8 times, divide by 8, en oversampler
    ADC1->CFGR2 = (0b0011UL << 5) | (0b010UL << 2) | ADC_CFGR2_ROVSE;

    // ==== DMA ====
    PDma = dmaStreamAlloc(ADC_DMA, IRQ_PRIO_MEDIUM, AdcTxIrq, nullptr);
    dmaStreamSetPeripheral(PDma, &ADC1->DR);
    dmaStreamSetMode      (PDma, ADC_DMA_MODE);
    ADC1->CFGR |= ADC_CFGR_DMAEN;    // Enable DMA
}

void Adc_t::Calibrate() {
    ADC1->CR &= ~ADC_CR_ADCALDIF;   // Calibration for single-ended inputs
    ADC1->CR |= ADC_CR_ADCAL;        // Start calibration
    while((ADC1->CR & ADC_CR_ADCAL) != 0);   // Let it to complete
    __NOP(); __NOP(); __NOP(); __NOP(); // ADEN bit cannot be set during ADCAL=1 and 4 ADC clock cycle after the ADCAL bit is cleared by hardware(end of the calibration).
}

void Adc_t::SetSequenceLength(uint32_t ALen) {
    ADC1->SQR1 &= ~ADC_SQR1_L;  // Clear count
    ADC1->SQR1 |= (ALen - 1);   // 0000: 1 conversion; 0001: 2 conversions...
}

void Adc_t::SetChannelSampleTime(uint32_t AChnl, AdcSampleTime_t ASampleTime) {
    uint32_t Offset;
    if(AChnl <= 9) { // 0...9
        Offset = AChnl * 3;
        ADC1->SMPR1 &= ~(0b111UL << Offset);    // Clear bits
        ADC1->SMPR1 |= (uint32_t)ASampleTime << Offset; // Set new bits
    }
    else {
        Offset = (AChnl - 10) * 3;
        ADC1->SMPR2 &= ~(0b111UL << Offset);    // Clear bits
        ADC1->SMPR2 |= (uint32_t)ASampleTime << Offset; // Set new bits
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
//    dmaStreamSetMemory0(ADC_DMA, IBuf);
//    dmaStreamSetTransactionSize(ADC_DMA, ADC_SEQ_LEN);
//    dmaStreamSetMode(ADC_DMA, ADC_DMA_MODE);
//    dmaStreamEnable(ADC_DMA);
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

void Adc_t::StopCalibrateEnable() {
    // Stop ADC
    if(IsEnabled()) {
        SET_BIT(ADC1->CR, ADC_CR_ADSTP);    // Stop any ongoing conversion
        while(READ_BIT(ADC1->CR, ADC_CR_ADSTP) != 0);   // Let it to complete
        Disable();
        while(IsEnabled());   // Let it to complete
    }
    Calibrate();
    // Clear flags by writing 1 to it
    ADC1->ISR |= ADC_ISR_ADRDY | ADC_ISR_EOC | ADC_ISR_EOS;
    ADC1->CR  |= ADC_CR_ADEN;    // Enable ADC
    while((ADC1->ISR & ADC_ISR_ADRDY) == 0);   // Let it wake
}

uint16_t Adc_t::MeasureOnceSync(uint32_t AChnl) {
    StopCalibrateEnable();
    // Setup channel (allowed only when ADC is enabled)
    SetSequenceLength(2); // Channel of interest and Vrefint
    SetChannelSampleTime(ADC_VREFINT_CHNL, ast12d5Cycles);
    SetChannelSampleTime(AChnl, ast12d5Cycles);
    SetSequenceItem(1, ADC_VREFINT_CHNL);
    SetSequenceItem(2, AChnl);
    // Setup DMA
    dmaStreamSetMemory0(PDma, IBuf);
    dmaStreamSetTransactionSize(PDma, 2);
    dmaStreamSetMode(PDma, ADC_DMA_MODE);
    dmaStreamEnable(PDma);
    // Start
    chSysLock();
    StartConversion();
    chThdSuspendS(&ThdRef);
    chSysUnlock();
//    while((ADC1->ISR & ADC_ISR_EOS) == 0);

    Printf("%u; %u\r", IBuf[0], IBuf[1]);
//    Disable();
    return Adc2mV(IBuf[1], IBuf[0]);
}

uint32_t Adc_t::Adc2mV(uint32_t AdcChValue, uint32_t VrefValue) {
    return ((3000UL * ADC_VREFINT_CAL / ADC_MAX_VALUE) * AdcChValue) / VrefValue;
}
#endif // L476

#endif  // ADC_REQUIRED
