/*
 * adc_f2.cpp
 *
 *  Created on: 25 ���. 2013 �.
 *      Author: kreyl
 */

#include <adcL151.h>
#include "board.h"
#include "MsgQ.h"
#include "shell.h"

#if ADC_REQUIRED

Adc_t Adc;
const uint8_t AdcChannels[ADC_CHANNEL_CNT] = ADC_CHANNELS;
static thread_reference_t ThdRef;
static bool FirstConversion;
static const stm32_dma_stream_t *PAdcDma = nullptr;


static THD_WORKING_AREA(waAdcThread, 128);
__noreturn
static void AdcThread(void *arg) {
    chRegSetThreadName("Adc");
    while(true) {
        chThdSleepMilliseconds(ADC_MEAS_PERIOD_MS);
        chSysLock();
        Adc.StartMeasurement();
        chThdSuspendS(&ThdRef);
        chSysUnlock();
        // Will be here after measurements done
//        Printf("AdcDone\r");
        if(FirstConversion) FirstConversion = false;
        else {
//            uint32_t VRef_adc = Adc.GetResult(ADC_VREFINT_CHNL);
//            Printf("VRef_adc=%u\r", VRef_adc);
            // Iterate all channels
            for(int i=0; i<ADC_CHANNEL_CNT; i++) {
                if(AdcChannels[i] == ADC_VREFINT_CHNL) continue; // Ignore VrefInt channel
                uint32_t Vadc = Adc.GetResult(AdcChannels[i]);
//                uint32_t Vmv = Adc.Adc2mV(Vadc, VRef_adc);
                uint32_t Vmv = (Vadc * 3300UL) / 4095UL;
//                Printf("N=%u; Vadc=%u; Vmv=%u\r", i, Vadc, Vmv);
                EvtQMain.SendNowOrExit(EvtMsg_t(evtIdAdcRslt, AdcChannels[i], Vmv));
            } // for
        } // not first conv
    } // while true
}

// Wrapper for IRQ
extern "C" {
void AdcTxIrq(void *p, uint32_t flags) {
    dmaStreamDisable(PAdcDma);
    Adc.Disable();
    // Wake thread
    chSysLockFromISR();
    chThdResumeI(&ThdRef, MSG_OK);
    chSysUnlockFromISR();
}
} // extern C

#if defined STM32L1XX
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
    PAdcDma = dmaStreamAlloc(ADC_DMA, IRQ_PRIO_LOW, AdcTxIrq, NULL);
    dmaStreamSetPeripheral(PAdcDma, &ADC1->DR);
    dmaStreamSetMode      (PAdcDma, ADC_DMA_MODE);
    // ==== Thread ====
    chThdCreateStatic(waAdcThread, sizeof(waAdcThread), NORMALPRIO, (tfunc_t)AdcThread, NULL);
}

void Adc_t::SetSequenceLength(uint32_t ALen) {
    ADC1->SQR1 &= ~ADC_SQR1_L;  // Clear count
    ADC1->SQR1 |= (ALen - 1) << 20;
}

void Adc_t::SetChannelSampleTime(uint32_t AChnl, AdcSampleTime_t ASampleTime) {
    uint32_t Offset;
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
}
void Adc_t::SetSequenceItem(uint8_t SeqIndx, uint32_t AChnl) {
    uint32_t Offset;
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
}

void Adc_t::StartMeasurement() {
    // DMA
    dmaStreamSetMemory0(PAdcDma, IBuf);
    dmaStreamSetTransactionSize(PAdcDma, ADC_SEQ_LEN);
    dmaStreamSetMode(PAdcDma, ADC_DMA_MODE);
    dmaStreamEnable(PAdcDma);
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

#endif  // ADC_REQUIRED
