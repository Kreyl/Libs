/*
 * adc_f2.cpp
 *
 *  Created on: 2013
 *      Author: kreyl
 */

#include <adcL151.h>
#include "board.h"
#include "MsgQ.h"
#include "shell.h"

#if ADC_REQUIRED

#define ADC_MAX_SEQ_LEN     27  // 1...27; Const, see ref man p.301

#if (ADC_SEQ_LEN > ADC_MAX_SEQ_LEN) || (ADC_SEQ_LEN == 0)
#error "Wrong ADC channel count and sample count"
#endif

inline constexpr const uint32_t kAdcMaxValue = 4096UL; // const: 2^12

enum ADCDiv {
    adcDiv1 = (uint32_t)(0b00 << 16),
    adcDiv2 = (uint32_t)(0b01 << 16),
    adcDiv4 = (uint32_t)(0b10 << 16),
};

// See datasheet, search VREFINT_CAL: Raw data acquired at temperature of 30 °C ±5 °C, VDDA= 3 V ±10 mV
static inline uint32_t GetVRefIntCal() {
    return static_cast<uint32_t>(*reinterpret_cast<volatile uint16_t*>(0x1FF80078UL));
}
inline constexpr const uint32_t kAdcVRefIntCalVoltage = 3000UL;

static uint16_t ibuf[ADC_SEQ_LEN];

static void SetupClk(ADCDiv div) {
    ADC->CCR = (ADC->CCR & ~ADC_CCR_ADCPRE) | (uint32_t)div;
}

static void SetSequenceLength(uint32_t alen) {
    ADC1->SQR1 &= ~ADC_SQR1_L;  // Clear count
    ADC1->SQR1 |= (alen - 1) << 20;
}

static void SetChannelSampleTime(uint32_t channel, Adc::SampleTime sample_time) {
    uint32_t offset, sample_time32 = static_cast<uint32_t>(sample_time);
    if(channel <= 9UL) {
        offset = channel * 3UL;
        ADC1->SMPR3 &= ~(0b111UL << offset);    // Clear bits
        ADC1->SMPR3 |= sample_time32 << offset; // Set new bits
    }
    else if(channel <= 19UL) {
        offset = (channel - 10UL) * 3UL;
        ADC1->SMPR2 &= ~(0b111UL << offset);    // Clear bits
        ADC1->SMPR2 |= sample_time32 << offset; // Set new bits
    }
    else {
        offset = (channel - 20UL) * 3UL;
        ADC1->SMPR1 &= ~(0b111UL << offset);    // Clear bits
        ADC1->SMPR1 |= sample_time32 << offset; // Set new bits
    }
}

static void SetSequenceItem(uint32_t seq_indx, uint32_t channel) {
    uint32_t offset;
    if(seq_indx <= 6UL) {
        offset = (seq_indx - 1UL) * 5UL;
        ADC1->SQR5 &= ~(0b11111UL << offset);
        ADC1->SQR5 |= channel << offset;
    }
    else if(seq_indx <= 12UL) {
        offset = (seq_indx - 7UL) * 5UL;
        ADC1->SQR4 &= ~(0b11111UL << offset);
        ADC1->SQR4 |= channel << offset;
    }
    else if(seq_indx <= 18UL) {
        offset = (seq_indx - 13UL) * 5UL;
        ADC1->SQR3 &= ~(0b11111UL << offset);
        ADC1->SQR3 |= channel << offset;
    }
    else if(seq_indx <= 24UL) {
        offset = (seq_indx - 19UL) * 5UL;
        ADC1->SQR2 &= ~(0b11111UL << offset);
        ADC1->SQR2 |= channel << offset;
    }
    else if(seq_indx <= 28UL) {    // 28 in high and medium density, 27 in others
        offset = (seq_indx - 25UL) * 5UL;
        ADC1->SQR1 &= ~(0b11111UL << offset);
        ADC1->SQR1 |= channel << offset;
    }
}


namespace Adc {

inline constexpr const SampleTime kAdcSampleTimeDefault = ast96Cycles;

const uint8_t kAdcChannels[ADC_CHANNEL_CNT] = ADC_CHANNELS;
static const stm32_dma_stream_t *pdma = nullptr;
#if defined ADC_MODE_PERIODIC_MEASUREMENT || defined  ADC_MODE_SYNC_MEASUREMENT
static thread_reference_t ThdRef;
#endif

#ifdef ADC_MODE_PERIODIC_MEASUREMENT
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
                if(kAdcChannels[i] == ADC_VREFINT_CHNL) continue; // Ignore VrefInt channel
                uint32_t Vadc = Adc.GetResult(kAdcChannels[i]);
//                uint32_t Vmv = Adc.Adc2mV(Vadc, VRef_adc);
                uint32_t Vmv = (Vadc * 3300UL) / 4095UL;
//                Printf("N=%u; Vadc=%u; Vmv=%u\r", i, Vadc, Vmv);
                EvtQMain.SendNowOrExit(EvtMsg_t(evtIdAdcRslt, kAdcChannels[i], Vmv));
            } // for
        } // not first conv
    } // while true
}
#endif

static void Disable() { ADC1->CR2 = 0; }
static void EnableVRef()  { ADC->CCR |= (uint32_t)ADC_CCR_TSVREFE; }
__attribute__((unused))
static void DisableVRef() { ADC->CCR &= (uint32_t)(~ADC_CCR_TSVREFE); }
static void StartConversion() { ADC1->CR2 |= ADC_CR2_SWSTART; }
__attribute__((unused))
static void WaitConversionCompletion() { while(!(ADC1->SR & ADC_SR_EOC)); }
__attribute__((unused))
void ClockOff() { rccDisableADC1(); }

// ==== DMA completed IRQ Handler ====
void AdcRdyIrq(void *p, uint32_t flags) {
    dmaStreamDisable(pdma);
    Disable();
#ifdef ADC_EN_AND_DIS_HSI
    Clk.DisableHSI();
#endif
    chSysLockFromISR();
#if defined ADC_MODE_PERIODIC_MEASUREMENT || defined ADC_MODE_SYNC_MEASUREMENT
    chThdResumeI(&ThdRef, MSG_OK); // Wake thread
#elif defined ADC_MODE_MEASURE_BY_REQUEST
    evt_q_main.SendNowOrExitI(EvtMsg_t(EvtId::AdcRslt));
#endif
    chSysUnlockFromISR();
}


void Init() {
    rccEnableADC1(FALSE);   	// Enable digital clock
    SetupClk(ADC_CLK_DIVIDER);  // Setup ADCCLK
    // Setup channels
    SetSequenceLength(ADC_SEQ_LEN);
    uint8_t seq_indx = 1;    // First sequence item is 1, not 0
    for(uint32_t i=0; i < ADC_CHANNEL_CNT; i++) {
		SetChannelSampleTime(kAdcChannels[i], kAdcSampleTimeDefault);
		for(uint32_t j=0; j<ADC_SAMPLE_CNT; j++)
            SetSequenceItem(seq_indx++, kAdcChannels[i]);
	}
    EnableVRef();
    // ==== DMA ====
    pdma = dmaStreamAlloc(ADC_DMA, IRQ_PRIO_LOW, AdcRdyIrq, NULL);
    dmaStreamSetPeripheral(pdma, &ADC1->DR);
    dmaStreamSetMode      (pdma, ADC_DMA_MODE);
#ifdef ADC_MODE_PERIODIC_MEASUREMENT // Thread
    chThdCreateStatic(waAdcThread, sizeof(waAdcThread), NORMALPRIO, (tfunc_t)AdcThread, NULL);
#endif
}

void StartMeasurement() {
#ifdef ADC_EN_AND_DIS_HSI
    Clk.EnableHSI();
#endif
    // DMA
    dmaStreamSetMemory0(pdma, ibuf);
    dmaStreamSetTransactionSize(pdma, ADC_SEQ_LEN);
    dmaStreamSetMode(pdma, ADC_DMA_MODE);
    dmaStreamEnable(pdma);
    // ADC
    ADC1->CR1 = ADC_CR1_SCAN;               // Mode = scan
    ADC1->CR2 = ADC_CR2_DMA | ADC_CR2_ADON; // Enable DMA, enable ADC
    StartConversion();
}

#if defined ADC_MODE_PERIODIC_MEASUREMENT || defined ADC_MODE_SYNC_MEASUREMENT
void Adc_t::StartMeasurementAndWaitCompletion() {
#ifdef ADC_EN_AND_DIS_HSI
    Clk.EnableHSI();
#endif
    // DMA
    dmaStreamSetMemory0(pdma, ibuf);
    dmaStreamSetTransactionSize(pdma, ADC_SEQ_LEN);
    dmaStreamSetMode(pdma, ADC_DMA_MODE);
    dmaStreamEnable(pdma);
    // ADC
    ADC1->CR1 = ADC_CR1_SCAN;               // Mode = scan
    ADC1->CR2 = ADC_CR2_DMA | ADC_CR2_ADON; // Enable DMA, enable ADC
    chSysLock();
    ThdRef = chThdGetSelfX();
    StartConversion();
    chThdSleepS(TIME_INFINITE); // Will be waken by IRQ
    chSysUnlock();
}
#endif

uint32_t GetResultAverage(uint8_t channel) {
    uint32_t indx = 0;
#if (ADC_CHANNEL_CNT > 1)
    // Find Channel indx
    for(uint32_t i=0; i < ADC_CHANNEL_CNT; i++) {
        if(kAdcChannels[i] == channel) {
            indx = i;
            break;
        }
    }
#endif
    // Find bounds
    uint32_t start = indx * ADC_SAMPLE_CNT;
    uint32_t stop  = start + ADC_SAMPLE_CNT;
    // Average values
    uint32_t rslt = 0;
    for(uint32_t i = start; i < stop; i++) rslt += ibuf[i];
    return rslt / ADC_SAMPLE_CNT;
}

uint32_t GetResultMedian(uint8_t channel) {
    uint32_t indx = 0;
#if (ADC_CHANNEL_CNT > 1)
    // Find Channel indx
    for(uint32_t i=0; i < ADC_CHANNEL_CNT; i++) {
        if(kAdcChannels[i] == channel) {
            indx = i;
            break;
        }
    }
#endif
    // Find bounds
    uint32_t start = indx * ADC_SAMPLE_CNT;
    return FindMediana<uint16_t>(&ibuf[start], ADC_SAMPLE_CNT);
}

uint32_t GetVDAmV(uint32_t Vref_ADC) {
    return (kAdcVRefIntCalVoltage * GetVRefIntCal()) / Vref_ADC;
}

uint32_t Adc2mV(uint32_t adc_ch_value, uint32_t Vref_value) {
    return (((kAdcVRefIntCalVoltage * GetVRefIntCal()) / kAdcMaxValue) * adc_ch_value) / Vref_value;
}
uint32_t GetVdda_mv(uint32_t vref_value) {
    return (kAdcVRefIntCalVoltage * GetVRefIntCal()) / vref_value;
}

} // namespace

#endif  // ADC_REQUIRED
