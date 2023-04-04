/*
 * kl_adc.h
 *
 *  Created on: 12.04.2013
 *      Author: kreyl
 */

#ifndef ADC_L476_H_
#define ADC_L476_H_

#include "kl_lib.h"
#include "board.h"
#include <vector>

#if ADC_REQUIRED

#define ADC_TIM                 TIM6 // TIM7 is not available as a trigger

#define ADC_MAX_VALUE           4095    // const: 2^12

#define ADC_VREFINT_CHNL        0   // Do not change, see ref man
#define ADC_TEMPERATURE_CHNL    17
#define ADC_VBAT_CHNL           18
#define ADC_MAX_SEQ_LEN         16  // 1...16; Const, see ref man p.38
// VRefInt calibration values for L476 and L433
#define ADC_VREFINT_CAL_mV      3000UL
#define ADC_VREFINT_CAL_ADC     (*(volatile uint16_t*)0x1FFF75AA)

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

enum VRefVoltage_t {vrefBufDisabled, vrefBufv2048, vrefBufv2500};

struct AdcChannel_t {
    GPIO_TypeDef *GPIO;
    uint32_t Pin;
    uint32_t ChannelN;
};

struct AdcSetup_t {
    // When the VREF+ pin is bonded with VDDA pin, the voltage reference buffer is not available and must be kept disabled
    VRefVoltage_t VRefBufVoltage;
    uint32_t SampleTime;
    enum Oversampling_t : uint32_t {
        oversmpDis = 0,
        oversmp2   = ((0b0001UL << 5) | (0b000UL << 2) | ADC_CFGR2_ROVSE),
        oversmp4   = ((0b0010UL << 5) | (0b001UL << 2) | ADC_CFGR2_ROVSE),
        oversmp8   = ((0b0011UL << 5) | (0b010UL << 2) | ADC_CFGR2_ROVSE),
        oversmp16  = ((0b0100UL << 5) | (0b011UL << 2) | ADC_CFGR2_ROVSE),
        oversmp32  = ((0b0101UL << 5) | (0b100UL << 2) | ADC_CFGR2_ROVSE),
        oversmp64  = ((0b0110UL << 5) | (0b101UL << 2) | ADC_CFGR2_ROVSE),
        oversmp128 = ((0b0111UL << 5) | (0b110UL << 2) | ADC_CFGR2_ROVSE),
        oversmp256 = ((0b1000UL << 5) | (0b111UL << 2) | ADC_CFGR2_ROVSE)
    } Oversampling;
    ftVoidVoid DoneCallbackI;
    std::vector<AdcChannel_t> Channels;
};

typedef std::vector<uint16_t> AdcBuf_t;

class InnAdc_t {
private:
    const stm32_dma_stream_t *PDma;
    AdcBuf_t IBuf1, IBuf2, *PBufW = &IBuf1, *PBufR = &IBuf2;
    void SetSequenceLength(uint32_t ALen);
    void SetChannelSampleTime(uint32_t AChnl, uint32_t ASampleTime);
    void SetSequenceItem(uint8_t SeqIndx, uint32_t AChnl);
    ftVoidVoid ICallbackI = nullptr;
    void DisableCalibrateEnableSetDMA(); // Service routine
    void DisableVrefBuf();
public:
    AdcBuf_t& GetBuf() { return *PBufR; }
    void Init(const AdcSetup_t& Setup);
    void Deinit();
    void EnableVref();
    void DisableVref();
    void StartSingleMeasurement();
    void StartPeriodicMeasurement(uint32_t FSmpHz);
    uint32_t Adc2mV(uint32_t AdcChValue, uint32_t VrefValue);
//    uint32_t GetResult(uint8_t AChannel);
    // Inner use
    void IOnDmaIrqI();
};

extern InnAdc_t InnAdc;

#endif // ADC_REQUIRED

#endif // ADC_L476_H_
