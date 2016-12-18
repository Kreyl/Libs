/*
 * ir.h
 *
 *  Created on: 04.07.2013
 *      Author: kreyl
 */

#pragma once

#include "hal.h"
#include "ch.h"
#include "kl_lib.h"
#include "board.h"
#include "uart.h"

#define IR_TX_ENABLED   TRUE
#define IR_RX_ENABLED   TRUE

#if 1 // ============================== IR Pkt =================================
union irPkt_t {
    uint16_t Word;
    struct {
        unsigned ID : 7;         // LSB
        unsigned Group : 3;
        unsigned Mode : 1;
        unsigned Damage : 3;
        unsigned ControlSum : 2; // MSB
    };
    uint8_t CalculateControlSum() {
        uint16_t Sum = 0;
        for(int i=2; i<=14; i+=2) {
            uint16_t w = Word << i;
            Sum ^= w;
        }
        return (Sum >>= 14);
    }
    bool IsIntact() {
        uint8_t CalcSum = CalculateControlSum();
        return CalcSum == ControlSum;
    }
    void Print() { Uart.Printf("Pkt: Word%04X; ID%u; Grp%u; Mode%u; Dmg%u; Sum%u\r", Word, ID, Group, Mode, Damage, ControlSum); }
} __packed;

#define IR_BIT_CNT          16
#endif

#if IR_TX_ENABLED // ========================== IR TX ==========================
#define IR_CARRIER_HZ       56000
#define MAX_PWR             255     // Top DAC value

// Delays, uS
#define IR_TICK_US          600 // Protocol smallest time unit, us
/* Header = 4 * IR_TICK_US
 * Space  = 1 * IR_TICK_US
 * Zero   = 1 * IR_TICK_US
 * One    = 2 * IR_TICK_US
 */

// Timings
#define IR_HEADER_US        2400
#define IR_ZERO_US          600
#define IR_ONE_US           1200

struct irChunk_t {
    uint8_t On;
    uint16_t Duration;
};
#define CHUNK_CNT   (1+1+(IR_BIT_CNT*2))    // Header + bit count

#define CARRIER_PERIOD_CNT  2
#define SAMPLING_FREQ_HZ    (CARRIER_PERIOD_CNT*IR_CARRIER_HZ)

#define IRLED_DMA_MODE  \
    STM32_DMA_CR_CHSEL(DAC_DMA_CHNL) | \
    DMA_PRIORITY_HIGH | \
    STM32_DMA_CR_MSIZE_BYTE | \
    STM32_DMA_CR_PSIZE_BYTE | \
    STM32_DMA_CR_MINC        | \
    STM32_DMA_CR_DIR_M2P     | \
    STM32_DMA_CR_CIRC

class irLed_t {
private:
    irChunk_t TxBuf[CHUNK_CNT], *PChunk; // Buffer of power values: header + all one's + 1 delay after
    Timer_t ChunkTmr{TMR_DAC_CHUNK};
    uint8_t CarrierArr[CARRIER_PERIOD_CNT], ZeroArr[CARRIER_PERIOD_CNT];
    Timer_t SamplingTmr{TMR_DAC_SMPL};
    void IDacCarrierDisable();
    void IDacCarrierEnable();
public:
    bool Busy;
    void Init();
    void TransmitWord(uint16_t wData, uint8_t Power);
    // Inner use
    void IChunkTmrHandler();
};

extern irLed_t irLed;
#endif

#if IR_RX_ENABLED // ========================== IR RX ==========================
#define IR_RX_POLLING_PERIOD_MS     90
#define IR_DEVIATION_US             150
#define IR_RX_PKT_TIMEOUT_MS

#define IR_RX_DMA_MODE  STM32_DMA_CR_CHSEL(IR_RX_TIM_DMA_CHNL) | \
                        DMA_PRIORITY_MEDIUM | \
                        STM32_DMA_CR_MSIZE_HWORD | \
                        STM32_DMA_CR_PSIZE_HWORD | \
                        STM32_DMA_CR_MINC |       /* Memory pointer increase */ \
                        STM32_DMA_CR_DIR_P2M |    /* Direction is peripheral to memory */ \
                        STM32_DMA_CR_CIRC         /* Circular buffer enable */

#define IR_RX_BUF_LEN   36

enum IrPktPartType_t {iptHeader, iptZero, iptOne, iptError};

class irReceiver_t {
private:
    Timer_t TmrRx{TMR_IR_RX};
    int32_t SzOld, RIndx;
    uint16_t IRxBuf[IR_RX_BUF_LEN];
    IrPktPartType_t MeasureDuration(uint16_t Duration) {
        if     (IS_LIKE(Duration, IR_HEADER_US, IR_DEVIATION_US)) return iptHeader;
        else if(IS_LIKE(Duration, IR_ZERO_US,   IR_DEVIATION_US)) return iptZero;
        else if(IS_LIKE(Duration, IR_ONE_US,    IR_DEVIATION_US)) return iptOne;
        else return iptError;
    }
    // Parsing
    int IBitCnt = -1; // Header not received
    systime_t IPktStartTime;
    irPkt_t ICurrentPkt;
public:
    void Init();
    irPkt_t LastPkt;
    void ITask();
};

extern irReceiver_t irRx;
#endif
