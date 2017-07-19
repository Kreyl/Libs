/*
 * ws2812b.cpp
 *
 *  Created on: 05 апр. 2014 г.
 *      Author: Kreyl
 */

#include "ws2812b.h"

#define LED_DMA_MODE    DMA_PRIORITY_HIGH \
                        | STM32_DMA_CR_MSIZE_HWORD \
                        | STM32_DMA_CR_PSIZE_HWORD \
                        | STM32_DMA_CR_MINC     /* Memory pointer increase */ \
                        | STM32_DMA_CR_DIR_M2P  /* Direction is memory to peripheral */ \
                        | STM32_DMA_CR_TCIE     /* Enable Transmission Complete IRQ */

// Tx timings: bit cnt
#define SEQ_1               0b1110  // 0xE
#define SEQ_0               0b1000  // 0x8

#define SEQ_00              0x88
#define SEQ_01              0x8E
#define SEQ_10              0xE8
#define SEQ_11              0xEE


LedWs_t LedWs;

extern "C" {
// Wrapper for Tx Completed IRQ
void LedTxcIrq(void *p, uint32_t flags) {
    dmaStreamDisable(LEDWS_DMA);
//    Uart.PrintfI("Irq\r");
}
} // "C"

void LedWs_t::Init() {
    PinSetupAlterFunc(LEDWS_PIN);
    ISpi.Setup(boMSB, cpolIdleLow, cphaFirstEdge, sclkDiv2, bitn16);
    ISpi.Enable();
    ISpi.EnableTxDma();

    Printf("Len=%u\r", TOTAL_W_CNT);

    // Zero buffer
    for(int i=0; i<TOTAL_W_CNT; i++) IBuf[i] = 0;

    // ==== DMA ====
    dmaStreamAllocate     (LEDWS_DMA, IRQ_PRIO_LOW, LedTxcIrq, NULL);
    dmaStreamSetPeripheral(LEDWS_DMA, &LEDWS_SPI->DR);
    dmaStreamSetMode      (LEDWS_DMA, LED_DMA_MODE);
}

void LedWs_t::AppendBitsMadeOfByte(uint8_t Byte) {
    uint8_t Bits, bMsb = 0, bLsb = 0;
    Bits = Byte & 0b11000000;
    if     (Bits == 0b00000000) bMsb = SEQ_00;
    else if(Bits == 0b01000000) bMsb = SEQ_01;
    else if(Bits == 0b10000000) bMsb = SEQ_10;
    else if(Bits == 0b11000000) bMsb = SEQ_11;

    Bits = Byte & 0b00110000;
    if     (Bits == 0b00000000) bLsb = SEQ_00;
    else if(Bits == 0b00010000) bLsb = SEQ_01;
    else if(Bits == 0b00100000) bLsb = SEQ_10;
    else if(Bits == 0b00110000) bLsb = SEQ_11;

    *PBuf++ = (bMsb << 8) | bLsb;

    Bits = Byte & 0b00001100;
    if     (Bits == 0b00000000) bMsb = SEQ_00;
    else if(Bits == 0b00000100) bMsb = SEQ_01;
    else if(Bits == 0b00001000) bMsb = SEQ_10;
    else if(Bits == 0b00001100) bMsb = SEQ_11;

    Bits = Byte & 0b00000011;
    if     (Bits == 0b00000000) bLsb = SEQ_00;
    else if(Bits == 0b00000001) bLsb = SEQ_01;
    else if(Bits == 0b00000010) bLsb = SEQ_10;
    else if(Bits == 0b00000011) bLsb = SEQ_11;

    *PBuf++ = (bMsb << 8) | bLsb;
}

void LedWs_t::ISetCurrentColors() {
    PBuf = IBuf + (RST_W_CNT / 2);    // First words are zero to form reset
    // Fill bit buffer
    for(uint32_t i=0; i<LED_CNT; i++) {
        AppendBitsMadeOfByte(ICurrentClr[i].G);
        AppendBitsMadeOfByte(ICurrentClr[i].R);
        AppendBitsMadeOfByte(ICurrentClr[i].B);
    }

    // Start transmission
    dmaStreamSetMemory0(LEDWS_DMA, IBuf);
    dmaStreamSetTransactionSize(LEDWS_DMA, TOTAL_W_CNT);
    dmaStreamSetMode(LEDWS_DMA, LED_DMA_MODE);
    dmaStreamEnable(LEDWS_DMA);
}

#if 1 // ============================ Effects ==================================
Effects_t Effects;

#define CHUNK_CNT   9
static LedChunk_t Chunk[CHUNK_CNT] = {
        {0, 16},
        {33, 17},
        {34, 49},
        {62, 50},
        {63, 78},
        {87, 79},
        {88, 104},
        {120, 105},
        {121, 131},
};

static THD_WORKING_AREA(waEffectsThread, 256);
__noreturn
static void EffectsThread(void *arg) {
    chRegSetThreadName("Effects");
    Effects.ITask();
}

__noreturn
void Effects_t::ITask() {
    while(true) {
        switch(IState) {
            case effIdle: chThdSleep(TIME_INFINITE); break;

            case effAllSmoothly: {
                uint32_t Delay = 0;
                for(uint8_t i=0; i<LED_CNT; i++) {
                    uint32_t tmp = ICalcDelayN(i);  // }
                    if(tmp > Delay) Delay = tmp;    // } Calculate Delay
                    LedWs.ICurrentClr[i].Adjust(DesiredClr[i]); // Adjust current color
                } // for
                LedWs.ISetCurrentColors();
                if(Delay == 0) {    // Setup completed
//                    App.SignalEvt(EVT_LED_DONE);
                    IState = effIdle;
                }
                else chThdSleepMilliseconds(Delay);
            } break;

            case effChunkRunningRandom: IProcessChunkRandom(); break;
        } // switch
    } // while true
}

void Effects_t::Init() {
    LedWs.Init();
    AllTogetherNow(clBlack);
    // Thread
    PThd = chThdCreateStatic(waEffectsThread, sizeof(waEffectsThread), HIGHPRIO, (tfunc_t)EffectsThread, NULL);
}


void Effects_t::AllTogetherNow(Color_t Color) {
    IState = effIdle;
    for(uint32_t i=0; i<LED_CNT; i++) LedWs.ICurrentClr[i] = Color;
    LedWs.ISetCurrentColors();
//    App.SignalEvt(EVT_LED_DONE);
}
void Effects_t::AllTogetherNow(ColorHSV_t Color) {
    IState = effIdle;
    Color_t rgb = Color.ToRGB();
    for(uint32_t i=0; i<LED_CNT; i++) LedWs.ICurrentClr[i] = rgb;
    LedWs.ISetCurrentColors();
//    App.SignalEvt(EVT_LED_DONE);
}

void Effects_t::AllTogetherSmoothly(Color_t Color, uint32_t ASmoothValue) {
    if(ASmoothValue == 0) AllTogetherNow(Color);
    else {
        chSysLock();
        for(uint32_t i=0; i<LED_CNT; i++) {
            DesiredClr[i] = Color;
            SmoothValue[i] = ASmoothValue;
        }
        IState = effAllSmoothly;
        chSchWakeupS(PThd, MSG_OK);
        chSysUnlock();
    }
}

//void Effects_t::AllTogetherSmoothly(ColorHSV_t Color, uint32_t ASmoothValue) {
//    if(ASmoothValue == 0) AllTogetherNow(Color);
//    else {
//        chSysLock();
//        for(uint32_t i=0; i<LED_CNT; i++) {
//            DesiredClr[i] = Color;
//            SmoothValue[i] = ASmoothValue;
//        }
//        IState = effAllSmoothly;
//        chSchWakeupS(PThd, MSG_OK);
//        chSysUnlock();
//    }
//}

void Effects_t::ChunkRunningRandom(Color_t Color, uint32_t NLeds, uint32_t ASmoothValue) {
    chSysLock();
    for(uint32_t i=0; i<CHUNK_CNT; i++) {
        Chunk[i].Color = Color;
        Chunk[i].NLeds = NLeds;
        Chunk[i].StartOver();
    }
    for(uint32_t i=0; i<LED_CNT; i++) {
        SmoothValue[i] = ASmoothValue;
    }
    IState = effChunkRunningRandom;
    chSchWakeupS(PThd, MSG_OK);
    chSysUnlock();
}

uint32_t t=0;

void Effects_t::IProcessChunkRandom() {
    uint32_t Delay = 0;
    for(uint32_t i=0; i<CHUNK_CNT; i++) {
        uint32_t ChunkDelay = Chunk[i].ProcessAndGetDelay();
        if(ChunkDelay > Delay) Delay = ChunkDelay;
    }
    LedWs.ISetCurrentColors();
    chThdSleepMilliseconds(Delay);
}

uint32_t Effects_t::ICalcDelayN(uint32_t n) {
    return LedWs.ICurrentClr[n].DelayToNextAdj(DesiredClr[n], SmoothValue[n]);
}

#if 1 // ============================== LedChunk ===============================
uint32_t LedChunk_t::ProcessAndGetDelay() {
    if(LedWs.ICurrentClr[Head] == Color) {   // Go on if done with current
        GetNext(&Head);
        GetNext(&Tail);
        Effects.DesiredClr[Head] = Color;
        Effects.DesiredClr[Tail] = clBlack;
    }
    // Iterate Leds
    uint32_t Delay = 0;
    int n = Start;
    do {
        uint32_t tmp = Effects.ICalcDelayN(n);  // }
        if(tmp > Delay) Delay = tmp;            // } Calculate Delay
        if(Delay!= 0) LedWs.ICurrentClr[n].Adjust(Effects.DesiredClr[n]); // Adjust current color
    } while(GetNext(&n) == retvOk);
    return Delay;
}

void LedChunk_t::StartOver() {
    Head = Start; //Random(Start, End);
    Tail = GetPrevN(Head, NLeds);
    Effects.DesiredClr[Head] = Color;
    Effects.DesiredClr[Tail] = clBlack;
}

uint8_t LedChunk_t::GetNext(int *PCurrent) {
    int curr = *PCurrent;
    if(curr == End) {
        *PCurrent = Start;
        return retvOverflow;
    }
    else {
        if(End > Start) *PCurrent = curr + 1;
        else *PCurrent = curr - 1;
        return retvOk;
    }
}

int LedChunk_t::GetPrevN(int Current, int N) {
    int Rslt;
    if(End > Start) {
        Rslt = Current - N;
        if(Rslt < Start) Rslt += 1 + End - Start;
    }
    else {
        Rslt = Current + N;
        if(Rslt > Start) Rslt -= Start - End + 1;
    }
    return Rslt;
}
#endif

#endif
