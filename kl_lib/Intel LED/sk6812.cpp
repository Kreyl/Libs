/*
 * ws2812b.cpp
 *
 *  Created on: 05 апр. 2014 г.
 *      Author: Kreyl
 */

#include <sk6812.h>

#define LED_DMA_MODE    DMA_PRIORITY_HIGH \
                        | STM32_DMA_CR_MSIZE_HWORD \
                        | STM32_DMA_CR_PSIZE_HWORD \
                        | STM32_DMA_CR_MINC     /* Memory pointer increase */ \
                        | STM32_DMA_CR_DIR_M2P  /* Direction is memory to peripheral */ \
                        | STM32_DMA_CR_TCIE     /* Enable Transmission Complete IRQ */

// Tx timings: bit cnt
/*
bit len @ 4MHz: 250
t0h 150...450       1
t0l 750...1050      3? 4
t1h 450...750       2
t1l 450...750       2
 */

#define SEQ_0               0b1000  // 0x8
#define SEQ_1               0b1100  // 0xC

#define SEQ_00              0x88
#define SEQ_01              0x8E
#define SEQ_10              0xE8
#define SEQ_11              0xEE


LedSk_t Leds;

extern "C" {
// Wrapper for Tx Completed IRQ
void LedTxcIrq(void *p, uint32_t flags) {
    dmaStreamDisable(LEDWS_DMA);
//    Uart.PrintfI("Irq\r");
}
} // "C"

void LedSk_t::Init() {
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

void LedSk_t::AppendBitsMadeOfByte(uint8_t Byte) {
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

void LedSk_t::ISetCurrentColors() {
    PBuf = IBuf + (RST_W_CNT / 2);    // First words are zero to form reset
    // Fill bit buffer
    for(uint32_t i=0; i<LED_CNT; i++) {
        AppendBitsMadeOfByte(ICurrentClr[i].G);
        AppendBitsMadeOfByte(ICurrentClr[i].R);
        AppendBitsMadeOfByte(ICurrentClr[i].B);
        AppendBitsMadeOfByte(ICurrentClr[i].W);
    }

    // Start transmission
    dmaStreamSetMemory0(LEDWS_DMA, IBuf);
    dmaStreamSetTransactionSize(LEDWS_DMA, TOTAL_W_CNT);
    dmaStreamSetMode(LEDWS_DMA, LED_DMA_MODE);
    dmaStreamEnable(LEDWS_DMA);
}

#if 1 // ============================ Effects ==================================
Effects_t Effects;

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
                    Leds.ICurrentClr[i].Adjust(DesiredClr[i]); // Adjust current color
                } // for
                Leds.ISetCurrentColors();
                if(Delay == 0) {    // Setup completed
//                    App.SignalEvt(EVT_LED_DONE);
                    IState = effIdle;
                }
                else chThdSleepMilliseconds(Delay);
            } break;
        } // switch
    } // while true
}

void Effects_t::Init() {
    Leds.Init();
    AllTogetherNow(clBlack);
    // Thread
    PThd = chThdCreateStatic(waEffectsThread, sizeof(waEffectsThread), HIGHPRIO, (tfunc_t)EffectsThread, NULL);
}


void Effects_t::AllTogetherNow(Color_t Color) {
    IState = effIdle;
    for(uint32_t i=0; i<LED_CNT; i++) Leds.ICurrentClr[i] = Color;
    Leds.ISetCurrentColors();
//    App.SignalEvt(EVT_LED_DONE);
}
void Effects_t::AllTogetherNow(ColorHSV_t Color) {
    IState = effIdle;
    Color_t rgb = Color.ToRGB();
    for(uint32_t i=0; i<LED_CNT; i++) Leds.ICurrentClr[i] = rgb;
    Leds.ISetCurrentColors();
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

uint32_t Effects_t::ICalcDelayN(uint32_t n) {
    return Leds.ICurrentClr[n].DelayToNextAdj(DesiredClr[n], SmoothValue[n]);
}

#endif
