/*
 * ws2812b.cpp
 *
 *  Created on: 05 апр. 2014 г.
 *      Author: Kreyl
 */

#include <sk6812.h>

#if 1 // =============================== LEDs ==================================
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

// Wrapper for Tx Completed IRQ
extern "C"
void LedTxcIrq(void *p, uint32_t flags) {
    dmaStreamDisable(LEDWS_DMA);
}

void LedSk_t::Init() {
    PinSetupAlterFunc(LEDWS_PIN);
    ISpi.Setup(boMSB, cpolIdleLow, cphaFirstEdge, sclkDiv2, bitn16);
    ISpi.Enable();
    ISpi.EnableTxDma();

    // Zero buffer
    for(int i=0; i<TOTAL_W_CNT; i++) IBuf[i] = 0;
    // Set colors to black
    for(uint32_t i=0; i<LED_CNT; i++) ICurrentClr[i] = clRGBWBlack;

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
#endif

#if 1 // ========================== Common Effects =============================
EffAllTogetherNow_t EffAllTogetherNow;
EffAllTogetherSmoothly_t EffAllTogetherSmoothly;
EffFadeOneByOne_t EffFadeOneByOne(180, clRGBWGreen, clRGBWBlack);

static EffBase_t *PCurrentEff = nullptr;
static thread_reference_t PThd = nullptr;
static Color_t DesiredClr[LED_CNT];

static THD_WORKING_AREA(waEffectsThread, 64);
__noreturn
static void EffectsThread(void *arg) {
    chRegSetThreadName("Effects");
    while(true) {
        if(PCurrentEff == nullptr) {
            chSysLock();
            chThdSuspendS(&PThd);
            chSysUnlock();
        }
        else {
            if(PCurrentEff->Process() == effEnd) {
                PCurrentEff = nullptr;
            }
        }
    }
}

uint32_t ICalcDelayN(uint32_t n, uint32_t SmoothValue) {
    return Leds.ICurrentClr[n].DelayToNextAdj(DesiredClr[n], SmoothValue);
}

void LedEffectsInit() {
    Leds.Init();
    chThdCreateStatic(waEffectsThread, sizeof(waEffectsThread), HIGHPRIO, (tfunc_t)EffectsThread, NULL);
}
#endif

#if 1 // ========================= Individual effects ==========================
void EffAllTogetherNow_t::SetupAndStart(Color_t Color) {
    PCurrentEff = nullptr;
    for(uint32_t i=0; i<LED_CNT; i++) Leds.ICurrentClr[i] = Color;
    Leds.ISetCurrentColors();
}

void EffAllTogetherSmoothly_t::SetupAndStart(Color_t Color, uint32_t ASmoothValue) {
    if(ASmoothValue == 0) EffAllTogetherNow.SetupAndStart(Color);
    else {
        chSysLock();
        ISmoothValue = ASmoothValue;
        for(uint32_t i=0; i<LED_CNT; i++) DesiredClr[i] = Color;
        PCurrentEff = this;
        chThdResumeS(&PThd, MSG_OK);
        chSysUnlock();
    }
}

EffState_t EffAllTogetherSmoothly_t::Process() {
    uint32_t Delay = 0;
    for(uint8_t i=0; i<LED_CNT; i++) {
        uint32_t tmp = ICalcDelayN(i, ISmoothValue);  // }
        if(tmp > Delay) Delay = tmp;                  // } Calculate Delay
        Leds.ICurrentClr[i].Adjust(DesiredClr[i]);    // Adjust current color
    } // for
    Leds.ISetCurrentColors();
    if (Delay == 0) return effEnd;  // Setup completed
    else {
        chThdSleepMilliseconds(Delay);
        return effInProgress;
    }
}


void EffFadeOneByOne_t::SetupIDs() {
    for(int i=0; i<LED_CNT; i++) IDs[i] = i;
}

void EffFadeOneByOne_t::SetupAndStart(int32_t ThrLo, int32_t ThrHi) {
    Printf("ThrLo: %d; ThrHi: %d\r", ThrLo, ThrHi);
    // Setup ColorLo
    for(int32_t i=0; i < ThrLo; i++) DesiredClr[i] = IClrLo;
    // Setup ColorHi
    for(int32_t i=ThrHi; i < LED_CNT; i++) DesiredClr[i] = IClrHi;
    // Setup gradient
    if(ThrHi > ThrLo) {
        int32_t Len = ThrHi - ThrLo;
        int32_t BrtStep = (255 * 1024) / Len;   // 255 is top brightness, 1024 is scaling coef
        for(int32_t i=0; i<Len; i++) {
            int32_t Indx = ThrLo + i;
            if(Indx >=0 and Indx < LED_CNT) {
                int32_t Brt = (i * BrtStep) / 1024;
                Printf("%d Brt: %d\r", Indx, Brt);
                DesiredClr[Indx].BeMixOf(IClrLo, IClrHi, Brt);
            }
        }
    } // if(ThrHi > ThrLo)
    // Start processing
    chSysLock();
    PCurrentEff = this;
    chThdResumeS(&PThd, MSG_OK);
    chSysUnlock();
}

#if 1 // ============================ All together =============================
#endif

#if 1 // ============================= One by one ==============================

#endif

#if 1 // ============================= Chunks ==================================
#define CHUNK_CNT   4
//static LedChunk_t Chunk[CHUNK_CNT] = {
//        {0, 0},
//        {1, 1},
//        {2, 2},
//        {3, 3},
//};

/*  0...(Thr-GRADIENT_LEN_ROWS): Color1;
 *  (Thr-GRADIENT_LEN_ROWS)...Thr: gradient
 *  Thr...LED_CNT: Color2
 */
//void Effects_t::SetChunksGradient(uint32_t Thr, uint32_t GradientLen) {
    // Set Color1 for chunks 0...(Thr-GRADIENT_LEN_ROWS)
//    for(int i=0; i<(Thr-GradientLen); i++) {
//        if(i >= CHUNK_CNT) break;   // Do not go beyond
//        Chunk[i].
//    }
//}
#endif



#endif
