/*
 * IntelLedEffs.h
 *
 *  Created on: 31 рту. 2017 у.
 *      Author: Kreyl
 */

#pragma once

#include <inttypes.h>
#include "color.h"
#include "ws2812b.h"
//#include "sk6812.h"
#include "ChunkTypes.h"

#if 1 // =============================== Chunk =================================
class LedChunk_t {
private:
    int Head, Tail;
    uint8_t GetNext(int *PCurrent);
    int GetPrevN(int Current, int N);
public:
    int Start, End, NLeds;
    Color_t Color;
    LedChunk_t(int AStart, int AEnd) {
        Start = AStart;
        End = AEnd;
        NLeds = 1;
        Head = AStart;
        Tail = AEnd;
        Color = clBlack;
    }
    uint32_t ProcessAndGetDelay();
    void StartOver();
};
#endif

#if 1 // ============================== Effects ================================
enum EffState_t {effNone, effAllTogetherSmoothly, effOneByOne, effSequence};

class Effects_t {
private:
    Color_t DesiredClr[LED_CNT];
    Neopixels_t *Leds;
    uint32_t ISmoothValue = 0;
    uint32_t CurrentIndx = 0;
    uint32_t ICalcDelayN(uint32_t n, uint32_t SmoothValue) {
        return Leds->ICurrentClr[n].DelayToNextAdj(DesiredClr[n], SmoothValue);
    }
    virtual_timer_t Tmr;
    void ProcessAllTogetherSmoothly();
    void ProcessOneByOne();
    // Sequences
    const LedRGBChunk_t *IPStartChunk, *IPCurrentChunk;
    int32_t RepeatCounter = -1;
    void ProcessSequence();
    SequencerLoopTask_t ISetup();
    void SetupDelay(uint32_t ms);
public:
    Effects_t(Neopixels_t *PLeds) : Leds(PLeds) {}
    void AllTogetherNow(Color_t Color);
    void AllTogetherSmoothly(Color_t Color, uint32_t ASmoothValue);
    void OneByOne(Color_t Color, uint32_t ASmoothValue);
    // Sequences
    void SeqAllTogetherStartOrRestart(const LedRGBChunk_t *PChunk);
    void SeqAllTogetherStartOrContinue(const LedRGBChunk_t *PChunk);
    // Inner use
    void Process();
    EffState_t State = effNone;
};

void CommonEffectsInit();

#endif

