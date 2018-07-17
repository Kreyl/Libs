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
enum EffState_t {effNone, effAllTogetherSmoothly, effOneByOne};

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
public:
    Effects_t(Neopixels_t *PLeds) : Leds(PLeds) {}
    void AllTogetherNow(Color_t Color);
    void AllTogetherSmoothly(Color_t Color, uint32_t ASmoothValue);
    void OneByOne(Color_t Color, uint32_t ASmoothValue);
    // Inner use
    void Process();
    EffState_t State = effNone;
};


/*
class EffAllTogetherSequence_t : public EffBase_t, public BaseSequencer_t<LedRGBChunk_t> {
private:
    Color_t ICurrColor;
    EffState_t Process() { return effEnd; } // Dummy, never used
    void ISwitchOff() {
        ICurrColor = clBlack;
    }
    void SetupColors();

    SequencerLoopTask_t ISetup() {
        if(ICurrColor != IPCurrentChunk->Color) {
            if(IPCurrentChunk->Value == 0) {     // If smooth time is zero,
                ICurrColor = IPCurrentChunk->Color;// set color now,
                SetupColors();
                IPCurrentChunk++;                // and goto next chunk
            }
            else {
                ICurrColor.Adjust(IPCurrentChunk->Color);
                SetupColors();
                // Check if completed now
                if(ICurrColor == IPCurrentChunk->Color) IPCurrentChunk++;
                else { // Not completed
                    // Calculate time to next adjustment
                    uint32_t Delay = ICurrColor.DelayToNextAdj(IPCurrentChunk->Color, IPCurrentChunk->Value);
                    SetupDelay(Delay);
                    return sltBreak;
                } // Not completed
            } // if time > 256
        } // if color is different
        else IPCurrentChunk++; // Color is the same, goto next chunk
        return sltProceed;
    }
};

class EffOneByOne_t : public EffBase_t {
protected:
    uint32_t ISmoothValue;
    uint32_t CurrentIndx;
public:
    void SetupAndStart(Color_t ATargetClr, uint32_t ASmoothValue);
    EffState_t Process();
};

class EffFadeOneByOne_t : public EffAllTogetherSmoothly_t {
private:
    uint8_t IDs[LED_CNT];
    Color_t IClrLo, IClrHi;
public:
    void SetupAndStart(int32_t ThrLo, int32_t ThrHi);
    void SetupIDs();
    EffFadeOneByOne_t(uint32_t ASmoothValue, Color_t AClrLo, Color_t AClrHi) :
        IClrLo(AClrLo), IClrHi(AClrHi) { ISmoothValue = ASmoothValue; }
};

extern EffAllTogetherNow_t EffAllTogetherNow;
extern EffAllTogetherSmoothly_t EffAllTogetherSmoothly;
extern EffOneByOne_t EffOneByOne;
extern EffFadeOneByOne_t EffFadeOneByOne;
extern EffAllTogetherSequence_t EffAllTogetherSequence;


*/

void CommonEffectsInit();

#endif

