/*
 * IntelLedEffs.cpp
 *
 *  Created on: 31 рту. 2017 у.
 *      Author: Kreyl
 */

#include "IntelLedEffs.h"
#include "MsgQ.h"

#if 1 // ========================== Common Effects =============================
struct EffMsg_t {
    Effects_t* Ptr;
    EffMsg_t() : Ptr(nullptr) {}
    EffMsg_t(Effects_t *APtr) : Ptr(APtr) {}
    EffMsg_t& operator = (const EffMsg_t &Right) {
        Ptr = Right.Ptr;
        return *this;
    }
} __attribute__((__packed__));


static EvtMsgQ_t<EffMsg_t, MAIN_EVT_Q_LEN> EvtQEffects;

static THD_WORKING_AREA(waEffectsThread, 256);
__noreturn
static void EffectsThread(void *arg) {
    chRegSetThreadName("Effects");
    while(true) {
        EffMsg_t Msg = EvtQEffects.Fetch(TIME_INFINITE);
        Msg.Ptr->Process();
    } // while
}

void CommonEffectsInit() {
    EvtQEffects.Init();
    chThdCreateStatic(waEffectsThread, sizeof(waEffectsThread), HIGHPRIO, (tfunc_t)EffectsThread, NULL);
}

void Effects_t::Process() {
    switch(State) {
        case effNone: break;
        case effAllTogetherSmoothly: ProcessAllTogetherSmoothly(); break;
        case effOneByOne: ProcessOneByOne(); break;
        case effSequence: ProcessSequence(); break;
    }
}

// Universal VirtualTimer callback
void TmrEffCallback(void *p) {
    chSysLockFromISR();
    EvtQEffects.SendNowOrExitI(EffMsg_t((Effects_t*)p));
    chSysUnlockFromISR();
}


void Effects_t::SetupDelay(uint32_t ms) {
    chVTSet(&Tmr, MS2ST(ms), TmrEffCallback, this);
}
#endif

#if 1 // =========================== Simple effects ============================
void Effects_t::AllTogetherNow(Color_t Color) {
    State = effNone;
    chVTReset(&Tmr);
    for(uint32_t i=0; i<LED_CNT; i++) Leds->ICurrentClr[i] = Color;
    Leds->ISetCurrentColors();
}

void Effects_t::AllTogetherSmoothly(Color_t Color, uint32_t ASmoothValue) {
    if(ASmoothValue == 0) AllTogetherNow(Color);
    else {
        ISmoothValue = ASmoothValue;
        for(int32_t i=0; i<LED_CNT; i++) DesiredClr[i] = Color;
        State = effAllTogetherSmoothly;
        SetupDelay(11); // Arm timer for some time
    }
}

void Effects_t::ProcessAllTogetherSmoothly() {
    uint32_t Delay = 0;
    for(int32_t i=0; i<LED_CNT; i++) {
        uint32_t tmp = ICalcDelayN(i, ISmoothValue);  // }
        if(tmp > Delay) Delay = tmp;                  // } Calculate Delay
        Leds->ICurrentClr[i].Adjust(DesiredClr[i]);   // Adjust current color
    } // for
    Leds->ISetCurrentColors();
    if (Delay == 0) State = effNone;  // Setup completed
    else SetupDelay(Delay); // Arm timer
}

void Effects_t::OneByOne(Color_t Color, uint32_t ASmoothValue) {
    if(ASmoothValue == 0) AllTogetherNow(Color);
    else {
        ISmoothValue = ASmoothValue;
        CurrentIndx = 0;
        for(int32_t i=0; i<LED_CNT; i++) DesiredClr[i] = Color;
        State = effOneByOne;
        SetupDelay(11); // Arm timer for some time
    }
}

void Effects_t::ProcessOneByOne() {
    while(true) {
        uint32_t Delay = ICalcDelayN(CurrentIndx, ISmoothValue);
        Leds->ICurrentClr[CurrentIndx].Adjust(DesiredClr[CurrentIndx]);
        Leds->ISetCurrentColors();
        if(Delay == 0) {
            CurrentIndx++;
            if(CurrentIndx >= LED_CNT) {
                State = effNone;  // Setup completed
                return;
            }
            else continue;
        }
        else {
            SetupDelay(Delay); // Arm timer
            break;
        }
    }
}
#endif

#if 1 // ========================= Sequences ===================================
void Effects_t::SeqAllTogetherStartOrContinue(const LedRGBChunk_t *PChunk) {
    if(PChunk == IPStartChunk) return; // Same sequence
    else SeqAllTogetherStartOrRestart(PChunk);
}

void Effects_t::SeqAllTogetherStartOrRestart(const LedRGBChunk_t *PChunk) {
    chSysLock();
    RepeatCounter = -1;
    IPStartChunk = PChunk;   // Save first chunk
    IPCurrentChunk = PChunk;
    State = effSequence;
    chSysUnlock();
    EvtQEffects.SendNowOrExit(EffMsg_t(this));
}

void Effects_t::ProcessSequence() {
    while(true) {   // Process the sequence
        switch(IPCurrentChunk->ChunkSort) {
            case csSetup: // setup now and exit if required
                if(ISetup() == sltBreak) return;
                break;

            case csWait: { // Start timer, pointing to next chunk
                    uint32_t Delay = IPCurrentChunk->Time_ms;
                    IPCurrentChunk++;
                    if(Delay != 0) {
                        SetupDelay(Delay);
                        return;
                    }
                }
                break;

            case csGoto:
                IPCurrentChunk = IPStartChunk + IPCurrentChunk->ChunkToJumpTo;
                SetupDelay(1);
                return;
                break;

            case csEnd:
                IPStartChunk = nullptr;
                IPCurrentChunk = nullptr;
                State = effNone;
                return;
                break;

            case csRepeat:
                if(RepeatCounter == -1) RepeatCounter = IPCurrentChunk->RepeatCnt;
                if(RepeatCounter == 0) {    // All was repeated, goto next
                    RepeatCounter = -1;     // reset counter
                    IPCurrentChunk++;
                }
                else {  // repeating in progress
                    IPCurrentChunk = IPStartChunk;  // Always from beginning
                    RepeatCounter--;
                }
                break;
        } // switch
    } // while

}

SequencerLoopTask_t Effects_t::ISetup() {
    SequencerLoopTask_t Rslt;
    // Iterate LEDs
    uint32_t Delay = 0;
    for(int32_t i=0; i<LED_CNT; i++) {
        Color_t &Clr = Leds->ICurrentClr[i];
        if(Clr != IPCurrentChunk->Color) {
            // If smooth time is zero, set color now
            if(IPCurrentChunk->Value == 0) Clr = IPCurrentChunk->Color;
            else {
                Clr.Adjust(IPCurrentChunk->Color);
                uint32_t tmp = Clr.DelayToNextAdj(DesiredClr[i], IPCurrentChunk->Value);
                if(tmp > Delay) Delay = tmp;
            }
        } // if color is different
    } // for

    // Check if completed now
    if(Delay == 0) {
        IPCurrentChunk++; // Color is the same, goto next chunk
        Rslt = sltProceed;
    }
    else { // Not completed
        // Calculate time to next adjustment and start timer
        SetupDelay(Delay);
        Rslt = sltBreak;
    } // Not completed
    Leds->ISetCurrentColors();
    return Rslt;
}

#endif
