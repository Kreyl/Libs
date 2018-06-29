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
    }
}

// Universal VirtualTimer callback
void TmrEffCallback(void *p) {
    chSysLockFromISR();
    EvtQEffects.SendNowOrExitI(EffMsg_t((Effects_t*)p));
    chSysUnlockFromISR();
}
#endif

#if 1 // ========================= Individual effects ==========================
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
        chVTSet(&Tmr, MS2ST(11), TmrEffCallback, this); // Arm timer for some time
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
    else chVTSet(&Tmr, MS2ST(Delay), TmrEffCallback, this); // Arm timer
}

/*
void EffOneByOne_t::SetupAndStart(Color_t ATargetClr, uint32_t ASmoothValue) {
    if(ASmoothValue == 0) EffAllTogetherNow.SetupAndStart(ATargetClr);
    else {
        chSysLock();
        ISmoothValue = ASmoothValue;
        CurrentIndx = 0;
        for(int32_t i=0; i<LED_CNT; i++) DesiredClr[i] = ATargetClr;
        PCurrentEff = this;
        chThdResumeS(&PThd, MSG_OK);
        chSysUnlock();
    }
}
EffState_t EffOneByOne_t::Process() {
    uint32_t Delay = ICalcDelayN(CurrentIndx, ISmoothValue);
    Leds.ICurrentClr[CurrentIndx].Adjust(DesiredClr[CurrentIndx]);
    Leds.ISetCurrentColors();
    if(Delay == 0) {
        CurrentIndx++;
        if(CurrentIndx >= LED_CNT) return effEnd;  // Setup completed
    }
    else chThdSleepMilliseconds(Delay);
    return effInProgress;
}

// ======================== EffAllTogetherSequence_t ===========================
void EffAllTogetherSequence_t::SetupColors() {
    for(int32_t i=0; i<LED_CNT; i++) Leds.ICurrentClr[i] = ICurrColor;
    Leds.ISetCurrentColors();
}

void EffFadeOneByOne_t::SetupIDs() {
    for(uint32_t i=0; i<LED_CNT; i++) IDs[i] = i;
}

void EffFadeOneByOne_t::SetupAndStart(int32_t ThrLo, int32_t ThrHi) {
//    Printf("ThrLo: %d; ThrHi: %d\r", ThrLo, ThrHi);
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
//                Printf("%d Brt: %d\r", Indx, Brt);
                DesiredClr[Indx].BeMixOf(IClrHi, IClrLo, Brt);
            }
        }
    } // if(ThrHi > ThrLo)
    // Start processing
    chSysLock();
    PCurrentEff = this;
    chThdResumeS(&PThd, MSG_OK);
    chSysUnlock();
}
*/
#endif
