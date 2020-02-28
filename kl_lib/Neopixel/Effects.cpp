/*
 * Effects.cpp
 *
 *  Created on: 5 dec 2019
 *      Author: Kreyl
 */

#include "Effects.h"
#include "ch.h"
#include "kl_lib.h"
#include "color.h"
#include "ws2812b.h"
#include "MsgQ.h"
#include "board.h"

extern Neopixels_t Leds;

#define BACK_CLR        (Color_t(255, 255, 153))
// On-off layer
#define SMOOTH_VAR      180

// Do not touch
#define BRT_MAX     255L

static void SetColorRing(int32_t Indx, Color_t Clr) {
    if(Indx < 0) return;
    int32_t NStart = 0;
    for(int32_t n=0; n < Leds.BandCnt; n++) { // Iterate bands
        int32_t Length = Leds.BandSetup[n].Length; // to make things shorter
        if(Indx < Length) {
            uint32_t i;
            if(Leds.BandSetup[n].Dir == dirForward) i = NStart + Indx;
            else i = NStart + Length - 1 - Indx;
            Leds.ClrBuf[i] = Clr;
        }
        NStart += Length; // Calculate start indx of next band
    }
}

void MixToBuf(Color_t Clr, int32_t Brt, int32_t Indx) {
//    Printf("%u\r", Brt);
    SetColorRing(Indx, Color_t(Clr, BACK_CLR, Brt));
}

#if 1 // ======= Flash =======
#define FLASH_CLR       (Color_t(63, 63, 255))
#define FLASH_CNT       2
void FlashTmrCallback(void *p);

class Flash_t {
private:
    int32_t IndxStart, Len;
    uint32_t DelayUpd_ms;  // Delay between updates
    virtual_timer_t ITmr;
    void StartTimerI(uint32_t ms) {
        chVTSetI(&ITmr, TIME_MS2I(ms), FlashTmrCallback, this);
    }
public:
    int32_t EndIndx = 24; // which Indx to touch to consider flash ends XXX not good
    Color_t Clr = FLASH_CLR;
    void Apply() {
        for(int32_t i=0; i<Len; i++) {
            MixToBuf(Clr, ((BRT_MAX * (Len - i)) / Len), IndxStart - i);
        }
    }

    void GenerateI(uint32_t DelayBefore_ms) {
        DelayUpd_ms = Random::Generate(36, 63);
        IndxStart = -1;
        Len = 4;
//        Len = Random::Generate(11, 18);
//        Len = Random::Generate(14, 18);
        // Start delay before
        StartTimerI(DelayBefore_ms);
    }

    void OnTmrI() {
        IndxStart++; // Time to move
        // Check if path completed
        if((IndxStart - Len) > EndIndx) {
//            GenerateI(450);
            GenerateI(Random::Generate(630, 1800));
        }
        else StartTimerI(DelayUpd_ms);
    }
};

void FlashTmrCallback(void *p) {
    chSysLockFromISR();
    ((Flash_t*)p)->OnTmrI();
    chSysUnlockFromISR();
}

Flash_t FlashBuf[FLASH_CNT];
#endif

#if 1 // ======= OnOff Layer =======
void OnOffTmrCallback(void *p);

class OnOffLayer_t {
private:
    int32_t Brt = 0;
    enum State_t {stIdle, stFadingOut, stFadingIn} State;
    virtual_timer_t ITmr;
    void StartTimerI(uint32_t ms) {
        chVTSetI(&ITmr, TIME_MS2I(ms), OnOffTmrCallback, nullptr);
    }
public:
    void Apply() {
        if(State == stIdle) return; // No movement here
        for(int32_t i=0; i<Leds.LedCntTotal; i++) {
            ColorHSV_t ClrH;
            ClrH.FromRGB(Leds.ClrBuf[i]);
            ClrH.V = (ClrH.V * Brt) / BRT_MAX;
            Leds.ClrBuf[i].FromHSV(ClrH.H, ClrH.S, ClrH.V);
        }
    }

    void FadeIn() {
        State = stFadingIn;
        chSysLock();
        StartTimerI(ClrCalcDelay(Brt, SMOOTH_VAR));
        chSysUnlock();
    }

    void FadeOut() {
        State = stFadingOut;
        chSysLock();
        StartTimerI(ClrCalcDelay(Brt, SMOOTH_VAR));
        chSysUnlock();
    }

    void OnTmrI() {
        switch(State) {
            case stFadingIn:
                if(Brt == BRT_MAX) {
                    State = stIdle;
                    EvtQMain.SendNowOrExitI(EvtMsg_t(evtIdFadeInDone));
                }
                else {
                    Brt++;
                    StartTimerI(ClrCalcDelay(Brt, SMOOTH_VAR));
                }
                break;

            case stFadingOut:
                if(Brt == 0) {
                    State = stIdle;
                    EvtQMain.SendNowOrExitI(EvtMsg_t(evtIdFadeOutDone));
                }
                else {
                    Brt--;
                    StartTimerI(ClrCalcDelay(Brt, SMOOTH_VAR));
                }
                break;

            default: break;
        }
    }
} OnOffLayer;

void OnOffTmrCallback(void *p) {
    chSysLockFromISR();
    OnOffLayer.OnTmrI();
    chSysUnlockFromISR();
}
#endif

// Thread
static THD_WORKING_AREA(waNpxThread, 512);
__noreturn
static void NpxThread(void *arg) {
    chRegSetThreadName("Npx");
    while(true) {
        chThdSleepMilliseconds(7);
        // Reset colors
        Leds.SetAll(BACK_CLR);
        // Iterate flashes
        for(Flash_t &IFlash : FlashBuf) IFlash.Apply();
        // Process OnOff
        OnOffLayer.Apply();
        // Show it
        Leds.SetCurrentColors();
    }
}

namespace Eff {
void Init() {
    for(uint32_t i=0; i<FLASH_CNT; i++) {
        chSysLock();
        FlashBuf[i].GenerateI(i * 450 + 9);
        chSysUnlock();
    }
    chThdCreateStatic(waNpxThread, sizeof(waNpxThread), NORMALPRIO, (tfunc_t)NpxThread, nullptr);
}

void FadeIn()  { OnOffLayer.FadeIn();  }
void FadeOut() { OnOffLayer.FadeOut(); }

} // namespace
