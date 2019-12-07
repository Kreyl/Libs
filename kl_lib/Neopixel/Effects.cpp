/*
 * Effects.cpp
 *
 *  Created on: 5 ???. 2019 ?.
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

// On-off layer
#define SMOOTH_VAR      720

// Flash
#define BACK_CLR        (Color_t(255, 207, 0))
#define FLASH_CLR       (Color_t(255, 255, 255))
#define FLASH_CNT       2

// Do not touch
#define BRT_MAX     255L

static void SetColorRing(int32_t Indx, Color_t Clr) {
    if(Indx >= PIX_PER_BAND or Indx < 0) return;
    Leds.ClrBuf[Indx] = Clr;   // Always for first chunk
    // Iterate bands
    for(int32_t n=2; n <= BAND_NUMBER; n++) {
        int32_t i = (n & 1)? (PIX_PER_BAND * (n-1) + Indx) : (PIX_PER_BAND * n - 1 - Indx);
        Leds.ClrBuf[i] = Clr;
    }
}

void MixToBuf(Color_t Clr, int32_t Brt, int32_t Indx) {
//    Printf("%u\r", Brt);
    SetColorRing(Indx, Color_t(FLASH_CLR, BACK_CLR, Brt));
}

#if 1 // ======= Flash =======
class Flash_t {
private:
    systime_t IStart = 0;
    Color_t Clr = FLASH_CLR;
    int32_t IndxStart, Len;
    uint32_t Delay_ms = 63;  // Delay between updates
public:
    void Generate() {
        Delay_ms = Random::Generate(27, 99);
        IndxStart = -1;
        Len = 4; // XXX Randomize
    }
    void Update() {
        // Check if time to move
        if(TIME_I2MS(chVTTimeElapsedSinceX(IStart)) >= Delay_ms) {
            IndxStart++;
            IStart = chVTGetSystemTimeX();
        }
        // Check if path completed
        if((IndxStart - Len) > (PIX_PER_BAND + 7)) Generate();
        // Draw it
        for(int32_t i=0; i<Len; i++) {
            MixToBuf(Clr, ((BRT_MAX * (Len - i)) / Len), IndxStart - i);
        }
    }
};

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
        for(uint32_t i=0; i<LED_CNT; i++) {
            ColorHSV_t ClrH(Leds.ClrBuf[i]);
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

    void UpdateI() {
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
    OnOffLayer.UpdateI();
    chSysUnlockFromISR();
}
#endif

// Thread
static THD_WORKING_AREA(waNpxThread, 512);
__noreturn
static void NpxThread(void *arg) {
    chRegSetThreadName("Npx");
    while(true) {
        chThdSleepMilliseconds(18);
        // Reset colors
        Leds.SetAll(BACK_CLR);
        // Iterate flashes
        for(Flash_t &IFlash : FlashBuf) IFlash.Update();
        // Process OnOff
        OnOffLayer.Apply();
        // Show it
        Leds.SetCurrentColors();
    }
}


void EffInit() {
    for(Flash_t &IFlash : FlashBuf) IFlash.Generate();
    chThdCreateStatic(waNpxThread, sizeof(waNpxThread), NORMALPRIO, (tfunc_t)NpxThread, nullptr);
}

void EffFadeIn()  { OnOffLayer.FadeIn();  }
void EffFadeOut() { OnOffLayer.FadeOut(); }
