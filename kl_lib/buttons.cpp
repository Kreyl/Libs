/*
 * keys.cpp
 *
 *  Created on: 07.02.2013
 *      Author: kreyl
 */

#include "buttons.h"
#include "ch.h"
#include "evt_mask.h"
#include "uart.h"
#include "main.h" // App.Thread is here

#if SIMPLESENSORS_ENABLED

CircBuf_t<BtnEvtInfo_t, BTNS_EVT_Q_LEN> EvtBuf;

// ==== Inner use ====
#if BTN_LONGPRESS
static bool IsLongPress[BUTTONS_CNT];
static systime_t LongPressTimer;
#endif
#if BTN_REPEAT
static bool IsRepeating[BUTTONS_CNT];
static systime_t RepeatTimer;
#endif
//static systime_t RepeatTimer, LongPressTimer;
#if BTN_COMBO
    bool IsCombo;
#endif
void AddEvtToQueue(BtnEvtInfo_t Evt);
void AddEvtToQueue(BtnEvt_t AType, uint8_t KeyIndx);

// ========================= Postprocessor for PinSns ==========================
void ProcessButtons(PinSnsState_t *BtnState, uint32_t Len) {
//    Uart.Printf("\r%A", BtnState, Len, ' ');
    for(uint8_t i=0; i<BUTTONS_CNT; i++) {
        // ==== Button Press ====
        if(BtnState[i] == BTN_PRESS_STATE) {
#if BTN_LONGPRESS
            IsLongPress[i] = false;
#endif
#if BTN_REPEAT
            IsRepeating[i] = false;
#endif
#if BTN_COMBO // Check if combo
            BtnEvtInfo_t IEvt;
            IEvt.BtnCnt = 0;
            for(uint8_t j=0; j<BUTTONS_CNT; j++) {
                if(BtnState[j] == BTN_HOLDDOWN_STATE or BtnState[j] == BTN_PRESS_STATE) {
                    IEvt.BtnID[IEvt.BtnCnt] = j;
                    IEvt.BtnCnt++;
                    if((j != i) and !IsCombo) {
                        IsCombo = true;
                        AddEvtToQueue(beCancel, j);
                    }
                }
            } // for j
            if(IEvt.BtnCnt > 1) { // Combo
                IEvt.Type = beCombo;
                AddEvtToQueue(IEvt);
                continue;
            }
            else IsCombo = false;
#endif // combo
            // Single key pressed, no combo
            AddEvtToQueue(bePress, i);  // Add single keypress
#if BTN_LONGPRESS
            LongPressTimer = chVTGetSystemTimeX();
#endif
#if BTN_REPEAT
            RepeatTimer = chTimeNow();
#endif
        } // if press

        // ==== Button Release ====
#if BTN_COMBO || BTN_RELEASE
        else if(BtnState[i] == BTN_RELEASE_STATE) {
#if BTN_COMBO // Check if combo completely released
            if(IsCombo) {
                IsCombo = false;
                for(uint8_t j=0; j<BUTTONS_CNT; j++) {
                    if(BtnState[j] == BTN_HOLDDOWN_STATE) {
                        IsCombo = true;
                        break;
                    }
                }
                continue; // do not send release evt (if enabled)
            } // if combo
#endif
#if BTN_RELEASE // Send evt if not combo
            AddEvtToQueue(beRelease, i);
#endif
        }
#endif // if combo or release

        // ==== Long Press ====
#if BTN_LONGPRESS || BTN_REPEAT
        else if(BtnState[i] == BTN_HOLDDOWN_STATE
#if BTN_COMBO
                and !IsCombo
#endif
                ) {
#if BTN_LONGPRESS // Check if long press
            if(!IsLongPress[i]) {
//                Uart.Printf("Elapsed %u\r", chVTTimeElapsedSinceX(LongPressTimer));
                if(chVTTimeElapsedSinceX(LongPressTimer) >= MS2ST(BTN_LONGPRESS_DELAY_MS)) {
                    IsLongPress[i] = true;
                    AddEvtToQueue(beLongPress, i);
                }
            }
#endif
#if BTN_REPEAT // Check if repeat
            if(!IsRepeating[i]) {
                if(TimeElapsed(&RepeatTimer, BTN_DELAY_BEFORE_REPEAT_MS)) {
                    IsRepeating[i] = true;
                    AddEvtToQueue(beRepeat, i);
                }
            }
            else {
                if(TimeElapsed(&RepeatTimer, BTN_REPEAT_PERIOD_MS)) {
                    AddEvtToQueue(beRepeat, i);
                }
            }
#endif
        } // if still pressed
#endif
    } // for i
}

void AddEvtToQueue(BtnEvtInfo_t Evt) {
    chSysLock();
    EvtBuf.Put(&Evt);
    App.SignalEvtI(EVT_BUTTONS);
    chSysUnlock();
}

void AddEvtToQueue(BtnEvt_t AType, uint8_t KeyIndx) {
    BtnEvtInfo_t IEvt;
    IEvt.Type = AType;
#if BTN_COMBO
    IEvt.BtnCnt = 1;
#endif
#if BUTTONS_CNT != 1
    IEvt.BtnID[0] = KeyIndx;
#endif
    chSysLock();
    EvtBuf.Put(&IEvt);
    App.SignalEvtI(EVT_BUTTONS);
    chSysUnlock();
}

uint8_t BtnGetEvt(BtnEvtInfo_t *PEvt) {
    return(EvtBuf.Get(PEvt));
}
#endif
