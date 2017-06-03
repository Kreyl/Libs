/*
 * keys.cpp
 *
 *  Created on: 07.02.2013
 *      Author: kreyl
 */

#include "buttons.h"
#include "ch.h"
#include "uart.h"
#include "MsgQ.h"

#if BUTTONS_ENABLED

#if BTN_GETSTATE_REQUIRED
static PinSnsState_t IBtnState[BUTTONS_CNT];
PinSnsState_t GetBtnState(uint8_t BtnID) {
    if(BtnID > BUTTONS_CNT) return pssNone;
    else return IBtnState[BtnID];
}
#endif

#if BTN_LONGPRESS
static bool IsLongPress[BUTTONS_CNT];
static systime_t LongPressTimer;
#endif
#if BTN_REPEAT
static bool IsRepeating[BUTTONS_CNT];
static systime_t RepeatTimer;
#endif
#if BTN_COMBO
    bool IsCombo;
#endif
static void AddEvtToQueue(BtnEvtInfo_t &Evt);
static void AddEvtToQueue(BtnEvt_t AType, uint8_t KeyIndx);

// ========================= Postprocessor for PinSns ==========================
void ProcessButtons(PinSnsState_t *BtnState, uint32_t Len) {
//    Uart.Printf("  %S\r", __FUNCTION__);
//    Uart.Printf("\r%A", BtnState, Len, ' ');
    for(uint8_t i=0; i<BUTTONS_CNT; i++) {
#if BTN_GETSTATE_REQUIRED
        IBtnState[i] = BtnState[i];
#endif
        // ==== Button Press ====
        if(BtnState[i] == BTN_PRESSING_STATE) {
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

#if BTN_SHORTPRESS // Single key pressed, no combo
            AddEvtToQueue(beShortPress, i);  // Add single keypress
#endif

#if BTN_LONGPRESS
            LongPressTimer = chVTGetSystemTimeX();
#endif
#if BTN_REPEAT
            RepeatTimer = chVTGetSystemTimeX();
#endif
        } // if press

        // ==== Button Release ====
#if BTN_COMBO || BTN_RELEASE
        else if(BtnState[i] == BTN_RELEASING_STATE) {
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

__unused
void AddEvtToQueue(BtnEvtInfo_t &Evt) {
    EvtMsg_t Msg(evtIdButtons);
    Msg.BtnEvtInfo = Evt;
    EvtQMain.SendNowOrExit(Msg);
}

void AddEvtToQueue(BtnEvt_t AType, uint8_t KeyIndx) {
    EvtMsg_t Msg(evtIdButtons);
    Msg.BtnEvtInfo.Type = AType;
#if BTN_COMBO
    Msg.BtnEvtInfo.BtnCnt = 1;
    Msg.BtnEvtInfo.BtnID[0] = KeyIndx;
#elif BUTTONS_CNT != 1
    Msg.BtnEvtInfo.BtnID = KeyIndx;
#endif
    EvtQMain.SendNowOrExit(Msg);
}
#endif
