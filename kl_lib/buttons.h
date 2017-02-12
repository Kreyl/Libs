/*
 * buttons.h
 *
 *  Created on: 07.02.2013
 *      Author: kreyl
 */

#pragma once

#include "hal.h"
#include "kl_lib.h"
#include "kl_buf.h"

#include "SimpleSensors.h"

#if SIMPLESENSORS_ENABLED

/*
 * Example:
if(Evt & EVT_BUTTONS) {
    BtnEvtInfo_t EInfo;
    while(BtnGetEvt(&EInfo) == OK) {
        if(EInfo.Type == bePress) {

        }
        else if(EInfo.Type == beLongPress) {

        }
    }
 */

// ================================= Settings ==================================
//#define BUTTONS_CNT                 1
// Select required events etc.
#define BTN_SHORTPRESS              TRUE   // beShortPress evt
#define BTN_RELEASE                 FALSE
#define BTN_LONGPRESS               FALSE    // Send LongPress evt
#define BTN_REPEAT                  TRUE   // Send Repeat evt
#define BTN_COMBO                   FALSE   // Allow combo
#define BTN_GETSTATE_REQUIRED       FALSE

#define BTN_REPEAT_PERIOD_MS        450
#define BTN_LONGPRESS_DELAY_MS      999
#define BTN_DELAY_BEFORE_REPEAT_MS  (BTN_REPEAT_PERIOD_MS + BTN_LONGPRESS_DELAY_MS)

#if BTN_COMBO
#define BTNS_EVT_Q_LEN              7   // Length of events' query
#else
#define BTNS_EVT_Q_LEN              1   // No need in queue if combo not allowed
#endif

// Select convenient names
enum BtnName_t {btnUp=0, btnDown=1};

// Define correct button behavior depending on schematic
#define BTN_IDLE_LOW                FALSE
// =============================================================================

// Selected depending on Idle state
#if BTN_IDLE_LOW
#define BTN_IDLE_STATE              pssLo
#define BTN_HOLDDOWN_STATE          pssHi
#define BTN_PRESSING_STATE          pssRising
#define BTN_RELEASING_STATE         pssFalling
#else
#define BTN_IDLE_STATE              pssHi
#define BTN_HOLDDOWN_STATE          pssLo
#define BTN_PRESSING_STATE          pssFalling
#define BTN_RELEASING_STATE         pssRising
#endif

// ==== Types ==== Do not touch
// BtnEvent: contains info about event type, count of participating btns and array with btn IDs
enum BtnEvt_t {beShortPress, beLongPress, beRelease, beCancel, beRepeat, beCombo};
struct BtnEvtInfo_t {
    BtnEvt_t Type;
#if BTN_COMBO
    uint8_t BtnCnt;
    uint8_t BtnID[BUTTONS_CNT];
#elif BUTTONS_CNT != 1
    uint8_t BtnID;
#endif
} __packed;

uint8_t BtnGetEvt(BtnEvtInfo_t *PEvt);
PinSnsState_t GetBtnState(uint8_t BtnID);
#endif
