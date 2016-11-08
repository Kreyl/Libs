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

#include "PinSnsSettings.h"
#include "SimpleSensors.h"

#if SIMPLESENSORS_ENABLED

/*
 * Example:
if(EvtMsk & EVTMSK_BUTTONS) {
    BtnEvtInfo_t EInfo;
    while(BtnGetEvt(&EInfo) == OK) {
        if(EInfo.Type == bePress) {

        }
        else if(EInfo.Type == beLongPress) {

        }
    }
 */

#define BUTTONS_CNT     1
// Select required events. BtnPress is a must.
#define BTN_RELEASE     FALSE
#define BTN_LONGPRESS   FALSE   // Send LongPress evt
#define BTN_REPEAT      FALSE   // Send Repeat evt
#define BTN_COMBO       FALSE   // Allow combo

#define BTN_REPEAT_PERIOD_MS        180
#define BTN_LONGPRESS_DELAY_MS      2007
#define BTN_DELAY_BEFORE_REPEAT_MS  (BTN_REPEAT_PERIOD_MS + BTN_LONGPRESS_DELAY_MS)

#if BTN_COMBO
#define BTNS_EVT_Q_LEN              7   // Length of events' query
#else
#define BTNS_EVT_Q_LEN              1   // No need in queue if combo not allowed
#endif

// Select convenient names
enum BtnName_t {btnSelect=0, btnPlus=1, btnMinus=2};

// Define correct button behavior depending on schematic
#define BTN_PRESS_STATE         pssRising
#define BTN_RELEASE_STATE       pssFalling
#define BTN_HOLDDOWN_STATE      pssHi

// ==== Types ==== Do not touch
// BtnEvent: contains info about event type, count of participating btns and array with btn IDs
enum BtnEvt_t {bePress, beLongPress, beRelease, beCancel, beRepeat, beCombo};
struct BtnEvtInfo_t {
    BtnEvt_t Type;
#if BTN_COMBO
    uint8_t BtnCnt;
#endif
#if BUTTONS_CNT != 1
    uint8_t BtnID[BUTTONS_CNT];
#endif
} __packed;

uint8_t BtnGetEvt(BtnEvtInfo_t *PEvt);
#endif
