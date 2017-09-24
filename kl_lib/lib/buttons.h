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

#ifndef BUTTONS_ENABLED
#define BUTTONS_ENABLED FALSE
#endif

#if BUTTONS_ENABLED

/*
 * Example:
            case evtIdButtons:
                Printf("Btn %u\r", Msg.BtnEvtInfo.BtnID);
                if(Msg.BtnEvtInfo.BtnID == 1) {
 */

// ================================= Settings ==================================
#define BUTTONS_CNT                 2
// Select required events etc.
#define BTN_SHORTPRESS              FALSE    // beShortPress evt
#define BTN_RELEASE                 TRUE
#define BTN_LONGPRESS               TRUE    // Send LongPress evt
#define BTN_REPEAT                  FALSE   // Send Repeat evt
#define BTN_COMBO                   FALSE    // Allow combo
#define BTN_LONG_COMBO              TRUE    // Allow longpress combo
#define BTN_GETSTATE_REQUIRED       TRUE

#define BTN_REPEAT_PERIOD_MS        180
#define BTN_LONGPRESS_DELAY_MS      999
#define BTN_DELAY_BEFORE_REPEAT_MS  450

// Define correct button behavior depending on schematic
#define BTN_IDLE_LOW                FALSE
// =============================================================================

// Selected depending on Idle state
#if BTN_IDLE_LOW // Change this
// Do not change this
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
enum BtnEvt_t {beShortPress, beLongPress, beRelease, beRepeat, beCombo, beLongCombo};
struct BtnEvtInfo_t {
    BtnEvt_t Type;
#if BTN_COMBO || BTN_LONG_COMBO
    uint8_t BtnCnt;
    uint8_t BtnID[BUTTONS_CNT];
#elif BUTTONS_CNT != 1
    uint8_t BtnID;
#endif
} __packed;

PinSnsState_t GetBtnState(uint8_t BtnID);
#endif
