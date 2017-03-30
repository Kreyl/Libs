/*
 * Sequences.h
 *
 *  Created on: 09 џэт. 2015 у.
 *      Author: Kreyl
 */

#pragma once

#include "ChunkTypes.h"

#if 0 // ============================ LED RGB blink ============================
const LedChunk_t lsqIdle[] = {
        {csSetup, 0, clBlack},
        {csEnd}
};

const LedChunk_t lsqError[] = {
        {csSetup, 0, clRed},
        {csWait, 4005},
        {csSetup, 0, clBlack},
        {csEnd}
};

// ======= Adding / removing IDs ========
// ==== Access ====
#define LSQ_ACCESS_ADD_CLR      clGreen
#define LSQ_ACCESS_REMOVE_CLR   clRed
const LedChunk_t lsqAddingAccessWaiting[] = {
        {csSetup, 0, LSQ_ACCESS_ADD_CLR},
        {csEnd}
};
const LedChunk_t lsqAddingAccessNew[] = {
        {csSetup, 0, clBlack},
        {csWait, 180},
        {csSetup, 0, LSQ_ACCESS_ADD_CLR},
        {csEnd}
};
const LedChunk_t lsqAddingAccessError[] = {
        {csSetup, 0, clRed},
        {csWait, 99},
        {csSetup, 0, clBlack},
        {csWait, 99},
        {csSetup, 0, clRed},
        {csWait, 99},
        {csSetup, 0, clBlack},
        {csWait, 99},
        {csSetup, 0, clRed},
        {csWait, 99},
        {csSetup, 0, clBlack},
        {csWait, 99},
        {csSetup, 0, LSQ_ACCESS_ADD_CLR},
        {csEnd}
};

const LedChunk_t lsqRemovingAccessWaiting[] = {
        {csSetup, 0, LSQ_ACCESS_REMOVE_CLR},
        {csEnd}
};
const LedChunk_t lsqRemovingAccessNew[] = {
        {csSetup, 0, clBlack},
        {csWait, 180},
        {csSetup, 0, LSQ_ACCESS_REMOVE_CLR},
        {csEnd}
};

// ==== Adder ====
#define LSQ_ADDER_ADD_CLR       clBlue
#define LSQ_ADDER_REMOVE_CLR    clMagenta
const LedChunk_t lsqAddingAdderWaiting[] = {
        {csSetup, 0, LSQ_ADDER_ADD_CLR},
        {csEnd}
};
const LedChunk_t lsqAddingAdderNew[] = {
        {csSetup, 0, clBlack},
        {csWait, 180},
        {csSetup, 0, LSQ_ADDER_ADD_CLR},
        {csEnd}
};
const LedChunk_t lsqAddingAdderError[] = {
        {csSetup, 0, clRed},
        {csWait, 99},
        {csSetup, 0, clBlack},
        {csWait, 99},
        {csSetup, 0, clRed},
        {csWait, 99},
        {csSetup, 0, clBlack},
        {csWait, 99},
        {csSetup, 0, clRed},
        {csWait, 99},
        {csSetup, 0, clBlack},
        {csWait, 99},
        {csSetup, 0, LSQ_ADDER_ADD_CLR},
        {csEnd}
};

const LedChunk_t lsqRemovingAdderWaiting[] = {
        {csSetup, 0, LSQ_ADDER_REMOVE_CLR},
        {csEnd}
};
const LedChunk_t lsqRemovingAdderNew[] = {
        {csSetup, 0, clBlack},
        {csWait, 180},
        {csSetup, 0, LSQ_ADDER_REMOVE_CLR},
        {csEnd}
};

// ==== Remover ====
#define LSQ_REMOVER_ADD_CLR     clCyan
#define LSQ_REMOVER_REMOVE_CLR  clYellow
const LedChunk_t lsqAddingRemoverWaiting[] = {
        {csSetup, 0, LSQ_REMOVER_ADD_CLR},
        {csEnd}
};
const LedChunk_t lsqAddingRemoverNew[] = {
        {csSetup, 0, clBlack},
        {csWait, 180},
        {csSetup, 0, LSQ_REMOVER_ADD_CLR},
        {csEnd}
};
const LedChunk_t lsqAddingRemoverError[] = {
        {csSetup, 0, clRed},
        {csWait, 99},
        {csSetup, 0, clBlack},
        {csWait, 99},
        {csSetup, 0, clRed},
        {csWait, 99},
        {csSetup, 0, clBlack},
        {csWait, 99},
        {csSetup, 0, clRed},
        {csWait, 99},
        {csSetup, 0, clBlack},
        {csWait, 99},
        {csSetup, 0, LSQ_REMOVER_ADD_CLR},
        {csEnd}
};

const LedChunk_t lsqRemovingRemoverWaiting[] = {
        {csSetup, 0, LSQ_REMOVER_REMOVE_CLR},
        {csEnd}
};
const LedChunk_t lsqRemovingRemoverNew[] = {
        {csSetup, 0, clBlack},
        {csWait, 180},
        {csSetup, 0, LSQ_REMOVER_REMOVE_CLR},
        {csEnd}
};

// ==== Erase all ====
const LedChunk_t lsqEraseAll[] = {
        {csSetup, 0, clRed},
        {csWait, 180},
        {csSetup, 0, clBlack},
        {csWait, 180},
        {csSetup, 0, clRed},
        {csWait, 180},
        {csSetup, 0, clBlack},
        {csWait, 180},
        {csSetup, 0, clRed},
        {csWait, 180},
        {csSetup, 0, clBlack},
        {csWait, 180},
        {csSetup, 0, clRed},
        {csWait, 180},
        {csSetup, 0, clBlack},
        {csEnd}
};

// General
const LedChunk_t lsqBlinkGreen[] = {
        {csSetup, 0, clGreen},
        {csWait, 180},
        {csSetup, 0, clBlack},
        {csEnd}
};

const LedChunk_t lsqBlinkGreenX2[] = {
        {csSetup, 0, clGreen},
        {csWait, 180},
        {csSetup, 0, clBlack},
        {csWait, 180},
        {csSetup, 0, clGreen},
        {csWait, 180},
        {csSetup, 0, clBlack},
//        {csWait, 999},
//        {csGoto, 0}
        {csEnd}
};
#endif

#if 1 // ============================ LED RGB ==================================
const LedRGBChunk_t lsqStart[] = {
        {csSetup, 99, clRed},
        {csSetup, 99, clGreen},
        {csSetup, 99, clBlue},
        {csSetup, 0, clBlack},
        {csEnd}
};

const LedRGBChunk_t lsqFailure[] = {
        {csSetup, 0, clRed},
        {csWait, 99},
        {csSetup, 0, clBlack},
        {csWait, 99},
        {csSetup, 0, clRed},
        {csWait, 99},
        {csSetup, 0, clBlack},
        {csWait, 99},
        {csSetup, 0, clRed},
        {csWait, 99},
        {csSetup, 0, clBlack},
        {csEnd}
};

//const LedRGBChunk_t lsqCharging[] = {
//        {csSetup, 540, {0,9,0}},
//        {csSetup, 540, clBlack},
//        {csWait, 900},
//        {csGoto, 0}
//};
//
//const LedRGBChunk_t lsqChargingDone[] = {
//        {csSetup, 0, {0,9,0}},
//        {csEnd}
//};
//
//const LedRGBChunk_t lsqDischarged[] = {
//        {csSetup, 0, clRed},
//        {csWait, 180},
//        {csSetup, 0, clBlack},
//        {csWait, 360},
//        {csGoto, 0}
//};
#endif

#if 1 // ======================== Simple LED blink =============================
#define BLINK_DELAY_MS      180
const BaseChunk_t lbsqBlink3[] = {
        {csSetup, 1},
        {csWait, BLINK_DELAY_MS},
        {csSetup, 0},
        {csWait, BLINK_DELAY_MS},
        {csSetup, 1},
        {csWait, BLINK_DELAY_MS},
        {csSetup, 0},
        {csWait, BLINK_DELAY_MS},
        {csSetup, 1},
        {csWait, BLINK_DELAY_MS},
        {csSetup, 0},
        {csEnd}
};
#endif

#if 0 // =========================== LED Smooth ================================
#define LED_TOP_BRIGHTNESS  255

const LedSmoothChunk_t lsqSmoothStart[] = {
        {csSetup, 207, LED_TOP_BRIGHTNESS},
        {csSetup, 207, 0},
        {csEnd}
};

const LedSmoothChunk_t lsqSmoothPrepare[] = {
        {csSetup, 207, LED_TOP_BRIGHTNESS},
        {csSetup, 207, 4},
        {csGoto, 0}
};

const LedSmoothChunk_t lsqSmoothReload[] = {
        {csSetup, 0, 54},
        {csWait, 99},
        {csSetup, 0, 4},
        {csWait, 99},
        {csGoto, 0}
};

const LedSmoothChunk_t lsqSmoothHit[] = {
        {csSetup, 0, LED_TOP_BRIGHTNESS},
        {csWait, 99},
        {csSetup, 0, 0},
        {csWait, 360},
        {csEnd}
};

const LedSmoothChunk_t lsqSmoothDamaged[] = {
        {csSetup, 0, 108},
        {csEnd}
};

const LedSmoothChunk_t lsqFire[] = {
        {csSetup, 0, LED_TOP_BRIGHTNESS},
        {csWait, 99},
        {csSetup, 54, 0},
        {csEnd}
};

const LedSmoothChunk_t lsqDamageEndOff[] = {
        {csSetup, 0, LED_TOP_BRIGHTNESS},
        {csWait, 99},
        {csSetup, 180, 0},
        {csEnd}
};

const LedSmoothChunk_t lsqDamageEndLow[] = {
        {csSetup, 0, 0},
        {csWait, 99},
        {csSetup, 0, LED_TOP_BRIGHTNESS},
        {csWait, 99},
        {csSetup, 180, 90},
        {csEnd}
};

const LedSmoothChunk_t lsqSteady[] = {
        {csSetup, 0, LED_TOP_BRIGHTNESS},
        {csEnd}
};
const LedSmoothChunk_t lsqSteadyLow[] = {
        {csSetup, 0, 18},
        {csEnd}
};
const LedSmoothChunk_t lsqOff[] = {
        {csSetup, 0, 0},
        {csEnd}
};

//const LedSmoothChunk_t lsqFadeOut[] = {
//        {csSetup, 630, 0},
//        {csEnd}
//};
//const LedSmoothChunk_t lsqEnterActive[] = {
//        {csSetup, 0, LED_TOP_BRIGHTNESS},
//        {csEnd}
//};
//const LedSmoothChunk_t lsqEnterIdle[] = {
//        {csSetup, 360, 0},
//        {csEnd}
//};

#endif

#if 0 // ============================= Beeper ==================================
#define BEEP_VOLUME     1   // Maximum 10

#if 1 // ==== Notes ====
#define La_2    880

#define Do_3    1047
#define Do_D_3  1109
#define Re_3    1175
#define Re_D_3  1245
#define Mi_3    1319
#define Fa_3    1397
#define Fa_D_3  1480
#define Sol_3   1568
#define Sol_D_3 1661
#define La_3    1720
#define Si_B_3  1865
#define Si_3    1976

#define Do_4    2093
#define Do_D_4  2217
#define Re_4    2349
#define Re_D_4  2489
#define Mi_4    2637
#define Fa_4    2794
#define Fa_D_4  2960
#define Sol_4   3136
#define Sol_D_4 3332
#define La_4    3440
#define Si_B_4  3729
#define Si_4    3951

// Length
#define OneSixteenth    90
#define OneEighth       (OneSixteenth * 2)
#define OneFourth       (OneSixteenth * 4)
#define OneHalfth       (OneSixteenth * 8)
#define OneWhole        (OneSixteenth * 16)
#endif

// MORSE
#define MORSE_TONE {csSetup, BEEP_VOLUME, Do_3}
#define MORSE_DOT_LENGTH 180
#define MORSE_DASH_LENGTH MORSE_DOT_LENGTH * 3
#define MORSE_PAUSE_LENGTH MORSE_DOT_LENGTH
#define MORSE_PAUSE {csSetup, 0}, {csWait, MORSE_PAUSE_LENGTH}
#define MORSE_DOT MORSE_TONE, {csWait, MORSE_DOT_LENGTH}, MORSE_PAUSE
#define MORSE_DASH MORSE_TONE, {csWait, MORSE_DASH_LENGTH}, MORSE_PAUSE

// Type, BEEP_VOLUME, freq
const BeepChunk_t bsqOn[] = {
        {csSetup, 10, 7000},
        {csEnd}
};

const BeepChunk_t bsqButton[] = {
        {csSetup, 1, 1975},
        {csWait, 54},
        {csSetup, 0},
        {csEnd}
};
const BeepChunk_t bsqBeepBeep[] = {
        {csSetup, BEEP_VOLUME, 1975},
        {csWait, 54},
        {csSetup, 0},
        {csWait, 54},
        {csSetup, BEEP_VOLUME, 1975},
        {csWait, 54},
        {csSetup, 0},
        {csEnd}
};

const BeepChunk_t bsqCharge[] = {
        MORSE_DOT, MORSE_DASH,
        {csEnd}
};
const BeepChunk_t bsqThrow[] = {
        MORSE_DOT,
        {csEnd}
};
const BeepChunk_t bsqPunch[] = {
        MORSE_DOT, MORSE_DASH, MORSE_DASH, MORSE_DOT,
        {csEnd}
};
const BeepChunk_t bsqLift[] = {
        MORSE_DOT, MORSE_DASH, MORSE_DOT, MORSE_DOT,
        {csEnd}
};
const BeepChunk_t bsqWarp[] = {
        MORSE_DOT, MORSE_DASH, MORSE_DASH,
        {csEnd}
};
const BeepChunk_t bsqBarrier[] = {
        MORSE_DASH, MORSE_DOT, MORSE_DOT, MORSE_DOT,
        {csEnd}
};
const BeepChunk_t bsqCleanse[] = {
        MORSE_DASH, MORSE_DOT, MORSE_DASH, MORSE_DOT,
        {csEnd}
};
const BeepChunk_t bsqSingular[] = {
        MORSE_DOT, MORSE_DOT, MORSE_DOT,
        {csEnd}
};
const BeepChunk_t bsqSong[] = {
        {csSetup, BEEP_VOLUME, Sol_3},
        {csWait, 360},
        {csSetup, BEEP_VOLUME, Mi_3},
        {csWait, 360},
        {csSetup, BEEP_VOLUME, Do_3},
        {csWait, 360},
        {csSetup, 0},
        {csEnd}
};
const BeepChunk_t bsqRelease[] = {
        MORSE_DOT, MORSE_DASH, MORSE_DOT,
        {csEnd}
};
const BeepChunk_t bsqPwrRelease[] = {
        MORSE_DASH, MORSE_DOT, MORSE_DOT, MORSE_DASH,
        {csEnd}
};


#if 1 // ==== Extensions ====
// Pill
const BeepChunk_t bsqBeepPillOk[] = {
        {csSetup, BEEP_VOLUME, Si_3},
        {csWait, 180},
        {csSetup, BEEP_VOLUME, Re_D_4},
        {csWait, 180},
        {csSetup, BEEP_VOLUME, Fa_D_4},
        {csWait, 180},
        {csSetup, 0},
        {csEnd}
};

const BeepChunk_t bsqBeepPillBad[] = {
        {csSetup, BEEP_VOLUME, Fa_4},
        {csWait, 180},
        {csSetup, BEEP_VOLUME, Re_4},
        {csWait, 180},
        {csSetup, BEEP_VOLUME, Si_3},
        {csWait, 180},
        {csSetup, 0},
        {csEnd}
};
#endif // ext
#endif // beeper

#if 0 // ============================== Vibro ==================================
#define VIBRO_VOLUME        100

#define VIBRO_SHORT_MS      99

const BaseChunk_t vsqBrr[] = {
        {csSetup, VIBRO_VOLUME},
        {csWait, VIBRO_SHORT_MS},
        {csSetup, 0},
        {csEnd}
};

const BaseChunk_t vsqBrrBrr[] = {
        {csSetup, VIBRO_VOLUME},
        {csWait, VIBRO_SHORT_MS},
        {csSetup, 0},
        {csWait, 99},
        {csSetup, VIBRO_VOLUME},
        {csWait, VIBRO_SHORT_MS},
        {csSetup, 0},
        {csEnd}
};

const BaseChunk_t vsqDischarged[] = {
        {csSetup, VIBRO_VOLUME},
        {csWait, VIBRO_SHORT_MS},
        {csSetup, 0},
        {csWait, 99},
        {csSetup, VIBRO_VOLUME},
        {csWait, VIBRO_SHORT_MS},
        {csSetup, 0},
        {csWait, 99},
        {csSetup, VIBRO_VOLUME},
        {csWait, VIBRO_SHORT_MS},
        {csSetup, 0},
        {csEnd}
};

// Gestures
#define VMORSE_TONE         {csSetup, VIBRO_VOLUME}
#define VMORSE_DOT_LENGTH   180
#define VMORSE_DASH_LENGTH  VMORSE_DOT_LENGTH * 3
#define VMORSE_PAUSE_LENGTH VMORSE_DOT_LENGTH
#define VMORSE_PAUSE        {csSetup, 0}, {csWait, VMORSE_PAUSE_LENGTH}
#define VMORSE_DOT          VMORSE_TONE, {csWait, VMORSE_DOT_LENGTH}, VMORSE_PAUSE
#define VMORSE_DASH         VMORSE_TONE, {csWait, VMORSE_DASH_LENGTH}, VMORSE_PAUSE


const BaseChunk_t vsqCharge[] = {
        VMORSE_DOT, VMORSE_DASH,
        {csEnd}
};
const BaseChunk_t vsqThrow[] = {
        VMORSE_DOT,
        {csEnd}
};
const BaseChunk_t vsqPunch[] = {
        VMORSE_DOT, VMORSE_DASH, VMORSE_DASH, VMORSE_DOT,
        {csEnd}
};
const BaseChunk_t vsqLift[] = {
        VMORSE_DOT, VMORSE_DASH, VMORSE_DOT, VMORSE_DOT,
        {csEnd}
};
const BaseChunk_t vsqWarp[] = {
        VMORSE_DOT, VMORSE_DASH, VMORSE_DASH,
        {csEnd}
};
const BaseChunk_t vsqBarrier[] = {
        VMORSE_DASH, VMORSE_DOT, VMORSE_DOT, VMORSE_DOT,
        {csEnd}
};
const BaseChunk_t vsqCleanse[] = {
        VMORSE_DASH, VMORSE_DOT, VMORSE_DASH, VMORSE_DOT,
        {csEnd}
};
const BaseChunk_t vsqSingular[] = {
        VMORSE_DOT, VMORSE_DOT, VMORSE_DOT,
        {csEnd}
};
const BaseChunk_t vsqSong[] = {
        {csSetup, 30},
        {csWait, 360},
        {csSetup, 60},
        {csWait, 360},
        {csSetup, 100},
        {csWait, 360},
        {csSetup, 0},
        {csEnd}
};
const BaseChunk_t vsqRelease[] = {
        VMORSE_DOT, VMORSE_DASH, VMORSE_DOT,
        {csEnd}
};
const BaseChunk_t vsqPwrRelease[] = {
        VMORSE_DASH, VMORSE_DOT, VMORSE_DOT, VMORSE_DASH,
        {csEnd}
};




/*
const BaseChunk_t vsqError[] = {
        {csSetup, VIBRO_VOLUME},
        {csWait, 999},
        {csSetup, 0},
        {csEnd}
};

const BaseChunk_t vsqSingle[] = {
        {csSetup, VIBRO_VOLUME},
        {csWait, VIBRO_SHORT_MS},
        {csSetup, 0},
        {csWait, 1800},
        {csGoto, 0}
};
const BaseChunk_t vsqPair[] = {
        {csSetup, VIBRO_VOLUME},
        {csWait, VIBRO_SHORT_MS},
        {csSetup, 0},
        {csWait, 99},
        {csSetup, VIBRO_VOLUME},
        {csWait, VIBRO_SHORT_MS},
        {csSetup, 0},
        {csWait, 1350},
        {csGoto, 0}
};
const BaseChunk_t vsqMany[] = {
        {csSetup, VIBRO_VOLUME},
        {csWait, VIBRO_SHORT_MS},
        {csSetup, 0},
        {csWait, 99},
        {csSetup, VIBRO_VOLUME},
        {csWait, VIBRO_SHORT_MS},
        {csSetup, 0},
        {csWait, 99},
        {csSetup, VIBRO_VOLUME},
        {csWait, VIBRO_SHORT_MS},
        {csSetup, 0},
        {csWait, 1008},
        {csGoto, 0}
};
*/
#endif
