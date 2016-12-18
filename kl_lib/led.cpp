/*
 * led.cpp
 *
 *  Created on: 9 дек. 2016 г.
 *      Author: Kreyl
 */

#include "led.h"

void LedSmoothTmrCallback(void *p) {
    chSysLockFromISR();
    ((LedSmooth_t*)p)->IProcessSequenceI();
    chSysUnlockFromISR();
}

void LedRGBTmrCallback(void *p) {
    chSysLockFromISR();
    ((LedRGB_t*)p)->IProcessSequenceI();
    chSysUnlockFromISR();
}
