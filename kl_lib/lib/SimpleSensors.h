/*
 * SimpleSensors.h
 *
 *  Created on: 17 џэт. 2015 у.
 *      Author: Kreyl
 */

#pragma once

/*
 * Simple sensors are sensors with two logic states: Low and High.
 */

#include "hal.h"
#include "kl_lib.h"

enum PinSnsState_t {pssNone, pssLo, pssHi, pssRising, pssFalling};
typedef void (*ftVoidPSnsStLen)(PinSnsState_t *PState, uint32_t Len);

// Single pin setup data
struct PinSns_t {
    GPIO_TypeDef *PGpio;
    uint16_t Pin;
    PinPullUpDown_t Pud;
    ftVoidPSnsStLen Postprocessor;
    void Init() const { PinSetupInput(PGpio, Pin, Pud); }
    void Off()  const { PinSetupAnalog(PGpio, Pin);  }
    bool IsHi() const { return PinIsHi(PGpio, Pin); }
    PinSns_t(GPIO_TypeDef *APGpio, uint16_t APin, PinPullUpDown_t APud, ftVoidPSnsStLen APostprocessor) :
        PGpio(APGpio), Pin(APin), Pud(APud), Postprocessor(APostprocessor) {}
};

namespace SimpleSensors {
    void Init();
    void Shutdown();
};
