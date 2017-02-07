/*
 * SnsPins.h
 *
 *  Created on: 17 џэт. 2015 у.
 *      Author: Kreyl
 */

/* ================ Documentation =================
 * There are several (may be 1) groups of sensors (say, buttons and USB connection).
 * There is GPIO and Pin data for every sensor.
 *
 */

#pragma once

#include "ch.h"
#include "hal.h"
#include "kl_lib.h"

#define SIMPLESENSORS_ENABLED   TRUE
#define SNS_POLL_PERIOD_MS      63

#if SIMPLESENSORS_ENABLED
enum PinSnsState_t {pssLo, pssHi, pssRising, pssFalling};
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
};

// ================================= Settings ==================================
// Button handler
extern void ProcessButtons(PinSnsState_t *BtnState, uint32_t Len);
// Charge handler
extern void ProcessChargePin(PinSnsState_t *State, uint32_t Len);

const PinSns_t PinSns[] = {
        // Buttons
        {BTN_UP_PIN,   ProcessButtons},
        {BTN_DOWN_PIN, ProcessButtons},
        // Charging
        {CHARGE_PIN,   ProcessChargePin}
};
#define PIN_SNS_CNT     countof(PinSns)

#endif  // if enabled
