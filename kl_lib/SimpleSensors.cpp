/*
 * SimpleSensors.cpp
 *
 *  Created on: 17 џэт. 2015 у.
 *      Author: Kreyl
 */

#include "SimpleSensors.h"

#if SIMPLESENSORS_ENABLED

SimpleSensors_t PinSensors;

// ==== Sensors Thread ====
static THD_WORKING_AREA(waPinSnsThread, 64);
__noreturn
static void SensorsThread(void *arg) {
    chRegSetThreadName("PinSensors");
    PinSensors.ITask();
}

void SimpleSensors_t::Init() {
    // Init pins
    for(uint32_t i=0; i < PIN_SNS_CNT; i++) {
        PinSns[i].Init();
        States[i] = pssLo;
    }
    // Create and start thread
    chThdCreateStatic(waPinSnsThread, sizeof(waPinSnsThread), (tprio_t)90, (tfunc_t)SensorsThread, NULL);
}

__noreturn
void SimpleSensors_t::ITask() {
    while(true) {
        chThdSleepMilliseconds(SNS_POLL_PERIOD_MS);
        ftVoidPSnsStLen PostProcessor = PinSns[0].Postprocessor;
        uint32_t GroupLen = 0;
        PinSnsState_t *PStates = &States[0];
        // ==== Iterate pins ====
        uint32_t i=0;
        while(i < PIN_SNS_CNT) {
            // Check pin
            if(PinSns[i].IsHi()) {
                if(States[i] == pssLo or States[i] == pssFalling) States[i] = pssRising;
                else States[i] = pssHi;
            }
            else { // is low
                if(States[i] == pssHi or States[i] == pssRising) States[i] = pssFalling;
                else States[i] = pssLo;
            }
            GroupLen++;
            // Call postprocessor if this was last pin in group (or last at all)
            i++;
            if((i >= PIN_SNS_CNT) or (PinSns[i].Postprocessor != PostProcessor)) {
                if(PostProcessor != nullptr) PostProcessor(PStates, GroupLen);
                // Prepare for next group
                if(i < PIN_SNS_CNT) {
                    PostProcessor = PinSns[i].Postprocessor;
                    PStates = &States[i];
                }
                GroupLen = 0;
            }
        } // while i
    } // while true
}

#endif
