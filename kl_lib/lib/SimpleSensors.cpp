/*
 * SimpleSensors.cpp
 *
 *  Created on: 17 џэт. 2015 у.
 *      Author: Kreyl
 */

#include "SimpleSensors.h"
#include "uart.h"

#if SIMPLESENSORS_ENABLED
#include "PinSnsSettings.h"

static PinSnsState_t States[PIN_SNS_CNT];

static THD_WORKING_AREA(waPinSnsThread, 128);
__noreturn
static void SensorsThread(void *arg) {
    chRegSetThreadName("PinSensors");
    while(true) {
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

            // Switch to another group if postprocessor changed
            if(PinSns[i].Postprocessor != PostProcessor) {
                if(PostProcessor != nullptr) PostProcessor(PStates, GroupLen);
                // Prepare for next group
                PostProcessor = PinSns[i].Postprocessor;
                GroupLen = 0;
                PStates = &States[i];
            }
            else GroupLen++;    // else increase group len
            i++;                // Get next pin
        } // while i
        // Execute postprocessor for last group
        if(PostProcessor != nullptr) PostProcessor(PStates, GroupLen);
        chThdSleepMilliseconds(SNS_POLL_PERIOD_MS);
    } // while true
}


namespace SimpleSensors {

void Init() {
    // Init pins
    for(uint32_t i=0; i < PIN_SNS_CNT; i++) {
        PinSns[i].Init();
        States[i] = pssNone;
    }
    // Create and start thread
    chThdCreateStatic(waPinSnsThread, sizeof(waPinSnsThread), NORMALPRIO, (tfunc_t)SensorsThread, NULL);
}

void Shutdown() {
    for(uint32_t i=0; i<PIN_SNS_CNT; i++) PinSns[i].Off();
}

} // namespace
#endif
