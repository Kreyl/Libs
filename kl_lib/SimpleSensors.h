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
#include "PinSnsSettings.h"

#if SIMPLESENSORS_ENABLED
class SimpleSensors_t {
private:
    PinSnsState_t States[PIN_SNS_CNT];
public:
    void Init();
    void Shutdown() { for(uint32_t i=0; i<PIN_SNS_CNT; i++) PinSns[i].Off(); }
    // Inner use
    void ITask();
};

extern SimpleSensors_t PinSensors;
#endif
