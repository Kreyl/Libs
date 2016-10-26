/*
 * kl_servo.h
 *
 *  Created on: 10 окт. 2016 г.
 *      Author: Kreyl
 */

#pragma once

#include "kl_lib.h"

/*
 * Example:
 * #define SERVO1_PIN      GPIOC, 9, TIM3, 4
 * Servo_t Servo1 {SERVO1_PIN, 544, 2380};
 * Servo1.Init();
 *
    else if(PCmd->NameIs("Set")) {
        if(PCmd->GetNextInt32(&dw32) != OK) PShell->Ack(FAILURE);
        Servo1.Set_us(dw32);
    }

    else if(PCmd->NameIs("Angle")) {
        if(PCmd->GetNextInt32(&dw32) != OK) PShell->Ack(FAILURE);
        Servo1.SetAngle_dg(dw32);
    }
 */

class Servo_t : private PinOutputPWM_t {
private:
    const uint32_t imin_us, imax_us;

public:
    void Init() const {
        PinOutputPWM_t::Init();
        PinOutputPWM_t::SetFrequencyHz(50);
        Set_us(imin_us);
//        Release();
    }
    void Set_us(uint32_t uS, uint32_t ReleaseAfter_ms = 0) const {
        PinOutputPWM_t::Set(uS);
    }

    void SetAngle_dg(uint32_t Angle, uint32_t ReleaseAfter_ms = 0) const {
        uint32_t us = Proportion<uint32_t>(Angle, 0, 180, imin_us, imax_us);
        Set_us(us, ReleaseAfter_ms);
    }

    void Release() const { PinOutputPWM_t::Set(0); }

    Servo_t(GPIO_TypeDef *PGpio, uint16_t Pin, TIM_TypeDef *PTimer, uint32_t TimerChnl,
            uint32_t min_us = 544, uint32_t max_us = 2400) :
                // Setup timer, making 1 tick = 1us
                PinOutputPWM_t(PGpio, Pin, PTimer, TimerChnl, invNotInverted, omPushPull, 19999),
                imin_us(min_us), imax_us(max_us) {}
};
