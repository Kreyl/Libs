/*
 * led.cpp
 *
 *  Created on: 03.11.2011
 *      Author: g.kruglov
 */

#include "led.h"

#if LED_RGB_ENABLE

LedRGB_t Led;

// ==== LED Thread ====
static WORKING_AREA(waLedThread, 128);
static void LedThread(void *arg) {
    chRegSetThreadName("Lcd");
    while(true) Led.Task();
}

void LedRGB_t::Init() {
    // ==== GPIO ====
	PinSetupAlterFuncOutput(GPIOA, 1, omPushPull);
	PinSetupAlterFuncOutput(GPIOA, 2, omPushPull);
	PinSetupAlterFuncOutput(GPIOA, 3, omPushPull);
    // ==== Timer ====
	LED_RCC_EN();
    // ==== Timebase and general ====
	LED_TIM->CR1 = 0x01;       // Enable timer, set clk division to 0, AutoReload not buffered
	LED_TIM->CR2 = 0;          // Output Idle State
	LED_TIM->PSC = 0;          // No clock division
	LED_TIM->ARR = 255;        // Autoreload register: top value of PWM
    // ==== Outputs ====
	//LED_TIM->BDTR = 0x8000;    // Enable output
	const uint16_t OutCmpBits = 0b01100000;   // output, PWM1
	LED_TIM->CCMR1 = OutCmpBits << 8;   // CCR2
	LED_TIM->CCMR2 = (OutCmpBits << 8) | OutCmpBits;
	LED_TIM->CCER = TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;
    // Initial values
    SetColor(clBlack);
    INeededColor = clBlack;
    // Thread
    PThread = chThdCreateStatic(waLedThread, sizeof(waLedThread), NORMALPRIO, (tfunc_t)LedThread, NULL);
}

void LedRGB_t::SetColorSmoothly(Color_t AColor) {
    INeededColor = AColor;
    if(IsSleeping) {
        chSysLock();
        chSchWakeupS(PThread, 0);
        IsSleeping = false;
        chSysUnlock();
    }
}

void LedRGB_t::Task(void) {
    if(ICurrentColor == INeededColor) {
        chSysLock();
        IsSleeping = true;
        chSchGoSleepS(THD_STATE_SUSPENDED);
        chSysUnlock();
    }
    else {
        chThdSleepMilliseconds(LED_SETUP_DELAY_MS);
        // Red channel
        if (ICurrentColor.Red != INeededColor.Red) {
            if(INeededColor.Red < ICurrentColor.Red) ICurrentColor.Red--;
            else ICurrentColor.Red++;
        }
        // Green channel
        if (ICurrentColor.Green != INeededColor.Green) {
            if(INeededColor.Green < ICurrentColor.Green) ICurrentColor.Green--;
            else ICurrentColor.Green++;
        }
        // Blue channel
        if (ICurrentColor.Blue != INeededColor.Blue) {
            if(INeededColor.Blue < ICurrentColor.Blue) ICurrentColor.Blue--;
            else ICurrentColor.Blue++;
        }
        SetColor(ICurrentColor);
    }
}
#endif
