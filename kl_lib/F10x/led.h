/*
 * led.h
 *
 *  Created on: 03.11.2011
 *      Author: g.kruglov
 */

#ifndef LED_H_
#define LED_H_

#include "stm32f10x.h"
#include "ch.h"
#include "hal.h"
#include "kl_lib_f100.h"
#include "color.h"
#include "main.h"

#if LED_RGB_ENABLE
/*
 * RGB LED
 * Provides smooth color change.
 */
#define LED_SETUP_DELAY_MS  45

#define LED_TIM         TIM2
#define LED_RED_CCR     CCR4
#define LED_GREEN_CCR   CCR3
#define LED_BLUE_CCR    CCR2
#define LED_RCC_EN()    rccEnableTIM2()

class LedRGB_t {
private:
	Color_t ICurrentColor, INeededColor;
	Thread *PThread;
	bool IsSleeping;
	void ISetRed  (uint8_t AValue) {LED_TIM->LED_RED_CCR   = AValue;}
	void ISetGreen(uint8_t AValue) {LED_TIM->LED_GREEN_CCR = AValue;}
	void ISetBlue (uint8_t AValue) {LED_TIM->LED_BLUE_CCR  = AValue;}
public:
	void Init();
	void Task();
	void SetColor(Color_t AColor) {
		ISetRed(AColor.Red);
		ISetGreen(AColor.Green);
		ISetBlue(AColor.Blue);
		ICurrentColor = AColor;
	}
	void SetColorSmoothly(Color_t AColor);
	void SetColorNow(Color_t AColor) {
	    SetColor(AColor);
	    INeededColor = AColor;
	}
	bool IsOff() { return (ICurrentColor == INeededColor) and (ICurrentColor == clBlack); }
//	void Blink(uint32_t ABlinkDelay, Color_t AColor) {
//	        IsInsideBlink = true;
//	        IBlinkDelay = ABlinkDelay;
//	        INeededColor = AColor;
//	        SetColor(AColor);
//	        Delay.Reset(&ITimer);
//	    }
};

extern LedRGB_t Led;
#endif

#endif /* LED_H_ */
