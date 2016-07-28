/*
 * gui.cpp
 *
 *  Created on: 13 ??? 2016 ?.
 *      Author: Kreyl
 */

#include <gui_engine.h>
#include "interface.h"
#include "fnt_Verdana27x27.h"

#include "stmpe811.h"
#include "uart.h"


Gui_t Gui;
ILI9341_t Lcd;
FrameBuffer_t<uint16_t, FRAMEBUFFER_LEN> FBuf;

extern const Page_t* Page[];

static THD_WORKING_AREA(waGuiThread, 256);
__noreturn
static THD_FUNCTION(GuiThread, arg) {
    chRegSetThreadName("Gui");
    Gui.ITask();
}

#if 1 // =========================== Implementation ============================
void Gui_t::Init() {
    CurrPage = Page[0];
    Lcd.Init();
    Touch.Init();
    chThdCreateStatic(waGuiThread, sizeof(waGuiThread), NORMALPRIO, GuiThread, NULL);
}

void Gui_t::DrawPage(uint8_t APage) {
    Lcd.Cls(clBlack);
    CurrPage = Page[APage];
    Page[APage]->Draw();
}

__noreturn
void Gui_t::ITask() {
    bool TouchProcessed = false, DetouchProcessed = true;
    while(true) {
        chThdSleepMilliseconds(TOUCH_POLLING_PERIOD_MS);
        if(Touch.IsTouched()) {
            if(TouchProcessed) Touch.DiscardData();
            else {   // New touch detected
                if(Touch.ReadData() == NEW) {
                    TouchProcessed = true;
                    DetouchProcessed = false;
//                    Uart.Printf("X=%d; Y=%d\r", Touch.X, Touch.Y);
                    CurrPage->ProcessTouch(Touch.X, Touch.Y);
                }
                else {
                    TouchProcessed = false;
                    DetouchProcessed = false;
                }
            } // new touch
        }
        else {
            TouchProcessed = false;
            if(!DetouchProcessed) {
                DetouchProcessed = true;
//                Uart.Printf("Detouch\r");
                CurrPage->ProcessDetouch(Touch.X, Touch.Y);
            }
        }
    } // while true
}
#endif
