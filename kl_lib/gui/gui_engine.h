/*
 * gui.h
 *
 *  Created on: 13 ??? 2016 ?.
 *      Author: Kreyl
 */

#pragma once

#include "kl_lib.h"
#include "kl_buf.h"
#include "ILI9341.h"
#include "ControlClasses.h"

#define FRAMEBUFFER_LEN             20000   // 200x100
#define TOUCH_POLLING_PERIOD_MS     18

class Gui_t {
private:
    const Page_t *CurrPage;
public:
    void Init();
    void DrawPage(uint8_t APage);
    // Inner use
    void ITask();
};

extern Gui_t Gui;
extern ILI9341_t Lcd;
