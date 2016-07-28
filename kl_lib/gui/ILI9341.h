/*
 * ILI9341.h
 *
 *  Created on: 13 ??? 2016 ?.
 *      Author: Kreyl
 */

#pragma once

#include "kl_lib.h"
#include "color.h"
#include "board.h"

class ILI9341_t {
private:
    void WriteCmd(uint8_t Cmd);
    void WriteData(uint16_t Data);
//    uint16_t ReadData();
    void SetBounds(uint16_t Left, uint16_t Top, uint16_t Width, uint16_t Height);
    void PrepareToWriteGRAM() { WriteCmd(0x2C); }
public:
    void Init();
    void Cls(Color_t Color) { DrawRect(0, 0, LCD_W, LCD_H, Color); }
    void DrawRect  (uint32_t Left, uint32_t Top, uint32_t Width, uint32_t Height, Color_t Color);
    void DrawRect  (uint32_t Left, uint32_t Top, uint32_t Width, uint32_t Height, uint16_t Color565);
    void DrawPoint (uint32_t x, uint32_t y, Color_t Color);
    void DrawLineHoriz(uint32_t x0, uint32_t y0, uint32_t Len, Color_t Color);
    void DrawLineVert (uint32_t x0, uint32_t y0, uint32_t Len, Color_t Color);
    void FillWindow(uint32_t Left, uint32_t Top, uint32_t Width, uint32_t Height, uint16_t *Ptr);
};
