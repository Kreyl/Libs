/*
 * ILI9341.cpp
 *
 *  Created on: 13 ??? 2016 ?.
 *      Author: Kreyl
 */

#include <Gui/ILI9341.h>
#include "board.h"
#include "uart.h"

#if 1 // ==== Pin driving functions ====
#define RstHi()  { PinSet  (LCD_RESET_GPIO, LCD_RESET_PIN); }
#define RstLo()  { PinClear(LCD_RESET_GPIO, LCD_RESET_PIN); }
#define CsHi()   { PinSet  (LCD_CSX_GPIO, LCD_CSX_PIN);     }
#define CsLo()   { PinClear(LCD_CSX_GPIO, LCD_CSX_PIN);     }
#define DcHi()   { PinSet  (LCD_DC_GPIO, LCD_DC_PIN); }
#define DcLo()   { PinClear(LCD_DC_GPIO, LCD_DC_PIN); }
#define WrHi()   { PinSet  (LCD_WR_GPIO, LCD_WR_PIN); }
#define WrLo()   { PinClear(LCD_WR_GPIO, LCD_WR_PIN); }
#define RdHi()   { PinSet  (LCD_RD_GPIO, LCD_RD_PIN); }
#define RdLo()   { PinClear(LCD_RD_GPIO, LCD_RD_PIN); }
#define Write(Value) PortSetValue(LCD_DATA_GPIO, Value)
#endif

void ILI9341_t::Init() {
    // ==== GPIO ====
    PinSetupOut(LCD_RESET_GPIO, LCD_RESET_PIN, omPushPull);
    PinSetupOut(LCD_CSX_GPIO,   LCD_CSX_PIN,   omPushPull);
    PinSetupOut(LCD_DC_GPIO,    LCD_DC_PIN,    omPushPull, psHigh);
    PinSetupOut(LCD_WR_GPIO,    LCD_WR_PIN,    omPushPull, psHigh);
    PinSetupOut(LCD_RD_GPIO,    LCD_RD_PIN,   omPushPull, psHigh);
    // Data port
    PortInit(LCD_DATA_GPIO, omPushPull, pudNone, psHigh);
    PortSetupOutput(LCD_DATA_GPIO);
    // ==== Init LCD ====
    // Initial signals
    RstHi();
    CsHi();
    RdHi();
    WrHi();
    // Reset LCD
    RstLo();
    chThdSleepMilliseconds(4);
    RstHi();
    chThdSleepMilliseconds(54);
    CsLo(); // Stay selected forever

    // Commands
    WriteCmd(0x11); // Sleep out
    chThdSleepMilliseconds(126);

    WriteCmd(0x29); // Display ON
    // Row order etc.
    WriteCmd(0x36);
    WriteData(0xE8);    // MY, MX, Row/Column exchange, BGR
    // Pixel format
    WriteCmd(0x3A);
    WriteData(0x55);    // 16 bit both RGB & MCU

//    WriteCmd(0x09);
//    PortSetupInput(LCD_DATA_GPIO);
//    for(uint8_t i=0; i<4; i++) {
//        RdLo();
//        RdHi();
//        uint16_t r = LCD_DATA_GPIO->IDR;
//        Uart.Printf("Lcd: %X\r", r);
//    }
//    PortSetupOutput(LCD_DATA_GPIO);
}

void ILI9341_t::WriteCmd(uint8_t Cmd) {
    DcLo();
    PortSetValue(LCD_DATA_GPIO, Cmd);
    WrLo();
    WrHi();
    DcHi();
}

void ILI9341_t::WriteData(uint16_t Data) {
    PortSetValue(LCD_DATA_GPIO, Data);
    WrLo();
    WrHi();
}

//uint16_t ILI9341_t::ReadData() {
//    PortSetupInput(LCD_DATA_GPIO);
//    RdLo();
//    uint16_t Rslt = PortGetValue(LCD_DATA_GPIO);
//    RdHi();
//    PortSetupOutput(LCD_DATA_GPIO);
//    return 0;//Rslt;
//}

void ILI9341_t::SetBounds(uint16_t Left, uint16_t Top, uint16_t Width, uint16_t Height) {
    uint16_t XEndAddr = Left + Width  - 1;
    uint16_t YEndAddr = Top  + Height - 1;
    // Write bounds
    WriteCmd(0x2A); // X
    WriteData(Left>>8);
    WriteData(Left);            // MSB will be ignored anyway
    WriteData(XEndAddr >> 8);
    WriteData(XEndAddr);
    WriteCmd(0x2B); // Y
    WriteData(Top >> 8);
    WriteData(Top);             // MSB will be ignored anyway
    WriteData(YEndAddr >> 8);
    WriteData(YEndAddr);
}

void ILI9341_t::FillWindow(uint32_t Left, uint32_t Top, uint32_t Width, uint32_t Height, uint16_t *Ptr) {
    SetBounds(Left, Top, Width, Height);
    uint32_t Cnt = Width * Height;
    PrepareToWriteGRAM();
//    uint32_t t = TIM5->CNT;
    while(Cnt--) WriteData(*Ptr++);
//    uint32_t delta = TIM5->CNT - t;
//    Uart.Printf("t=%u\r", delta);
}

void ILI9341_t::DrawRect(uint32_t Left, uint32_t Top, uint32_t Width, uint32_t Height, uint16_t Color565) {
    SetBounds(Left, Top, Width, Height);
    uint32_t Cnt = Width * Height;
    PrepareToWriteGRAM();
    while(Cnt--) WriteData(Color565);
}

void ILI9341_t::DrawRect(uint32_t Left, uint32_t Top, uint32_t Width, uint32_t Height, Color_t Color) {
    SetBounds(Left, Top, Width, Height);
    uint32_t Cnt = Width * Height;
    uint16_t Clr565 = Color.RGBTo565();
    // Fill LCD
    PrepareToWriteGRAM();
    while(Cnt--) WriteData(Clr565);
}

void ILI9341_t::DrawPoint (uint32_t x, uint32_t y, Color_t Color) {
    SetBounds(x, y, 1, 1);
    uint16_t Clr565 = Color.RGBTo565();
    PrepareToWriteGRAM();
    WriteData(Clr565);
}

void ILI9341_t::DrawLineHoriz(uint32_t x0, uint32_t y0, uint32_t Len, Color_t Color) {
    SetBounds(x0, y0, Len, 1);
    uint16_t Clr565 = Color.RGBTo565();
    PrepareToWriteGRAM();
    while(Len--) WriteData(Clr565);
}
void ILI9341_t::DrawLineVert (uint32_t x0, uint32_t y0, uint32_t Len, Color_t Color) {
    SetBounds(x0, y0, 1, Len);
    uint16_t Clr565 = Color.RGBTo565();
    PrepareToWriteGRAM();
    while(Len--) WriteData(Clr565);
}
