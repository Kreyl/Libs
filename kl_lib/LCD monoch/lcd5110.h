/*
 * File:   lcd5110.h
 * Author: Kreyl Laurelindo
 *
 * Created on 31.10.2017
 */

#pragma once

#include "kl_lib.h"
#include "board.h"
#include "shell.h"

#if 1 // ========================== GPIO etc ===================================
#define LCD_VIDEOBUF_SIZE   504     // = 84px * 6rows
#define LCD_WIDTH		    84
#define LCD_HEIGHT		    48

// Settings
#define LCD_DELAY()         DelayLoop(180)
#define USE_LARGE_FONTS     FALSE
#if USE_LARGE_FONTS
#include "lcd_LargeFonts.h"
#endif

//#define LCD_MIRROR_X_AXIS   TRUE
#define LCD_MIRROR_Y_AXIS   TRUE

#define LCD_STR_HEIGHT      8
#define LCD_STR_WIDTH       14
#endif

enum PseudoGraph_t {
    CornerTopLeftDouble = 0x99,
    CornerTopRightDouble = 0x8B,
    CornerBottomLeftDouble = 0x98,
    CornerBottomRightDouble = 0x8C,
    LineHorizDouble = 0x9D,
    LineHorizDoubleUp = 0x9A,
    LineHorizDoubleDown = 0x9B,
    LineVertDouble = 0x8A,
    LineVertDoubleLeft = 0x89,
    LineVertDoubleRight = 0x9C,
    LineCrossDouble = 0x9E,
};

enum Invert_t {NotInverted, Inverted};

class Lcd_t: public PrintfHelper_t {
private:
    const PinOutputPWM_t BckLt{LCD_BCKLT_PIN};
    uint8_t IBuf[LCD_VIDEOBUF_SIZE];
    int16_t CurrentPosition;   // x, y to place data to
    bool IInverted = false;
    // Pin driving functions
    void RES_Hi() { PinSetHi(LCD_RST_PIN); }
    void RES_Lo() { PinSetLo(LCD_RST_PIN); }
    void CLK_Hi() { PinSetHi(LCD_CLK_PIN); LCD_DELAY(); }
    void CLK_Lo() { PinSetLo(LCD_CLK_PIN); LCD_DELAY(); }
    void Din_Hi () { PinSetHi(LCD_DIN_PIN);  LCD_DELAY(); }
    void Din_Lo () { PinSetLo(LCD_DIN_PIN);  LCD_DELAY(); }
    void CE_Hi () { PinSetHi(LCD_CE_PIN);  LCD_DELAY(); }
    void CE_Lo () { PinSetLo(LCD_CE_PIN);  LCD_DELAY(); }
    void DC_Hi () { PinSetHi(LCD_DC_PIN);  LCD_DELAY(); }
    void DC_Lo () { PinSetLo(LCD_DC_PIN);  LCD_DELAY(); }
    semaphore_t semLcd;
    void WriteCmd(uint8_t ACmd);
    uint8_t IPutChar(char c);
    void IStartTransmissionIfNotYet(){};
    // High-level
    void GotoXY(uint8_t x, uint8_t y) { CurrentPosition =  x + y*84; }
public:
    void Init();
    void Shutdown();
    void Update();
    void SetBacklight(uint8_t ABrightness)  { BckLt.Set(ABrightness); }
    uint8_t GetBacklight() { return BckLt.Get(); }
    // High-level
    void GotoCharXY(uint8_t x, uint8_t y) { CurrentPosition = (x*6 + y*84); }
    void DrawChar(uint8_t AChar, Invert_t AInvert);
    void Print(const uint8_t x, const uint8_t y, const char *S, ...);
    void PrintInverted(const uint8_t x, const uint8_t y, const char *S, ...);
#if USE_LARGE_FONTS
    void PrintfFont(const uint8_t *PFont, uint8_t x, uint8_t y, const char *S, ...);
#endif
    void Cls();
    void DrawImage(const uint8_t x, const uint8_t y, const uint8_t *Img);
    void DrawPixel(const uint8_t x, const uint8_t y, Invert_t AInvert);
    // Symbols printing
    void Symbols(const uint8_t x, const uint8_t y, ...);

    void IIrqHandler();
    friend class Interface_t;
};

extern Lcd_t Lcd;
