/*
 * File:   lcd110x.h
 * Author: Kreyl Laurelindo
 *
 * Created on 24 �������� 2010 �., 14:25
 */

#pragma once

#include "kl_lib.h"
#include "board.h"
#include "shell.h"

#if 1 // ========================== GPIO etc ===================================
#define LCD_VIDEOBUF_SIZE   864     // = 96px * 9rows
#define LCD_WIDTH		    96
#define LCD_HEIGHT		    65

// Settings
#define USE_LARGE_FONTS     FALSE
#if USE_LARGE_FONTS
#include "lcd_LargeFonts.h"
#endif

//#define LCD_MIRROR_X_AXIS   TRUE
#define LCD_MIRROR_Y_AXIS   TRUE

#define LCD_STR_HEIGHT      8
#define LCD_STR_WIDTH       16
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
    const PinOutputPWM_t BckLt{LCD_BCKLT};
    uint16_t IBuf[LCD_VIDEOBUF_SIZE];
    uint16_t CurrentPosition;   // x, y to place data to
    bool Inverted = false;
    // Pin driving functions
    void XRES_Hi() { PinSetHi(LCD_XRES); }
    void XRES_Lo() { PinSetLo(LCD_XRES); }
    void SCLK_Hi() { PinSetHi(LCD_SCLK); }
    void SCLK_Lo() { PinSetLo(LCD_SCLK); }
    void SDA_Hi () { PinSetHi(LCD_SDA);  }
    void SDA_Lo () { PinSetLo(LCD_SDA);  }
    void XCS_Hi () { PinSetHi(LCD_XCS);  }
    void XCS_Lo () { PinSetLo(LCD_XCS);  }
    semaphore_t semLcd;
    void WriteCmd(uint8_t ACmd);
    uint8_t IPutChar(char c);
    void IStartTransmissionIfNotYet(){};
    // High-level
    void GotoXY(uint8_t x, uint8_t y) { CurrentPosition =  x + y*96; }
public:
    void Init();
    void Shutdown();
    void Backlight(uint8_t ABrightness)  { BckLt.Set(ABrightness); }
    // High-level
    void GotoCharXY(uint8_t x, uint8_t y) { CurrentPosition =  x*6 + y*96; }
    void DrawChar(uint8_t AChar, Invert_t AInvert);
    void Printf(const uint8_t x, const uint8_t y, const char *S, ...);
    void PrintfInverted(const uint8_t x, const uint8_t y, const char *S, ...);
#if USE_LARGE_FONTS
    void PrintfFont(const uint8_t *PFont, uint8_t x, uint8_t y, const char *S, ...);
#endif
    void Cls();
    void DrawImage(const uint8_t x, const uint8_t y, const uint8_t *Img);
    // Symbols printing
    void Symbols(const uint8_t x, const uint8_t y, ...);
    // Constructor
    Lcd_t(): CurrentPosition(0) {
        for(uint32_t i=0; i<LCD_VIDEOBUF_SIZE; i++) IBuf[i] = 0;
    }
};

extern Lcd_t Lcd;
