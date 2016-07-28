/*
 * ControlClasses.cpp
 *
 *  Created on: 23 θώνÿ 2016 γ.
 *      Author: Kreyl
 */

#include <gui_engine.h>
#include "kl_lib.h"
#include "uart.h"
#include "ControlClasses.h"

extern FrameBuffer_t<uint16_t, FRAMEBUFFER_LEN> FBuf;

#if 1 // ==== Common ====
void DrawText(uint16_t Left, uint16_t Top, uint16_t Width, uint16_t Height,
        const char* S, PFont_t Font, Justify_t Justify, Color_t ClrFront) {
    // String width
    uint32_t StrW = Font->GetStringWidth(S);
    if(StrW > Width) StrW = Width;
    uint32_t XC = 0;    // x in buffer
    // Select the anchor position
    switch(Justify) {
        case jstLeft:   XC = 0;                  break;
        case jstCenter: XC = (Width - StrW) / 2; break;
        case jstRight:  XC = Width - StrW - 1;   break;
    }
    int32_t YC = Height / 2 - Font->YCenterLine;   // y in buffer
    if(YC < 0) YC = 0;

    uint16_t Clr565 = ClrFront.RGBTo565();
    char c;
    while((c = *S++) != 0) {
        uint8_t width;
        uint8_t *ptr;
        Font->GetChar(c, &width, &ptr);
        // Iterate pixels
        for(uint8_t x=0; x<width; x++) {
            uint32_t YCounter = 0;
            for(uint8_t y=0; y<Font->RowsCnt; y++) {
                uint8_t b = *ptr++;   // bit msk
                // Put pixels to buffer
                for(uint8_t i=0; i<8; i++) {
                    if(b & 0x01) FBuf.Put(x+XC, i+y*8+YC, Clr565);  // Modify only "black" pixels
                    b >>= 1;
                    if(++YCounter >= Font->Height) break;
                }
            } // y
        } // x
        XC += width+1;
    } // while
}

void PrintInt(uint16_t Left, uint16_t Top,
        int32_t N, PFont_t Font, Justify_t Justify, Color_t ClrFront,
        Color_t ClrBack) {
    char S[12];
    kl_bufprint(S, 7, "%d", N);
    uint32_t w = Font->GetStringWidth(S), h = Font->Height;
    if(FBuf.Setup(w, h) == OK) {
        // Fill rect
        uint16_t Color565 = ClrBack.RGBTo565();
        for(uint32_t iy=0; iy<h; iy++) {
            for(uint32_t ix=0; ix<w; ix++) FBuf.Put(ix, iy, Color565);
        }
        // Text
        DrawText(Left, Top, w, h, S, Font, Justify, ClrFront);
        Lcd.FillWindow(Left, Top, w, h, FBuf.Buf);
    }
}

#endif

void Control_t::FillRect(Color_t ClrTop, Color_t ClrBottom) const {
    if(ClrTop == ClrBottom) {
        uint16_t Color565 = ClrTop.RGBTo565();
        for(uint32_t y=0; y<Height; y++) {
            for(uint32_t x=0; x<Width; x++) FBuf.Put(x, y, Color565);
        }
    }
    else {
        uint32_t dalpha = 255 / Height;
        uint32_t alpha = 0;
        for(uint32_t y=0; y<Height; y++, alpha += dalpha) {
            uint16_t Color565 = ColorBlend(ClrBottom, ClrTop, alpha);
            for(uint32_t x=0; x<Width; x++) FBuf.Put(x, y, Color565);
        }
    }
}

// ==== Lines ====
void LineHoriz_t::Draw() const { Lcd.DrawRect(Left, Top, Width, Height, Clr); }

// ==== Button ====
void Button_t::Draw(BtnState_t State) const {
//    Uart.Printf("%u %u %u %u %S\r", Left, Width, Top, Height, Text);
    if(FBuf.Setup(Width, Height) != OK) {
        Uart.Printf("Too Large Btn\r");
        return;
    }
    // Draw shape depending on state
    if(State == btnReleased) FillRect(ClrReleasedTop, ClrReleasedBottom);
    else FillRect(ClrPressedTop, ClrPressedBottom);
    DrawText(Left, Top, Width-1, Height-1, Text, Font, jstCenter, ClrText);
    Lcd.FillWindow(Left, Top, Width, Height, FBuf.Buf);
}

// ==== Textbox ====
void Textbox_t::Draw() const {
//    Uart.Printf("%u %u %u %u %S\r", Left, Width, Top, Height, Text);
    if(FBuf.Setup(Width, Height) != OK) {
        Uart.Printf("Too Large Textbox\r");
        return;
    }
    // Draw shape
    FillRect(ClrBack, ClrBack);
    // Text
    DrawText(Left, Top, Width-1, Height-1, Text, Font, jstCenter, ClrText);
    Lcd.FillWindow(Left, Top, Width, Height, FBuf.Buf);
}

// ================================== Chart ====================================
void Chart_t::Clear() { Lcd.DrawRect(Left, Top, Width, Height, ClrBack); }

uint32_t Chart_t::ScaledY(float y) {
    float YmaxPx = Top + Height;
    float Scale = (float)Height / (Ymax - Ymin);
    float ys = YmaxPx - (y - Ymin) * Scale;
    // Restrict result
    if(ys > YmaxPx) ys = YmaxPx;
    else if(ys < Top) ys = Top;
    return (uint32_t)ys;
}
uint32_t Chart_t::ScaledX(float x) {
    float XmaxPx = Left + Width;
    float Scale = (float)Width / (Xmax - Xmin);
    float xs = Left + (x - Xmin) * Scale;
    // Restrict result
    if(xs > XmaxPx) xs = XmaxPx;
    else if(xs < Left) xs = Left;
    return (uint32_t)xs;
}

void Chart_t::AddLineHoriz(float y, Color_t AColor) {
    uint32_t ys = ScaledY(y);
    Lcd.DrawLineHoriz(Left, ys, Width, AColor);
    PrintInt(0, ys-Font->Height, (int32_t)y, Font, jstLeft, ClrText, ClrBack);
}
void Chart_t::AddLineVert(float x, Color_t AColor) {
    uint32_t xs = ScaledX(x);
    Lcd.DrawLineVert(xs, Top, Height, AColor);
    PrintInt(xs+1, Top+Height-Font->Height, (int32_t)(x / 1000), Font, jstLeft, ClrText, ClrBack);
}


void Series_t::AddPoint(float x, float y) {
    Lcd.DrawPoint(Parent->ScaledX(x), Parent->ScaledY(y), Color);
}

void Series_t::Clear() {
//    Uart.Printf("%.1f; %.1f\r", XScale, YScale);
}

