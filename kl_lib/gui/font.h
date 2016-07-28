/*
 * font.h
 *
 *  Created on: 14 ??? 2016 ?.
 *      Author: Kreyl
 */

#pragma once

//#include "uart.h"
#include <stdint.h>

#define FONT_ROWS_CNT(HeightPx)         ((HeightPx + 7) / 8)
#define FONT_CHAR_W(WidthPx, HeightPx)  (WidthPx * FONT_ROWS_CNT(HeightPx))

template <uint32_t CharSz>
struct FontChar_t {
    uint8_t Width;
    uint8_t data[CharSz];
};

class Font_t {
public:
    uint8_t Width, Height;
    uint8_t FirstCharCode;
    uint8_t RowsCnt;
    uint8_t YCenterLine;
private:
    uint32_t CharLen;
    const uint8_t* CharData;
public:
    Font_t(uint8_t AWidth, uint8_t AHeight, uint8_t AFirstCharCode,
            uint8_t AYCenterLine, const uint8_t *ACharData) :
                Width(AWidth), Height(AHeight), FirstCharCode(AFirstCharCode),
                RowsCnt(FONT_ROWS_CNT(AHeight)), YCenterLine(AYCenterLine),
                CharLen(Width*RowsCnt + 1),
                CharData(ACharData) { }

    uint32_t GetStringWidth(const char* S) const {
        uint32_t W = 0;
        char c;
        while((c = *S++) != 0) W += GetCharWidth(c)+1;
        return W;
    }
    uint32_t GetCharWidth(char c) const { return CharData[(c - FirstCharCode) * CharLen]; }
    void GetChar(char c, uint8_t *PWidth, uint8_t **PData) const {
        uint32_t Offset = (c - FirstCharCode) * CharLen;
        *PWidth = CharData[Offset];
        *PData = (uint8_t*)&CharData[Offset+1];
    }
};

typedef const Font_t* PFont_t;
