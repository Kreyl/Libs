/*
 * Effects.h
 *
 *  Created on: 15 марта 2016 г.
 *      Author: Kreyl
 */

#pragma once

#include "board.h"
#include "ch.h"
#include "color.h"
#include "ws2812b.h"

class LedChunk_t {
private:
    int Head, Tail;
    uint8_t GetNext(int *PCurrent);
    int GetPrevN(int Current, int N);
public:
    int Start, End, NLeds;
    Color_t Color;
    LedChunk_t(int AStart, int AEnd) {
        Start = AStart;
        End = AEnd;
        NLeds = 1;
        Head = AStart;
        Tail = AEnd;
        Color = clBlack;
    }
    uint32_t ProcessAndGetDelay();
    void StartOver();
};

class Effects_t {
private:
    thread_t *PThd;
public:
    void Init();
    // Effects
    void AllTogetherNow(Color_t Color);
    void AllTogetherSmoothly(Color_t Color, uint32_t ASmoothValue);
    void ChunkRun(Color_t Color, uint32_t NLeds);
    void SinusRun(Color_t Color1, Color_t Color2);
    void Flashes();
};

extern Effects_t Effects;
