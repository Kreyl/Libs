/*
 * ColorTable.h
 *
 *  Created on: 23 èþë. 2016 ã.
 *      Author: Kreyl
 */

#pragma once

#include "color.h"

#define COLOR_TABLE_SHORT       1

struct ColorTable_t {
    uint32_t Indx;
    const uint32_t Count;
    const Color_t *Clr;
    const Color_t* GetNext() {
        const Color_t *Rslt = &Clr[Indx];
        Indx++;
        if(Indx >= Count) Indx=0;
        return Rslt;
    }
    const Color_t* GetCurrent() { return &Clr[Indx]; }
    ColorTable_t(const Color_t *ptr, const uint32_t ACnt) :
        Indx(0), Count(ACnt), Clr(ptr) {}
};

extern ColorTable_t ColorTable;
