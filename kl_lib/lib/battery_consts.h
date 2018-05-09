/*
 * battery_consts.h
 *
 *  Created on: 30 дек. 2013 г.
 *      Author: kreyl
 */

#pragma once

#ifndef countof
#define countof(A)  (sizeof(A)/sizeof(A[0]))
#endif

// Table of charge
struct mVPercent_t {
    uint16_t mV;
    uint8_t Percent;
    uint8_t k;
};

//enum BatteryState_t {bstDischarging, bstCharging, bstIdle};
enum BatteryState_t {bsNone, bsEmpty, bsHalf, bsFull};

#if 0 // ========================= Alkaline 1.5V ===============================
static const mVPercent_t mVPercentTableAlkaline[] = {
        {1440, 100},
        {1370, 80},
        {1270, 60},
        {1170, 40},
        {1080, 20},
        {930,  10}
};
#define mVPercentTableAlkalineSz    countof(mVPercentTableAlkaline)

static uint8_t mV2PercentAlkaline(uint16_t mV) {
    for(uint8_t i=0; i<mVPercentTableAlkalineSz; i++)
        if(mV >= mVPercentTableAlkaline[i].mV) return mVPercentTableAlkaline[i].Percent;
    return 0;
}
#endif

#if 0 // ========================= Alkaline 4.5V ===============================
static const mVPercent_t mVPercentTableAlkaline[] = {
        {4320, 100},
        {4110, 80},
        {3810, 60},
        {3510, 40},
        {3240, 20},
        {2790, 10}
};
#define mVPercentTableAlkalineSz    countof(mVPercentTableAlkaline)

static uint8_t mV2PercentAlkaline(uint16_t mV) {
    for(uint8_t i=0; i<mVPercentTableAlkalineSz; i++)
        if(mV >= mVPercentTableAlkaline[i].mV) return mVPercentTableAlkaline[i].Percent;
    return 0;
}
#endif

#if 1 // ============================ Li-Ion ===================================
static const mVPercent_t mVPercentTableLiIon[] = {
        {4100, 100, 10},
        {4000, 90,  10},
        {3900, 80,  10},
        {3850, 70,  5},
        {3800, 60,  5},
        {3780, 50,  2},
        {3740, 40,  4},
        {3700, 30,  4},
        {3670, 20,  3},
        {3640, 10,  3}
};
#define mVPercentTableLiIonSz    countof(mVPercentTableLiIon)

__unused
static uint8_t mV2PercentLiIon(uint16_t mV) {
    for(uint8_t i=0; i<mVPercentTableLiIonSz; i++)
        if(mV >= mVPercentTableLiIon[i].mV) return mVPercentTableLiIon[i].Percent;
    return 0;
}
#endif

#if 0 // ============================ EEMB =====================================
#define BAT_TOP_mV          4140
#define BAT_ZERO_mV         3340
#define BAT_END_mV          3100    // Do not operate if Ubat <= BAT_END_V
#define BAT_PERCENT_STEP    8

#define mV2Percent(V)   (((V) > BAT_TOP_mV)? 100 : (((V) > BAT_ZERO_mV)? (100 - (BAT_TOP_mV - (V)) / BAT_PERCENT_STEP) : 0))
#endif

#if 0 // ============================ 3V Li ====================================
static const mVPercent_t mVPercentTableLi3V[] = {
        {2640, 100},
        {2550, 80},
        {2410, 60},
        {2210, 40},
        {2080, 20},
        {1950, 10}
};
#endif
