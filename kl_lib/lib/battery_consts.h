/*
 * battery_consts.h
 *
 *  Created on: 30 Dec 2013
 *      Author: kreyl
 */

#pragma once

#include <cstdint>
//#include <iterator> // For std::size

namespace Battery {

enum class ChargingState {Discharging, Charging, Idle};
enum class ChargeState {Unknown, Empty, Half, Full};

struct mVPercent {
    uint32_t mV;
    uint32_t percent;
};

uint32_t mV2Percent(uint32_t mV, mVPercent *table, uint32_t tbl_sz) {
    for (uint32_t i = 0; i < tbl_sz; ++i) {
        if (mV >= table[i].mV) {
            return table[i].percent;
        }
    }
    return 0;
}

static inline constexpr mVPercent mVPercentTblAlkaline1v5[] = {
    {1550, 100},
    {1500, 90},
    {1450, 80},
    {1400, 60},
    {1350, 40},
    {1300, 20},
    {1200, 10},
    {1100, 5},
};

static inline constexpr mVPercent mVPercentTblAlkaline3v0[] = {
    {1550 * 2, 100},
    {1500 * 2, 90},
    {1450 * 2, 80},
    {1400 * 2, 60},
    {1350 * 2, 40},
    {1300 * 2, 20},
    {1200 * 2, 10},
    {1100 * 2, 5},
};

static inline constexpr uint32_t kLowVoltageAlkaline3v0_mV = mVPercentTblAlkaline3v0[7].mV;

static inline constexpr mVPercent mVPercentTblAlkaline4v5[] = {
    {1550 * 3, 100},
    {1500 * 3, 90},
    {1450 * 3, 80},
    {1400 * 3, 60},
    {1350 * 3, 40},
    {1300 * 3, 20},
    {1200 * 3, 10},
    {1100 * 3, 5},
};
} // namespace


#if 0 // ============================ Li-Ion ===================================
static const mVPercent mVPercentTableLiIon[] = {
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
static const mVPercent mVPercentTableLi3V[] = {
        {2640, 100},
        {2550, 80},
        {2410, 60},
        {2210, 40},
        {2080, 20},
        {1950, 10}
};
#endif
