/*
 * pill.h
 *
 *  Created on: 22 мая 2014 г.
 *      Author: g.kruglov
 */

#pragma once

enum PillType_t { pilltypeAbility, pilltypeSetup };

struct Pill_t {
    union {
        int32_t TypeInt32;
        PillType_t Type;
    };
    union {
        int32_t AbilityMsk;
        int32_t GyroTimeout;
    };
    // Contains dose value after pill application
//    int32_t ChargeCnt;          // offset 4
//    bool IsOk() const {
//        if(dw32 < 0 and dw32 > 0b111111111111) return false;
//        return true;
//    }
} __attribute__ ((__packed__));
#define PILL_SZ     sizeof(Pill_t)
#define PILL_SZ32   (sizeof(Pill_t) / sizeof(int32_t))
