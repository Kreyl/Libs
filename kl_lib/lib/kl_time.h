/*
 * kl_time.h
 *
 *  Created on: 13.06.2012
 *      Author: kreyl
 */

#pragma once

#include <stdint.h>
#include "kl_lib.h"
#include "uart.h"

enum DayOfWeek_t {AnyDay=0, Monday=1, Tuesday=2, Wednesday=3, Thursday=4, Friday=5, Saturday=6, Sunday=7};

#define SECS_IN_A_DAY       (24UL * 60UL * 60UL)
#define YEAR_MIN            2000
#define YEAR_MAX            2099
#define LEAPYEAR(year)      (!((year) % 4) and (((year) % 100) or !((year) % 400)))
#define YEARSIZE(year)      (LEAPYEAR(year) ? 366 : 365)

const uint8_t MonthDays[2][12] = {
    {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31},
    {31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31}
};

struct Time_t {
    int32_t H, M, S;
};

struct TimeHM_t {
    int32_t H, M;
    bool IsValid() { return (H >= 0 and H <= 23) and (M >= 0 and M <= 59); }
    TimeHM_t& operator = (const TimeHM_t &Right) {
        H = Right.H;
        M = Right.M;
        return *this;
    }
};

struct DateTime_t {
    int32_t H, M, S;
    int32_t Year;
    int32_t Month;
    int32_t Day;
    void IncH() { H = (H+1 > 23)? 0: H+1; }
    void IncM() { M = (M+1 > 59)? 0: M+1; }
    void IncYear() { Year++; }
    void IncMonth() { Month = (Month+1 > 12)? 1 : Month+1; }
    void IncDay() {
        uint32_t Leap = LEAPYEAR(Year)? 1 : 0;
        uint8_t MaxDayCnt = MonthDays[Leap][Month-1];
        Day = (Day+1 > MaxDayCnt)? 1: Day+1;
    }

    void DecH() { H = (H <= 0)? 23 : H-1; }
    void DecM() { M = (M <= 0)? 59 : M-1; }
    void DecYear() { Year--; }
    void DecMonth() { Month = (Month <= 1)? 12 : Month-1; }
    void DecDay() {
        if(Day <= 1) {
            uint32_t Leap = LEAPYEAR(Year)? 1 : 0;
            Day = MonthDays[Leap][Month-1];
        }
        else Day--;
    }

    DateTime_t& operator = (const DateTime_t &Right) {
       H = Right.H;
       M = Right.M;
       S = Right.S;
       Year = Right.Year;
       Month = Right.Month;
       Day = Right.Day;
       return *this;
    }

    bool IsValid() {
        if((Year >= 2018 and Year <= 2036) and (Month >= 1 and Month <= 12)) {
            // Check day
            uint32_t Leap = LEAPYEAR(Year)? 1 : 0;
            uint8_t MaxDayCnt = MonthDays[Leap][Month-1];
            if(Day >= 1 and Day <= MaxDayCnt) {
                return (H >= 0 and H <= 23) and (M >= 0 and M <= 59) and (S >= 0 and S <= 59);
            }
        }
        return false;
    }

    void Print() const { Printf("%04u/%02u/%02u %02u:%02u:%02u\r", Year, Month, Day, H, M, S); }
    DateTime_t(int32_t AH, int32_t AM, int32_t AS, int32_t AYear, int32_t AMonth, int32_t ADay) :
        H(AH), M(AM), S(AS), Year(AYear), Month(AMonth), Day(ADay) {}
    DateTime_t() : H(0), M(0), S(0), Year(0), Month(0), Day(0) {}
};

class TimeCounter_t {
public:
    DateTime_t Curr;
    void GetDateTime();
    DayOfWeek_t CurrDayOfWeek();
    void SetDateTime();
    void DisableIrq();
    void EnableIrq();

    void BeFast();
    void BeNormal();

    void Init();
};

extern TimeCounter_t Time;
