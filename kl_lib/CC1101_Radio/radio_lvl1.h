/*
 * radio_lvl1.h
 *
 *  Created on: Nov 17, 2013
 *      Author: kreyl
 */

#pragma once

#include <kl_lib.h>
#include "ch.h"
#include "cc1101.h"
#include "kl_buf.h"
#include "uart.h"

#if 0 // ========================= Signal levels ===============================
// Python translation for db
#define RX_LVL_TOP      1000
// Jolaf: str(tuple(1 + int(sqrt(float(i) / 65) * 99) for i in xrange(0, 65 + 1)))
//const int32_t dBm2Percent1000Tbl[66] = {10, 130, 180, 220, 250, 280, 310, 330, 350, 370, 390, 410, 430, 450, 460, 480, 500, 510, 530, 540, 550, 570, 580, 590, 610, 620, 630, 640, 650, 670, 680, 690, 700, 710, 720, 730, 740, 750, 760, 770, 780, 790, 800, 810, 820, 830, 840, 850, 860, 860, 870, 880, 890, 900, 910, 920, 920, 930, 940, 950, 960, 960, 970, 980, 990, 1000};
const int32_t dBm2Percent1000Tbl[86] = {
         10, 117, 162, 196, 225, 250, 273, 294, 314, 332,
        350, 366, 382, 397, 412, 426, 440, 453, 466, 478,
        490, 502, 514, 525, 536, 547, 558, 568, 578, 588,
        598, 608, 617, 627, 636, 645, 654, 663, 672, 681,
        689, 698, 706, 714, 722, 730, 738, 746, 754, 762,
        769, 777, 784, 792, 799, 806, 814, 821, 828, 835,
        842, 849, 856, 862, 869, 876, 882, 889, 895, 902,
        908, 915, 921, 927, 934, 940, 946, 952, 958, 964,
        970, 976, 982, 988, 994, 1000
};

static inline int32_t dBm2Percent(int32_t Rssi) {
    if(Rssi < -100) Rssi = -100;
    else if(Rssi > -15) Rssi = -15;
    Rssi += 100;    // 0...85
    return dBm2Percent1000Tbl[Rssi];
}

// Conversion Lvl1000 <=> Lvl250
#define Lvl1000ToLvl250(Lvl1000) ((uint8_t)((Lvl1000 + 3) / 4))

static inline void Lvl250ToLvl1000(uint16_t *PLvl) {
    *PLvl = (*PLvl) * 4;
}

// Sensitivity Constants, percent [1...1000]. Feel if RxLevel > SnsConst.
#define RLVL_NEVER              10000
#define RLVL_2M                 800     // 0...4m
#define RLVL_4M                 700     // 1...20m
#define RLVL_10M                600
#define RLVL_50M                1
#define RLVL_PELENGATOR         RLVL_4M // LED will lit if rlevel is higher

#endif

#if 1 // =========================== Pkt_t =====================================
union rPkt_t {
    uint32_t DWord32;
    struct {
        uint8_t ID;
        uint8_t State;
        uint8_t CheckID;
        uint8_t CheckState;
    };
    bool operator == (const rPkt_t &APkt) { return (DWord32 == APkt.DWord32); }
    rPkt_t& operator = (const rPkt_t &Right) { DWord32 = Right.DWord32; return *this; }
} __attribute__ ((__packed__));
#define RPKT_LEN    sizeof(rPkt_t)
#endif

//#define THE_WORD        0xCA115EA1

// ==== Sizes ====
#define RXTABLE_SZ      9
#define RXTABLE_MAX_CNT 3   // Do not receive if this count reached. Will not indicate more anyway.

#if 1 // ======================= Channels & cycles =============================
#define REMCTRL_ID      8   // ID of remote control

#define RCHNL_MIN       0
#define RCHNL_MAX       8
#define ID2RCHNL(ID)    (RCHNL_MIN + ID)
#endif

#if 1 // =========================== Timings ===================================
#define RX_T_MS                 180      // pkt duration at 10k is around 12 ms
#define RX_SLEEP_T_MS           810
#define MIN_SLEEP_DURATION_MS   18
#endif

class RxTable_t {
private:
    rPkt_t IBuf[RXTABLE_SZ];
    uint32_t Cnt = 0;
public:
    void AddOrReplaceExisting(rPkt_t &APkt) {
        for(uint32_t i=0; i<Cnt; i++) {
            if(IBuf[i].ID == APkt.ID) {
                IBuf[i] = APkt; // Replace with newer pkt
                return;
            }
        }
        IBuf[Cnt] = APkt;
        if(Cnt < (RXTABLE_SZ-1)) Cnt++;
    }
    uint32_t GetCount() { return Cnt; }
    void Clear() { Cnt = 0; }
    uint8_t GetPktByID(uint8_t ID, rPkt_t **ptr) {
        for(uint32_t i=0; i<Cnt; i++) {
            if(IBuf[i].ID == ID) {
                *ptr = &IBuf[i];
                return OK;
            }
        }
        return FAILURE;
    }
    bool IDPresents(uint8_t ID) {
        for(uint32_t i=0; i<Cnt; i++) {
            if(IBuf[i].ID == ID) return true;
        }
        return false;
    }
    void Print() {
        Uart.Printf("RxTable Cnt: %u\r", Cnt);
        for(uint32_t i=0; i<Cnt; i++) Uart.Printf("ID: %u; State: %u\r", IBuf[i].ID, IBuf[i].State);
    }
};

class rLevel1_t {
private:
    rPkt_t Pkt;
    void TryToSleep(uint32_t SleepDuration) {
        if(SleepDuration >= MIN_SLEEP_DURATION_MS) CC.EnterPwrDown();
        chThdSleepMilliseconds(SleepDuration);
    }
public:
    thread_t *PThd;
    int8_t Rssi;
    RxTable_t RxTable;
    bool MustTx = false;
    uint8_t Init();
    // Inner use
    void ITask();
};

extern rLevel1_t Radio;
