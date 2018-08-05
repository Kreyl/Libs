/*
 * radio_lvl1.h
 *
 *  Created on: Nov 17, 2013
 *      Author: kreyl
 */

#pragma once

#include "kl_lib.h"
#include "ch.h"
#include "cc1101.h"
#include "kl_buf.h"
#include "shell.h"
#include "MsgQ.h"

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

#define CC_TX_PWR   CC_Pwr0dBm

#if 1 // =========================== Pkt_t =====================================
struct Param_t {
    int Dogan : 4;
    bool Dead: 1;
    bool Corrupted: 1;
    bool IsInTodash: 1;
} __packed;

union rPkt_t  {
    uint32_t DWord;
    uint16_t Word;
    struct {
        uint16_t ID : 13;
        uint8_t Cycle: 3;
        uint16_t TimeSrcID;
        uint8_t Influence;
        union {
            uint8_t Param;
            struct {
                int Dogan : 4;
                bool Dead: 1;
                bool Corrupted: 1;
                bool IsInTodash: 1;
            } __packed;
        } __packed;
    } __packed;
    rPkt_t& operator = (const rPkt_t &Right) {
        DWord = Right.DWord;
        Word = Right.Word;
        return *this;
    }
    void Print() { Printf("ID: %u; Ccl: %u; Tsrc: %u; Inf: %u; P: %u\r", ID,Cycle,TimeSrcID,Influence,Param); }
} __packed;

#define RPKT_LEN    sizeof(rPkt_t)
#endif

#define RSSI_MIN            -75

#if 1 // ======================= Channels & cycles =============================
#define ID_FIREFLY          400
#define RCHNL               7
#define RCYCLE_CNT          4
#define ARMLET_CNT          82
#define LUSTRA_CNT          200
#define RSLOT_CNT           (ARMLET_CNT + LUSTRA_CNT)
#endif

#if 1 // =========================== Timings ===================================
#define TIMESLOT_DURATION_ST    20
#define MIN_SLEEP_DURATION_MS   18
#define SCYCLES_TO_KEEP_TIMESRC 4   // After that amount of supercycles, TimeSrcID become self ID
#endif

#if 1 // ============================= RX Table ================================
#define RXTABLE_SZ              4
#define RXT_PKT_REQUIRED        FALSE
class RxTable_t {
private:
#if RXT_PKT_REQUIRED
    rPkt_t IBuf[RXTABLE_SZ];
#else
    uint8_t IdBuf[RXTABLE_SZ];
#endif
    uint32_t Cnt = 0;
public:
#if RXT_PKT_REQUIRED
    void AddOrReplaceExistingPkt(rPkt_t &APkt) {
        if(Cnt >= RXTABLE_SZ) return;   // Buffer is full, nothing to do here
        for(uint32_t i=0; i<Cnt; i++) {
            if(IBuf[i].ID == APkt.ID) {
                IBuf[i] = APkt; // Replace with newer pkt
                return;
            }
        }
        IBuf[Cnt] = APkt;
        Cnt++;
    }

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
#else
    void AddId(uint8_t ID) {
        if(Cnt >= RXTABLE_SZ) return;   // Buffer is full, nothing to do here
        for(uint32_t i=0; i<Cnt; i++) {
            if(IdBuf[i] == ID) return;
        }
        IdBuf[Cnt] = ID;
        Cnt++;
    }

#endif
    uint32_t GetCount() { return Cnt; }
    void Clear() { Cnt = 0; }

    void Print() {
        Printf("RxTable Cnt: %u\r", Cnt);
        for(uint32_t i=0; i<Cnt; i++) {
#if RXT_PKT_REQUIRED
            Printf("ID: %u; State: %u\r", IBuf[i].ID, IBuf[i].State);
#else
            Printf("ID: %u\r", IdBuf[i]);
#endif
        }
    }
};
#endif


// Message queue
#define R_MSGQ_LEN      9
enum RmsgId_t { rmsgSetPwr, rmsgSetChnl, rmsgTimeToRx, rmsgTimeToTx, rmsgTimeToSleep, rmsgPktRx };
struct RMsg_t {
    RmsgId_t Cmd;
    uint8_t Value;
    RMsg_t() : Cmd(rmsgSetPwr), Value(0) {}
    RMsg_t(RmsgId_t ACmd) : Cmd(ACmd), Value(0) {}
    RMsg_t(RmsgId_t ACmd, uint8_t AValue) : Cmd(ACmd), Value(AValue) {}
} __attribute__((packed));

class rLevel1_t {
public:
    int8_t Rssi;
    rPkt_t PktTx, PktRx;
    EvtMsgQ_t<RMsg_t, R_MSGQ_LEN> RMsgQ;
    uint8_t Init();
    // Inner use
    void ITask();
};

extern rLevel1_t Radio;
