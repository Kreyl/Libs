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

#define CC_TX_PWR   CC_PwrPlus5dBm

#if 1 // =========================== Pkt_t =====================================
union rPkt_t  {
    uint32_t DWord[2];
    struct {
        uint8_t Length;
        int8_t Ch[4];
        uint8_t R1, R2;
        uint8_t Btns;
    } __packed;
    rPkt_t& operator = (const rPkt_t &Right) {
        DWord[0] = Right.DWord[0];
        DWord[1] = Right.DWord[1];
        return *this;
    }
    void Print() { Printf("%d %d %d %d %d %d; %X\r", Ch[0],Ch[1],Ch[2],Ch[3],R1, R2, Btns); }
} __packed;

#define RPKT_LEN    7   // 7 bytes of payload

struct rPktReply_t {
    uint8_t Length;
    uint8_t Reply;
} __packed;

#define REPLY_PKT_LEN   1

#endif

#if 1 // ======================= Channels & cycles =============================
#define RCHNL_SRV       0
#define ID2RCHNL(ID)    (RCHNL_MIN + ID)
#endif

#if 1 // =========================== Timings ===================================
#define RX_T_MS                 11
#define RX_SLEEP_T_MS           810
#define MIN_SLEEP_DURATION_MS   18
#define RETRY_CNT               2

#endif

#define RMSG_Q_LEN      18
#define RMSGID_PKT      1
#define RMSGID_CHNL     2

union RMsg_t {
    uint32_t DWord[3];
    rPkt_t Pkt;
    struct {
        uint32_t _Rsrvd;
        uint32_t Value;
        uint32_t ID;
    };
    RMsg_t& operator = (const RMsg_t &Right) {
        DWord[0] = Right.DWord[0];
        DWord[1] = Right.DWord[1];
        DWord[2] = Right.DWord[2];
        return *this;
    }
    RMsg_t() {
        DWord[0] = 0;
        DWord[1] = 0;
        DWord[2] = 0;
    }
    RMsg_t(rPkt_t &APkt)  { ID = RMSGID_PKT;  Pkt = APkt; }
    RMsg_t(uint8_t AChnl) { ID = RMSGID_CHNL; Value = AChnl; _Rsrvd = 0; }
} __attribute__((__packed__));


class rLevel1_t {
private:
    void TryToSleep(uint32_t SleepDuration) {
//        if(SleepDuration >= MIN_SLEEP_DURATION_MS) CC.EnterPwrDown();
        chThdSleepMilliseconds(SleepDuration); // XXX
    }
public:
    int8_t Rssi;
    EvtMsgQ_t<RMsg_t, RMSG_Q_LEN> RMsgQ;
    rPktReply_t rPktReply;
    uint8_t Init();
    void SetChannel(uint8_t NewChannel);
    // Inner use
    void ITask();
};

extern rLevel1_t Radio;
