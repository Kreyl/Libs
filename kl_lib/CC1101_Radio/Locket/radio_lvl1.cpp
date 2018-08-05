/*
 * radio_lvl1.cpp
 *
 *  Created on: Nov 17, 2013
 *      Author: kreyl
 */

#include "radio_lvl1.h"
#include "cc1101.h"
#include "uart.h"
#include "main.h"

cc1101_t CC(CC_Setup0);

//#define DBG_PINS

#ifdef DBG_PINS
#define DBG_GPIO1   GPIOB
#define DBG_PIN1    10
#define DBG1_SET()  PinSetHi(DBG_GPIO1, DBG_PIN1)
#define DBG1_CLR()  PinSetLo(DBG_GPIO1, DBG_PIN1)
#define DBG_GPIO2   GPIOB
#define DBG_PIN2    9
#define DBG2_SET()  PinSetHi(DBG_GPIO2, DBG_PIN2)
#define DBG2_CLR()  PinSetLo(DBG_GPIO2, DBG_PIN2)
#else
#define DBG1_SET()
#define DBG1_CLR()
#endif

rLevel1_t Radio;
extern int32_t ID;

#if 1 // ================================ Task =================================
static THD_WORKING_AREA(warLvl1Thread, 256);
__noreturn
static void rLvl1Thread(void *arg) {
    chRegSetThreadName("rLvl1");
    while(true) {
        // Process queue
        RMsg_t msg = Radio.RMsgQ.Fetch(TIME_IMMEDIATE);
        if(msg.Cmd == R_MSG_SET_PWR) CC.SetTxPower(msg.Value);
        if(msg.Cmd == R_MSG_SET_CHNL) CC.SetChannel(msg.Value);
        // Process task
        Radio.TaskTransmitter();
    } // while true
}

void rLevel1_t::TaskTransmitter() {
    PktTx.ID = ID;
    DBG1_SET();
    CC.Recalibrate();
    CC.Transmit(&PktTx, RPKT_LEN);
    DBG1_CLR();
//    Printf("%u\r", PktTx.ID);
    chThdSleepMilliseconds(11);
}

//void rLevel1_t::TaskReceiverSingle() {
////    uint8_t Ch = ID2RCHNL(App.ID - 1);
////    CC.SetChannel(Ch);
//    CC.SetChannel(RCHNL_COMMON);
//    uint8_t RxRslt = CC.Receive(11, &PktRx, &Rssi);   // Double pkt duration + TX sleep time
//    if(RxRslt == OK) {
////        Uart.Printf("Ch=%u; Rssi=%d\r", Ch, Rssi);
//        Uart.Printf("ForceRssi=%d\r", Rssi);
////        if(PktRx.DWord32 == THE_WORD and Rssi > -63) App.SignalEvt(EVT_RADIO);
//    }
//}

//void rLevel1_t::TaskReceiverManyByID() {
//    for(int N=0; N<4; N++) { // Iterate channels N times
//        // Iterate channels
//        for(int32_t i = ID_MIN; i <= ID_MAX; i++) {
//            if(i == ID) continue;   // Do not listen self
//            CC.SetChannel(ID2RCHNL(i));
////            Printf("%u\r", i);
//            CC.Recalibrate();
//            uint8_t RxRslt = CC.Receive(18, &PktRx, &Rssi);   // Double pkt duration + TX sleep time
//            if(RxRslt == retvOk) {
//                Printf("Ch=%u; Rssi=%d\r", ID2RCHNL(i), Rssi);
//                if(PktRx.DWord32 == THE_WORD and Rssi > RSSI_MIN) RxTable.AddId(i);
////                else Printf("PktErr\r");
//            }
//        } // for i
//        TryToSleep(270);
//    } // For N
//    EvtMsg_t msg(evtIdCheckRxTable);
//    EvtQMain.SendNowOrExit(msg);
//}

//void rLevel1_t::TaskReceiverManyByChannel() {
//    // Iterate channels
//    for(int32_t i = RCHNL_MIN; i <= RCHNL_MAX; i++) {
//        CC.SetChannel(i);
//        uint8_t RxRslt = CC.Receive(27, &PktRx, &Rssi);   // Double pkt duration
//        if(RxRslt == OK) {
//            Uart.Printf("Ch=%u; Rssi=%d\r", i, Rssi);
//            App.SignalEvt(EVT_RX);
//            TryToSleep(999);
//        }
//    } // for i
//    TryToSleep(270);
//}

//void rLevel1_t::TaskFeelEachOtherSingle() {
//    int8_t TopRssi = -126;
//    // Alice is boss
//    if((App.ID & 0x01) == 1) {
//        CC.SetChannel(ID2RCHNL(App.ID));
//        for(int i=0; i<7; i++) {
//            DBG1_SET();
//            CC.Transmit(&PktTx);
//            DBG1_CLR();
//            // Listen for an answer
//            uint8_t RxRslt = CC.Receive(18, &PktRx, &Rssi);   // Double pkt duration + TX sleep time
//            if(RxRslt == OK) {
////                Uart.Printf("i=%d; Rssi=%d\r", i, Rssi);
//                if(Rssi > TopRssi) TopRssi = Rssi;
//            }
//            TryToSleep(126);
//        } // for
//    }
//    // Bob does what Alice says
//    else {
//        CC.SetChannel(ID2RCHNL(App.ID - 1));
//        for(int i=0; i<7; i++) {
//            // Listen for a command
//            uint8_t RxRslt = CC.Receive(144, &PktRx, &Rssi);   // Double pkt duration + TX sleep time
//            if(RxRslt == OK) {
////                Uart.Printf("i=%d; Rssi=%d\r", i, Rssi);
//                if(Rssi > TopRssi) TopRssi = Rssi;
//                // Transmit reply
//                DBG1_SET();
//                CC.Transmit(&PktTx);
//                DBG1_CLR();
//            }
//            TryToSleep(45);
//        } // for
//    }
//    // Signal Evt if something received
//    if(TopRssi > -126) {
//        Rssi = TopRssi;
////        App.SignalEvt(EVT_RADIO);
//    }
//}

//void rLevel1_t::TaskFeelEachOtherMany() {
//    CC.SetChannel(RCHNL_EACH_OTH);
//    CC.SetTxPower(TxPwr);
//    PktTx.DWord32 = App.ID;
//    for(uint32_t CycleN=0; CycleN < CYCLE_CNT; CycleN++) {  // Iterate cycles
//        uint32_t TxSlot = Random(0, (SLOT_CNT-1));          // Decide when to transmit
//        // If TX slot is not zero, receive at zero cycle or sleep otherwise
////        Uart.Printf("Txs=%u C=%u\r", TxSlot, CycleN);
//        if(TxSlot != 0) {
//            uint32_t TimeBefore = TxSlot * SLOT_DURATION_MS;
////            Uart.Printf("TB=%u\r", TimeBefore);
//            if(CycleN == 0 and RxTable.GetCount() < RXTABLE_SZ) TryToReceive(TimeBefore);
//            else TryToSleep(TimeBefore);
//        }
//        // ==== TX ====
//        DBG1_SET();
//        CC.Transmit(&PktTx);
//        DBG1_CLR();
//
//        // If TX slot is not last, receive at zero cycle or sleep otherwise
//        if(TxSlot != (SLOT_CNT-1)) {
//            uint32_t TimeAfter = ((SLOT_CNT-1) - TxSlot) * SLOT_DURATION_MS;
////            Uart.Printf("TA=%u\r\r", TimeAfter);
//            if(CycleN == 0 and RxTable.GetCount() < RXTABLE_SZ) TryToReceive(TimeAfter);
//            else TryToSleep(TimeAfter);
//        }
//    } // for
//}

//void rLevel1_t::TryToReceive(uint32_t RxDuration) {
//    systime_t TotalDuration_st = MS2ST(RxDuration);
//    systime_t TimeStart = chVTGetSystemTimeX();
//    systime_t RxDur_st = TotalDuration_st;
//    while(true) {
//        uint8_t RxRslt = CC.Receive_st(RxDur_st, &PktRx, &Rssi);
//        if(RxRslt == retvOk) {
////            Uart.Printf("\rRID = %X", PktRx.DWord);
//            Uart.Printf("OtherRssi=%d\r", Rssi);
//            if(Rssi > RSSI_MIN) {
//                chSysLock();
//                RxTable.AddId(PktRx.DWord32);
//                chSysUnlock();
//            }
//        }
//        // Check if repeat or get out
//        systime_t Elapsed_st = chVTTimeElapsedSinceX(TimeStart);
//        if(Elapsed_st >= TotalDuration_st) break;
//        else RxDur_st = TotalDuration_st - Elapsed_st;
//    }
//}
#endif // task

void rLevel1_t::TryToSleep(uint32_t SleepDuration) {
    if(SleepDuration >= MIN_SLEEP_DURATION_MS) CC.EnterPwrDown();
    chThdSleepMilliseconds(SleepDuration);
}

#if 1 // ============================
uint8_t rLevel1_t::Init() {
#ifdef DBG_PINS
    PinSetupOut(DBG_GPIO1, DBG_PIN1, omPushPull);
//    PinSetupOut(DBG_GPIO2, DBG_PIN2, omPushPull);
#endif

    RMsgQ.Init();
    if(CC.Init() == retvOk) {
        CC.SetTxPower(CC_PwrMinus30dBm);
        CC.SetPktSize(RPKT_LEN);
        CC.SetChannel(ID2RCHNL(ID));
//        CC.EnterPwrDown();
        // Thread
        chThdCreateStatic(warLvl1Thread, sizeof(warLvl1Thread), HIGHPRIO, (tfunc_t)rLvl1Thread, NULL);
        return retvOk;
    }
    else return retvFail;
}
#endif
