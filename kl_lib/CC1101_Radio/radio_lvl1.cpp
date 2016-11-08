/*
 * radio_lvl1.cpp
 *
 *  Created on: Nov 17, 2013
 *      Author: kreyl
 */

#include "radio_lvl1.h"
#include "evt_mask.h"
#include "main.h"
#include "cc1101.h"
#include "uart.h"
//#include "led.h"

//#define DBG_PINS

#ifdef DBG_PINS
#define DBG_GPIO1   GPIOB
#define DBG_PIN1    4
#define DBG1_SET()  PinSet(DBG_GPIO1, DBG_PIN1)
#define DBG1_CLR()  PinClear(DBG_GPIO1, DBG_PIN1)
#define DBG_GPIO2   GPIOB
#define DBG_PIN2    9
#define DBG2_SET()  PinSet(DBG_GPIO2, DBG_PIN2)
#define DBG2_CLR()  PinClear(DBG_GPIO2, DBG_PIN2)
#else
#define DBG1_SET()
#define DBG1_CLR()
#endif

rLevel1_t Radio;

#if 1 // ================================ Task =================================
static THD_WORKING_AREA(warLvl1Thread, 256);
__noreturn
static void rLvl1Thread(void *arg) {
    chRegSetThreadName("rLvl1");
    while(true) {
        switch(App.Mode) {
            case modeTx:
                if(Radio.MustTx) Radio.TaskTransmitter();
                else Radio.TryToSleep(450);
                break;

            case modeLevel1:
            case modeLevel2:
                Radio.TaskReceiverSingle(); // Rx part
                Radio.TaskFeelEachOtherMany();
                break;
        } // switch
    } // while true
}

void rLevel1_t::TaskTransmitter() {
//    CC.SetChannel(ID2RCHNL(App.ID));
    CC.SetChannel(RCHNL_COMMON);
    PktTx.DWord32 = THE_WORD;
    DBG1_SET();
    CC.Transmit(&PktTx);
    DBG1_CLR();
}

void rLevel1_t::TaskReceiverSingle() {
//    uint8_t Ch = ID2RCHNL(App.ID - 1);
//    CC.SetChannel(Ch);
    CC.SetChannel(RCHNL_COMMON);
    uint8_t RxRslt = CC.Receive(11, &PktRx, &Rssi);   // Double pkt duration + TX sleep time
    if(RxRslt == OK) {
//        Uart.Printf("Ch=%u; Rssi=%d\r", Ch, Rssi);
        Uart.Printf("ForceRssi=%d\r", Rssi);
//        if(PktRx.DWord32 == THE_WORD and Rssi > -63) App.SignalEvt(EVT_RADIO);
        if(PktRx.DWord32 == THE_WORD) App.SignalEvt(EVT_RADIO_FORCE);
    }
}

void rLevel1_t::TaskReceiverMany() {
    for(int N=0; N<2; N++) {
        // Iterate channels
        for(int32_t i = ID_MIN; i <= ID_MAX; i++) {
            if(i == App.ID) continue;   // Do not listen self
            CC.SetChannel(ID2RCHNL(i));
            uint8_t RxRslt = CC.Receive(17, &PktRx, &Rssi);   // Double pkt duration + TX sleep time
            if(RxRslt == OK) {
//                Uart.Printf("Ch=%u; Rssi=%d\r", ID2RCHNL(i), Rssi);
                if(PktRx.DWord32 == THE_WORD and Rssi > RSSI_MIN) RxTable.AddId(i);
                else Uart.Printf("PktErr\r");
            }
        } // for i
        TryToSleep(270);
    } // For N
//    App.SignalEvt(EVT_RADIO); // RX table ready
}

void rLevel1_t::TaskFeelEachOtherSingle() {
    int8_t TopRssi = -126;
    // Alice is boss
    if((App.ID & 0x01) == 1) {
        CC.SetChannel(ID2RCHNL(App.ID));
        for(int i=0; i<7; i++) {
            DBG1_SET();
            CC.Transmit(&PktTx);
            DBG1_CLR();
            // Listen for an answer
            uint8_t RxRslt = CC.Receive(18, &PktRx, &Rssi);   // Double pkt duration + TX sleep time
            if(RxRslt == OK) {
//                Uart.Printf("i=%d; Rssi=%d\r", i, Rssi);
                if(Rssi > TopRssi) TopRssi = Rssi;
            }
            TryToSleep(126);
        } // for
    }
    // Bob does what Alice says
    else {
        CC.SetChannel(ID2RCHNL(App.ID - 1));
        for(int i=0; i<7; i++) {
            // Listen for a command
            uint8_t RxRslt = CC.Receive(144, &PktRx, &Rssi);   // Double pkt duration + TX sleep time
            if(RxRslt == OK) {
//                Uart.Printf("i=%d; Rssi=%d\r", i, Rssi);
                if(Rssi > TopRssi) TopRssi = Rssi;
                // Transmit reply
                DBG1_SET();
                CC.Transmit(&PktTx);
                DBG1_CLR();
            }
            TryToSleep(45);
        } // for
    }
    // Signal Evt if something received
    if(TopRssi > -126) {
        Rssi = TopRssi;
//        App.SignalEvt(EVT_RADIO);
    }
}

void rLevel1_t::TaskFeelEachOtherMany() {
    CC.SetChannel(RCHNL_EACH_OTH);
    CC.SetTxPower(TxPwr);
    PktTx.DWord32 = App.ID;
    for(uint32_t CycleN=0; CycleN < CYCLE_CNT; CycleN++) {  // Iterate cycles
        uint32_t TxSlot = Random(0, (SLOT_CNT-1));          // Decide when to transmit
        // If TX slot is not zero, receive at zero cycle or sleep otherwise
//        Uart.Printf("Txs=%u C=%u\r", TxSlot, CycleN);
        if(TxSlot != 0) {
            uint32_t TimeBefore = TxSlot * SLOT_DURATION_MS;
//            Uart.Printf("TB=%u\r", TimeBefore);
            if(CycleN == 0 and RxTable.GetCount() < RXTABLE_SZ) TryToReceive(TimeBefore);
            else TryToSleep(TimeBefore);
        }
        // ==== TX ====
        DBG1_SET();
        CC.Transmit(&PktTx);
        DBG1_CLR();

        // If TX slot is not last, receive at zero cycle or sleep otherwise
        if(TxSlot != (SLOT_CNT-1)) {
            uint32_t TimeAfter = ((SLOT_CNT-1) - TxSlot) * SLOT_DURATION_MS;
//            Uart.Printf("TA=%u\r\r", TimeAfter);
            if(CycleN == 0 and RxTable.GetCount() < RXTABLE_SZ) TryToReceive(TimeAfter);
            else TryToSleep(TimeAfter);
        }
    } // for
}

void rLevel1_t::TryToReceive(uint32_t RxDuration) {
    systime_t TotalDuration_st = MS2ST(RxDuration);
    systime_t TimeStart = chVTGetSystemTimeX();
    systime_t RxDur_st = TotalDuration_st;
    while(true) {
        uint8_t RxRslt = CC.Receive_st(RxDur_st, &PktRx, &Rssi);
        if(RxRslt == OK) {
//            Uart.Printf("\rRID = %X", PktRx.DWord);
            Uart.Printf("OtherRssi=%d\r", Rssi);
            if(Rssi > RSSI_MIN) {
                chSysLock();
                RxTable.AddId(PktRx.DWord32);
                chSysUnlock();
            }
        }
        // Check if repeat or get out
        systime_t Elapsed_st = chVTTimeElapsedSinceX(TimeStart);
        if(Elapsed_st >= TotalDuration_st) break;
        else RxDur_st = TotalDuration_st - Elapsed_st;
    }
}
#endif // task

#if 1 // ============================
uint8_t rLevel1_t::Init() {
#ifdef DBG_PINS
    PinSetupOut(DBG_GPIO1, DBG_PIN1, omPushPull);
    PinSetupOut(DBG_GPIO2, DBG_PIN2, omPushPull);
#endif    // Init radioIC
    if(CC.Init() == OK) {
        CC.SetTxPower(CC_Pwr0dBm);
        CC.SetPktSize(RPKT_LEN);
//        CC.SetChannel(ID2RCHNL(App.ID));
//        CC.EnterPwrDown();
        // Thread
        PThd = chThdCreateStatic(warLvl1Thread, sizeof(warLvl1Thread), HIGHPRIO, (tfunc_t)rLvl1Thread, NULL);
        return OK;
    }
    else return FAILURE;
}
#endif
