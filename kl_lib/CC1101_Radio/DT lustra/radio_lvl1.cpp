/*
 * radio_lvl1.cpp
 *
 *  Created on: Nov 17, 2013
 *      Author: kreyl
 */

#include "radio_lvl1.h"
#include "cc1101.h"
#include "MsgQ.h"
#include "led.h"
#include "Sequences.h"
#include "EvtMsgIDs.h"

cc1101_t CC(CC_Setup0);
extern uint16_t ID;
extern uint8_t Influence;

#define DBG_PINS

#ifdef DBG_PINS
#define DBG_GPIO1   GPIOC
#define DBG_PIN1    13
#define DBG1_SET()  PinSetHi(DBG_GPIO1, DBG_PIN1)
#define DBG1_CLR()  PinSetLo(DBG_GPIO1, DBG_PIN1)
//#define DBG_GPIO2   GPIOB
//#define DBG_PIN2    9
//#define DBG2_SET()  PinSet(DBG_GPIO2, DBG_PIN2)
//#define DBG2_CLR()  PinClear(DBG_GPIO2, DBG_PIN2)
#else
#define DBG1_SET()
#define DBG1_CLR()
#endif

rLevel1_t Radio;
void TmrTimeslotCallback(void *p);
static volatile enum CCState_t {ccstIdle, ccstRx, ccstTx} CCState = ccstIdle;


static class RadioTime_t {
private:
    virtual_timer_t TmrTimeslot;
    void StopTimerI()  { chVTResetI(&TmrTimeslot); }
    uint16_t TimeSrcTimeout;
public:
    volatile uint32_t CycleN = 0, TimeSlot = 0;
    uint16_t TimeSrcId = ID;
    void StartTimerI() { chVTSetI(&TmrTimeslot, TIMESLOT_DURATION_ST, TmrTimeslotCallback, nullptr); }
    void IncTimeSlot() {
        TimeSlot++;
        if(TimeSlot >= RSLOT_CNT) {
            TimeSlot = 0;
            CycleN++;
            if(CycleN >= RCYCLE_CNT) {
                CycleN = 0;
                // Check TimeSrc timeout
                if(TimeSrcTimeout >= SCYCLES_TO_KEEP_TIMESRC) TimeSrcId = ID;
                else TimeSrcTimeout++;
            }
        }
    }
    void Adjust() {
        // Adjust time if theirs TimeSrc < OursTimeSrc
        if(Radio.PktRx.TimeSrcID < TimeSrcId) {
            chSysLock();
            StopTimerI();
            CycleN = Radio.PktRx.Cycle;
            TimeSlot = Radio.PktRx.ID;
            TimeSrcId = Radio.PktRx.TimeSrcID;
            TimeSrcTimeout = 0; // Reset Time Src Timeout
            IOnNewTimeslot();
            chSysUnlock();
//            Printf("New time: ccl %u; slot %u; Src %u\r", CycleN, TimeSlot, TimeSrcId);
        }
    }

    void IOnNewTimeslot() {
        StartTimerI();
        IncTimeSlot();
        if(TimeSlot == ID) Radio.RMsgQ.SendNowOrExitI(RMsg_t(rmsgTimeToTx));
        else { // Not our timeslot
            if(CycleN == 0) { // Enter RX if not yet
                if(CCState != ccstRx) Radio.RMsgQ.SendNowOrExitI(RMsg_t(rmsgTimeToRx));
            }
            else { // CycleN != 0
                if(CCState != ccstIdle) Radio.RMsgQ.SendNowOrExitI(RMsg_t(rmsgTimeToSleep));
            }
        }

    }
} RadioTime;


void TmrTimeslotCallback(void *p) {
    chSysLockFromISR();
    RadioTime.IOnNewTimeslot();
    chSysUnlockFromISR();
}

void RxCallback() {
    Radio.RMsgQ.SendNowOrExitI(RMsg_t(rmsgPktRx));
}

#if 1 // ================================ Task =================================
static THD_WORKING_AREA(warLvl1Thread, 256);
__noreturn
static void rLvl1Thread(void *arg) {
    chRegSetThreadName("rLvl1");
    Radio.ITask();
}

__noreturn
void rLevel1_t::ITask() {
    while(true) {
        RMsg_t msg = RMsgQ.Fetch(TIME_INFINITE);
        switch(msg.Cmd) {
            case rmsgTimeToTx:  // will never be here if firefly
//                Printf("Tttx\r");
                CCState = ccstTx;
                PktTx.ID = ID;
                PktTx.Cycle = RadioTime.CycleN;
                PktTx.TimeSrcID = RadioTime.TimeSrcId;
                PktTx.Influence = Influence;
//                Printf("ST: %u;  ", chVTGetSystemTimeX());
//                PktTx.Print();
                DBG1_SET();
                CC.Recalibrate(); // Recalibrate before every TX, do not calibrate before RX
                CC.Transmit(&PktTx, RPKT_LEN);
                DBG1_CLR();
                break;

            case rmsgTimeToRx:
//                Printf("Ttrx\r");
                if(ID == ID_FIREFLY) CC.Recalibrate();
                CCState = ccstRx;
                CC.ReceiveAsync(RxCallback);
                break;

            case rmsgTimeToSleep:
                CCState = ccstIdle;
                CC.EnterIdle();
                break;

            case rmsgPktRx:
                CCState = ccstIdle;
                if(CC.ReadFIFO(&PktRx, &Rssi, RPKT_LEN) == retvOk) {  // if pkt successfully received
//                    Printf("Rssi %d; ", Rssi);
//                    PktRx.Print();
                    RadioTime.Adjust();
                    EvtMsg_t Msg(evtIdNewRPkt);
                    Msg.b[0] = PktRx.Influence;
                    Msg.b[1] = PktRx.Param;
                    Msg.b[2] = (uint8_t)Rssi;
                    EvtQMain.SendNowOrExit(Msg);
                }
                break;

            case rmsgSetPwr: CC.SetTxPower(msg.Value); break;
            case rmsgSetChnl: CC.SetChannel(msg.Value); break;
        } // switch
    } // while
}
#endif // task

#if 1 // ============================
uint8_t rLevel1_t::Init() {
#ifdef DBG_PINS
    PinSetupOut(DBG_GPIO1, DBG_PIN1, omPushPull);
//    PinSetupOut(DBG_GPIO2, DBG_PIN2, omPushPull);
#endif    // Init radioIC

    RMsgQ.Init();
    if(CC.Init() == retvOk) {
        CC.SetPktSize(RPKT_LEN);
        CC.SetChannel(RCHNL);
        CC.SetTxPower(CC_TX_PWR);
        // Measure timeslot duration
//        systime_t TimeStart = chVTGetSystemTimeX();
//        CC.Recalibrate();
//        CC.Transmit(&PktTx, RPKT_LEN);
//        systime_t TimeslotDuration = chVTTimeElapsedSinceX(TimeStart);
//        Printf("Timeslot duration, systime: %u\r", TimeslotDuration);
        chSysLock();
        RadioTime.StartTimerI();
        RadioTime.TimeSrcId = ID;
        chSysUnlock();

        // Thread
        chThdCreateStatic(warLvl1Thread, sizeof(warLvl1Thread), HIGHPRIO, (tfunc_t)rLvl1Thread, NULL);
        return retvOk;
    }
    else return retvFail;
}
#endif
