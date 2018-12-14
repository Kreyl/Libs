/*
 * EvtMsg.h
 *
 *  Created on: 21 апр. 2017 г.
 *      Author: Kreyl
 */

#pragma once

#include <inttypes.h>
#include "ch.h"
#include "kl_lib.h"
#include "EvtMsgIDs.h"
//#include "board.h"
#if BUTTONS_ENABLED
#include "buttons.h"
#endif

/*
 * Example of other msg:
 * union RMsg_t {
    uint32_t DWord[3];
    CmdUniversal_t Cmd;
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
    RMsg_t(CmdUniversal_t *PCmd) {
        Cmd.CmdID = PCmd->CmdID;
        Cmd.SnsID = PCmd->SnsID;
        Cmd.w16[0] = PCmd->w16[0];
        Cmd.w16[1] = PCmd->w16[1];
        Cmd.w16[2] = PCmd->w16[2];
    }
} __attribute__((__packed__));
[...]
EvtMsgQ_t<RMsg_t, RMSG_Q_LEN> MsgQ;
 */

#define MAIN_EVT_Q_LEN      18  // Messages in queue
#define EMSG_DATA8_CNT      7   // ID + 7 bytes = 8 = 2x DWord32
#define EMSG_DATA16_CNT     3   // ID + 3x2bytes = 7

union EvtMsg_t {
    uint32_t DWord[2];
    struct {
        union {
            void* Ptr;
            struct {
                int32_t Value;
                uint8_t ValueID;
            } __attribute__((__packed__));
//            uint8_t b[EMSG_DATA8_CNT];
//            uint16_t w16[EMSG_DATA16_CNT];
#if BUTTONS_ENABLED
            BtnEvtInfo_t BtnEvtInfo;
#endif
        } __attribute__((__packed__));
        uint8_t ID;
    } __attribute__((__packed__));

    EvtMsg_t& operator = (const EvtMsg_t &Right) {
        DWord[0] = Right.DWord[0];
        DWord[1] = Right.DWord[1];
        return *this;
    }
    EvtMsg_t() : Ptr(nullptr), ID(0) {}
    EvtMsg_t(uint8_t AID) : ID(AID) {}
    EvtMsg_t(uint8_t AID, void *APtr) : Ptr(APtr), ID(AID) {}
    EvtMsg_t(uint8_t AID, int32_t AValue) : Value(AValue), ID(AID) {}
    EvtMsg_t(uint8_t AID, uint8_t AValueID, int32_t AValue) : Value(AValue), ValueID(AValueID), ID(AID) {}
} __attribute__((__packed__));


template<typename T, uint32_t Sz>
class EvtMsgQ_t {
private:
    union {
        uint64_t __Align;
        T IBuf[Sz];
    };
    T *ReadPtr, *WritePtr;
    semaphore_t FullSem;    // Full counter
    semaphore_t EmptySem;   // Empty counter
public:
    EvtMsgQ_t() : __Align(0), ReadPtr(IBuf), WritePtr(IBuf) {}
    void Init() {
        ReadPtr = IBuf;
        WritePtr = IBuf;
        chSemObjectInit(&EmptySem, Sz);
        chSemObjectInit(&FullSem, (cnt_t)0);
    }

    /* Retrieves a message from a mailbox, returns zero Msg if failed.
     * The invoking thread waits until a message is posted in the mailbox
     * for a timeout (may be TIME_INFINITE or TIME_IMMEDIATE */
    T Fetch(systime_t Timeout) {
        T Msg;
        *(uint8_t*)&Msg = 0;    // Init it with zero somehow
        chSysLock();
        if(chSemWaitTimeoutS(&FullSem, Timeout) == MSG_OK) {
            // There is something in the queue, get it
            Msg = *ReadPtr++;
            if(ReadPtr >= &IBuf[Sz]) ReadPtr = IBuf;  // Circulate pointer
            chSemSignalI(&EmptySem);
            chSchRescheduleS();
        }
        chSysUnlock();
        return Msg;
    }

    /* Posts a message into a mailbox.
     * The function returns a timeout condition if the queue is full */
    uint8_t SendNowOrExitI(const T &Msg) {
        if(chSemGetCounterI(&EmptySem) <= (cnt_t)0) return retvTimeout; // Q is full
        chSemFastWaitI(&EmptySem);
        *WritePtr++ = Msg;
        if(WritePtr >= &IBuf[Sz]) WritePtr = IBuf;  // Circulate pointer
        chSemSignalI(&FullSem);
        return retvOk;
    }

    uint8_t SendNowOrExit(const T &Msg) {
        chSysLock();
        uint8_t Rslt = SendNowOrExitI(Msg);
        chSchRescheduleS();
        chSysUnlock();
        return Rslt;
    }

    /* Posts a message into a mailbox.
     * The invoking thread waits until a empty slot in the mailbox becomes available
     * or the specified time runs out. */
    uint8_t SendWaitingAbility(const T &Msg, systime_t timeout) {
        chSysLock();
        msg_t rdymsg = chSemWaitTimeoutS(&EmptySem, timeout);
        if(rdymsg == MSG_OK) {
            *WritePtr++ = Msg;
            if(WritePtr >= &IBuf[Sz]) WritePtr = IBuf;  // Circulate pointer
            chSemSignalI(&FullSem);
            chSchRescheduleS();
        }
        chSysUnlock();
        if(rdymsg == MSG_TIMEOUT) return retvTimeout;
        else if(rdymsg == MSG_OK) return retvOk;
        else return retvFail;
    }

    uint32_t GetFullCnt() {
        chSysLock();
        uint32_t rslt = chSemGetCounterI(&FullSem);
        chSysUnlock();
        return rslt;
    }
};

/* Always presents in main:
 * EvtMsgQ_t<EvtMsg_t, MAIN_EVT_Q_LEN> EvtQMain;
 * ...
 * EvtQMain.Init();
 * ...
 * EvtMsg_t Msg = EvtQMain.Fetch(TIME_INFINITE);
 * switch(Msg.ID) {
        case evtIdButtons:
        break;
        ...
        default: Printf("Unhandled Msg %u\r", Msg.ID); break;
    } // Switch
 */
extern EvtMsgQ_t<EvtMsg_t, MAIN_EVT_Q_LEN> EvtQMain;
