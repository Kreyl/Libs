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

#define MAIN_EVT_Q_LEN      18  // Messages in queue
#define EMSG_DATA_SZ        7   // ID + 7 bytes = 2x DWord32

union EvtMsg_t {
    uint32_t DWord[2];
    struct {
        uint8_t ID;
        union {
            void* Ptr;
            uint8_t b[EMSG_DATA_SZ];
        };
    };
    EvtMsg_t& operator = (const EvtMsg_t &Right) {
        DWord[0] = Right.DWord[0];
        DWord[1] = Right.DWord[1];
        return *this;
    }
    EvtMsg_t() : ID(0), Ptr(nullptr) {}
    EvtMsg_t(uint8_t AID) : ID(AID) {}
    EvtMsg_t(uint8_t AID, void *APtr) : ID(AID), Ptr(APtr) {}
} __attribute__((__packed__));


template<uint32_t Sz>
class EvtMsgQ_t {
private:
    EvtMsg_t IBuf[Sz];
    EvtMsg_t *ReadPtr = IBuf, *WritePtr = IBuf;
    semaphore_t FullSem;    // Full counter
    semaphore_t EmptySem;   // Empty counter
public:
    void Init() {
        chSemObjectInit(&EmptySem, Sz);
        chSemObjectInit(&FullSem, (cnt_t)0);
    }

    /* Retrieves a message from a mailbox, returns zero Msg if failed.
     * The invoking thread waits until a message is posted in the mailbox
     * for a timeout (may be TIME_INFINITE or TIME_IMMEDIATE */
    EvtMsg_t Fetch(systime_t Timeout) {
        EvtMsg_t Msg;
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
    uint8_t SendNowOrExitI(EvtMsg_t Msg) {
        if(chSemGetCounterI(&EmptySem) <= (cnt_t)0) return retvTimeout; // Q is full
        chSemFastWaitI(&EmptySem);
        *WritePtr++ = Msg;
        if(WritePtr >= &IBuf[Sz]) WritePtr = IBuf;  // Circulate pointer
        chSemSignalI(&FullSem);
        return retvOk;
    }
    uint8_t SendNowOrExitI(uint8_t MsgID) {
        EvtMsg_t Msg(MsgID);
        return SendNowOrExitI(Msg);
    }

    uint8_t SendNowOrExit(EvtMsg_t Msg) {
        chSysLock();
        uint8_t Rslt = SendNowOrExitI(Msg);
        chSysUnlock();
        return Rslt;
    }

    /* Posts a message into a mailbox.
     * The invoking thread waits until a empty slot in the mailbox becomes available
     * or the specified time runs out. */
    uint8_t SendWaitingAbility(EvtMsg_t Msg, systime_t timeout) {
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
};

// Always presents in main
extern EvtMsgQ_t<MAIN_EVT_Q_LEN> MainEvtQ;
