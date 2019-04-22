/*
 * ChunkTypes.h
 *
 *  Created on: 08 џэт. 2015 у.
 *      Author: Kreyl
 */

#pragma once

#include "color.h"
#include "ch.h"
#include "MsgQ.h"

enum ChunkSort_t {csSetup, csWait, csGoto, csEnd, csRepeat};

// The beginning of any sort of chunk. Everyone must contain it.
#define BaseChunk_Vars \
    ChunkSort_t ChunkSort;          \
    union {                         \
        uint32_t Value;             \
        uint32_t Volume;            \
        uint32_t Time_ms;           \
        uint32_t ChunkToJumpTo;     \
        int32_t RepeatCnt;          \
    }

// ==== Different types of chunks ====
struct BaseChunk_t {
    BaseChunk_Vars;
};

// RGB LED chunk
struct LedRGBChunk_t {
    BaseChunk_Vars;
    Color_t Color;
} __attribute__((packed));

// HSV LED chunk
struct LedHSVChunk_t {
    BaseChunk_Vars;
    ColorHSV_t Color;
} __attribute__((packed));

// LED Smooth
struct LedSmoothChunk_t {
    BaseChunk_Vars;
    uint8_t Brightness;
} __attribute__((packed));

// Beeper
struct BeepChunk_t {   // Value == Volume
    BaseChunk_Vars;
    uint16_t Freq_Hz;
} __attribute__((packed));


#if 1 // ====================== Base sequencer class ===========================
enum SequencerLoopTask_t {sltProceed, sltBreak};

template <class TChunk>
class BaseSequencer_t : private IrqHandler_t {
protected:
    virtual_timer_t ITmr;
    const TChunk *IPStartChunk, *IPCurrentChunk;
    int32_t RepeatCounter = -1;
    EvtMsg_t IEvtMsg;
    virtual void ISwitchOff() = 0;
    virtual SequencerLoopTask_t ISetup() = 0;
    void SetupDelay(uint32_t ms) { chVTSetI(&ITmr, TIME_MS2I(ms), TmrKLCallback, this); }

    // Process sequence
    void IIrqHandler() {
        if(chVTIsArmedI(&ITmr)) chVTResetI(&ITmr);  // Reset timer
        while(true) {   // Process the sequence
            switch(IPCurrentChunk->ChunkSort) {
                case csSetup: // setup now and exit if required
                    if(ISetup() == sltBreak) return;
                    break;

                case csWait: { // Start timer, pointing to next chunk
                        uint32_t Delay = IPCurrentChunk->Time_ms;
                        IPCurrentChunk++;
                        if(Delay != 0) {
                            SetupDelay(Delay);
                            return;
                        }
                    }
                    break;

                case csGoto:
                    IPCurrentChunk = IPStartChunk + IPCurrentChunk->ChunkToJumpTo;
                    if(IEvtMsg.ID != evtIdNone) EvtQMain.SendNowOrExitI(IEvtMsg);
                    SetupDelay(1);
                    return;
                    break;

                case csEnd:
                    if(IEvtMsg.ID != evtIdNone) EvtQMain.SendNowOrExitI(IEvtMsg);
                    IPStartChunk = nullptr;
                    IPCurrentChunk = nullptr;
                    return;
                    break;

                case csRepeat:
                    if(RepeatCounter == -1) RepeatCounter = IPCurrentChunk->RepeatCnt;
                    if(RepeatCounter == 0) {    // All was repeated, goto next
                        RepeatCounter = -1;     // reset counter
                        IPCurrentChunk++;
                    }
                    else {  // repeating in progress
                        IPCurrentChunk = IPStartChunk;  // Always from beginning
                        RepeatCounter--;
                    }
                    break;
            } // switch
        } // while
    } // IProcessSequenceI
public:
    void SetupSeqEndEvt(EvtMsg_t AEvtMsg) { IEvtMsg = AEvtMsg; }

    void StartOrRestart(const TChunk *PChunk) {
        chSysLock();
        RepeatCounter = -1;
        IPStartChunk = PChunk;   // Save first chunk
        IPCurrentChunk = PChunk;
        IIrqHandler();
        chSysUnlock();
    }

    void StartOrContinue(const TChunk *PChunk) {
        if(PChunk == IPStartChunk) return; // Same sequence
        else StartOrRestart(PChunk);
    }

    void Stop() {
        if(IPStartChunk != nullptr) {
            chSysLock();
            if(chVTIsArmedI(&ITmr)) chVTResetI(&ITmr);
            IPStartChunk = nullptr;
            IPCurrentChunk = nullptr;
            chSysUnlock();
        }
        ISwitchOff();
    }
    const TChunk* GetCurrentSequence() { return IPStartChunk; }
    bool IsIdle() { return (IPStartChunk == nullptr and IPCurrentChunk == nullptr); }
};
#endif
