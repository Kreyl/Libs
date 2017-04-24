/*
 * ChunkTypes.h
 *
 *  Created on: 08 џэт. 2015 у.
 *      Author: Kreyl
 */

#pragma once

#include "color.h"
#include "ch.h"
//#include "uart.h"

enum ChunkSort_t {csSetup, csWait, csGoto, csEnd};

// The beginning of any sort of chunk. Everyone must contain it.
#define BaseChunk_Vars \
    ChunkSort_t ChunkSort;          \
    union {                         \
        uint32_t Value;             \
        uint32_t Volume;            \
        uint32_t Time_ms;           \
        uint32_t ChunkToJumpTo;     \
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
    thread_t *PThread;
    eventmask_t EvtEnd;
    virtual void ISwitchOff() = 0;
    virtual SequencerLoopTask_t ISetup() = 0;
    void SetupDelay(uint32_t ms) { chVTSetI(&ITmr, MS2ST(ms), TmrKLCallback, this); }

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
                    SetupDelay(1);
                    return;
                    break;

                case csEnd:
//                    if(PThread != nullptr) chEvtSignalI(PThread, EvtEnd); // XXX
                    IPStartChunk = nullptr;
                    IPCurrentChunk = nullptr;
                    return;
                    break;
            } // switch
        } // while
    } // IProcessSequenceI
public:
    void SetupSeqEndEvt(thread_t *APThread, eventmask_t AEvt = 0) {
        PThread = APThread;
        EvtEnd = AEvt;
    }

    void StartOrRestart(const TChunk *PChunk) {
        chSysLock();
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
};
#endif
