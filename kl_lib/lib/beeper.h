/*
 * beeper.h
 *
 *  Created on: 22 марта 2015 г.
 *      Author: Kreyl
 */

#pragma once

#include "ChunkTypes.h"
#include "kl_lib.h"

class Beeper_t : public BaseSequencer_t<BeepChunk_t> {
private:
    const PinOutputPWM_t IPin;
    void ISwitchOff() { IPin.Set(0); }
    SequencerLoopTask_t ISetup() {
        IPin.SetFrequencyHz(IPCurrentChunk->Freq_Hz);
        IPin.Set(IPCurrentChunk->Volume);
        IPCurrentChunk++;   // Always goto next
        return sltProceed;  // Always proceed
    }
public:
    Beeper_t(const PwmSetup_t APinSetup) : BaseSequencer_t(), IPin(APinSetup) {}
    void Init() { IPin.Init(); }
    void Beep(uint32_t Freq_Hz, uint8_t Volume) {
        IPin.SetFrequencyHz(Freq_Hz);
        IPin.Set(Volume);
    }
    void Off() { IPin.Set(0); }
};
