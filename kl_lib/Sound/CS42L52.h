/*
 * CS42L52.h
 *
 *  Created on: 15 марта 2017 г.
 *      Author: Kreyl
 */

#pragma once

#include "kl_lib.h"

#define MIC_EN              FALSE

#define CS42_I2C_ADDR       0x4A
#define AU_BATMON_ENABLE    TRUE
#define AU_VA_mv            2500    // Required for battery voltage calculation

enum AuBeepFreq_t {abfC4=0b0000, abfC5=0b0001, abfD5=0b0010, abfE5=0b0011, abfF5=0b0100, abfG5=0b0101, abfA5=0b0110,
    abfB5=0b0111, abfC6=0b1000, abfD6=0b1001, abfE6=0b1010, abfF6=0b1011, abfG6=0b1100, abfA6=0b1101, abfB6=0b1110, abfC7=0b1111
};

enum AuBeepOnTime_t { abot86=0b0000, abot430=0b0001, abot780=0b0010, abot1s20=0b0011, abot1s50=0b0100, abot1s80=0b0101 };

union SampleStereo_t {
    uint32_t DWord32;
    struct {
        int16_t Left, Right;
    } __packed;
} __packed;

typedef int16_t SampleMono_t;

enum MonoStereo_t { Stereo, Mono };

class CS42L52_t {
private:
    void EnableSAI() {
        AU_SAI_A->CR1 |= SAI_xCR1_SAIEN;
#if MIC_EN
        AU_SAI_B->CR1 |= SAI_xCR1_SAIEN;
#endif
    }
    void DisableSAI() {
        AU_SAI_A->CR1 &= ~SAI_xCR1_SAIEN;
#if MIC_EN
        AU_SAI_B->CR1 &= ~SAI_xCR1_SAIEN;
#endif
    }
    int8_t IVolume = 0;
    bool IsOn;
public:
    void Init();
    void Standby();
    void Resume();
    u8 ReadReg(u8 RegAddr, u8 *PValue);
    u8 WriteReg(u8 RegAddr, u8 Value);
    u8 WriteMany(u8 StartAddr, u8 *PValues, u8 Len);
    u8 WriteTwoTheSame(u8 StartAddr, u8 Value);
    // Mid-level functions
    void SetMicGain(u8 Gain);
    u8 SetPGAGain(i8 Gain);
    u8 SetAdcVolume(i8 Volume_dB);
    u8 SetAdcMixerVolume(i8 Volume_dB);
    u8 SetPcmMixerVolume(i8 Volume_dB);

    u8 MuteAdcMixer() { return WriteTwoTheSame(0x18, 0x80); } // Set mixer mute bit
    u8 MutePcmMixer() { return WriteTwoTheSame(0x1A, 0x80); } // Set mixer mute bit

    void SetupNoiseGate(EnableDisable_t En, uint8_t Threshold, uint8_t Delay);

    void GetStatus();

    // Hi-level
    void BeepSingle(AuBeepFreq_t Freq, AuBeepOnTime_t OnTime, int8_t Volume_dB);
    u8 SetMasterVolume(i8 Volume_dB);
    u8 SetHeadphoneVolume(i8 Volume_dB);
    u8 SetSpeakerVolume(i8 Volume_dB);

    void VolumeUp();
    void VolumeDown();
    void SetVolume(int8_t AVolume);
    int8_t GetVolume() { return IVolume; }

    // Enable/Disable
    void EnableMicSystem();
    void DisableMicSystem();
    void EnableHeadphones();
    void DisableHeadphones();
    void EnableSpeakerMono();
    void DisableSpeakers();

    // Rx/Tx
    void SetupMonoStereo(MonoStereo_t MonoStereo);
    void SetupSampleRate(uint32_t SampleRate);
    void TransmitBuf(void *Buf, uint32_t Sz16);
    bool IsTransmitting();
    void Stop();

    void StartStream();
    void PutSampleI(SampleStereo_t &Sample);
#if AU_BATMON_ENABLE
    uint32_t GetBatteryVmv();
#endif
};

void AuOnNewSampleI(SampleStereo_t &Sample);
