/*
 * AuPlayer.h
 *
 *  Created on: 4 θώνÿ 2017 γ.
 *      Author: Kreyl
 */

#pragma once

#include "kl_lib.h"
#include "ff.h"
#include "kl_fs_utils.h"

#define FRAME_BUF_SZ    (1152UL * 2UL * 2UL) // 1152 by MP3 standard, 2 chnls, 16bit per chnl
#define DECODE_BUF_SZ   1024

struct SndBuf_t {
    uint32_t SampleCnt;
    uint8_t Buf[FRAME_BUF_SZ];
};

class AuPlayer_t {
private:
    FIL IFile;
    SndBuf_t IBuf1, IBuf2, *PCurBuf = &IBuf1;
    void IPlay();
    void ReadToBuf(SndBuf_t *PBuf, uint32_t *PNextSampleRate);
    uint32_t CurrSampleRate, NextSampleRate;
public:
    void Init();

    void Play(char* AFName, char* AKey, char* AIV);
    uint8_t Pause();
    uint8_t Stop();
    uint8_t Resume();
    void WaitEnd();
    // Inner use
    void ITask();
    void IHandleIrq();
    void TransmitBuf(SndBuf_t *PBuf);
};

extern AuPlayer_t Player;
