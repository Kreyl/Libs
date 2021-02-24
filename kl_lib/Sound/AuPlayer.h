/*
 * AuPlayer.h
 *
 *  Created on: 4 ���� 2017 �.
 *      Author: Kreyl
 */

#pragma once

#include "kl_lib.h"
#include "ff.h"
#include "CS42L52.h"
#include "audiotrack.h"
#include "kl_fs_utils.h"

#define BUF_SZ_FRAME        1024UL

// Callbacks. Returns true if OK
size_t TellCallback(void *file_context);
bool SeekCallback(void *file_context, size_t offset);
size_t ReadCallback(void *file_context, uint8_t *buffer, size_t length);
extern CS42L52_t Codec;

struct SndBuf_t {
    uint32_t Buf[BUF_SZ_FRAME], Sz;
};

enum PlayMode_t {spmSingle, spmRepeat};

class FSound_t {
private:
    FIL IFile;
public:
    AudioTrack Track {TellCallback, SeekCallback, ReadCallback, 2};
    SndBuf_t Buf1, Buf2;
    bool IsSoundFx = false;
    uint8_t Start(PlayMode_t Mode) {
        return Track.start(&IFile,
                (Mode == spmSingle)? AudioTrack::Mode::Single : AudioTrack::Mode::Continuous,
                        true)? retvOk : retvFail;
    }
    uint8_t TryOpenFile(const char* AFname) {
        f_close(&IFile);
        return TryOpenFileRead(AFname, &IFile);
    }
    void ReadToBuf(SndBuf_t *PBuf) {
        PBuf->Sz = Track.play((int16_t*)PBuf->Buf, BUF_SZ_FRAME); // returns how many frames was read
    }
    void FadeOut();
    bool IsPlaying() { return (Track.running() or Buf1.Sz != 0 or Buf2.Sz != 0); }
};

class AuPlayer_t {
private:
    FSound_t ISnd1, ISnd2, *ICurSnd, *INextSnd;
    void ISwitchSnds();
    SndBuf_t *PCurBuf;
    void IPlayNext(const char* AFName, PlayMode_t AMode = spmSingle);
    void IPrepareToPlayNext(const char* AFName, PlayMode_t AMode = spmSingle);
public:
    void Init();

    const char* FileToPlayNext;

    void Play(const char* AFName, PlayMode_t AMode);
    void Stop();
    void WaitEnd();
    PlayMode_t GetPlaymode() {
        return (ICurSnd->Track.mode() == WavReader::Mode::Single)? spmSingle : spmRepeat;
    }
    bool IsPlayingNow() { return ICurSnd->IsPlaying(); }
    void FadeOut() { ICurSnd->FadeOut(); }

    // Inner use
    void ITask();
    void IHandleIrq();
    void TransmitBuf(SndBuf_t *PBuf) {
        Codec.TransmitBuf(PBuf->Buf, PBuf->Sz*2);    // Sz16 == SzFrame*2
    }
    AuPlayer_t() : ISnd1(), ISnd2(), ICurSnd(&ISnd1), INextSnd(&ISnd2),
            PCurBuf(nullptr), FileToPlayNext(nullptr) {}
};

extern AuPlayer_t AuPlayer;
