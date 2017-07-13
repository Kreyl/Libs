/*
 * AuPlayer.h
 *
 *  Created on: 4 θώνÿ 2017 γ.
 *      Author: Kreyl
 */

#pragma once

#include "kl_lib.h"
#include "ff.h"

#define FRAME_BUF_SZ        4096
#define MAX_NAME_LEN        128

class AuPlayer_t {
private:
    FIL IFile;
    uint32_t PreviousN;
    uint32_t Buf1[(FRAME_BUF_SZ/4)], Buf2[(FRAME_BUF_SZ/4)], *PCurBuf, BufSz;
    uint8_t OpenWav(const char* AFileName);
public:
    void Init();

    uint8_t Play(const char* AFileName);
    void Stop();
    void PlayRandomFileFromDir(const char* DirName);
    void FadeOut();
    bool IsPlayingNow = false;

    void Rewind();
    // Inner use
    void ITask();
    void IHandleIrq();
};
