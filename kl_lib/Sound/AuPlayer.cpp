/*
 * AuPlayer.cpp
 *
 *  Created on: 4 θώνÿ 2017 γ.
 *      Author: Kreyl
 */

#include "kl_fs_utils.h"
#include "AuPlayer.h"
#include "kl_lib.h"
#include "shell.h"
#include "CS42L52.h"
#include "MsgQ.h"

//#define DBG_PINS

#ifdef DBG_PINS
#define DBG_GPIO1   GPIOB
#define DBG_PIN1    13
#define DBG1_SET()  PinSetHi(DBG_GPIO1, DBG_PIN1)
#define DBG1_CLR()  PinSetLo(DBG_GPIO1, DBG_PIN1)
#else
#define DBG1_SET()
#define DBG1_CLR()
#endif

extern CS42L52_t Audio;
extern AuPlayer_t Player;

static char Filename[MAX_NAME_LEN];

struct WavFileInfo_t {
    uint32_t SampleRate;
    uint32_t BytesPerSecond;
    uint16_t FrameSz;
    uint32_t InitialDataChunkOffset, FinalDataChunkOffset;
    uint16_t BitsPerSample;
    uint16_t ChannelCnt;
    uint32_t FrameCnt;
    uint32_t Size;
    // Current state
    uint32_t NextDataChunkOffset, CurDataChunkFrameCnt;
    uint32_t ChunkSz;
};
static WavFileInfo_t Info;

enum PlayerEffect_t {peNone, peFadeOut};
static PlayerEffect_t Effect = peNone;

static int8_t StartVolume, CurrentVolume;
static systime_t ITime;

// DMA Tx Completed IRQ
static thread_reference_t ThdRef = nullptr;
extern "C"
void DmaSAITxIrq(void *p, uint32_t flags) {
    chSysLockFromISR();
    Player.IHandleIrq();
    chSysUnlockFromISR();
}

void AuPlayer_t::IHandleIrq() {
    if(BufSz != 0) {
        PCurBuf = (PCurBuf == Buf1)? Buf2 : Buf1;
        Audio.TransmitBuf(PCurBuf, BufSz);
    }
    chThdResumeI(&ThdRef, MSG_OK);
}

// Thread
static THD_WORKING_AREA(waAudioThread, 2048);
__noreturn
static void AudioThread(void *arg) {
    chRegSetThreadName("Audio");
    Player.ITask();
}

__noreturn
void AuPlayer_t::ITask() {
    while(true) {
        chSysLock();
        chThdSuspendS(&ThdRef); // Wait IRQ
        chSysUnlock();
        if(BufSz != 0) {
            uint32_t *PBufToFill = (PCurBuf == Buf1)? Buf2 : Buf1;
            // Fill buff
            if(Info.ChunkSz != 0) {
                BufSz = MIN(Info.ChunkSz, FRAME_BUF_SZ);
                if(TryRead(&IFile, PBufToFill, BufSz) != retvOk) {
                    f_close(&IFile);
                    BufSz = 0;
                }
                Info.ChunkSz -= BufSz;
            }
            else BufSz = 0;

            // Effects
            if(Effect == peFadeOut) {
                if(chVTTimeElapsedSinceX(ITime) >= MS2ST(63)) {
                    if(CurrentVolume > -63) {
                        CurrentVolume--;
                        Audio.SetVolume(CurrentVolume);
                    }
                    else BufSz = 0; // End of fade
                }
            }
        } // if(BufSz != 0)
        else {  // End of file
            f_close(&IFile);
            Audio.Stop();
            IsPlayingNow = false;
            if(Effect == peFadeOut) Audio.SetVolume(StartVolume);
            Effect = peNone;
            EvtMsg_t Msg(evtIdPlayEnd);
            EvtQMain.SendNowOrExit(Msg);
        }
    } // while true
}

void AuPlayer_t::Init() {
#ifdef DBG_PINS
    PinSetupOut(DBG_GPIO1, DBG_PIN1, omPushPull);
#endif    // Init radioIC
    chThdCreateStatic(waAudioThread, sizeof(waAudioThread), NORMALPRIO, (tfunc_t)AudioThread, NULL);
}

uint8_t AuPlayer_t::Play(const char* AFileName) {
    Printf("Play %S\r", AFileName);
    // Try to open file
    if(OpenWav(AFileName) != retvOk) return retvFail;
    // Setup audio
    Audio.SetupParams((Info.ChannelCnt == 1)? Mono : Stereo, Info.SampleRate);

    // Fill both buffers
    char ChunkID[4] = {0, 0, 0, 0};

    if(f_lseek(&IFile, Info.NextDataChunkOffset) != FR_OK) goto end;
    if(TryRead(&IFile, ChunkID, 4) != retvOk) goto end;
    if(TryRead<uint32_t>(&IFile, &Info.ChunkSz) != retvOk) goto end;
    if(memcmp(ChunkID, "data", 4) == 0) {  // "data" found
        // Read first buf
        PCurBuf = Buf1;
        BufSz = MIN(Info.ChunkSz, FRAME_BUF_SZ);
        if(TryRead(&IFile, Buf1, BufSz) != retvOk) goto end;
        // Start transmission
        Audio.TransmitBuf(PCurBuf, BufSz);
        // Read second buf
        Info.ChunkSz -= BufSz;
        if(Info.ChunkSz != 0) {
            BufSz = MIN(Info.ChunkSz, FRAME_BUF_SZ);
            if(TryRead(&IFile, Buf2, BufSz) != retvOk) goto end;
            Info.ChunkSz -= BufSz;
        }
    }
    IsPlayingNow = true;
    return retvOk;
    end:
    f_close(&IFile);
    IsPlayingNow = false;
    return retvFail;
}

void AuPlayer_t::Stop() {
    if(!IsPlayingNow) return;
    BufSz = 0;
    Audio.Stop();
    f_close(&IFile);
    EvtMsg_t Msg(evtIdPlayEnd);
    EvtQMain.SendNowOrExit(Msg);
}

void AuPlayer_t::FadeOut() {
    if(!IsPlayingNow) return;
    StartVolume = Audio.GetVolume();
    CurrentVolume = StartVolume;
    ITime = chVTGetSystemTimeX();
    Effect = peFadeOut;
}

uint8_t AuPlayer_t::OpenWav(const char* AFileName) {
    // Open file
    if(TryOpenFileRead(AFileName, &IFile) != retvOk) return retvFail;
    uint32_t NextChunkOffset, ChunkSz;
    uint16_t uw16;

    // Check if starts with RIFF
    char ChunkID[4] = {0, 0, 0, 0};
    if(TryRead(&IFile, ChunkID, 4) != retvOk or (memcmp(ChunkID, "RIFF", 4) != 0)) goto end;
    // Get file size
    if(TryRead<uint32_t>(&IFile, &Info.Size) != retvOk) goto end;
    // Check riff type
    if(TryRead(&IFile, ChunkID, 4) != retvOk or (memcmp(ChunkID, "WAVE", 4) != 0)) goto end;
    // Check format
    if(TryRead(&IFile, ChunkID, 4) != retvOk or (memcmp(ChunkID, "fmt ", 4) != 0)) goto end;
    // Get offset of next chunk
    if(TryRead<uint32_t>(&IFile, &ChunkSz) != retvOk) goto end;
    NextChunkOffset = IFile.fptr + ChunkSz;
    if((NextChunkOffset & 1) != 0) NextChunkOffset++;
    // Read format
    if(TryRead<uint16_t>(&IFile, &uw16) != retvOk) goto end;
//    Printf("Fmt: %X\r", uw16);
    if(uw16 != 1) goto end; // PCM only
    // Channel cnt
    if(TryRead<uint16_t>(&IFile, &Info.ChannelCnt) != retvOk) goto end;
    if(Info.ChannelCnt > 2) goto end;
    // Sample rate
    if(TryRead<uint32_t>(&IFile, &Info.SampleRate) != retvOk) goto end;
    // Bytes per second
    if(TryRead<uint32_t>(&IFile, &Info.BytesPerSecond) != retvOk) goto end;
    // Block alignment == frame sz
    if(TryRead<uint16_t>(&IFile, &Info.FrameSz) != retvOk) goto end;
    // Bits per sample
    if(TryRead<uint16_t>(&IFile, &Info.BitsPerSample) != retvOk) goto end;

//    Printf("Sz: %u\r", Info.Size);
//    Printf("NextCh: %u\r", NextChunkOffset);
//    Printf("ChnlCnt: %u\r", Info.ChannelCnt);
//    Printf("SmplRt: %u\r", Info.SampleRate);
//    Printf("BytesPerSecond: %u\r", Info.BytesPerSecond);
//    Printf("BlkAlgn: %u\r", Info.FrameSz);
//    Printf("BitsPerSample: %u\r", Info.BitsPerSample);

    // Find data chunk
    while(true) {
        // Move to data chunk
        if(f_lseek(&IFile, NextChunkOffset) != FR_OK) goto end;
        // Read chunk ID & sz
        if(TryRead(&IFile, ChunkID, 4) != retvOk) goto end;
        if(TryRead<uint32_t>(&IFile, &ChunkSz) != retvOk) goto end;

        if(memcmp(ChunkID, "data", 4) == 0) {  // "data" found
            // Calc offsets
            Info.InitialDataChunkOffset = NextChunkOffset;
            if((Info.InitialDataChunkOffset & 1) != 0) Info.InitialDataChunkOffset++;
            Info.FinalDataChunkOffset = IFile.fptr + ChunkSz;
            if((Info.FinalDataChunkOffset & 1) != 0) Info.FinalDataChunkOffset++;
//            Printf("DataStart: %u; DataEnd: %u\r", Info.InitialDataChunkOffset, Info.FinalDataChunkOffset);
            break;  // Data found
        }

        if(memcmp(ChunkID, "LIST", 4) == 0) {  // "LIST" found
            if(TryRead(&IFile, ChunkID, 4) != retvOk) goto end;
            if(memcmp(ChunkID, "wavl", 4) == 0) {
                // Calc offsets
                Info.InitialDataChunkOffset = IFile.fptr;   // Here is "data", definitely
                if((Info.InitialDataChunkOffset & 1) != 0) Info.InitialDataChunkOffset++;
                Info.FinalDataChunkOffset = IFile.fptr + ChunkSz;
                if((Info.FinalDataChunkOffset & 1) != 0) Info.FinalDataChunkOffset++;
//                Printf("DataStart: %u; DataEnd: %u\r", Info.InitialDataChunkOffset, Info.FinalDataChunkOffset);
                break;
            }
            else {  // Not wavl, so skip LIST alltogether
                ChunkSz -= 4;   // take in account 4 bytes of chunk type that just was read
            }
        }

        // Proceed with data search
        NextChunkOffset = IFile.fptr + ChunkSz;
        if(NextChunkOffset & 1) NextChunkOffset++;
    } // while true
    // Offsets ready
    Rewind();
    return retvOk;

    end:
    f_close(&IFile);
    return retvFail;
}

void AuPlayer_t::Rewind() {
    Info.NextDataChunkOffset = Info.InitialDataChunkOffset;
    Info.CurDataChunkFrameCnt = 0;
}

void AuPlayer_t::PlayRandomFileFromDir(const char* DirName) {
    uint32_t Cnt=0;
    uint8_t Rslt = CountFilesInDir(DirName, "wav", &Cnt);
    if(Rslt != retvOk or Cnt == 0) return;       // Get out if nothing to play
//    Printf("R=%u; Cnt=%u\r", Rslt, Cnt);
    // Select number of file
    uint32_t N = 0;
    if(Cnt > 1) {   // Get random number if count > 1
        do {
            N = Random(0, Cnt-1);   // [0; Cnt-1]
        } while(N == PreviousN);    // skip same as previous
    }
//    Printf("; Random=%u", N);
    PreviousN = N;
    // Iterate files in dir until success
    uint32_t Counter = 0;
    Rslt = f_opendir(&Dir, DirName);
    if(Rslt != FR_OK) return;
    while(true) {
        Rslt = f_readdir(&Dir, &FileInfo);
        if(Rslt != FR_OK) return;
        if((FileInfo.fname[0] == 0) and (FileInfo.lfname[0] == 0)) return;  // somehow no files left
        else { // Filename ok, check if not dir
            if(!(FileInfo.fattrib & AM_DIR)) {
                // Check if wav or mp3
                char *FName = (FileInfo.lfname[0] == 0)? FileInfo.fname : FileInfo.lfname;
//                Uart.Printf("\r%S  Cnt=%u", FName, Counter);
                uint32_t Len = strlen(FName);
                if(Len > 4) {
                    if(strcasecmp(&FName[Len-3], "wav") == 0) {
                        if(N == Counter) {
                            // Build full filename with path
                            // Check if root dir. Empty string allowed, too
                            int Len = strlen(DirName);
                            if((Len > 1) or (Len == 1 and *DirName != '/' and *DirName != '\\')) {
                                strcpy(Filename, DirName);
                                Filename[Len] = '/';
                            }
                            strcpy(&Filename[Len+1], FName);
                            Play(Filename);
                            return;
                        }
                        else Counter++;
                    }
                } // if Len>4
            } // if not dir
        } // Filename o
    } // while true
}

