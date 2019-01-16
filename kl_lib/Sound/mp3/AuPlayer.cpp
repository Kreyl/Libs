/*
 * AuPlayer.cpp
 *
 *  Created on: 4 θώνÿ 2017 γ.
 *      Author: Kreyl
 */

#include "AuPlayer.h"
#include "kl_lib.h"
#include "shell.h"
#include "MsgQ.h"
#include "kl_sd.h"
#include "CS42L52.h"
#include "mp3common.h"
#include "kl_crypto.h"
#include "Content.h"

//#define DBG_PINS

#ifdef DBG_PINS
#define DBG_GPIO    GPIOE
#define DBG_PIN1    7
#define DBG1_SET()  PinSetHi(DBG_GPIO, DBG_PIN1)
#define DBG1_CLR()  PinSetLo(DBG_GPIO, DBG_PIN1)
#define DBG_PIN2    8
#define DBG2_SET()  PinSetHi(DBG_GPIO, DBG_PIN2)
#define DBG2_CLR()  PinSetLo(DBG_GPIO, DBG_PIN2)
#else
#define DBG1_SET()
#define DBG1_CLR()
#define DBG2_SET()
#define DBG2_CLR()
#endif

AuPlayer_t Player;
extern CS42L52_t Codec;
static enum AuPlayerState_t {
    apstIdle, apstPlaying, apstPause, apstMustPause, apstMustStop
} AuPlayerState = apstIdle;

#if 1 // ============================ MP3 decoder ==============================
static HMP3Decoder hMP3Decoder = nullptr;
static MP3FrameInfo mp3FrameInfo;
static uint8_t DecodeBuf[DECODE_BUF_SZ], *InputDataPtr;
static int32_t BytesInDecodeBuf = 0; // Bytes left in buffer after prev frame decoded
#endif

enum AudioEvt_t {aevtPlay, aevtNewBufReqd, aevtNewSampleRate, aevtSoundEnd};
static EvtMsgQ_t<AudioEvt_t, 7> EvtQAudio;
static thread_reference_t ThdRef;
static CryptoFile_t CryptoFile;

// DMA Tx Completed IRQ
extern "C"
void DmaSAITxIrq(void *p, uint32_t flags) {
    chSysLockFromISR();
    Player.IHandleIrq();
    chSysUnlockFromISR();
}

void AuPlayer_t::IHandleIrq() {
    DBG1_SET();
    PCurBuf = (PCurBuf == &IBuf1)? &IBuf2 : &IBuf1;
    //PrintfI("%u\r", PCurBuf->Sz);
    if(PCurBuf->SampleCnt == 0) {  // End of file
        ContentMaster.OnSoundEndI();
        EvtQAudio.SendNowOrExitI(aevtSoundEnd);
        AuPlayerState = apstIdle;
        // Wake waiting thread if any
        chThdResumeI(&ThdRef, MSG_OK);   // NotNull check performed inside chThdResume
    } // if Sz == 0
    else {
        if(CurrSampleRate != NextSampleRate) {
            CurrSampleRate = NextSampleRate;
            EvtQAudio.SendNowOrExitI(aevtNewSampleRate);
        }
        // Pause if must
        if(AuPlayerState == apstMustPause) {
            AuPlayerState = apstPause;
            ContentMaster.OnSoundPauseI();
            // Wake waiting thread if any
            chThdResumeI(&ThdRef, MSG_OK);   // NotNull check performed inside chThdResume
        }
        // Stop if must
        else if(AuPlayerState == apstMustStop) {
            AuPlayerState = apstIdle;
            ContentMaster.OnSoundStopI();
            EvtQAudio.SendNowOrExitI(aevtSoundEnd);
            // Wake waiting thread if any
            chThdResumeI(&ThdRef, MSG_OK);   // NotNull check performed inside chThdResume
        }
        else {
            TransmitBuf(PCurBuf);
            EvtQAudio.SendNowOrExitI(aevtNewBufReqd);
        }
    }
    DBG1_CLR();
}

// Thread
static THD_WORKING_AREA(waAudioThread, 1024);   // Hangs with long file names when less than 1024 bytes here
__noreturn
static void AudioThread(void *arg) {
    chRegSetThreadName("Audio");
    Player.ITask();
}

__noreturn
void AuPlayer_t::ITask() {
    while(true) {
        AudioEvt_t Msg = EvtQAudio.Fetch(TIME_INFINITE);
        if(!SD.IsReady) {
            AuPlayerState = apstIdle;
            ContentMaster.OnSoundEnd();
            continue;
        }
        switch(Msg) {
            case aevtNewBufReqd: {
                SndBuf_t *PBufToFill = (PCurBuf == &IBuf1)? &IBuf2 : &IBuf1;
                ReadToBuf(PBufToFill, &NextSampleRate);
            } break;

            case aevtPlay: IPlay(); break;

            case aevtNewSampleRate:
                Codec.SetupSampleRate(CurrSampleRate);
                Printf("New sample rate: %u\r", CurrSampleRate);
                break;

            case aevtSoundEnd: break;
        } // switch
    } // while true
}

void AuPlayer_t::Init() {
#ifdef DBG_PINS
    PinSetupOut(DBG_GPIO, DBG_PIN1, omPushPull);
    PinSetupOut(DBG_GPIO, DBG_PIN2, omPushPull);
#endif
    // Send zeroes
    IBuf1.SampleCnt = FRAME_BUF_SZ / 2;
    PCurBuf = &IBuf1;
    TransmitBuf(PCurBuf);
    // Proceed with init
    EvtQAudio.Init();
    chThdCreateStatic(waAudioThread, sizeof(waAudioThread), NORMALPRIO, (tfunc_t)AudioThread, NULL);
}

void AuPlayer_t::IPlay() {
    Printf("Play %S\r", CryptoFile.Name);
    // Reinit decoder
    if(hMP3Decoder) MP3FreeDecoder(hMP3Decoder);
    hMP3Decoder = MP3InitDecoder();
    if (hMP3Decoder == nullptr) {
        Printf("MP3 decoder init failed\r");
        return;
    }

    if(CryptoFile.OpenFile("MISHKA The First                ") == retvOk) {
        BytesInDecodeBuf = 0;
        // Fill both buffers
        ReadToBuf(&IBuf1, &CurrSampleRate);
        ReadToBuf(&IBuf2, &NextSampleRate);
        PCurBuf = &IBuf1;
        Codec.SetupSampleRate(CurrSampleRate);
        if(mp3FrameInfo.nChans == 1) Codec.SetupMonoStereo(Mono);
        else Codec.SetupMonoStereo(Stereo);
        AuPlayerState = apstPlaying;
        TransmitBuf(PCurBuf);
    }
    else {
        Printf("Open %S failed\r", CryptoFile.Name);
        AuPlayerState = apstIdle;
        ContentMaster.OnSoundEnd();
        // Wake waiting thread if any
        chThdResume(&ThdRef, MSG_OK);   // NotNull check performed inside chThdResume
    }
}

void AuPlayer_t::ReadToBuf(SndBuf_t *PBuf, uint32_t *PNextSampleRate) {
//    Printf("ReadToBuf\r");
    DBG2_SET();
    while(true) {
        // Find beginning of frame
        while(true) {
            // Fill buffer if empty
            if(BytesInDecodeBuf == 0) {
                if(CryptoFile.GetBuf(DecodeBuf, DECODE_BUF_SZ, &BytesInDecodeBuf) != retvOk) return;
                InputDataPtr = DecodeBuf;
            }

            // Find beginning of the frame: look for Sync word
            int offset = MP3FindSyncWord(InputDataPtr, BytesInDecodeBuf);
            if(offset >= 0) {
                // Move pointer to beginning of the frame
                InputDataPtr += offset;
                BytesInDecodeBuf -= offset;
                int error = MP3GetNextFrameInfo(hMP3Decoder, &mp3FrameInfo, InputDataPtr); // Get frame info
                // Check if layer3 and stereo
                if(error == ERR_MP3_NONE and mp3FrameInfo.layer == 3) break;
                else { // Look for next frame
                    InputDataPtr += 2; // Skip 11 bits of Sync word
                    BytesInDecodeBuf -= 2;
                }
            }
            else BytesInDecodeBuf = 0; // sync not found, read next chunk of data
        } // while

        // Frame found, prepare to decode: shift buffer and append it
        if(BytesInDecodeBuf < DECODE_BUF_SZ) {
            memcpy(DecodeBuf, InputDataPtr, BytesInDecodeBuf); // Shift buffer
            // Append Framebuf from Cryptobuf
            int32_t BytesToAppend = DECODE_BUF_SZ - BytesInDecodeBuf;
            int32_t BytesAppended;
            if(CryptoFile.GetBuf(&DecodeBuf[BytesInDecodeBuf], BytesToAppend, &BytesAppended) != retvOk) {
                PBuf->SampleCnt = 0;
                return;
            }
            InputDataPtr = DecodeBuf;
            BytesInDecodeBuf += BytesAppended;
        } // if buf not full

        // Decode frame
        int error = MP3Decode(hMP3Decoder, &InputDataPtr, (int*)&BytesInDecodeBuf, (short*)PBuf->Buf, 0);
        if(error < 0) Printf("mp3Error: %d; BytesInBuf: %d\r", error, BytesInDecodeBuf);
        else { // decode ok
            MP3GetLastFrameInfo(hMP3Decoder, &mp3FrameInfo);
            *PNextSampleRate = mp3FrameInfo.samprate; // Get sampling rate
            PBuf->SampleCnt = mp3FrameInfo.outputSamps;
            return;
        }
    } // while true
    DBG2_CLR();
}

void AuPlayer_t::TransmitBuf(SndBuf_t *PBuf) {
    Codec.TransmitBuf(PBuf->Buf, PBuf->SampleCnt); // one sample per channel
}

#if 1 // ============================= High level ==============================
void AuPlayer_t::Play(char* AFName, char* AKey, char* AIV) {
    if(AFName == nullptr) return;
    CryptoFile.IV = AIV;
    CryptoFile.Key = AKey;
    CryptoFile.Name = AFName;
    EvtQAudio.SendNowOrExit(aevtPlay);
}

// Will be stopped inside IRQ. Retval required for BLE.
uint8_t AuPlayer_t::Stop() {
    if(AuPlayerState == apstPlaying) {
        chSysLock();
        AuPlayerState = apstMustStop;
        chThdSuspendS(&ThdRef); // Wait stop
        chSysUnlock();
        return 0;
    }
    else return 1;
}

// Will be paused inside IRQ
uint8_t AuPlayer_t::Pause() {
    if(AuPlayerState == apstPlaying) {        chSysLock();
        AuPlayerState = apstMustPause;
        chThdSuspendS(&ThdRef); // Wait pause to begin
        chSysUnlock();
        return 0;
    }
    else return 1;
}

uint8_t AuPlayer_t::Resume() {
    if(AuPlayerState == apstPause) {
        chSysLock();
        AuPlayerState = apstPlaying;
        TransmitBuf(PCurBuf);
        EvtQAudio.SendNowOrExitI(aevtNewBufReqd);
        ContentMaster.OnSoundResumeI();
        chSysUnlock();
        return 0;
    }
    else return 1;
}

void AuPlayer_t::WaitEnd() {
    chSysLock();
    chThdSuspendS(&ThdRef);
    chSysUnlock();
}
#endif
