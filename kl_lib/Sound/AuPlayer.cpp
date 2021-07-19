#include "AuPlayer.h"
#include "kl_lib.h"
#include "shell.h"
#include "MsgQ.h"
#include "kl_sd.h"

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

AuPlayer_t AuPlayer;

union EvtMsgAudio_t {
    uint32_t DWord[2];
    struct {
        char* Filename;
        PlayMode_t Mode : 8;
        uint8_t ID;
    } __attribute__((__packed__));
    EvtMsgAudio_t& operator = (const EvtMsg_t &Right) {
        DWord[0] = Right.DWord[0];
        DWord[1] = Right.DWord[1];
        return *this;
    }
    EvtMsgAudio_t() : Filename(nullptr), ID(0) {}
    EvtMsgAudio_t(uint8_t AID) : ID(AID) {}
    EvtMsgAudio_t(uint8_t AID, char *AFilename, PlayMode_t AMode) : Filename(AFilename), Mode(AMode), ID(AID) {}
} __attribute__((__packed__));

static EvtMsgQ_t<EvtMsgAudio_t, MAIN_EVT_Q_LEN> EvtQAudio;
enum AudioEvt_t {aevtPlay, aevtNewBufReqd, aevtOnSoundSwitch};
static thread_reference_t ThdRef;

// DMA Tx Completed IRQ
void IDmaSAITxIrq() { AuPlayer.IHandleIrq(); }

void AuPlayer_t::IHandleIrq() {
    PCurBuf = (PCurBuf == &ICurSnd->Buf1)? &ICurSnd->Buf2 : &ICurSnd->Buf1;
//    PrintfI("%u\r", PCurBuf->Sz);
    if(PCurBuf->Sz == 0) {  // End of file, start next
        // Play next if needed
        ISwitchSnds();
        PCurBuf = &ICurSnd->Buf1;
        if(PCurBuf->Sz != 0) TransmitBuf(PCurBuf);
        EvtQAudio.SendNowOrExitI(EvtMsgAudio_t(aevtOnSoundSwitch));
    } // if Sz == 0
    else {
        TransmitBuf(PCurBuf);
        EvtMsgAudio_t Msg(aevtNewBufReqd);
        EvtQAudio.SendNowOrExitI(Msg);
    }
}

// Thread
static THD_WORKING_AREA(waAudioThread, 1024);   // Hangs with long file names when less than 1024 bytes here
__noreturn
static void AudioThread(void *arg) {
    chRegSetThreadName("Audio");
    AuPlayer.ITask();
}

__noreturn
void AuPlayer_t::ITask() {
    while(true) {
        EvtMsgAudio_t Msg = EvtQAudio.Fetch(TIME_INFINITE);
        if(!SD.IsReady) {
            EvtQMain.SendNowOrExit(EvtMsg_t(evtIdAudioPlayEnd));
            continue;
        }
        switch(Msg.ID) {
            case aevtNewBufReqd: {
                SndBuf_t *PBufToFill;
                PBufToFill = (PCurBuf == &ICurSnd->Buf1)? &ICurSnd->Buf2 : &ICurSnd->Buf1;
                ICurSnd->ReadToBuf(PBufToFill);
                if(PBufToFill->Sz == 0 and (INextSnd->Buf1.Sz == 0 or !INextSnd->IsSoundFx)) {
                    // Wait Irq
                    while(Codec.IsTransmitting()) chThdSleepMilliseconds(1);
//                    Codec.Stop();

                    EvtQMain.SendNowOrExit(EvtMsg_t(evtIdAudioPlayEnd));
                    // Wake waiting thread if any
                    chThdResume(&ThdRef, MSG_OK);   // NotNull check performed inside chThdResume
                }
            } break;

            case aevtPlay:
                IPlayNext(Msg.Filename, Msg.Mode);
                break;

            case aevtOnSoundSwitch:
                Codec.SetupSampleRate(ICurSnd->Track.samplingRate());
                break;
        } // switch
    } // while true
}

void AuPlayer_t::IPlayNext(const char* AFName, PlayMode_t AMode) {
    bool WasPlaying = ICurSnd->IsPlaying();
    // Fade current sound
    IPrepareToPlayNext(AFName, AMode);
    if(WasPlaying) ICurSnd->FadeOut();
}

void AuPlayer_t::Init() {
#ifdef DBG_PINS
    PinSetupOut(DBG_GPIO1, DBG_PIN1, omPushPull);
#endif    // Init radioIC
    EvtQAudio.Init();
    chThdCreateStatic(waAudioThread, sizeof(waAudioThread), NORMALPRIO, (tfunc_t)AudioThread, NULL);
}

// Prepare next sound
void AuPlayer_t::IPrepareToPlayNext(const char* AFName, PlayMode_t AMode) {
    if(INextSnd->TryOpenFile(AFName) == retvOk) {
        if(INextSnd->Start(AMode) == retvOk) {
            Printf("Play %S\r", AFName);
            INextSnd->IsSoundFx = true;
            // Fill both buffers
            INextSnd->ReadToBuf(&INextSnd->Buf1);
            INextSnd->ReadToBuf(&INextSnd->Buf2);
            // If not playing, play immediately
            if(!ICurSnd->IsPlaying()) {
                ISwitchSnds();
                PCurBuf = &ICurSnd->Buf1;
                TransmitBuf(PCurBuf);
                EvtQAudio.SendNowOrExit(EvtMsgAudio_t(aevtOnSoundSwitch));
            }
        }
        else {
            Printf("File %S is not supported\r", AFName);
            EvtQMain.SendNowOrExit(EvtMsg_t(evtIdAudioPlayEnd));
            // Wake waiting thread if any
            chThdResume(&ThdRef, MSG_OK);   // NotNull check performed inside chThdResume
        }
    }
    else {
        Printf("Open %S failed\r", AFName);
        EvtQMain.SendNowOrExit(EvtMsg_t(evtIdAudioPlayEnd));
        // Wake waiting thread if any
        chThdResume(&ThdRef, MSG_OK);   // NotNull check performed inside chThdResume
    }
}

void AuPlayer_t::Play(const char* AFName, PlayMode_t Mode) {
    Codec.SaiDmaCallbackI = IDmaSAITxIrq;
    if(AFName == nullptr) return;
    EvtMsgAudio_t Msg(aevtPlay, (char*)AFName, Mode);
    EvtQAudio.SendNowOrExit(Msg);
}

void AuPlayer_t::Stop() {    ICurSnd->FadeOut();
}

void AuPlayer_t::WaitEnd() {
    chSysLock();
    chThdSuspendS(&ThdRef);
    chSysUnlock();
}

void AuPlayer_t::ISwitchSnds() {
    if(ICurSnd == &ISnd1) {
        ICurSnd = &ISnd2;
        INextSnd = &ISnd1;
    }
    else {
        ICurSnd = &ISnd1;
        INextSnd = &ISnd2;
    }
    INextSnd->Buf1.Sz = 0;
    INextSnd->Buf2.Sz = 0;
    INextSnd->Track.stop();
}


void FSound_t::FadeOut() {
#ifdef HAS_COSINE_TABLE
    Track.stop(AudioTrack::Fade::CosineOut, START_STOP_FADE_DUR);
#else
    Track.stop(AudioTrack::Fade::LinearOut, START_STOP_FADE_DUR);
#endif
}



#if 1 // ========================== Callbacks ==================================
size_t TellCallback(void *file_context) {
    FIL *pFile = (FIL*)file_context;
    return pFile->fptr;
}

bool SeekCallback(void *file_context, size_t offset) {
    FIL *pFile = (FIL*)file_context;
    FRESULT rslt = f_lseek(pFile, offset);
    if(rslt == FR_OK) return true;
    else {
        Printf("SeekErr %u\r", rslt);
        return false;
    }
}

size_t ReadCallback(void *file_context, uint8_t *buffer, size_t length) {
    FIL *pFile = (FIL*)file_context;
    uint32_t ReadSz=0;
    FRESULT rslt = f_read(pFile, buffer, length, &ReadSz);
    if(rslt == FR_OK) {
        return ReadSz;
    }
    else {
//        Printf("ReadErr %u\r", rslt);
        return 0;
    }
}
#endif
