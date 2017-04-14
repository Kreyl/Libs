#include "sound.h"
#include <string.h>
#include "evt_mask.h"
#include "board.h"
#include "main.h"

//#if SOUND_ENABLED

Sound_t Sound;

// Mode register
//#define VS_MODE_REG_VALUE   0x0802  // Native SDI mode, Layer I + II enabled
#define VS_MODE_REG_VALUE   0x0803  // Native SDI mode, Layer I + II enabled, differential output

// After file end, send several zeroes
#define ZERO_SEQ_LEN        128
static const uint8_t SZero = 0;

static uint8_t ReadWriteByte(uint8_t AByte);

// ================================= IRQ =======================================
extern "C" {
// DMA irq
void SIrqDmaHandler(void *p, uint32_t flags) {
    chSysLockFromISR();
    chEvtSignalI(Sound.PThread, VS_EVT_DMA_DONE);
    chSysUnlockFromISR();
}
} // extern c

// =========================== Implementation ==================================
static THD_WORKING_AREA(waSoundThread, 1024);
__noreturn
static void SoundThread(void *arg) {
    chRegSetThreadName("Sound");
    Sound.ITask();
}

__noreturn
void Sound_t::ITask() {
    while(true) {
        eventmask_t EvtMsk = chEvtWaitAny(ALL_EVENTS);
        if(EvtMsk & VS_EVT_DMA_DONE) {
            ISpi.WaitBsyHi2Lo();            // Wait SPI transaction end
            XCS_Hi();                       // }
            XDCS_Hi();                      // } Stop SPI
            if(IDreq.IsHi()) ISendNextData();   // More data allowed, send it now
            else {
                chThdSleepMilliseconds(1); // Allow VS to end up with data processing
                chSysLock();
                IDreq.EnableIrq(IRQ_PRIO_MEDIUM); // Enable dreq irq
                chSysUnlock();
            }
        }

        if(EvtMsk & VS_EVT_DREQ_IRQ) ISendNextData();

        // Play new request
        if(EvtMsk & VS_EVT_COMPLETED) {
//        	Uart.Printf("\rComp");
            AddCmd(VS_REG_MODE, 0x0004);    // Soft reset
            if(IFilename != NULL) IPlayNew();
            else App.SignalEvt(EVT_PLAY_ENDS); // Raise event if nothing to play
        }
        // Stop request
        else if(EvtMsk & VS_EVT_STOP) {
//            Uart.Printf("\rStop");
            PrepareToStop();
        }
        // Data read request
        else if(EvtMsk & VS_EVT_READ_NEXT) {
//            Uart.Printf("\rreadNext");
            FRESULT rslt = FR_OK;
            bool EofAtStart = f_eof(&IFile);
            // Read next if not EOF
            if(!EofAtStart) {
                if     (Buf1.DataSz == 0) { /*Uart.Printf("1");*/ rslt = Buf1.ReadFromFile(&IFile); }
                else if(Buf2.DataSz == 0) { /*Uart.Printf("2");*/ rslt = Buf2.ReadFromFile(&IFile); }
            }
            // Check if was EOF or if error occured during reading. Do not do it if EOF occured during reading.
//            if(rslt != FR_OK) Uart.Printf("\rsndReadErr=%u", rslt);
            if((rslt != FR_OK) or (EofAtStart and IDmaIdle and (Buf1.DataSz == 0) and (Buf2.DataSz == 0))) PrepareToStop();
            else StartTransmissionIfNotBusy();
        }
    } // while true
}

void Sound_t::Init() {
    // ==== GPIO init ====
    PinSetupOut(VS_GPIO, VS_RST, omPushPull);
    PinSetupOut(VS_GPIO, VS_XCS, omPushPull);
    PinSetupOut(VS_GPIO, VS_XDCS, omPushPull);
    XCS_Hi();
    XDCS_Hi();
    PinSetupAlterFunc(VS_GPIO, VS_XCLK, omPushPull, pudNone, VS_AF);
    PinSetupAlterFunc(VS_GPIO, VS_SO,   omPushPull, pudNone, VS_AF);
    PinSetupAlterFunc(VS_GPIO, VS_SI,   omPushPull, pudNone, VS_AF);
#if VS_AMPF_EXISTS
    PinSetupOut(VS_AMPF_GPIO, VS_AMPF_PIN, omPushPull);
    AmpfOn();
#endif

    // ==== SPI init ====
    ISpi.Setup(boMSB, cpolIdleLow, cphaFirstEdge, sclkDiv8);
    ISpi.Enable();
    ISpi.EnableTxDma();

    // ==== DMA ====
    // Here only unchanged parameters of the DMA are configured.
    dmaStreamAllocate     (VS_DMA, IRQ_PRIO_MEDIUM, SIrqDmaHandler, NULL);
    dmaStreamSetPeripheral(VS_DMA, &VS_SPI->DR);
    dmaStreamSetMode      (VS_DMA, VS_DMA_MODE);

    // ==== Variables ====
    State = sndStopped;
    IDmaIdle = true;
    PBuf = &Buf1;
    IAttenuation = VS_INITIAL_ATTENUATION;
    chMBObjectInit(&CmdBox, CmdBuf, VS_CMD_BUF_SZ);

    // ==== Init VS ====
    Rst_Lo();
    chThdSleepMilliseconds(180);
    Rst_Hi();
    chThdSleepMilliseconds(45);
    Clk.EnableMCO1(mco1HSE, mcoDiv1);   // Only after reset, as pins are grounded when Rst is Lo
    chThdSleepMicroseconds(45);
    // ==== DREQ IRQ ====
    IDreq.Init(ttRising);
    // ==== Thread ====
    PThread = chThdCreateStatic(waSoundThread, sizeof(waSoundThread), NORMALPRIO, (tfunc_t)SoundThread, NULL);
    StartTransmissionIfNotBusy();   // Send init commands
}

void Sound_t::Shutdown() {
#if VS_AMPF_EXISTS
    AmpfOff();
#endif
    Clk.DisableMCO1();  // Switch clk off as XTALI & XTALO grounded in reset
    Rst_Lo();           // enter shutdown mode
}

void Sound_t::IPlayNew() {
    AddCmd(VS_REG_MODE, VS_MODE_REG_VALUE);
    AddCmd(VS_REG_CLOCKF, (0x8000 + (12000000/2000)));
    AddCmd(VS_REG_VOL, ((IAttenuation * 256) + IAttenuation));

    FRESULT rslt;
    // Open new file
//    Uart.Printf("Play %S at %u\r", IFilename, IStartPosition);
    rslt = f_open(&IFile, IFilename, FA_READ+FA_OPEN_EXISTING);
    if(rslt != FR_OK) {
        if (rslt == FR_NO_FILE) Uart.Printf("%S: not found\r", IFilename);
        else Uart.Printf("OpenFile error: %u\r", rslt);
        IFilename = NULL;
        Stop();
        return;
    }
    IFilename = NULL;
    // Check if zero file
    if(IFile.fsize == 0) {
        f_close(&IFile);
        Uart.Printf("Empty file\r");
        Stop();
        return;
    }
    // Fast forward to start position if not zero
    if(IStartPosition != 0) {
        if(IStartPosition < IFile.fsize) f_lseek(&IFile, IStartPosition);
    }

    // Initially, fill both buffers
    if(Buf1.ReadFromFile(&IFile) != retvOk) { Stop(); return; }
    // Fill second buffer if needed
    if(Buf1.DataSz == VS_DATA_BUF_SZ) Buf2.ReadFromFile(&IFile);

    PBuf = &Buf1;
    State = sndPlaying;
    StartTransmissionIfNotBusy();
}

// ================================ Inner use ==================================
void Sound_t::AddCmd(uint8_t AAddr, uint16_t AData) {
    VsCmd_t FCmd;
    FCmd.OpCode = VS_WRITE_OPCODE;
    FCmd.Address = AAddr;
    FCmd.Data = __REV16(AData);
    // Add cmd to queue
    chMBPost(&CmdBox, FCmd.Msg, TIME_INFINITE);
    StartTransmissionIfNotBusy();
}

void Sound_t::ISendNextData() {
//    Uart.Printf("sn\r");
    IDreq.DisableIrq();
    dmaStreamDisable(VS_DMA);
    IDmaIdle = false;
    // If command queue is not empty, send command
    msg_t msg = chMBFetch(&CmdBox, &ICmd.Msg, TIME_IMMEDIATE);
    if(msg == MSG_OK) {
//        Uart.PrintfI("\rvCmd: %A\r", &ICmd, 4, ' ');
        XCS_Lo();   // Start Cmd transmission
        chThdSleepMilliseconds(1);
        dmaStreamSetMemory0(VS_DMA, &ICmd);
        dmaStreamSetTransactionSize(VS_DMA, sizeof(VsCmd_t));
        dmaStreamSetMode(VS_DMA, VS_DMA_MODE | STM32_DMA_CR_MINC);  // Memory pointer increase
        dmaStreamEnable(VS_DMA);
    }
    // Send next chunk of data if any
    else if(State == sndPlaying) {
//        Uart.PrintfI("\rD");
        // Send data if buffer is not empty
        if(PBuf->DataSz != 0) {
            XDCS_Lo();  // Start data transmission
            uint32_t FLength = (PBuf->DataSz > 32)? 32 : PBuf->DataSz;
            dmaStreamSetMemory0(VS_DMA, PBuf->PData);
            dmaStreamSetTransactionSize(VS_DMA, FLength);
            dmaStreamSetMode(VS_DMA, VS_DMA_MODE | STM32_DMA_CR_MINC);  // Memory pointer increase
            dmaStreamEnable(VS_DMA);
            // Process pointers and lengths
            PBuf->DataSz -= FLength;
            PBuf->PData += FLength;
        }
        else IDmaIdle = true;   // Will come true if both buffers are empty

        // Check if buffer is now empty
        if(PBuf->DataSz == 0) {
            // Prepare to read next chunk
//            Uart.Printf("*");
            chEvtSignal(PThread, VS_EVT_READ_NEXT);
            // Switch to next buf
            PBuf = (PBuf == &Buf1)? &Buf2 : &Buf1;
        }
    }
    else if(State == sndWritingZeroes) {
//        Uart.Printf("\rZ");
        if(ZeroesCount == 0) { // Was writing zeroes, now all over
            State = sndStopped;
            IDmaIdle = true;
//            Uart.Printf("vEnd\r");
            chSysLock();
            chEvtSignalI(PThread, VS_EVT_COMPLETED);
            chSysUnlock();
        }
        else SendZeroes();
    }
    else {
//    	Uart.PrintfI("\rI");
        if(!IDreq.IsHi()) IDreq.EnableIrq(IRQ_PRIO_MEDIUM);
        else IDmaIdle = true;
    }
}

void Sound_t::PrepareToStop() {
    State = sndWritingZeroes;
    ZeroesCount = ZERO_SEQ_LEN;
    if(IFile.fs != 0) f_close(&IFile);
    StartTransmissionIfNotBusy();
}

void Sound_t::SendZeroes() {
//    Uart.Printf("sz\r");
    XDCS_Lo();  // Start data transmission
    uint32_t FLength = (ZeroesCount > 32)? 32 : ZeroesCount;
    dmaStreamSetMemory0(VS_DMA, &SZero);
    dmaStreamSetTransactionSize(VS_DMA, FLength);
    dmaStreamSetMode(VS_DMA, VS_DMA_MODE);  // Do not increase memory pointer
    dmaStreamEnable(VS_DMA);
    ZeroesCount -= FLength;
}

uint8_t ReadWriteByte(uint8_t AByte) {
    VS_SPI->DR = AByte;
    while(!(VS_SPI->SR & SPI_SR_RXNE));
//    while(!(VS_SPI->SR & SPI_SR_BSY));
    return (uint8_t)(VS_SPI->DR);
}

// ==== Commands ====
uint8_t Sound_t::CmdRead(uint8_t AAddr, uint16_t* AData) {
//    uint8_t IReply;
    uint16_t IData;
    // Wait until ready
    //if ((IReply = BusyWait()) != OK) return IReply; // Get out in case of timeout
    XCS_Lo();   // Start transmission
    ReadWriteByte(VS_READ_OPCODE);  // Send operation code
    ReadWriteByte(AAddr);           // Send addr
    *AData = ReadWriteByte(0);      // Read upper byte
    *AData <<= 8;
    IData = ReadWriteByte(0);       // Read lower byte
    *AData += IData;
    XCS_Hi();   // End transmission
    return retvOk;
}
uint8_t Sound_t::CmdWrite(uint8_t AAddr, uint16_t AData) {
//    uint8_t IReply;
    // Wait until ready
//    if ((IReply = BusyWait()) != OK) return IReply; // Get out in case of timeout
    XCS_Lo();                       // Start transmission
    ReadWriteByte(VS_WRITE_OPCODE); // Send operation code
    ReadWriteByte(AAddr);           // Send addr
    ReadWriteByte(AData >> 8);      // Send upper byte
    ReadWriteByte(0x00FF & AData);  // Send lower byte
    XCS_Hi();                       // End transmission
    return retvOk;
}

//#endif // #if SOUND_ENABLED
