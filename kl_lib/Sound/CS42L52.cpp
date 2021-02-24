/*
 * CS42L52.cpp
 *
 *  Created on: 15 ����� 2017 �.
 *      Author: Kreyl
 */

#include "CS42L52.h"
#include "shell.h"
#include "kl_i2c.h"

static const PinOutput_t PinRst(AU_RESET);
static const stm32_dma_stream_t *PDmaTx;
//static const stm32_dma_stream_t *PDmaRx;

__attribute__((weak))
void AuOnNewSampleI(SampleStereo_t &Sample) { }

#if 1 // ============================= CS Defines ==============================
#define CS_R_PWR_CTRL1          0x02
#define CS_PWR_DOWN_ADCA        (1<<1)

#define CS_R_PWR_CTRL2          0x03
#define CS_PWR_DOWN_MIC_A       (1<<1)
#define CS_PWR_DOWN_MIC_B       (1<<2)
#define CS_PWR_DOWN_BIAS        (1<<0)
#define CS_PWR_ON_MIC_A         (0<<1)
#define CS_PWR_ON_MIC_B         (0<<2)
#define CS_PWR_ON_BIAS          (0<<0)

#define CS_R_PWR_CTRL3          0x04

#define CS_R_INPUT_SELECT_A     0x08
#define CS_R_INPUT_SELECT_B     0x09
#define CS_ADCIN_AIN1           (0b000 << 5)
#define CS_ADCIN_AIN2           (0b001 << 5)
#define CS_ADCIN_AIN3           (0b010 << 5)
#define CS_ADCIN_AIN4           (0b011 << 5)
#define CS_ADCIN_PGA            (0b100 << 5)

#define CS_PGASEL_NONE          0b00000
#define CS_PGASEL_AIN1          0b00001
#define CS_PGASEL_AIN2          0b00010
#define CS_PGASEL_AIN3          0b00100
#define CS_PGASEL_AIN4          0b01000
#define CS_PGASEL_MIC           0b10000
// Combinations are possible: (CS_PGASEL_MIC+CS_PGASEL_AIN1)

// MicAmp control
#define CS_R_MICAMP_CTRL_A      0x10
#define CS_R_MICAMP_CTRL_B      0x11
#define CS_SEL_MIC1             (0 << 6)
#define CS_SEL_MIC2             (1 << 6)
#define CS_MIC_SINGLE_ENDED     (0 << 5)
#define CS_MIC_DIFFERENTIAL     (1 << 5)

#define CS_R_MISC_ADC_CTRL      0x0C
#define CS_DIGMUX_ADC           (0<<6)
#define CS_DIGMUX_DSP           (1<<6)
#define CS_DIGSUM_ADCA_ADCB     (0b00 << 4)
#define CS_DIGSUM_ADCB_ADCA     (0b11 << 4)
#define CS_ADCB_MUTE            (1<<1)
#define CS_ADCA_MUTE            (1<<0)
#define CS_ADCB_NOMUTE          (0<<1)
#define CS_ADCA_NOMUTE          (0<<0)

// Noise gate (both threshold & Boost)
#define CS_NG_THR_34dB          (0b1000)
#define CS_NG_THR_37dB          (0b1001)
#define CS_NG_THR_40dB          (0b1010)
#define CS_NG_THR_43dB          (0b1011)
#define CS_NG_THR_46dB          (0b1100)
#define CS_NG_THR_52dB          (0b1101)
#define CS_NG_THR_58dB          (0b1110)
#define CS_NG_THR_64dB          (0b0000)
#define CS_NG_THR_67dB          (0b0001)
#define CS_NG_THR_70dB          (0b0010)
#define CS_NG_THR_73dB          (0b0011)
#define CS_NG_THR_76dB          (0b0100)
#define CS_NG_THR_82dB          (0b0101)

#define CS_NG_DELAY_50ms        0b00
#define CS_NG_DELAY_100ms       0b01
#define CS_NG_DELAY_150ms       0b10
#define CS_NG_DELAY_200ms       0b11

#define AU_ALC_OVR_CTRL 0b00000000  // PGAA and ALCA Transition Ctl: no override of analog ramp and zero cross, gain=0
#endif

#if 1 // =========================== SAI defins ================================
#define SAI_IRQ_NUMBER          74
#define SAI_IRQ_HANDLER         Vector168

#define SAI_FIFO_THR_EMPTY      0
#define SAI_FIFO_THR_1_4        1
#define SAI_FIFO_THR_1_2        2
#define SAI_FIFO_THR_3_4        3
#define SAI_FIFO_THR_FULL       4
#define SAI_FIFO_THR            SAI_FIFO_THR_1_2

#define SAI_CR1_DATASZ_8BIT     ((uint32_t)(0b010 << 5))
#define SAI_CR1_DATASZ_10BIT    ((uint32_t)(0b011 << 5))
#define SAI_CR1_DATASZ_16BIT    ((uint32_t)(0b100 << 5))
#define SAI_CR1_DATASZ_20BIT    ((uint32_t)(0b101 << 5))
#define SAI_CR1_DATASZ_24BIT    ((uint32_t)(0b110 << 5))
#define SAI_CR1_DATASZ_32BIT    ((uint32_t)(0b111 << 5))

#define SAI_SYNC_ASYNC          ((uint32_t)(0b00 << 10))
#define SAI_SYNC_INTERNAL       ((uint32_t)(0b01 << 10))

#define SAI_RISING_EDGE         ((uint32_t)(0 << 9))
#define SAI_FALLING_EDGE        ((uint32_t)(1 << 9))

// Slots related
#define SAI_SLOT_CNT            2
#define SAI_SlotActive_0        (1 << 16)
#define SAI_SlotActive_1        (1 << 17)
#define SAI_SLOTSZ_EQ_DATASZ    (0b00 << 6)
#define SAI_SLOTSZ_16bit        (0b01 << 6)
#define SAI_SLOTSZ_32bit        (0b10 << 6)

#define SAI_MASTER_TX           ((uint32_t)0x00000000)
#define SAI_MASTER_RX           (SAI_xCR1_MODE_0)
#define SAI_SLAVE_TX            (SAI_xCR1_MODE_1)
#define SAI_SLAVE_RX            (SAI_xCR1_MODE_1 | SAI_xCR1_MODE_0)

#define SAI_DMATX_MONO_MODE  \
                        STM32_DMA_CR_CHSEL(SAI_DMA_CHNL) |   \
                        DMA_PRIORITY_MEDIUM | \
                        STM32_DMA_CR_MSIZE_HWORD | \
                        STM32_DMA_CR_PSIZE_HWORD | \
                        STM32_DMA_CR_MINC |     /* Memory pointer increase */ \
                        STM32_DMA_CR_DIR_M2P |  /* Direction is memory to peripheral */ \
                        STM32_DMA_CR_TCIE       /* Enable Transmission Complete IRQ */

#define SAI_DMATX_STEREO_MODE  \
                        STM32_DMA_CR_CHSEL(SAI_DMA_CHNL) |   \
                        DMA_PRIORITY_MEDIUM | \
                        STM32_DMA_CR_MSIZE_WORD | \
                        STM32_DMA_CR_PSIZE_WORD | \
                        STM32_DMA_CR_MINC |     /* Memory pointer increase */ \
                        STM32_DMA_CR_DIR_M2P |  /* Direction is memory to peripheral */ \
                        STM32_DMA_CR_TCIE       /* Enable Transmission Complete IRQ */

#define SAI_DMARX_MODE  STM32_DMA_CR_CHSEL(Chnl) |   \
                        DMA_PRIORITY_LOW | \
                        STM32_DMA_CR_MSIZE_BYTE | \
                        STM32_DMA_CR_PSIZE_BYTE | \
                        STM32_DMA_CR_MINC |         /* Memory pointer increase */ \
                        STM32_DMA_CR_DIR_P2M |      /* Direction is peripheral to memory */ \
                        STM32_DMA_CR_CIRC           /* Circular buffer enable */
//                        STM32_DMA_CR_TCIE           /* Enable Transmission Complete IRQ */
#endif

// DMA Tx Completed IRQ
extern "C"
void DmaSAITxIrq(void *p, uint32_t flags) {
    chSysLockFromISR();
    if(Codec.SaiDmaCallbackI) Codec.SaiDmaCallbackI();
    chSysUnlockFromISR();
}

void CS42L52_t::Init() {
    PinRst.Init();
    // Remove reset
    PinRst.SetHi();
    chThdSleepMilliseconds(18);
    AU_i2c.CheckAddress(0x4A); // Otherwise it does not work.
//    AU_i2c.ScanBus();
    // Check if connected
    uint8_t b;
    uint8_t r = ReadReg(0x01, &b);
    if(r != retvOk) { Printf("CS42L52: read fail (%u)\r", r); return; }
//    if(b != 0xE3) { Printf("CS42L52: wrong ID %X\r", b); return; }
#if 1 // === Setup registers ===
    // PwrCtrl 1: Power on codec only
    WriteReg(CS_R_PWR_CTRL1, 0b11111110);
    IsOn = true;
    // PwrCtrl 2: Mic power down
    WriteReg(CS_R_PWR_CTRL2, CS_PWR_DOWN_MIC_A | CS_PWR_DOWN_MIC_B | CS_PWR_DOWN_BIAS);
    // PwrCtrl 3: Everything is off
    WriteReg(CS_R_PWR_CTRL3, 0xFF);
    // Clocking Control: no Auto, 32kHz, not27MHz, ratio 128/64, no MCLK divide
    WriteReg(0x05, (0 << 7) | (0b10 << 5) | (1 << 4) | (0 << 3) | (0b01 << 1));
    // Interface Control 1: Master, SCLK not inverted, ADC output=LeftJ, DSP mode en, DAC input=LeftJ, WordLen=16
    WriteReg(0x06, (1<<7) | (0<<5) | (1<<4) | (0b00<<2) | 0b11);
    // Interface Control 2: DigLoop disabled, no 3-state, spk/hph not inverted, MicBias=0.8VAA = 0.8*2.5 = 2V
    WriteReg(0x07, 0b00000011);
    // Input A&B Select: ADC connected to PGA, PGA connected to Mic
    WriteReg(CS_R_INPUT_SELECT_A, CS_ADCIN_PGA | CS_PGASEL_MIC);
    WriteReg(CS_R_INPUT_SELECT_B, CS_ADCIN_PGA | CS_PGASEL_NONE);
    // Analog and HPF Control: for both A & B: HPF en, Continuous DC Subtraction, volume change with Soft-Ramp enabled and Zero-Cross
    WriteReg(0x0A, 0b10101111);
    // ADC HPF Corner Frequency: normal setting (default)
    //WriteReg(0x0B, 0b0000);
    // Misc. ADC Control (0x0C): B=A dis, sig src ADC, Left=ADCA & Right=ADCB, not inverted, not muted
    WriteReg(CS_R_MISC_ADC_CTRL, CS_DIGMUX_ADC | CS_DIGSUM_ADCA_ADCB | CS_ADCA_NOMUTE | CS_ADCB_NOMUTE);
    // Playback Control 1: HPGain=0.6, PLYBCKB=A dis, PCM not inverted, no mute
    WriteReg(0x0D, 0b01100000);
    // Miscellaneous Controls: Passthrough Analog dis, Passthrough Mute dis, no freeze, De-emphasis dis, Digital soft ramp en, zero-cross dis
    WriteReg(0x0E, 0b00000010);
    // Playback Control 2: HPA & HPB mute dis, SpkA & SpkB mute dis, SPKB=A dis, Spk swap dis, Spk Mono dis, Spk Mute 50/50 dis
//    WriteReg(0x0F, 0b00000000);
    WriteReg(0x0F, 0b00000010); // Spk mono en

    SetMicGain(16); // Setup mic and set gain
    SetPGAGain(0);

    SetAdcVolume(0);        // Volume of ADC
    MuteAdcMixer();         // Disable input sound to mixed with output on-chip
    SetPcmMixerVolume(0);   // Volume of digital input
    SetMasterVolume(0);
    SetHeadphoneVolume(0);
    SetSpeakerVolume(0);
    WriteReg(0x26, 0);  // No ch swap: no ADC, neither PCM

//    SetupNoiseGate(Enable, CS_NG_THR_46dB, CS_NG_DELAY_50ms);
#if AU_BATMON_ENABLE
    WriteReg(0x2F, 0b01001000); // Bat compensation dis, VP monitor en
#endif
#endif // Setup regs
#if 1 // ======= Setup SAI =======
    // === Clock ===
    Clk.EnableMCO(mcoHSE, mcoDiv1); // Master clock output
    AU_SAI_RccEn();

    // === GPIOs ===
    PinSetupAlterFunc(AU_LRCK); // Left/Right (Frame sync) clock output
    PinSetupAlterFunc(AU_SCLK); // Bit clock output
    PinSetupAlterFunc(AU_SDIN); // SAI_A is Slave Transmitter

    DisableSAI();   // All settings must be changed when both blocks are disabled
    // Sync setup: SaiA async, SaiB sync
    AU_SAI->GCR = 0;    // No external sync input/output

    // === Setup SAI_A as async Slave Transmitter ===
    // Stereo mode, Async, MSB first, Rising edge, Data Sz = 16bit, Free protocol, Slave Tx
    AU_SAI_A->CR1 = SAI_SYNC_ASYNC | SAI_RISING_EDGE | SAI_CR1_DATASZ_16BIT | SAI_SLAVE_TX;
    // No offset, FS Active Low, FS Active Lvl Len = 1, Frame Len = 32
    AU_SAI_A->FRCR = ((1 - 1) << 8) | (62 - 1);
    // 0 & 1 slots en, N slots = 2, slot size = 16bit, no offset
    AU_SAI_A->SLOTR = SAI_SlotActive_0 | SAI_SlotActive_1 | ((SAI_SLOT_CNT - 1) << 8) | SAI_SLOTSZ_16bit;
    AU_SAI_A->IMR = 0;  // No irq on TX

#if MIC_EN    // === Setup SAI_B as Slave Receiver ===
    PinSetupAlterFunc(AU_SDOUT); // SAI_B is Slave Receiver
    // Stereo mode, sync with sub-block, MSB first, Rising edge, Data Sz = 16bit, Free protocol, Slave Rx
    AU_SAI_B->CR1 = SAI_SYNC_INTERNAL | SAI_RISING_EDGE | SAI_CR1_DATASZ_16BIT | SAI_SLAVE_RX;
    AU_SAI_B->FRCR = AU_SAI_A->FRCR;
    AU_SAI_B->SLOTR = AU_SAI_A->SLOTR;
    AU_SAI_B->IMR = 0;  // No irq on RX
#endif
#endif

#if 1 // ==== DMA ====
    AU_SAI_A->CR1 |= SAI_xCR1_DMAEN;
    PDmaTx = dmaStreamAlloc(SAI_DMA_A, IRQ_PRIO_MEDIUM, DmaSAITxIrq, nullptr);
    dmaStreamSetPeripheral(PDmaTx, &AU_SAI_A->DR);
#endif
}

void CS42L52_t::Deinit() {
    if(PDmaTx) {
        dmaStreamDisable(PDmaTx);
        dmaStreamFree(PDmaTx);
        PDmaTx = nullptr;
    }
    AU_SAI_A->CR2 = SAI_xCR2_FFLUSH;
    Clk.DisableMCO();
    PinRst.SetLo();
    AU_SAI_RccDis();
    IsOn = false;
}

void CS42L52_t::Standby() {
    WriteReg(CS_R_PWR_CTRL1, 0xFF);
    IsOn = false;
}

void CS42L52_t::Resume() {
    // PwrCtrl 1: Power on codec only
    WriteReg(CS_R_PWR_CTRL1, 0b11111110);
    IsOn = true;
}

#if 1 // ==================== Low-level gains and volumes ======================
// Set mic gain: +16...+32 dB
void CS42L52_t::SetMicGain(uint8_t Gain_dB) {
    if(Gain_dB < 16) Gain_dB = 16;
    else if(Gain_dB > 32) Gain_dB = 32;
    Gain_dB -= 16;
    Gain_dB *=2;   // 0.5dB step
    uint8_t b = CS_MIC_DIFFERENTIAL | Gain_dB;
    WriteTwoTheSame(CS_R_MICAMP_CTRL_A, b);
}

// Set PGA gain: -6...+12 dB
uint8_t CS42L52_t::SetPGAGain(int8_t Gain_dB) {
    if(Gain_dB < -6 or Gain_dB > 12) return retvBadValue;
    Gain_dB *=2;   // 0.5dB step
    uint8_t b = AU_ALC_OVR_CTRL | (Gain_dB & 0b00111111);
    return WriteTwoTheSame(0x12, b);
}

uint8_t CS42L52_t::SetAdcVolume(int8_t Volume_dB) {
    if(Volume_dB < -96 or Volume_dB > 24) return retvBadValue;
    return WriteTwoTheSame(0x16, Volume_dB);
}

uint8_t CS42L52_t::SetAdcMixerVolume(int8_t Volume_dB) {
    if(Volume_dB < -51 or Volume_dB > 12) return retvBadValue;
    Volume_dB *= 2; // 0.5dB step
    return WriteTwoTheSame(0x18, Volume_dB);
}
uint8_t CS42L52_t::SetPcmMixerVolume(int8_t Volume_dB) {
    if(Volume_dB < -51 or Volume_dB > 12) return retvBadValue;
    Volume_dB *= 2; // 0.5dB step
    return WriteTwoTheSame(0x1A, Volume_dB);
}
#endif

#if 1 // ============================= Tx/Rx ===================================
void CS42L52_t::SetupMonoStereo(MonoStereo_t MonoStereo) {
    dmaStreamDisable(PDmaTx);
    DisableSAI();   // All settings must be changed when both blocks are disabled
    // Wait until really disabled
    while(AU_SAI_A->CR1 & SAI_xCR1_SAIEN);
    // Setup mono/stereo
    if(MonoStereo == Stereo) AU_SAI_A->CR1 &= ~SAI_xCR1_MONO;
    else AU_SAI_A->CR1 |= SAI_xCR1_MONO;
    AU_SAI_A->CR2 = SAI_xCR2_FFLUSH | SAI_FIFO_THR; // Flush FIFO
}

void CS42L52_t::SetupSampleRate(uint32_t SampleRate) {  // Setup sample rate. No Auto, 32kHz, not27MHz
//    Printf("CS fs: %u\r", SampleRate);
    uint8_t                      v = (0b10 << 5) | (1 << 4) | (0 << 3) | (0b01 << 1);    // 16 kHz
    if     (SampleRate == 22050) v = (0b10 << 5) | (0 << 4) | (0 << 3) | (0b11 << 1);
    else if(SampleRate == 44100) v = (0b01 << 5) | (0 << 4) | (0 << 3) | (0b11 << 1);
    else if(SampleRate == 48000) v = (0b01 << 5) | (0 << 4) | (0 << 3) | (0b01 << 1);
    else if(SampleRate == 96000) v = (0b00 << 5) | (0 << 4) | (0 << 3) | (0b01 << 1);
    WriteReg(0x05, v);
//    Printf("v: %X\r", v);
}

void CS42L52_t::TransmitBuf(volatile void *Buf, uint32_t Sz16) {
    dmaStreamDisable(PDmaTx);
    dmaStreamSetMemory0(PDmaTx, Buf);
    dmaStreamSetMode(PDmaTx, SAI_DMATX_MONO_MODE);
    dmaStreamSetTransactionSize(PDmaTx, Sz16);
    dmaStreamEnable(PDmaTx);
    EnableSAI(); // Start tx
}

bool CS42L52_t::IsTransmitting() {
    return (dmaStreamGetTransactionSize(PDmaTx) != 0);
}

void CS42L52_t::Stop() {
    dmaStreamDisable(PDmaTx);
    AU_SAI_A->CR2 = SAI_xCR2_FFLUSH;
}

void CS42L52_t::StartStream() {
    DisableSAI();   // All settings must be changed when both blocks are disabled
    dmaStreamDisable(PDmaTx);
#if MIC_EN
    dmaStreamDisable(SAI_DMA_B);
#endif
    AU_SAI_A->CR1 &= ~(SAI_xCR1_MONO | SAI_xCR1_DMAEN); // Always stereo, no DMA
    AU_SAI_A->CR2 = SAI_xCR2_FFLUSH | SAI_FIFO_THR; // Flush FIFO
    AU_SAI_B->CR2 = SAI_xCR2_FFLUSH | SAI_FIFO_THR; // Flush FIFO
    // Setup IRQ
    AU_SAI_B->IMR = SAI_xIMR_FREQIE;
    nvicEnableVector(SAI_IRQ_NUMBER, IRQ_PRIO_MEDIUM);
    EnableSAI();
}

void CS42L52_t::PutSampleI(SampleStereo_t &Sample) {
    AU_SAI_A->DR = Sample.Right;    // }
    AU_SAI_A->DR = Sample.Left;     // } Somehow Left will be sent first if put last
}
#endif

#if 1 // ============================ Some other ===============================
void CS42L52_t::BeepSingle(AuBeepFreq_t Freq, AuBeepOnTime_t OnTime, int8_t Volume_dB) {
    uint8_t b[3];
    // Freq & On Time
    b[0] = ((uint8_t)Freq) << 4;
    b[0] |= (uint8_t)OnTime;
    // Volume
    if(Volume_dB < -56 or Volume_dB > 6) return;
    Volume_dB += 6;
    Volume_dB /= 2;
    b[1] = (uint8_t)Volume_dB;
    // Beep Configuration: single Beep enable, beep mix enable
    b[2] = 0b01000000;
    WriteMany(0x1C, b, 3);
}

void CS42L52_t::GetStatus() {
    uint8_t b;
    ReadReg(0x2E, &b);
    Printf("Au status: %X\r", b);
}

void CS42L52_t::SetupNoiseGate(EnableDisable_t En, uint8_t Threshold, uint8_t Delay) {
    if(En == Enable) WriteReg(0x2D, (1 << 6) | (Threshold << 2) | Delay);
    else WriteReg(0x2D, 0);
}

#if AU_BATMON_ENABLE
uint32_t CS42L52_t::GetBatteryVmv() {
//    if(!IsOn) {
//        PinRst.SetHi();
//        Clk.EnableMCO(mcoHSE, mcoDiv1); // Master clock output
//        chThdSleepMilliseconds(18);
//        AU_i2c.CheckAddress(0x4A); // Otherwise it does not work.
//        WriteReg(CS_R_PWR_CTRL1, 0b11111110); // PwrCtrl 1: Power on codec only
//        WriteReg(0x2F, 0b01001000); // Bat compensation dis, VP monitor en
//        chThdSleepMilliseconds(999);
//    }

    uint8_t b;
    ReadReg(0x30, &b);
    uint32_t Rslt = ((uint32_t)b * 10UL * AU_VA_mv) / 633UL;

//    if(!IsOn) {
//        Clk.DisableMCO();
//        PinRst.SetLo();
//    }

    return Rslt;
}
#endif

uint8_t CS42L52_t::ReadReg(uint8_t RegAddr, uint8_t *PValue) {
    return AU_i2c.WriteRead(CS42_I2C_ADDR, &RegAddr, 1, PValue, 1);
}
uint8_t CS42L52_t::WriteReg(uint8_t RegAddr, uint8_t Value) {
    uint8_t b[2];
    b[0] = RegAddr;
    b[1] = Value;
    return AU_i2c.Write(CS42_I2C_ADDR, b, 2);
}

uint8_t CS42L52_t::WriteTwoTheSame(uint8_t StartAddr, uint8_t Value) {
    uint8_t b[3];
    b[0] = StartAddr | 0x80;    // Set INCR bit to autoincrease address
    b[1] = Value;
    b[2] = Value;
    return AU_i2c.Write(CS42_I2C_ADDR, b, 3);
}

uint8_t CS42L52_t::WriteMany(uint8_t StartAddr, uint8_t *PValues, uint8_t Len) {
    return AU_i2c.Write(CS42_I2C_ADDR, PValues, Len);
}
#endif

#if 1 // ============================= Volumes =================================
// -102...12 dB
uint8_t CS42L52_t::SetMasterVolume(int8_t Volume_dB) {
    if(Volume_dB < -102 or Volume_dB > 12) return retvBadValue;
    Volume_dB *= 2; // 0.5dB step
    return WriteTwoTheSame(0x20, Volume_dB);
}

// -96...0 dB
uint8_t CS42L52_t::SetHeadphoneVolume(int8_t Volume_dB) {
    if(Volume_dB < -96 or Volume_dB > 0) return retvBadValue;
    IVolume = Volume_dB;
    Volume_dB *= 2; // 0.5dB step
    return WriteTwoTheSame(0x22, Volume_dB);
}

// -96...0 dB
uint8_t CS42L52_t::SetSpeakerVolume(int8_t Volume_dB) {
    if(Volume_dB < -96 or Volume_dB > 0) return retvBadValue;
    Volume_dB *= 2; // 0.5dB step
    return WriteTwoTheSame(0x24, Volume_dB);
}

void CS42L52_t::VolumeUp() {
    IVolume += 3;
    if(IVolume > 0) IVolume = 0;
    if(IsOn) SetHeadphoneVolume(IVolume);
}

void CS42L52_t::VolumeDown() {
    IVolume -= 3;
    if(IVolume < -45) IVolume = -45;
    if(IsOn) SetHeadphoneVolume(IVolume);
}

void CS42L52_t::SetVolume(int8_t AVolume) {
    if(IsOn) SetHeadphoneVolume(AVolume);
}

#endif

#if 1 // ========================= Enable/Disable ==============================
void CS42L52_t::EnableMicSystem() {
    WriteReg(CS_R_PWR_CTRL1, 0); // Power on charge pump, PGA and ADC
    WriteReg(CS_R_PWR_CTRL2, CS_PWR_ON_MIC_A | CS_PWR_DOWN_MIC_B | CS_PWR_ON_BIAS);
}
void CS42L52_t::DisableMicSystem() {
    WriteReg(CS_R_PWR_CTRL1, 0b11111110); // Power on codec only
    WriteReg(CS_R_PWR_CTRL2, CS_PWR_DOWN_MIC_A | CS_PWR_DOWN_MIC_B | CS_PWR_DOWN_BIAS);
}

void CS42L52_t::EnableHeadphones() {
    uint8_t Reg = 0;
    ReadReg(CS_R_PWR_CTRL3, &Reg);
    Reg &= 0x0F;        // Clear headphones ctrl bits
    Reg |= 0b10100000;  // Headphones always on
    WriteReg(CS_R_PWR_CTRL3, Reg);
}

void CS42L52_t::DisableHeadphones() {
    uint8_t Reg = 0;
    ReadReg(CS_R_PWR_CTRL3, &Reg);
    Reg |= 0b11110000;  // Headphones always off
    WriteReg(CS_R_PWR_CTRL3, Reg);
}

void CS42L52_t::EnableSpeakerMono() {
    uint8_t Reg = 0;
    ReadReg(CS_R_PWR_CTRL3, &Reg);
    Reg &= 0xF0;        // Clear speaker ctrl bits
    Reg |= 0b00001010;  // SpeakerA En , SpeakerB En (will not work otherwise)
    WriteReg(CS_R_PWR_CTRL3, Reg);
    WriteReg(0x0F, 0b00000010); // Spk mono en
}

void CS42L52_t::DisableSpeakers() {
    uint8_t Reg = 0;
    ReadReg(CS_R_PWR_CTRL3, &Reg);
    Reg |= 0b00001111;  // Speakers always off
    WriteReg(CS_R_PWR_CTRL3, Reg);
}
#endif

#if 1 // ============================== IRQ ====================================
extern "C"
OSAL_IRQ_HANDLER(SAI_IRQ_HANDLER) {
    OSAL_IRQ_PROLOGUE();
    if(AU_SAI_B->SR & SAI_xSR_FREQ) {
        SampleStereo_t Sample;
        Sample.Left = AU_SAI_B->DR;
        Sample.Right = AU_SAI_B->DR;
        AuOnNewSampleI(Sample);
    }
    OSAL_IRQ_EPILOGUE();
}

#endif
