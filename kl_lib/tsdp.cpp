/*
 * tsdp.cpp
 *
 *  Created on: Dec 10, 2019
 *      Author: laurelindo
 *  More TSDP info: https://wiki.yandex-team.ru/users/laurelindo/Tempo-TSDP18xx/
 */

#include "kl_lib.h"
#include "tsdp.h"
#include "shell.h"
#include "usb_audio.h"
#include "led.h"

#if 1 // =========================== SAI defins ================================
#define SAI_IRQ_NUMBER          74
#define SAI_IRQ_HANDLER         Vector168

#define SAI_FIFO_THR_EMPTY      0
#define SAI_FIFO_THR_1_4        1
#define SAI_FIFO_THR_1_2        2
#define SAI_FIFO_THR_3_4        3
#define SAI_FIFO_THR_FULL       4
#define SAI_FIFO_THR            SAI_FIFO_THR_EMPTY

#define SAI_CR1_DATASZ_8BIT     (0b010UL << 5)
#define SAI_CR1_DATASZ_10BIT    (0b011UL << 5)
#define SAI_CR1_DATASZ_16BIT    (0b100UL << 5)
#define SAI_CR1_DATASZ_20BIT    (0b101UL << 5)
#define SAI_CR1_DATASZ_24BIT    (0b110UL << 5)
#define SAI_CR1_DATASZ_32BIT    (0b111UL << 5)

#define SAI_SYNC_ASYNC          ((uint32_t)(0b00 << 10))
#define SAI_SYNC_INTERNAL       ((uint32_t)(0b01 << 10))

#define SAI_RISING_EDGE         ((uint32_t)(0 << 9))
#define SAI_FALLING_EDGE        ((uint32_t)(1 << 9))

// Slots related
#define SAI_SLOT_CNT            8UL
#define SAI_SLOTSZ_EQ_DATASZ    (0b00UL << 6)
#define SAI_SLOTSZ_16bit        (0b01UL << 6)
#define SAI_SLOTSZ_32bit        (0b10UL << 6)

#define SAI_MASTER_TX           ((uint32_t)0x00000000)
#define SAI_MASTER_RX           (SAI_xCR1_MODE_0)
#define SAI_SLAVE_TX            (SAI_xCR1_MODE_1)
#define SAI_SLAVE_RX            (SAI_xCR1_MODE_1 | SAI_xCR1_MODE_0)

#define SAI_DMARX_MODE  STM32_DMA_CR_CHSEL(SAI_DMA_CHNL) | \
                        DMA_PRIORITY_MEDIUM |      \
                        STM32_DMA_CR_MSIZE_HWORD | \
                        STM32_DMA_CR_PSIZE_HWORD | \
                        STM32_DMA_CR_MINC |        \
                        STM32_DMA_CR_DIR_P2M |     \
                        STM32_DMA_CR_TCIE
#endif

#if 1 // Buffers and IRQ
#define SAMPLE_CNT  (8 * 4) // 8 samples 4 times => IRQ 4 times per ms
#define BUF_SZ      (SAMPLE_CNT * 2)
static int16_t Buf1[SAMPLE_CNT], Buf2[SAMPLE_CNT], *BufW = Buf1;

void DmaRxEndIRQ(void *p, uint32_t flags) { Tsdp.OnDmaIrq(); }

//extern LedBlinker_t Leds[LED_CNT];
//int32_t Threshold = 200;

//static int32_t Ave[8];
//static int32_t Max[8], Min[8];
//static int32_t Cnt = 0;

void TSDP_t::OnDmaIrq() {
    chSysLockFromISR();
//    PrintfI("I\r");
//    PinSetHi(GPIOB, 15);
    dmaStreamDisable(PDmaRx);
    BufW = (BufW == Buf1)? Buf2 : Buf1;
    dmaStreamSetMemory0(PDmaRx, BufW);
    dmaStreamSetTransactionSize(PDmaRx, SAMPLE_CNT);
    dmaStreamSetMode(PDmaRx, SAI_DMARX_MODE);
    dmaStreamEnable(PDmaRx);

    int16_t* BufR = (BufW == Buf1)? Buf2 : Buf1;
//    PrintfI("%d\r", BufR[0]);
//    for(int i=0; i<8; i++) {
//        if(BufR[i] > Threshold or BufR[i] < -Threshold) Leds[i].On();
//        else Leds[i].Off();
//        Ave[i] += BufR[i];
//        if(BufR[i] > Max[i]) Max[i] = BufR[i];
//        if(BufR[i] < Min[i]) Min[i] = BufR[i];
//    }

    UsbAu.OnNewSamplesI(BufR, SAMPLE_CNT);

//    if(Cnt++ >= 16000) {
//        for(int i=0; i<8; i++) {
//            Ave[i] /= Cnt;
//            PrintfI(" %d %d %d;", Ave[i], Min[i], Max[i]);
//            PrintfI(" %d %d;", Min[i], Max[i]);
//            Ave[i] = 0;
//            Max[i] = -2000000000;
//            Min[i] =  2000000000;
//        }
//        Cnt = 0;
//        PrintfI("\r\n");
//    }
//    PinSetLo(GPIOB, 15);
    chSysUnlockFromISR();
}
#endif

void TSDP_t::Init() {
#if 1 // ==== Mode setup ====
    PinSetupOut(TSDP_SCLK_POL, omPushPull);
    PinSetupOut(TSDP_2CH_TDM1, omPushPull);
    PinSetupOut(TSDP_2CH_TDM2, omPushPull);
    PinSetupOut(TSDP_WL_MSB,   omPushPull);
    PinSetupOut(TSDP_WL_LSB,   omPushPull);
    PinSetupOut(TSDP_OSMODE1,  omPushPull);
    PinSetupOut(TSDP_OSMODE2,  omPushPull);
    PinSetupOut(TSDP_OSMODE3,  omPushPull);
    // fs 16 to 24, Valid SCLK / LRCLK Ratios 128, 256, 384, 512
    PinSetHi(TSDP_OSMODE3); // 1
    PinSetHi(TSDP_OSMODE2); // 1
    PinSetLo(TSDP_OSMODE1); // 0
    // PCM Word Length = 16bits
    PinSetHi(TSDP_WL_MSB); // 1
    PinSetHi(TSDP_WL_LSB); // 1
    // TDM, up to 8 Channel Output Mode, Double-Edged Clocking on PDM Source
    PinSetHi(TSDP_2CH_TDM1); // 1
    PinSetHi(TSDP_2CH_TDM2); // 1
    // If SCLK_POL is LOW, data is transmitted on the rising edge of SCLK, otherwise, data is transmited on the falling edge of SCLK
    PinSetLo(TSDP_SCLK_POL); // 0
//    PinSetHi(TSDP_SCLK_POL); // 1
#endif

#if 1 // ==== SAI ====
    // === Clock ===
    RCC->APB2ENR |= RCC_APB2ENR_SAI1EN; // Enable clock
    RCC->CCIPR &= ~RCC_CCIPR_SAI1SEL;   // }
    RCC->CCIPR |= 0b01UL << 22;         // } SAI input frequency is PLLSAI2_P

    // === GPIOs ===
    PinSetupAlterFunc(TSDP_SAI_LRCK); // Left/Right (Frame sync) clock output
    PinSetupAlterFunc(TSDP_SAI_SCK);  // Bit clock output
    PinSetupAlterFunc(TSDP_SAI_SD);   // SAI_A is Master Receiver

    DisableSAI();   // All settings must be changed when both blocks are disabled
    // Sync setup: SaiA async
    TSDP_SAI->GCR = 0;    // No external sync input/output
    // === Setup SAI_A as async Master Receiver ===
    // Async, Rising edge, MSB first, Data Sz = 16bit, Free protocol
    TSDP_SAI_A->CR1 = SAI_RISING_EDGE | SAI_CR1_DATASZ_16BIT | SAI_MASTER_RX;
    // FIFO Threshold = 0, flush FIFO
    TSDP_SAI_A->CR2 = SAI_xCR2_FFLUSH | SAI_FIFO_THR;
    // No offset, FS Active High, FS Active Lvl Len = 1, Frame bit sz = 8*16=128
    TSDP_SAI_A->FRCR = SAI_xFRCR_FSPOL | ((1UL - 1UL) << 8) | (128UL - 1UL);
    // 0...7 slots en, N slots = 8, slot size = DataSz, no offset
    TSDP_SAI_A->SLOTR = (0xFFUL << 16) | ((SAI_SLOT_CNT - 1UL) << 8);
    TSDP_SAI_A->IMR = 0;  // No irq
#endif

#if 1 // ==== DMA ====
    TSDP_SAI_A->CR1 |= SAI_xCR1_DMAEN; // Enable DMA
    PDmaRx = dmaStreamAlloc(SAI_DMA_A, IRQ_PRIO_MEDIUM, DmaRxEndIRQ, nullptr);
    dmaStreamSetPeripheral(PDmaRx, &TSDP_SAI_A->DR);
    dmaStreamSetMemory0(PDmaRx, BufW);
    dmaStreamSetTransactionSize(PDmaRx, SAMPLE_CNT);
    dmaStreamSetMode(PDmaRx, SAI_DMARX_MODE);
    dmaStreamEnable(PDmaRx);
#endif

//    TSDP_SAI_A->IMR = SAI_xIMR_FREQIE;
//    nvicEnableVector(SAI_IRQ_NUMBER, IRQ_PRIO_MEDIUM);

//    EnableSAI();
}

void TSDP_t::StartRx(Fs_t Fs) {
    // Setup Fs
    Clk.DisablePllSai2();
    TSDP_SAI_A->CR1 &= ~SAI_xCR1_MCKDIV;
    switch(Fs) {
        case fs16000:
            // fSAI = 4MHz * 43 / 7 = 24.571429MHz; fSAI / (MCLK_div=6) / 256 = 16kHz
            Clk.SetupPllSai2(43, 2, 7);
            TSDP_SAI_A->CR1 |= 3UL << 20; // MCLKDIV=3 => div=6
            break;

        case fs22050:
            // fSAI = 4MHz * 48 / 17 = 11.29411765MHz; fSAI / (MCLK_div=2) / 256 = 22.059kHz
            Clk.SetupPllSai2(48, 2, 17);
            TSDP_SAI_A->CR1 |= 1UL << 20; // MCLKDIV=1 => div=2
            break;

        case fs32000:
            // fSAI = 4MHz * 86 / 7 = 24.571429MHz; fSAI / (MCLK_div=6) / 256 = 32kHz
            Clk.SetupPllSai2(86, 2, 7);
            TSDP_SAI_A->CR1 |= 3UL << 20; // MCLKDIV=3 => div=6
            break;

        case fs44100:
            // fSAI = 4MHz * 48 / 17 = 11.29411765MHz; fSAI / (MCLK_div=1) / 256 = 44.118kHz
            Clk.SetupPllSai2(48, 2, 17);
            TSDP_SAI_A->CR1 |= 0UL << 20; // MCLKDIV=0 => div=1
            break;

        case fs48000:
            // fSAI = 4MHz * 43 / 7 = 24.571429MHz; fSAI / (MCLK_div=2) / 256 = 48kHz
            Clk.SetupPllSai2(43, 2, 7);
            TSDP_SAI_A->CR1 |= 1UL << 20; // MCLKDIV=1 => div=2
            break;
    } // switch

    if(Clk.EnablePllSai2() == retvOk) {
        Clk.EnablePllSai2POut();
        Printf("SAI2 en; %X\r", RCC->CR);
    }
    else {
        Printf("SAI1 fail\r");
        return;
    }

    // Start transmission
    dmaStreamSetMemory0(PDmaRx, BufW);
    dmaStreamSetMode(PDmaRx, SAI_DMARX_MODE);
    dmaStreamSetTransactionSize(PDmaRx, SAMPLE_CNT); // 2 samples per item
    dmaStreamEnable(PDmaRx);
    EnableSAI(); // Start tx
}


void TSDP_t::OnNewSampleI() {

}

#if 0 // ============================== IRQ ====================================
extern "C"
OSAL_IRQ_HANDLER(SAI_IRQ_HANDLER) {
    OSAL_IRQ_PROLOGUE();
    if(TSDP_SAI_A->SR & SAI_xSR_FREQ) {
//        Sample.Left = AU_SAI_B->DR;
//        Sample.Right = AU_SAI_B->DR;
//        Tsdp.OnNewSampleI();
        Period++;
        (void)TSDP_SAI_A->DR;
        (void)TSDP_SAI_A->DR;
//        (void)TSDP_SAI_A->DR;
//        (void)TSDP_SAI_A->DR;
//        (void)TSDP_SAI_A->DR;
//        (void)TSDP_SAI_A->DR;
//        (void)TSDP_SAI_A->DR;
//        (void)TSDP_SAI_A->DR;
    }
    OSAL_IRQ_EPILOGUE();
}

#endif
