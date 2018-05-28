/*
 * kl_nfc.cpp
 *
 *  Created on: 2 џэт. 2018 у.
 *      Author: Kreyl
 */

#include "kl_nfc.h"
#include "board.h"
#include "crc.h"

#define UART_DMA_TX_MODE(Chnl) \
                            STM32_DMA_CR_CHSEL(Chnl) | \
                            DMA_PRIORITY_LOW | \
                            STM32_DMA_CR_MSIZE_BYTE | \
                            STM32_DMA_CR_PSIZE_BYTE | \
                            STM32_DMA_CR_MINC |       /* Memory pointer increase */ \
                            STM32_DMA_CR_DIR_M2P |    /* Direction is memory to peripheral */ \
                            STM32_DMA_CR_TCIE         /* Enable Transmission Complete IRQ */

#define UART_DMA_RX_MODE(Chnl) \
                            STM32_DMA_CR_CHSEL((Chnl)) | \
                            DMA_PRIORITY_MEDIUM | \
                            STM32_DMA_CR_MSIZE_BYTE | \
                            STM32_DMA_CR_PSIZE_BYTE | \
                            STM32_DMA_CR_MINC |       /* Memory pointer increase */ \
                            STM32_DMA_CR_DIR_P2M |    /* Direction is peripheral to memory */ \
                            STM32_DMA_CR_CIRC         /* Circular buffer enable */


// Settings
static const UartParams_t KlNfcParams = {
        UART4,
        GPIOA, 0,
        GPIOA, 1,
        // DMA
        KLNFC_DMA_TX, KLNFC_DMA_RX,
        UART_DMA_TX_MODE(KLNFC_DMA_CHNL), UART_DMA_RX_MODE(KLNFC_DMA_CHNL),
#if defined STM32F072xB || defined STM32L4XX
        UART_USE_INDEPENDENT_CLK
#endif
};

KlNfc_t Nfc(KLNFC_TX_PIN, &KlNfcParams);
NfcPkt_t IPktTx, IPktRx;
static uint8_t *IPtr;
bool WasEE = false;
static thread_reference_t ThdRef = nullptr;
static enum NfcPktState_t { npsStart, npsData } PktState = npsStart;

static THD_WORKING_AREA(waKlNfc, 256);
__noreturn
static void KlNfcThread(void *arg) {
    chRegSetThreadName("KlNfc");
    Nfc.ITask();
}

static void ResetRx() {
//    Printf("ResetRx\r");
    PktState = npsData;   // pkt started
    IPtr = (uint8_t*)&IPktRx;
    WasEE = false;
}
static void AppendPkt(uint8_t b) {
//    Printf("App %X\r", b);
    *IPtr++ = b;
    if(IPtr == ((uint8_t*)&IPktRx + NFCPKT_SZ)) { // End of pkt
        PktState = npsStart;
//        IPktRx.Print();
        // Check crc
        uint16_t RcvdCrc = IPktRx.crc;
        IPktRx.CalculateCrc();
        if(RcvdCrc == IPktRx.crc) { // CRC ok
            IPktRx.Print();
        }
        else Printf("Nfc CRC err\r");
    }
}

__noreturn
void KlNfc_t::ITask() {
    while(true) {
        uint32_t Delay_ms = Random::Generate(36, 108);
        chThdSleepMilliseconds(Delay_ms);
//        Transmit(IPktTx);
        // Check what received
        uint8_t b;
        while(GetByte(&b) == retvOk) {
//            Printf("%X %u %u\r", b, PktState, WasEE);
            if(PktState == npsStart) {
                if(b == 0xEE) ResetRx();
            }
            else { // receiving data
                if(b == 0xEE) {
                    if(WasEE) {
                        WasEE = false;
                        AppendPkt(b);
                    }
                    else WasEE = true;
                }
                else { // Not EE
                    if(WasEE) ResetRx();    // Start of new pkt occured
                    AppendPkt(b);
                }
            } // rcvng data
        } // while get byte
    } // while true
}

void KlNfc_t::Transmit(NfcPkt_t &Pkt) {
    DisableRx();
    ITxPin.EnablePin();
    // Calculate crc
    Pkt.CalculateCrc();
    // Send StartByte
    IPutByte(0xEE);
    // Send packet byte by byte
    uint8_t *p = (uint8_t*)&Pkt;
    for(uint8_t i=0; i<NFCPKT_SZ; i++) {
        if(*p == 0xEE) IPutByte(0xEE);  // Duplicate EE
        IPutByte(*p++);
    }
    // Enter TX and wait IRQ
    chSysLock();
    IStartTransmissionIfNotYet();
    chThdSuspendS(&ThdRef); // Wait IRQ
    chSysUnlock();          // Will be here when IRQ fires
    ITxPin.DisablePin();
    EnableRx();
}

static void TCCallback() {
//    PrintfI("TC\r");
    chThdResumeI(&ThdRef, MSG_OK);
}
void KlNfc_t::IOnTxEnd() {
//    PrintfI("DMATxE\r");
    EnableTCIrq(IRQ_PRIO_MEDIUM, TCCallback);
}

void KlNfc_t::Init() {
    // Power pin
    PinSetupOut(GPIOC, 1, omPushPull);
    PinSetHi(GPIOC, 1);
    // TX pin
    ITxPin.Init();
    ITxPin.SetFrequencyHz(1300000);
    ITxPin.Timer.SetTriggerInput(tiETRF);
    ITxPin.Timer.SetEtrPolarity(invInverted);
    ITxPin.Timer.SelectSlaveMode(smGated);
    ITxPin.Set(1);
    ITxPin.DisablePin();
//     Modulation input pin: T2 ETR
    PinSetupAlterFunc(GPIOA, 15, omPushPull, pudNone, AF1);
    // UART
    BaseUart_t::Init(10000);

    IPktTx.ID = 7;
    chThdCreateStatic(waKlNfc, sizeof(waKlNfc), NORMALPRIO, (tfunc_t)KlNfcThread, NULL);
}


void NfcPkt_t::CalculateCrc() {
    crc = calc_crc16((char*)this, 4);
}
