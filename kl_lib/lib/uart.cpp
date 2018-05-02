/*
 * cmd_uart.cpp
 *
 *  Created on: 15.04.2013
 *      Author: kreyl
 */

#include "MsgQ.h"
#include <string.h>
#include "uart.h"
#include "kl_lib.h"

#if 1 // ==================== Common and eternal ===============================
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

// Pins Alternate function
#if defined STM32L4XX || defined STM32F0XX
#define UART_TX_REG     TDR
#define UART_RX_REG     RDR
#elif defined STM32L1XX || defined STM32F2XX
#define UART_TX_REG     DR
#define UART_RX_REG     DR
#else
#error "Not defined"
#endif

// PutChar functions
//void PutCharCmd(char c) { Uart.i

// Settings
static const UartParams_t UartParams = {
        CMD_UART,
        UART_GPIO, UART_TX_PIN,
        UART_GPIO, UART_RX_PIN,
        // DMA
        UART_DMA_TX, UART_DMA_RX,
        UART_DMA_TX_MODE(UART_DMA_CHNL), UART_DMA_RX_MODE(UART_DMA_CHNL),
#if defined STM32F072xB || defined STM32L4XX
        UART_USE_INDEPENDENT_CLK
#endif
};

#endif // Common and eternal

// ===================================== Variables =============================
thread_reference_t IRxThd = nullptr;
CmdUart_t Uart {&UartParams};

#if 1 // ========================= Base UART ===================================
#if 1 // ==== TX ====

#if UART_USE_TXE_IRQ // ============ TxC IRQ =============
static ftVoidVoid ITxC1IrqCallback = nullptr;
static ftVoidVoid ITxC2IrqCallback = nullptr;
static ftVoidVoid ITxC3IrqCallback = nullptr;
static ftVoidVoid ITxC4IrqCallback = nullptr;
static ftVoidVoid ITxC5IrqCallback = nullptr;
static ftVoidVoid ITxC6IrqCallback = nullptr;

void BaseUart_t::EnableTCIrq(const uint32_t Priority, ftVoidVoid ACallback) {
    ITxC1IrqCallback = ACallback;
    if(Params->Uart == USART1) {
        ITxC1IrqCallback = ACallback;
        nvicEnableVector(USART1_IRQn, Priority);
    }
    else if(Params->Uart == USART2) {
        ITxC2IrqCallback = ACallback;
        nvicEnableVector(USART2_IRQn, Priority);
    }
    else if(Params->Uart == USART3) {
        ITxC3IrqCallback = ACallback;
        nvicEnableVector(USART3_IRQn, Priority);
    }
    else if(Params->Uart == UART4) {
        ITxC4IrqCallback = ACallback;
        nvicEnableVector(UART4_IRQn, Priority);
    }
    else if(Params->Uart == UART5) {
        ITxC5IrqCallback = ACallback;
        nvicEnableVector(UART5_IRQn, Priority);
    }
    else if(Params->Uart == USART6) {
        ITxC6IrqCallback = ACallback;
        nvicEnableVector(USART6_IRQn, Priority);
    }
    Params->Uart->CR1 |= USART_CR1_TCIE;
}

extern "C" {
void VectorD4() {   // USART1
    CH_IRQ_PROLOGUE();
    chSysLockFromISR();
    uint32_t SR = USART1->SR;
    if(SR & USART_SR_TC) {
        if(ITxC1IrqCallback != nullptr) ITxC1IrqCallback();
        else PrintfC("Unhandled %S\r", __FUNCTION__);
        USART1->SR &= ~USART_SR_TC;
    }
    chSysUnlockFromISR();
    CH_IRQ_EPILOGUE();
}

void VectorD8() {   // USART2
    CH_IRQ_PROLOGUE();
    chSysLockFromISR();
    uint32_t SR = USART2->SR;
    if(SR & USART_SR_TC) {
        if(ITxC1IrqCallback != nullptr) ITxC2IrqCallback();
        else PrintfC("Unhandled %S\r", __FUNCTION__);
        USART2->SR &= ~USART_SR_TC;
    }
    chSysUnlockFromISR();
    CH_IRQ_EPILOGUE();
}

void VectorDC() {   // USART3
    CH_IRQ_PROLOGUE();
    chSysLockFromISR();
    uint32_t SR = USART3->SR;
    if(SR & USART_SR_TC) {
        if(ITxC1IrqCallback != nullptr) ITxC3IrqCallback();
        else PrintfC("Unhandled %S\r", __FUNCTION__);
        USART3->SR &= ~USART_SR_TC;
    }
    chSysUnlockFromISR();
    CH_IRQ_EPILOGUE();
}

void Vector110() {   // UART4
    CH_IRQ_PROLOGUE();
    chSysLockFromISR();
    uint32_t SR = UART4->SR;
    if(SR & USART_SR_TC) {
        if(ITxC1IrqCallback != nullptr) ITxC4IrqCallback();
        else PrintfC("Unhandled %S\r", __FUNCTION__);
        UART4->SR &= ~USART_SR_TC;
    }
    chSysUnlockFromISR();
    CH_IRQ_EPILOGUE();
}
void Vector114() {   // UART5
    CH_IRQ_PROLOGUE();
    chSysLockFromISR();
    uint32_t SR = UART5->SR;
    if(SR & USART_SR_TC) {
        if(ITxC1IrqCallback != nullptr) ITxC5IrqCallback();
        else PrintfC("Unhandled %S\r", __FUNCTION__);
        UART5->SR &= ~USART_SR_TC;
    }
    chSysUnlockFromISR();
    CH_IRQ_EPILOGUE();
}

void Vector15C() {   // USART6
    CH_IRQ_PROLOGUE();
    chSysLockFromISR();
    uint32_t SR = USART6->SR;
    if(SR & USART_SR_TC) {
        if(ITxC1IrqCallback != nullptr) ITxC6IrqCallback();
        else PrintfC("Unhandled %S\r", __FUNCTION__);
        USART6->SR &= ~USART_SR_TC;
    }
    chSysUnlockFromISR();
    CH_IRQ_EPILOGUE();
}

} // extern C


#endif

#if UART_USE_DMA
// Wrapper for TX IRQ
extern "C"
void DmaUartTxIrq(void *p, uint32_t flags) { ((BaseUart_t*)p)->IRQDmaTxHandler(); }

// ==== TX DMA IRQ ====
void BaseUart_t::IRQDmaTxHandler() {
    dmaStreamDisable(Params->PDmaTx);    // Registers may be changed only when stream is disabled
    IFullSlotsCount -= ITransSize;
    PRead += ITransSize;
    if(PRead >= (TXBuf + UART_TXBUF_SZ)) PRead = TXBuf; // Circulate pointer

    if(IFullSlotsCount == 0) {  // Nothing left to send
        IDmaIsIdle = true;
        IOnTxEnd();
    }
    else ISendViaDMA();
}

void BaseUart_t::ISendViaDMA() {
    uint32_t PartSz = (TXBuf + UART_TXBUF_SZ) - PRead; // Cnt from PRead to end of buf
    ITransSize = MIN_(IFullSlotsCount, PartSz);
    if(ITransSize != 0) {
        IDmaIsIdle = false;
        dmaStreamSetMemory0(Params->PDmaTx, PRead);
        dmaStreamSetTransactionSize(Params->PDmaTx, ITransSize);
        dmaStreamSetMode(Params->PDmaTx, Params->DmaModeTx);
        dmaStreamEnable(Params->PDmaTx);
    }
}

uint8_t BaseUart_t::IPutByte(uint8_t b) {
    if(IFullSlotsCount >= UART_TXBUF_SZ) return retvOverflow;
    *PWrite++ = b;
    if(PWrite >= &TXBuf[UART_TXBUF_SZ]) PWrite = TXBuf;   // Circulate buffer
    IFullSlotsCount++;
    return retvOk;
}

void BaseUart_t::IStartTransmissionIfNotYet() {
    if(IDmaIsIdle) ISendViaDMA();
}
#else // DMA not used
uint8_t Uart_t::IPutChar(char c) {
    return IPutCharNow(c);
}
void Uart_t::IStartTransmissionIfNotYet() { }
#endif

uint8_t BaseUart_t::IPutByteNow(uint8_t b) {
#if defined STM32L1XX || defined STM32F2XX || defined STM32F4XX || defined STM32F10X_LD_VL
    while(!(Params->Uart->SR & USART_SR_TXE));
    Params->Uart->UART_TX_REG = b;
    while(!(Params->Uart->SR & USART_SR_TXE));
#elif defined STM32F0XX || defined STM32L4XX
    while(!(Params->Uart->ISR & USART_ISR_TXE));
    Params->Uart->UART_TX_REG = b;
    while(!(Params->Uart->ISR & USART_ISR_TXE));
#endif
    return retvOk;
}
#endif // TX

#if UART_RX_ENABLED // ==== RX ====
uint32_t BaseUart_t::GetRcvdBytesCnt() {
#if defined STM32F2XX || defined STM32F4XX
    int32_t WIndx = UART_RXBUF_SZ - Params->PDmaRx->stream->NDTR;
#else
    int32_t WIndx = UART_RXBUF_SZ - Params->PDmaRx->channel->CNDTR;
#endif
    int32_t Rslt = WIndx - RIndx;
    if(Rslt < 0) Rslt += UART_RXBUF_SZ;
    return Rslt;
}

uint8_t BaseUart_t::GetByte(uint8_t *b) {
    if(GetRcvdBytesCnt() == 0) return retvEmpty;
    *b = IRxBuf[RIndx++];
    if(RIndx >= UART_RXBUF_SZ) RIndx = 0;
    return retvOk;
}

#endif // RX

#if 1 // ==== Init ====
void BaseUart_t::Init(uint32_t ABaudrate) {
    AlterFunc_t PinAF;
    // ==== Tx pin ====
#if defined STM32L4XX || defined STM32L1XX || defined STM32F2XX
    PinAF = AF7;
#if defined UART4
    if(Params->Uart == UART4) PinAF = AF8;
#endif
#if defined UART5
    if(Params->Uart == UART5) PinAF = AF8;
#endif
#if defined USART6
    if(Params->Uart == USART6) PinAF = AF8;
#endif

#elif defined STM32F0XX
    if(Params->PGpioTx == GPIOA) PinAF = AF1;
    else if(Params->PGpioTx == GPIOB) PinAF = AF0;
#else
#error "UART AF not defined"
#endif
    PinSetupAlterFunc(Params->PGpioTx, Params->PinTx, omPushPull, pudNone, PinAF);
    IBaudrate = ABaudrate;
    // ==== Clock ====
    if     (Params->Uart == USART1) { rccEnableUSART1(FALSE); }
    else if(Params->Uart == USART2) { rccEnableUSART2(FALSE); }
#if defined USART3
    else if(Params->Uart == USART3) { rccEnableUSART3(FALSE); }
#endif
#if defined UART4
    else if(Params->Uart == UART4) { rccEnableUART4(FALSE); }
#endif
#if defined UART5
    else if(Params->Uart == UART5) { rccEnableUART5(FALSE); }
#endif
#if defined USART6
    else if(Params->Uart == USART6) { rccEnableUSART6(FALSE); }
#endif
    // Setup independent clock if possible and required
#if defined STM32F072xB
#error "Independed clock not implemented"
    // Setup HSI as UART's clk src
    if(UART == USART1) RCC->CFGR3 |= RCC_CFGR3_USART1SW_HSI;
    else if(UART == USART2) RCC->CFGR3 |= RCC_CFGR3_USART2SW_HSI;
#elif defined STM32L4XX
    if(Params->UseIndependedClock) {
        Clk.EnableHSI();    // HSI used as independent clock
        if     (Params->Uart == USART1) RCC->CCIPR |= 0b10;
        else if(Params->Uart == USART2) RCC->CCIPR |= 0b10 << 2;
        else if(Params->Uart == USART3) RCC->CCIPR |= 0b10 << 4;
        else if(Params->Uart == UART4)  RCC->CCIPR |= 0b10 << 6;
        else if(Params->Uart == UART5)  RCC->CCIPR |= 0b10 << 8;
    }
#endif
    OnClkChange();  // Setup baudrate

    Params->Uart->CR2 = 0;  // Nothing that interesting there
#if UART_USE_DMA    // ==== DMA ====
    // Remap DMA request if needed
#if defined STM32F0XX
    if(Params->PDmaTx == STM32_DMA1_STREAM4) SYSCFG->CFGR1 |= SYSCFG_CFGR1_USART1TX_DMA_RMP;
#endif
    dmaStreamAllocate     (Params->PDmaTx, IRQ_PRIO_MEDIUM, DmaUartTxIrq, this);
    dmaStreamSetPeripheral(Params->PDmaTx, &Params->Uart->UART_TX_REG);
    dmaStreamSetMode      (Params->PDmaTx, Params->DmaModeTx);
    IDmaIsIdle = true;
#endif

#if UART_RX_ENABLED
    Params->Uart->CR1 = USART_CR1_TE | USART_CR1_RE;        // TX & RX enable
    Params->Uart->CR3 = USART_CR3_DMAT | USART_CR3_DMAR;    // Enable DMA at TX & RX
    // ==== Rx pin ====
#if defined STM32L4XX || defined STM32L1XX || defined STM32F2XX
    PinAF = AF7; // for all USARTs save 4/5/6
#if defined UART4
    if(Params->Uart == UART4) PinAF = AF8;
#endif
#if defined UART5
    if(Params->Uart == UART5) PinAF = AF8;
#endif
#if defined USART6
    if(Params->Uart == USART6) PinAF = AF8;
#endif
#elif defined STM32F0XX
    if(Params->PGpioRx == GPIOA) PinAF = AF1;
    else if(Params->PGpioRx == GPIOB) PinAF = AF0;
#else
#error "UART AF not defined"
#endif
    PinSetupAlterFunc(Params->PGpioRx, Params->PinRx, omOpenDrain, pudPullUp, PinAF);
    // Remap DMA request if needed
#if defined STM32F0XX
    if(Params->PDmaRx == STM32_DMA1_STREAM5) SYSCFG->CFGR1 |= SYSCFG_CFGR1_USART1RX_DMA_RMP;
#endif
    // DMA
    dmaStreamAllocate     (Params->PDmaRx, IRQ_PRIO_LOW, nullptr, NULL);
    dmaStreamSetPeripheral(Params->PDmaRx, &Params->Uart->UART_RX_REG);
    dmaStreamSetMemory0   (Params->PDmaRx, IRxBuf);
    dmaStreamSetTransactionSize(Params->PDmaRx, UART_RXBUF_SZ);
    dmaStreamSetMode      (Params->PDmaRx, Params->DmaModeRx);
    dmaStreamEnable       (Params->PDmaRx);
#else // if UART_RX_ENABLED
    Params->Uart->CR1 = USART_CR1_TE;     // Transmitter enabled
#if UART_USE_DMA
    Params->Uart->CR3 = USART_CR3_DMAT;   // Enable DMA at transmitter
#endif
#endif
    Params->Uart->CR1 |= USART_CR1_UE;    // Enable USART
}

void BaseUart_t::Shutdown() {
    Params->Uart->CR1 &= ~USART_CR1_UE; // UART Disable
    if     (Params->Uart == USART1) { rccDisableUSART1(FALSE); }
    else if(Params->Uart == USART2) { rccDisableUSART2(FALSE); }
#if defined USART3
    else if(Params->Uart == USART3) { rccDisableUSART3(FALSE); }
#endif
#if defined UART4
    else if(Params->Uart == UART4) { rccDisableUART4(FALSE); }
#endif
#if defined UART5
    else if(Params->Uart == UART5) { rccDisableUART5(FALSE); }
#endif
}

void BaseUart_t::OnClkChange() {
#if defined STM32L1XX || defined STM32F100_MCUCONF
    if(Params->Uart == USART1) Params->Uart->BRR = Clk.APB2FreqHz / IBaudrate;
    else                       Params->Uart->BRR = Clk.APB1FreqHz / IBaudrate;
#elif defined STM32F072xB
    if(Params->Uart == USART1 or Params->Uart == USART2) Params->Uart->BRR = HSI_FREQ_HZ / IBaudrate;
    else Params->Uart->BRR = Clk.APBFreqHz / IBaudrate;
#elif defined STM32F0XX
    Params->Uart->BRR = Clk.APBFreqHz / IBaudrate;
#elif defined STM32F2XX || defined STM32F4XX
    if(Params->Uart == USART1 or Params->Uart == USART6) Params->Uart->BRR = Clk.APB2FreqHz / IBaudrate;
    else Params->Uart->BRR = Clk.APB1FreqHz / IBaudrate;
#elif defined STM32L4XX
    if(Params->UseIndependedClock) Params->Uart->BRR = HSI_FREQ_HZ / IBaudrate;
    else {
        if(Params->Uart == USART1) Params->Uart->BRR = Clk.APB2FreqHz / IBaudrate;
        else Params->Uart->BRR = Clk.APB1FreqHz / IBaudrate; // All others at APB1
    }
#endif
}
#endif // Init
#endif // Base UART

#if 1 // ========================= Cmd UART ====================================
#if UART_RX_ENABLED // ==== RX ====
static THD_WORKING_AREA(waUartRxThread, 128);
__noreturn
static void UartRxThread(void *arg) {
    chRegSetThreadName("UartRx");
    while(true) {
        chThdSleepMilliseconds(UART_RX_POLLING_MS);
        Uart.IRxTask();
    }
}

void CmdUart_t::IRxTask() {
    // Iterate received bytes
    uint8_t b;
    while(GetByte(&b) == retvOk) {
        if(Cmd.PutChar(b) == pdrNewCmd) {
            chSysLock();
            EvtMsg_t Msg(evtIdShellCmd, (Shell_t*)this);
            if(EvtQMain.SendNowOrExitI(Msg) == retvOk) {
                chSchGoSleepS(CH_STATE_SUSPENDED); // Wait until cmd processed
            }
            chSysUnlock();  // Will be here when application signals that cmd processed
        } // if new cmd
    } // whilw get byte
}

void CmdUart_t::SignalCmdProcessed() {
    chSysLock();
    if(IRxThd->state == CH_STATE_SUSPENDED) chSchReadyI(IRxThd);
    chSysUnlock();
}
#endif

void CmdUart_t::Init(uint32_t ABaudrate) {
    BaseUart_t::Init(ABaudrate);
#if UART_RX_ENABLED
    // Create RX Thread if not created
    if(IRxThd == nullptr) IRxThd = chThdCreateStatic(waUartRxThread, sizeof(waUartRxThread), NORMALPRIO, UartRxThread, NULL);
#endif
}
#endif

#if BYTE_UART_EN // ========================= Byte UART ========================
static const UartParams_t ByteUartParams = {
        FT_UART,
        FT_GPIO, FT_TX,
        FT_GPIO, FT_RX,
        // DMA
        FT_UART_DMA_TX, FT_UART_DMA_RX,
        UART_DMA_TX_MODE(FT_UART_DMA_CHNL), UART_DMA_RX_MODE(FT_UART_DMA_CHNL),
#if defined STM32F072xB || defined STM32L4XX
        false    // Use independed clock
#endif
};

ByteUart_t ByteUart(&ByteUartParams);
thread_reference_t IByteRxThd = nullptr;

static THD_WORKING_AREA(waByteUartRxThread, 128);
__noreturn
static void ByteUartRxThread(void *arg) {
    chRegSetThreadName("ByteUartRx");
    while(true) {
        chThdSleepMilliseconds(UART_RX_POLLING_MS);
        ByteUart.IRxTask();
    }
}

void ByteUart_t::IRxTask() {
    if(CmdProcessInProgress) return;    // Busy processing cmd
    // Iterate received bytes
//    Printf("1\r");
    uint8_t b;
    while(GetByte(&b) == retvOk) {
        if(Cmd.PutChar(b) == pdrNewCmd) {
            EvtMsg_t Msg(evtIdByteCmd, (ByteShell_t*)this);
            CmdProcessInProgress = (EvtQMain.SendNowOrExit(Msg) == retvOk);
        }
    }
}

void ByteUart_t::Init(uint32_t ABaudrate) {
    BaseUart_t::Init(ABaudrate);
#if UART_RX_ENABLED
    // Create RX Thread if not created
    if(IByteRxThd == nullptr) {
        IByteRxThd = chThdCreateStatic(waByteUartRxThread, sizeof(waByteUartRxThread),
                NORMALPRIO, ByteUartRxThread, NULL);
    }
#endif
}
#endif
