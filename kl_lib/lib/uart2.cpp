/*
 * cmd_uart.cpp
 *
 *  Created on: 15.04.2013
 *      Author: kreyl
 */

#include "MsgQ.h"
#include <string.h>
#include "uart2.h"
#include "kl_lib.h"

#if 1 // ========================= Base UART ===================================
#if 1 // ==== TX ====

#if UART_USE_TXE_IRQ // ============ TxC IRQ =============
static ftVoidVoid ITxC1IrqCallback = nullptr;
static ftVoidVoid ITxC2IrqCallback = nullptr;
static ftVoidVoid ITxC3IrqCallback = nullptr;
#if defined UART4
static ftVoidVoid ITxC4IrqCallback = nullptr;
#endif
#if defined UART5
static ftVoidVoid ITxC5IrqCallback = nullptr;
#endif
#if defined USART6
static ftVoidVoid ITxC6IrqCallback = nullptr;
#endif

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
#if defined STM32L4XX
        nvicEnableVector(USART3_IRQn, Priority);
#elif defined STM32F0XX
        nvicEnableVector(USART3_4_IRQn, Priority);
#endif
    }
#if defined UART4
    else if(Params->Uart == UART4) {
        ITxC4IrqCallback = ACallback;
        nvicEnableVector(UART4_IRQn, Priority);
    }
#endif
#if defined UART5
    else if(Params->Uart == UART5) {
        ITxC5IrqCallback = ACallback;
        nvicEnableVector(UART5_IRQn, Priority);
    }
#endif
#if defined USART6
    else if(Params->Uart == USART6) {
        ITxC6IrqCallback = ACallback;
        nvicEnableVector(USART6_IRQn, Priority);
    }
#endif
    Params->Uart->CR1 |= USART_CR1_TCIE;
}
#endif

#if 1 // ===== IRQs =====
#if defined STM32L4XX
extern "C" {

static BaseUart_t *IUart1=nullptr;
void VectorD4() {   // USART1
    CH_IRQ_PROLOGUE();
    chSysLockFromISR();
    uint32_t SR = USART1->ISR;
    USART1->ICR = 0x121BCF; // Clear flags
    if(IUart1) IUart1->OnUartIrqI(SR);
    chSysUnlockFromISR();
    CH_IRQ_EPILOGUE();
}

static BaseUart_t *IUart2=nullptr;
void VectorD8() {   // USART2
    CH_IRQ_PROLOGUE();
    chSysLockFromISR();
    uint32_t SR = USART2->ISR;
    USART2->ICR = 0x121BCF; // Clear flags
    if(IUart2) IUart2->OnUartIrqI(SR);
    chSysUnlockFromISR();
    CH_IRQ_EPILOGUE();
}

#ifdef USART3
static BaseUart_t *IUart3=nullptr;
void VectorDC() {   // USART3
    CH_IRQ_PROLOGUE();
    chSysLockFromISR();
    uint32_t SR = USART3->ISR;
    USART3->ICR = 0x121BCF; // Clear flags
    if(IUart3) IUart3->OnUartIrqI(SR);
    chSysUnlockFromISR();
    CH_IRQ_EPILOGUE();
}
#endif

#ifdef UART4
static BaseUart_t *IUart4=nullptr;
void Vector110() {   // UART4
    CH_IRQ_PROLOGUE();
    chSysLockFromISR();
    uint32_t SR = UART4->ISR;
    UART4->ICR = 0x121BCF; // Clear flags
    if(IUart4) IUart4->OnUartIrqI(SR);
    chSysUnlockFromISR();
    CH_IRQ_EPILOGUE();
}
#endif

#ifdef UART5
static BaseUart_t *IUart5=nullptr;
void Vector114() {   // UART5
    CH_IRQ_PROLOGUE();
    chSysLockFromISR();
    uint32_t SR = UART5->ISR;
    UART5->ICR = 0x121BCF; // Clear flags
    if(IUart5) IUart5->OnUartIrqI(SR);
    chSysUnlockFromISR();
    CH_IRQ_EPILOGUE();
}
#endif
} // extern C
#elif defined STM32F0XX
static BaseUart_t *IUart1=nullptr, *IUart2=nullptr, *IUart3=nullptr;

extern "C" {
void VectorAC() {   // USART1
    CH_IRQ_PROLOGUE();
    chSysLockFromISR();
    uint32_t SR = USART1->ISR;
    USART1->ICR = 0x121B5F; // Clear flags
    if(IUart1) IUart1->OnUartIrqI(SR);
    chSysUnlockFromISR();
    CH_IRQ_EPILOGUE();
}

void VectorB0() {   // USART2
    CH_IRQ_PROLOGUE();
    chSysLockFromISR();
    uint32_t SR = USART2->ISR;
    USART2->ICR = 0x121B5F; // Clear flags
    if(IUart2) IUart2->OnUartIrqI(SR);
    chSysUnlockFromISR();
    CH_IRQ_EPILOGUE();
}

void VectorB4() {   // USART3
    CH_IRQ_PROLOGUE();
    chSysLockFromISR();
    uint32_t SR = USART3->ISR;
    USART3->ICR = 0x121B5F; // Clear flags
    if(IUart3) IUart3->OnUartIrqI(SR);
    chSysUnlockFromISR();
    CH_IRQ_EPILOGUE();
}
} // extern C
#endif
#endif // IRQs

// Wrapper for TX IRQ
extern "C"
void DmaUartTxIrq(void *p, uint32_t flags) { ((BaseUart_t*)p)->IRQDmaTxHandler(); }

// ==== TX DMA IRQ ====
void BaseUart_t::IRQDmaTxHandler() {
    dmaStreamDisable(PDmaTx);    // Registers may be changed only when stream is disabled
    IFullSlotsCount -= ITransSize;
    PRead += ITransSize;
    if(PRead >= (TXBuf + UART_TXBUF_SZ)) PRead = TXBuf; // Circulate pointer

    if(IFullSlotsCount == 0) {  // Nothing left to send
        IDmaIsIdle = true;
    }
    else ISendViaDMA();
}

void BaseUart_t::ISendViaDMA() {
    uint32_t PartSz = (TXBuf + UART_TXBUF_SZ) - PRead; // Cnt from PRead to end of buf
    ITransSize = MIN_(IFullSlotsCount, PartSz);
    if(ITransSize != 0) {
        IDmaIsIdle = false;
        dmaStreamSetMemory0(PDmaTx, PRead);
        dmaStreamSetTransactionSize(PDmaTx, ITransSize);
        dmaStreamSetMode(PDmaTx, Params->DmaModeTx);
        dmaStreamEnable(PDmaTx);
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

uint8_t BaseUart_t::IPutByteNow(uint8_t b) {
    while(!(Params->Uart->ISR & USART_ISR_TXE));
    Params->Uart->TDR = b;
    while(!(Params->Uart->ISR & USART_ISR_TXE));
    return retvOk;
}
#endif // TX

#if 1 // ==== RX ====
uint8_t BaseUart_t::GetByte(uint8_t *b) {
#if defined STM32F2XX || defined STM32F4XX || defined STM32F7XX
    int32_t WIndx = UART_RXBUF_SZ - PDmaRx->stream->NDTR;
#else
    int32_t WIndx = UART_RXBUF_SZ - PDmaRx->channel->CNDTR;
#endif
    int32_t BytesCnt = WIndx - RIndx;
    if(BytesCnt < 0) BytesCnt += UART_RXBUF_SZ;
    if(BytesCnt == 0) return retvEmpty;
    *b = IRxBuf[RIndx++];
    if(RIndx >= UART_RXBUF_SZ) RIndx = 0;
    return retvOk;
}

void BaseUart_t::FlushRx() {
    uint8_t b;
    while(GetByte(&b) == retvOk);
}
#endif // RX

#if 1 // ==== Init ====
void BaseUart_t::Init() {
    AlterFunc_t PinAF = AF1;
#if 1 // ==== Tx pin ====
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
#elif defined STM32F7XX
    if(Params->Uart == USART1) { // AF4, AF7
        if(Params->PGpioTx == GPIOB and Params->PinTx == 14) PinAF = AF4;
        else PinAF = AF7;
    }
    else if(Params->Uart == USART2 or Params->Uart == USART3) PinAF = AF7;
    else if(Params->Uart == UART4) { // AF6, AF8
        if(Params->PGpioTx == GPIOA and Params->PinTx == 12) PinAF = AF6;
        else PinAF = AF8;
    }
    else if(Params->Uart == UART5) { // AF1, AF7, AF8
        if(Params->PGpioTx == GPIOB and Params->PinTx == 6) PinAF = AF1;
        else if(Params->PGpioTx == GPIOB and Params->PinTx == 9) PinAF = AF7;
        else PinAF = AF8;
    }
    else if(Params->Uart == USART6 or Params->Uart == UART8) PinAF = AF8;
    else if(Params->Uart == UART7) { // AF8, AF12
        if(Params->PGpioTx == GPIOA or Params->PGpioTx == GPIOB) PinAF = AF12;
        else PinAF = AF8;
    }
#else
#error "UART AF not defined"
#endif
    PinSetupAlterFunc(Params->PGpioTx, Params->PinTx, omPushPull, pudNone, PinAF);
#endif

#if 1 // Setup independent clock if possible and required
#if defined STM32F072xB
    if(Params->ClkSrc == uartclkHSI) Clk.EnableHSI();
    uint32_t Offset = (Params->Uart == USART1)? 0 : 16;
    RCC->CFGR3 &= ~(0b11UL << Offset);
    RCC->CFGR3 |= ((uint32_t)Params->ClkSrc) << Offset;
#elif defined STM32L4XX
    uint32_t Offset = 0; // Usart1
    if(Params->Uart == USART2) Offset = 2;
    else if(Params->Uart == USART3) Offset = 4;
#ifdef UART4
    else if(Params->Uart == UART4) Offset = 6;
#endif
#ifdef UART5
    else if(Params->Uart == UART5) Offset = 8;
#endif
    RCC->CCIPR &= ~(0b11UL << Offset); // Clear current bits
    RCC->CCIPR |= ((uint32_t)Params->ClkSrc) << Offset;
    // Enable HSI if needed
    if(Params->ClkSrc == uartclkHSI) Clk.EnableHSI();
#elif defined STM32F7XX
    uint32_t Offset = 0; // Usart1
    if(Params->Uart == USART2) Offset = 2;
    else if(Params->Uart == USART3) Offset = 4;
    else if(Params->Uart == UART4) Offset = 6;
    else if(Params->Uart == UART5) Offset = 8;
    else if(Params->Uart == USART6) Offset = 10;
    else if(Params->Uart == UART7) Offset = 12;
    else if(Params->Uart == UART8) Offset = 14;
    RCC->DCKCFGR2 &= ~(0b11UL << Offset); // Clear current bits
    RCC->DCKCFGR2 |= ((uint32_t)Params->ClkSrc) << Offset;
    // Enable HSI if needed
    if(Params->ClkSrc == uartclkHSI) Clk.EnableHSI();
#endif
#endif // Independent clock

#if 1 // ==== Clock ====
    if     (Params->Uart == USART1) { rccEnableUSART1(FALSE); IUart1 = this; }
    else if(Params->Uart == USART2) { rccEnableUSART2(FALSE); IUart2 = this; }
#if defined USART3
    else if(Params->Uart == USART3) { rccEnableUSART3(FALSE); IUart3 = this; }
#endif
#if defined UART4
    else if(Params->Uart == UART4) { rccEnableUART4(FALSE); IUart4 = this; }
#endif
#if defined UART5
    else if(Params->Uart == UART5) { rccEnableUART5(FALSE); IUart5 = this; }
#endif
#if defined USART6
    else if(Params->Uart == USART6) { rccEnableUSART6(FALSE); }
#endif
#if defined UART7
    else if(Params->Uart == UART7) { rccEnableUART7(FALSE); }
#endif
#if defined UART8
    else if(Params->Uart == UART8) { rccEnableUART8(FALSE); }
#endif
#endif // Clock

    OnClkChange();  // Setup baudrate

    Params->Uart->CR2 = 0;  // Nothing that interesting there
    // ==== DMA ====
    // Remap DMA request if needed
#if defined STM32F0XX
    if(PDmaTx == STM32_DMA1_STREAM4) SYSCFG->CFGR1 |= SYSCFG_CFGR1_USART1TX_DMA_RMP;
#endif
    PDmaTx = dmaStreamAlloc(Params->DmaTxID, IRQ_PRIO_MEDIUM, DmaUartTxIrq, this);
    dmaStreamSetPeripheral(PDmaTx, &Params->Uart->TDR);
    dmaStreamSetMode      (PDmaTx, Params->DmaModeTx);
    IDmaIsIdle = true;

    // ==== RX ====
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
#elif defined STM32F7XX
    if(Params->Uart == USART1) { // AF4, AF7
        if(Params->PGpioTx == GPIOB and Params->PinTx == 15) PinAF = AF4;
        else PinAF = AF7;
    }
    else if(Params->Uart == USART2 or Params->Uart == USART3) PinAF = AF7;
    else if(Params->Uart == UART4) { // AF6, AF8
        if(Params->PGpioTx == GPIOA and Params->PinTx == 11) PinAF = AF6;
        else PinAF = AF8;
    }
    else if(Params->Uart == UART5) { // AF1, AF7, AF8
        if(Params->PGpioTx == GPIOB and Params->PinTx == 5) PinAF = AF1;
        else if(Params->PGpioTx == GPIOB and Params->PinTx == 8) PinAF = AF7;
        else PinAF = AF8;
    }
    else if(Params->Uart == USART6 or Params->Uart == UART8) PinAF = AF8;
    else if(Params->Uart == UART7) { // AF8, AF12
        if(Params->PGpioTx == GPIOA or Params->PGpioTx == GPIOB) PinAF = AF12;
        else PinAF = AF8;
    }
#else
#error "UART AF not defined"
#endif
    PinSetupAlterFunc(Params->PGpioRx, Params->PinRx, omOpenDrain, pudPullUp, PinAF);
    // Remap DMA request if needed
#if defined STM32F0XX
    if(PDmaRx == STM32_DMA1_STREAM5) SYSCFG->CFGR1 |= SYSCFG_CFGR1_USART1RX_DMA_RMP;
#endif
    // DMA
    PDmaRx = dmaStreamAlloc(Params->DmaRxID, IRQ_PRIO_MEDIUM, nullptr, NULL);
    dmaStreamSetPeripheral(PDmaRx, &Params->Uart->RDR);
    dmaStreamSetMemory0   (PDmaRx, IRxBuf);
    dmaStreamSetTransactionSize(PDmaRx, UART_RXBUF_SZ);
    dmaStreamSetMode      (PDmaRx, Params->DmaModeRx);
    dmaStreamEnable       (PDmaRx);

    // Enable IRQ on <LF>
    Params->Uart->CR2 |= ((uint32_t)('\n')) << 24; // What to recognize
    Params->Uart->CR1 |= USART_CR1_CMIE; // Enable IRQ on match
    if(Params->Uart == USART1) nvicEnableVector(USART1_IRQn, IRQ_PRIO_LOW);
    else if(Params->Uart == USART2) nvicEnableVector(USART2_IRQn, IRQ_PRIO_LOW);
#if defined STM32L4XX
    else if(Params->Uart == USART3) nvicEnableVector(USART3_IRQn, IRQ_PRIO_LOW);
#elif defined STM32F0XX
    else if(Params->Uart == USART3) nvicEnableVector(USART3_4_IRQn, IRQ_PRIO_LOW);
#else
#error "USART3 IRQ not defined"
#endif
#if defined UART4
    else if(Params->Uart == UART4) nvicEnableVector(UART4_IRQn, IRQ_PRIO_LOW);
#endif
#if defined UART5
    else if(Params->Uart == UART5) nvicEnableVector(UART5_IRQn, IRQ_PRIO_LOW);
#endif

    Params->Uart->CR1 |= USART_CR1_UE;    // Enable USART
}

void BaseUart_t::Shutdown() {
    Params->Uart->CR1 &= ~USART_CR1_UE; // UART Disable
    if     (Params->Uart == USART1) { rccDisableUSART1(); }
    else if(Params->Uart == USART2) { rccDisableUSART2(); }
#if defined USART3
    else if(Params->Uart == USART3) { rccDisableUSART3(); }
#endif
#if defined UART4
    else if(Params->Uart == UART4) { rccDisableUART4(); }
#endif
#if defined UART5
    else if(Params->Uart == UART5) { rccDisableUART5(); }
#endif
#if defined USART6
    else if(Params->Uart == USART6) { rccDisableUSART6(); }
#endif
#if defined UART7
    else if(Params->Uart == UART7) { rccDisableUART7(); }
#endif
#if defined UART8
    else if(Params->Uart == UART8) { rccDisableUART8(); }
#endif
}

void BaseUart_t::OnClkChange() {
#if defined STM32L1XX || defined STM32F1XX
    if(Params->Uart == USART1) Params->Uart->BRR = Clk.APB2FreqHz / Params->Baudrate;
    else                       Params->Uart->BRR = Clk.APB1FreqHz / Params->Baudrate;
#elif defined STM32F072xB
    if(Params->Uart == USART1 or Params->Uart == USART2) Params->Uart->BRR = HSI_FREQ_HZ / Params->Baudrate;
    else Params->Uart->BRR = Clk.APBFreqHz / Params->Baudrate;
#elif defined STM32F0XX
    Params->Uart->BRR = Clk.APBFreqHz / IBaudrate;
#elif defined STM32F2XX || defined STM32F4XX
    if(Params->Uart == USART1 or Params->Uart == USART6) Params->Uart->BRR = Clk.APB2FreqHz / Params->Baudrate;
    else Params->Uart->BRR = Clk.APB1FreqHz / Params->Baudrate;
#elif defined STM32L4XX
    switch(Params->ClkSrc) {
        case uartclkPCLK:
            if(Params->Uart == USART1) Params->Uart->BRR = Clk.APB2FreqHz / Params->Baudrate;
            else Params->Uart->BRR = Clk.APB1FreqHz / Params->Baudrate;
            break;
        case uartclkSYSCLK:
            Params->Uart->BRR = Clk.GetSysClkHz() / Params->Baudrate;
            break;
        case uartclkHSI:
            Params->Uart->BRR = HSI_FREQ_HZ / Params->Baudrate;
            break;
        case uartclkLSE:
            Params->Uart->BRR = LSE_FREQ_HZ / Params->Baudrate;
            break;
    } // switch
#elif defined STM32F7XX
    switch(Params->ClkSrc) {
        case uartclkPCLK:
            if(Params->Uart == USART1 or Params->Uart == USART6)
                Params->Uart->BRR = Clk.APB2FreqHz / Params->Baudrate;
            else Params->Uart->BRR = Clk.APB1FreqHz / Params->Baudrate;
            break;
        case uartclkSYSCLK:
            Params->Uart->BRR = Clk.GetSysClkHz() / Params->Baudrate;
            break;
        case uartclkHSI:
            Params->Uart->BRR = HSI_FREQ_HZ / Params->Baudrate;
            break;
        case uartclkLSE:
            Params->Uart->BRR = LSE_FREQ_HZ / Params->Baudrate;
            break;
    } // switch
#else
#error "UART BRR not defined"
#endif
}
#endif // Init

#endif // Base UART

#if 1 // ========================== CMD UART ===================================
void CmdUart_t::OnUartIrqI(uint32_t flags) {
    if(flags & USART_ISR_CMF) {
        EvtQMain.SendNowOrExitI(EvtMsg_t(evtIdShellCmdRcvd, (void*)this));
    }
}

uint8_t CmdUart_t::ReceiveBinaryToBuf(uint8_t *ptr, uint32_t Len, uint32_t Timeout_ms) {
    uint8_t Rslt = retvOk;
    // Wait for previous TX to complete
    while(!IDmaIsIdle);
    while(!(Params->Uart->ISR & USART_ISR_TXE)); // Wait
    Params->Uart->CR1 &= ~USART_CR1_CMIE; // Disable IRQ on char match
    // Setup DMA to given buffer
    dmaStreamDisable(PDmaRx);
    dmaStreamSetMemory0(PDmaRx, ptr);
    dmaStreamSetTransactionSize(PDmaRx, Len);
    dmaStreamSetMode(PDmaRx, Params->DmaModeRx & (~STM32_DMA_CR_CIRC));
    dmaStreamEnable(PDmaRx);
    // Start transmission
    systime_t Start = chVTGetSystemTimeX();
    Params->Uart->TDR = '>';
    while(dmaStreamGetTransactionSize(PDmaRx) > 0U) {
        if(chVTTimeElapsedSinceX(Start) > TIME_MS2I(Timeout_ms)) {
            Rslt = retvTimeout;
            break;
        }
    }
    // Return to self buffer
    dmaStreamDisable(PDmaRx);
    dmaStreamSetMemory0(PDmaRx, IRxBuf);
    dmaStreamSetTransactionSize(PDmaRx, UART_RXBUF_SZ);
    dmaStreamSetMode(PDmaRx, Params->DmaModeRx);
    Params->Uart->CR1 |= USART_CR1_CMIE; // Enable IRQ on match
    RIndx = 0; // Reset RX buf pointer
    dmaStreamEnable(PDmaRx);
    return Rslt;
}

uint8_t CmdUart_t::TransmitBinaryFromBuf(uint8_t *ptr, uint32_t Len, uint32_t Timeout_ms) {
    systime_t Start = chVTGetSystemTimeX();
    // Wait '>'
    uint8_t b = 0;
    while(b != '>') {
        GetByte(&b);
        if(chVTTimeElapsedSinceX(Start) > TIME_MS2I(Timeout_ms)) return retvTimeout;
    }
    // Wait for previousTX to complete
    while(!IDmaIsIdle);
    while(!(Params->Uart->ISR & USART_ISR_TXE));
    // Setup DMA to given buffer
    dmaStreamDisable(PDmaTx);
    dmaStreamSetMemory0(PDmaTx, ptr);
    dmaStreamSetTransactionSize(PDmaTx, Len);
    dmaStreamSetMode(PDmaTx, Params->DmaModeTx & (~STM32_DMA_CR_TCIE));
    dmaStreamEnable(PDmaTx);
    dmaWaitCompletion(PDmaTx);
    return retvOk;
}
#endif

#if 1 // ========================== HostUart485_t ==============================
void HostUart485_t::OnUartIrqI(uint32_t flags) {
    if(flags & USART_ISR_CMF) {
        chThdResumeI(&ThdRef, MSG_OK); // NotNull check perfprmed inside chThdResumeI
    }
}

uint8_t HostUart485_t::TryParseRxBuff() {
    uint8_t b;
    while(GetByte(&b) == retvOk) {
        if(Reply.PutChar(b) == pdrNewCmd) return retvOk;
    } // while get byte
    return retvFail;
}

uint8_t HostUart485_t::SendCmd(uint32_t Timeout_ms, int32_t RetryCnt, const char* ACmd, uint32_t Addr, const char *format, ...) {
    while(RetryCnt-- > 0) {
        FlushRx();
        Print("%S %u", ACmd, Addr);
        if(format and *format != 0) {
            IPutByte(' '); // Add space after addr if something follows
            va_list args;
            va_start(args, format);
            IVsPrintf(format, args);
            va_end(args);
        }
        chSysLock();
        PrintEOL();
        // Receive reply
        msg_t Rslt = chThdSuspendTimeoutS(&ThdRef, TIME_MS2I(Timeout_ms)); // Wait IRQ
        chSysUnlock();  // Will be here when IRQ will fire, or timeout occur
        if(Rslt == MSG_OK) {
            if(TryParseRxBuff() == retvOk) return retvOk;
        }
    }
    return retvTimeout;
}

void HostUart485_t::SendBroadcast(uint32_t Delay_ms, int32_t RepeatCnt, const char* ACmd, const char *format, ...) {
    while(RepeatCnt-- > 0) {
        Print("%S", ACmd); // FF means everybody
        if(format and *format != 0) {
            IPutByte(' '); // Add space after addr if something follows
            va_list args;
            va_start(args, format);
            IVsPrintf(format, args);
            va_end(args);
        }
        PrintEOL();
        if(Delay_ms) chThdSleepMilliseconds(Delay_ms);
    }
}

uint8_t HostUart485_t::SendCmdAndTransmitBuf(uint32_t Timeout_ms, uint8_t *PBuf, uint32_t Len, const char* ACmd, uint32_t Addr, const char *format, ...) {
    Print("%S %u", ACmd, Addr);
    if(format and *format != 0) {
        IPutByte(' '); // Add space after addr if something follows
        va_list args;
        va_start(args, format);
        IVsPrintf(format, args);
        va_end(args);
    }
    PrintEOL();
    if(TransmitBinaryFromBuf(PBuf, Len, Timeout_ms) == retvOk) {
        // Receive reply
        chSysLock();
        msg_t Rslt = MSG_OK;
        if(TryParseRxBuff() == retvOk) { // Maybe reply is already there
            chSysUnlock();
            return retvOk;
        }
        else {
            Rslt = chThdSuspendTimeoutS(&ThdRef, TIME_MS2I(Timeout_ms)); // Wait IRQ
        }
        chSysUnlock();
        if(Rslt == MSG_OK) return TryParseRxBuff();
        else return retvTimeout;
    }
    else return retvNoAnswer;
}

uint8_t HostUart485_t::SendCmdAndReceiveBuf(uint32_t Timeout_ms, uint8_t *PBuf, uint32_t Len, const char* ACmd, uint32_t Addr, const char *format, ...) {
    Print("%S %u", ACmd, Addr);
    if(format and *format != 0) {
        IPutByte(' '); // Add space after addr if something follows
        va_list args;
        va_start(args, format);
        IVsPrintf(format, args);
        va_end(args);
    }
    PrintEOL();
    return ReceiveBinaryToBuf(PBuf, Len, Timeout_ms);
}
#endif

#if 0 // ========================== CmdUart422_t ===============================
void CmdUart422_t::OnUartIrqI(uint32_t flags) {
    if(flags & USART_ISR_CMF) {
        if (WaitingReply)
            chThdResumeI(&ThdRef, MSG_OK); // NotNull check perfprmed inside chThdResumeI
        else
            EvtQMain.SendNowOrExitI(EvtMsg_t(evtIdShell422CmdRcvd, (void*)this));
    }
}

uint8_t CmdUart422_t::TryParseRxBuff() {
    uint8_t b;
    while(GetByte(&b) == retvOk) {
        if(Reply.PutChar(b) == pdrNewCmd) return retvOk;
    } // while get byte
    return retvFail;
}

uint8_t CmdUart422_t::SendCmd(uint32_t Timeout_ms, int32_t RetryCnt, const char* ACmd, const char *format, ...) {
    while(RetryCnt-- > 0) {
        FlushRx();
        Print("%S", ACmd);
        if(format and *format != 0) {
            IPutByte(' '); // Add space after cmd if something follows
            va_list args;
            va_start(args, format);
            IVsPrintf(format, args);
            va_end(args);
        }
        chSysLock();
        PrintEOL();
        // Receive reply
        WaitingReply = true;
        msg_t Rslt = chThdSuspendTimeoutS(&ThdRef, TIME_MS2I(Timeout_ms)); // Wait IRQ
        WaitingReply = false;
        chSysUnlock();  // Will be here when IRQ will fire, or timeout occur
        if(Rslt == MSG_OK) {
            if(TryParseRxBuff() == retvOk) return retvOk;
        }
    }
    return retvTimeout;
}

uint8_t CmdUart422_t::SendCmdAndTransmitBuf(uint32_t Timeout_ms, uint8_t *PBuf, uint32_t Len, const char* ACmd, const char *format, ...) {
    Print("%S", ACmd);
    if(format and *format != 0) {
        IPutByte(' '); // Add space after cmd if something follows
        va_list args;
        va_start(args, format);
        IVsPrintf(format, args);
        va_end(args);
    }
    PrintEOL();
    if(TransmitBinaryFromBuf(PBuf, Len, Timeout_ms) == retvOk) {
        // Receive reply
        chSysLock();
        msg_t Rslt = MSG_OK;
        if(TryParseRxBuff() == retvOk) { // Maybe reply is already there
            chSysUnlock();
            return retvOk;
        }
        else {
            WaitingReply = true;
            Rslt = chThdSuspendTimeoutS(&ThdRef, TIME_MS2I(Timeout_ms)); // Wait IRQ
            WaitingReply = false;
        }
        chSysUnlock();
        if(Rslt == MSG_OK) return TryParseRxBuff();
        else return retvTimeout;
    }
    else return retvNoAnswer;
}

uint8_t CmdUart422_t::SendCmdAndReceiveBuf(uint32_t Timeout_ms, uint8_t *PBuf, uint32_t Len, const char* ACmd, const char *format, ...) {
    Print("%S", ACmd);
    if(format and *format != 0) {
        IPutByte(' '); // Add space after cmd if something follows
        va_list args;
        va_start(args, format);
        IVsPrintf(format, args);
        va_end(args);
    }
    PrintEOL();
    return ReceiveBinaryToBuf(PBuf, Len, Timeout_ms);
}
#endif

#if MODBUS_UART_EN // ========================= Modbus =========================
ProcessDataResult_t ModbusCmd_t::PutChar(char c) {
    // Start of cmd
    if(c == ':') {
        Started = true;
        Cnt = 0;
    }
    // End of cmd
    else if((c == '\r') or (c == '\n')) {   // end of line, check if cmd completed
        Started = false;
        if(Cnt >= 6) { // if not too short
            IString[Cnt] = 0; // End of string
            Cnt = 0;
            if(Parse() == retvOk) return pdrNewCmd;
        }
    }
    // Some other char
    else {
        if(Started) { // Ignore if not
            // Check if ascii
            if((c >= '0' and c <= '9') or (c >= 'A' and c <= 'F') or (c >= 'a' and c <= 'f')) {
                if(Cnt < (CMD_BUF_SZ-1)) IString[Cnt++] = c;  // Add char if buffer not full
            }
        }
    }
    return pdrProceed;
}

uint8_t CharToByte(char c, uint8_t *PRslt) {
    if(c >= '0' and c <= '9') { *PRslt = (c - '0'); return retvOk; }
    else if(c >= 'A' and c <= 'F') { *PRslt = (0xA + c - 'A'); return retvOk; }
    else if(c >= 'a' and c <= 'f') { *PRslt = (0xA + c - 'a'); return retvOk; }
    else return retvFail;
}

uint8_t TwoCharsToByte(char c1, char c2, uint8_t *PRslt) {
    uint8_t b1, b2;
    if(CharToByte(c1, &b1) != retvOk) return retvFail;
    if(CharToByte(c2, &b2) != retvOk) return retvFail;
    b1 <<= 4;
    b1 |= b2;
    *PRslt = b1;
    return retvOk;
}

uint8_t ModbusCmd_t::Parse() {
    // Addr
    if(TwoCharsToByte(IString[0], IString[1], &Addr) != retvOk) return retvFail;
    // Function
    if(TwoCharsToByte(IString[2], IString[3], &Function) != retvOk) return retvFail;
    // Data
    char* p = &IString[4];
    uint8_t LRC = Addr + Function;
    DataCnt = 0;
    while(true) {
        uint8_t b;
        if(TwoCharsToByte(p[0], p[1], &b) != retvOk) break; // End of string
        Data[DataCnt++] = b;
        LRC += b;
        p += 2;
    }
    // Check LRC
    if(LRC == 0) {
        DataCnt--; // Remove last LRC byte
        return retvOk;
    }
    else return retvFail;
}

void ModbusUart485_t::ProcessByteIfReceived() {
    if(!RxProcessed) return;
    uint8_t b;
    while(GetByte(&b) == retvOk) {
        if(Cmd.PutChar(b) == pdrNewCmd) {
            RxProcessed = false;
//            EvtQMain.SendNowOrExit(EvtMsg_t(evtIdModbusCmd));
        } // if new cmd
    } // while get byte
}

void ModbusUart485_t::IOnTxEnd() {
#ifdef USART_SR_TC
    Params->Uart->SR &= ~USART_SR_TC; // Clear TxCompleted flag
    for(volatile uint32_t i=0; i<1000; i++) {
        if(Params->Uart->SR & USART_SR_TC) break; // wait last bit to be shifted out
    }
#else
    Params->Uart->ISR &= ~USART_ISR_TC; // Clear TxCompleted flag
    for(volatile uint32_t i=0; i<1000; i++) {
        if(Params->Uart->ISR & USART_ISR_TC) break; // wait last bit to be shifted out
    }
#endif
    PinTxRx.SetLo();
}

void ModbusUart485_t::Reply() {
    // Calc LRC
    uint8_t LRC = Cmd.Addr + Cmd.Function;
    for(uint32_t i=0; i<Cmd.DataCnt; i++) LRC += Cmd.Data[i];
    LRC = (uint8_t)(-(int32_t)LRC);
    Print(":%02X%02X%A%02X\r\n", Cmd.Addr, Cmd.Function, Cmd.Data, Cmd.DataCnt, 0, LRC);
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
