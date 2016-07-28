/*
 * cmd_uart.cpp
 *
 *  Created on: 15.04.2013
 *      Author: kreyl
 */

#include "cmd_uart_f10x.h"
#include <string.h>

CmdUart_t Uart;

static inline void FPutChar(char c) { Uart.IPutChar(c); }

void CmdUart_t::IPutChar(char c) {
    *PWrite++ = c;
    if(PWrite >= &TXBuf[UART_TXBUF_SIZE]) PWrite = TXBuf;   // Circulate buffer
}

void CmdUart_t::Printf(const char *format, ...) {
    uint32_t MaxLength = (PWrite < PRead)? (PRead - PWrite) : ((UART_TXBUF_SIZE + PRead) - PWrite);
    va_list args;
    va_start(args, format);
    IFullSlotsCount += kl_vsprintf(FPutChar, MaxLength, format, args);
    va_end(args);

    // Start transmission if Idle
    if(IDmaIsIdle) {
        IDmaIsIdle = false;
        dmaStreamSetMemory0(UART_DMA_TX, PRead);
        uint32_t PartSz = (TXBuf + UART_TXBUF_SIZE) - PRead;    // Char count from PRead to buffer end
        ITransSize = (IFullSlotsCount > PartSz)? PartSz : IFullSlotsCount;  // How many to transmit now
        dmaStreamSetTransactionSize(UART_DMA_TX, ITransSize);
        dmaStreamSetMode(UART_DMA_TX, UART_DMA_TX_MODE);
        dmaStreamEnable(UART_DMA_TX);
    }
}

#if UART_RX_ENABLED
static inline bool TryConvertToDigit(uint8_t b, uint8_t *p) {
    if((b >= '0') and (b <= '9')) {
        *p = b - '0';
        return true;
    }
    else if((b >= 'A') and (b <= 'F')) {
        *p = 0x0A + b - 'A';
        return true;
    }
    else return false;
}
static inline bool IsDelimiter(uint8_t b) { return (b == ',') or (b == ' '); }
static inline bool IsEnd(uint8_t b) { return (b == '\r') or (b == '\n'); }

static WORKING_AREA(waUartRxThread, 256);
static void UartRxThread(void *arg) {
    chRegSetThreadName("UartRx");
    while(true) Uart.IRxTask();
}

void CmdUart_t::IRxTask() {
    chThdSleepMilliseconds(UART_RX_POLLING_MS);
    int32_t Sz = UART_RXBUF_SZ - UART_DMA_RX->channel->CNDTR;   // Number of bytes copied to buffer since restart
    if(Sz != SzOld) {
        int32_t ByteCnt = Sz - SzOld;
        if(ByteCnt < 0) ByteCnt += UART_RXBUF_SZ;   // Handle buffer circulation
        SzOld = Sz;
        for(int32_t i=0; i<ByteCnt; i++) {          // Iterate received bytes
            IProcessByte(IRxBuf[RIndx++]);
            if(RIndx >= UART_RXBUF_SZ) RIndx = 0;
        }
    }
}

void CmdUart_t::IProcessByte(uint8_t b) {
    if(IsEnd(b)) {
        if(NbrIndx == 2) UartCmdCallback(Nbr[0], Nbr[1], Nbr[2]);
        IResetCmd();
        return;
    }
    uint8_t d=0;
    switch(RxState) {
        case rsNbr1:
            if(TryConvertToDigit(b, &d)) {
                Nbr[NbrIndx] = d;
                RxState = rsNbr2;
            }
            else if(IsDelimiter(b)) {
                if(NbrIndx == 2) IResetCmd();
                else {
                    NbrIndx++;
                    RxState = rsNbr1;
                }
            }
            else IResetCmd();
            break;

        case rsNbr2:
            if(TryConvertToDigit(b, &d)) {
                Nbr[NbrIndx] = Nbr[NbrIndx]*10 + d;
                RxState = rsNbr3;
            }
            else if(IsDelimiter(b)) {
                if(NbrIndx == 2) IResetCmd();
                else {
                    NbrIndx++;
                    RxState = rsNbr1;
                }
            }
            else IResetCmd();
            break;

        case rsNbr3:
            if(TryConvertToDigit(b, &d)) {
                Nbr[NbrIndx] = Nbr[NbrIndx]*10 + d;
                if(NbrIndx == 2) RxState = rsWaitingEnd;
            }
            else if(IsDelimiter(b)) {
                if(NbrIndx == 2) IResetCmd();
                else {
                    NbrIndx++;
                    RxState = rsNbr1;
                }
            }
            else IResetCmd();
            break;

        case rsWaitingEnd:  // Will be here in case of something else after CMD
            IResetCmd();
            break;
    } // switch
}
#endif

// ==== Init & DMA ====
// Wrapper for IRQ
extern "C" {
void CmdUartTxIrq(void *p, uint32_t flags) { Uart.IRQDmaTxHandler(); }
}

void CmdUart_t::Init(uint32_t ABaudrate) {
    PWrite = TXBuf;
    PRead = TXBuf;
    IDmaIsIdle = true;
    IFullSlotsCount = 0;
    PinSetupAlterFuncOutput(UART_GPIO, UART_TX_PIN, omPushPull);      // TX1

    // ==== USART configuration ====
    UART_RCC_ENABLE();
    UART->CR1 = USART_CR1_UE;     // Enable USART
    if(UART == USART1) UART->BRR = Clk.APB2FreqHz / ABaudrate;
    else               UART->BRR = Clk.APB1FreqHz / ABaudrate;
    UART->CR2 = 0;
    // ==== DMA ====
    dmaStreamAllocate     (UART_DMA_TX, IRQ_PRIO_MEDIUM, CmdUartTxIrq, NULL);
    dmaStreamSetPeripheral(UART_DMA_TX, &UART->DR);
    dmaStreamSetMode      (UART_DMA_TX, UART_DMA_TX_MODE);

#if UART_RX_ENABLED
    UART->CR1 = USART_CR1_TE | USART_CR1_RE;        // TX & RX enable
    UART->CR3 = USART_CR3_DMAT | USART_CR3_DMAR;    // Enable DMA at TX & RX
    IResetCmd();
    PinSetupIn(UART_GPIO, UART_RX_PIN,  pudPullUp);
    dmaStreamAllocate     (UART_DMA_RX, IRQ_PRIO_LOW, nullptr, NULL);
    dmaStreamSetPeripheral(UART_DMA_RX, &UART->DR);
    dmaStreamSetMemory0   (UART_DMA_RX, IRxBuf);
    dmaStreamSetTransactionSize(UART_DMA_RX, UART_RXBUF_SZ);
    dmaStreamSetMode      (UART_DMA_RX, UART_DMA_RX_MODE);
    dmaStreamEnable       (UART_DMA_RX);
    // Create and start thread
    chThdCreateStatic(waUartRxThread, sizeof(waUartRxThread), NORMALPRIO, (tfunc_t)UartRxThread, NULL);
#else
    UART->CR1 = USART_CR1_TE;     // Transmitter enabled
    UART->CR3 = USART_CR3_DMAT;   // Enable DMA at transmitter
#endif
    UART->CR1 |= USART_CR1_UE;       // Enable USART
}

// ==== TX DMA IRQ ====
void CmdUart_t::IRQDmaTxHandler() {
    dmaStreamDisable(UART_DMA_TX);    // Registers may be changed only when stream is disabled
    IFullSlotsCount -= ITransSize;
    PRead += ITransSize;
    if(PRead >= (TXBuf + UART_TXBUF_SIZE)) PRead = TXBuf; // Circulate pointer

    if(IFullSlotsCount == 0) IDmaIsIdle = true; // Nothing left to send
    else {  // There is something to transmit more
        dmaStreamSetMemory0(UART_DMA_TX, PRead);
        uint32_t PartSz = (TXBuf + UART_TXBUF_SIZE) - PRead;
        ITransSize = (IFullSlotsCount > PartSz)? PartSz : IFullSlotsCount;
        dmaStreamSetTransactionSize(UART_DMA_TX, ITransSize);
        dmaStreamSetMode(UART_DMA_TX, UART_DMA_TX_MODE);
        dmaStreamEnable(UART_DMA_TX);    // Restart DMA
    }
}
