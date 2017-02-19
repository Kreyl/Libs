/*
 * cmd_uart.h
 *
 *  Created on: 15.04.2013
 *      Author: kreyl
 */

#pragma once

#include "kl_lib.h"
#include "kl_sprintf.h"
#include <cstring>
#include "shell.h"
#include "board.h"

// Set to true if RX needed
#define UART_RX_ENABLED     TRUE

#define UART_USE_DMA        TRUE
// On F0xx MCU, remap is done automatically

// ==== TX ====
#define UART_TXBUF_SZ       256

#define UART_DMA_TX_MODE    STM32_DMA_CR_CHSEL(UART_DMA_CHNL) | \
                            DMA_PRIORITY_LOW | \
                            STM32_DMA_CR_MSIZE_BYTE | \
                            STM32_DMA_CR_PSIZE_BYTE | \
                            STM32_DMA_CR_MINC |       /* Memory pointer increase */ \
                            STM32_DMA_CR_DIR_M2P |    /* Direction is memory to peripheral */ \
                            STM32_DMA_CR_TCIE         /* Enable Transmission Complete IRQ */

#if UART_RX_ENABLED // ==== RX ====
#define UART_RXBUF_SZ       99 // unprocessed bytes
#define UART_CMD_BUF_SZ     54 // payload bytes

#define UART_RX_POLLING_MS  99
#define UART_DMA_RX_MODE    STM32_DMA_CR_CHSEL(UART_DMA_CHNL) | \
                            DMA_PRIORITY_MEDIUM | \
                            STM32_DMA_CR_MSIZE_BYTE | \
                            STM32_DMA_CR_PSIZE_BYTE | \
                            STM32_DMA_CR_MINC |       /* Memory pointer increase */ \
                            STM32_DMA_CR_DIR_P2M |    /* Direction is peripheral to memory */ \
                            STM32_DMA_CR_CIRC         /* Circular buffer enable */
#endif

class Uart_t : public Shell_t {
private:
    uint32_t IBaudrate;
#if UART_USE_DMA
    char TXBuf[UART_TXBUF_SZ];
    char *PRead = TXBuf, *PWrite = TXBuf;
    bool IDmaIsIdle = true;
    uint32_t IFullSlotsCount = 0, ITransSize;
    void ISendViaDMA();
#endif
#if UART_RX_ENABLED
    int32_t SzOld, RIndx;
    uint8_t IRxBuf[UART_RXBUF_SZ];
#endif
public:
#if UART_RX_ENABLED
    void Init(uint32_t ABaudrate, GPIO_TypeDef *PGpioTx, const uint16_t APinTx, GPIO_TypeDef *PGpioRx, const uint16_t APinRx);
#else
    void Init(uint32_t ABaudrate, GPIO_TypeDef *PGpioTx, const uint16_t APinTx);
#endif
    void DeInit() {
        UART->CR1 &= ~USART_CR1_UE; // UART Disable
        if(UART == USART1) { rccDisableUSART1(FALSE); }
        else if(UART == USART2) { rccDisableUSART2(FALSE); }
    }
    void OnClkChange();
#if UART_USE_DMA
    void Printf(const char *S, ...);
    void PrintfI(const char *S, ...);
    void FlushTx() { while(!IDmaIsIdle); }  // wait DMA
#else
    void Printf(const char *S, ...) {}
    void PrintfI(const char *S, ...) {}
#endif
    void PrintfNow(const char *S, ...);

    // Inner use
#if UART_USE_DMA
    void IRQDmaTxHandler();
    void IPutChar(char c);
    void IPrintf(const char *format, va_list args);
#endif
#if UART_RX_ENABLED
    void IRxTask();
#endif
};

extern Uart_t Uart;

#define UartPrintfFunc()    Uart.Printf(" %S\r", __FUNCTION__)
