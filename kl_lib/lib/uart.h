/*
 * cmd_uart.h
 *
 *  Created on: 15.04.2013
 *      Author: kreyl
 */

#pragma once

#include "kl_lib.h"
#include <cstring>
#include "shell.h"
#include "board.h"

struct UartParams_t {
    USART_TypeDef* Uart;
    GPIO_TypeDef *PGpioTx;
    uint16_t PinTx;
    GPIO_TypeDef *PGpioRx;
    uint16_t PinRx;
    // DMA
    const stm32_dma_stream_t *PDmaTx;
    const stm32_dma_stream_t *PDmaRx;
    uint32_t DmaModeTx, DmaModeRx;
    // MCU-specific
#if defined STM32F072xB || defined STM32L4XX
    bool UseIndependedClock;
#endif
};

#define UART_USE_DMA        TRUE

// Set to true if RX needed
#define UART_RX_ENABLED     TRUE

#if UART_RX_ENABLED // ==== RX ====
#define UART_RXBUF_SZ       99 // unprocessed bytes
#define UART_CMD_BUF_SZ     54 // payload bytes
#define UART_RX_POLLING_MS  99
#endif

// ==== Base class ====
class BaseUart_t {
private:
    const UartParams_t *Params;
    uint32_t IBaudrate;
#if UART_USE_DMA
    char TXBuf[UART_TXBUF_SZ];
    char *PRead, *PWrite;
    bool IDmaIsIdle;
    uint32_t IFullSlotsCount, ITransSize;
    void ISendViaDMA();
#endif
#if UART_RX_ENABLED
    int32_t OldWIndx, RIndx;
    uint8_t IRxBuf[UART_RXBUF_SZ];
#endif
protected:
    uint8_t IPutByte(uint8_t b);
    uint8_t IPutByteNow(uint8_t b);
    void IStartTransmissionIfNotYet();
    // ==== Constructor ====
    BaseUart_t(const UartParams_t *APParams) : Params(APParams), IBaudrate(115200)
#if UART_USE_DMA
    , PRead(TXBuf), PWrite(TXBuf), IDmaIsIdle(true), IFullSlotsCount(0), ITransSize(0)
#endif
#if UART_RX_ENABLED
    , OldWIndx(0), RIndx(0)
#endif
    {}
public:
    void Init(uint32_t ABaudrate);
    void Shutdown();
    void OnClkChange();
#if UART_USE_DMA
    void FlushTx() { while(!IDmaIsIdle) chThdSleepMilliseconds(1); }  // wait DMA
#endif
    // Inner use
#if UART_USE_DMA
    void IRQDmaTxHandler();
#endif
#if UART_RX_ENABLED
    uint32_t GetRcvdBytesCnt();
    uint8_t GetByte(uint8_t *b);
#endif
};

class CmdUart_t : public BaseUart_t, public PrintfHelper_t, public Shell_t {
private:
    uint8_t IPutChar(char c) { return IPutByte(c);  }
    void IStartTransmissionIfNotYet() { BaseUart_t::IStartTransmissionIfNotYet(); }
    void Printf(const char *format, ...) {
        va_list args;
        va_start(args, format);
        IVsPrintf(format, args);
        va_end(args);
    }
public:
    CmdUart_t(const UartParams_t *APParams) : BaseUart_t(APParams) {}
    void Init(uint32_t ABaudrate);
    void IRxTask();
};

#define BYTE_UART_EN    FALSE
#if BYTE_UART_EN
class ByteUart_t : public BaseUart_t, public ByteShell_t {
//private:
//    uint8_t IPutChar(char c) { return IPutByte(c);  }
//    void IStartTransmissionIfNotYet() { BaseUart_t::IStartTransmissionIfNotYet(); }
public:
    ByteUart_t(const UartParams_t *APParams) : BaseUart_t(APParams) {}
    void Init(uint32_t ABaudrate);
    uint8_t IPutChar(char c) { return IPutByte(c); }
    void IStartTransmissionIfNotYet() { BaseUart_t::IStartTransmissionIfNotYet(); }
    void IRxTask();
};
#endif
