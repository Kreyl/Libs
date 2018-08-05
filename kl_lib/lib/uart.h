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
    uint32_t Baudrate;
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
    UartParams_t(uint32_t ABaudrate, USART_TypeDef* AUart, GPIO_TypeDef *APGpioTx,
            uint16_t APinTx, GPIO_TypeDef *APGpioRx, uint16_t APinRx,
            const stm32_dma_stream_t *APDmaTx, const stm32_dma_stream_t *APDmaRx,
            uint32_t ADmaModeTx, uint32_t ADmaModeRx
#if defined STM32F072xB || defined STM32L4XX
    , bool AUseIndependedClock
#endif
    ) :         Baudrate(ABaudrate), Uart(AUart), PGpioTx(APGpioTx), PinTx(APinTx),
                PGpioRx(APGpioRx), PinRx(APinRx), PDmaTx(APDmaTx), PDmaRx(APDmaRx),
                DmaModeTx(ADmaModeTx), DmaModeRx(ADmaModeRx)
#if defined STM32F072xB || defined STM32L4XX
                , UseIndependedClock(AUseIndependedClock)
#endif
    {}
};

#define UART_USE_DMA        TRUE
#define UART_USE_TXE_IRQ    FALSE

#define UART_CMD_BUF_SZ     54 // payload bytes
#define UART_RX_POLLING_MS  99

// ==== Base class ====
class BaseUart_t {
private:
    const UartParams_t *Params;
#if UART_USE_DMA
    char TXBuf[UART_TXBUF_SZ];
    char *PRead, *PWrite;
    bool IDmaIsIdle;
    uint32_t IFullSlotsCount, ITransSize;
    void ISendViaDMA();
#endif
    int32_t OldWIndx, RIndx;
    uint8_t IRxBuf[UART_RXBUF_SZ];
protected:
    bool RxProcessed = true;
    virtual_timer_t TmrRx;
    void SignalRxProcessed();
    uint8_t IPutByte(uint8_t b);
    uint8_t IPutByteNow(uint8_t b);
    void IStartTransmissionIfNotYet();
    virtual void IOnTxEnd() = 0;
    // ==== Constructor ====
    BaseUart_t(const UartParams_t *APParams) : Params(APParams)
#if UART_USE_DMA
    , PRead(TXBuf), PWrite(TXBuf), IDmaIsIdle(true), IFullSlotsCount(0), ITransSize(0)
#endif
    , OldWIndx(0), RIndx(0)
    {}
public:
    void Init();
    void StartRx();
    void Shutdown();
    void OnClkChange();
    // Enable/Disable
    void EnableTx()  { Params->Uart->CR1 |= USART_CR1_TE; }
    void DisableTx() { Params->Uart->CR1 &= ~USART_CR1_TE; }
    void EnableRx()  { Params->Uart->CR1 |= USART_CR1_RE; }
    void DisableRx() { Params->Uart->CR1 &= ~USART_CR1_RE; }
#if UART_USE_DMA
    void FlushTx() { while(!IDmaIsIdle) chThdSleepMilliseconds(1); }  // wait DMA
#endif
    void EnableTCIrq(const uint32_t Priority, ftVoidVoid ACallback);
    // Inner use
#if UART_USE_DMA
    void IRQDmaTxHandler();
#endif
    uint32_t GetRcvdBytesCnt();
    uint8_t GetByte(uint8_t *b);
    virtual void IIrqHandler() = 0;
};

class CmdUart_t : public BaseUart_t, public PrintfHelper_t, public Shell_t {
private:
    void IOnTxEnd() {} // Dummy
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
    void IIrqHandler();
    void SignalCmdProcessed() { BaseUart_t::SignalRxProcessed(); }
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
