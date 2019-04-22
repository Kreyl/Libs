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

extern "C"
void DmaUartTxIrq(void *p, uint32_t flags);

struct UartParams_t {
    uint32_t Baudrate;
    USART_TypeDef* Uart;
    GPIO_TypeDef *PGpioTx;
    uint16_t PinTx;
    GPIO_TypeDef *PGpioRx;
    uint16_t PinRx;
    // DMA
    uint32_t DmaTxID, DmaRxID;
    uint32_t DmaModeTx, DmaModeRx;
    // MCU-specific
#if defined STM32F072xB || defined STM32L4XX
    bool UseIndependedClock;
#endif
    UartParams_t(uint32_t ABaudrate, USART_TypeDef* AUart,
            GPIO_TypeDef *APGpioTx, uint16_t APinTx,
            GPIO_TypeDef *APGpioRx, uint16_t APinRx,
            uint32_t ADmaTxID, uint32_t ADmaRxID,
            uint32_t ADmaModeTx, uint32_t ADmaModeRx
#if defined STM32F072xB || defined STM32L4XX
    , bool AUseIndependedClock
#endif
    ) : Baudrate(ABaudrate), Uart(AUart),
            PGpioTx(APGpioTx), PinTx(APinTx), PGpioRx(APGpioRx), PinRx(APinRx),
            DmaTxID(ADmaTxID), DmaRxID(ADmaRxID),
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
protected:
    const stm32_dma_stream_t *PDmaTx;
    const stm32_dma_stream_t *PDmaRx;
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
    virtual void ProcessByteIfReceived() = 0;
#if UART_USE_DMA
    void IRQDmaTxHandler();
#endif
    uint8_t GetByte(uint8_t *b);
};

class CmdUart_t : public BaseUart_t, public PrintfHelper_t, public Shell_t {
private:
    void IOnTxEnd() {} // Dummy
    uint8_t IPutChar(char c) { return IPutByte(c);  }
    void IStartTransmissionIfNotYet() { BaseUart_t::IStartTransmissionIfNotYet(); }
    void Print(const char *format, ...) {
        va_list args;
        va_start(args, format);
        IVsPrintf(format, args);
        va_end(args);
    }
public:
    CmdUart_t(const UartParams_t *APParams) : BaseUart_t(APParams) {}
    void ProcessByteIfReceived();
    void SignalCmdProcessed() { BaseUart_t::SignalRxProcessed(); }
};

class CmdUart485_t : public CmdUart_t {
private:
    PinOutput_t PinTxRx;
    void IStartTransmissionIfNotYet() {
        PinTxRx.SetHi();
        BaseUart_t::IStartTransmissionIfNotYet();
    }
    void IOnTxEnd() {
#ifdef USART_SR_TC
        while(!(Params->Uart->SR & USART_SR_TC)); // wait last bit to be shifted out
#else
        while(!(Params->Uart->ISR & USART_ISR_TC)); // wait last bit to be shifted out
#endif
        PinTxRx.SetLo();
    }
public:
    void Init() {
        CmdUart_t::Init();
        PinTxRx.Init();
        PinTxRx.SetLo();
    }
    CmdUart485_t(const UartParams_t *APParams, GPIO_TypeDef *APGPIO, uint16_t APin, PinOutMode_t AOutputType) :
        CmdUart_t(APParams), PinTxRx(APGPIO, APin, AOutputType) {}
};

#if 1 // ==== Modbus ====
#define MODBUS_DATA_LEN     (252+1) // + LRC

class ModbusCmd_t {
private:
    uint32_t Cnt;
    bool Started = false;
    char IString[CMD_BUF_SZ];
    uint8_t Parse();
public:
    union {
        uint64_t __Align;
        struct {
            uint8_t Addr;
            uint8_t Function;
            uint8_t Data[MODBUS_DATA_LEN];
        };
    };
    uint32_t DataCnt;

    ProcessDataResult_t PutChar(char c);
    void Reset() { Started = false; }

};

class ModbusUart485_t : public BaseUart_t, public PrintfHelper_t {
private:
    PinOutput_t PinTxRx;
    uint8_t IPutChar(char c) { return IPutByte(c);  }
    void IStartTransmissionIfNotYet() {
        PinTxRx.SetHi();
        BaseUart_t::IStartTransmissionIfNotYet();
    }
    void IOnTxEnd();

    void Print(const char *format, ...) {
        va_list args;
        va_start(args, format);
        IVsPrintf(format, args);
        va_end(args);
    }
public:
    ModbusCmd_t Cmd;

    void Init() {
        BaseUart_t::Init();
        PinTxRx.Init();
        PinTxRx.SetLo();
    }
    void ProcessByteIfReceived();
    void SignalCmdProcessed() { BaseUart_t::SignalRxProcessed(); }

    void Reply();

    ModbusUart485_t(const UartParams_t *APParams, GPIO_TypeDef *APGPIO, uint16_t APin, PinOutMode_t AOutputType) :
        BaseUart_t(APParams), PinTxRx(APGPIO, APin, AOutputType) {}
};
#endif

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
