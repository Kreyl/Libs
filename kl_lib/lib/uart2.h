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
    uint32_t DmaTxID, DmaRxID;
    uint32_t DmaModeTx, DmaModeRx;
    // MCU-specific
#if defined STM32F072xB || defined STM32L4XX || defined STM32F7XX
    uartClk_t ClkSrc;
#endif
    UartParams_t(uint32_t ABaudrate, USART_TypeDef* AUart,
            GPIO_TypeDef *APGpioTx, uint16_t APinTx,
            GPIO_TypeDef *APGpioRx, uint16_t APinRx,
            uint32_t ADmaTxID, uint32_t ADmaRxID,
            uint32_t ADmaModeTx, uint32_t ADmaModeRx
#if defined STM32F072xB || defined STM32L4XX || defined STM32F7XX
    , uartClk_t AClkSrc
#endif
    ) : Baudrate(ABaudrate), Uart(AUart),
            PGpioTx(APGpioTx), PinTx(APinTx), PGpioRx(APGpioRx), PinRx(APinRx),
            DmaTxID(ADmaTxID), DmaRxID(ADmaRxID),
            DmaModeTx(ADmaModeTx), DmaModeRx(ADmaModeRx)
#if defined STM32F072xB || defined STM32L4XX || defined STM32F7XX
        , ClkSrc(AClkSrc)
#endif
    {}
};

#define UART_USE_TXE_IRQ    FALSE

#define UART_CMD_BUF_SZ     54 // payload bytes

// ==== Base class ====
class BaseUart_t {
protected:
    const stm32_dma_stream_t *PDmaTx = nullptr;
    const stm32_dma_stream_t *PDmaRx = nullptr;
    const UartParams_t *Params;
    char TXBuf[UART_TXBUF_SZ];
    char *PRead, *PWrite;
    bool IDmaIsIdle;
    uint32_t IFullSlotsCount, ITransSize;
    void ISendViaDMA();
    int32_t RIndx;
    uint8_t IRxBuf[UART_RXBUF_SZ];
    uint8_t IPutByte(uint8_t b);
    uint8_t IPutByteNow(uint8_t b);
    void IStartTransmissionIfNotYet();
    // ==== Constructor ====
    BaseUart_t(const UartParams_t &APParams) : Params(&APParams)
    , PRead(TXBuf), PWrite(TXBuf), IDmaIsIdle(true), IFullSlotsCount(0), ITransSize(0)
    , RIndx(0)
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
    void FlushTx() { while(!IDmaIsIdle) chThdSleepMilliseconds(1); }  // wait DMA
    void EnableTCIrq(const uint32_t Priority, ftVoidVoid ACallback);
    // Inner use
    void IRQDmaTxHandler();
    uint8_t GetByte(uint8_t *b);
    virtual void OnUartIrqI(uint32_t flags) = 0;
};

class CmdUart_t : public BaseUart_t, public PrintfHelper_t, public Shell_t {
private:
    uint8_t IPutChar(char c) { return IPutByte(c);  }
    void IStartTransmissionIfNotYet() { BaseUart_t::IStartTransmissionIfNotYet(); }
    void Print(const char *format, ...) {
        va_list args;
        va_start(args, format);
        IVsPrintf(format, args);
        va_end(args);
    }
public:
    CmdUart_t(const UartParams_t &APParams) : BaseUart_t(APParams) {}
    uint8_t TryParseRxBuff() {
        uint8_t b;
        while(GetByte(&b) == retvOk) {
            if(Cmd.PutChar(b) == pdrNewCmd) return retvOk;
        } // while get byte
        return retvFail;
    }
    uint8_t ReceiveBinaryToBuf(uint8_t *ptr, uint32_t Len, uint32_t Timeout_ms);
    uint8_t TransmitBinaryFromBuf(uint8_t *ptr, uint32_t Len, uint32_t Timeout_ms);
    void OnUartIrqI(uint32_t flags);
};

class CmdUart485_t : public CmdUart_t {
private:
    GPIO_TypeDef *PGpioDE;
    uint16_t PinDE;
    AlterFunc_t AltFuncDE;
public:
    void Init() {
        CmdUart_t::Init();
        PinSetupAlterFunc(PGpioDE, PinDE, omPushPull, pudNone, AltFuncDE);
        Params->Uart->CR1 &= ~USART_CR1_UE;   // Disable USART
        Params->Uart->CR3 |= USART_CR3_DEM;   // Enable DriverEnable signal
        Params->Uart->CR1 |= USART_CR1_UE;    // Enable USART
    }
    CmdUart485_t(const UartParams_t &APParams, GPIO_TypeDef *APGPIO, uint16_t APin, AlterFunc_t AAf) :
        CmdUart_t(APParams), PGpioDE(APGPIO), PinDE(APin), AltFuncDE(AAf) {}
};

class HostUart485_t : private BaseUart_t, public PrintfHelper_t {
private:
    GPIO_TypeDef *PGpioDE;
    uint16_t PinDE;
    AlterFunc_t AltFuncDE;
    thread_reference_t ThdRef = nullptr;
    uint8_t IPutChar(char c) { return IPutByte(c);  }
    void IStartTransmissionIfNotYet() { BaseUart_t::IStartTransmissionIfNotYet(); }
    void Print(const char *format, ...);
    uint8_t TryParseRxBuff();
public:
    void Init() {
        BaseUart_t::Init();
                PinSetupAlterFunc(PGpioDE, PinDE, omPushPull, pudNone, AltFuncDE);
                Params->Uart->CR1 &= ~USART_CR1_UE;   // Disable USART
                Params->Uart->CR3 |= USART_CR3_DEM;   // Enable DriverEnable signal
                Params->Uart->CR1 |= USART_CR1_UE;    // Enable USART
    }
    HostUart485_t(const UartParams_t &APParams, GPIO_TypeDef *APGPIO, uint16_t APin, AlterFunc_t AAf) :
        BaseUart_t(APParams), PGpioDE(APGPIO), PinDE(APin), AltFuncDE(AAf) {}

    uint8_t SendCmd(uint32_t Timeout_ms, const char* ACmd, uint32_t Addr, char* S = nullptr);
    Cmd_t Reply;
    void OnUartIrqI(uint32_t flags);
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
