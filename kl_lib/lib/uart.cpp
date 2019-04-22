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
// Pins Alternate function
#if defined STM32L4XX || defined STM32F0XX
#define UART_TX_REG     TDR
#define UART_RX_REG     RDR
#elif defined STM32L1XX || defined STM32F2XX || defined STM32F1XX
#define UART_TX_REG     DR
#define UART_RX_REG     DR
#else
#error "Not defined"
#endif

// Array of utilized UARTs to organize RX
static BaseUart_t* PUarts[UARTS_CNT];
#endif // Common and eternal

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
    dmaStreamDisable(PDmaTx);    // Registers may be changed only when stream is disabled
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

#if 1 // ==== RX ====
static thread_reference_t RXThread = nullptr;
static THD_WORKING_AREA(waUartRxThread, 128);

__noreturn
static void UartRxThread(void *arg) {
    chRegSetThreadName("UartRx");
    while(true) {
        chThdSleepMilliseconds(UART_RX_POLLING_MS);
        // Iterate UARTs
        for(BaseUart_t* ptr : PUarts) {
            if(ptr != nullptr) ptr->ProcessByteIfReceived();
        } // for
    } // while true
}

uint8_t BaseUart_t::GetByte(uint8_t *b) {
#if defined STM32F2XX || defined STM32F4XX
    int32_t WIndx = UART_RXBUF_SZ - Params->PDmaRx->stream->NDTR;
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
#endif // RX

#if 1 // ==== Init ====
void BaseUart_t::Init() {
    AlterFunc_t PinAF = AF1;
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
#elif defined STM32F1XX
    // Do nothing as F1xx does not use AF number
#else
#error "UART AF not defined"
#endif
    PinSetupAlterFunc(Params->PGpioTx, Params->PinTx, omPushPull, pudNone, PinAF);
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
    if(Params->UseIndependedClock) {
        Clk.EnableHSI();    // HSI used as independent clock
        if     (Params->Uart == USART1) RCC->CFGR3 |= RCC_CFGR3_USART1SW_HSI;
        else if(Params->Uart == USART2) RCC->CFGR3 |= RCC_CFGR3_USART2SW_HSI;
    }
#elif defined STM32L4XX
    if(Params->UseIndependedClock) {
        Clk.EnableHSI();    // HSI used as independent clock
        if     (Params->Uart == USART1) RCC->CCIPR |= 0b10;
        else if(Params->Uart == USART2) RCC->CCIPR |= 0b10 << 2;
        else if(Params->Uart == USART3) RCC->CCIPR |= 0b10 << 4;
#ifdef UART4
        else if(Params->Uart == UART4)  RCC->CCIPR |= 0b10 << 6;
#endif
#ifdef UART5
        else if(Params->Uart == UART5)  RCC->CCIPR |= 0b10 << 8;
#endif
    }
#endif
    OnClkChange();  // Setup baudrate

    Params->Uart->CR2 = 0;  // Nothing that interesting there
#if UART_USE_DMA    // ==== DMA ====
    // Remap DMA request if needed
#if defined STM32F0XX
    if(Params->PDmaTx == STM32_DMA1_STREAM4) SYSCFG->CFGR1 |= SYSCFG_CFGR1_USART1TX_DMA_RMP;
#endif
    PDmaTx = dmaStreamAlloc(Params->DmaTxID, IRQ_PRIO_MEDIUM, DmaUartTxIrq, this);
    dmaStreamSetPeripheral(PDmaTx, &Params->Uart->UART_TX_REG);
    dmaStreamSetMode      (PDmaTx, Params->DmaModeTx);
    IDmaIsIdle = true;
#endif

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
#elif defined STM32F1XX
    // Do nothing as F1xx does not use AF number
#else
#error "UART AF not defined"
#endif
#ifdef STM32F1XX // Setup pin as input
    PinSetupInput(Params->PGpioRx, Params->PinRx, pudPullUp);
#else
    PinSetupAlterFunc(Params->PGpioRx, Params->PinRx, omOpenDrain, pudPullUp, PinAF);
#endif
    // Remap DMA request if needed
#if defined STM32F0XX
    if(Params->PDmaRx == STM32_DMA1_STREAM5) SYSCFG->CFGR1 |= SYSCFG_CFGR1_USART1RX_DMA_RMP;
#endif
    // DMA
    PDmaRx = dmaStreamAlloc(Params->DmaRxID, IRQ_PRIO_MEDIUM, nullptr, NULL);
    dmaStreamSetPeripheral(PDmaRx, &Params->Uart->UART_RX_REG);
    dmaStreamSetMemory0   (PDmaRx, IRxBuf);
    dmaStreamSetTransactionSize(PDmaRx, UART_RXBUF_SZ);
    dmaStreamSetMode      (PDmaRx, Params->DmaModeRx);
    dmaStreamEnable       (PDmaRx);
    Params->Uart->CR1 |= USART_CR1_UE;    // Enable USART

    // Prepare and start RX
    for(int i=0; i<UARTS_CNT; i++) {
        if(PUarts[i] == nullptr) {
            PUarts[i] = this;
            break;
        }
    }
    if(RXThread == nullptr) {
        RXThread = chThdCreateStatic(waUartRxThread, sizeof(waUartRxThread), NORMALPRIO, (tfunc_t)UartRxThread, NULL);
    }
}

void BaseUart_t::Shutdown() {
    Params->Uart->CR1 &= ~USART_CR1_UE; // UART Disable
    if     (Params->Uart == USART1) { rccDisableUSART1(); }
    else if(Params->Uart == USART2) { rccDisableUSART2(); }
#if defined USART3
    else if(Params->Uart == USART3) { rccDisableUSART3(); }
#endif
#if defined UART4
    else if(Params->Uart == UART4) { rccDisableUART4(FALSE); }
#endif
#if defined UART5
    else if(Params->Uart == UART5) { rccDisableUART5(FALSE); }
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
    if(Params->Uart == USART1 or Params->Uart == USART6) Params->Uart->BRR = Clk.APB2FreqHz / IBaudrate;
    else Params->Uart->BRR = Clk.APB1FreqHz / IBaudrate;
#elif defined STM32L4XX
    if(Params->UseIndependedClock) Params->Uart->BRR = HSI_FREQ_HZ / Params->Baudrate;
    else {
        if(Params->Uart == USART1) Params->Uart->BRR = Clk.APB2FreqHz / Params->Baudrate;
        else Params->Uart->BRR = Clk.APB1FreqHz / Params->Baudrate; // All others at APB1
    }
#endif
}
#endif // Init

void BaseUart_t::SignalRxProcessed() {
    chSysLock();
    RxProcessed = true;
    chSysUnlock();
}

#endif // Base UART

#if 1 // ========================= Cmd UART ====================================
void CmdUart_t::ProcessByteIfReceived() {
    if(!RxProcessed) return;
    uint8_t b;
    while(GetByte(&b) == retvOk) {
        if(Cmd.PutChar(b) == pdrNewCmd) {
            RxProcessed = false;
            EvtQMain.SendNowOrExit(EvtMsg_t(evtIdShellCmd, (Shell_t*)this));
        } // if new cmd
    } // while get byte
//    PrintfI("e\r");
}
#endif

#if 1 // ==== Modbus ====
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
