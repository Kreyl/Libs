/*
 * i2cL476.cpp
 *
 *  Created on: 2 мая 2016 г.
 *      Author: Kreyl
 */

#include "i2cL476.h"
#include "uart.h"

#if 1 // ==== Inner defines ====
#define I2C_INT_MASK    ((uint32_t)(I2C_ISR_TCR | I2C_ISR_TC | I2C_ISR_STOPF | I2C_ISR_NACKF | I2C_ISR_ADDR | I2C_ISR_RXNE | I2C_ISR_TXIS))
#define I2C_ERROR_MASK  ((uint32_t)(I2C_ISR_BERR | I2C_ISR_ARLO | I2C_ISR_OVR | I2C_ISR_PECERR | I2C_ISR_TIMEOUT | I2C_ISR_ALERT))

#define I2C_NO_ERROR               0x00    // No error
#define I2C_BUS_ERROR              0x01    // Bus Error
#define I2C_ARBITRATION_LOST       0x02    // Arbitration Lost
#define I2C_ACK_FAILURE            0x04    // Acknowledge Failure
#define I2C_OVERRUN                0x08    // Overrun/Underrun
#define I2C_PEC_ERROR              0x10    // PEC Error in reception
#define I2C_TIMEOUT                0x20    // Hardware timeout
#define I2C_SMB_ALERT              0x40    // SMBus Alert
#endif

#if I2C1_ENABLED
static const i2cParams_t I2C1Params = {
        I2C1,
        I2C1_GPIO, I2C1_SCL, I2C1_SDA, I2C_AF,
        0xE14,                          // Calculated by Cube for 100kHz
        I2C1_DMA_TX,
        I2C1_DMA_RX,
        (STM32_DMA_CR_PSIZE_BYTE | STM32_DMA_CR_MSIZE_BYTE | STM32_DMA_CR_MINC | STM32_DMA_CR_DIR_M2P | STM32_DMA_CR_CHSEL(I2C1_DMA_CHNL) | DMA_PRIORITY_MEDIUM),
        (STM32_DMA_CR_PSIZE_BYTE | STM32_DMA_CR_MSIZE_BYTE | STM32_DMA_CR_MINC | STM32_DMA_CR_DIR_P2M | STM32_DMA_CR_CHSEL(I2C1_DMA_CHNL) | DMA_PRIORITY_MEDIUM),
        STM32_I2C1_EVENT_NUMBER,
        STM32_I2C1_ERROR_NUMBER
};
i2c_t i2c1 {&I2C1Params};
#endif

#if I2C2_ENABLED
static const i2cParams_t I2C2Params = {
        I2C2,
        I2C2_GPIO, I2C2_SCL, I2C2_SDA, I2C_AF,
        0xE14,                          // Calculated by Cube for 100kHz
        I2C2_DMA_TX,
        I2C2_DMA_RX,
        (STM32_DMA_CR_PSIZE_BYTE | STM32_DMA_CR_MSIZE_BYTE | STM32_DMA_CR_MINC | STM32_DMA_CR_DIR_M2P | STM32_DMA_CR_CHSEL(I2C2_DMA_CHNL) | DMA_PRIORITY_MEDIUM),
        (STM32_DMA_CR_PSIZE_BYTE | STM32_DMA_CR_MSIZE_BYTE | STM32_DMA_CR_MINC | STM32_DMA_CR_DIR_P2M | STM32_DMA_CR_CHSEL(I2C2_DMA_CHNL) | DMA_PRIORITY_MEDIUM),
        STM32_I2C2_EVENT_NUMBER,
        STM32_I2C2_ERROR_NUMBER
};
i2c_t i2c2 {&I2C2Params};
#endif

#if I2C3_ENABLED
static const i2cParams_t I2C3Params = {
        I2C3,
        I2C3_GPIO, I2C3_SCL, I2C3_SDA, I2C_AF,
        0xE14,                          // Calculated by Cube for 100kHz
        I2C3_DMA_TX,
        I2C3_DMA_RX,
        (STM32_DMA_CR_PSIZE_BYTE | STM32_DMA_CR_MSIZE_BYTE | STM32_DMA_CR_MINC | STM32_DMA_CR_DIR_M2P | STM32_DMA_CR_CHSEL(I2C3_DMA_CHNL) | DMA_PRIORITY_MEDIUM),
        (STM32_DMA_CR_PSIZE_BYTE | STM32_DMA_CR_MSIZE_BYTE | STM32_DMA_CR_MINC | STM32_DMA_CR_DIR_P2M | STM32_DMA_CR_CHSEL(I2C3_DMA_CHNL) | DMA_PRIORITY_MEDIUM),
        STM32_I2C3_EVENT_NUMBER,
        STM32_I2C3_ERROR_NUMBER
};
i2c_t i2c3 {&I2C3Params};
#endif

void i2c_t::Init() {
    // GPIO
    PinSetupAlterFunc(PParams->PGpio, PParams->SclPin, omOpenDrain, pudNone, PParams->PinAF);
    PinSetupAlterFunc(PParams->PGpio, PParams->SdaPin, omOpenDrain, pudNone, PParams->PinAF);
#if I2C_USE_SEMAPHORE
    chBSemObjectInit(&BSemaphore, NOT_TAKEN);
#endif
    // I2C
    I2C_TypeDef *pi2c = PParams->pi2c;  // To make things shorter
    pi2c->CR1 = 0;  // Clear PE bit => disable and reset i2c
    if(pi2c == I2C1) {
        rccResetI2C1();
        rccEnableI2C1(FALSE);
    }
    else if(pi2c == I2C2) {
        rccResetI2C2();
        rccEnableI2C2(FALSE);
    }
    else if(pi2c == I2C3) {
        rccResetI2C3();
        rccEnableI2C3(FALSE);
    }
    pi2c->TIMINGR = PParams->Timing;    // setup timings
    // Analog filter enabled, digital disabled, clk stretch enabled, DMA enabled
    pi2c->CR1 = I2C_CR1_TXDMAEN | I2C_CR1_RXDMAEN;
    // DMA
    dmaStreamAllocate(PParams->PDmaTx, IRQ_PRIO_MEDIUM, nullptr, nullptr);
    dmaStreamAllocate(PParams->PDmaRx, IRQ_PRIO_MEDIUM, nullptr, nullptr);
    dmaStreamSetPeripheral(PParams->PDmaTx, &pi2c->TXDR);
    dmaStreamSetPeripheral(PParams->PDmaRx, &pi2c->RXDR);
    // IRQ
    nvicEnableVector(PParams->IrqEvtNumber, IRQ_PRIO_MEDIUM);
    nvicEnableVector(PParams->IrqErrorNumber, IRQ_PRIO_MEDIUM);
}

void i2c_t::ScanBus() {
    if(chBSemWait(&BSemaphore) != MSG_OK) return;
    Uart.Printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f");
    uint32_t AddrHi, Addr;
    I2C_TypeDef *pi2c = PParams->pi2c;  // To make things shorter
    for(AddrHi = 0; AddrHi < 0x80; AddrHi += 0x10) {
        Uart.Printf("\r%02X: ", AddrHi);
        for(uint32_t n=0; n < 0x10; n++) {
            Addr = AddrHi + n;
            if(Addr <= 0x01 or Addr > 0x77) Uart.Printf("   ");
            else {
                IReset(); // Reset I2C
                // Set addr and autoend; NBYTES = 0
                pi2c->CR2 = (Addr << 1) | I2C_CR2_AUTOEND;
                pi2c->CR2 |= I2C_CR2_START;     // Start
                while(!(pi2c->ISR & I2C_ISR_STOPF));
                if(pi2c->ISR & I2C_ISR_NACKF) Uart.Printf("__ ");
                else Uart.Printf("%02X ", Addr);
            }
        } // for lo
    } // for hi
    // Disable I2C
    pi2c->CR1 &= ~I2C_CR1_PE;
    Uart.Printf("\r");
    chBSemSignal(&BSemaphore);
}

uint8_t i2c_t::CheckAddress(uint32_t Addr) {
    if(chBSemWait(&BSemaphore) != MSG_OK) return FAILURE;
    uint8_t Rslt;
    I2C_TypeDef *pi2c = PParams->pi2c;  // To make things shorter
    if(IBusyWait() != OK) {
        Rslt = BUSY;
        Uart.Printf("i2cC Busy\r");
        goto ChckEnd;
    }
    IReset(); // Reset I2C
    pi2c->CR2 = (Addr << 1) | I2C_CR2_AUTOEND;
    pi2c->CR2 |= I2C_CR2_START;     // Start
    while(!(pi2c->ISR & I2C_ISR_STOPF));
    if(pi2c->ISR & I2C_ISR_NACKF) Rslt = NOT_FOUND;
    else Rslt = OK;

    ChckEnd:
    chBSemSignal(&BSemaphore);
    return Rslt;
}

uint8_t i2c_t::Write(uint32_t Addr, uint8_t *WPtr, uint32_t WLength) {
    if(chBSemWait(&BSemaphore) != MSG_OK) return FAILURE;
    uint8_t Rslt;
    msg_t r;
    I2C_TypeDef *pi2c = PParams->pi2c;  // To make things shorter
    if(WLength == 0 or WPtr == nullptr) { Rslt = CMD_ERROR; goto WriteEnd; }
    if(IBusyWait() != OK) {
        Rslt = BUSY;
        Uart.Printf("i2cW Busy\r");
        goto WriteEnd;
    }
    IReset(); // Reset I2C
    // Prepare TX DMA
    dmaStreamSetMode(PParams->PDmaTx, PParams->DmaModeTx);
    dmaStreamSetMemory0(PParams->PDmaTx, WPtr);
    dmaStreamSetTransactionSize(PParams->PDmaTx, WLength);
    // Prepare tx
    IState = istWrite;  // Nothing to read
    pi2c->CR2 = (Addr << 1) | (WLength << 16);
    dmaStreamEnable(PParams->PDmaTx);   // Enable TX DMA
    // Enable IRQs: TX completed, error, NAck
    pi2c->CR1 |= (I2C_CR1_TCIE | I2C_CR1_ERRIE | I2C_CR1_NACKIE);
    pi2c->CR2 |= I2C_CR2_START;         // Start transmission
    // Wait completion
    chSysLock();
    r = chThdSuspendTimeoutS(&PThd, MS2ST(I2C_TIMEOUT_MS));
    chSysUnlock();
    // Disable IRQs
    pi2c->CR1 &= ~(I2C_CR1_TCIE | I2C_CR1_ERRIE | I2C_CR1_NACKIE);
    if(r == MSG_TIMEOUT) {
        pi2c->CR2 |= I2C_CR2_STOP;
        Rslt = TIMEOUT;
    }
    else Rslt = (IState == istFailure)? FAILURE : OK;
    WriteEnd:
    chBSemSignal(&BSemaphore);
    return Rslt;
}

uint8_t i2c_t::WriteRead(uint32_t Addr, uint8_t *WPtr, uint32_t WLength, uint8_t *RPtr, uint32_t RLength) {
    if(chBSemWait(&BSemaphore) != MSG_OK) return FAILURE;
    uint8_t Rslt;
    msg_t r;
    I2C_TypeDef *pi2c = PParams->pi2c;  // To make things shorter
    if(WLength == 0 or WPtr == nullptr) { Rslt = CMD_ERROR; goto WriteReadEnd; }
    if(IBusyWait() != OK) {
        Rslt = BUSY;
        Uart.Printf("i2cWR Busy\r");
        goto WriteReadEnd;
    }
    IReset(); // Reset I2C
    // Prepare TX DMA
    dmaStreamSetMode(PParams->PDmaTx, PParams->DmaModeTx);
    dmaStreamSetMemory0(PParams->PDmaTx, WPtr);
    dmaStreamSetTransactionSize(PParams->PDmaTx, WLength);
    if(RLength != 0 and RPtr != nullptr) {
        // Prepare RX DMA
        dmaStreamSetMode(PParams->PDmaRx, PParams->DmaModeRx);
        dmaStreamSetMemory0(PParams->PDmaRx, RPtr);
        dmaStreamSetTransactionSize(PParams->PDmaRx, RLength);
        ILen = RLength;
        IState = istWriteRead;
    }
    else IState = istWrite;  // Nothing to read

    pi2c->CR2 = (Addr << 1) | (WLength << 16);
    dmaStreamEnable(PParams->PDmaTx);   // Enable TX DMA
    // Enable IRQs: TX completed, error, NAck
    pi2c->CR1 |= (I2C_CR1_TCIE | I2C_CR1_ERRIE | I2C_CR1_NACKIE);
    pi2c->CR2 |= I2C_CR2_START;         // Start transmission
    // Wait completion
    chSysLock();
    r = chThdSuspendTimeoutS(&PThd, MS2ST(I2C_TIMEOUT_MS));
    chSysUnlock();
    // Disable IRQs
    pi2c->CR1 &= ~(I2C_CR1_TCIE | I2C_CR1_ERRIE | I2C_CR1_NACKIE);
    if(r == MSG_TIMEOUT) {
        pi2c->CR2 |= I2C_CR2_STOP;
        Rslt = TIMEOUT;
    }
    else Rslt = (IState == istFailure)? FAILURE : OK;
    WriteReadEnd:
    chBSemSignal(&BSemaphore);
    return Rslt;
}

uint8_t i2c_t::WriteWrite(uint32_t Addr, uint8_t *WPtr1, uint32_t WLength1, uint8_t *WPtr2, uint32_t WLength2) {
    if(chBSemWait(&BSemaphore) != MSG_OK) return FAILURE;
    uint8_t Rslt;
    msg_t r;
    I2C_TypeDef *pi2c = PParams->pi2c;  // To make things shorter
    if(WLength1 == 0 or WPtr1 == nullptr) { Rslt = CMD_ERROR; goto WriteWriteEnd; }
    if(IBusyWait() != OK) { Rslt = BUSY; goto WriteWriteEnd; }
    IReset(); // Reset I2C
    // Prepare TX DMA
    dmaStreamSetMode(PParams->PDmaTx, PParams->DmaModeTx);
    dmaStreamSetMemory0(PParams->PDmaTx, WPtr1);
    dmaStreamSetTransactionSize(PParams->PDmaTx, WLength1);
    // Prepare transmission
    if(WLength2 != 0 and WPtr2 != nullptr) {
        IState = istWriteWrite;
        IPtr = WPtr2;
        ILen = WLength2;
        pi2c->CR2 = (Addr << 1) | (WLength1 << 16) | I2C_CR2_RELOAD;
    }
    else { // No second write
        IState = istWrite;
        pi2c->CR2 = (Addr << 1) | (WLength1 << 16);
    }
    dmaStreamEnable(PParams->PDmaTx);   // Enable TX DMA
    // Enable IRQs: TX completed, error, NAck
    pi2c->CR1 |= (I2C_CR1_TCIE | I2C_CR1_ERRIE | I2C_CR1_NACKIE);
    pi2c->CR2 |= I2C_CR2_START;         // Start transmission
    // Wait completion
    chSysLock();
    r = chThdSuspendTimeoutS(&PThd, MS2ST(I2C_TIMEOUT_MS));
    chSysUnlock();
    // Disable IRQs
    pi2c->CR1 &= ~(I2C_CR1_TCIE | I2C_CR1_ERRIE | I2C_CR1_NACKIE);
    if(r == MSG_TIMEOUT) {
        pi2c->CR2 |= I2C_CR2_STOP;
        Rslt = TIMEOUT;
    }
    else Rslt = (IState == istFailure)? FAILURE : OK;
    WriteWriteEnd:
    chBSemSignal(&BSemaphore);
    return Rslt;
}

void i2c_t::IReset() {
    PParams->pi2c->CR1 &= ~I2C_CR1_PE;
    __NOP(); __NOP(); __NOP();  // Wait 3 cycles
    PParams->pi2c->CR1 |= I2C_CR1_PE;
}

void i2c_t::Standby() {
    PParams->pi2c->CR1 &= ~I2C_CR1_PE;
    PinSetupAnalog(PParams->PGpio, PParams->SclPin);
    PinSetupAnalog(PParams->PGpio, PParams->SdaPin);
    __NOP(); __NOP(); __NOP();  // Wait 3 cycles
}

void i2c_t::Resume() {
    PParams->pi2c->CR1 |= I2C_CR1_PE;
    PinSetupAlterFunc(PParams->PGpio, PParams->SclPin, omOpenDrain, pudNone, PParams->PinAF);
    PinSetupAlterFunc(PParams->PGpio, PParams->SdaPin, omOpenDrain, pudNone, PParams->PinAF);
}

uint8_t i2c_t::IBusyWait() {
    uint8_t RetryCnt = 4;
    while(RetryCnt--) {
        if(!(PParams->pi2c->ISR & I2C_ISR_BUSY)) return OK;
        chThdSleepMilliseconds(1);
    }
    return TIMEOUT;
}


void i2c_t::IServeIRQ(uint32_t isr) {
//    Uart.PrintfI("isr: %X\r", isr);
    I2C_TypeDef *pi2c = PParams->pi2c;  // To make things shorter
#if 1 // ==== NACK ====
    if((isr & I2C_ISR_NACKF) != 0) {
        // Stop DMA
        dmaStreamDisable(PParams->PDmaTx);
        dmaStreamDisable(PParams->PDmaRx);
        // Stop transaction
        pi2c->CR2 |= I2C_CR2_STOP;
        // Disable IRQs
        pi2c->CR1 &= ~(I2C_CR1_TCIE | I2C_CR1_TXIE | I2C_CR1_RXIE);
        IState = istFailure;
        IWakeup();
        return;
    }
#endif
#if 1 // ==== TX partly completed ====
    if((isr & I2C_ISR_TCR) != 0) {
        dmaStreamDisable(PParams->PDmaTx);
        if(IState == istWriteWrite) {
            // Send next ILen bytes
            pi2c->CR2 = (pi2c->CR2 & ~(I2C_CR2_NBYTES | I2C_CR2_RELOAD)) | (ILen << 16);
            // Prepare and enable TX DMA for second write
            dmaStreamSetMode(PParams->PDmaTx, PParams->DmaModeTx);
            dmaStreamSetMemory0(PParams->PDmaTx, IPtr);
            dmaStreamSetTransactionSize(PParams->PDmaTx, ILen);
            dmaStreamEnable(PParams->PDmaTx);
            IState = istWrite;
        }
    }
#endif
#if 1 // ==== TX completed ====
    if((isr & I2C_ISR_TC) != 0) {
        dmaStreamDisable(PParams->PDmaTx);  // }
        dmaStreamDisable(PParams->PDmaRx);  // } Both sorts of transaction may be completed
        if(IState == istWriteRead) {  // Write phase completed
            // Receive ILen bytes
            pi2c->CR2 = (pi2c->CR2 & ~I2C_CR2_NBYTES) | I2C_CR2_RD_WRN | (ILen << 16);
            dmaStreamEnable(PParams->PDmaRx);
            pi2c->CR2 |= I2C_CR2_START; // Send repeated start
            IState = istRead;
        } // if WriteRead
        else { // istWrite, istRead
            IState = istIdle;
            pi2c->CR2 |= I2C_CR2_STOP;
            pi2c->CR1 &= ~I2C_CR1_TCIE; // Disable TransferComplete IRQ
            IWakeup();
        }
    }
#endif
}

void i2c_t::IServeErrIRQ(uint32_t isr) {
//    Uart.PrintfI("isre: %X\r", isr);
    // Stop DMA
    dmaStreamDisable(PParams->PDmaTx);
    dmaStreamDisable(PParams->PDmaRx);
    // Check errors
    uint32_t Errors = 0;
    if(isr & I2C_ISR_BERR) Errors |= I2C_BUS_ERROR;
    if(isr & I2C_ISR_ARLO) Errors |= I2C_ARBITRATION_LOST;
    if(isr & I2C_ISR_OVR)  Errors |= I2C_OVERRUN;
    if(isr & I2C_ISR_TIMEOUT) Errors |= I2C_TIMEOUT;
    // If some error has been identified then wake the waiting thread
    if(Errors != I2C_NO_ERROR) {
        Uart.PrintfI("i2c err: %X\r", Errors);
        IWakeup();
    }
}

void i2c_t::IWakeup() {
    chSysLockFromISR();
    chThdResumeI(&PThd, MSG_OK);
    chSysUnlockFromISR();
}

#if 1 // =============================== IRQs ==================================
extern "C" {
#if I2C1_ENABLED // ==== I2C1 ====
OSAL_IRQ_HANDLER(STM32_I2C1_EVENT_HANDLER) {
//    Uart.PrintfI("i2c1 irq\r");
    uint32_t isr = I2C1->ISR;
    OSAL_IRQ_PROLOGUE();
    I2C1->ICR = isr & I2C_INT_MASK; // Clear IRQ bits
    i2c1.IServeIRQ(isr);
    OSAL_IRQ_EPILOGUE();
}

OSAL_IRQ_HANDLER(STM32_I2C1_ERROR_HANDLER) {
    uint32_t isr = I2C1->ISR;
    OSAL_IRQ_PROLOGUE();
    I2C1->ICR = isr & I2C_ERROR_MASK; // Clear IRQ bits
    i2c1.IServeErrIRQ(isr);
    OSAL_IRQ_EPILOGUE();
}
#endif
#if I2C2_ENABLED // ==== I2C2 ====
OSAL_IRQ_HANDLER(STM32_I2C2_EVENT_HANDLER) {
    uint32_t isr = I2C2->ISR;
    OSAL_IRQ_PROLOGUE();
    I2C2->ICR = isr & I2C_INT_MASK; // Clear IRQ bits
    i2c2.IServeIRQ(isr);
    OSAL_IRQ_EPILOGUE();
}

OSAL_IRQ_HANDLER(STM32_I2C2_ERROR_HANDLER) {
    uint32_t isr = I2C2->ISR;
    OSAL_IRQ_PROLOGUE();
    I2C2->ICR = isr & I2C_ERROR_MASK; // Clear IRQ bits
    i2c2.IServeErrIRQ(isr);
    OSAL_IRQ_EPILOGUE();
}
#endif
#if I2C3_ENABLED// ==== I2C3 ====
OSAL_IRQ_HANDLER(STM32_I2C3_EVENT_HANDLER) {
    uint32_t isr = I2C3->ISR;
    OSAL_IRQ_PROLOGUE();
    I2C3->ICR = isr & I2C_INT_MASK; // Clear IRQ bits
    i2c3.IServeIRQ(isr);
    OSAL_IRQ_EPILOGUE();
}

OSAL_IRQ_HANDLER(STM32_I2C3_ERROR_HANDLER) {
    uint32_t isr = I2C3->ISR;
    OSAL_IRQ_PROLOGUE();
    I2C3->ICR = isr & I2C_ERROR_MASK; // Clear IRQ bits
    i2c3.IServeErrIRQ(isr);
    OSAL_IRQ_EPILOGUE();
}
#endif
} // extern C
#endif
