#include "uart.h"
#include "kl_i2c.h"

#define I2C_DMATX_MODE(Chnl) \
                        STM32_DMA_CR_CHSEL(Chnl) |   \
                        DMA_PRIORITY_LOW | \
                        STM32_DMA_CR_MSIZE_BYTE | \
                        STM32_DMA_CR_PSIZE_BYTE | \
                        STM32_DMA_CR_MINC |     /* Memory pointer increase */ \
                        STM32_DMA_CR_DIR_M2P |  /* Direction is memory to peripheral */ \
                        STM32_DMA_CR_TCIE

#define I2C_DMARX_MODE(Chnl) \
                        STM32_DMA_CR_CHSEL(Chnl) |   \
                        DMA_PRIORITY_LOW | \
                        STM32_DMA_CR_MSIZE_BYTE | \
                        STM32_DMA_CR_PSIZE_BYTE | \
                        STM32_DMA_CR_MINC |         /* Memory pointer increase */ \
                        STM32_DMA_CR_DIR_P2M |      /* Direction is peripheral to memory */ \
                        STM32_DMA_CR_TCIE

#if defined STM32L1XX || defined STM32F2XX
#if defined STM32F2XX
#define I2C1_DMA_CHNL   1
#define I2C2_DMA_CHNL   7
#define I2C3_DMA_CHNL   3
#endif

#if I2C1_ENABLED
static const i2cParams_t I2C1Params = {
        I2C1,
        I2C1_GPIO, I2C1_SCL, I2C1_SDA,
        I2C1_BAUDRATE,
        I2C1_DMA_TX,
        I2C1_DMA_RX,
        I2C_DMATX_MODE(I2C1_DMA_CHNL),
        I2C_DMARX_MODE(I2C1_DMA_CHNL)
};
i2c_t i2c1 {&I2C1Params};
#endif

#if I2C2_ENABLED
static const i2cParams_t I2C2Params = {
        I2C2,
        I2C2_GPIO, I2C2_SCL, I2C2_SDA,
        I2C2_BAUDRATE,
        I2C2_DMA_TX,
        I2C2_DMA_RX,
        (I2C_DMATX_MODE(I2C2_DMA_CHNL)),
        (I2C_DMARX_MODE(I2C2_DMA_CHNL))
};
i2c_t i2c2 {&I2C2Params};
#endif

extern "C"
void i2cDmaIrqHandler(void *p, uint32_t flags) {
    chSysLockFromISR();
    i2c_t *pi2c = (i2c_t*)p;
    chThdResumeI(&pi2c->ThdRef, (msg_t)0);
    chSysUnlockFromISR();
}

void i2c_t::Init() {
    Standby();
    Resume();
#if I2C_USE_SEMAPHORE
    chBSemObjectInit(&BSemaphore, NOT_TAKEN);
#endif
    // ==== DMA ====
    // Here only unchanged parameters of the DMA are configured.
    dmaStreamAllocate(PParams->PDmaTx, IRQ_PRIO_MEDIUM, i2cDmaIrqHandler, this);
    dmaStreamSetPeripheral(PParams->PDmaTx, &PParams->pi2c->DR);
    dmaStreamAllocate(PParams->PDmaRx, IRQ_PRIO_MEDIUM, i2cDmaIrqHandler, this);
    dmaStreamSetPeripheral(PParams->PDmaRx, &PParams->pi2c->DR);
}

void i2c_t::Standby() {
    if(PParams->pi2c == I2C1) { rccResetI2C1(); rccDisableI2C1(FALSE); }
#ifdef I2C2
    else if(PParams->pi2c == I2C2) { rccResetI2C2(); rccDisableI2C2(FALSE); }
#endif
#if defined I2C3
    else if (PParams->pi2c == I2C3) { rccResetI2C3(); rccDisableI2C3(FALSE); }
#endif
    // Disable GPIOs
    PinSetupAnalog(PParams->PGpio, PParams->SclPin);
    PinSetupAnalog(PParams->PGpio, PParams->SdaPin);
}

void i2c_t::Resume() {
    Error = false;
    // ==== GPIOs ====
    AlterFunc_t PinAF;
#if defined STM32L1XX || defined STM32F2XX
    PinAF = AF4; // for all I2Cs everywhere
//#elif defined STM32F0XX
//    PinAF = AF4;
#else
#error "I2C AF not defined"
#endif

    PinSetupAlterFunc(PParams->PGpio, PParams->SclPin, omOpenDrain, pudNone, PinAF);
    PinSetupAlterFunc(PParams->PGpio, PParams->SdaPin, omOpenDrain, pudNone, PinAF);
    // ==== Clock and reset ====
    if(PParams->pi2c == I2C1) { rccEnableI2C1(FALSE); rccResetI2C1(); }
#ifdef I2C2
    else if (PParams->pi2c == I2C2) { rccEnableI2C2(FALSE); rccResetI2C2(); }
#endif
#ifdef I2C3
    else if (PParams->pi2c == I2C3) { rccEnableI2C3(FALSE); rccResetI2C3(); }
#endif

    // Minimum clock is 2 MHz
    uint32_t ClkMhz = Clk.APB1FreqHz / 1000000;
    uint16_t tmpreg = PParams->pi2c->CR2;
    tmpreg &= (uint16_t)~I2C_CR2_FREQ;
    if(ClkMhz < 2)  ClkMhz = 2;
    if(ClkMhz > 32) ClkMhz = 32;
    tmpreg |= ClkMhz;
    PParams->pi2c->CR2 = tmpreg;
    PParams->pi2c->CR1 &= (uint16_t)~I2C_CR1_PE; // Disable i2c to setup TRise & CCR
    PParams->pi2c->TRISE = (uint16_t)(((ClkMhz * 300) / 1000) + 1);
    // 16/9
    tmpreg = (uint16_t)(Clk.APB1FreqHz / (PParams->BitrateHz * 25));
    if(tmpreg == 0) tmpreg = 1; // minimum allowed value
    tmpreg |= I2C_CCR_FS | I2C_CCR_DUTY;
    PParams->pi2c->CCR = tmpreg;
    PParams->pi2c->CR1 |= I2C_CR1_PE;    // Enable i2c back
    // ==== DMA ====
    PParams->pi2c->CR2 |= I2C_CR2_DMAEN;
}

void i2c_t::IReset() {
    Standby();
    Resume();
}

uint8_t i2c_t::WriteRead(uint8_t Addr,
        uint8_t *WPtr, uint8_t WLength,
        uint8_t *RPtr, uint8_t RLength) {
#if I2C_USE_SEMAPHORE
    if(chBSemWait(&BSemaphore) != MSG_OK) return retvBusy;
#endif
    uint8_t Rslt = retvOk;
    if(IBusyWait() != retvOk) { Rslt = retvBusy; goto WriteReadEnd; }
    // Clear flags
    PParams->pi2c->SR1 = 0;
    while(RxIsNotEmpty()) (void)PParams->pi2c->DR;   // Read DR until it empty
    ClearAddrFlag();
    // Start transmission
    SendStart();
    if(WaitEv5() != retvOk) { Rslt = retvFail; goto WriteReadEnd; }
    SendAddrWithWrite(Addr);
    if(WaitEv6() != retvOk) { SendStop(); Rslt = retvFail; goto WriteReadEnd; }
    ClearAddrFlag();
    // Start TX DMA if needed
    if(WLength != 0) {
        if(WaitEv8() != retvOk) { Rslt = retvFail; goto WriteReadEnd; }
        dmaStreamSetMemory0(PParams->PDmaTx, WPtr);
        dmaStreamSetMode   (PParams->PDmaTx, PParams->DmaModeTx);
        dmaStreamSetTransactionSize(PParams->PDmaTx, WLength);
        chSysLock();
        dmaStreamEnable(PParams->PDmaTx);
        chThdSuspendS(&ThdRef);    // Wait IRQ
        chSysUnlock();
        dmaStreamDisable(PParams->PDmaTx);
    }
    // Read if needed
    if(RLength != 0) {
        if(WaitEv8() != retvOk) { Rslt = retvFail; goto WriteReadEnd; }
        // Send repeated start
        SendStart();
        if(WaitEv5() != retvOk) { Rslt = retvFail; goto WriteReadEnd; }
        SendAddrWithRead(Addr);
        if(WaitEv6() != retvOk) { SendStop(); Rslt = retvFail; goto WriteReadEnd; }
        // If single byte is to be received, disable ACK before clearing ADDR flag
        if(RLength == 1) AckDisable();
        else AckEnable();
        ClearAddrFlag();
        dmaStreamSetMemory0(PParams->PDmaRx, RPtr);
        dmaStreamSetMode   (PParams->PDmaRx, PParams->DmaModeRx);
        dmaStreamSetTransactionSize(PParams->PDmaRx, RLength);
        SignalLastDmaTransfer(); // Inform DMA that this is last transfer => do not ACK last byte
        chSysLock();
        dmaStreamEnable(PParams->PDmaRx);
        chThdSuspendS(&ThdRef);    // Wait IRQ
        chSysUnlock();
        dmaStreamDisable(PParams->PDmaRx);
    } // if != 0
    else WaitBTF(); // if nothing to read, just stop
    SendStop();
    WriteReadEnd:
#if I2C_USE_SEMAPHORE
    chBSemSignal(&BSemaphore);
#endif
    return Rslt;
}

uint8_t i2c_t::WriteWrite(uint8_t Addr,
        uint8_t *WPtr1, uint8_t WLength1,
        uint8_t *WPtr2, uint8_t WLength2) {
#if I2C_USE_SEMAPHORE
    if(chBSemWait(&BSemaphore) != MSG_OK) return retvBusy;
#endif
    uint8_t Rslt = retvOk;
    if(IBusyWait() != retvOk) { Rslt = retvBusy; goto WriteWriteEnd; }
    // Clear flags
    PParams->pi2c->SR1 = 0;
    while(RxIsNotEmpty()) (void)PParams->pi2c->DR;   // Read DR until it empty
    ClearAddrFlag();
    // Start transmission
    SendStart();
    if(WaitEv5() != retvOk) { Rslt = retvFail; goto WriteWriteEnd; }
    SendAddrWithWrite(Addr);
    if(WaitEv6() != retvOk) { SendStop(); Rslt = retvFail; goto WriteWriteEnd; }
    ClearAddrFlag();
    // Start TX DMA if needed
    if(WLength1 != 0) {
        if(WaitEv8() != retvOk) { Rslt = retvFail; goto WriteWriteEnd; }
        dmaStreamSetMemory0(PParams->PDmaTx, WPtr1);
        dmaStreamSetMode   (PParams->PDmaTx, PParams->DmaModeTx);
        dmaStreamSetTransactionSize(PParams->PDmaTx, WLength1);
        chSysLock();
        dmaStreamEnable(PParams->PDmaTx);
        chThdSuspendS(&ThdRef);    // Wait IRQ
        chSysUnlock();
        dmaStreamDisable(PParams->PDmaTx);
    }
    if(WLength2 != 0) {
        if(WaitEv8() != retvOk) { Rslt = retvFail; goto WriteWriteEnd; }
        dmaStreamSetMemory0(PParams->PDmaTx, WPtr2);
        dmaStreamSetMode   (PParams->PDmaTx, PParams->DmaModeTx);
        dmaStreamSetTransactionSize(PParams->PDmaTx, WLength2);
        chSysLock();
        dmaStreamEnable(PParams->PDmaTx);
        chThdSuspendS(&ThdRef);    // Wait IRQ
        chSysUnlock();
        dmaStreamDisable(PParams->PDmaTx);
    }
    WaitBTF();
    SendStop();
    WriteWriteEnd:
#if I2C_USE_SEMAPHORE
    chBSemSignal(&BSemaphore);
#endif
    return Rslt;
}

uint8_t i2c_t::CheckAddress(uint32_t Addr) {
#if I2C_USE_SEMAPHORE
    if(chBSemWait(&BSemaphore) != MSG_OK) return retvFail;
#endif
    uint8_t Rslt = retvFail;
    if(IBusyWait() != retvOk) {
        Rslt = retvBusy;
        Printf("i2cC Busy\r");
        goto ChckEnd;
    }
    IReset(); // Reset I2C
    // Clear flags
    PParams->pi2c->SR1 = 0;
    while(RxIsNotEmpty()) (void)PParams->pi2c->DR;   // Read DR until it empty
    ClearAddrFlag();
    // Start transmission
    SendStart();
    if(WaitEv5() == retvOk) {
        SendAddrWithWrite(Addr);
        if(WaitEv6() == retvOk) Rslt = retvOk;
        else Rslt = retvNotFound;
    }
    SendStop();
    ChckEnd:
#if I2C_USE_SEMAPHORE
    chBSemSignal(&BSemaphore);
#endif
    return Rslt;
}

uint8_t i2c_t::Write(uint8_t Addr, uint8_t *WPtr1, uint8_t WLength1) {
#if I2C_USE_SEMAPHORE
    if(chBSemWait(&BSemaphore) != MSG_OK) return retvBusy;
#endif
    uint8_t Rslt = retvOk;
    if(IBusyWait() != retvOk) { Rslt = retvBusy; goto WriteEnd; }
    // Clear flags
    PParams->pi2c->SR1 = 0;
    while(RxIsNotEmpty()) (void)PParams->pi2c->DR;   // Read DR until it empty
    ClearAddrFlag();
    // Start transmission
    SendStart();
    if(WaitEv5() != retvOk) { Rslt = retvFail; goto WriteEnd; }
    SendAddrWithWrite(Addr);
    if(WaitEv6() != retvOk) { SendStop(); Rslt = retvFail; goto WriteEnd; }
    ClearAddrFlag();
    // Start TX DMA if needed
    if(WLength1 != 0) {
        if(WaitEv8() != retvOk) { Rslt = retvFail; goto WriteEnd; }
        dmaStreamSetMemory0(PParams->PDmaTx, WPtr1);
        dmaStreamSetMode   (PParams->PDmaTx, PParams->DmaModeTx);
        dmaStreamSetTransactionSize(PParams->PDmaTx, WLength1);
        chSysLock();
        dmaStreamEnable(PParams->PDmaTx);
        chThdSuspendS(&ThdRef);    // Wait IRQ
        chSysUnlock();
        dmaStreamDisable(PParams->PDmaTx);
    }
    WaitBTF();
    SendStop();
    WriteEnd:
#if I2C_USE_SEMAPHORE
    chBSemSignal(&BSemaphore);
#endif
    return Rslt;
}

void i2c_t::ScanBus() {
    Printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f");
    uint8_t AddrHi, Addr;
    for(AddrHi = 0; AddrHi < 0x80; AddrHi += 0x10) {
        Printf("\r%02X: ", AddrHi);
        for(uint8_t n=0; n<0x10; n++) {
            Addr = AddrHi + n;
            if(Addr <= 0x01 or Addr > 0x77) Printf("   ");
            else {
                // Try to get response from addr
                if(IBusyWait() != retvOk) {
                    Printf("i2cBusyWait\r");
                    return;
                }
                // Clear flags
                PParams->pi2c->SR1 = 0;
                while(RxIsNotEmpty()) (void)PParams->pi2c->DR;   // Read DR until it empty
                ClearAddrFlag();
                // Start transmission
                SendStart();
                if(WaitEv5() != retvOk) continue;
                SendAddrWithWrite(Addr);
                if(WaitEv6() == retvOk) Printf("%02X ", Addr);
                else Printf("__ ");
                SendStop();
            }
        } // for n
    } // for AddrHi
    PrintfEOL();
}

// ==== Flag operations ====
#define RETRY_CNT_LONG      450
#define RETRY_CNT_SHORT     450
// Busy flag
uint8_t i2c_t::IBusyWait() {
    uint8_t RetryCnt = 4;
    while(RetryCnt--) {
        if(!(PParams->pi2c->SR2 & I2C_SR2_BUSY)) return retvOk;
        chThdSleepMilliseconds(1);
    }
    Error = true;
    return retvTimeout;
}

// BUSY, MSL & SB flags
uint8_t i2c_t::WaitEv5() {
    uint32_t RetryCnt = RETRY_CNT_LONG;
    while(RetryCnt--) {
        uint16_t Flag1 = PParams->pi2c->SR1;
        uint16_t Flag2 = PParams->pi2c->SR2;
        if((Flag1 & I2C_SR1_SB) and (Flag2 & (I2C_SR2_MSL | I2C_SR2_BUSY))) return retvOk;
    }
    Error = true;
    return retvFail;
}

uint8_t i2c_t::WaitEv6() {
    uint32_t RetryCnt = RETRY_CNT_SHORT;
    uint16_t Flag1;
    do {
        Flag1 = PParams->pi2c->SR1;
        if((RetryCnt-- == 0) or (Flag1 & I2C_SR1_AF)) return retvFail;   // Fail if timeout or NACK
    } while(!(Flag1 & I2C_SR1_ADDR)); // ADDR set when Address is sent and ACK received
    return retvOk;
}

uint8_t i2c_t::WaitEv8() {
    uint32_t RetryCnt = RETRY_CNT_SHORT;
    while(RetryCnt--)
        if(PParams->pi2c->SR1 & I2C_SR1_TXE) return retvOk;
    Error = true;
    return retvTimeout;
}

uint8_t i2c_t::WaitRx() {
    uint32_t RetryCnt = RETRY_CNT_LONG;
    while(RetryCnt--)
        if(PParams->pi2c->SR1 & I2C_SR1_RXNE) return retvOk;
    return retvTimeout;
}

uint8_t i2c_t::WaitStop() {
    uint32_t RetryCnt = RETRY_CNT_LONG;
    while(RetryCnt--)
        if(PParams->pi2c->CR1 & I2C_CR1_STOP) return retvOk;
    return retvTimeout;
}

uint8_t i2c_t::WaitBTF() {
    uint32_t RetryCnt = RETRY_CNT_LONG;
    while(RetryCnt--)
        if(PParams->pi2c->SR1 & I2C_SR1_BTF) return retvOk;
    return retvTimeout;
}
#endif // MCU type

#if defined STM32L4XX || defined STM32F030

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
        I2C1_DMA_TX,
        I2C1_DMA_RX,
        I2C_DMATX_MODE(I2C1_DMA_CHNL),
        I2C_DMARX_MODE(I2C1_DMA_CHNL),
#if defined STM32L4XX
        STM32_I2C1_EVENT_NUMBER,
        STM32_I2C1_ERROR_NUMBER,
        I2C_CLK_SRC
#else
        STM32_I2C1_GLOBAL_NUMBER,
        STM32_I2C1_GLOBAL_NUMBER,
#endif
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
        I2C3_DMA_TX,
        I2C3_DMA_RX,
        I2C_DMATX_MODE(I2C3_DMA_CHNL),
        I2C_DMARX_MODE(I2C3_DMA_CHNL),
#if defined STM32L4XX
        STM32_I2C3_EVENT_NUMBER,
        STM32_I2C3_ERROR_NUMBER,
        I2C_CLK_SRC
#else
        STM32_I2C1_GLOBAL_NUMBER,
        STM32_I2C1_GLOBAL_NUMBER,
#endif
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
    // Clock
#if defined STM32L4XX
    if(PParams->ClkSrc == i2cclkHSI) Clk.EnableHSI();    // HSI used as independent clock
    Clk.SetI2CClkSrc(PParams->pi2c, PParams->ClkSrc);
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
#ifdef I2C3
    else if(pi2c == I2C3) {
        rccResetI2C3();
        rccEnableI2C3(FALSE);
    }
#endif
    // ==== Setup timings ====
    // Get input clock
    uint32_t ClkHz;
    if(PParams->ClkSrc == i2cclkHSI) {
        Clk.SetI2CClkSrc(pi2c, i2cclkHSI);
        ClkHz = HSI_FREQ_HZ;
    }
#if defined STM32L4XX
    else if(PParams->ClkSrc == i2cclkPCLK1) {
        Clk.SetI2CClkSrc(pi2c, i2cclkPCLK1);
        ClkHz = Clk.APB1FreqHz;
    }
#endif
    else {
        Clk.SetI2CClkSrc(pi2c, i2cclkSYSCLK);
        ClkHz = Clk.GetSysClkHz();
    }
    // Calc prescaler
    uint32_t Prescaler = ClkHz / 16000000;
    if(Prescaler > 0) Prescaler--;
    // Calc Scl & Sda len
    uint32_t SclLen = ((ClkHz / 2) / I2C_BAUDRATE_HZ) - 1;
    if(SclLen >= 4) SclLen -= 4;
    pi2c->TIMINGR = (Prescaler << 28) | 0x00100000 | (SclLen << 8) | SclLen;

    // Analog filter enabled, digital disabled, clk stretch enabled, DMA enabled
    pi2c->CR1 = I2C_CR1_TXDMAEN | I2C_CR1_RXDMAEN;
    // ==== DMA ====
    dmaStreamAllocate(PParams->PDmaTx, IRQ_PRIO_MEDIUM, nullptr, nullptr);
    dmaStreamAllocate(PParams->PDmaRx, IRQ_PRIO_MEDIUM, nullptr, nullptr);
    dmaStreamSetPeripheral(PParams->PDmaTx, &pi2c->TXDR);
    dmaStreamSetPeripheral(PParams->PDmaRx, &pi2c->RXDR);
    // ==== IRQ ====
    nvicEnableVector(PParams->IrqEvtNumber, IRQ_PRIO_MEDIUM);
    nvicEnableVector(PParams->IrqErrorNumber, IRQ_PRIO_MEDIUM);
}

void i2c_t::ScanBus() {
#if I2C_USE_SEMAPHORE
    if(chBSemWait(&BSemaphore) != MSG_OK) return;
#endif
    Printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f");
    uint32_t AddrHi, Addr;
    I2C_TypeDef *pi2c = PParams->pi2c;  // To make things shorter
    for(AddrHi = 0; AddrHi < 0x80; AddrHi += 0x10) {
        Printf("\r%02X: ", AddrHi);
        for(uint32_t n=0; n < 0x10; n++) {
            Addr = AddrHi + n;
            if(Addr <= 0x01 or Addr > 0x77) Printf("   ");
            else {
                IReset(); // Reset I2C
                // Set addr and autoend; NBYTES = 0
                pi2c->CR2 = (Addr << 1) | I2C_CR2_AUTOEND;
                pi2c->CR2 |= I2C_CR2_START;     // Start
                while(!(pi2c->ISR & I2C_ISR_STOPF));
                if(pi2c->ISR & I2C_ISR_NACKF) Printf("__ ");
                else Printf("%02X ", Addr);
            }
        } // for lo
    } // for hi
    // Disable I2C
    pi2c->CR1 &= ~I2C_CR1_PE;
    Printf("\r");
#if I2C_USE_SEMAPHORE
    chBSemSignal(&BSemaphore);
#endif
}

uint8_t i2c_t::CheckAddress(uint32_t Addr) {
#if I2C_USE_SEMAPHORE
    if(chBSemWait(&BSemaphore) != MSG_OK) return FAILURE;
#endif
    uint8_t Rslt;
    I2C_TypeDef *pi2c = PParams->pi2c;  // To make things shorter
    if(IBusyWait() != retvOk) {
        Rslt = retvBusy;
        Printf("i2cC Busy\r");
        goto ChckEnd;
    }
    IReset(); // Reset I2C
    pi2c->CR2 = (Addr << 1) | I2C_CR2_AUTOEND;
    pi2c->CR2 |= I2C_CR2_START;     // Start
    while(!(pi2c->ISR & I2C_ISR_STOPF));
    if(pi2c->ISR & I2C_ISR_NACKF) Rslt = retvNotFound;
    else Rslt = retvOk;

    ChckEnd:
#if I2C_USE_SEMAPHORE
    chBSemSignal(&BSemaphore);
#endif
    return Rslt;
}

uint8_t i2c_t::Write(uint32_t Addr, uint8_t *WPtr, uint32_t WLength) {
#if I2C_USE_SEMAPHORE
    if(chBSemWait(&BSemaphore) != MSG_OK) return FAILURE;
#endif
    uint8_t Rslt;
    msg_t r;
    I2C_TypeDef *pi2c = PParams->pi2c;  // To make things shorter
    if(WLength == 0 or WPtr == nullptr) { Rslt = retvCmdError; goto WriteEnd; }
    if(IBusyWait() != retvOk) {
        Rslt = retvBusy;
        Printf("i2cW Busy\r");
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
        Rslt = retvTimeout;
    }
    else Rslt = (IState == istFailure)? retvFail : retvOk;
    WriteEnd:
#if I2C_USE_SEMAPHORE
    chBSemSignal(&BSemaphore);
#endif
    return Rslt;
}

uint8_t i2c_t::WriteRead(uint32_t Addr, uint8_t *WPtr, uint32_t WLength, uint8_t *RPtr, uint32_t RLength) {
#if I2C_USE_SEMAPHORE
    if(chBSemWait(&BSemaphore) != MSG_OK) return FAILURE;
#endif
    uint8_t Rslt;
    msg_t r;
    I2C_TypeDef *pi2c = PParams->pi2c;  // To make things shorter
    if(WLength == 0 or WPtr == nullptr) { Rslt = retvCmdError; goto WriteReadEnd; }
    if(IBusyWait() != retvOk) {
        Rslt = retvBusy;
        Printf("i2cWR Busy\r");
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
        Rslt = retvTimeout;
    }
    else Rslt = (IState == istFailure)? retvFail : retvOk;
    WriteReadEnd:
#if I2C_USE_SEMAPHORE
    chBSemSignal(&BSemaphore);
#endif
    return Rslt;
}

uint8_t i2c_t::WriteWrite(uint32_t Addr, uint8_t *WPtr1, uint32_t WLength1, uint8_t *WPtr2, uint32_t WLength2) {
#if I2C_USE_SEMAPHORE
    if(chBSemWait(&BSemaphore) != MSG_OK) return FAILURE;
#endif
    uint8_t Rslt;
    msg_t r;
    I2C_TypeDef *pi2c = PParams->pi2c;  // To make things shorter
    if(WLength1 == 0 or WPtr1 == nullptr) { Rslt = retvCmdError; goto WriteWriteEnd; }
    if(IBusyWait() != retvOk) { Rslt = retvBusy; goto WriteWriteEnd; }
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
        Rslt = retvTimeout;
    }
    else Rslt = (IState == istFailure)? retvFail : retvOk;
    WriteWriteEnd:
#if I2C_USE_SEMAPHORE
    chBSemSignal(&BSemaphore);
#endif
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
        if(!(PParams->pi2c->ISR & I2C_ISR_BUSY)) return retvOk;
        chThdSleepMilliseconds(1);
    }
    return retvTimeout;
}


void i2c_t::IServeIRQ(uint32_t isr) {
//    PrintfI("isr: %X\r", isr);
    I2C_TypeDef *pi2c = PParams->pi2c;  // To make things shorter
#if 1 // ==== NACK ====
    if((isr & I2C_ISR_NACKF) != 0) {
        PrintfI("i2c 0x%X NACK\r", (pi2c->CR2 >> 1) & 0xFF);
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
        PrintfI("i2c err: %X\r", Errors);
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
#if defined STM32L4XX
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
#else
OSAL_IRQ_HANDLER(STM32_I2C1_GLOBAL_HANDLER) {
//    Uart.PrintfI("i2c1 irq\r");
    uint32_t isr = I2C1->ISR;
    uint32_t isrEvt = isr & I2C_INT_MASK;
    uint32_t isrErr = isr & I2C_ERROR_MASK;
    OSAL_IRQ_PROLOGUE();
    I2C1->ICR = isr; // Clear IRQ bits
    if(isrEvt != 0) i2c1.IServeIRQ(isrEvt);
    if(isrErr != 0) i2c1.IServeErrIRQ(isrErr);
    OSAL_IRQ_EPILOGUE();
}
#endif // MCU type
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

#endif // L476
