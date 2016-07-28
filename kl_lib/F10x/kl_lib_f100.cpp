/*
 * kl_lib_f0.cpp
 *
 *  Created on: 10.12.2012
 *      Author: kreyl
 */

#include "kl_lib_f100.h"
#include "kl_sprintf.h"
#include <stdarg.h>
#include <string.h>

// ================================ Timer ======================================
void Timer_t::Init(TIM_TypeDef* PTmr) {
    ITmr = PTmr;
    if     (ITmr == TIM1)  { rccEnableTIM1(FALSE); }
    else if(ITmr == TIM2)  { rccEnableTIM2(FALSE); }
    else if(ITmr == TIM3)  { rccEnableTIM3(FALSE); }
    else if(ITmr == TIM6)  { rccEnableAPB1(RCC_APB1ENR_TIM6EN, FALSE); }
    else if(ITmr == TIM7)  { rccEnableAPB1(RCC_APB1ENR_TIM7EN, FALSE); }
    else if(ITmr == TIM15) { rccEnableAPB2(RCC_APB2ENR_TIM15EN, FALSE); }
    else if(ITmr == TIM16) { rccEnableAPB2(RCC_APB2ENR_TIM16EN, FALSE); }
    else if(ITmr == TIM17) { rccEnableAPB2(RCC_APB2ENR_TIM17EN, FALSE); }
    // Clock src
    if(ANY_OF_4(ITmr, TIM1, TIM15, TIM16, TIM17)) PClk = &Clk.APB2FreqHz;
    else PClk = &Clk.APB1FreqHz;
}

void Timer_t::InitPwm(GPIO_TypeDef *GPIO, uint16_t N, uint8_t Chnl, Inverted_t Inverted, bool EnablePreload) {
    // GPIO
    PinSetupAlterFuncOutput(GPIO, N, omPushPull);
    // Enable outputs for advanced timers
    ITmr->BDTR = TIM_BDTR_MOE | TIM_BDTR_AOE;
    // Output
    uint16_t tmp = (Inverted == invInverted)? 0b1110 : 0b1100; // PWM mode 1 or 2
    if(EnablePreload) tmp |= 0b1;
    switch(Chnl) {
        case 1:
            PCCR = &ITmr->CCR1;
            ITmr->CCMR1 |= (tmp << 3);
            ITmr->CCER  |= TIM_CCER_CC1E;
            break;

        case 2:
            PCCR = &ITmr->CCR2;
            ITmr->CCMR1 |= (tmp << 11);
            ITmr->CCER  |= TIM_CCER_CC2E;
            break;

        case 3:
            PCCR = &ITmr->CCR3;
            ITmr->CCMR2 |= (tmp << 3);
            ITmr->CCER  |= TIM_CCER_CC3E;
            break;

        case 4:
            PCCR = &ITmr->CCR4;
            ITmr->CCMR2 |= (tmp << 11);
            ITmr->CCER  |= TIM_CCER_CC4E;
            break;

        default: break;
    }
}

// ================================ PWM pin ====================================
void PwmPin_t::Init(GPIO_TypeDef *GPIO, uint16_t N, uint8_t TimN, uint8_t Chnl, uint16_t TopValue, bool Inverted) {
    PinSetupAlterFuncOutput(GPIO, N, omPushPull);
    switch(TimN) {
        case 1:
            Tim = TIM1;
            rccEnableTIM1(FALSE);
            break;
        case 2:
            Tim = TIM2;
            rccEnableTIM2(FALSE);
            break;

        case 3:
            Tim = TIM3;
            rccEnableTIM3(FALSE);
            break;
#if !defined (STM32F10X_LD) && !defined (STM32F10X_LD_VL)
        case 4:
            Tim = TIM4;
            rccEnableTIM4(FALSE);
            break;
#endif
        case 15:
            Tim = TIM15;
            rccEnableAPB2(RCC_APB2ENR_TIM15EN, FALSE);
            break;
        case 16:
            Tim = TIM16;
            rccEnableAPB2(RCC_APB2ENR_TIM16EN, FALSE);
            break;

        case 17:
            Tim = TIM17;
            rccEnableAPB2(RCC_APB2ENR_TIM17EN, FALSE);
            break;

        default: return; break;
    }

    // Clock src
    if(ANY_OF_4(TimN, 1, 15, 16, 17)) PClk = &Clk.APB2FreqHz;
    else PClk = &Clk.APB1FreqHz;

    // Common
    Tim->CR1 = TIM_CR1_CEN | TIM_CR1_ARPE; // Enable timer, set clk division to 0, AutoReload buffered
    Tim->CR2 = 0;
    Tim->ARR = TopValue;
    Tim->BDTR = TIM_BDTR_MOE | TIM_BDTR_AOE;

    // Output
    uint16_t tmp = Inverted? 0b111 : 0b110; // PWM mode 1 or 2
    switch(Chnl) {
        case 1:
            PCCR = &Tim->CCR1;
            Tim->CCMR1 |= (tmp << 4);
            Tim->CCER  |= TIM_CCER_CC1E;
            break;

        case 2:
            PCCR = &Tim->CCR2;
            Tim->CCMR1 |= (tmp << 12);
            Tim->CCER  |= TIM_CCER_CC2E;
            break;

        case 3:
            PCCR = &Tim->CCR3;
            Tim->CCMR2 |= (tmp << 4);
            Tim->CCER  |= TIM_CCER_CC3E;
            break;

        case 4:
            PCCR = &Tim->CCR4;
            Tim->CCMR2 |= (tmp << 12);
            Tim->CCER  |= TIM_CCER_CC4E;
            break;

        default: break;
    }
    *PCCR = 0;
}

void PwmPin_t::SetFreqHz(uint32_t FreqHz) {
    uint32_t divider = Tim->ARR * FreqHz;
    if(divider == 0) return;
    uint32_t FPrescaler = *PClk / divider;
    if(FPrescaler != 0) FPrescaler--;   // do not decrease in case of high freq
    Tim->PSC = (uint16_t)FPrescaler;
}

#if CH_DBG_ENABLED // ========================= DEBUG ==========================
void chDbgPanic(const char *msg1) {
    Uart.PrintNow(msg1);
}
#endif

// =============================== I2C =========================================
void i2cDmaIrqHandler(void *p, uint32_t flags) {
    chSysLockFromIsr();
    //Uart.Printf("===T===");
    Thread *PThd = ((i2c_t*)p)->PRequestingThread;
    if (PThd != NULL) {
        ((i2c_t*)p)->PRequestingThread = NULL;
        chSchReadyI(PThd);
    }
    chSysUnlockFromIsr();
}

void i2c_t::Init(I2C_TypeDef *pi2c, uint32_t BitrateHz) {
    ii2c = pi2c;
    IBitrateHz = BitrateHz;
    if(ii2c == I2C1) {
        IPGpio = GPIOB;
        ISclPin = 6;
        ISdaPin = 7;
        PDmaTx = STM32_DMA1_STREAM6;
        PDmaRx = STM32_DMA1_STREAM7;
    }
    Standby();
    Resume();

    // ==== DMA ====
    // Here only unchanged parameters of the DMA are configured.
    // Setup Dma TX
    dmaStreamAllocate(PDmaTx, IRQ_PRIO_MEDIUM, i2cDmaIrqHandler, this);
    dmaStreamSetPeripheral(PDmaTx, &ii2c->DR);
    dmaStreamSetMode      (PDmaTx,
            DMA_PRIORITY_LOW |
            STM32_DMA_CR_MSIZE_BYTE |
            STM32_DMA_CR_PSIZE_BYTE |
            STM32_DMA_CR_MINC |         // Memory pointer increase
            STM32_DMA_CR_DIR_M2P |      // Direction is memory to peripheral
            STM32_DMA_CR_TCIE           // Enable Transmission Complete IRQ
             );

    // Setup Dma RX
    dmaStreamAllocate(PDmaRx, IRQ_PRIO_MEDIUM, i2cDmaIrqHandler, this);
    dmaStreamSetPeripheral(PDmaRx, &ii2c->DR);
    dmaStreamSetMode      (PDmaRx,
            DMA_PRIORITY_LOW |
            STM32_DMA_CR_MSIZE_BYTE |
            STM32_DMA_CR_PSIZE_BYTE |
            STM32_DMA_CR_MINC |         // Memory pointer increase
            STM32_DMA_CR_DIR_P2M        // Direction is peripheral to memory
            | STM32_DMA_CR_TCIE         // Enable Transmission Complete IRQ
             );
}

void i2c_t::Standby() {
    if      (ii2c == I2C1) { rccResetI2C1(); rccDisableI2C1(FALSE); }
#if !defined (STM32F10X_LD) && !defined (STM32F10X_LD_VL)
    else if (ii2c == I2C2) { rccResetI2C2(); rccDisableI2C2(FALSE); }
#endif
    // Disable GPIOs
    PinSetupAnalog(IPGpio, ISclPin);
    PinSetupAnalog(IPGpio, ISdaPin);
}

void i2c_t::Resume() {
    Error = false;
    // ==== GPIOs ====
    PinSetupAlterFuncOutput(IPGpio, ISclPin, omOpenDrain);
    PinSetupAlterFuncOutput(IPGpio, ISdaPin, omOpenDrain);
    // ==== Clock and reset ====
    if      (ii2c == I2C1) { rccEnableI2C1(FALSE); rccResetI2C1(); }
#if !defined (STM32F10X_LD) && !defined (STM32F10X_LD_VL)
    else if (ii2c == I2C2) { rccEnableI2C2(FALSE); rccResetI2C2(); }
    else if (ii2c == I2C3) { rccEnableI2C3(FALSE); rccResetI2C3(); }
#endif
    // Minimum clock is 2 MHz
    uint32_t ClkMhz = Clk.APB1FreqHz / 1000000;
    uint16_t tmpreg = ii2c->CR2;
    tmpreg &= (uint16_t)~I2C_CR2_FREQ;
    if(ClkMhz < 2)  ClkMhz = 2;
    if(ClkMhz > 24) ClkMhz = 24;
    tmpreg |= ClkMhz;
    ii2c->CR2 = tmpreg;
    ii2c->CR1 &= (uint16_t)~I2C_CR1_PE; // Disable i2c to setup TRise & CCR
    ii2c->TRISE = (uint16_t)(((ClkMhz * 300) / 1000) + 1);
    // 16/9
    tmpreg = (uint16_t)(Clk.APB1FreqHz / (IBitrateHz * 25));
    if(tmpreg == 0) tmpreg = 1; // minimum allowed value
    tmpreg |= I2C_CCR_FS | I2C_CCR_DUTY;
    ii2c->CCR = tmpreg;
    ii2c->CR1 |= I2C_CR1_PE;    // Enable i2c back
    // ==== DMA ====
    ii2c->CR2 |= I2C_CR2_DMAEN;
}

void i2c_t::Reset() {
    Standby();
    Resume();
}

uint8_t i2c_t::CmdWriteRead(uint8_t Addr,
        uint8_t *WPtr, uint8_t WLength,
        uint8_t *RPtr, uint8_t RLength) {
    if(IBusyWait() != OK) return FAILURE;
    // Clear flags
    ii2c->SR1 = 0;
    while(RxIsNotEmpty()) (void)ii2c->DR;   // Read DR until it empty
    ClearAddrFlag();
    // Start transmission
    SendStart();
    if(WaitEv5() != OK) return FAILURE;
    SendAddrWithWrite(Addr);
    if(WaitEv6() != OK) { SendStop(); return FAILURE; }
    // Start TX DMA if needed
    if(WLength != 0) {
        if(WaitEv8() != OK) return FAILURE;
        dmaStreamSetMemory0(PDmaTx, WPtr);
        dmaStreamSetTransactionSize(PDmaTx, WLength);
        PRequestingThread = chThdSelf();
        dmaStreamEnable(PDmaTx);
        chSysLock();
        chSchGoSleepS(THD_STATE_SUSPENDED); // Sleep until end
        chSysUnlock();
        dmaStreamDisable(PDmaTx);
    }
    // Read if needed
    if(RLength != 0) {
        if(WaitEv8() != OK) return FAILURE;
        // Send repeated start
        SendStart();
        if(WaitEv5() != OK) return FAILURE;
        SendAddrWithRead(Addr);
        if(WaitEv6() != OK) { SendStop(); return FAILURE; }
        // If number of data to be read is 1, then DMA cannot be used
        if(RLength == 1) {
            AckDisable();
            SendStop();
            if(WaitRx() != OK) return FAILURE;
            *RPtr = ReceiveData();
            if(WaitStop() != OK) return FAILURE;
            return OK;
        }
        else {  // more than 1 byte, use DMA
            AckEnable();
            dmaStreamSetMemory0(PDmaRx, RPtr);
            dmaStreamSetTransactionSize(PDmaRx, RLength);
            DmaLastTransferSet(); // Inform DMA that this is last transfer => do not ACK last byte
            PRequestingThread = chThdSelf();
            dmaStreamEnable(PDmaRx);
            chSysLock();
            chSchGoSleepS(THD_STATE_SUSPENDED); // Sleep until end
            chSysUnlock();
            dmaStreamDisable(PDmaRx);
        } // if lng==1
    } // if != 0
    SendStop();
    return OK;
}

uint8_t i2c_t::CmdWriteWrite(uint8_t Addr,
        uint8_t *WPtr1, uint8_t WLength1,
        uint8_t *WPtr2, uint8_t WLength2) {
    if(IBusyWait() != OK) return FAILURE;
    // Clear flags
    ii2c->SR1 = 0;
    while(RxIsNotEmpty()) (void)ii2c->DR;   // Read DR until it empty
    ClearAddrFlag();
    // Start transmission
    SendStart();
    if(WaitEv5() != OK) return FAILURE;
    SendAddrWithWrite(Addr);
    if(WaitEv6() != OK) { SendStop(); return FAILURE; }
    // Start TX DMA if needed
    if(WLength1 != 0) {
        if(WaitEv8() != OK) return FAILURE;
        dmaStreamSetMemory0(PDmaTx, WPtr1);
        dmaStreamSetTransactionSize(PDmaTx, WLength1);
        PRequestingThread = chThdSelf();
        dmaStreamEnable(PDmaTx);
        chSysLock();
        chSchGoSleepS(THD_STATE_SUSPENDED); // Sleep until end
        chSysUnlock();
        dmaStreamDisable(PDmaTx);
    }
    if(WLength2 != 0) {
        if(WaitEv8() != OK) return FAILURE;
        dmaStreamSetMemory0(PDmaTx, WPtr2);
        dmaStreamSetTransactionSize(PDmaTx, WLength2);
        PRequestingThread = chThdSelf();
        dmaStreamEnable(PDmaTx);
        chSysLock();
        chSchGoSleepS(THD_STATE_SUSPENDED); // Sleep until end
        chSysUnlock();
        dmaStreamDisable(PDmaTx);
    }
    SendStop();
    return OK;
}

// ==== Flag operations ====
// Busy flag
uint8_t i2c_t::IBusyWait() {
    uint8_t RetryCnt = 4;
    while(RetryCnt--) {
        if(!(ii2c->SR2 & I2C_SR2_BUSY)) return OK;
        chThdSleepMilliseconds(1);
    }
    Error = true;
    return TIMEOUT;
}

// BUSY, MSL & SB flags
uint8_t i2c_t::WaitEv5() {
    uint32_t RetryCnt = 450;
    while(RetryCnt--) {
        uint16_t Flag1 = ii2c->SR1;
        uint16_t Flag2 = ii2c->SR2;
        if((Flag1 & I2C_SR1_SB) and (Flag2 & (I2C_SR2_MSL | I2C_SR2_BUSY))) return OK;
    }
    Error = true;
    return FAILURE;
}

uint8_t i2c_t::WaitEv6() {
    uint32_t RetryCnt = 45;
    uint16_t Flag1;
    do {
        Flag1 = ii2c->SR1;
        if((RetryCnt-- == 0) or (Flag1 & I2C_SR1_AF)) return FAILURE;   // Fail if timeout or NACK
    } while(!(Flag1 & I2C_SR1_ADDR)); // ADDR set when Address is sent and ACK received
    (void)ii2c->SR2;    // Clear address flag
    return OK;
}

uint8_t i2c_t::WaitEv8() {
    uint32_t RetryCnt = 45;
    while(RetryCnt--)
        if(ii2c->SR1 & I2C_SR1_TXE) return OK;
    Error = true;
    return TIMEOUT;
}

uint8_t i2c_t::WaitRx() {
    uint32_t RetryCnt = 450;
    while(RetryCnt--)
        if(ii2c->SR1 & I2C_SR1_RXNE) return OK;
    return TIMEOUT;
}

uint8_t i2c_t::WaitStop() {
    uint32_t RetryCnt = 450;
    while(RetryCnt--)
        if(ii2c->CR1 & I2C_CR1_STOP) return OK;
    return TIMEOUT;
}
