/*
 * kl_lib.cpp
 *
 *  Created on: 10.12.2012
 *      Author: kreyl
 */

#include "shell.h"
#include "MsgQ.h"
#include <malloc.h>
#include "board.h"
#include <string>
#include "chversion.h"

#if 0 // ============================ General ==================================
// To replace standard error handler in case of virtual methods implementation
//extern "C" void __cxa_pure_virtual() {
//    Uart.PrintfNow("pure_virtual\r");
//}

// Amount of memory occupied by thread
uint32_t GetThdFreeStack(void *wsp, uint32_t size) {
    uint32_t n = 0;
    uint32_t RequestedSize = size - (sizeof(thread_t) +
            (size_t)PORT_GUARD_PAGE_SIZE +
            sizeof (struct port_intctx) +
            sizeof (struct port_extctx) +
            (size_t)PORT_INT_REQUIRED_STACK);
#if CH_DBG_FILL_THREADS
    uint8_t *startp = (uint8_t *)wsp;
    uint8_t *endp = (uint8_t *)wsp + RequestedSize;
    while (startp < endp)
        if(*startp++ == CH_DBG_STACK_FILL_VALUE) ++n;
#endif
    return n;
}

void PrintThdFreeStack(void *wsp, uint32_t size) {
    uint32_t RequestedSize = size - (sizeof(thread_t) +
            (size_t)PORT_GUARD_PAGE_SIZE +
            sizeof (struct port_intctx) +
            sizeof (struct port_extctx) +
            (size_t)PORT_INT_REQUIRED_STACK);

    Printf("Free stack memory: %u of %u bytes\r",
            GetThdFreeStack(wsp, size), RequestedSize);
}

#endif

/********************************************
arena;     total space allocated from system
ordblks;   number of non-inuse chunks
hblks;     number of mmapped regions
hblkhd;    total space in mmapped regions
uordblks;  total allocated space
fordblks;  total non-inuse space
keepcost;  top-most, releasable (via malloc_trim) space
**********************************************/
void PrintMemoryInfo() {
    struct mallinfo info = mallinfo();
    Printf(
            "total space allocated from system: %u\r"
            "number of non-inuse chunks: %u\r"
            "number of mmapped regions: %u\r"
            "total space in mmapped regions: %u\r"
            "total allocated space: %u\r"
            "total non-inuse space: %u\r"
            "top-most, releasable: %u\r",
            info.arena, info.ordblks, info.hblks, info.hblkhd,
            info.uordblks, info.fordblks, info.keepcost);
}

extern "C"
caddr_t _sbrk(int incr) {
    extern uint8_t __heap_base__;
    extern uint8_t __heap_end__;

    static uint8_t *current_end = &__heap_base__;
    uint8_t *current_block_address = current_end;

    incr = (incr + 3) & (~3);
    if(current_end + incr > &__heap_end__) {
        errno = ENOMEM;
        return (caddr_t) -1;
    }
    current_end += incr;
    return (caddr_t)current_block_address;
}

#ifdef DMA_MEM2MEM
static thread_reference_t MemThdRef;
static void DmaMem2MemIrq(void *p, uint32_t flags) {
    chSysLockFromISR();
    chThdResumeI(&MemThdRef, MSG_OK);
    chSysUnlockFromISR();
}

namespace Mem2MemDma { // ========== MEM2MEM DMA ===========

void Init() {
    dmaStreamAllocate(DMA_MEM2MEM, IRQ_PRIO_HIGH, DmaMem2MemIrq, nullptr);
}

#define MEM2MEM_DMA_MODE_BYTE(Chnl) \
    (STM32_DMA_CR_CHSEL(Chnl) | \
    DMA_PRIORITY_VERYHIGH | STM32_DMA_CR_MSIZE_BYTE | STM32_DMA_CR_PSIZE_BYTE | \
    STM32_DMA_CR_MINC | STM32_DMA_CR_PINC | STM32_DMA_CR_DIR_M2M | STM32_DMA_CR_TCIE)

#define MEM2MEM_DMA_MODE_W16(Chnl) \
    (STM32_DMA_CR_CHSEL(Chnl) | \
    DMA_PRIORITY_VERYHIGH | STM32_DMA_CR_MSIZE_HWORD | STM32_DMA_CR_PSIZE_HWORD | \
    STM32_DMA_CR_MINC | STM32_DMA_CR_PINC | STM32_DMA_CR_DIR_M2M | STM32_DMA_CR_TCIE)

#define MEM2MEM_DMA_MODE_W32(Chnl) \
    (STM32_DMA_CR_CHSEL(Chnl) | \
    DMA_PRIORITY_VERYHIGH | STM32_DMA_CR_MSIZE_WORD | STM32_DMA_CR_PSIZE_WORD | \
    STM32_DMA_CR_MINC | STM32_DMA_CR_PINC | STM32_DMA_CR_DIR_M2M | STM32_DMA_CR_TCIE)

void MemCpy(void *Dst, void *Src, uint32_t Sz) {
    dmaStreamSetPeripheral(DMA_MEM2MEM, Src);
    dmaStreamSetMemory0(DMA_MEM2MEM, Dst);
    // Check if W32 copy is possible
    if(!(Sz & 0b11UL) and !((uint32_t)Dst & 0b11UL) and !((uint32_t)Src & 0b11UL)) {
        dmaStreamSetTransactionSize(DMA_MEM2MEM, Sz / 4);
        dmaStreamSetMode(DMA_MEM2MEM, MEM2MEM_DMA_MODE_W32(1));
    }
    // Check if W16 is possible
    else if(!(Sz & 0b1UL) and !((uint32_t)Dst & 0b1UL) and !((uint32_t)Src & 0b1UL)) {
        dmaStreamSetTransactionSize(DMA_MEM2MEM, Sz / 2);
        dmaStreamSetMode(DMA_MEM2MEM, MEM2MEM_DMA_MODE_W16(1));
    }
    // Otherwise, copy byte by byte
    else {
        dmaStreamSetTransactionSize(DMA_MEM2MEM, Sz);
        dmaStreamSetMode(DMA_MEM2MEM, MEM2MEM_DMA_MODE_BYTE(1));
    }
    chSysLock();
    dmaStreamEnable(DMA_MEM2MEM);
    chThdSuspendS(&MemThdRef);
    dmaStreamDisable(DMA_MEM2MEM);
    chSysUnlock();
}

} // namespace
#endif

#if defined STM32L4XX
namespace Random {
void TrueInit() {
    rccEnableAHB2(RCC_AHB2ENR_RNGEN, FALSE);
    RNG->CR = RNG_CR_RNGEN; // Enable random generator
    while((RNG->SR & RNG_SR_DRDY) == 0);    // Wait for new random value
}

void TrueDeinit() {
    RNG->CR = 0;
    rccDisableAHB2(RCC_AHB2ENR_RNGEN);
}

uint32_t TrueGenerate(uint32_t LowInclusive, uint32_t HighInclusive) {
    while((RNG->SR & RNG_SR_DRDY) == 0);    // Wait for new random value
    uint32_t dw = RNG->DR;
    uint32_t rslt = (dw % (HighInclusive + 1 - LowInclusive)) + LowInclusive;
//    PrintfI("%u; l %u; h %u; r %u\r", dw, LowInclusive, HighInclusive, rslt);
    return rslt;
}

void SeedWithTrue() {
    while((RNG->SR & RNG_SR_DRDY) == 0);    // Wait for new random value
    uint32_t dw = RNG->DR;
    Seed(dw);
}

} // namespace
#endif

#if 1 // ============================= Timer ===================================
void Timer_t::Init() const {
#ifdef TIM1
    if(ITmr == TIM1)  { rccEnableTIM1(FALSE); }
#endif
#ifdef TIM2
    if(ITmr == TIM2)  { rccEnableTIM2(FALSE); }
#endif
#ifdef TIM3
    else if(ITmr == TIM3)  { rccEnableTIM3(FALSE); }
#endif
#ifdef TIM4
    else if(ITmr == TIM4)  { rccEnableTIM4(FALSE); }
#endif
#ifdef TIM5
    else if(ITmr == TIM5)  { rccEnableTIM5(FALSE); }
#endif
#ifdef TIM6
    else if(ITmr == TIM6)  { rccEnableTIM6(FALSE); }
#endif
#ifdef TIM7
    else if(ITmr == TIM7)  { rccEnableTIM7(FALSE); }
#endif
#ifdef TIM8
    else if(ITmr == TIM8)  { rccEnableTIM8(FALSE); }
#endif
#ifdef TIM9
    else if(ITmr == TIM9)  { rccEnableTIM9(FALSE); }
#endif
#ifdef TIM10
    else if(ITmr == TIM10)  { rccEnableTIM10(FALSE); }
#endif
#ifdef TIM11
    else if(ITmr == TIM11)  { rccEnableTIM11(FALSE); }
#endif
#ifdef TIM12
    else if(ITmr == TIM12)  { rccEnableTIM12(FALSE); }
#endif
#ifdef TIM13
    else if(ITmr == TIM13)  { rccEnableTIM13(FALSE); }
#endif
#ifdef TIM14
    else if(ITmr == TIM14)  { rccEnableTIM14(FALSE); }
#endif
#ifdef TIM15
    else if(ITmr == TIM15)  { rccEnableTIM15(FALSE); }
#endif
#ifdef TIM16
    else if(ITmr == TIM16)  { rccEnableTIM16(FALSE); }
#endif
#ifdef TIM17
    else if(ITmr == TIM17)  { rccEnableTIM17(FALSE); }
#endif
#ifdef LPTIM1
#if defined STM32F7XX
    else if(ILPTim == LPTIM1)  { rccEnableAPB1(RCC_APB1ENR_LPTIM1EN, FALSE); }
#else
    else if(ILPTim == LPTIM1)  { rccEnableAPB1R1(RCC_APB1ENR1_LPTIM1EN, FALSE); }
#endif // MCU
#endif
#ifdef LPTIM2
    else if(ILPTim == LPTIM2)  { rccEnableAPB1R2(RCC_APB1ENR2_LPTIM2EN, FALSE); }
#endif
}

void Timer_t::Deinit() const {
    TMR_DISABLE(ITmr);
#ifdef TIM1
    if(ITmr == TIM1)  { rccDisableTIM1(); }
#endif
#ifdef TIM2
    if(ITmr == TIM2)  { rccDisableTIM2(); }
#endif
#ifdef TIM3
    else if(ITmr == TIM3)  { rccDisableTIM3(); }
#endif
#ifdef TIM4
    else if(ITmr == TIM4)  { rccDisableTIM4(); }
#endif
#ifdef TIM5
    else if(ITmr == TIM5)  { rccDisableTIM5(); }
#endif
#ifdef TIM6
    else if(ITmr == TIM6)  { rccDisableTIM6(); }
#endif
#ifdef TIM7
    else if(ITmr == TIM7)  { rccDisableTIM7(); }
#endif
#ifdef TIM8
    else if(ITmr == TIM8)  { rccDisableTIM8(); }
#endif
#ifdef TIM9
    else if(ITmr == TIM9)  { rccDisableTIM9(); }
#endif
#ifdef TIM10
    else if(ITmr == TIM10)  { rccDisableTIM10(); }
#endif
#ifdef TIM11
    else if(ITmr == TIM11)  { rccDisableTIM11(); }
#endif
#ifdef TIM12
    else if(ITmr == TIM12)  { rccDisableTIM12(); }
#endif
#ifdef TIM13
    else if(ITmr == TIM13)  { rccDisableTIM13(); }
#endif
#ifdef TIM14
    else if(ITmr == TIM14)  { rccDisableTIM14(); }
#endif
#ifdef TIM15
    else if(ITmr == TIM15)  { rccDisableTIM15(); }
#endif
#ifdef TIM16
    else if(ITmr == TIM16)  { rccDisableTIM16(); }
#endif
#ifdef TIM17
    else if(ITmr == TIM17)  { rccDisableTIM17(); }
#endif
#ifdef LPTIM1
#if defined STM32F7XX
    else if(ILPTim == LPTIM1)  { rccDisableAPB1(RCC_APB1ENR_LPTIM1EN); }
#else
    else if(ILPTim == LPTIM1)  { rccDisableAPB1R1(RCC_APB1ENR1_LPTIM1EN); }
#endif // MCU
#endif
#ifdef LPTIM2
    else if(ILPTim == LPTIM2)  { rccDisableAPB1R2(RCC_APB1ENR2_LPTIM2EN); }
#endif
}

void Timer_t::SetupPrescaler(uint32_t PrescaledFreqHz) const {
    ITmr->PSC = (Clk.GetTimInputFreq(ITmr) / PrescaledFreqHz) - 1;
}

void PinOutputPWM_t::Init() const {
    Timer_t::Init();

#if defined STM32L4XX
    if(ILPTim == LPTIM1 or ILPTim == LPTIM2) {
        // Enable timer to allow further operations
        ILPTim->CR |= LPTIM_CR_ENABLE;
        if(ILpmSetup.Inverted == invNotInverted) ILPTim->CFGR |= LPTIM_CFGR_WAVPOL;
        else ILPTim->CFGR &= ~LPTIM_CFGR_WAVPOL;
        ILPTim->ARR = ILpmSetup.TopValue;
        // Start timer
        ILPTim->CR |= LPTIM_CR_CNTSTRT;
    }
    else {
#endif

#if !defined STM32L151xB
    ITmr->BDTR = 0xC000;   // Main output Enable
#endif
    ITmr->CR1 |= TIM_CR1_ARPE;
    ITmr->ARR = ISetup.TopValue;
    // Setup Output
    uint16_t tmp = (ISetup.Inverted == invInverted)? 0b111 : 0b110; // PWM mode 1 or 2
    switch(ISetup.TimerChnl) {
        case 1:
            ITmr->CCMR1 |= (tmp << 4);
            ITmr->CCER  |= TIM_CCER_CC1E;
            break;
        case 2:
            ITmr->CCMR1 |= (tmp << 12);
            ITmr->CCER  |= TIM_CCER_CC2E;
            break;
        case 3:
            ITmr->CCMR2 |= (tmp << 4);
            ITmr->CCER  |= TIM_CCER_CC3E;
            break;
        case 4:
            ITmr->CCMR2 |= (tmp << 12);
            ITmr->CCER  |= TIM_CCER_CC4E;
            break;
        default: break;
    }
    Enable();
#if defined STM32L4XX
    } // if LPTIM
#endif

    // GPIO
#if defined STM32L1XX
    AlterFunc_t AF = AF1; // For TIM2
    if(ANY_OF_2(ITmr, TIM3, TIM4)) AF = AF2;
    else if(ANY_OF_3(ITmr, TIM9, TIM10, TIM11)) AF = AF3;
    PinSetupAlterFunc(ISetup.PGpio, ISetup.Pin, ISetup.OutputType, pudNone, AF);
#elif defined STM32F0XX
    if     (ITmr == TIM1)  PinSetupAlterFunc(ISetup.PGpio, ISetup.Pin, ISetup.OutputType, pudNone, AF2);
    else if(ITmr == TIM3) {
        if(ISetup.PGpio == GPIOA or ISetup.PGpio == GPIOB) PinSetupAlterFunc(ISetup.PGpio, ISetup.Pin, ISetup.OutputType, pudNone, AF1);
        else PinSetupAlterFunc(ISetup.PGpio, ISetup.Pin, ISetup.OutputType, pudNone, AF0);
    }
    else if(ITmr == TIM14) {
        if(ISetup.PGpio == GPIOA) PinSetupAlterFunc(ISetup.PGpio, ISetup.Pin, ISetup.OutputType, pudNone, AF4);
        else PinSetupAlterFunc(ISetup.PGpio, ISetup.Pin, ISetup.OutputType, pudNone, AF0);
    }
#ifdef TIM15
    else if(ITmr == TIM15) {
        if(ISetup.PGpio == GPIOA) PinSetupAlterFunc(ISetup.PGpio, ISetup.Pin, ISetup.OutputType, pudNone, AF0);
        else PinSetupAlterFunc(ISetup.PGpio, ISetup.Pin, ISetup.OutputType, pudNone, AF1);
    }
#endif
    else if(ITmr == TIM16 or ITmr == TIM17) {
        if(ISetup.PGpio == GPIOA) PinSetupAlterFunc(ISetup.PGpio, ISetup.Pin, ISetup.OutputType, pudNone, AF5);
        else PinSetupAlterFunc(ISetup.PGpio, ISetup.Pin, ISetup.OutputType, pudNone, AF2);
    }
#elif defined STM32F2XX || defined STM32F4XX
    if(ANY_OF_2(ITmr, TIM1, TIM2)) PinSetupAlterFunc(ISetup.PGpio, ISetup.Pin, ISetup.OutputType, pudNone, AF1);
    else if(ANY_OF_3(ITmr, TIM3, TIM4, TIM5)) PinSetupAlterFunc(ISetup.PGpio, ISetup.Pin, ISetup.OutputType, pudNone, AF2);
    else if(ANY_OF_4(ITmr, TIM8, TIM9, TIM10, TIM11)) PinSetupAlterFunc(ISetup.PGpio, ISetup.Pin, ISetup.OutputType, pudNone, AF3);
    else if(ANY_OF_3(ITmr, TIM12, TIM13, TIM14)) PinSetupAlterFunc(ISetup.PGpio, ISetup.Pin, ISetup.OutputType, pudNone, AF9);
#elif defined STM32F100_MCUCONF
    PinSetupAlterFunc(ISetup.PGpio, ISetup.Pin, ISetup.OutputType, pudNone, AF0);   // Alternate function is dummy
#elif defined STM32L4XX
    AlterFunc_t AF = AF1;
    if(ITmr == TIM1 or ITmr == TIM2 or ILPTim == LPTIM1) AF = AF1;
#ifdef TIM3
    else if(ITmr == TIM3) AF = AF2;
#endif
#ifdef TIM4
    else if(ITmr == TIM4) AF = AF2;
#endif
#ifdef TIM5
    else if(ITmr == TIM5) AF = AF2;
#endif
#ifdef TIM8
    else if(ITmr == TIM8) AF = AF3;
#endif

#ifdef TIM15
    else if(ITmr == TIM15) AF = AF14;
#endif
#ifdef TIM16
    else if(ITmr == TIM16) AF = AF14;
#endif
#ifdef TIM17
    else if(ITmr == TIM17) AF = AF14;
#endif
#ifdef LPTIM2
    else if(ILPTim == LPTIM2) AF = AF14;
#endif
    PinSetupAlterFunc(ISetup.PGpio, ISetup.Pin, ISetup.OutputType, pudNone, AF);
#endif
}

void Timer_t::SetUpdateFrequencyChangingPrescaler(uint32_t FreqHz) const {
    // Figure out input timer freq
    uint32_t UpdFreqMax = Clk.GetTimInputFreq(ITmr) / (ITmr->ARR + 1);
    uint32_t Psc = UpdFreqMax / FreqHz;
    if(Psc != 0) Psc--;
//    Uart.Printf("InputFreq=%u; UpdFreqMax=%u; div=%u; ARR=%u\r", InputFreq, UpdFreqMax, div, ITmr->ARR);
    ITmr->PSC = Psc;
    ITmr->CNT = 0;  // Reset counter to start from scratch
}

void Timer_t::SetUpdateFrequencyChangingTopValue(uint32_t FreqHz) const {
    uint32_t UpdFreqMax = Clk.GetTimInputFreq(ITmr) / (ITmr->PSC + 1);
    uint32_t TopVal  = (UpdFreqMax / FreqHz);
    if(TopVal != 0) TopVal--;
    SetTopValue(TopVal);
    ITmr->CNT = 0;  // Reset counter to start from scratch
}

void Timer_t::SetUpdateFrequencyChangingBoth(uint32_t FreqHz) const {
    uint32_t Psc = (Clk.GetTimInputFreq(ITmr) / FreqHz) / 0x10000;
    ITmr->PSC = Psc;
    SetUpdateFrequencyChangingTopValue(FreqHz);
}

void Timer_t::SetTmrClkFreq(uint32_t FreqHz) const {
    uint32_t Psc = Clk.GetTimInputFreq(ITmr) / FreqHz;
    if(Psc != 0) Psc--;
    ITmr->PSC = Psc;
}
#endif

#if 1 // ========================= Virtual Timers ==============================
// Universal VirtualTimer callback
#if CH_VERSION_YEAR == 19
void TmrKLCallback(void *p) {
#else
void TmrKLCallback(virtual_timer_t *vtp, void *p) {
#endif
    chSysLockFromISR();
    ((IrqHandler_t*)p)->IIrqHandler();
    chSysUnlockFromISR();
}

void TmrKL_t::IIrqHandler() {    // Call it inside callback
    EvtQMain.SendNowOrExitI(EvtMsg_t(EvtId));
    if(TmrType == tktPeriodic) StartI();
}

void TmrKL_t::StartI() {
    if(Period == 0) EvtQMain.SendNowOrExitI(EvtMsg_t(EvtId)); // Do not restart even if periodic: this will not work good anyway
    else chVTSetI(&Tmr, Period, TmrKLCallback, this); // Will be reset before start
}
#endif

#if 1 // ============================= DEBUG ===================================
extern "C" {

void chDbgPanic(const char *msg1) {
#if CH_USE_REGISTRY
    Uart.PrintfNow("\r%S @ %S\r", msg1, chThdSelf()->p_name);
#else
    Printf("\r%S\r", msg1);
#endif
}

void PrintErrMsg(const char* S) {
    CMD_UART->CR3 &= ~USART_CR3_DMAT;
    while(*S != 0) {
//        ITM_SendChar(*S++);
#if defined STM32L1XX || defined STM32F2XX || defined STM32F1XX
        while(!(CMD_UART->SR & USART_SR_TXE));
        CMD_UART->DR = *S;
#else
        while(!(CMD_UART->ISR & USART_ISR_TXE));
        CMD_UART->TDR = *S;
#endif
        S++;
    }
}

void HardFault_Handler(void) {
    PrintErrMsg("\rHardFault\r");
    __ASM volatile("BKPT #01");
    while(true);
}

} // extern C
#endif

#if 1 // ================= FLASH & EEPROM ====================
#define FLASH_EraseTimeout      TIME_MS2I(45)
#define FLASH_ProgramTimeout    TIME_MS2I(45)
namespace Flash {

// ==== Common ====
void ClearPendingFlags() {
#ifdef STM32L1XX
    FLASH->SR = FLASH_SR_EOP | FLASH_SR_PGAERR | FLASH_SR_WRPERR;
#elif defined STM32L4XX
    FLASH->SR = FLASH_SR_EOP | FLASH_SR_PROGERR | FLASH_SR_WRPERR;
#elif defined STM32F7XX

#else
    FLASH->SR = FLASH_SR_EOP | FLASH_SR_PGERR | FLASH_SR_WRPRTERR;
#endif
}

#if defined STM32L4XX
void ClearErrFlags() {
    FLASH->SR |= FLASH_SR_OPTVERR | FLASH_SR_RDERR | FLASH_SR_FASTERR |
            FLASH_SR_MISERR | FLASH_SR_PGSERR | FLASH_SR_SIZERR |
            FLASH_SR_PGAERR | FLASH_SR_WRPERR | FLASH_SR_PROGERR | FLASH_SR_OPERR;
}

// Wait for a Flash operation to complete or a TIMEOUT to occur
uint8_t WaitForLastOperation(systime_t Timeout_st) {
    systime_t start = chVTGetSystemTimeX();
    while(FLASH->SR & FLASH_SR_BSY) {
        if(Timeout_st != TIME_INFINITE) {
            if(chVTTimeElapsedSinceX(start) >= Timeout_st) return retvTimeout;
        }
    }
    if((FLASH->SR & FLASH_SR_OPERR) or (FLASH->SR & FLASH_SR_PROGERR) or
            (FLASH->SR & FLASH_SR_WRPERR) or (FLASH->SR & FLASH_SR_PGAERR) or
            (FLASH->SR & FLASH_SR_SIZERR) or (FLASH->SR & FLASH_SR_PGSERR) or
            (FLASH->SR & FLASH_SR_MISERR) or (FLASH->SR & FLASH_SR_FASTERR) or
            (FLASH->SR & FLASH_SR_RDERR) or (FLASH->SR & FLASH_SR_OPTVERR)) {
        return retvFail;
    }
    // Clear EOP if set
    if(FLASH->SR & FLASH_SR_EOP) FLASH->SR |= FLASH_SR_EOP;
    return retvOk;
}
#else
static uint8_t GetStatus(void) {
    if(FLASH->SR & FLASH_SR_BSY) return retvBusy;
#if defined STM32L1XX
    else if(FLASH->SR & FLASH_SR_WRPERR) return retvWriteProtect;
    else if(FLASH->SR & (uint32_t)0x1E00) return retvFail;
#elif defined STM32F2XX

#elif defined STM32F7XX

#else
    else if(FLASH->SR & FLASH_SR_PGERR) return retvFail;
    else if(FLASH->SR & FLASH_SR_WRPRTERR) return retvFail;
#endif
    else return retvOk;
}

uint8_t WaitForLastOperation(systime_t Timeout_st) {
    uint8_t status = retvOk;
    // Wait for a Flash operation to complete or a TIMEOUT to occur
    systime_t Start = chVTGetSystemTimeX();
    do {
        status = GetStatus();
        if(chVTTimeElapsedSinceX(Start) >= Timeout_st) return retvTimeout;
    } while(status == retvBusy);
    return status;
}
#endif

#if defined STM32L1XX
// When properly executed, the unlocking sequence clears the PELOCK bit in the FLASH_PECR register
static void UnlockEEAndPECR() {
    if(FLASH->PECR & FLASH_PECR_PELOCK) {
        // Unlocking the Data memory and FLASH_PECR register access
        FLASH->PEKEYR = 0x89ABCDEF;
        FLASH->PEKEYR = 0x02030405;
        FLASH->SR = FLASH_SR_WRPERR;        // Clear WriteProtectErr
        FLASH->PECR &= ~FLASH_PECR_FTDW;    // Disable fixed time programming
    }
}
// To lock the FLASH_PECR and the data EEPROM again, the software only needs to set the PELOCK bit in FLASH_PECR
static void LockEEAndPECR() { FLASH->PECR |= FLASH_PECR_PELOCK; }
#endif // L151

// ==== Flash ====
#if defined FLASH_CR_LOCK
bool IsLocked() { return (bool)(FLASH->CR & FLASH_CR_LOCK); }
#endif

void UnlockFlash() {
#if defined STM32L1XX
    UnlockEEAndPECR();
    FLASH->PRGKEYR = 0x8C9DAEBF;
    FLASH->PRGKEYR = 0x13141516;
#else
    FLASH->KEYR = 0x45670123;
    FLASH->KEYR = 0xCDEF89AB;
#endif
}
void LockFlash() {
#if defined STM32L1XX
    FLASH->PECR |= FLASH_PECR_PRGLOCK;
#else
    WaitForLastOperation(FLASH_ProgramTimeout);
    FLASH->CR |= FLASH_CR_LOCK;
#endif
}

// Beware: for L4xx, use Page Address (0...255), not absolute address kind of 0x08003f00. For Fxx, absolute addr is required.
uint8_t ErasePage(uint32_t PageAddress) {
    uint8_t status = WaitForLastOperation(FLASH_EraseTimeout);
    if(status == retvOk) {
#if defined STM32L1XX
        // PECR and Flash must be unlocked
        FLASH->PECR |= FLASH_PECR_ERASE;
        FLASH->PECR |= FLASH_PECR_PROG;
        // Write 0x0000 0000 to the first word of the page to erase
        *((volatile uint32_t*)PageAddress) = 0;
        status = WaitForLastOperation(FLASH_EraseTimeout);
        FLASH->PECR &= ~FLASH_PECR_PROG;
        FLASH->PECR &= ~FLASH_PECR_ERASE;
#elif defined STM32L4XX
        chSysLock();
        ClearErrFlags();    // Clear all error programming flags
        uint32_t Reg = FLASH->CR;
#ifdef FLASH_CR_BKER
        Reg &= ~(FLASH_CR_PNB | FLASH_CR_BKER);
#else
        Reg &= ~FLASH_CR_PNB;
#endif
        Reg |= (PageAddress << FLASH_CR_PNB_Pos) | FLASH_CR_PER;
        FLASH->CR = Reg;
        FLASH->CR |= FLASH_CR_STRT;
        status = WaitForLastOperation(FLASH_EraseTimeout);
        FLASH->CR &= ~FLASH_CR_PER; // Disable the PageErase Bit
        chSysUnlock();
#elif defined STM32F7XX

#else
        FLASH->CR |= FLASH_CR_PER;
        FLASH->AR = PageAddress;
        FLASH->CR |= FLASH_CR_STRT;
        __NOP(); // The software should start checking if the BSY bit equals 0 at least one CPU cycle after setting the STRT bit.
        // Wait for last operation to be completed
        status = WaitForLastOperation(FLASH_EraseTimeout);
        // Disable the PER Bit
        FLASH->CR &= 0x00001FFD;
#endif
    }
    return status;
}

#if defined STM32L4XX
uint8_t ProgramBuf32(uint32_t Address, uint32_t *PData, int32_t ASzBytes) {
//    Printf("PrgBuf %X  %u\r", Address, ASzBytes); chThdSleepMilliseconds(45);
    ASzBytes = 8 * ((ASzBytes + 7) / 8);
    uint8_t status = WaitForLastOperation(FLASH_ProgramTimeout);
    if(status == retvOk) {
        chSysLock();
        ClearErrFlags();
        FLASH->ACR &= ~FLASH_ACR_DCEN;      // Deactivate the data cache to avoid data misbehavior
        FLASH->CR |= FLASH_CR_PG;           // Enable flash writing
        // Write data
        while(ASzBytes > 0 and status == retvOk) {
            // Write Word64
            *(volatile uint32_t*)Address = *PData++;
            Address += 4;
            *(volatile uint32_t*)Address = *PData++;
            Address += 4;
            ASzBytes -= 8;
            status = WaitForLastOperation(FLASH_ProgramTimeout);
        }
        FLASH->CR &= ~FLASH_CR_PG;          // Disable flash writing
        // Flush the caches to be sure of the data consistency
        FLASH->ACR |= FLASH_ACR_ICRST;      // }
        FLASH->ACR &= ~FLASH_ACR_ICRST;     // } Reset instruction cache
        FLASH->ACR |= FLASH_ACR_ICEN;       // Enable instruction cache
        FLASH->ACR |= FLASH_ACR_DCRST;      // }
        FLASH->ACR &= ~FLASH_ACR_DCRST;     // } Reset data cache
        FLASH->ACR |= FLASH_ACR_DCEN;       // Enable data cache
        chSysUnlock();
    }
    return status;
}
#else
uint8_t ProgramWord(uint32_t Address, uint32_t Data) {
    uint8_t status = WaitForLastOperation(FLASH_ProgramTimeout);
    if(status == retvOk) {
#if defined STM32L1XX
        // PECR and Flash must be unlocked
        *((volatile uint32_t*)Address) = Data;
        status = WaitForLastOperation(FLASH_ProgramTimeout);
#else
        FLASH->CR |= FLASH_CR_PG;
        // Program the new first half word
        *(volatile uint16_t*)Address = (uint16_t)Data;
        status = WaitForLastOperation(FLASH_ProgramTimeout);
        if(status == retvOk) {
            // Program the new second half word
            uint32_t tmp = Address + 2;
            *(volatile uint16_t*)tmp = Data >> 16;
            status = WaitForLastOperation(FLASH_ProgramTimeout);
        }
        FLASH->CR &= 0x00001FFE;  // FLASH_CR_PG_Reset Disable the PG Bit
#endif
    }
    return status;
}

uint8_t ProgramBuf(void *PData, uint32_t ByteSz, uint32_t Addr) {
    uint8_t status = retvOk;
    uint32_t *p = (uint32_t*)PData;
    uint32_t DataWordCount = (ByteSz + 3) / 4;
    chSysLock();
    UnlockFlash();
    // Erase flash
    ClearPendingFlags();
    status = ErasePage(Addr);
//    Uart.PrintfI("  Flash erase %u: %u\r", status);
    if(status != retvOk) {
        PrintfI("Flash erase error\r");
        goto end;
    }
    // Program flash
    for(uint32_t i=0; i<DataWordCount; i++) {
        status = ProgramWord(Addr, *p);
        if(status != retvOk) {
            PrintfI("Flash write error\r");
            goto end;
        }
        Addr += 4;
        p++;
    }
    end:
    LockFlash();
    chSysUnlock();
    return status;
}
#endif

// ==== Option bytes ====
void UnlockOptionBytes() {
#ifdef STM32L4XX
    FLASH->OPTKEYR = 0x08192A3B;
    FLASH->OPTKEYR = 0x4C5D6E7F;
#elif defined STM32L1XX
    UnlockEEAndPECR();
    FLASH->OPTKEYR = 0xFBEAD9C8;
    FLASH->OPTKEYR = 0x24252627;
#elif defined STM32F2XX

#elif defined STM32F7XX

#else
    UnlockFlash();
    FLASH->OPTKEYR = FLASH_OPTKEY1;
    FLASH->OPTKEYR = FLASH_OPTKEY2;
#endif
}
void LockOptionBytes() {
#ifdef STM32L4XX
    FLASH->CR |= FLASH_CR_OPTLOCK;
#elif defined STM32L1XX
    // To lock the option byte block again, the software only needs to set the OPTLOCK bit in FLASH_PECR
    FLASH->PECR |= FLASH_PECR_OPTLOCK;
#elif defined STM32F2XX

#elif defined STM32F7XX

#else
    CLEAR_BIT(FLASH->CR, FLASH_CR_OPTWRE);
    LockFlash();
#endif
}

void WriteOptionBytes(uint32_t OptReg) {
    ClearPendingFlags();
    if(WaitForLastOperation(FLASH_ProgramTimeout) == retvOk) {
#ifdef STM32L1XX
        uint32_t OptBytes = *(volatile uint32_t*)0x1FF80000;
        OptBytes &= 0xFF00FF00; // Clear RDP and nRDP
        OptBytes |= OptReg;      // Write RDP
        OptBytes |= (OptReg ^ 0xFF) << 16; // Write nRDP;
        *(volatile uint32_t*)0x1FF80000 = OptBytes;
        WaitForLastOperation(FLASH_ProgramTimeout);
#elif defined STM32L4XX
        FLASH->OPTR = OptReg;
        FLASH->CR |= FLASH_CR_OPTSTRT;
        WaitForLastOperation(FLASH_ProgramTimeout);
#elif defined STM32F2XX

#elif defined STM32F7XX

#else
        // Erase option bytes
        SET_BIT(FLASH->CR, FLASH_CR_OPTER);
        SET_BIT(FLASH->CR, FLASH_CR_STRT);
        uint8_t Rslt = WaitForLastOperation(FLASH_ProgramTimeout);
        CLEAR_BIT(FLASH->CR, FLASH_CR_OPTER);
        if(Rslt == retvOk) {
            SET_BIT(FLASH->CR, FLASH_CR_OPTPG); // Enable the Option Bytes Programming operation
            OB->RDP = OptReg;
            WaitForLastOperation(FLASH_ProgramTimeout);
            CLEAR_BIT(FLASH->CR, FLASH_CR_OPTPG); // Disable the Option Bytes Programming operation
        }
#endif
    }
}

#if defined FLASH_OPTR_BFB2
void ToggleBootBankAndReset() {
    uint32_t Optr = FLASH->OPTR;
    // switch BFB bit and enable dualbank just in case
    Optr = (Optr ^ FLASH_OPTR_BFB2) | FLASH_OPTR_DUALBANK;
    Flash::LockFlash(); // Just in case if not locked
    while(FLASH->SR & FLASH_SR_BSY);
    Flash::UnlockFlash();
    Flash::UnlockOptionBytes();
    FLASH->OPTR = Optr;
    FLASH->CR |= FLASH_CR_OPTSTRT;
    while(FLASH->SR & FLASH_SR_BSY);
    // Option byte loading requested. Will reset MCU.
    FLASH->CR |= FLASH_CR_OBL_LAUNCH;
    Flash::LockOptionBytes(); // Must never be here, but who knows.
    Flash::LockFlash();
}
#endif

// ==== Firmare lock ====
bool FirmwareIsLocked() {
#ifdef STM32L4XX
    return (FLASH->OPTR & 0xFF) != 0xAA;
#elif defined STM32L1XX
    return (FLASH->OBR & 0xFF) != 0xAA;
#elif defined STM32F2XX
    return false;
#elif defined STM32F7XX
    return false;
#else
    return (FLASH->OBR & 0b0110);
#endif
}

void LockFirmware() {
    chSysLock();
#ifdef STM32L4XX
    UnlockFlash();
    ClearPendingFlags();
    UnlockOptionBytes();
    if(WaitForLastOperation(FLASH_ProgramTimeout) == retvOk) {
        uint32_t reg = FLASH->OPTR;
        reg &= 0xFFFFFF00; // Any value except 0xAA or 0xCC
        FLASH->OPTR = reg;
        FLASH->CR |= FLASH_CR_OPTSTRT;
        WaitForLastOperation(FLASH_ProgramTimeout);

        reg = FLASH->PCROP1ER;
        reg |= 0x80000000;
        FLASH->PCROP1ER = reg;
        FLASH->CR |= FLASH_CR_OPTSTRT;
        WaitForLastOperation(FLASH_ProgramTimeout);
        FLASH->CR |= FLASH_CR_OBL_LAUNCH; // Option byte loading requested
        LockFlash();    // Will lock option bytes too
        WaitForLastOperation(FLASH_ProgramTimeout);
    }
#elif defined STM32F0XX

#else
    WriteOptionBytes(0x1D); // Any value except 0xAA or 0xCC
    // Set the OBL_Launch bit to reset system and launch the option byte loading
#ifdef STM32L1XX
    FLASH->PECR |= FLASH_PECR_OBL_LAUNCH;
#elif defined STM32F2XX || defined STM32F1XX

#elif defined STM32F7XX

#else
    SET_BIT(FLASH->CR, FLASH_CR_OBL_LAUNCH);
#endif
#endif
    chSysUnlock();
}

#ifdef STM32L4XX
bool IwdgIsFrozenInStandby() {
    return !(FLASH->OPTR & FLASH_OPTR_IWDG_STDBY);
}
void IwdgFrozeInStandby() {
    chSysLock();
    UnlockFlash();
    ClearPendingFlags();
    UnlockOptionBytes();
    if(WaitForLastOperation(FLASH_ProgramTimeout) == retvOk) {
        uint32_t OptReg = FLASH->OPTR;
        OptReg &= ~FLASH_OPTR_IWDG_STDBY;
        FLASH->OPTR = OptReg;
        FLASH->CR |= FLASH_CR_OPTSTRT;
        WaitForLastOperation(FLASH_ProgramTimeout);
        SET_BIT(FLASH->CR, FLASH_CR_OBL_LAUNCH); // cannot be written when option bytes are locked
        LockFlash();
    }
    chSysUnlock();
}
#endif

// ==== Dualbank ====
#if defined STM32L4XX
bool DualbankIsEnabled() {
    return (FLASH->OPTR & FLASH_OPTR_DUALBANK);
}
void DisableDualbank() {
    chSysLock();
    UnlockFlash();
    ClearPendingFlags();
    UnlockOptionBytes();
    if(WaitForLastOperation(FLASH_ProgramTimeout) == retvOk) {
        uint32_t OptReg = FLASH->OPTR;
        OptReg &= ~(FLASH_OPTR_DUALBANK | FLASH_OPTR_BFB2);
        FLASH->OPTR = OptReg;
        FLASH->CR |= FLASH_CR_OPTSTRT;
        WaitForLastOperation(FLASH_ProgramTimeout);
        SET_BIT(FLASH->CR, FLASH_CR_OBL_LAUNCH); // cannot be written when option bytes are locked
        LockFlash();
    }
    chSysUnlock();
}

bool SleepInResetIsEnabled() {
    return (!(FLASH->OPTR & FLASH_OPTR_nRST_SHDW) or !(FLASH->OPTR & FLASH_OPTR_nRST_STDBY) or !(FLASH->OPTR & FLASH_OPTR_nRST_STOP));
}

void DisableSleepInReset() {
    chSysLock();
    UnlockFlash();
    ClearPendingFlags();
    UnlockOptionBytes();
    if(WaitForLastOperation(FLASH_ProgramTimeout) == retvOk) {
        uint32_t OptReg = FLASH->OPTR;
        OptReg |= FLASH_OPTR_nRST_SHDW | FLASH_OPTR_nRST_STDBY | FLASH_OPTR_nRST_STOP;
        FLASH->OPTR = OptReg;
        FLASH->CR |= FLASH_CR_OPTSTRT;
        WaitForLastOperation(FLASH_ProgramTimeout);
        SET_BIT(FLASH->CR, FLASH_CR_OBL_LAUNCH); // cannot be written when option bytes are locked
        LockFlash();
    }
    chSysUnlock();
}
#endif

}; // Namespace FLASH
#endif

#if defined STM32L1XX // =================== Internal EEPROM ===================
#define EEPROM_BASE_ADDR    ((uint32_t)0x08080000)
namespace EE {
uint32_t Read32(uint32_t Addr) {
    return *((uint32_t*)(Addr + EEPROM_BASE_ADDR));
}

uint8_t Write32(uint32_t Addr, uint32_t W) {
    Addr += EEPROM_BASE_ADDR;
//    Uart.Printf("EAdr=%u\r", Addr);
    Flash::UnlockEEAndPECR();
    // Wait for last operation to be completed
    uint8_t status = Flash::WaitForLastOperation(FLASH_ProgramTimeout);
    if(status == retvOk) {
        *(volatile uint32_t*)Addr = W;
        status = Flash::WaitForLastOperation(FLASH_ProgramTimeout);
    }
    Flash::LockEEAndPECR();
    return status;
}

void ReadBuf(void *PDst, uint32_t Sz, uint32_t Addr) {
    uint32_t *p32 = (uint32_t*)PDst;
    Sz = Sz / 4;  // Size in words32
    while(Sz--) {
        *p32 = Read32(Addr);
        p32++;
        Addr += 4;
    }
}

uint8_t WriteBuf(void *PSrc, uint32_t Sz, uint32_t Addr) {
    uint32_t *p32 = (uint32_t*)PSrc;
    Addr += EEPROM_BASE_ADDR;
    Sz = (Sz + 3) / 4;  // Size in words32
    Flash::UnlockEEAndPECR();
    // Wait for last operation to be completed
    uint8_t status = Flash::WaitForLastOperation(FLASH_ProgramTimeout);
    while((status == retvOk) and (Sz > 0))  {
        *(volatile uint32_t*)Addr = *p32;
        status = Flash::WaitForLastOperation(FLASH_ProgramTimeout);
        p32++;
        Addr += 4;
        Sz--;
    }
    Flash::LockEEAndPECR();
    return status;
}

};
#endif

#if 1 // =========================== External IRQ ==============================
// IRQ handlers
extern "C" {
extern void PrintfCNow(const char *format, ...);

#if INDIVIDUAL_EXTI_IRQ_REQUIRED
IrqHandler_t* ExtiIrqHandler[16];
#else
#if defined STM32L1XX || defined STM32F4XX || defined STM32F2XX || defined STM32L4XX || defined STM32F1XX
ftVoidVoid ExtiIrqHandler[5], ExtiIrqHandler_9_5, ExtiIrqHandler_15_10;
#elif defined STM32F030 || defined STM32F0
ftVoidVoid ExtiIrqHandler_0_1, ExtiIrqHandler_2_3, ExtiIrqHandler_4_15;
#endif
#endif // INDIVIDUAL_EXTI_IRQ_REQUIRED

#if defined STM32L1XX || defined STM32F2XX || defined STM32L4XX || defined STM32F1XX
// EXTI pending register
#if defined STM32L1XX || defined STM32F2XX || defined STM32F1XX
#define EXTI_PENDING_REG    EXTI->PR
#elif defined STM32L4XX
#define EXTI_PENDING_REG    EXTI->PR1
#endif

// EXTI 0
void Vector58() {
    CH_IRQ_PROLOGUE();
    chSysLockFromISR();
    ftVoidVoid handler = ExtiIrqHandler[0];
    if(handler != nullptr) handler();
    else PrintfC("Unhandled %S\r", __FUNCTION__);
    EXTI_PENDING_REG = 0x0001; // Clean IRQ flags
    chSysUnlockFromISR();
    CH_IRQ_EPILOGUE();
}

// EXTI 1
void Vector5C() {
    CH_IRQ_PROLOGUE();
    chSysLockFromISR();
    ftVoidVoid handler = ExtiIrqHandler[1];
    if(handler != nullptr) handler();
    else PrintfC("Unhandled %S\r", __FUNCTION__);
    EXTI_PENDING_REG = 0x0002; // Clean IRQ flags
    chSysUnlockFromISR();
    CH_IRQ_EPILOGUE();
}

// EXTI 2
void Vector60() {
    CH_IRQ_PROLOGUE();
    chSysLockFromISR();
    ftVoidVoid handler = ExtiIrqHandler[2];
    if(handler != nullptr) handler();
    else PrintfC("Unhandled %S\r", __FUNCTION__);
    EXTI_PENDING_REG = 0x0004; // Clean IRQ flags
    chSysUnlockFromISR();
    CH_IRQ_EPILOGUE();
}

// EXTI 3
void Vector64() {
    CH_IRQ_PROLOGUE();
    chSysLockFromISR();
    ftVoidVoid handler = ExtiIrqHandler[3];
    if(handler != nullptr) handler();
    else PrintfC("Unhandled %S\r", __FUNCTION__);
    EXTI_PENDING_REG = 0x0008; // Clean IRQ flags
    chSysUnlockFromISR();
    CH_IRQ_EPILOGUE();
}

// EXTI 4
void Vector68() {
    CH_IRQ_PROLOGUE();
    chSysLockFromISR();
    ftVoidVoid handler = ExtiIrqHandler[4];
    if(handler != nullptr) handler();
    else PrintfC("Unhandled %S\r", __FUNCTION__);
    EXTI_PENDING_REG = 0x0010; // Clean IRQ flags
    chSysUnlockFromISR();
    CH_IRQ_EPILOGUE();
}

// EXTI 9_5
void Vector9C() {
    CH_IRQ_PROLOGUE();
    chSysLockFromISR();
#if INDIVIDUAL_EXTI_IRQ_REQUIRED
    for(int i=5; i<=9; i++) {
        if(ExtiIrqHandler[i] != nullptr) ExtiIrqHandler[i]->IIrqHandler();
    }
#else
    if(ExtiIrqHandler_9_5 != nullptr) ExtiIrqHandler_9_5();
    else PrintfC("Unhandled %S\r", __FUNCTION__);
#endif
    EXTI_PENDING_REG = 0x03E0; // Clean IRQ flags
    chSysUnlockFromISR();
    CH_IRQ_EPILOGUE();
}

// EXTI 15_10
void VectorE0() {
    CH_IRQ_PROLOGUE();
    chSysLockFromISR();
#if INDIVIDUAL_EXTI_IRQ_REQUIRED
    for(int i=10; i<=15; i++) {
        if(ExtiIrqHandler[i] != nullptr) ExtiIrqHandler[i]->IIrqHandler();
    }
#else
    if(ExtiIrqHandler_15_10 != nullptr) ExtiIrqHandler_15_10();
    else PrintfC("Unhandled %S\r", __FUNCTION__);
#endif
    EXTI_PENDING_REG = 0xFC00; // Clean IRQ flags
    chSysUnlockFromISR();
    CH_IRQ_EPILOGUE();
}

#elif defined STM32F030 || defined STM32F0
// EXTI0_1
void Vector54() {
    CH_IRQ_PROLOGUE();
    chSysLockFromISR();
#if INDIVIDUAL_EXTI_IRQ_REQUIRED
    uint32_t ClearMask = 0;
    if(EXTI->PR & (1<<0)) {
        ClearMask = 1<<0;
        if(ExtiIrqHandler[0] != nullptr) ExtiIrqHandler[0]->IIrqHandler();
    }
    if(EXTI->PR & (1<<1)) {
        ClearMask += 1<<1;
        if(ExtiIrqHandler[1] != nullptr) ExtiIrqHandler[1]->IIrqHandler();
    }
    EXTI->PR = ClearMask;
#else
    if(ExtiIrqHandler_0_1 != nullptr) ExtiIrqHandler_0_1();
//    else PrintfCNow("Unhandled %S\r", __FUNCTION__);
    EXTI->PR = 0x0003;  // Clean IRQ flag
#endif
    chSysUnlockFromISR();
    CH_IRQ_EPILOGUE();
}

// EXTI2_3
void Vector58() {
    CH_IRQ_PROLOGUE();
    chSysLockFromISR();
#if INDIVIDUAL_EXTI_IRQ_REQUIRED
    uint32_t ClearMask = 0;
        if(EXTI->PR & (1<<2)) {
            ClearMask = 1<<2;
            if(ExtiIrqHandler[2] != nullptr) ExtiIrqHandler[2]->IIrqHandler();
        }
        if(EXTI->PR & (1<<3)) {
            ClearMask += 1<<3;
            if(ExtiIrqHandler[3] != nullptr) ExtiIrqHandler[3]->IIrqHandler();
        }
        EXTI->PR = ClearMask;
#else
    if(ExtiIrqHandler_2_3 != nullptr) ExtiIrqHandler_2_3();
//    else PrintfCNow("Unhandled %S\r", __FUNCTION__);
    EXTI->PR = 0x000C;  // Clean IRQ flag
#endif
    chSysUnlockFromISR();
    CH_IRQ_EPILOGUE();
}

// EXTI4_15
void Vector5C() {
    CH_IRQ_PROLOGUE();
    chSysLockFromISR();
#if INDIVIDUAL_EXTI_IRQ_REQUIRED
    uint32_t ClearMask = 0;
    for(int i=4; i<=15; i++) {
        uint32_t Mask = 1<<i;
        if(EXTI->PR & Mask) {
            ClearMask += Mask;
            if(ExtiIrqHandler[i] != nullptr) ExtiIrqHandler[i]->IIrqHandler();
        }
    }
    EXTI->PR = ClearMask;
#else
    if(ExtiIrqHandler_4_15 != nullptr) ExtiIrqHandler_4_15();
//    else PrintfCNow("Unhandled %S\r", __FUNCTION__);
    EXTI->PR = 0xFFF0;  // Clean IRQ flag
#endif
    chSysUnlockFromISR();
    CH_IRQ_EPILOGUE();
}
#endif
} // extern c
#endif

#if 1 // ============== Conversion operations ====================
namespace Convert {
void U16ToArrAsBE(uint8_t *PArr, uint16_t N) {
    uint8_t *p8 = (uint8_t*)&N;
    *PArr++ = *(p8 + 1);
    *PArr   = *p8;
}
void U32ToArrAsBE(uint8_t *PArr, uint32_t N) {
    uint8_t *p8 = (uint8_t*)&N;
    *PArr++ = *(p8 + 3);
    *PArr++ = *(p8 + 2);
    *PArr++ = *(p8 + 1);
    *PArr   = *p8;
}
uint16_t ArrToU16AsBE(uint8_t *PArr) {
    uint16_t N;
    uint8_t *p8 = (uint8_t*)&N;
    *p8++ = *(PArr + 1);
    *p8 = *PArr;
    return N;
}
uint32_t ArrToU32AsBE(uint8_t *PArr) {
    uint32_t N;
    uint8_t *p8 = (uint8_t*)&N;
    *p8++ = *(PArr + 3);
    *p8++ = *(PArr + 2);
    *p8++ = *(PArr + 1);
    *p8 = *PArr;
    return N;
}

uint8_t TryStrToUInt32(char* S, uint32_t *POutput) {
    if(*S == '\0') return retvEmpty;
    char *p;
    *POutput = strtoul(S, &p, 0);
    return (*p == 0)? retvOk : retvNotANumber;
}
uint8_t TryStrToInt32(char* S, int32_t *POutput) {
    if(*S == '\0') return retvEmpty;
    char *p;
    *POutput = strtol(S, &p, 0);
    return (*p == '\0')? retvOk : retvNotANumber;
}

uint16_t BuildUint16(uint8_t Lo, uint8_t Hi) {
    uint16_t r = Hi;
    r <<= 8;
    r |= Lo;
    return r;
}

uint32_t BuildUint32(uint8_t Lo, uint8_t MidLo, uint8_t MidHi, uint8_t Hi) {
    uint32_t r = Hi;
    r <<= 8;
    r |= MidHi;
    r <<= 8;
    r |= MidLo;
    r <<= 8;
    r |= Lo;
    return r;
}

// ==== Float ====
uint8_t TryStrToFloat(char* S, float *POutput) {
    if(*S == '\0') return retvEmpty;
    char *p;
    *POutput = strtof(S, &p);
    return (*p == '\0')? retvOk : retvNotANumber;
}
}; // namespace
#endif

#if 1 // ============================== IWDG ===================================
namespace Iwdg {
enum Pre_t {
    iwdgPre4 = 0x00,
    iwdgPre8 = 0x01,
    iwdgPre16 = 0x02,
    iwdgPre32 = 0x03,
    iwdgPre64 = 0x04,
    iwdgPre128 = 0x05,
    iwdgPre256 = 0x06
};

void DisableInDebug() {
#ifdef APB1FZR1
    DBGMCU->APB1FZR1 |= DBGMCU_APB1FZR1_DBG_IWDG_STOP;
#endif
}

static void Enable() { IWDG->KR = 0xCCCC; }
static void EnableAccess() { IWDG->KR = 0x5555; }

static void SetPrescaler(Pre_t Prescaler) { IWDG->PR = (uint32_t)Prescaler; }
static void SetReload(uint16_t Reload) { IWDG->RLR = Reload; }

void SetTimeout(uint32_t ms) {
    EnableAccess();
    SetPrescaler(iwdgPre256);
    uint32_t Count = (ms * (LSI_FREQ_HZ/1000UL)) / 256UL;
    LimitMaxValue(Count, 0xFFF);
    SetReload(Count);
    Reload();   // Reload and lock access
}

void InitAndStart(uint32_t ms) {
    Clk.EnableLSI();    // Start LSI
    SetTimeout(ms);     // Start IWDG
    Enable();
}


void GoSleep(uint32_t Timeout_ms) {
    chSysLock();
    Clk.EnableLSI();        // Start LSI
    SetTimeout(Timeout_ms); // Start IWDG
    Enable();
    // Enter standby mode
    Sleep::EnterStandby();
    chSysUnlock();
}
};
#endif

#if 1 // ============================== Clocking ===============================
Clk_t Clk;
#define CLK_STARTUP_TIMEOUT     9999
#ifndef CRYSTAL_FREQ_HZ
#define CRYSTAL_FREQ_HZ     12000000
#endif

#if defined STM32L1XX
// ==== Inner use ====
uint8_t Clk_t::EnableHSE() {
    RCC->CR |= RCC_CR_HSEON;    // Enable HSE
    // Wait until ready, 1ms typical according to datasheet
    uint32_t StartUpCounter=0;
    do {
        if(RCC->CR & RCC_CR_HSERDY) return retvOk;   // HSE is ready
        StartUpCounter++;
    } while(StartUpCounter < 45000);
    RCC->CR &= ~RCC_CR_HSEON;   // Disable HSE
    return retvTimeout;
}

uint8_t Clk_t::EnableHSI() {
    RCC->CR |= RCC_CR_HSION;
    // Wait until ready
    uint32_t StartUpCounter=0;
    do {
        if(RCC->CR & RCC_CR_HSIRDY) return 0;   // HSI is ready
        StartUpCounter++;
    } while(StartUpCounter < CLK_STARTUP_TIMEOUT);
    return 1; // Timeout
}

uint8_t Clk_t::EnablePLL() {
    RCC->CR |= RCC_CR_PLLON;
    // Wait until ready
    uint32_t StartUpCounter=0;
    do {
        if(RCC->CR & RCC_CR_PLLRDY) return 0;   // PLL is ready
        StartUpCounter++;
    } while(StartUpCounter < CLK_STARTUP_TIMEOUT);
    return 1; // Timeout
}

uint8_t Clk_t::EnableMSI() {
    RCC->CR |= RCC_CR_MSION;
    // Wait until ready
    uint32_t StartUpCounter=0;
    do {
        if(RCC->CR & RCC_CR_MSIRDY) return 0;   // MSI is ready
        StartUpCounter++;
    } while(StartUpCounter < CLK_STARTUP_TIMEOUT);
    return 1; // Timeout
}

void Clk_t::UpdateFreqValues() {
    uint32_t tmp, PllMul, PllDiv;
    uint32_t SysClkHz;
    // Tables
    const uint32_t MSIClk[8] = {65536, 131072, 262144, 524188, 1048000, 2097000, 4194000};
    const uint8_t PllMulTable[9] = {3, 4, 6, 8, 12, 16, 24, 32, 48};
    const uint8_t AHBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
    const uint8_t APBPrescTable[8] = {0, 0, 0, 0, 1, 2, 3, 4};

    // Figure out SysClk
    tmp = RCC->CFGR & RCC_CFGR_SWS;
    tmp >>= 2;
    switch(tmp) {
        case 0b00: // MSI
            tmp = (RCC->ICSCR & RCC_ICSCR_MSIRANGE) >> 13;
            SysClkHz = MSIClk[tmp];
            break;

        case 0b01: // HSI
            SysClkHz = HSI_FREQ_HZ;
            break;

        case 0b10: // HSE
            SysClkHz = CRYSTAL_FREQ_HZ;
            break;

        case 0b11: // PLL used as system clock source
            // Get different PLL dividers
            tmp = (RCC->CFGR & RCC_CFGR_PLLMUL) >> 18;
            PllMul = PllMulTable[tmp];
            PllDiv = ((RCC->CFGR & RCC_CFGR_PLLDIV) >> 22) +1;
            // Which src is used as pll input?
            SysClkHz = ((RCC->CFGR & RCC_CFGR_PLLSRC) == RCC_CFGR_PLLSRC_HSI)? HSI_FREQ_HZ : CRYSTAL_FREQ_HZ;
            SysClkHz = (SysClkHz * PllMul) / PllDiv;
            break;
    } // switch

    // AHB freq
    tmp = AHBPrescTable[((RCC->CFGR & RCC_CFGR_HPRE) >> 4)];
    AHBFreqHz = SysClkHz >> tmp;
    // APB freq
    uint32_t APB1prs = (RCC->CFGR & RCC_CFGR_PPRE1) >> 8;
    uint32_t APB2prs = (RCC->CFGR & RCC_CFGR_PPRE2) >> 11;
    tmp = APBPrescTable[APB1prs];
    APB1FreqHz = AHBFreqHz >> tmp;
    tmp = APBPrescTable[APB2prs];
    APB2FreqHz = AHBFreqHz >> tmp;
}

uint32_t Clk_t::GetTimInputFreq(TIM_TypeDef* ITmr) {
    uint32_t InputFreq = 0;
    // APB2
    if(ANY_OF_3(ITmr, TIM9, TIM10, TIM11)) {
        uint32_t APB2prs = (RCC->CFGR & RCC_CFGR_PPRE2) >> 11;
        if(APB2prs < 0b100) InputFreq = Clk.APB2FreqHz; // APB2CLK = HCLK / 1
        else InputFreq = Clk.APB2FreqHz * 2;           // APB2CLK = HCLK / (not 1)
    }
    // APB1
    else {
        uint32_t APB1prs = (RCC->CFGR & RCC_CFGR_PPRE1) >> 8;
        if(APB1prs < 0b100) InputFreq = Clk.APB1FreqHz; // APB1CLK = HCLK / 1
        else  InputFreq = Clk.APB1FreqHz * 2;           // APB1CLK = HCLK / (not 1)
    }
    return InputFreq;
}

// ==== Common use ====
// AHB, APB
void Clk_t::SetupBusDividers(AHBDiv_t AHBDiv, APBDiv_t APB1Div, APBDiv_t APB2Div) {
    // Setup dividers
    uint32_t tmp = RCC->CFGR;
    tmp &= ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2);  // Clear bits
    tmp |= ((uint32_t)AHBDiv)  << 4;
    tmp |= ((uint32_t)APB1Div) << 8;
    tmp |= ((uint32_t)APB2Div) << 11;
    RCC->CFGR = tmp;
}

// Enables HSI, switches to HSI
uint8_t Clk_t::SwitchToHSI() {
    if(EnableHSI() != 0) return 1;
    uint32_t tmp = RCC->CFGR;
    tmp &= ~RCC_CFGR_SW;
    tmp |=  RCC_CFGR_SW_HSI;  // Select HSI as system clock src
    RCC->CFGR = tmp;
    while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI); // Wait till ready
    return 0;
}

// Enables HSE, switches to HSE
uint8_t Clk_t::SwitchToHSE() {
    // Try to enable HSE several times
    for(uint32_t i=0; i<11; i++) {
        if(EnableHSE() == retvOk) {
            uint32_t tmp = RCC->CFGR;
            tmp &= ~RCC_CFGR_SW;
            tmp |=  RCC_CFGR_SW_HSE;  // Select HSE as system clock src
            RCC->CFGR = tmp;
            while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSE); // Wait till ready
            return retvOk;
        }
        else {
            DisableHSE();
            for(volatile uint32_t i=0; i<999; i++);
        }
    } // for
    return retvFail;
}

// Enables HSE, enables PLL, switches to PLL
uint8_t Clk_t::SwitchToPLL() {
    if(EnablePLL() != 0) return 2;
    // Select PLL as system clock src
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    // Wait until ready
    while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
    return retvOk;
}

// Enables MSI, switches to MSI
uint8_t Clk_t::SwitchToMSI() {
    if(EnableMSI() != 0) return 1;
    uint32_t tmp = RCC->CFGR;
    tmp &= ~RCC_CFGR_SW;
    tmp |=  RCC_CFGR_SW_MSI;      // Select MSI as system clock src
    RCC->CFGR = tmp;
    while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_MSI); // Wait until ready
    return retvOk;
}

// Disable PLL first!
// HsePreDiv: 1...16; PllMul: pllMul[]
uint8_t Clk_t::SetupPLLDividers(PllMul_t PllMul, PllDiv_t PllDiv) {
    if(RCC->CR & RCC_CR_PLLON) return 1;    // PLL must be disabled to change dividers
    uint32_t tmp = RCC->CFGR;
    tmp &= RCC_CFGR_PLLDIV | RCC_CFGR_PLLMUL;
    tmp |= ((uint32_t)PllDiv) << 22;
    tmp |= ((uint32_t)PllMul) << 18;
    RCC->CFGR = tmp;
    return 0;
}

void Clk_t::SetupFlashLatency(uint8_t AHBClk_MHz) {
    // Get VCore
    uint32_t tmp = PWR->CR;
    tmp &= PWR_CR_VOS;
    tmp >>= 11;
    VCore_t VCore = (VCore_t)tmp;
    if(     ((VCore == vcore1V2) and (AHBClk_MHz > 2)) or
            ((VCore == vcore1V5) and (AHBClk_MHz > 8)) or
            ((VCore == vcore1V8) and (AHBClk_MHz > 16))
            ) {
        FLASH->ACR |= FLASH_ACR_LATENCY;
    }
    else FLASH->ACR &= ~FLASH_ACR_LATENCY;
}

//void Clk_t::SetupAdcClk(ADCDiv_t ADCDiv) {
//    uint32_t tmp = RCC->CFGR;
//    tmp &= ~RCC_CFGR_ADCPRE;
//    tmp |= (uint32_t)ADCDiv;
//    RCC->CFGR = tmp;
//}

void Clk_t::PrintFreqs() {
    Printf("AHBFreq=%uMHz; APB1Freq=%uMHz; APB2Freq=%uMHz\r",
            Clk.AHBFreqHz/1000000, Clk.APB1FreqHz/1000000, Clk.APB2FreqHz/1000000);
}

// ==== V Core ====
void SetupVCore(VCore_t AVCore) {
    // PWR clock enable
    RCC->APB1ENR = RCC_APB1ENR_PWREN;
    // Core voltage setup
    while((PWR->CSR & PWR_CSR_VOSF) != 0); // Wait until regulator is stable
    uint32_t tmp = PWR->CR;
    tmp &= ~PWR_CR_VOS;
    tmp |= ((uint32_t)AVCore) << 11;
    PWR->CR = tmp;
    while((PWR->CSR & PWR_CSR_VOSF) != 0); // Wait until regulator is stable
}
#elif defined STM32F1XX
void Clk_t::UpdateFreqValues() {
    uint32_t tmp;
    uint32_t SysClkHz;
    // Tables
    const uint8_t AHBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
    const uint8_t APBPrescTable[8] = {0, 0, 0, 0, 1, 2, 3, 4};

    // Figure out SysClk
    tmp = RCC->CFGR & RCC_CFGR_SWS;
    tmp >>= 2;
    switch(tmp) {
        case 0b01: // HSE
            SysClkHz = CRYSTAL_FREQ_HZ;
            break;

        case 0b10: { // PLL used as system clock source
            // Get different PLL dividers
            uint32_t PllMul = ((RCC->CFGR & RCC_CFGR_PLLMULL) >> 18) + 2;
            uint32_t PllSrc = RCC->CFGR & RCC_CFGR_PLLSRC;
            if(PllSrc == 0) { // HSI oscillator clock divided by 2 selected as PLL clock entry
                SysClkHz = (HSI_FREQ_HZ >> 1) * PllMul;
            }
            else { // HSE or HSE/2
                if(RCC->CFGR & RCC_CFGR_PLLXTPRE) { // HSE/2
                    SysClkHz = (CRYSTAL_FREQ_HZ >> 1) * PllMul;
                }
                else { // HSE
                    SysClkHz = CRYSTAL_FREQ_HZ * PllMul;
                }
            }
        } break;

        default: // 0b00, HSI
            SysClkHz = HSI_FREQ_HZ;
            break;
    } // switch

    // AHB freq
    tmp = AHBPrescTable[((RCC->CFGR & RCC_CFGR_HPRE) >> 4)];
    AHBFreqHz = SysClkHz >> tmp;
    // APB freq
    uint32_t APB1prs = (RCC->CFGR & RCC_CFGR_PPRE1) >> 8;
    uint32_t APB2prs = (RCC->CFGR & RCC_CFGR_PPRE2) >> 11;
    tmp = APBPrescTable[APB1prs];
    APB1FreqHz = AHBFreqHz >> tmp;
    tmp = APBPrescTable[APB2prs];
    APB2FreqHz = AHBFreqHz >> tmp;
}

void Clk_t::PrintFreqs() {
    Printf("AHBFreq=%uMHz; APB1Freq=%uMHz; APB2Freq=%uMHz\r",
            Clk.AHBFreqHz/1000000, Clk.APB1FreqHz/1000000, Clk.APB2FreqHz/1000000);
}

uint32_t Clk_t::GetTimInputFreq(TIM_TypeDef* ITmr) {
    uint32_t InputFreq = 0;
    // APB2
    if(ANY_OF_4(ITmr, TIM1, TIM15, TIM16, TIM17)) {
        uint32_t APB2prs = (RCC->CFGR & RCC_CFGR_PPRE2) >> 11;
        if(APB2prs < 0b100) InputFreq = Clk.APB2FreqHz; // APB2CLK = HCLK / 1
        else InputFreq = Clk.APB2FreqHz * 2;           // APB2CLK = HCLK / (not 1)
    }
    // APB1
    else {
        uint32_t APB1prs = (RCC->CFGR & RCC_CFGR_PPRE1) >> 8;
        if(APB1prs < 0b100) InputFreq = Clk.APB1FreqHz; // APB1CLK = HCLK / 1
        else  InputFreq = Clk.APB1FreqHz * 2;           // APB1CLK = HCLK / (not 1)
    }
    return InputFreq;
}

void Clk_t::SetupFlashLatency(uint8_t AHBClk_MHz) {
    uint32_t tmp = FLASH->ACR;
#ifdef STM32F1XX
    tmp &= ~FLASH_ACR_HLFCYA;
#else
    tmp &= ~FLASH_ACR_LATENCY;  // Clear Latency bits: 0 wait states
#endif
    if(AHBClk_MHz > 24 and AHBClk_MHz <= 48) tmp |= 1; // 1 wait state
    else if(AHBClk_MHz > 48) tmp |= 2; // 2 wait states
    FLASH->ACR = tmp;
}

uint8_t Clk_t::SetupPllMulDiv(PllMul_t PllMul, PreDiv_t PreDiv) {
    if(RCC->CR & RCC_CR_PLLON) return retvBusy; // PLL must be disabled to change dividers
    uint32_t tmp = RCC->CFGR & ~RCC_CFGR_PLLMULL;
    tmp |= ((uint32_t)PllMul) << 18;
    RCC->CFGR = tmp;
    return retvOk;
}

void Clk_t::SetupBusDividers(AHBDiv_t AHBDiv, APBDiv_t APB1Div, APBDiv_t APB2Div) {
    // Setup dividers
    uint32_t tmp = RCC->CFGR;
    tmp &= ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2);  // Clear bits
    tmp |= ((uint32_t)AHBDiv)  << 4;
    tmp |= ((uint32_t)APB1Div) << 8;
    tmp |= ((uint32_t)APB2Div) << 11;
    RCC->CFGR = tmp;
}

uint8_t Clk_t::EnablePLL() {
    RCC->CR |= RCC_CR_PLLON;
    __NOP();__NOP();__NOP();__NOP();
    // Wait until ready
    uint32_t StartUpCounter=0;
    do {
        if(RCC->CR & RCC_CR_PLLRDY) return retvOk;   // PLL is ready
        StartUpCounter++;
    } while(StartUpCounter < CLK_STARTUP_TIMEOUT);
    return retvTimeout;
}

uint8_t Clk_t::SwitchToPLL() {
    RCC->CFGR |= RCC_CFGR_SW_PLL;   // Select PLL as system clock src
    __NOP();__NOP();__NOP();__NOP();
    while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL); // Wait until ready
    return retvOk;
}

#elif defined STM32F0XX
#include "CRS_defins.h"
// ==== Inner use ====
uint8_t Clk_t::EnableHSE() {
    RCC->CR |= RCC_CR_HSEON;    // Enable HSE
    // Wait until ready
    uint32_t StartUpCounter=0;
    do {
        if(RCC->CR & RCC_CR_HSERDY) return 0;   // HSE is ready
        StartUpCounter++;
    } while(StartUpCounter < CLK_STARTUP_TIMEOUT);
    return 1; // Timeout
}

uint8_t Clk_t::EnableHSI() {
    RCC->CR |= RCC_CR_HSION;
    // Wait until ready
    uint32_t StartUpCounter=0;
    do {
        if(RCC->CR & RCC_CR_HSIRDY) return 0;   // HSE is ready
        StartUpCounter++;
    } while(StartUpCounter < CLK_STARTUP_TIMEOUT);
    return 1; // Timeout
}

uint8_t Clk_t::EnablePLL() {
    RCC->CR |= RCC_CR_PLLON;
    // Wait until ready
    uint32_t StartUpCounter=0;
    do {
        if(RCC->CR & RCC_CR_PLLRDY) return 0;   // PLL is ready
        StartUpCounter++;
    } while(StartUpCounter < CLK_STARTUP_TIMEOUT);
    return 1; // Timeout
}

#ifdef RCC_CR2_HSI48ON
uint8_t Clk_t::EnableHSI48() {
    RCC->CR2 |= RCC_CR2_HSI48ON;
    for(volatile uint32_t i=0; i<999; i++); // Let it to stabilize. Otherwise program counter flies to space with Ozzy Osbourne
    uint32_t StartUpCounter=0;
    do {
        if(RCC->CR2 & RCC_CR2_HSI48RDY) return 0;   // Clock is ready
        StartUpCounter++;
    } while(StartUpCounter < CLK_STARTUP_TIMEOUT);
    return 1; // Timeout
}
#endif

uint32_t Clk_t::GetSysClkHz() {
    uint32_t tmp, PllSrc, PreDiv, PllMul;
    uint32_t SysClkHz = HSI_FREQ_HZ;
    // Figure out SysClk
    tmp = (RCC->CFGR & RCC_CFGR_SWS) >> 2;
    switch(tmp) {
        case csHSI:   SysClkHz = HSI_FREQ_HZ; break;
        case csHSE:   SysClkHz = CRYSTAL_FREQ_HZ; break;
        case csPLL: // PLL used as system clock source
            // Get different PLL dividers
            PreDiv = (RCC->CFGR2 & RCC_CFGR2_PREDIV) + 1;
            PllMul = ((RCC->CFGR & RCC_CFGR_PLLMUL) >> 18) + 2;
            if(PllMul > 16) PllMul = 16;
            // Which src is used as pll input?
            PllSrc = RCC->CFGR & RCC_CFGR_PLLSRC;
            switch(PllSrc) {
                case RCC_CFGR_PLLSRC_HSI_DIV2:   SysClkHz = HSI_FREQ_HZ / 2; break;
#ifdef RCC_CFGR_PLLSRC_HSI_PREDIV
                case RCC_CFGR_PLLSRC_HSI_PREDIV: SysClkHz = HSI_FREQ_HZ / PreDiv; break;
#endif
                case RCC_CFGR_PLLSRC_HSE_PREDIV: SysClkHz = CRYSTAL_FREQ_HZ / PreDiv; break;
#ifdef RCC_CFGR_PLLSRC_HSI48_PREDIV
                case RCC_CFGR_PLLSRC_HSI48_PREDIV: SysClkHz = HSI48_FREQ_HZ / PreDiv; break;
#endif
                default: break;
            }
            SysClkHz *= PllMul;
            break;
#ifdef RCC_CFGR_PLLSRC_HSI48_PREDIV
        case csHSI48: SysClkHz = HSI48_FREQ_HZ; break;
#endif
    } // switch
    return SysClkHz;
}


void Clk_t::UpdateFreqValues() {
    // AHB freq
    const uint8_t AHBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
    uint32_t tmp = AHBPrescTable[((RCC->CFGR & RCC_CFGR_HPRE) >> 4)];
    AHBFreqHz = GetSysClkHz() >> tmp;
    // APB freq
    const uint8_t APBPrescTable[8] = {0, 0, 0, 0, 1, 2, 3, 4};
    tmp = APBPrescTable[(RCC->CFGR & RCC_CFGR_PPRE) >> 8];
    APBFreqHz = AHBFreqHz >> tmp;
    // Timer multi
    TimerClkMulti = (tmp == 0)? 1 : 2;
    // ==== Update prescaler in System Timer ====
    uint32_t Psc = (SYS_TIM_CLK / OSAL_ST_FREQUENCY) - 1;
    TMR_DISABLE(STM32_ST_TIM);          // Stop counter
    uint32_t Cnt = STM32_ST_TIM->CNT;   // Save current time
    STM32_ST_TIM->PSC = Psc;
    TMR_GENERATE_UPD(STM32_ST_TIM);
    STM32_ST_TIM->CNT = Cnt;            // Restore time
    TMR_ENABLE(STM32_ST_TIM);
}

uint32_t Clk_t::GetTimInputFreq(TIM_TypeDef* ITmr) {
    uint32_t InputFreq = 0;
    uint32_t APB1prs = (RCC->CFGR & RCC_CFGR_PPRE) >> 8;
    if(APB1prs < 0b100) InputFreq = Clk.APBFreqHz;      // APB1CLK = HCLK / 1
    else InputFreq = Clk.APBFreqHz * 2;                 // APB1CLK = HCLK / (not 1)
    return InputFreq;
}

// ==== Common use ====
// AHB, APB
void Clk_t::SetupBusDividers(AHBDiv_t AHBDiv, APBDiv_t APBDiv) {
    uint32_t tmp = RCC->CFGR;
    tmp &= ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE);  // Clear bits
    tmp |= ((uint32_t)AHBDiv)  << 4;
    tmp |= ((uint32_t)APBDiv) << 8;
    RCC->CFGR = tmp;
}
void Clk_t::SetupBusDividers(uint32_t Dividers) {
    uint32_t tmp = RCC->CFGR;
    tmp &= ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE);  // Clear bits
    tmp |= Dividers;
    RCC->CFGR = tmp;
}

static inline uint8_t WaitSWS(uint32_t Desired) {
    uint32_t StartUpCounter=0;
    do {
        if((RCC->CFGR & RCC_CFGR_SWS) == Desired) return retvOk; // Done
        StartUpCounter++;
    } while(StartUpCounter < CLK_STARTUP_TIMEOUT);
    return retvTimeout;
}

// Enables HSI, switches to HSI
uint8_t Clk_t::SwitchTo(ClkSrc_t AClkSrc) {
    uint32_t tmp = RCC->CFGR & ~RCC_CFGR_SW;
    switch(AClkSrc) {
        case csHSI:
            if(EnableHSI() != retvOk) return 1;
            RCC->CFGR = tmp | RCC_CFGR_SW_HSI;  // Select HSI as system clock src
            return WaitSWS(RCC_CFGR_SWS_HSI);
            break;

        case csHSE:
            if(EnableHSE() != retvOk) return 2;
            RCC->CFGR = tmp | RCC_CFGR_SW_HSE;  // Select HSE as system clock src
            return WaitSWS(RCC_CFGR_SWS_HSE);
            break;

        case csPLL:
            if(EnableHSE() != retvOk) return 3;
            if(EnablePLL() != retvOk) return 4;
            RCC->CFGR = tmp | RCC_CFGR_SW_PLL; // Select PLL as system clock src
            return WaitSWS(RCC_CFGR_SWS_PLL);
            break;

#ifdef RCC_CFGR_SW_HSI48
        case csHSI48:
            if(EnableHSI48() != retvOk) return retvFail;
            RCC->CFGR = tmp | RCC_CFGR_SW_HSI48;
            return WaitSWS(RCC_CFGR_SWS_HSI48);
            break;
#endif
    } // switch
    return retvFail;
}

// Disable PLL first!
// HsePreDiv: 1...16; PllMul: pllMul[]
uint8_t Clk_t::SetupPLLDividers(uint8_t HsePreDiv, PllMul_t PllMul) {
    if(RCC->CR & RCC_CR_PLLON) return 1;    // PLL must be disabled to change dividers
    // Set HSE divider
    HsePreDiv--;
    if(HsePreDiv > 0x0F) HsePreDiv = 0x0F;
    uint32_t tmp = RCC->CFGR2;
    tmp &= ~RCC_CFGR2_PREDIV;
    tmp |= HsePreDiv;
    RCC->CFGR2 = tmp;
    // Setup PLL divider
    tmp = RCC->CFGR;
    tmp &= ~RCC_CFGR_PLLMUL;
    tmp |= ((uint32_t)PllMul) << 18;
    RCC->CFGR = tmp;
    return 0;
}

void Clk_t::SetupPLLSrc(PllSrc_t Src) {
    if(Src == plsHSIdiv2) RCC->CFGR &= ~RCC_CFGR_PLLSRC;
    else RCC->CFGR |= RCC_CFGR_PLLSRC;
}

// Setup Flash latency depending on CPU freq. Page 60 of ref manual.
// Call after UpdateFreqValues.
void Clk_t::SetupFlashLatency(uint32_t FrequencyHz) {
    uint32_t tmp = FLASH->ACR;
    if(FrequencyHz <= 24000000) tmp &= ~FLASH_ACR_LATENCY;
    else tmp |= FLASH_ACR_LATENCY;
    FLASH->ACR = tmp;
}

void Clk_t::PrintFreqs() {
    Printf("AHBFreq=%uMHz; APBFreq=%uMHz\r", Clk.AHBFreqHz/1000000, Clk.APBFreqHz/1000000);
}

#ifdef RCC_CFGR_SW_HSI48
void Clk_t::EnableCRS() {
    RCC->APB1ENR |= RCC_APB1ENR_CRSEN;      // Enable CRS clocking
    RCC->APB1RSTR |= RCC_APB1RSTR_CRSRST;   // }
    RCC->APB1RSTR &= ~RCC_APB1RSTR_CRSRST;  // } Reset CRS
    // Configure Synchronization input
    // Clear SYNCDIV[2:0], SYNCSRC[1:0] & SYNCSPOL bits
    CRS->CFGR &= ~(CRS_CFGR_SYNCDIV | CRS_CFGR_SYNCSRC | CRS_CFGR_SYNCPOL);
    // Configure CRS prescaler, source & polarity
    CRS->CFGR |= (CRS_PRESCALER | CRS_SOURCE | CRS_POLARITY);
    // Configure Frequency Error Measurement
    CRS->CFGR &= ~(CRS_CFGR_RELOAD | CRS_CFGR_FELIM);
    CRS->CFGR |= (CRS_RELOAD_VAL | (CRS_ERROR_LIMIT << 16));
    // Adjust HSI48 oscillator smooth trimming
    CRS->CR &= ~CRS_CR_TRIM;
    CRS->CR |= (HSI48_CALIBRATN << 8);
    // Enable auto trimming
    CRS->CR |= CRS_CR_AUTOTRIMEN;
    // Setup USB clock source = HSI48
    RCC->CFGR3 &= ~RCC_CFGR3_USBSW;
    // Enable Frequency error counter
    CRS->CR |= CRS_CR_CEN;
}

void Clk_t::DisableCRS() {
    CRS->CR &= ~CRS_CR_CEN;
    RCC->APB1ENR &= ~RCC_APB1ENR_CRSEN;
    // Setup USB clock source = PLLCLK to allow switch off HSI48
    RCC->CFGR3 |= RCC_CFGR3_USBSW;
}

void Clk_t::SwitchToHsi48() {
    ISavedAhbDividers = GetAhbApbDividers();
//    Uart.PrintfNow("cr21=%X\r", RCC->CR2);
    IHsi48WasOn = IsHSI48On();
    chSysLock();
    SetupFlashLatency(48000000);
    SetupBusDividers(ahbDiv1, apbDiv1);
    if(!IHsi48WasOn) SwitchTo(csHSI48);  // Switch HSI48 on if was off
    UpdateFreqValues();
    chSysUnlock();
}
#endif

void Clk_t::SwitchToHsi() {
    chSysLock();
    SetupBusDividers(ISavedAhbDividers);
#ifdef RCC_CFGR_SW_HSI48
    if(!IHsi48WasOn) {    // Switch hsi48 off if was off
        SwitchTo(csHSI);
        DisableHSI48();
    }
#endif
    UpdateFreqValues();
    SetupFlashLatency(AHBFreqHz);
    chSysUnlock();
}

/*
 * Early initialization code.
 * This initialization must be performed just after stack setup and before
 * any other initialization.
 */
void __early_init(void) {
    // Enable HSI. It is enabled by default, but who knows.
    RCC->CR |= RCC_CR_HSION;
    while(!(RCC->CR & RCC_CR_HSIRDY));
    // SYSCFG clock enabled here because it is a multi-functional unit
    // shared among multiple drivers using external IRQs
    // DMA depends on it, too
    rccEnableAPB2(RCC_APB2ENR_SYSCFGEN, 1);
}

#elif defined STM32F40_41xxx || defined STM32F2XX
uint8_t Clk_t::EnableHSE() {
    RCC->CR |= RCC_CR_HSEON;    // Enable HSE
    // Wait until ready
    uint32_t StartUpCounter=0;
    do {
        if(RCC->CR & RCC_CR_HSERDY) return retvOk;   // HSE is ready
        StartUpCounter++;
    } while(StartUpCounter < CLK_STARTUP_TIMEOUT);
    return retvTimeout;
}

uint8_t Clk_t::EnableHSI() {
    RCC->CR |= RCC_CR_HSION;
    // Wait until ready
    uint32_t StartUpCounter=0;
    do {
        if(RCC->CR & RCC_CR_HSIRDY) return retvOk;   // HSE is ready
        StartUpCounter++;
    } while(StartUpCounter < CLK_STARTUP_TIMEOUT);
    return retvTimeout;
}

uint8_t Clk_t::EnablePLL() {
    RCC->CR |= RCC_CR_PLLON;
    // Wait until ready
    uint32_t StartUpCounter=0;
    do {
        if(RCC->CR & RCC_CR_PLLRDY) return retvOk;   // PLL is ready
        StartUpCounter++;
    } while(StartUpCounter < CLK_STARTUP_TIMEOUT);
    return retvTimeout;
}

void Clk_t::EnableLsi() {
    RCC->CSR |= RCC_CSR_LSION;
    while ((RCC->CSR & RCC_CSR_LSIRDY) == 0);
}

void Clk_t::UpdateFreqValues() {
    uint32_t tmp, pllvco=0, InputDiv_M, Multi_N, SysDiv_P;
    uint32_t SysFreqHz;     // SYSCLK, 168 MHz max, used for Ethernet PTP clk
    // Figure out SysClk
    tmp = RCC->CFGR & RCC_CFGR_SWS;
    switch(tmp) {
        case 0x04: // HSE
            SysFreqHz = CRYSTAL_FREQ_HZ;
            break;

        case 0x08: // PLL used as system clock source
            // Get different PLL dividers
            InputDiv_M = RCC->PLLCFGR & RCC_PLLCFGR_PLLM;
            Multi_N    = (RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6;
            SysDiv_P   = (((RCC->PLLCFGR & RCC_PLLCFGR_PLLP) >> 16) + 1 ) * 2;
            // Calculate pll freq
            pllvco = (RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC_HSE)? CRYSTAL_FREQ_HZ : HSI_FREQ_HZ;
            pllvco = (pllvco / InputDiv_M) * Multi_N;
            SysFreqHz = pllvco / SysDiv_P;
            break;

        default: // HSI
            SysFreqHz = HSI_FREQ_HZ;
            break;
    } // switch

    // AHB freq
    const uint8_t AHBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
    tmp = AHBPrescTable[((RCC->CFGR & RCC_CFGR_HPRE) >> 4)];
    AHBFreqHz = SysFreqHz >> tmp;
    // APB freqs
    const uint8_t APBPrescTable[8] = {0, 0, 0, 0, 1, 2, 3, 4};
    tmp = APBPrescTable[(RCC->CFGR & RCC_CFGR_PPRE1) >> 10];
    APB1FreqHz = AHBFreqHz >> tmp;
    tmp = APBPrescTable[(RCC->CFGR & RCC_CFGR_PPRE2) >> 13];
    APB2FreqHz = AHBFreqHz >> tmp;

    // ==== USB and SDIO freq ====
//    UsbSdioFreqHz = 0;      // Will be changed only in case of PLL enabled
//    if(RCC->CR & RCC_CR_PLLON) {
//        // Get different PLL dividers
//        InputDiv_M = RCC->PLLCFGR & RCC_PLLCFGR_PLLM;
//        Multi_N    = (RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6;
//        uint32_t SysDiv_Q = (RCC->PLLCFGR & RCC_PLLCFGR_PLLQ) >> 24;
//        // Calculate pll freq
//        pllvco = (RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC_HSE)? CRYSTAL_FREQ_HZ : HSI_FREQ_HZ;
//        pllvco = (pllvco / InputDiv_M) * Multi_N;
//        if(SysDiv_Q >= 2) UsbSdioFreqHz = pllvco / SysDiv_Q;
//    }

    // ==== Update prescaler in System Timer ====
    TMR_DISABLE(STM32_ST_TIM);          // Stop system counter
    uint32_t Psc = (SYS_TIM_CLK / OSAL_ST_FREQUENCY) - 1;
    uint32_t Cnt = STM32_ST_TIM->CNT;   // Save current time
    STM32_ST_TIM->PSC = Psc;
    TMR_GENERATE_UPD(STM32_ST_TIM);
    __NOP(); __NOP(); __NOP(); __NOP(); // Let it to update in peace
    STM32_ST_TIM->CNT = Cnt;            // Restore time
    TMR_ENABLE(STM32_ST_TIM);
}

uint32_t Clk_t::GetTimInputFreq(TIM_TypeDef* ITmr) {
    uint32_t InputFreq = 0;
    if(ANY_OF_5(ITmr, TIM1, TIM8, TIM9, TIM10, TIM11)) {    // APB2
        uint32_t APB2prs = (RCC->CFGR & RCC_CFGR_PPRE2) >> 13;
        if(APB2prs < 0b100) InputFreq = Clk.APB2FreqHz; // APB2CLK = HCLK / 1
        else  InputFreq = Clk.APB2FreqHz * 2;           // APB2CLK = HCLK / (not 1)
    }
    else {                                              // APB1
        uint32_t APB1prs = (RCC->CFGR & RCC_CFGR_PPRE1) >> 10;
        if(APB1prs < 0b100) InputFreq = Clk.APB1FreqHz; // APB1CLK = HCLK / 1
        else  InputFreq = Clk.APB1FreqHz * 2;           // APB1CLK = HCLK / (not 1)
    }
    return InputFreq;
}

// ==== Common use ====
// AHB, APB1, APB2
void Clk_t::SetupBusDividers(AHBDiv_t AHBDiv, APBDiv_t APB1Div, APBDiv_t APB2Div) {
    // Setup dividers
    uint32_t tmp = RCC->CFGR;
    tmp &= ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2);  // Clear bits
    tmp |= ((uint32_t)AHBDiv)  << 4;
    tmp |= ((uint32_t)APB1Div) << 10;
    tmp |= ((uint32_t)APB2Div) << 13;
    RCC->CFGR = tmp;
}

// Enables HSI, switches to HSI
uint8_t Clk_t::SwitchToHSI() {
    if(EnableHSI() != 0) return 1;
    RCC->CFGR &= ~RCC_CFGR_SW;      // }
    RCC->CFGR |=  RCC_CFGR_SW_HSI;  // } Select HSI as system clock src
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI); // Wait till ready
    return 0;
}

// Enables HSE, switches to HSE
uint8_t Clk_t::SwitchToHSE() {
    if(EnableHSE() != 0) return 1;
    RCC->CFGR &= ~RCC_CFGR_SW;      // }
    RCC->CFGR |=  RCC_CFGR_SW_HSE;  // } Select HSE as system clock src
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSE); // Wait till ready
    return 0;
}

// Enables HSE, enables PLL, switches to PLL
uint8_t Clk_t::SwitchToPLL() {
    if(EnableHSE() != 0) return 1;
    if(EnablePLL() != 0) return 2;
    RCC->CFGR &= ~RCC_CFGR_SW;          // }
    RCC->CFGR |=  RCC_CFGR_SW_PLL;      // } Select PLL as system clock src
    while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL); // Wait until ready
    return 0;
}

// Disable PLL first!
// InputDiv_M: 2...63;  Multi_N:  2...432;
// SysDiv_P: sd2,4,6,8; UsbDiv_Q: 2...15.
uint8_t Clk_t::SetupPllMulDiv(uint8_t InputDiv_M, uint16_t Multi_N, PllSysDiv_P_t SysDiv_P, uint8_t UsbDiv_Q) {
    if(RCC->CR & RCC_CR_PLLON) return 1;    // PLL must be disabled to change dividers
    RCC->PLLCFGR =
            RCC_PLLCFGR_PLLSRC_HSE |        // Use only HSE as src
            ((uint32_t)InputDiv_M << 0) |
            ((uint32_t)Multi_N  << 6) |
            ((uint32_t)SysDiv_P << 16) |
            ((uint32_t)UsbDiv_Q << 24);
    return 0;
}

// Setup Flash latency depending on CPU freq and voltage. Page 54 of ref manual.
uint8_t Clk_t::SetupFlashLatency(uint8_t AHBClk_MHz, uint16_t Voltage_mV) {
    uint32_t tmp = FLASH->ACR;
    tmp &= ~FLASH_ACR_LATENCY;  // Clear Latency bits
    tmp |= FLASH_ACR_ICEN | FLASH_ACR_DCEN; // Enable instruction & data prefetch by ART
    if((2700 < Voltage_mV) and (Voltage_mV <= 3600)) {
        if     (AHBClk_MHz <=  30) tmp |= FLASH_ACR_LATENCY_0WS;
        else if(AHBClk_MHz <=  60) tmp |= FLASH_ACR_LATENCY_1WS;
        else if(AHBClk_MHz <=  90) tmp |= FLASH_ACR_LATENCY_2WS;
        else if(AHBClk_MHz <= 120) tmp |= FLASH_ACR_LATENCY_3WS;
        else if(AHBClk_MHz <= 150) tmp |= FLASH_ACR_LATENCY_4WS;
        else                       tmp |= FLASH_ACR_LATENCY_5WS;
    }
    else if((2400 < Voltage_mV) and (Voltage_mV <= 2700)) {
        if     (AHBClk_MHz <= 24) tmp |= FLASH_ACR_LATENCY_0WS;
        else if(AHBClk_MHz <= 48) tmp |= FLASH_ACR_LATENCY_1WS;
        else if(AHBClk_MHz <= 72) tmp |= FLASH_ACR_LATENCY_2WS;
        else if(AHBClk_MHz <= 96) tmp |= FLASH_ACR_LATENCY_3WS;
        else                      tmp |= FLASH_ACR_LATENCY_4WS;
    }
    else if((2100 < Voltage_mV) and (Voltage_mV <= 2400)) {
        if     (AHBClk_MHz <= 18) tmp |= FLASH_ACR_LATENCY_0WS;
        else if(AHBClk_MHz <= 36) tmp |= FLASH_ACR_LATENCY_1WS;
        else if(AHBClk_MHz <= 54) tmp |= FLASH_ACR_LATENCY_2WS;
        else if(AHBClk_MHz <= 72) tmp |= FLASH_ACR_LATENCY_3WS;
        else if(AHBClk_MHz <= 90) tmp |= FLASH_ACR_LATENCY_4WS;
        else if(AHBClk_MHz <=108) tmp |= FLASH_ACR_LATENCY_5WS;
        else                      tmp |= FLASH_ACR_LATENCY_6WS;
    }
    else if((1650 < Voltage_mV) and (Voltage_mV <= 2100)) {
        if     (AHBClk_MHz <= 16) tmp |= FLASH_ACR_LATENCY_0WS;
        else if(AHBClk_MHz <= 32) tmp |= FLASH_ACR_LATENCY_1WS;
        else if(AHBClk_MHz <= 48) tmp |= FLASH_ACR_LATENCY_2WS;
        else if(AHBClk_MHz <= 64) tmp |= FLASH_ACR_LATENCY_3WS;
        else if(AHBClk_MHz <= 80) tmp |= FLASH_ACR_LATENCY_4WS;
        else if(AHBClk_MHz <= 96) tmp |= FLASH_ACR_LATENCY_5WS;
        else if(AHBClk_MHz <=112) tmp |= FLASH_ACR_LATENCY_6WS;
        else                      tmp |= FLASH_ACR_LATENCY_7WS;
    }
    else return 1;

    FLASH->ACR = tmp;
    return 0;
}

void Clk_t::EnableMCO1(Mco1Src_t Src, McoDiv_t Div) {
    PinSetupAlterFunc(GPIOA, 8, omPushPull, pudNone, AF0, psHigh);
    RCC->CFGR &= ~(RCC_CFGR_MCO1 | RCC_CFGR_MCO1PRE);   // First, disable output and clear settings
    RCC->CFGR |= ((uint32_t)Src) | ((uint32_t)Div << 24);
}
void Clk_t::DisableMCO1() {
    PinSetupAnalog(GPIOA, 8);
    RCC->CFGR &= ~(RCC_CFGR_MCO1 | RCC_CFGR_MCO1PRE);
}
void Clk_t::EnableMCO2(Mco2Src_t Src, McoDiv_t Div) {
    PinSetupAlterFunc(GPIOC, 9, omPushPull, pudNone, AF0, psHigh);
    RCC->CFGR &= ~(RCC_CFGR_MCO2 | RCC_CFGR_MCO2PRE);   // First, disable output and clear settings
    RCC->CFGR |= ((uint32_t)Src) | ((uint32_t)Div << 27);
}
void Clk_t::DisableMCO2() {
    PinSetupAnalog(GPIOC, 9);
    RCC->CFGR &= ~(RCC_CFGR_MCO2 | RCC_CFGR_MCO2PRE);
}

void Clk_t::PrintFreqs() {
    Printf(
            "AHBFreq=%uMHz; APB1Freq=%uMHz; APB2Freq=%uMHz\r",
            Clk.AHBFreqHz/1000000, Clk.APB1FreqHz/1000000, Clk.APB2FreqHz/1000000);
}

void Clk_t::SetCoreClk(CoreClk_t CoreClk) {
    EnablePrefetch();
    // Enable HSE
    if(CoreClk >= cclk16MHz) {
        if(EnableHSE() != retvOk) return;   // Try to enable HSE
        DisablePLL();
    }
    // Setup dividers
    switch(CoreClk) {
        case cclk8MHz:
        case cclk12MHz:
        case cclk64MHz:
        case cclk80MHz:
            break;
        // Setup PLL (must be disabled first)
        case cclk16MHz:
            // 12MHz / 6 * 192 / (6 and 8) => 64 and 48MHz
            if(SetupPllMulDiv(6, 192, pllSysDiv6, 8) != retvOk) return;
            SetupBusDividers(ahbDiv4, apbDiv1, apbDiv1); // 16 MHz AHB, 16 MHz APB1, 16 MHz APB2
            SetupFlashLatency(16);
            break;
        case cclk24MHz:
            // 12MHz / 6 * 192 / (8 and 8) => 48 and 48MHz
            if(SetupPllMulDiv(6, 192, pllSysDiv8, 8) != retvOk) return;
            SetupBusDividers(ahbDiv2, apbDiv1, apbDiv1); // 24 MHz AHB, 24 MHz APB1, 24 MHz APB2
            SetupFlashLatency(24);
            break;
        case cclk48MHz:
            // 12MHz / 6 * 192 / (8 and 8) => 48 and 48MHz
            if(SetupPllMulDiv(6, 192, pllSysDiv8, 8) != retvOk) return;
            SetupBusDividers(ahbDiv1, apbDiv2, apbDiv2); // 48 MHz AHB, 24 MHz APB1, 24 MHz APB2
            SetupFlashLatency(48);
            break;
        case cclk72MHz:
            // 12MHz / 12 * 288 / (4 and 6) => 72 and 48MHz
            if(SetupPllMulDiv(12, 288, pllSysDiv4, 6) != retvOk) return;
            SetupBusDividers(ahbDiv1, apbDiv4, apbDiv2); // 72 MHz AHB, 18 MHz APB1, 36 MHz APB2
            SetupFlashLatency(48);
            break;
    } // switch

    if(CoreClk >= cclk16MHz) {
        if(EnablePLL() == retvOk) SwitchToPLL();
    }
}




/*
 * Early initialization code.
 * This initialization must be performed just after stack setup and before
 * any other initialization.
 */
void __early_init(void) {
    RCC->APB1ENR = RCC_APB1ENR_PWREN;   // PWR clock enable
    PWR->CR = 0;                        // PWR initialization

    // Enable HSI. It is enabled by default, but who knows.
    RCC->CR |= RCC_CR_HSION;
    while(!(RCC->CR & RCC_CR_HSIRDY));

    // SYSCFG clock enabled here because it is a multi-functional unit
    // shared among multiple drivers using external IRQs
    rccEnableAPB2(RCC_APB2ENR_SYSCFGEN, 1);
}
#elif defined STM32L4XX // =====================================================
const uint8_t AHBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
const uint8_t APBPrescTable[8] = {0, 0, 0, 0, 1, 2, 3, 4};
const uint32_t MSIRangeTable[12] = {
        100000, 200000, 400000, 800000, 1000000, 2000000,
        4000000, 8000000, 16000000, 24000000, 32000000, 48000000};

uint32_t Clk_t::GetSysClkHz() {
    uint32_t tmp, MSIRange;
    // ==== Get MSI Range frequency ====
    if((RCC->CR & RCC_CR_MSIRGSEL) == 0) tmp = (RCC->CSR & RCC_CSR_MSISRANGE) >> 8;  // MSISRANGE from RCC_CSR applies
    else tmp = (RCC->CR & RCC_CR_MSIRANGE) >> 4;    // MSIRANGE from RCC_CR applies
    MSIRange = MSIRangeTable[tmp];                  // MSI frequency range in Hz

    tmp = (RCC->CFGR & RCC_CFGR_SWS) >> 2;  // System clock switch status
    switch(tmp) {
        case 0b00: return MSIRange; break; // MSI
        case 0b01: return HSI_FREQ_HZ; break; // HSI
        case 0b10: return CRYSTAL_FREQ_HZ; break;// HSE
        case 0b11: {
            uint32_t PllSrc, PllM, PllR, PllVCO;
            /* PLL used as system clock source
             * PLL_VCO = (CRYSTAL_FREQ_HZ or HSI_FREQ_HZ or MSI_FREQ_HZ / PLLM) * PLLN
             * SYSCLK = PLL_VCO / PLLR */
            PllSrc = (RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC);
            PllM = ((RCC->PLLCFGR & RCC_PLLCFGR_PLLM) >> 4) + 1 ;
            switch(PllSrc) {
                case 0x02:  // HSI used as PLL clock source
                    PllVCO = (HSI_FREQ_HZ / PllM);
                    break;
                case 0x03:  // HSE used as PLL clock source
                    PllVCO = (CRYSTAL_FREQ_HZ / PllM);
                    break;
                default:    // MSI used as PLL clock source
                    PllVCO = (MSIRange / PllM);
                    break;
            } // switch(PllSrc)
            PllVCO *= ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 8);
            PllR = (((RCC->PLLCFGR & RCC_PLLCFGR_PLLR) >> 25) + 1) * 2;
            return PllVCO / PllR;
        } break;
        default: return 0; break;
    } // switch
}


void Clk_t::UpdateFreqValues() {
    // AHB freq
    uint32_t tmp = AHBPrescTable[((RCC->CFGR & RCC_CFGR_HPRE) >> 4)];
    AHBFreqHz = GetSysClkHz() >> tmp;
    // APB freq
    uint32_t APB1prs = (RCC->CFGR & RCC_CFGR_PPRE1) >> 8;
    uint32_t APB2prs = (RCC->CFGR & RCC_CFGR_PPRE2) >> 11;
    tmp = APBPrescTable[APB1prs];
    APB1FreqHz = AHBFreqHz >> tmp;
    tmp = APBPrescTable[APB2prs];
    APB2FreqHz = AHBFreqHz >> tmp;

    // ==== Update prescaler in System Timer ====
    uint32_t Psc = (SYS_TIM_CLK / OSAL_ST_FREQUENCY) - 1;
    TMR_DISABLE(STM32_ST_TIM);          // Stop counter
    uint32_t Cnt = STM32_ST_TIM->CNT;   // Save current time
    STM32_ST_TIM->PSC = Psc;
    TMR_GENERATE_UPD(STM32_ST_TIM);
    STM32_ST_TIM->CNT = Cnt;            // Restore time
    TMR_ENABLE(STM32_ST_TIM);
}

void Clk_t::PrintFreqs() {
    Printf(
            "AHBFreq=%uMHz; APB1Freq=%uMHz; APB2Freq=%uMHz\r",
            AHBFreqHz/1000000, APB1FreqHz/1000000, APB2FreqHz/1000000);
}

uint32_t Clk_t::GetTimInputFreq(TIM_TypeDef* ITmr) {
    uint32_t InputFreq = 0;
    if(
            ITmr == TIM1
#ifdef TIM8
            or ITmr == TIM8
#endif
            or ITmr == TIM15
            or ITmr == TIM16
#ifdef TIM17
            or ITmr == TIM17
#endif
            ) {   // APB2
        uint32_t APB2prs = (RCC->CFGR & RCC_CFGR_PPRE2) >> 11;
        if(APB2prs < 0b100) InputFreq = Clk.APB2FreqHz; // APB2CLK = HCLK / 1
        else InputFreq = Clk.APB2FreqHz * 2;            // APB2CLK = HCLK / (not 1)
    }
    else { // LPTIM1 & 2 included                                             // APB1
        uint32_t APB1prs = (RCC->CFGR & RCC_CFGR_PPRE1) >> 8;
        LPTIM_TypeDef* ILPTim = (LPTIM_TypeDef*)ITmr;
        if(ILPTim == LPTIM1 or ILPTim == LPTIM2) InputFreq = Clk.APB1FreqHz;
        else {
            if(APB1prs < 0b100) InputFreq = Clk.APB1FreqHz; // APB1CLK = HCLK / 1
            else InputFreq = Clk.APB1FreqHz * 2;            // APB1CLK = HCLK / (not 1)
        }
    }
    return InputFreq;
}

// AHB, APB1, APB2
void Clk_t::SetupBusDividers(AHBDiv_t AHBDiv, APBDiv_t APB1Div, APBDiv_t APB2Div) {
    // Setup dividers
    uint32_t tmp = RCC->CFGR;
    tmp &= ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2);  // Clear bits
    tmp |= ((uint32_t)AHBDiv)  << 4;
    tmp |= ((uint32_t)APB1Div) << 8;
    tmp |= ((uint32_t)APB2Div) << 11;
    RCC->CFGR = tmp;
}

// Setup Flash latency depending on CPU freq and voltage. Page 54 of ref manual.
void Clk_t::SetupFlashLatency(uint8_t AHBClk_MHz, MCUVoltRange_t VoltRange) {
    uint32_t tmp = FLASH->ACR;
    tmp &= ~FLASH_ACR_LATENCY;  // Clear Latency bits
    tmp |= FLASH_ACR_ICEN | FLASH_ACR_DCEN; // Enable instruction & data prefetch by ART
    if(VoltRange == mvrHiPerf) {
        if     (AHBClk_MHz <= 16) tmp |= FLASH_ACR_LATENCY_0WS;
        else if(AHBClk_MHz <= 32) tmp |= FLASH_ACR_LATENCY_1WS;
        else if(AHBClk_MHz <= 48) tmp |= FLASH_ACR_LATENCY_2WS;
        else if(AHBClk_MHz <= 64) tmp |= FLASH_ACR_LATENCY_3WS;
        else                      tmp |= FLASH_ACR_LATENCY_4WS;
    }
    else { // Low perfomance
        if     (AHBClk_MHz <=  6) tmp |= FLASH_ACR_LATENCY_0WS;
        else if(AHBClk_MHz <= 12) tmp |= FLASH_ACR_LATENCY_1WS;
        else if(AHBClk_MHz <= 18) tmp |= FLASH_ACR_LATENCY_2WS;
        else if(AHBClk_MHz <= 26) tmp |= FLASH_ACR_LATENCY_3WS;
        else                      tmp |= FLASH_ACR_LATENCY_4WS;
    }
    FLASH->ACR = tmp;
//    while(FLASH->ACR != tmp);
}

void Clk_t::SetVoltageRange(MCUVoltRange_t VoltRange) {
    uint32_t tmp = PWR->CR1;
    tmp &= ~PWR_CR1_VOS;
    if(VoltRange == mvrHiPerf) tmp |= (0b01 << 9);
    else tmp |= (0b10 << 9);
    PWR->CR1 = tmp;
}

// M: [1; 8]
uint8_t Clk_t::SetupM(uint32_t M) {
    if(M < 1 or M > 8) return retvBadValue;
    uint32_t tmp = RCC->PLLCFGR;
    tmp &= ~RCC_PLLCFGR_PLLM;
    tmp |= (M - 1) << 4;
    RCC->PLLCFGR = tmp;
    return retvOk;
}

void Clk_t::SetupPllSrc(PllSrc_t PllSrc) {
    uint32_t tmp = RCC->PLLCFGR;
    tmp &= ~RCC_PLLCFGR_PLLSRC;
    tmp |= (uint32_t)PllSrc;
    RCC->PLLCFGR = tmp;
}

PllSrc_t Clk_t::GetPllSrc() {
    uint32_t tmp = RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC;
    return (PllSrc_t)tmp;
}

// M: 1...8; N: 8...86; R: 2,4,6,8
uint8_t Clk_t::SetupPll(uint32_t N, uint32_t R, uint32_t Q) {
    if(!((N >= 8 and N <= 86) and (R == 2 or R == 4 or R == 6 or R == 8))) return retvBadValue;
    if(RCC->CR & RCC_CR_PLLON) return retvBusy; // PLL must be disabled to change dividers
    R = (R / 2) - 1;    // 2,4,6,8 => 0,1,2,3
    Q = (Q / 2) - 1;    // 2,4,6,8 => 0,1,2,3
    uint32_t tmp = RCC->PLLCFGR;
    tmp &= ~(RCC_PLLCFGR_PLLR | RCC_PLLCFGR_PLLREN | RCC_PLLCFGR_PLLQ | RCC_PLLCFGR_PLLQEN |
            RCC_PLLCFGR_PLLP | RCC_PLLCFGR_PLLPEN | RCC_PLLCFGR_PLLN);
    tmp |=  (N << 8) |
            (R << 25) |
            (Q << 21);
    RCC->PLLCFGR = tmp;
    return retvOk;
}

void Clk_t::SetupPllSai1(uint32_t N, uint32_t R, uint32_t Q, uint32_t P) {
    MODIFY_REG(RCC->PLLSAI1CFGR, RCC_PLLSAI1CFGR_PLLSAI1N, N << 8);
    MODIFY_REG(RCC->PLLSAI1CFGR, RCC_PLLSAI1CFGR_PLLSAI1R, ((R >> 1U) - 1U) << 25);
    MODIFY_REG(RCC->PLLSAI1CFGR, RCC_PLLSAI1CFGR_PLLSAI1Q, ((Q >> 1U) - 1U) << 21);
    if(P == 7) CLEAR_BIT(RCC->PLLSAI1CFGR, RCC_PLLSAI1CFGR_PLLSAI1P);
    else SET_BIT(RCC->PLLSAI1CFGR, RCC_PLLSAI1CFGR_PLLSAI1P);
}

void Clk_t::SetupPllSai2(uint32_t N, uint32_t R, uint32_t P) {
    MODIFY_REG(RCC->PLLSAI2CFGR, RCC_PLLSAI2CFGR_PLLSAI2N, N << 8);
    MODIFY_REG(RCC->PLLSAI2CFGR, RCC_PLLSAI2CFGR_PLLSAI2R, ((R >> 1U) - 1U) << 25);
    if(P == 7) CLEAR_BIT(RCC->PLLSAI2CFGR, RCC_PLLSAI2CFGR_PLLSAI2P);
    else SET_BIT(RCC->PLLSAI2CFGR, RCC_PLLSAI2CFGR_PLLSAI2P);
}

void Clk_t::SetupSai1Qas48MhzSrc() {
    uint32_t tmp = RCC->CCIPR;
    tmp &= ~RCC_CCIPR_CLK48SEL;
    tmp |= ((uint32_t)src48PllSai1Q) << 26;
    RCC->CCIPR = tmp;
}

void Clk_t::SetupSai1Qas48MhzSrcWidhADC() {
    uint32_t tmp = RCC->CCIPR;
    tmp &= ~RCC_CCIPR_ADCSEL;
    tmp |= ((uint32_t)src48PllSai1Q) << 28; // SAI1R is ADC clock
    tmp &= ~RCC_CCIPR_CLK48SEL;
    tmp |= ((uint32_t)src48PllSai1Q) << 26;
    RCC->CCIPR = tmp;
}

void Clk_t::SetupPllQas48MhzSrc() {
    uint32_t tmp = RCC->CCIPR;
    tmp &= ~RCC_CCIPR_CLK48SEL;
    tmp |= ((uint32_t)src48PllQ) << 26;
    RCC->CCIPR = tmp;
}

// ==== Enable/Disable ====
uint8_t Clk_t::EnableHSI() {
    RCC->CR |= RCC_CR_HSION;
    // Wait until ready
    uint32_t StartUpCounter=0;
    do {
        if(RCC->CR & RCC_CR_HSIRDY) return retvOk;   // HSE is ready
        StartUpCounter++;
    } while(StartUpCounter < CLK_STARTUP_TIMEOUT);
    return retvTimeout;
}
uint8_t Clk_t::EnableHSE() {
    RCC->CR |= RCC_CR_HSEON;    // Enable HSE
    // Wait until ready
    uint32_t StartupCounter=0;
    do {
        if(RCC->CR & RCC_CR_HSERDY) return retvOk;   // HSE is ready
        StartupCounter++;
    } while(StartupCounter < CLK_STARTUP_TIMEOUT);
    return retvTimeout;
}
uint8_t Clk_t::EnablePLL() {
    RCC->CR |= RCC_CR_PLLON;
    // Wait until ready
    uint32_t StartUpCounter=0;
    do {
        if(RCC->CR & RCC_CR_PLLRDY) return retvOk;   // PLL is ready
        StartUpCounter++;
    } while(StartUpCounter < CLK_STARTUP_TIMEOUT);
    return retvTimeout;
}

void Clk_t::DisablePLL() {
    RCC->CR &= ~RCC_CR_PLLON;
    while(RCC->CR & RCC_CR_PLLRDY); // Wait until ready
}

uint8_t Clk_t::EnablePllSai1() {
    SET_BIT(RCC->CR, RCC_CR_PLLSAI1ON); // Enable SAI
    // Wait till PLLSAI1 is ready. May fail if PLL source disabled or not selected.
    uint32_t t = 45000;
    while(READ_BIT(RCC->CR, RCC_CR_PLLSAI1RDY) == 0) {
        if(t-- == 0) {
            Printf("Sai1On Timeout %X\r", RCC->CR);
            return retvFail;
        }
    }
    return retvOk;
}

uint8_t Clk_t::EnablePllSai2() {
    SET_BIT(RCC->CR, RCC_CR_PLLSAI2ON); // Enable SAI
    // Wait till PLLSAI1 is ready. May fail if PLL source disabled or not selected.
    uint32_t t = 45000;
    while(READ_BIT(RCC->CR, RCC_CR_PLLSAI2RDY) == 0) {
        if(t-- == 0) {
            Printf("Sai2On Timeout %X\r", RCC->CR);
            return retvFail;
        }
    }
    return retvOk;
}

void Clk_t::DisablePllSai1() {
    RCC->CR &= ~RCC_CR_PLLSAI1ON;
    while(RCC->CR & RCC_CR_PLLSAI1RDY); // Wait until ready
}
void Clk_t::DisablePllSai2() {
    RCC->CR &= ~RCC_CR_PLLSAI2ON;
    while(RCC->CR & RCC_CR_PLLSAI2RDY); // Wait until ready
}

uint8_t Clk_t::EnableMSI() {
    RCC->CR |= RCC_CR_MSION;
    // Wait until ready
    uint32_t StartUpCounter=0;
    do {
        if(RCC->CR & RCC_CR_MSIRDY) return retvOk;
        StartUpCounter++;
    } while(StartUpCounter < CLK_STARTUP_TIMEOUT);
    return retvTimeout;
}

// ==== Switch ====
// Enables HSE, switches to HSE
uint8_t Clk_t::SwitchToHSE() {
    if(EnableHSE() != 0) return 1;
    uint32_t tmp = RCC->CFGR;
    tmp &= ~RCC_CFGR_SW;
    tmp |= RCC_CFGR_SW_HSE;
    RCC->CFGR = tmp;
    while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSE); // Wait till ready
    return 0;
}

// Enables HSE, enables PLL, switches to PLL
uint8_t Clk_t::SwitchToPLL() {
    uint32_t tmp = RCC->CFGR;
    tmp &= ~RCC_CFGR_SW;
    tmp |= RCC_CFGR_SW_PLL;
    RCC->CFGR = tmp;
    while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL); // Wait until ready
    return retvOk;
}

uint8_t Clk_t::SwitchToMSI() {
    if(EnableMSI() != retvOk) return retvFail;
    uint32_t tmp = RCC->CFGR;
    tmp &= ~RCC_CFGR_SW;
    tmp |= RCC_CFGR_SW_MSI;
    RCC->CFGR = tmp;
    while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_MSI); // Wait until ready
    return retvOk;
}

uint32_t Clk_t::GetSaiClkHz() {
        uint32_t FreqRslt = 0;
        // Get PLL clk input
        uint32_t pllvco = CRYSTAL_FREQ_HZ;    // Change this if other clock used
        // Freq = Freq / PLLM
        pllvco /= (READ_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLM) >> POSITION_VAL(RCC_PLLCFGR_PLLM)) + 1U;
        uint32_t srcclk = READ_BIT(RCC->CCIPR, RCC_CCIPR_SAI1SEL);
        if(srcclk == RCC_CCIPR_SAI1SEL_1) {
            if(READ_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLPEN) != 0) {
                // Freq = pllvco * PLLN / PLLP
                uint32_t plln = READ_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLN) >> POSITION_VAL(RCC_PLLCFGR_PLLN);
                uint32_t pllp = 7U;
                if(READ_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLP) != 0) pllp = 17U;
                FreqRslt = (pllvco * plln) / pllp;
            }
        }
        else if(srcclk == 0) {  // RCC_SAI1CLKSOURCE_PLLSAI1 || RCC_SAI2CLKSOURCE_PLLSAI1
            if(READ_BIT(RCC->PLLSAI1CFGR, RCC_PLLSAI1CFGR_PLLSAI1PEN) != 0) {
                // Freq = pllvco * PLLSAI1N / PLLSAI1P
                uint32_t plln = READ_BIT(RCC->PLLSAI1CFGR, RCC_PLLSAI1CFGR_PLLSAI1N) >> POSITION_VAL(RCC_PLLSAI1CFGR_PLLSAI1N);
                uint32_t pllp = 7U;
                if(READ_BIT(RCC->PLLSAI1CFGR, RCC_PLLSAI1CFGR_PLLSAI1P) != RESET) pllp = 17U;
                FreqRslt = (pllvco * plln) / pllp;
            }
        }
        return FreqRslt;
    }

#elif defined STM32F7XX
const uint8_t AHBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
const uint8_t APBPrescTable[8] = {0, 0, 0, 0, 1, 2, 3, 4};

uint32_t Clk_t::GetSysClkHz() {
    uint32_t ClkSwitch = (RCC->CFGR & RCC_CFGR_SWS) >> 2;  // System clock switch status
    switch(ClkSwitch) {
        case 0b00: return HSI_FREQ_HZ; break; // HSI
        case 0b01: return CRYSTAL_FREQ_HZ; break;// HSE
        case 0b10: { // PLL
            /* PLL used as system clock source
             * PLL_VCO = (CRYSTAL_FREQ_HZ or HSI_FREQ_HZ) / PLLM * PLLN
             * SYSCLK = PLL_VCO / PLLP */
            uint32_t PllSrc = (RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC);
            uint32_t PllM = RCC->PLLCFGR & RCC_PLLCFGR_PLLM;
            uint32_t PllVCO = (PllSrc == 0)? (HSI_FREQ_HZ / PllM) : (CRYSTAL_FREQ_HZ / PllM);
            PllVCO *= ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6);
            uint32_t PllP = (((RCC->PLLCFGR & RCC_PLLCFGR_PLLP) >> 16) + 1) * 2;
            return PllVCO / PllP;
        } break;
        default: return 0; break;
    } // switch
}

void Clk_t::UpdateFreqValues() {
    // AHB freq
    uint32_t tmp = AHBPrescTable[((RCC->CFGR & RCC_CFGR_HPRE) >> 4)];
    AHBFreqHz = GetSysClkHz() >> tmp;
    // APB freq
    uint32_t APB1prs = (RCC->CFGR & RCC_CFGR_PPRE1) >> 10;
    uint32_t APB2prs = (RCC->CFGR & RCC_CFGR_PPRE2) >> 13;
    tmp = APBPrescTable[APB1prs];
    APB1FreqHz = AHBFreqHz >> tmp;
    tmp = APBPrescTable[APB2prs];
    APB2FreqHz = AHBFreqHz >> tmp;

    // ==== Update prescaler in System Timer ====
    uint32_t Psc = (SYS_TIM_CLK / OSAL_ST_FREQUENCY) - 1;
    TMR_DISABLE(STM32_ST_TIM);          // Stop counter
    uint32_t Cnt = STM32_ST_TIM->CNT;   // Save current time
    STM32_ST_TIM->PSC = Psc;
    TMR_GENERATE_UPD(STM32_ST_TIM);
    STM32_ST_TIM->CNT = Cnt;            // Restore time
    TMR_ENABLE(STM32_ST_TIM);
}

uint32_t Clk_t::GetTimInputFreq(TIM_TypeDef* ITmr) {
    uint32_t InputFreq = 0;
    // APB2
    if(ITmr == TIM1 or ITmr == TIM8 or ITmr == TIM9 or ITmr == TIM10 or ITmr == TIM11) {
        uint32_t APB2prs = (RCC->CFGR & RCC_CFGR_PPRE2) >> 13;
        if(RCC->DCKCFGR1 & RCC_DCKCFGR1_TIMPRE) {
            if(APB2prs == 1 or APB2prs == 2 or APB2prs == 4) InputFreq = AHBFreqHz;
            else InputFreq = Clk.APB2FreqHz * 4;
        }
        else {
            if(APB2prs == 1) InputFreq = Clk.APB2FreqHz;
            else InputFreq = Clk.APB2FreqHz * 2;
        }
    }
    // APB1
    else {
        LPTIM_TypeDef* ILPTim = (LPTIM_TypeDef*)ITmr;
        if(ILPTim == LPTIM1) {
            InputFreq = Clk.APB1FreqHz; // Others clock options are not implemented
        }
        else { // not a LPTIM
            uint32_t APB1prs = (RCC->CFGR & RCC_CFGR_PPRE1) >> 10;
            if(RCC->DCKCFGR1 & RCC_DCKCFGR1_TIMPRE) {
                if(APB1prs == 1 or APB1prs == 2 or APB1prs == 4) InputFreq = AHBFreqHz;
                else InputFreq = Clk.APB1FreqHz * 4;
            }
            else {
                if(APB1prs == 1) InputFreq = Clk.APB1FreqHz;
                else InputFreq = Clk.APB1FreqHz * 2;
            }
        }
    }
    return InputFreq;
}

#if 1 // ==== Clock setup ====
void Clk_t::SwitchToHSI() {
    if(EnableHSI() != retvOk) return;
    uint32_t tmp = RCC->CFGR;
    tmp &= ~RCC_CFGR_SW;
    tmp |= RCC_CFGR_SW_HSI;
    RCC->CFGR = tmp;
    while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI); // Wait until ready
}

void Clk_t::SwitchToPLL() {
    uint32_t tmp = RCC->CFGR;
    tmp &= ~RCC_CFGR_SW;
    tmp |= RCC_CFGR_SW_PLL;
    RCC->CFGR = tmp;
    while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL); // Wait until ready
}

void Clk_t::SetCoreClk80MHz() {
    EnablePrefetch();
    // First, switch to HSI if clock src is not HSI
    if((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI) SwitchToHSI();
    // Disable PLL and SAI, enable HSE
    DisablePLL();
    DisablePLLSai();
    DisablePLLI2S();
    if(EnableHSE() != retvOk) return;
    SetupPllSrc(pllsrcHse);
    SetVoltageScale(mvScale3);
    // Setup dividers
    if(AHBFreqHz < 80000000) SetupFlashLatency(80, 3300);
    // 12MHz / 6 = 2; 2 * 160 / 4 = 80; Q and R are don't care
    SetupPllMulDiv(6, 160, 4, 8, 2);
    SetupFlashLatency(80, 3300);
    // APB1 is 54MHz max, APB2 is 108MHz max
    SetupBusDividers(ahbDiv1, apbDiv2, apbDiv1);
    if(EnablePLL() == retvOk) SwitchToPLL();
}

// PLL_SAI output P used to produce 48MHz, it is selected as PLL48CLK
void Clk_t::Setup48Mhz() {
    // Get SAI input freq
    uint32_t InputFreq;
    uint32_t PllM = RCC->PLLCFGR & RCC_PLLCFGR_PLLM;
    // 0 is HSI, 1 is HSE
    if(RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) InputFreq = (CRYSTAL_FREQ_HZ / PllM);
    else InputFreq = (HSI_FREQ_HZ / PllM);

    // Setup PLLSai
    DisablePLLSai();
    switch(InputFreq) {
        case 2000000:  SetupPllSai(96, 4, 8, 2); break; // 2 * 96 / 4 = 48
        case 3000000:  SetupPllSai(32, 2, 2, 7); break; // 3 * 32 / 2 = 48
        case 4000000:  SetupPllSai(24, 2, 2, 7); break; // 4 * 24 / 2 = 48
        case 12000000: SetupPllSai( 8, 2, 2, 7); break; // 12 * 8 / 2 = 48
        default: return;
    }
    // Setup Sai1P as 48MHz source
    if(EnablePLLSai() == retvOk) {
        RCC->DCKCFGR2 &= ~RCC_DCKCFGR2_SDMMC2SEL; // 48MHz selected as SDMMC2 clk
        RCC->DCKCFGR2 &= ~RCC_DCKCFGR2_SDMMC1SEL; // 48MHz selected as SDMMC1 clk
        RCC->DCKCFGR2 |=  RCC_DCKCFGR2_CK48MSEL;  // 48MHz clk from PLLSAI selected
    }
}

// Scale3: f<=144Mhz; Scale2: 144<f<=169MHz; Scale1: 168<f<=216MHz
void Clk_t::SetVoltageScale(MCUVoltScale_t VoltScale) {
    uint32_t tmp = PWR->CR1;
    tmp &= ~PWR_CR1_VOS;
    tmp |= ((uint32_t)VoltScale) << 14;
    PWR->CR1 = tmp;
}

static const uint32_t FlashLatencyTbl[4][10] = {
        {30, 60, 90, 120, 150, 180, 210, 216}, // 2.7...3.6 V
        {24, 48, 72, 96, 120, 144, 168, 192, 216}, // 2.4...2.7 V
        {22, 44, 66, 88, 110, 132, 154, 176, 198, 216}, // 2.1...2.4 V
        {20, 40, 60, 80, 100, 120, 140, 160, 180}, // 1.8...2.1 V
};
void Clk_t::SetupFlashLatency(uint8_t AHBClk_MHz, uint32_t MCUVoltage_mv) {
    uint32_t VoltIndx;
    if(MCUVoltage_mv < 2100) VoltIndx = 3;
    else if(MCUVoltage_mv < 2400) VoltIndx = 2;
    else if(MCUVoltage_mv < 2700) VoltIndx = 1;
    else VoltIndx = 0;
    // Iterate freqs
    for(uint32_t i=0; i<10; i++) {
        if(AHBClk_MHz <= FlashLatencyTbl[VoltIndx][i]) {
            // Setup latency (which equals to i)
            uint32_t tmp = FLASH->ACR & ~FLASH_ACR_LATENCY;
            tmp |= i;
            FLASH->ACR |= tmp;
            return;
        }
    }
}

void Clk_t::SetupPllMulDiv(uint32_t M, uint32_t N, uint32_t P, uint32_t Q, uint32_t R) {
    if(RCC->CR & RCC_CR_PLLON) return; // PLL must be disabled to change dividers
    P = (P / 2) - 1;    // 2,4,6,8 => 0,1,2,3
    uint32_t tmp = RCC->PLLCFGR;
    tmp &= ~(RCC_PLLCFGR_PLLR | RCC_PLLCFGR_PLLQ | RCC_PLLCFGR_PLLP | RCC_PLLCFGR_PLLN | RCC_PLLCFGR_PLLM);
    tmp |= (M << 0) | (N << 6) | (P << 16) | (Q << 24) | (R << 28);
    RCC->PLLCFGR = tmp;
}

void Clk_t::SetupBusDividers(AHBDiv_t AHBDiv, APBDiv_t APB1Div, APBDiv_t APB2Div) {
    uint32_t tmp = RCC->CFGR;
    tmp &= ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2);  // Clear bits
    tmp |= ((uint32_t)AHBDiv)  << 4;
    tmp |= ((uint32_t)APB1Div) << 10;
    tmp |= ((uint32_t)APB2Div) << 13;
    RCC->CFGR = tmp;
}

void Clk_t::SetupPllSai(uint32_t N, uint32_t P, uint32_t Q, uint32_t R) {
    uint32_t tmp = RCC->PLLSAICFGR;
    tmp &= ~(RCC_PLLSAICFGR_PLLSAIR | RCC_PLLSAICFGR_PLLSAIQ | RCC_PLLSAICFGR_PLLSAIP | RCC_PLLSAICFGR_PLLSAIN);
    P = (P / 2) - 1;    // 2,4,6,8 => 0,1,2,3
    tmp |= (R << 28) | (Q << 24) | (P << 16) | (N << 6);
    RCC->PLLSAICFGR = tmp;
}
#endif

void Clk_t::PrintFreqs() {
    Printf("AHBFreq=%uMHz; APB1Freq=%uMHz; APB2Freq=%uMHz\r",
            AHBFreqHz/1000000, APB1FreqHz/1000000, APB2FreqHz/1000000);
}

#if 1 // ==== Enable/Disable ====
uint8_t Clk_t::EnableHSI() {
    RCC->CR |= RCC_CR_HSION;
    // Wait until ready
    uint32_t StartUpCounter=0;
    do {
        if(RCC->CR & RCC_CR_HSIRDY) return retvOk;
        StartUpCounter++;
    } while(StartUpCounter < CLK_STARTUP_TIMEOUT);
    return retvTimeout;
}

uint8_t Clk_t::EnableHSE() {
    RCC->CR |= RCC_CR_HSEON;    // Enable HSE
    // Wait until ready
    uint32_t StartupCounter=0;
    do {
        if(RCC->CR & RCC_CR_HSERDY) return retvOk;   // HSE is ready
        StartupCounter++;
    } while(StartupCounter < CLK_STARTUP_TIMEOUT);
    return retvTimeout;
}

uint8_t Clk_t::EnablePLL() {
    RCC->CR |= RCC_CR_PLLON;
    // Wait until ready
    uint32_t StartUpCounter=0;
    do {
        if(RCC->CR & RCC_CR_PLLRDY) return retvOk;   // PLL is ready
        StartUpCounter++;
    } while(StartUpCounter < CLK_STARTUP_TIMEOUT);
    return retvTimeout;
}

uint8_t Clk_t::EnablePLLSai() {
    RCC->CR |= RCC_CR_PLLSAION; // Enable SAI
    // Wait till PLLSAI1 is ready. May fail if PLL source disabled or not selected.
    uint32_t StartUpCounter=0;
    do {
        if(RCC->CR & RCC_CR_PLLSAIRDY) return retvOk;   // PLL is ready
        StartUpCounter++;
    } while(StartUpCounter < 45000);
    Printf("SaiRdy Timeout %X\r", RCC->CR);
    return retvTimeout;
}

void Clk_t::DisablePLL() {
    RCC->CR &= ~RCC_CR_PLLON;
    while(RCC->CR & RCC_CR_PLLRDY); // Wait until ready
}

void Clk_t::DisablePLLSai() {
    RCC->CR &= ~RCC_CR_PLLSAION;
    while(RCC->CR & RCC_CR_PLLSAIRDY); // Wait until ready
}
void Clk_t::DisablePLLI2S() {
    RCC->CR &= ~RCC_CR_PLLI2SON;
    while(RCC->CR & RCC_CR_PLLI2SRDY); // Wait until ready
}
#endif

#endif // MCU

#endif // Clocking

#if 1 // ================================= SPI =================================
void Spi_t::Setup(BitOrder_t BitOrder, CPOL_t CPOL, CPHA_t CPHA,
        int32_t Bitrate_Hz, BitNumber_t BitNumber) const {
    // Clocking
    if      (PSpi == SPI1) { rccEnableSPI1(FALSE); }
#ifdef SPI2
    else if (PSpi == SPI2) { rccEnableSPI2(FALSE); }
#endif
#ifdef SPI3
    else if (PSpi == SPI3) { rccEnableSPI3(FALSE); }
#endif
#ifdef SPI4
    else if (PSpi == SPI4) { rccEnableSPI4(FALSE); }
#endif
#ifdef SPI5
    else if (PSpi == SPI5) { rccEnableSPI5(FALSE); }
#endif
#ifdef SPI6
    else if (PSpi == SPI6) { rccEnableSPI6(FALSE); }
#endif
    // Mode: Master, NSS software controlled and is 1, 8bit, NoCRC, FullDuplex
    PSpi->CR1 = SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_MSTR;
    if(BitOrder == boLSB) PSpi->CR1 |= SPI_CR1_LSBFIRST;    // MSB/LSB
    if(CPOL == cpolIdleHigh) PSpi->CR1 |= SPI_CR1_CPOL;     // CPOL
    if(CPHA == cphaSecondEdge) PSpi->CR1 |= SPI_CR1_CPHA;   // CPHA
    // Baudrate
    int32_t div;
#if defined STM32L1XX || defined STM32F4XX || defined STM32F2XX || defined STM32L4XX || defined STM32F1XX
    if(PSpi == SPI1) div = Clk.APB2FreqHz / Bitrate_Hz;
    else div = Clk.APB1FreqHz / Bitrate_Hz;
#elif defined STM32F030 || defined STM32F0
    div = Clk.APBFreqHz / Bitrate_Hz;
#elif defined STM32F7XX
    if(PSpi == SPI2 or PSpi == SPI3) div = Clk.APB1FreqHz / Bitrate_Hz;
    else div = Clk.APB2FreqHz / Bitrate_Hz;
#else
#error "SPI div not defined"
#endif
    SpiClkDivider_t ClkDiv = sclkDiv2;
    if     (div > 128) ClkDiv = sclkDiv256;
    else if(div > 64) ClkDiv = sclkDiv128;
    else if(div > 32) ClkDiv = sclkDiv64;
    else if(div > 16) ClkDiv = sclkDiv32;
    else if(div > 8)  ClkDiv = sclkDiv16;
    else if(div > 4)  ClkDiv = sclkDiv8;
    else if(div > 2)  ClkDiv = sclkDiv4;
    PSpi->CR1 |= ((uint16_t)ClkDiv) << 3;
    // Bit number
#if defined STM32L1XX || defined STM32F10X_LD_VL || defined STM32F2XX || defined STM32F4XX
    if(BitNumber == bitn16) PSpi->CR1 |= SPI_CR1_DFF;
    PSpi->CR2 = 0;
#elif defined STM32F030 || defined STM32F072xB || defined STM32L4XX || defined STM32F7XX
    if(BitNumber == bitn16) PSpi->CR2 = (uint16_t)0b1111 << 8;  // 16 bit, RXNE generated when 16 bit is received
    else PSpi->CR2 = ((uint16_t)0b0111 << 8) | SPI_CR2_FRXTH;   // 8 bit, RXNE generated when 8 bit is received
#endif
}
void Spi_t::PrintFreq() const {
    uint32_t SpiFreqHz;
#if defined STM32L1XX || defined STM32F4XX || defined STM32F2XX || defined STM32L4XX || defined STM32F1XX
    if(PSpi == SPI1) SpiFreqHz = Clk.APB2FreqHz;
    else SpiFreqHz = Clk.APB1FreqHz;
#elif defined STM32F030 || defined STM32F0
    SpiFreqHz = Clk.APBFreqHz;
#elif defined STM32F7XX
    if(PSpi == SPI2 or PSpi == SPI3) SpiFreqHz = Clk.APB1FreqHz;
    else SpiFreqHz = Clk.APB2FreqHz;
#endif
    uint16_t ClkDiv = (PSpi->CR1 >> 3)&0b111;
    if (ClkDiv == sclkDiv2) SpiFreqHz /= 2;
    else if (ClkDiv == sclkDiv4) SpiFreqHz /= 4;
    else if (ClkDiv == sclkDiv8) SpiFreqHz /= 8;
    else if (ClkDiv == sclkDiv16) SpiFreqHz /= 16;
    else if (ClkDiv == sclkDiv32) SpiFreqHz /= 32;
    else if (ClkDiv == sclkDiv64) SpiFreqHz /= 64;
    else if (ClkDiv == sclkDiv128) SpiFreqHz /= 128;
    else if (ClkDiv == sclkDiv256) SpiFreqHz /= 256;
    Printf("SPI Freq=%uHz\r", SpiFreqHz);
}

// IRQs
static ftVoidVoid Spi1RxIrqHandler = nullptr;
#ifdef SPI2
static ftVoidVoid Spi2RxIrqHandler = nullptr;
#endif
#ifdef SPI3
static ftVoidVoid Spi3RxIrqHandler = nullptr;
#endif
#ifdef SPI4
static ftVoidVoid Spi4RxIrqHandler = nullptr;
#endif
#ifdef SPI5
static ftVoidVoid Spi5RxIrqHandler = nullptr;
#endif
#ifdef SPI6
static ftVoidVoid Spi6RxIrqHandler = nullptr;
#endif

void Spi_t::SetupRxIrqCallback(ftVoidVoid AIrqHandler) const {
    if(PSpi == SPI1) Spi1RxIrqHandler = AIrqHandler;
#ifdef SPI2
    else if(PSpi == SPI2) Spi2RxIrqHandler = AIrqHandler;
#endif
#ifdef SPI3
    else if(PSpi == SPI3) Spi3RxIrqHandler = AIrqHandler;
#endif
#ifdef SPI4
    else if(PSpi == SPI4) Spi4RxIrqHandler = AIrqHandler;
#endif
#ifdef SPI5
    else if(PSpi == SPI5) Spi5RxIrqHandler = AIrqHandler;
#endif
#ifdef SPI6
    else if(PSpi == SPI6) Spi6RxIrqHandler = AIrqHandler;
#endif
}

extern "C" {
void VectorCC() {   // SPI1
    CH_IRQ_PROLOGUE();
    chSysLockFromISR();
    uint32_t SR = SPI1->SR;
    if((SR & SPI_SR_RXNE) and Spi1RxIrqHandler) Spi1RxIrqHandler();
    chSysUnlockFromISR();
    CH_IRQ_EPILOGUE();
}

#ifdef SPI2
void VectorD0() {   // SPI2
    CH_IRQ_PROLOGUE();
    chSysLockFromISR();
    uint32_t SR = SPI2->SR;
    if((SR & SPI_SR_RXNE) and Spi2RxIrqHandler) Spi2RxIrqHandler();
    chSysUnlockFromISR();
    CH_IRQ_EPILOGUE();
}
#endif
#ifdef SPI3
void Vector10C() {   // SPI3
    CH_IRQ_PROLOGUE();
    chSysLockFromISR();
    uint32_t SR = SPI3->SR;
    if((SR & SPI_SR_RXNE) and Spi3RxIrqHandler) Spi3RxIrqHandler();
    chSysUnlockFromISR();
    CH_IRQ_EPILOGUE();
}
#endif
} // extern C

#endif
