/*
 * kl_lib_f0.cpp
 *
 *  Created on: 10.12.2012
 *      Author: kreyl
 */

#include "shell.h"
#include <stdarg.h>
#include <string.h>
#include "MsgQ.h"
#include <malloc.h>

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

#if 1 // ============================ kl_string ================================
__always_inline
inline int kl_tolower(char c) {
    return (c >= 'A' and c <= 'Z')? (c + ('a' - 'A')) : c;
}

/* Compare S1 and S2, ignoring case, returning less than, equal to or
   greater than zero if S1 is lexicographically less than,
   equal to or greater than S2.  */
int kl_strcasecmp(const char *s1, const char *s2) {
  const unsigned char *p1 = (const unsigned char *) s1;
  const unsigned char *p2 = (const unsigned char *) s2;
  int result;
  if (p1 == p2) return 0;
  while((result = kl_tolower(*p1) - kl_tolower(*p2++)) == 0) {
      if(*p1++ == '\0') break;
  }
  return result;
}
#endif

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
    srandom(dw);
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
    else if(ILPTim == LPTIM1)  { rccEnableAPB1R1(RCC_APB1ENR1_LPTIM1EN, FALSE); }
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
    else if(ILPTim == LPTIM1)  { rccDisableAPB1R1(RCC_APB1ENR1_LPTIM1EN); }
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
    else if(ITmr == TIM3)  PinSetupAlterFunc(ISetup.PGpio, ISetup.Pin, ISetup.OutputType, pudNone, AF1);
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
    PinSetupAlterFunc(GPIO, N, OutputType, pudNone, AF0);   // Alternate function is dummy
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
    uint32_t div = UpdFreqMax / FreqHz;
    if(div != 0) div--;
//    Uart.Printf("InputFreq=%u; UpdFreqMax=%u; div=%u; ARR=%u\r", InputFreq, UpdFreqMax, div, ITmr->ARR);
    ITmr->PSC = div;
    ITmr->CNT = 0;  // Reset counter to start from scratch
}

void Timer_t::SetUpdateFrequencyChangingTopValue(uint32_t FreqHz) const {
    uint32_t TopVal  = (Clk.GetTimInputFreq(ITmr) / FreqHz) - 1;
//    Uart.Printf("Topval = %u\r", TopVal);
    SetTopValue(TopVal);
}
#endif

#if 1 // ========================= Virtual Timers ==============================
// Universal VirtualTimer callback
void TmrKLCallback(void *p) {
    chSysLockFromISR();
    ((IrqHandler_t*)p)->IIrqHandler();
    chSysUnlockFromISR();
}

void TmrKL_t::IIrqHandler() {    // Call it inside callback
    EvtMsg_t Msg(EvtId);
    EvtQMain.SendNowOrExitI(Msg);
    if(TmrType == tktPeriodic) StartI();
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
    USART1->CR3 &= ~USART_CR3_DMAT;
    while(*S != 0) {
//        ITM_SendChar(*S++);
#if defined STM32L1XX || defined STM32F2XX
        while(!(USART1->SR & USART_SR_TXE));
        USART1->DR = *S;
#else
        while(!(USART1->ISR & USART_ISR_TXE));
        USART1->TDR = *S;
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
#elif defined STM32F2XX

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
static uint8_t WaitForLastOperation(systime_t Timeout_st) {
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

#else
    else if(FLASH->SR & FLASH_SR_PGERR) return retvFail;
    else if(FLASH->SR & FLASH_SR_WRPRTERR) return retvFail;
#endif
    else return retvOk;
}

static uint8_t WaitForLastOperation(systime_t Timeout_st) {
    uint8_t status = retvOk;
    // Wait for a Flash operation to complete or a TIMEOUT to occur
    do {
        status = GetStatus();
        Timeout_st--;
    } while((status == retvBusy) and (Timeout_st != 0x00));
    if(Timeout_st == 0x00) status = retvTimeout;
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
#elif defined STM32F2XX

#else
    WaitForLastOperation(FLASH_ProgramTimeout);
    FLASH->CR |= FLASH_CR_LOCK;
#endif
}

// Beware: use Page Address (0...255), not absolute address kind of 0x08003f00
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
#elif defined STM32F2XX

#else
        FLASH->CR |= FLASH_CR_PER;
        FLASH->AR = PageAddress;
        FLASH->CR |= FLASH_CR_STRT;
        // Wait for last operation to be completed
        status = WaitForLastOperation(FLASH_EraseTimeout);
        // Disable the PER Bit
        FLASH->CR &= 0x00001FFD;
#endif
    }
    return status;
}

#if defined STM32L4XX
uint8_t ProgramDWord(uint32_t Address, uint64_t Data) {
    uint8_t status = WaitForLastOperation(FLASH_ProgramTimeout);
    if(status == retvOk) {
        chSysLock();
        ClearErrFlags();
        // Deactivate the data cache to avoid data misbehavior
        FLASH->ACR &= ~FLASH_ACR_DCEN;
        // Program Dword
        SET_BIT(FLASH->CR, FLASH_CR_PG);    // Enable flash writing
        *(volatile uint32_t*)Address = (uint32_t)Data;
        *(volatile uint32_t*)(Address + 4) = (uint32_t)(Data >> 32);
        status = WaitForLastOperation(FLASH_ProgramTimeout);
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
        FLASH->CR |= 0x00000001; // FLASH_CR_PG_Set
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

#else
    CLEAR_BIT(FLASH->CR, FLASH_CR_OPTWRE);
    LockFlash();
#endif
}

void WriteOptionByteRDP(uint8_t Value) {
    UnlockFlash();
    ClearPendingFlags();
    UnlockOptionBytes();
    if(WaitForLastOperation(FLASH_ProgramTimeout) == retvOk) {
#ifdef STM32L1XX
        uint32_t OptBytes = *(volatile uint32_t*)0x1FF80000;
        OptBytes &= 0xFF00FF00; // Clear RDP and nRDP
        OptBytes |= Value;      // Write RDP
        OptBytes |= (Value ^ 0xFF) << 16; // Write nRDP;
        *(volatile uint32_t*)0x1FF80000 = OptBytes;
        WaitForLastOperation(FLASH_ProgramTimeout);
#elif defined STM32L4XX
        uint32_t OptReg = FLASH->OPTR;
        OptReg &= ~FLASH_OPTR_RDP_Msk;  // Clear RDP
        OptReg |= Value;
        FLASH->OPTR = OptReg;
        FLASH->CR |= FLASH_CR_OPTSTRT;
        WaitForLastOperation(FLASH_ProgramTimeout);
#elif defined STM32F2XX

#else
        // Erase option bytes
        SET_BIT(FLASH->CR, FLASH_CR_OPTER);
        SET_BIT(FLASH->CR, FLASH_CR_STRT);
        uint8_t Rslt = WaitForLastOperation(FLASH_ProgramTimeout);
        CLEAR_BIT(FLASH->CR, FLASH_CR_OPTER);
        if(Rslt == retvOk) {
            SET_BIT(FLASH->CR, FLASH_CR_OPTPG); // Enable the Option Bytes Programming operation
            OB->RDP = Value;
            WaitForLastOperation(FLASH_ProgramTimeout);
            CLEAR_BIT(FLASH->CR, FLASH_CR_OPTPG); // Disable the Option Bytes Programming operation
        }
#endif
    }
    LockOptionBytes();
    LockFlash();
}

// ==== Firmare lock ====
bool FirmwareIsLocked() {
#ifdef STM32L4XX
    return (FLASH->OPTR & 0xFF) != 0xAA;
#elif defined STM32L1XX
    return (FLASH->OBR & 0xFF) != 0xAA;
#elif defined STM32F2XX
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
#else
    WriteOptionByteRDP(0x1D); // Any value except 0xAA or 0xCC
    // Set the OBL_Launch bit to reset system and launch the option byte loading
#ifdef STM32L1XX
    FLASH->PECR |= FLASH_PECR_OBL_LAUNCH;
#elif defined STM32F2XX || defined STM32F1XX

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
#if defined STM32L476
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
    if(*((uint32_t*)(Addr)) == W) return retvOk;
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

#if IWDG_ENABLED // =========================== IWDG ===========================
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

#if defined STM32L4XX
void DisableInDebug() {
    DBGMCU->APB1FZR1 |= DBGMCU_APB1FZR1_DBG_IWDG_STOP;
}
#endif

static void Enable() { IWDG->KR = 0xCCCC; }
static void EnableAccess() { IWDG->KR = 0x5555; }

static void SetPrescaler(Pre_t Prescaler) { IWDG->PR = (uint32_t)Prescaler; }
static void SetReload(uint16_t Reload) { IWDG->RLR = Reload; }

void SetTimeout(uint32_t ms) {
    EnableAccess();
    SetPrescaler(iwdgPre256);
    uint32_t Count = (ms * (LSI_FREQ_HZ/1000UL)) / 256UL;
    TRIM_VALUE(Count, 0xFFF);
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
    if(EnableHSE() != 0) return 1;
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
    tmp |= RCC_CFGR_PLLSRC_HSE;
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
        case 0b00: // HSI
            SysClkHz = HSI_FREQ_HZ;
            break;

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

void Clk_t::SetCoreClk(CoreClk_t CoreClk) {
    EnablePrefetch();
    // Enable/disable HSE
//    if(CoreClk >= cclk16MHz) {
//        if(EnableHSE() != retvOk) return;   // Try to enable HSE
//        DisablePLL();
//    }
    SetupPLLSrc(pllSrcHSIdiv2);

    // Setup dividers
    switch(CoreClk) {
        case cclk8MHz:
            break;
        // Setup PLL (must be disabled first)
        case cclk12MHz:
            SetupFlashLatency(12);
            // 4MHz * 3 => 12MHz
            if(SetupPllMulDiv(pllMul3, preDiv1) != retvOk) return;
            if(EnablePLL() == retvOk) SwitchToPLL();
            break;
        case cclk16MHz:
            SetupFlashLatency(16);
            // 4MHz * 4 => 16MHz
            if(SetupPllMulDiv(pllMul4, preDiv1) != retvOk) return;
            if(EnablePLL() == retvOk) SwitchToPLL();
            break;
        case cclk24MHz:
            SetupFlashLatency(24);
            // 4MHz * 6 => 24MHz
            if(SetupPllMulDiv(pllMul6, preDiv1) != retvOk) return;
            if(EnablePLL() == retvOk) SwitchToPLL();
            break;
        case cclk48MHz:
            SetupFlashLatency(48);
            // 4MHz * 12 => 48MHz
            if(SetupPllMulDiv(pllMul12, preDiv1) != retvOk) return;
            if(EnablePLL() == retvOk) SwitchToPLL();
            break;
        case cclk72MHz:
            // 12MHz / 1 * 24 => 72 and 48MHz
//            if(SetupPllMulDiv(1, 24, 4, 6) != retvOk) return;
//            SetupFlashLatency(72, mvrHiPerf);
            break;
    } // switch
}

void Clk_t::SetupFlashLatency(uint8_t AHBClk_MHz) {
    uint32_t tmp = FLASH->ACR;
    tmp &= ~FLASH_ACR_LATENCY;  // Clear Latency bits: 0 wait states
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
            Clk.AHBFreqHz/1000000, Clk.APB1FreqHz/1000000, Clk.APB2FreqHz/1000000);
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

void Clk_t::SetCoreClk(CoreClk_t CoreClk) {
    EnablePrefeth();
    // First, switch to MSI if clock src is not MSI
    if((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_MSI) {
        if(SwitchToMSI() != retvOk) return;
    }

    // Disable PLL and SAI1, enable HSE
    DisablePLL();
    DisableSai1();
    if(CoreClk >= cclk16MHz) {
        if(EnableHSE() != retvOk) return;
        SetVoltageRange(mvrHiPerf);
    }

    // Setup dividers
    switch(CoreClk) {
        case cclk8MHz:
            break;
        // Setup PLL (must be disabled first)
        case cclk16MHz:
            if(AHBFreqHz < (uint32_t)CoreClk) SetupFlashLatency(16, mvrHiPerf);
            // 12MHz / 1 = 12; 12 * 8 / 6 = 16
            if(SetupPllMulDiv(1, 8, 6, 2) != retvOk) return;
            SetupFlashLatency(16, mvrHiPerf);
            break;
        case cclk24MHz:
            if(AHBFreqHz < (uint32_t)CoreClk) SetupFlashLatency(24, mvrHiPerf);
            // 12MHz / 1 = 12; 12 * 8 / 4 = 24
            if(SetupPllMulDiv(1, 8, 4, 2) != retvOk) return;
            SetupFlashLatency(24, mvrHiPerf);
            break;
        case cclk48MHz:
            if(AHBFreqHz < (uint32_t)CoreClk) SetupFlashLatency(48, mvrHiPerf);
            // 12MHz / 1 = 12; 12 * 8 / 2 => 48
            if(SetupPllMulDiv(1, 8, 2, 2) != retvOk) return;
            SetupFlashLatency(48, mvrHiPerf);
            break;
        case cclk64MHz:
            if(AHBFreqHz < (uint32_t)CoreClk) SetupFlashLatency(64, mvrHiPerf);
            // 12MHz / 3 = 4; 4 * 32 / 2 => 64
            if(SetupPllMulDiv(3, 32, 2, 2) != retvOk) return;
            SetupFlashLatency(64, mvrHiPerf);
            break;
        case cclk72MHz:
            if(AHBFreqHz < (uint32_t)CoreClk) SetupFlashLatency(72, mvrHiPerf);
            // 12MHz / 1 = 12; 12 * 12 / 2 = 72
            if(SetupPllMulDiv(1, 24, 4, 6) != retvOk) return;
            SetupFlashLatency(72, mvrHiPerf);
            break;
        case cclk80MHz:
            if(AHBFreqHz < (uint32_t)CoreClk) SetupFlashLatency(80, mvrHiPerf);
            // 12MHz / 3 = 4; 4 * 40 / 2 = 80; * 24 / 2 = 48
            if(SetupPllMulDiv(3, 40, 2, 2) != retvOk) return;
            SetupFlashLatency(80, mvrHiPerf);
            break;
        default: break;
    } // switch

    if(CoreClk >= cclk16MHz) {
        SetupBusDividers(ahbDiv1, apbDiv1, apbDiv1);
        if(EnablePLL() == retvOk) {
            EnablePLLROut();
            SwitchToPLL();
        }
    }
}


void Clk_t::SetVoltageRange(MCUVoltRange_t VoltRange) {
    uint32_t tmp = PWR->CR1;
    tmp &= ~PWR_CR1_VOS;
    if(VoltRange == mvrHiPerf) tmp |= (0b01 << 9);
    else tmp |= (0b10 << 9);
    PWR->CR1 = tmp;
}

// M: 1...8; N: 8...86; R: 2,4,6,8
uint8_t Clk_t::SetupPllMulDiv(uint32_t M, uint32_t N, uint32_t R, uint32_t Q) {
    if(!((M >= 1 and M <= 8) and (N >= 8 and N <= 86) and (R == 2 or R == 4 or R == 6 or R == 8))) return retvBadValue;
    if(RCC->CR & RCC_CR_PLLON) return retvBusy; // PLL must be disabled to change dividers
    R = (R / 2) - 1;    // 2,4,6,8 => 0,1,2,3
    Q = (Q / 2) - 1;    // 2,4,6,8 => 0,1,2,3
    uint32_t tmp = RCC->PLLCFGR;
    tmp &= ~(RCC_PLLCFGR_PLLR | RCC_PLLCFGR_PLLREN | RCC_PLLCFGR_PLLQ | RCC_PLLCFGR_PLLQEN |
            RCC_PLLCFGR_PLLP | RCC_PLLCFGR_PLLPEN | RCC_PLLCFGR_PLLN | RCC_PLLCFGR_PLLM | RCC_PLLCFGR_PLLSRC);
    tmp |= RCC_PLLCFGR_PLLSRC_HSE | // Use only HSE as src
            ((M - 1) << 4) |
            (N << 8) |
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

void Clk_t::SetupSai1Qas48MhzSrc() {
    // Get SAI input freq
    uint32_t InputFreq, tmp;
    uint32_t PllM = ((RCC->PLLCFGR & RCC_PLLCFGR_PLLM) >> 4) + 1;
    uint32_t PllSrc = (RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC);
    PllM = ((RCC->PLLCFGR & RCC_PLLCFGR_PLLM) >> 4) + 1 ;
    switch(PllSrc) {
        case 0x02:  // HSI used as PLL clock source
            InputFreq = (HSI_FREQ_HZ / PllM);
            break;
        case 0x03:  // HSE used as PLL clock source
            InputFreq = (CRYSTAL_FREQ_HZ / PllM);
            break;
        default: {   // MSI used as PLL clock source
            uint32_t MSIRange;
            // Get MSI Range frequency
            if((RCC->CR & RCC_CR_MSIRGSEL) == 0) tmp = (RCC->CSR & RCC_CSR_MSISRANGE) >> 8;  // MSISRANGE from RCC_CSR applies
            else tmp = (RCC->CR & RCC_CR_MSIRANGE) >> 4;    // MSIRANGE from RCC_CR applies
            MSIRange = MSIRangeTable[tmp];                  // MSI frequency range in Hz
            // Calc freq
            InputFreq = (MSIRange / PllM);
        } break;
    } // switch(PllSrc)

    // Setup Sai
    DisableSai1();
    switch(InputFreq) {
        case 2000000:  SetupPllSai1(48, 2, 2, 7); break; // 2 * 48 / 2 = 48
        case 3000000:  SetupPllSai1(32, 2, 2, 7); break; // 3 * 32 / 2 = 48
        case 4000000:  SetupPllSai1(24, 2, 2, 7); break; // 4 * 24 / 2 = 48
        case 12000000: SetupPllSai1( 8, 2, 2, 7); break; // 12 * 8 / 2 = 48
        default: return;
    }

    if(EnableSai1() == retvOk) {
        // Setup Sai1Q as 48MHz source
        EnableSai1QOut(); // Enable 48MHz output
        tmp = RCC->CCIPR;
        tmp &= ~RCC_CCIPR_CLK48SEL;
        tmp |= ((uint32_t)src48PllSai1Q) << 26;
        RCC->CCIPR = tmp;
    }
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

uint8_t Clk_t::EnableSai1() {
    SET_BIT(RCC->CR, RCC_CR_PLLSAI1ON); // Enable SAI
    // Wait till PLLSAI1 is ready. May fail if PLL source disabled or not selected.
    uint32_t t = 45000;
    while(READ_BIT(RCC->CR, RCC_CR_PLLSAI1RDY) == 0) {
        if(t-- == 0) {
            Printf("SaiOn Timeout %X\r", RCC->CR);
            return retvFail;
        }
    }
    return retvOk;
}

void Clk_t::DisableSai1() {
    RCC->CR &= ~RCC_CR_PLLSAI1ON;
    while(RCC->CR & RCC_CR_PLLSAI1RDY); // Wait until ready
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
#elif defined STM32


#endif

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
    // Mode: Master, NSS software controlled and is 1, 8bit, NoCRC, FullDuplex
    PSpi->CR1 = SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_MSTR;
    if(BitOrder == boLSB) PSpi->CR1 |= SPI_CR1_LSBFIRST;    // MSB/LSB
    if(CPOL == cpolIdleHigh) PSpi->CR1 |= SPI_CR1_CPOL;     // CPOL
    if(CPHA == cphaSecondEdge) PSpi->CR1 |= SPI_CR1_CPHA;   // CPHA
    // Baudrate
    int32_t div;
#if defined STM32L1XX || defined STM32F4XX || defined STM32F2XX || defined STM32L4XX
    if(PSpi == SPI1) div = Clk.APB2FreqHz / Bitrate_Hz;
    else div = Clk.APB1FreqHz / Bitrate_Hz;
#elif defined STM32F030 || defined STM32F0
    div = Clk.APBFreqHz / Bitrate_Hz;
#endif
    SpiClkDivider_t ClkDiv = sclkDiv2;
    if     (div >= 128) ClkDiv = sclkDiv256;
    else if(div >= 64) ClkDiv = sclkDiv128;
    else if(div >= 32) ClkDiv = sclkDiv64;
    else if(div >= 16) ClkDiv = sclkDiv32;
    else if(div >= 8)  ClkDiv = sclkDiv16;
    else if(div >= 4)  ClkDiv = sclkDiv8;
    else if(div >= 2)  ClkDiv = sclkDiv4;
    PSpi->CR1 |= ((uint16_t)ClkDiv) << 3;
    // Bit number
#if defined STM32L1XX || defined STM32F10X_LD_VL || defined STM32F2XX || defined STM32F4XX
    if(BitNumber == bitn16) PSpi->CR1 |= SPI_CR1_DFF;
    PSpi->CR2 = 0;
#elif defined STM32F030 || defined STM32F072xB || defined STM32L4XX
    if(BitNumber == bitn16) PSpi->CR2 = (uint16_t)0b1111 << 8;  // 16 bit, RXNE generated when 16 bit is received
    else PSpi->CR2 = ((uint16_t)0b0111 << 8) | SPI_CR2_FRXTH;   // 8 bit, RXNE generated when 8 bit is received
#endif
}

// IRQs
static ftVoidVoid Spi1RxIrqHandler = nullptr;
#ifdef SPI2
static ftVoidVoid Spi2RxIrqHandler = nullptr;
#endif
#ifdef SPI3
static ftVoidVoid Spi3RxIrqHandler = nullptr;
#endif

void Spi_t::SetupRxIrqCallback(ftVoidVoid AIrqHandler) const {
    if(PSpi == SPI1) Spi1RxIrqHandler = AIrqHandler;
#ifdef SPI2
    else if(PSpi == SPI2) Spi2RxIrqHandler = AIrqHandler;
#endif
#ifdef SPI3
    else if(PSpi == SPI3) Spi3RxIrqHandler = AIrqHandler;
#endif
}

extern "C" {
void VectorCC() {   // SPI1
    CH_IRQ_PROLOGUE();
    chSysLockFromISR();
    uint32_t SR = SPI1->SR;
    if(SR & SPI_SR_RXNE and Spi1RxIrqHandler) Spi1RxIrqHandler();
    chSysUnlockFromISR();
    CH_IRQ_EPILOGUE();
}

#ifdef SPI2
void VectorD0() {   // SPI2
    CH_IRQ_PROLOGUE();
    chSysLockFromISR();
    uint32_t SR = SPI2->SR;
    if(SR & SPI_SR_RXNE and Spi2RxIrqHandler) Spi2RxIrqHandler();
    chSysUnlockFromISR();
    CH_IRQ_EPILOGUE();
}
#endif
#ifdef SPI3
void Vector10C() {   // SPI3
    CH_IRQ_PROLOGUE();
    chSysLockFromISR();
    uint32_t SR = SPI3->SR;
    if(SR & SPI_SR_RXNE and Spi3RxIrqHandler) Spi3RxIrqHandler();
    chSysUnlockFromISR();
    CH_IRQ_EPILOGUE();
}
#endif
} // extern C

#endif
