/*
 * kl_lib_f0.cpp
 *
 *  Created on: 10.12.2012
 *      Author: kreyl
 */

#include "kl_lib.h"
#include <stdarg.h>
#include <string.h>
#include "uart.h"
#include "main.h"   // App is there

#if 1 // ============================= Timer ===================================
static uint32_t GetTimInputFreq(TIM_TypeDef* ITmr) {
    uint32_t InputFreq = 0;
#if defined STM32L1XX
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
#elif defined STM32F0XX
    uint32_t APB1prs = (RCC->CFGR & RCC_CFGR_PPRE) >> 8;
    if(APB1prs < 0b100) InputFreq = Clk.APBFreqHz;      // APB1CLK = HCLK / 1
    else InputFreq = Clk.APBFreqHz * 2;                 // APB1CLK = HCLK / (not 1)
#elif defined STM32L4XX
    if(ANY_OF_5(ITmr, TIM1, TIM8, TIM15, TIM16, TIM17)) {   // APB2
        uint32_t APB2prs = (RCC->CFGR & RCC_CFGR_PPRE2) >> 11;
        if(APB2prs < 0b100) InputFreq = Clk.APB2FreqHz; // APB2CLK = HCLK / 1
        else InputFreq = Clk.APB2FreqHz * 2;            // APB2CLK = HCLK / (not 1)
    }
    else {                                              // APB1
        uint32_t APB1prs = (RCC->CFGR & RCC_CFGR_PPRE1) >> 8;
        if(APB1prs < 0b100) InputFreq = Clk.APB1FreqHz; // APB1CLK = HCLK / 1
        else InputFreq = Clk.APB1FreqHz * 2;            // APB1CLK = HCLK / (not 1)
    }
#elif defined STM32F2XX
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
#else
#error "Timer Clk setup error"
#endif
    return InputFreq;
}

void Timer_t::Init() const {
#ifdef TIM1
    if     (ITmr == TIM1)  { rccEnableTIM1(FALSE); }
#endif
#ifdef TIM2
    else if(ITmr == TIM2)  { rccEnableTIM2(FALSE); }
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
    else if(ITmr == TIM15)  { rccDisableTIM15(FALSE); }
#endif
#ifdef TIM16
    else if(ITmr == TIM16)  { rccDisableTIM16(FALSE); }
#endif
#ifdef TIM17
    else if(ITmr == TIM17)  { rccDisableTIM17(FALSE); }
#endif
}

void Timer_t::Deinit() const {
    TMR_DISABLE(ITmr);
#ifdef TIM1
    if     (ITmr == TIM1)  { rccDisableTIM1(FALSE); }
#endif
#ifdef TIM2
    else if(ITmr == TIM2)  { rccDisableTIM2(FALSE); }
#endif
#ifdef TIM3
    else if(ITmr == TIM3)  { rccDisableTIM3(FALSE); }
#endif
#ifdef TIM4
    else if(ITmr == TIM4)  { rccDisableTIM4(FALSE); }
#endif
#ifdef TIM5
    else if(ITmr == TIM5)  { rccDisableTIM5(FALSE); }
#endif
#ifdef TIM6
    else if(ITmr == TIM6)  { rccDisableTIM6(FALSE); }
#endif
#ifdef TIM7
    else if(ITmr == TIM7)  { rccDisableTIM7(FALSE); }
#endif
#ifdef TIM8
    else if(ITmr == TIM8)  { rccDisableTIM8(FALSE); }
#endif
#ifdef TIM9
    else if(ITmr == TIM9)  { rccDisableTIM9(FALSE); }
#endif
#ifdef TIM10
    else if(ITmr == TIM10)  { rccDisableTIM10(FALSE); }
#endif
#ifdef TIM11
    else if(ITmr == TIM11)  { rccDisableTIM11(FALSE); }
#endif
#ifdef TIM12
    else if(ITmr == TIM12)  { rccDisableTIM12(FALSE); }
#endif
#ifdef TIM13
    else if(ITmr == TIM13)  { rccDisableTIM13(FALSE); }
#endif
#ifdef TIM14
    else if(ITmr == TIM14)  { rccDisableTIM14(FALSE); }
#endif
#ifdef TIM15
    else if(ITmr == TIM15)  { rccDisableTIM15(FALSE); }
#endif
#ifdef TIM16
    else if(ITmr == TIM16)  { rccDisableTIM16(FALSE); }
#endif
#ifdef TIM17
    else if(ITmr == TIM17)  { rccDisableTIM17(FALSE); }
#endif
}

void Timer_t::SetupPrescaler(uint32_t PrescaledFreqHz) const {
    ITmr->PSC = (GetTimInputFreq(ITmr) / PrescaledFreqHz) - 1;
}

void PinOutputPWM_t::Init() const {
    Timer_t::Init();
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
    if(ITmr == TIM1 or ITmr == TIM2) AF = AF1;
    else if(ITmr == TIM3 or ITmr == TIM4 or ITmr == TIM5) AF = AF2;
    else if(ITmr == TIM8) AF = AF3;
    else if(ITmr == TIM15 or ITmr == TIM16 or ITmr == TIM17) AF = AF14;
    PinSetupAlterFunc(ISetup.PGpio, ISetup.Pin, ISetup.OutputType, pudNone, AF);
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
}

void Timer_t::SetUpdateFrequencyChangingPrescaler(uint32_t FreqHz) const {
    // Figure out input timer freq
    uint32_t UpdFreqMax = GetTimInputFreq(ITmr) / (ITmr->ARR + 1);
    uint32_t div = UpdFreqMax / FreqHz;
    if(div != 0) div--;
//    Uart.Printf("InputFreq=%u; UpdFreqMax=%u; div=%u; ARR=%u\r", InputFreq, UpdFreqMax, div, ITmr->ARR);
    ITmr->PSC = div;
	ITmr->CNT = 0;  // Reset counter to start from scratch
}

void Timer_t::SetUpdateFrequencyChangingTopValue(uint32_t FreqHz) const {
    uint32_t TopVal  = (GetTimInputFreq(ITmr) / FreqHz) - 1;
//    Uart.Printf("Topval = %u\r", TopVal);
    SetTopValue(TopVal);
}
#endif

#if TIMER_KL // =================== Virtual Timers =====================
// Universal VirtualTimer callback
void TmrKLCallback(void *p) {
    reinterpret_cast<TmrKL_t*>(p)->CallbackHandler();
}
#endif

#if CH_DBG_ENABLED // ========================= DEBUG ==========================
void chDbgPanic(const char *msg1) {
#if CH_USE_REGISTRY
    Uart.PrintfNow("\r%S @ %S\r", msg1, chThdSelf()->p_name);
#else
    Uart.PrintfNow("\r%S\r", msg1);
#endif
}
#endif

#ifdef FLASH_LIB_KL // ==================== FLASH & EEPROM =====================
// Here not-fast write is used. I.e. interface will erase the word if it is not the same.
uint8_t Eeprom_t::Write32(uint32_t Addr, uint32_t W) {
    Addr += EEPROM_BASE_ADDR;
//    Uart.Printf("EAdr=%u\r", Addr);
    UnlockEE();
    // Wait for last operation to be completed
    uint8_t status = WaitForLastOperation();
    if(status == OK) {
        *(volatile uint32_t*)Addr = W;
        status = WaitForLastOperation();
    }
    LockEE();
    return status;
}

void Eeprom_t::ReadBuf(void *PDst, uint32_t Sz, uint32_t Addr) {
    uint32_t *p32 = (uint32_t*)PDst;
    Sz = Sz / 4;  // Size in words32
    while(Sz--) {
        *p32 = Read32(Addr);
        p32++;
        Addr += 4;
    }
}

uint8_t Eeprom_t::WriteBuf(void *PSrc, uint32_t Sz, uint32_t Addr) {
    uint32_t *p32 = (uint32_t*)PSrc;
    Addr += EEPROM_BASE_ADDR;
    Sz = (Sz + 3) / 4;  // Size in words32
    UnlockEE();
    // Wait for last operation to be completed
    uint8_t status = WaitForLastOperation();
    while((status == OK) and (Sz > 0))  {
        *(volatile uint32_t*)Addr = *p32;
        status = WaitForLastOperation();
        p32++;
        Addr += 4;
        Sz--;
    }
    LockEE();
    return status;
}

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
void U16ChangeEndianness(uint16_t *p) { *p = __REV16(*p); }
uint8_t TryStrToUInt32(char* S, uint32_t *POutput) {
    if(*S == '\0') return EMPTY;
    char *p;
    *POutput = strtoul(S, &p, 0);
    return (*p == 0)? OK : NOT_A_NUMBER;
}
uint8_t TryStrToInt32(char* S, int32_t *POutput) {
    if(*S == '\0') return EMPTY;
    char *p;
    *POutput = strtol(S, &p, 0);
    return (*p == '\0')? OK : NOT_A_NUMBER;
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
    if(*S == '\0') return EMPTY;
    char *p;
    *POutput = strtof(S, &p);
    return (*p == '\0')? OK : NOT_A_NUMBER;
}
}; // namespace
#endif

#if 1 // ============================== Clocking ===============================
Clk_t Clk;

#define CLK_STARTUP_TIMEOUT     9999

#if defined STM32L1XX
// ==== Inner use ====
uint8_t Clk_t::EnableHSE() {
    RCC->CR |= RCC_CR_HSEON;    // Enable HSE
    // Wait until ready, 1ms typical according to datasheet
    uint32_t StartUpCounter=0;
    do {
        if(RCC->CR & RCC_CR_HSERDY) return OK;   // HSE is ready
        StartUpCounter++;
    } while(StartUpCounter < CLK_STARTUP_TIMEOUT);
    RCC->CR &= ~RCC_CR_HSEON;   // Disable HSE
    return TIMEOUT;
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
    uint32_t APB2prs = (RCC->CFGR & RCC_CFGR_PPRE2) >> 8;
    tmp = APBPrescTable[APB1prs];
    APB1FreqHz = AHBFreqHz >> tmp;
    tmp = APBPrescTable[APB2prs];
    APB2FreqHz = AHBFreqHz >> tmp;
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
        if(EnableHSE() == OK) {
            uint32_t tmp = RCC->CFGR;
            tmp &= ~RCC_CFGR_SW;
            tmp |=  RCC_CFGR_SW_HSE;  // Select HSE as system clock src
            RCC->CFGR = tmp;
            while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSE); // Wait till ready
            return OK;
        }
        else {
            DisableHSE();
            for(volatile uint32_t i=0; i<999; i++);
        }
    } // for
    return FAILURE;
}

// Enables HSE, enables PLL, switches to PLL
uint8_t Clk_t::SwitchToPLL() {
    if(EnableHSE() != 0) return 1;
    if(EnablePLL() != 0) return 2;
    uint32_t tmp = RCC->CFGR;
    tmp &= ~RCC_CFGR_SW;
    tmp |=  RCC_CFGR_SW_PLL;      // Select PLL as system clock src
    RCC->CFGR = tmp;
    while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL); // Wait until ready
    return 0;
}

// Enables MSI, switches to MSI
uint8_t Clk_t::SwitchToMSI() {
    if(EnableMSI() != 0) return 1;
    uint32_t tmp = RCC->CFGR;
    tmp &= ~RCC_CFGR_SW;
    tmp |=  RCC_CFGR_SW_MSI;      // Select MSI as system clock src
    RCC->CFGR = tmp;
    while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_MSI); // Wait until ready
    return 0;
}

// Disable PLL first!
// HsePreDiv: 1...16; PllMul: pllMul[]
uint8_t Clk_t::SetupPLLMulDiv(PllMul_t PllMul, PllDiv_t PllDiv) {
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
    FLASH->ACR |= FLASH_ACR_ACC64;  // Enable 64-bit access
    FLASH->ACR |= FLASH_ACR_PRFTEN; // May be written only when ACC64 is already set
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
    Uart.Printf(
            "AHBFreq=%uMHz; APB1Freq=%uMHz; APB2Freq=%uMHz\r",
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

void Clk_t::UpdateFreqValues() {
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

    // AHB freq
    const uint8_t AHBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
    tmp = AHBPrescTable[((RCC->CFGR & RCC_CFGR_HPRE) >> 4)];
    AHBFreqHz = SysClkHz >> tmp;
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
        if((RCC->CFGR & RCC_CFGR_SWS) == Desired) return OK; // Done
        StartUpCounter++;
    } while(StartUpCounter < CLK_STARTUP_TIMEOUT);
    return TIMEOUT;
}

// Enables HSI, switches to HSI
uint8_t Clk_t::SwitchTo(ClkSrc_t AClkSrc) {
    uint32_t tmp = RCC->CFGR & ~RCC_CFGR_SW;
    switch(AClkSrc) {
        case csHSI:
            if(EnableHSI() != OK) return 1;
            RCC->CFGR = tmp | RCC_CFGR_SW_HSI;  // Select HSI as system clock src
            return WaitSWS(RCC_CFGR_SWS_HSI);
            break;

        case csHSE:
            if(EnableHSE() != OK) return 2;
            RCC->CFGR = tmp | RCC_CFGR_SW_HSE;  // Select HSE as system clock src
            return WaitSWS(RCC_CFGR_SWS_HSE);
            break;

        case csPLL:
            if(EnablePLL() != OK) return 3;
            RCC->CFGR = tmp | RCC_CFGR_SW_PLL; // Select PLL as system clock src
            return WaitSWS(RCC_CFGR_SWS_PLL);
            break;

#ifdef RCC_CFGR_SW_HSI48
        case csHSI48:
            if(EnableHSI48() != OK) return FAILURE;
            RCC->CFGR = tmp | RCC_CFGR_SW_HSI48;
            return WaitSWS(RCC_CFGR_SWS_HSI48);
            break;
#endif
    } // switch
    return FAILURE;
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

// Setup Flash latency depending on CPU freq. Page 60 of ref manual.
// Call after UpdateFreqValues.
void Clk_t::SetupFlashLatency(uint32_t FrequencyHz) {
    uint32_t tmp = FLASH->ACR;
    if(FrequencyHz <= 24000000) tmp &= ~FLASH_ACR_LATENCY;
    else tmp |= FLASH_ACR_LATENCY;
    FLASH->ACR = tmp;
}

void Clk_t::PrintFreqs() {
    Uart.Printf(
            "AHBFreq=%uMHz; APBFreq=%uMHz\r",
            Clk.AHBFreqHz/1000000, Clk.APBFreqHz/1000000);
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
    rccEnableAPB2(RCC_APB2ENR_SYSCFGEN, 1);
}
#elif defined STM32F2XX
uint8_t Clk_t::HSEEnable() {
    RCC->CR |= RCC_CR_HSEON;    // Enable HSE
    // Wait until ready
    uint32_t StartUpCounter=0;
    do {
        if(RCC->CR & RCC_CR_HSERDY) return 0;   // HSE is ready
        StartUpCounter++;
    } while(StartUpCounter < CLK_STARTUP_TIMEOUT);
    return 1; // Timeout
}

uint8_t Clk_t::HSIEnable() {
    RCC->CR |= RCC_CR_HSION;
    // Wait until ready
    uint32_t StartUpCounter=0;
    do {
        if(RCC->CR & RCC_CR_HSIRDY) return 0;   // HSE is ready
        StartUpCounter++;
    } while(StartUpCounter < CLK_STARTUP_TIMEOUT);
    return 1; // Timeout
}

uint8_t Clk_t::PLLEnable() {
    RCC->CR |= RCC_CR_PLLON;
    // Wait until ready
    uint32_t StartUpCounter=0;
    do {
        if(RCC->CR & RCC_CR_PLLRDY) return 0;   // PLL is ready
        StartUpCounter++;
    } while(StartUpCounter < CLK_STARTUP_TIMEOUT);
    return 1; // Timeout
}

void Clk_t::LsiEnable() {
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
    if(HSIEnable() != 0) return 1;
    RCC->CFGR &= ~RCC_CFGR_SW;      // }
    RCC->CFGR |=  RCC_CFGR_SW_HSI;  // } Select HSI as system clock src
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI); // Wait till ready
    return 0;
}

// Enables HSE, switches to HSE
uint8_t Clk_t::SwitchToHSE() {
    if(HSEEnable() != 0) return 1;
    RCC->CFGR &= ~RCC_CFGR_SW;      // }
    RCC->CFGR |=  RCC_CFGR_SW_HSE;  // } Select HSE as system clock src
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSE); // Wait till ready
    return 0;
}

// Enables HSE, enables PLL, switches to PLL
uint8_t Clk_t::SwitchToPLL() {
    if(HSEEnable() != 0) return 1;
    if(PLLEnable() != 0) return 2;
    RCC->CFGR &= ~RCC_CFGR_SW;          // }
    RCC->CFGR |=  RCC_CFGR_SW_PLL;      // } Select PLL as system clock src
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL); // Wait until ready
    return 0;
}

// Disable PLL first!
// InputDiv_M: 2...63;  Multi_N:  2...432;
// SysDiv_P: sd2,4,6,8; UsbDiv_Q: 2...15.
uint8_t Clk_t::SetupPLLDividers(uint8_t InputDiv_M, uint16_t Multi_N, PllSysDiv_P_t SysDiv_P, uint8_t UsbDiv_Q) {
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

void Clk_t::MCO1Enable(Mco1Src_t Src, McoDiv_t Div) {
    PinSetupAlterFunc(GPIOA, 8, omPushPull, pudNone, AF0, psHigh);
    RCC->CFGR &= ~(RCC_CFGR_MCO1 | RCC_CFGR_MCO1PRE);   // First, disable output and clear settings
    RCC->CFGR |= ((uint32_t)Src) | ((uint32_t)Div << 24);
}
void Clk_t::MCO1Disable() {
    PinSetupAnalog(GPIOA, 8);
    RCC->CFGR &= ~(RCC_CFGR_MCO1 | RCC_CFGR_MCO1PRE);
}
void Clk_t::MCO2Enable(Mco2Src_t Src, McoDiv_t Div) {
    PinSetupAlterFunc(GPIOC, 9, omPushPull, pudNone, AF0, psHigh);
    RCC->CFGR &= ~(RCC_CFGR_MCO2 | RCC_CFGR_MCO2PRE);   // First, disable output and clear settings
    RCC->CFGR |= ((uint32_t)Src) | ((uint32_t)Div << 27);
}
void Clk_t::MCO2Disable() {
    PinSetupAnalog(GPIOC, 9);
    RCC->CFGR &= ~(RCC_CFGR_MCO2 | RCC_CFGR_MCO2PRE);
}

void Clk_t::PrintFreqs() {
    Uart.Printf(
            "AHBFreq=%uMHz; APB1Freq=%uMHz; APB2Freq=%uMHz\r",
            Clk.AHBFreqHz/1000000, Clk.APB1FreqHz/1000000, Clk.APB2FreqHz/1000000);
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
void Clk_t::UpdateFreqValues() {
    const uint8_t AHBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
    const uint8_t APBPrescTable[8] = {0, 0, 0, 0, 1, 2, 3, 4};
    const uint32_t MSIRangeTable[12] = {
            100000, 200000, 400000, 800000, 1000000, 2000000,
            4000000, 8000000, 16000000, 24000000, 32000000, 48000000};

    uint32_t tmp, MSIRange;

    // ==== Get MSI Range frequency ====
    if((RCC->CR & RCC_CR_MSIRGSEL) == 0) tmp = (RCC->CSR & RCC_CSR_MSISRANGE) >> 8;  // MSISRANGE from RCC_CSR applies
    else tmp = (RCC->CR & RCC_CR_MSIRANGE) >> 4;    // MSIRANGE from RCC_CR applies
    MSIRange = MSIRangeTable[tmp];                  // MSI frequency range in Hz

    // ==== Figure out SysClk ====
    uint32_t SysClkHz;// = HSI_FREQ_HZ;
    tmp = (RCC->CFGR & RCC_CFGR_SWS) >> 2;  // System clock switch status
    switch(tmp) {
        case 0b00: // MSI
            SysClkHz = MSIRange;
            break;

        case 0b01: // HSI
            SysClkHz = HSI_FREQ_HZ;
            break;

        case 0b10: // HSE
            SysClkHz = CRYSTAL_FREQ_HZ;
            break;

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
            SysClkHz = PllVCO / PllR;
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
    Uart.Printf(
            "AHBFreq=%uMHz; APB1Freq=%uMHz; APB2Freq=%uMHz\r",
            Clk.AHBFreqHz/1000000, Clk.APB1FreqHz/1000000, Clk.APB2FreqHz/1000000);
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

void Clk_t::SetHiPerfMode() {
    if(HiPerfModeEnabled) return;
    __unused uint8_t Rslt = FAILURE;
    // Try to enable HSE
    if(EnableHSE() == OK) {
        // Setup PLL (must be disabled first)
        if(SetupPllMulDiv(1, 24, 4, 6) == OK) { // 12MHz / 1 * 24 => 72 and 48MHz
            SetupBusDividers(ahbDiv1, apbDiv1, apbDiv1);
            SetVoltageRange(mvrHiPerf);
            SetupFlashLatency(72, mvrHiPerf);
            EnablePrefeth();
            // Switch clock
            if(EnablePLL() == OK) {
                if(SwitchToPLL() == OK) {
                    Rslt = OK;
                    HiPerfModeEnabled = true;
                } // sw 2 PLL
            } // en PLL
        } // if setup pll div
    } // if Enable HSE
    // Switch back if failure
    // TODO if(Rslt != OK)
}

void Clk_t::SetLoPerfMode() {

}

void Clk_t::SetVoltageRange(MCUVoltRange_t VoltRange) {
    uint32_t tmp = PWR->CR1;
    tmp &= ~PWR_CR1_VOS;
    if(VoltRange == mvrHiPerf) tmp |= (0b01 << 9);
    else tmp |= (0b10 << 9);
    PWR->CR1 = tmp;
}

// M: 1...8; N: 8...86; R: 2,4,6,8
uint8_t Clk_t::SetupPllMulDiv(uint32_t M, uint32_t N, uint32_t R, uint32_t Q, uint32_t P) {
    if(!((M >= 1 and M <= 8) and (N >= 8 and N <= 86) and (R == 2 or R == 4 or R == 6 or R == 8))) return CMD_ERROR;
    if(RCC->CR & RCC_CR_PLLON) return BUSY; // PLL must be disabled to change dividers
    R = (R / 2) - 1;    // 2,4,6,8 => 0,1,2,3
    Q = (Q / 2) - 1;    // 2,4,6,8 => 0,1,2,3
    uint32_t tmp = RCC->PLLCFGR;
    tmp &= ~(RCC_PLLCFGR_PLLR | RCC_PLLCFGR_PLLREN | RCC_PLLCFGR_PLLQ | RCC_PLLCFGR_PLLQEN |
            RCC_PLLCFGR_PLLP | RCC_PLLCFGR_PLLPEN | RCC_PLLCFGR_PLLN | RCC_PLLCFGR_PLLM | RCC_PLLCFGR_PLLSRC);
    tmp |= RCC_PLLCFGR_PLLSRC_HSE | // Use only HSE as src
            ((M - 1) << 4) |
            (N << 8) |
            (R << 25) | RCC_PLLCFGR_PLLREN |    // PLLCLK output enable
            (Q << 21) | RCC_PLLCFGR_PLLQEN;     // PLL48M1CLK output enable
    RCC->PLLCFGR = tmp;
    return 0;
}

uint8_t Clk_t::SetupPllSai1(uint32_t N, uint32_t R) {
    // Disable PLLSAI1
    CLEAR_BIT(RCC->CR, RCC_CR_PLLSAI1ON);
    // Wait till PLLSAI1 is ready to be updated
    uint32_t t = 45000;
    while(READ_BIT(RCC->CR, RCC_CR_PLLSAI1RDY) != 0) {
        if(t-- == 0) {
            Uart.Printf("Sai1Off Timeout %X\r", RCC->CR);
            return FAILURE;
        }
    }
    // Setup dividers
    MODIFY_REG(RCC->PLLSAI1CFGR, RCC_PLLSAI1CFGR_PLLSAI1N, N << 8);
    MODIFY_REG(RCC->PLLSAI1CFGR, RCC_PLLSAI1CFGR_PLLSAI1R, ((R >> 1U) - 1U) << 25);
    SET_BIT(RCC->CR, RCC_CR_PLLSAI1ON); // Enable SAI
    // Wait till PLLSAI1 is ready. May fail if PLL source disabled or not selected.
    t = 45000;
    while(READ_BIT(RCC->CR, RCC_CR_PLLSAI1RDY) == 0) {
        if(t-- == 0) {
            Uart.Printf("SaiOn Timeout %X\r", RCC->CR);
            return FAILURE;
        }
    }
    return OK;
}

void Clk_t::Select48MhzSrc(Src48MHz_t Src) {
    uint32_t tmp = RCC->CCIPR;
    tmp &= ~RCC_CCIPR_CLK48SEL;
    tmp |= ((uint32_t)Src) << 26;
    RCC->CCIPR = tmp;
}

// ==== Enable/Disable ====
uint8_t Clk_t::EnableHSI() {
    RCC->CR |= RCC_CR_HSION;
    // Wait until ready
    uint32_t StartUpCounter=0;
    do {
        if(RCC->CR & RCC_CR_HSIRDY) return OK;   // HSE is ready
        StartUpCounter++;
    } while(StartUpCounter < CLK_STARTUP_TIMEOUT);
    return TIMEOUT;
}
uint8_t Clk_t::EnableHSE() {
    RCC->CR |= RCC_CR_HSEON;    // Enable HSE
    // Wait until ready
    uint32_t StartupCounter=0;
    do {
        if(RCC->CR & RCC_CR_HSERDY) return OK;   // HSE is ready
        StartupCounter++;
    } while(StartupCounter < CLK_STARTUP_TIMEOUT);
    return TIMEOUT;
}
uint8_t Clk_t::EnablePLL() {
    RCC->CR |= RCC_CR_PLLON;
    // Wait until ready
    uint32_t StartUpCounter=0;
    do {
        if(RCC->CR & RCC_CR_PLLRDY) return OK;   // PLL is ready
        StartUpCounter++;
    } while(StartUpCounter < CLK_STARTUP_TIMEOUT);
    return TIMEOUT;
}

// ==== Switch ====
// Enables HSE, switches to HSE
uint8_t Clk_t::SwitchToHSE() {
    if(EnableHSE() != 0) return 1;
    RCC->CFGR &= ~RCC_CFGR_SW;      // }
    RCC->CFGR |=  RCC_CFGR_SW_HSE;  // } Select HSE as system clock src
    while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSE); // Wait till ready
    return 0;
}

// Enables HSE, enables PLL, switches to PLL
uint8_t Clk_t::SwitchToPLL() {
    RCC->CFGR |= RCC_CFGR_SW_PLL;   // Select PLL as system clock src
    while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL); // Wait until ready
    return OK;
}

#endif

#endif // Clocking
