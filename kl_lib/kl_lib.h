/*
 * kl_lib.h
 *
 *  Created on: 10.12.2012
 *      Author: kreyl
 */

#pragma once

#include "ch.h"
#include "hal.h"
#include "core_cmInstr.h"
#include <cstdlib>
#include <sys/cdefs.h>

/*
Build time:
Define symbol BUILD_TIME in main.cpp options with value "\"${current_date}\"".
Maybe, to calm Eclipse, it will be required to write extra quote in the end: "\"${current_date}\"""
*/

// Lib version
#define KL_LIB_VERSION      "20160701_1045"

#if defined STM32L1XX
#include "stm32l1xx.h"
#elif defined STM32F0XX
#include "stm32f0xx.h"
#elif defined STM32F2XX
#include "stm32f2xx.h"
#elif defined STM32F4XX
#include "stm32f4xx.h"
#elif defined STM32F10X_LD_VL
#include "stm32f10x.h"
#elif defined STM32L4XX
#include "stm32l4xx.h"
#endif

#if 1 // ============================ General ==================================
#define __noreturn      __attribute__((noreturn))
#define __packed        __attribute__((__packed__))
#define __align4        __attribute__((aligned (4)))
// Also remember __unused and __always_inline
#ifndef countof
#define countof(A)  (sizeof(A)/sizeof(A[0]))
#endif

/* ==== Function in RAM ====
 * Add to LD script, inside .data section, the following line:
     *(.fastrun)         // RAM-Functions
     Example:
        . = ALIGN(4);
        *(.fastrun)     // RAM-Functions
        PROVIDE(_edata = .);
        _data_end = .;
    } > DATA_RAM AT > flash
 */
#define __ramfunc __attribute__ ((long_call, section (".fastrun")))

/*
 * Early initialization code.
 * This initialization must be performed just after stack setup and before
 * any other initialization.
 */
extern "C" {
void __early_init(void);
}

#ifndef TRUE
#define TRUE    1
#endif
#ifndef FALSE
#define FALSE   0
#endif

// Return values
#define OK              0
#define FAILURE         1
#define TIMEOUT         2
#define BUSY            3
#define IN_PROGRESS     4
#define CMD_ERROR       5
#define CMD_UNKNOWN     6
#define NEW             7
#define LAST            8
#define EMPTY           9
#define OVERFLOW        10
#define NOT_A_NUMBER    11
#define WRITE_PROTECT   12
#define END_OF_FILE     13
#define NOT_FOUND       14

// Binary semaphores
#define NOT_TAKEN       false
#define TAKEN           true

enum BitOrder_t {boMSB, boLSB};
enum LowHigh_t  {Low, High};
enum RiseFall_t {rfRising, rfFalling, rfNone};
enum Inverted_t {invNotInverted, invInverted};
enum PinOutMode_t {omPushPull = 0, omOpenDrain = 1};
enum BitNumber_t {bitn8, bitn16, bitn32};

typedef void (*ftVoidVoid)(void);
typedef void (*ftVoidPVoid)(void*p);
typedef void (*ftVoidPVoidLen)(void*p, uint32_t Len);

// ==== Math ====
#define MIN(a, b)   ( ((a)<(b))? (a) : (b) )
#define MAX(a, b)   ( ((a)>(b))? (a) : (b) )
#define ABS(a)      ( ((a) < 0)? -(a) : (a) )
#define TRIM_VALUE(v, Max)  { if((v) > (Max)) (v) = (Max); }
#define IS_LIKE(v, precise, deviation)  (((precise - deviation) < v) and (v < (precise + deviation)))
#define BitIsSet(r, b)  ((r) & (b))

template <typename T>
static T Average(T *p, uint32_t Len) {
    T Rslt = 0;
    for(uint32_t i=0; i<Len; i++) Rslt += *p++;
    Rslt /= Len;
    return Rslt;
}

#define ANY_OF_2(a, b1, b2)             (((a)==(b1)) or ((a)==(b2)))
#define ANY_OF_3(a, b1, b2, b3)         (((a)==(b1)) or ((a)==(b2)) or ((a)==(b3)))
#define ANY_OF_4(a, b1, b2, b3, b4)     (((a)==(b1)) or ((a)==(b2)) or ((a)==(b3)) or ((a)==(b4)))
#define ANY_OF_5(a, b1, b2, b3, b4, b5) (((a)==(b1)) or ((a)==(b2)) or ((a)==(b3)) or ((a)==(b4)) or ((a)==(b5)))

// IRQ priorities
#define IRQ_PRIO_LOW            15  // Minimum
#define IRQ_PRIO_MEDIUM         9
#define IRQ_PRIO_HIGH           7
#define IRQ_PRIO_VERYHIGH       4   // Higher than systick

// DMA
#define DMA_PRIORITY_LOW        STM32_DMA_CR_PL(0b00)
#define DMA_PRIORITY_MEDIUM     STM32_DMA_CR_PL(0b01)
#define DMA_PRIORITY_HIGH       STM32_DMA_CR_PL(0b10)
#define DMA_PRIORITY_VERYHIGH   STM32_DMA_CR_PL(0b11)

namespace Convert { // ============== Conversion operations ====================
union DWordBytes_t {
    uint32_t DWord;
    uint8_t b[4];
};
union WordBytes_t {
    uint16_t Word;
    uint8_t b[2];
} __attribute__((packed));

void U16ToArrAsBE(uint8_t *PArr, uint16_t N);
void U32ToArrAsBE(uint8_t *PArr, uint32_t N);
uint16_t ArrToU16AsBE(uint8_t *PArr);
uint32_t ArrToU32AsBE(uint8_t *PArr);
void U16ChangeEndianness(uint16_t *p);
#define ReverseByteOrder32(p)   (p) = __REV(p)
uint8_t TryStrToUInt32(char* S, uint32_t *POutput);
uint8_t TryStrToInt32(char* S, int32_t *POutput);
uint16_t BuildUint16(uint8_t Lo, uint8_t Hi);
uint32_t BuildUint32(uint8_t Lo, uint8_t MidLo, uint8_t MidHi, uint8_t Hi);
uint8_t TryStrToFloat(char* S, float *POutput);
}; // namespace

// Init, to calm compiler
extern "C" {
void __attribute__ ((weak)) _init(void)  {}
}
#endif

#if 1 // ========================== Uniq ID ====================================
#if defined STM32L1XX
#if STM32L1XX_PROD_CAT == 1 || STM32L1XX_PROD_CAT == 2
#define UNIQ_ID_BASE    0x1FF80050
#else
#define UNIQ_ID_BASE    0x1FF800D0
#endif
static inline uint32_t GetUniqID1() {
    return *((uint32_t*)(UNIQ_ID_BASE + 0x00));
}
static inline uint32_t GetUniqID2() {
    return *((uint32_t*)(UNIQ_ID_BASE + 0x04));
}
static inline uint32_t GetUniqID3() {
    return *((uint32_t*)(UNIQ_ID_BASE + 0x14));
}
#endif


#endif

#if 1 // ======================= Virtual Timer =================================
#define TIMER_KL    TRUE
/*
 * Example:
 * TmrKL_t TmrCheckBtn {MS2ST(54), EVT_BUTTONS, tktPeriodic};
 * TmrCheckBtn.InitAndStart(chThdGetSelfX());
 */

void TmrKLCallback(void *p);    // Universal VirtualTimer callback

enum TmrKLType_t {tktOneShot, tktPeriodic};

class TmrKL_t {
private:
    virtual_timer_t Tmr;
    void StartI() { chVTSetI(&Tmr, Period, TmrKLCallback, this); }
    thread_t *PThread;
    systime_t Period;
    eventmask_t EvtMsk;
    TmrKLType_t TmrType;
public:
    void InitAndStart(thread_t *APThread) {
        PThread = APThread;
        Start();
    }
    void InitAndStart() {
        PThread = chThdGetSelfX();
        Start();
    }

    void Init(thread_t *APThread) { PThread = APThread; }
    void Init() { PThread = chThdGetSelfX(); }

    void Start() {
        chSysLock();
        StartI();
        chSysUnlock();
    }
    void Start(systime_t NewPeriod) {
        chSysLock();
        chVTResetI(&Tmr);
        Period = NewPeriod;
        StartI();
        chSysUnlock();
    }
    void StartIfNotRunning() {
        chSysLock();
        if(!chVTIsArmedI(&Tmr)) StartI();
        chSysUnlock();
    }
    void Stop() { chVTReset(&Tmr); }
    void Restart() {
        chVTReset(&Tmr);
        Start();
    }
    void CallbackHandler() {    // Call it inside callback
        chSysLockFromISR();
        chEvtSignalI(PThread, EvtMsk);
        if(TmrType == tktPeriodic) StartI();
        chSysUnlockFromISR();
    }
    TmrKL_t(systime_t APeriod, eventmask_t AEvtMsk, TmrKLType_t AType) :
        PThread(nullptr), Period(APeriod), EvtMsk(AEvtMsk), TmrType(AType) {}
    // Dummy period is set
    TmrKL_t(eventmask_t AEvtMsk, TmrKLType_t AType) :
            PThread(nullptr), Period(S2ST(9)), EvtMsk(AEvtMsk), TmrType(AType) {}
};
#endif

#if 1 // ========================== Random =====================================
static inline int Random(int LowInclusive, int HighInclusive) {
    return (rand() % (HighInclusive + 1 - LowInclusive)) + LowInclusive;
}
static inline void RandomSeed(unsigned int Seed) { srand(Seed); }
#endif

#if 0 // =========================== Time ======================================
static inline bool TimeElapsed(systime_t *PSince, uint32_t Delay_ms) {
    chSysLock();
    bool Rslt = (chVTGetSystemTimeX() - *PSince) > MS2ST(Delay_ms);
    if(Rslt) *PSince = chVTGetSystemTimeX();
    chSysUnlock();
    return Rslt;
}
#endif

#if 1 // ========================== Simple delay ===============================
static inline void DelayLoop(volatile uint32_t ACounter) { while(ACounter--); }
//static inline void Delay_ms(uint32_t Ams) {
//    volatile uint32_t __ticks = (Clk.AHBFreqHz / 4000) * Ams;
//    DelayLoop(__ticks);
//}
#endif

#if 0 // ======================= Power and backup unit =========================
#define REBOOT()                SCB_AIRCR = (AIRCR_VECTKEY | 0x04)

#if defined STM32F2XX || defined STM32F4XX || defined STM32F10X_LD_VL
namespace BackupSpc {
static inline void EnableAccess() {
    rccEnablePWRInterface(FALSE);
    rccEnableBKPInterface(FALSE);
    PWR->CR |= PWR_CR_DBP;
}
static inline void DisableAccess() { PWR->CR &= ~PWR_CR_DBP; }

static inline void Reset() {
    RCC->BDCR |=  RCC_BDCR_BDRST;
    RCC->BDCR &= ~RCC_BDCR_BDRST;
}
} // namespace
#endif // STM32F2xx/F4xx
#endif

#if 0 // ============================= RTC =====================================
namespace Rtc {
#if defined STM32F10X_LD_VL
// Wait until the RTC registers (RTC_CNT, RTC_ALR and RTC_PRL) are synchronized with RTC APB clock.
// Required after an APB reset or an APB clock stop.
static inline void WaitForSync() {
    RTC->CRL &= (uint16_t)~RTC_CRL_RSF; //Clear RSF flag
    while((RTC->CRL & RTC_CRL_RSF) == 0);
}

// Waits until last write operation on RTC registers has finished.
// This function must be called before any write to RTC registers.
static inline void WaitForLastTask() { while((RTC->CRL & RTC_CRL_RTOFF) == 0); }

static inline void SetClkSrcLSE() {
    RCC->BDCR &= ~RCC_BDCR_RTCSEL;  // Clear bits
    RCC->BDCR |=  RCC_BDCR_RTCSEL_LSE;
}
static inline void EnableClk() { RCC->BDCR |= RCC_BDCR_RTCEN; }

#define RTC_LSB_MASK     ((uint32_t)0x0000FFFF)  // RTC LSB Mask
#define PRLH_MSB_MASK    ((uint32_t)0x000F0000)  // RTC Prescaler MSB Mask
static inline void EnterConfigMode() { RTC->CRL |= RTC_CRL_CNF; }
static inline void ExitConfigMode()  { RTC->CRL &= ~((uint16_t)RTC_CRL_CNF); }

static inline void SetPrescaler(uint32_t PrescalerValue) {
    EnterConfigMode();
    RTC->PRLH = (PrescalerValue & PRLH_MSB_MASK) >> 16;
    RTC->PRLL = (PrescalerValue & RTC_LSB_MASK);
    ExitConfigMode();
}

static inline void SetCounter(uint32_t CounterValue) {
    EnterConfigMode();
    RTC->CNTH = CounterValue >> 16;
    RTC->CNTL = (CounterValue & RTC_LSB_MASK);
    ExitConfigMode();
}

static inline void EnableSecondIRQ() {
    WaitForLastTask();
    RTC->CRH |= RTC_CRH_SECIE;
}
static inline void ClearSecondIRQFlag() {
    RTC->CRL &= ~RTC_CRL_SECF;
}
#endif
} // namespace
#endif

#if 1 // =========================== HW Timers =================================
enum TmrTrigInput_t {tiITR0=0x00, tiITR1=0x10, tiITR2=0x20, tiITR3=0x30, tiTIED=0x40, tiTI1FP1=0x50, tiTI2FP2=0x60, tiETRF=0x70};
enum TmrMasterMode_t {mmReset=0x00, mmEnable=0x10, mmUpdate=0x20, mmComparePulse=0x30, mmCompare1=0x40, mmCompare2=0x50, mmCompare3=0x60, mmCompare4=0x70};
enum TmrSlaveMode_t {smDisable=0, smEncoder1=1, smEncoder2=2, smEncoder3=3, smReset=4, smGated=5, smTrigger=6, smExternal=7};
enum ExtTrigPol_t {etpInverted=0x8000, etpNotInverted=0x0000};
enum ExtTrigPsc_t {etpOff=0x0000, etpDiv2=0x1000, etpDiv4=0x2000, etpDiv8=0x30000};

#if defined STM32F10X_LD_VL
#define TMR_PCCR(PTimer, AChannel)  ((uint16_t*)(&PTimer->CCR1 + ((AChannel-1) * 2)))
#else
#define TMR_PCCR(PTimer, AChannel)  ((uint32_t*)(&PTimer->CCR1 + AChannel-1))
#endif
#define TMR_ENABLE(PTimer)          PTimer->CR1 |=  TIM_CR1_CEN;
#define TMR_DISABLE(PTimer)         PTimer->CR1 &= ~TIM_CR1_CEN;
#define TMR_GENERATE_UPD(PTimer)    PTimer->EGR = TIM_EGR_UG;

class Timer_t {
protected:
    TIM_TypeDef* ITmr;
public:
    Timer_t(TIM_TypeDef *APTimer) : ITmr(APTimer) {}
    void Init() const;
    void Deinit() const;
    void Enable() const { TMR_ENABLE(ITmr); }
    void Disable() const { TMR_DISABLE(ITmr); }
    void SetUpdateFrequency(uint32_t FreqHz) const;
    void SetTopValue(uint32_t Value) const { ITmr->ARR = Value; }
    uint32_t GetTopValue() const { return ITmr->ARR; }
    void SetupPrescaler(uint32_t PrescaledFreqHz) const;
    void SetCounter(uint32_t Value) const { ITmr->CNT = Value; }
    uint32_t GetCounter() const { return ITmr->CNT; }
    // Master/Slave
    void SetTriggerInput(TmrTrigInput_t TrgInput) const {
        uint16_t tmp = ITmr->SMCR;
        tmp &= ~TIM_SMCR_TS;   // Clear bits
        tmp |= (uint16_t)TrgInput;
        ITmr->SMCR = tmp;
    }
    void MasterModeSelect(TmrMasterMode_t MasterMode) const {
        uint16_t tmp = ITmr->CR2;
        tmp &= ~TIM_CR2_MMS;
        tmp |= (uint16_t)MasterMode;
        ITmr->CR2 = tmp;
    }
    void SlaveModeSelect(TmrSlaveMode_t SlaveMode) const {
        uint16_t tmp = ITmr->SMCR;
        tmp &= ~TIM_SMCR_SMS;
        tmp |= (uint16_t)SlaveMode;
        ITmr->SMCR = tmp;
    }
    // DMA, Irq, Evt
    void EnableDmaOnTrigger() const { ITmr->DIER |= TIM_DIER_TDE; }
    void GenerateUpdateEvt()  const { ITmr->EGR = TIM_EGR_UG; }
    void EnableIrqOnUpdate()  const { ITmr->DIER |= TIM_DIER_UIE; }
    void EnableIrq(uint32_t IrqChnl, uint32_t IrqPriority) const { nvicEnableVector(IrqChnl, IrqPriority); }
    void ClearIrqPendingBit() const { ITmr->SR &= ~TIM_SR_UIF; }
};
#endif

#if 1 // ===================== Simple pin manipulations ========================
enum PinPullUpDown_t {
    pudNone = 0b00,
    pudPullUp = 0b01,
    pudPullDown = 0b10
};

struct PinInputSetup_t {
    GPIO_TypeDef *PGpio;
    uint16_t Pin;
    PinPullUpDown_t PullUpDown;
};

struct PwmSetup_t {
    GPIO_TypeDef *PGpio;
    uint16_t Pin;
    TIM_TypeDef *PTimer;
    uint32_t TimerChnl;
    Inverted_t Inverted;
    PinOutMode_t OutputType;
    uint32_t TopValue;
};

#if defined STM32F2XX || defined STM32F4XX
enum PinSpeed_t {
    ps2MHz  = 0b00,
    ps25MHz = 0b01,
    ps50MHz = 0b10,
    ps100MHz = 0b11
};
#define PIN_SPEED_DEFAULT   ps50MHz
#elif defined STM32L1XX
enum PinSpeed_t {
    psVeryLow = 0b00,
    psLow = 0b01,
    psMedium = 0b10,
    psHigh = 0b11
};
#define PIN_SPEED_DEFAULT   psMedium
#elif defined STM32F0XX
enum PinSpeed_t {
    psLow = 0b00,
    psMedium = 0b01,
    psHigh = 0b11
};
#define PIN_SPEED_DEFAULT   psMedium
#elif defined STM32L4XX
enum PinSpeed_t {
    psLow = 0b00,
    psMedium = 0b01,
    psHigh = 0b10,
    psVeryHigh = 0b11
};
#define PIN_SPEED_DEFAULT   psMedium
#endif

enum AlterFunc_t {
    AF0=0, AF1=1, AF2=2, AF3=3, AF4=4, AF5=5, AF6=6, AF7=7,
#if defined STM32F2XX || defined STM32F4XX || defined STM32L1XX || defined STM32L4XX
    AF8=8, AF9=9,AF10=10, AF11=11, AF12=12, AF13=13, AF14=14, AF15=15
#endif
};

// Set/clear
#if defined STM32L1XX || defined STM32F2XX || defined STM32F4XX || defined STM32F042x6
__always_inline
static inline void PinSetHi(GPIO_TypeDef *PGpio, uint16_t APin) { PGpio->BSRRL = (1 << APin); }
__always_inline
static inline void PinSetLo(GPIO_TypeDef *PGpio, uint16_t APin) { PGpio->BSRRH = (1 << APin); }

#elif defined STM32F0XX || defined STM32F10X_LD_VL || defined STM32L4XX
__always_inline
static inline void PinSetHi(GPIO_TypeDef *PGpio, uint32_t APin) { PGpio->BSRR = 1 << APin; }
__always_inline
static inline void PinSetLo(GPIO_TypeDef *PGpio, uint32_t APin) { PGpio->BRR = 1 << APin;  }
#endif
__always_inline
static inline void PinToggle(GPIO_TypeDef *PGpio, uint32_t APin) { PGpio->ODR ^= 1 << APin; }
// Check input
__always_inline
static inline bool PinIsHi(GPIO_TypeDef *PGpio, uint32_t APin) {
    return PGpio->IDR & (1 << APin);
}
__always_inline
static inline bool PinIsLo(GPIO_TypeDef *PGpio, uint32_t APin) {
    return !(PGpio->IDR & (1 << APin));
}

// Setup
static void PinClockEnable(GPIO_TypeDef *PGpioPort) {
#if defined STM32F2XX || defined STM32F4XX
    if     (PGpioPort == GPIOA) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    else if(PGpioPort == GPIOB) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    else if(PGpioPort == GPIOC) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    else if(PGpioPort == GPIOD) RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    else if(PGpioPort == GPIOE) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
#elif defined STM32F10X_LD_VL
    if     (PGpioPort == GPIOA) RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    else if(PGpioPort == GPIOB) RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
    else if(PGpioPort == GPIOC) RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
    else if(PGpioPort == GPIOD) RCC->APB2ENR |= RCC_APB2ENR_IOPDEN;
#elif defined STM32L4XX
    if     (PGpioPort == GPIOA) RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
    else if(PGpioPort == GPIOB) RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
    else if(PGpioPort == GPIOC) RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;
    else if(PGpioPort == GPIOD) RCC->AHB2ENR |= RCC_AHB2ENR_GPIODEN;
    else if(PGpioPort == GPIOH) RCC->AHB2ENR |= RCC_AHB2ENR_GPIOHEN;
#else
    if     (PGpioPort == GPIOA) RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    else if(PGpioPort == GPIOB) RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    else if(PGpioPort == GPIOC) RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
#ifdef GPIOD
    else if(PGpioPort == GPIOD) RCC->AHBENR |= RCC_AHBENR_GPIODEN;
#endif
#ifdef GPIOF
    else if(PGpioPort == GPIOF) RCC->AHBENR |= RCC_AHBENR_GPIOFEN;
#endif

#endif // MCU type
}

// ==== Fast setup ====
static inline void PinSetupModeOut(GPIO_TypeDef *PGpioPort, const uint16_t APinNumber) {
    uint8_t Offset = APinNumber*2;
    PGpioPort->MODER &= ~(0b11 << Offset);  // clear previous bits
    PGpioPort->MODER |=   0b01 << Offset;   // Set new bits
}
static inline void PinSetupModeAnalog(GPIO_TypeDef *PGpioPort, const uint16_t APinNumber) {
    PGpioPort->MODER |= 0b11 << (APinNumber*2);
}

// ==== Full-sized setup ====
static inline void PinSetupOut(
        GPIO_TypeDef *PGpioPort,
        const uint16_t APinNumber,
        const PinOutMode_t PinOutMode,
        const PinSpeed_t ASpeed = PIN_SPEED_DEFAULT
        ) {
    // Clock
    PinClockEnable(PGpioPort);
#if defined STM32F10X_LD_VL
    uint32_t CnfMode = ((uint32_t)PinOutMode << 2) | (uint32_t)ASpeed;
    if(APinNumber < 8) {
        uint8_t Offset = APinNumber*4;
        PGpioPort->CRL &= ~((uint32_t)(0b1111 << Offset));  // Clear both mode and cnf
        PGpioPort->CRL |= CnfMode << Offset;
    }
    else {
        uint8_t Offset = (APinNumber - 8) * 4;
        PGpioPort->CRH &= ~((uint32_t)(0b1111 << Offset));  // Clear both mode and cnf
        PGpioPort->CRH |= CnfMode << Offset;
    }
#else
    uint8_t Offset = APinNumber*2;
    // Setup mode
    PGpioPort->MODER &= ~(0b11 << Offset);  // clear previous bits
    PGpioPort->MODER |=   0b01 << Offset;   // Set new bits
    // Setup output type
    PGpioPort->OTYPER &= ~(1<<APinNumber);
    PGpioPort->OTYPER |= (uint32_t)PinOutMode << APinNumber;
    // Setup Pull-Up or Pull-Down
    PGpioPort->PUPDR &= ~(0b11 << Offset); // clear previous bits
    PGpioPort->PUPDR |= (uint32_t)pudNone << Offset;
    // Setup speed
    PGpioPort->OSPEEDR &= ~(0b11 << Offset); // clear previous bits
    PGpioPort->OSPEEDR |= (uint32_t)ASpeed << Offset;
#endif
}

static inline void PinSetupInput(
        GPIO_TypeDef *PGpio,
        const uint16_t PinN,
        const PinPullUpDown_t PullUpDown,
        const PinSpeed_t ASpeed = PIN_SPEED_DEFAULT) {
    uint8_t Offset = PinN*2;
    // Clock
    PinClockEnable(PGpio);
#if defined STM32F10X_LD_VL
        uint32_t CnfMode;
        if(APullUpDown == pudNone) CnfMode = 0b0100;
        else {
            CnfMode = 0b1000;
            if(APullUpDown == pudPullDown) PGpioPort->ODR &= ~((uint32_t)(1<<APinNumber));
            else PGpioPort->ODR |= (uint32_t)(1<<APinNumber);
        }
        if(APinNumber < 8) {
            uint8_t Offset = APinNumber*4;
            PGpioPort->CRL &= ~((uint32_t)(0b1111 << Offset));  // Clear both mode and cnf
            PGpioPort->CRL |= CnfMode << Offset;
        }
        else {
            uint8_t Offset = (APinNumber - 8) * 4;
            PGpioPort->CRH &= ~((uint32_t)(0b1111 << Offset));  // Clear both mode and cnf
            PGpioPort->CRH |= CnfMode << Offset;
        }
#else
        // Setup mode
        PGpio->MODER &= ~(0b11 << Offset); // clear previous bits
        // Setup Pull-Up or Pull-Down
        PGpio->PUPDR &= ~(0b11 << Offset); // clear previous bits
        PGpio->PUPDR |= (uint32_t)PullUpDown << Offset;
#endif
}

static inline void PinSetupAnalog(GPIO_TypeDef *PGpioPort, const uint16_t APinNumber) {
    // Clock
    PinClockEnable(PGpioPort);
#if defined STM32F10X_LD_VL
    if(APinNumber < 8) {
        uint8_t Offset = APinNumber*4;
        PGpioPort->CRL &= ~((uint32_t)(0b1111 << Offset));  // Clear both mode and cnf
    }
    else {
        uint8_t Offset = (APinNumber - 8) * 4;
        PGpioPort->CRH &= ~((uint32_t)(0b1111 << Offset));  // Clear both mode and cnf
    }
#else
    // Setup mode
    PGpioPort->MODER |= 0b11 << (APinNumber*2);  // Set new bits
#endif
}

static inline void PinSetupAlterFunc(
        GPIO_TypeDef *PGpioPort,
        const uint16_t APinNumber,
        const PinOutMode_t PinOutMode,
        const PinPullUpDown_t APullUpDown,
        const AlterFunc_t AAlterFunc,
        const PinSpeed_t ASpeed = PIN_SPEED_DEFAULT) {
    // Clock
    PinClockEnable(PGpioPort);
#if defined STM32F10X_LD_VL
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;     // Enable AFIO clock
    // Setup
    uint32_t CnfMode = ((uint32_t)PinOutMode << 2) | 0b1000 | (uint32_t)ASpeed;
    if(APinNumber < 8) {
        uint8_t Offset = APinNumber*4;
        PGpioPort->CRL &= ~((uint32_t)(0b1111 << Offset));  // Clear both mode and cnf
        PGpioPort->CRL |= CnfMode << Offset;
    }
    else {
        uint8_t Offset = (APinNumber - 8) * 4;
        PGpioPort->CRH &= ~((uint32_t)(0b1111 << Offset));  // Clear both mode and cnf
        PGpioPort->CRH |= CnfMode << Offset;
    }
#else
    uint32_t Offset = APinNumber*2;
    // Setup mode
    PGpioPort->MODER &= ~(0b11 << Offset);  // clear previous bits
    PGpioPort->MODER |= 0b10 << Offset;     // Set new bits (AF mode)
    // Setup output type
    if(PinOutMode == omPushPull) PGpioPort->OTYPER &= ~(1<<APinNumber);
    else PGpioPort->OTYPER |= 1 << APinNumber;  // Open Drain
    // Setup Pull-Up or Pull-Down
    PGpioPort->PUPDR &= ~(0b11 << Offset); // clear previous bits
    PGpioPort->PUPDR |= (uint32_t)APullUpDown << Offset;
    // Setup speed
    PGpioPort->OSPEEDR &= ~(0b11 << Offset); // clear previous bits
    PGpioPort->OSPEEDR |= (uint32_t)ASpeed << Offset;
    // Setup Alternate Function
    uint32_t n = (APinNumber <= 7)? 0 : 1;      // 0 if 0...7, 1 if 8..15
    Offset = 4 * ((APinNumber <= 7)? APinNumber : APinNumber - 8);
    PGpioPort->AFR[n] &= ~(0b1111 << Offset);
    PGpioPort->AFR[n] |= (uint32_t)AAlterFunc << Offset;
#endif
}

// ==== Port setup ====
static inline void PortInit(GPIO_TypeDef *PGpioPort,
        const PinOutMode_t PinOutMode,
        const PinPullUpDown_t APullUpDown = pudNone,
        const PinSpeed_t ASpeed = PIN_SPEED_DEFAULT
        ) {
    // Clock
    PinClockEnable(PGpioPort);
    // Setup output type
    if(PinOutMode == omPushPull) PGpioPort->OTYPER = 0;
    else PGpioPort->OTYPER = 0xFFFF;
    // Setup Pull-Up or Pull-Down
    if(APullUpDown == pudPullUp) PGpioPort->PUPDR = 0x55555555; // 01 01 01 01...
    else if(APullUpDown == pudPullDown) PGpioPort->PUPDR = 0xAAAAAAAA; // 10 10 10 10...
    else PGpioPort->PUPDR = 0x00000000; // no pull
    // Setup speed
    switch(ASpeed) {
#if defined STM32L1XX
        case psVeryLow:  PGpioPort->OSPEEDR = 0x00000000; break;
        case psLow:      PGpioPort->OSPEEDR = 0x55555555; break;
        case psMedium:   PGpioPort->OSPEEDR = 0xAAAAAAAA; break;
        case psHigh:     PGpioPort->OSPEEDR = 0xFFFFFFFF; break;
#elif defined STM32L4XX
        case psVeryHigh: PGpioPort->OSPEEDR = 0xFFFFFFFF; break;
        case psLow:      PGpioPort->OSPEEDR = 0x00000000; break;
        case psMedium:   PGpioPort->OSPEEDR = 0x55555555; break;
        case psHigh:     PGpioPort->OSPEEDR = 0xAAAAAAAA; break;
#elif defined STM32F0XX
        case psLow:      PGpioPort->OSPEEDR = 0x00000000; break;
        case psMedium:   PGpioPort->OSPEEDR = 0x55555555; break;
        case psHigh:     PGpioPort->OSPEEDR = 0xFFFFFFFF; break;
#endif
    }
}

__always_inline
static inline void PortSetupOutput(GPIO_TypeDef *PGpioPort) {
    PGpioPort->MODER = 0x55555555;
}
__always_inline
static inline void PortSetupInput(GPIO_TypeDef *PGpioPort) {
    PGpioPort->MODER = 0x00000000;
}

__always_inline
static inline void PortSetValue(GPIO_TypeDef *PGpioPort, uint16_t Data) {
    PGpioPort->ODR = Data;
}
__always_inline
static inline uint16_t PortGetValue(GPIO_TypeDef *PGpioPort) {
    return PGpioPort->IDR;
}
#endif // Simple pin manipulations

#if defined STM32F10X_LD_VL // Disable JTAG, leaving SWD
static inline void JtagDisable() {
    bool AfioWasEnabled = (RCC->APB2ENR & RCC_APB2ENR_AFIOEN);
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;     // Enable AFIO
    uint32_t tmp = AFIO->MAPR;
    tmp &= ~0x07000000;
    tmp |= 0x02000000;
    AFIO->MAPR = tmp;
    if(!AfioWasEnabled) RCC->APB2ENR &= ~RCC_APB2ENR_AFIOEN;
}
#endif

#if 1 // ===================== Pin classes ========================
class PinOutput_t {
private:
    GPIO_TypeDef *PGpio;
    uint16_t Pin;
    PinOutMode_t OutputType;
public:
    void Init() const { PinSetupOut(PGpio, Pin, OutputType); }
    void Deinit() const { PinSetupAnalog(PGpio, Pin); }
    void Hi() const { PinSetHi(PGpio, Pin); }
    void Lo() const { PinSetLo(PGpio, Pin); }
    PinOutput_t(GPIO_TypeDef *APGPIO, uint16_t APin, PinOutMode_t AOutputType) :
        PGpio(APGPIO), Pin(APin), OutputType(AOutputType) {}
};

class PinInput_t {
private:
    const PinInputSetup_t ISetup;
public:
    void Init() const { PinSetupInput(ISetup.PGpio, ISetup.Pin, ISetup.PullUpDown); }
    void Deinit() const { PinSetupAnalog(ISetup.PGpio, ISetup.Pin); }
    bool IsHi() const { return PinIsHi(ISetup.PGpio, ISetup.Pin); }
    PinInput_t(const PinInputSetup_t &ASetup) : ISetup(ASetup) {}
};


// ==== PWM output ====
/* Example:
 * #define LED_R_PIN { GPIOB, 1, TIM3, 4, invInverted, omPushPull, 255 }
 * PinOutputPWM_t Led {LedPin};
*/
class PinOutputPWM_t : private Timer_t {
private:
    const PwmSetup_t ISetup;
public:
    void Set(const uint16_t AValue) const { *TMR_PCCR(ITmr, ISetup.TimerChnl) = AValue; }    // CCR[N] = AValue
    void Init() const;
    void Deinit() const { Timer_t::Deinit(); PinSetupAnalog(ISetup.PGpio, ISetup.Pin); }
    void SetFrequencyHz(uint32_t FreqHz) const { Timer_t::SetUpdateFrequency(FreqHz); }
    PinOutputPWM_t(const PwmSetup_t &ASetup) : Timer_t(ASetup.PTimer), ISetup(ASetup) {}
};
#endif

#if 1 // ==== External IRQ ====
enum ExtiTrigType_t {ttRising, ttFalling, ttRisingFalling};

#if defined STM32L1XX || defined STM32F4XX || defined STM32L4XX
#define PIN2IRQ_CHNL(Pin)   \
    (((Pin) > 9)? EXTI15_10_IRQn : (((Pin) > 4)? EXTI9_5_IRQn : ((Pin) + EXTI0_IRQn)))
#elif defined STM32F030 || defined STM32F0
#define PIN2IRQ_CHNL(Pin)   \
    (((Pin) > 3)? EXTI4_15_IRQn : (((Pin) > 1)? EXTI2_3_IRQn : EXTI0_1_IRQn))
#endif

/* Example:
 const PinIrq_t GPinDrdy {ACC_DRDY_PIN};
 GPinDrdy.Init(ACC_DRDY_GPIO, pudNone, ttRising);
*/
class PinIrq_t {
public:
    GPIO_TypeDef *PGpio;
    uint16_t PinN;
    PinPullUpDown_t PullUpDown;
    PinIrq_t(GPIO_TypeDef *APGpio, uint16_t APinN, PinPullUpDown_t APullUpDown) :
        PGpio(APGpio), PinN(APinN), PullUpDown(APullUpDown) {}

    void SetTriggerType(ExtiTrigType_t ATriggerType) const {
        uint32_t IrqMsk = 1 << PinN;
        switch(ATriggerType) {
#if defined STM32L4XX
            case ttRising:
                EXTI->RTSR1 |=  IrqMsk;  // Rising trigger enabled
                EXTI->FTSR1 &= ~IrqMsk;  // Falling trigger disabled
                break;
            case ttFalling:
                EXTI->RTSR1 &= ~IrqMsk;  // Rising trigger disabled
                EXTI->FTSR1 |=  IrqMsk;  // Falling trigger enabled
                break;
            case ttRisingFalling:
                EXTI->RTSR1 |=  IrqMsk;  // Rising trigger enabled
                EXTI->FTSR1 |=  IrqMsk;  // Falling trigger enabled
                break;
#else
            case ttRising:
                EXTI->RTSR |=  IrqMsk;  // Rising trigger enabled
                EXTI->FTSR &= ~IrqMsk;  // Falling trigger disabled
                break;
            case ttFalling:
                EXTI->RTSR &= ~IrqMsk;  // Rising trigger disabled
                EXTI->FTSR |=  IrqMsk;  // Falling trigger enabled
                break;
            case ttRisingFalling:
                EXTI->RTSR |=  IrqMsk;  // Rising trigger enabled
                EXTI->FTSR |=  IrqMsk;  // Falling trigger enabled
                break;
#endif
        } // switch
    }

    void Init(ExtiTrigType_t ATriggerType) const {
        // Init pin as input
        PinSetupInput(PGpio, PinN, PullUpDown);
        rccEnableAPB2(RCC_APB2ENR_SYSCFGEN, FALSE); // Enable sys cfg controller
        // Connect EXTI line to the pin of the port
        uint8_t Indx   = PinN / 4;               // Indx of EXTICR register
        uint8_t Offset = (PinN & 0x03) * 4;      // Offset in EXTICR register
        SYSCFG->EXTICR[Indx] &= ~((uint32_t)0b1111 << Offset);  // Clear port-related bits
        // GPIOA requires all zeroes => nothing to do in this case
        if     (PGpio == GPIOB) SYSCFG->EXTICR[Indx] |= (uint32_t)0b0001 << Offset;
        else if(PGpio == GPIOC) SYSCFG->EXTICR[Indx] |= (uint32_t)0b0010 << Offset;
        // Configure EXTI line
        uint32_t IrqMsk = 1 << PinN;
#if defined STM32L4XX
        EXTI->IMR1  |=  IrqMsk;      // Interrupt mode enabled
        EXTI->EMR1  &= ~IrqMsk;      // Event mode disabled
        SetTriggerType(ATriggerType);
        EXTI->PR1    =  IrqMsk;      // Clean irq flag
#else
        EXTI->IMR  |=  IrqMsk;      // Interrupt mode enabled
        EXTI->EMR  &= ~IrqMsk;      // Event mode disabled
        SetTriggerType(ATriggerType);
        EXTI->PR    =  IrqMsk;      // Clean irq flag
#endif
    }
    void EnableIrq(const uint32_t Priority) const { nvicEnableVector(PIN2IRQ_CHNL(PinN), Priority); }
    void DisableIrq() const { nvicDisableVector(PIN2IRQ_CHNL(PinN)); }
#if defined STM32L4XX
    void CleanIrqFlag() const { EXTI->PR1 = (1 << PinN); }
    bool IsIrqPending() const { return BitIsSet(EXTI->PR1, (1 << PinN)); }
    void GenerateIrq()  const { EXTI->SWIER1 = (1 << PinN); }
#else
    void CleanIrqFlag() const { EXTI->PR = (1 << PinN); }
    bool IsIrqPending() const { return BitIsSet(EXTI->PR, (1 << PinN)); }
    void GenerateIrq()  const { EXTI->SWIER = (1 << PinN); }
#endif
};
#endif // EXTI

#if 0 // ============================== IWDG ===================================
enum IwdgPre_t {
    iwdgPre4 = 0x00,
    iwdgPre8 = 0x01,
    iwdgPre16 = 0x02,
    iwdgPre32 = 0x03,
    iwdgPre64 = 0x04,
    iwdgPre128 = 0x05,
    iwdgPre256 = 0x06
};

class IWDG_t {
private:
    void EnableAccess() { IWDG->KR = 0x5555; }
    void SetPrescaler(IwdgPre_t Prescaler) { IWDG->PR = (uint32_t)Prescaler; }
    void SetReload(uint16_t Reload) { IWDG->RLR = Reload; }
public:
    void Reload() { IWDG->KR = 0xAAAA; }
    void Enable() { IWDG->KR = 0xCCCC; }
    void SetTimeout(uint32_t ms) {
        EnableAccess();
        SetPrescaler(iwdgPre256);
        uint32_t Count = (ms * (LSI_FREQ_HZ/1000)) / 256;
        TRIM_VALUE(Count, 0xFFF);
        SetReload(Count);
        Reload();
    }
    bool ResetOccured() {
        if(RCC->CSR & RCC_CSR_IWDGRSTF) {
            RCC->CSR |= RCC_CSR_RMVF;   // Clear flags
            return true;
        }
        else return false;
    }
    void GoSleep(uint32_t Timeout_ms) {
        chSysLock();
        // Start LSI
        Clk.EnableLSI();
        // Start IWDG
        SetTimeout(Timeout_ms);
        Enable();
        // Enter standby mode
        SCB->SCR |= SCB_SCR_SLEEPDEEP;
        PWR->CR = PWR_CR_PDDS;
        PWR->CR |= PWR_CR_CWUF;
        __WFI();
        chSysUnlock();
    }
};
#endif

#if 1 // ============================== Sleep ==================================
namespace Sleep {
static inline void EnterStandby() {
#if defined STM32F0XX || defined STM32L4XX
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
#else
    SCB->SCR |= SCB_SCR_SLEEPDEEP;
#endif

#if defined STM32L4XX
    uint32_t tmp = PWR->CR1 & ~PWR_CR1_LPMS;
    tmp |= PWR_CR1_LPMS_SHUTDOWN;
    PWR->CR1 = tmp;
#else
    PWR->CR = PWR_CR_PDDS;
    PWR->CR |= PWR_CR_CWUF;
#endif
    __WFI();
}

#if defined STM32L4XX
static inline void EnableWakeup1Pin()  { PWR->CR3 |=  PWR_CR3_EWUP1; }
static inline void DisableWakeup1Pin() { PWR->CR3 &= ~PWR_CR3_EWUP1; }
static inline void EnableWakeup2Pin()  { PWR->CR3 |=  PWR_CR3_EWUP2; }
static inline void DisableWakeup2Pin() { PWR->CR3 &= ~PWR_CR3_EWUP2; }
#else
static inline void EnableWakeup1Pin()  { PWR->CSR |=  PWR_CSR_EWUP1; }
static inline void DisableWakeup1Pin() { PWR->CSR &= ~PWR_CSR_EWUP1; }

static inline bool WasInStandby() { return (PWR->CSR & PWR_CSR_SBF); }
static inline void ClearStandbyFlag() { PWR->CR |= PWR_CR_CSBF; }
#endif

}; // namespace
#endif

#if 1 // ============================== SPI ====================================
enum CPHA_t {cphaFirstEdge, cphaSecondEdge};
enum CPOL_t {cpolIdleLow, cpolIdleHigh};
enum SpiBaudrate_t {
    sbFdiv2   = 0b000,
    sbFdiv4   = 0b001,
    sbFdiv8   = 0b010,
    sbFdiv16  = 0b011,
    sbFdiv32  = 0b100,
    sbFdiv64  = 0b101,
    sbFdiv128 = 0b110,
    sbFdiv256 = 0b111,
};

class Spi_t {
private:
    SPI_TypeDef *PSpi;
public:
    Spi_t(SPI_TypeDef *ASpi) : PSpi(ASpi) {}
    // Example: boMSB, cpolIdleLow, cphaFirstEdge, sbFdiv2, bitn8
    void Setup(BitOrder_t BitOrder, CPOL_t CPOL, CPHA_t CPHA,
            SpiBaudrate_t Baudrate, BitNumber_t BitNumber = bitn8) const {
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
        PSpi->CR1 |= ((uint16_t)Baudrate) << 3;                 // Baudrate
#if defined STM32L1XX || defined STM32F10X_LD_VL || defined STM32F4XX
        if(BitNumber == bitn16) PSpi->CR1 |= SPI_CR1_DFF;
        PSpi->CR2 = 0;
#elif defined STM32F030
        PSpi->CR2 = (uint16_t)0b1111 << 8;  // 16 bit data size only
#elif defined STM32F072xB
        if(BitNumber == bitn16) PSpi->CR2 = (uint16_t)0b1111 << 8;
        else PSpi->CR2 = ((uint16_t)0b0111 << 8) | SPI_CR2_FRXTH; // 8 bit
        PSpi->I2SCFGR &= ~((uint16_t)SPI_I2SCFGR_I2SMOD);       // Disable I2S
#elif defined STM32L4XX
        // Generate RXNE when FIFO level is greater or equal to 1/4 (8 bit)
        if(BitNumber == bitn8) PSpi->CR2 |= 0x0700; // 8bit
        else if(BitNumber == bitn16) PSpi->CR2 |= 0x0F00;
        PSpi->CR2 |= SPI_CR2_FRXTH;

#endif
    }
    void Enable ()       const { PSpi->CR1 |=  SPI_CR1_SPE; }
    void Disable()       const { PSpi->CR1 &= ~SPI_CR1_SPE; }
    void EnableTxDma()   const { PSpi->CR2 |=  SPI_CR2_TXDMAEN; }
    void DisableTxDma()  const { PSpi->CR2 &= ~SPI_CR2_TXDMAEN; }
    void EnableRxDma()   const { PSpi->CR2 |=  SPI_CR2_RXDMAEN; }
    void DisableRxDma()  const { PSpi->CR2 &= ~SPI_CR2_RXDMAEN; }
    void SetRxOnly()     const { PSpi->CR1 |=  SPI_CR1_RXONLY; }
    void SetFullDuplex() const { PSpi->CR1 &= ~SPI_CR1_RXONLY; }
#if defined STM32F072xB
    void WaitFTLVLZero() const { while(PSpi->SR & SPI_SR_FTLVL); }
#endif
    void WaitBsyHi2Lo()  const { while(PSpi->SR & SPI_SR_BSY); }
    void ClearRxBuf()    const { while(PSpi->SR & SPI_SR_RXNE) (void)PSpi->DR; }
    uint8_t ReadWriteByte(uint8_t AByte) const {
        *((volatile uint8_t*)&PSpi->DR) = AByte;
        while(!(PSpi->SR & SPI_SR_RXNE));  // Wait for SPI transmission to complete
        return *((volatile uint8_t*)&PSpi->DR);
    }
    uint16_t ReadWriteWord(uint16_t Word) const {
        PSpi->DR = Word;
        while(!(PSpi->SR & SPI_SR_RXNE));
        return PSpi->DR;
    }
#if defined STM32L4XX
    void WriteRead3Bytes(uint8_t *ptr) const {
        *((volatile uint8_t*)&PSpi->DR) = ptr[0];
        *((volatile uint8_t*)&PSpi->DR) = ptr[1];
        *((volatile uint8_t*)&PSpi->DR) = ptr[2];
        while(!(PSpi->SR & SPI_SR_RXNE));
        ptr[0] = *((volatile uint8_t*)&PSpi->DR);
        while(!(PSpi->SR & SPI_SR_RXNE));
        ptr[1] = *((volatile uint8_t*)&PSpi->DR);
        while(!(PSpi->SR & SPI_SR_RXNE));
        ptr[2] = *((volatile uint8_t*)&PSpi->DR);
    }
#endif
};
#endif

#if I2C1_ENABLED // ========================= I2C ==============================
struct i2cParams_t {
    I2C_TypeDef *pi2c;
    GPIO_TypeDef *PGpio;
    uint16_t SclPin;
    uint16_t SdaPin;
    AlterFunc_t PinAF;
    uint32_t BitrateHz;
    // DMA
    const stm32_dma_stream_t *PDmaTx;
    const stm32_dma_stream_t *PDmaRx;
};


/* Example:
 * i2c_t i2c (I2C_ACC, ACC_I2C_GPIO, ACC_I2C_SCL_PIN, ACC_I2C_SDA_PIN,
 * 400000, I2C_ACC_DMA_TX, I2C_ACC_DMA_RX );
 */

#define I2C_DMATX_MODE  STM32_DMA_CR_CHSEL(DmaChnl) |   \
                        DMA_PRIORITY_LOW | \
                        STM32_DMA_CR_MSIZE_BYTE | \
                        STM32_DMA_CR_PSIZE_BYTE | \
                        STM32_DMA_CR_MINC |     /* Memory pointer increase */ \
                        STM32_DMA_CR_DIR_M2P |  /* Direction is memory to peripheral */ \
                        STM32_DMA_CR_TCIE       /* Enable Transmission Complete IRQ */

#define I2C_DMARX_MODE  STM32_DMA_CR_CHSEL(DmaChnl) |   \
                        DMA_PRIORITY_LOW | \
                        STM32_DMA_CR_MSIZE_BYTE | \
                        STM32_DMA_CR_PSIZE_BYTE | \
                        STM32_DMA_CR_MINC |         /* Memory pointer increase */ \
                        STM32_DMA_CR_DIR_P2M |      /* Direction is peripheral to memory */ \
                        STM32_DMA_CR_TCIE           /* Enable Transmission Complete IRQ */

class i2c_t {
private:
    const i2cParams_t *PParams;
    void SendStart()     { PParams->pi2c->CR1 |= I2C_CR1_START; }
    void SendStop()      { PParams->pi2c->CR1 |= I2C_CR1_STOP; }
    void AckEnable()     { PParams->pi2c->CR1 |= I2C_CR1_ACK; }
    void AckDisable()    { PParams->pi2c->CR1 &= ~I2C_CR1_ACK; }
    bool RxIsNotEmpty()  { return (PParams->pi2c->SR1 & I2C_SR1_RXNE); }
    void ClearAddrFlag() { (void)PParams->pi2c->SR1; (void)PParams->pi2c->SR2; }
    void SignalLastDmaTransfer() { PParams->pi2c->CR2 |= I2C_CR2_LAST; }
    // Address and data
    void SendAddrWithWrite(uint8_t Addr) { PParams->pi2c->DR = (uint8_t)(Addr<<1); }
    void SendAddrWithRead (uint8_t Addr) { PParams->pi2c->DR = ((uint8_t)(Addr<<1)) | 0x01; }
    void SendData(uint8_t b) { PParams->pi2c->DR = b; }
    uint8_t ReceiveData() { return PParams->pi2c->DR; }
    // Flags operations
    uint8_t IBusyWait();
    uint8_t WaitEv5();
    uint8_t WaitEv6();
    uint8_t WaitEv8();
    uint8_t WaitAck();
    uint8_t WaitRx();
    uint8_t WaitStop();
    uint8_t WaitBTF();
    binary_semaphore_t BSemaphore;
public:
    bool Error;
    thread_reference_t ThdRef;
    void Init();
    void Standby();
    void Resume();
    void Reset();
    void ScanBus();
    uint8_t WriteRead(uint8_t Addr, uint8_t *WPtr, uint8_t WLength, uint8_t *RPtr, uint8_t RLength);
    uint8_t WriteWrite(uint8_t Addr, uint8_t *WPtr1, uint8_t WLength1, uint8_t *WPtr2, uint8_t WLength2);
    uint8_t Write(uint8_t Addr, uint8_t *WPtr1, uint8_t WLength1);
    i2c_t(const i2cParams_t *APParams) : PParams(APParams),
                Error(false), ThdRef(nullptr) {}
};

#if I2C1_ENABLED
extern i2c_t i2c1;
#endif
#endif // i2c common

#if 0 // ====================== FLASH & EEPROM =================================
#define FLASH_LIB_KL
#define EEPROM_BASE_ADDR    ((uint32_t)0x08080000)
// ==== Flash keys ====
#define FLASH_PDKEY1    ((uint32_t)0x04152637) // Flash power down key1
// Flash power down key2: used with FLASH_PDKEY1 to unlock the RUN_PD bit in FLASH_ACR
#define FLASH_PDKEY2    ((uint32_t)0xFAFBFCFD)
#define FLASH_PEKEY1    ((uint32_t)0x89ABCDEF) // Flash program erase key1
// Flash program erase key: used with FLASH_PEKEY2 to unlock the write access
// to the FLASH_PECR register and data EEPROM
#define FLASH_PEKEY2    ((uint32_t)0x02030405)
#define FLASH_PRGKEY1   ((uint32_t)0x8C9DAEBF) // Flash program memory key1
// Flash program memory key2: used with FLASH_PRGKEY2 to unlock the program memory
#define FLASH_PRGKEY2   ((uint32_t)0x13141516)
#define FLASH_OPTKEY1   ((uint32_t)0xFBEAD9C8) // Flash option key1
// Flash option key2: used with FLASH_OPTKEY1 to unlock the write access to the option byte block
#define FLASH_OPTKEY2   ((uint32_t)0x24252627)

#define FLASH_WAIT_TIMEOUT  36000
class Flash_t {
public:
    static uint8_t GetStatus() {
        if(FLASH->SR & FLASH_SR_BSY) return BUSY;
        else if(FLASH->SR & FLASH_SR_WRPERR) return WRITE_PROTECT;
        else if(FLASH->SR & (uint32_t)0x1E00) return FAILURE;
        else return OK;
    }
    static uint8_t WaitForLastOperation() {
        uint32_t Timeout = FLASH_WAIT_TIMEOUT;
        while(Timeout--) {
            // Get status
            uint8_t status = GetStatus();
            if(status != BUSY) return status;
        }
        return TIMEOUT;
    }
    static void UnlockEE() {
        if(FLASH->PECR & FLASH_PECR_PELOCK) {
            // Unlocking the Data memory and FLASH_PECR register access
            chSysLock();
            FLASH->PEKEYR = FLASH_PEKEY1;
            FLASH->PEKEYR = FLASH_PEKEY2;
            chSysUnlock();
            FLASH->SR = FLASH_SR_WRPERR;        // Clear WriteProtectErr
            FLASH->PECR &= ~FLASH_PECR_FTDW;    // Disable fixed time programming
        }
    }
    static void LockEE() { FLASH->PECR |= FLASH_PECR_PELOCK; }
};

class Eeprom_t : private Flash_t {
public:
    uint32_t Read32(uint32_t Addr) { return *((uint32_t*)(Addr + EEPROM_BASE_ADDR)); }
    uint8_t Write32(uint32_t Addr, uint32_t W);
    void ReadBuf(void *PDst, uint32_t Sz, uint32_t Addr);
    uint8_t WriteBuf(void *PSrc, uint32_t Sz, uint32_t Addr);
};


#endif

#if 1 // =========================== Clocking ==================================
#if defined STM32L1XX
#include "stm32l1xx.h"
/*
 * Right after reset, CPU works on internal MSI source.
 * To switch to external src (HSE) without dividing (i.e. SysClk == CrystalFreq),
 * call SwitchToHSE(), and then optionally HSIDisable().
 * To switch from HSE to HSI, call SwitchToHSI() then optionally HSEDisable().
 * To switch to PLL, disable it first with PLLDisable(), then setup dividers
 * with SetupPLLDividers(), then call SwitchToPLL(). Then disable HSI if needed.
 *
 * Do not forget to update Freq values after switching.
 *
 * AHB  freq max = 32 MHz;
 * APB1 freq max = 32 MHz;
 * APB2 freq max = 32 MHz;
 */

#define HSI_FREQ_HZ         16000000    // Freq of internal generator, not adjustable
#define LSI_FREQ_HZ         37000       // Freq of internal generator, not adjustable

enum ClkSrc_t {csHSI, csHSE, csPLL, csMSI};
enum PllMul_t {
    pllMul3 = 0b0000,
    pllMul4 = 0b0001,
    pllMul6 = 0b0010,
    pllMul8 = 0b0011,
    pllMul12= 0b0100,
    pllMul16= 0b0101,
    pllMul24= 0b0110,
    pllMul32= 0b0111,
    pllMul48= 0b1000,
};

enum PllDiv_t {pllDiv2=0b01, pllDiv3=0b10, pllDiv4=0b11};

enum AHBDiv_t {
    ahbDiv1=0b0000,
    ahbDiv2=0b1000,
    ahbDiv4=0b1001,
    ahbDiv8=0b1010,
    ahbDiv16=0b1011,
    ahbDiv64=0b1100,
    ahbDiv128=0b1101,
    ahbDiv256=0b1110,
    ahbDiv512=0b1111
};
enum APBDiv_t {apbDiv1=0b000, apbDiv2=0b100, apbDiv4=0b101, apbDiv8=0b110, apbDiv16=0b111};

class Clk_t {
private:
    uint8_t EnableHSE();
    uint8_t EnablePLL();
    uint8_t EnableMSI();
public:
    // Frequency values
    uint32_t AHBFreqHz;     // HCLK: AHB Bus, Core, Memory, DMA; 32 MHz max
    uint32_t APB1FreqHz;    // PCLK1: APB1 Bus clock; 32 MHz max
    uint32_t APB2FreqHz;    // PCLK2: APB2 Bus clock; 32 MHz max
    // SysClk switching
    uint8_t SwitchToHSI();
    uint8_t SwitchToHSE();
    uint8_t SwitchToPLL();
    uint8_t SwitchToMSI();
    void DisableHSE() { RCC->CR &= ~RCC_CR_HSEON; }
    uint8_t EnableHSI();
    void DisableHSI() { RCC->CR &= ~RCC_CR_HSION; }
    void DisablePLL() { RCC->CR &= ~RCC_CR_PLLON; }
    void DisableMSI() { RCC->CR &= ~RCC_CR_MSION; }
    // MSI
    void SetMSI4MHz() {
        uint32_t tmp = RCC->ICSCR & (~RCC_ICSCR_MSIRANGE);
        tmp |= RCC_ICSCR_MSIRANGE_6;
        RCC->ICSCR = tmp;
    }
    void SetupBusDividers(AHBDiv_t AHBDiv, APBDiv_t APB1Div, APBDiv_t APB2Div);
    uint8_t SetupPLLMulDiv(PllMul_t PllMul, PllDiv_t PllDiv);
    void UpdateFreqValues();
    //void UpdateSysTick() { SysTick->LOAD = AHBFreqHz / CH_FREQUENCY - 1; }
    void SetupFlashLatency(uint8_t AHBClk_MHz);
    // LSI
    void EnableLSI() {
        RCC->CSR |= RCC_CSR_LSION;
        while(!(RCC->CSR & RCC_CSR_LSIRDY));
    }
    void DisableLSI() { RCC->CSR &= RCC_CSR_LSION; }
    // LSE
    void StartLSE() {
        RCC->APB1ENR |= RCC_APB1ENR_PWREN;
        PWR->CR |= PWR_CR_DBP;
        RCC->CSR |= RCC_CSR_LSEON;
    }
    bool IsLseOn() { return (RCC->CSR & RCC_CSR_LSERDY); }
    void DisableLSE() {
        RCC->APB1ENR |= RCC_APB1ENR_PWREN;
        PWR->CR |= PWR_CR_DBP;
        RCC->CSR &= ~RCC_CSR_LSEON;
    }

    void PrintFreqs();
};

// ==== V Core ====
enum VCore_t {vcore1V2=0b11, vcore1V5=0b10, vcore1V8=0b01};
void SetupVCore(VCore_t AVCore);

#elif defined STM32F0XX
/*
 * Right after reset, CPU works on internal (HSI) source.
 * To switch to external src (HSE) without dividing (i.e. SysClk == CrystalFreq),
 * call SwitchToHSE(), and then optionally HSIDisable().
 * To switch from HSE to HSI, call SwitchToHSI() then optionally HSEDisable().
 * To switch to PLL, disable it first with PLLDisable(), then setup dividers
 * with SetupPLLDividers(), then call SwitchToPLL(). Then disable HSI if needed.
 *
 * Do not forget to update Freq values after switching.
 *
 * Keep in mind that Flash latency need to be increased at higher speeds.
 * Tune it with SetupFlashLatency.
 *
 * AHB  freq max = 48 MHz;
 * APB  freq max = 48 MHz;
 */

#define HSI_FREQ_HZ     8000000 // Freq of internal generator, not adjustable
#define HSI48_FREQ_HZ   48000000

enum PllMul_t {
    pllMul2=0,
    pllMul3=1,
    pllMul4=2,
    pllMul5=3,
    pllMul6=4,
    pllMul7=5,
    pllMul8=6,
    pllMul9=7,
    pllMul10=8,
    pllMul11=9,
    pllMul12=10,
    pllMul13=11,
    pllMul14=12,
    pllMul15=13,
    pllMul16=14
};

#ifdef STM32F030
enum PllSrc_t {plsHSIdiv2=0b00, plsHSIdivPREDIV=0b01, plsHSEdivPREDIV=0b10};
enum ClkSrc_t {csHSI=0b00, csHSE=0b01, csPLL=0b10};
#else
enum PllSrc_t {plsHSIdiv2=0b00, plsHSIdivPREDIV=0b01, plsHSEdivPREDIV=0b10, plsHSI48divPREDIV=0b11};
enum ClkSrc_t {csHSI=0b00, csHSE=0b01, csPLL=0b10, csHSI48=0b11};
#endif

enum AHBDiv_t {
    ahbDiv1=0b0000,
    ahbDiv2=0b1000,
    ahbDiv4=0b1001,
    ahbDiv8=0b1010,
    ahbDiv16=0b1011,
    ahbDiv64=0b1100,
    ahbDiv128=0b1101,
    ahbDiv256=0b1110,
    ahbDiv512=0b1111
};
enum APBDiv_t {apbDiv1=0b000, apbDiv2=0b100, apbDiv4=0b101, apbDiv8=0b110, apbDiv16=0b111};

class Clk_t {
private:
    uint8_t EnableHSE();
    uint8_t EnableHSI();
    uint8_t EnablePLL();
    // To Hsi48 and back again
    uint32_t ISavedAhbDividers;
#ifdef RCC_CFGR_SW_HSI48
    bool IHsi48WasOn;
#endif
public:
    // Frequency values
    uint32_t AHBFreqHz;     // HCLK: AHB Bus, Core, Memory, DMA; 48 MHz max
    uint32_t APBFreqHz;     // PCLK: APB Bus clock; 48 MHz max
    uint8_t TimerClkMulti = 1;
    // SysClk switching
    uint8_t SwitchTo(ClkSrc_t AClkSrc);
#if 1 // To Hsi48 and back again
    void SwitchToHsi48();
    void SwitchToHsi();
#endif
    // Clk Enables
    uint8_t EnableHSI48();
    void EnableCRS();
    void EnableCSS()    { RCC->CR  |=  RCC_CR_CSSON; }
    // Clk Disables
    void DisableCSS()   { RCC->CR  &= ~RCC_CR_CSSON; }
    void DisableHSE()   { RCC->CR  &= ~RCC_CR_HSEON; }
    void DisableHSI()   { RCC->CR  &= ~RCC_CR_HSION; }
    void DisablePLL()   { RCC->CR  &= ~RCC_CR_PLLON; }
#ifdef RCC_CR2_HSI48ON
    void DisableHSI48() { RCC->CR2 &= ~RCC_CR2_HSI48ON; }
#endif
    void DisableCRS();
    // Checks
#ifdef RCC_CR2_HSI48ON
    bool IsHSI48On() { return (RCC->CR2 & RCC_CR2_HSI48ON); }
#endif
    uint32_t GetAhbApbDividers() { return RCC->CFGR & (RCC_CFGR_HPRE | RCC_CFGR_PPRE); }
    // Setups
#ifdef RCC_CFGR3_USBSW
    void SelectUSBClock_HSI48() { RCC->CFGR3 &= ~RCC_CFGR3_USBSW; }
#endif
    void SetupBusDividers(AHBDiv_t AHBDiv, APBDiv_t APBDiv);
    void SetupBusDividers(uint32_t Dividers);
    uint8_t SetupPLLDividers(uint8_t HsePreDiv, PllMul_t PllMul);
    void UpdateFreqValues();
    void SetupFlashLatency(uint32_t FrequencyHz);
    void EnablePrefetch()  { FLASH->ACR |=  FLASH_ACR_PRFTBE; }
    void DisablePrefetch() { FLASH->ACR &= ~FLASH_ACR_PRFTBE; }

    void PrintFreqs();
};

#elif defined STM32F4xx_MCUCONF
#include "stm32f4xx.h"
#include "board.h"

#define HSI_FREQ_HZ         16000000    // Freq of internal generator, not adjustable
#define LSI_FREQ_HZ         32000       // Freq of low power internal generator, may vary depending on voltage, not adjustable
#define CLK_STARTUP_TIMEOUT 2007        // tics

enum ClkSrc_t {csHSI, csHSE, csPLL};
enum AHBDiv_t {
    ahbDiv1=0b0000,
    ahbDiv2=0b1000,
    ahbDiv4=0b1001,
    ahbDiv8=0b1010,
    ahbDiv16=0b1011,
    ahbDiv64=0b1100,
    ahbDiv128=0b1101,
    ahbDiv256=0b1110,
    ahbDiv512=0b1111
};
enum APBDiv_t {apbDiv1=0b000, apbDiv2=0b100, apbDiv4=0b101, apbDiv8=0b110, apbDiv16=0b111};
enum PllSysDiv_P_t {pllSysDiv2=0b00, pllSysDiv4=0b01, pllSysDiv6=0b10, pllSysDiv8=0b11};
enum Mco1Src_t {mco1HSI=0x00000000, mco1LSE=0x00200000, mco1HSE=0x00400000, mco1PLL=0x00600000};
enum Mco2Src_t {mco2Sys=0x00000000, mco2PLLI2S=0x40000000, mco2HSE=0x80000000, mco2PLL=0xC0000000};
enum McoDiv_t {mcoDiv1=0b000, mcoDiv2=0b100, mcoDiv3=0b101, mcoDiv4=0b110, mcoDiv5=0b111};

class Clk_t {
private:
    uint8_t HSEEnable();
    uint8_t HSIEnable();
    uint8_t PLLEnable();
public:
    // Frequency values
    uint32_t AHBFreqHz;     // HCLK: AHB Buses, Core, Memory, DMA; 120 MHz max
    uint32_t APB1FreqHz;    // PCLK1: APB1 Bus clock; 30 MHz max
    uint32_t APB2FreqHz;    // PCLK2: APB2 Bus clock; 60 MHz max
    uint32_t UsbSdioFreqHz; // Clock is intended to be 48 MHz
    uint8_t TimerAPB1ClkMulti = 1;
    uint8_t TimerAPB2ClkMulti = 1;
    // Clk switching
    uint8_t SwitchToHSI();
    uint8_t SwitchToHSE();
    uint8_t SwitchToPLL();
    void HSEDisable() { RCC->CR &= ~RCC_CR_HSEON; }
    void HSIDisable() { RCC->CR &= ~RCC_CR_HSION; }
    void PLLDisable() { RCC->CR &= ~RCC_CR_PLLON; }
    void LsiEnable();
    // Dividers
    void SetupBusDividers(AHBDiv_t AHBDiv, APBDiv_t APB1Div, APBDiv_t APB2Div);
    uint8_t SetupPLLDividers(uint8_t InputDiv_M, uint16_t Multi_N, PllSysDiv_P_t SysDiv_P, uint8_t UsbDiv_Q);
    void UpdateFreqValues();
    uint8_t SetupFlashLatency(uint8_t AHBClk_MHz, uint16_t Voltage_mV=3300);
    // Disabling the prefetch buffer avoids extra Flash access that consumes 20 mA for 128-bit line fetching.
    void EnablePrefetch()  { FLASH->ACR |=  FLASH_ACR_PRFTEN; }
    void DisablePrefetch() { FLASH->ACR &= ~FLASH_ACR_PRFTEN; }
    // Special frequencies
    void SetFreq12Mhz() {
        if(AHBFreqHz < 12000000) SetupFlashLatency(12); // Rise flash latency now if current freq > required
        SetupBusDividers(ahbDiv4, apbDiv1, apbDiv1);
        UpdateFreqValues();
        SetupFlashLatency(AHBFreqHz/1000000);
    }
    void SetFreq48Mhz() {
        if(AHBFreqHz < 48000000) SetupFlashLatency(48);     // Rise flash latency now if current freq > required
//        SetupBusDividers(ahbDiv1, apbDiv2, apbDiv1);    // APB1: 30MHz max; APB2: 60MHz max
        SetupBusDividers(ahbDiv1, apbDiv4, apbDiv4);    // Peripheral freqs stay the same
        UpdateFreqValues();
        SetupFlashLatency(AHBFreqHz/1000000);
    }

    void PrintFreqs();

    // I2S
    void SetupI2SClk(uint32_t APLL_I2S_N, uint32_t APLL_I2S_R) {
        RCC->CFGR &= ~RCC_CFGR_I2SSRC;              // PLLI2S clock used as I2S clock source
        RCC->PLLI2SCFGR = (APLL_I2S_N << 6) | (APLL_I2S_R << 28); // Configure PLLI2S
        RCC->CR |= RCC_CR_PLLI2SON;                 // Enable PLLI2S
        while((RCC->CR & RCC_CR_PLLI2SRDY) == 0);   // Wait till PLLI2S is ready
    }

    // Clock output
    void MCO1Enable(Mco1Src_t Src, McoDiv_t Div);
    void MCO1Disable();
    void MCO2Enable(Mco2Src_t Src, McoDiv_t Div);
    void MCO2Disable();
};

/*
 * Early initialization code.
 * This initialization must be performed just after stack setup and before
 * any other initialization.
 */
extern "C" {
void __early_init(void);
}

#elif defined STM32L4XX
// Values of the Internal oscillator in Hz
#define MSI_FREQ_HZ     4000000UL
#define HSI_FREQ_HZ     16000000UL
// Top frequency in all domains is 80 MHz

enum AHBDiv_t {
    ahbDiv1=0b0000,
    ahbDiv2=0b1000,
    ahbDiv4=0b1001,
    ahbDiv8=0b1010,
    ahbDiv16=0b1011,
    ahbDiv64=0b1100,
    ahbDiv128=0b1101,
    ahbDiv256=0b1110,
    ahbDiv512=0b1111
};
enum APBDiv_t {apbDiv1=0b000, apbDiv2=0b100, apbDiv4=0b101, apbDiv8=0b110, apbDiv16=0b111};

enum MCUVoltRange_t {mvrHiPerf, mvrLoPerf};

enum Src48MHz_t { src48None = 0b00, src48PllSai1Q = 0b01, src48PllQ = 0b10, src48Msi = 0b11 };

class Clk_t {
private:
    uint8_t EnableHSE();
    uint8_t EnablePLL();
    bool HiPerfModeEnabled = false;
public:
    // Frequency values
    uint32_t AHBFreqHz;     // HCLK: AHB Buses, Core, Memory, DMA
    uint32_t APB1FreqHz;    // PCLK1: APB1 Bus clock
    uint32_t APB2FreqHz;    // PCLK2: APB2 Bus clock
    // SysClk switching
    uint8_t SwitchToHSI();
    uint8_t SwitchToHSE();
    uint8_t SwitchToPLL();
    uint8_t SwitchToMSI();
    void DisableHSE() { RCC->CR &= ~RCC_CR_HSEON; }
    uint8_t EnableHSI();
    void DisableHSI() { RCC->CR &= ~RCC_CR_HSION; }
    void DisablePLL() { RCC->CR &= ~RCC_CR_PLLON; }
    void DisableMSI() { RCC->CR &= ~RCC_CR_MSION; }

    void SetupBusDividers(AHBDiv_t AHBDiv, APBDiv_t APB1Div, APBDiv_t APB2Div);
    uint8_t SetupPLLMulDiv(uint32_t M, uint32_t N, uint32_t R, uint32_t Q, uint32_t P = 8);
    void UpdateFreqValues();
    void EnablePrefeth() { FLASH->ACR |= FLASH_ACR_PRFTEN; }
    void SetupFlashLatency(uint8_t AHBClk_MHz, MCUVoltRange_t VoltRange);
    void SetVoltageRange(MCUVoltRange_t VoltRange);
    void Select48MhzSrc(Src48MHz_t Src);
    // LSI
    void EnableLSI() {
        RCC->CSR |= RCC_CSR_LSION;
        while(!(RCC->CSR & RCC_CSR_LSIRDY));
    }
    void DisableLSI() { RCC->CSR &= RCC_CSR_LSION; }

    void SetHiPerfMode();
    void SetLoPerfMode();

    // Setup independent clock
//    void SetUart

    void PrintFreqs();
};
#endif

extern Clk_t Clk;

#endif // Clocking
