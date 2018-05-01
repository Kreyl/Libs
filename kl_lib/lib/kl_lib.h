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
#include "EvtMsgIDs.h"

/*
Build time:
Define symbol BUILD_TIME in main.cpp options with value "\"${current_date}\"".
Maybe, to calm Eclipse, it will be required to write extra quote in the end: "\"${current_date}\"""
*/

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

#ifndef TRUE
#define TRUE    1
#endif
#ifndef FALSE
#define FALSE   0
#endif

// Short type names
typedef uint8_t u8;
typedef uint8_t Byte;
typedef int8_t  i8;
typedef uint16_t u16;
typedef int16_t s16;
typedef uint32_t u32;
typedef int32_t s32;
typedef uint64_t u64;
typedef int64_t s64;

// Return values
#define retvOk              0
#define retvFail            1
#define retvTimeout         2
#define retvBusy            3
#define retvInProgress      4
#define retvCmdError        5
#define retvCmdUnknown      6
#define retvBadValue        7
#define retvNew             8
#define retvLast            9
#define retvEmpty           10
#define retvOverflow        11
#define retvNotANumber      12
#define retvWriteProtect    13
#define retvEndOfFile       14
#define retvNotFound        15

// Binary semaphores
#define NOT_TAKEN       false
#define TAKEN           true

enum BitOrder_t {boMSB, boLSB};
enum LowHigh_t  {Low, High};
enum RiseFall_t {rfRising, rfFalling, rfNone};
enum Inverted_t {invNotInverted, invInverted};
enum PinOutMode_t {omPushPull = 0, omOpenDrain = 1};
enum BitNumber_t {bitn8, bitn16, bitn32};
enum EnableDisable_t {Enable, Disable};

typedef void (*ftVoidVoid)(void);
typedef void (*ftVoidPVoid)(void*p);
typedef void (*ftVoidPVoidLen)(void*p, uint32_t Len);

// Virtual class for IRQ handlers and timer callbacks
class IrqHandler_t {
public:
    virtual void IIrqHandler() = 0;
};

// ==== Math ====
#define MIN_(a, b)   ( ((a)<(b))? (a) : (b) )
#define MAX_(a, b)   ( ((a)>(b))? (a) : (b) )
#define ABS(a)      ( ((a) < 0)? -(a) : (a) )
#define TRIM_VALUE(v, Max)  { if((v) > (Max)) (v) = (Max); }
#define IS_LIKE(v, precise, deviation)  (((precise - deviation) < v) and (v < (precise + deviation)))
#define BitIsSet(r, b)  ((r) & (b))

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

template <typename T>
static T Average(T *p, uint32_t Len) {
    T Rslt = 0;
    for(uint32_t i=0; i<Len; i++) Rslt += *p++;
    Rslt /= Len;
    return Rslt;
}

template <typename T>
static inline T Proportion(T MinX, T MaxX, T MinY, T MaxY, T x) {
    return (((x - MaxX) * (MaxY - MinY)) / (MaxX - MinX)) + MaxY;
}

template <typename T>
static T FindMediana(T *Arr, int32_t N) {
    int32_t L = 1, r = N, i, j, k = N / 2;
    T x;
    while(L < r) {
        x = Arr[k];
        i = L;
        j = r;
        do {
            while(Arr[i] < x) i++;
            while(x < Arr[j]) j--;
            if(i <= j) {
                T tmp = Arr[i];
                Arr[i] = Arr[j];
                Arr[j] = tmp;
                i++;
                j--;
            }
            if(j < k) L = i;
            if(k < i) r = j;
        } while(i <= j);
    }
    return Arr[k];
}

// Amount of memory occupied by thread
uint32_t GetThdFreeStack(void *wsp, uint32_t size);
void PrintThdFreeStack(void *wsp, uint32_t size);

/*
 * Early initialization code.
 * This initialization must be performed just after stack setup and before
 * any other initialization.
 */
extern "C" {
void __early_init(void);
}

namespace Convert { // ============== Conversion operations ====================
union DWordBytes_t {
    uint32_t DWord;
    uint8_t b[4];
    DWordBytes_t& operator = (const DWordBytes_t &Right) {
        DWord = Right.DWord;
        return *this;
    }
};
union WordBytes_t {
    uint16_t Word;
    uint8_t b[2];
    WordBytes_t& operator = (const WordBytes_t &Right) {
        Word = Right.Word;
        return *this;
    }
} __attribute__((packed));

void U16ToArrAsBE(uint8_t *PArr, uint16_t N);
void U32ToArrAsBE(uint8_t *PArr, uint32_t N);
uint16_t ArrToU16AsBE(uint8_t *PArr);
uint32_t ArrToU32AsBE(uint8_t *PArr);
//void ReverseByteOrder16(uint16_t *p);
//void ReverseByteOrder16(int16_t *p);
#define ReverseByteOrder16(p)   (p) = __REV16(p)
#define ReverseByteOrder32(p)   (p) = __REV(p)
uint8_t TryStrToUInt32(char* S, uint32_t *POutput);
uint8_t TryStrToInt32(char* S, int32_t *POutput);
uint16_t BuildUint16(uint8_t Lo, uint8_t Hi);
uint32_t BuildUint32(uint8_t Lo, uint8_t MidLo, uint8_t MidHi, uint8_t Hi);
uint8_t TryStrToFloat(char* S, float *POutput);
}; // namespace
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
#elif defined STM32L4XX
#define UNIQ_ID_BASE    0x1FFF7590
static inline uint32_t GetUniqID1() {
    return *((uint32_t*)(UNIQ_ID_BASE + 0x00));
}
static inline uint32_t GetUniqID2() {
    return *((uint32_t*)(UNIQ_ID_BASE + 0x04));
}
static inline uint32_t GetUniqID3() {
    return *((uint32_t*)(UNIQ_ID_BASE + 0x08));
}
#endif
#endif

#if 1 // ======================= Virtual Timer =================================
/*
 * Example:
 * TmrKL_t TmrCheckBtn {MS2ST(54), evtIdBattery, tktPeriodic};
 * TmrCheckBtn.InitAndStart(chThdGetSelfX());
 */

void TmrKLCallback(void *p);    // Universal VirtualTimer callback

enum TmrKLType_t {tktOneShot, tktPeriodic};

class TmrKL_t : private IrqHandler_t {
private:
    virtual_timer_t Tmr;
    void StartI() { chVTSetI(&Tmr, Period, TmrKLCallback, this); }  // Will be reset before start
    systime_t Period;
    EvtMsgId_t EvtId;
    TmrKLType_t TmrType;
    void IIrqHandler();
public:
    void StartOrRestart() {
        chSysLock();
        StartI();
        chSysUnlock();
    }
    void StartOrRestart(systime_t NewPeriod) {
        chSysLock();
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

    void SetNewPeriod_ms(uint32_t NewPeriod) { Period = MS2ST(NewPeriod); }
    void SetNewPeriod_s(uint32_t NewPeriod) { Period = S2ST(NewPeriod); }

    TmrKL_t(systime_t APeriod, EvtMsgId_t AEvtId, TmrKLType_t AType) :
        Period(APeriod), EvtId(AEvtId), TmrType(AType) {}
    // Dummy period is set
    TmrKL_t(EvtMsgId_t AEvtId, TmrKLType_t AType) :
            Period(S2ST(9)), EvtId(AEvtId), TmrType(AType) {}
};
#endif

#if 1 // ========================== Random =====================================
namespace Random {
//uint32_t last = 1;
// Generate pseudo-random value
static inline long int Generate(long int LowInclusive, long int HighInclusive) {
    uint32_t last = random();
    return (last % (HighInclusive + 1 - LowInclusive)) + LowInclusive;
}
// Seed pseudo-random generator with new seed
static inline void Seed(unsigned int Seed) { srandom(Seed); }

// True random
#if defined STM32L4XX
void TrueInit();
void TrueDeinit();

// Generate truly random value
uint32_t TrueGenerate(uint32_t LowInclusive, uint32_t HighInclusive);
// Seed pseudo random with true random
void SeedWithTrue();
#endif
} // namespace
#endif

// ========================== Simple delay ===============================
static inline void DelayLoop(volatile uint32_t ACounter) { while(ACounter--); }

#if 1 // ======================= Power and backup unit =========================
// See Programming manual: http://www.st.com/content/ccc/resource/technical/document/programming_manual/6c/3a/cb/e7/e4/ea/44/9b/DM00046982.pdf/files/DM00046982.pdf/jcr:content/translations/en.DM00046982.pdf
// On writes, write 0x5FA to VECTKEY, otherwise the write is ignored. 4 is SYSRESETREQ: System reset request
#define REBOOT()                SCB->AIRCR = 0x05FA0004

#if defined STM32F2XX || defined STM32F4XX || defined STM32F10X_LD_VL
namespace BackupSpc {
    static inline void EnableAccess() {
        rccEnablePWRInterface(FALSE);
        rccEnableBKPSRAM(FALSE);
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
    void SetUpdateFrequencyChangingPrescaler(uint32_t FreqHz) const;
    void SetUpdateFrequencyChangingTopValue(uint32_t FreqHz) const;
    void SetTopValue(uint32_t Value) const { ITmr->ARR = Value; }
    uint32_t GetTopValue() const { return ITmr->ARR; }
    void EnableArrBuffering()  const { ITmr->CR1 |=  TIM_CR1_ARPE; }
    void DisableArrBuffering() const { ITmr->CR1 &= ~TIM_CR1_ARPE; }
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
    void SetEtrPolarity(Inverted_t AInverted) {
        if(AInverted == invInverted) ITmr->SMCR |= TIM_SMCR_ETP;
        else ITmr->SMCR &= ~TIM_SMCR_ETP;
    }
    void SelectMasterMode(TmrMasterMode_t MasterMode) const {
        uint16_t tmp = ITmr->CR2;
        tmp &= ~TIM_CR2_MMS;
        tmp |= (uint16_t)MasterMode;
        ITmr->CR2 = tmp;
    }
    void SelectSlaveMode(TmrSlaveMode_t SlaveMode) const {
        uint16_t tmp = ITmr->SMCR;
        tmp &= ~TIM_SMCR_SMS;
        tmp |= (uint16_t)SlaveMode;
        ITmr->SMCR = tmp;
    }
    // DMA, Irq, Evt
    void EnableDmaOnTrigger() const { ITmr->DIER |= TIM_DIER_TDE; }
    void EnableDMAOnCapture(uint8_t CaptureReq) const { ITmr->DIER |= (1 << (CaptureReq + 8)); }
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
    psLow  = 0b00,
    psMedium = 0b01,
    psFast = 0b10,
    psHigh = 0b11
};
#define PIN_SPEED_DEFAULT   psFast
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
#if defined STM32F4XX || defined STM32F042x6
__always_inline
static inline void PinSetHi(GPIO_TypeDef *PGpio, uint16_t APin) { PGpio->BSRRL = (1 << APin); }
__always_inline
static inline void PinSetLo(GPIO_TypeDef *PGpio, uint16_t APin) { PGpio->BSRRH = (1 << APin); }
#elif defined STM32F2XX || defined STM32L1XX
__always_inline
static inline void PinSetHi(GPIO_TypeDef *PGpio, uint32_t APin) { PGpio->BSRR = 1 << APin; }
__always_inline
static inline void PinSetLo(GPIO_TypeDef *PGpio, uint32_t APin) { PGpio->BSRR = 1 << (APin + 16);  }
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
static inline bool PinIsHi(GPIO_TypeDef *PGpio, uint32_t APin) { return PGpio->IDR & (1 << APin); }
__always_inline
static inline bool PinIsHi(const GPIO_TypeDef *PGpio, uint32_t APin) { return PGpio->IDR & (1 << APin); }
__always_inline
static inline bool PinIsLo(GPIO_TypeDef *PGpio, uint32_t APin) { return !(PGpio->IDR & (1 << APin)); }
__always_inline
static inline bool PinIsLo(const GPIO_TypeDef *PGpio, uint32_t APin) { return !(PGpio->IDR & (1 << APin)); }

// Setup
static void PinClockEnable(const GPIO_TypeDef *PGpioPort) {
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
    else if(PGpioPort == GPIOE) RCC->AHB2ENR |= RCC_AHB2ENR_GPIOEEN;
    else if(PGpioPort == GPIOF) RCC->AHB2ENR |= RCC_AHB2ENR_GPIOFEN;
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

#ifdef STM32L4XX
static inline void PinConnectAdc(GPIO_TypeDef *PGpioPort, const uint16_t APinNumber) {
    SET_BIT(PGpioPort->ASCR, 1<<APinNumber);
}
static inline void PinDisconnectAdc(GPIO_TypeDef *PGpioPort, const uint16_t APinNumber) {
    CLEAR_BIT(PGpioPort->ASCR, 1<<APinNumber);
}
#endif

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
#elif defined STM32F2XX
        case psLow:     PGpioPort->OSPEEDR = 0x00000000; break;
        case psMedium:  PGpioPort->OSPEEDR = 0x55555555; break;
        case psFast:    PGpioPort->OSPEEDR = 0xAAAAAAAA; break;
        case psHigh:    PGpioPort->OSPEEDR = 0xFFFFFFFF; break;
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
    void SetHi() const { PinSetHi(PGpio, Pin); }
    void SetLo() const { PinSetLo(PGpio, Pin); }
    void Set(uint8_t Value) {
        if(Value == 0) SetLo();
        else SetHi();
    }
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
    uint32_t Get() const { return *TMR_PCCR(ITmr, ISetup.TimerChnl); }
    void Init() const;
    void Deinit() const { Timer_t::Deinit(); PinSetupAnalog(ISetup.PGpio, ISetup.Pin); }
    void SetFrequencyHz(uint32_t FreqHz) const { Timer_t::SetUpdateFrequencyChangingPrescaler(FreqHz); }
    PinOutputPWM_t(const PwmSetup_t &ASetup) : Timer_t(ASetup.PTimer), ISetup(ASetup) {}
};
#endif

#if 1 // =========================== External IRQ ==============================
enum ExtiTrigType_t {ttRising, ttFalling, ttRisingFalling};

/* Make your class descendant of IrqHandler_t:
class cc1101_t : public IrqHandler_t {
    const PinIrq_t IGdo0;
    void IIrqHandler() { ... }
    cc1101_t(...) ... IGdo0(APGpio, AGdo0, pudNone, this), ...
}
    ...IGdo0.Init(ttFalling);
    ...IGdo0.EnableIrq(IRQ_PRIO_HIGH);
*/

// Pin to IRQ channel
#if defined STM32L1XX || defined STM32F4XX || defined STM32F2XX || defined STM32L4XX
#define PIN2IRQ_CHNL(Pin)   (((Pin) > 9)? EXTI15_10_IRQn : (((Pin) > 4)? EXTI9_5_IRQn : ((Pin) + EXTI0_IRQn)))
#elif defined STM32F030 || defined STM32F0
#define PIN2IRQ_CHNL(Pin)   (((Pin) > 3)? EXTI4_15_IRQn : (((Pin) > 1)? EXTI2_3_IRQn : EXTI0_1_IRQn))
#endif

// IRQ handlers
extern "C" {
#if INDIVIDUAL_EXTI_IRQ_REQUIRED
extern IrqHandler_t *ExtiIrqHandler[16];
#else
#if defined STM32L1XX || defined STM32F4XX || defined STM32F2XX || defined STM32L4XX
extern IrqHandler_t *ExtiIrqHandler[5], *ExtiIrqHandler_9_5, *ExtiIrqHandler_15_10;
#elif defined STM32F030 || defined STM32F0
extern IrqHandler_t *ExtiIrqHandler_0_1, *ExtiIrqHandler_2_3, *ExtiIrqHandler_4_15;
#endif
#endif // INDIVIDUAL_EXTI_IRQ_REQUIRED
}

class PinIrq_t {
public:
    GPIO_TypeDef *PGpio;
    uint16_t PinN;
    PinPullUpDown_t PullUpDown;
    PinIrq_t(GPIO_TypeDef *APGpio, uint16_t APinN, PinPullUpDown_t APullUpDown, IrqHandler_t *PIrqHandler) :
        PGpio(APGpio), PinN(APinN), PullUpDown(APullUpDown) {
#if INDIVIDUAL_EXTI_IRQ_REQUIRED
        PIrqHandler[APinN] = PIrqHandler;
#else
    #if defined STM32L1XX || defined STM32F4XX || defined STM32F2XX || defined STM32L4XX
        if(APinN >= 0 and APinN <= 4) ExtiIrqHandler[APinN] = PIrqHandler;
        else if(APinN <= 9) ExtiIrqHandler_9_5 = PIrqHandler;
        else ExtiIrqHandler_15_10 = PIrqHandler;
    #elif defined STM32F030 || defined STM32F0
        if(APinN == 0 or APinN == 1) ExtiIrqHandler_0_1 = PIrqHandler;
        else if(APinN == 2 or APinN == 3) ExtiIrqHandler_2_3 = PIrqHandler;
        else ExtiIrqHandler_4_15 = PIrqHandler;
    #endif
#endif // INDIVIDUAL_EXTI_IRQ_REQUIRED
    }

    bool IsHi() const { return PinIsHi(PGpio, PinN); }

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

    // ttRising, ttFalling, ttRisingFalling
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

#if 1 // ============================== IWDG ===================================
namespace Iwdg {

// Up to 32000 ms
void InitAndStart(uint32_t ms);

static inline void Reload() { IWDG->KR = 0xAAAA; }

static inline bool ResetOccured() {
    if(RCC->CSR & RCC_CSR_IWDGRSTF) {
        RCC->CSR |= RCC_CSR_RMVF;   // Clear flags
        return true;
    }
    else return false;
}

void GoSleep(uint32_t Timeout_ms);

};
#endif

#if 1 // ============================== Sleep ==================================
namespace Sleep {
#if defined STM32L4XX
static inline void EnterStandby() {
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
    uint32_t tmp = PWR->CR1 & ~PWR_CR1_LPMS;
    tmp |= PWR_CR1_LPMS_STANDBY;
    PWR->CR1 = tmp;
    __WFI();
}

// It is impossible to distinct power-on and wake from shutdown
static inline void EnterShutdown() {
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
    uint32_t tmp = PWR->CR1 & ~PWR_CR1_LPMS;
    tmp |= PWR_CR1_LPMS_SHUTDOWN;
    PWR->CR1 = tmp;
    __WFI();
}

static inline void EnableWakeup1Pin(RiseFall_t RiseFall)  {
    if(RiseFall == rfFalling) PWR->CR4 |= PWR_CR4_WP1;    // Detection on low level (falling edge)
    else PWR->CR4 &= ~PWR_CR4_WP1;  // Detection on high level (rising edge)
    PWR->CR3 |=  PWR_CR3_EWUP1;
}
static inline void DisableWakeup1Pin() { PWR->CR3 &= ~PWR_CR3_EWUP1; }
static inline void EnableWakeup2Pin(RiseFall_t RiseFall)  {
    if(RiseFall == rfFalling) PWR->CR4 |= PWR_CR4_WP2;
    else PWR->CR4 &= ~PWR_CR4_WP2;
    PWR->CR3 |=  PWR_CR3_EWUP2;
}
static inline void DisableWakeup2Pin() { PWR->CR3 &= ~PWR_CR3_EWUP2; }
static inline void EnableWakeup4Pin(RiseFall_t RiseFall)  {
    if(RiseFall == rfFalling) PWR->CR4 |= PWR_CR4_WP4;
    else PWR->CR4 &= ~PWR_CR4_WP4;
    PWR->CR3 |=  PWR_CR3_EWUP4;
}
static inline void DisableWakeup4Pin() { PWR->CR3 &= ~PWR_CR3_EWUP4; }

static inline bool WasInStandby()      { return (PWR->SR1 & PWR_SR1_SBF); }
static inline bool WkupOccured()       { return (PWR->SR1 & 0x1F); }
static inline void ClearStandbyFlag()  { PWR->SCR |= PWR_SCR_CSBF; }
static inline void ClearWUFFlags()     { PWR->SCR |= 0x1F; }
#else
static inline void EnterStandby() {
#if defined STM32F0XX || defined STM32L4XX || defined STM32F2XX
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
#else
    SCB->SCR |= SCB_SCR_SLEEPDEEP;
#endif
    PWR->CR = PWR_CR_PDDS;
    // Command to clear WUF (wakeup flag) and wait two sys clock cycles to allow it be cleared
    PWR->CR |= PWR_CR_CWUF;
    __NOP(); __NOP();
    __WFI();
}

#if defined STM32F2XX
static inline void EnableWakeupPin()  { PWR->CSR |=  PWR_CSR_EWUP; }
static inline void DisableWakeupPin() { PWR->CSR &= ~PWR_CSR_EWUP; }
#else
static inline void EnableWakeup1Pin()  { PWR->CSR |=  PWR_CSR_EWUP1; }
static inline void DisableWakeup1Pin() { PWR->CSR &= ~PWR_CSR_EWUP1; }
static inline void EnableWakeup2Pin()  { PWR->CSR |=  PWR_CSR_EWUP2; }
static inline void DisableWakeup2Pin() { PWR->CSR &= ~PWR_CSR_EWUP2; }
#endif
static inline bool WasInStandby() { return (PWR->CSR & PWR_CSR_SBF); }
static inline void ClearStandbyFlag() { PWR->CR |= PWR_CR_CSBF; }
#endif

}; // namespace
#endif

#if 1 // ============================== SPI ====================================
enum CPHA_t {cphaFirstEdge, cphaSecondEdge};
enum CPOL_t {cpolIdleLow, cpolIdleHigh};
enum SpiClkDivider_t {
    sclkDiv2   = 0b000,
    sclkDiv4   = 0b001,
    sclkDiv8   = 0b010,
    sclkDiv16  = 0b011,
    sclkDiv32  = 0b100,
    sclkDiv64  = 0b101,
    sclkDiv128 = 0b110,
    sclkDiv256 = 0b111,
};

class Spi_t {
public:
    SPI_TypeDef *PSpi;
    Spi_t(SPI_TypeDef *ASpi) : PSpi(ASpi) {}
    // Example: boMSB, cpolIdleLow, cphaFirstEdge, sbFdiv2, bitn8
    void Setup(BitOrder_t BitOrder, CPOL_t CPOL, CPHA_t CPHA,
            SpiClkDivider_t Divider, BitNumber_t BitNumber = bitn8) const {
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
        PSpi->CR1 |= ((uint16_t)Divider) << 3;                  // Clock divider
#if defined STM32L1XX || defined STM32F10X_LD_VL || defined STM32F2XX || defined STM32F4XX
        if(BitNumber == bitn16) PSpi->CR1 |= SPI_CR1_DFF;
        PSpi->CR2 = 0;
#elif defined STM32F030 || defined STM32F072xB || defined STM32L4XX
        if(BitNumber == bitn16) PSpi->CR2 = (uint16_t)0b1111 << 8;  // 16 bit, RXNE generated when 16 bit is received
        else PSpi->CR2 = ((uint16_t)0b0111 << 8) | SPI_CR2_FRXTH;   // 8 bit, RXNE generated when 8 bit is received
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
    void WaitTxEHi()     const { while(!(PSpi->SR & SPI_SR_TXE)); }
    void ClearRxBuf()    const { while(PSpi->SR & SPI_SR_RXNE) (void)PSpi->DR; }
    void ClearOVR()      const { (void)PSpi->DR; (void)PSpi->SR; (void)PSpi->DR; }
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

// =========================== Flash and Option bytes ==========================
namespace Flash {

void UnlockFlash();
void LockFlash();

uint8_t ErasePage(uint32_t PageAddress);
#if defined STM32L4XX
uint8_t ProgramDWord(uint32_t Address, uint64_t Data);
#else
uint8_t ProgramWord(uint32_t Address, uint32_t Data);
uint8_t ProgramBuf(void *PData, uint32_t ByteSz, uint32_t Addr);
#endif

bool FirmwareIsLocked();
void LockFirmware();
void UnlockFirmware();

bool IwdgIsFrozenInStandby();
void IwdgFrozeInStandby();

#if defined STM32L4XX
bool DualbankIsEnabled();
void DisableDualbank();
#endif

}; // Namespace

#if defined STM32L1XX // =================== Internal EEPROM ===================
namespace EE {
    uint32_t Read32(uint32_t Addr);
    uint8_t Write32(uint32_t Addr, uint32_t W);
    void ReadBuf(void *PDst, uint32_t Sz, uint32_t Addr);
    uint8_t WriteBuf(void *PSrc, uint32_t Sz, uint32_t Addr);
};
#endif

#if 1 // =========================== Clocking ==================================
// Common
enum CoreClk_t {cclk8MHz, cclk16MHz, cclk24MHz, cclk48MHz, cclk72MHz};

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

enum PllSrc_t {pllSrcHSI16, pllSrcHSE};
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
    uint8_t SetupPLLDividers(PllMul_t PllMul, PllDiv_t PllDiv);
    void SetupPLLSrc(PllSrc_t Src) {
        if(Src == pllSrcHSI16) RCC->CFGR &= ~RCC_CFGR_PLLSRC;
        else RCC->CFGR |= RCC_CFGR_PLLSRC;
    }
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

    void EnablePrefetch() {
        FLASH->ACR |= FLASH_ACR_ACC64;  // Enable 64-bit access
        FLASH->ACR |= FLASH_ACR_PRFTEN; // May be written only when ACC64 is already set
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
enum PllSrc_t {pllSrcHSIdiv2, pllSrcHSE};
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
enum i2cClk_t { i2cclkHSI = 0, i2cclkSYSCLK = 1 };

class Clk_t {
private:
    uint8_t EnableHSE();
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
    uint8_t EnableHSI();
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
    void SetupPLLSrc(PllSrc_t Src);
    uint32_t GetSysClkHz();
    void UpdateFreqValues();
    void SetupFlashLatency(uint32_t FrequencyHz);
    void EnablePrefetch()  { FLASH->ACR |=  FLASH_ACR_PRFTBE; }
    void DisablePrefetch() { FLASH->ACR &= ~FLASH_ACR_PRFTBE; }

    void SetI2CClkSrc(I2C_TypeDef *i2c, i2cClk_t ClkSrc) {
        uint32_t tmp = RCC->CFGR3;
        if(i2c == I2C1) {   // i2c1 only is configured
            tmp &= ~RCC_CFGR3_I2C1SW;
            tmp |= ((uint32_t)ClkSrc) << 12;
            RCC->CFGR3 = tmp;
        }
    }

    void PrintFreqs();
};

#elif defined STM32F4xx_MCUCONF || defined STM32F2XX
#define HSI_FREQ_HZ         16000000    // Freq of internal generator, not adjustable
#define LSI_FREQ_HZ         32000       // Freq of low power internal generator, may vary depending on voltage, not adjustable
//#define CLK_STARTUP_TIMEOUT 2007        // tics

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
    uint8_t EnableHSE();
    uint8_t EnableHSI();
    uint8_t EnablePLL();
public:
    // Frequency values
    uint32_t AHBFreqHz;     // HCLK: AHB Buses, Core, Memory, DMA; 120 MHz max
    uint32_t APB1FreqHz;    // PCLK1: APB1 Bus clock; 30 MHz max
    uint32_t APB2FreqHz;    // PCLK2: APB2 Bus clock; 60 MHz max
    // Clk switching
    uint8_t SwitchToHSI();
    uint8_t SwitchToHSE();
    uint8_t SwitchToPLL();
    void DisableHSE() { RCC->CR &= ~RCC_CR_HSEON; }
    void DisableHSI() { RCC->CR &= ~RCC_CR_HSION; }
    void DisablePLL() { RCC->CR &= ~RCC_CR_PLLON; }
    void EnableLsi();
    bool IsHSEEnabled() { return (RCC->CR & RCC_CR_HSERDY); }
    // Dividers
    void SetupBusDividers(AHBDiv_t AHBDiv, APBDiv_t APB1Div, APBDiv_t APB2Div);
    uint8_t SetupPllMulDiv(uint8_t InputDiv_M, uint16_t Multi_N, PllSysDiv_P_t SysDiv_P, uint8_t UsbDiv_Q);
    void UpdateFreqValues();
    uint8_t SetupFlashLatency(uint8_t AHBClk_MHz, uint16_t Voltage_mV=3300);
    uint32_t GetTimInputFreq(TIM_TypeDef* ITmr);
    // Disabling the prefetch buffer avoids extra Flash access that consumes 20 mA for 128-bit line fetching.
    void EnablePrefetch()  { FLASH->ACR |=  FLASH_ACR_PRFTEN; }
    void DisablePrefetch() { FLASH->ACR &= ~FLASH_ACR_PRFTEN; }

    void PrintFreqs();

    // I2S
    void SetupI2SClk(uint32_t APLL_I2S_N, uint32_t APLL_I2S_R) {
        RCC->CFGR &= ~RCC_CFGR_I2SSRC;              // PLLI2S clock used as I2S clock source
        RCC->PLLI2SCFGR = (APLL_I2S_N << 6) | (APLL_I2S_R << 28); // Configure PLLI2S
        RCC->CR |= RCC_CR_PLLI2SON;                 // Enable PLLI2S
        while((RCC->CR & RCC_CR_PLLI2SRDY) == 0);   // Wait till PLLI2S is ready
    }

    // Clock output
    void EnableMCO1(Mco1Src_t Src, McoDiv_t Div);
    void DisableMCO1();
    void EnableMCO2(Mco2Src_t Src, McoDiv_t Div);
    void DisableMCO2();

    void SetCoreClk(CoreClk_t CoreClk);
};

#elif defined STM32L4XX
// Values of the Internal oscillator in Hz
#define MSI_FREQ_HZ     4000000UL
#define HSI_FREQ_HZ     16000000UL
#define LSI_FREQ_HZ     32000UL
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
enum PllSrc_t { pllsrcNone = 0b00, pllsrcMsi = 0b01, pllsrcHsi16 = 0b10, pllsrcHse = 0b11 };

enum McoSrc_t {mcoNone=0b0000, mcoSYSCLK=0b0001, mcoMSI=0b0010, mcoHSI16=0b0011, mcoHSE=0b0100, mcoMainPLL=0b0101, mcoLSI=0b0110, mcoLSE=0b0111 };
enum McoDiv_t {mcoDiv1=0b000, mcoDiv2=0b001, mcoDiv4=0b010, mcoDiv8=0b011, mcoDiv16 = 0b100};

enum i2cClk_t { i2cclkPCLK1 = 0, i2cclkSYSCLK = 1, i2cclkHSI = 2 };
enum uartClk_t {uartclkPCLK = 0, uartclkSYSCLK = 1, uartclkHSI = 2, uartclkLSE = 3 };

class Clk_t {
private:
    uint8_t EnableHSE();
    uint8_t EnablePLL();
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
    // PLL and PLLSAI
    void SetupPllSrc(PllSrc_t Src) { MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLLSRC, ((uint32_t)Src)); }
    uint8_t SetupPllMulDiv(uint32_t M, uint32_t N, uint32_t R, uint32_t Q);
    uint8_t SetupPllSai1(uint32_t N, uint32_t R, uint32_t P);
    void EnableSai1ROut() { SET_BIT(RCC->PLLSAI1CFGR, RCC_PLLSAI1CFGR_PLLSAI1REN); }
    void EnableSai1POut() { SET_BIT(RCC->PLLSAI1CFGR, RCC_PLLSAI1CFGR_PLLSAI1PEN); }

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

    void SetCoreClk(CoreClk_t CoreClk);

    uint32_t GetSysClkHz();

    // Setup independent clock
    void SetI2CClkSrc(I2C_TypeDef *i2c, i2cClk_t ClkSrc) {
        uint32_t tmp = RCC->CCIPR;
        if(i2c == I2C1) {
            tmp &= ~RCC_CCIPR_I2C1SEL;
            tmp |= ((uint32_t)ClkSrc) << 12;
        }
        else if(i2c == I2C2) {
            tmp &= ~RCC_CCIPR_I2C2SEL;
            tmp |= ((uint32_t)ClkSrc) << 14;
        }
        else if(i2c == I2C3) {
            tmp &= ~RCC_CCIPR_I2C3SEL;
            tmp |= ((uint32_t)ClkSrc) << 16;
        }
        RCC->CCIPR = tmp;
    }

    void SetUartClkSrc(USART_TypeDef *uart, uartClk_t ClkSrc) {
        uint32_t tmp = RCC->CCIPR;
        if(uart == USART1) {
            tmp &= ~RCC_CCIPR_USART1SEL;
            tmp |= ((uint32_t)ClkSrc) << 0;
        }
        else if(uart == USART2) {
            tmp &= ~RCC_CCIPR_USART2SEL;
            tmp |= ((uint32_t)ClkSrc) << 2;
        }
        else if(uart == USART3) {
            tmp &= ~RCC_CCIPR_USART3SEL;
            tmp |= ((uint32_t)ClkSrc) << 4;
        }
        else if(uart == UART4) {
            tmp &= ~RCC_CCIPR_UART4SEL;
            tmp |= ((uint32_t)ClkSrc) << 6;
        }
        else if(uart == UART5) {
            tmp &= ~RCC_CCIPR_UART5SEL;
            tmp |= ((uint32_t)ClkSrc) << 8;
        }
        else if(uart == LPUART1) {
            tmp &= ~RCC_CCIPR_LPUART1SEL;
            tmp |= ((uint32_t)ClkSrc) << 10;
        }
        RCC->CCIPR = tmp;
    }
    uint32_t GetUartClkHz(USART_TypeDef *uart) {
        // Get clock src
        uartClk_t ClkSrc = uartclkPCLK;
        if     (uart == USART1) ClkSrc = (uartClk_t)((RCC->CCIPR & RCC_CCIPR_USART1SEL) >> 0);
        else if(uart == USART2) ClkSrc = (uartClk_t)((RCC->CCIPR & RCC_CCIPR_USART2SEL) >> 2);
        else if(uart == USART3) ClkSrc = (uartClk_t)((RCC->CCIPR & RCC_CCIPR_USART3SEL) >> 4);
        else if(uart == UART4)  ClkSrc = (uartClk_t)((RCC->CCIPR & RCC_CCIPR_UART4SEL)  >> 6);
        else if(uart == UART5)  ClkSrc = (uartClk_t)((RCC->CCIPR & RCC_CCIPR_UART5SEL)  >> 8);
        else if(uart == LPUART1) ClkSrc = (uartClk_t)((RCC->CCIPR & RCC_CCIPR_LPUART1SEL) >> 10);
        // Get clock
        switch(ClkSrc) {
            case uartclkPCLK:
                if(uart == USART1) return APB2FreqHz;
                else return APB1FreqHz;
                break;
            case uartclkSYSCLK:
                return (AHBFreqHz << AHBPrescTable[((RCC->CFGR & RCC_CFGR_HPRE) >> 4)]);
                break;
            case uartclkHSI:
                return 16000000;
                break;
            default: // uartclkLSE:
                return 32768;
                break;
        }
    }

    uint32_t GetSaiClkHz();

    // Clock output
    void EnableMCO(McoSrc_t Src, McoDiv_t Div) {
        PinSetupAlterFunc(GPIOA, 8, omPushPull, pudNone, AF0, psHigh);
        RCC->CFGR &= ~(RCC_CFGR_MCOSEL | RCC_CFGR_MCOPRE);   // First, disable output and clear settings
        RCC->CFGR |= (((uint32_t)Src) << 24) | ((uint32_t)Div << 28);
    }
    void DisableMCO() {
        PinSetupAnalog(GPIOA, 8);
        RCC->CFGR &= ~(RCC_CFGR_MCOSEL | RCC_CFGR_MCOPRE);
    }

    void PrintFreqs();
};
#endif

extern Clk_t Clk;

#endif // Clocking
