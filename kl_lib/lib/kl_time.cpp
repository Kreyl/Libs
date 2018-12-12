/*
 * kl_time.cpp
 *
 *  Created on: 13.06.2012
 *      Author: kreyl
 */

#include "kl_time.h"
#include "interface.h"
#include "MsgQ.h"

TimeCounter_t Time;

#define DATE2REG(Ytens, Yunits, WeekDay, Mtens, Munits, Dtens, Dunits)  \
    ((Ytens<<20)|(Yunits<<16)|(WeekDay<<13)|(Mtens<<12)|(Munits<<8)|(Dtens<<4)|Dunits)

#define RTC_TR_RESERVED_MASK    0x007F7F7FU
#define RTC_DR_RESERVED_MASK    0x00FFFF3FU

#if defined STM32F10X_LD_VL
#define BCKPREG_CHECK   BKP->DR1
#elif defined STM32F072xB || defined STM32F2XX
#define BCKPREG_CHECK   RTC->BKP0R
#elif defined STM32L4XX
#define BCKPREG_CHECK   RTC->BKP0R
#endif

static inline bool IsSetup()  { return (BCKPREG_CHECK == 0xA5A5); }
static inline void SetSetup() { BCKPREG_CHECK = 0xA5A5; }

void TimeCounter_t::Init() {
    BackupSpc::EnableAccess();
    if(!IsSetup()) {
        Printf("Time: Not set\r");
        // ==== Rtc config ====
        BackupSpc::Reset();     // Reset Backup Domain
        Clk.EnableLSE();
        // Let it start for a second
        systime_t StartTime = chVTGetSystemTimeX();
        while(true) {
            if(Clk.IsLseOn()) {
#if !defined STM32F2XX && !defined STM32L4XX
                Clk.SetLSELevel(lselvlLow); // Set Low power of crystal
#endif
                Rtc::SetClkSrcLSE();    // Select LSE as RTC Clock Source
                Rtc::EnableClk();       // Enable RTC Clock
                // Init RTC
                chSysLock();
                Rtc::DisableWriteProtection();
                chSysUnlock();
                Rtc::EnterInitMode();
                // Program both the prescaler factors
                RTC->PRER = (0x7FUL << 16) | (0xFFUL);  // async pre = 128, sync = 256 => 32768->1
                // Clear RTC_CR FMT (24h format), OSEL (no output), disable WUTE
                RTC->CR &= ~(RTC_CR_FMT | RTC_CR_OSEL | RTC_CR_WUTE | RTC_CR_WUCKSEL);
#if !defined STM32F2XX
                RTC->CR |= RTC_CR_BYPSHAD;  // Bypass shadow regs
#endif
                // ==== Setup wake-up timer ====
                // Wait WakeUp timer to allow changes
//                while(!BitIsSet(RTC->ISR, RTC_ISR_WUTWF));
//                RTC->WUTR = 2047; // Flag is set every WUTR+1 cycles => every second
//                RTC->CR &= ~(0b111UL); // RTC/16 clock is selected
//                RTC->CR |= RTC_CR_WUTE | RTC_CR_WUTIE;  // Enable Wake-up timer and its irq

                // Setup default time & date
                RTC->TR = 0;    // Time = 0
                RTC->DR = DATE2REG(1, 7, 7, 0, 9, 1, 8); // 17 09 18
                Rtc::ExitInitMode();
#if !defined STM32F2XX
                if(!(RTC->CR & RTC_CR_BYPSHAD))
#endif
                    Rtc::WaitSync(); // Sync not required if shadow regs bypassed
                Rtc::EnableWriteProtection();
                SetSetup();
                Printf("32768 Started\r");
                break;
            }
            else if(chVTTimeElapsedSinceX(StartTime) > MS2ST(999)) {
                // Timeout
                Printf("32768 Failure\r");
                break;
            }
        } // while
    }
    else Printf("Time is set\r");
    // Enable every-second-IRQ
    Rtc::ClearWakeupFlag(); // Otherwise already set flag will not trigger interrupt
#if defined STM32F2XX
    // Allow IRQ on EXTI line 22 (connected to RTC Wakeup event) and select rising edge
    EXTI->IMR |= EXTI_IMR_MR22;
    EXTI->RTSR |= EXTI_RTSR_TR22;
    nvicEnableVector(RTC_WKUP_IRQn, IRQ_PRIO_LOW);
#elif defined STM32L4XX

#else
    // Allow IRQ on EXTI line 20 (connected to Wakeup timer) and select rising edge
    EXTI->IMR |= EXTI_IMR_MR20;
    EXTI->RTSR |= EXTI_RTSR_TR20;
    nvicEnableVector(RTC_IRQn, IRQ_PRIO_LOW);
#endif
}

void TimeCounter_t::BeFast() {
    chSysLock();
    Rtc::DisableWriteProtection();
    Rtc::EnterInitMode();
    // Program both the prescaler factors
    RTC->PRER = (0x7UL << 16) | (0xFFUL);  // async pre = 8, sync = 256 => 32768->1
    Rtc::ExitInitMode();
    Rtc::EnableWriteProtection();
    chSysUnlock();
}

void TimeCounter_t::BeNormal() {
    chSysLock();
    Rtc::DisableWriteProtection();
    Rtc::EnterInitMode();
    // Program both the prescaler factors
    RTC->PRER = (0x7FUL << 16) | (0xFFUL);  // async pre = 128, sync = 256 => 32768->1
    Rtc::ExitInitMode();
    Rtc::EnableWriteProtection();
    chSysUnlock();
}


//void TimeCounter_t::DisableIrq() {
//    chSysLock();
//    Rtc::DisableWriteProtection();
//    chSysUnlock();
//    RTC->CR &= ~(RTC_CR_WUTE | RTC_CR_WUTIE);
//    Rtc::EnableWriteProtection();
//#if defined STM32F2XX
//    EXTI->PR |= EXTI_PR_PR22;   // Clear exti flag
//#else
//    EXTI->PR |= EXTI_PR_PR20;   // Clear exti flag
//#endif
//    Rtc::ClearWakeupFlag();
//}

//void TimeCounter_t::EnableIrq() {
//    Rtc::ClearWakeupFlag();
//#if defined STM32F2XX
//    EXTI->PR |= EXTI_PR_PR22;   // Clear exti flag
//#else
//    EXTI->PR |= EXTI_PR_PR20;   // Clear exti flag
//#endif
//    chSysLock();
//    Rtc::DisableWriteProtection();
//    chSysUnlock();
//    RTC->CR |= RTC_CR_WUTE | RTC_CR_WUTIE;
//    Rtc::EnableWriteProtection();
//}

void TimeCounter_t::GetDateTime() {
    Curr.Year  = ((RTC->DR >> 20) & 0b1111)* 10 + ((RTC->DR >> 16) & 0b1111) + 2000;
    Curr.Month = ((RTC->DR >> 12) & 0b1  ) * 10 + ((RTC->DR >> 8)  & 0b1111);
    Curr.Day   = ((RTC->DR >>  4) & 0b11 ) * 10 + ((RTC->DR >> 0)  & 0b1111);
    Curr.H     = ((RTC->TR >> 20) & 0b11 ) * 10 + ((RTC->TR >> 16) & 0b1111);
    Curr.M     = ((RTC->TR >> 12) & 0b111) * 10 + ((RTC->TR >> 8)  & 0b1111);
    Curr.S     = ((RTC->TR >>  4) & 0b111) * 10 + ((RTC->TR >> 0)  & 0b1111);
}

DayOfWeek_t TimeCounter_t::CurrDayOfWeek() {
    return (DayOfWeek_t)((RTC->DR >> 13) & 0b111UL);
}

void TimeCounter_t::SetDateTime() {
    uint32_t tmpD = 0x2000;  // Set week day to default monday (not used, but is a must)
    // Year
    uint32_t tens = (Curr.Year - 2000) / 10;
    uint32_t units = Curr.Year - 2000 - tens * 10;
    tmpD |= (tens << 20) | (units << 16);
    // Month
    tens = Curr.Month / 10;
    units = Curr.Month - tens * 10;
    tmpD |= (tens << 12) | (units << 8);
    // Day
    tens = Curr.Day / 10;
    units = Curr.Day - tens * 10;
    tmpD |= (tens << 4) | (units << 0);

    uint32_t tmpT = 0;
    // Hour
    tens = Curr.H / 10;
    units = Curr.H - tens * 10;
    tmpT |= (tens << 20) | (units << 16);
    // Minute
    tens = Curr.M / 10;
    units = Curr.M - tens * 10;
    tmpT |= (tens << 12) | (units << 8);
    // Seconds are always zero
    // Write the values
    chSysLock();
    Rtc::DisableWriteProtection();
    Rtc::EnterInitMode();
    RTC->TR = tmpT & RTC_TR_RESERVED_MASK;
    RTC->DR = tmpD;
    (void)RTC->DR; // Stinky magic to make data really be written in DR.
    Rtc::ExitInitMode();
#if !defined STM32F2XX
    if(!(RTC->CR & RTC_CR_BYPSHAD))
#endif
        Rtc::WaitSync();
    Rtc::EnableWriteProtection();
    SetSetup();
    chSysUnlock();
}

//extern "C" {
//#if defined STM32F2XX
//CH_IRQ_HANDLER(Vector4C) {
//    EXTI->PR |= EXTI_PR_PR22;   // Clear exti flag
//#else
//CH_IRQ_HANDLER(Vector48) {
//    EXTI->PR |= EXTI_PR_PR20;   // Clear exti flag
//#endif
//    CH_IRQ_PROLOGUE();
//    Rtc::ClearWakeupFlag();
////    PrintfI("RtcIrq\r");
//    chSysLockFromISR();
//    EvtQMain.SendNowOrExitI(EvtMsg_t(evtIdEverySecond));
//    chSysUnlockFromISR();
//    CH_IRQ_EPILOGUE();
//}
//} // extern c
