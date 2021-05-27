/*
 * kl_time.cpp
 *
 *  Created on: 13.06.2012
 *      Author: kreyl
 */

#include "kl_time.h"
#include "MsgQ.h"

TimeCounter_t Time;

#define DATE2REG(Ytens, Yunits, WeekDay, Mtens, Munits, Dtens, Dunits)  \
    ((Ytens<<20)|(Yunits<<16)|(WeekDay<<13)|(Mtens<<12)|(Munits<<8)|(Dtens<<4)|Dunits)

#define RTC_TR_RESERVED_MASK    0x007F7F7FU
#define RTC_DR_RESERVED_MASK    0x00FFFF3FU

#if defined STM32F10X_LD_VL
#define BKPREG_CHECK        BKP->DR1     // Register to store "IsStored" const
#elif defined STM32F072xB
#define BKPREG_CHECK        RTC->BKP0R
#endif

void TimeCounter_t::Init() {
    if(!IsSetup()) {
        Printf("Nothing is set\r");
        // ==== Rtc config ====
        BackupSpc::Reset();     // Reset Backup Domain
        Clk.SetLSELevel(lselvlLow); // Set Low power of crystal
        Clk.EnableLSE();
        // Let it start for a second
        systime_t StartTime = chVTGetSystemTimeX();
        while(true) {
            if(Clk.IsLseOn()) {
                Printf("32768 clk started\r");
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
                RTC->CR |= RTC_CR_BYPSHAD;  // Bypass shadow regs

                // ==== Setup wake-up timer ====
                // Wait WakeUp timer to allow changes
                while(!BitIsSet(RTC->ISR, RTC_ISR_WUTWF));
//                RTC->WUTR = 0;      // Flag is set every WUTR+1 cycles => every second with 1Hz input freq
//                RTC->CR |= 0b100UL; // ck_spre (usually 1 Hz) clock is selected
                RTC->WUTR = 2047; // Flag is set every WUTR+1 cycles => every second
                RTC->CR &= ~(0b111UL); // RTC/16 clock is selected
                RTC->CR |= RTC_CR_WUTE | RTC_CR_WUTIE;  // Enable Wake-up timer and its irq

                // Setup default time & date
                RTC->TR = 0;    // Time = 0
                RTC->DR = DATE2REG(1, 7, 7, 0, 9, 1, 8); // 17 09 18
                Rtc::ExitInitMode();
                if(!(RTC->CR & RTC_CR_BYPSHAD)) Rtc::WaitSync(); // Sync not required if shadow regs bypassed
                Rtc::EnableWriteProtection();
                SetSetup();
                break;
            }
            else if(chVTTimeElapsedSinceX(StartTime) > TIME_MS2I(999)) {
                // Timeout
                Printf("32768 Failure\r");
//                Interface.Error("32768 Fail");
                break;
            }
        } // while
    }
    else Printf("Time is setup\r");
    // Enable every-second-IRQ
#if defined STM32F072xB
    // Allow IRQ on EXTI line 20 (connected to Wakeup timer) and select rising edge
    EXTI->IMR |= EXTI_IMR_MR20;
    EXTI->RTSR |= EXTI_RTSR_TR20;
#elif defined STM32F7XX
    EXTI->IMR |= EXTI_IMR_MR22;
    EXTI->RTSR |= EXTI_RTSR_TR22;
#endif
    Rtc::ClearWakeupFlag(); // Otherwise already set flag will not trigger interrupt
#if defined STM32F072xB
    nvicEnableVector(RTC_IRQn, IRQ_PRIO_LOW);
#elif defined STM32F7XX
    nvicEnableVector(RTC_WKUP_IRQn, IRQ_PRIO_LOW);
#endif
}

bool TimeCounter_t::IsSetup() {
    return (BackupSpc::ReadRegister(BCKP_REG_SETUP_INDX) == 0xA5A5);
}
void TimeCounter_t::SetSetup() {
    BackupSpc::WriteRegister(BCKP_REG_SETUP_INDX, 0xA5A5);
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


void TimeCounter_t::DisableIrq() {
    chSysLock();
    Rtc::DisableWriteProtection();
    chSysUnlock();
    RTC->CR &= ~(RTC_CR_WUTE | RTC_CR_WUTIE);
    Rtc::EnableWriteProtection();
#if defined STM32F072xB
    EXTI->PR |= EXTI_PR_PR20;   // Clear exti flag
#elif defined STM32F7XX
    EXTI->PR |= EXTI_PR_PR22;
#endif
    Rtc::ClearWakeupFlag();
}

void TimeCounter_t::EnableIrq() {
    Rtc::ClearWakeupFlag();
#if defined STM32F072xB
    EXTI->PR |= EXTI_PR_PR20;   // Clear exti flag
#elif defined STM32F7XX
    EXTI->PR |= EXTI_PR_PR22;
#endif
    chSysLock();
    Rtc::DisableWriteProtection();
    chSysUnlock();
    RTC->CR |= RTC_CR_WUTE | RTC_CR_WUTIE;
    Rtc::EnableWriteProtection();
}

void TimeCounter_t::GetDateTime() {
    Curr.Year  = ((RTC->DR >> 20) & 0b1111)* 10 + ((RTC->DR >> 16) & 0b1111) + 2000;
    Curr.Month = ((RTC->DR >> 12) & 0b1  ) * 10 + ((RTC->DR >> 8)  & 0b1111);
    Curr.Day   = ((RTC->DR >>  4) & 0b11 ) * 10 + ((RTC->DR >> 0)  & 0b1111);
    Curr.H     = ((RTC->TR >> 20) & 0b11 ) * 10 + ((RTC->TR >> 16) & 0b1111);
    Curr.M     = ((RTC->TR >> 12) & 0b111) * 10 + ((RTC->TR >> 8)  & 0b1111);
    Curr.S     = ((RTC->TR >>  4) & 0b111) * 10 + ((RTC->TR >> 0)  & 0b1111);
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
    RTC->DR = tmpD;
    RTC->TR = tmpT & RTC_TR_RESERVED_MASK;
    Rtc::ExitInitMode();
    if(!(RTC->CR & RTC_CR_BYPSHAD)) Rtc::WaitSync();
    Rtc::EnableWriteProtection();
    SetSetup();
    chSysUnlock();
}

extern "C" {
#if defined STM32F072xB
CH_IRQ_HANDLER(Vector48) {
#elif defined STM32F7XX
CH_IRQ_HANDLER(Vector4C) {
#endif
    CH_IRQ_PROLOGUE();
    Rtc::ClearWakeupFlag();
    // Clear exti flag
#if defined STM32F072xB
    EXTI->PR |= EXTI_PR_PR20;
#elif defined STM32F7XX
    EXTI->PR |= EXTI_PR_PR22;
#endif
//    PrintfI("RtcIrq\r");
    chSysLockFromISR();
    EvtQMain.SendNowOrExitI(EvtMsg_t(evtIdEverySecond));
    chSysUnlockFromISR();
    CH_IRQ_EPILOGUE();
}
} // extern c
