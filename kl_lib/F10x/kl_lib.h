/*
 * kl_lib.h
 *
 *  Created on: 18.02.2012
 *      Author: kreyl
 */

#ifndef KL_GPIO_H_
#define KL_GPIO_H_

#include <inttypes.h>
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_dma.h"

// =============================== General =====================================
#define PACKED      __attribute__ ((__packed__))
#define NORETURN    __attribute__ ((__noreturn__))
#ifndef countof
#define countof(A)  (sizeof(A)/sizeof(A[0]))
#endif

// Functional types
typedef void(*ftVoid_Void)(void);

// ===================== Single pin manipulations ==============================
enum PinMode_t {
    pmOutPushPull    = 0b0011,  // }
    pmOutOpenDrain   = 0b0111,  // }
    pmOutAFPushPull  = 0b1011,  // }
    pmOutAFOpenDrain = 0b1111,  // } 50 MHz by default
    pmInAnalog       = 0b0000,
    pmInFloating     = 0b0100,
    pmInPullUp       = 0b1100,  // 11 is reserved; used here for distinction
    pmInPullDown     = 0b1000
};

enum PinSpeed_t {
    ps10MHz = 0b1101,
    ps2MHz  = 0b1110,
    ps50MHz = 0b1111
};

// Set/clear
static inline void klPinSet    (GPIO_TypeDef *PGpioPort, const uint16_t APinNumber) { PGpioPort->BSRR = (uint16_t)(1<<APinNumber); }
static inline void klPinClear  (GPIO_TypeDef *PGpioPort, const uint16_t APinNumber) { PGpioPort->BRR  = (uint16_t)(1<<APinNumber); }
static inline void klPinToggle (GPIO_TypeDef *PGpioPort, const uint16_t APinNumber) { PGpioPort->ODR ^= (uint16_t)(1<<APinNumber); }
// Check state
static inline bool klPinIsSet  (GPIO_TypeDef *PGpioPort, const uint16_t APinNumber) { return (PGpioPort->IDR & (uint16_t)(1<<APinNumber)); }
// Setup
static inline void klPinSetup  (GPIO_TypeDef *PGpioPort, const uint16_t APinNumber, const PinMode_t AMode, const PinSpeed_t ASpeed = ps50MHz) {
    // ==== Clock ====
    uint16_t IClock = 0;
    if     (PGpioPort == GPIOA) IClock = RCC_APB2Periph_GPIOA;
    else if(PGpioPort == GPIOB) IClock = RCC_APB2Periph_GPIOB;
    else if(PGpioPort == GPIOC) IClock = RCC_APB2Periph_GPIOC;
    else if(PGpioPort == GPIOD) IClock = RCC_APB2Periph_GPIOD;
    if((AMode == pmOutAFOpenDrain) or (AMode == pmOutAFPushPull)) IClock |= RCC_APB2Periph_AFIO;
    RCC->APB2ENR |= IClock;
    // ==== Pin ====
    uint32_t CnfMode = (uint32_t)AMode;
    // If output, modify mode with speed
    if(CnfMode & 0b0011) CnfMode &= (uint32_t)ASpeed;
    // Setup pull-up or pull-down
    else if(AMode == pmInPullUp) {
        CnfMode = (uint32_t)pmInPullDown;
        PGpioPort->ODR |= 1<<APinNumber;
    }
    else if(AMode == pmInPullDown) PGpioPort->ODR &= ~(1<<APinNumber);
    // Write needed bits to control reg
    if(APinNumber <= 7) {
        PGpioPort->CRL &= ~((uint32_t)0b1111 << (APinNumber * 4));  // clear previous bits
        PGpioPort->CRL |= (CnfMode << (APinNumber * 4));
    }
    else {
        PGpioPort->CRH &= ~((uint32_t)0b1111 << ((APinNumber - 8) * 4)); // clear previous bits
        PGpioPort->CRH |= (CnfMode << ((APinNumber - 8) * 4));
    }
}
// Fast switch
static inline void klPinOutPushPull(GPIO_TypeDef *PGpioPort, const uint16_t APinNumber, const PinSpeed_t ASpeed = ps50MHz) {
    uint32_t CnfMode = (uint32_t)pmOutPushPull & (uint32_t)ASpeed;
    // Write needed bits to control reg
    if(APinNumber <= 7) {
        PGpioPort->CRL &= ~((uint32_t)0b1111 << (APinNumber * 4));  // clear previous bits
        PGpioPort->CRL |= (CnfMode << (APinNumber * 4));
    }
    else {
        PGpioPort->CRH &= ~((uint32_t)0b1111 << ((APinNumber - 8) * 4)); // clear previous bits
        PGpioPort->CRH |= (CnfMode << ((APinNumber - 8) * 4));
    }
}
static inline void klPinInPullDown(GPIO_TypeDef *PGpioPort, const uint16_t APinNumber) {
    PGpioPort->ODR &= ~(1<<APinNumber);
    // Write needed bits to control reg
    if(APinNumber <= 7) {
        PGpioPort->CRL &= ~((uint32_t)0b1111 << (APinNumber * 4));  // clear previous bits
        PGpioPort->CRL |= ((uint32_t)pmInPullDown << (APinNumber * 4));
    }
    else {
        PGpioPort->CRH &= ~((uint32_t)0b1111 << ((APinNumber - 8) * 4)); // clear previous bits
        PGpioPort->CRH |= ((uint32_t)pmInPullDown << ((APinNumber - 8) * 4));
    }
}


// Disable JTAG, leaving SWD
static inline void klJtagDisable(void) {
    bool AfioWasEnabled = (RCC->APB2ENR & RCC_APB2ENR_AFIOEN);
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;     // Enable AFIO
    uint32_t tmp = AFIO->MAPR;
    tmp &= ~0x07000000; // Clear SWJ_CFG bits
    tmp |= 0x02000000;  // Set SWJ_CFG bits to 010 => JTAG disabled and SWD enabled
    AFIO->MAPR = tmp;   // Write data back to register
    if (!AfioWasEnabled) RCC->APB2ENR &= ~RCC_APB2ENR_AFIOEN;
}

// ==== Class for single pin manipulation ====
class klPinIrq_t {
private:
    uint16_t IPinMask;
    uint8_t IChannel;
public:
    void IrqSetup(GPIO_TypeDef *PGpioPort, const uint8_t APinNumber, EXTITrigger_TypeDef ATriggerType);
    void IrqEnable(void);
    void IrqDisable(void);
};

// ==== Timer ====
#define TIM_FREQ_MAX    0xFFFFFFFF
class klTimer_t {
protected:
    TIM_TypeDef* ITimer;
public:
    void Init(TIM_TypeDef* PTimer, uint16_t ATopValue, uint32_t AFreqHz);
    void Enable(void)  { ITimer->CR1 |= TIM_CR1_CEN; }
    void Disable(void) { ITimer->CR1 &= (uint16_t)(~((uint16_t)TIM_CR1_CEN)); }
    void SetFreqHz(uint32_t AFreqHz);
};

/*
 * TIM_OCPolarity_High, TIM_OCPolarity_Low
 */
class klPwmChannel_t : klTimer_t {
private:
    uint8_t INumber;
public:
    void Init(TIM_TypeDef* PTimer, uint16_t ATopValue, uint32_t AFreqHz, uint8_t ANumber, uint16_t APolarity);
    void Enable(void);
    void Disable(void);
    void DisableTimer(void) { klTimer_t::Disable(); }
    void Set(uint16_t AValue);
    void SetFreqHz(uint32_t AFreqHz) { klTimer_t::SetFreqHz(AFreqHz); }
};

// ========================= System clock and ID ===============================
enum Clk_t {clk1MHzInternal, clk2MHzInternal, clk4MHzInternal, clk8MHzInternal, clk8MHzExternal,
#if defined STM32F10X_HD
	clk36MHzExternal, clk72MHzExternal,
#endif
};
void InitClock(Clk_t AClk);

void GetUniqueID(uint32_t *IDArr);

// ============================ Delay and time =================================
// Systick is used here
extern uint32_t ITickCounter;
class Delay_t {
public:
    void Init(void);
    // Simple loop-based delays, no init needed
    void Loop (volatile uint32_t ACounter) { while(ACounter--); }
    void ms (uint32_t Ams);
    // Every tick is 0.5 ms long => prescaler = SYSCLK/2000. 0.5 ms needed for correct work when SysCoreClk >= 65 535 000 Hz
    //void PrescalerUpdate(void) { TIM2->PSC = (uint16_t)(SystemCoreClock / 2000) - 1; }
    // Timer-driven delays
    bool Elapsed(uint32_t *AVar, const uint32_t ADelay);
    void Reset  (uint32_t *AVar) { *AVar = ITickCounter; }
    void Bypass (uint32_t *AVar, const uint32_t ADelay) { *AVar = (uint32_t)(ITickCounter - ADelay); }
};
extern Delay_t Delay;
// IRQ handler
extern "C" {
void SysTick_Handler(void);
}

// ============================== UART command =================================
#define UART_TXBUF_SIZE     45
#define UART_DMA_CHNL       DMA1_Channel4

//#define RX_ENABLED

#ifdef RX_ENABLED
#define UART_RXBUF_SIZE     45
enum CmdState_t {csNone, csInProgress, csReady};
#endif

class CmdUnit_t {
private:
    uint8_t TXBuf1[UART_TXBUF_SIZE], TXBuf2[UART_TXBUF_SIZE];
    uint8_t *PBuf, TxIndx;
    bool IDmaIsIdle;
#ifdef RX_ENABLED
    CmdState_t CmdState;
    char RXBuf[UART_RXBUF_SIZE];
    uint8_t RxIndx;
    void CmdReset(void) { RxIndx = 0; CmdState = csNone; }
#endif
    void IStartTx(void);
    void IBufWrite(uint8_t AByte);
public:
    char UintToHexChar (uint8_t b) { return ((b<=0x09) ? (b+'0') : (b+'A'-10)); }
    void Printf(const char *S, ...);
    void FlushTx(void);
    void Init(uint32_t ABaudrate);
    void Task(void);
#ifdef RX_ENABLED
    void NewCmdHandler(void);   // Place it where needed
#endif
    // IRQ
    void IRQHandler(void);
};

// RX IRQ
#ifdef RX_ENABLED
extern "C" {
void USART1_IRQHandler(void);
}
#endif

extern CmdUnit_t Uart;


#endif /* KL_GPIO_H_ */
