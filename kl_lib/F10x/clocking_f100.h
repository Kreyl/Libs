/*
 * clocking_f0.h
 *
 *  Created on: 20.01.2013
 *      Author: kreyl
 */

#ifndef CLOCKING_H_
#define CLOCKING_H_

#include "stm32f10x.h"

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
 * Flash latency requires no attention as top freq of 24 MHz is too low to affect Flash.
 *
 * AHB  freq max = 24 MHz;
 * APB1 freq max = 24 MHz;
 * APB2 freq max = 24 MHz;
 */

#define CRYSTAL_FREQ_HZ     8000000     // Freq of external crystal, change accordingly
#define HSI_FREQ_HZ         HSI_VALUE   // Freq of internal generator, not adjustable
#define LSI_FREQ_HZ         40000       // Freq of internal generator, not adjustable

enum ClkSrc_t {csHSI, csHSE, csPLL};
enum PllMul_t {
    pllMul2 = 0b0000,
    pllMul3 = 0b0001,
    pllMul4 = 0b0010,
    pllMul5 = 0b0011,
    pllMul6 = 0b0100,
    pllMul7 = 0b0101,
    pllMul8 = 0b0110,
    pllMul9 = 0b0111,
    pllMul10= 0b1000,
    pllMul11= 0b1001,
    pllMul12= 0b1010,
    pllMul13= 0b1011,
    pllMul14= 0b1100,
    pllMul15= 0b1101,
    pllMul16= 0b1110
};

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

enum ADCDiv_t {
    adcDiv2=RCC_CFGR_ADCPRE_DIV2,
    adcDiv4=RCC_CFGR_ADCPRE_DIV4,
    adcDiv6=RCC_CFGR_ADCPRE_DIV6,
    adcDiv8=RCC_CFGR_ADCPRE_DIV8
};

class Clk_t {
private:
    uint8_t HSEEnable();
    uint8_t HSIEnable();
    uint8_t PLLEnable();
public:
    // Frequency values
    uint32_t AHBFreqHz;     // HCLK: AHB Bus, Core, Memory, DMA; 24 MHz max
    uint32_t APB1FreqHz;    // PCLK1: APB1 Bus clock; 24 MHz max
    uint32_t APB2FreqHz;    // PCLK2: APB2 Bus clock; 24 MHz max
    // SysClk switching
    uint8_t SwitchToHSI();
    uint8_t SwitchToHSE();
    uint8_t SwitchToPLL();
    void HSEDisable() { RCC->CR &= ~RCC_CR_HSEON; }
    void HSIDisable() { RCC->CR &= ~RCC_CR_HSION; }
    void PLLDisable() { RCC->CR &= ~RCC_CR_PLLON; }
    void SetupBusDividers(AHBDiv_t AHBDiv, APBDiv_t APB1Div, APBDiv_t APB2Div);
    uint8_t SetupPLLDividers(uint8_t HsePreDiv, PllMul_t PllMul);
    void UpdateFreqValues();
    // Other clocks
    void SetupAdcClk(ADCDiv_t ADCDiv);
    void LsiEnable() {
        RCC->CSR |= RCC_CSR_LSION;
        while(!(RCC->CSR & RCC_CSR_LSIRDY));
    }
    void LsiDisable() { RCC->CSR &= RCC_CSR_LSION; }
};

extern Clk_t Clk;

/*
 * Early initialization code.
 * This initialization must be performed just after stack setup and before
 * any other initialization.
 */
extern "C" {
void __early_init(void);
}

#endif /* CLOCKING_H_ */
