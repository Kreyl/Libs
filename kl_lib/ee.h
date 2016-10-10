/*
 * ee.h
 *
 *  Created on: 3 мая 2016 г.
 *      Author: Kreyl
 */

#pragma once

#include "board.h"
#include "kl_lib.h"
#ifdef STM32L4XX
#include "i2cL476.h"
#endif

#define EE_I2C_ADDR     0x50    // A0=A1=A2=0
// Number of bytes to be written simultaneously. IC dependant, see datasheet.
#define EE_PAGE_SZ      8

class EE_t {
private:
    i2c_t *i2c;
    const PinOutput_t IPwrPin;
public:
    void Init() const { IPwrPin.Init(); }
    void Resume() const {
        IPwrPin.Init();
        IPwrPin.Hi();
        __NOP(); __NOP(); __NOP(); __NOP(); // Allow power to rise
        i2c->Resume();
    }
    void Standby() const {
        i2c->Standby();
        IPwrPin.Lo();
        IPwrPin.Deinit();
    }
    EE_t(i2c_t *pi2c, const PinOutput_t APwrPin) : i2c(pi2c), IPwrPin(APwrPin) {}
    uint8_t Read (uint8_t MemAddr, void *Ptr, uint32_t Length) const;
    uint8_t Write(uint8_t MemAddr, void *Ptr, uint32_t Length) const;
};

extern const EE_t ee;
