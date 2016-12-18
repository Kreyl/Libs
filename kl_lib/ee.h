/*
 * ee.h
 *
 *  Created on: 3 мая 2016 г.
 *      Author: Kreyl
 */

#pragma once

#include "board.h"
#include "kl_lib.h"
#include "kl_i2c.h"

#define EE_I2C_ADDR     0x50    // A0=A1=A2=0
// Number of bytes to be written simultaneously. IC dependant, see datasheet.
#define EE_PAGE_SZ      8

class EE_t {
private:
    i2c_t *i2c;
#ifdef EE_PWR_PIN
    const PinOutput_t IPwrPin;
#endif
public:
    void Init() const {
#ifdef EE_PWR_PIN
        IPwrPin.Init();
#endif
    }
    void Resume() const {
#ifdef EE_PWR_PIN
        IPwrPin.Init();
        IPwrPin.SetHi();
        __NOP(); __NOP(); __NOP(); __NOP(); // Allow power to rise
#endif
        i2c->Resume();
    }
    void Standby() const {
        i2c->Standby();
#ifdef EE_PWR_PIN
        IPwrPin.SetLo();
        IPwrPin.Deinit();
#endif
    }

#ifdef EE_PWR_PIN
    EE_t(i2c_t *pi2c, const PinOutput_t APwrPin) : i2c(pi2c), IPwrPin(APwrPin) {}
#else
    EE_t(i2c_t *pi2c) : i2c(pi2c) {}
#endif
    uint8_t Read (uint8_t MemAddr, void *Ptr, uint32_t Length) const;
    uint8_t Write(uint8_t MemAddr, void *Ptr, uint32_t Length) const;
};

extern const EE_t ee;
