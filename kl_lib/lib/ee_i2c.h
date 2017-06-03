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

    uint8_t Read(uint8_t MemAddr, void *Ptr, uint32_t Length) const {
        Resume();
        uint8_t Rslt = i2c->WriteRead(EE_I2C_ADDR, &MemAddr, 1, (uint8_t*)Ptr, Length);
        Standby();
//        Uart.Printf("Read: %u\r", Rslt);
        return Rslt;
    }

    template <typename T>
    uint8_t Read(uint8_t MemAddr, T* Ptr) const {
        Resume();
        uint8_t Rslt = i2c->WriteRead(EE_I2C_ADDR, &MemAddr, 1, (uint8_t*)Ptr, sizeof(T));
        Standby();
        return Rslt;
    }

    uint8_t Write(uint8_t MemAddr, void *Ptr, uint32_t Length) const {
//        Uart.Printf("Wr: %u; len=%u\r", MemAddr, Length);
        uint8_t *p8 = (uint8_t*)Ptr;
        Resume();
        // Write page by page
        while(Length) {
            uint8_t ToWriteCnt = (Length > EE_PAGE_SZ)? EE_PAGE_SZ : Length;
            // Try to write
            uint32_t Retries = 0;
            while(true) {
//                Uart.Printf("Wr: try %u\r", Retries);
                if(i2c->WriteWrite(EE_I2C_ADDR, &MemAddr, 1, p8, ToWriteCnt) == retvOk) {
                    Length -= ToWriteCnt;
                    p8 += ToWriteCnt;
                    MemAddr += ToWriteCnt;
                    break;  // get out of trying
                }
                else {
                    Retries++;
                    if(Retries > 7) {
                        Uart.Printf("EE Timeout1\r");
                        Standby();
                        return retvTimeout;
                    }
                    chThdSleepMilliseconds(1);   // Allow memory to complete writing
                }
            } // while trying
        } // while(Length)
        // Wait completion
        uint32_t Retries = 0;
        do {
    //        Uart.Printf("Wait: try %u\r", Retries);
            chThdSleepMilliseconds(1);
            Retries++;
            if(Retries > 7) {
                Uart.Printf("EE Timeout2\r");
                Standby();
                return retvTimeout;
            }
        } while(i2c->CheckAddress(EE_I2C_ADDR) != retvOk);
        Standby();
        return retvOk;
    }

    template <typename T>
    uint8_t Write(uint8_t MemAddr, T *Ptr) const {
        return Write(MemAddr, Ptr, sizeof(T));
    }
};
