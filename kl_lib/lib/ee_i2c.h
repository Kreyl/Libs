/*
 * ee.h
 *
 *  Created on: 3 ��� 2016 �.
 *      Author: Kreyl
 */

#pragma once

#include "board.h"
#include "kl_lib.h"
#include "kl_i2c.h"

/*
 * i2cAddr typically is [0x50; 0x57].
 * Page sz is number of bytes to be written simultaneously. IC dependant, see datasheet.
 * Bank Sz makes sense for multi-bank ICs, say 24c08.
 */

template <uint8_t Selfi2cAddr, uint32_t PageSz, uint32_t BankSz>
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

    uint8_t Read(uint32_t MemAddr, void *Ptr, uint32_t Length) const {
        uint8_t *p8 = (uint8_t*)Ptr;
        // Read bank by bank
        while(Length) {
            uint32_t BytesLeftInBank = BankSz - (MemAddr % BankSz);
            uint32_t ToReadCnt = (Length > BytesLeftInBank)? BytesLeftInBank : Length;
            // Construct i2c Addr with high address bits: A10, A9 and A8
            uint8_t i2cAddr = Selfi2cAddr | ((MemAddr >> 8) & 0x03);
            uint8_t ReadAddr = MemAddr & 0xFF;
            if(i2c->WriteRead(i2cAddr, &ReadAddr, 1, p8, ToReadCnt) != retvOk) return retvFail;
            Length -= ToReadCnt;
            p8 += ToReadCnt;
            MemAddr += ToReadCnt;
        }
        return retvOk;
    }

    template <typename T>
    uint8_t Read(uint8_t MemAddr, T* Ptr) const {
//        Resume();
        uint8_t Rslt = i2c->WriteRead(Selfi2cAddr, &MemAddr, 1, (uint8_t*)Ptr, sizeof(T));
//        Standby();
        return Rslt;
    }

    uint8_t Write(uint32_t MemAddr, void *Ptr, uint32_t Length) const {
//        Printf("Wr: %u; len=%u\r", MemAddr, Length);
        uint8_t *p8 = (uint8_t*)Ptr;
        // Write page by page
        while(Length) {
            uint32_t BytesLeftInPage = PageSz - (MemAddr % PageSz);
            uint32_t ToWriteCnt = (Length > BytesLeftInPage)? BytesLeftInPage : Length;
            // Construct i2c Addr with high address bits: A10, A9 and A8
            uint8_t i2cAddr = Selfi2cAddr | ((MemAddr >> 8) & 0x03);
            uint8_t WriteAddr = MemAddr & 0xFF;
            //Printf("MemAddr: %u; BytesLeftInPage: %u; ToWriteCnt: %u; i2cAddr: %X; WriteAddr: %u\r\n", MemAddr, BytesLeftInPage, ToWriteCnt, i2cAddr, WriteAddr);
            // Try to write
            uint32_t Retries = 0;
            while(true) {
//                Printf("Wr: try %u\r", Retries);
                if(i2c->WriteWrite(i2cAddr, &WriteAddr, 1, p8, ToWriteCnt) == retvOk) {
                    Length -= ToWriteCnt;
                    p8 += ToWriteCnt;
                    MemAddr += ToWriteCnt;
                    chThdSleepMilliseconds(7);
                    break;  // get out of trying
                }
                else {
                    Retries++;
                    if(Retries > 7) {
                        Printf("EE Timeout1\r");
                        Standby();
                        return retvTimeout;
                    }
                    chThdSleepMilliseconds(1);   // Allow memory to complete writing
                }
            } // while trying
        } // while(Length)
        return retvOk;
    }

    template <typename T>
    uint8_t Write(uint32_t MemAddr, T *Ptr) const {
        return Write(MemAddr, Ptr, sizeof(T));
    }
};
