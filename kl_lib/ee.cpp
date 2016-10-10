/*
 * ee.cpp
 *
 *  Created on: 3 мая 2016 г.
 *      Author: Kreyl
 */

#include "ee.h"
#include "board.h"
#include "kl_lib.h"
#include "uart.h"

const EE_t ee { &i2c3, EE_PWR_PIN };


uint8_t EE_t::Read(uint8_t MemAddr, void *Ptr, uint32_t Length) const {
    Resume();
    uint8_t Rslt = i2c->WriteRead(EE_I2C_ADDR, &MemAddr, 1, (uint8_t*)Ptr, Length);
    Standby();
    return Rslt;
}

uint8_t EE_t::Write(uint8_t MemAddr, void *Ptr, uint32_t Length) const {
    uint8_t *p8 = (uint8_t*)Ptr;
    Resume();
    // Write page by page
    while(Length) {
        uint8_t ToWriteCnt = (Length > EE_PAGE_SZ)? EE_PAGE_SZ : Length;
        // Try to write
        uint32_t Retries = 0;
        while(true) {
//            Uart.Printf("Wr: try %u\r", Retries);
            if(i2c->WriteWrite(EE_I2C_ADDR, &MemAddr, 1, p8, ToWriteCnt) == OK) {
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
                    return TIMEOUT;
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
            return TIMEOUT;
        }
    } while(i2c->CheckAddress(EE_I2C_ADDR) != OK);
    Standby();
    return OK;
}

