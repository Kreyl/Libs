/*
 * cc1101.h
 *
 *  Created on: Feb 12, 2013
 *      Author: g.kruglov
 */

#pragma once

#include <inttypes.h>
#include "kl_lib.h"
#include "cc1101defins.h"
#include "cc1101_rf_settings.h"
#include "cc_gpio.h"

#define CC_BUSYWAIT_TIMEOUT     99000   // tics, not ms

class cc1101_t {
private:
    uint8_t IState; // Inner CC state, returned as first byte
    Spi_t ISpi;
    uint8_t IPktSz;
    // Pins
    uint8_t BusyWait() {
        for(uint32_t i=0; i<CC_BUSYWAIT_TIMEOUT; i++) {
            if(!PinIsHi(CC_GPIO, CC_MISO)) return OK;
        }
        return FAILURE;
    }
    // General
    void RfConfig();
    int8_t RSSI_dBm(uint8_t ARawRSSI);
    // Registers and buffers
    uint8_t WriteRegister(const uint8_t Addr, const uint8_t AData);
    uint8_t ReadRegister(const uint8_t Addr);
    uint8_t WriteStrobe(uint8_t AStrobe);
    uint8_t WriteTX(uint8_t* Ptr, uint8_t Length);
    // Strobes
    uint8_t Reset()       { return WriteStrobe(CC_SRES); }
    uint8_t EnterTX()     { return WriteStrobe(CC_STX);  }
    uint8_t EnterRX()     { return WriteStrobe(CC_SRX);  }
    uint8_t FlushRxFIFO() { return WriteStrobe(CC_SFRX); }
public:
    uint8_t Init();
    void SetChannel(uint8_t AChannel);
    void SetTxPower(uint8_t APwr)  { WriteRegister(CC_PATABLE, APwr); }
    void SetPktSize(uint8_t ASize) { WriteRegister(CC_PKTLEN, ASize); IPktSz = ASize; }
    // State change
    void Transmit(void *Ptr);
    uint8_t Receive(uint32_t Timeout_ms, void *Ptr, int8_t *PRssi=nullptr);
    uint8_t Receive_st(systime_t Timeout_st, void *Ptr, int8_t *PRssi=nullptr);
    uint8_t EnterIdle()    { return WriteStrobe(CC_SIDLE); }
    uint8_t EnterPwrDown() { return WriteStrobe(CC_SPWD);  }
    uint8_t Recalibrate() {
        while(IState != CC_STB_IDLE) {
            if(EnterIdle() != OK) return FAILURE;
        }
        if(WriteStrobe(CC_SCAL) != OK) return FAILURE;
        return BusyWait();
    }
    uint8_t ReadFIFO(void *Ptr, int8_t *PRssi);
    cc1101_t(): IState(0), ISpi(CC_SPI), IPktSz(0) {}
};

extern cc1101_t CC;
