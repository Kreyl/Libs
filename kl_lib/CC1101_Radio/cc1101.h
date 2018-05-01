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

#define CC_BUSYWAIT_TIMEOUT     99000   // tics, not ms

class cc1101_t : public IrqHandler_t {
private:
    const Spi_t ISpi;
    const GPIO_TypeDef *PGpio;
    const uint16_t Sck, Miso, Mosi, Cs;
    const PinIrq_t IGdo0;
    volatile uint8_t IState; // Inner CC state, returned as first byte
    thread_reference_t ThdRef;
    ftVoidVoid ICallback;
    // Pins
    uint8_t BusyWait() {
        for(uint32_t i=0; i<CC_BUSYWAIT_TIMEOUT; i++) {
            if(PinIsLo(PGpio, Miso)) return retvOk;
        }
        return retvFail;
    }
    void CsHi() { PinSetHi((GPIO_TypeDef*)PGpio, Cs); }
    void CsLo() { PinSetLo((GPIO_TypeDef*)PGpio, Cs); }
    // General
    void RfConfig();
    int8_t RSSI_dBm(uint8_t ARawRSSI);
    // Registers and buffers
    uint8_t WriteRegister(const uint8_t Addr, const uint8_t AData);
    uint8_t ReadRegister(const uint8_t Addr, uint8_t *PData);
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
    void SetPktSize(uint8_t ASize) { WriteRegister(CC_PKTLEN, ASize); }
    // State change
    void Transmit(void *Ptr, uint8_t Len);
    uint8_t Receive(uint32_t Timeout_ms, void *Ptr, uint8_t Len,  int8_t *PRssi=nullptr);
    uint8_t EnterIdle()    { return WriteStrobe(CC_SIDLE); }
    uint8_t EnterPwrDown() { return WriteStrobe(CC_SPWD);  }
    uint8_t GetState()     {
        WriteStrobe(CC_SNOP);
        return IState;
    }
    uint8_t Recalibrate() {
        do {
            if(EnterIdle() != retvOk) return retvFail;
        } while(IState != CC_STB_IDLE);
        if(WriteStrobe(CC_SCAL) != retvOk) return retvFail;
        return BusyWait();
    }
    void ReceiveAsync(ftVoidVoid Callback);
    void TransmitAsync(void *Ptr, uint8_t Len, ftVoidVoid Callback);

    uint8_t ReadFIFO(void *Ptr, int8_t *PRssi, uint8_t Len);

    void IIrqHandler() {
        if(ICallback != nullptr) {
            ICallback();
            ICallback = nullptr;
        }
        else chThdResumeI(&ThdRef, MSG_OK);  // NotNull check performed inside chThdResumeI
    }
    cc1101_t(
            SPI_TypeDef *ASpi, GPIO_TypeDef *APGpio,
            uint16_t ASck, uint16_t AMiso, uint16_t AMosi, uint16_t ACs, uint16_t AGdo0):
        ISpi(ASpi), PGpio(APGpio),
        Sck(ASck), Miso(AMiso), Mosi(AMosi), Cs(ACs),
        IGdo0(APGpio, AGdo0, pudNone, this),
        IState(0), ThdRef(nullptr), ICallback(nullptr) {}
};
