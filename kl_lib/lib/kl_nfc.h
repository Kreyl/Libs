/*
 * kl_nfc.h
 *
 *  Created on: 2 џэт. 2018 у.
 *      Author: Kreyl
 */

#pragma once

#include "kl_lib.h"
#include "uart.h"

union NfcPkt_t {
    uint32_t DWord;
    struct {
        uint16_t ID;
        uint8_t Cmd;
        uint8_t Value;
        uint16_t crc; // must be last in pkt
    } __packed;
    NfcPkt_t& operator = (const NfcPkt_t &Right) {
        DWord = Right.DWord;
        return *this;
    }
    void CalculateCrc();
    void Print() { Printf("Id: %u; Cmd: %X; V: %u; crc: %X\r", ID, Cmd, Value, crc); }
} __packed;

#define NFCPKT_SZ       sizeof(NfcPkt_t)

class KlNfc_t : private BaseUart_t {
private:
    PinOutputPWM_t ITxPin;
    void IOnTxEnd();
public:
    NfcPkt_t PktRx;
    void Init();
    void Transmit(NfcPkt_t &Pkt);
    KlNfc_t(PwmSetup_t ATxPin, const UartParams_t *APParams) : BaseUart_t(APParams), ITxPin(ATxPin) {}
    // Inner use
    void ITask();
};

extern KlNfc_t Nfc;
