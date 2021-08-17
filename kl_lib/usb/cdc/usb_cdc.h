/*
 * usb_cdc.h
 *
 *  Created on: 03 ����. 2015 �.
 *      Author: Kreyl
 */

#pragma once

#include "hal.h"
#include "shell.h"

class UsbCDC_t : public PrintfHelper_t, public Shell_t {
private:
    void IStartTransmissionIfNotYet() {} // Dummy
    uint8_t IPutChar(char c);
public:
    void Init();
    void Connect();
    void Disconnect();
    bool IsActive();
    void Print(const char *format, ...) {
        va_list args;
        va_start(args, format);
        IVsPrintf(format, args);
        va_end(args);
    }
    void SignalCmdProcessed();
    uint8_t ReceiveBinaryToBuf(uint8_t *ptr, uint32_t Len, uint32_t Timeout_ms);
    uint8_t TransmitBinaryFromBuf(uint8_t *ptr, uint32_t Len, uint32_t Timeout_ms);
};

extern UsbCDC_t UsbCDC;
