/*
 * usb_cdc.h
 *
 *  Created on: 03 сент. 2015 г.
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
};

extern UsbCDC_t UsbCDC;
