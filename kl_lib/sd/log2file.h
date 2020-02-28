/*
 * log2file.h
 *
 *  Created on: 16 февр. 2020 г.
 *      Author: layst
 */

#pragma once

#include "ch.h"
#include "shell.h"

#define LOG_ENABLED     FALSE
#define LOG_BUF_SZ      512
#define LOG_PUT_TIME    TRUE

class Log_t : public PrintfHelper_t {
private:
    uint8_t IPutChar(char c);
    void IStartTransmissionIfNotYet() {} // dummy
public:
    void Start(const char* AFname);
    void Write(const char *format, ...);
    void Stop();
};

extern Log_t Log;
