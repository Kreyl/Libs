/*
 * log2file.cpp
 *
 *  Created on: 16 февр. 2020 г.
 *      Author: layst
 */

#include "log2file.h"
#include "kl_lib.h"
#include "stdarg.h"
#include "kl_fs_utils.h"

Log_t Log;

#if LOG_ENABLED
static FIL LogFile;
static uint8_t IBuf1[LOG_BUF_SZ], IBuf2[LOG_BUF_SZ], *ptr = IBuf1, *ptrEnd = &IBuf1[LOG_BUF_SZ];
//static ch_semaphore ISem;

void Log_t::Start(const char* AFname) {
    TryOpenFileRewrite(AFname, &LogFile);
//    chSemObjectInit(&ISem, 1);
}

uint8_t Log_t::IPutChar(char c) {
    *ptr++ = c;
    if(ptr >= ptrEnd) {
        uint32_t bw;
        // Switch buffer
        if(ptrEnd == &IBuf1[LOG_BUF_SZ]) {
            ptr = IBuf2;
            ptrEnd = &IBuf2[LOG_BUF_SZ];
            f_write(&LogFile, IBuf1, LOG_BUF_SZ, &bw);
        }
        else { // was buf2
            ptr = IBuf1;
            ptrEnd = &IBuf1[LOG_BUF_SZ];
            f_write(&LogFile, IBuf2, LOG_BUF_SZ, &bw);
        }
    }
    return retvOk;
}

void Log_t::Write(const char *format, ...) {
    if(!FileIsOpen(&LogFile)) return;
//    chSemWait(&ISem);
#if LOG_PUT_TIME
    IPutUint(chVTGetSystemTimeX(), 10, 7, ' ');
    IPutChar(' ');
#endif
    va_list args;
    va_start(args, format);
    IVsPrintf(format, args);
    va_end(args);
//    chSemSignal(&ISem);
}

void Log_t::Stop() {
    uint32_t bw;
    if(ptrEnd == &IBuf1[LOG_BUF_SZ]) {
        uint32_t Sz = ptr - IBuf1;
        if(Sz) f_write(&LogFile, IBuf1, Sz, &bw);
    }
    else { // was buf2
        uint32_t Sz = ptr - IBuf2;
        if(Sz) f_write(&LogFile, IBuf2, Sz, &bw);
    }
    f_close(&LogFile);
}

#else
void Log_t::Start(const char* AFname) { }
void Log_t::Write(const char *format, ...) { }
void Log_t::Stop() { }
uint8_t Log_t::IPutChar(char c) { return retvOk; }
#endif
