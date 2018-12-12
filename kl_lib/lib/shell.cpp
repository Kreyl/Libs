/*
 * shell.cpp
 *
 *  Created on: 21 апр. 2017 г.
 *      Author: Kreyl
 */

#include "shell.h"
#include "uart.h"

extern CmdUart_t Uart;

void Printf(const char *format, ...) {
    va_list args;
    va_start(args, format);
    chSysLock();
    Uart.IVsPrintf(format, args);
    chSysUnlock();
    va_end(args);
}

void Printf(CmdUart_t &AUart, const char *format, ...) {
    va_list args;
    va_start(args, format);
    chSysLock();
    AUart.IVsPrintf(format, args);
    chSysUnlock();
    va_end(args);
}

void PrintfI(const char *format, ...) {
    va_list args;
    va_start(args, format);
    Uart.IVsPrintf(format, args);
    va_end(args);
}

void PrintfEOL() {
    Uart.PrintEOL();
}

extern "C" {
void PrintfC(const char *format, ...) {
    va_list args;
    va_start(args, format);
    Uart.IVsPrintf(format, args);
    va_end(args);
}
} // exern C


class PrintToBuf_t : public PrintfHelper_t {
public:
    char *S;
    uint8_t IPutChar(char c) {
        *S++ = c;
        return retvOk;
    }
    void IStartTransmissionIfNotYet() {}
};

char* PrintfToBuf(char* PBuf, const char *format, ...) {
    PrintToBuf_t PtB;
    PtB.S = PBuf;
    va_list args;
    va_start(args, format);
    PtB.IVsPrintf(format, args);
    va_end(args);
    *PtB.S = 0;
    return PtB.S;
}

#if 0
void ByteShell_t::Reply(uint8_t CmdCode, uint32_t Len, uint8_t *PData) {
//    Printf("BSendCmd %X; %u; %A\r", CmdCode, Len, PData, Len, ' ');
    // Send StartOfCmd
    if(IPutChar('#') != retvOk) return;
    // Send command code
    if(IPutChar(HalfByte2Char(CmdCode >> 4)) != retvOk) return;
    if(IPutChar(HalfByte2Char(CmdCode)) != retvOk) return;
    // Send data
    for(uint32_t i=0; i<Len; i++) {
        if(IPutChar(HalfByte2Char((*PData) >> 4)) != retvOk) return;
        if(IPutChar(HalfByte2Char(*PData++)) != retvOk) return;
    }
    // Send EOL
    if(IPutChar('\r') != retvOk) return;
    if(IPutChar('\n') != retvOk) return;
    IStartTransmissionIfNotYet();
}
#endif

#if PRINTF_FLOAT_EN
#define FLOAT_PRECISION     9
static const long power10Table[FLOAT_PRECISION] = {
    10, 100, 1000, 10000, 100000, 1000000, 10000000, 100000000, 1000000000
};
#endif

void PrintfHelper_t::PrintEOL() {
    IPutChar('\r');
    IPutChar('\n');
    IStartTransmissionIfNotYet();
}

void PrintfHelper_t::IVsPrintf(const char *format, va_list args) {
    const char *fmt = format;
    uint32_t width = 0, precision;
    char c, filler;
    while(true) {
        c = *fmt++;
        if(c == 0) goto End;
        if(c != '%') {  // Not %
            if(IPutChar(c) != retvOk) goto End;
            else continue;
        }

        // Here goes optional width specification.
        // If it starts with zero (zero_padded is true), it means we use '0' instead of ' ' as a filler.
        filler = ' ';
        if(*fmt == '0') {
            fmt++;
            filler = '0';
        }

        width = 0;
        while(true) {
            c = *fmt++;
            if(c >= '0' && c <= '9') c -= '0';
            else if (c == '*') c = va_arg(args, int);
            else break;
            width = width * 10 + c;
        }

        precision = 0;
        if(c == '.') {
            while(true) {
                c = *fmt++;
                if(c >= '0' && c <= '9') c -= '0';
                else if(c == '*') c = va_arg(args, int);
                else break;
                precision = precision * 10 + c;
            }
        }

        // Command decoding
        switch(c) {
            case 'c':
                if(IPutChar(va_arg(args, int)) != retvOk) goto End;
                break;

            case 's':
            case 'S': {
                char *s = va_arg(args, char*);
                while(*s != 0) {
                    if(IPutChar(*s++) != retvOk) goto End;
                }
            }
            break;

            case 'X':
                if(IPutUint(va_arg(args, uint32_t), 16, width, filler) != retvOk) goto End;
                break;
            case 'u':
                if(IPutUint(va_arg(args, uint32_t), 10, width, filler) != retvOk) goto End;
                break;

            case 'd':
            case 'i':
            {
                int32_t n = va_arg(args, int32_t);
                if(n < 0) {
                    if(IPutChar('-') != retvOk) goto End;
                    n = -n;
                }
                if(IPutUint(n, 10, width, filler) != retvOk) goto End;
            }
            break;

#if PRINTF_FLOAT_EN
            case 'f': {
                float f = (float)va_arg(args, double);
                if (f < 0) {
                    if(IPutChar('-') != retvOk) goto End;
                    f = -f;
                }
                int32_t n;
                if((precision == 0) || (precision > FLOAT_PRECISION)) precision = FLOAT_PRECISION;
                n = (int32_t)f;
                if(IPutUint(n, 10, width, filler) != retvOk) goto End;
                if(IPutChar('.') != retvOk) goto End;
                filler = '0';
                width = precision;
                n = (long)((f - n) * power10Table[precision - 1]);
                if(IPutUint(n, 10, width, filler) != retvOk) goto End;
            } break;
#endif

            case 'A': {
                uint8_t *arr = va_arg(args, uint8_t*);
                int32_t n = va_arg(args, int32_t);
                int32_t Delimiter = va_arg(args, int32_t);
                filler = '0';       // }
                width = 2;          // } 01 02 0A etc.; not 1 2 A
                for(int32_t i = 0; i < n; i++) {
                    if((i > 0) && (Delimiter != 0)) { // do not place delimiter before or after array
                        if(IPutChar((char)Delimiter) != retvOk) goto End;
                    }
                    if(IPutUint(arr[i], 16, width, filler) != retvOk) goto End;
                }
            } break;

            case '%':
                if(IPutChar('%') != retvOk) goto End;
                break;
        } // switch
    } // while
    End:
    IStartTransmissionIfNotYet();
}

uint8_t PrintfHelper_t::IPutUint(uint32_t n, uint32_t base, uint32_t width, char filler) {
    char digits[10];
    uint32_t len = 0;
    // Place digits to buffer
    do {
        uint32_t digit = n % base;
        n /= base;
        digits[len++] = (digit < 10)? '0'+digit : 'A'+digit-10;
    } while(n > 0);
    // Add padding
    for(uint32_t i = len; i < width; i++) {
        if(IPutChar(filler) != retvOk) return retvOverflow;
    }
    // Print digits
    while(len > 0) {
        if(IPutChar(digits[--len]) != retvOk) return retvOverflow;
    }
    return retvOk;
} // IPutUint
