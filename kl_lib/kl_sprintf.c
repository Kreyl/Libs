#include "kl_sprintf.h"
#include <stdint.h>
#include <stdbool.h>
#include "chtypes.h"
#include "board.h"

#if PRINTF_FLOAT_EN
#define FLOAT_PRECISION     9
static const long power10Table[FLOAT_PRECISION] = {
    10, 100, 1000, 10000, 100000, 1000000, 10000000, 100000000, 1000000000
};
#endif

uint32_t kl_vsprintf(ftVoidChar PPutChar, uint32_t MaxLength, const char *format, va_list args) {
    uint32_t CharCnt = 0, width = 0, precision;
    char filler;
    // Print number n to buffer p in base base. If number is shorter
    // than width, it's prepended with spaces or zeros (if zero_padded
    // is set) from the left.
    void IPutUint(uint32_t n, uint32_t base) {
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
            PPutChar(filler);
            if(++CharCnt >= MaxLength) return;
        }
        // Print digits
        len = (len < (MaxLength - CharCnt))? len : (MaxLength - CharCnt);
        CharCnt += len;
        while(len > 0) PPutChar(digits[--len]);
    } // IPutUint

    const char *fmt = format;
    char c;
    while(true) {
        c = *fmt++;
        if(c == 0) goto End;
        if(c != '%') {
            PPutChar(c);
            if(++CharCnt >= MaxLength) goto End;
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
                PPutChar(va_arg(args, int));
                CharCnt++;
                break;

            case 's':
            case 'S':
            {
                char *s = va_arg(args, char*);
                while((*s != 0) && (CharCnt++ < MaxLength)) PPutChar(*s++);
            }
            break;

            case 'X': IPutUint(va_arg(args, uint32_t), 16); break;
            case 'u': IPutUint(va_arg(args, uint32_t), 10); break;

            case 'd':
            case 'i':
            {
                int32_t n = va_arg(args, int32_t);
                if(n < 0) {
                    PPutChar('-');
                    if(++CharCnt >= MaxLength) goto End;
                    n = -n;
                }
                IPutUint(n, 10);
            }
            break;

#if PRINTF_FLOAT_EN
            case 'f': {
                float f = (float)va_arg(args, double);
                if (f < 0) {
                    PPutChar('-');
                    if(++CharCnt >= MaxLength) goto End;
                    f = -f;
                }
                int32_t n;
                if((precision == 0) || (precision > FLOAT_PRECISION)) precision = FLOAT_PRECISION;
                n = (int32_t)f;
                IPutUint(n, 10);
                if(CharCnt >= MaxLength) goto End;
                PPutChar('.');
                if(++CharCnt >= MaxLength) goto End;
                filler = '0';
                width = precision;
                n = (long)((f - n) * power10Table[precision - 1]);
                IPutUint(n, 10);
            } break;
#endif

            case 'A': {
                uint8_t *arr = va_arg(args, uint8_t*);
                int32_t n = va_arg(args, int32_t);
                int32_t Delimiter = va_arg(args, int32_t);
                filler = '0';       // }
                width = 2;          // } 01 02 0A etc.; not 1 2 A
                for(int32_t i = 0; i < n; i++) {
                    if((i > 0) && (Delimiter != 0)) {
                        PPutChar((char)Delimiter); // do not place delimiter before or after array
                        if(++CharCnt >= MaxLength) goto End;
                    }
                    IPutUint(arr[i], 16);
                    if(CharCnt >= MaxLength) goto End;
                }
            } break;

            case '%':
                PPutChar('%');
                CharCnt++;
                break;

        } // switch
    } // while
    End:
    return CharCnt;
}



uint32_t kl_bufprint(char *Buf, uint32_t MaxLength, const char *format, ...) {
    void IPutChar(char c) { *Buf++ = c; }
    va_list args;
    va_start(args, format);
    uint32_t Rslt = kl_vsprintf(IPutChar, MaxLength-1, format, args);
    va_end(args);
    *Buf = 0;   // End of string
    return Rslt;
}
