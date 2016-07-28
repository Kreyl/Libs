#ifndef KL_SPRINTF_H
#define KL_SPRINTF_H

#include <stdarg.h>
#include <stdint.h>

/*
Supported format specifiers:
%s - string
%c - char
%d - int
%[0][<width>]u - uint
%[0][<width>]X - uint as hex
%A - pair (uint8_t *arr, int len) as hex array
*/

#ifndef ftVoidChar
typedef void(*ftVoidChar)(char);
#endif

#ifdef __cplusplus
extern "C" {
#endif
uint32_t kl_vsprintf(ftVoidChar PPutChar, uint32_t MaxLength, const char *format, va_list args);
#ifdef __cplusplus
}
#endif

#endif
