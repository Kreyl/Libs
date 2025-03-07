/*
 * kl_string.h
 *
 *  Created on: 5.12.2020
 *      Author: layst
 */

#ifndef KL_STRING_H_
#define KL_STRING_H_

#include "types.h"
#include "color.h"

using RetvValColor = RetvVal<Color_t>;

namespace Str {

char* Cpy(char* dst, const char* src);
char* Chr(const char* s, char c);
int32_t CmpCase(const char *s1, const char *s2);
int32_t CmpN(const char *s1, const char *s2, int n);
int32_t CmpNCase(const char *s1, const char *s2, int n);
char* Tokens(char* s, const char* delim, char**remainer);
int32_t sscanf(const char* s, const char* format, ...);
int32_t Len(const char* s);
int32_t ToInt32(const char* nptr, char** endptr, int base);
uint32_t ToUint32(const char* nptr, char** endptr, int base);

// FF->0xFF
RetvValU8 HexToByte(const char* S);

RetvValColor HexToColor(const char* S);
retv HexToColor(const char* S, Color_t* pclr);

bool StartsWith(const char* s, const char* prefix);
bool StartsWithCase(const char* s, const char* prefix);
bool EndsWith(const char* s, const char* suffix);
bool EndsWithCase(const char* s, const char* suffix);

char *CopyReplacingSpecCharsWSpace(char* dst, const char* src);

} // namespace Str

#endif // KL_STRING_H_
