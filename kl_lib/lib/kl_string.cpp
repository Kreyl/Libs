/*
 * kl_string.cpp
 *
 *  Created on: 5.12.2020
 *      Author: layst
 */

#include "kl_string.h"

static inline constexpr const uint32_t kUlongMax = 0xFFFFFFFFUL;
static inline constexpr const int32_t kLongMax = 0x7FFFFFFFUL;
static inline constexpr const int32_t kLongMin = 0x80000000UL;

__attribute__((__always_inline__))
static inline char ToLower(char c) {
    return (c >= 'A' and c <= 'Z')? (c + ('a' - 'A')) : c;
}

// tab; newline; vertical-tab; form-feed; carriage-return; space;
static inline bool IsSpace(char c) { return (c >= 0x09 and c <=0x0D) or (c == ' '); }

// 1; 2; 3; 4; 5; 6; 7; 8; 9; 0;
static inline bool IsDigit(char c) { return (c >= '0' and c <= '9'); }

// A; B; C; D; E; F; G; H; I; J; K; L; M; N; O; P; Q; R; S; T; U; V; W; X; Y; Z; a; b; c; d; e; f; g; h; i; j; k; l; m; n; o; p; q; r; s; t; u; v; w; x; y; z;
static inline bool IsAlpha(char c) { return (c >= 'A' and c <= 'Z') or (c >= 'a' and c <= 'z'); }

// A; B; C; D; E; F; G; H; I; J; K; L; M; N; O; P; Q; R; S; T; U; V; W; X; Y; Z;
static inline bool IsUpper(char c) { return (c >= 'A' and c <= 'Z'); }

static inline RetvValU8 char2byte(char c) {
    RetvValU8 r(retv::Ok);
    if     (c >= '0' and c <= '9') *r =  c - '0';
    else if(c >= 'a' and c <= 'f') *r = (c - 'a') + 10;
    else if(c >= 'A' and c <= 'F') *r = (c - 'A') + 10;
    else r.rslt = retv::Fail;
    return r;
}

namespace Str {

/* Compare S1 and S2, ignoring case. Returning less than, equal to or
   greater than zero if S1 is lexicographically less than,
   equal to or greater than S2.  */
int32_t CmpCase(const char *s1, const char *s2) {
  const unsigned char *p1 = (const unsigned char *) s1;
  const unsigned char *p2 = (const unsigned char *) s2;
  int result;
  if(p1 == p2) return 0;
  while((result = ToLower(*p1) - ToLower(*p2++)) == 0) {
      if(*p1++ == '\0') break;
  }
  return result;
}

// Compare N chars of S1 and S2
int32_t CmpN(const char *s1, const char *s2, int n) {
    const unsigned char *p1 = (const unsigned char *)s1;
    const unsigned char *p2 = (const unsigned char *)s2;
    int result;
    if(p1 == p2) return 0;
    while(n-- > 0 and (result = *p1 - *p2++) == 0) {
        if(*p1++ == '\0') break;
    }
    return result;
}

// Compare N chars of S1 and S2, ignoring case
int32_t CmpNCase(const char *s1, const char *s2, int n) {
    const unsigned char *p1 = (const unsigned char *) s1;
    const unsigned char *p2 = (const unsigned char *) s2;
    int result;
    if(p1 == p2) return 0;
    while(n-- > 0 and (result = ToLower(*p1) - ToLower(*p2++)) == 0) {
        if(*p1++ == '\0') break;
    }
    return result;
}

// Split str to tokens. When S==nullptr, proceed using remainer instead of starting from the beginning
char* Tokens(char* s, const char* delim, char**remainer) {
    if(s == nullptr and (s = *remainer) == nullptr) return nullptr;
    char* spanp;
    // Skip leading delimiters
    cont:
    char c = *s++, sc;
    for(spanp = (char*)delim; (sc = *spanp++) != 0;) {
        if(c == sc) goto cont;
    }

    if(c == 0) {    // no non-delimiter characters left, but string ended
        *remainer = nullptr;
        return nullptr;
    }
    char* tok = s - 1;
    while(true) {
        c = *s++;
        spanp = (char*)delim;
        do {
            if((sc = *spanp++) == c) {
                if(c == 0) s = nullptr;
                else *(s-1) = 0;
                *remainer = s;
                return tok;
            }
        } while (sc != 0);
    }
}

// Returns length of string
int32_t Len(const char* s) {
    if(s == nullptr) return 0;
    int32_t length = 0;
    while(s[length] != '\0') ++length;
    return length;
}


long ToInt32(const char* nptr, char** endptr, int base) {
    const char *s = nptr;
    unsigned long acc;
    int c;
    unsigned long cutoff;
    int neg = 0, any, cutlim;

    /*
     * Skip white space and pick up leading +/- sign if any.
     * If base is 0, allow 0x for hex and 0 for octal, else
     * assume decimal; if base is already 16, allow 0x.
     */
    do {
        c = *s++;
    } while(IsSpace(c));
    if(c == '-') {
        neg = 1;
        c = *s++;
    }
    else if(c == '+') c = *s++;
    if((base == 0 || base == 16) && c == '0' && (*s == 'x' || *s == 'X')) {
        c = s[1];
        s += 2;
        base = 16;
    }
    if(base == 0) base = c == '0' ? 8 : 10;

    /*
     * Compute the cutoff value between legal numbers and illegal
     * numbers.  That is the largest legal value, divided by the
     * base.  An input number that is greater than this value, if
     * followed by a legal input character, is too big.  One that
     * is equal to this value may be valid or not; the limit
     * between valid and invalid numbers is then based on the last
     * digit.  For instance, if the range for longs is
     * [-2147483648..2147483647] and the input base is 10,
     * cutoff will be set to 214748364 and cutlim to either
     * 7 (neg==0) or 8 (neg==1), meaning that if we have accumulated
     * a value > 214748364, or equal but the next digit is > 7 (or 8),
     * the number is too big, and we will return a range error.
     *
     * Set any if any `digits' consumed; make it negative to indicate
     * overflow.
     */
    cutoff = neg ? -(unsigned long) kLongMin : kLongMax;
    cutlim = cutoff % (unsigned long) base;
    cutoff /= (unsigned long) base;
    for(acc = 0, any = 0;; c = *s++) {
        if(IsDigit(c)) c -= '0';
        else if(IsAlpha(c)) c -= IsUpper(c) ? 'A' - 10 : 'a' - 10;
        else break;
        if(c >= base) break;
        if(any < 0 || acc > cutoff || (acc == cutoff && c > cutlim)) any = -1;
        else {
            any = 1;
            acc *= base;
            acc += c;
        }
    }
    if(any < 0) {
        acc = neg ? kLongMin : kLongMax;
    }
    else if(neg) acc = -acc;
    if(endptr != 0) *endptr = (char*) (any ? s - 1 : nptr);
    return (acc);
}

unsigned long ToUint32(const char* nptr, char** endptr, int base) {
    const char *s = nptr;
    unsigned long acc, cutoff;
    int c;
    int neg = 0, any, cutlim;

    do {
        c = *s++;
    } while(IsSpace(c));
    if(c == '-') {
        neg = 1;
        c = *s++;
    }
    else if(c == '+') c = *s++;
    if((base == 0 || base == 16) && c == '0' && (*s == 'x' || *s == 'X')) {
        c = s[1];
        s += 2;
        base = 16;
    }
    if(base == 0) base = c == '0' ? 8 : 10;

    cutoff = kUlongMax / (unsigned long)base;
    cutlim = kUlongMax % (unsigned long)base;
    for(acc = 0, any = 0; ; c = *s++) {
        if(IsDigit(c)) c -= '0';
        else if(IsAlpha(c)) { c -= IsUpper(c) ? 'A' - 10 : 'a' - 10; }
        else break;
        if(c >= base) break;
        if(any < 0) continue;
        if(acc >= cutoff && c > cutlim) {
            any = -1;
            acc = kUlongMax;
        }
        else {
            any = 1;
            acc *= (unsigned long)base;
            acc += c;
        }
    }
    if(neg && any > 0) acc = -acc;
    if(endptr != 0) *endptr = (char*) (any ? s - 1 : nptr);
    return acc;
}

// Convert first two chars to byte
RetvValU8 HexToByte(const char* S) {
    RetvValU8 r(retv::Fail);
    if(S != nullptr) {
        RetvValU8 b = char2byte(*S++);
        if(b.IsOk()) {
            *r = (*b) << 4;
            b = char2byte(*S);
            if(b.IsOk()) {
                *r |= *b;
                r.rslt = retv::Ok;
            }
        }
    }
    return r;
}

// Convert 18FF0A to Color_t(0x18, 0xFF, 0x0A)
RetvValColor HexToColor(const char* S) {
    RetvValColor r(retv::Fail);
    RetvValU8 b = HexToByte(S);
    if(b.IsOk()) {
        r->R = *b;
        S += 2;
        b = HexToByte(S);
        if(b.IsOk()) {
            r->G = *b;
            S += 2;
            b = HexToByte(S);
            if(b.IsOk()) {
                r->B = *b;
                r.rslt = retv::Ok;
            }
        }
    }
    return r;
}

bool StartsWith(const char* s, const char* prefix) {
    if(s == prefix) return true;
    while(*s++ == *prefix) {
        prefix++;
        if(*prefix == '\0') return true;
    }
    return false;
}

} // namespace Str