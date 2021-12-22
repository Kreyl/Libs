/*
 * kl_string.cpp
 *
 *  Created on: 5 дек. 2020 г.
 *      Author: layst
 */

#include "kl_string.h"

__inline__ __attribute__((__always_inline__))
int kl_tolower(char c) {
    return (c >= 'A' and c <= 'Z')? (c + ('a' - 'A')) : c;
}

/* Compare S1 and S2, ignoring case, returning less than, equal to or
   greater than zero if S1 is lexicographically less than,
   equal to or greater than S2.  */
int kl_strcasecmp(const char *s1, const char *s2) {
  const unsigned char *p1 = (const unsigned char *) s1;
  const unsigned char *p2 = (const unsigned char *) s2;
  int result;
  if (p1 == p2) return 0;
  while((result = kl_tolower(*p1) - kl_tolower(*p2++)) == 0) {
      if(*p1++ == '\0') break;
  }
  return result;
}

char* kl_strtok(register char* s, register const char* delim, register char**PLast) {
    if(s == nullptr and (s = *PLast) == nullptr) return nullptr;
    register char* spanp;
    // Skip leading delimiters
    cont:
    register char c = *s++, sc;
    for(spanp = (char*)delim; (sc = *spanp++) != 0;) {
        if(c == sc) goto cont;
    }

    if(c == 0) {    // no non-delimiter characters left, but string ended
        *PLast = nullptr;
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
                *PLast = s;
                return tok;
            }
        } while (sc != 0);
    }
}

int kl_strlen(const char* s) {
    const char *fs;
    for(fs = s; *fs; ++fs);
    return (fs - s);
}

