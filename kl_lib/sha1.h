/*
 * sha1.h
 *
 *  Created on: 13.06.2012
 *      Author: g.kruglov
 */

#ifndef SHA1_H_
#define SHA1_H_

#include <stdint.h>

#define SHA_OUTPUT_CHAR
//#define SHA_OUTPUT_ARR

#ifdef SHA_OUTPUT_CHAR
extern char Sha1String[41]; // 40+\0
#endif

#ifdef SHA_OUTPUT_ARR
extern uint32_t Sha1Array[5];
#endif

void Sha1(const char *S);


#endif /* SHA1_H_ */
