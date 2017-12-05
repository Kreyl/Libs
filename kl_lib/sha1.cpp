/*
 * sha1.cpp
 *
 *  Created on: 13.06.2012
 *      Author: g.kruglov
 */

#include "sha1.h"
#include <stdint.h>
#include "kl_string.h"

char Sha1String[41];
uint32_t Sha1Array[5];

void Sha1Reset(void);
void Sha1Result(void);
void SHA1PadMsg(void);
void SHA1ProcessMsgBlock(void);

// This structure holds context information for the hashing operation
struct SHA1Context_t {
    uint32_t MsgDigest[5];  // Message Digest (output)
    uint32_t LenLow;        // Message length in bits
    uint32_t LenHigh;       // Message length in bits
    char MsgBlock[64];      // 512-bit message blocks
    uint32_t MsgBlockIndx;  // Index in message block array
} Context;

void Sha1Reset(void) {
    Context.LenLow = 0;
    Context.LenHigh = 0;
    Context.MsgBlockIndx = 0;

    Context.MsgDigest[0] = 0x67452301;
    Context.MsgDigest[1] = 0xEFCDAB89;
    Context.MsgDigest[2] = 0x98BADCFE;
    Context.MsgDigest[3] = 0x10325476;
    Context.MsgDigest[4] = 0xC3D2E1F0;
}

// Output result to string
void Sha1Result(void) {
    SHA1PadMsg();
#ifdef SHA_OUTPUT_CHAR
    // Convert to string
    klSPrintf(Sha1String, "%X8%X8%X8%X8%X8",
            Context.MsgDigest[0],
            Context.MsgDigest[1],
            Context.MsgDigest[2],
            Context.MsgDigest[3],
            Context.MsgDigest[4]
            );
#endif
#ifdef SHA_OUTPUT_ARR
    for (uint8_t i=0; i<5; i++) Sha1Array[i] = Context.MsgDigest[i];
#endif
}

void Sha1(const char *S) {
    Sha1Reset();
    char c;
    while((c = *S++) != 0) {
        Context.MsgBlock[Context.MsgBlockIndx++] = c;
        Context.LenLow += 8;
        if (Context.LenLow == 0) Context.LenHigh++;  // We do not process situation of 2^64 length overflow
        if (Context.MsgBlockIndx == 64) SHA1ProcessMsgBlock();
    } // while
    Sha1Result();
}

/*
 *  SHA1PadMessage
 *      According to the standard, the message must be padded to an even
 *      512 bits.  The first padding bit must be a '1'.  The last 64
 *      bits represent the length of the original message.  All bits in
 *      between should be 0.  This function will pad the message
 *      according to those rules by filling the Message_Block array
 *      accordingly.  It will also call SHA1ProcessMessageBlock()
 *      appropriately.  When it returns, it can be assumed that the
 *      message digest has been computed.
 */
void SHA1PadMsg(void) {
    /*
     *  Check to see if the current message block is too small to hold
     *  the initial padding bits and length.  If so, we will pad the
     *  block, process it, and then continue padding into a second
     *  block.
     */
    Context.MsgBlock[Context.MsgBlockIndx++] = 0x80;
    if (Context.MsgBlockIndx > 56) {
        while(Context.MsgBlockIndx < 64) Context.MsgBlock[Context.MsgBlockIndx++] = 0;
        SHA1ProcessMsgBlock();
    }
    while(Context.MsgBlockIndx < 56) Context.MsgBlock[Context.MsgBlockIndx++] = 0;

    // Store the message length at the last 8 octets
    Context.MsgBlock[56] = (Context.LenHigh >> 24) & 0xFF;
    Context.MsgBlock[57] = (Context.LenHigh >> 16) & 0xFF;
    Context.MsgBlock[58] = (Context.LenHigh >> 8)  & 0xFF;
    Context.MsgBlock[59] = (Context.LenHigh)       & 0xFF;
    Context.MsgBlock[60] = (Context.LenLow >> 24)  & 0xFF;
    Context.MsgBlock[61] = (Context.LenLow >> 16)  & 0xFF;
    Context.MsgBlock[62] = (Context.LenLow >> 8)   & 0xFF;
    Context.MsgBlock[63] = (Context.LenLow)        & 0xFF;

    SHA1ProcessMsgBlock();
}

#define SHA1CircularShift(bits, word)   ((((word) << (bits)) & 0xFFFFFFFF) | ((word) >> (32-(bits))))

/*
 *  SHA1ProcessMessageBlock
 *      This function will process the next 512 bits of the message
 *      stored in the Message_Block array.
 *  Comments:
 *      Many of the variable names in the SHAContext, especially the
 *      single character names, were used because those were the names
 *      used in the publication.
  */
void SHA1ProcessMsgBlock(void) {
    const uint32_t K[] = {  // Constants defined in SHA-1
        0x5A827999,
        0x6ED9EBA1,
        0x8F1BBCDC,
        0xCA62C1D6
    };
    uint32_t t;             // Loop counter
    uint32_t temp;          // Temporary word value
    uint32_t W[80];         // Word sequence
    uint32_t A, B, C, D, E; // Word buffers

    // Initialize the first 16 words in the array W
    for(t=0; t < 16; t++) {
        W[t] =  ((uint32_t) Context.MsgBlock[t * 4    ]) << 24;
        W[t] |= ((uint32_t) Context.MsgBlock[t * 4 + 1]) << 16;
        W[t] |= ((uint32_t) Context.MsgBlock[t * 4 + 2]) << 8;
        W[t] |= ((uint32_t) Context.MsgBlock[t * 4 + 3]);
    }

    // Initialize other words in the array W
    for(t=16; t < 80; t++) {
        W[t] = SHA1CircularShift(1, W[t-3] ^ W[t-8] ^ W[t-14] ^ W[t-16]);
    }

    A = Context.MsgDigest[0];
    B = Context.MsgDigest[1];
    C = Context.MsgDigest[2];
    D = Context.MsgDigest[3];
    E = Context.MsgDigest[4];

    for(t=0; t < 20; t++) {
        temp =  SHA1CircularShift(5,A) + ((B & C) | ((~B) & D)) + E + W[t] + K[0];
        E = D;
        D = C;
        C = SHA1CircularShift(30,B);
        B = A;
        A = temp;
    }

    for(t=20; t < 40; t++) {
        temp = SHA1CircularShift(5,A) + (B ^ C ^ D) + E + W[t] + K[1];
        E = D;
        D = C;
        C = SHA1CircularShift(30,B);
        B = A;
        A = temp;
    }

    for(t=40; t < 60; t++) {
        temp = SHA1CircularShift(5,A) + ((B & C) | (B & D) | (C & D)) + E + W[t] + K[2];
        E = D;
        D = C;
        C = SHA1CircularShift(30,B);
        B = A;
        A = temp;
    }

    for(t=60; t < 80; t++) {
        temp = SHA1CircularShift(5,A) + (B ^ C ^ D) + E + W[t] + K[3];
        E = D;
        D = C;
        C = SHA1CircularShift(30,B);
        B = A;
        A = temp;
    }

    Context.MsgDigest[0] += A;
    Context.MsgDigest[1] += B;
    Context.MsgDigest[2] += C;
    Context.MsgDigest[3] += D;
    Context.MsgDigest[4] += E;

    Context.MsgBlockIndx = 0;
}

