/*
 * kl_crypto.cpp
 *
 *  Created on: 28 июн. 2023 г.
 *      Author: laurelindo
 */

#include "kl_crypto.h"
#include <cstring> // memcpy
#include "kl_lib.h"

#define SHA_Ch(x, y, z)      (((x) & ((y) ^ (z))) ^ (z))
#define SHA_Maj(x, y, z)     (((x) & ((y) | (z))) | ((y) & (z)))

/* Define the SHA shift, rotate left, and rotate right macros */
#define SHA256_SHR(bits,word)   ((word) >> (bits))
#define SHA256_ROTL(bits,word)  (((word) << (bits)) | ((word) >> (32-(bits))))
#define SHA256_ROTR(bits,word)  (((word) >> (bits)) | ((word) << (32-(bits))))

/* Define the SHA SIGMA and sigma macros */
#define SHA256_SIGMA0(word)     (SHA256_ROTR( 2,word) ^ SHA256_ROTR(13,word) ^ SHA256_ROTR(22,word))
#define SHA256_SIGMA1(word)     (SHA256_ROTR( 6,word) ^ SHA256_ROTR(11,word) ^ SHA256_ROTR(25,word))
#define SHA256_sigma0(word)     (SHA256_ROTR( 7,word) ^ SHA256_ROTR(18,word) ^ SHA256_SHR( 3,word))
#define SHA256_sigma1(word)     (SHA256_ROTR(17,word) ^ SHA256_ROTR(19,word) ^ SHA256_SHR(10,word))

void SHA256_t::Reset() {
    DataSz = 0;
    BlockIndx  = 0;
    // Initial Hash Values: FIPS 180-3 section 5.3.3
    Hash[0] = 0x6A09E667;
    Hash[1] = 0xBB67AE85;
    Hash[2] = 0x3C6EF372;
    Hash[3] = 0xA54FF53A;
    Hash[4] = 0x510E527F;
    Hash[5] = 0x9B05688C;
    Hash[6] = 0x1F83D9AB;
    Hash[7] = 0x5BE0CD19;
}

/*
 * This helper function will process the next 512 bits of the message stored in
 * the Message_Block array.
 * Many of the variable names in this code, especially the
 * single character names, were used because those were the
 * names used in the Secure Hash Standard.
*/
void SHA256_t::ProcessBlock(uint8_t *ptr) {
    /* Constants defined in FIPS 180-3, section 4.2.2 */
    static const uint32_t K[64] = {
        0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b,
        0x59f111f1, 0x923f82a4, 0xab1c5ed5, 0xd807aa98, 0x12835b01,
        0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7,
        0xc19bf174, 0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc,
        0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da, 0x983e5152,
        0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147,
        0x06ca6351, 0x14292967, 0x27b70a85, 0x2e1b2138, 0x4d2c6dfc,
        0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
        0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819,
        0xd6990624, 0xf40e3585, 0x106aa070, 0x19a4c116, 0x1e376c08,
        0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f,
        0x682e6ff3, 0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208,
        0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2
    };
    int        t, t4;                   /* Loop counter */
    uint32_t   temp1, temp2;            /* Temporary word value */
    uint32_t   W[64];                   /* Word sequence */
    uint32_t   A, B, C, D, E, F, G, H;  /* Word buffers */

    // Initialize the first 16 words in the array W
    for(t = t4 = 0; t < 16; t++, t4 += 4)
        W[t] = (((uint32_t) ptr[t4]    ) << 24)
             | (((uint32_t) ptr[t4 + 1]) << 16)
             | (((uint32_t) ptr[t4 + 2]) << 8)
             | (((uint32_t) ptr[t4 + 3]));

    for(t = 16; t < 64; t++)
        W[t] = SHA256_sigma1(W[t - 2]) + W[t - 7] + SHA256_sigma0(W[t - 15]) + W[t - 16];

    A = Hash[0];
    B = Hash[1];
    C = Hash[2];
    D = Hash[3];
    E = Hash[4];
    F = Hash[5];
    G = Hash[6];
    H = Hash[7];

    for(t = 0; t < 64; t++) {
        temp1 = H + SHA256_SIGMA1(E) + SHA_Ch(E, F, G) + K[t] + W[t];
        temp2 = SHA256_SIGMA0(A) + SHA_Maj(A, B, C);
        H = G;
        G = F;
        F = E;
        E = D + temp1;
        D = C;
        C = B;
        B = A;
        A = temp1 + temp2;
    }

    Hash[0] += A;
    Hash[1] += B;
    Hash[2] += C;
    Hash[3] += D;
    Hash[4] += E;
    Hash[5] += F;
    Hash[6] += G;
    Hash[7] += H;

    BlockIndx = 0;
}

// This function accepts an array of octets as the next portion of the message
void SHA256_t::Update(uint8_t* pData, uint32_t Sz) {
    DataSz += Sz;
    // If there is data left in buff, concatenate it to process as new chunk
    if(BlockIndx != 0) {
        uint32_t BytesToPut = SHA256BlockSize - BlockIndx;
        memcpy(&Block[BlockIndx], pData, BytesToPut);
        pData += BytesToPut;
        Sz -= BytesToPut;
        ProcessBlock(Block); // Do not worry about BlockIndx. It will be set to zero in ProcessBlock.
    }
    // Process Data
    while (Sz  >= SHA256BlockSize) {
        ProcessBlock(pData);
        pData += SHA256BlockSize;
        Sz -= SHA256BlockSize;
    }
    // Save remaining data in buff, will be reused on next call or finalize
    memcpy(Block, pData, Sz);
    BlockIndx += Sz;
}

void SHA256_t::Finalize() {
    Block[BlockIndx++] = 0x80; // The first padding bit must be a '1'
    // Check if there is enough space to put 8 bytes of 64-bit length. If no - fill with zeros and process
    if(BlockIndx >= (SHA256BlockSize - 8)) {
        memset(&Block[BlockIndx], 0, (SHA256BlockSize - BlockIndx));
        ProcessBlock(Block);
    }
    // Fill the rest of the block with zeros
    memset(Block, 0, (SHA256BlockSize - 8));
    // Store the message length as the last 8 bytes
    uint64_t Length = DataSz * 8;
    Block[56] = (uint8_t) (Length >> 56);
    Block[57] = (uint8_t) (Length >> 48);
    Block[58] = (uint8_t) (Length >> 40);
    Block[59] = (uint8_t) (Length >> 32);
    Block[60] = (uint8_t) (Length >> 24);
    Block[61] = (uint8_t) (Length >> 16);
    Block[62] = (uint8_t) (Length >> 8);
    Block[63] = (uint8_t) (Length);
    ProcessBlock(Block);
}

void SHA256_t::Read2BinArr(uint8_t Message_Digest[SHA256HashSize]) {
    for(int i = 0; i < SHA256HashSize; ++i)
        Message_Digest[i] = (uint8_t) (Hash[i >> 2] >> 8 * (3 - (i & 0x03)));
}

// Hash array and return 32-byte binary array
void SHA256_t::Hash2BinArr(uint8_t *pData, uint32_t Sz, uint8_t* Hash) {
    Reset();
    Update(pData, Sz);
    Finalize();
    Read2BinArr(Hash);
}

void Sha256Hash2BinArr(uint8_t *pData, uint32_t Sz, uint8_t* Hash) {
    SHA256_t Sha;
    Sha.Hash2BinArr(pData, Sz, Hash);
}
