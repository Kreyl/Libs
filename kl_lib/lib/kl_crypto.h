/*
 * kl_crypto.h
 *
 *  Created on: 28 июн. 2023 г.
 *      Author: laurelindo
 */

#ifndef KL_LIB_KL_CRYPTO_H_
#define KL_LIB_KL_CRYPTO_H_

#include <inttypes.h>

#define SHA256HashSize   32
#define SHA256BlockSize  64

class SHA256_t {
private:
    uint32_t Hash[SHA256HashSize/4];
    uint32_t DataSz; // Data size in bytes
    int_least16_t BlockIndx;
    uint8_t Block[SHA256BlockSize];
    void ProcessBlock(uint8_t *ptr);
public:
    // Step by step
    void Reset(); // Initialization, must be called before any further use
    void Update(uint8_t* pData, uint32_t Sz); // Process block of data of arbitary length
    void GetResult(uint8_t Digest[SHA256HashSize]);
    // All in one
    void DoHash(uint8_t *pData, uint32_t Sz, uint8_t* Hash);
};

// Single function
void Sha256DoHash(uint8_t *pData, uint32_t Sz, uint8_t* Hash);

/* In case of chunk-bychunk aproach, do next
 * Reset(); Update(chunk1); Update(chunk2); ... Update(chunkLast); GetResult(rslt);
 */
class HMAC_t {
private:
    uint8_t k_opad[SHA256BlockSize];
    SHA256_t ISha;
public:
    void Reset(uint8_t *key, uint32_t key_len);
    void Update(uint8_t *pData, uint32_t Sz);
    void GetResult(uint8_t Digest[SHA256HashSize]);
};

void HMAC_SHA256(uint8_t *pData, uint32_t Sz,
        uint8_t *pKey, uint32_t KeySz, uint8_t digest[SHA256HashSize]);

#endif /* KL_LIB_KL_CRYPTO_H_ */
