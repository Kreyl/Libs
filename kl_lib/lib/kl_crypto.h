/*
 * kl_crypto.h
 *
 *  Created on: 28 июн. 2023 г.
 *      Author: laurelindo
 */

#ifndef KL_LIB_KL_CRYPTO_H_
#define KL_LIB_KL_CRYPTO_H_

#include <inttypes.h>

#define SHA256HashSize          32
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
    void Finalize();  // Produces final hash values (digest) to be read
    void Read2BinArr(uint8_t Message_Digest[SHA256HashSize]);
    // All in one
    void Hash2BinArr(uint8_t *pData, uint32_t Sz, uint8_t* Hash);
};

void Sha256Hash2BinArr(uint8_t *pData, uint32_t Sz, uint8_t* Hash);

#endif /* KL_LIB_KL_CRYPTO_H_ */
