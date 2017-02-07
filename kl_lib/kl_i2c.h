#pragma once

#include "kl_lib.h"

#if defined STM32L1XX
struct i2cParams_t {
    I2C_TypeDef *pi2c;
    GPIO_TypeDef *PGpio;
    uint16_t SclPin;
    uint16_t SdaPin;
    AlterFunc_t PinAF;
    uint32_t BitrateHz;
    // DMA
    const stm32_dma_stream_t *PDmaTx;
    const stm32_dma_stream_t *PDmaRx;
};

/* Example:
 * i2c_t i2c (I2C_ACC, ACC_I2C_GPIO, ACC_I2C_SCL_PIN, ACC_I2C_SDA_PIN,
 * 400000, I2C_ACC_DMA_TX, I2C_ACC_DMA_RX );
 */

#define I2C_DMATX_MODE  STM32_DMA_CR_CHSEL(DmaChnl) |   \
                        DMA_PRIORITY_LOW | \
                        STM32_DMA_CR_MSIZE_BYTE | \
                        STM32_DMA_CR_PSIZE_BYTE | \
                        STM32_DMA_CR_MINC |     /* Memory pointer increase */ \
                        STM32_DMA_CR_DIR_M2P |  /* Direction is memory to peripheral */ \
                        STM32_DMA_CR_TCIE       /* Enable Transmission Complete IRQ */

#define I2C_DMARX_MODE  STM32_DMA_CR_CHSEL(DmaChnl) |   \
                        DMA_PRIORITY_LOW | \
                        STM32_DMA_CR_MSIZE_BYTE | \
                        STM32_DMA_CR_PSIZE_BYTE | \
                        STM32_DMA_CR_MINC |         /* Memory pointer increase */ \
                        STM32_DMA_CR_DIR_P2M |      /* Direction is peripheral to memory */ \
                        STM32_DMA_CR_TCIE           /* Enable Transmission Complete IRQ */

class i2c_t {
private:
    const i2cParams_t *PParams;
    void IReset();
    void SendStart()     { PParams->pi2c->CR1 |= I2C_CR1_START; }
    void SendStop()      { PParams->pi2c->CR1 |= I2C_CR1_STOP; }
    void AckEnable()     { PParams->pi2c->CR1 |= I2C_CR1_ACK; }
    void AckDisable()    { PParams->pi2c->CR1 &= ~I2C_CR1_ACK; }
    bool RxIsNotEmpty()  { return (PParams->pi2c->SR1 & I2C_SR1_RXNE); }
    void ClearAddrFlag() { (void)PParams->pi2c->SR1; (void)PParams->pi2c->SR2; }
    void SignalLastDmaTransfer() { PParams->pi2c->CR2 |= I2C_CR2_LAST; }
    // Address and data
    void SendAddrWithWrite(uint8_t Addr) { PParams->pi2c->DR = (uint8_t)(Addr<<1); }
    void SendAddrWithRead (uint8_t Addr) { PParams->pi2c->DR = ((uint8_t)(Addr<<1)) | 0x01; }
    void SendData(uint8_t b) { PParams->pi2c->DR = b; }
    uint8_t ReceiveData() { return PParams->pi2c->DR; }
    // Flags operations
    uint8_t IBusyWait();
    uint8_t WaitEv5();
    uint8_t WaitEv6();
    uint8_t WaitEv8();
    uint8_t WaitAck();
    uint8_t WaitRx();
    uint8_t WaitStop();
    uint8_t WaitBTF();
#if I2C_USE_SEMAPHORE
    binary_semaphore_t BSemaphore;
#endif
public:
    bool Error;
    thread_reference_t ThdRef;
    void Init();
    void ScanBus();
    void Standby();
    void Resume();
    uint8_t CheckAddress(uint32_t Addr);
    uint8_t Write     (uint8_t Addr, uint8_t *WPtr1, uint8_t WLength1);
    uint8_t WriteRead (uint8_t Addr, uint8_t *WPtr,  uint8_t WLength,  uint8_t *RPtr, uint8_t RLength);
    uint8_t WriteWrite(uint8_t Addr, uint8_t *WPtr1, uint8_t WLength1, uint8_t *WPtr2, uint8_t WLength2);
    i2c_t(const i2cParams_t *APParams) : PParams(APParams),
                Error(false), ThdRef(nullptr) {}
};

#if I2C1_ENABLED
extern i2c_t i2c1;
#endif

#endif // MCU type

#if defined STM32L476 || defined STM32F030
struct i2cParams_t {
    I2C_TypeDef *pi2c;
    GPIO_TypeDef *PGpio;
    uint16_t SclPin;
    uint16_t SdaPin;
    AlterFunc_t PinAF;
    uint32_t Timing;    // Setting for TIMINGR register
    // DMA
    const stm32_dma_stream_t *PDmaTx;
    const stm32_dma_stream_t *PDmaRx;
    uint32_t DmaModeTx, DmaModeRx;
    // IRQ
    uint32_t IrqEvtNumber, IrqErrorNumber;
};

#define I2C_TIMEOUT_MS      999

enum i2cState_t {istIdle, istWriteRead, istWriteWrite, istRead, istWrite, istFailure};

class i2c_t {
private:
    const i2cParams_t *PParams;
    uint8_t IBusyWait();
    void IReset();
    thread_reference_t PThd;
    i2cState_t IState;
    void IWakeup();
    uint8_t *IPtr;  // }
    uint32_t ILen;  // } required for WriteWrite method
#if I2C_USE_SEMAPHORE
    binary_semaphore_t BSemaphore;
#endif
public:
    i2c_t(const i2cParams_t *APParams) : PParams(APParams), PThd(nullptr), IState(istIdle), IPtr(nullptr), ILen(0) {}
    void Init();
    void ScanBus();
    void Standby();
    void Resume();
    uint8_t CheckAddress(uint32_t Addr);
    uint8_t Write     (uint32_t Addr, uint8_t *WPtr,  uint32_t WLength);
    uint8_t WriteRead (uint32_t Addr, uint8_t *WPtr,  uint32_t WLength, uint8_t *RPtr, uint32_t RLength);
    uint8_t WriteWrite(uint32_t Addr, uint8_t *WPtr1, uint32_t WLength1, uint8_t *WPtr2, uint32_t WLength2);
    // Inner use
    void IServeIRQ(uint32_t isr);
    void IServeErrIRQ(uint32_t isr);
};

extern i2c_t i2c1, i2c2, i2c3;
#endif
