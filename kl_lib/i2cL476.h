/*
 * i2cL476.h
 *
 *  Created on: 2 мая 2016 г.
 *      Author: Kreyl
 */

#pragma once

#include "kl_lib.h"

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
#define I2C_USE_SEMAPHORE   TRUE

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
