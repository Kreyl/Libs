/*
 * soft_spi.h
 *
 *  Created on: 8 мая 2018 г.
 *      Author: Kreyl
 */

#pragma once

#include "kl_lib.h"

#define SOFT_SPI_DELAY()    { __NOP(); }

#define SOFT_SPI_SCK_IDLE_LOW   FALSE   // Change this
#define SOFT_SPI_FIRST_EDGE     FALSE   // Change this


class SoftSpi_t {
private:
    PinOutput_t ISck, IMosi, ICs;
    PinInput_t IMiso;
public:
    SoftSpi_t(
            GPIO_TypeDef *APGpioSck, uint16_t APinSck,
            GPIO_TypeDef *APGpioMiso, uint16_t APinMiso,
            GPIO_TypeDef *APGpioMosi, uint16_t APinMosi,
            GPIO_TypeDef *APGpioCs, uint16_t APinCs) :
                ISck(APGpioSck, APinSck, omPushPull),
                IMosi(APGpioMosi, APinMosi, omPushPull),
                ICs(APGpioCs, APinCs, omPushPull),
                IMiso(APGpioMiso, APinMiso, pudPullDown) {}
    void Init() const {
        ISck.Init();
        IMosi.Init();
        ICs.Init();
        IMiso.Init();
        ICs.SetHi();
#if SOFT_SPI_SCK_IDLE_LOW
        ISck.SetLo();
#else
        ISck.SetHi();
#endif
    }
    void SetCsHi() const { ICs.SetHi(); }
    void SetCsLo() const { ICs.SetLo(); }
    uint8_t WriteReadByte(uint8_t AByte) const {
        uint8_t b = 0;
#if SOFT_SPI_SCK_IDLE_LOW && SOFT_SPI_FIRST_EDGE
#error " Soft SPI mode not implemented"
#elif !SOFT_SPI_SCK_IDLE_LOW && SOFT_SPI_FIRST_EDGE
#error " Soft SPI mode not implemented"
#elif SOFT_SPI_SCK_IDLE_LOW && !SOFT_SPI_FIRST_EDGE
#error " Soft SPI mode not implemented"
#elif !SOFT_SPI_SCK_IDLE_LOW && !SOFT_SPI_FIRST_EDGE
        for(int i=0; i<8; i++) {
            ISck.SetLo();
            b <<= 1;
            // Put data on bus
            if(AByte & 0x80) IMosi.SetHi();
            else IMosi.SetLo();
            AByte <<= 1;
            ISck.SetHi();
            // Get data from bus
            if(IMiso.IsHi()) b |= 1;
        }
#endif
        return b;
    }
};
