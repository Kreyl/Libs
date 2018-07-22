/*
 * ws2812b.cpp
 *
 *  Created on: 05 апр. 2014 г.
 *      Author: Kreyl
 */

#include "ws2812b.h"

// Tx timings: bit cnt
#define SEQ_1               0b1110  // 0xE
#define SEQ_0               0b1000  // 0x8

#define SEQ_00              0x88
#define SEQ_01              0x8E
#define SEQ_10              0xE8
#define SEQ_11              0xEE

void NpxDmaDone(void *p, uint32_t flags) {
    ((Neopixels_t*)p)->OnDmaDone();
}

void Neopixels_t::Init() {
#if NPX_POWER_PIN_EN
    PwrPin.Init();
    PwrPin.SetLo(); // PwrOff
    chThdSleepMilliseconds(180);

#endif
    PinSetupAlterFunc(Params->PGpio, Params->Pin, omPushPull, pudNone, Params->Af);
    Params->ISpi.Setup(boMSB, cpolIdleLow, cphaFirstEdge, sclkDiv2, bitn16);
    Params->ISpi.Enable();
    Params->ISpi.EnableTxDma();

    Printf("Led BufSz=%u bytes\r", sizeof(IBuf));

    // Zero buffer
    for(int i=0; i<TOTAL_W_CNT; i++) IBuf[i] = 0;

    // ==== DMA ====
    dmaStreamAllocate     (Params->PDma, IRQ_PRIO_LOW, NpxDmaDone, (void*)this);
    dmaStreamSetPeripheral(Params->PDma, &Params->ISpi.PSpi->DR);
    dmaStreamSetMode      (Params->PDma, Params->DmaMode);
}

void Neopixels_t::AppendBitsMadeOfByte(uint8_t Byte) {
    uint8_t Bits, bMsb = 0, bLsb = 0;
    Bits = Byte & 0b11000000;
    if     (Bits == 0b00000000) bMsb = SEQ_00;
    else if(Bits == 0b01000000) bMsb = SEQ_01;
    else if(Bits == 0b10000000) bMsb = SEQ_10;
    else if(Bits == 0b11000000) bMsb = SEQ_11;

    Bits = Byte & 0b00110000;
    if     (Bits == 0b00000000) bLsb = SEQ_00;
    else if(Bits == 0b00010000) bLsb = SEQ_01;
    else if(Bits == 0b00100000) bLsb = SEQ_10;
    else if(Bits == 0b00110000) bLsb = SEQ_11;

    *PBuf++ = (bMsb << 8) | bLsb;

    Bits = Byte & 0b00001100;
    if     (Bits == 0b00000000) bMsb = SEQ_00;
    else if(Bits == 0b00000100) bMsb = SEQ_01;
    else if(Bits == 0b00001000) bMsb = SEQ_10;
    else if(Bits == 0b00001100) bMsb = SEQ_11;

    Bits = Byte & 0b00000011;
    if     (Bits == 0b00000000) bLsb = SEQ_00;
    else if(Bits == 0b00000001) bLsb = SEQ_01;
    else if(Bits == 0b00000010) bLsb = SEQ_10;
    else if(Bits == 0b00000011) bLsb = SEQ_11;

    *PBuf++ = (bMsb << 8) | bLsb;
}

void Neopixels_t::ISetCurrentColors() {
    PBuf = IBuf + (RST_W_CNT / 2);    // First words are zero to form reset
    // Fill bit buffer
#if NPX_POWER_PIN_EN
    IsAllBlack = true;
#endif
    for(uint32_t i=0; i<LED_CNT; i++) {
        AppendBitsMadeOfByte(ICurrentClr[i].G);
        AppendBitsMadeOfByte(ICurrentClr[i].R);
        AppendBitsMadeOfByte(ICurrentClr[i].B);
#if NPX_POWER_PIN_EN
        // Check if black
        if(ICurrentClr[i].R != 0 or ICurrentClr[i].G != 0 or ICurrentClr[i].B != 0) IsAllBlack = false;
#endif
    }
    if(!IsAllBlack) {
        PwrPin.SetHi(); // Power on
        chThdSleepMilliseconds(1);
    }
    // Start transmission
    dmaStreamDisable(Params->PDma);
    dmaStreamSetMemory0(Params->PDma, IBuf);
    dmaStreamSetTransactionSize(Params->PDma, TOTAL_W_CNT);
    dmaStreamSetMode(Params->PDma, Params->DmaMode);
    dmaStreamEnable(Params->PDma);
}

void Neopixels_t::OnDmaDone() {
#if NPX_POWER_PIN_EN
    if(IsAllBlack) PwrPin.SetLo(); // Pwr off
#endif
}
