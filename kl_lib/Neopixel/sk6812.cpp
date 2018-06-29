/*
 * ws2812b.cpp
 *
 *  Created on: 05 апр. 2014 г.
 *      Author: Kreyl
 */

#include "sk6812.h"

// Tx timings: bit cnt
/*
bit len @ 4MHz: 250
t0h 150...450       1
t0l 750...1050      3? 4
t1h 450...750       2
t1l 450...750       2
 */

#define SEQ_0               0b1000  // 0x8
#define SEQ_1               0b1100  // 0xC

#define SEQ_00              0x88
#define SEQ_01              0x8E
#define SEQ_10              0xE8
#define SEQ_11              0xEE

// Wrapper for Tx Completed IRQ
//extern "C"
//void LedTxcIrq(void *p, uint32_t flags) {
//    dmaStreamDisable(((Neopixels_t*)p)->Params->PDma);
//}

void Neopixels_t::Init() {
    PinSetupAlterFunc(Params->PGpio, Params->Pin, omPushPull, pudNone, Params->Af);
    Params->ISpi.Setup(boMSB, cpolIdleLow, cphaFirstEdge, sclkDiv4, bitn16);
    Params->ISpi.Enable();
    Params->ISpi.EnableTxDma();
//    Printf("Led BufSz=%u bytes\r", sizeof(IBuf));
    // Zero buffer
    for(uint32_t i=0; i<TOTAL_W_CNT; i++) IBuf[i] = 0;
    // Set colors to black
    for(uint32_t i=0; i<LED_CNT; i++) ICurrentClr[i] = clRGBWBlack;

    // ==== DMA ====
//    dmaStreamAllocate     (Params->PDma, IRQ_PRIO_LOW, LedTxcIrq, this);
    dmaStreamAllocate     (Params->PDma, IRQ_PRIO_LOW, nullptr, nullptr);
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
    for(uint32_t i=0; i<LED_CNT; i++) {
        AppendBitsMadeOfByte(ICurrentClr[i].G);
        AppendBitsMadeOfByte(ICurrentClr[i].R);
        AppendBitsMadeOfByte(ICurrentClr[i].B);
        AppendBitsMadeOfByte(ICurrentClr[i].W);
    }
    // Start transmission
    dmaStreamDisable(Params->PDma);
    dmaStreamSetMemory0(Params->PDma, IBuf);
    dmaStreamSetTransactionSize(Params->PDma, TOTAL_W_CNT);
    dmaStreamSetMode(Params->PDma, Params->DmaMode);
    dmaStreamEnable(Params->PDma);
}
