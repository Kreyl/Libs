/*
 * cc1101.cpp
 *
 *  Created on: Feb 12, 2013
 *      Author: g.kruglov
 */

#include "cc1101.h"
#include "uart.h"

#define CC_MAX_BAUDRATE_HZ  6500000

uint8_t cc1101_t::Init() {
    // ==== GPIO ====
#if defined STM32L1XX || defined STM32F4XX || defined STM32L4XX || defined STM32F2XX
    AlterFunc_t CC_AF;
    if(ISpi.PSpi == SPI1 or ISpi.PSpi == SPI2) CC_AF = AF5;
    else CC_AF = AF6;
#elif defined STM32F030 || defined STM32F0
#define CC_AF   AF0
#endif
    PinSetupOut      ((GPIO_TypeDef*)PGpio, Cs,   omPushPull);
    PinSetupAlterFunc((GPIO_TypeDef*)PGpio, Sck,  omPushPull, pudNone, CC_AF);
    PinSetupAlterFunc((GPIO_TypeDef*)PGpio, Miso, omPushPull, pudNone, CC_AF);
    PinSetupAlterFunc((GPIO_TypeDef*)PGpio, Mosi, omPushPull, pudNone, CC_AF);
    IGdo0.Init(ttFalling);
    CsHi();
    // ==== SPI ====
    // MSB first, master, ClkLowIdle, FirstEdge, Baudrate no more than 6.5MHz
    uint32_t div;
#if defined STM32L1XX || defined STM32F4XX || defined STM32L4XX || defined STM32F2XX
    if(ISpi.PSpi == SPI1) div = Clk.APB2FreqHz / CC_MAX_BAUDRATE_HZ;
    else div = Clk.APB1FreqHz / CC_MAX_BAUDRATE_HZ;
#elif defined STM32F030 || defined STM32F0
    div = Clk.APBFreqHz / CC_MAX_BAUDRATE_HZ;
#endif
    SpiClkDivider_t ClkDiv = sclkDiv2;
    if     (div > 128) ClkDiv = sclkDiv256;
    else if(div > 64) ClkDiv = sclkDiv128;
    else if(div > 32) ClkDiv = sclkDiv64;
    else if(div > 16) ClkDiv = sclkDiv32;
    else if(div > 8)  ClkDiv = sclkDiv16;
    else if(div > 4)  ClkDiv = sclkDiv8;
    else if(div > 2)  ClkDiv = sclkDiv4;

    ISpi.Setup(boMSB, cpolIdleLow, cphaFirstEdge, ClkDiv);
    ISpi.Enable();
    // ==== Init CC ====
    if(Reset() != retvOk) {
        ISpi.Disable();
        Printf("CC Rst Fail\r");
        return retvFail;
    }
    // Check if Write/Read ok
    if(WriteRegister(CC_PKTLEN, 7) != retvOk) {
        Printf("CC W Fail\r");
        return retvFail;
    }
    uint8_t b = 0;
    if(ReadRegister(CC_PKTLEN, &b) == retvOk) {
        if(b != 7) {
            Printf("CC R/W Fail; rpl=%u\r", b);
            return retvFail;
        }
    }
    else {
        Printf("CC R Fail\r");
        return retvFail;
    }
    // Proceed with init
    FlushRxFIFO();
    RfConfig();
    IGdo0.EnableIrq(IRQ_PRIO_HIGH);
    Printf("CC init ok\r");
    return retvOk;
}

#if 1 // ==== Setup CC with needed values ====
void cc1101_t::RfConfig() {
    WriteRegister(CC_FSCTRL1,  CC_FSCTRL1_VALUE);    // Frequency synthesizer control.
    WriteRegister(CC_FSCTRL0,  CC_FSCTRL0_VALUE);    // Frequency synthesizer control.
    WriteRegister(CC_FREQ2,    CC_FREQ2_VALUE);      // Frequency control word, high byte.
    WriteRegister(CC_FREQ1,    CC_FREQ1_VALUE);      // Frequency control word, middle byte.
    WriteRegister(CC_FREQ0,    CC_FREQ0_VALUE);      // Frequency control word, low byte.
    WriteRegister(CC_MDMCFG4,  CC_MDMCFG4_VALUE);    // Modem configuration.
    WriteRegister(CC_MDMCFG3,  CC_MDMCFG3_VALUE);    // Modem configuration.
    WriteRegister(CC_MDMCFG2,  CC_MDMCFG2_VALUE);    // Modem configuration.
    WriteRegister(CC_MDMCFG1,  CC_MDMCFG1_VALUE);    // Modem configuration.
    WriteRegister(CC_MDMCFG0,  CC_MDMCFG0_VALUE);    // Modem configuration.
    WriteRegister(CC_CHANNR,   CC_CHANNR_VALUE);     // Channel number.
    WriteRegister(CC_DEVIATN,  CC_DEVIATN_VALUE);    // Modem deviation setting (when FSK modulation is enabled).
    WriteRegister(CC_FREND1,   CC_FREND1_VALUE);     // Front end RX configuration.
    WriteRegister(CC_FREND0,   CC_FREND0_VALUE);     // Front end RX configuration.
    WriteRegister(CC_MCSM0,    CC_MCSM0_VALUE);      // Main Radio Control State Machine configuration.
    WriteRegister(CC_FOCCFG,   CC_FOCCFG_VALUE);     // Frequency Offset Compensation Configuration.
    WriteRegister(CC_BSCFG,    CC_BSCFG_VALUE);      // Bit synchronization Configuration.
    WriteRegister(CC_AGCCTRL2, CC_AGCCTRL2_VALUE);   // AGC control.
    WriteRegister(CC_AGCCTRL1, CC_AGCCTRL1_VALUE);   // AGC control.
    WriteRegister(CC_AGCCTRL0, CC_AGCCTRL0_VALUE);   // AGC control.
    WriteRegister(CC_FSCAL3,   CC_FSCAL3_VALUE);     // Frequency synthesizer calibration.
    WriteRegister(CC_FSCAL2,   CC_FSCAL2_VALUE);     // Frequency synthesizer calibration.
    WriteRegister(CC_FSCAL1,   CC_FSCAL1_VALUE);     // Frequency synthesizer calibration.
    WriteRegister(CC_FSCAL0,   CC_FSCAL0_VALUE);     // Frequency synthesizer calibration.
    WriteRegister(CC_TEST2,    CC_TEST2_VALUE);      // Various test settings.
    WriteRegister(CC_TEST1,    CC_TEST1_VALUE);      // Various test settings.
    WriteRegister(CC_TEST0,    CC_TEST0_VALUE);      // Various test settings.
    WriteRegister(CC_FIFOTHR,  CC_FIFOTHR_VALUE);    // fifo threshold
    WriteRegister(CC_IOCFG2,   CC_IOCFG2_VALUE);     // GDO2 output pin configuration.
    WriteRegister(CC_IOCFG0,   CC_IOCFG0_VALUE);     // GDO0 output pin configuration.
    WriteRegister(CC_PKTCTRL1, CC_PKTCTRL1_VALUE);   // Packet automation control.
    WriteRegister(CC_PKTCTRL0, CC_PKTCTRL0_VALUE);   // Packet automation control.

    WriteRegister(CC_PATABLE, CC_Pwr0dBm);

    WriteRegister(CC_MCSM2, CC_MCSM2_VALUE);
    WriteRegister(CC_MCSM1, CC_MCSM1_VALUE);
}
#endif

#if 1 // ======================= TX, RX, freq and power ========================
void cc1101_t::SetChannel(uint8_t AChannel) {
    while(IState != CC_STB_IDLE) EnterIdle();   // CC must be in IDLE mode
    WriteRegister(CC_CHANNR, AChannel);         // Now set channel
}

uint8_t cc1101_t::FlushRxFIFO() {
//    while(IState != CC_STB_IDLE) EnterIdle();
    return WriteStrobe(CC_SFRX);
}
uint8_t cc1101_t::FlushTxFIFO() {
//    while(!(IState == CC_STB_IDLE or IState == CC_STB_TX_UNDF)) EnterIdle();
    return WriteStrobe(CC_SFTX);
}

//void cc1101_t::WaitUntilChannelIsBusy() {
//    uint8_t b;
//    for(uint32_t i=0; i<207; i++) {
//        b = ReadRegister(CC_PKTSTATUS);
//        if(b & 0b00010000) break;
//        //Uart.Printf("C");
//    }
//    //Uart.Printf("\r");
//}

void cc1101_t::Transmit(void *Ptr, uint8_t Len) {
    ICallback = nullptr;
//     WaitUntilChannelIsBusy();   // If this is not done, time after time FIFO is destroyed

    if(Len < 64) {
        chSysLock();
        WriteTX((uint8_t*)Ptr, Len);
        EnterTX();
        chThdSuspendS(&ThdRef); // Wait IRQ
        chSysUnlock();          // Will be here when IRQ fires
    }
    else {
        uint8_t *p = (uint8_t*)Ptr;
        uint8_t BytesToWrite = 63;
        bool FirstTime = true;
        while(true) {
//            Printf("btr %u\r", BytesToWrite);
            WriteTX(p, BytesToWrite);
            Len -= BytesToWrite;
            if(Len == 0) break;
            if(FirstTime) { // Change IO purpose once
                EnterTX();
                WriteRegister(CC_IOCFG0, 0x02); // Asserts when the TX FIFO is filled at or above the TX FIFO threshold. De-asserts when the TX FIFO is below the same threshold.
                FirstTime = false;
            }
            p += BytesToWrite;
            BytesToWrite = MIN_(Len, 30);
            // Wait until FIFO below threshold
            chSysLock();
            chThdSuspendS(&ThdRef); // Wait IRQ: FIFO is ready for 30 bytes more
            chSysUnlock();          // Will be here when IRQ fires
        }
        // All data sent to FIFO
        WriteRegister(CC_IOCFG0, CC_IOCFG0_VALUE); // Write IO 0 back
        // Wait end of packet
        chSysLock();
        chThdSuspendS(&ThdRef); // Wait IRQ
        chSysUnlock();          // Will be here when IRQ fires
//        Printf("end\r");
    }
}

uint8_t cc1101_t::TransmitWithCCA(void *Ptr, uint8_t Len, int16_t RssiThreshold) {
    EnterRX();
    chThdSleep(5); // Allow it to enter RX
    // Read RSSI
    uint8_t rawrssi;
    ReadRegister(CC_RSSI, &rawrssi);
    int16_t rssi = RSSI_dBm(rawrssi);
    if(rssi < RssiThreshold) {
        Transmit(Ptr, Len);
        return retvOk;
    }
    else {
//        Printf("raw: %03u; r: %03d\r", rawrssi, rssi);
        EnterIdle();
        return retvBusy;
    }
}

// Enter RX mode and wait reception for Timeout_ms.
uint8_t cc1101_t::Receive(uint32_t Timeout_ms, void *Ptr, uint8_t Len, int8_t *PRssi) {
    FlushRxFIFO();
    chSysLock();
    EnterRX();
    msg_t Rslt = chThdSuspendTimeoutS(&ThdRef, MS2ST(Timeout_ms));    // Wait IRQ
    chSysUnlock();  // Will be here when IRQ will fire, or timeout occur - with appropriate message

    if(Rslt == MSG_TIMEOUT) {   // Nothing received, timeout occured
        EnterIdle();            // Get out of RX mode
        return retvTimeout;
    }
    else return ReadFIFO(Ptr, PRssi, Len);
    return retvOk;
}

uint8_t cc1101_t::ReceiveLong(uint32_t Timeout_ms, void *Ptr, uint8_t *PLen, int8_t *PRssi) {
//    Printf("RL\r");
    FlushRxFIFO();

    chSysLock();
    IGdo0.SetTriggerType(ttRising); // IRQ when sync word received
    EnterRX();
    msg_t Rslt = chThdSuspendTimeoutS(&ThdRef, MS2ST(Timeout_ms));    // Wait IRQ
    chSysUnlock();  // Will be here when IRQ will fire, or timeout occur - with appropriate message

    if(Rslt == MSG_TIMEOUT) {   // Nothing received, timeout occured
        IGdo0.SetTriggerType(ttFalling);
        EnterIdle();            // Get out of RX mode
        return retvTimeout;
    }
    else {  // Sync word received, wait first bytes in FIFO
        WriteRegister(CC_IOCFG0, 0x00);

        chSysLock();
        Rslt = chThdSuspendTimeoutS(&ThdRef, MS2ST(Timeout_ms));    // Wait IRQ
        chSysUnlock();  // Will be here when IRQ will fire, or timeout occur - with appropriate message

        if(Rslt == MSG_TIMEOUT) {   // Nothing received, timeout occured
            IGdo0.SetTriggerType(ttFalling);
            WriteRegister(CC_IOCFG0, CC_IOCFG0_VALUE); // Write IO 0 back
            EnterIdle();            // Get out of RX mode
            return retvTimeout;
        }
        else { // There is something in FIFO
            uint8_t Len;
            ReadRegister(CC_RXBYTES, &Len);
            Len &= 0x7F;    // Remove MS bit
            // Read Len bytes from FIFO
            uint8_t *p = (uint8_t*)Ptr;
            CsLo();
            ISpi.ReadWriteByte(CC_FIFO|CC_READ_FLAG|CC_BURST_FLAG); // Address with read & burst flags
            for(uint8_t i=0; i<Len; i++) *p++ = ISpi.ReadWriteByte(0);
            CsHi();
            // Get Pkt length (first byte of what received)
            uint8_t TotalLen = *(uint8_t*)Ptr + 1; // +1 byte of Len at the beginning of pkt
//            Printf("Len: %u\r", TotalLen);
            TotalLen -= Len;
            if(TotalLen < 52) {
                WriteRegister(CC_IOCFG0, 0x06);
                IGdo0.SetTriggerType(ttFalling);
            }

            while(TotalLen != 0) {
                // Wait next bytes in FIFO
                chSysLock();
                Rslt = chThdSuspendTimeoutS(&ThdRef, MS2ST(Timeout_ms));    // Wait IRQ
                chSysUnlock();  // Will be here when IRQ will fire, or timeout occur - with appropriate message
                ReadRegister(CC_RXBYTES, &Len);
                Len &= 0x7F;    // Remove MS bit
                if(Len == 0) break;
                TRIM_VALUE(Len, TotalLen);
                CsLo();
                ISpi.ReadWriteByte(CC_FIFO|CC_READ_FLAG|CC_BURST_FLAG); // Address with read & burst flags
                for(uint8_t i=0; i<Len; i++) *p++ = ISpi.ReadWriteByte(0);
                CsHi();
                TotalLen -= Len;
                if(TotalLen < 52) {
                    WriteRegister(CC_IOCFG0, 0x06);
                    IGdo0.SetTriggerType(ttFalling);
                }
            }
            uint8_t b;
            // Read two extra bytes of RSSI and LQI
            if(PRssi != nullptr) {
                ReadRegister(CC_FIFO, &b);
                *PRssi = RSSI_dBm(b);
            }
            IGdo0.SetTriggerType(ttFalling);
            WriteRegister(CC_IOCFG0, CC_IOCFG0_VALUE); // Write IO 0 back
            // Read pkt status
            if(ReadRegister(CC_PKTSTATUS, &b) != retvOk) return retvFail;
            if(b & 0x80) return retvOk;  // CRC OK
            else return retvFail;
        }
    }
}

// Return RSSI in dBm
int16_t cc1101_t::RSSI_dBm(uint8_t ARawRSSI) {
    int16_t RSSI = ARawRSSI;
    if (RSSI >= 128) RSSI -= 256;
    RSSI = (RSSI / 2) - 74;    // now it is in dBm
    return RSSI;
}

void cc1101_t::ReceiveAsync(ftVoidVoid Callback) {
    FlushRxFIFO();
    ICallback = Callback;
    EnterRX();
}
void cc1101_t::TransmitAsync(void *Ptr, uint8_t Len, ftVoidVoid Callback) {
    EnterTX(); // Start transmission of preamble
    WriteTX((uint8_t*)Ptr, Len);
    ICallback = Callback;
}
#endif

#if 1 // ======================== Registers & Strobes ==========================
uint8_t cc1101_t::ReadRegister (uint8_t ARegAddr, uint8_t *PData) {
    CsLo();                     // Start transmission
    if(BusyWait() != retvOk) {  // Wait for chip to become ready
        CsHi();
        return retvFail;
    }
    ISpi.ReadWriteByte(ARegAddr | CC_READ_FLAG);    // Transmit header byte
    *PData = ISpi.ReadWriteByte(0);                 // Read reply
    CsHi();                                         // End transmission
    return retvOk;
}
uint8_t cc1101_t::WriteRegister (uint8_t ARegAddr, uint8_t AData) {
    CsLo();                     // Start transmission
    if(BusyWait() != retvOk) {      // Wait for chip to become ready
        CsHi();
        return retvFail;
    }
    ISpi.ReadWriteByte(ARegAddr);   // Transmit header byte
    ISpi.ReadWriteByte(AData);      // Write data
    CsHi();                         // End transmission
    return retvOk;
}
uint8_t cc1101_t::WriteStrobe (uint8_t AStrobe) {
    CsLo();                     // Start transmission
    if(BusyWait() != retvOk) {  // Wait for chip to become ready
        CsHi();
        return retvFail;
    }
    IState = ISpi.ReadWriteByte(AStrobe);   // Write strobe
    CsHi();                                 // End transmission
    IState &= 0b01110000;                   // Mask needed bits
    return retvOk;
}

uint8_t cc1101_t::WriteTX(uint8_t* Ptr, uint8_t Length) {
    CsLo();                                                     // Start transmission
    if(BusyWait() != retvOk) { // Wait for chip to become ready
        CsHi();
        return retvFail;
    }
    ISpi.ReadWriteByte(CC_FIFO|CC_WRITE_FLAG|CC_BURST_FLAG);    // Address with write & burst flags
//    Printf("TX: ");
    for(uint8_t i=0; i<Length; i++) {
        uint8_t b = *Ptr++;
        ISpi.ReadWriteByte(b);  // Write bytes
//        Printf("%X ", b);
    }
    CsHi();    // End transmission
//    Printf("\r");
    return retvOk;
}

uint8_t cc1101_t::ReadFIFO(void *Ptr, int8_t *PRssi, uint8_t Len) {
    uint8_t b, *p = (uint8_t*)Ptr;
     // Check if received successfully
     if(ReadRegister(CC_PKTSTATUS, &b) != retvOk) return retvFail;
//     Printf("St: %X  ", b);
     if(b & 0x80) {  // CRC OK
         // Read FIFO
         CsLo();                    // Start transmission
         if(BusyWait() != retvOk) { // Wait for chip to become ready
             CsHi();
             return retvFail;
         }
         ISpi.ReadWriteByte(CC_FIFO|CC_READ_FLAG|CC_BURST_FLAG); // Address with read & burst flags
         for(uint8_t i=0; i<Len; i++) { // Read bytes
             b = ISpi.ReadWriteByte(0);
             *p++ = b;
//             Printf("%X ", b);
         }
         // Receive two additional info bytes
         b = ISpi.ReadWriteByte(0); // RSSI
         ISpi.ReadWriteByte(0);     // LQI
         CsHi();                    // End transmission
//         PrintfEOL();
         if(PRssi != nullptr) *PRssi = RSSI_dBm(b);
         return retvOk;
     }
     else return retvFail;
}
#endif
