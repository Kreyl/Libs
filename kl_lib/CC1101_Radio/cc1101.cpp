/*
 * cc1101.cpp
 *
 *  Created on: Feb 12, 2013
 *      Author: g.kruglov
 */

#include "cc1101.h"
#include "uart.h"

#define CC_MAX_BAUDRATE_HZ  6500000

extern cc1101_t CC;

void CCIrqHandler() { CC.IIrqHandler(); }

uint8_t cc1101_t::Init() {
    // ==== GPIO ====
#if defined STM32L1XX || defined STM32F4XX || defined STM32L4XX || defined STM32F2XX
    AlterFunc_t CC_AF;
    if(ISpi.PSpi == SPI1 or ISpi.PSpi == SPI2) CC_AF = AF5;
    else CC_AF = AF6;
#elif defined STM32F030 || defined STM32F0 ||defined STM32F1XX
#define CC_AF   AF0
#endif
    PinSetupOut      ((GPIO_TypeDef*)CSGpio, Cs,   omPushPull);
    PinSetupAlterFunc((GPIO_TypeDef*)SpiGpio, Sck,  omPushPull, pudNone, CC_AF);
    PinSetupAlterFunc((GPIO_TypeDef*)SpiGpio, Miso, omPushPull, pudNone, CC_AF);
    PinSetupAlterFunc((GPIO_TypeDef*)SpiGpio, Mosi, omPushPull, pudNone, CC_AF);
    IGdo0.Init(ttFalling);

    CsHi();
    // ==== SPI ====
    // MSB first, master, ClkLowIdle, FirstEdge, Baudrate no more than 6.5MHz
    ISpi.Setup(boMSB, cpolIdleLow, cphaFirstEdge, CC_MAX_BAUDRATE_HZ);
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
    // Common regs
    WriteRegister(CC_FREQ2,    CC_FREQ2_VALUE);      // Frequency control word, high byte.
    WriteRegister(CC_FREQ1,    CC_FREQ1_VALUE);      // Frequency control word, middle byte.
    WriteRegister(CC_FREQ0,    CC_FREQ0_VALUE);      // Frequency control word, low byte.
    WriteRegister(CC_MDMCFG1,  CC_MDMCFG1_VALUE);    // Modem configuration.
    WriteRegister(CC_MDMCFG0,  CC_MDMCFG0_VALUE);    // Modem configuration.
    WriteRegister(CC_MCSM0,    CC_MCSM0_VALUE);      // Main Radio Control State Machine configuration.
    WriteRegister(CC_FIFOTHR,  CC_FIFOTHR_VALUE);    // fifo threshold
    WriteRegister(CC_IOCFG2,   CC_IOCFG2_VALUE);     // GDO2 output pin configuration.
    WriteRegister(CC_IOCFG0,   CC_IOCFG0_VALUE);     // GDO0 output pin configuration.
    WriteRegister(CC_PKTCTRL1, CC_PKTCTRL1_VALUE);   // Packet automation control.
    WriteRegister(CC_PKTCTRL0, CC_PKTCTRL0_VALUE);   // Packet automation control.
    WriteRegister(CC_PATABLE, CC_Pwr0dBm);
    WriteRegister(CC_MCSM2, CC_MCSM2_VALUE);
    WriteRegister(CC_MCSM1, CC_MCSM1_VALUE);

    IGdo0.EnableIrq(IRQ_PRIO_HIGH);
    Printf("CC init ok\r");
    return retvOk;
}

#if 1 // ======================= TX, RX, freq and power ========================
void cc1101_t::PowerOff() {
    while(IState != CC_STB_IDLE) EnterIdle();
    EnterPwrDown();
}

void cc1101_t::PrintStateI() {
    GetStatus();
    PrintfI("0x%02X\r", IState);
}

void cc1101_t::SetChannel(uint8_t AChannel) {
    while(IState != CC_STB_IDLE) EnterIdle();   // CC must be in IDLE mode
    WriteRegister(CC_CHANNR, AChannel);         // Now set channel
}

void cc1101_t::SetBitrate(const CCRegValue_t* BRSetup) {
    while(IState != CC_STB_IDLE) EnterIdle();   // CC must be in IDLE mode
    for(int i=0; i<CC_BRSETUP_CNT; i++) {
        WriteRegister(BRSetup[i].Reg, BRSetup[i].Value);
    }
}

void cc1101_t::TransmitAsyncX(uint8_t *Ptr, uint8_t Len, ftVoidVoid Callback) {
    EnterTX();
#if CC_CCA_MODE != 0
    // if prev state == RX, ClearChannelCheck is applied.
    if(IState == CC_STB_RX) {
        DELAY_LOOP_34uS(); // Wait >30 us
        GetStatus();
        if(IState != CC_STB_TX) return; // TX not entered
    }
#endif
    ICallback = Callback;
    WriteTX(Ptr, Len);
}

void cc1101_t::TransmitCcaX(uint8_t *Ptr, uint8_t Len, ftVoidVoid Callback) {
    GetStatus();
    if(IState != CC_STB_RX) {
        EnterRX(); // if prev state == RX, ClearChannelCheck is applied.
        DELAY_LOOP_144uS(); // 100...150 uS for 250kBaud
    }
    EnterTX();
    DELAY_LOOP_34uS(); // Wait >30 us
    GetStatus();
    if(IState == CC_STB_TX) { // TX entered
        ICallback = Callback;
        WriteTX(Ptr, Len);
    }
    else CC.EnterIdle();
}

void cc1101_t::TransmitAsyncX(uint8_t *Ptr, uint8_t Len) {
    EnterTX();  // Start transmission of preamble while writing FIFO
    WriteTX(Ptr, Len);
}

void cc1101_t::Transmit(uint8_t *Ptr, uint8_t Len) {
    EnterTX();  // Start transmission of preamble while writing FIFO
    chSysLock();
    ICallback = nullptr;
    WriteTX(Ptr, Len);
    WriteTX((uint8_t*)Ptr, Len);
    // Enter TX and wait IRQ
    chThdSuspendS(&ThdRef); // Wait IRQ
    chSysUnlock();          // Will be here when IRQ fires
}

// Enter RX mode and wait reception for Timeout_ms.
uint8_t cc1101_t::Receive(uint32_t Timeout_ms, uint8_t *Ptr, uint8_t Len, int8_t *PRssi) {
    return Receive_st(TIME_MS2I(Timeout_ms), Ptr, Len, PRssi);
}

uint8_t cc1101_t::Receive_st(sysinterval_t Timeout_st, uint8_t *Ptr, uint8_t Len, int8_t *PRssi) {
    FlushRxFIFO();
    chSysLock();
    EnterRX();
    msg_t Rslt = chThdSuspendTimeoutS(&ThdRef, Timeout_st);    // Wait IRQ
    chSysUnlock();  // Will be here when IRQ will fire, or timeout occur - with appropriate message

    if(Rslt == MSG_TIMEOUT) {   // Nothing received, timeout occured
        EnterIdle();            // Get out of RX mode
        return retvTimeout;
    }
    else return ReadFIFO(Ptr, PRssi, Len);
}

void cc1101_t::ReceiveAsyncI(ftVoidVoid Callback) {
    GetStatus();
    if(IState != CC_STB_RX) { // Not in RX
        EnterIdle();
        FlushRxFIFO();
        EnterRX();
    }
    ICallback = Callback;
}

/*
void cc1101_t::ReceiveAsync(ftVoidVoid Callback) {
    chSysLock();
    GetStatus();
    if(IState != CC_STB_RX) { // Not in RX
        EnterIdle();
        FlushRxFIFO();
        EnterRX();
    }
    ICallback = Callback;
    chSysUnlock();
}

uint8_t cc1101_t::RxCcaTx_st(uint8_t *PtrTx, uint8_t Len,  int8_t *PRssi) {
    chSysLock();
    ICallback = nullptr;
    // Enter RX if not yet
    GetStatus();
//    PrintfI("S1: %X\r", IState);
    if(IState != CC_STB_RX) { // Not in RX
        EnterRX();
        chThdSleepS(TIME_US2I(117)); // 100...150 uS for 250kBaud
    }
    EnterTX();
    // Where we are?
    chThdSleepS(TIME_US2I(108));
    GetStatus();
//    PrintfI("S2: %X\r", IState);
    if(IState != CC_STB_TX) { // Tx entered
        WriteTX((uint8_t*)PtrTx, Len);
        chThdSuspendS(&ThdRef); // Wait IRQ
        chSysUnlock();          // Will be here when IRQ fires
        return retvOk;
    }
    else {
        chSysUnlock();
        return retvFail;
    }
}

uint8_t cc1101_t::RxIfNotYet_st(sysinterval_t RxTimeout_st, uint8_t *PtrRx, uint8_t Len,  int8_t *PRssi) {
    // Enter RX if not yet
    chSysLock();
    GetStatus();
//    PrintfI("S3: %X\r", IState);
    if(IState != CC_STB_RX) { // Not in RX
        EnterIdle();
        FlushRxFIFO();
        EnterRX();
    }
    msg_t Rslt = chThdSuspendTimeoutS(&ThdRef, RxTimeout_st); // Wait IRQ
    chSysUnlock();
    if(Rslt == MSG_TIMEOUT) return retvFail; // No IRQ occured
    else { // IRQ fired
        return ReadFIFO(PtrRx, PRssi, Len);
    }
}
*/
// Return RSSI in dBm
int8_t cc1101_t::RSSI_dBm(uint8_t ARawRSSI) {
    int16_t RSSI = ARawRSSI;
    if (RSSI >= 128) RSSI -= 256;
    RSSI = (RSSI / 2) - 74;    // now it is in dBm
    return RSSI;
}
#endif

#if 1 // ======================== Registers & Strobes ==========================
uint8_t cc1101_t::ReadRegister (uint8_t ARegAddr, uint8_t *PData) {
    CsLo();                     // Start transmission
    if(BusyWait() != retvOk) {  // Wait for chip to become ready
        CsHi();
        return retvFail;
    }
    IState = ISpi.ReadWriteByte(ARegAddr | CC_READ_FLAG) & 0b01110000; // Transmit header byte
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

uint8_t cc1101_t::ReadFIFO(uint8_t *p, int8_t *PRssi, uint8_t Len) {
    uint8_t b;
     // Check if received successfully
     if(ReadRegister(CC_PKTSTATUS, &b) != retvOk) return retvFail;
//     PrintfI("PktSt: %02X\r", b);
     if(b & 0x80) {  // CRC OK
         // Read FIFO
         CsLo();
         if(BusyWait() != retvOk) { // Wait for chip to become ready
             CsHi();
             return retvFail;
         }
         ISpi.ReadWriteByte(CC_FIFO|CC_READ_FLAG|CC_BURST_FLAG); // Address with read & burst flags
         for(uint8_t i=0; i<Len; i++) { // Read bytes
             b = ISpi.ReadWriteByte(0);
             *p++ = b;
             // Uart.Printf(" %X", b);
         }
         // Receive two additional info bytes
         b = ISpi.ReadWriteByte(0); // RSSI
         ISpi.ReadWriteByte(0);     // LQI
         CsHi();                    // End transmission
         if(PRssi != nullptr) *PRssi = RSSI_dBm(b);
         return retvOk;
     }
     else return retvFail;
}
#endif

void cc1101_t::IIrqHandler() {
//    PrintfI("i %X\r", ICallback);
    if(ICallback != nullptr) {
        ICallback();
//        ICallback = nullptr;
    }
    else chThdResumeI(&ThdRef, MSG_OK);  // NotNull check performed inside chThdResumeI
}
