/*
 * usb_cdc.cpp
 *
 *  Created on: 03 сент. 2015 г.
 *      Author: Kreyl
 */

#include <descriptors_msdcdc.h>
#include <usb_msdcdc.h>
#include "hal_usb.h"
#include "board.h"
#include "kl_lib.h"
#include "mem_msd_glue.h"
#include "kl_usb_defins.h"
#include "MsgQ.h"
#include "EvtMsgIDs.h"
#include "hal_serial_usb.h"

UsbMsdCdc_t UsbMsdCdc;
SerialUSBDriver SDU1;

#define USBDrv          USBD1   // USB driver to use

#define EVT_USB_READY           EVENT_MASK(10)
#define EVT_USB_RESET           EVENT_MASK(11)
#define EVT_USB_SUSPEND         EVENT_MASK(12)
#define EVT_USB_CONNECTED       EVENT_MASK(13)
#define EVT_USB_DISCONNECTED    EVENT_MASK(14)
#define EVT_USB_IN_DONE         EVENT_MASK(15)
#define EVT_USB_OUT_DONE        EVENT_MASK(16)

static uint8_t SByte;
static bool ISayIsReady = true;
static thread_t *PMsdThd, *PCdcThd;

static bool OnSetupPkt(USBDriver *usbp);
static void OnMSDDataIn(USBDriver *usbp, usbep_t ep);
static void OnMSDDataOut(USBDriver *usbp, usbep_t ep);

#if 1 // ========================== Endpoints ==================================
// ==== EP1 ====
static USBInEndpointState ep1instate;
static USBOutEndpointState ep1outstate;

// EP1 initialization structure (both IN and OUT).
static const USBEndpointConfig ep1config = {
    USB_EP_MODE_TYPE_BULK,
    NULL,                   // setup_cb
    sduDataTransmitted,     // in_cb
    sduDataReceived,        // out_cb
    64,                     // in_maxsize
    64,                     // out_maxsize
    &ep1instate,            // in_state
    &ep1outstate,           // out_state
    2,                      // in_multiplier: Determines the space allocated for the TXFIFO as multiples of the packet size
    NULL                    // setup_buf: Pointer to a buffer for setup packets. Set this field to NULL for non-control endpoints
};

// ==== EP2 ====
static USBInEndpointState ep2instate;

// EP2 initialization structure (IN only).
static const USBEndpointConfig ep2config = {
    USB_EP_MODE_TYPE_INTR,
    NULL,
    sduInterruptTransmitted,
    NULL,
    16,
    0,
    &ep2instate,
    NULL,
    1,
    NULL
};

// ==== EP3 ====
static USBInEndpointState ep3instate;
static USBOutEndpointState ep3outstate;

// EP1 initialization structure (both IN and OUT).
static const USBEndpointConfig ep3config = {
    USB_EP_MODE_TYPE_BULK,
    NULL,                   // setup_cb
    OnMSDDataIn,            // in_cb
    OnMSDDataOut,           // out_cb
    64,                     // in_maxsize
    64,                     // out_maxsize
    &ep3instate,            // in_state
    &ep3outstate,           // out_state
    2,                      // in_multiplier: Determines the space allocated for the TXFIFO as multiples of the packet size
    NULL                    // setup_buf: Pointer to a buffer for setup packets. Set this field to NULL for non-control endpoints
};
#endif

#if 1 // ============================ Events ===================================
static void SOFHandler(USBDriver *usbp) {
  osalSysLockFromISR();
  sduSOFHookI(&SDU1);
  osalSysUnlockFromISR();
}

static void usb_event(USBDriver *usbp, usbevent_t event) {
    switch (event) {
        case USB_EVENT_RESET:
            PrintfI("UsbRst\r");
            return;
        case USB_EVENT_ADDRESS:
            PrintfI("UsbAddr\r");
            return;
        case USB_EVENT_CONFIGURED: {
            chSysLockFromISR();
            /* Enable the endpoints specified in the configuration.
            Note, this callback is invoked from an ISR so I-Class functions must be used.*/
            // CDC
            usbInitEndpointI(usbp, EP_CDC_DATA_IN,   &ep1config); // Single ep used for both IN and OUT
            usbInitEndpointI(usbp, EP_CDC_INTERRUPT, &ep2config);
            sduConfigureHookI(&SDU1);   // Resetting the state of the CDC subsystem
            // MSD
            usbInitEndpointI(usbp, EP_MSD_IN_ID,  &ep3config); // Single ep used for both IN and OUT

            ISayIsReady = true;
            EvtMsg_t Msg(evtIdUsbReady);
            EvtQMain.SendNowOrExitI(Msg);    // Signal to main thread
            chEvtSignalI(PMsdThd, EVT_USB_READY);
            chSysUnlockFromISR();
            return;
        } break;
        case USB_EVENT_SUSPEND:
        case USB_EVENT_WAKEUP:
        case USB_EVENT_STALLED:
        case USB_EVENT_UNCONFIGURED:
            return;
    } // switch
}

/* ==== Setup Packet handler ====
 * true         Message handled internally.
 * false        Message not handled. */
bool OnSetupPkt(USBDriver *usbp) {
    SetupPkt_t *Setup = (SetupPkt_t*)usbp->setup;
//    PrintfI("%X %X %X %X %X\r", Setup->bmRequestType, Setup->bRequest, Setup->wValue, Setup->wIndex, Setup->wLength);

    if(Setup->ReqType.Type == TYPE_CLASS) {
        // === CDC handler ===
        if(Setup->wIndex == 1) {
            if(sduRequestsHook(usbp) == true) return true;
        }
        // === MSD handler ===
        else {
            // GetMaxLun
            if(Setup->ReqType.Direction == DIR_DEV2HOST and
               Setup->bRequest == MS_REQ_GetMaxLUN and
               Setup->wLength == 1)
            {
//                PrintfI("MS_REQ_GetMaxLUN\r");
                SByte = 0;  // Maximum LUN ID
                usbSetupTransfer(usbp, &SByte, 1, NULL);
                return true;
            }
            // Reset
            if(Setup->ReqType.Direction == DIR_HOST2DEV and
               Setup->bRequest == MS_REQ_MassStorageReset and
               Setup->wLength == 0)
            {
//                PrintfI("MS_REQ_MassStorageReset\r");
                // TODO: remove Stall condition
                return true; // Acknowledge reception
            }
        } // if MSD
    } // if class
    return false;
}

void OnMSDDataIn(USBDriver *usbp, usbep_t ep) {
    chSysLockFromISR();
    chEvtSignalI(PMsdThd, EVT_USB_IN_DONE);
    chSysUnlockFromISR();
}

void OnMSDDataOut(USBDriver *usbp, usbep_t ep) {
    chSysLockFromISR();
    chEvtSignalI(PMsdThd, EVT_USB_OUT_DONE);
    chSysUnlockFromISR();
}

#endif

#if 1  // ==== USB driver configuration ====
const USBConfig UsbCfg = {
    usb_event,          // This callback is invoked when an USB driver event is registered
    GetDescriptor,      // Device GET_DESCRIPTOR request callback
    OnSetupPkt,         // This hook allows to be notified of standard requests or to handle non standard requests
    SOFHandler          // Start Of Frame callback
};

// Serial over USB driver configuration
const SerialUSBConfig SerUsbCfg = {
    &USBD1,             // USB driver to use
    EP_CDC_DATA_IN,     // Bulk IN endpoint used for outgoing data transfer
    EP_CDC_DATA_OUT,    // Bulk OUT endpoint used for incoming data transfer
    EP_CDC_INTERRUPT    // Interrupt IN endpoint used for notifications
};
#endif

#if 1 // ========================== MSD Thread =================================
static MS_CommandBlockWrapper_t CmdBlock;
static MS_CommandStatusWrapper_t CmdStatus;
static SCSI_RequestSenseResponse_t SenseData;
static SCSI_ReadCapacity10Response_t ReadCapacity10Response;
static SCSI_ReadFormatCapacitiesResponse_t ReadFormatCapacitiesResponse;
static uint8_t Buf[MSD_DATABUF_SZ];

static void SCSICmdHandler();
// Scsi commands
static void CmdTestReady();
static uint8_t CmdStartStopUnit();
static uint8_t CmdInquiry();
static uint8_t CmdRequestSense();
static uint8_t CmdReadCapacity10();
static uint8_t CmdSendDiagnostic();
static uint8_t CmdReadFormatCapacities();
static uint8_t CmdRead10();
static uint8_t CmdWrite10();
static uint8_t CmdModeSense6();
static uint8_t ReadWriteCommon(uint32_t *PAddr, uint16_t *PLen);
static void BusyWaitIN();
static uint8_t BusyWaitOUT();
static void TransmitBuf(uint8_t *Ptr, uint32_t Len);
static uint8_t ReceiveToBuf(uint8_t *Ptr, uint32_t Len);

static THD_WORKING_AREA(waUsbThd, 128);
static THD_FUNCTION(UsbThd, arg) {
    chRegSetThreadName("Usb");
    while(true) {
        uint32_t EvtMsk = chEvtWaitAny(ALL_EVENTS);
        if(EvtMsk & EVT_USB_READY) {
            // Receive header
            chSysLock();
            usbStartReceiveI(&USBDrv, EP_MSD_OUT_ID, (uint8_t*)&CmdBlock, MS_CMD_SZ);
            chSysUnlock();
        }

        if(EvtMsk & EVT_USB_OUT_DONE) {
            SCSICmdHandler();
            // Receive header again
            chSysLock();
            usbStartReceiveI(&USBDrv, EP_MSD_OUT_ID, (uint8_t*)&CmdBlock, MS_CMD_SZ);
            chSysUnlock();
        }
    } // while true
}

#endif

#if 1 // ========================== RX Thread ==================================
static inline bool IsCdcActive() { return (SDU1.config->usbp->state == USB_ACTIVE); }

static THD_WORKING_AREA(waThdCDCRX, 128);
static THD_FUNCTION(ThdCDCRX, arg) {
    chRegSetThreadName("CDCRX");
    while(true) {
        if(IsCdcActive()) {
            msg_t m = SDU1.vmt->get(&SDU1);
            if(m > 0) {
//                SDU1.vmt->put(&SDU1, (uint8_t)m);   // repeat what was sent
                if(UsbMsdCdc.Cmd.PutChar((char)m) == pdrNewCmd) {
                    chSysLock();
                    EvtQMain.SendNowOrExitI(EvtMsg_t(evtIdShellCmd, (Shell_t*)&UsbMsdCdc));
                    chSchGoSleepS(CH_STATE_SUSPENDED); // Wait until cmd processed
                    chSysUnlock();  // Will be here when application signals that cmd processed
                }
            } // if >0
        } // if active
        else chThdSleepMilliseconds(540);
    } // while true
}

uint8_t UsbMsdCdc_t::IPutChar(char c) {
    return (SDU1.vmt->put(&SDU1, (uint8_t)c) == MSG_OK)? retvOk : retvFail;
}

void UsbMsdCdc_t::SignalCmdProcessed() {
    chSysLock();
    if(PCdcThd->state == CH_STATE_SUSPENDED) chSchReadyI(PCdcThd);
    chSysUnlock();
}
#endif

void UsbMsdCdc_t::Init() {
#if defined STM32L4XX
    PinSetupAlterFunc(USB_DM, omPushPull, pudNone, USB_AF, psVeryHigh);
    PinSetupAlterFunc(USB_DP, omPushPull, pudNone, USB_AF, psVeryHigh);

    RCC->AHB2ENR |= RCC_AHB2ENR_OTGFSEN;
    RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;
    // Connect VddUSB
    PWR->CR2 |= PWR_CR2_USV;
#elif defined STM32F2XX
    PinSetupAlterFunc(USB_DM, omPushPull, pudNone, USB_AF, psHigh);
    PinSetupAlterFunc(USB_DP, omPushPull, pudNone, USB_AF, psHigh);
#else
    PinSetupAnalog(GPIOA, 11);
    PinSetupAnalog(GPIOA, 12);
#endif
    // Variables
    SenseData.ResponseCode = 0x70;
    SenseData.AddSenseLen = 0x0A;
    // MSD Thread
    PMsdThd = chThdCreateStatic(waUsbThd, sizeof(waUsbThd), NORMALPRIO, (tfunc_t)UsbThd, NULL);
    usbInit();
    sduObjectInit(&SDU1);
    sduStart(&SDU1, &SerUsbCfg);
    // CDC thread
    PCdcThd = chThdCreateStatic(waThdCDCRX, sizeof(waThdCDCRX), NORMALPRIO, ThdCDCRX, NULL);
}

void UsbMsdCdc_t::Reset() {
    // Wake thread if sleeping
    chSysLock();
    if(PMsdThd->state == CH_STATE_SUSPENDED) chSchReadyI(PMsdThd);
    chSysUnlock();
}

void UsbMsdCdc_t::Connect() {
    usbDisconnectBus(&USBDrv);
    chThdSleepMilliseconds(500);
    usbStart(&USBDrv, &UsbCfg);
    usbConnectBus(&USBDrv);
}
void UsbMsdCdc_t::Disconnect() {
    usbDisconnectBus(&USBDrv);
    usbStop(&USBDrv);
}

void TransmitBuf(uint8_t *Ptr, uint32_t Len) {
    chSysLock();
    usbStartTransmitI(&USBDrv, EP_MSD_IN_ID, Ptr, Len);
    chSysUnlock();
    BusyWaitIN();
}

uint8_t ReceiveToBuf(uint8_t *Ptr, uint32_t Len) {
    chSysLock();
    usbStartReceiveI(&USBDrv, EP_MSD_IN_ID, Ptr, Len);
    chSysUnlock();
    return BusyWaitOUT();
}

void BusyWaitIN() {
    chEvtWaitAny(ALL_EVENTS);
}

uint8_t BusyWaitOUT() {
    eventmask_t evt = chEvtWaitAnyTimeout(EVT_USB_OUT_DONE, MS2ST(MSD_TIMEOUT_MS));
    return (evt == 0)? retvTimeout : retvOk;
}

#if 1 // =========================== SCSI ======================================
//#define DBG_PRINT_CMD   TRUE
void SCSICmdHandler() {
//    Uart.Printf("Sgn=%X; Tag=%X; Len=%u; Flags=%X; LUN=%u; SLen=%u; SCmd=%A\r", CmdBlock.Signature, CmdBlock.Tag, CmdBlock.DataTransferLen, CmdBlock.Flags, CmdBlock.LUN, CmdBlock.SCSICmdLen, CmdBlock.SCSICmdData, CmdBlock.SCSICmdLen, ' ');
//    Printf("SCmd=%A\r", CmdBlock.SCSICmdData, CmdBlock.SCSICmdLen, ' ');
    uint8_t CmdRslt = retvFail;
    switch(CmdBlock.SCSICmdData[0]) {
        case SCSI_CMD_TEST_UNIT_READY:    CmdTestReady();     return; break;    // Will report itself
        case SCSI_CMD_START_STOP_UNIT:    CmdRslt = CmdStartStopUnit(); break;
        case SCSI_CMD_INQUIRY:            CmdRslt = CmdInquiry(); break;
        case SCSI_CMD_REQUEST_SENSE:      CmdRslt = CmdRequestSense(); break;
        case SCSI_CMD_READ_CAPACITY_10:   CmdRslt = CmdReadCapacity10(); break;
        case SCSI_CMD_SEND_DIAGNOSTIC:    CmdRslt = CmdSendDiagnostic(); break;
        case SCSI_READ_FORMAT_CAPACITIES: CmdRslt = CmdReadFormatCapacities(); break;
        case SCSI_CMD_WRITE_10:           CmdRslt = CmdWrite10(); break;
        case SCSI_CMD_READ_10:            CmdRslt = CmdRead10(); break;
        case SCSI_CMD_MODE_SENSE_6:       CmdRslt = CmdModeSense6(); break;
        // These commands should just succeed, no handling required
        case SCSI_CMD_PREVENT_ALLOW_MEDIUM_REMOVAL:
        case SCSI_CMD_VERIFY_10:
        case SCSI_CMD_SYNCHRONIZE_CACHE_10:
            CmdRslt = retvOk;
            CmdBlock.DataTransferLen = 0;
            break;
        default:
            Printf("MSCmd %X not supported\r", CmdBlock.SCSICmdData[0]);
            // Update the SENSE key to reflect the invalid command
            SenseData.SenseKey = SCSI_SENSE_KEY_ILLEGAL_REQUEST;
            SenseData.AdditionalSenseCode = SCSI_ASENSE_INVALID_COMMAND;
            SenseData.AdditionalSenseQualifier = SCSI_ASENSEQ_NO_QUALIFIER;
            break;
    } // switch
    // Update Sense if command was successfully processed
    if(CmdRslt == retvOk) {
        SenseData.SenseKey = SCSI_SENSE_KEY_GOOD;
        SenseData.AdditionalSenseCode = SCSI_ASENSE_NO_ADDITIONAL_INFORMATION;
        SenseData.AdditionalSenseQualifier = SCSI_ASENSEQ_NO_QUALIFIER;
    }

    // Send status
    CmdStatus.Signature = MS_CSW_SIGNATURE;
    CmdStatus.Tag = CmdBlock.Tag;
    if(CmdRslt == retvOk) {
        CmdStatus.Status = SCSI_STATUS_OK;
        // DataTransferLen decreased at cmd handlers
        CmdStatus.DataTransferResidue = CmdBlock.DataTransferLen;
    }
    else {
        CmdStatus.Status = SCSI_STATUS_CHECK_CONDITION;
        CmdStatus.DataTransferResidue = 0;    // 0 or requested length?
    }

    // Stall if cmd failed and there is data to send
//    bool ShouldSendStatus = true;
//    if((CmdRslt != retvOk)) {
//        chSysLock();
//        ShouldSendStatus = !usbStallTransmitI(&USBDrv, EP_DATA_IN_ID);  // transmit status if successfully stalled
//        chSysUnlock();
//    }
//    if(ShouldSendStatus) {
        TransmitBuf((uint8_t*)&CmdStatus, sizeof(MS_CommandStatusWrapper_t));
//    }
}

void CmdTestReady() {
#if DBG_PRINT_CMD
    Printf("CmdTestReady (Rdy: %u)\r", ISayIsReady);
#endif
    CmdBlock.DataTransferLen = 0;
    CmdStatus.Signature = MS_CSW_SIGNATURE;
    CmdStatus.Tag = CmdBlock.Tag;
    CmdStatus.DataTransferResidue = CmdBlock.DataTransferLen;

    if(ISayIsReady) {
        CmdStatus.Status = SCSI_STATUS_OK;
        SenseData.SenseKey = SCSI_SENSE_KEY_GOOD;
        SenseData.AdditionalSenseCode = SCSI_ASENSE_NO_ADDITIONAL_INFORMATION;
        SenseData.AdditionalSenseQualifier = SCSI_ASENSEQ_NO_QUALIFIER;
    }
    else {
        CmdStatus.Status = SCSI_STATUS_CHECK_CONDITION;
        SenseData.SenseKey = SCSI_SENSE_KEY_NOT_READY;
        SenseData.AdditionalSenseCode = SCSI_ASENSE_MEDIUM_NOT_PRESENT;
        SenseData.AdditionalSenseQualifier = SCSI_ASENSEQ_NO_QUALIFIER;
    }

    TransmitBuf((uint8_t*)&CmdStatus, sizeof(MS_CommandStatusWrapper_t));
}

uint8_t CmdStartStopUnit() {
#if DBG_PRINT_CMD
    Printf("CmdStartStopUnit [4]=%02X\r", CmdBlock.SCSICmdData[4]);
#endif
    if((CmdBlock.SCSICmdData[4] & 0x03) == 0x02) {  // Eject
        ISayIsReady = false;
    }
    else if((CmdBlock.SCSICmdData[4] & 0x03) == 0x03) {  // Load
        ISayIsReady = true;
    }
    return retvOk;
}

uint8_t CmdInquiry() {
#if DBG_PRINT_CMD
    Printf("CmdInquiry %u\r", CmdBlock.SCSICmdData[1] & 0x01);
#endif
    uint16_t RequestedLength = Convert::BuildUint16(CmdBlock.SCSICmdData[4], CmdBlock.SCSICmdData[3]);
    uint16_t BytesToTransfer;
    if(CmdBlock.SCSICmdData[1] & 0x01) { // Evpd is set
        BytesToTransfer = MIN_(RequestedLength, PAGE0_INQUIRY_DATA_SZ);
        TransmitBuf((uint8_t*)&Page00InquiryData, BytesToTransfer);
    }
    else {
        // Transmit InquiryData
        BytesToTransfer = MIN_(RequestedLength, sizeof(SCSI_InquiryResponse_t));
        TransmitBuf((uint8_t*)&InquiryData, BytesToTransfer);
    }
    // Succeed the command and update the bytes transferred counter
    CmdBlock.DataTransferLen -= BytesToTransfer;
    return retvOk;
}
uint8_t CmdRequestSense() {
#if DBG_PRINT_CMD
    Printf("CmdRequestSense\r");
#endif
    uint16_t RequestedLength = CmdBlock.SCSICmdData[4];
    uint16_t BytesToTransfer = MIN_(RequestedLength, sizeof(SenseData));
    // Transmit SenceData
    TransmitBuf((uint8_t*)&SenseData, BytesToTransfer);
    // Succeed the command and update the bytes transferred counter
    CmdBlock.DataTransferLen -= BytesToTransfer;
    return retvOk;
}
uint8_t CmdReadCapacity10() {
#if DBG_PRINT_CMD
    Printf("CmdReadCapacity10\r");
#endif
    ReadCapacity10Response.LastBlockAddr = __REV((uint32_t)MSD_BLOCK_CNT - 1);
    ReadCapacity10Response.BlockSize = __REV((uint32_t)MSD_BLOCK_SZ);
    // Transmit SenceData
    TransmitBuf((uint8_t*)&ReadCapacity10Response, sizeof(ReadCapacity10Response));
    // Succeed the command and update the bytes transferred counter
    CmdBlock.DataTransferLen -= sizeof(ReadCapacity10Response);
    return retvOk;
}
uint8_t CmdSendDiagnostic() {
    Printf("CmdSendDiagnostic\r");
    return retvCmdUnknown;
}
uint8_t CmdReadFormatCapacities() {
#if DBG_PRINT_CMD
    Printf("CmdReadFormatCapacities\r");
#endif
    ReadFormatCapacitiesResponse.Length = 0x08;
    ReadFormatCapacitiesResponse.NumberOfBlocks = __REV(MSD_BLOCK_CNT);
    // 01b Unformatted Media - Maximum formattable capacity for this cartridge
    // 10b Formatted Media - Current media capacity
    // 11b No Cartridge in Drive - Maximum formattable capacity
    ReadFormatCapacitiesResponse.DescCode = 0x02;
    ReadFormatCapacitiesResponse.BlockSize[0] = (uint8_t)((uint32_t)MSD_BLOCK_SZ >> 16);
    ReadFormatCapacitiesResponse.BlockSize[1] = (uint8_t)((uint32_t)MSD_BLOCK_SZ >> 8);
    ReadFormatCapacitiesResponse.BlockSize[2] = (uint8_t)((uint32_t)MSD_BLOCK_SZ);
    // Transmit Data
    TransmitBuf((uint8_t*)&ReadFormatCapacitiesResponse, sizeof(ReadFormatCapacitiesResponse));
    // Succeed the command and update the bytes transferred counter
    CmdBlock.DataTransferLen -= sizeof(ReadFormatCapacitiesResponse);
    return retvOk;
}

uint8_t ReadWriteCommon(uint32_t *PAddr, uint16_t *PLen) {
    *PAddr = Convert::BuildUint32(CmdBlock.SCSICmdData[5], CmdBlock.SCSICmdData[4], CmdBlock.SCSICmdData[3], CmdBlock.SCSICmdData[2]);
    *PLen  = Convert::BuildUint16(CmdBlock.SCSICmdData[8], CmdBlock.SCSICmdData[7]);
//    Uart.Printf("Addr=%u; Len=%u\r", *PAddr, *PLen);
    // Check block addr
    if((*PAddr + *PLen) > MSD_BLOCK_CNT) {
        Printf("Out Of Range: Addr %u, Len %u\r", *PAddr, *PLen);
        SenseData.SenseKey = SCSI_SENSE_KEY_ILLEGAL_REQUEST;
        SenseData.AdditionalSenseCode = SCSI_ASENSE_LOGICAL_BLOCK_ADDRESS_OUT_OF_RANGE;
        SenseData.AdditionalSenseQualifier = SCSI_ASENSEQ_NO_QUALIFIER;
        return retvFail;
    }
    // Check cases 4, 5: (Hi != Dn); and 3, 11, 13: (Hn, Ho != Do)
    if(CmdBlock.DataTransferLen != (*PLen) * MSD_BLOCK_SZ) {
        Printf("Wrong length\r");
        SenseData.SenseKey = SCSI_SENSE_KEY_ILLEGAL_REQUEST;
        SenseData.AdditionalSenseCode = SCSI_ASENSE_INVALID_COMMAND;
        SenseData.AdditionalSenseQualifier = SCSI_ASENSEQ_NO_QUALIFIER;
        return retvFail;
    }
    return retvOk;
}

uint8_t CmdRead10() {
#if DBG_PRINT_CMD
    Printf("CmdRead10\r");
#endif
    uint32_t BlockAddress=0;
    uint16_t TotalBlocks=0;
    if(ReadWriteCommon(&BlockAddress, &TotalBlocks) != retvOk) return retvFail;
    // ==== Send data ====
    uint32_t BlocksToRead, BytesToSend; // Intermediate values
    bool Rslt;
    while(TotalBlocks != 0) {
        BlocksToRead = MIN_(MSD_DATABUF_SZ / MSD_BLOCK_SZ, TotalBlocks);
        BytesToSend = BlocksToRead * MSD_BLOCK_SZ;
        Rslt = MSDRead(BlockAddress, Buf, BlocksToRead);
//        Uart.Printf("%A\r", Buf, 50, ' ');
        if(Rslt == retvOk) {
            TransmitBuf(Buf, BytesToSend);
            CmdBlock.DataTransferLen -= BytesToSend;
            TotalBlocks  -= BlocksToRead;
            BlockAddress += BlocksToRead;
        }
        else {
            Printf("Rd fail\r");
            // TODO: handle read error
            return retvFail;
        }
    } // while
    return retvOk;
}

uint8_t CmdWrite10() {
#if DBG_PRINT_CMD
    Printf("CmdWrite10\r");
#endif
#if READ_ONLY
    SenseData.SenseKey = SCSI_SENSE_KEY_DATA_PROTECT;
    SenseData.AdditionalSenseCode = SCSI_ASENSE_WRITE_PROTECTED;
    SenseData.AdditionalSenseQualifier = SCSI_ASENSEQ_NO_QUALIFIER;
    return retvFail;
#else
    // Check case 8: Hi != Do
    if(CmdBlock.Flags & 0x80) {
        SenseData.SenseKey = SCSI_SENSE_KEY_ILLEGAL_REQUEST;
        SenseData.AdditionalSenseCode = SCSI_ASENSE_INVALID_COMMAND;
        return retvFail;
    }
    // TODO: Check if ready
    if(false) {
        SenseData.SenseKey = SCSI_SENSE_KEY_NOT_READY;
        SenseData.AdditionalSenseCode = SCSI_ASENSE_MEDIUM_NOT_PRESENT;
        return retvFail;
    }
    uint32_t BlockAddress=0;
    uint16_t TotalBlocks=0;
    // Get transaction size
    if(ReadWriteCommon(&BlockAddress, &TotalBlocks) != retvOk) return retvFail;
//    Uart.Printf("Addr=%u; Len=%u\r", BlockAddress, TotalBlocks);
    uint32_t BlocksToWrite, BytesToReceive;
    uint8_t Rslt = retvOk;

    while(TotalBlocks != 0) {
        // Fill Buf1
        BytesToReceive = MIN_(MSD_DATABUF_SZ, TotalBlocks * MSD_BLOCK_SZ);
        BlocksToWrite  = BytesToReceive / MSD_BLOCK_SZ;
        if(ReceiveToBuf(Buf, BytesToReceive) != retvOk) {
            Printf("Rcv fail\r");
            return retvFail;
        }
        // Write Buf to memory
        Rslt = MSDWrite(BlockAddress, Buf, BlocksToWrite);
        if(Rslt != retvOk) {
            Printf("Wr fail\r");
            return retvFail;
        }
        CmdBlock.DataTransferLen -= BytesToReceive;
        TotalBlocks -= BlocksToWrite;
        BlockAddress += BlocksToWrite;
    } // while
    return retvOk;
#endif
}

uint8_t CmdModeSense6() {
#if DBG_PRINT_CMD
    Printf("CmdModeSense6\r");
#endif
    uint16_t RequestedLength = CmdBlock.SCSICmdData[4];
    uint16_t BytesToTransfer = MIN_(RequestedLength, MODE_SENSE6_DATA_SZ);
    TransmitBuf((uint8_t*)&Mode_Sense6_data, BytesToTransfer);
    // Succeed the command and update the bytes transferred counter
    CmdBlock.DataTransferLen -= BytesToTransfer;
    return retvOk;
}
#endif
