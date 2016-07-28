/*
 * usb_cdc.cpp
 *
 *  Created on: 03 сент. 2015 г.
 *      Author: Kreyl
 */

#include <descriptors_msd.h>
#include <usb_msd.h>
#include "usb.h"
#include "main.h"
#include "board.h"
#include "kl_lib.h"
#include "mem_msd_glue.h"
#include "kl_usb_defins.h"

UsbMsd_t UsbMsd;

static uint8_t SByte;

static bool OnSetupPkt(USBDriver *usbp);
static void OnDataInCompleted(USBDriver *usbp, usbep_t ep);
static void OnDataOutCompleted(USBDriver *usbp, usbep_t ep);

#if 1 // ========================== Endpoints ==================================
// ==== EP1 ====
static USBInEndpointState ep1instate;
static USBOutEndpointState ep1outstate;

// EP1 initialization structure (both IN and OUT).
static const USBEndpointConfig ep1config = {
    USB_EP_MODE_TYPE_BULK,
    NULL,                   // setup_cb
    OnDataInCompleted,      // in_cb
    OnDataOutCompleted,     // out_cb
    64,                     // in_maxsize
    64,                     // out_maxsize
    &ep1instate,            // in_state
    &ep1outstate,           // out_state
    2,                      // in_multiplier: Determines the space allocated for the TXFIFO as multiples of the packet size
    NULL                    // setup_buf: Pointer to a buffer for setup packets. Set this field to NULL for non-control endpoints
};
#endif

#if 1 // ============================ Events ===================================
static void usb_event(USBDriver *usbp, usbevent_t event) {
    switch (event) {
        case USB_EVENT_RESET:
//            UsbMsd.IsReady = false;
            return;
        case USB_EVENT_ADDRESS:
            return;
        case USB_EVENT_CONFIGURED:
            chSysLockFromISR();
            /* Enable the endpoints specified in the configuration.
            Note, this callback is invoked from an ISR so I-Class functions must be used.*/
            usbInitEndpointI(usbp, EP_DATA_IN_ID,  &ep1config);
            usbInitEndpointI(usbp, EP_DATA_OUT_ID, &ep1config);
            App.SignalEvtI(EVTMSK_USB_READY);
//            UsbMsd.IsReady = true;
            chEvtSignalI(UsbMsd.PThread, EVTMSK_USB_READY);
            chSysUnlockFromISR();
            return;
        case USB_EVENT_SUSPEND:
//            UsbMsd.IsReady = false;
            return;
        case USB_EVENT_WAKEUP:
            return;
        case USB_EVENT_STALLED:
            return;
    } // switch
}

#endif

#if 1  // ==== USB driver configuration ====
const USBConfig UsbCfg = {
    usb_event,          // This callback is invoked when an USB driver event is registered
    GetDescriptor,      // Device GET_DESCRIPTOR request callback
    OnSetupPkt,         // This hook allows to be notified of standard requests or to handle non standard requests
    NULL                // Start Of Frame callback
};
#endif

/* ==== Setup Packet handler ====
 * true         Message handled internally.
 * false        Message not handled. */
bool OnSetupPkt(USBDriver *usbp) {
    SetupPkt_t *Setup = (SetupPkt_t*)usbp->setup;
//    Uart.PrintfI("%X %X %X %X %X\r", Setup->bmRequestType, Setup->bRequest, Setup->wValue, Setup->wIndex, Setup->wLength);
//    Uart.PrintfI("RT.Dir: %X; RT.Type: %X; RT.Rec: %X\r", Setup->ReqType.Direction, Setup->ReqType.Type, Setup->ReqType.Recipient);
    if(Setup->ReqType.Direction == DIR_DEV2HOST and
       Setup->ReqType.Type == TYPE_CLASS and
       Setup->ReqType.Recipient == RCPT_INTERFACE and
       Setup->bRequest == MS_REQ_GetMaxLUN and
       Setup->wLength == 1)
    {
//        Uart.PrintfI("MS_REQ_GetMaxLUN\r");
        SByte = 0;  // Maximum LUN ID
        usbSetupTransfer(usbp, &SByte, 1, NULL);
        return true;
    }

    if(Setup->ReqType.Direction == DIR_HOST2DEV and
       Setup->ReqType.Type == TYPE_CLASS and
       Setup->ReqType.Recipient == RCPT_INTERFACE and
       Setup->bRequest == MS_REQ_MassStorageReset and
       Setup->wLength == 0)
    {
//        Uart.PrintfI("MS_REQ_MassStorageReset\r");
        // TODO: remove Stall condition
        return true; // Acknowledge reception
    }

    return false;
}

void OnDataInCompleted(USBDriver *usbp, usbep_t ep) {
    chSysLockFromISR();
//    Uart.PrintfI("inDone\r");
    chEvtSignalI(UsbMsd.PThread, EVTMSK_USB_IN_DONE);
    chSysUnlockFromISR();
}

void OnDataOutCompleted(USBDriver *usbp, usbep_t ep) {
    chSysLockFromISR();
//    Uart.PrintfI("OutDone\r");
    chEvtSignalI(UsbMsd.PThread, EVTMSK_USB_OUT_DONE);
    chSysUnlockFromISR();
}


#if 1 // ========================== MSD Thread =================================
static THD_WORKING_AREA(waUsbThd, 512);
static THD_FUNCTION(UsbThd, arg) {
    chRegSetThreadName("Usb");
    UsbMsd.Task();
}

__attribute__((__noreturn__))
void UsbMsd_t::Task() {
    while(true) {
        uint32_t EvtMsk = chEvtWaitAny(ALL_EVENTS);
        if(EvtMsk & EVTMSK_USB_READY) {
            // Receive header
            chSysLock();
            usbStartReceiveI(&USBDrv, EP_DATA_OUT_ID, (uint8_t*)&CmdBlock, MS_CMD_SZ);
            chSysUnlock();
        }

        if(EvtMsk & EVTMSK_USB_OUT_DONE) {
            SCSICmdHandler();
            // Receive header again
            chSysLock();
            usbStartReceiveI(&USBDrv, EP_DATA_OUT_ID, (uint8_t*)&CmdBlock, MS_CMD_SZ);
            chSysUnlock();
        }
    }
}
#endif

void UsbMsd_t::Init() {
    PinSetupAnalog(GPIOA, 11);
    PinSetupAnalog(GPIOA, 12);
    // Variables
    SenseData.ResponseCode = 0x70;
    SenseData.AddSenseLen = 0x0A;
    // Thread
    PThread = chThdCreateStatic(waUsbThd, sizeof(waUsbThd), NORMALPRIO, (tfunc_t)UsbThd, NULL);
}

void UsbMsd_t::Reset() {
    // Wake thread if sleeping
    chSysLock();
    if(PThread->p_state == CH_STATE_SUSPENDED) chSchReadyI(PThread);
    chSysUnlock();
}

void UsbMsd_t::Connect() {
    usbInit();
    usbDisconnectBus(&USBDrv);
    chThdSleepMilliseconds(1500);
    usbStart(&USBDrv, &UsbCfg);
    usbConnectBus(&USBDrv);
}
void UsbMsd_t::Disconnect() {
    usbDisconnectBus(&USBDrv);
    usbStop(&USBDrv);
}

void UsbMsd_t::TransmitBuf(uint8_t *Ptr, uint32_t Len) {
    chSysLock();
    usbStartTransmitI(&USBDrv, EP_DATA_IN_ID, Ptr, Len);
    chSysUnlock();
    BusyWaitIN();
}

uint8_t UsbMsd_t::ReceiveToBuf(uint8_t *Ptr, uint32_t Len) {
    chSysLock();
    usbStartReceiveI(&USBDrv, EP_DATA_OUT_ID, Ptr, Len);
    chSysUnlock();
    return BusyWaitOUT();
}

void UsbMsd_t::BusyWaitIN() {
    chEvtWaitAny(EVTMSK_USB_IN_DONE);
}

uint8_t UsbMsd_t::BusyWaitOUT() {
    eventmask_t evt = chEvtWaitAnyTimeout(EVTMSK_USB_OUT_DONE, MS2ST(MSD_TIMEOUT_MS));
    return (evt == 0)? TIMEOUT : OK;
}

#if 1 // =========================== SCSI ======================================
void UsbMsd_t::SCSICmdHandler() {
//    Uart.Printf("Sgn=%X; Tag=%X; Len=%u; Flags=%X; LUN=%u; SLen=%u; SCmd=%A\r", CmdBlock.Signature, CmdBlock.Tag, CmdBlock.DataTransferLen, CmdBlock.Flags, CmdBlock.LUN, CmdBlock.SCSICmdLen, CmdBlock.SCSICmdData, CmdBlock.SCSICmdLen, ' ');
    uint8_t CmdRslt = FAILURE;
    switch(CmdBlock.SCSICmdData[0]) {
        case SCSI_CMD_INQUIRY:            CmdRslt = CmdInquiry(); break;
        case SCSI_CMD_REQUEST_SENSE:      CmdRslt = CmdRequestSense(); break;
        case SCSI_CMD_READ_CAPACITY_10:   CmdRslt = CmdReadCapacity10(); break;
        case SCSI_CMD_SEND_DIAGNOSTIC:    CmdRslt = CmdSendDiagnostic(); break;
        case SCSI_READ_FORMAT_CAPACITIES: CmdRslt = CmdReadFormatCapacities(); break;
        case SCSI_CMD_WRITE_10:           CmdRslt = CmdWrite10(); break;
        case SCSI_CMD_READ_10:            CmdRslt = CmdRead10(); break;
        case SCSI_CMD_MODE_SENSE_6:       CmdRslt = CmdModeSense6(); break;
        // These commands should just succeed, no handling required
        case SCSI_CMD_TEST_UNIT_READY:
        case SCSI_CMD_PREVENT_ALLOW_MEDIUM_REMOVAL:
        case SCSI_CMD_VERIFY_10:
        case SCSI_CMD_START_STOP_UNIT:
        case SCSI_CMD_SYNCHRONIZE_CACHE_10:
            CmdRslt = OK;
            CmdBlock.DataTransferLen = 0;
            break;
        default:
            Uart.Printf("\rMSCmd %X not supported", CmdBlock.SCSICmdData[0]);
            // Update the SENSE key to reflect the invalid command
            SenseData.SenseKey = SCSI_SENSE_KEY_ILLEGAL_REQUEST;
            SenseData.AdditionalSenseCode = SCSI_ASENSE_INVALID_COMMAND;
            SenseData.AdditionalSenseQualifier = SCSI_ASENSEQ_NO_QUALIFIER;
            // Update status
            //CmdStatus.DataTransferResidue = 0;    // 0 or requested length?
            break;
    } // switch
    // Update Sense if command was successfully processed
    if(CmdRslt == OK) {
        SenseData.SenseKey = SCSI_SENSE_KEY_GOOD;
        SenseData.AdditionalSenseCode = SCSI_ASENSE_NO_ADDITIONAL_INFORMATION;
        SenseData.AdditionalSenseQualifier = SCSI_ASENSEQ_NO_QUALIFIER;
    }

    // Send status
    CmdStatus.Status = (CmdRslt == OK)? MS_SCSI_COMMAND_Pass : MS_SCSI_COMMAND_Fail;
    CmdStatus.Signature = MS_CSW_SIGNATURE;
    CmdStatus.Tag = CmdBlock.Tag;
    // DataTransferLen decreased at cmd handlers
    CmdStatus.DataTransferResidue = CmdBlock.DataTransferLen;
    // Stall if cmd failed and there is data to send
    bool ShouldSendStatus = true;
    if((CmdRslt != OK)) {
        chSysLock();
        ShouldSendStatus = !usbStallTransmitI(&USBDrv, EP_DATA_IN_ID);  // transmit status if successfully stalled
        chSysUnlock();
    }
    if(ShouldSendStatus) {
        TransmitBuf((uint8_t*)&CmdStatus, sizeof(MS_CommandStatusWrapper_t));
    }
}

uint8_t UsbMsd_t::CmdInquiry() {
//    Uart.Printf("CmdInquiry\r");
    uint16_t RequestedLength = Convert::BuildUint16(CmdBlock.SCSICmdData[4], CmdBlock.SCSICmdData[3]);
    uint16_t BytesToTransfer;
    if(CmdBlock.SCSICmdData[1] & 0x01) {
        BytesToTransfer = MIN(RequestedLength, PAGE0_INQUIRY_DATA_SZ);
        TransmitBuf((uint8_t*)&Page00InquiryData, BytesToTransfer);
    }
    else {
        // Transmit InquiryData
        BytesToTransfer = MIN(RequestedLength, sizeof(SCSI_InquiryResponse_t));
        TransmitBuf((uint8_t*)&InquiryData, BytesToTransfer);
    }
    // Succeed the command and update the bytes transferred counter
    CmdBlock.DataTransferLen -= BytesToTransfer;
    return OK;
}
uint8_t UsbMsd_t::CmdRequestSense() {
//    Uart.Printf("CmdRequestSense\r");
    uint16_t RequestedLength = CmdBlock.SCSICmdData[4];
    uint16_t BytesToTransfer = MIN(RequestedLength, sizeof(SenseData));
    // Transmit SenceData
    TransmitBuf((uint8_t*)&SenseData, BytesToTransfer);
    // Succeed the command and update the bytes transferred counter
    CmdBlock.DataTransferLen -= BytesToTransfer;
    return OK;
}
uint8_t UsbMsd_t::CmdReadCapacity10() {
//    Uart.Printf("CmdReadCapacity10\r");
    ReadCapacity10Response.LastBlockAddr = __REV((uint32_t)MSD_BLOCK_CNT - 1);
    ReadCapacity10Response.BlockSize = __REV((uint32_t)MSD_BLOCK_SZ);
    // Transmit SenceData
    TransmitBuf((uint8_t*)&ReadCapacity10Response, sizeof(ReadCapacity10Response));
    // Succeed the command and update the bytes transferred counter
    CmdBlock.DataTransferLen -= sizeof(ReadCapacity10Response);
    return OK;
}
uint8_t UsbMsd_t::CmdSendDiagnostic() {
    Uart.Printf("CmdSendDiagnostic\r");
    return CMD_UNKNOWN;
}
uint8_t UsbMsd_t::CmdReadFormatCapacities() {
//    Uart.Printf("CmdReadFormatCapacities\r");
    ReadFormatCapacitiesResponse.Length = 0x08;
    ReadFormatCapacitiesResponse.NumberOfBlocks = __REV(MSD_BLOCK_CNT);
    // 01b Unformatted Media - Maximum formattable capacity for this cartridge
    // 10b Formatted Media - Current media capacity
    // 11b No Cartridge in Drive - Maximum formattable capacity
    ReadFormatCapacitiesResponse.DescCode = 0x02;
    ReadFormatCapacitiesResponse.BlockSize[0] = (uint8_t)(MSD_BLOCK_SZ >> 16);
    ReadFormatCapacitiesResponse.BlockSize[1] = (uint8_t)(MSD_BLOCK_SZ >> 8);
    ReadFormatCapacitiesResponse.BlockSize[2] = (uint8_t)(MSD_BLOCK_SZ);
    // Transmit Data
    TransmitBuf((uint8_t*)&ReadFormatCapacitiesResponse, sizeof(ReadFormatCapacitiesResponse));
    // Succeed the command and update the bytes transferred counter
    CmdBlock.DataTransferLen -= sizeof(ReadFormatCapacitiesResponse);
    return OK;
}

uint8_t UsbMsd_t::ReadWriteCommon(uint32_t *PAddr, uint16_t *PLen) {
    *PAddr = Convert::BuildUint32(CmdBlock.SCSICmdData[5], CmdBlock.SCSICmdData[4], CmdBlock.SCSICmdData[3], CmdBlock.SCSICmdData[2]);
    *PLen  = Convert::BuildUint16(CmdBlock.SCSICmdData[8], CmdBlock.SCSICmdData[7]);
//    Uart.Printf("Addr=%u; Len=%u\r", BlockAddress, TotalBlocks);
    // Check block addr
    if((*PAddr + *PLen) > MSD_BLOCK_CNT) {
        Uart.Printf("Out Of Range: Addr %u, Len %u\r", *PAddr, *PLen);
        SenseData.SenseKey = SCSI_SENSE_KEY_ILLEGAL_REQUEST;
        SenseData.AdditionalSenseCode = SCSI_ASENSE_LOGICAL_BLOCK_ADDRESS_OUT_OF_RANGE;
        SenseData.AdditionalSenseQualifier = SCSI_ASENSEQ_NO_QUALIFIER;
        return FAILURE;
    }
    // Check cases 4, 5: (Hi != Dn); and 3, 11, 13: (Hn, Ho != Do)
    if(CmdBlock.DataTransferLen != (*PLen) * MSD_BLOCK_SZ) {
        Uart.Printf("Wrong length\r");
        SenseData.SenseKey = SCSI_SENSE_KEY_ILLEGAL_REQUEST;
        SenseData.AdditionalSenseCode = SCSI_ASENSE_INVALID_COMMAND;
        SenseData.AdditionalSenseQualifier = SCSI_ASENSEQ_NO_QUALIFIER;
        return FAILURE;
    }
    return OK;
}

uint8_t UsbMsd_t::CmdRead10() {
//    Uart.Printf("CmdRead10\r");
    uint32_t BlockAddress=0;
    uint16_t TotalBlocks=0;
    if(ReadWriteCommon(&BlockAddress, &TotalBlocks) != OK) return FAILURE;
    // ==== Send data ====
    uint32_t BlocksToRead, BytesToSend; // Intermediate values
    bool Rslt;
    while(TotalBlocks != 0) {
        BlocksToRead = MIN(MSD_DATABUF_SZ / MSD_BLOCK_SZ, TotalBlocks);
        BytesToSend = BlocksToRead * MSD_BLOCK_SZ;
        Rslt = MSDRead(BlockAddress, Buf, BlocksToRead);
//        Uart.Printf("%A\r", Buf, 50, ' ');
        if(Rslt == OK) {
            TransmitBuf(Buf, BytesToSend);
            CmdBlock.DataTransferLen -= BytesToSend;
            TotalBlocks  -= BlocksToRead;
            BlockAddress += BlocksToRead;
        }
        else {
            Uart.Printf("Rd fail\r");
            // TODO: handle read error
            return FAILURE;
        }
    } // while
    return OK;
}

uint8_t UsbMsd_t::CmdWrite10() {
//    Uart.Printf("CmdWrite10\r");
#if READ_ONLY
    SenseData.SenseKey = SCSI_SENSE_KEY_DATA_PROTECT;
    SenseData.AdditionalSenseCode = SCSI_ASENSE_WRITE_PROTECTED;
    SenseData.AdditionalSenseQualifier = SCSI_ASENSEQ_NO_QUALIFIER;
    return FAILURE;
#else
    // Check case 8: Hi != Do
    if(CmdBlock.Flags & 0x80) {
        SenseData.SenseKey = SCSI_SENSE_KEY_ILLEGAL_REQUEST;
        SenseData.AdditionalSenseCode = SCSI_ASENSE_INVALID_COMMAND;
        return FAILURE;
    }
    // TODO: Check if ready
    if(false) {
        SenseData.SenseKey = SCSI_SENSE_KEY_NOT_READY;
        SenseData.AdditionalSenseCode = SCSI_ASENSE_MEDIUM_NOT_PRESENT;
        return FAILURE;
    }
    uint32_t BlockAddress=0;
    uint16_t TotalBlocks=0;
    // Get transaction size
    if(ReadWriteCommon(&BlockAddress, &TotalBlocks) != OK) return FAILURE;
//    Uart.Printf("Addr=%u; Len=%u\r", BlockAddress, TotalBlocks);
    uint32_t BlocksToWrite, BytesToReceive;
    uint8_t Rslt = OK;

    while(TotalBlocks != 0) {
        // Fill Buf1
        BytesToReceive = MIN(MSD_DATABUF_SZ, TotalBlocks * MSD_BLOCK_SZ);
        BlocksToWrite  = BytesToReceive / MSD_BLOCK_SZ;
        if(ReceiveToBuf(Buf, BytesToReceive) != OK) {
            Uart.Printf("Rcv fail\r");
            return FAILURE;
        }
        // Write Buf to memory
        Rslt = MSDWrite(BlockAddress, Buf, BlocksToWrite);
        if(Rslt != OK) {
            Uart.Printf("Wr fail\r");
            return FAILURE;
        }
        CmdBlock.DataTransferLen -= BytesToReceive;
        TotalBlocks -= BlocksToWrite;
        BlockAddress += BlocksToWrite;
    } // while
    return OK;
#endif
}

uint8_t UsbMsd_t::CmdModeSense6() {
//    Uart.Printf("CmdModeSense6\r");
    TransmitBuf((uint8_t*)&Mode_Sense6_data, MODE_SENSE6_DATA_SZ);
    // Succeed the command and update the bytes transferred counter
    CmdBlock.DataTransferLen -= MODE_SENSE6_DATA_SZ;
    return OK;
}
#endif
