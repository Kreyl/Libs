/*
 * usb_cdc.cpp
 *
 *  Created on: 2015
 *      Author: Kreyl
 */

#include "descriptors_msdcdc.h"
#include "usb_msdcdc.h"
#include "board.h"
#include "gd_lib_F405.h"
#include "mem_msd_glue.h"
#include "usb_dev.h"
#include "scsi.h"
#include "MsgQ.h"
#include "EvtMsgIDs.h"

UsbMsdCdc usb_msd_cdc;
UsbDev usb_hw{USB_FS};//, USB_EP_CNT};

static uint8_t sbyte;

#define CDC_OUT_BUF_SZ  EP_CDC_BULK_SZ

#if 1 // ============ Mass Storage constants, types, variables =================
// Enum for the Mass Storage class specific control requests that can be issued by the USB bus host
enum MS_ClassRequests_t {
    // Mass Storage class-specific request to retrieve the total number of Logical Units (drives) in the SCSI device.
    MS_REQ_GetMaxLUN = 0xFE,
    // Mass Storage class-specific request to reset the Mass Storage interface, ready for the next command.
    MS_REQ_MassStorageReset = 0xFF,
};

#pragma pack(push, 1)
// Mass Storage Class Command Block Wrapper
struct MS_CommandBlockWrapper_t {
    uint32_t Signature;         // Command block signature, must be MS_CBW_SIGNATURE to indicate a valid Command Block
    uint32_t Tag;               // Unique command ID value, to associate a command block wrapper with its command status wrapper
    uint32_t DataTransferLen;   // Length of the optional data portion of the issued command, in bytes
    uint8_t  Flags;             // Command block flags, indicating command data direction
    uint8_t  LUN;               // Logical Unit number this command is issued to
    uint8_t  SCSICmdLen;        // Length of the issued SCSI command within the SCSI command data array
    uint8_t  SCSICmdData[16];   // Issued SCSI command in the Command Block
};
#define MS_CMD_SZ   sizeof(MS_CommandBlockWrapper_t)

// Mass Storage Class Command Status Wrapper
struct MS_CommandStatusWrapper_t {
    uint32_t Signature;          // Status block signature, must be \ref MS_CSW_SIGNATURE to indicate a valid Command Status
    uint32_t Tag;                // Unique command ID value, to associate a command block wrapper with its command status wrapper
    uint32_t DataTransferResidue;// number of bytes of data not processed in the SCSI command
    uint8_t  Status;             // Status code of the issued command - a value from the MS_CommandStatusCodes_t enum
};
#pragma pack(pop)

void MSDStartReceiveHdrI();
void OnMSDDataOut(uint32_t sz);
void OnMSDDataIn();
static bool isay_is_ready = true;
static Thread *pmsd_thd = nullptr;
#endif

// CDC Reception buffers and methods
namespace CdcOutQ {
enum class TransferAction { SendEvt, WakeThd };
static TransferAction action = TransferAction::SendEvt;

static uint8_t buf1[CDC_OUT_BUF_SZ], buf2[CDC_OUT_BUF_SZ], *pbuf_w = buf1;
static Buf_t buf_to_parse;
static Thread *pwaiting_thd = nullptr;

// OUT transfer end callback
void OnTransferEnd(uint32_t sz) {
    Sys::LockFromIRQ();
    // Save what received
    buf_to_parse.ptr = pbuf_w;
    buf_to_parse.sz = sz;
    // Switch buffers and start new reception
    pbuf_w = (pbuf_w == buf1)? buf2 : buf1;
    usb_hw.StartReceiveI(EP_CDC_DATA, pbuf_w, CDC_OUT_BUF_SZ);
    // Take necessary action
    if(action == TransferAction::SendEvt) evt_q_main.SendNowOrExitI(EvtMsg(EvtId::UsbCdcDataRcvd));
    else Sys::WakeI(&pwaiting_thd, retv::Ok);
    Sys::UnlockFromIRQ();
}
} // namespace

// Transmission buffers: several buffers of EP_BULK_SZ
static BufQ_t<uint8_t, EP_CDC_BULK_SZ, USB_TXBUF_CNT> cdc_in_buf;

// IN transfer end callback
void CdcOnBulkInTransferEnd() {
    // Unlock the buffer just sent to allow it to be written again
    cdc_in_buf.UnlockBuf();
    // start tx if buf is not empty
    if(usb_hw.IsActive() and !cdc_in_buf.IsEmpty()) {
        BufType_t<uint8_t> buf = cdc_in_buf.GetAndLockBuf();
        Sys::LockFromIRQ();
        usb_hw.StartTransmitI(EP_CDC_DATA, buf.ptr, buf.sz);
        Sys::UnlockFromIRQ();
    }
}

#pragma region // ====================== CDC Line Coding related ========================
#define CDC_SET_LINE_CODING         0x20U
#define CDC_GET_LINE_CODING         0x21U
#define CDC_SET_CONTROL_LINE_STATE  0x22U

#define LC_STOP_1                   0U
#define LC_STOP_1P5                 1U
#define LC_STOP_2                   2U

#define LC_PARITY_NONE              0U
#define LC_PARITY_ODD               1U
#define LC_PARITY_EVEN              2U
#define LC_PARITY_MARK              3U
#define LC_PARITY_SPACE             4U

// Line Coding
static struct CDCLinecoding {
    uint32_t dwDTERate = 115200;
    uint8_t bCharFormat = LC_STOP_1;
    uint8_t bParityType = LC_PARITY_NONE;
    uint8_t bDataBits = 8;
} __attribute__((packed)) linecoding;
static const uint32_t kCdcLinecodingSz = sizeof(CDCLinecoding);
#pragma endregion

#pragma region // ====================== Endpoints config ===============================
// InMultiplier determines the space allocated for the TXFIFO as multiples of the packet size
// ==== EP1 ==== both IN and OUT
static const UsbDev::EpConfig ep_cfg_cdc_bulk = {
        .cb_out_transfer_end = CdcOutQ::OnTransferEnd,
        .cb_in_transfer_end = CdcOnBulkInTransferEnd,
        .type = UsbDev::EpType::Bulk,
        .out_pkt_sz_max = EP_CDC_BULK_SZ,
        .in_pkt_sz_max = EP_CDC_BULK_SZ,
        .in_multiplier = 2
};

// ==== EP2 ==== Interrupt, IN only. Actually not used.
static const UsbDev::EpConfig ep_cfg_cdc_interrupt = {
        .cb_out_transfer_end = nullptr,
        .cb_in_transfer_end = nullptr,
        .type = UsbDev::EpType::Interrupt,
        .out_pkt_sz_max = 0, // IN only
        .in_pkt_sz_max = EP_INTERRUPT_SZ,
        .in_multiplier = 1
};

// ==== EP3 ==== both IN and OUT
static const UsbDev::EpConfig ep_cfg_msd = {
        .cb_out_transfer_end = OnMSDDataOut,
        .cb_in_transfer_end  = OnMSDDataIn,
        .type = UsbDev::EpType::Bulk,
        .out_pkt_sz_max = EP_MSD_BULK_SZ,
        .in_pkt_sz_max = EP_MSD_BULK_SZ,
        .in_multiplier = 2
};
#pragma endregion

#pragma region // ============================ Events ===================================
// Setup request callback: process class-related requests
retv UsbDev::SetupReqHookCallback(uint8_t **ppbuf, uint32_t *psz, ftVoidVoid *callback_end_transfer) {
    retv r = retv::NotFound; // NotFound by default
    if(setup_pkt.type == kReqTypeClass) {
        // === CDC handler ===
        if(setup_pkt.wIndex == 1) {
            *callback_end_transfer = nullptr;
            switch(setup_pkt.bRequest) {
                case CDC_GET_LINE_CODING: // Send linecoding
                    r = retv::Ok;
                    *ppbuf = reinterpret_cast<uint8_t*>(&linecoding);
                    *psz = kCdcLinecodingSz;
                    break;
                case CDC_SET_LINE_CODING: // Receive data into linecoding
                    r = retv::Ok;
                    *ppbuf = reinterpret_cast<uint8_t*>(&linecoding);
                    *psz = kCdcLinecodingSz;
                    break;
                case CDC_SET_CONTROL_LINE_STATE: // Nothing to do, there are no control lines
                    r = retv::Ok;
                    *psz = 0; // Receive nothing
                    break;
                default: break;
            } // switch
        } // if windex == 1

        // === MSD handler ===
        else {
            // GetMaxLun
            if(setup_pkt.direction == kReqDirDev2Host and
               setup_pkt.bRequest == MS_REQ_GetMaxLUN and
               setup_pkt.wLength == 1) {
//                PrintfI("MS_REQ_GetMaxLUN\r");
                sbyte = 0;  // Maximum LUN ID
                r = retv::Ok;
                *ppbuf = &sbyte;
                *psz = 1;
            }
            // Reset
            else if(setup_pkt.direction == kReqDirHost2Dev and
                setup_pkt.bRequest == MS_REQ_MassStorageReset and
                setup_pkt.wLength == 0) {
//                PrintfI("MS_REQ_MassStorageReset\r");
                // TODO: remove Stall condition
                r = retv::Ok; // Acknowledge reception
            }
        } // if MSD
    } // if class
    return r;
}

// this callback is invoked from an ISR so I-Class functions must be used
void UsbDev::EventCallback(Evt event) {
    switch(event) {
        case Evt::Reset:
            return;
        case Evt::Address:
            return;
        case Evt::Configured:
            Sys::LockFromIRQ();
            // ==== CDC ====
            InitEp(EP_CDC_DATA,      &ep_cfg_cdc_bulk);
            InitEp(EP_CDC_INTERRUPT, &ep_cfg_cdc_interrupt);
            // Reset queues
            CdcOutQ::pbuf_w = (CdcOutQ::pbuf_w == CdcOutQ::buf1)? CdcOutQ::buf2 : CdcOutQ::buf1;
            // start reception. In this case, transaction size is limited to EP max size
            StartReceiveI(EP_CDC_DATA, CdcOutQ::pbuf_w, CDC_OUT_BUF_SZ);
            // ==== MSD ====
            InitEp(EP_MSD_DATA, &ep_cfg_msd);
            MSDStartReceiveHdrI();
            isay_is_ready = true;
            // ==== Sys ====
            evt_q_main.SendNowOrExitI(EvtMsg(EvtId::UsbReady)); // Inform main thread
            Sys::UnlockFromIRQ();
            return;
        case Evt::Suspend:
        case Evt::Wakeup:
        case Evt::Stalled:
        case Evt::Unconfigured:
            return;
    } // switch
}
#pragma endregion // Events

#pragma region // =========================== CDC =======================================
retv UsbMsdCdc::IPutCharI(char c) {
    if(!usb_hw.IsActive()) return retv::Disconnected;
    retv r = cdc_in_buf.Put(c);
    if(cdc_in_buf.IsFullBufPresent() and !usb_hw.IsEpTransmitting(EP_CDC_DATA)) { // New buffer is full
        BufType_t<uint8_t> buf = cdc_in_buf.GetAndLockBuf();
        usb_hw.StartTransmitI(EP_CDC_DATA, buf.ptr, buf.sz);
    }
    return r;
}

void UsbMsdCdc::IStartTransmissionIfNotYetI() {
    // start tx if it has not already started and if buf is not empty.
    if(usb_hw.IsActive() and !usb_hw.IsEpTransmitting(EP_CDC_DATA) and !cdc_in_buf.IsEmpty()) {
        BufType_t<uint8_t> buf = cdc_in_buf.GetAndLockBuf();
        usb_hw.StartTransmitI(EP_CDC_DATA, buf.ptr, buf.sz);
    }
}

retv UsbMsdCdc::TryParseRxBuff() {
    while(CdcOutQ::buf_to_parse.sz) {
        CdcOutQ::buf_to_parse.sz--;
        if(cmd.PutChar(*CdcOutQ::buf_to_parse.ptr++) == pdrNewCmd) return retv::Ok;
    }
    return retv::Fail;
}

retv UsbMsdCdc::ReceiveFile(
        uint8_t *pbuf1, uint8_t *pbuf2,
        uint32_t buf_sz, uint32_t total_sz,
        uint32_t timeout_ms, ftRetvPU8U32 buf_end_callback) {
    return retv::Ok;
}

// Send '>' and receive what follows
// retv UsbMsdCdc::ReceiveBinaryToBuf(uint8_t *ptr, uint32_t len, uint32_t timeout_ms) {
//     CdcOutQ::action = CdcOutQ::TransferAction::WakeThd; // Do not send evt to main q on buf reception
//     if(IPutChar('>') != retv::Ok) return retv::Fail;
//     IStartTransmissionIfNotYet();
//     // Wait for data to be received
//     Sys::Lock();
//     systime_t start = Sys::GetSysTimeX();
//     systime_t time_left, timeout_st = TIME_MS2I(timeout_ms);
//     while(len != 0) {
//         // Calculate time left to wait
//         systime_t elapsed = Sys::TimeElapsedSince(start);
//         if(elapsed > timeout_st) break;
//         time_left = timeout_st - elapsed;
//         // Wait data
//         CdcOutQ::pwaiting_thd = Sys::GetSelfThdX();
//         if(Sys::SleepS(time_left) == retv::Timeout) break; // Timeout occured
//         // Will be here after successful reception; put data to buffer
//         if(CdcOutQ::buf_to_parse.sz > len) CdcOutQ::buf_to_parse.sz = len; // Flush too large data
//         memcpy(ptr, CdcOutQ::buf_to_parse.ptr, CdcOutQ::buf_to_parse.sz);
//         len -= CdcOutQ::buf_to_parse.sz;
//         ptr += CdcOutQ::buf_to_parse.sz;
//     }
//     CdcOutQ::action = CdcOutQ::TransferAction::SendEvt; // Return to normal life
//     Sys::Unlock();
//     return (len == 0)? retv::Ok : retv::Fail; // Check if everything was received
// }

// Wait '>' and then transmit buffer
retv UsbMsdCdc::TransmitBinaryFromBuf(uint8_t *ptr, uint32_t len, uint32_t timeout_ms) {
    if(usb_hw.IsEpTransmitting(EP_CDC_DATA)) return retv::Busy;
    retv r = retv::Timeout;
    Sys::Lock();
    CdcOutQ::action = CdcOutQ::TransferAction::WakeThd; // Do not send evt to main q on buf reception
    systime_t start = Sys::GetSysTimeX();
    systime_t time_left, timeout_st = TIME_MS2I(timeout_ms);
    // Wait '>'
    while(r == retv::Timeout) {
        // Calculate time left to wait
        systime_t elapsed = Sys::TimeElapsedSince(start);
        if(elapsed > timeout_st) break;
        time_left = timeout_st - elapsed;
        // Wait data
        CdcOutQ::pwaiting_thd = Sys::GetSelfThdX();
        if(Sys::SleepS(time_left) == retv::Timeout) break; // Timeout occured
        // Will be here after successful reception; check if '>' present
        for(uint32_t i=0; i<CdcOutQ::buf_to_parse.sz; i++) {
            if(CdcOutQ::buf_to_parse.ptr[i] == '>') {
                // Found
                r = retv::Ok;
                break;
            }
        } // for
    }
    // Will be here after either timeout or successful '>' reception
    CdcOutQ::action = CdcOutQ::TransferAction::SendEvt; // Return to normal life
    if(r == retv::Ok) { // Transmit data
        if(usb_hw.IsActive()) usb_hw.StartTransmitI(EP_CDC_DATA, ptr, len);
        else r = retv::Disconnected;
    }
    Sys::Unlock();
    return r;
}
#pragma endregion // CDC

#pragma region // ============================= MSD =====================================
//#define DBG_PRINT_CMD   TRUE

static MS_CommandBlockWrapper_t cmd_block;
static MS_CommandStatusWrapper_t cmd_status;
static SCSI_RequestSenseResponse sense_data;
static SCSI_ReadCapacity10Response read_capacity10_response;
static SCSI_ReadFormatCapacitiesResponse read_format_capacities_response;
static uint32_t buf32[(MSD_DATABUF_SZ/4)];

static void SCSICmdHandler();
// Scsi commands
static void CmdTestUnitReady();
static retv CmdStartStopUnit();
static retv CmdInquiry();
static retv CmdRequestSense();
static retv CmdReadCapacity10();
static retv CmdSendDiagnostic();
static retv CmdReadFormatCapacities();
static retv CmdRead10();
static retv CmdWrite10();
static retv CmdModeSense6();

// ==== Thread ====
static THD_WORKSPACE(wa_msd_thd, 256);
__attribute__((noreturn)) static void MsdThd() {
    while(true) {
        Sys::Lock();
        pmsd_thd = Sys::GetSelfThdX();
        retv r = Sys::SleepS(TIME_INFINITE); // Wait forever until new data is received
        Sys::Unlock();
        if(r == retv::Ok) SCSICmdHandler(); // New header received
        Sys::Lock();
        MSDStartReceiveHdrI();
        Sys::Unlock();
    }
}

// Receive header
void MSDStartReceiveHdrI() {
    usb_hw.StartReceiveI(EP_MSD_DATA, (uint8_t*)&cmd_block, MS_CMD_SZ);
}

void OnMSDDataOut(uint32_t sz) {
    Sys::LockFromIRQ();
    if(pmsd_thd and pmsd_thd->state == ThdState::Sleeping) Sys::WakeI(&pmsd_thd, retv::Ok);
    Sys::UnlockFromIRQ();
}

void OnMSDDataIn() {
    Sys::LockFromIRQ();
    if(pmsd_thd and pmsd_thd->state == ThdState::Sleeping) Sys::WakeI(&pmsd_thd, retv::Ok);
    Sys::UnlockFromIRQ();
}

void TransmitBuf(uint32_t *ptr, uint32_t len) {
    Sys::Lock();
    pmsd_thd = Sys::GetSelfThdX();
    usb_hw.StartTransmitI(EP_MSD_DATA, (uint8_t*)ptr, len);
    Sys::SleepS(TIME_INFINITE); // Wait forever until data is transmitted
    Sys::Unlock();
}

retv ReceiveToBuf(uint32_t *ptr, uint32_t len) {
    Sys::Lock();
    pmsd_thd = Sys::GetSelfThdX();
    usb_hw.StartReceiveI(EP_MSD_DATA, (uint8_t*)ptr, len);
    retv r = Sys::SleepS(TIME_INFINITE); // Wait forever until data is received
    Sys::Unlock();
    return r;
}
#pragma endregion // MSD

void UsbMsdCdc::Init() {
    usb_hw.Init();
    // Variables
    sense_data.ResponseCode = 0x70;
    sense_data.AddSenseLen = 0x0A;
    // MSD Thread
    Sys::CreateThd(wa_msd_thd, sizeof(wa_msd_thd), NORMALPRIO, MsdThd);
}

void UsbMsdCdc::Reset() {
    // Wake thread if sleeping
    Sys::Lock();
    if(pmsd_thd and pmsd_thd->state == ThdState::Sleeping) Sys::WakeI(&pmsd_thd, retv::Reset);
    Sys::Unlock();
}

void UsbMsdCdc::Connect()    { usb_hw.Connect(); }
void UsbMsdCdc::Disconnect() { usb_hw.Disconnect(); }
bool UsbMsdCdc::IsActive()   { return usb_hw.IsActive(); }

#if 1 // =========================== SCSI ======================================
// #define DBG_PRINT_CMD   TRUE
void SCSICmdHandler() {
//    Printf("Sgn=%X; Tag=%X; len=%u; Flags=%X; LUN=%u; SLen=%u; SCmd=%A\r", cmd_block.Signature, cmd_block.Tag, cmd_block.DataTransferLen, cmd_block.Flags, cmd_block.LUN, cmd_block.SCSICmdLen, cmd_block.SCSICmdData, cmd_block.SCSICmdLen, ' ');
//    Printf("SCmd=%A\r", cmd_block.SCSICmdData, cmd_block.SCSICmdLen, ' ');
    retv cmd_rslt = retv::Fail;
    switch(cmd_block.SCSICmdData[0]) {
        case 0x00: CmdTestUnitReady(); return; // Will report inside
        case 0x03: cmd_rslt = CmdRequestSense(); break;
        case 0x12: cmd_rslt = CmdInquiry(); break;
        case 0x1A: cmd_rslt = CmdModeSense6(); break;
        case 0x1B: cmd_rslt = CmdStartStopUnit(); break;
        case 0x1D: cmd_rslt = CmdSendDiagnostic(); break;
        case 0x23: cmd_rslt = CmdReadFormatCapacities(); break;
        case 0x25: cmd_rslt = CmdReadCapacity10(); break;
        case 0x28: cmd_rslt = CmdRead10(); break;
        case 0x2A: cmd_rslt = CmdWrite10(); break;
        // These commands should just succeed, no handling required
        case 0x1E: // Prevent/Allow Medium Removal
        case 0x2F: // Verify10
        case 0x35: // SynchronizeCache10
            cmd_rslt = retv::Ok;
            cmd_block.DataTransferLen = 0;
            break;
        // Not implemented
        case 0x15: // ModeSelect6
        case 0x55: // ModeSelect10
        case 0x5A: // ModeSense10
        case 0xA0: // ReportLUNs
        default:
            Printf("MSCmd %X not supported\r", cmd_block.SCSICmdData[0]);
            // Update the SENSE key to reflect the invalid command
            sense_data.SenseKey = SCSI_SENSE_KEY_ILLEGAL_REQUEST;
            sense_data.AdditionalSenseCode = SCSI_ASENSE_INVALID_COMMAND;
            sense_data.AdditionalSenseQualifier = SCSI_ASENSEQ_NO_QUALIFIER;
            break;
    } // switch
    // Update Sense if command was successfully processed
    if(cmd_rslt == retv::Ok) {
        sense_data.SenseKey = SCSI_SENSE_KEY_GOOD;
        sense_data.AdditionalSenseCode = SCSI_ASENSE_NO_ADDITIONAL_INFORMATION;
        sense_data.AdditionalSenseQualifier = SCSI_ASENSEQ_NO_QUALIFIER;
    }

    // Send status
    cmd_status.Signature = MS_CSW_SIGNATURE;
    cmd_status.Tag = cmd_block.Tag;
    if(cmd_rslt == retv::Ok) {
        cmd_status.Status = SCSI_STATUS_OK;
        cmd_status.DataTransferResidue = cmd_block.DataTransferLen; // DataTransferLen decreased by cmd handler
    }
    else {
        cmd_status.Status = SCSI_STATUS_CHECK_CONDITION;
        cmd_status.DataTransferResidue = 0;    // 0 or requested length?
    }
    TransmitBuf((uint32_t*)&cmd_status, sizeof(MS_CommandStatusWrapper_t));
}

void CmdTestUnitReady() {
#if DBG_PRINT_CMD
    Printf("CmdTestReady (Rdy: %u)\r", isay_is_ready);
#endif
    cmd_block.DataTransferLen = 0;
    cmd_status.Signature = MS_CSW_SIGNATURE;
    cmd_status.Tag = cmd_block.Tag;
    cmd_status.DataTransferResidue = cmd_block.DataTransferLen;
    if(isay_is_ready) {
        cmd_status.Status = SCSI_STATUS_OK;
        sense_data.SenseKey = SCSI_SENSE_KEY_GOOD;
        sense_data.AdditionalSenseCode = SCSI_ASENSE_NO_ADDITIONAL_INFORMATION;
        sense_data.AdditionalSenseQualifier = SCSI_ASENSEQ_NO_QUALIFIER;
    }
    else {
        cmd_status.Status = SCSI_STATUS_CHECK_CONDITION;
        sense_data.SenseKey = SCSI_SENSE_KEY_NOT_READY;
        sense_data.AdditionalSenseCode = SCSI_ASENSE_MEDIUM_NOT_PRESENT;
        sense_data.AdditionalSenseQualifier = SCSI_ASENSEQ_NO_QUALIFIER;
    }
    TransmitBuf((uint32_t*)&cmd_status, sizeof(MS_CommandStatusWrapper_t));
}

retv CmdStartStopUnit() {
#if DBG_PRINT_CMD
    Printf("CmdStartStopUnit [4]=%02X\r", cmd_block.SCSICmdData[4]);
#endif
    if((cmd_block.SCSICmdData[4] & 0x03) == 0x02) {  // Eject
        isay_is_ready = false;
    }
    else if((cmd_block.SCSICmdData[4] & 0x03) == 0x03) {  // Load
        isay_is_ready = true;
    }
    return retv::Ok;
}

retv CmdInquiry() {
#if DBG_PRINT_CMD
    Printf("CmdInquiry %u\r", cmd_block.SCSICmdData[1] & 0x01);
#endif
    uint16_t requested_len =  Convert::BuildU16(cmd_block.SCSICmdData[4], cmd_block.SCSICmdData[3]);
    uint16_t bytes_to_transfer;
    if(cmd_block.SCSICmdData[1] & 0x01) { // Evpd is set
        bytes_to_transfer = MIN_(requested_len, PAGE0_INQUIRY_DATA_SZ);
        TransmitBuf((uint32_t*)&kPage00InquiryData, bytes_to_transfer);
    }
    else {
        // Transmit InquiryData
        bytes_to_transfer = MIN_(requested_len, sizeof(SCSI_InquiryResponse));
        TransmitBuf((uint32_t*)&kInquiryData, bytes_to_transfer);
    }
    // Succeed the command and update the bytes transferred counter
    cmd_block.DataTransferLen -= bytes_to_transfer;
    return retv::Ok;
}

retv CmdRequestSense() {
#if DBG_PRINT_CMD
    Printf("CmdRequestSense\r");
#endif
    uint16_t requested_len = cmd_block.SCSICmdData[4];
    uint16_t bytes_to_transfer = MIN_(requested_len, sizeof(sense_data));
    // Transmit SenceData
    TransmitBuf((uint32_t*)&sense_data, bytes_to_transfer);
    // Succeed the command and update the bytes transferred counter
    cmd_block.DataTransferLen -= bytes_to_transfer;
    return retv::Ok;
}

retv CmdReadCapacity10() {
#if DBG_PRINT_CMD
    Printf("CmdReadCapacity10\r");
#endif
    read_capacity10_response.LastBlockAddr = __REV(MsdMem::GetBlockCnt() - 1);
    read_capacity10_response.BlockSize = __REV(MsdMem::GetBlockSz());
    // Transmit SenceData
    TransmitBuf((uint32_t*)&read_capacity10_response, sizeof(read_capacity10_response));
    // Succeed the command and update the bytes transferred counter
    cmd_block.DataTransferLen -= sizeof(read_capacity10_response);
    return retv::Ok;
}

retv CmdSendDiagnostic() {
    Printf("CmdSendDiagnostic\r");
    return retv::CmdUnknown;
}

retv CmdReadFormatCapacities() {
#if DBG_PRINT_CMD
    Printf("CmdReadFormatCapacities\r");
#endif
    read_format_capacities_response.Length = 0x08;
    read_format_capacities_response.NumberOfBlocks = __REV(MsdMem::GetBlockCnt());
    // 01b Unformatted Media - Maximum formattable capacity for this cartridge
    // 10b Formatted Media - Current media capacity
    // 11b No Cartridge in Drive - Maximum formattable capacity
    read_format_capacities_response.DescCode = 0x02;
    read_format_capacities_response.BlockSize[0] = (uint8_t)(MsdMem::GetBlockSz() >> 16);
    read_format_capacities_response.BlockSize[1] = (uint8_t)(MsdMem::GetBlockSz() >> 8);
    read_format_capacities_response.BlockSize[2] = (uint8_t)(MsdMem::GetBlockSz());
    // Transmit Data
    TransmitBuf((uint32_t*)&read_format_capacities_response, sizeof(read_format_capacities_response));
    // Succeed the command and update the bytes transferred counter
    cmd_block.DataTransferLen -= sizeof(read_format_capacities_response);
    return retv::Ok;
}

struct AddrLen { uint32_t addr, len; };
using RetvValAL = RetvVal<AddrLen>;

RetvValAL PrepareAddrAndLen() {
    RetvValAL r;
    r->addr = Convert::BuildU132(cmd_block.SCSICmdData[5], cmd_block.SCSICmdData[4], cmd_block.SCSICmdData[3], cmd_block.SCSICmdData[2]);
    r->len  = Convert::BuildU16(cmd_block.SCSICmdData[8], cmd_block.SCSICmdData[7]);
    // Check block addr
    if((r->addr + r->len) > MsdMem::GetBlockCnt()) {
        Printf("Out Of Range: addr %u, len %u\r", r->addr, r->len);
        sense_data.SenseKey = SCSI_SENSE_KEY_ILLEGAL_REQUEST;
        sense_data.AdditionalSenseCode = SCSI_ASENSE_LOGICAL_BLOCK_ADDRESS_OUT_OF_RANGE;
        sense_data.AdditionalSenseQualifier = SCSI_ASENSEQ_NO_QUALIFIER;
        r.rslt = retv::Fail;
    }
    // Check cases 4, 5: (Hi != Dn); and 3, 11, 13: (Hn, Ho != Do)
    if(cmd_block.DataTransferLen != r->len * MsdMem::GetBlockSz()) {
        Printf("Wrong length\r");
        sense_data.SenseKey = SCSI_SENSE_KEY_ILLEGAL_REQUEST;
        sense_data.AdditionalSenseCode = SCSI_ASENSE_INVALID_COMMAND;
        sense_data.AdditionalSenseQualifier = SCSI_ASENSEQ_NO_QUALIFIER;
        r.rslt = retv::Fail;
    }
    return r; // Ok by default
}

retv CmdRead10() {
#if DBG_PRINT_CMD
    Printf("CmdRead10\r");
#endif
    RetvValAL al = PrepareAddrAndLen();
    if(al.NotOk()) return retv::Fail;
    // Send data
    uint32_t blocks_to_read, bytes_to_send; // Intermediate values
    while(al->len != 0) {
        blocks_to_read = MIN_(MSD_DATABUF_SZ / MsdMem::GetBlockSz(), al->len);
        bytes_to_send = blocks_to_read * MsdMem::GetBlockSz();
        if(MsdMem::Read(al->addr, (uint8_t*)buf32, blocks_to_read) == retv::Ok) {
            TransmitBuf(buf32, bytes_to_send);
            cmd_block.DataTransferLen -= bytes_to_send;
            al->len  -= blocks_to_read;
            al->addr += blocks_to_read;
        }
        else {
            Printf("Rd fail\r");
            // TODO: handle read error
            return retv::Fail;
        }
    } // while
    return retv::Ok;
}

retv CmdWrite10() {
#if DBG_PRINT_CMD
    Printf("CmdWrite10\r");
#endif
#if MSD_READ_ONLY
    sense_data.SenseKey = SCSI_SENSE_KEY_DATA_PROTECT;
    sense_data.AdditionalSenseCode = SCSI_ASENSE_WRITE_PROTECTED;
    sense_data.AdditionalSenseQualifier = SCSI_ASENSEQ_NO_QUALIFIER;
    return retvFail;
#else
    // Check case 8: Hi != Do
    if(cmd_block.Flags & 0x80) {
        sense_data.SenseKey = SCSI_SENSE_KEY_ILLEGAL_REQUEST;
        sense_data.AdditionalSenseCode = SCSI_ASENSE_INVALID_COMMAND;
        return retv::Fail;
    }
    // TODO: Check if ready
    if(false) {
        sense_data.SenseKey = SCSI_SENSE_KEY_NOT_READY;
        sense_data.AdditionalSenseCode = SCSI_ASENSE_MEDIUM_NOT_PRESENT;
        return retv::Fail;
    }
    // Get transaction size
    RetvValAL al = PrepareAddrAndLen();
    if(al.NotOk()) return retv::Fail;
//    Printf("Addr=%u; len=%u\r", BlockAddress, TotalBlocks);
    uint32_t blocks_to_write, bytes_to_receive;

    while(al->len != 0) {
        // Fill Buf
        bytes_to_receive = MIN_(MSD_DATABUF_SZ, al->len * MsdMem::GetBlockSz());
        blocks_to_write  = bytes_to_receive / MsdMem::GetBlockSz();
        if(ReceiveToBuf(buf32, bytes_to_receive) != retv::Ok) {
            Printf("Rcv fail\r");
            return retv::Fail;
        }
        // Write Buf to memory
        if(MsdMem::Write(al->addr, (uint8_t*)buf32, blocks_to_write).NotOk()) return retv::Fail;
        cmd_block.DataTransferLen -= bytes_to_receive;
        al->len -= blocks_to_write;
        al->addr += blocks_to_write;
    } // while
    return retv::Ok;
#endif
}

retv CmdModeSense6() {
#if DBG_PRINT_CMD
    Printf("CmdModeSense6\r");
#endif
    uint16_t requested_len = cmd_block.SCSICmdData[4];
    uint16_t bytes_to_transfer = MIN_(requested_len, MODE_SENSE6_DATA_SZ);
    TransmitBuf((uint32_t*)&kModeSense6Data, bytes_to_transfer);
    // Succeed the command and update the bytes transferred counter
    cmd_block.DataTransferLen -= bytes_to_transfer;
    return retv::Ok;
}
#endif

// =================== USB IRQ Handler ===================
extern "C"
void USB_IRQ_HANDLER() {
    Sys::IrqPrologue();
    usb_hw.ProcessIrq();
    Sys::IrqEpilogue();
}