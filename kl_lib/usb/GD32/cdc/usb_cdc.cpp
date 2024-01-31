#include "usb_cdc.h"
#include "usb.h"
#include "descriptors_cdc.h"
#include "MsgQ.h"
#include "kl_buf.h"

#define OUT_BUF_SZ  EP_BULK_SZ

UsbCDC_t UsbCDC;

// Reception buffers and methods
namespace OutQ {
static enum class TransferAction { SendEvt, WakeThd } Action = TransferAction::SendEvt;

static uint8_t Buf1[OUT_BUF_SZ], Buf2[OUT_BUF_SZ], *pBufW = Buf1;
static Buf_t BufToParse;
static Thread_t *pWaitingThd = nullptr;

// OUT transfer end callback
void OnTransferEnd(uint32_t Sz) {
    Sys::LockFromIRQ();
    // Save what received
    BufToParse.Ptr = pBufW;
    BufToParse.Sz = Sz;
    // Switch buffers and start new reception
    pBufW = (pBufW == Buf1)? Buf2 : Buf1;
    Usb::StartReceiveI(EP_CDC_DATA_OUT, pBufW, OUT_BUF_SZ);
    // Take necessary action
    if(Action == TransferAction::SendEvt) EvtQMain.SendNowOrExitI(EvtMsg_t(EvtId::UsbDataRcvd));
    else Sys::WakeI(&pWaitingThd, retv::Ok);
    Sys::UnlockFromIRQ();
}

} // namespace

// Transmission buffers: several buffers of EP_BULK_SZ
static BufQ_t<uint8_t, EP_BULK_SZ, USB_TXBUF_CNT> InBuf;

// IN transfer end callback
void OnBulkInTransferEnd() {
    // Unlock the buffer just sent to allow it to be written again
    InBuf.UnlockBuf();
    // Start tx if buf is not empty
    if(Usb::IsActive() and !InBuf.IsEmpty()) {
        BufType_t<uint8_t> buf = InBuf.GetAndLockBuf();
        Sys::LockFromIRQ();
        Usb::StartTransmitI(EP_CDC_DATA_IN, buf.Ptr, buf.Sz);
        Sys::UnlockFromIRQ();
    }
}

#if 1 // ====================== CDC Line Coding related ========================
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
#define CDC_LINECODING_SZ   7UL
static union CDCLinecoding_t {
    struct {
        uint32_t dwDTERate = 115200;
        uint8_t bCharFormat = LC_STOP_1;
        uint8_t bParityType = LC_PARITY_NONE;
        uint8_t bDataBits = 8;
    };
    uint8_t Buf8[CDC_LINECODING_SZ];
} linecoding;

// Setup request callback: process class-related requests
Usb::StpReqCbRpl_t Usb::SetupReqHookCallback() {
    Usb::StpReqCbRpl_t Reply;
    if(Usb::SetupPkt.Type == USB_REQTYPE_CLASS) {
        switch(Usb::SetupPkt.bRequest) {
            case CDC_GET_LINE_CODING: // Send linecoding
                Reply.Retval = retv::Ok;
                Reply.Buf = linecoding.Buf8;
                Reply.Sz = CDC_LINECODING_SZ;
                break;
            case CDC_SET_LINE_CODING: // Receive data into linecoding
                Reply.Retval = retv::Ok;
                Reply.Buf= linecoding.Buf8;
                Reply.Sz = CDC_LINECODING_SZ;
                break;
            case CDC_SET_CONTROL_LINE_STATE: // Nothing to do, there are no control lines
                Reply.Retval = retv::Ok;
                Reply.Sz = 0; // Receive nothing
                break;
            default: break;
        }
    }
    return Reply;
}
#endif

#if 1 // ====================== Endpoints config ===============================
// Determines the space allocated for the TXFIFO as multiples of the packet size
// ==== EP1 ==== both IN and OUT
static const Usb::EpConfig_t BulkCfg = {
        .OutTransferEndCallback = OutQ::OnTransferEnd,
        .InTransferEndCallback = OnBulkInTransferEnd,
        .Type = Usb::EpType::Bulk,
        .OutMaxPktSz = EP_BULK_SZ,
        .InMaxPktSz = EP_BULK_SZ,
        .InMultiplier = 2
};

// ==== EP2 ==== Interrupt, IN only. Actually not used.
static const Usb::EpConfig_t InterruptCfg = {
        .OutTransferEndCallback = nullptr,
        .InTransferEndCallback = nullptr,
        .Type = Usb::EpType::Interrupt,
        .OutMaxPktSz = 0, // IN only
        .InMaxPktSz = EP_INTERRUPT_SZ,
        .InMultiplier = 1
};
#endif

void Usb::EventCallback(Usb::Evt event) {
    switch(event) {
        case Usb::Evt::Reset:
            return;
        case Usb::Evt::Address:
            return;
        case Usb::Evt::Configured:
            Sys::LockFromIRQ();
            /* Enable the endpoints specified in the configuration.
            Note, this callback is invoked from an ISR so I-Class functions must be used.*/
            Usb::InitEp(EP_CDC_DATA_IN, &BulkCfg);
            Usb::InitEp(EP_CDC_INTERRUPT, &InterruptCfg);
            // Reset queues
            OutQ::pBufW = (OutQ::pBufW == OutQ::Buf1)? OutQ::Buf2 : OutQ::Buf1;
            // Start reception. In this case, transaction size is limited to EP max size
            Usb::StartReceiveI(EP_CDC_DATA_OUT, OutQ::pBufW, OUT_BUF_SZ);
            EvtQMain.SendNowOrExitI(EvtMsg_t(EvtId::UsbReady)); // Inform main thread
            Sys::UnlockFromIRQ();
            return;
        case Usb::Evt::Suspend:
        case Usb::Evt::Wakeup:
        case Usb::Evt::Stalled:
        case Usb::Evt::Unconfigured:
            return;
    } // switch
}

retv UsbCDC_t::IPutChar(char c) {
    if(!Usb::IsActive()) return retv::Disconnected;
    retv r = InBuf.Put(c);
    if(InBuf.IsFullBufPresent() and !Usb::IsEpTransmitting(EP_CDC_DATA_IN)) { // New buffer is full
        BufType_t<uint8_t> buf = InBuf.GetAndLockBuf();
        Usb::StartTransmit(EP_CDC_DATA_IN, buf.Ptr, buf.Sz);
    }
    return r;
}

void UsbCDC_t::IStartTransmissionIfNotYet() {
    // Start tx if it has not already started and if buf is not empty.
    if(Usb::IsActive() and !Usb::IsEpTransmitting(EP_CDC_DATA_IN) and !InBuf.IsEmpty()) {
        BufType_t<uint8_t> buf = InBuf.GetAndLockBuf();
        Usb::StartTransmit(EP_CDC_DATA_IN, buf.Ptr, buf.Sz);
    }
}

retv UsbCDC_t::TryParseRxBuff() {
    while(OutQ::BufToParse.Sz) {
        OutQ::BufToParse.Sz--;
        if(Cmd.PutChar(*OutQ::BufToParse.Ptr++) == pdrNewCmd) return retv::Ok;
    }
    return retv::Fail;
}

// Send '>' and receive what follows
retv UsbCDC_t::ReceiveBinaryToBuf(uint8_t *ptr, uint32_t Len, uint32_t Timeout_ms) {
    OutQ::Action = OutQ::TransferAction::WakeThd; // Do not send evt to main q on buf reception
    if(IPutChar('>') != retv::Ok) return retv::Fail;
    IStartTransmissionIfNotYet();
    // Wait for data to be received
    Sys::Lock();
    systime_t Start = Sys::GetSysTimeX();
    systime_t TimeLeft, Timeout_st = TIME_MS2I(Timeout_ms);
    while(Len != 0) {
        // Calculate time left to wait
        systime_t Elapsed = Sys::TimeElapsedSince(Start);
        if(Elapsed > Timeout_st) break;
        TimeLeft = Timeout_st - Elapsed;
        // Wait data
        OutQ::pWaitingThd = Sys::GetSelfThd();
        if(Sys::SleepS(TimeLeft) == retv::Timeout) break; // Timeout occured
        // Will be here after successful reception; put data to buffer
        if(OutQ::BufToParse.Sz > Len) OutQ::BufToParse.Sz = Len; // Flush too large data
        memcpy(ptr, OutQ::BufToParse.Ptr, OutQ::BufToParse.Sz);
        Len -= OutQ::BufToParse.Sz;
        ptr += OutQ::BufToParse.Sz;
    }
    OutQ::Action = OutQ::TransferAction::SendEvt; // Return to normal life
    Sys::Unlock();
    return (Len == 0)? retv::Ok : retv::Fail; // Check if everything was received
}

// Wait '>' and then transmit buffer
retv UsbCDC_t::TransmitBinaryFromBuf(uint8_t *ptr, uint32_t Len, uint32_t Timeout_ms) {
    if(Usb::IsEpTransmitting(EP_CDC_DATA_IN)) return retv::Busy;
    retv r = retv::Timeout;
    Sys::Lock();
    OutQ::Action = OutQ::TransferAction::WakeThd; // Do not send evt to main q on buf reception
    systime_t Start = Sys::GetSysTimeX();
    systime_t TimeLeft, Timeout_st = TIME_MS2I(Timeout_ms);
    // Wait '>'
    while(r == retv::Timeout) {
        // Calculate time left to wait
        systime_t Elapsed = Sys::TimeElapsedSince(Start);
        if(Elapsed > Timeout_st) break;
        TimeLeft = Timeout_st - Elapsed;
        // Wait data
        OutQ::pWaitingThd = Sys::GetSelfThd();
        if(Sys::SleepS(TimeLeft) == retv::Timeout) break; // Timeout occured
        // Will be here after successful reception; check if '>' present
        for(uint32_t i=0; i<OutQ::BufToParse.Sz; i++) {
            if(OutQ::BufToParse.Ptr[i] == '>') {
                // Found
                r = retv::Ok;
                break;
            }
        } // for
    }
    // Will be here after either timeout or successful '>' reception
    OutQ::Action = OutQ::TransferAction::SendEvt; // Return to normal life
    if(r == retv::Ok) { // Transmit data
        if(Usb::IsActive()) Usb::StartTransmitI(EP_CDC_DATA_IN, ptr, Len);
        else r = retv::Disconnected;
    }
    Sys::Unlock();
    return r;
}

void UsbCDC_t::Connect()    { Usb::Connect(); }
void UsbCDC_t::Disconnect() { Usb::Disconnect(); }
bool UsbCDC_t::IsActive()   { return Usb::IsActive(); }
