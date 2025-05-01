#include "usb_dev.h"
#include "gd_lib_F405.h"
#include "yartos.h"
#include "shell.h"
#include "board.h"

// Standard request constants
static const uint32_t kReqGetStatus        = 0U;
static const uint32_t kReqClearFeature     = 1U;
static const uint32_t kReqSetFeature       = 3U;
static const uint32_t kReqSetAddress       = 5U;
static const uint32_t kReqGetDescriptor    = 6U;
static const uint32_t kReqSetDescriptor    = 7U;
static const uint32_t kReqGetConfiguration = 8U;
static const uint32_t kReqSetConfiguration = 9U;
static const uint32_t kReqGetInterface     = 10U;
static const uint32_t kReqSetInterface     = 11U;
static const uint32_t kReqSynchFrame       = 12U;

static const uint32_t kFeatureEndpointHalt = 0U;
static const uint32_t kFeatureDevRemoteWakeup = 1U;
static const uint32_t kFeatureTestMode     = 2U;

// Macro to construct single number out of two. To simplify processing of standard request.
#define REC_REQ(recipient, request)         (recipient | (request << 8))

// Short constants to transmit pusb statuses
static const uint8_t zero_status[2]   = {0x00, 0x00};
static const uint8_t active_status[2] = {0x00, 0x00};
static const uint8_t halted_status[2] = {0x01, 0x00};

#pragma region // ======================== Memory Control ===============================
// ==== RX FIFO ====
void UsbDev::RxFifoFlush() {
    pusb->GRSTCTL = GRSTCTL_RXFF;
    while(pusb->GRSTCTL & GRSTCTL_RXFF);
    DelayLoop(18); // Wait for 3 PHY Clocks
}

void UsbDev::RxFifoReadToBuf(uint8_t *buf, uint32_t n, uint32_t max) {
    uint32_t w = 0;
    uint32_t i = 0;
    while(i < n) {
        if((i & 3) == 0) w = pusb->FIFO[0][0];
        if(i < max) {
            *buf++ = (uint8_t)w;
            w >>= 8;
        }
        i++;
    }
}

void UsbDev::RxFifoReadToFunc(ftVoidU8 put_func, uint32_t n) {
    uint32_t w = 0;
    uint32_t i = 0;
    while(i < n) {
        if((i & 3) == 0) w = pusb->FIFO[0][0];
        put_func(static_cast<uint8_t>(w));
        w >>= 8;
        i++;
    }
}

void UsbDev::TxFifoFlush(uint32_t fifo) {
    pusb->GRSTCTL = GRSTCTL_TXFNUM(fifo) | GRSTCTL_TXFF;
    while(pusb->GRSTCTL & GRSTCTL_TXFF); // Wait completion
    DelayLoop(18); // Wait for 3 PHY Clocks
}

uint32_t UsbDev::TxFifoAllocate(uint32_t size) {
    uint32_t next = txfifo_ptr_next; // Save current address
    txfifo_ptr_next += size;
    Sys::DbgAssert(txfifo_ptr_next <= USB_FIFO_MEM_SZ32, "OTG FIFO memory overflow");
    return next;
}

void UsbDev::TxFifoFill(uint32_t ep) { // otg_txfifo_handler
    EpState *isp = &ep_info[ep].state_in; // -1 as no EP0 info
    const EpConfig* cfg = ep_info[ep].cfg;
    // The TXFIFO is filled until there is space left and data to be transmitted
    while(isp->cnt < isp->sz) { // Do until all the data will be sent
        // Number of bytes remaining in the current transaction
        uint32_t n = isp->sz - isp->cnt;
        if(n > cfg->in_pkt_sz_max) n = cfg->in_pkt_sz_max;
        // Check that the TXFIFO has enough space for the next packet
        if(((pusb->ie[ep].DIEPTFSTAT & DIEPTFSTAT_IEPTFS) * 4UL) < n) return;
        // Fill it
        volatile uint32_t *fifop = pusb->FIFO[ep]; // Destination
        uint8_t *buf = &isp->pbuf[isp->cnt]; // Src
        isp->cnt += n; // prepare for next time
        while(true) {
            *fifop = *((uint32_t*)buf);
            if(n <= 4UL) break;
            n -= 4UL;
            buf += 4UL;
        }
    } // while
    pusb->DIEPFEINTEN &= ~DIEPEMPMSK_INEPTXFEM(ep); // Disable FIFO_EMPTY IRQ
}
#pragma endregion // Mem control

#pragma region // ======== EP0 =========
void UsbDev::Ep0SetupPktCallback() {
    // Is the EP0 state machine in the correct state for handling setup packets?
    if(ep0.state != Ep0::Sta::STP_WAITING) { // Unexpected, could require handling with a warning event
        ep0.state = Ep0::Sta::STP_WAITING;   // Reset the EP0 state machine and proceed
    }
    // Read the setup data from buffer to setup_pkt struct
    setup_pkt.qw64 = ep0.setup_buf_qw64;
    // Do not process if app handler processed the request
    if(SetupReqHookCallback(&ep0.ptr_next, &ep0.transfer_len, &ep0.end_transaction_callback).NotOk()) {
        // Callback did not process the request. Try to process standard req, stall if fail
        if(setup_pkt.type != kReqTypeStd or DefaultRequestHandler().NotOk()) {
            /* Error response, the state machine goes into an error state, the low
                level layer will have to reset it to USB_EP0_WAITING_SETUP after
                receiving a SETUP packet.*/
            StallIn(0);
            StallOut(0);
            EventCallback(Evt::Stalled);
            ep0.state = Ep0::Sta::ERROR;
            return;
        }
    }
#if USB_SET_ADDRESS_ACK_BY_HW // Zero-length packet sent by hardware
    if(Usb::setup_pkt.bRequest == USB_REQ_SET_ADDRESS) return;
#endif
    // Data to transfer is prepared either by StdReqHookCallback, or by DefaultRequestHandler using PrepareSetupTransfer
    uint32_t max = setup_pkt.wLength;
    // The transfer size cannot exceed the specified amount
    if(ep0.transfer_len > max) ep0.transfer_len = max;
    if(setup_pkt.direction == kReqDirDev2Host) { // IN phase
        if(ep0.transfer_len != 0U) {
            // Start the transmit phase
            ep0.state = Ep0::Sta::IN_TX;
            Sys::LockFromIRQ();
            StartTransmitI(0, ep0.ptr_next, ep0.transfer_len);
            Sys::UnlockFromIRQ();
        }
        else { // No transmission phase, directly receiving the zero-sized status packet
            ep0.state = Ep0::Sta::OUT_WAITING_STS;
            Sys::LockFromIRQ();
            StartReceiveI(0, nullptr, 0);
            Sys::UnlockFromIRQ();
        }
    }
    else { // HOST2DEV, OUT phase
        if(ep0.transfer_len != 0U) { // There is something to receive
            // Start the receive phase
            ep0.state = Ep0::Sta::OUT_RX;
            Sys::LockFromIRQ();
            StartReceiveI(0, ep0.ptr_next, ep0.transfer_len);
            Sys::UnlockFromIRQ();
        }
        else { // No receive phase, directly sending the zero sized status packet
            ep0.state = Ep0::Sta::IN_SENDING_STS;
            Sys::LockFromIRQ();
            StartTransmitI(0, nullptr, 0);
            Sys::UnlockFromIRQ();
        }
    }
}

void UsbDev::Ep0InCallback() {
    uint32_t max;
    switch(ep0.state) {
        case Ep0::Sta::IN_TX:
            max = setup_pkt.wLength;
            /* If the transmitted size is less than the requested size and it is a
             multiple of the maximum packet size then a zero size packet must be
             transmitted.*/
            if((ep0.transfer_len < max) and ((ep0.transfer_len % kEp0sz) == 0U)) {
                Sys::LockFromIRQ();
                StartTransmitI(0, nullptr, 0);
                Sys::UnlockFromIRQ();
                ep0.state = Ep0::Sta::IN_WAITING_TX0;
                return;
            }
            // Fall through
        // Transmit phase over, receiving the zero sized status packet
        case Ep0::Sta::IN_WAITING_TX0:
            ep0.state = Ep0::Sta::OUT_WAITING_STS;
            Sys::LockFromIRQ();
            StartReceiveI(0, nullptr, 0);
            Sys::UnlockFromIRQ();
            return;
        // Status packet sent, invoke the callback if defined
        case Ep0::Sta::IN_SENDING_STS:
            if(ep0.end_transaction_callback) ep0.end_transaction_callback();
            ep0.state = Ep0::Sta::STP_WAITING;
            return;
        // Invalid states in the IN phase
        case Ep0::Sta::STP_WAITING:
        case Ep0::Sta::OUT_WAITING_STS:
        case Ep0::Sta::OUT_RX:
            Sys::DbgAssert(false, "EP0 IN error");
            // Fall through
        case Ep0::Sta::ERROR:
            /* Error response, the state machine goes into an error state, the low
             level layer will have to reset it to USB_EP0_WAITING_SETUP after
             receiving a SETUP packet.*/
            StallIn(0);
            StallOut(0);
            EventCallback(Evt::Stalled);
            ep0.state = Ep0::Sta::ERROR;
            return;
        default:
            Sys::DbgAssert(false, "EP0 IN invalid state");
    }
}

void UsbDev::Ep0OutCallback() {
    EpState *osp = &ep_info[0].state_out;
    /* If the transaction only covers part of the total transfer,
     * another transaction is immediately initiated to cover the remainder */
    if(((osp->cnt % kEp0sz) == 0) and (osp->sz < osp->total_sz)) {
        osp->sz = osp->total_sz - osp->sz;
        osp->cnt = 0;
        Sys::LockFromIRQ();
        StartOutTransfer(0);
        Sys::UnlockFromIRQ();
        return;
    }
    // All data received
    ep_receiving_mask &= ~(1UL << 0);
    switch(ep0.state) {
        // Receive phase over, send the zero sized status packet
        case Ep0::Sta::OUT_RX:
            ep0.state = Ep0::Sta::IN_SENDING_STS;
            Sys::LockFromIRQ();
            StartTransmitI(0, nullptr, 0);
            Sys::UnlockFromIRQ();
            return;
        // Status packet received, it must be zero sized, invoking the callback if defined
        case Ep0::Sta::OUT_WAITING_STS:
            if(osp->cnt != 0U) break;
            if(ep0.end_transaction_callback) ep0.end_transaction_callback();
            ep0.state = Ep0::Sta::STP_WAITING;
            return;
        // Invalid states in the OUT phase
        case Ep0::Sta::STP_WAITING:
        case Ep0::Sta::IN_TX:
        case Ep0::Sta::IN_WAITING_TX0:
        case Ep0::Sta::IN_SENDING_STS:
            Sys::DbgAssert(false, "EP0 OUT error");
            // Fall through
        case Ep0::Sta::ERROR:
            /* Error response, the state machine goes into an error state, the low
             level layer will have to reset it to USB_EP0_WAITING_SETUP after
             receiving a SETUP packet.*/
            StallIn(0);
            StallOut(0);
            EventCallback(Evt::Stalled);
            ep0.state = Ep0::Sta::ERROR;
            return;
        default:
            Sys::DbgAssert(false, "EP0 OUT invalid state");
    }
}

void UsbDev::Ep0Reset() {
    ep_info[0].state_in.sz = 0;
    ep_info[0].state_in.cnt = 0;
    ep_info[0].state_in.total_sz = 0;
    ep_info[0].state_out.sz = 0;
    ep_info[0].state_out.cnt = 0;
    ep_info[0].state_out.total_sz = 0;
}
#pragma endregion // Ep0

#pragma region // ============================= Endpoints ===============================
UsbDev::EpSta UsbDev::GetStatusIn(uint32_t ep) {
    uint32_t ctl = pusb->ie[ep].DIEPCTL;
    if(!(ctl & DIEPCTL_EPACT)) return EpSta::DISABLED;
    if(  ctl & DIEPCTL_STALL)  return EpSta::STALLED;
    return EpSta::ACTIVE;
}

UsbDev::EpSta UsbDev::GetStatusOut(uint32_t ep) {
    uint32_t ctl = pusb->oe[ep].DOEPCTL;
    if(!(ctl & DOEPCTL_EPACT)) return EpSta::DISABLED;
    if(  ctl & DOEPCTL_STALL)  return EpSta::STALLED;
    return EpSta::ACTIVE;
}

void UsbDev::StartInTransfer(uint32_t ep) {
    EpState *isp = &ep_info[ep].state_in;
    const EpConfig *cfg = ep_info[ep].cfg;
    // Transfer initialization
    isp->total_sz = isp->sz;
    // Special case, sending zero size packet
    if(isp->sz == 0) pusb->ie[0].DIEPLEN = DIEPLEN_PKTCNT(1) | DIEPLEN_TLEN(0);
    else { // Ordinal case
        if(ep == 0 and isp->sz > kEp0sz) isp->sz = kEp0sz; // Single pkt only for Ep0
        uint32_t pktcnt = (isp->sz + cfg->in_pkt_sz_max - 1UL) / cfg->in_pkt_sz_max;
        pusb->ie[ep].DIEPLEN = DIEPLEN_MCNT(1) | DIEPLEN_PKTCNT(pktcnt) | DIEPLEN_TLEN(isp->sz);
    }
    // Special case for isochronous endpoint
    if(cfg->type == EpType::Iso) { // Toggle odd/even bit
        if(pusb->DSTAT & DSTAT_FNRSOF_ODD) pusb->ie[ep].DIEPCTL |= DIEPCTL_SETEVENFRM;
        else pusb->ie[ep].DIEPCTL |= DIEPCTL_SETODDFRM;
    }
    // Start operation
    pusb->ie[ep].DIEPCTL |= DIEPCTL_EPEN | DIEPCTL_CNAK;
    pusb->DIEPFEINTEN |= DIEPEMPMSK_INEPTXFEM(ep);
}

/* Transaction size is rounded to a multiple of packet size because the
    following requirement in the RM:
    "For OUT transfers, the transfer size field in the endpoint's transfer
    size register must be a multiple of the maximum packet size of the
    endpoint, adjusted to the Word boundary".*/
void UsbDev::StartOutTransfer(uint32_t ep) {
    EpState *osp = &ep_info[ep].state_out;
    const EpConfig *cfg = ep_info[ep].cfg;
    // Transfer initialization
    osp->total_sz = osp->sz;
    if(ep == 0 and osp->sz > kEp0sz) osp->sz = kEp0sz; // Single pkt only for Ep0
    uint32_t max_sz = cfg->out_pkt_sz_max;
    uint32_t pktcnt = (osp->sz + max_sz - 1UL) / max_sz;
    uint32_t rxsize = (pktcnt * max_sz + 3U) & 0xFFFFFFFCUL;
    // Check if size is not too large
//    Sys::DbgAssert(rxsize < 0x7FFFF, "Bad Size"); // rxsize is 19 bit value
//    Sys::DbgAssert(pcnt < 0x3FF, "Bad Size"); // len is 10 bit value
    // Setup transaction parameters in DOEPLEN
    pusb->oe[ep].DOEPLEN = DOEPLEN_STPCNT(3) | DOEPLEN_PCNT(pktcnt) | DOEPLEN_TLEN(rxsize);
    // Special case of isochronous endpoint
    if(cfg->type == EpType::Iso) { // Toggle odd/even bit
        if(pusb->DSTAT & DSTAT_FNRSOF_ODD) pusb->oe[ep].DOEPCTL |= DIEPCTL_SETEVENFRM;
        else pusb->oe[ep].DOEPCTL |= DIEPCTL_SETODDFRM;
    }
    // Start operation
    pusb->oe[ep].DOEPCTL |= DOEPCTL_EPEN | DOEPCTL_CNAK;
}

// Disable endpoints except irq for ep0
void UsbDev::EpDisableAll() {
    for(uint32_t i=0; i < USB_EP_CNT; i++) {
        pusb->ie[i].DIEPCTL = 0;
        pusb->ie[i].DIEPLEN = 0;
        pusb->ie[i].DIEPINTF = 0xFFFFFFFF;
        pusb->oe[i].DOEPCTL = 0;
        pusb->oe[i].DOEPLEN = 0;
        pusb->oe[i].DOEPINTF = 0xFFFFFFFF;
    }
    pusb->DAEPINTEN = DAEPINTEN_OEPIE(0) | DAEPINTEN_IEPIE(0);
}

void UsbDev::DisableEndpointsI() {
    Sys::DbgAssert(usb_state == UsbSta::Active, "invalid state");
    ep_transmitting_mask &= 1UL; // Do not touch Ep0
    ep_receiving_mask &= 1UL;    // Do not touch Ep0
    TxFifoReset();
    EpDisableAll();
}

void UsbDev::InitEp(uint32_t ep, const EpConfig *pcfg) {
    Sys::DbgAssert(ep > 0 and ep <= USB_EP_CNT, "Bad ep indx");
    ep_info[ep].cfg = pcfg;
    uint32_t ctl = DIEPCTL_SD0PID | DIEPCTL_EPACT;
    // IN and OUT common parameters
    switch(pcfg->type) {
        case EpType::Ctrl:      ctl |= DIEPCTL_EPTYPE_CTRL; break;
        case EpType::Iso:       ctl |= DIEPCTL_EPTYPE_ISO;  break;
        case EpType::Bulk:      ctl |= DIEPCTL_EPTYPE_BULK; break;
        case EpType::Interrupt: ctl |= DIEPCTL_EPTYPE_INTR; break;
    }

    // OUT endpoint activation or deactivation
    pusb->oe[ep].DOEPLEN = 0;
    if(pcfg->out_pkt_sz_max != 0) { // OUT enabled
        pusb->oe[ep].DOEPCTL = ctl | DOEPCTL_MPL(pcfg->out_pkt_sz_max);
        pusb->DAEPINTEN |= DAEPINTEN_OEPIE(ep);
    }
    else { // Disable OUT functionality
        pusb->oe[ep].DOEPCTL &= ~DOEPCTL_EPACT;
        pusb->DAEPINTEN &= ~DAEPINTEN_OEPIE(ep);
    }

    // IN endpoint activation or deactivation
    pusb->ie[ep].DIEPLEN = 0;
    if(pcfg->in_pkt_sz_max != 0) {
        // Allocate FIFO
        uint32_t fsize = pcfg->in_pkt_sz_max / 4; // FIFO data is 32-bit wide
        fsize *= pcfg->in_multiplier;
        // Next is -1, as Ep0 has special register
        pusb->DIEPTFLEN[ep-1] = DIEPTFLEN_IEPTXFD(fsize) | DIEPTFLEN_IEPTXRSAR(TxFifoAllocate(fsize));
        TxFifoFlush(ep);
        pusb->ie[ep].DIEPCTL = ctl | DIEPCTL_TXFNUM(ep) | DIEPCTL_MPL(pcfg->in_pkt_sz_max);
        pusb->DAEPINTEN |= DAEPINTEN_IEPIE(ep);
    }
    else { // Disable IN functionality
        pusb->DIEPTFLEN[ep-1] = 0x02000400; // Reset value
        TxFifoFlush(ep);
        pusb->ie[ep].DIEPCTL &= ~DIEPCTL_EPACT;
        pusb->DAEPINTEN &= ~DAEPINTEN_IEPIE(ep);
    }
}
#pragma endregion

#pragma region // =========================== USB core ==================================
void UsbDev::SetAddress() {
    address = setup_pkt.wValue & 0xFFUL;
    pusb->DCFG = (pusb->DCFG & ~DCFG_DAR_MASK) | DCFG_DAR(address);
    EventCallback(Evt::Address);
    usb_state = UsbSta::Selected;
}

retv UsbDev::DefaultRequestHandler() {
    Buf_t descriptor;
    // Concatenate recipient and Request into single data word
    uint32_t recipient_and_req = static_cast<uint32_t>(setup_pkt.recipient) | (static_cast<uint32_t>(setup_pkt.bRequest) << 8);
    uint16_t w16;
    // Decode the request
    switch(recipient_and_req) {
        /* The Get Status request directed at the device will return two bytes, only lesser bits D0 and D1 are used.
         * If D0 is set, then this indicates the device is self powered. If clear, the device is bus powered.
         * If D1 is set, the device has remote wakeup enabled and can wake the host up during suspend.
         * The remote wakeup bit can be controlled by the SetFeature and ClearFeature requests
         * with a feature selector of DEVICE_REMOTE_WAKEUP (0x01)
         */
        case REC_REQ(kReqRecpntDevice, kReqGetStatus): // Just return the current status word
            w16 = (USB_REMOTE_WKUP_EN << 1) | USB_SELF_POWERED;
            ep0.PrepareSetupTransfer(reinterpret_cast<uint8_t*>(&w16), 2, nullptr);
            return retv::Ok;

        // Only the DEVICE_REMOTE_WAKEUP is handled here, any other feature number is handled as an error
#if USB_REMOTE_WKUP_EN == 1U
        case REC_REQ(kReqRecpntDevice, kReqClearFeature):
            if(setup_pkt.wValue == kFeatureDevRemoteWakeup) {
                // status &= ~(1U << 1); // Clear wkup bit
                ep0.PrepareSetupTransfer(nullptr, 0, nullptr);
                return retv::Ok;
            }
            return retv::Fail;
        case REC_REQ(kReqRecpntDevice, kReqSetFeature):
            if(setup_pkt.wValue == kFeatureDevRemoteWakeup) {
                // status |= (1U << 1); // Set wkup bit
                ep0.PrepareSetupTransfer(nullptr, 0, nullptr);
                return retv::Ok;
            }
            return retv::Fail;
#endif

        case REC_REQ(kReqRecpntDevice, kReqSetAddress):
            /* The SET_ADDRESS handling can be performed here or postponed after
             the status packet depending on the USB_SET_ADDRESS_MODE low
             driver setting.*/
#if USB_SET_ADDR_AFTER_ZEROPKT  // (1) Send ZeroPkt (2) Set Address
            ep0.PrepareSetupTransfer(nullptr, 0, SetAddress);
#else // (1) Set Address (2) Send ZeroPkt
            SetAddress();
            ep0.PrepareSetupTransfer(nullptr, 0, nullptr);
#endif
            return retv::Ok;

        case REC_REQ(kReqRecpntDevice, kReqGetDescriptor):
        case REC_REQ(kReqRecpntInterface, kReqGetDescriptor):
            descriptor = GetDescriptor(setup_pkt.dsc_type, setup_pkt.dsc_index, setup_pkt.wIndex);
            if(descriptor.ptr == nullptr) return retv::Fail;
            // Length is first byte of descriptor
            ep0.PrepareSetupTransfer(descriptor.ptr, descriptor.sz, nullptr);
            return retv::Ok;

        case REC_REQ(kReqRecpntDevice, kReqGetConfiguration): // Return the last selected configuration
            ep0.PrepareSetupTransfer(&configuration, 1, nullptr);
            return retv::Ok;

        case REC_REQ(kReqRecpntDevice, kReqSetConfiguration):
            // f the pusb device is already active, we need to perform the clear procedure on the current configuration
            if(usb_state == UsbSta::Active) {
                // Clear current configuration
                Sys::LockFromIRQ();
                DisableEndpointsI();
                Sys::UnlockFromIRQ();
                configuration = 0U;
                usb_state = UsbSta::Selected;
                EventCallback(Evt::Unconfigured);
            }
            if(setup_pkt.cfgNumber != 0) { // Set new configuration
                configuration = setup_pkt.cfgNumber;
                usb_state = UsbSta::Active;
                EventCallback(Evt::Configured);
            }
            ep0.PrepareSetupTransfer(nullptr, 0, nullptr);
            return retv::Ok;

        case REC_REQ(kReqRecpntInterface, kReqGetStatus):
        case REC_REQ(kReqRecpntEndpoint,  kReqSynchFrame):
            // Just send two zero bytes
            ep0.PrepareSetupTransfer((uint8_t*)zero_status, 2, nullptr);
            return retv::Ok;

        case REC_REQ(kReqRecpntEndpoint, kReqGetStatus):
            if(setup_pkt.ep.dir == 1) { // IN ep
                switch(GetStatusIn(setup_pkt.ep.number)) {
                    case EpSta::STALLED:
                        ep0.PrepareSetupTransfer((uint8_t*)halted_status, 2, nullptr);
                        return retv::Ok;
                    case EpSta::ACTIVE:
                        ep0.PrepareSetupTransfer((uint8_t*) active_status, 2, nullptr);
                        return retv::Ok;
                    case EpSta::DISABLED:
                    default: return retv::Fail;
                }
            }
            else { // OUT ep
                switch(GetStatusOut(setup_pkt.ep.number)) {
                    case EpSta::STALLED:
                        ep0.PrepareSetupTransfer((uint8_t*)halted_status, 2, nullptr);
                        return retv::Ok;
                    case EpSta::ACTIVE:
                        ep0.PrepareSetupTransfer((uint8_t*)active_status, 2, nullptr);
                        return retv::Ok;
                    case EpSta::DISABLED:
                    default: return retv::Fail;
                }
            }

        case REC_REQ(kReqRecpntEndpoint, kReqClearFeature): // Only ENDPOINT_HALT is handled as feature
            if(setup_pkt.epFeatureSelector != kFeatureEndpointHalt) return retv::Fail;
            // Clear the EP status, not valid for EP0, it is ignored in that case
            if(setup_pkt.ep.number != 0U) {
                if(setup_pkt.ep.dir == 1) ClearIn(setup_pkt.ep.number);
                else ClearOut(setup_pkt.ep.number);
            }
            ep0.PrepareSetupTransfer(nullptr, 0, nullptr);
            return retv::Ok;

        case REC_REQ(kReqRecpntEndpoint, kReqSetFeature): // Only ENDPOINT_HALT is handled as feature
            if(setup_pkt.epFeatureSelector != kFeatureEndpointHalt) return retv::Fail;
            // Stall the EP, not valid for EP0, it is ignored in that case
            if(setup_pkt.ep.number != 0U) {
                if(setup_pkt.ep.dir == 1) StallIn(setup_pkt.ep.number);
                else StallOut(setup_pkt.ep.number);
            }
            ep0.PrepareSetupTransfer(nullptr, 0, nullptr);
            return retv::Ok;

        default: return retv::Fail;
    } // switch
}

// Reset core and delay of at least 3 PHY cycles
void UsbDev::ResetCore() {
    pusb->GRSTCTL |= GRSTCTL_CSRST;
    DelayLoop(12);
    while(pusb->GRSTCTL & GRSTCTL_CSRST);
    DelayLoop(18);
}

void UsbDev::StartCore() {
    Sys::Lock();
    if(usb_state == UsbSta::Stop) {
        if(pusb == USB_FS) {
            Clock::EnUSBFS();
            Clock::ResetUSBFS();
        }
        else {
            Clock::EnUSBHS();
            Clock::ResetUSBHS();
        }
        Nvic::EnableVector(USB_IRQ_NUMBER, USB_IRQ_PRIO);
        // Forced device mode, pusb turn-around time = TRDT_VALUE_FS, Full Speed 1.1 PHY
        pusb->GUSBCS = GUSBCS_FDM | GUSBCS_UTT(UTT_VALUE_FS) | GUSBCS_EMBPHY; // EMBPHY marked as reserved, but who knows
        pusb->DCFG = DCFG_DS_FULL_SPEED; // Always start with FS
        pusb->PWRCLKCTL = 0; // En clocks
        ResetCore(); // Soft reset pusb core
        // Consider VBUS voltage always valid, en VBUSA & B (required), pwron embd PHY
        pusb->GCCFG = GCCFG_VBUSIG | GCCFG_VBUSACEN | GCCFG_VBUSBCEN | GCCFG_PWRON;
        pusb->GCCFG |= GCCFG_SOFOEN; // Enable SOF output. May not be necessary, but who cares? But must be enabled for CTC.
        DelayLoop(20);
        DisconnectBus(); 
        DelayLoop(9);
        pusb->GAHBCS = 0; // Interrupts on TXFIFOs half empty, global irqs dis
        EpDisableAll();
        // Clear all pending Device Interrupts, only the pusb Reset interrupt is required initially
        pusb->DIEPINTEN  = 0;
        pusb->DOEPINTEN  = 0;
        pusb->DAEPINTEN = 0;
#if USB_SOF_CB_EN
        pusb->GINTEN = GINTEN_ENUMFIE | GINTEN_RSTIE | GINTEN_SPIE |
        GINTEN_ESPIE | GINTEN_SESIE | GINTEN_WKUPIE | GINTEN_ISOINCIE | GINTEN_ISOONCIE |
        GINTEN_SOFIE;
#else
        pusb->GINTEN = GINTEN_ENUMFIE | GINTEN_RSTIE | GINTEN_SPIE |
            GINTEN_ESPIE | GINTEN_SESIE | GINTEN_WKUPIE | GINTEN_ISOINCIE | GINTEN_ISOONCIE;
#endif
        pusb->GINTF  = 0xFFFFFFFF;     // Clear all pending IRQs, if any
        pusb->GAHBCS |= GAHBCS_GINTEN; // Global interrupts enable
    }
    usb_state = UsbSta::Ready;
    Sys::Unlock();
}

void UsbDev::StopCore() {
    Sys::Lock();
    if(usb_state != UsbSta::Stop) {
        EpDisableAll();
        pusb->DAEPINTEN = 0;
        pusb->GAHBCS   = 0;
        pusb->GCCFG    = 0;
        Nvic::DisableVector(USB_IRQ_NUMBER);
        if(pusb == USB_FS) Clock::DisUSBFS();
        else Clock::DisUSBHS();
    }
    usb_state = UsbSta::Stop;
    Ep0Reset();
    Sys::RescheduleS();
    Sys::Unlock();
}
#pragma endregion

#pragma region // ============================ IRQ Handlers =============================
void UsbDev::OnIrqReset() {
    usb_state = UsbSta::Ready;
    // Reset internal state
    ep_transmitting_mask = 0;
    ep_receiving_mask = 0;
    address = 0;
    configuration = 0;
    Ep0Reset();
    TxFifoFlush(0);
    // Clear and disable all ep irqs
    pusb->DIEPFEINTEN = 0;
    pusb->DAEPINTEN = DAEPINTEN_OEPIE(0) | DAEPINTEN_IEPIE(0);

    // Set all endpoints in NAK mode, clear interrupts
    for(unsigned i = 0; i <= USB_EP_CNT; i++) {
        pusb->ie[i].DIEPCTL = DIEPCTL_SNAK;
        pusb->oe[i].DOEPCTL = DOEPCTL_SNAK;
        pusb->ie[i].DIEPINTF = 0xFFFFFFFF;
        pusb->oe[i].DOEPINTF = 0xFFFFFFFF;
    }

    TxFifoReset(); // Reset the FIFO memory allocator
    // Init RX FIFO size, the address is always zero
    pusb->GRFLEN = USB_RX_FIFO_SZ32;
    RxFifoFlush();

    // Reset the device address to zero
    pusb->DCFG = (pusb->DCFG & ~DCFG_DAR_MASK) | DCFG_DAR(0);

    // Enable EP-related interrupt sources
    pusb->GINTEN  |= GINTEN_RXFNEIE | GINTEN_OEPIE  | GINTEN_IEPIE;
    pusb->DIEPINTEN  = DIEPINTEN_CITOEN | DIEPINTEN_TFEN;
    pusb->DOEPINTEN  = DOEPINTEN_STPFEN | DOEPINTEN_TFEN;

    // EP0 initialization, it is a special case
    ep0.state = Ep0::Sta::STP_WAITING; // EP0 state machine initialization
    pusb->oe[0].DOEPLEN = DOEPLEN_STPCNT(3);
    pusb->oe[0].DOEPCTL = DOEPCTL_SD0PID | DOEPCTL_EPACT | DOEPCTL_EPTYPE_CTRL | DOEPCTL_MPL(kEp0sz);
    pusb->ie[0].DIEPLEN = 0;
    pusb->ie[0].DIEPCTL = DIEPCTL_SD0PID | DIEPCTL_EPACT | DIEPCTL_EPTYPE_CTRL |
                          DIEPCTL_TXFNUM(0) | DIEPCTL_MPL(kEp0sz);
    pusb->DIEP0TFLEN = DIEPTFLEN_IEPTXFD(kEp0sz / 4) | DIEPTFLEN_IEPTXRSAR(TxFifoAllocate(kEp0sz / 4));
    EventCallback(Evt::Reset);
}

void UsbDev::OnIrqWakeup() {
    usb_state = saved_state; // State transition, returning to the previous state
    EventCallback(Evt::Wakeup);
}

void UsbDev::OnIrqSuspend() {
    saved_state = usb_state;
    usb_state = UsbSta::Suspended;
    EventCallback(Evt::Suspend);
}

// Isochronous IN transfer failed handler
void UsbDev::OnIrqIsoInFailed() {
    for(uint32_t ep=1; ep <= USB_EP_CNT; ep++) { // Exclude Ep0
        // Endpoint is ISO and is enabled -> ISOC IN transfer failed
        if(((pusb->ie[ep].DIEPCTL & DIEPCTL_EPTYPE_MASK) == DIEPCTL_EPTYPE_ISO) and (pusb->ie[ep].DIEPCTL & DIEPCTL_EPEN)) {
            // Disable endpoint
            pusb->ie[ep].DIEPCTL |= (DIEPCTL_EPD | DIEPCTL_SNAK);
            while(pusb->ie[ep].DIEPCTL & DIEPCTL_EPEN);
            TxFifoFlush(ep); // Flush FIFO
            ep_transmitting_mask &= ~(1UL << ep);
            CallInTransferEndCallback(ep); // Prepare data for next frame
        }
    }
}

// Isochronous OUT transfer failed handler
void UsbDev::OnIrqIsoOutFailed() {
    for(uint32_t ep=1; ep <= USB_EP_CNT; ep++) { // Exclude Ep0
        // Endpoint is ISO and is enabled -> ISOC OUT transfer failed
        if(((pusb->oe[ep].DOEPCTL & DOEPCTL_EPTYPE_MASK) == DOEPCTL_EPTYPE_ISO) and (pusb->oe[ep].DOEPCTL & DOEPCTL_EPEN)) {
            // Disable endpoint
            /* Core stucks here */
            /*otgp->oe[ep].DOEPCTL |= (DOEPCTL_EPDIS | DOEPCTL_SNAK);
             while (otgp->oe[ep].DOEPCTL & DOEPCTL_EPENA); */
            ep_receiving_mask &= ~(1UL << ep);
            CallOutTransferEndCallback(ep, ep_info[ep].state_out.cnt); // Prepare transfer for next frame
        }
    }
}

// Incoming packets handler. USBFS sets this bit when there is at least one packet or status entry in the Rx FIFO
void UsbDev::OnIrqRxFifoNotEmpty() {
    // Pop the event word
    uint32_t sts = pusb->GRSTATP;
    // Event details
    uint32_t cnt = (sts & GRSTATP_BCOUNT_MASK) >> GRSTATP_BCOUNT_OFFSET;
    uint32_t ep  = (sts & GRSTATP_EPNUM_MASK) >> GRSTATP_EPNUM_OFFSET;
    EpState *osp;
    switch(sts & GRSTATP_RPCKST_MASK) {
        case GRSTATP_SETUP_DATA:
            RxFifoReadToBuf(ep0.setup_buf, cnt, 8);
            break;
        case GRSTATP_SETUP_COMP:
            break;
        case GRSTATP_OUT_DATA:
            osp = &ep_info[ep].state_out;
            RxFifoReadToBuf(&osp->pbuf[osp->cnt], cnt, osp->sz - osp->cnt);
            osp->cnt += cnt;
            break;
        case GRSTATP_OUT_COMP:
            break;
        case GRSTATP_OUT_GLOBAL_NAK:
            break;
        default:
            break;
    }
}

void UsbDev::OnIrqEpOut(uint32_t ep) {
    uint32_t epint = pusb->oe[ep].DOEPINTF;
    pusb->oe[ep].DOEPINTF = epint; // Clear all EP IRQ flags
    // Setup packets are handled using a specific callback
    if((ep == 0) and (epint & DOEPINTF_STPF) and (pusb->DOEPINTEN & DOEPINTEN_STPFEN)) Ep0SetupPktCallback();
    // Transfer complete
    if((epint & DOEPINTF_TF) && (pusb->DOEPINTEN & DOEPINTEN_TFEN)) {
        if(ep == 0) { // EP0 requires special handling
#if USB_SEQUENCE_WORKAROUND
      /* If an OUT transaction end interrupt is processed while the state
         machine is not in an OUT state then it is ignored, this is caused
         on some devices (L4) apparently injecting spurious data complete
         words in the RX FIFO.*/
            if(!(ep0.state == Ep0::Sta::OUT_RX or ep0.state == Ep0::Sta::OUT_WAITING_STS)) return;
#endif
            Ep0OutCallback();
        } // ep0
        else {
            ep_receiving_mask &= ~(1UL << ep);
            CallOutTransferEndCallback(ep, ep_info[ep].state_out.cnt);
        }
    } // if XFRC
}

void UsbDev::OnIrqEpIn(uint32_t ep) {
    uint32_t epint = pusb->ie[ep].DIEPINTF;
    pusb->ie[ep].DIEPINTF = epint; // Clear all EP IRQ flags
    // Timeouts not handled yet, not sure how to handle
    if(epint & DIEPINTF_CITO) { }
    // Transfer complete
    if((epint & DIEPINTF_TF) and (pusb->DIEPINTEN & DIEPINTEN_TFEN)) {
        EpState *isp = &ep_info[ep].state_in;
        if(isp->sz < isp->total_sz) {
            /* If the transaction only covers part of the total transfer,
             * another transaction is immediately initiated to cover the remainder */
            isp->pbuf += isp->sz;
            isp->sz = isp->total_sz - isp->sz;
            isp->cnt = 0;
            Sys::LockFromIRQ();
            StartInTransfer(ep);
            Sys::UnlockFromIRQ();
        }
        else { // End on IN transfer
            ep_transmitting_mask &= ~(1UL << ep);
            if(ep == 0) Ep0InCallback();
            else CallInTransferEndCallback(ep);
        }
    } // XFRC
    // TX FIFO empty or emptying => fill it
    if((epint & DIEPINTF_TXFE) and (pusb->DIEPFEINTEN & DIEPEMPMSK_INEPTXFEM(ep))) {
        TxFifoFill(ep);
    }
}
#pragma endregion

void UsbDev::StartReceiveI(uint32_t ep, uint8_t *pbuf, uint32_t max_sz) {
    Sys::DbgCheckClassI();
    // Set ep flag
    ep_receiving_mask |= (1UL << ep);
    // Setup transfer
    EpState *osp = &ep_info[ep].state_out;
    osp->pbuf = pbuf;
    osp->sz = max_sz;
    osp->cnt = 0;
    StartOutTransfer(ep);
}

void UsbDev::StartTransmitI(uint32_t ep, uint8_t *pbuf, uint32_t sz) {
    Sys::DbgCheckClassI();
    Sys::DbgAssert(!IsEpTransmitting(ep), "already transmitting");
    // Set ep flag
    ep_transmitting_mask |= (1UL << ep);
    // Setup transfer
    EpState *isp = &ep_info[ep].state_in;
    isp->pbuf = pbuf;
    isp->sz = sz;
    isp->cnt = 0;
    StartInTransfer(ep);
}

void UsbDev::StartTransmit(uint32_t ep, uint8_t *pbuf, uint32_t sz) {
    Sys::Lock();
    StartTransmitI(ep, pbuf, sz);
    Sys::Unlock();
}

#pragma region // ============================== IRQ =============================
void UsbDev::ProcessIrq() {
    uint32_t sts  = pusb->GINTF;
    sts &= pusb->GINTEN;
    pusb->GINTF = sts;
    // === Process what happened ====
    if(sts & GINTF_RST) { // Reset
        OnIrqReset();
        return; // the core has been reset => do not process other flags
    }

    if(sts & GINTF_WKUPIF) { // Wake-up
        // If clocks are gated off, turn them back on (may be the case if coming out of suspend mode)
        if(pusb->PWRCLKCTL & (PWRCLKCTL_SHCLK | PWRCLKCTL_SUCLK)) pusb->PWRCLKCTL &= ~(PWRCLKCTL_SHCLK | PWRCLKCTL_SUCLK);
        // Clear the Remote Wake-up Signaling
        pusb->DCTL &= ~DCTL_RWKUP;
        OnIrqWakeup();
    }

    // Suspend handling
    if(sts & GINTF_SP) OnIrqSuspend();

    // Enumeration done
    if(sts & GINTF_ENUMF) {
        // Full or High speed timing selection
        if((pusb->DSTAT & DSTAT_ES_MASK) == DSTAT_ES_HIGH_SPEED)
            pusb->GUSBCS = (pusb->GUSBCS & ~GUSBCS_UTT_MASK) | GUSBCS_UTT(UTT_VALUE_HS);
        else
            pusb->GUSBCS = (pusb->GUSBCS & ~GUSBCS_UTT_MASK) | GUSBCS_UTT(UTT_VALUE_FS);
    }

#if USB_SOF_CB_EN // SOF interrupt handling
    if(sts & GINTF_SOF) Usb::SOFCallback();
#endif

    // Isochronous IN failed
    if(sts & GINTF_ISOINCIF) OnIrqIsoInFailed();
    // Isochronous OUT failed
    if(sts & GINTF_ISOONCIF) OnIrqIsoOutFailed();

    // Performing the whole FIFO emptying in the ISR, it is advised to keep this IRQ at a very low priority level
    if(sts & GINTF_RXFNEIF) OnIrqRxFifoNotEmpty();

    // IN/OUT endpoints event handling
    uint32_t src = pusb->DAEPINT;
    if(sts & GINTF_OEPIF) {
        if(src & (1 << 16)) OnIrqEpOut(0);
        if(src & (1 << 17)) OnIrqEpOut(1);
        if(src & (1 << 18)) OnIrqEpOut(2);
        if(src & (1 << 19)) OnIrqEpOut(3);
#if kEpCnt >= 4
        if(src & (1 << 20)) OnIrqEpOut(4);
#endif
#if kEpCnt >= 5
        if(src & (1 << 21)) OnIrqEpOut(5);
#endif
#if kEpCnt >= 6
        if(src & (1 << 22)) OnIrqEpOut(6);
#endif
#if kEpCnt >= 7
        if(src & (1 << 23)) OnIrqEpOut(7);
#endif
#if kEpCnt >= 8
        if(src & (1 << 24)) OnIrqEpOut(8);
#endif
    }
    if(sts & GINTF_IEPIF) {
        if(src & (1 << 0)) OnIrqEpIn(0);
        if(src & (1 << 1)) OnIrqEpIn(1);
        if(src & (1 << 2)) OnIrqEpIn(2);
        if(src & (1 << 3)) OnIrqEpIn(3);
#if kEpCnt >= 4
        if(src & (1 << 4)) OnIrqEpIn(4);
#endif
#if kEpCnt >= 5
        if(src & (1 << 5)) OnIrqEpIn(5);
#endif
#if kEpCnt >= 6
        if(src & (1 << 6)) OnIrqEpIn(6);
#endif
#if kEpCnt >= 7
        if(src & (1 << 7)) OnIrqEpIn(7);
#endif
#if kEpCnt >= 8
        if(src & (1 << 8)) OnIrqEpIn(8);
#endif
    }
}
#pragma endregion

#pragma region // ============================== Public =============================
void UsbDev::Init() {
    Gpio::SetupAlterFunc(USB_DM);
    Gpio::SetupAlterFunc(USB_DP);
    // ep_info.resize(kEpCnt + 1UL); // +1 to add EP0 info
}

void UsbDev::Connect() {
    DisconnectBus();
    Sys::SleepMilliseconds(99);
    StartCore();
    ConnectBus();
}

void UsbDev::Disconnect() {
    StopCore();
    DisconnectBus();
}
#pragma endregion
