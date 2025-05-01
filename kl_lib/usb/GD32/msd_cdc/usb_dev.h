/*
 * This module provides only low-level services, such as transmitting, receiving
 * and default Setup Request handling. All other must provide higher-level driver.
 */

#ifndef USB_DEV_H_
#define USB_DEV_H_

#include "types.h"
#include "board.h"
#include "kl_buf.h"
#include "GD32F4xx.h"

// Params
#define USB_RX_FIFO_SZ      512U // Why 512? Because why not. There is 1280 bytes total, for both RX & TX
#define USB_RX_FIFO_SZ32    (USB_RX_FIFO_SZ / 4) // rx_fifo_size in 32-bit words
#define USB_FIFO_MEM_SZ32   320 // Total FIFO size in 32-bit words; hardware is such as it is.
#define UTT_VALUE_FS        5 // Turnaround time in PHY clocks for Full Speed. No info why 5.
#define UTT_VALUE_HS        9 // Turnaround time in PHY clocks for High Speed. No info why 9.

// Control Endpoint must have a packet size of 64 bytes. Do not touch this.
inline constexpr const uint32_t kEp0sz = 64UL;  

class UsbDev {
public: 
    enum class EpType { Ctrl, Iso, Bulk, Interrupt };
    // Default values for Ep0
    struct EpConfig {
        ftVoidU32 cb_out_transfer_end = nullptr;
        ftVoidVoid cb_in_transfer_end = nullptr;
        EpType type = EpType::Ctrl;
        uint32_t out_pkt_sz_max = kEp0sz; // FS: up to 1023 bytes, HS: up to 1024 bytes. Must be 0 if not used
        uint32_t in_pkt_sz_max = kEp0sz;  // FS: up to 1023 bytes, HS: up to 1024 bytes. Must be 0 if not used
        uint32_t in_multiplier = 1;
    };

private: 
    // const uint32_t kEpCnt;
    USB_Type *pusb;
    enum class UsbSta { Stop, Ready, Selected, Active, Suspended };
    UsbSta usb_state = UsbSta::Stop, saved_state = UsbSta::Stop;
    uint32_t address = 0; // Assigned USB address
    uint8_t configuration = 0; // Current configuration
#if USB_REMOTE_WKUP_EN == 1U
    uint16_t status; // Contains wkup bit and self-powered bit
#endif
    
#pragma region // ==== Mem control ====
    // RX FIFO
    void RxFifoFlush();
    void RxFifoReadToBuf(uint8_t *buf, uint32_t n, uint32_t max);
    void RxFifoReadToFunc(ftVoidU8 Put, uint32_t n);
    // TX FIFO
    uint32_t txfifo_ptr_next; // Pointer to the next address in the packet memory
    void TxFifoReset() { txfifo_ptr_next = USB_RX_FIFO_SZ32; }
    void TxFifoFlush(uint32_t fifo);
    uint32_t TxFifoAllocate(uint32_t size);
    void TxFifoFill(uint32_t ep);
#pragma endregion // Mem control

protected:
#pragma region // ==== Setup pkt ====
#pragma pack(push, 1)
    union SetupPkt {
        uint64_t qw64;
        uint8_t buf8[8];
        struct {
            union {                   // [0]
                uint8_t bmRequestType;
                struct { // From LSB to MSB
                    uint8_t recipient: 5;
                    uint8_t type: 2;
                    uint8_t direction: 1;
                };
            };
            uint8_t bRequest;         // [1]
            union {
                uint16_t wValue;      // [2;3]
                struct { // From LSB to MSB
                    uint8_t dsc_index; // [2]
                    uint8_t dsc_type;  // [3]
                };
                uint8_t cfgNumber;    // [2]
                uint8_t epFeatureSelector; // [2]
            };
            union {
                uint16_t wIndex;      // [4;5]
                struct { // From LSB to MSB
                    uint8_t number : 4;
                    uint8_t __reserved46 : 3;
                    uint8_t dir : 1;
                } ep;
            };
            uint16_t wLength;   // [6;7]
        };
    };
    #pragma pack(pop)
    SetupPkt setup_pkt;

    static const uint32_t kReqDirHost2Dev = 0UL;
    static const uint32_t kReqDirDev2Host = 1UL;

    static const uint32_t kReqRecpntDevice = 0UL;
    static const uint32_t kReqRecpntInterface = 1UL;
    static const uint32_t kReqRecpntEndpoint = 2UL;
    static const uint32_t kReqRecpntOther = 3UL;

    static const uint32_t kReqTypeStd = 0UL;
    static const uint32_t kReqTypeClass = 1UL;
    static const uint32_t kReqTypeVendor = 2UL;
#pragma endregion

    // Callbacks. Must be implemented at higher level.
    enum class Evt { Reset = 0, Address = 1, Configured = 2, Unconfigured = 3, Suspend = 4, Wakeup = 5, Stalled = 6 };
    void EventCallback(Evt event);
#if USB_SOF_CB_EN
    void SOFCallback();
#endif
    #pragma region // ==== Endpoints ====
    // const uint32_t kEpCnt;
    enum class EpSta { DISABLED, STALLED, ACTIVE };
    uint32_t ep_transmitting_mask = 0, ep_receiving_mask = 0;

    // ==== Ep Buffers ====
    struct EpState {
        uint32_t sz = 0;         // Requested transmit transfer size
        uint32_t cnt = 0;        // Transmitted bytes so far
        uint8_t *pbuf = nullptr; // Pointer to the transmission linear buffer
        uint32_t total_sz = 0;   // Total transmit transfer size
    };

    struct EpInfo {
        const EpConfig* cfg;
        EpState state_in, state_out;
    };
    EpInfo ep_info[USB_EP_CNT+1UL]; // Include Ep0

    void CallOutTransferEndCallback(uint32_t ep, uint32_t sz) {
        ftVoidU32 cb = ep_info[ep].cfg->cb_out_transfer_end;
        if(cb) cb(sz);
    }
    void CallInTransferEndCallback(uint32_t ep) {
        ftVoidVoid cb = ep_info[ep].cfg->cb_in_transfer_end;
        if(cb) cb();
    }
    void StallIn (uint32_t ep) { pusb->ie[ep].DIEPCTL |=  DIEPCTL_STALL; }
    void StallOut(uint32_t ep) { pusb->oe[ep].DOEPCTL |=  DOEPCTL_STALL; }
    void ClearIn (uint32_t ep) { pusb->ie[ep].DIEPCTL &= ~DIEPCTL_STALL; }
    void ClearOut(uint32_t ep) { pusb->oe[ep].DOEPCTL &= ~DOEPCTL_STALL; }
    EpSta GetStatusIn(uint32_t ep);
    EpSta GetStatusOut(uint32_t ep);
    
    void StartInTransfer(uint32_t ep); 
    void StartOutTransfer(uint32_t ep);
    void EpDisableAll();
    void DisableEndpointsI();

    class Ep0 {
    public:
        enum class Sta {
            STP_WAITING,        // Waiting for SETUP data
            IN_TX,              // Transmitting
            IN_WAITING_TX0,     // Waiting transmit 0
            IN_SENDING_STS,     // Sending status
            OUT_WAITING_STS,    // Waiting status
            OUT_RX,             // Receiving
            ERROR               // Error, EP0 stalled
        } state = Sta::STP_WAITING;

        union {
            uint64_t setup_buf_qw64;
            uint8_t setup_buf[8];
        };
        uint8_t *ptr_next = nullptr;
        uint32_t transfer_len = 0;
        ftVoidVoid end_transaction_callback = nullptr;
        void PrepareSetupTransfer(uint8_t *pbuf, uint32_t len, ftVoidVoid aend_transaction_callback) {
            ptr_next = pbuf;
            transfer_len = len;
            end_transaction_callback = aend_transaction_callback;
        }
    } ep0;

    void Ep0SetupPktCallback();
    void Ep0InCallback();
    void Ep0OutCallback();
    void Ep0Reset();
    #pragma endregion

    void SetAddress();
    void ResetCore();
    void StartCore();
    void StopCore();
    inline void ConnectBus()    { pusb->DCTL &= ~DCTL_SD; }
    inline void DisconnectBus() { pusb->DCTL |=  DCTL_SD; }
    inline void DisableGlobalIRQs() { pusb->GAHBCS &= ~GAHBCS_GINTEN; }
    inline void EnableGlobalIRQs()  { pusb->GAHBCS |=  GAHBCS_GINTEN; }

    // IRQs
    void OnIrqReset();
    void OnIrqWakeup();
    void OnIrqSuspend();
    void OnIrqIsoInFailed();
    void OnIrqIsoOutFailed();
    void OnIrqRxFifoNotEmpty();
    void OnIrqEpOut(uint32_t ep);
    void OnIrqEpIn(uint32_t ep);

    // Call this when USB is configured: from EventCallback
    void InitEp(uint32_t ep, const EpConfig *pcfg);

    // Try to process default setup request
    retv DefaultRequestHandler();
    // Setup request callback: returns OK if request processed, or !Ok if standard processing required
    retv SetupReqHookCallback(uint8_t **ppbuf, uint32_t *psz, ftVoidVoid *callback_end_transfer);

    // Getting descriptors. Must be implemented in descriptors_xxx.cpp
    Buf_t GetDescriptor(uint8_t dtype,  uint8_t dindex, uint16_t lang);
public:
    // Don't forget to setup USB clock = 48MHz
    // UsbDev(USB_Type *apusb, const uint32_t aep_cnt): pusb(apusb), kEpCnt(aep_cnt) {}
    UsbDev(USB_Type *apusb): pusb(apusb) {}
    void Init();
    void Connect();
    void Disconnect();
    bool IsActive() { return usb_state == UsbSta::Active; }
    void ProcessIrq();
    // Endpoints transfer starting
    void StartReceiveI(uint32_t ep, uint8_t *pbuf, uint32_t max_sz);
    void StartTransmitI(uint32_t ep, uint8_t *pbuf, uint32_t sz);
    void StartTransmit(uint32_t ep, uint8_t *pbuf, uint32_t sz);
    // Endpoints transfer status
    bool IsEpTransmitting(uint32_t ep) { return ep_transmitting_mask & (1UL << ep); }
    bool IsEpReceiving(uint32_t ep)    { return ep_receiving_mask    & (1UL << ep); }
}; // class

/* ==== Put it somewhere ===
extern "C"
void USB_IRQ_HANDLER() {
    Sys::IrqPrologue();
    ProcessIrq();
    Sys::IrqEpilogue();
}
*/

#endif /* USB_DEV_H_ */
