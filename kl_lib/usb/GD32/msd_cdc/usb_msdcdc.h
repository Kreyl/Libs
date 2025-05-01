#ifndef USB_MSD_CDC_H__
#define USB_MSD_CDC_H__

#include "shell.h"

#ifndef MSD_READ_ONLY
#define MSD_READ_ONLY   FALSE
#endif

#define MSD_TIMEOUT_MS   2700
#define MSD_DATABUF_SZ   4096

class UsbMsdCdc : public Shell {
private:
    void IStartTransmissionIfNotYetI(); // Required for printf implementation
    retv IPutCharI(char c);             // Required for printf implementation
public:
    void Init();
    void Reset();
    void Connect();
    void Disconnect();
    bool IsActive();
    retv TryParseRxBuff(); // Call this when something ip received
    retv ReceiveFile(
        uint8_t *pbuf1, uint8_t *pbuf2,
        uint32_t buf_sz, uint32_t total_sz,
        uint32_t timeout_ms, ftRetvPU8U32 buf_end_callback);
    retv TransmitBinaryFromBuf(uint8_t *ptr, uint32_t Len, uint32_t Timeout_ms);
};

extern UsbMsdCdc usb_msd_cdc;

#endif // USB_MSD_CDC_H__
