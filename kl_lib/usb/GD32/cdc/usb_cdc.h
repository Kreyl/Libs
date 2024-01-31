#ifndef USB_CDC_H__
#define USB_CDC_H__

#include "shell.h"

class UsbCDC_t : public Shell_t {
private:
    void IStartTransmissionIfNotYet(); // Required for printf implementation
    retv IPutChar(char c);             // Required for printf implementation
public:
    void Connect();
    void Disconnect();
    bool IsActive();
    retv TryParseRxBuff(); // Call this when something ip received
    retv ReceiveBinaryToBuf(uint8_t *ptr, uint32_t Len, uint32_t Timeout_ms);
    retv TransmitBinaryFromBuf(uint8_t *ptr, uint32_t Len, uint32_t Timeout_ms);
};

extern UsbCDC_t UsbCDC;

#endif // USB_CDC_H__
