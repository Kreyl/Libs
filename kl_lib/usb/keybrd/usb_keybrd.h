/*
 * usb_audio.h
 *
 *  Created on: 05 сент. 2015 г.
 *      Author: Kreyl
 */

#ifndef USB_USB_KEYBRD_H_
#define USB_USB_KEYBRD_H_

#include "HIDClassCommon.h"
#include "kl_buf.h"

// Size of buffer for reports
#define USB_KBRD_REPBUF_CNT		7

class UsbKBrd_t {
private:
public:
    void Init();
    void Connect();
    // Data
    void PressKey(uint8_t KeyCode);
    void DepressKey(uint8_t KeyCode);
    void PressAndRelease(uint8_t KeyCode);
    // Inner use
    uint8_t Protocol = 1;	// When initialized, all devices default to report protocol
    uint8_t IdleRate = 0;
	CircBuf_t<USB_KeyboardReport_Data_t, USB_KBRD_REPBUF_CNT> IReports;
    USB_KeyboardReport_Data_t LastReport;
    void ISendInReportI();	// Send report through IN endpoint
};

extern UsbKBrd_t UsbKBrd;

#endif /* USB_USB_KEYBRD_H_ */
