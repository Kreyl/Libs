/*
 * usb_cdc.cpp
 *
 *  Created on: 03 сент. 2015 г.
 *      Author: Kreyl
 */

#include "descriptors_keybrd.h"
#include "usb_keybrd.h"
#include "ch.h"
#include "hal.h"
#include "usb.h"
#include "usb_lld.h"
#include "main.h"
#include "kl_usb_defins.h"

UsbKBrd_t UsbKBrd;

static bool OnSetupPkt(USBDriver *usbp);
static void OnUsbEvent(USBDriver *usbp, usbevent_t event);
static uint8_t DummyByte;

const USBDescriptor *pDesc;

#if 1 // ========================== Endpoints ==================================
// ==== EP1 ====
static USBInEndpointState ep1Instate;

static void EpInCallback(USBDriver *usbp, usbep_t ep) {
	chSysLockFromISR();
	UsbKBrd.ISendInReportI();
	chSysUnlockFromISR();
}

// EP1 initialization structure (both IN and OUT).
static const USBEndpointConfig ep1config = {
    USB_EP_MODE_TYPE_INTR,
    NULL,                   // setup_cb
	EpInCallback,      		// in_cb
    NULL,                   // out_cb
    64,                     // in_maxsize
    64,                     // out_maxsize
    &ep1Instate,            // in_state
    NULL,                   // out_state
    2,                      // in_multiplier: Determines the space allocated for the TXFIFO as multiples of the packet size
    NULL                    // setup_buf: Pointer to a buffer for setup packets. Set this field to NULL for non-control endpoints
};

#endif

#if 1 // ======================== Events & Config ==============================
// ==== USB driver configuration ====
static const USBConfig UsbCfg = {
	OnUsbEvent,         // This callback is invoked when an USB driver event is registered
    GetDescriptor,      // Device GET_DESCRIPTOR request callback
    OnSetupPkt,         // This hook allows to be notified of standard requests or to handle non standard requests
    NULL                // Start Of Frame callback
};

void OnUsbEvent(USBDriver *usbp, usbevent_t event) {
//	Uart.PrintfI("USB evt=%X\r", event);
    switch (event) {
        case USB_EVENT_RESET:
            return;
        case USB_EVENT_ADDRESS:
            return;
        case USB_EVENT_CONFIGURED:
            chSysLockFromISR();
            /* Enable the endpoints specified in the configuration.
            Note, this callback is invoked from an ISR so I-Class functions must be used.*/
            usbInitEndpointI(usbp, EP_DATA_IN_ID, &ep1config);
            App.SignalEvtI(EVTMSK_USB_READY);
            chSysUnlockFromISR();
            return;
        case USB_EVENT_SUSPEND:
            chSysLockFromISR();
            App.SignalEvtI(EVTMSK_USB_SUSPEND);
            chSysUnlockFromISR();
            return;
        case USB_EVENT_WAKEUP:
            return;
        case USB_EVENT_STALLED:
            return;
    } // switch
}

#endif

/* ==== Setup Packet handler ====
 * true         Message handled internally.
 * false        Message not handled. */
#define REQDIR_DEVICETOHOST        (1 << 7)
#define REQTYPE_CLASS              (1 << 5)
#define REQREC_INTERFACE           (1 << 0)

bool OnSetupPkt(USBDriver *usbp) {
    SetupPkt_t *Setup = (SetupPkt_t*)usbp->setup;
//    Uart.PrintfI("\rSetup: %A", usbp->setup, 8, ' ');
    if(Setup->ReqType.Type == TYPE_CLASS) {
//    	Uart.PrintfI("\rSetup: %A", usbp->setup, 8, ' ');
    	switch(Setup->bRequest) {
    		// This request is mandatory and must be supported by all devices
    		case HID_REQ_GetReport:
    			// The wValue field specifies the Report Type in the high byte and the Report ID in the low byte
    			if(Setup->wValueMSB == 1) {	// 1 == Input
    				usbSetupTransfer(usbp, (uint8_t*)&UsbKBrd.LastReport, USB_KEYBRD_REPORT_SZ, NULL);
    				return true;
    			}
    			break;
    		case HID_REQ_SetReport:
    		    if(Setup->wLength == 1) {
    		        usbSetupTransfer(usbp, &DummyByte, 1, NULL);    // Just receive it doing nothing else
    		        return true;
    		    }
    		    break;

    		// This request is required only for boot devices
    		case HID_REQ_GetProtocol:
    			// The Get_Protocol request reads which protocol is currently active (either the boot
    			// protocol or the report protocol)
    			usbSetupTransfer(usbp, &UsbKBrd.Protocol, 1, NULL);
    			return true;
    			break;
    		// This request is required only for boot devices
    		case HID_REQ_SetProtocol:
    			UsbKBrd.Protocol = Setup->wValue;
    			return true;
    			break;

    		case HID_REQ_GetIdle:
    			usbSetupTransfer(usbp, &UsbKBrd.IdleRate, 1, NULL);
    			return true;
				break;
    		case HID_REQ_SetIdle:
    			UsbKBrd.IdleRate = Setup->wValueMSB;
    			return true;
    			break;
    	} // switch
    } // if class

    // GetDescriptor for HID class
    if(Setup->ReqType.Direction == DIR_DEV2HOST and
       Setup->ReqType.Type == TYPE_STANDARD and
       Setup->ReqType.Recipient == RCPT_INTERFACE and
       Setup->bRequest == USB_REQ_GET_DESCRIPTOR) {
    	// The low byte is the Descriptor Index used to specify the set for Physical
    	// Descriptors, and is reset to zero for other HID class descriptors
    	if(Setup->wValueLSB == 0) {
    		pDesc = usbp->config->get_descriptor_cb(usbp, Setup->wValueMSB, 0, Setup->wIndex);
			if(pDesc != NULL) {
				usbSetupTransfer(usbp, (uint8_t*)pDesc->ud_string, pDesc->ud_size, NULL);
				return true;
			}
		}
    }
    return false;
}

void UsbKBrd_t::Init() {
    PinSetupAnalog(GPIOA, 11);
    PinSetupAnalog(GPIOA, 12);
    // Objects
    usbInit();
}

void UsbKBrd_t::Connect() {
    usbDisconnectBus();
    chThdSleepMilliseconds(1008);
    usbStart(&USBD1, &UsbCfg);
    usbConnectBus();
}

void UsbKBrd_t::ISendInReportI() {
	if(usbGetDriverStateI(&USBDrv) != USB_ACTIVE) return;
	if(usbGetTransmitStatusI(&USBDrv, EP_DATA_IN_ID)) return;	// Endpoint busy

	USB_KeyboardReport_Data_t *p = nullptr;
	if(IReports.GetPAndMove(&p) == EMPTY) {
		// If should repeat, but nothing new in buffer get last report to send
		if(IdleRate != 0) IReports.GetLastP(&p);
	}
	// Send report if there are changes or should send repeatedly
	if(p != nullptr) {
		usbStartTransmitI(&USBDrv, EP_DATA_IN_ID, (uint8_t*)p, USB_KEYBRD_REPORT_SZ);
	}
}

#if 1 // ========================= High level ===============================
void UsbKBrd_t::PressKey(uint8_t KeyCode) {
	chSysLock();
	// Check if already pressed
	bool AlreadyPressed = false;
	uint8_t EmptyPosition = 0xFF;
	for(uint8_t i=0; i<6; i++) {
		if(LastReport.KeyCode[i] == KeyCode) {
			AlreadyPressed = true;
			break;
		}
		// Find empty slot
		else if(LastReport.KeyCode[i] == 0 and EmptyPosition == 0xFF) EmptyPosition = i;
	}
	if(!AlreadyPressed and EmptyPosition < 6) {
		LastReport.KeyCode[EmptyPosition] = KeyCode;
		IReports.Put(&LastReport);
		ISendInReportI();
	}
	chSysUnlock();
}
void UsbKBrd_t::DepressKey(uint8_t KeyCode) {
	chSysLock();
	for(uint8_t i=0; i<6; i++) {
		if(LastReport.KeyCode[i] == KeyCode) {
			LastReport.KeyCode[i] = 0;
			IReports.Put(&LastReport);
			ISendInReportI();
			break;
		}
	}
	chSysUnlock();
}

void UsbKBrd_t::PressAndRelease(uint8_t KeyCode) {
	PressKey(KeyCode);
	DepressKey(KeyCode);
}
#endif
