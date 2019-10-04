/*
 * usb_cdc.cpp
 *
 *  Created on: 03 сент. 2015 г.
 *      Author: Kreyl
 */

#include "hal.h"
#include "usb_cdc.h"
#include "hal_usb.h"
#include "descriptors_cdc.h"
#include "hal_serial_usb.h"
#include "MsgQ.h"

UsbCDC_t UsbCDC;
SerialUSBDriver SDU1;
static thread_t *PCdcThd;

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
        case USB_EVENT_CONFIGURED:
            chSysLockFromISR();
            /* Enable the endpoints specified in the configuration.
            Note, this callback is invoked from an ISR so I-Class functions must be used.*/
            usbInitEndpointI(usbp, EP_CDC_DATA_IN, &ep1config);
            usbInitEndpointI(usbp, EP_CDC_INTERRUPT, &ep2config);
            sduConfigureHookI(&SDU1);   // Resetting the state of the CDC subsystem
            EvtQMain.SendNowOrExitI(EvtMsg_t(evtIdUsbReady));    // Signal to main thread
            chSysUnlockFromISR();
            return;
        case USB_EVENT_SUSPEND:
        case USB_EVENT_WAKEUP:
        case USB_EVENT_STALLED:
        case USB_EVENT_UNCONFIGURED:
            return;
    } // switch
}

#endif

#if 1  // ==== USB driver configuration ====
const USBConfig UsbCfg = {
    usb_event,          // This callback is invoked when an USB driver event is registered
    GetDescriptor,      // Device GET_DESCRIPTOR request callback
    sduRequestsHook,    // This hook allows to be notified of standard requests or to handle non standard requests
    SOFHandler          // Start Of Frame callback
};

// Serial over USB driver configuration
const SerialUSBConfig SerUsbCfg = {
    &USBD1,                     // USB driver to use
    EP_CDC_DATA_IN,           // Bulk IN endpoint used for outgoing data transfer
    EP_CDC_DATA_OUT,          // Bulk OUT endpoint used for incoming data transfer
    EP_CDC_INTERRUPT  // Interrupt IN endpoint used for notifications
};
#endif

#if 1 // ========================== RX Thread ==================================
bool UsbCDC_t::IsActive() { return (SDU1.config->usbp->state == USB_ACTIVE); }

static THD_WORKING_AREA(waThdCDCRX, 128);
static THD_FUNCTION(ThdCDCRX, arg) {
    chRegSetThreadName("CDCRX");
    while(true) {
        if(UsbCDC.IsActive()) {
            msg_t m = SDU1.vmt->get(&SDU1);
            if(m > 0) {
//                SDU1.vmt->put(&SDU1, (uint8_t)m);   // repeat what was sent
                if(UsbCDC.Cmd.PutChar((char)m) == pdrNewCmd) {
                    chSysLock();
                    EvtQMain.SendNowOrExitI(EvtMsg_t(evtIdShellCmd, (Shell_t*)&UsbCDC));
                    chSchGoSleepS(CH_STATE_SUSPENDED); // Wait until cmd processed
                    chSysUnlock();  // Will be here when application signals that cmd processed
                }
            } // if >0
        } // if active
        else chThdSleepMilliseconds(540);
    } // while true
}

uint8_t UsbCDC_t::IPutChar(char c) {
    return (SDU1.vmt->putt(&SDU1, (uint8_t)c, TIME_MS2I(999)) == MSG_OK)? retvOk : retvFail;
}

void UsbCDC_t::SignalCmdProcessed() {
    chSysLock();
    if(PCdcThd->state == CH_STATE_SUSPENDED) chSchReadyI(PCdcThd);
    chSysUnlock();
}
#endif

void UsbCDC_t::Init() {
#if defined STM32L4XX || defined STM32F2XX
    PinSetupAlterFunc(USB_DM, omPushPull, pudNone, USB_AF, psHigh);
    PinSetupAlterFunc(USB_DP, omPushPull, pudNone, USB_AF, psHigh);
#else
    PinSetupAnalog(USB_DM);
    PinSetupAnalog(USB_DP);
#endif
    // Objects
    usbInit();
    sduObjectInit(&SDU1);
    sduStart(&SDU1, &SerUsbCfg);
    // RX thread
    PCdcThd = chThdCreateStatic(waThdCDCRX, sizeof(waThdCDCRX), NORMALPRIO, ThdCDCRX, NULL);
}

void UsbCDC_t::Connect() {
#if defined STM32F1XX
    // Disconnect everything
    PinSetupAnalog(USB_PULLUP);
    PinSetupAnalog(USB_DM);
    PinSetupAnalog(USB_DP);
#else
    usbDisconnectBus(SerUsbCfg.usbp);
#endif
    chThdSleepMilliseconds(99);
    usbStart(SerUsbCfg.usbp, &UsbCfg);
#if defined STM32F1XX
    PinSetupAlterFunc(USB_DM, omPushPull, pudNone, USB_AF, psHigh);
    PinSetupAlterFunc(USB_DP, omPushPull, pudNone, USB_AF, psHigh);
    PinSetHi(USB_PULLUP);
    PinSetupOut(USB_PULLUP, omPushPull);
#else
    usbConnectBus(SerUsbCfg.usbp);
#endif
}
void UsbCDC_t::Disconnect() {
    usbStop(SerUsbCfg.usbp);
#if defined STM32F1XX
    // Disconnect everything
    PinSetupAnalog(USB_PULLUP);
    PinSetupAnalog(USB_DM);
    PinSetupAnalog(USB_DP);
#else
    usbDisconnectBus(SerUsbCfg.usbp);
#endif
}
