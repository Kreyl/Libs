/*
 * descriptors_audio.c
 *
 *  Created on: 04 09 2015 ã.
 *      Author: Kreyl
 */

#include "descriptors_keybrd.h"
#include "HIDReportData.h"
#include "board.h"
#include "HIDClassCommon.h"

#if 1 // ==== Constants (not to change) ====
#define EP_DIR_IN           0x80
#define EP_DIR_OUT          0x00
// Descriptor-type endpoint codes
#define EP_TYPE_CONTROL     0x00
#define EP_TYPE_ISOCHRONOUS 0x01
#define EP_TYPE_BULK        0x02
#define EP_TYPE_INTERRUPT   0x03

// HID
#define USB_DESCRIPTOR_HID	0x21
#define USB_DESCRIPTOR_HID_REPORT	0x22
#endif

#define VERSION_BCD(Major, Minor, Revision) \
		(uint16_t)( ((Major & 0xFF) << 8) | ((Minor & 0x0F) << 4) | (Revision & 0x0F) )

#if 1 // ==== USB Device Descriptor ====
static const uint8_t DeviceDescriptorData[18] = {
  USB_DESC_DEVICE       (0x0200,        // bcdUSB (2.0).
                         0x00,          // bDeviceClass: Device defined at Interface level
                         0x00,          // bDeviceSubClass: Unused
                         0x00,          // bDeviceProtocol: Unused
                         64,            // bMaxPacketSize.
                         USBD_VID,      // idVendor (ST).
                         USBD_PID,      // idProduct.
						 VERSION_BCD(2,0,0), // bcdDevice.
                         1,             // iManufacturer.
                         2,             // iProduct.
                         3,             // iSerialNumber.
                         1)             // bNumConfigurations.
};

// Device Descriptor wrapper
static const USBDescriptor DeviceDescriptor = {
  sizeof DeviceDescriptorData,
  DeviceDescriptorData
};
#endif

#if 1 // ==== Configuration Descriptor tree ====
/** HID class report descriptor. This is a special descriptor constructed with values from the
 *  USBIF HID class specification to describe the reports and capabilities of the HID device. This
 *  descriptor is parsed by the host and its contents used to determine what data (and in what encoding)
 *  the device will send, and what it may be sent back from the host. Refer to the HID specification for
 *  more details on HID report descriptors.
 */
static const uint8_t KeyboardReport[] = {
		HID_DESCRIPTOR_KEYBOARD(6)
};

#define USB_CONFIG_POWER_MA(mA) ((mA) >> 1)
#define USB_DESC_TRIBYTE(w)             \
        (uint8_t)((w) & 255U),          \
        (uint8_t)(((w) >> 8)  & 255U),  \
        (uint8_t)(((w) >> 16) & 255U)

#define CFG_TOTAL_LEN           34
static const uint8_t ConfigurationDescriptorData[] = {
    // ==== Configuration Descriptor ====
    USB_DESC_CONFIGURATION(CFG_TOTAL_LEN, // wTotalLength
                         1,             // bNumInterfaces
                         1,             // bConfigurationNumber
                         0,             // iConfiguration
                         0x80,          // bmAttributes (bus powered)
                         USB_CONFIG_POWER_MA(100)   // bMaxPower
                         ),

    // === HID Interface Descriptor ====
    USB_DESC_INTERFACE  (0,             // bInterfaceNumber
                         0,             // bAlternateSetting.
                         1,             // bNumEndpoints.
						 0x03,          // bInterfaceClass: HID
                         0x01,          // bInterfaceSubClass: Boot Interface Subclass
                         0x01,          // bInterfaceProtocol: Keyboard
                         0),            // iInterface

	// ==== HID Descriptor ====
	USB_DESC_BYTE        (0x09),	            // bLength
	USB_DESC_BYTE        (USB_DESCRIPTOR_HID),	// bDescriptorType (HID class HID descriptor)
	USB_DESC_WORD		 (VERSION_BCD(1,1,1)),  // bcdHID: HID Class	Specification release
	USB_DESC_BYTE        (0), 		// bCountryCode
	USB_DESC_BYTE        (1), 		// bNumDescriptors: number of class descriptors (always at least one i.e. Report descriptor)
	USB_DESC_BYTE        (0x22),	// bDescriptorType: HID report descriptor
	USB_DESC_WORD		 (sizeof(KeyboardReport)), // wDescriptorLength: total size of the Report descriptor

    // ==== Standard Endpoint Descriptor ====
	USB_DESC_ENDPOINT     (
		  (EP_DATA_IN_ID | EP_DIR_IN),  // bEndpointAddress
		  EP_TYPE_INTERRUPT,       		// bmAttributes
		  EP_INTERRUPT_SZ,              // wMaxPacketSize
		  10),                          // bInterval, ms
};

// Configuration Descriptor wrapper.
static const USBDescriptor ConfigurationDescriptor = {
    sizeof ConfigurationDescriptorData,
    ConfigurationDescriptorData
};

static const USBDescriptor HIDDescriptor = {
		9,
		&ConfigurationDescriptorData[18]
};
static const USBDescriptor ReportDescriptor = {
		sizeof KeyboardReport,
		KeyboardReport
};

#endif

#if 1 // ==== Strings ====
//Macro to calculate the Unicode length of a string with a given number of Unicode characters
#define USB_STRING_LEN(UnicodeChars)      (2 + ((UnicodeChars) << 1))

typedef struct {
    uint8_t  bLength;                // Size of the descriptor, in bytes
    uint8_t  bDescriptorType;        // Type of the descriptor, given by the specific class
    uint16_t  bString[];
} __attribute__ ((__packed__)) StringDescriptor_t;

// U.S. English language identifier
static const uint8_t LanguageString[] = {
        USB_DESC_BYTE (4),      // bLength
        USB_DESC_BYTE (USB_DESCRIPTOR_STRING),  // bDescriptorType
        USB_DESC_WORD (0x0409)  // wLANGID (U.S. English)
};

// Vendor string
static const uint8_t VendorString[] = {
        USB_DESC_BYTE (USB_STRING_LEN(8)),
        USB_DESC_BYTE (USB_DESCRIPTOR_STRING),  // bDescriptorType
        USB_DESC_WORD('O'),
        USB_DESC_WORD('s'),
        USB_DESC_WORD('t'),
        USB_DESC_WORD('r'),
        USB_DESC_WORD('a'),
        USB_DESC_WORD('n'),
        USB_DESC_WORD('n'),
        USB_DESC_WORD('a'),
};

// Device Description string
static const uint8_t ProductString[] = {
        USB_DESC_BYTE (USB_STRING_LEN(8)),
        USB_DESC_BYTE (USB_DESCRIPTOR_STRING),  // bDescriptorType
        USB_DESC_WORD('K'),
        USB_DESC_WORD('e'),
        USB_DESC_WORD('y'),
        USB_DESC_WORD('b'),
        USB_DESC_WORD('o'),
        USB_DESC_WORD('a'),
        USB_DESC_WORD('r'),
        USB_DESC_WORD('d'),
};

// Serial number string
static const uint8_t SerialNumber[] = {
        USB_DESC_BYTE (USB_STRING_LEN(3)),
        USB_DESC_BYTE (USB_DESCRIPTOR_STRING),  // bDescriptorType
        USB_DESC_WORD('1'),
        USB_DESC_WORD('2'),
        USB_DESC_WORD('3'),
};


// Strings wrappers array
static const USBDescriptor DescStrings[] = {
        {sizeof LanguageString, LanguageString},
        {sizeof VendorString,   VendorString},
        {sizeof ProductString,  ProductString},
        {sizeof SerialNumber,   SerialNumber}
};
#endif

//extern void PrintfC(const char *format, ...);

// Handles the GET_DESCRIPTOR callback. All required descriptors must be handled here
const USBDescriptor *GetDescriptor(USBDriver *usbp, uint8_t dtype, uint8_t dindex, uint16_t lang) {
//    PrintfC("\rGetDsc: t=%u; i=%u", dtype, dindex);
    switch(dtype) {
        case USB_DESCRIPTOR_DEVICE:
            return &DeviceDescriptor;
            break;
        case USB_DESCRIPTOR_CONFIGURATION:
            return &ConfigurationDescriptor;
            break;
        case USB_DESCRIPTOR_STRING:
            if(dindex < 4) {
//                PrintfC("\rStr: %u  %A", DescStrings[dindex].ud_size, DescStrings[dindex].ud_string, DescStrings[dindex].ud_size, ' ');
                return &DescStrings[dindex];
            }
            break;
        case USB_DESCRIPTOR_HID:
        	return &HIDDescriptor;
        	break;
        case USB_DESCRIPTOR_HID_REPORT:
        	return &ReportDescriptor;
        	break;
    } // switch
    return NULL;
}

