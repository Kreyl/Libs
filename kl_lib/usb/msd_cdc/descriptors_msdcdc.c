/*
 * descriptors_cdc.c
 *
 *  Created on: 03 ñåíò. 2015 ã.
 *      Author: Kreyl
 */

#include <descriptors_msdcdc.h>
#include <inttypes.h>
#include <stddef.h>


#if 1 // ==== Constants (not to change) ====
#define EP_DIR_IN           0x80
#define EP_DIR_OUT          0x00
// Descriptor-type endpoint codes
#define EP_TYPE_CONTROL     0x00
#define EP_TYPE_ISOCHRONOUS 0x01
#define EP_TYPE_BULK        0x02
#define EP_TYPE_INTERRUPT   0x03
#endif

#if 1 // ==== USB Device Descriptor ====
static const uint8_t device_descriptor_data[18] = {
  USB_DESC_DEVICE       (0x0110,        /* bcdUSB (1.1).                    */
                         0xEF,          /* bDeviceClass                     */
                         0x02,          /* bDeviceSubClass.                 */
                         0x01,          /* bDeviceProtocol.                 */
                         64,            /* bMaxPacketSize.                  */
                         0x0483,        /* idVendor (ST).                   */
                         0x374C,        /* idProduct.                       */
                         0x0200,        /* bcdDevice.                       */
                         1,             /* iManufacturer.                   */
                         2,             /* iProduct.                        */
                         3,             /* iSerialNumber.                   */
                         1)             /* bNumConfigurations.              */
};

// Device Descriptor wrapper.
static const USBDescriptor device_descriptor = {
  sizeof device_descriptor_data,
  device_descriptor_data
};
#endif

#if 1 // ==== Configuration Descriptor tree ====
#define USB_CONFIG_POWER_MA(mA) ((mA) >> 1)
#define CS_INTERFACE            0x24    // Descriptor type

static const uint8_t configuration_descriptor_data[] = {
    // ==== Header ====
    USB_DESC_CONFIGURATION(98,          /* wTotalLength.                    */
                         3,             /* bNumInterfaces.                  */
                         1,             /* bConfigurationValue.             */
                         0,             /* iConfiguration.                  */
                         0x80,          /* bmAttributes (bus powered).     */
                         USB_CONFIG_POWER_MA(100)   // bMaxPower
                         ),

    // ==== Mass Storage Interface ====
    USB_DESC_BYTE         (9),    // bLength
    USB_DESC_BYTE         (0x04), // bDescriptorType = interface descriptor
    USB_DESC_BYTE         (0),    // bInterfaceNumber
    USB_DESC_BYTE         (0),    // bAlternateSetting
    USB_DESC_BYTE         (2),    // bNumEndpoints
    USB_DESC_BYTE         (0x08), // bInterfaceClass = Mass Storage class
    USB_DESC_BYTE         (0x06), // bInterfaceSubClass = SCSI Transparent Command Set subclass of the Mass storage class
    USB_DESC_BYTE         (0x50), // bInterfaceProtocol = Bulk Only Transport protocol of the Mass Storage class
    USB_DESC_BYTE         (0),    // iInterface = No descriptor
    // MS_DataOutEndpoint
    USB_DESC_BYTE        (0x07),  // bLength
    USB_DESC_BYTE        (0x05),  // bDescriptorType
    USB_DESC_BYTE        (EP_DIR_OUT | EP_MSD_OUT_ID), // bEndpointAddress
    USB_DESC_BYTE        (EP_TYPE_BULK), // bmAttributes
    USB_DESC_WORD        (EP_BULK_SZ), // wMaxPacketSize
    USB_DESC_BYTE        (0),     // bInterval
    // MS_DataInEndpoint
    USB_DESC_BYTE        (0x07),  // bLength
    USB_DESC_BYTE        (0x05),  // bDescriptorType
    USB_DESC_BYTE        (EP_DIR_IN | EP_MSD_IN_ID), // bEndpointAddress
    USB_DESC_BYTE        (EP_TYPE_BULK), // bmAttributes
    USB_DESC_WORD        (EP_BULK_SZ), // wMaxPacketSize
    USB_DESC_BYTE        (0),     // bInterval

    // ==== CDC IAD ====
    USB_DESC_BYTE         (8),    // bLength
    USB_DESC_BYTE         (0x0B), // bDescriptorType = IAD
    USB_DESC_BYTE         (1),    // bFirstInterface
    USB_DESC_BYTE         (2),    // bInterfaceCount
    USB_DESC_BYTE         (0x02), // bFunctionClass
    USB_DESC_BYTE         (0x02), // bFunctionSubClass
    USB_DESC_BYTE         (0x01), // bFunctionProtocol
    USB_DESC_BYTE         (0),    // iFunction

    // ==== CDC Control interface ====
    USB_DESC_BYTE         (9),    // bLength
    USB_DESC_BYTE         (0x04), // bDescriptorType = interface descriptor
    USB_DESC_BYTE         (1),    // bInterfaceNumber
    USB_DESC_BYTE         (0),    // bAlternateSetting
    USB_DESC_BYTE         (1),    // bNumEndpoints
    USB_DESC_BYTE         (0x02), // bInterfaceClass (Communications Interface Class, CDC section 4.2).
    USB_DESC_BYTE         (0x02), // bInterfaceSubClass (Abstract Control Model, CDC section 4.3)
    USB_DESC_BYTE         (0x01), // bInterfaceProtocol (AT commands, CDC section 4.4) @KL: 0x02? Protocol: V.25ter (AT commands). For compatibility with standard host drivers, a generic virtual COM-port device should specify the V.25ter protocol even if the device doesn’t use AT commands
    USB_DESC_BYTE         (0),    // iInterface
    // Header Functional Descriptor (CDC section 5.2.3)
    USB_DESC_BYTE         (5),            // bLength
    USB_DESC_BYTE         (CS_INTERFACE), // bDescriptorType (CS_INTERFACE)
    USB_DESC_BYTE         (0x00),         // bDescriptorSubtype (Header Functional Descriptor
    USB_DESC_BCD          (0x0110),       // bcdCDC
    // Call Management Functional Descriptor
    USB_DESC_BYTE         (5),            // bFunctionLength
    USB_DESC_BYTE         (CS_INTERFACE), // bDescriptorType (CS_INTERFACE)
    USB_DESC_BYTE         (0x01),         // bDescriptorSubtype (Call Management Functional Descriptor).
    USB_DESC_BYTE         (0x00),         // bmCapabilities (D0+D1)
    USB_DESC_BYTE         (2),            // bDataInterface: Interface 2 is for data
    // ACM Functional Descriptor
    USB_DESC_BYTE         (4),            // bFunctionLength
    USB_DESC_BYTE         (CS_INTERFACE), // bDescriptorType (CS_INTERFACE)
    USB_DESC_BYTE         (0x02),         // bDescriptorSubtype (Abstract Control Management Descriptor).
    USB_DESC_BYTE         (0x02),         // bmCapabilities
    // Union Functional Descriptor
    USB_DESC_BYTE         (5),            // bFunctionLength
    USB_DESC_BYTE         (CS_INTERFACE), // bDescriptorType (CS_INTERFACE)
    USB_DESC_BYTE         (0x06),         // bDescriptorSubtype (Union Functional Descriptor)
    USB_DESC_BYTE         (1),            // bMasterInterface (Communication Class Interface)
    USB_DESC_BYTE         (2),            // bSlaveInterface0 (Data Class Interface)
    // INT Endpoint Descriptor
    USB_DESC_BYTE         (7),            // bLength
    USB_DESC_BYTE         (0x05),         // bDescriptorType
    USB_DESC_BYTE         (EP_DIR_IN | EP_CDC_INTERRUPT), // bEndpointAddress
    USB_DESC_BYTE         (EP_TYPE_INTERRUPT), // bmAttributes
    USB_DESC_WORD         (EP_INTERRUPT_SZ), // wMaxPacketSize
    USB_DESC_BYTE         (0xFF),         // bInterval

    // ==== CDC Data Interface ====
    USB_DESC_BYTE         (9),    // bLength
    USB_DESC_BYTE         (0x04), // bDescriptorType = interface descriptor
    USB_DESC_BYTE         (2),    // bInterfaceNumber
    USB_DESC_BYTE         (0),    // bAlternateSetting
    USB_DESC_BYTE         (2),    // bNumEndpoints
    USB_DESC_BYTE         (0x0A), // bInterfaceClass (Data Class Interface, CDC section 4.5)
    USB_DESC_BYTE         (0x00), // bInterfaceSubClass (CDC section 4.6)
    USB_DESC_BYTE         (0x00), // bInterfaceProtocol (CDC section 4.7)
    USB_DESC_BYTE         (0),    // iInterface
    // OUT Endpoint Descriptor
    USB_DESC_BYTE        (7),     // bLength
    USB_DESC_BYTE        (0x05),  // bDescriptorType
    USB_DESC_BYTE        (EP_DIR_OUT | EP_CDC_DATA_OUT), // bEndpointAddress
    USB_DESC_BYTE        (EP_TYPE_BULK), // bmAttributes
    USB_DESC_WORD        (EP_BULK_SZ), // wMaxPacketSize
    USB_DESC_BYTE        (0),     // bInterval
    // IN Endpoint Descriptor
    USB_DESC_BYTE        (7),     // bLength
    USB_DESC_BYTE        (0x05),  // bDescriptorType
    USB_DESC_BYTE        (EP_DIR_IN | EP_CDC_DATA_IN), // bEndpointAddress
    USB_DESC_BYTE        (EP_TYPE_BULK), // bmAttributes
    USB_DESC_WORD        (EP_BULK_SZ), // wMaxPacketSize
    USB_DESC_BYTE        (0),     // bInterval
};

// Configuration Descriptor wrapper.
static const USBDescriptor configuration_descriptor = {
  sizeof configuration_descriptor_data,
  configuration_descriptor_data
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
#define LANG_DESC_LEN       USB_STRING_LEN(2)
static const StringDescriptor_t LanguageString = {
        bLength: LANG_DESC_LEN,
        bDescriptorType: USB_DESCRIPTOR_STRING,
        bString: {0x09, 0x04}   // == 0409
};

// Vendor string
#define VENDOR_DESC_LEN     USB_STRING_LEN(9)
static const StringDescriptor_t ManufacturerString = {
        bLength: VENDOR_DESC_LEN,
        bDescriptorType: USB_DESCRIPTOR_STRING,
        bString: {'W','a','r','S','a','b','e','r', 's'}
};

// Device Description string
#define PRODUCT_DESC_LEN    USB_STRING_LEN(11)
static const StringDescriptor_t ProductString = {
        bLength: PRODUCT_DESC_LEN,
        bDescriptorType: USB_DESCRIPTOR_STRING,
        bString: {'S','a','b','e','r',' ','w',' ','C','D','C'}
};

// Serial number string
#define SERNUM_DESC_LEN     USB_STRING_LEN(5)
static const StringDescriptor_t SerialNumber = {
        bLength: SERNUM_DESC_LEN,
        bDescriptorType: USB_DESCRIPTOR_STRING,
        bString: {1, 2, 3, 4, 5}
};

// Strings wrappers array
static const USBDescriptor vcom_strings[] = {
        {LANG_DESC_LEN,    (uint8_t*)&LanguageString},
        {VENDOR_DESC_LEN,  (uint8_t*)&ManufacturerString},
        {PRODUCT_DESC_LEN, (uint8_t*)&ProductString},
        {SERNUM_DESC_LEN,  (uint8_t*)&SerialNumber}
};
#endif

/*
 * Handles the GET_DESCRIPTOR callback. All required descriptors must be
 * handled here.
 */

const USBDescriptor *GetDescriptor(USBDriver *usbp, uint8_t dtype, uint8_t dindex, uint16_t lang) {
    switch (dtype) {
        case USB_DESCRIPTOR_DEVICE:
            return &device_descriptor;
            break;
        case USB_DESCRIPTOR_CONFIGURATION:
            return &configuration_descriptor;
            break;
        case USB_DESCRIPTOR_STRING:
            if (dindex < 4) return &vcom_strings[dindex];
    } // switch
    return NULL;
}

