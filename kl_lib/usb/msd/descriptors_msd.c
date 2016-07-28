/*
 * descriptors_cdc.c
 *
 *  Created on: 03 сент. 2015 г.
 *      Author: Kreyl
 */

#include <descriptors_msd.h>
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
                         0x00,          /* bDeviceClass                     */
                         0x00,          /* bDeviceSubClass.                 */
                         0x00,          /* bDeviceProtocol.                 */
                         64,            /* bMaxPacketSize.                  */
                         0x21BB,        /* idVendor (WWPass)                */
                         11    ,        /* idProduct.                       */
                         0x0001,        /* bcdDevice.                       */
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

#if 1 // ==== Configuration Descriptor tree for a CDC ====
#define USB_CONFIG_POWER_MA(mA) ((mA) >> 1)
#define CS_INTERFACE            0x24    // Descriptor type

static const uint8_t configuration_descriptor_data[] = {
  // ==== Header ====
  USB_DESC_CONFIGURATION(32,            /* wTotalLength.                    */
                         1,             /* bNumInterfaces.                  */
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
  USB_DESC_BYTE        (EP_DIR_OUT | EP_DATA_OUT_ID), // bEndpointAddress
  USB_DESC_BYTE        (EP_TYPE_BULK), // bmAttributes
  USB_DESC_WORD        (EP_BULK_SZ), // wMaxPacketSize
  USB_DESC_BYTE        (0),     // bInterval
  // MS_DataInEndpoint
  USB_DESC_BYTE        (0x07),  // bLength
  USB_DESC_BYTE        (0x05),  // bDescriptorType
  USB_DESC_BYTE        (EP_DIR_IN | EP_DATA_IN_ID), // bEndpointAddress
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
static const StringDescriptor_t LanguageString = {
        bLength: USB_STRING_LEN(2),
        bDescriptorType: USB_DESCRIPTOR_STRING,
        bString: {0x09, 0x04}   // == 0409
};

// Vendor string
static const StringDescriptor_t ManufacturerString = {
        bLength: USB_STRING_LEN(8),
        bDescriptorType: USB_DESCRIPTOR_STRING,
        bString: {'O','s','t','r','a','n','n','a'}
};

// Device Description string
static const StringDescriptor_t ProductString = {
        bLength: USB_STRING_LEN(12),
        bDescriptorType: USB_DESCRIPTOR_STRING,
        bString: {'M','a','s','s',' ','s','t','o','r','a','g','e'}
};

// Serial number string
static const StringDescriptor_t SerialNumber = {
        bLength: USB_STRING_LEN(3),
        bDescriptorType: USB_DESCRIPTOR_STRING,
        bString: {1, 2, 3}
};

// Strings wrappers array
static const USBDescriptor vcom_strings[] = {
        {sizeof LanguageString,     (uint8_t*)&LanguageString},
        {sizeof ManufacturerString, (uint8_t*)&ManufacturerString},
        {sizeof ProductString,      (uint8_t*)&ProductString},
        {sizeof SerialNumber,       (uint8_t*)&SerialNumber}
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

