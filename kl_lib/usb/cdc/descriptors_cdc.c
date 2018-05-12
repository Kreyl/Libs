/*
 * descriptors_cdc.c
 *
 *  Created on: 03 ñåíò. 2015 ã.
 *      Author: Kreyl
 */

#include "ch.h"
#include "hal.h"
#include "descriptors_cdc.h"


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
static const uint8_t vcom_device_descriptor_data[18] = {
  USB_DESC_DEVICE       (0x0110,        /* bcdUSB (1.1).                    */
                         0x02,          /* bDeviceClass (CDC).              */
                         0x00,          /* bDeviceSubClass.                 */
                         0x00,          /* bDeviceProtocol.                 */
                         64,            /* bMaxPacketSize.                  */
                         0x0483,        /* idVendor (ST).                   */
                         0x5740,        /* idProduct.                       */
                         0x0200,        /* bcdDevice.                       */
                         1,             /* iManufacturer.                   */
                         2,             /* iProduct.                        */
                         3,             /* iSerialNumber.                   */
                         1)             /* bNumConfigurations.              */
};

// Device Descriptor wrapper.
static const USBDescriptor vcom_device_descriptor = {
  sizeof vcom_device_descriptor_data,
  vcom_device_descriptor_data
};
#endif

#if 1 // ==== Configuration Descriptor tree for a CDC ====
#define USB_CONFIG_POWER_MA(mA) ((mA) >> 1)
#define CS_INTERFACE            0x24    // Descriptor type

static const uint8_t vcom_configuration_descriptor_data[67] = {
  /* Configuration Descriptor.*/
  USB_DESC_CONFIGURATION(67,            /* wTotalLength.                    */
                         0x02,          /* bNumInterfaces.                  */
                         0x01,          /* bConfigurationValue.             */
                         0,             /* iConfiguration.                  */
                         0x80,          /* bmAttributes (bus powered).     */
                         USB_CONFIG_POWER_MA(100)   // bMaxPower
                         ),
  // === CDC Control Interface ////
  USB_DESC_INTERFACE    (0x00,          /* bInterfaceNumber.                */
                         0x00,          /* bAlternateSetting.               */
                         0x01,          /* bNumEndpoints.                   */
                         0x02,          // bInterfaceClass (Communications Interface Class, CDC section 4.2).
                         0x02,          // bInterfaceSubClass (Abstract Control Model, CDC section 4.3)
                         0x01,          // bInterfaceProtocol (AT commands, CDC section 4.4) @KL: 0x02? Protocol: V.25ter (AT commands). For compatibility with standard host drivers, a generic virtual COM-port device should specify the V.25ter protocol even if the device doesn’t use AT commands
                         0),            /* iInterface.                      */
  /* Header Functional Descriptor (CDC section 5.2.3).*/
  USB_DESC_BYTE         (5),            /* bLength.                         */
  USB_DESC_BYTE         (CS_INTERFACE), // bDescriptorType (CS_INTERFACE)
  USB_DESC_BYTE         (0x00),         // bDescriptorSubtype (Header Functional Descriptor
  USB_DESC_BCD          (0x0110),       /* bcdCDC.                          */
  /* Call Management Functional Descriptor. */
  USB_DESC_BYTE         (5),            /* bFunctionLength.                 */
  USB_DESC_BYTE         (CS_INTERFACE), // bDescriptorType (CS_INTERFACE)
  USB_DESC_BYTE         (0x01),         // bDescriptorSubtype (Call Management Functional Descriptor).
  USB_DESC_BYTE         (0x00),         /* bmCapabilities (D0+D1).          */
  USB_DESC_BYTE         (0x01),         /* bDataInterface.                  */
  /* ACM Functional Descriptor.*/
  USB_DESC_BYTE         (4),            /* bFunctionLength.                 */
  USB_DESC_BYTE         (CS_INTERFACE), // bDescriptorType (CS_INTERFACE)
  USB_DESC_BYTE         (0x02),         // bDescriptorSubtype (Abstract Control Management Descriptor).
  USB_DESC_BYTE         (0x02),         /* bmCapabilities.                  */
  /* Union Functional Descriptor.*/
  USB_DESC_BYTE         (5),            /* bFunctionLength.                 */
  USB_DESC_BYTE         (CS_INTERFACE), // bDescriptorType (CS_INTERFACE)
  USB_DESC_BYTE         (0x06),         // bDescriptorSubtype (Union Functional Descriptor)
  USB_DESC_BYTE         (0x00),         // bMasterInterface (Communication Class Interface)
  USB_DESC_BYTE         (0x01),         // bSlaveInterface0 (Data Class Interface)
  /* Endpoint 2 Descriptor.*/
  USB_DESC_ENDPOINT     (
          (EP_CDC_INTERRUPT | EP_DIR_IN), // bEndpointAddress
          EP_TYPE_INTERRUPT,            // bmAttributes
          EP_INTERRUPT_SZ,              // wMaxPacketSize
          0xFF),                        // bInterval
  // ==== CDC Data Interface ====
  USB_DESC_INTERFACE    (0x01,          /* bInterfaceNumber.                */
                         0x00,          /* bAlternateSetting.               */
                         2,             /* bNumEndpoints.                   */
                         0x0A,          // bInterfaceClass (Data Class Interface, CDC section 4.5)
                         0x00,          // bInterfaceSubClass (CDC section 4.6)
                         0x00,          // bInterfaceProtocol (CDC section 4.7)
                         0x00),         /* iInterface.                      */
  /* Endpoint 3 Descriptor.*/
  USB_DESC_ENDPOINT     (
          (EP_CDC_DATA_OUT | EP_DIR_OUT), // bEndpointAddress
          EP_TYPE_BULK,                 // bmAttributes (Bulk)
          EP_BULK_SZ,                   // wMaxPacketSize
          0x00),                        // bInterval
  /* Endpoint 1 Descriptor.*/
  USB_DESC_ENDPOINT     (
          (EP_CDC_DATA_IN | EP_DIR_IN), // bEndpointAddress
          EP_TYPE_BULK,                 // bmAttributes (Bulk)
          EP_BULK_SZ,                   // wMaxPacketSize
          0x00)                         // bInterval
};

// Configuration Descriptor wrapper.
static const USBDescriptor vcom_configuration_descriptor = {
  sizeof vcom_configuration_descriptor_data,
  vcom_configuration_descriptor_data
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
        bLength: USB_STRING_LEN(11),
        bDescriptorType: USB_DESCRIPTOR_STRING,
        bString: {'V','i','r','t','u','a','l',' ','C','O','M'}
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
            return &vcom_device_descriptor;
            break;
        case USB_DESCRIPTOR_CONFIGURATION:
            return &vcom_configuration_descriptor;
            break;
        case USB_DESCRIPTOR_STRING:
            if (dindex < 4) return &vcom_strings[dindex];
    } // switch
    return NULL;
}

