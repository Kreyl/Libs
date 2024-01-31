#include "descriptors_cdc.h"
#include "usb.h"
#include "shell.h"

#if 1 // ==== Constants (do not touch) ====
#define EP_DIR_IN           0x80
#define EP_DIR_OUT          0x00
// Descriptor-type endpoint codes
#define EP_TYPE_CONTROL     0x00
#define EP_TYPE_ISOCHRONOUS 0x01
#define EP_TYPE_BULK        0x02
#define EP_TYPE_INTERRUPT   0x03

#define USB_DESCRIPTOR_DEVICE               1U
#define USB_DESCRIPTOR_CONFIGURATION        2U
#define USB_DESCRIPTOR_STRING               3U
#define USB_DESCRIPTOR_INTERFACE            4U
#define USB_DESCRIPTOR_ENDPOINT             5U
//#define USB_DESCRIPTOR_DEVICE_QUALIFIER     6U
//#define USB_DESCRIPTOR_OTHER_SPEED_CFG      7U
//#define USB_DESCRIPTOR_INTERFACE_POWER      8U
//#define USB_DESCRIPTOR_INTERFACE_ASSOCIATION 11U
#endif


#pragma pack(push, 1)
struct DscHeader_t {
    uint8_t  bLength;
    uint8_t  bDescriptorType;
};

#if 1 // ========================== Device Descriptor ==========================
static const struct DeviceDescriptor_t {
    DscHeader_t Hdr = { sizeof(DeviceDescriptor_t), USB_DESCRIPTOR_DEVICE };
    /* 0xJJMN where JJ is the major version number, M is the minor version number
       and N is the sub minor version number. e.g. USB 2.0 is 0x0200, USB 1.1 is 0x0110 */
    uint16_t bcdUSB          = 0x0110;
    uint8_t  bDeviceClass    = 0x02; // CDC
    uint8_t  bDeviceSubClass = 0x00;
    uint8_t  bDeviceProtocol = 0x00;
    uint8_t  bMaxPacketSize  = EP0_SZ; // Maximum Packet Size for Zero Endpoint
    uint16_t idVendor        = 0x0483; // ST
    uint16_t idProduct       = 0x5740;
    uint16_t bcdDevice       = 0x0200;
    uint8_t  iManufacturer   = 1;
    uint8_t  iProduct        = 2;
    uint8_t  iSerialNumber   = 3;
    uint8_t  bNumConfigurations = 1;
} DeviceDescriptor;
#endif

#if 1 // ================ Configuration Descriptor for a CDC ===================
#define USB_CONFIG_POWER_MA(mA) ((mA) >> 1)
#define CS_INTERFACE            0x24    // DescriptorType of HeaderFuncDesc
// There may be several configurations in single CfgDesc
struct Configuration_t {
    DscHeader_t Hdr = { sizeof(Configuration_t), USB_DESCRIPTOR_CONFIGURATION };
    uint16_t wTotalLength;
    uint8_t  bNumInterfaces;
    uint8_t  bConfigurationValue;
    uint8_t  iConfiguration;
    uint8_t  bmAttributes;
    uint8_t  bMaxPower;
};

struct Interface_t {
    DscHeader_t Hdr = { sizeof(Interface_t), USB_DESCRIPTOR_INTERFACE };
    uint8_t  bInterfaceNumber;
    uint8_t  bAlternateSetting;
    uint8_t  bNumEndpoints;
    uint8_t  bInterfaceClass;
    uint8_t  bInterfaceSubClass;
    uint8_t  bInterfaceProtocol;
    uint8_t  iInterface;
};

struct Endpoint_t {
    DscHeader_t Hdr = { sizeof(Endpoint_t), USB_DESCRIPTOR_ENDPOINT };
    uint8_t  bEndpointAddress;
    uint8_t  bmAttributes;
    uint16_t wMaxPacketSize;
    uint8_t  bInterval;
};

static const struct ConfigDescriptor_t {
    // ==== Configuration part ====
    Configuration_t Configuration = {
            .wTotalLength = sizeof(ConfigDescriptor_t),
            .bNumInterfaces = 2,
            .bConfigurationValue = 1,
            .iConfiguration = 0,
            .bmAttributes = 0x80, // Bus powered
            .bMaxPower = USB_CONFIG_POWER_MA(100)
    };

    // === CDC Control Interface ====
    Interface_t CdcControlInterface = {
            .bInterfaceNumber = 0,
            .bAlternateSetting = 0,
            .bNumEndpoints = 1,
            .bInterfaceClass = 0x02, // Communications Interface Class, CDC section 4.2
            .bInterfaceSubClass = 0x02, // Abstract Control Model, CDC section 4.3
            /* AT commands, CDC section 4.4. @KL: 0x02? Protocol: V.25ter (AT commands).
             * For compatibility with standard host drivers, a generic virtual COM-port device
             * should specify the V.25ter protocol even if the device doesn't use AT commands */
            .bInterfaceProtocol = 0x01,
            .iInterface = 0
    };
    struct HeaderFunctionalDescriptor_t { // CDC section 5.2.3
        DscHeader_t Hdr = { sizeof(HeaderFunctionalDescriptor_t), CS_INTERFACE };
        uint8_t  bDescriptorSubtype = 0x00;
        uint16_t bcdCDC = 0x0110;
    } HeaderFunctionalDescriptor;
    struct CallManagementFunctionalDescriptor_t {
        DscHeader_t Hdr = { sizeof(CallManagementFunctionalDescriptor_t), CS_INTERFACE };
        uint8_t  bDescriptorSubtype = 0x01; // Call Management Functional Descriptor
        uint8_t  bmCapabilities = 0x00; // (D0+D1)
        uint8_t  bDataInterface = 0x01;
    } CallManagementFunctionalDescriptor;
    struct ACMFunctionalDescriptor_t {
        DscHeader_t Hdr = { sizeof(ACMFunctionalDescriptor_t), CS_INTERFACE };
        uint8_t  bDescriptorSubtype = 0x02; // Abstract Control Management Descriptor
        uint8_t  bmCapabilitie = 0x02;
    } ACMFunctionalDescriptor;
    struct UnionFunctionalDescriptor_t {
        DscHeader_t Hdr = { sizeof(UnionFunctionalDescriptor_t), CS_INTERFACE };
        uint8_t  bDescriptorSubtype = 0x06; // Union Functional Descriptor
        uint8_t  bMasterInterface = 0x00; // Communication Class Interface
        uint8_t  bSlaveInterface = 0x01; // Data Class Interface
    } UnionFunctionalDescriptor;
    // Endpoint 2 Descriptor
    Endpoint_t Ep2 = {
            .bEndpointAddress = (EP_CDC_INTERRUPT | EP_DIR_IN),
            .bmAttributes = EP_TYPE_INTERRUPT,
            .wMaxPacketSize = EP_INTERRUPT_SZ,
            .bInterval = 0xFF
    };

    // ==== CDC Data Interface ====
    Interface_t CdcDataInterface = {
            .bInterfaceNumber = 1,
            .bAlternateSetting = 0,
            .bNumEndpoints = 2,
            .bInterfaceClass = 0x0A, // Data Class Interface, CDC section 4.5
            .bInterfaceSubClass = 0x00, // CDC section 4.6
            .bInterfaceProtocol = 0x01, // CDC section 4.7
            .iInterface = 0
    };
    // Endpoint 3 Descriptor
    Endpoint_t Ep3 = {
            .bEndpointAddress = (EP_CDC_DATA_OUT | EP_DIR_OUT),
            .bmAttributes = EP_TYPE_BULK,
            .wMaxPacketSize = EP_BULK_SZ,
            .bInterval = 0x00
    };
    // Endpoint 1 Descriptor
    Endpoint_t Ep1 = {
            .bEndpointAddress = (EP_CDC_DATA_IN | EP_DIR_IN),
            .bmAttributes = EP_TYPE_BULK,
            .wMaxPacketSize = EP_BULK_SZ,
            .bInterval = 0x00
    };
} ConfigDescriptor;
#endif

#if 1 // ============================== Strings ================================
struct StringDescriptor_t {
    uint8_t bLength;
    uint8_t bDescriptorType;
    char16_t bString[1];    // At least 2 bytes
};

#define STRING_DESC(s)  \
        struct { \
            uint8_t bLength = (2 + sizeof(s) - 2); /* Exclude last '\0' */ \
            uint8_t bDescriptorType = USB_DESCRIPTOR_STRING; \
            char16_t bString[sizeof(s)/sizeof(s[0])] = s; \
        }

// U.S. English language identifier
static const StringDescriptor_t LanguageString = {
        .bLength = (2 + 2), // bLength, bDescriptorType, one char16_t
        .bDescriptorType = USB_DESCRIPTOR_STRING,
        .bString = {0x0409}
};

static const STRING_DESC(u"Ostranna") VendorString;
static const STRING_DESC(u"Virtual COM") DeviceDescriptionString;
static const STRING_DESC(u"123") SerialNumberString;

static const StringDescriptor_t *StringDescriptors[] = {
        &LanguageString,
        (StringDescriptor_t*)&VendorString,
        (StringDescriptor_t*)&DeviceDescriptionString,
        (StringDescriptor_t*)&SerialNumberString
};
#endif
#pragma pack(pop)

Buf_t Usb::GetDescriptor(uint8_t dtype,  uint8_t dindex, uint16_t lang) {
    Buf_t Dsc;
    switch(dtype) {
        case USB_DESCRIPTOR_DEVICE:
            Dsc.Sz = sizeof(DeviceDescriptor);
            Dsc.Ptr = (uint8_t*)&DeviceDescriptor;
            break;
        case USB_DESCRIPTOR_CONFIGURATION:
            Dsc.Sz = sizeof(ConfigDescriptor);
            Dsc.Ptr = (uint8_t*)&ConfigDescriptor;
            break;
        case USB_DESCRIPTOR_STRING:
            if(dindex < 4) {
                Dsc.Sz = StringDescriptors[dindex]->bLength;
                Dsc.Ptr = (uint8_t*)StringDescriptors[dindex];
            }
            break;
    } // switch
    return Dsc;
}
