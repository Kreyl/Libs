#include "descriptors_msdcdc.h"
#include "usb_dev.h"
#include "shell.h"

#if 1 // ==== Constants (do not touch) ====
#define EP_DIR_IN           0x80
#define EP_DIR_OUT          0x00
// Descriptor-type endpoint codes
#define EP_TYPE_CONTROL     0x00
#define EP_TYPE_ISOCHRONOUS 0x01
#define EP_TYPE_BULK        0x02
#define EP_TYPE_INTERRUPT   0x03

static const uint8_t kDscDevice = 1U;
static const uint8_t kDscConfiguration = 2U;
static const uint8_t kDscString = 3U;
static const uint8_t kDscInterface = 4U;
static const uint8_t kDscEndpoint = 5U;
// static const uint8_t kDscTypeDeviceQualifier = 6U;
// static const uint8_t kDscTypeOtherSpeedConfiguration = 7U;
// static const uint8_t kDscTypeInterfacePower = 8U;
static const uint8_t kDscInterfaceAssociation = 11U;
#endif

#pragma pack(push, 1)

struct DscHeader {
    uint8_t bLength;
    uint8_t bDescriptorType;
};

// ========================== Device Descriptor ==========================
static const struct DeviceDescriptor {
    DscHeader header = { sizeof(DeviceDescriptor), kDscDevice};
    /* 0xJJMN where JJ is the major version number, M is the minor version number
       and N is the sub minor version number. e.g. USB 2.0 is 0x0200, USB 1.1 is 0x0110 */
    uint16_t bcdUSB          = 0x0110;
    uint8_t  bDeviceClass    = 0xEF; // Misc device class
    uint8_t  bDeviceSubClass = 0x02; // Common class
    uint8_t  bDeviceProtocol = 0x01; // Interface Association Descriptor
    uint8_t  bMaxPacketSize  = kEp0sz; // Maximum Packet Size for Zero Endpoint
    uint16_t idVendor        = 0x0483; // ST
    uint16_t idProduct       = 0x374C;
    uint16_t bcdDevice       = 0x0200;
    uint8_t  iManufacturer   = 1;
    uint8_t  iProduct        = 2;
    uint8_t  iSerialNumber   = 3;
    uint8_t  bNumConfigurations = 1;
} device_descriptor;

#if 1 // ==================== configuration Descriptor =========================
// ==== Common use types ====
#define USB_CONFIG_POWER_MA(mA) ((mA) >> 1)
// There may be several configurations in single CfgDesc
struct Configuration {
    DscHeader header = { sizeof(Configuration), kDscConfiguration };
    uint16_t wTotalLength;
    uint8_t  bNumInterfaces;
    uint8_t  bConfigurationValue;
    uint8_t  iConfiguration;
    uint8_t  bmAttributes;
    uint8_t  bMaxPower;
};

struct Interface_t {
    DscHeader header = { sizeof(Interface_t), kDscInterface };
    uint8_t  bInterfaceNumber;
    uint8_t  bAlternateSetting;
    uint8_t  bNumEndpoints;
    uint8_t  bInterfaceClass;
    uint8_t  bInterfaceSubClass;
    uint8_t  bInterfaceProtocol;
    uint8_t  iInterface;
};

struct Endpoint_t {
    DscHeader header = { sizeof(Endpoint_t), kDscEndpoint };
    uint8_t  bEndpointAddress;
    uint8_t  bmAttributes;
    uint16_t wMaxPacketSize;
    uint8_t  bInterval;
};

struct IAD_t {
    DscHeader header = { sizeof(IAD_t), kDscInterfaceAssociation };
    uint8_t bFirstInterface;
    uint8_t bInterfaceCount;
    uint8_t bFunctionClass;
    uint8_t bFunctionSubClass;
    uint8_t bFunctionProtocol;
    uint8_t iFunction;
};

// Class-specific defines
#define CS_INTERFACE            0x24    // DescriptorType of HeaderFuncDesc

static const struct ConfigDescriptor_t {
    // ==== Configuration part ====
    Configuration configuration = {
            .wTotalLength = sizeof(ConfigDescriptor_t),
            .bNumInterfaces = 3, // 1 for MSD and 2 for CDC
            .bConfigurationValue = 1,
            .iConfiguration = 0,
            .bmAttributes = 0x80, // Bus powered
            .bMaxPower = USB_CONFIG_POWER_MA(100)
    };

    // MSD IAD is not required as MSD contains single interface only
    // ==== Mass Storage Interface ====
    Interface_t MsdInterface = {
            .bInterfaceNumber = 0,
            .bAlternateSetting = 0,
            .bNumEndpoints = 2,
            .bInterfaceClass = 0x08, // Mass Storage class
            .bInterfaceSubClass = 0x06, // SCSI Transparent Command Set subclass of the Mass storage class
            .bInterfaceProtocol = 0x50, // Bulk Only Transport protocol of the Mass Storage class
            .iInterface = 0
    };
    // Endpoint OUT
    Endpoint_t EpMsdOut = {
            .bEndpointAddress = (EP_MSD_DATA | EP_DIR_OUT),
            .bmAttributes = EP_TYPE_BULK,
            .wMaxPacketSize = EP_MSD_BULK_SZ,
            .bInterval = 0x00
    };
    // Endpoint IN
    Endpoint_t EpMsdIn = {
            .bEndpointAddress = (EP_MSD_DATA | EP_DIR_IN),
            .bmAttributes = EP_TYPE_BULK,
            .wMaxPacketSize = EP_MSD_BULK_SZ,
            .bInterval = 0x00
    };

    // ==== CDC Interface Association Descriptor ====
    IAD_t CdcIADDescriptor = {
        .bFirstInterface = 1,
        .bInterfaceCount = 2,
        .bFunctionClass = 0x02, // Communications Interface Class, CDC section 4.2
        .bFunctionSubClass = 0x02, // Abstract Control Model, CDC section 4.3
        .bFunctionProtocol = 0x01,
        .iFunction = 0
    };

    // ==== CDC Control Interface ====
    Interface_t CdcControlInterface = {
            .bInterfaceNumber = 1,
            .bAlternateSetting = 0,
            .bNumEndpoints = 1,
            .bInterfaceClass = 0x02, // Communications Interface Class, CDC section 4.2
            .bInterfaceSubClass = 0x02, // Abstract Control Model, CDC section 4.3
            /* AT commands, CDC section 4.4. @KL: 0x02? Protocol: V.25ter (AT commands).
             * For compatibility with standard host drivers, a generic virtual COM-port device
             * should specify the V.25ter protocol even if the device doesn't use AT commands */
            .bInterfaceProtocol = 0x01, // AT commands, CDC section 4.4
            .iInterface = 0
    };
    struct HeaderFunctionalDescriptor_t { // CDC section 5.2.3
        DscHeader header = { sizeof(HeaderFunctionalDescriptor_t), CS_INTERFACE };
        uint8_t  bDescriptorSubtype = 0x00;
        uint16_t bcdCDC = 0x0110;
    } HeaderFunctionalDescriptor;
    struct CallManagementFunctionalDescriptor_t {
        DscHeader header = { sizeof(CallManagementFunctionalDescriptor_t), CS_INTERFACE };
        uint8_t  bDescriptorSubtype = 0x01; // Call Management Functional Descriptor
        uint8_t  bmCapabilities = 0x00;     // (D0+D1)
        uint8_t  bDataInterface = 2;        // Interface 2 is for data
    } CallManagementFunctionalDescriptor;
    struct ACMFunctionalDescriptor_t {
        DscHeader header = { sizeof(ACMFunctionalDescriptor_t), CS_INTERFACE };
        uint8_t  bDescriptorSubtype = 0x02; // Abstract Control Management Descriptor
        uint8_t  bmCapabilitie = 0x02;
    } ACMFunctionalDescriptor;
    struct UnionFunctionalDescriptor_t {
        DscHeader header = { sizeof(UnionFunctionalDescriptor_t), CS_INTERFACE };
        uint8_t  bDescriptorSubtype = 0x06; // Union Functional Descriptor
        uint8_t  bMasterInterface = 1;      // Communication Class Interface
        uint8_t  bSlaveInterface = 2;       // Data Class Interface
    } UnionFunctionalDescriptor;
    // Interrupt Endpoint Descriptor
    Endpoint_t EpCdcInterrupt = {
            .bEndpointAddress = (EP_CDC_INTERRUPT | EP_DIR_IN),
            .bmAttributes = EP_TYPE_INTERRUPT,
            .wMaxPacketSize = EP_INTERRUPT_SZ,
            .bInterval = 0xFF
    };

    // ==== CDC Data Interface ====
    Interface_t CdcDataInterface = {
            .bInterfaceNumber = 2,
            .bAlternateSetting = 0,
            .bNumEndpoints = 2,
            .bInterfaceClass = 0x0A, // Data Class Interface, CDC section 4.5
            .bInterfaceSubClass = 0x00, // CDC section 4.6
            .bInterfaceProtocol = 0x00, // CDC section 4.7
            .iInterface = 0
    };
    Endpoint_t EpCdcDataOut = {
            .bEndpointAddress = (EP_CDC_DATA | EP_DIR_OUT),
            .bmAttributes = EP_TYPE_BULK,
            .wMaxPacketSize = EP_CDC_BULK_SZ,
            .bInterval = 0x00
    };
    Endpoint_t EpCdcDataIn = {
            .bEndpointAddress = (EP_CDC_DATA | EP_DIR_IN),
            .bmAttributes = EP_TYPE_BULK,
            .wMaxPacketSize = EP_CDC_BULK_SZ,
            .bInterval = 0x00
    };
} config_descriptor;
#endif

#if 1 // ============================== Strings ================================
struct StringDescriptor {
    uint8_t bLength;
    uint8_t bDescriptorType;
    char16_t bString[1];    // At least 2 bytes
};

#define STRING_DESC(s)  \
        struct { \
            uint8_t bLength = (2 + sizeof(s) - 2); /* Exclude last '\0' */ \
            uint8_t bDescriptorType = kDscString; \
            char16_t bString[sizeof(s)/sizeof(s[0])] = s; \
        }

// U.S. English language identifier
static const StringDescriptor language_string = {
        .bLength = (2 + 2), // bLength, bDescriptorType, one char16_t
        .bDescriptorType = kDscString,
        .bString = {0x0409}
};

static const STRING_DESC(u"Ostranna") vendor_string;
static const STRING_DESC(u"VirtualCOM and MSD") device_description_string;
static const STRING_DESC(u"124") serial_number_string;

static const StringDescriptor *string_descriptors[] = {
        &language_string,
        (StringDescriptor*)&vendor_string,
        (StringDescriptor*)&device_description_string,
        (StringDescriptor*)&serial_number_string
};
#endif
#pragma pack(pop)

Buf_t UsbDev::GetDescriptor(uint8_t dtype,  uint8_t dindex, uint16_t lang) {
    Buf_t dsc;
    switch(dtype) {
        case kDscDevice:
            dsc.sz = sizeof(device_descriptor);
            dsc.ptr = (uint8_t*)&device_descriptor;
            break;
        case kDscConfiguration:
            dsc.sz = sizeof(config_descriptor);
            dsc.ptr = (uint8_t*)&config_descriptor;
            break;
        case kDscString:
            if(dindex < 4) {
                dsc.sz = string_descriptors[dindex]->bLength;
                dsc.ptr = (uint8_t*)string_descriptors[dindex];
            }
            break;
    } // switch
    return dsc;
}
