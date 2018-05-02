/*
 * usb_mst.h
 *
 *  Created on: 2016/01/29 ã.
 *      Author: Kreyl
 */

#pragma once

#include "hal.h"
#include "scsi.h"
#include "shell.h"

/*
 * Beware! 24MHz core clock is not enough for USB to be happy!
 */

#if 1 // ================= Mass Storage constants and types ====================
// Enum for the Mass Storage class specific control requests that can be issued by the USB bus host
enum MS_ClassRequests_t {
    // Mass Storage class-specific request to retrieve the total number of Logical Units (drives) in the SCSI device.
    MS_REQ_GetMaxLUN = 0xFE,
    // Mass Storage class-specific request to reset the Mass Storage interface, ready for the next command.
    MS_REQ_MassStorageReset = 0xFF,
};

// Mass Storage Class Command Block Wrapper
struct MS_CommandBlockWrapper_t {
    uint32_t Signature;         // Command block signature, must be MS_CBW_SIGNATURE to indicate a valid Command Block
    uint32_t Tag;               // Unique command ID value, to associate a command block wrapper with its command status wrapper
    uint32_t DataTransferLen;   // Length of the optional data portion of the issued command, in bytes
    uint8_t  Flags;             // Command block flags, indicating command data direction
    uint8_t  LUN;               // Logical Unit number this command is issued to
    uint8_t  SCSICmdLen;        // Length of the issued SCSI command within the SCSI command data array
    uint8_t  SCSICmdData[16];   // Issued SCSI command in the Command Block
} __attribute__ ((__packed__));
#define MS_CMD_SZ   sizeof(MS_CommandBlockWrapper_t)

// Mass Storage Class Command Status Wrapper
struct MS_CommandStatusWrapper_t {
    uint32_t Signature;          // Status block signature, must be \ref MS_CSW_SIGNATURE to indicate a valid Command Status
    uint32_t Tag;                // Unique command ID value, to associate a command block wrapper with its command status wrapper
    uint32_t DataTransferResidue;// Number of bytes of data not processed in the SCSI command
    uint8_t  Status;             // Status code of the issued command - a value from the MS_CommandStatusCodes_t enum
} __attribute__ ((__packed__));
#endif

#define MSD_TIMEOUT_MS   2700
#define MSD_DATABUF_SZ   4096

class UsbMsdCdc_t : public PrintfHelper_t, public Shell_t {
private:
    void IStartTransmissionIfNotYet() {} // Dummy
    uint8_t IPutChar(char c);
public:
    void Init();
    void Reset();
    void Connect();
    void Disconnect();
    void Printf(const char *format, ...) {
        va_list args;
        va_start(args, format);
        IVsPrintf(format, args);
        va_end(args);
    }
    void SignalCmdProcessed();
};

extern UsbMsdCdc_t UsbMsdCdc;
