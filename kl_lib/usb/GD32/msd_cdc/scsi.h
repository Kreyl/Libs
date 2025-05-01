/*
 * scsi.h
 *
 *  Created on: Oct 9, 2013
 *      Author: g.kruglov
 */

#ifndef SCSI_H__
#define SCSI_H__

#include "inttypes.h"

#pragma pack(push, 1)
/*  Mass Storage Class SCSI Sense Structure
 *  Type define for a SCSI Sense structure. Structures of this type are filled out by the
 *  device via the MS_Host_RequestSense() function, indicating the current sense data of the
 *  device (giving explicit error codes for the last issued command). For details of the
 *  structure contents, refer to the SCSI specifications.
 */
struct SCSI_RequestSenseResponse {
    unsigned ResponseCode: 7;
    unsigned VALID: 1;

    uint8_t  Obsolete;

    unsigned SenseKey            : 4;
    unsigned Reserved            : 1;
    unsigned ILI                 : 1;
    unsigned EOM                 : 1;
    unsigned FileMark            : 1;

    uint8_t  Information[4];

    uint8_t  AddSenseLen;
    uint8_t  CmdSpecificInfo[4];
    uint8_t  AdditionalSenseCode;
    uint8_t  AdditionalSenseQualifier;
    uint8_t  FieldReplaceableUnitCode;
    uint8_t  SenseKeySpecific[3];
};

/** Mass Storage Class SCSI Inquiry Structure.
 *  Type define for a SCSI Inquiry structure. Structures of this type are filled out by the
 *  device via the MS_Host_GetInquiryData() function, retrieving the attached device's
 *  information.
 *  For details of the structure contents, refer to the SCSI specifications.
 */
struct SCSI_InquiryResponse {
    uint8_t Peripheral;
    uint8_t Removable;
    uint8_t Version;
    uint8_t ResponseDataFormat;
    uint8_t AdditionalLength;
    uint8_t Sccstp;
    uint8_t bqueetc;
    uint8_t CmdQue;
    uint8_t VendorID[8];
    uint8_t ProductID[16];
    uint8_t ProductRev[4];
};

struct SCSI_ReadCapacity10Response {
    uint32_t LastBlockAddr;
    uint32_t BlockSize;
};

struct SCSI_ReadFormatCapacitiesResponse {
    uint8_t Reserved[3];    // }
    uint8_t Length;         // } Header
    uint32_t NumberOfBlocks;
    uint8_t DescCode;
    uint8_t BlockSize[3];
};
#pragma pack(pop)

// Constants
inline const SCSI_InquiryResponse kInquiryData = {
        Peripheral:         0x00,   // direct access block device, connected
        Removable:          0x80,   // device is removable
        Version:            0x04,   // SPC-2 compliance
        ResponseDataFormat: 0x02,
        AdditionalLength:   31,   // == 36-5
        Sccstp:             0x00,
        bqueetc:            0x00,
        CmdQue:             0x00,
        VendorID:           {'O','s', 't', 'r', 'a', 'n', 'n', 'a'},
        ProductID:          {'M','a','s','s','S','t','o','r','a','g','e'},
        ProductRev:         {'0', '0', '0', '2'}
};

// USB Mass storage Page 0 Inquiry Data
#define PAGE0_INQUIRY_DATA_SZ   7
inline const uint8_t kPage00InquiryData[PAGE0_INQUIRY_DATA_SZ] = {
    0x00,
    0x00,
    0x00,
    (PAGE0_INQUIRY_DATA_SZ - 4),
    0x00,
    0x80,
    0x83
};

// USB Mass storage sense 6 Data
#define MODE_SENSE6_DATA_SZ     8
inline const uint8_t kModeSense6Data[MODE_SENSE6_DATA_SZ] = {
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00
};

// Magic signature for a Command Status Wrapper used in the Mass Storage Bulk-Only transport protocol
#define MS_CSW_SIGNATURE                        0x53425355UL

// Status return codes
#define SCSI_STATUS_OK                          0x00
#define SCSI_STATUS_CHECK_CONDITION             0x02
#define SCSI_STATUS_BUSY                        0x08

#if 1 // ==== SCSI Sense Key Values ====
/** SCSI Sense Code to indicate no error has occurred. */
#define SCSI_SENSE_KEY_GOOD                            0x00
/** SCSI Sense Code to indicate that the device has recovered from an error. */
#define SCSI_SENSE_KEY_RECOVERED_ERROR                 0x01
/** SCSI Sense Code to indicate that the device is not ready for a new command. */
#define SCSI_SENSE_KEY_NOT_READY                       0x02
/** SCSI Sense Code to indicate an error whilst accessing the medium. */
#define SCSI_SENSE_KEY_MEDIUM_ERROR                    0x03
/** SCSI Sense Code to indicate a hardware error has occurred. */
#define SCSI_SENSE_KEY_HARDWARE_ERROR                  0x04
/** SCSI Sense Code to indicate that an illegal request has been issued. */
#define SCSI_SENSE_KEY_ILLEGAL_REQUEST                 0x05
/** SCSI Sense Code to indicate that the unit requires attention from the host to indicate a reset event, medium removal or other condition. */
#define SCSI_SENSE_KEY_UNIT_ATTENTION                  0x06
/** SCSI Sense Code to indicate that a write attempt on a protected block has been made. */
#define SCSI_SENSE_KEY_DATA_PROTECT                    0x07
/** SCSI Sense Code to indicate an error while trying to write to a write-once medium. */
#define SCSI_SENSE_KEY_BLANK_CHECK                     0x08
/** SCSI Sense Code to indicate a vendor specific error has occurred. */
#define SCSI_SENSE_KEY_VENDOR_SPECIFIC                 0x09
/** SCSI Sense Code to indicate that an EXTENDED COPY command has aborted due to an error. */
#define SCSI_SENSE_KEY_COPY_ABORTED                    0x0A
/** SCSI Sense Code to indicate that the device has aborted the issued command. */
#define SCSI_SENSE_KEY_ABORTED_COMMAND                 0x0B
/** SCSI Sense Code to indicate an attempt to write past the end of a partition has been made. */
#define SCSI_SENSE_KEY_VOLUME_OVERFLOW                 0x0D
/** SCSI Sense Code to indicate that the source data did not match the data read from the medium. */
#define SCSI_SENSE_KEY_MISCOMPARE                      0x0E
#endif

#if 1 // ==== SCSI Additional Sense Codes ====
/** SCSI Additional Sense Code to indicate no additional sense information is available. */
#define SCSI_ASENSE_NO_ADDITIONAL_INFORMATION          0x00
/** SCSI Additional Sense Code to indicate that the logical unit (LUN) addressed is not ready. */
#define SCSI_ASENSE_LOGICAL_UNIT_NOT_READY             0x04
/** SCSI Additional Sense Code to indicate an invalid field was encountered while processing the issued command. */
#define SCSI_ASENSE_INVALID_FIELD_IN_CDB               0x24
/** SCSI Additional Sense Code to indicate that a medium that was previously indicated as not ready has now become ready for use. */
#define SCSI_ASENSE_NOT_READY_TO_READY_CHANGE          0x28
/** SCSI Additional Sense Code to indicate that an attempt to write to a protected area was made. */
#define SCSI_ASENSE_WRITE_PROTECTED                    0x27
/** SCSI Additional Sense Code to indicate an error whilst formatting the device medium. */
#define SCSI_ASENSE_FORMAT_ERROR                       0x31
/** SCSI Additional Sense Code to indicate an invalid command was issued. */
#define SCSI_ASENSE_INVALID_COMMAND                    0x20
/** SCSI Additional Sense Code to indicate a write to a block out outside of the medium's range was issued. */
#define SCSI_ASENSE_LOGICAL_BLOCK_ADDRESS_OUT_OF_RANGE 0x21
/** SCSI Additional Sense Code to indicate that no removable medium is inserted into the device. */
#define SCSI_ASENSE_MEDIUM_NOT_PRESENT                 0x3A
#endif

#if 1 // ==== SCSI Additional Sense Key Code Qualifiers ====
/** SCSI Additional Sense Qualifier Code to indicate no additional sense qualifier information is available. */
#define SCSI_ASENSEQ_NO_QUALIFIER                      0x00
/** SCSI Additional Sense Qualifier Code to indicate that a medium format command failed to complete. */
#define SCSI_ASENSEQ_FORMAT_COMMAND_FAILED             0x01
// SCSI Additional Sense Qualifier Code to indicate that an initializing command must be issued before the issued command can be executed.
#define SCSI_ASENSEQ_INITIALIZING_COMMAND_REQUIRED     0x02
/** SCSI Additional Sense Qualifier Code to indicate that an operation is currently in progress. */
#define SCSI_ASENSEQ_OPERATION_IN_PROGRESS             0x07
#endif

#endif // SCSI_H__
