/*
 * kl_usb_defins.h
 *
 *  Created on: 30 џэт. 2016 у.
 *      Author: Kreyl
 */

#pragma once
/*
Offset | Field         | Size | Value   | Description
--------------------------------------------------------------------------------
0      | bmRequestType | 1    | Bit-Map | D7 Data Phase Transfer Direction
       |               |      |         |   0 = Host to Device
       |               |      |         |   1 = Device to Host
       |               |      |         | D6..5 Type
       |               |      |         |   0 = Standard
       |               |      |         |   1 = Class
       |               |      |         |   2 = Vendor
       |               |      |         |   3 = Reserved
       |               |      |         | D4..0 Recipient
       |               |      |         |   0 = Device
       |               |      |         |   1 = Interface
       |               |      |         |   2 = Endpoint
       |               |      |         |   3 = Other
       |               |      |         |   4..31 = Reserved
--------------------------------------------------------------------------------
1      | bRequest      | 1    |         | Request
2      | wValue        | 2    |         | Value
4      | wIndex        | 2    |         | Index or Offset
6      | wLength       | 2    | Count   | Number of bytes to transfer if there is a data phase
*/

struct SetupPkt_t {
    union {
        uint8_t bmRequestType;
        struct {
            uint8_t Recipient : 5;
            uint8_t Type : 2;
            uint8_t Direction : 1;
        } ReqType;
    };
    uint8_t bRequest;
    union {
        uint16_t wValue;
        struct {
            uint8_t wValueLSB;
            uint8_t wValueMSB;
        };
    };
    uint16_t wIndex;
    uint16_t wLength;
} __attribute__((packed));

#define DIR_HOST2DEV    0
#define DIR_DEV2HOST    1

#define TYPE_STANDARD   0
#define TYPE_CLASS      1
#define TYPE_VENDOR     2

#define RCPT_DEV        0
#define RCPT_INTERFACE  1
#define RCPT_ENDPOINT   2
#define RCPT_OTHER      3
