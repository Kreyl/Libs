/*
 * kl_usb_defins.h
 *
 *  Created on: 30 џэт. 2016 у.
 *      Author: Kreyl
 */

#pragma once

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
    uint16_t wValue;
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
