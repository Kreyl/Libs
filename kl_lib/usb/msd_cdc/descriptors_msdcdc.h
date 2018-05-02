/*
 * descriptors_mst.h
 *
 *  Created on: 2016/01/29 ã.
 *      Author: Kreyl
 */

#pragma once

#include "ch.h"
#include "hal.h"
#include "hal_usb.h"

// Endpoints to be used for CDC
#define EP_CDC_DATA_IN      1
#define EP_CDC_DATA_OUT     1
#define EP_CDC_INTERRUPT    2

// Endpoints to be used for MSD
#define EP_MSD_IN_ID        3
#define EP_MSD_OUT_ID       3

// Endpoint Sizes for Full-Speed devices
#define EP0_SZ              64  // Control Endpoint must have a packet size of 64 bytes
#define EP_INTERRUPT_SZ     8   // Max size is 64 bytes
#define EP_BULK_SZ          64  // Max size is 64 bytes

#ifdef __cplusplus
extern "C" {
#endif
const USBDescriptor *GetDescriptor(USBDriver *usbp, uint8_t dtype, uint8_t dindex, uint16_t lang);
#ifdef __cplusplus
}
#endif
