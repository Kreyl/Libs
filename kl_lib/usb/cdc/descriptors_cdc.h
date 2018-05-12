/*
 * descriptors_cdc.h
 *
 *  Created on: 03 сент. 2015 г.
 *      Author: Kreyl
 */

#pragma once

// Endpoints to be used for CDC
#define EP_CDC_DATA_IN      1
#define EP_CDC_DATA_OUT     1
#define EP_CDC_INTERRUPT    2

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
