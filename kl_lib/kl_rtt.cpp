/*
 * kl_rtt.cpp
 *
 *  Created on: 14 дек. 2023 г.
 *      Author: laurelindo
 */

#include "kl_rtt.h"
#include <string.h> // for memcpy

#include "shell.h"

// Some cores support out-of-order memory accesses (reordering of memory accesses in the core)
// For such cores, we need to define a memory barrier to guarantee the order of certain accesses to the RTT ring buffers.
// Needed for:
//   Cortex-M7 (ARMv7-M)
//   Cortex-M23 (ARM-v8M)
//   Cortex-M33 (ARM-v8M)
//   Cortex-A/R (ARM-v7A/R)

#define _CC_HAS_RTT_ASM_SUPPORT 1
// ARM 7/9: __ARM_ARCH_5__ / __ARM_ARCH_5E__ / __ARM_ARCH_5T__ / __ARM_ARCH_5T__ / __ARM_ARCH_5TE__
#if (defined __ARM_ARCH_7M__)                 // Cortex-M3
  #define _CORE_HAS_RTT_ASM_SUPPORT 1
#elif (defined __ARM_ARCH_7EM__)              // Cortex-M4/M7
#define _CORE_HAS_RTT_ASM_SUPPORT 1
#define _CORE_NEEDS_DMB           1         // Only Cortex-M7 needs a DMB but we cannot distinguish M4 and M7 here...
#define RTT__DMB() __asm volatile ("dmb\n" : : :);
#elif (defined __ARM_ARCH_8M_BASE__)          // Cortex-M23
  #define _CORE_HAS_RTT_ASM_SUPPORT 0
  #define _CORE_NEEDS_DMB           1
  #define RTT__DMB() __asm volatile ("dmb\n" : : :);
#elif (defined __ARM_ARCH_8M_MAIN__)          // Cortex-M33
  #define _CORE_HAS_RTT_ASM_SUPPORT 1
  #define _CORE_NEEDS_DMB           1
  #define RTT__DMB() __asm volatile ("dmb\n" : : :);
#elif (defined __ARM_ARCH_8_1M_MAIN__)        // Cortex-M85
  #define _CORE_HAS_RTT_ASM_SUPPORT 1
  #define _CORE_NEEDS_DMB           1
  #define RTT__DMB() __asm volatile ("dmb\n" : : :);
#elif ((defined __ARM_ARCH_7A__) || (defined __ARM_ARCH_7R__))  // Cortex-A/R 32-bit ARMv7-A/R
  #define _CORE_NEEDS_DMB           1
  #define RTT__DMB() __asm volatile ("dmb\n" : : :);
#else
  #define _CORE_HAS_RTT_ASM_SUPPORT 0
#endif

  // If IDE and core support the ASM version, enable ASM version by default
#ifndef _CORE_HAS_RTT_ASM_SUPPORT
#define _CORE_HAS_RTT_ASM_SUPPORT 0              // Default for unknown cores
#endif
#if (_CC_HAS_RTT_ASM_SUPPORT && _CORE_HAS_RTT_ASM_SUPPORT)
#define RTT_USE_ASM                           (1)
#else
#define RTT_USE_ASM                           (0)
#endif


// Description for a circular buffer (also called "ring buffer") which is used as up-buffer (T->H)
typedef struct {
  const     char*    sName;         // Optional name. Standard names so far are: "Terminal", "SysView", "J-Scope_t4i4"
            char*    pBuffer;       // Pointer to start of buffer
            unsigned SizeOfBuffer;  // Buffer size in bytes. Note that one byte is lost, as this implementation does not fill up the buffer in order to avoid the problem of being unable to distinguish between full and empty.
            unsigned WrOff;         // Position of next item to be written by either target.
  volatile  unsigned RdOff;         // Position of next item to be read by host. Must be volatile since it may be modified by host.
            unsigned Flags;         // Contains configuration flags. Flags[31:24] are used for validity check and must be zero. Flags[23:2] are reserved for future use. Flags[1:0] = RTT operating mode.
} SEGGER_RTT_BUFFER_UP;

// Description for a circular buffer (also called "ring buffer") which is used as down-buffer (H->T)
typedef struct {
  const     char*    sName;         // Optional name. Standard names so far are: "Terminal", "SysView", "J-Scope_t4i4"
            char*    pBuffer;       // Pointer to start of buffer
            unsigned SizeOfBuffer;  // Buffer size in bytes. Note that one byte is lost, as this implementation does not fill up the buffer in order to avoid the problem of being unable to distinguish between full and empty.
  volatile  unsigned WrOff;         // Position of next item to be written by host. Must be volatile since it may be modified by host.
            unsigned RdOff;         // Position of next item to be read by target (down-buffer).
            unsigned Flags;         // Contains configuration flags. Flags[31:24] are used for validity check and must be zero. Flags[23:2] are reserved for future use. Flags[1:0] = RTT operating mode.
} SEGGER_RTT_BUFFER_DOWN;

// RTT control block which describes the number of buffers available as well as the configuration for each buffer
typedef struct {
  char                    acID[16];                                 // Initialized to "SEGGER RTT"
  int                     MaxNumUpBuffers;                          // Initialized to SEGGER_RTT_MAX_NUM_UP_BUFFERS (type. 2)
  int                     MaxNumDownBuffers;                        // Initialized to SEGGER_RTT_MAX_NUM_DOWN_BUFFERS (type. 2)
  SEGGER_RTT_BUFFER_UP    aUp[RTT_BUFTX_CNT];       // Up buffers, transferring information up from target via debug probe to host
  SEGGER_RTT_BUFFER_DOWN  aDown[RTT_BUFRX_CNT];   // Down buffers, transferring information down from host via debug probe to target
#if SEGGER_RTT__CB_PADDING
  unsigned char           aDummy[SEGGER_RTT__CB_PADDING];
#endif
} SEGGER_RTT_CB;

SEGGER_RTT_CB _SEGGER_RTT;
static char _acUpBuffer  [RTT_BUFTX_SZ];
static char _acDownBuffer[RTT_BUFRX_SZ];

void RTT_t::IInit() {
    volatile SEGGER_RTT_CB *p; // Volatile to make sure that compiler cannot change the order of accesses to the control block
    static const char _aInitStr[] = "\0\0\0\0\0\0TTR REGGES"; // Init complete ID string to make sure that things also work if RTT is linked to a no-init memory area
    unsigned i;
    // Initialize control block
    p = (volatile SEGGER_RTT_CB*) ((uintptr_t) &_SEGGER_RTT);
    memset((SEGGER_RTT_CB*) p, 0, sizeof(_SEGGER_RTT)); // Make sure that the RTT CB is always zero initialized.
    p->MaxNumUpBuffers = RTT_BUFTX_CNT;
    p->MaxNumDownBuffers = RTT_BUFRX_CNT;

    // Initialize up buffer 0
    p->aUp[0].sName = "Terminal";
    p->aUp[0].pBuffer = _acUpBuffer;
    p->aUp[0].SizeOfBuffer = RTT_BUFTX_SZ;
    p->aUp[0].RdOff = 0u;
    p->aUp[0].WrOff = 0u;
    p->aUp[0].Flags = RTT_MODE_DEFAULT;

    // Initialize down buffer 0
    p->aDown[0].sName = "Terminal";
    p->aDown[0].pBuffer = _acDownBuffer;
    p->aDown[0].SizeOfBuffer = RTT_BUFRX_SZ;
    p->aDown[0].RdOff = 0u;
    p->aDown[0].WrOff = 0u;
    p->aDown[0].Flags = RTT_MODE_DEFAULT;

    // Finish initialization of the control block.
    // Copy Id string backwards to make sure that "SEGGER RTT" is not found in initializer memory (usually flash),
    // as this would cause J-Link to "find" the control block at a wrong address.
    RTT__DMB(); // Force order of memory accesses for cores that may perform out-of-order memory accesses
    for(i = 0; i < sizeof(_aInitStr) - 1; ++i) {
        p->acID[i] = _aInitStr[sizeof(_aInitStr) - 2 - i]; // Skip terminating \0 at the end of the array
    }
    RTT__DMB(); // Force order of memory accesses for cores that may perform out-of-order memory accesses
}

RTT_t::RTT_t() { IInit(); }

// Put char to Buf[0]
retv RTT_t::IPutChar(char c) {
    volatile SEGGER_RTT_CB* pRTTCBInit = (volatile SEGGER_RTT_CB*)((uintptr_t)&_SEGGER_RTT);
    if(pRTTCBInit->acID[0] != 'S') IInit();
    // Get "to-host" ring buffer[0].
    SEGGER_RTT_BUFFER_UP* pRing = (SEGGER_RTT_BUFFER_UP*)((uintptr_t)&_SEGGER_RTT.aUp[0]);
    // Get write position and handle wrap-around if necessary
    unsigned WrOff = pRing->WrOff + 1;
    if(WrOff == pRing->SizeOfBuffer) WrOff = 0;
    // Check if free space is available
    if(WrOff == pRing->RdOff) {
        if(pRing->Flags == RTT_MODE_BLOCK_IF_FIFO_FULL) { // Wait for free space if mode is set to blocking
            while(WrOff == pRing->RdOff);
        }
        else return retv::Overflow;
    }
    // Output byte
    volatile char* pDst = (pRing->pBuffer + pRing->WrOff);
    *pDst = c;
    RTT__DMB(); // Force data write to be complete before writing the <WrOff>, in case CPU is allowed to change the order of memory accesses
    pRing->WrOff = WrOff;
    return retv::Ok;
}

uint32_t RTT_t::GetTxBytesCnt(uint32_t BufferIndex) {
    volatile SEGGER_RTT_CB *pRTTCB = (volatile SEGGER_RTT_CB*) ((uintptr_t) &_SEGGER_RTT);
    unsigned RdOff = pRTTCB->aUp[BufferIndex].RdOff;
    unsigned WrOff = pRTTCB->aUp[BufferIndex].WrOff;
    if(RdOff <= WrOff) return WrOff - RdOff;
    else return pRTTCB->aUp[BufferIndex].SizeOfBuffer - (WrOff - RdOff);
}

uint32_t RTT_t::GetRxBytesCnt(uint32_t BufferIndex) {
    SEGGER_RTT_BUFFER_DOWN* pRing = (SEGGER_RTT_BUFFER_DOWN*)((uintptr_t)&_SEGGER_RTT.aDown[BufferIndex]);
    unsigned v = pRing->WrOff;
    return v - pRing->RdOff;
}
