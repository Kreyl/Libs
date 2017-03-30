/*
 * File:   cc2500defins.h
 * Author: Kreyl Laurelindo
 *
 * Created on 3 Март 2010 г., 11:08
 */

#ifndef _CC1101DEFINS_H
#define	_CC1101DEFINS_H

#include <inttypes.h>

// =================================== Power ===================================
#define CC_PwrMinus30dBm       0x03
#define CC_PwrMinus27dBm       0x08
#define CC_PwrMinus25dBm       0x0D
#define CC_PwrMinus20dBm       0x17
#define CC_PwrMinus15dBm       0x1D
#define CC_PwrMinus10dBm       0x26
#define CC_PwrMinus6dBm        0x37
#define CC_Pwr0dBm             0x50
#define CC_PwrPlus5dBm         0x86
#define CC_PwrPlus7dBm         0xCD
#define CC_PwrPlus10dBm        0xC5
#define CC_PwrPlus12dBm        0xC0

// ======================= Registers, strobes etc. =============================
// Flags
#define CC_BURST_FLAG   0b01000000
#define CC_WRITE_FLAG   0b00000000
#define CC_READ_FLAG    0b10000000

// Strobes
#define CC_SRES     0x30
#define CC_SFSTXON  0x31
#define CC_SXOFF    0x32
#define CC_SCAL     0x33
#define CC_SRX      0x34
#define CC_STX      0x35
#define CC_SIDLE    0x36
#define CC_SWOR     0x38
#define CC_SPWD     0x39
#define CC_SFRX     0x3A
#define CC_SFTX     0x3B
#define CC_SWORRST  0x3C
#define CC_SNOP     0x3D

// Status registers
#define CC_VERSION      0x31|CC_BURST_FLAG
#define CC_FREQEST      0x32|CC_BURST_FLAG
#define CC_LQI          0x33|CC_BURST_FLAG
#define CC_RSSI         0x34|CC_BURST_FLAG
#define CC_MARCSTATE    0x35|CC_BURST_FLAG
#define CC_WORTIME1     0x36|CC_BURST_FLAG
#define CC_WORTIME0     0x37|CC_BURST_FLAG
#define CC_PKTSTATUS    0x38|CC_BURST_FLAG
#define CC_VCO_VC_DAC   0x39|CC_BURST_FLAG
#define CC_TXBYTES      0x3A|CC_BURST_FLAG
#define CC_RXBYTES      0x3B|CC_BURST_FLAG
#define CC_RCCTRL1_STATUS 0x3C|CC_BURST_FLAG
#define CC_RCCTRL0_STATUS 0x3D|CC_BURST_FLAG

// States
#define CC_ST_SLEEP         0
#define CC_ST_IDLE          1
#define CC_ST_XOFF          2
#define CC_ST_MANCAL3       3
#define CC_ST_MANCAL4       4
#define CC_ST_MANCAL5       5
#define CC_ST_FS_WAKEUP6    6
#define CC_ST_FS_WAKEUP7    7
#define CC_ST_CALIBRATE8    8
#define CC_ST_SETTLING9     9
#define CC_ST_SETTLING10    10
#define CC_ST_SETTLING11    11
#define CC_ST_CALIBRATE12   12
#define CC_ST_RX13          13
#define CC_ST_RX14          14
#define CC_ST_RX15          15
#define CC_ST_TXRX_SETTLING 16
#define CC_ST_RX_OVERFLOW   17
#define CC_ST_FSTXON        18
#define CC_ST_TX19          19
#define CC_ST_TX20          20
#define CC_ST_RXTX_SETTLING 21
#define CC_ST_TX_UNDERFLOW  22

// Status byte states
#define CC_STB_IDLE         0x00
#define CC_STB_RX           0x10
#define CC_STB_TX           0x20
#define CC_STB_FSTXON       0x30
#define CC_STB_CALIBRATE    0x40
#define CC_STB_SETTLING     0x50
#define CC_STB_RX_OVF       0x60
#define CC_STB_TX_UNDF      0x70

// Config registers addresses
#define CC_IOCFG2   0x00
#define CC_IOCFG1   0x01
#define CC_IOCFG0   0x02
#define CC_FIFOTHR  0x03
#define CC_SYNC1    0x04
#define CC_SYNC0    0x05
#define CC_PKTLEN   0x06
#define CC_PKTCTRL1 0x07
#define CC_PKTCTRL0 0x08
#define CC_ADDR     0x09
#define CC_CHANNR   0x0A
#define CC_FSCTRL1  0x0B
#define CC_FSCTRL0  0x0C
#define CC_FREQ2    0x0D
#define CC_FREQ1    0x0E
#define CC_FREQ0    0x0F
#define CC_MDMCFG4  0x10
#define CC_MDMCFG3  0x11
#define CC_MDMCFG2  0x12
#define CC_MDMCFG1  0x13
#define CC_MDMCFG0  0x14
#define CC_DEVIATN  0x15
#define CC_MCSM2    0x16
#define CC_MCSM1    0x17
#define CC_MCSM0    0x18
#define CC_FOCCFG   0x19
#define CC_BSCFG    0x1A
#define CC_AGCCTRL2 0x1B
#define CC_AGCCTRL1 0x1C
#define CC_AGCCTRL0 0x1D
#define CC_WOREVT1  0x1E
#define CC_WOREVT0  0x1F
#define CC_WORCTRL  0x20
#define CC_FREND1   0x21
#define CC_FREND0   0x22
#define CC_FSCAL3   0x23
#define CC_FSCAL2   0x24
#define CC_FSCAL1   0x25
#define CC_FSCAL0   0x26
#define CC_RCCTRL1  0x27
#define CC_RCCTRL0  0x28
#define CC_FSTEST   0x29
#define CC_PTEST    0x2A
#define CC_AGCTEST  0x2B
#define CC_TEST2    0x2C
#define CC_TEST1    0x2D
#define CC_TEST0    0x2E

#define CC_PATABLE  0x3E

// FIFO
#define CC_FIFO     0x3F


#endif	/* _CC1101DEFINS_H */

